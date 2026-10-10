package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.BowSpray;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.TestHull;
import dev.mcmod.aerocore.api.UnsteadyWake;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.util.Mth;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.vehicle.Boat;
import net.minecraft.world.level.Level;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.tick.EntityTickEvent;
import net.neoforged.neoforge.event.tick.LevelTickEvent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Ship wakes on the sea surface. Every wake-making source (AeroCore hulls, vanilla boats) drives its own UnsteadyWake:
 * the linear free-surface response to its pressure, integrated in time on a 128 x 128 grid that follows it, so wakes
 * keep the true track (turns curve them, a stopped boat leaves its wake running on, a starting boat sends out starting
 * waves). Each level tick the sources' states are handed to one background thread, which advances every solver by one
 * tick and publishes immutable snapshots; readers on any thread see the snapshot of the previous tick's end. Wakes add
 * to the surface every consumer reads (SeaSurface, the water sampler of AeroCore bodies, the renderer); their orbital
 * velocity and dynamic pressure are not passed on (the wake acts hydrostatically through the raised or lowered
 * surface), and a body never feels its own wake (that is in its resistance). Deep water is assumed. Outside persistent
 * sea regions, a calm zone (not saved) covers each wake grid so it can be drawn and felt.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class Wakes {
    /** Grid points per side of every wake solver. */
    public static final int GRID = 128;
    /** Below this speed the bow throws no spray worth emitting (Wakes.moving). */
    public static final double MIN_SPEED = 1.0;
    /** A source at rest sleeps once its waves fall below this, m; a source that stopped reporting is dropped. */
    private static final double QUIET_ELEVATION = 2e-4;
    private static final int UNSEEN_TICKS = 2, MAX_PENDING = 4;
    /** Wake grids lead their source by up to a quarter of their width along its motion, reached in about 3 s. */
    private static final double LEAD_FRACTION = 0.25, LEAD_RATE = 1 - Math.exp(-MinecraftUnits.TICK_SECONDS / 3);
    /** Zones are snapped to 16 m and their radius to 32 m steps, so moving boats remesh chunks rarely. */
    private static final int ZONE_GRID = 16, ZONE_STEP = 32;
    private static final double RHO = FluidState.FRESH_WATER_20C.densityKgPerM3(), G = Atmosphere.GRAVITY_MPS2;

    /** What raises the wake: its body-frame pressure, the solver grid spacing it needs, and its bow spray. */
    public interface Design {
        /** Pressure transform in the body frame (x forward from the source's wake origin). */
        UnsteadyWake.Forcing forcing();
        /** Grid spacing, m: fine enough for the waves the source makes (the grid is GRID points wide). */
        double spacing();
        /** Bow position in the body frame (forward of the origin, half-width) and its spray at this speed. */
        Bow bow(double speed);
        /** Width of the white, aerated water the hull leaves behind its origin (a dry transom), m; 0 for none. */
        default double whiteWaterWidth(double speed) { return 0; }
    }

    /** Where spray leaves the bow (body frame, m) and how. */
    public record Bow(double forwardM, double halfWidthM, BowSpray spray) {}

    /** A moving source as the spray emitter sees it this tick. */
    public record Moving(double originX, double originZ, double dirX, double dirZ, double speed, double stillY, Design design) {}

    /** A wake-making AeroCore body. */
    public interface Source {
        Design wakeDesign();
        /** World point the design's body frame is measured from. */
        Vec3d wakeOrigin();
        /** World direction of the design's forward (x) axis; only its horizontal part is used. */
        Vec3d wakeForward();
        /** World velocity, m/s. */
        Vec3d wakeVelocity();
    }

    private static final class ThinShip {
        /** The test hull's waves are a few metres long at its speeds: 0.5 m spacing, a 64 m grid. */
        static final double SPACING = 0.5;
        static final UnsteadyWake.Forcing FORCING;
        static {
            SlenderHull hull = TestHull.HULL;
            SlenderHull.Particulars d = TestHull.model().design();
            FORCING = UnsteadyWake.thinShip((x, depth) -> hull.halfBreadth(x, depth, d.waterlineY()), d.xAftM(), d.xForwardM(), d.draftM(),
                    60, 12, RHO, 1.2 * Math.PI / SPACING);
        }
    }

    /** The slender test hull at its design waterline: Michell's thin-ship sources as their double-body pressure. */
    public static final Design TEST_HULL = new Design() {
        @Override public UnsteadyWake.Forcing forcing() { return ThinShip.FORCING; }
        @Override public double spacing() { return ThinShip.SPACING; }
        /** Entrance half-angle from the design waterline over the forward 0.3 m. */
        @Override public Bow bow(double speed) {
            SlenderHull.Particulars d = TestHull.model().design();
            double step = 0.3, tip = TestHull.HULL.halfBreadth(d.xForwardM(), 0, d.waterlineY()), back = TestHull.HULL.halfBreadth(d.xForwardM() - step, 0, d.waterlineY());
            double angle = Math.atan((back - tip) / step);
            return new Bow(d.xForwardM(), tip, BowSpray.displacementBow(speed, angle, d.draftM(), G));
        }
    };

    /**
     * A planing hull as a uniform pressure over its wetted bottom (chine beam by the wetted keel length from the transom)
     * carrying its weight, the usual far-field representation of a planing boat; spray is Savitsky's whisker spray at
     * the chines where they first wet, L_C = L_K - b tan(beta) / (pi tan(trim)) from the transom.
     */
    public static Design planingBoat(double beam, double deadrise, double wettedLength, double weightN, double trim) {
        double length = Math.max(0.25, wettedLength), pressure = weightN / (length * beam);
        UnsteadyWake.Forcing forcing = UnsteadyWake.rectangle(pressure, 0, length, beam / 2);
        return new Design() {
            @Override public UnsteadyWake.Forcing forcing() { return forcing; }
            /** Planing speeds make long waves: 0.75 m spacing, a 96 m grid. */
            @Override public double spacing() { return 0.75; }
            /** A ventilated transom (C_vT = U / sqrt(g B) past PlaningDynamics.PLANING) sheds white water across the beam. */
            @Override public double whiteWaterWidth(double speed) {
                return speed / Math.sqrt(G * beam) >= dev.mcmod.aerocore.api.PlaningDynamics.PLANING ? beam : 0;
            }
            @Override public Bow bow(double speed) {
                if (!(trim > 0)) { return new Bow(length, beam / 2, BowSpray.NONE); }
                double chine = Math.max(0, length - beam * Math.tan(deadrise) / (Math.PI * Math.tan(trim)));
                return new Bow(chine, beam / 2, BowSpray.planing(speed, trim, deadrise, G));
            }
        };
    }

    private static final class Tracked {
        Design design;
        UnsteadyWake solver;
        double originX, originZ, dirX = 1, dirZ, stillY = Double.NaN, velocityX, velocityZ, leadX, leadZ, centreX, centreZ;
        long seen = Long.MIN_VALUE;
        boolean asleep = true, reported;
    }
    private static final class LevelWakes {
        final Map<Integer, Tracked> sources = new HashMap<>();
        volatile List<WaveRegion> zones = List.of();
        volatile double maxElevation;
    }
    /** One solver step: where its grid centre goes and the load at the step's end (null when the source is gone). */
    private record Job(UnsteadyWake solver, double centreX, double centreZ, UnsteadyWake.Load load, boolean reset) {}

    private static final Map<Level, LevelWakes> LEVELS = Collections.synchronizedMap(new WeakHashMap<>());
    private static final org.slf4j.Logger LOGGER = com.mojang.logging.LogUtils.getLogger();
    private static final AtomicInteger PENDING = new AtomicInteger();
    /** Tests: step the solvers on the ticking thread (a GameTest server ticks faster than real time). */
    public static volatile boolean synchronous;
    private static final ExecutorService WORKER = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "AeroCore wakes");
        thread.setDaemon(true);
        return thread;
    });

    private Wakes() {}

    private static LevelWakes of(Level level) {
        synchronized (LEVELS) { return LEVELS.computeIfAbsent(level, l -> new LevelWakes()); }
    }

    /**
     * Records a source's state for this tick: origin is the design's frame origin, forward its x axis and velocity in
     * world units (m, m/s), or velocity null to derive it from the origin's motion since the last report (vanilla boats
     * moved by player packets have no server-side velocity). Derived velocities are smoothed over a few ticks; that
     * filter only steers the grid's lead and the bow spray.
     */
    public static void update(Level level, int id, Design design, Vec3d origin, Vec3d forward, Vec3d velocity) {
        if (!finite(origin) || !finite(forward) || (velocity != null && !finite(velocity))) { return; }
        double fx = forward.x(), fz = forward.z(), length = Math.hypot(fx, fz);
        if (!(length > 1e-6)) { return; }
        LevelWakes wakes = of(level);
        long tick = level.getGameTime();
        double still = SeaSurface.stillSurface(level, origin.x(), origin.y(), origin.z());
        synchronized (wakes) {
            Tracked t = wakes.sources.get(id);
            if (t == null) {
                if (Double.isNaN(still)) { return; }
                t = new Tracked();
                t.originX = origin.x(); t.originZ = origin.z();
                wakes.sources.put(id, t);
            }
            double vx, vz;
            if (velocity != null) { vx = velocity.x(); vz = velocity.z(); }
            else {
                long dt = t.seen == Long.MIN_VALUE ? 0 : tick - t.seen;
                double rawX = dt > 0 ? (origin.x() - t.originX) / (dt * MinecraftUnits.TICK_SECONDS) : t.velocityX;
                double rawZ = dt > 0 ? (origin.z() - t.originZ) / (dt * MinecraftUnits.TICK_SECONDS) : t.velocityZ;
                vx = t.velocityX + 0.3 * (rawX - t.velocityX); vz = t.velocityZ + 0.3 * (rawZ - t.velocityZ);
            }
            t.velocityX = vx; t.velocityZ = vz;
            t.design = design;
            t.originX = origin.x(); t.originZ = origin.z();
            t.dirX = fx / length; t.dirZ = fz / length;
            if (!Double.isNaN(still)) { t.stillY = still; }
            t.seen = tick;
            t.reported = !Double.isNaN(still);
        }
    }

    private static boolean finite(Vec3d v) { return Double.isFinite(v.x()) && Double.isFinite(v.y()) && Double.isFinite(v.z()); }

    /** The sources moving at MIN_SPEED or more on a free surface this tick. */
    public static List<Moving> moving(Level level) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null) { return List.of(); }
        long tick = level.getGameTime();
        List<Moving> result = new ArrayList<>();
        synchronized (wakes) {
            for (Tracked t : wakes.sources.values()) {
                double speed = Math.hypot(t.velocityX, t.velocityZ);
                if (tick - t.seen <= UNSEEN_TICKS && speed >= MIN_SPEED && t.design != null && !Double.isNaN(t.stillY)) {
                    result.add(new Moving(t.originX, t.originZ, t.dirX, t.dirZ, speed, t.stillY, t.design));
                }
            }
        }
        return result;
    }

    /**
     * Adds the wakes at (x, z) of sources riding the free surface at stillY (excluding entity id exclude, -1 for none)
     * into out[0] (elevation, m), out[1] and out[2] (slopes d/dx, d/dz).
     */
    public static void sample(Level level, double stillY, double x, double z, int exclude, double[] out) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null || wakes.maxElevation == 0) { return; }
        double[] local = new double[3];
        synchronized (wakes) {
            for (Map.Entry<Integer, Tracked> entry : wakes.sources.entrySet()) {
                Tracked t = entry.getValue();
                if (entry.getKey() == exclude || t.solver == null || Math.abs(t.stillY - stillY) > WaveRegion.LEVEL_TOLERANCE) { continue; }
                UnsteadyWake.Snapshot s = t.solver.snapshot();
                if (s.maxElevation() == 0) { continue; }
                s.sample(x, z, local);
                out[0] += local[0]; out[1] += local[1]; out[2] += local[2];
            }
        }
    }

    /** The latest wake snapshot of source id (null when it has no live wake), for the source's own wave resistance. */
    public static UnsteadyWake.Snapshot snapshot(Level level, int id) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null) { return null; }
        synchronized (wakes) {
            Tracked t = wakes.sources.get(id);
            return t == null || t.solver == null ? null : t.solver.snapshot();
        }
    }

    public static double elevation(Level level, double stillY, double x, double z, int exclude) {
        double[] out = new double[3];
        sample(level, stillY, x, z, exclude, out);
        return out[0];
    }

    /** Largest wake elevation present in the level, m (for the band the surface can move through). */
    public static double maxElevation(Level level) {
        LevelWakes wakes = LEVELS.get(level);
        return wakes == null ? 0 : wakes.maxElevation;
    }

    /**
     * The finest grid spacing (m) of the wakes on this still level whose grids overlap the box, or infinity: the
     * renderer keeps its mesh that fine there.
     */
    public static double finestSpacing(Level level, double stillY, double minX, double minZ, double maxX, double maxZ) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null || wakes.maxElevation == 0) { return Double.POSITIVE_INFINITY; }
        double finest = Double.POSITIVE_INFINITY;
        synchronized (wakes) {
            for (Tracked tr : wakes.sources.values()) {
                if (tr.solver == null || Math.abs(tr.stillY - stillY) > WaveRegion.LEVEL_TOLERANCE) { continue; }
                UnsteadyWake.Snapshot snap = tr.solver.snapshot();
                double h = snap.halfSize();
                if (snap.maxElevation() > 0 && minX < snap.centreX() + h && maxX > snap.centreX() - h && minZ < snap.centreZ() + h && maxZ > snap.centreZ() - h) {
                    finest = Math.min(finest, tr.design.spacing());
                }
            }
        }
        return finest;
    }

    /** Calm zones covering wakes outside persistent regions (recomputed every tick). */
    public static List<WaveRegion> zones(Level level) {
        LevelWakes wakes = LEVELS.get(level);
        return wakes == null ? List.of() : wakes.zones;
    }

    /**
     * Adds wake elevation and slopes on the renderer grid x0 + i s, z0 + k s (i, k < points; index i * points + k) for
     * sources riding the region's still level.
     */
    public static void addToGrid(Level level, WaveRegion region, double x0, double z0, double s, int points, double t,
                                 double[] height, double[] slopeX, double[] slopeZ) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null || wakes.maxElevation == 0) { return; }
        double span = (points - 1) * s;
        List<UnsteadyWake.Snapshot> near = new ArrayList<>();
        synchronized (wakes) {
            for (Tracked tr : wakes.sources.values()) {
                if (tr.solver == null || Math.abs(tr.stillY - region.stillWaterY()) > WaveRegion.LEVEL_TOLERANCE) { continue; }
                UnsteadyWake.Snapshot snap = tr.solver.snapshot();
                double h = snap.halfSize();
                if (snap.maxElevation() > 0 && x0 < snap.centreX() + h && x0 + span > snap.centreX() - h
                        && z0 < snap.centreZ() + h && z0 + span > snap.centreZ() - h) { near.add(snap); }
            }
        }
        if (near.isEmpty()) { return; }
        double[] out = new double[3];
        for (UnsteadyWake.Snapshot snap : near) {
            for (int i = 0; i < points; i++) for (int k = 0; k < points; k++) {
                snap.sample(x0 + i * s, z0 + k * s, out);
                if (out[0] == 0 && out[1] == 0 && out[2] == 0) { continue; }
                int p = i * points + k;
                height[p] += out[0]; slopeX[p] += out[1]; slopeZ[p] += out[2];
            }
        }
    }

    /**
     * End of tick: hands every live solver its step to the background thread, drops sources gone quiet, and refreshes
     * the elevation bound and the calm zones from the published snapshots.
     */
    private static void endTick(Level level) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null) { return; }
        long tick = level.getGameTime();
        List<WaveRegion> persistent = SeaSurface.persistentRegions(level), zones = new ArrayList<>();
        List<Job> jobs = new ArrayList<>();
        double max = 0;
        synchronized (wakes) {
            for (Iterator<Tracked> it = wakes.sources.values().iterator(); it.hasNext(); ) {
                Tracked t = it.next();
                boolean present = tick - t.seen <= UNSEEN_TICKS && t.reported;
                UnsteadyWake.Snapshot snap = t.solver == null ? null : t.solver.snapshot();
                double waves = snap == null ? 0 : snap.maxElevation();
                if (!present && waves < QUIET_ELEVATION) { it.remove(); continue; }
                max = Math.max(max, waves);
                job(t, present, waves, jobs);
                if (snap != null && waves > 0) {
                    double reach = snap.halfSize() * Math.sqrt(2);
                    if (SeaSurface.regionAt(persistent, snap.centreX(), snap.centreZ()) == null) {
                        double radius = Math.min(WaveRegion.MAX_RADIUS, Math.ceil((reach + ZONE_GRID) / ZONE_STEP) * ZONE_STEP);
                        double x = Math.floorDiv(Mth.floor(snap.centreX()), ZONE_GRID) * ZONE_GRID + ZONE_GRID / 2.0;
                        double z = Math.floorDiv(Mth.floor(snap.centreZ()), ZONE_GRID) * ZONE_GRID + ZONE_GRID / 2.0;
                        WaveRegion zone = new WaveRegion(x, z, radius, t.stillY, WaveField.NONE);
                        if (!zones.contains(zone)) { zones.add(zone); }
                    }
                }
            }
        }
        wakes.maxElevation = max;
        wakes.zones = List.copyOf(zones);
        if (jobs.isEmpty()) { return; }
        if (synchronous) { run(jobs); return; }
        // A worker more than a few ticks behind skips ticks (the wakes then run slow) rather than queueing without bound.
        if (PENDING.get() >= MAX_PENDING) { return; }
        PENDING.incrementAndGet();
        WORKER.execute(() -> {
            try { run(jobs); } finally { PENDING.decrementAndGet(); }
        });
    }

    private static void run(List<Job> jobs) {
        try {
            for (Job job : jobs) {
                if (job.reset()) { job.solver().reset(job.centreX(), job.centreZ(), job.load()); }
                else { job.solver().step(job.centreX(), job.centreZ(), job.load()); }
            }
        } catch (RuntimeException e) {
            LOGGER.error("Wake step failed", e);
        }
    }

    /**
     * Plans one source's step. Its grid centre leads the source along its velocity by up to a quarter of the grid
     * (reached over about 3 s, scaled down below 1 m/s) so most of the grid lies behind it. A source at rest on quiet
     * water sleeps; it restarts in hydrostatic balance when it moves.
     */
    private static void job(Tracked t, boolean present, double waves, List<Job> jobs) {
        Design design = t.design;
        if (design == null) { return; }
        double speed = Math.hypot(t.velocityX, t.velocityZ);
        boolean moving = present && speed > 0.05;
        if (t.solver == null || present && t.solver.spacing() != design.spacing()) {
            // Solvers (1.6 MB each) exist only while their source moves or its waves last.
            if (!moving) { t.solver = null; return; }
            t.solver = new UnsteadyWake(GRID, design.spacing(), Double.POSITIVE_INFINITY, RHO, G, MinecraftUnits.TICK_SECONDS);
            t.asleep = true;
        }
        if (!moving && !t.asleep && waves < QUIET_ELEVATION) {
            if (present) { t.solver = null; return; }
        }
        double size = GRID * design.spacing(), scale = Math.min(1, speed);
        double targetX = speed > 1e-6 ? t.velocityX / speed * LEAD_FRACTION * size * scale : 0;
        double targetZ = speed > 1e-6 ? t.velocityZ / speed * LEAD_FRACTION * size * scale : 0;
        UnsteadyWake.Load load = present ? new UnsteadyWake.Load(design.forcing(), t.originX, t.originZ, t.dirX, t.dirZ,
                t.velocityX * t.dirX + t.velocityZ * t.dirZ) : null;
        if (t.asleep) {
            t.leadX = 0; t.leadZ = 0; t.asleep = false;
            t.centreX = t.originX; t.centreZ = t.originZ;
            jobs.add(new Job(t.solver, t.centreX, t.centreZ, load, true));
            return;
        }
        if (present) {
            t.leadX += (targetX - t.leadX) * LEAD_RATE; t.leadZ += (targetZ - t.leadZ) * LEAD_RATE;
            t.centreX = t.originX + t.leadX; t.centreZ = t.originZ + t.leadZ;
            jobs.add(new Job(t.solver, t.centreX, t.centreZ, load, false));
        } else {
            // A vanished source: its grid stays where it was and its waves run out.
            jobs.add(new Job(t.solver, t.centreX, t.centreZ, null, false));
        }
    }

    @SubscribeEvent public static void entityTick(EntityTickEvent.Post event) {
        Entity entity = event.getEntity();
        if (entity instanceof Source source) {
            update(entity.level(), entity.getId(), source.wakeDesign(), source.wakeOrigin(), source.wakeForward(), source.wakeVelocity());
        } else if (entity instanceof Boat boat) {
            double yaw = Math.toRadians(boat.getYRot());
            update(boat.level(), boat.getId(), RowingBoats.wakeDesign(boat), new Vec3d(boat.getX(), boat.getY(), boat.getZ()),
                    new Vec3d(-Math.sin(yaw), 0, Math.cos(yaw)), null);
        }
    }

    @SubscribeEvent public static void levelTick(LevelTickEvent.Post event) { endTick(event.getLevel()); }
}
