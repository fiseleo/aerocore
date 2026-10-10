package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.BowSpray;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.KelvinWake;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.TestHull;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.neoforge.AeroCore;
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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Ship wakes on the sea surface. Each moving source (AeroCore hulls, vanilla boats) carries the steady Kelvin pattern
 * (KelvinWake) for its current speed, rigidly attached to its position and direction of motion: a steady-state, straight
 * course approximation, so wakes behind accelerating or turning boats are not their true history. Patterns are
 * computed off-thread per speed bin (3 % steps, one computation in flight per design) and cross-fade over a second when
 * the bin changes; a stopped source fades out. Wakes add to the surface every consumer reads (SeaSurface, the water
 * sampler of AeroCore bodies, the renderer); their orbital velocity and dynamic pressure are not modelled (the wake acts
 * hydrostatically through the raised or lowered surface), and a body never feels its own wake (that is in its
 * resistance). Outside persistent sea regions, a calm zone (not saved) follows each wake so it can be drawn and felt.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class Wakes {
    /** Pattern grid spacing, m, and the shortest wave kept (four grid points per wavelength). */
    public static final double SPACING = 0.5, SHORTEST_WAVELENGTH = 4 * SPACING;
    /** Below this speed the resolvable waves are negligible; a slower source fades its last pattern out. */
    public static final double MIN_SPEED = 1.0;
    private static final double BIN_RATIO = 1.03, MIN_LENGTH = 24, MAX_LENGTH = 96;
    private static final int FADE_TICKS = 20, UNSEEN_TICKS = 2, MAX_CACHED = 48;
    /** Zones are snapped to 16 m and their radius to 32 m steps, so moving boats remesh chunks rarely. */
    private static final int ZONE_GRID = 16, ZONE_STEP = 32;

    /** What raises the wake and how its amplitude function is built; key() identifies cached patterns. */
    public interface Design {
        String key();
        KelvinWake wake(double speed, double patternLength);
        /** Bow position in the body frame (forward of the origin, half-width) and its spray at this speed. */
        Bow bow(double speed);
    }

    /** Where spray leaves the bow (body frame, m) and how. */
    public record Bow(double forwardM, double halfWidthM, BowSpray spray) {}

    /** A moving source as the spray emitter sees it this tick. */
    public record Moving(double originX, double originZ, double dirX, double dirZ, double speed, double stillY, Design design) {}

    /** A wake-making AeroCore body. */
    public interface Source {
        Design wakeDesign();
        /** World point the design's body frame is measured from (x forward along the motion). */
        Vec3d wakeOrigin();
        /** World velocity, m/s. */
        Vec3d wakeVelocity();
    }

    /** The slender test hull at its design waterline: Michell thin-ship sources. */
    public static final Design TEST_HULL = new Design() {
        @Override public String key() { return "aerocore:test_hull"; }
        /** Entrance half-angle from the design waterline over the forward 0.3 m. */
        @Override public Bow bow(double speed) {
            SlenderHull.Particulars d = TestHull.model().design();
            double step = 0.3, tip = TestHull.HULL.halfBreadth(d.xForwardM(), 0, d.waterlineY()), back = TestHull.HULL.halfBreadth(d.xForwardM() - step, 0, d.waterlineY());
            double angle = Math.atan((back - tip) / step);
            return new Bow(d.xForwardM(), tip, BowSpray.displacementBow(speed, angle, d.draftM(), Atmosphere.GRAVITY_MPS2));
        }
        @Override public KelvinWake wake(double speed, double length) {
            SlenderHull hull = TestHull.HULL;
            SlenderHull.Particulars d = TestHull.model().design();
            return KelvinWake.thinShip((x, depth) -> hull.halfBreadth(x, depth, d.waterlineY()), d.xAftM(), d.xForwardM(), d.draftM(),
                    60, 12, speed, Atmosphere.GRAVITY_MPS2, SHORTEST_WAVELENGTH, length);
        }
    };

    /**
     * A vanilla boat as a uniform surface pressure over its square footprint, equal to the hydrostatic pressure at the
     * draft vanilla floats it at: its buoyancy (waterLevel - y) / height * g_boat / 0.65 balances g_boat at 0.65 of
     * the box height, so p = rho g 0.65 h. Vanilla's floating rule, not a measured boat.
     */
    public static Design vanillaBoat(double width, double height) {
        double draft = 0.65 * height, pressure = FluidState.FRESH_WATER_20C.densityKgPerM3() * Atmosphere.GRAVITY_MPS2 * draft;
        return new Design() {
            @Override public String key() { return "minecraft:boat:" + width + ":" + height; }
            /** A square box bow: blunt (90 degree entrance), at vanilla's floating draft. */
            @Override public Bow bow(double speed) {
                return new Bow(width / 2, width / 2, BowSpray.displacementBow(speed, Math.PI / 2, draft, Atmosphere.GRAVITY_MPS2));
            }
            @Override public KelvinWake wake(double speed, double length) {
                return KelvinWake.pressure(KelvinWake.rectangle(pressure, -width / 2, width / 2, width / 2), -width / 2, width / 2, width / 2,
                        FluidState.FRESH_WATER_20C.densityKgPerM3(), speed, Atmosphere.GRAVITY_MPS2, SHORTEST_WAVELENGTH, length);
            }
        };
    }

    /**
     * A planing hull as a uniform pressure over its wetted bottom (chine beam by the wetted keel length from the transom)
     * carrying its weight, the usual far-field representation of a planing boat; spray is Savitsky's whisker spray at
     * the chines where they first wet, L_C = L_K - b tan(beta) / (pi tan(trim)) from the transom. The key quantises the
     * wetted length to 0.25 m and the trim to 0.5 deg so patterns can be cached.
     */
    public static Design planingBoat(double beam, double deadrise, double wettedLength, double weightN, double trim) {
        double length = Math.max(0.25, Math.round(wettedLength / 0.25) * 0.25), angle = Math.toRadians(Math.round(Math.toDegrees(trim) * 2) / 2.0);
        double pressure = weightN / (length * beam);
        return new Design() {
            @Override public String key() { return "aerocore:planing:" + beam + ":" + length; }
            @Override public KelvinWake wake(double speed, double patternLength) {
                return KelvinWake.pressure(KelvinWake.rectangle(pressure, 0, length, beam / 2), 0, length, beam / 2,
                        FluidState.FRESH_WATER_20C.densityKgPerM3(), speed, Atmosphere.GRAVITY_MPS2, SHORTEST_WAVELENGTH, patternLength);
            }
            @Override public Bow bow(double speed) {
                if (!(angle > 0)) { return new Bow(length, beam / 2, BowSpray.NONE); }
                double chine = Math.max(0, length - beam * Math.tan(deadrise) / (Math.PI * Math.tan(angle)));
                return new Bow(chine, beam / 2, BowSpray.planing(speed, angle, deadrise, Atmosphere.GRAVITY_MPS2));
            }
        };
    }

    private static final class Tracked {
        Design design;
        double originX, originZ, dirX = 1, dirZ, stillY, velocityX, velocityZ;
        KelvinWake.Pattern current, previous;
        long switched, seen = Long.MIN_VALUE;
    }
    private static final class LevelWakes {
        final Map<Integer, Tracked> sources = new HashMap<>();
        volatile List<WaveRegion> zones = List.of();
        volatile double maxElevation;
    }

    private static final ThreadLocal<double[]> LOCAL = ThreadLocal.withInitial(() -> new double[3]);
    private static final Map<Level, LevelWakes> LEVELS = Collections.synchronizedMap(new WeakHashMap<>());
    private static final Map<String, CompletableFuture<KelvinWake.Pattern>> CACHE = new LinkedHashMap<>(64, 0.75f, true);
    private static final Map<String, CompletableFuture<KelvinWake.Pattern>> IN_FLIGHT = new HashMap<>();
    private static final ExecutorService WORKER = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "AeroCore wake patterns");
        thread.setDaemon(true);
        thread.setPriority(Thread.MIN_PRIORITY);
        return thread;
    });

    private Wakes() {}

    private static LevelWakes of(Level level) {
        synchronized (LEVELS) { return LEVELS.computeIfAbsent(level, l -> new LevelWakes()); }
    }

    /** Speed of a bin, m/s. */
    public static double binSpeed(int bin) { return MIN_SPEED * Math.pow(BIN_RATIO, bin); }
    public static int bin(double speed) { return (int) Math.round(Math.log(speed / MIN_SPEED) / Math.log(BIN_RATIO)); }
    /** Pattern length: eight transverse wavelengths, kept between 24 and 96 m. */
    public static double patternLength(double speed) {
        return Math.clamp(8 * 2 * Math.PI * speed * speed / Atmosphere.GRAVITY_MPS2, MIN_LENGTH, MAX_LENGTH);
    }

    /** Computes a pattern on the calling thread (tests and tools). */
    public static KelvinWake.Pattern computeNow(Design design, int bin) {
        double speed = binSpeed(bin), length = patternLength(speed);
        return design.wake(speed, length).pattern(length, SPACING);
    }

    /** Computes and caches a pattern on the calling thread, so the next update finds it ready (tests). */
    public static void prefetch(Design design, int bin) {
        KelvinWake.Pattern pattern = computeNow(design, bin);
        synchronized (CACHE) { CACHE.put(design.key() + "#" + bin, CompletableFuture.completedFuture(pattern)); }
    }

    /** The pattern if ready; otherwise starts it when nothing else of this design is being computed, and returns null. */
    private static KelvinWake.Pattern request(Design design, int bin) {
        String key = design.key() + "#" + bin;
        synchronized (CACHE) {
            CompletableFuture<KelvinWake.Pattern> cached = CACHE.get(key);
            if (cached != null) { return cached.isDone() && !cached.isCompletedExceptionally() ? cached.join() : null; }
            CompletableFuture<KelvinWake.Pattern> busy = IN_FLIGHT.get(design.key());
            if (busy != null && !busy.isDone()) { return null; }
            CompletableFuture<KelvinWake.Pattern> future = CompletableFuture.supplyAsync(() -> computeNow(design, bin), WORKER);
            CACHE.put(key, future);
            IN_FLIGHT.put(design.key(), future);
            if (CACHE.size() > MAX_CACHED) {
                Iterator<Map.Entry<String, CompletableFuture<KelvinWake.Pattern>>> it = CACHE.entrySet().iterator();
                while (CACHE.size() > MAX_CACHED && it.hasNext()) { if (it.next().getValue().isDone()) { it.remove(); } }
            }
            return null;
        }
    }

    /**
     * Records a source's state for this tick: origin is the design's frame origin in world space, velocity in m/s,
     * or null to derive it from the origin's motion since the last report (vanilla boats moved by player packets have
     * no server-side velocity). Derived velocities are smoothed over a few ticks so the speed bin does not flicker; this
     * filter is a display choice.
     */
    public static void update(Level level, int id, Design design, Vec3d origin, Vec3d velocity) {
        if (!finite(origin) || (velocity != null && !finite(velocity))) { return; }
        LevelWakes wakes = of(level);
        long tick = level.getGameTime();
        double still = SeaSurface.stillSurface(level, origin.x(), origin.y(), origin.z());
        synchronized (wakes) {
            Tracked t = wakes.sources.get(id);
            if (t == null) {
                if (Double.isNaN(still)) { return; }
                t = new Tracked();
                t.originX = origin.x(); t.originZ = origin.z(); t.switched = tick;
                wakes.sources.put(id, t);
            }
            double vx, vz;
            if (velocity != null) { vx = velocity.x(); vz = velocity.z(); }
            else {
                long dt = t.seen == Long.MIN_VALUE ? 0 : tick - t.seen;
                double rawX = dt > 0 ? (origin.x() - t.originX) / (dt * 0.05) : t.velocityX, rawZ = dt > 0 ? (origin.z() - t.originZ) / (dt * 0.05) : t.velocityZ;
                vx = t.velocityX + 0.3 * (rawX - t.velocityX); vz = t.velocityZ + 0.3 * (rawZ - t.velocityZ);
            }
            t.velocityX = vx; t.velocityZ = vz;
            double speed = Math.hypot(vx, vz);
            t.design = design;
            t.originX = origin.x(); t.originZ = origin.z();
            if (speed > 0.2) { t.dirX = vx / speed; t.dirZ = vz / speed; }
            t.seen = tick;
            boolean stopping = Double.isNaN(still) || speed < MIN_SPEED;
            KelvinWake.Pattern next = stopping ? null : request(design, bin(speed));
            if (!Double.isNaN(still)) { t.stillY = still; }
            if (stopping ? t.current != null : next != null && next != t.current) {
                t.previous = fade(t, tick) > 0.5 ? t.current : t.previous;
                t.current = stopping ? null : next;
                t.switched = tick;
            }
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

    /** Weight of the current pattern (the previous one gets 1 - weight). */
    private static double fade(Tracked t, long tick) { return Math.clamp((tick - t.switched) / (double) FADE_TICKS, 0, 1); }
    /** Overall weight: sources that stopped reporting fade out. */
    private static double presence(Tracked t, long tick) {
        long unseen = tick - t.seen - UNSEEN_TICKS;
        return unseen <= 0 ? 1 : Math.max(0, 1 - unseen / (double) FADE_TICKS);
    }

    /**
     * Adds the wakes at (x, z) of sources riding the free surface at stillY (excluding entity id exclude, -1 for none)
     * into out[0] (elevation, m), out[1] and out[2] (slopes d/dx, d/dz).
     */
    public static void sample(Level level, double stillY, double x, double z, int exclude, double[] out) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null) { return; }
        long tick = level.getGameTime();
        double[] local = LOCAL.get();
        synchronized (wakes) {
            for (Map.Entry<Integer, Tracked> entry : wakes.sources.entrySet()) {
                Tracked t = entry.getValue();
                if (entry.getKey() == exclude || Math.abs(t.stillY - stillY) > WaveRegion.LEVEL_TOLERANCE) { continue; }
                double w = presence(t, tick);
                if (w == 0) { continue; }
                double f = fade(t, tick);
                add(t, t.current, w * f, x, z, local, out);
                add(t, t.previous, w * (1 - f), x, z, local, out);
            }
        }
    }

    public static double elevation(Level level, double stillY, double x, double z, int exclude) {
        double[] out = new double[3];
        sample(level, stillY, x, z, exclude, out);
        return out[0];
    }

    private static void add(Tracked t, KelvinWake.Pattern p, double weight, double x, double z, double[] local, double[] out) {
        if (p == null || weight == 0) { return; }
        double dx = x - t.originX, dz = z - t.originZ;
        // Body frame: forward along the motion, lateral to its left; the patterns are port-starboard symmetric.
        double forward = dx * t.dirX + dz * t.dirZ, lateral = -dx * t.dirZ + dz * t.dirX;
        p.sample(forward, lateral, local);
        if (local[0] == 0 && local[1] == 0 && local[2] == 0) { return; }
        out[0] += weight * local[0];
        out[1] += weight * (local[1] * t.dirX - local[2] * t.dirZ);
        out[2] += weight * (local[1] * t.dirZ + local[2] * t.dirX);
    }

    /** Largest wake elevation present in the level, m (for the band the surface can move through). */
    public static double maxElevation(Level level) {
        LevelWakes wakes = LEVELS.get(level);
        return wakes == null ? 0 : wakes.maxElevation;
    }

    /** Calm zones following wakes outside persistent regions (recomputed every tick). */
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
        double span = (points - 1) * s, cx = x0 + span / 2, cz = z0 + span / 2;
        boolean near = false;
        synchronized (wakes) {
            for (Tracked tr : wakes.sources.values()) {
                if (Math.abs(tr.stillY - region.stillWaterY()) > WaveRegion.LEVEL_TOLERANCE) { continue; }
                double reach = reach(tr) + span;
                if (Math.hypot(cx - tr.originX, cz - tr.originZ) < reach) { near = true; break; }
            }
        }
        if (!near) { return; }
        double[] out = new double[3];
        for (int i = 0; i < points; i++) for (int k = 0; k < points; k++) {
            out[0] = 0; out[1] = 0; out[2] = 0;
            sample(level, region.stillWaterY(), x0 + i * s, z0 + k * s, -1, out);
            int p = i * points + k;
            height[p] += out[0]; slopeX[p] += out[1]; slopeZ[p] += out[2];
        }
    }

    private static double reach(Tracked t) {
        double r = 0;
        for (KelvinWake.Pattern p : new KelvinWake.Pattern[]{t.current, t.previous}) {
            if (p != null) { r = Math.max(r, Math.hypot(p.length() + Math.abs(p.bowX()), p.halfWidth()) + 2); }
        }
        return r;
    }

    /** End of tick: drop faded sources, refresh the elevation bound and the calm zones. */
    private static void endTick(Level level) {
        LevelWakes wakes = LEVELS.get(level);
        if (wakes == null) { return; }
        long tick = level.getGameTime();
        List<WaveRegion> persistent = SeaSurface.persistentRegions(level), zones = new ArrayList<>();
        double max = 0;
        synchronized (wakes) {
            for (Iterator<Tracked> it = wakes.sources.values().iterator(); it.hasNext(); ) {
                Tracked t = it.next();
                boolean idle = t.current == null && (t.previous == null || fade(t, tick) >= 1);
                if (presence(t, tick) == 0 || (idle && tick - t.seen > UNSEEN_TICKS)) { it.remove(); continue; }
                double length = 0;
                for (KelvinWake.Pattern p : new KelvinWake.Pattern[]{t.current, t.previous}) {
                    if (p != null) { max = Math.max(max, p.maxElevation()); length = Math.max(length, reach(t)); }
                }
                if (length == 0 || SeaSurface.regionAt(persistent, t.originX, t.originZ) != null) { continue; }
                double radius = Math.min(WaveRegion.MAX_RADIUS, Math.ceil((length + ZONE_GRID) / ZONE_STEP) * ZONE_STEP);
                double x = Math.floorDiv(Mth.floor(t.originX), ZONE_GRID) * ZONE_GRID + ZONE_GRID / 2.0;
                double z = Math.floorDiv(Mth.floor(t.originZ), ZONE_GRID) * ZONE_GRID + ZONE_GRID / 2.0;
                WaveRegion zone = new WaveRegion(x, z, radius, t.stillY, WaveField.NONE);
                if (!zones.contains(zone)) { zones.add(zone); }
            }
        }
        wakes.maxElevation = max;
        wakes.zones = List.copyOf(zones);
    }

    @SubscribeEvent public static void entityTick(EntityTickEvent.Post event) {
        Entity entity = event.getEntity();
        if (entity instanceof Source source) {
            update(entity.level(), entity.getId(), source.wakeDesign(), source.wakeOrigin(), source.wakeVelocity());
        } else if (entity instanceof Boat boat) {
            update(boat.level(), boat.getId(), vanillaBoat(boat.getBbWidth(), boat.getBbHeight()), new Vec3d(boat.getX(), boat.getY(), boat.getZ()), null);
        }
    }

    @SubscribeEvent public static void levelTick(LevelTickEvent.Post event) { endTick(event.getLevel()); }
}
