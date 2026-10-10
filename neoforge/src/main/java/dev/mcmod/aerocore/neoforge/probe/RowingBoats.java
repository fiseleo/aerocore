package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.BowSpray;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Rowboat;
import dev.mcmod.aerocore.api.UnsteadyWake;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveRadiation;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.mixin.BoatAccessor;
import net.minecraft.core.BlockPos;
import net.minecraft.tags.FluidTags;
import net.minecraft.util.Mth;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.animal.Animal;
import net.minecraft.world.entity.npc.AbstractVillager;
import net.minecraft.world.entity.monster.AbstractIllager;
import net.minecraft.world.entity.monster.Zombie;
import net.minecraft.world.entity.monster.AbstractSkeleton;
import net.minecraft.world.entity.monster.piglin.AbstractPiglin;
import net.minecraft.world.entity.monster.Witch;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.entity.vehicle.Boat;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.Vec3;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Vanilla boats as rowing prams (Rowboat): real masses (hull and passengers), buoyancy on the shared sea surface,
 * friction, form and wave resistance (from each boat's own unsteady wake), radiation, and a rower's power through the
 * oars. Runs on the side that moves the boat (the controlling player's client, or the server for empty and
 * mob-ridden boats) in place of vanilla's floatBoat and controlBoat, and hands vanilla the tick's displacement so its
 * block collision still applies. Pitch and roll are kept here for the renderer; boats moved by another side only
 * integrate their attitude. Away from water (on land, in the air, in rapids) vanilla's rules stay.
 * <p>
 * Player input is mapped to oars (a control rule, not physics): forward rows both oars, back backs water with both,
 * left alone pulls the right oar, forward + left pulls the right oar with full and the left with 40 % of its share of
 * the power, back + left backs the left oar fully and the right at 40 %; right mirrors left.
 */
public final class RowingBoats {
    /** Paddle animation advance per tick for Rowboat.STROKE_RATE (vanilla: pi/8, 75 strokes a minute). */
    public static final float PADDLE_STEP = (float) (2 * Math.PI * Rowboat.STROKE_RATE * MinecraftUnits.TICK_SECONDS);
    private static final double RHO = FluidState.FRESH_WATER_20C.densityKgPerM3(), DT = MinecraftUnits.TICK_SECONDS;
    /** Seated people: own moment of inertia about their centre of mass, 4-7 kg m^2 (Santschi et al. 1963); the middle. */
    private static final double SEATED_INERTIA = 5.5;
    /** A boat resting this long on still vanilla water stops being simulated until something disturbs it. */
    private static final int SLEEP_TICKS = 40;

    private static final class Physics {
        double pitch, roll, pitchRate, rollRate, yawRate, prevPitch, prevRoll;
        Vec3d velocity;
        Vec3 expected = Vec3.ZERO;
        WaveRadiation.Memory memory;
        int quiet;
        boolean asleep, simulated;
        Rowboat.Result last;
    }
    private static final Map<Boat, Physics> STATES = Collections.synchronizedMap(new WeakHashMap<>());

    private RowingBoats() {}

    private static Physics physics(Boat boat) {
        return STATES.computeIfAbsent(boat, b -> {
            Physics p = new Physics();
            p.velocity = MinecraftUnits.toMetersPerSecond(b.getDeltaMovement());
            return p;
        });
    }

    /** The people and animals aboard, at vanilla's seats. */
    static List<Rowboat.Passenger> passengers(Boat boat) {
        List<Entity> riders = boat.getPassengers();
        List<Rowboat.Passenger> list = new ArrayList<>();
        for (int i = 0; i < riders.size(); i++) {
            Entity e = riders.get(i);
            double forward = riders.size() > 1 ? (i == 0 ? 0.2 : -0.6) : 0;
            if (riders.size() > 1 && e instanceof Animal) { forward += 0.2; }
            double mass = EntityMasses.massKg(e);
            if (seated(e)) {
                list.add(new Rowboat.Passenger(mass, forward, Rowboat.FLOOR + Rowboat.SEATED_HEIGHT, SEATED_INERTIA));
            } else {
                // Other riders as a uniform box of their hitbox standing on the floor (a rule, not data).
                double w = e.getBbWidth(), h = e.getBbHeight();
                list.add(new Rowboat.Passenger(mass, forward, Rowboat.FLOOR + h / 2, mass * (w * w + h * h) / 12));
            }
        }
        return list;
    }

    private static boolean seated(Entity e) {
        return e instanceof Player || e instanceof AbstractVillager || e instanceof AbstractIllager || e instanceof Zombie
                || e instanceof AbstractSkeleton || e instanceof AbstractPiglin || e instanceof Witch;
    }

    /** Still-water draft of the boat with its riders, m (the gunwale height if it cannot float). */
    public static double draft(Boat boat) {
        double draft = Rowboat.draft(Rowboat.mass(passengers(boat)).massKg(), RHO);
        return Double.isNaN(draft) ? Rowboat.DEPTH : draft;
    }

    /** The boat's bottom as the pressure that raises its wake, and its blunt transom bow. */
    public static Wakes.Design wakeDesign(Boat boat) {
        double draft = draft(boat);
        UnsteadyWake.Forcing forcing = Rowboat.wakeForcing(draft, RHO);
        return new Wakes.Design() {
            @Override public UnsteadyWake.Forcing forcing() { return forcing; }
            /** Rowing speeds make waves 1-3 m long: 0.25 m spacing, a 32 m grid. */
            @Override public double spacing() { return 0.25; }
            @Override public Wakes.Bow bow(double speed) {
                return new Wakes.Bow(Rowboat.LENGTH / 2, Rowboat.BEAM / 2, BowSpray.displacementBow(speed, Math.PI / 2, draft, Atmosphere.GRAVITY_MPS2));
            }
        };
    }

    /** Surface around the boat (sea region surface without its own wake, or vanilla still water). */
    private static Rowboat.Water water(Boat boat) {
        Level level = boat.level();
        double t = SeaSurface.time(level), bottom = boat.getY();
        int id = boat.getId();
        double[] wake = new double[3], sea = new double[3];
        return new Rowboat.Water() {
            @Override public void sample(double x, double z, double[] out) {
                WaveRegion region = SeaSurface.governing(level, x, z, bottom - 1, bottom + Rowboat.DEPTH);
                if (region != null) {
                    region.surface(x, z, t, sea);
                    double eta = sea[0], sx = sea[1], sz = sea[2];
                    wake[0] = 0; wake[1] = 0; wake[2] = 0;
                    Wakes.sample(level, region.stillWaterY(), x, z, id, wake);
                    out[0] = region.stillWaterY() + eta + wake[0]; out[1] = sx + wake[1]; out[2] = sz + wake[2];
                    return;
                }
                out[0] = vanillaSurface(level, x, z, bottom); out[1] = 0; out[2] = 0;
            }
            @Override public Vec3d velocity(double x, double z) {
                WaveRegion region = SeaSurface.governing(level, x, z, bottom - 1, bottom + Rowboat.DEPTH);
                // Vanilla flowing water pushes boats itself (Entity fluid pushing), so vanilla water counts as still here.
                if (region == null) { return Vec3d.ZERO; }
                return SeaSurface.waterVelocity(region, x, bottom, z, SeaSurface.surfaceY(level, region, x, z, t, id), t);
            }
        };
    }

    /** Top of the water in the column at (x, z) near the boat's bottom, or NaN. */
    private static double vanillaSurface(Level level, double x, double z, double bottom) {
        int bx = Mth.floor(x), bz = Mth.floor(z), top = Mth.floor(bottom + Rowboat.DEPTH + 1);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        for (int y = top; y >= Mth.floor(bottom - 1); y--) {
            pos.set(bx, y, bz);
            net.minecraft.world.level.material.FluidState fluid = level.getFluidState(pos);
            if (fluid.is(FluidTags.WATER)) { return y + fluid.getHeight(level, pos); }
        }
        return Double.NaN;
    }

    /** The wind of the sea region the boat is in (tapered like its waves), at standard sea-level air density; calm elsewhere. */
    private static Rowboat.Air air(Boat boat) {
        WaveRegion region = SeaSurface.governing(boat.level(), boat.getX(), boat.getZ(), boat.getY() - 1, boat.getY() + Rowboat.DEPTH);
        if (region == null || region.wind().calm()) { return Rowboat.Air.CALM; }
        return new Rowboat.Air(region.wind().vector().scale(region.weight(boat.getX(), boat.getZ())), Rowboat.Air.CALM.density());
    }

    private static Rowboat.Water ownWake(Boat boat) {
        UnsteadyWake.Snapshot snap = Wakes.snapshot(boat.level(), boat.getId());
        if (snap == null) { return NO_WAKE; }
        return new Rowboat.Water() {
            @Override public void sample(double x, double z, double[] out) { snap.sample(x, z, out); }
            @Override public Vec3d velocity(double x, double z) { return Vec3d.ZERO; }
        };
    }
    private static final Rowboat.Water NO_WAKE = new Rowboat.Water() {
        @Override public void sample(double x, double z, double[] out) { out[0] = 0; out[1] = 0; out[2] = 0; }
        @Override public Vec3d velocity(double x, double z) { return Vec3d.ZERO; }
    };

    private static Rowboat.Oars oars(Boat boat) {
        if (!(boat.getControllingPassenger() instanceof Player)) { return Rowboat.Oars.NONE; }
        BoatAccessor in = (BoatAccessor) boat;
        boolean up = in.aerocore$inputUp(), down = in.aerocore$inputDown(), left = in.aerocore$inputLeft(), right = in.aerocore$inputRight();
        double turn = (left ? 1 : 0) - (right ? 1 : 0), drive = up && !down ? 1 : down && !up ? -1 : 0;
        if (drive == 0) { return turn > 0 ? new Rowboat.Oars(0, 0.5) : turn < 0 ? new Rowboat.Oars(0.5, 0) : Rowboat.Oars.NONE; }
        double l = 0.5, r = 0.5;
        // Forward + left eases the left oar; back + left eases the right one (the boat's bow swings left either way).
        if (turn > 0) { if (drive > 0) { l = 0.2; } else { r = 0.2; } }
        if (turn < 0) { if (drive > 0) { r = 0.2; } else { l = 0.2; } }
        return new Rowboat.Oars(drive * l, drive * r);
    }

    /**
     * In place of vanilla's floatBoat (and controlBoat) on the side that moves the boat: true when the boat is at the
     * water and was simulated. Sets the tick's displacement as the boat's delta movement and turns it (and its riders,
     * through deltaRotation).
     */
    public static boolean floatBoat(Boat boat) {
        Physics p = physics(boat);
        p.simulated = false;
        BoatAccessor access = (BoatAccessor) boat;
        if (access.aerocore$status() == Boat.Status.UNDER_FLOWING_WATER || access.aerocore$status() == Boat.Status.ON_LAND) { forget(boat, p); return false; }
        Level level = boat.level();
        Rowboat.Water water = water(boat);
        double[] probe = new double[3];
        water.sample(boat.getX(), boat.getZ(), probe);
        if (Double.isNaN(probe[0]) || probe[0] < boat.getY() - 0.5) { forget(boat, p); return false; }
        // Whatever moved the boat since our last step (collisions, pushes, fluid flow, bubble columns) as an impulse.
        Vec3 now = boat.getDeltaMovement();
        p.velocity = p.velocity.add(MinecraftUnits.fromMinecraft(now.subtract(p.expected)).scale(1 / DT));
        Rowboat.Oars oars = oars(boat);
        boolean region = SeaSurface.governing(level, boat.getX(), boat.getZ(), boat.getY() - 1, boat.getY() + Rowboat.DEPTH) != null;
        boolean still = !region && Wakes.maxElevation(level) == 0 && oars == Rowboat.Oars.NONE;
        double motion = p.velocity.length() + Math.abs(p.pitchRate) + Math.abs(p.rollRate) + Math.abs(p.yawRate);
        p.quiet = still && motion < 2e-3 ? p.quiet + 1 : 0;
        p.prevPitch = p.pitch; p.prevRoll = p.roll;
        if (p.quiet > SLEEP_TICKS) {
            // Resting on still vanilla water: nothing to integrate until something disturbs it.
            p.asleep = true; p.memory = null;
            p.velocity = Vec3d.ZERO; p.pitchRate = 0; p.rollRate = 0; p.yawRate = 0;
            boat.setDeltaMovement(Vec3.ZERO); p.expected = Vec3.ZERO;
            access.aerocore$setDeltaRotation(0);
            p.simulated = true;
            return true;
        }
        p.asleep = false;
        Rowboat.Result result = step(boat, p, water, oars);
        Rowboat.State s = result.state();
        Vec3 displacement = new Vec3(s.bottom().x() - boat.getX(), s.bottom().y() - boat.getY(), s.bottom().z() - boat.getZ());
        boat.setDeltaMovement(displacement);
        p.expected = displacement;
        float yaw = (float) Math.toDegrees(s.yaw());
        access.aerocore$setDeltaRotation(Mth.wrapDegrees(yaw - boat.getYRot()));
        boat.setYRot(yaw);
        if (level.isClientSide && boat.getControllingPassenger() instanceof Player) { boat.setPaddleState(oars.left() != 0, oars.right() != 0); }
        p.simulated = true;
        return true;
    }

    private static Rowboat.Result step(Boat boat, Physics p, Rowboat.Water water, Rowboat.Oars oars) {
        if (p.memory == null) {
            List<Vec3d> points = new ArrayList<>();
            for (double[] q : Rowboat.RADIATION_POINTS) { points.add(new Vec3d(q[0], 0, q[1])); }
            p.memory = new WaveRadiation.Memory(points, DT);
            double radius = Math.sqrt(Rowboat.BOTTOM_LENGTH * Rowboat.BOTTOM_BEAM / 4 / Math.PI);
            p.memory.setRadii(new double[]{radius, radius, radius, radius});
        }
        Rowboat.State s = new Rowboat.State(new Vec3d(boat.getX(), boat.getY(), boat.getZ()), p.velocity, Math.toRadians(boat.getYRot()),
                p.pitch, p.roll, p.yawRate, p.pitchRate, p.rollRate);
        Rowboat.Result result = Rowboat.step(s, passengers(boat), oars, water, ownWake(boat), air(boat), RHO, DT, p.memory);
        Rowboat.State n = result.state();
        p.velocity = n.velocity(); p.pitch = n.pitch(); p.roll = n.roll();
        p.pitchRate = n.pitchRate(); p.rollRate = n.rollRate(); p.yawRate = n.yawRate();
        p.last = result;
        return result;
    }

    private static void forget(Boat boat, Physics p) {
        p.prevPitch = p.pitch; p.prevRoll = p.roll;
        // Out of the water the hull rests level on vanilla's box; its tilt relaxes over about a quarter second.
        p.pitch *= 0.8; p.roll *= 0.8; p.pitchRate = 0; p.rollRate = 0; p.yawRate = 0;
        p.memory = null; p.quiet = 0;
        p.velocity = MinecraftUnits.toMetersPerSecond(boat.getDeltaMovement());
        p.expected = boat.getDeltaMovement();
    }

    /** Whether this tick's movement was ours (vanilla's controlBoat must then not add its own). */
    public static boolean controlsBoat(Boat boat) {
        Physics p = STATES.get(boat);
        return p != null && p.simulated;
    }

    /**
     * End of a boat's tick on a client that does not move it: integrate pitch and roll on the local surface, with the
     * position and heading the server sends.
     */
    public static void afterTick(Boat boat) {
        if (!boat.level().isClientSide || boat.isControlledByLocalInstance()) { return; }
        Physics p = physics(boat);
        p.prevPitch = p.pitch; p.prevRoll = p.roll;
        Rowboat.Water water = water(boat);
        double[] probe = new double[3];
        water.sample(boat.getX(), boat.getZ(), probe);
        if (Double.isNaN(probe[0]) || probe[0] < boat.getY() - 0.5) { p.pitch *= 0.8; p.roll *= 0.8; p.pitchRate = 0; p.rollRate = 0; return; }
        p.velocity = new Vec3d(boat.getX() - boat.xo, boat.getY() - boat.yo, boat.getZ() - boat.zo).scale(1 / DT);
        p.yawRate = Math.toRadians(Mth.wrapDegrees(boat.getYRot() - boat.yRotO)) / DT;
        step(boat, p, water, Rowboat.Oars.NONE);
    }

    /** Interpolated {pitch, roll} in radians for rendering (bow up and right side down positive). */
    public static float[] attitude(Boat boat, float partialTick) {
        Physics p = STATES.get(boat);
        if (p == null) { return new float[]{0, 0}; }
        return new float[]{(float) Mth.lerp(partialTick, p.prevPitch, p.pitch), (float) Mth.lerp(partialTick, p.prevRoll, p.roll)};
    }

    /**
     * The hull's pitch and roll as a world-frame rotation about its bottom centre (pitch about the boat's right axis,
     * bow up; roll about its forward axis, right side down), or null when level.
     */
    public static Quaternionf tilt(Boat boat, float partialTick) {
        float[] a = attitude(boat, partialTick);
        if (a[0] == 0 && a[1] == 0) { return null; }
        double yaw = Math.toRadians(boat.getViewYRot(partialTick));
        float fx = (float) -Math.sin(yaw), fz = (float) Math.cos(yaw), rx = (float) -Math.cos(yaw), rz = (float) -Math.sin(yaw);
        return new Quaternionf().rotateAxis(a[1], fx, 0, fz).rotateAxis(a[0], rx, 0, rz);
    }

    /** A seat offset from the boat's position (vanilla's attachment point) turned with the hull, or null when level. */
    public static Vec3 tiltSeat(Boat boat, Vec3 seat) {
        Quaternionf q = tilt(boat, 1);
        if (q == null) { return null; }
        Vector3f v = q.transform(new Vector3f((float) seat.x, (float) seat.y, (float) seat.z));
        return new Vec3(v.x, v.y, v.z);
    }

    /**
     * Camera roll (degrees, positive turns the view's right side down) of someone in this boat looking along viewYaw:
     * the hull's roll seen ahead, its pitch seen to the side.
     */
    public static float cameraRoll(Boat boat, float viewYaw, float partialTick) {
        float[] a = attitude(boat, partialTick);
        double delta = Math.toRadians(viewYaw - boat.getViewYRot(partialTick));
        return (float) Math.toDegrees(a[1] * Math.cos(delta) + a[0] * Math.sin(delta));
    }

    /** The last step's diagnostics for this boat, or null (tests and the inspect readout). */
    public static Rowboat.Result last(Boat boat) {
        Physics p = STATES.get(boat);
        return p == null ? null : p.last;
    }
}
