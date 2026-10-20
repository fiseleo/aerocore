package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * A towing tank and a heeling test for any vehicle in still water (offline benches), stepped by VehicleDynamics with its
 * controls at rest. Towed, the vehicle runs along +x at a fixed speed, free to heave and trim but held in sway, roll and
 * yaw; its resistance is the fluid loads' (buoyancy, lift, drag, radiation) component against the tow, averaged over the
 * run's last two seconds after it has settled. A run whose trim passes LOST_TRIM_DEG either way (it dived or reared up)
 * is stopped there and marked lost, its figures those of that moment (resistance NaN). Heeled, it is held at the angle with no speed, free to sink and trim, and
 * its righting arm is the fluid moment about the longitudinal axis over its weight (GZ, positive back toward upright).
 * Before either it is settled where it floats at rest (lightly damped while settling only, not while measuring).
 */
public final class TowingTank {
    private TowingTank() {}

    /** The step used (s), the settling time at rest (s), a run's length (s) and the window it is averaged over (s). */
    public static final double DT = 0.005, SETTLE_S = 6, RUN_S = 10, AVERAGE_S = 2;
    /** Trim (deg, either way) at which a run is given up as lost. */
    public static final double LOST_TRIM_DEG = 30;

    /**
     * A steady run: total resistance and the part of it that is air drag (from the parts' "airDragN" diagnostics), N;
     * trim (deg, bow up); rise of the centre of mass above where it floats at rest (m, negative when it squats); and
     * the swing of the trim over the averaging window (deg: large when it porpoises); lost when it dived or reared up.
     */
    public record Run(double speedMps, double resistanceN, double airN, double trimDeg, double riseM, double trimSwingDeg, boolean lost) {}

    /** A righting arm (m, positive back toward upright) at a heel (deg, starboard down). */
    public record Heel(double heelDeg, double gzM) {}

    /** Floating at rest in still water at y = 0 (heave and pitch only), its origin started on the surface. */
    public static RigidBodyState rest(Vehicle v, FluidState water, FluidState air, Vec3d gravity) {
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleEnvironment env = VehicleEnvironment.calm(0, water, air);
        for (int i = 0; i < SETTLE_S / DT; i++) {
            VehicleDynamics.step(v, st, env, Map.of(), gravity, List.of(), null, i * DT, DT);
            RigidBodyState s = held(st.body(), v.properties(), 0, 0);
            st.setBody(new RigidBodyState(s.position(), s.velocity().scale(0.98), s.orientation(), s.angularMomentum().scale(0.98)));
        }
        RigidBodyState s = st.body();
        return new RigidBodyState(s.position(), Vec3d.ZERO, s.orientation(), Vec3d.ZERO);
    }

    /** Runs at each speed (m/s), each from rest. */
    public static List<Run> tow(Vehicle v, FluidState water, FluidState air, Vec3d gravity, double... speeds) {
        RigidBodyState rest = rest(v, water, air, gravity);
        List<Run> out = new ArrayList<>();
        for (double speed : speeds) { out.add(run(v, rest, water, air, gravity, speed)); }
        return out;
    }

    /** One run at a speed (m/s) from a state at rest (rest()). */
    public static Run run(Vehicle v, RigidBodyState rest, FluidState water, FluidState air, Vec3d gravity, double speed) {
        VehicleEnvironment env = VehicleEnvironment.calm(0, water, air);
        Vec3d up = gravity.lengthSquared() > 0 ? gravity.normalized().scale(-1) : Vec3d.Y;
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setBody(held(rest, v.properties(), speed, 0));
        double resistance = 0, airN = 0, trim = 0, rise = 0, low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
        int samples = 0;
        for (int i = 0; i < RUN_S / DT; i++) {
            VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, Map.of(), gravity, List.of(), null, i * DT, DT);
            st.setBody(held(st.body(), v.properties(), speed, 0));
            if (Math.abs(trimDeg(st.body())) > LOST_TRIM_DEG) {
                return new Run(speed, Double.NaN, Double.NaN, trimDeg(st.body()), st.body().position().dot(up) - rest.position().dot(up), Double.NaN, true);
            }
            if (i * DT < RUN_S - AVERAGE_S) { continue; }
            Wrench fluid = fluid(r);
            resistance += -fluid.force().dot(Vec3d.X);
            for (Map<String, Double> d : r.diagnostics().values()) { airN += d.getOrDefault("airDragN", 0.0); }
            double t = trimDeg(st.body());
            trim += t; low = Math.min(low, t); high = Math.max(high, t);
            rise += st.body().position().dot(up) - rest.position().dot(up);
            samples++;
        }
        return new Run(speed, resistance / samples, airN / samples, trim / samples, rise / samples, high - low, false);
    }

    /** Righting arms at each heel (deg), each settled from rest at that heel. */
    public static List<Heel> heel(Vehicle v, FluidState water, FluidState air, Vec3d gravity, double... heelsDeg) {
        RigidBodyState rest = rest(v, water, air, gravity);
        VehicleEnvironment env = VehicleEnvironment.calm(0, water, air);
        double weight = v.properties().massKg() * gravity.length();
        List<Heel> out = new ArrayList<>();
        for (double heelDeg : heelsDeg) {
            double phi = Math.toRadians(heelDeg);
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setBody(held(rest, v.properties(), 0, phi));
            Wrench fluid = Wrench.ZERO;
            for (int i = 0; i < SETTLE_S / DT; i++) {
                VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, Map.of(), gravity, List.of(), null, i * DT, DT);
                RigidBodyState s = held(st.body(), v.properties(), 0, phi);
                st.setBody(new RigidBodyState(s.position(), s.velocity().scale(0.98), s.orientation(), s.angularMomentum().scale(0.98)));
                fluid = fluid(r);
            }
            // heel is a rotation about +x by -phi (starboard, +z, down); righting turns it back, about +x
            out.add(new Heel(heelDeg, weight > 0 ? fluid.torque().dot(Vec3d.X) / weight : 0));
        }
        return out;
    }

    /** The fluid's loads on the vehicle: everything but its weight, thrust, contact and the inertia terms. */
    private static Wrench fluid(VehicleDynamics.Result r) {
        return r.load(AppliedForce.Kind.BUOYANCY).add(r.load(AppliedForce.Kind.LIFT)).add(r.load(AppliedForce.Kind.DRAG)).add(r.load(AppliedForce.Kind.RADIATION));
    }

    /** Trim (deg, bow up) of a body heading along +x. */
    public static double trimDeg(RigidBodyState s) {
        Vec3d forward = s.orientation().rotate(Vec3d.X);
        return Math.toDegrees(Math.atan2(forward.y(), Math.hypot(forward.x(), forward.z())));
    }

    /** Held along +x at a speed and a heel (rad, starboard down): free only to heave and trim. */
    private static RigidBodyState held(RigidBodyState s, RigidBodyProperties p, double speed, double heel) {
        Vec3d omega = s.angularVelocity(p);
        double pitch = Math.toRadians(trimDeg(s));
        Orientation q = Orientation.rotationVector(Vec3d.Z.scale(pitch)).multiply(Orientation.rotationVector(Vec3d.X.scale(-heel)));
        Vec3d pitchOnly = new Vec3d(0, 0, omega.z());
        return new RigidBodyState(new Vec3d(s.position().x(), s.position().y(), 0), new Vec3d(speed, s.velocity().y(), 0), q, p.inertia().applyWorld(q, pitchOnly));
    }
}
