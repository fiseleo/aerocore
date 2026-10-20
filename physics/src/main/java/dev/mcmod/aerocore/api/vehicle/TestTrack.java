package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

/**
 * An offline test track (0.24): a vehicle on a ground (Ground: flat, slopes, boxes) in still air, driven by a driver
 * that sets the controls each step, for benches and tests (0-100 km/h, stopping, the skidpad, climbing). Like
 * TowingTank it runs thousands of substeps a second: for design screens and tests, never in the game's tick.
 */
public final class TestTrack {
    private TestTrack() {}

    /** The substep, s. */
    public static final double DT = 0.005;
    /** Standard gravity, m/s^2. */
    public static final double G = 9.80665;

    /** Sets the controls for each step from the time (s) and the vehicle's state. */
    @FunctionalInterface
    public interface Driver { Map<String, Double> controls(double timeS, VehicleState state); }

    /** One recorded step: time (s), the vehicle frame's origin and velocity (world), speed (m/s) and the parts' diagnostics. */
    public record Sample(double timeS, Vec3d origin, Vec3d velocity, double speedMps, Map<String, Map<String, Double>> diagnostics) {}

    /**
     * A new state with the vehicle frame's origin at a point (world), turned by yaw (rad, about +y; 0 faces +x), let
     * down on the ground and settled for some seconds with the given controls (brakes on, say).
     */
    public static VehicleState settle(Vehicle v, Ground ground, FluidState air, Vec3d origin, double yawRad, Map<String, Double> controls, double seconds) {
        VehicleState st = v.newState(origin, Orientation.rotationVector(Vec3d.Y.scale(yawRad)), DT);
        drive(v, st, ground, air, (t, s) -> controls, seconds, s -> false, Integer.MAX_VALUE);
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), Vec3d.ZERO, b.orientation(), Vec3d.ZERO));
        return st;
    }

    /**
     * Steps the vehicle for at most some seconds, until stop says so, recording every n-th step (and the last); the
     * state is left where the drive ended. Time in the samples starts at 0.
     */
    public static List<Sample> drive(Vehicle v, VehicleState st, Ground ground, FluidState air, Driver driver, double seconds, Predicate<VehicleState> stop,
                                     int every) {
        VehicleEnvironment environment = VehicleEnvironment.land(ground, air);
        Vec3d gravity = Vec3d.Y.scale(-G);
        List<Sample> samples = new ArrayList<>();
        int steps = (int) Math.round(seconds / DT);
        for (int i = 0; i < steps; i++) {
            double t = i * DT;
            VehicleDynamics.Result r = VehicleDynamics.step(v, st, environment, driver.controls(t, st), gravity, List.of(), null, t, DT);
            boolean last = stop.test(st) || i == steps - 1;
            if (every > 0 && ((i + 1) % every == 0 || last)) {
                Vec3d velocity = st.body().velocity();
                samples.add(new Sample(t + DT, st.origin(), velocity, velocity.length(), r.diagnostics()));
            }
            if (last) { break; }
        }
        return samples;
    }

    /**
     * 0.36: sets each vehicle's controls (by its index in the convoy) and any further loads on it (world offsets from its
     * COM: a push for a tractor with no engine) from the time and its state.
     */
    public interface ConvoyDriver {
        Map<String, Double> controls(int vehicle, double timeS, VehicleState state);
        default List<AppliedForce> external(int vehicle, double timeS, VehicleState state) { return List.of(); }
    }

    /** 0.36: one recorded step of a convoy: the time, each vehicle's sample and what each joint carried. */
    public record ConvoySample(double timeS, List<Sample> vehicles, List<ContactSolver.JointResult> joints) {}

    /**
     * 0.36: where a follower's frame origin goes (world) for its coupling point to be on its leader's, turned by yaw (rad,
     * about +y), level: the hitched start of a convoy.
     */
    public static Vec3d hitchedOrigin(Vehicle leader, VehicleState leaderState, Coupling coupling, double yawRad) {
        Vec3d at = leaderState.body().worldPoint(leader.properties(), coupling.leaderPoint());
        return at.subtract(Orientation.rotationVector(Vec3d.Y.scale(yawRad)).rotate(coupling.followerPoint()));
    }

    /**
     * 0.36: steps vehicles joined by couplings together (a tractor and its trailers) for at most some seconds, until stop
     * says so, recording every n-th step (and the last): each prepares its loads, one ContactSolver.solveAll takes the
     * joints, each finishes, and separateAll takes out the joints' drift. A follower gets its leader's controls on the
     * coupling's forwarded channels (list the joints leader first along a chain). The states are left where it ended.
     */
    public static List<ConvoySample> convoy(List<Vehicle> vehicles, List<VehicleState> states, List<ContactSolver.Joint> joints, Ground ground, FluidState air,
                                            ConvoyDriver driver, double seconds, Predicate<List<VehicleState>> stop, int every) {
        if (vehicles.size() != states.size()) { throw new IllegalArgumentException("A state for each vehicle"); }
        VehicleEnvironment environment = VehicleEnvironment.land(ground, air);
        Vec3d gravity = Vec3d.Y.scale(-G);
        int n = vehicles.size(), steps = (int) Math.round(seconds / DT);
        List<ConvoySample> samples = new ArrayList<>();
        VehicleDynamics.Prepared[] prepared = new VehicleDynamics.Prepared[n];
        VehicleDynamics.Result[] results = new VehicleDynamics.Result[n];
        for (int i = 0; i < steps; i++) {
            double t = i * DT;
            List<Map<String, Double>> controls = new ArrayList<>(n);
            for (int k = 0; k < n; k++) { controls.add(new HashMap<>(driver.controls(k, t, states.get(k)))); }
            for (ContactSolver.Joint j : joints) {
                for (String channel : j.coupling().forwarded()) {
                    Double value = controls.get(j.leader()).get(channel);
                    if (value != null) { controls.get(j.follower()).put(channel, value); }
                }
            }
            List<ContactSolver.Body> bodies = new ArrayList<>(n);
            for (int k = 0; k < n; k++) {
                prepared[k] = VehicleDynamics.prepare(vehicles.get(k), states.get(k), environment, controls.get(k), gravity, driver.external(k, t, states.get(k)), t, DT);
                bodies.add(new ContactSolver.Body(prepared[k].damped(), vehicles.get(k).properties()));
            }
            ContactSolver.Solution solved = ContactSolver.solveAll(bodies, List.of(), joints, DT);
            for (int k = 0; k < n; k++) {
                ContactSolver.BodyResult r = solved.bodies().get(k);
                results[k] = VehicleDynamics.finish(prepared[k], r.state(), r.wrench(), r.forces(), 0, null);
            }
            List<ContactSolver.Body> now = new ArrayList<>(n);
            for (int k = 0; k < n; k++) { now.add(new ContactSolver.Body(states.get(k).body(), vehicles.get(k).properties())); }
            List<RigidBodyState> separated = ContactSolver.separateAll(now, b -> List.of(), joints);
            for (int k = 0; k < n; k++) { if (separated.get(k) != now.get(k).state()) { states.get(k).setBody(separated.get(k)); } }
            boolean last = stop.test(states) || i == steps - 1;
            if (every > 0 && ((i + 1) % every == 0 || last)) {
                List<Sample> each = new ArrayList<>(n);
                for (int k = 0; k < n; k++) {
                    Vec3d velocity = states.get(k).body().velocity();
                    each.add(new Sample(t + DT, states.get(k).origin(), velocity, velocity.length(), results[k].diagnostics()));
                }
                samples.add(new ConvoySample(t + DT, List.copyOf(each), solved.joints()));
            }
            if (last) { break; }
        }
        return samples;
    }
}
