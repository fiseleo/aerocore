package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;

import java.util.Map;

/**
 * One part's view of a step (VehicleDynamics.prepare): what it reads (the vehicle's start state, the environment,
 * its inputs, rotor speeds) and where it writes its loads. All world quantities: forces in N with offsets from the
 * vehicle's COM, torques about the COM. The part frame view (partState, partProperties) is the same rigid body seen
 * with the part's axes, so models written for a whole body in its own frame (a hull) can be evaluated for a mounted
 * part unchanged.
 */
public final class PartStep {
    private final VehicleDynamics.Collector out;
    private final Vehicle.Installed installed;
    private final RigidBodyState state, partState;
    private final RigidBodyProperties properties;
    private final Vec3d omega, gravity;
    private final VehicleEnvironment environment;
    private final Map<String, Double> controls;
    private final double time, dt;

    PartStep(VehicleDynamics.Collector out, Vehicle.Installed installed, RigidBodyState state, RigidBodyProperties properties, Vec3d omega,
             VehicleEnvironment environment, Map<String, Double> controls, Vec3d gravity, double time, double dt) {
        this.out = out; this.installed = installed; this.state = state; this.properties = properties; this.omega = omega;
        this.environment = environment; this.controls = controls; this.gravity = gravity; this.time = time; this.dt = dt;
        Orientation q = state.orientation().multiply(installed.rotation());
        // Same COM point and velocity; the angular momentum is a world vector, unchanged by the choice of body axes.
        this.partState = new RigidBodyState(state.position(), state.velocity(), q, state.angularMomentum());
    }

    // ----- reading -----
    public String id() { return installed.id(); }
    public Mount mount() { return installed.mount(); }
    public RigidBodyState state() { return state; }
    public RigidBodyProperties properties() { return properties; }
    public Vec3d angularVelocity() { return omega; }
    public VehicleEnvironment environment() { return environment; }
    public Vec3d gravity() { return gravity; }
    public double time() { return time; }
    public double dt() { return dt; }

    /** The vehicle's state with the part's axes (same COM point, velocity and angular momentum). */
    public RigidBodyState partState() { return partState; }
    /** The vehicle's mass, COM and inertia expressed in the part frame, to go with partState. */
    public RigidBodyProperties partProperties() { return installed.partProperties(); }

    /** A part-frame point in the vehicle frame. */
    public Vec3d vehiclePoint(Vec3d partPoint) { return installed.mount().position().add(installed.rotation().rotate(partPoint)); }
    /** World offset from the COM of a part-frame point. */
    public Vec3d offset(Vec3d partPoint) { return state.worldOffset(properties, vehiclePoint(partPoint)); }
    /** World position of a part-frame point. */
    public Vec3d point(Vec3d partPoint) { return state.position().add(offset(partPoint)); }
    /** World direction of a part-frame direction. */
    public Vec3d axis(Vec3d partDirection) { return state.orientation().rotate(installed.rotation().rotate(partDirection)); }
    /** World velocity of a part-frame point. */
    public Vec3d velocity(Vec3d partPoint) { return state.velocity().add(omega.cross(offset(partPoint))); }

    /** The input bound to role (offset + scale * channel, summed over bindings), or fallback when nothing is bound. */
    public double input(String role, double fallback) {
        double sum = 0;
        boolean bound = false;
        for (Binding b : installed.bindings()) {
            if (!b.role().equals(role)) { continue; }
            bound = true;
            Double value = controls.get(b.channel());
            sum += b.offset() + b.scale() * (value == null || !Double.isFinite(value) ? 0 : value);
        }
        return bound ? sum : fallback;
    }

    /** This part's i-th internal state at the step start (Part.internalStates). */
    public double internal(int i) {
        if (i < 0 || i >= installed.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        return out.internals[installed.firstInternal() + i];
    }
    /** Sets this part's i-th internal state for the end of the step (committed by VehicleDynamics.finish). */
    public void setInternal(int i, double value) {
        if (i < 0 || i >= installed.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Internal state must be finite"); }
        out.nextInternals[installed.firstInternal() + i] = value;
    }

    /** Speed of this part's k-th rotor at the step start, rad/s. */
    public double rotorSpeed(int k) { return out.rotorSpeed(installed.firstRotor() + k); }
    /** Waterplane radius last recorded for this part's k-th radiation point (unsmoothed), m. */
    public double lastRadiationRadius(int k) { return out.lastRadius(installed.firstRadiation() + k); }
    /** Thrust in water and in air (N, by the disk's immersion) and diameter (m) of the propeller whose race this part sits in, or null. */
    public double[] slipstream() { return installed.slipstreamFrom() == null ? null : out.thrust(installed.slipstreamFrom()); }

    // ----- writing -----
    /** A load that is also drawn (its kind sorts it in the result's sums). */
    public void force(AppliedForce force) { out.force(force, true); }
    /** A load whose wrench is given in full (for example with a couple or a moment not at one point), drawn as display. */
    public void load(Wrench wrench, AppliedForce.Kind kind) { out.wrench(wrench, kind); }
    /** An arrow for display only; the load must be given by load() as well. */
    public void display(AppliedForce force) { out.force(force, false); }
    /** Wave excitation or ideal-fluid terms, counted with the added-mass reaction as inertia. */
    public void inertia(Wrench wrench) { out.inertia(wrench); }
    /** A 6 x 6 added mass (world axes, about the COM, v then omega), taken implicitly. */
    public void addedMass(double[][] world) { out.addedMass(world); }
    /** An isotropic point added mass (taken implicitly; it also drifts with the body as entrained water) and the excitation force it carries. */
    public void pointMass(RigidBodyDynamics.PointMass mass, Vec3d excitation) { out.pointMass(mass, excitation); }
    /** A point drag, taken implicitly. */
    public void pointDrag(RigidBodyDynamics.PointDrag drag) { out.pointDrag(drag); }
    /**
     * A torque on this part's k-th rotor about its axis (N m) and its derivative with the rotor's speed (N m s/rad).
     * external: from outside the vehicle (a fluid on a propeller); otherwise from the vehicle itself (a motor's stator),
     * whose reaction the vehicle then feels.
     */
    public void rotorTorque(int k, double torqueNm, double perRadPerSecond, boolean external) {
        out.rotorTorque(installed.firstRotor() + k, torqueNm, perRadPerSecond, external);
    }
    /** Waterplane radius (m) of this part's k-th radiation point at the step start, and the share (0..1) of its memory force to apply. */
    public void radiationRadius(int k, double radiusM, double share) { out.radiation(installed.firstRadiation() + k, radiusM, share); }
    /** Publishes this part's thrust in water and in air (N) and its diameter (m) for surfaces in its race. */
    public void thrust(double waterThrustN, double airThrustN, double diameterM) { out.thrust(installed.id(), waterThrustN, airThrustN, diameterM); }
    /** A number shown to players or used by benches (key in camelCase with units, for example "thrustN"). */
    public void diagnostic(String key, double value) { out.diagnostic(installed.id(), key, value); }
}
