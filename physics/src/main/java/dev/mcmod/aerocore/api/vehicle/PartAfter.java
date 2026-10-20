package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.LinkedHashMap;

/**
 * One part's view of a finished step (Part.after, 0.24): the vehicle's end state (after contact and drift), the end
 * speeds of its rotors, what its drivetrain rows and rolling forces carried, what it kept from loads() (carried), and
 * its internal states and shown values to update for the next step. World quantities as in PartStep.
 */
public final class PartAfter {
    private final VehicleDynamics.Prepared prepared;
    private final Vehicle.Installed installed;
    private final VehicleState st;
    private final RigidBodyState state;
    private final RigidBodyProperties properties;
    private final Vec3d omega;

    PartAfter(VehicleDynamics.Prepared prepared, Vehicle.Installed installed, VehicleState st, RigidBodyState state) {
        this.prepared = prepared; this.installed = installed; this.st = st; this.state = state;
        this.properties = prepared.vehicle().properties();
        this.omega = state.angularVelocity(properties);
    }

    public String id() { return installed.id(); }
    /** The vehicle's state at the step's end. */
    public RigidBodyState state() { return state; }
    /** The vehicle's state at the step's start (as loads() saw it). */
    public RigidBodyState startState() { return prepared.start(); }
    public RigidBodyProperties properties() { return properties; }
    public Vec3d angularVelocity() { return omega; }
    public double dt() { return prepared.dt(); }
    /** The time at the step's start, s. */
    public double time() { return prepared.drive().time(); }

    /** A part-frame point in the vehicle frame. */
    public Vec3d vehiclePoint(Vec3d partPoint) { return installed.mount().position().add(installed.rotation().rotate(partPoint)); }
    /** World offset from the end COM of a part-frame point. */
    public Vec3d offset(Vec3d partPoint) { return state.worldOffset(properties, vehiclePoint(partPoint)); }
    /** World position of a part-frame point at the end. */
    public Vec3d point(Vec3d partPoint) { return state.position().add(offset(partPoint)); }
    /** World direction of a part-frame direction at the end. */
    public Vec3d axis(Vec3d partDirection) { return orientation().rotate(partDirection); }
    /** A world direction in the part frame at the end. */
    public Vec3d partDirection(Vec3d worldDirection) { return orientation().inverse().rotate(worldDirection); }
    /** World velocity of a part-frame point at the end. */
    public Vec3d velocity(Vec3d partPoint) { return state.velocity().add(omega.cross(offset(partPoint))); }
    private Orientation orientation() { return state.orientation().multiply(installed.rotation()); }

    private Vehicle.Rotor rotor(int k) {
        if (k < 0 || k >= installed.part().rotors().size()) { throw new IndexOutOfBoundsException("Rotor " + k + " of part " + installed.id()); }
        return prepared.vehicle().rotors().get(installed.firstRotor() + k);
    }
    /** Speed of this part's k-th rotor at the step's end (relative to the vehicle), rad/s. */
    public double rotorSpeed(int k) { Vehicle.Rotor r = rotor(k); return r.ratio() * prepared.trainSpeeds()[r.train()]; }
    /** Speed of this part's k-th rotor at the step's start, rad/s. */
    public double startRotorSpeed(int k) { rotor(k); return prepared.drive().startRotorSpeeds()[installed.firstRotor() + k]; }

    /** Number of rows this part made in loads(). */
    public int rows() { return prepared.drive().rows()[installed.index()]; }
    /** The torque the part's row (handle from PartStep) carried, N m (on a rotor whose coefficient is 1). */
    public double rowTorque(int row) {
        if (row < 0 || row >= rows()) { throw new IndexOutOfBoundsException("Row " + row + " of part " + installed.id()); }
        return prepared.drive().rowTorques()[prepared.drive().firstRow()[installed.index()] + row];
    }
    /** Number of rolling forces this part made in loads(). */
    public int rollingForces() { return prepared.drive().rolling()[installed.index()]; }
    /** The force (N) the part's rolling force (handle from PartStep) ended at. */
    public double rollingForce(int handle) {
        if (handle < 0 || handle >= rollingForces()) { throw new IndexOutOfBoundsException("Rolling force " + handle + " of part " + installed.id()); }
        return prepared.drive().rollingForces()[prepared.drive().firstRolling()[installed.index()] + handle];
    }
    /** What this part kept in loads() (PartStep.carry), or null. */
    public Object carried() { return prepared.drive().carried()[installed.index()]; }

    /** This part's i-th internal state as loads() left it. */
    public double internal(int i) {
        if (i < 0 || i >= installed.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        return st.internals[installed.firstInternal() + i];
    }
    /** Sets this part's i-th internal state for the next step. */
    public void setInternal(int i, double value) {
        if (i < 0 || i >= installed.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Internal state must be finite"); }
        st.internals[installed.firstInternal() + i] = value;
    }
    /** This part's i-th shown value. */
    public double shown(int i) {
        if (i < 0 || i >= installed.part().shownValues()) { throw new IndexOutOfBoundsException("Shown value " + i); }
        return st.shown[installed.firstShown() + i];
    }
    /** Sets this part's i-th shown value for clients. */
    public void show(int i, double value) {
        if (i < 0 || i >= installed.part().shownValues()) { throw new IndexOutOfBoundsException("Shown value " + i); }
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Shown value must be finite"); }
        st.shown[installed.firstShown() + i] = value;
    }
    /** A number shown to players or used by benches, added to this step's diagnostics. */
    public void diagnostic(String key, double value) {
        prepared.diagnostics().computeIfAbsent(installed.id(), k -> new LinkedHashMap<>()).put(key, value);
    }
}
