package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveRadiation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The changing part of one vehicle: rigid-body state (COM, velocity, orientation, angular momentum), the speed of
 * each shaft train (rad/s of its root rotor), the heave wave-radiation memory with the last waterplane radii, and what
 * the last step reported (forces for arrows, part diagnostics). Mutable; VehicleDynamics.finish advances it. The
 * substep dt is fixed for the radiation memory.
 */
public final class VehicleState {
    private final Vehicle vehicle;
    private final double dt;
    private RigidBodyState body;
    final double[] trainSpeeds;
    final WaveRadiation.Memory radiation;
    final double[] radii;
    /** Whether the radiation memory has taken its first radii (VehicleDynamics.finish primes it at the first step). */
    boolean primed;
    List<AppliedForce> forces = List.of();
    Map<String, Map<String, Double>> diagnostics = Map.of();

    VehicleState(Vehicle vehicle, RigidBodyState body, double dt) {
        if (!(dt > 0) || !Double.isFinite(dt)) { throw new IllegalArgumentException("Substep must be positive"); }
        this.vehicle = vehicle; this.body = body; this.dt = dt;
        trainSpeeds = new double[vehicle.trains()];
        radiation = vehicle.radiationPoints().isEmpty() ? null : new WaveRadiation.Memory(vehicle.radiationPoints(), dt);
        radii = new double[vehicle.radiationPoints().size()];
    }

    public Vehicle vehicle() { return vehicle; }
    public double dt() { return dt; }
    public RigidBodyState body() { return body; }
    /** Replaces the rigid-body state (a contact correction, a teleport); shaft speeds and wave memory stay. */
    public void setBody(RigidBodyState body) { this.body = body; }
    public double trainSpeed(int train) { return trainSpeeds[train]; }
    public void setTrainSpeed(int train, double radPerSecond) {
        if (!Double.isFinite(radPerSecond)) { throw new IllegalArgumentException("Speed must be finite"); }
        trainSpeeds[train] = radPerSecond;
    }
    /** Speed of a rotor (index in Vehicle.rotors), rad/s. */
    public double rotorSpeed(int rotor) {
        Vehicle.Rotor r = vehicle.rotors().get(rotor);
        return r.ratio() * trainSpeeds[r.train()];
    }
    /** Loads of the last step, for arrows (offsets from that step's COM). */
    public List<AppliedForce> forces() { return forces; }
    /** Diagnostics of the last step by part id. */
    public Map<String, Map<String, Double>> diagnostics() { return diagnostics; }
    /** World position of the vehicle frame's origin. */
    public Vec3d origin() { return vehicle.origin(body); }

    /** Rigid-body state and train speeds (the wave memory is not saved; it refills within its 8 s span). */
    public Map<String, Object> toJson() {
        Orientation q = body.orientation();
        List<Object> speeds = new ArrayList<>();
        for (double s : trainSpeeds) { speeds.add(s); }
        return Fields.object("position", Fields.vector(body.position()), "velocity", Fields.vector(body.velocity()),
                "orientation", List.of(q.w(), q.x(), q.y(), q.z()), "angularMomentum", Fields.vector(body.angularMomentum()), "trainSpeeds", speeds);
    }

    public static VehicleState fromJson(Vehicle vehicle, Object json, double dt) {
        Fields f = Fields.of(json, "state");
        List<Object> q = f.list("orientation");
        if (q.size() != 4) { throw new IllegalArgumentException("state.orientation: expected [w, x, y, z]"); }
        double[] c = new double[4];
        for (int i = 0; i < 4; i++) {
            if (!(q.get(i) instanceof Number n)) { throw new IllegalArgumentException("state.orientation: expected numbers"); }
            c[i] = n.doubleValue();
        }
        double norm = Math.sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + c[3] * c[3]);
        if (!(norm > 0)) { throw new IllegalArgumentException("state.orientation: zero quaternion"); }
        Orientation orientation = new Orientation(c[0] / norm, c[1] / norm, c[2] / norm, c[3] / norm);
        VehicleState state = new VehicleState(vehicle, new RigidBodyState(f.vec("position"), f.vec("velocity", Vec3d.ZERO), orientation,
                f.vec("angularMomentum", Vec3d.ZERO)), dt);
        List<Object> speeds = f.list("trainSpeeds", List.of());
        // A blueprint changed since saving may have another number of trains: keep what still fits.
        for (int i = 0; i < Math.min(speeds.size(), state.trainSpeeds.length); i++) {
            if (speeds.get(i) instanceof Number n && Double.isFinite(n.doubleValue())) { state.trainSpeeds[i] = n.doubleValue(); }
        }
        return state;
    }
}
