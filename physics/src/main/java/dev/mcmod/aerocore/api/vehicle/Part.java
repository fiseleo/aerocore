package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;

import java.util.List;

/**
 * A compiled part: everything derived from its design, in the part's own frame (x forward, y up, z starboard).
 * Immutable and shareable between vehicles; per-vehicle state (shaft speeds, wave memory) lives in VehicleState.
 */
public interface Part {
    PartDesign design();
    /** Mass, centre and inertia in the part frame. */
    MassProperties mass();
    /** Contact spheres in the part frame. */
    List<CollisionSphere> collision();
    /** EXTRAPOLATION issues found while compiling (paths relative to the design). */
    List<DesignIssue> issues();

    /**
     * Rotating elements (a motor's rotor, a propeller, a gearbox's input and output) that shafts can join; axis in the
     * part frame. A shaft joins the first rotor unless it names another by its port (Rotor.name, 0.24).
     */
    default List<Rotor> rotors() { return List.of(); }
    /** The index of the rotor named port; for null or blank the first rotor (0); -1 when there is none. */
    default int port(String port) {
        List<Rotor> rotors = rotors();
        if (port == null || port.isBlank()) { return rotors.isEmpty() ? -1 : 0; }
        for (int k = 0; k < rotors.size(); k++) { if (port.equals(rotors.get(k).name())) { return k; } }
        return -1;
    }
    /**
     * Pairs of this part's rotors it couples itself through the drivetrain solve (a gearbox's input and output by its
     * gears and clutches, 0.24). Shafts must not put both rotors of a link in one train: the part's own coupling would
     * then fight the shaft's.
     */
    default List<Link> links() { return List.of(); }
    /** Input roles bindings may drive ("throttle", "deflection"). */
    default List<String> inputs() { return List.of(); }

    /**
     * A fuel tank (0.26): its name, volume (m^3), centre in the part frame, the fuel a new vehicle has in it ("" none)
     * and how much (kg). What it holds rides with the vehicle as a point mass with its weight (VehicleDynamics); parts
     * draw on it through PartStep.burn. An own tank (0.28: a booster's propellant grain) feeds only its part, and a part
     * with own tanks draws only on them.
     */
    record Tank(String name, double capacityM3, Vec3d centre, String fuel, double initialKg, boolean own) {
        public Tank {
            java.util.Objects.requireNonNull(name, "name"); java.util.Objects.requireNonNull(centre, "centre"); java.util.Objects.requireNonNull(fuel, "fuel");
            if (!(capacityM3 > 0) || !(initialKg >= 0)) { throw new IllegalArgumentException("A tank holds a positive volume and a nonnegative fill"); }
        }
        /** A tank any part of the vehicle draws on. */
        public Tank(String name, double capacityM3, Vec3d centre, String fuel, double initialKg) { this(name, capacityM3, centre, fuel, initialKg, false); }
    }

    /** The part's fuel tanks (0.26; none by default). */
    default List<Tank> tanks() { return List.of(); }
    /** Heave wave-radiation points in the part frame (WaveRadiation point absorbers); radii are reported each step. */
    default List<Vec3d> radiationPoints() { return List.of(); }
    /** Boxes people can stand on, in the part frame. */
    default List<WorldBox> deck() { return List.of(); }
    /** Rider positions in the part frame. */
    default List<Vec3d> seats() { return List.of(); }
    /**
     * Named frames other parts can be mounted on (part frame; Mount.on): a body's "wing", "mast" or "transom". A mount
     * on one gives its offset in the point's frame.
     */
    default java.util.Map<String, Mount> mountPoints() { return java.util.Map.of(); }
    /** Parts whose results others read in the same step (a propeller's thrust for its slipstream) go first. */
    default boolean early() { return false; }

    /**
     * 0.33: the frame this part twists as (a chassis, a car body; FrameTwist), or null: rigid. A vehicle has one at most; it
     * keeps the stations' twists (rad) as its first internal states, which VehicleDynamics solves (FrameFlex) and writes.
     */
    default FrameTwist frame() { return null; }

    /**
     * 0.33: whether this part's loads depend on where a twisting frame puts it (wheels). When the vehicle has a frame,
     * such parts are probed (probe) before the frame's twist is solved and their loads follow it (PartStep.flexLift,
     * flexRate).
     */
    default boolean probes() { return false; }

    /**
     * 0.33: the probe before a twisting frame is solved: report with PartStep.corner each wheel's force on the body and
     * its stiffness within the step, at the twist the step starts with; keep (PartStep.keep) what loads() can reuse (the
     * ground the wheels found). Write no loads here.
     */
    default void probe(PartStep step) {}
    /**
     * Number of internal state values this part keeps per vehicle (an engine's manifold pressure and temperatures);
     * they live in VehicleState, start at initialInternals() and are read and written through PartStep.
     */
    default int internalStates() { return 0; }
    /** Values of the internal states for a new vehicle (zeros unless the part says otherwise). */
    default double[] initialInternals() { return new double[internalStates()]; }
    /** The wake this part makes on water (a hull), or null; the game takes the first part that has one. */
    default WakeSource wake() { return null; }
    /**
     * Number of values this part shows (a wheel's spin angle, steering angle and suspension travel, 0.24): kept per
     * vehicle in VehicleState, written through PartStep.show or PartAfter.show, and sent to clients by the game so the
     * model draws what the physics did. Zero at a new vehicle.
     */
    default int shownValues() { return 0; }

    /** This part's loads on the vehicle at the start of a step, written into step. */
    void loads(PartStep step);

    /**
     * After the step (0.24): the drivetrain's end speeds, what its rows carried (a clutch's torque, a tyre's force) and
     * the vehicle's end state, for internal states that follow them (a tyre's deflection, a brake's temperature).
     * Called for every part once per step, after the contact solve.
     */
    default void after(PartAfter step) {}

    /**
     * Builds what compile() left for the first step because it takes a moment (a hull's resistance tables, a
     * propeller's four-quadrant table), so a caller can do it off the game thread. Safe to call from any thread.
     */
    default void warmUp() {}

    /**
     * A rotating element: spin axis in the part frame (positive rotation by the right-hand rule), its moment of inertia
     * about that axis (kg m^2), whether it drives (a motor's rotor: a shaft train is solved from its driver), and its
     * angular momentum per rad/s of the rotor's speed (kg m^2). The two differ behind a gearbox: an engine whose output
     * shaft is geared down from a fast turbine has that turbine's inertia times the ratio squared at the output (what
     * the train accelerates) but only times the ratio in angular momentum (what turns the airframe gyroscopically and
     * pushes back on it while spinning up).
     */
    record Rotor(String name, Vec3d axis, double inertiaKgM2, boolean driver, double momentumKgM2) {
        public Rotor {
            if (name == null) { name = ""; }
            axis = axis.normalized();
            if (!(inertiaKgM2 > 0) || !Double.isFinite(inertiaKgM2)) { throw new IllegalArgumentException("Rotor inertia must be positive"); }
            if (!Double.isFinite(momentumKgM2)) { throw new IllegalArgumentException("Rotor momentum must be finite"); }
        }
        /** An unnamed rotor (shafts reach it as the first rotor only). */
        public Rotor(Vec3d axis, double inertiaKgM2, boolean driver, double momentumKgM2) { this("", axis, inertiaKgM2, driver, momentumKgM2); }
        /** A rotor whose angular momentum is its inertia times its speed (no gearbox inside the part). */
        public Rotor(Vec3d axis, double inertiaKgM2, boolean driver) { this("", axis, inertiaKgM2, driver, inertiaKgM2); }
        /** A named rotor (a port shafts can name) whose angular momentum is its inertia times its speed. */
        public Rotor(String name, Vec3d axis, double inertiaKgM2, boolean driver) { this(name, axis, inertiaKgM2, driver, inertiaKgM2); }
    }

    /** Two of a part's rotors (indices in rotors()) that the part couples itself. */
    record Link(int a, int b) {}
}
