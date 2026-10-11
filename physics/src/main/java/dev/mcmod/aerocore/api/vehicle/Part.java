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

    /** Rotating elements (a motor's rotor, a propeller) that shafts can join; axis in the part frame. */
    default List<Rotor> rotors() { return List.of(); }
    /** Input roles bindings may drive ("throttle", "deflection"). */
    default List<String> inputs() { return List.of(); }
    /** Heave wave-radiation points in the part frame (WaveRadiation point absorbers); radii are reported each step. */
    default List<Vec3d> radiationPoints() { return List.of(); }
    /** Boxes people can stand on, in the part frame. */
    default List<WorldBox> deck() { return List.of(); }
    /** Rider positions in the part frame. */
    default List<Vec3d> seats() { return List.of(); }
    /** Parts whose results others read in the same step (a propeller's thrust for its slipstream) go first. */
    default boolean early() { return false; }

    /** This part's loads on the vehicle at the start of a step, written into step. */
    void loads(PartStep step);

    /**
     * Builds what compile() left for the first step because it takes a moment (a hull's resistance tables, a
     * propeller's four-quadrant table), so a caller can do it off the game thread. Safe to call from any thread.
     */
    default void warmUp() {}

    /**
     * A rotating element: spin axis in the part frame (positive rotation by the right-hand rule), its moment of inertia
     * about that axis (kg m^2), and whether it drives (a motor's rotor: a shaft train is solved from its driver).
     */
    record Rotor(Vec3d axis, double inertiaKgM2, boolean driver) {
        public Rotor {
            axis = axis.normalized();
            if (!(inertiaKgM2 > 0) || !Double.isFinite(inertiaKgM2)) { throw new IllegalArgumentException("Rotor inertia must be positive"); }
        }
    }
}
