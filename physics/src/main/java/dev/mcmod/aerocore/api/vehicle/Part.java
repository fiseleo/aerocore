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

    /**
     * A cargo hold (0.42): its name, volume (m^3) and centre in the part frame. What the game puts in it
     * (VehicleState.setHold, kg) rides with the vehicle as a point mass with its weight, as a tank's fuel does; what the
     * cargo is (items, their masses) is the game's business. A new vehicle's holds are empty.
     */
    record Hold(String name, double capacityM3, Vec3d centre) {
        public Hold {
            java.util.Objects.requireNonNull(name, "name"); java.util.Objects.requireNonNull(centre, "centre");
            if (!(capacityM3 > 0) || !Double.isFinite(capacityM3)) { throw new IllegalArgumentException("A hold has a positive volume"); }
        }
    }

    /** The part's cargo holds (0.42; none by default). */
    default List<Hold> holds() { return List.of(); }

    /**
     * 0.42: the names of the things about this part that wear out or get damaged (an axle's "tyres", "brakes" and
     * "suspension"; a wing's "structure"). For each the vehicle keeps how much of it is used up, 0 (new) to 1 (failed), in
     * VehicleState: the part reads it (PartStep.used) to lose performance and adds its own wear (PartStep.wear,
     * PartAfter.wear); the game adds damage from outside (VehicleState.setUsed: a crash, corrosion) and takes it away
     * (a repair). Kept by part id and name, so a blueprint may change around it. None by default: the part never wears.
     * A part must behave to the last bit as before 0.42 while everything is 0.
     */
    default List<String> conditions() { return List.of(); }
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
     * 0.43: how this part moves against what it is mounted on (an excavator's boom about its foot pin, a slewing ring's
     * upper half, a loader's front frame about the centre hinge, a telescopic section, a dump body's tip), or null: it
     * sits fixed. The part, everything mounted on its mount points and everything below them move together; the vehicle
     * stays one rigid body whose mass, inertia and geometry follow the joints (VehiclePose). The value q is radians
     * (REVOLUTE: a turn about axis through origin) or metres (PRISMATIC: a slide along axis), 0 as the part is built
     * and mounted, kept within minimum..maximum, initial for a new vehicle. The part drives it in loads
     * (PartStep.driveJoint); undriven it is held.
     */
    record Joint(Kind kind, Vec3d origin, Vec3d axis, double minimum, double maximum, double initial) {
        public enum Kind { REVOLUTE, PRISMATIC }
        public Joint {
            java.util.Objects.requireNonNull(kind, "kind"); java.util.Objects.requireNonNull(origin, "origin");
            if (!(axis.length() > 0)) { throw new IllegalArgumentException("A joint has an axis"); }
            axis = axis.normalized();
            if (Double.isNaN(minimum) || Double.isNaN(maximum) || minimum > maximum || !Double.isFinite(initial) || initial < minimum || initial > maximum) {
                throw new IllegalArgumentException("A joint's range is ordered and holds its initial value: " + minimum + " .. " + maximum + ", " + initial);
            }
        }
        /** A turning joint about axis through origin (part frame), radians. */
        public static Joint revolute(Vec3d origin, Vec3d axis, double minimumRad, double maximumRad, double initialRad) {
            return new Joint(Kind.REVOLUTE, origin, axis, minimumRad, maximumRad, initialRad);
        }
        /** A sliding joint along axis (part frame), metres. */
        public static Joint prismatic(Vec3d axis, double minimumM, double maximumM, double initialM) {
            return new Joint(Kind.PRISMATIC, Vec3d.ZERO, axis, minimumM, maximumM, initialM);
        }
    }

    /** 0.43: this part's joint (Joint), or null: it sits fixed where it is mounted. */
    default Joint joint() { return null; }

    /**
     * 0.43: a tool's cutting edge (part frame): a straight edge from a to b (a bucket's lip, a blade's cutting edge, a
     * ripper's tip as a short one, a cutter drum's width along its bottom), cutting into the ground the way cut points
     * (unit, part frame: the edge's motion when it digs), at most depthM deep at once (what the tool holds above its
     * edge: a blade's height, a bucket's depth), at a rake (rad from the horizontal in the cut's direction: a dozer
     * blade's 55 degrees, a bucket's lip 30 to 45). The part computes the cutting force in its loads from the ground it
     * finds (VehicleEnvironment.terrain, api.earthmoving.Digging) and reports what it cut (PartStep.cut) into its hold
     * ("" for none: a blade pushes it on, a ripper leaves it loosened). Kinds: EDGE, a wide edge moving through the
     * ground (bucket, blade, moldboard, scraper's edge); TOOTH, a narrow one (ripper shank, a trencher's tooth); MILL, a
     * drum turning on its own rotor (a cold planer's, a roadheader's head, a bucket wheel) whose picks cut at the rim's
     * speed, the edge its axis; FACE, a tunnel boring machine's cutterhead (a disc of radius |b - a| about a, facing the
     * cut) with disc cutters (face).
     */
    record Tool(String name, Vec3d a, Vec3d b, Vec3d cut, double depthM, double rakeRad, Kind kind, String hold, Face face) {
        public enum Kind { EDGE, TOOTH, MILL, FACE }
        public Tool {
            java.util.Objects.requireNonNull(name); java.util.Objects.requireNonNull(a); java.util.Objects.requireNonNull(b);
            java.util.Objects.requireNonNull(kind); java.util.Objects.requireNonNull(hold);
            if (!(cut.length() > 0) || !(depthM > 0) || !Double.isFinite(depthM) || !(rakeRad > 0 && rakeRad < Math.PI)) {
                throw new IllegalArgumentException("A tool cuts some way, some depth, at a rake");
            }
            if (kind == Kind.FACE && face == null) { throw new IllegalArgumentException("A cutterhead has its discs"); }
            cut = cut.normalized();
        }
        /** An edge or tooth (no cutterhead). */
        public Tool(String name, Vec3d a, Vec3d b, Vec3d cut, double depthM, double rakeRad, Kind kind, String hold) {
            this(name, a, b, cut, depthM, rakeRad, kind, hold, null);
        }
        /** The edge's width, m (a cutterhead's radius). */
        public double widthM() { return b.subtract(a).length(); }
    }

    /** 0.43: a cutterhead's disc cutters: how many, their diameter and tip width (m), and their spacing on the face (m). */
    record Face(int discs, double discDiameterM, double tipWidthM, double spacingM) {
        public Face {
            if (discs < 1 || !(discDiameterM > 0) || !(tipWidthM > 0) || !(spacingM > 0)) { throw new IllegalArgumentException("Disc cutter values out of range"); }
        }
    }

    /** 0.43: this part's cutting tools (none by default). */
    default List<Tool> tools() { return List.of(); }

    /**
     * 0.33: the frame this part twists as (a chassis, a car body; FrameTwist), or null: rigid. A vehicle has one at most; it
     * keeps the stations' twists (rad) as its first internal states, which VehicleDynamics solves (FrameFlex) and writes.
     */
    default FrameTwist frame() { return null; }

    /**
     * 0.42: the share (0..1) of its frame's twist stiffness this part has left this step: a damaged chassis twists more
     * easily (FramePart). Asked before the twist is solved, and only of the part the vehicle twists as, when it has
     * conditions; read the condition with PartStep.used and write nothing. 1: as designed, the solve as before 0.42.
     */
    default double frameStiffness(PartStep step) { return 1; }

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
