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
    /** Internal states of the parts (Part.internalStates), in installed order. */
    final double[] internals;
    /** Values the parts show (Part.shownValues), in installed order. */
    final double[] shown;
    /** Whether the radiation memory has taken its first radii (VehicleDynamics.finish primes it at the first step). */
    boolean primed;
    /** Fuel in each tank (Vehicle.tanks), kg, and which fuel ("" empty) (0.26). */
    final double[] tankKg;
    final String[] tankFuel;
    /** Each part's share of the fuel it asked for in the last step (1 when it asked none, or the vehicle has no tanks). */
    final double[] supply;
    /** Parts that asked to be cast off (PartStep.jettison, 0.28), until the game takes them. */
    final java.util.Set<String> jettisons = new java.util.LinkedHashSet<>();
    /** How much of each part's conditions is used up (Part.conditions; 0 new .. 1 failed), in installed order (0.42). */
    final double[] used;
    /** Cargo in each hold (Vehicle.holds), kg (0.42). */
    final double[] holdKg;
    /** 0.43: each joint's value (Vehicle.joints; rad or m) and rate, and the pose they make (built when asked, kept while they stand). */
    final double[] jointValues, jointRates;
    /**
     * 0.43: what the parts outside a joint did to the parts it moves in the last step (contacts on their spheres, their
     * tyres' dampers), as a generalized force on the joint (N m or N): the next step's drive takes it with this step's
     * loads. And the generalized force each joint's actuator gave (the drive's effort; for pressures and gauges).
     */
    final double[] jointLagged, jointEffort;
    /** 0.43: the whole generalized load on each joint the last step's drive took (weight, loads, lagged contacts; N m or N). */
    final double[] jointLoad;
    /** 0.43: the moving parts' momentum against the vehicle frame in the last step: linear (world, kg m/s) and angular (vehicle frame, about the COM). */
    Vec3d relativeLinear = Vec3d.ZERO, relativeAngular = Vec3d.ZERO;
    private VehiclePose pose;
    private double[] poseValues;
    /**
     * 0.43: each hydraulic circuit's last step (by name, VehicleDynamics): the share of the flow its actuators asked for
     * that its pumps could give (1 when they could give it all), its pressure (Pa), the flow its pumps could give and
     * the flow asked (m^3/s). Not saved: a circuit is back within a step.
     */
    final Map<String, double[]> circuits = new java.util.LinkedHashMap<>();
    /**
     * 0.43: what the tools cut since the game last took it (PartStep.cut): by part, tool, cell, volume (m^3 of bank
     * material) and the hold it went into. Not saved: the game takes it every tick (takeCuts).
     */
    final List<Cut> cuts = new ArrayList<>();

    /** 0.43: a tool's cut: the part's id, the tool's name, the cell, the bank volume (m^3) and the hold's id it went into ("" none). */
    public record Cut(String part, String tool, int x, int y, int z, double m3, String hold) {}

    /** 0.43: what the tools cut since the last call (each cell's volume summed per tool), and clears it. */
    public List<Cut> takeCuts() {
        if (cuts.isEmpty()) { return List.of(); }
        java.util.LinkedHashMap<String, Cut> sum = new java.util.LinkedHashMap<>();
        for (Cut c : cuts) {
            String key = c.part() + "|" + c.tool() + "|" + c.x() + "|" + c.y() + "|" + c.z() + "|" + c.hold();
            sum.merge(key, c, (a, b) -> new Cut(a.part(), a.tool(), a.x(), a.y(), a.z(), a.m3() + b.m3(), a.hold()));
        }
        cuts.clear();
        return List.copyOf(sum.values());
    }
    List<AppliedForce> forces = List.of();
    Map<String, Map<String, Double>> diagnostics = Map.of();

    VehicleState(Vehicle vehicle, RigidBodyState body, double dt) {
        if (!(dt > 0) || !Double.isFinite(dt)) { throw new IllegalArgumentException("Substep must be positive"); }
        this.vehicle = vehicle; this.body = body; this.dt = dt;
        trainSpeeds = new double[vehicle.trains()];
        radiation = vehicle.radiationPoints().isEmpty() ? null : new WaveRadiation.Memory(vehicle.radiationPoints(), dt);
        radii = new double[vehicle.radiationPoints().size()];
        internals = new double[vehicle.internals()];
        shown = new double[vehicle.shownValues()];
        tankKg = new double[vehicle.tanks().size()];
        tankFuel = new String[tankKg.length];
        for (int k = 0; k < tankKg.length; k++) {
            Part.Tank t = vehicle.tanks().get(k).tank();
            tankKg[k] = t.fuel().isEmpty() ? 0 : t.initialKg();
            tankFuel[k] = tankKg[k] > 0 ? t.fuel() : "";
        }
        supply = new double[vehicle.installed().size()];
        java.util.Arrays.fill(supply, 1);
        used = new double[vehicle.conditions()];
        holdKg = new double[vehicle.holds().size()];
        for (Vehicle.Installed i : vehicle.installed()) {
            double[] start = i.part().initialInternals();
            if (start.length != i.part().internalStates()) { throw new IllegalStateException("Part " + i.id() + " gives the wrong number of initial internals"); }
            System.arraycopy(start, 0, internals, i.firstInternal(), start.length);
        }
        int joints = vehicle.joints().size();
        jointValues = new double[joints]; jointRates = new double[joints]; jointLagged = new double[joints]; jointEffort = new double[joints];
        jointLoad = new double[joints];
        for (Vehicle.Joint j : vehicle.joints()) { jointValues[j.index()] = j.joint().initial(); }
    }

    public Vehicle vehicle() { return vehicle; }

    // ----- joints (0.43) -----

    /** The vehicle's geometry at its joints' values now (VehiclePose); the compiled geometry for a vehicle without joints. */
    public VehiclePose pose() {
        if (jointValues.length == 0) { return vehicle.restPose(); }
        if (pose == null || !java.util.Arrays.equals(poseValues, jointValues)) {
            pose = vehicle.pose(jointValues);
            poseValues = jointValues.clone();
        }
        return pose;
    }
    /** Mass, centre and inertia of the vehicle as it stands (Vehicle.properties for a vehicle without joints). */
    public dev.mcmod.aerocore.api.RigidBodyProperties properties() { return pose().properties(); }
    /** Joint j's value (rad or m; Vehicle.joints). */
    public double jointValue(int joint) { return jointValues[joint]; }
    /** Joint j's rate in the last step (rad/s or m/s). */
    public double jointRate(int joint) { return jointRates[joint]; }
    /** The generalized force joint j's actuator gave in the last step (N m or N; positive pushes q up). */
    public double jointEffort(int joint) { return jointEffort[joint]; }
    /** The generalized load on joint j from outside what it moves in the last step, weight included (N m or N). */
    public double jointLoad(int joint) { return jointLoad[joint]; }
    /** The joint of part partId, or -1 when it has none. */
    public int joint(String partId) {
        Vehicle.Installed p = vehicle.installed(partId);
        return p == null ? -1 : vehicle.jointOf(p.index());
    }
    /** All joint values (a copy), for sending to clients. */
    public double[] jointValues() { return jointValues.clone(); }

    /** 0.43: a hydraulic circuit's share of the asked flow its pumps gave in the last step (1 for a circuit not seen). */
    public double hydraulicShare(String circuit) { double[] c = circuits.get(circuit); return c == null ? 1 : c[0]; }
    /** 0.43: a hydraulic circuit's pressure in the last step, Pa (0 for a circuit not seen). */
    public double hydraulicPressure(String circuit) { double[] c = circuits.get(circuit); return c == null ? 0 : c[1]; }
    /** 0.43: the flow a circuit's pumps could give and the flow its actuators asked in the last step, m^3/s ({supply, demand}). */
    public double[] hydraulicFlows(String circuit) { double[] c = circuits.get(circuit); return c == null ? new double[2] : new double[]{c[2], c[3]}; }
    /** 0.43: the names of the hydraulic circuits seen in the last step. */
    public java.util.Set<String> hydraulicCircuits() { return java.util.Set.copyOf(circuits.keySet()); }

    /**
     * Sets joints from outside a step (a client's copy of the server's, a command, a teleport), within their ranges,
     * at rest; the vehicle frame stays where it is and the body's centre of mass follows the parts. Values that do not
     * fit or are not finite are skipped.
     */
    public void setJointValues(double[] values) {
        if (jointValues.length == 0) { return; }
        Vec3d origin = origin();
        for (int k = 0; k < Math.min(values.length, jointValues.length); k++) {
            if (!Double.isFinite(values[k])) { continue; }
            Part.Joint j = vehicle.joints().get(k).joint();
            jointValues[k] = Math.clamp(values[k], j.minimum(), j.maximum());
            jointRates[k] = 0;
        }
        relativeLinear = Vec3d.ZERO; relativeAngular = Vec3d.ZERO;
        reference(origin);
    }

    /** Puts the body's centre of mass where the pose puts it with the vehicle frame's origin at origin (world), same motion. */
    private void reference(Vec3d origin) {
        Vec3d com = origin.add(body.orientation().rotate(pose().properties().centerOfMassLocal()));
        body = new RigidBodyState(com, body.velocity(), body.orientation(), body.angularMomentum());
    }

    /** Copies another state's joints by part id (a changed blueprint keeps what is still there), keeping this one's frame. */
    private void carryJoints(VehicleState from) {
        if (jointValues.length == 0 || from.jointValues.length == 0) { return; }
        double[] values = jointValues.clone();
        for (Vehicle.Joint j : vehicle.joints()) {
            int was = from.joint(j.id());
            if (was >= 0) { values[j.index()] = from.jointValues[was]; }
        }
        setJointValues(values);
    }
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
    /** A part's i-th internal state (Part.internalStates), for display and tests. */
    public double internal(String partId, int i) {
        for (Vehicle.Installed p : vehicle.installed()) {
            if (p.id().equals(partId)) {
                if (i < 0 || i >= p.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
                return internals[p.firstInternal() + i];
            }
        }
        throw new IllegalArgumentException("No part " + partId);
    }

    /**
     * Sets a part's i-th internal state from outside its step (0.28): what the game does to a part between steps, such
     * as new reeds in a worn pulsejet's valve. The part reads it at its next step; values that are not finite are refused.
     */
    public void setInternal(String partId, int i, double value) {
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Internal state must be finite"); }
        Vehicle.Installed p = vehicle.installed(partId);
        if (p == null) { throw new IllegalArgumentException("No part " + partId); }
        if (i < 0 || i >= p.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        internals[p.firstInternal() + i] = value;
    }

    /** A part's i-th shown value (Part.shownValues): a wheel's spin angle, steering, suspension travel. */
    public double shown(String partId, int i) {
        Vehicle.Installed p = vehicle.installed(partId);
        if (p == null) { throw new IllegalArgumentException("No part " + partId); }
        if (i < 0 || i >= p.part().shownValues()) { throw new IndexOutOfBoundsException("Shown value " + i); }
        return shown[p.firstShown() + i];
    }
    /** All shown values in installed order (a copy), for sending to clients. */
    public double[] shownValues() { return shown.clone(); }
    /** Replaces the shown values (a client's copy of the server's); values that do not fit or are not finite are skipped. */
    public void setShownValues(double[] values) {
        for (int i = 0; i < Math.min(values.length, shown.length); i++) { if (Double.isFinite(values[i])) { shown[i] = values[i]; } }
    }

    /** Fuel in a tank (index in Vehicle.tanks), kg (0.26). */
    public double tankKg(int tank) { return tankKg[tank]; }
    /** The fuel in a tank ("" when empty). */
    public String tankFuel(int tank) { return tankFuel[tank]; }
    /** All the fuel aboard, kg. */
    public double fuelKg() { double kg = 0; for (double t : tankKg) { kg += t; } return kg; }
    /** A part's share of the fuel it asked for in the last step (by installed index). */
    public double fuelSupply(int part) { return supply[part]; }
    /**
     * Sets a tank's contents (refuelling, draining): a fuel ("" empties it) and its mass, at most the tank's volume of
     * that fuel at density (kg/m^3); returns the mass set.
     */
    public double setTank(int tank, String fuel, double kg, double densityKgPerM3) {
        if (!(kg >= 0) || !Double.isFinite(kg) || !(densityKgPerM3 > 0)) { throw new IllegalArgumentException("Fuel mass nonnegative, density positive"); }
        double max = vehicle.tanks().get(tank).tank().capacityM3() * densityKgPerM3;
        tankKg[tank] = fuel.isEmpty() ? 0 : Math.min(kg, max);
        tankFuel[tank] = tankKg[tank] > 0 ? fuel : "";
        return tankKg[tank];
    }

    // ----- condition and cargo (0.42) -----

    private int condition(String partId, int i) {
        Vehicle.Installed p = vehicle.installed(partId);
        if (p == null) { throw new IllegalArgumentException("No part " + partId); }
        if (i < 0 || i >= p.part().conditions().size()) { throw new IndexOutOfBoundsException("Condition " + i + " of part " + partId); }
        return vehicle.firstCondition(p.index()) + i;
    }

    /** How much of a part's i-th condition (Part.conditions) is used up: 0 new .. 1 failed. */
    public double used(String partId, int i) { return used[condition(partId, i)]; }

    /** The same by name; 0 for a part or a name the vehicle does not have (nothing there to wear). */
    public double used(String partId, String name) {
        Vehicle.Installed p = vehicle.installed(partId);
        int i = p == null ? -1 : p.part().conditions().indexOf(name);
        return i < 0 ? 0 : used[vehicle.firstCondition(p.index()) + i];
    }

    /**
     * Sets how much of a part's i-th condition is used up, from outside its step: damage (a crash, corrosion) or a
     * repair. Kept within 0..1; a value that is not a number is refused.
     */
    public void setUsed(String partId, int i, double value) {
        if (Double.isNaN(value)) { throw new IllegalArgumentException("Condition must be a number"); }
        used[condition(partId, i)] = Math.clamp(value, 0.0, 1.0);
    }

    /** Adds to a part's i-th condition (damage; negative mends) and returns what it then is, within 0..1. */
    public double addUsed(String partId, int i, double amount) {
        if (Double.isNaN(amount)) { throw new IllegalArgumentException("Condition must be a number"); }
        int at = condition(partId, i);
        return used[at] = Math.clamp(used[at] + amount, 0.0, 1.0);
    }

    /** All condition values in installed order (a copy), for sending to clients. */
    public double[] usedValues() { return used.clone(); }
    /** Replaces the condition values (a client's copy of the server's); values that do not fit or are not finite are skipped. */
    public void setUsedValues(double[] values) {
        for (int i = 0; i < Math.min(values.length, used.length); i++) { if (Double.isFinite(values[i])) { used[i] = Math.clamp(values[i], 0.0, 1.0); } }
    }
    /** The worst of all conditions (0 for a vehicle with none): whether anything needs looking at. */
    public double worstUsed() { double w = 0; for (double u : used) { w = Math.max(w, u); } return w; }

    /** Cargo in a hold (index in Vehicle.holds), kg. */
    public double holdKg(int hold) { return holdKg[hold]; }
    /** All the cargo aboard, kg. */
    public double cargoKg() { double kg = 0; for (double h : holdKg) { kg += h; } return kg; }
    /**
     * Sets the mass of what a hold carries (kg; the game adds up its cargo). The mass is not held to the hold's volume:
     * what fits is the game's rule, and an overloaded vehicle is the physics' to show.
     */
    public void setHold(int hold, double kg) {
        if (!(kg >= 0) || !Double.isFinite(kg)) { throw new IllegalArgumentException("Cargo mass must be finite and nonnegative"); }
        holdKg[hold] = kg;
    }

    /**
     * Takes another state's conditions and cargo, by part id and name and by hold id: a state built anew (after a
     * physics fault) keeps what its hardware has worn and what it carries.
     */
    public void keepHardware(VehicleState from) { from.carryCondition(this); carryJoints(from); }

    /**
     * Condition by part id and name and cargo by hold id, carried into out (separate, refitted, placed, restyled): they
     * belong to the hardware, whatever else about the part changed. A part or hold that is new starts new and empty.
     */
    private void carryCondition(VehicleState out) {
        Vehicle next = out.vehicle;
        if (used.length > 0 && out.used.length > 0) {
            java.util.Map<String, Vehicle.Installed> old = new java.util.HashMap<>();
            for (Vehicle.Installed i : vehicle.installed()) { old.put(i.id(), i); }
            for (Vehicle.Installed i : next.installed()) {
                List<String> names = i.part().conditions();
                Vehicle.Installed was = names.isEmpty() ? null : old.get(i.id());
                if (was == null) { continue; }
                List<String> before = was.part().conditions();
                for (int k = 0; k < names.size(); k++) {
                    int j = before.indexOf(names.get(k));
                    if (j >= 0) { out.used[next.firstCondition(i.index()) + k] = used[vehicle.firstCondition(was.index()) + j]; }
                }
            }
        }
        if (holdKg.length > 0 && out.holdKg.length > 0) {
            java.util.Map<String, Integer> old = new java.util.HashMap<>();
            for (int k = 0; k < vehicle.holds().size(); k++) { old.put(vehicle.holds().get(k).id(), k); }
            for (int k = 0; k < next.holds().size(); k++) {
                Integer was = old.get(next.holds().get(k).id());
                if (was != null) { out.holdKg[k] = holdKg[was]; }
            }
        }
    }

    /** The parts that have asked to be cast off since the last call (PartStep.jettison, 0.28), by id; clears them. */
    public List<String> takeJettisons() {
        List<String> out = List.copyOf(jettisons);
        jettisons.clear();
        return out;
    }

    /**
     * The state of what is left after parts are cast off (0.28): next is this vehicle's blueprint without them, compiled
     * (VehicleBlueprint.without; its parts keep their mounts, so the vehicle frame is the same). The rest keeps its place
     * and motion: its centre of mass is where that point lies in the old body, moving with it (v + omega x r), turning at
     * the same angular velocity with the new inertia (L = I omega). Shaft trains take their rotors' speeds, parts their
     * internal states and shown values, tanks their contents, all by part id; the wave-radiation memory starts afresh
     * (it refills within its 8 s).
     */
    public VehicleState separate(Vehicle next) {
        RigidBodyState s = body;
        var p = properties();
        var q = next.properties();
        Vec3d omega = s.angularVelocity(p);
        Vec3d com = s.worldPoint(p, q.centerOfMassLocal());
        Vec3d velocity = s.velocity().add(omega.cross(com.subtract(s.position())));
        VehicleState out = new VehicleState(next, new RigidBodyState(com, velocity, s.orientation(), q.inertia().applyWorld(s.orientation(), omega)), dt);
        carryParts(out);
        carryCondition(out);
        out.carryJoints(this);
        java.util.Map<String, Integer> tanks = tankIndex();
        for (int k = 0; k < next.tanks().size(); k++) {
            Integer was = tanks.get(next.tanks().get(k).id());
            if (was != null) { out.tankKg[k] = tankKg[was]; out.tankFuel[k] = tankFuel[was]; }
        }
        out.diagnostics = diagnostics;   // until the next step replaces them
        return out;
    }

    /** Internal states, shown values and shaft speeds carried into out by part id (separate, refitted). */
    private void carryParts(VehicleState out) {
        Vehicle next = out.vehicle;
        // rotors by part id and index within the part
        java.util.Map<String, Vehicle.Installed> old = new java.util.HashMap<>();
        for (Vehicle.Installed i : vehicle.installed()) { old.put(i.id(), i); }
        boolean[] set = new boolean[next.trains()];
        for (Vehicle.Installed i : next.installed()) {
            Vehicle.Installed was = old.get(i.id());
            if (was == null || was.part().internalStates() != i.part().internalStates() || was.part().shownValues() != i.part().shownValues()) { continue; }
            System.arraycopy(internals, was.firstInternal(), out.internals, i.firstInternal(), i.part().internalStates());
            System.arraycopy(shown, was.firstShown(), out.shown, i.firstShown(), i.part().shownValues());
            for (int k = 0; k < Math.min(i.part().rotors().size(), was.part().rotors().size()); k++) {
                Vehicle.Rotor r = next.rotors().get(i.firstRotor() + k);
                if (set[r.train()] || r.ratio() == 0) { continue; }
                out.trainSpeeds[r.train()] = rotorSpeed(was.firstRotor() + k) / r.ratio();
                set[r.train()] = true;
            }
        }
    }

    private java.util.Map<String, Integer> tankIndex() {
        java.util.Map<String, Integer> tanks = new java.util.HashMap<>();
        for (int k = 0; k < vehicle.tanks().size(); k++) { tanks.put(vehicle.tanks().get(k).id(), k); }
        return tanks;
    }

    /** A refitted state and the fuel that did not fit the new tanks, kg by tank id (VehicleState.refitted). */
    public record Refitted(VehicleState state, Map<String, Double> spilledKg) {
        public Refitted { spilledKg = Map.copyOf(spilledKg); }
        /** All the fuel spilled, kg. */
        public double spilledTotalKg() { double kg = 0; for (double v : spilledKg.values()) { kg += v; } return kg; }
    }

    /**
     * 0.34: this vehicle refitted in place (a modification shop's work): next is its blueprint with parts reshaped,
     * resized or retuned, compiled, so anything about the physics may differ (mass, parts' internals, shafts, tanks).
     * The vehicle is set down at rest where it stands, its frame's origin and orientation kept (as placed does), and
     * keeps by part id what still fits: internal states and shown values of parts whose counts did not change (the
     * others start afresh, as a new part would), shaft trains their rotors' speeds, fuel supply, and pending cast-offs
     * of parts still there. Tanks keep their contents by tank id up to the new capacity at the fuel's density
     * (densityKgPerM3 of the fuel's name); what does not fit is reported as spilled, and a tank that is new is empty
     * (a refit makes no fuel). The wave-radiation memory starts afresh.
     */
    public Refitted refitted(Vehicle next, java.util.function.ToDoubleFunction<String> densityKgPerM3) {
        VehicleState out = next.newState(origin(), body.orientation(), dt);
        carryParts(out);
        carryCondition(out);
        out.carryJoints(this);
        java.util.Map<String, Integer> tanks = tankIndex();
        Map<String, Double> spilled = new java.util.LinkedHashMap<>();
        for (int k = 0; k < next.tanks().size(); k++) {
            Vehicle.Tank t = next.tanks().get(k);
            Integer was = tanks.get(t.id());
            out.tankKg[k] = 0;
            out.tankFuel[k] = "";
            if (was == null || !(tankKg[was] > 0)) { continue; }
            double density = densityKgPerM3.applyAsDouble(tankFuel[was]);
            double max = t.tank().capacityM3() * (density > 0 && Double.isFinite(density) ? density : 1000);
            out.tankKg[k] = Math.min(tankKg[was], max);
            out.tankFuel[k] = out.tankKg[k] > 0 ? tankFuel[was] : "";
            if (tankKg[was] > out.tankKg[k] + 1e-12) { spilled.put(t.id(), tankKg[was] - out.tankKg[k]); }
        }
        for (int k = 0; k < vehicle.tanks().size(); k++) {   // tanks that went spill all they held
            String id = vehicle.tanks().get(k).id();
            if (tankKg[k] > 0 && next.tanks().stream().noneMatch(t -> t.id().equals(id))) { spilled.put(id, tankKg[k]); }
        }
        java.util.Map<String, Integer> parts = new java.util.HashMap<>();
        for (Vehicle.Installed i : vehicle.installed()) { parts.put(i.id(), i.index()); }
        for (Vehicle.Installed i : next.installed()) {
            Integer was = parts.get(i.id());
            if (was != null) { out.supply[i.index()] = supply[was]; }
        }
        for (String id : jettisons) { if (next.installed(id) != null) { out.jettisons.add(id); } }
        return new Refitted(out, spilled);
    }

    /** refitted with AeroCore's fuel densities (Fuel.byName's liquid density, water's for a fuel it does not know). */
    public Refitted refitted(Vehicle next) {
        return refitted(next, fuel -> {
            dev.mcmod.aerocore.api.engine.Fuel f = dev.mcmod.aerocore.api.engine.Fuel.byName(fuel);
            return f == null ? 1000 : f.liquidDensityKgPerM3();
        });
    }

    /**
     * 0.31: the state of this vehicle with a new look (a repainted part, stickers): next is its blueprint with only
     * appearance changed, compiled. Nothing about the motion changes, so everything is carried over as it is: the body,
     * shaft speeds, internal states, shown values, tanks and fuel supply, the wave-radiation memory, the last loads and
     * pending cast-offs (the vehicle steps on exactly as it would have). Throws IllegalArgumentException when next is
     * not the same vehicle to the physics: other mass properties, parts (ids, order, their state and shown counts,
     * rotors), shaft trains, tanks or radiation points.
     */
    public VehicleState restyled(Vehicle next) {
        if (!next.properties().equals(vehicle.properties())) { throw new IllegalArgumentException("Restyling cannot change the mass properties"); }
        List<Vehicle.Installed> a = vehicle.installed(), b = next.installed();
        boolean same = a.size() == b.size() && next.trains() == vehicle.trains() && next.internals() == vehicle.internals()
                && next.shownValues() == vehicle.shownValues() && next.rotors().size() == vehicle.rotors().size()
                && next.tanks().size() == vehicle.tanks().size() && next.radiationPoints().equals(vehicle.radiationPoints());
        for (int i = 0; same && i < a.size(); i++) {
            Vehicle.Installed x = a.get(i), y = b.get(i);
            same = x.id().equals(y.id()) && x.firstInternal() == y.firstInternal() && x.firstShown() == y.firstShown() && x.firstRotor() == y.firstRotor()
                    && x.part().internalStates() == y.part().internalStates() && x.part().shownValues() == y.part().shownValues()
                    && x.part().rotors().size() == y.part().rotors().size();
        }
        for (int k = 0; same && k < next.tanks().size(); k++) { same = next.tanks().get(k).id().equals(vehicle.tanks().get(k).id()); }
        if (!same) { throw new IllegalArgumentException("Restyling cannot change the parts, shafts, tanks or floats"); }
        VehicleState out = new VehicleState(next, body, dt);
        System.arraycopy(trainSpeeds, 0, out.trainSpeeds, 0, trainSpeeds.length);
        System.arraycopy(internals, 0, out.internals, 0, internals.length);
        System.arraycopy(shown, 0, out.shown, 0, shown.length);
        System.arraycopy(tankKg, 0, out.tankKg, 0, tankKg.length);
        System.arraycopy(tankFuel, 0, out.tankFuel, 0, tankFuel.length);
        System.arraycopy(supply, 0, out.supply, 0, supply.length);
        System.arraycopy(radii, 0, out.radii, 0, radii.length);
        if (radiation != null) { out.radiation.copyFrom(radiation); }
        out.primed = primed;
        out.jettisons.addAll(jettisons);
        carryCondition(out);
        // 0.43: the joints as they stand (the body is the same, so the pose must be too)
        System.arraycopy(jointValues, 0, out.jointValues, 0, jointValues.length);
        System.arraycopy(jointRates, 0, out.jointRates, 0, jointRates.length);
        System.arraycopy(jointLagged, 0, out.jointLagged, 0, jointLagged.length);
        System.arraycopy(jointEffort, 0, out.jointEffort, 0, jointEffort.length);
        System.arraycopy(jointLoad, 0, out.jointLoad, 0, jointLoad.length);
        out.relativeLinear = relativeLinear; out.relativeAngular = relativeAngular;
        out.forces = forces;
        out.diagnostics = diagnostics;
        return out;
    }

    /**
     * 0.32: this vehicle set down somewhere else at rest, as Vehicle.newState places it (the frame's origin at origin,
     * turned by q), keeping what is aboard: shaft speeds (an idling engine runs on), internal states, shown values, tanks
     * and fuel supply, and pending cast-offs. The wave-radiation memory starts afresh, as after separate (the water there
     * is another).
     */
    public VehicleState placed(Vec3d origin, Orientation q) {
        VehicleState out = vehicle.newState(origin, q, dt);
        System.arraycopy(trainSpeeds, 0, out.trainSpeeds, 0, trainSpeeds.length);
        System.arraycopy(internals, 0, out.internals, 0, internals.length);
        System.arraycopy(shown, 0, out.shown, 0, shown.length);
        System.arraycopy(tankKg, 0, out.tankKg, 0, tankKg.length);
        System.arraycopy(tankFuel, 0, out.tankFuel, 0, tankFuel.length);
        System.arraycopy(supply, 0, out.supply, 0, supply.length);
        out.jettisons.addAll(jettisons);
        System.arraycopy(used, 0, out.used, 0, used.length);
        System.arraycopy(holdKg, 0, out.holdKg, 0, holdKg.length);
        out.carryJoints(this);
        out.diagnostics = diagnostics;   // until the next step replaces them
        return out;
    }

    /** Loads of the last step, for arrows (offsets from that step's COM). */
    public List<AppliedForce> forces() { return forces; }
    /** Diagnostics of the last step by part id. */
    public Map<String, Map<String, Double>> diagnostics() { return diagnostics; }
    /** World position of the vehicle frame's origin. */
    public Vec3d origin() { return body.worldPoint(properties(), Vec3d.ZERO); }

    /** Rigid-body state, train speeds and part internals (the wave memory is not saved; it refills within its 8 s span). */
    public Map<String, Object> toJson() {
        Orientation q = body.orientation();
        List<Object> speeds = new ArrayList<>(), parts = new ArrayList<>();
        for (double s : trainSpeeds) { speeds.add(s); }
        for (double s : internals) { parts.add(s); }
        List<Object> tanks = new ArrayList<>();
        for (int k = 0; k < tankKg.length; k++) { tanks.add(Fields.object("fuel", tankFuel[k], "kg", tankKg[k])); }
        Map<String, Object> out = Fields.object("position", Fields.vector(body.position()), "velocity", Fields.vector(body.velocity()),
                "orientation", List.of(q.w(), q.x(), q.y(), q.z()), "angularMomentum", Fields.vector(body.angularMomentum()), "trainSpeeds", speeds,
                "internals", parts, "tanks", tanks);
        // 0.42: what is worn (by part id and condition name) and what the holds carry (by hold id); nothing when new and empty
        Map<String, Object> condition = new java.util.LinkedHashMap<>();
        for (Vehicle.Installed i : vehicle.installed()) {
            List<String> names = i.part().conditions();
            Map<String, Object> of = new java.util.LinkedHashMap<>();
            for (int k = 0; k < names.size(); k++) {
                double u = used[vehicle.firstCondition(i.index()) + k];
                if (u > 0) { of.put(names.get(k), u); }
            }
            if (!of.isEmpty()) { condition.put(i.id(), of); }
        }
        if (!condition.isEmpty()) { out.put("condition", condition); }
        Map<String, Object> holds = new java.util.LinkedHashMap<>();
        for (int k = 0; k < holdKg.length; k++) { if (holdKg[k] > 0) { holds.put(vehicle.holds().get(k).id(), holdKg[k]); } }
        if (!holds.isEmpty()) { out.put("holds", holds); }
        // 0.43: each joint's value and rate by its part's id; only a vehicle with joints has them
        if (jointValues.length > 0) {
            Map<String, Object> joints = new java.util.LinkedHashMap<>();
            for (Vehicle.Joint j : vehicle.joints()) { joints.put(j.id(), List.of(jointValues[j.index()], jointRates[j.index()])); }
            out.put("joints", joints);
        }
        return out;
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
        // tanks saved before 0.26 (none listed) keep the design's fill
        if (f.has("tanks")) {
            List<Fields> tanks = f.objects("tanks");
            if (tanks.size() == state.tankKg.length) {
                for (int k = 0; k < tanks.size(); k++) {
                    double kg = tanks.get(k).number("kg", 0);
                    state.tankKg[k] = Double.isFinite(kg) && kg > 0 ? kg : 0;
                    state.tankFuel[k] = state.tankKg[k] > 0 ? tanks.get(k).string("fuel", "") : "";
                }
            }
        }
        List<Object> parts = f.list("internals", List.of());
        if (parts.size() == state.internals.length) {   // a changed blueprint starts its parts afresh
            for (int i = 0; i < parts.size(); i++) {
                if (parts.get(i) instanceof Number n && Double.isFinite(n.doubleValue())) { state.internals[i] = n.doubleValue(); }
            }
        }
        // 0.42: by part id and name, so a changed blueprint keeps what still is there; saves from before have neither
        if (f.raw("condition") instanceof Map<?, ?> condition) {
            for (Vehicle.Installed i : vehicle.installed()) {
                if (!(condition.get(i.id()) instanceof Map<?, ?> of)) { continue; }
                List<String> names = i.part().conditions();
                for (int k = 0; k < names.size(); k++) {
                    if (of.get(names.get(k)) instanceof Number n && Double.isFinite(n.doubleValue())) {
                        state.used[vehicle.firstCondition(i.index()) + k] = Math.clamp(n.doubleValue(), 0.0, 1.0);
                    }
                }
            }
        }
        if (f.raw("holds") instanceof Map<?, ?> holds) {
            for (int k = 0; k < state.holdKg.length; k++) {
                if (holds.get(vehicle.holds().get(k).id()) instanceof Number n && Double.isFinite(n.doubleValue()) && n.doubleValue() > 0) {
                    state.holdKg[k] = n.doubleValue();
                }
            }
        }
        // 0.43: joints by part id; the saved position is the centre of mass with them as saved, so no re-referencing
        if (state.jointValues.length > 0 && f.raw("joints") instanceof Map<?, ?> joints) {
            for (Vehicle.Joint j : vehicle.joints()) {
                if (!(joints.get(j.id()) instanceof List<?> pair) || pair.size() != 2) { continue; }
                if (pair.get(0) instanceof Number value && Double.isFinite(value.doubleValue())) {
                    state.jointValues[j.index()] = Math.clamp(value.doubleValue(), j.joint().minimum(), j.joint().maximum());
                }
                if (pair.get(1) instanceof Number r && Double.isFinite(r.doubleValue())) { state.jointRates[j.index()] = r.doubleValue(); }
            }
            JointDrive.Relative rel = JointDrive.relative(vehicle, state.pose(), state, state.jointRates);
            state.relativeLinear = orientation.rotate(rel.linear());
            state.relativeAngular = rel.angular();
        }
        return state;
    }
}
