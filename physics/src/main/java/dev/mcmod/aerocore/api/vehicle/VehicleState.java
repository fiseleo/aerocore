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
        for (Vehicle.Installed i : vehicle.installed()) {
            double[] start = i.part().initialInternals();
            if (start.length != i.part().internalStates()) { throw new IllegalStateException("Part " + i.id() + " gives the wrong number of initial internals"); }
            System.arraycopy(start, 0, internals, i.firstInternal(), start.length);
        }
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
        var p = vehicle.properties();
        var q = next.properties();
        Vec3d omega = s.angularVelocity(p);
        Vec3d com = s.worldPoint(p, q.centerOfMassLocal());
        Vec3d velocity = s.velocity().add(omega.cross(com.subtract(s.position())));
        VehicleState out = new VehicleState(next, new RigidBodyState(com, velocity, s.orientation(), q.inertia().applyWorld(s.orientation(), omega)), dt);
        carryParts(out);
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
        out.diagnostics = diagnostics;   // until the next step replaces them
        return out;
    }

    /** Loads of the last step, for arrows (offsets from that step's COM). */
    public List<AppliedForce> forces() { return forces; }
    /** Diagnostics of the last step by part id. */
    public Map<String, Map<String, Double>> diagnostics() { return diagnostics; }
    /** World position of the vehicle frame's origin. */
    public Vec3d origin() { return vehicle.origin(body); }

    /** Rigid-body state, train speeds and part internals (the wave memory is not saved; it refills within its 8 s span). */
    public Map<String, Object> toJson() {
        Orientation q = body.orientation();
        List<Object> speeds = new ArrayList<>(), parts = new ArrayList<>();
        for (double s : trainSpeeds) { speeds.add(s); }
        for (double s : internals) { parts.add(s); }
        List<Object> tanks = new ArrayList<>();
        for (int k = 0; k < tankKg.length; k++) { tanks.add(Fields.object("fuel", tankFuel[k], "kg", tankKg[k])); }
        return Fields.object("position", Fields.vector(body.position()), "velocity", Fields.vector(body.velocity()),
                "orientation", List.of(q.w(), q.x(), q.y(), q.z()), "angularMomentum", Fields.vector(body.angularMomentum()), "trainSpeeds", speeds,
                "internals", parts, "tanks", tanks);
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
        return state;
    }
}
