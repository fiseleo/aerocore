package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A compiled vehicle: its parts at their mounts, the rigid body they make (mass, COM and inertia in the vehicle frame,
 * from every part by the parallel-axis theorem), contact spheres, deck boxes, seats, wave-radiation points and shaft
 * trains, all in the vehicle frame (x forward, y up, z starboard; its origin is wherever the blueprint put it).
 * Immutable; one Vehicle serves any number of VehicleStates.
 */
public final class Vehicle {
    /** One installed part with what the step needs precomputed. */
    public record Installed(int index, String id, Part part, Mount mount, Orientation rotation, List<Binding> bindings,
                            int firstRotor, int firstRadiation, RigidBodyProperties partProperties, String slipstreamFrom) {}

    /**
     * A rotor in the vehicle: its part, spin axis and position in the vehicle frame, inertia, whether it drives, its
     * shaft train, its speed as a multiple of the train speed, and the gear efficiency on the path from the train's root.
     */
    public record Rotor(int part, Vec3d axis, Vec3d position, double inertiaKgM2, boolean driver, int train, double ratio, double efficiency) {}

    private final VehicleBlueprint blueprint;
    private final List<Installed> installed;
    private final MassProperties mass;
    private final RigidBodyProperties properties;
    private final List<CollisionSphere> collision;
    private final List<Vec3d> radiationPoints, seats;
    private final List<WorldBox> deck;
    private final List<Rotor> rotors;
    private final int trains;
    private final List<DesignIssue> issues;

    private Vehicle(VehicleBlueprint blueprint, List<Installed> installed, MassProperties mass, RigidBodyProperties properties,
                    List<CollisionSphere> collision, List<Vec3d> radiationPoints, List<Vec3d> seats, List<WorldBox> deck,
                    List<Rotor> rotors, int trains, List<DesignIssue> issues) {
        this.blueprint = blueprint; this.installed = installed; this.mass = mass; this.properties = properties; this.collision = collision;
        this.radiationPoints = radiationPoints; this.seats = seats; this.deck = deck; this.rotors = rotors; this.trains = trains; this.issues = issues;
    }

    public VehicleBlueprint blueprint() { return blueprint; }
    public List<Installed> installed() { return installed; }
    public MassProperties mass() { return mass; }
    public RigidBodyProperties properties() { return properties; }
    public List<CollisionSphere> collision() { return collision; }
    public List<Vec3d> radiationPoints() { return radiationPoints; }
    public List<Vec3d> seats() { return seats; }
    public List<WorldBox> deck() { return deck; }
    public List<Rotor> rotors() { return rotors; }
    public int trains() { return trains; }
    /** EXTRAPOLATION issues of the parts and the assembly (INVALID ones stop compile()). */
    public List<DesignIssue> issues() { return issues; }
    /** Body contact against blocks; per-block materials come from the obstacles (BlockMaterials). */
    public ContactMaterial contactMaterial() { return ContactMaterial.STEEL_ON_ROCK; }

    /** Part.warmUp of every part (about a second for a new hull design): call it off the game thread before stepping. */
    public void warmUp() { for (Installed i : installed) { i.part().warmUp(); } }

    public Installed installed(String id) {
        for (Installed i : installed) { if (i.id().equals(id)) { return i; } }
        return null;
    }

    /**
     * A new runtime state with the vehicle frame's origin at origin (world) turned by q, at rest, stepped with a fixed
     * substep dt (s; the wave-radiation memory is built for it).
     */
    public VehicleState newState(Vec3d origin, Orientation q, double dt) {
        RigidBodyState body = new RigidBodyState(origin.add(q.rotate(properties.centerOfMassLocal())), Vec3d.ZERO, q, Vec3d.ZERO);
        return new VehicleState(this, body, dt);
    }

    /** World position of the vehicle frame's origin in a state. */
    public Vec3d origin(RigidBodyState s) { return s.worldPoint(properties, Vec3d.ZERO); }

    static Vehicle compile(VehicleBlueprint blueprint) {
        List<DesignIssue> issues = new ArrayList<>(blueprint.check());
        DesignException.throwIfInvalid(issues);
        List<VehicleBlueprint.Entry> entries = blueprint.parts();
        List<Part> parts = new ArrayList<>();
        for (VehicleBlueprint.Entry e : entries) {
            try {
                Part part = PartTypes.compile(e.design());
                parts.add(part);
                for (DesignIssue i : part.issues()) { issues.add(i.under("parts." + e.id())); }
                for (Binding b : e.bindings()) {
                    if (!part.inputs().contains(b.role())) {
                        issues.add(DesignIssue.invalid("parts." + e.id() + ".bindings." + b.role(), "This part has no input " + b.role() + " (has " + part.inputs() + ")"));
                    }
                }
            } catch (DesignException failed) {
                for (DesignIssue i : failed.issues()) { issues.add(i.under("parts." + e.id())); }
                parts.add(null);
            }
        }
        DesignException.throwIfInvalid(issues);
        Map<String, Integer> index = new HashMap<>();
        for (int i = 0; i < entries.size(); i++) { index.put(entries.get(i).id(), i); }

        // Mass of the whole from every part at its mount.
        List<MassProperties> masses = new ArrayList<>();
        for (int i = 0; i < entries.size(); i++) {
            Mount m = entries.get(i).mount();
            masses.add(parts.get(i).mass().transformed(m.position(), m.rotation()));
        }
        MassProperties total = MassProperties.combine(masses);
        InertiaTensor inertia = null;
        if (!(total.massKg() > 0)) {
            issues.add(DesignIssue.invalid("parts", "The vehicle has no mass"));
        } else {
            try { inertia = total.inertia(); } catch (IllegalArgumentException degenerate) {
                issues.add(DesignIssue.invalid("parts", "All the mass lies on one line or point; the vehicle cannot rotate stably"));
            }
        }
        DesignException.throwIfInvalid(issues);
        RigidBodyProperties properties = new RigidBodyProperties(total.massKg(), total.centre(), inertia);

        // Slipstream sources by surface.
        Map<String, String> slipstreams = new HashMap<>();
        for (Connection c : blueprint.connections()) {
            if (c instanceof Connection.Slipstream s) {
                if (slipstreams.put(s.surface(), s.propeller()) != null) {
                    issues.add(DesignIssue.invalid("connections", "Part " + s.surface() + " is in more than one slipstream"));
                }
            }
        }

        List<Installed> installed = new ArrayList<>();
        List<CollisionSphere> collision = new ArrayList<>();
        List<Vec3d> radiation = new ArrayList<>(), seats = new ArrayList<>();
        List<WorldBox> deck = new ArrayList<>();
        List<Rotor> rotors = new ArrayList<>();
        for (int i = 0; i < entries.size(); i++) {
            VehicleBlueprint.Entry e = entries.get(i);
            Part part = parts.get(i);
            Mount m = e.mount();
            Orientation r = m.rotation(), inverse = r.inverse();
            // The whole body seen in the part's axes: COM in the part frame, inertia rotated into the part axes.
            MassProperties seen = MassProperties.of(total.massKg(), Vec3d.ZERO, inertia).transformed(Vec3d.ZERO, inverse);
            RigidBodyProperties partProperties = new RigidBodyProperties(total.massKg(), inverse.rotate(total.centre().subtract(m.position())), seen.inertia());
            installed.add(new Installed(i, e.id(), part, m, r, e.bindings(), rotors.size(), radiation.size(), partProperties, slipstreams.get(e.id())));
            for (CollisionSphere s : part.collision()) { collision.add(new CollisionSphere(m.apply(s.centerLocal()), s.radiusM())); }
            for (Vec3d p : part.radiationPoints()) { radiation.add(m.apply(p)); }
            for (Vec3d p : part.seats()) { seats.add(m.apply(p)); }
            for (WorldBox b : part.deck()) { deck.add(transformed(b, m)); }
            for (Part.Rotor rotor : part.rotors()) {
                rotors.add(new Rotor(i, r.rotate(rotor.axis()), m.position(), rotor.inertiaKgM2(), rotor.driver(), -1, 1, 1));
            }
        }
        int trains = trains(blueprint, installed, rotors, index, issues);
        DesignException.throwIfInvalid(issues);
        return new Vehicle(blueprint, List.copyOf(installed), total, properties, List.copyOf(collision), List.copyOf(radiation), List.copyOf(seats),
                List.copyOf(deck), List.copyOf(rotors), trains, List.copyOf(issues));
    }

    /** The axis-aligned box (vehicle frame) around a part-frame box at a mount. */
    private static WorldBox transformed(WorldBox b, Mount m) {
        double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (int c = 0; c < 8; c++) {
            Vec3d corner = new Vec3d((c & 1) == 0 ? b.min().x() : b.max().x(), (c & 2) == 0 ? b.min().y() : b.max().y(), (c & 4) == 0 ? b.min().z() : b.max().z());
            Vec3d v = m.apply(corner);
            double[] xyz = {v.x(), v.y(), v.z()};
            for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], xyz[k]); hi[k] = Math.max(hi[k], xyz[k]); }
        }
        return new WorldBox(new Vec3d(lo[0], lo[1], lo[2]), new Vec3d(hi[0], hi[1], hi[2]), b.material());
    }

    /**
     * Groups the rotors into shaft trains (each a single degree of freedom): breadth-first from a driver, the speed of
     * the far end of a shaft is the near end's divided by its ratio, efficiencies multiply along the path. Loops are
     * refused (their efficiencies and ratios are ambiguous).
     */
    private static int trains(VehicleBlueprint blueprint, List<Installed> installed, List<Rotor> rotors, Map<String, Integer> index, List<DesignIssue> issues) {
        record Edge(int a, int b, double ratio, double efficiency) {}
        List<List<Edge>> adjacency = new ArrayList<>();
        for (int k = 0; k < rotors.size(); k++) { adjacency.add(new ArrayList<>()); }
        for (int c = 0; c < blueprint.connections().size(); c++) {
            if (!(blueprint.connections().get(c) instanceof Connection.Shaft s)) { continue; }
            Installed from = installed.get(index.get(s.from())), to = installed.get(index.get(s.to()));
            if (from.part().rotors().isEmpty() || to.part().rotors().isEmpty()) {
                issues.add(DesignIssue.invalid("connections[" + c + "]", "A shaft joins parts that turn (" + (from.part().rotors().isEmpty() ? s.from() : s.to()) + " has no rotor)"));
                continue;
            }
            Edge edge = new Edge(from.firstRotor(), to.firstRotor(), s.ratio(), s.efficiency());
            adjacency.get(edge.a()).add(edge);
            adjacency.get(edge.b()).add(edge);
        }
        double[] ratio = new double[rotors.size()], efficiency = new double[rotors.size()];
        int[] train = new int[rotors.size()];
        java.util.Arrays.fill(train, -1);
        int trains = 0;
        List<Integer> order = new ArrayList<>();
        for (int k = 0; k < rotors.size(); k++) { if (rotors.get(k).driver()) { order.add(k); } }
        for (int k = 0; k < rotors.size(); k++) { if (!rotors.get(k).driver()) { order.add(k); } }
        java.util.Set<Edge> used = new java.util.HashSet<>();
        for (int root : order) {
            if (train[root] >= 0) { continue; }
            int t = trains++;
            train[root] = t; ratio[root] = 1; efficiency[root] = 1;
            ArrayDeque<Integer> queue = new ArrayDeque<>(List.of(root));
            while (!queue.isEmpty()) {
                int k = queue.poll();
                for (Edge e : adjacency.get(k)) {
                    if (!used.add(e)) { continue; }
                    int other = e.a() == k ? e.b() : e.a();
                    double speed = e.a() == k ? ratio[k] / e.ratio() : ratio[k] * e.ratio();
                    if (train[other] >= 0) {
                        issues.add(DesignIssue.invalid("connections", "Shafts form a loop through " + installed.get(rotors.get(other).part()).id()
                                + "; loops are not supported"));
                        continue;
                    }
                    train[other] = t; ratio[other] = speed; efficiency[other] = efficiency[k] * e.efficiency();
                    queue.add(other);
                }
            }
        }
        for (int k = 0; k < rotors.size(); k++) {
            Rotor r = rotors.get(k);
            rotors.set(k, new Rotor(r.part(), r.axis(), r.position(), r.inertiaKgM2(), r.driver(), train[k], ratio[k], efficiency[k]));
        }
        return trains;
    }
}
