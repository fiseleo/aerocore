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
                            int firstRotor, int firstRadiation, RigidBodyProperties partProperties, String slipstreamFrom, int firstInternal,
                            int firstShown) {}

    /**
     * A rotor in the vehicle: its part, spin axis and position in the vehicle frame, inertia, whether it drives, its
     * shaft train, its speed as a multiple of the train speed, the gear efficiency on the path from the train's root, and
     * its angular momentum per rad/s of its speed (Part.Rotor).
     */
    public record Rotor(int part, Vec3d axis, Vec3d position, double inertiaKgM2, boolean driver, int train, double ratio, double efficiency,
                        double momentumKgM2) {}

    /**
     * A freewheel shaft between two trains: when engaged the driven train turns at factor times the driving one, the
     * power passing with the shaft's efficiency.
     */
    public record Freewheel(int from, int to, double factor, double efficiency) {}

    /** A fuel tank in the vehicle (0.26): its part (installed index), id ("part.tank"), the part's tank and its centre in the vehicle frame. */
    public record Tank(int part, String id, Part.Tank tank, Vec3d centre) {}

    /** A cargo hold in the vehicle (0.42): its part (installed index), id ("part.hold"), the part's hold and its centre in the vehicle frame. */
    public record Hold(int part, String id, Part.Hold hold, Vec3d centre) {}

    /**
     * The vehicle's twisting frame (0.33): its part (installed index), the stations along the vehicle's x, their twist
     * stiffness (FrameTwist) and their shares of the vehicle's mass (each part's at its centre, split between the stations
     * either side), which the rigid body's roll is the mean of.
     */
    public record Flex(int part, double[] stationX, double[][] twistNmPerRad, double[] weights) {}

    /**
     * 0.43: a joint of the vehicle (Part.joint): its index among the vehicle's joints, its part (installed index) and id,
     * the joint, the part it is attached to (-1 at the root) and where it sits in that part's frame (its own mount at the
     * root): the joint turns or slides the part from there.
     */
    public record Joint(int index, int part, String id, Part.Joint joint, int parent, Mount base) {}

    /** 0.43: a tool of the vehicle (Part.tools): its part (installed index), id ("part.tool") and the tool. */
    public record Tool(int part, String id, Part.Tool tool) {}

    private final VehicleBlueprint blueprint;
    private final List<Installed> installed;
    private final MassProperties mass;
    private final RigidBodyProperties properties;
    private final List<CollisionSphere> collision;
    private final List<Vec3d> radiationPoints, seats;
    private final List<WorldBox> deck;
    private final List<Rotor> rotors;
    private final int trains;
    private final List<Freewheel> freewheels;
    private final List<Tank> tanks;
    private final List<Hold> holds;
    private final List<DesignIssue> issues;
    private final Flex flex;
    /** Where each part's condition values start (Part.conditions, by installed index), and how many there are in all (0.42). */
    private final int[] firstCondition;
    private final int conditions;
    /** 0.43: the part each is attached to (-1 at the root), its placement there, its own joint (-1 none), the joints above it (root first, its own last). */
    private final int[] parent, jointOf;
    private final Mount[] relative;
    private final int[][] chain;
    private final List<Joint> joints;
    private final List<Tool> tools;
    private final VehiclePose rest;

    private Vehicle(VehicleBlueprint blueprint, List<Installed> installed, MassProperties mass, RigidBodyProperties properties,
                    List<CollisionSphere> collision, List<Vec3d> radiationPoints, List<Vec3d> seats, List<WorldBox> deck,
                    List<Rotor> rotors, int trains, List<Freewheel> freewheels, List<Tank> tanks, List<Hold> holds, List<DesignIssue> issues, Flex flex,
                    int[] parent, Mount[] relative, List<MassProperties> masses) {
        this.blueprint = blueprint; this.installed = installed; this.mass = mass; this.properties = properties; this.collision = collision;
        this.radiationPoints = radiationPoints; this.seats = seats; this.deck = deck; this.rotors = rotors; this.trains = trains;
        this.freewheels = freewheels; this.tanks = tanks; this.holds = holds; this.issues = issues; this.flex = flex;
        firstCondition = new int[installed.size()];
        int n = 0;
        for (Installed i : installed) {
            firstCondition[i.index()] = n;
            List<String> names = i.part().conditions();
            if (new java.util.HashSet<>(names).size() != names.size()) { throw new IllegalStateException("Part " + i.id() + " names a condition twice: " + names); }
            n += names.size();
        }
        conditions = n;
        this.parent = parent; this.relative = relative;
        jointOf = new int[installed.size()];
        java.util.Arrays.fill(jointOf, -1);
        List<Joint> js = new ArrayList<>();
        for (Installed i : installed) {
            Part.Joint j = i.part().joint();
            if (j == null) { continue; }
            jointOf[i.index()] = js.size();
            js.add(new Joint(js.size(), i.index(), i.id(), j, parent[i.index()], relative[i.index()]));
        }
        joints = List.copyOf(js);
        List<Tool> ts = new ArrayList<>();
        for (Installed i : installed) { for (Part.Tool t : i.part().tools()) { ts.add(new Tool(i.index(), i.id() + "." + t.name(), t)); } }
        tools = List.copyOf(ts);
        chain = new int[installed.size()][];
        for (Installed i : installed) {
            List<Integer> up = new ArrayList<>();
            for (int at = i.index(), guard = 0; at >= 0 && guard <= installed.size(); at = parent[at], guard++) {
                if (jointOf[at] >= 0) { up.add(0, jointOf[at]); }
            }
            chain[i.index()] = up.stream().mapToInt(Integer::intValue).toArray();
        }
        if (joints.isEmpty()) {
            rest = VehiclePose.fixed(this, masses);
        } else {
            double[] start = new double[joints.size()];
            for (Joint j : joints) { start[j.index()] = j.joint().initial(); }
            rest = VehiclePose.of(this, start);
        }
    }

    /** 0.43: the vehicle's joints (Part.joint), in installed order; none for most vehicles. */
    public List<Joint> joints() { return joints; }
    /** 0.43: the vehicle's tools (Part.tools), in installed order. */
    public List<Tool> tools() { return tools; }
    /** 0.43: the pose of a new vehicle (every joint at its initial value); for a vehicle without joints the compiled geometry itself. */
    public VehiclePose restPose() { return rest; }
    /** 0.43: the pose at these joint values (Vehicle.joints order); the rest pose without joints. */
    public VehiclePose pose(double[] values) {
        if (joints.isEmpty()) { return rest; }
        if (values.length != joints.size()) { throw new IllegalArgumentException("Expected " + joints.size() + " joint values"); }
        return VehiclePose.of(this, values);
    }
    /** The part installed part i is attached to (-1 when it is mounted on the vehicle itself). */
    public int parent(int part) { return parent[part]; }
    /** Where installed part i sits in its parent's frame (its own mount at the root). */
    Mount relative(int part) { return relative[part]; }
    /** Installed part i's own joint (index in joints()), or -1. */
    public int jointOf(int part) { return jointOf[part]; }
    /** The joints that move installed part i, root first (its own last when it has one); empty when it sits fixed. */
    public int[] chain(int part) { return chain[part]; }
    /** Whether joint j moves installed part i. */
    public boolean moves(int joint, int part) { for (int j : chain[part]) { if (j == joint) { return true; } } return false; }

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
    /** Freewheel shafts between trains. */
    public List<Freewheel> freewheels() { return freewheels; }
    /** Fuel tanks of all parts, in installed order (0.26). */
    public List<Tank> tanks() { return tanks; }
    /** Cargo holds of all parts, in installed order (0.42). */
    public List<Hold> holds() { return holds; }
    /** Total condition values of all parts (Part.conditions), in installed order (0.42). */
    public int conditions() { return conditions; }
    /** Where a part's condition values start (by installed index). */
    public int firstCondition(int part) { return firstCondition[part]; }
    /** The twisting frame (0.33), or null: the vehicle is one rigid body. */
    public Flex flex() { return flex; }
    /** EXTRAPOLATION issues of the parts and the assembly (INVALID ones stop compile()). */
    public List<DesignIssue> issues() { return issues; }
    /** Body contact against blocks; per-block materials come from the obstacles (BlockMaterials). */
    public ContactMaterial contactMaterial() { return ContactMaterial.STEEL_ON_ROCK; }

    /** Total internal state values of all parts (Part.internalStates). */
    public int internals() {
        int n = 0;
        for (Installed i : installed) { n += i.part().internalStates(); }
        return n;
    }

    /** Total shown values of all parts (Part.shownValues), in installed order. */
    public int shownValues() {
        int n = 0;
        for (Installed i : installed) { n += i.part().shownValues(); }
        return n;
    }

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
        RigidBodyState body = new RigidBodyState(origin.add(q.rotate(rest.properties().centerOfMassLocal())), Vec3d.ZERO, q, Vec3d.ZERO);
        return new VehicleState(this, body, dt);
    }

    /** World position of the vehicle frame's origin in a state. */
    public Vec3d origin(RigidBodyState s) { return s.worldPoint(properties, Vec3d.ZERO); }

    /**
     * Each part's placement in the vehicle frame: its own mount, or for one attached to another part's mount point that
     * part's placement, then the point, then the offset. Unknown parts and points and attachments in a ring are issues.
     */
    static List<Mount> resolveMounts(List<VehicleBlueprint.Entry> entries, List<Part> parts, Map<String, Integer> index, List<DesignIssue> issues) {
        Mount[] resolved = new Mount[entries.size()];
        for (int i = 0; i < entries.size(); i++) { resolve(i, entries, parts, index, issues, resolved, new java.util.HashSet<>()); }
        List<Mount> out = new ArrayList<>();
        for (Mount m : resolved) { out.add(m == null ? Mount.ORIGIN : m); }
        return out;
    }

    private static Mount resolve(int i, List<VehicleBlueprint.Entry> entries, List<Part> parts, Map<String, Integer> index, List<DesignIssue> issues,
                                 Mount[] resolved, java.util.Set<Integer> visiting) {
        if (resolved[i] != null) { return resolved[i]; }
        VehicleBlueprint.Entry e = entries.get(i);
        Mount m = e.mount();
        if (!m.attached()) { return resolved[i] = m; }
        String path = "parts." + e.id() + ".attach";
        Integer parent = index.get(m.attachPart());
        if (parent == null) { issues.add(DesignIssue.invalid(path, "No part " + m.attachPart() + " to attach to")); return resolved[i] = m.detached(); }
        if (!visiting.add(i)) { issues.add(DesignIssue.invalid(path, "Parts are attached to each other in a ring")); return resolved[i] = m.detached(); }
        Part host = parts.get(parent);
        Mount point = host == null ? null : host.mountPoints().get(m.attachPoint());
        if (point == null) {
            issues.add(DesignIssue.invalid(path, "Part " + m.attachPart() + " has no mount point " + m.attachPoint()
                    + (host == null ? "" : " (has " + host.mountPoints().keySet() + ")")));
            return resolved[i] = m.detached();
        }
        Mount base = resolve(parent, entries, parts, index, issues, resolved, visiting);
        return resolved[i] = base.then(point).then(m.detached());
    }

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
        List<Mount> mounts = resolveMounts(entries, parts, index, issues);
        DesignException.throwIfInvalid(issues);

        // Mass of the whole from every part at its mount.
        List<MassProperties> masses = new ArrayList<>();
        for (int i = 0; i < entries.size(); i++) {
            Mount m = mounts.get(i);
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
        List<Tank> tanks = new ArrayList<>();
        List<Hold> holds = new ArrayList<>();
        List<Rotor> rotors = new ArrayList<>();
        int internals = 0, shown = 0;
        for (int i = 0; i < entries.size(); i++) {
            VehicleBlueprint.Entry e = entries.get(i);
            Part part = parts.get(i);
            Mount m = mounts.get(i);
            Orientation r = m.rotation(), inverse = r.inverse();
            // The whole body seen in the part's axes: COM in the part frame, inertia rotated into the part axes.
            MassProperties seen = MassProperties.of(total.massKg(), Vec3d.ZERO, inertia).transformed(Vec3d.ZERO, inverse);
            RigidBodyProperties partProperties = new RigidBodyProperties(total.massKg(), inverse.rotate(total.centre().subtract(m.position())), seen.inertia());
            installed.add(new Installed(i, e.id(), part, m, r, e.bindings(), rotors.size(), radiation.size(), partProperties, slipstreams.get(e.id()), internals, shown));
            internals += part.internalStates();
            shown += part.shownValues();
            for (CollisionSphere s : part.collision()) { collision.add(new CollisionSphere(m.apply(s.centerLocal()), s.radiusM())); }
            for (Vec3d p : part.radiationPoints()) { radiation.add(m.apply(p)); }
            for (Vec3d p : part.seats()) { seats.add(m.apply(p)); }
            for (WorldBox b : part.deck()) { deck.add(transformed(b, m)); }
            for (Part.Tank t : part.tanks()) { tanks.add(new Tank(i, e.id() + "." + t.name(), t, m.apply(t.centre()))); }
            for (Part.Hold h : part.holds()) { holds.add(new Hold(i, e.id() + "." + h.name(), h, m.apply(h.centre()))); }
            for (Part.Rotor rotor : part.rotors()) {
                rotors.add(new Rotor(i, r.rotate(rotor.axis()), m.position(), rotor.inertiaKgM2(), rotor.driver(), -1, 1, 1, rotor.momentumKgM2()));
            }
        }
        List<Freewheel> freewheels = new ArrayList<>();
        int trains = trains(blueprint, installed, rotors, index, issues, freewheels);
        Flex flex = flex(installed, masses, issues);
        // 0.43: each part's parent and its placement there, for joints (Part.joint) to move what is attached below them
        int[] parent = new int[entries.size()];
        Mount[] relative = new Mount[entries.size()];
        for (int i = 0; i < entries.size(); i++) {
            Mount m = entries.get(i).mount();
            Integer host = m.attached() ? index.get(m.attachPart()) : null;
            Mount point = host == null ? null : parts.get(host).mountPoints().get(m.attachPoint());
            parent[i] = point == null ? -1 : host;
            relative[i] = point == null ? mounts.get(i) : point.then(m.detached());
        }
        if (flex != null && installed.get(flex.part()).part().joint() != null) {
            issues.add(DesignIssue.invalid("parts." + installed.get(flex.part()).id(), "A twisting frame does not move on a joint"));
        }
        DesignException.throwIfInvalid(issues);
        Vehicle v = new Vehicle(blueprint, List.copyOf(installed), total, properties, List.copyOf(collision), List.copyOf(radiation), List.copyOf(seats),
                List.copyOf(deck), List.copyOf(rotors), trains, List.copyOf(freewheels), List.copyOf(tanks), List.copyOf(holds), List.copyOf(issues), flex,
                parent, relative, masses);
        if (flex != null && v.chain(flex.part()).length > 0) {
            throw new DesignException(List.of(DesignIssue.invalid("parts." + installed.get(flex.part()).id(), "A twisting frame does not ride on a joint")));
        }
        return v;
    }

    /** The vehicle's twisting frame (0.33): one part's at most, mounted level and square (no turn), its stations' mass shares. */
    private static Flex flex(List<Installed> installed, List<MassProperties> masses, List<DesignIssue> issues) {
        Installed with = null;
        for (Installed i : installed) {
            if (i.part().frame() == null) { continue; }
            if (with != null) { issues.add(DesignIssue.invalid("parts." + i.id(), "A vehicle twists as one frame: " + with.id() + " has one already")); continue; }
            Mount m = i.mount();
            if (Math.abs(m.yawDeg()) > 1e-9 || Math.abs(m.pitchDeg()) > 1e-9 || Math.abs(m.rollDeg()) > 1e-9) {
                issues.add(DesignIssue.invalid("parts." + i.id() + ".mount", "A twisting frame is mounted square: not turned"));
                continue;
            }
            if (i.part().internalStates() < i.part().frame().stations()) { throw new IllegalStateException("Part " + i.id() + " keeps no twist states"); }
            with = i;
        }
        if (with == null) { return null; }
        FrameTwist f = with.part().frame();
        double[] x = f.stationX().clone();
        for (int k = 0; k < x.length; k++) { x[k] += with.mount().position().x(); }
        double[] w = new double[x.length];
        double sum = 0;
        for (MassProperties m : masses) {
            double[] sh = FrameFlex.shape(x, m.centre().x());
            for (int k = 0; k < w.length; k++) { w[k] += sh[k] * m.massKg(); }
            sum += m.massKg();
        }
        for (int k = 0; k < w.length; k++) { w[k] = sum > 0 ? w[k] / sum : 1.0 / w.length; }
        return new Flex(with.index(), x, f.twistNmPerRad(), w);
    }

    /** A rotor's port name for messages (its index when it has none). */
    private static String port(Installed i, int k) {
        String name = i.part().rotors().get(k).name();
        return name.isEmpty() ? "#" + k : name;
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
     * refused (their efficiencies and ratios are ambiguous). Freewheel shafts separate trains and are listed instead.
     * A shaft joins the rotors its ports name (the first when none); the two rotors of a part's link (Part.links, its
     * own gears or clutches) must stay in different trains.
     */
    private static int trains(VehicleBlueprint blueprint, List<Installed> installed, List<Rotor> rotors, Map<String, Integer> index, List<DesignIssue> issues,
                              List<Freewheel> freewheels) {
        record Link(int from, int to, double ratio, double efficiency) {}
        List<Link> links = new ArrayList<>();
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
            int a = from.part().port(s.fromPort()), b = to.part().port(s.toPort());
            if (a < 0 || b < 0) {
                Installed missing = a < 0 ? from : to;
                List<String> names = new ArrayList<>();
                for (Part.Rotor r : missing.part().rotors()) { if (!r.name().isEmpty()) { names.add(r.name()); } }
                issues.add(DesignIssue.invalid("connections[" + c + "]." + (a < 0 ? "fromPort" : "toPort"),
                        "Part " + missing.id() + " has no rotor " + (a < 0 ? s.fromPort() : s.toPort()) + " (has " + names + ")"));
                continue;
            }
            if (s.freewheel()) { links.add(new Link(from.firstRotor() + a, to.firstRotor() + b, s.ratio(), s.efficiency())); continue; }
            Edge edge = new Edge(from.firstRotor() + a, to.firstRotor() + b, s.ratio(), s.efficiency());
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
            rotors.set(k, new Rotor(r.part(), r.axis(), r.position(), r.inertiaKgM2(), r.driver(), train[k], ratio[k], efficiency[k], r.momentumKgM2()));
        }
        for (Installed i : installed) {
            for (Part.Link l : i.part().links()) {
                int n = i.part().rotors().size();
                if (l.a() < 0 || l.a() >= n || l.b() < 0 || l.b() >= n || l.a() == l.b()) {
                    throw new IllegalStateException("Part " + i.id() + " links rotors " + l.a() + " and " + l.b() + " of " + n);
                }
                if (train[i.firstRotor() + l.a()] == train[i.firstRotor() + l.b()]) {
                    issues.add(DesignIssue.invalid("connections", "Shafts join " + i.id() + "'s rotors " + port(i, l.a()) + " and " + port(i, l.b())
                            + ", which the part couples itself"));
                }
            }
        }
        // a freewheel from rotor a to rotor b: b's train speed = (ratio_a / (shaft ratio x ratio_b)) x a's train speed
        for (Link l : links) {
            if (train[l.from()] == train[l.to()]) {
                issues.add(DesignIssue.invalid("connections", "A freewheel joins rotors already geared together; loops are not supported"));
                continue;
            }
            freewheels.add(new Freewheel(train[l.from()], train[l.to()], ratio[l.from()] / (l.ratio() * ratio[l.to()]), l.efficiency()));
        }
        return trains;
    }
}
