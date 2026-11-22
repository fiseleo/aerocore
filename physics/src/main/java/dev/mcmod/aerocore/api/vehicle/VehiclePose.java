package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;

import java.util.ArrayList;
import java.util.List;

/**
 * 0.43: a vehicle's geometry at one set of joint values (Part.joint): where every part sits in the vehicle frame, the
 * mass, centre and inertia of the whole, and everything the step reads from the parts' geometry (contact spheres,
 * seats, decks, tank and hold centres, rotor axes, radiation points), plus each joint's origin and axis. A vehicle
 * without joints has one pose, its rest pose, built from the same objects Vehicle has always held, so its steps are
 * unchanged to the last bit. Immutable.
 */
public final class VehiclePose {
    private final Vehicle vehicle;
    private final double[] values;
    private final Vec3d[] position;
    private final Orientation[] rotation;
    private final MassProperties[] partMass;
    private final MassProperties total;
    private final RigidBodyProperties properties;
    private final List<CollisionSphere> collision;
    private final int[] collisionOwner;
    private final List<Vec3d> radiationPoints, seats;
    private final List<WorldBox> deck;
    private final Vec3d[] tankCentre, holdCentre, rotorAxis, rotorPosition, jointOrigin, jointAxis;

    private VehiclePose(Vehicle vehicle, double[] values, Vec3d[] position, Orientation[] rotation, MassProperties[] partMass, MassProperties total,
                        RigidBodyProperties properties, List<CollisionSphere> collision, int[] collisionOwner, List<Vec3d> radiationPoints, List<Vec3d> seats,
                        List<WorldBox> deck, Vec3d[] tankCentre, Vec3d[] holdCentre, Vec3d[] rotorAxis, Vec3d[] rotorPosition, Vec3d[] jointOrigin, Vec3d[] jointAxis) {
        this.vehicle = vehicle; this.values = values; this.position = position; this.rotation = rotation; this.partMass = partMass; this.total = total;
        this.properties = properties; this.collision = collision; this.collisionOwner = collisionOwner; this.radiationPoints = radiationPoints;
        this.seats = seats; this.deck = deck; this.tankCentre = tankCentre; this.holdCentre = holdCentre; this.rotorAxis = rotorAxis;
        this.rotorPosition = rotorPosition; this.jointOrigin = jointOrigin; this.jointAxis = jointAxis;
    }

    /** The joint values (Vehicle.joints order) this pose is for (a copy). */
    public double[] values() { return values.clone(); }
    double value(int joint) { return values[joint]; }
    /** Where installed part i's frame origin sits (vehicle frame). */
    public Vec3d position(int part) { return position[part]; }
    /** Installed part i's frame turned into the vehicle frame. */
    public Orientation rotation(int part) { return rotation[part]; }
    /** A part-frame point of installed part i in the vehicle frame. */
    public Vec3d apply(int part, Vec3d partPoint) { return position[part].add(rotation[part].rotate(partPoint)); }
    /** Installed part i's mass, centre and inertia in the vehicle frame. */
    public MassProperties partMass(int part) { return partMass[part]; }
    /** The whole vehicle's mass properties in the vehicle frame. */
    public MassProperties mass() { return total; }
    /** Mass, centre and inertia of the whole as a rigid body at these joint values. */
    public RigidBodyProperties properties() { return properties; }
    /** Contact spheres (vehicle frame). */
    public List<CollisionSphere> collision() { return collision; }
    /** The installed part each contact sphere belongs to. */
    public int collisionOwner(int sphere) { return collisionOwner[sphere]; }
    public List<Vec3d> radiationPoints() { return radiationPoints; }
    public List<Vec3d> seats() { return seats; }
    public List<WorldBox> deck() { return deck; }
    /** Centre of tank k (Vehicle.tanks) in the vehicle frame. */
    public Vec3d tankCentre(int tank) { return tankCentre[tank]; }
    /** Centre of hold k (Vehicle.holds) in the vehicle frame. */
    public Vec3d holdCentre(int hold) { return holdCentre[hold]; }
    /** Spin axis of rotor k (Vehicle.rotors) in the vehicle frame. */
    public Vec3d rotorAxis(int rotor) { return rotorAxis[rotor]; }
    /** Position of rotor k's part in the vehicle frame. */
    public Vec3d rotorPosition(int rotor) { return rotorPosition[rotor]; }
    /** A point on joint j's axis (vehicle frame). */
    public Vec3d jointOrigin(int joint) { return jointOrigin[joint]; }
    /** Joint j's axis (vehicle frame, unit). */
    public Vec3d jointAxis(int joint) { return jointAxis[joint]; }

    /** The rest pose of a vehicle without joints: the objects Vehicle compiled. */
    static VehiclePose fixed(Vehicle v, List<MassProperties> masses) {
        int n = v.installed().size();
        Vec3d[] position = new Vec3d[n];
        Orientation[] rotation = new Orientation[n];
        for (Vehicle.Installed i : v.installed()) { position[i.index()] = i.mount().position(); rotation[i.index()] = i.rotation(); }
        int[] owner = new int[v.collision().size()];
        int at = 0;
        for (Vehicle.Installed i : v.installed()) { for (int k = 0; k < i.part().collision().size(); k++) { owner[at++] = i.index(); } }
        Vec3d[] tanks = new Vec3d[v.tanks().size()], holds = new Vec3d[v.holds().size()], axes = new Vec3d[v.rotors().size()], at2 = new Vec3d[axes.length];
        for (int k = 0; k < tanks.length; k++) { tanks[k] = v.tanks().get(k).centre(); }
        for (int k = 0; k < holds.length; k++) { holds[k] = v.holds().get(k).centre(); }
        for (int k = 0; k < axes.length; k++) { axes[k] = v.rotors().get(k).axis(); at2[k] = v.rotors().get(k).position(); }
        return new VehiclePose(v, new double[0], position, rotation, masses.toArray(new MassProperties[0]), v.mass(), v.properties(), v.collision(), owner,
                v.radiationPoints(), v.seats(), v.deck(), tanks, holds, axes, at2, new Vec3d[0], new Vec3d[0]);
    }

    /**
     * The pose of a vehicle with joints at the given values (Vehicle.joints order): each part placed through its parent
     * (the part it is attached to: its mount point, then the offset), then its own joint. Throws IllegalArgumentException
     * when the mass lies on a line (the vehicle could not turn), as compile does.
     */
    static VehiclePose of(Vehicle v, double[] values) {
        List<Vehicle.Installed> installed = v.installed();
        int n = installed.size();
        Vec3d[] position = new Vec3d[n];
        Orientation[] rotation = new Orientation[n];
        for (int i = 0; i < n; i++) { place(v, values, i, position, rotation); }
        MassProperties[] masses = new MassProperties[n];
        List<MassProperties> all = new ArrayList<>(n);
        for (int i = 0; i < n; i++) { masses[i] = installed.get(i).part().mass().transformed(position[i], rotation[i]); all.add(masses[i]); }
        MassProperties total = MassProperties.combine(all);
        InertiaTensor inertia = total.inertia();
        RigidBodyProperties properties = new RigidBodyProperties(total.massKg(), total.centre(), inertia);
        List<CollisionSphere> collision = new ArrayList<>();
        List<Integer> owners = new ArrayList<>();
        List<Vec3d> radiation = new ArrayList<>(), seats = new ArrayList<>();
        List<WorldBox> deck = new ArrayList<>();
        Vec3d[] tanks = new Vec3d[v.tanks().size()], holds = new Vec3d[v.holds().size()];
        Vec3d[] axes = new Vec3d[v.rotors().size()], at = new Vec3d[axes.length];
        int tank = 0, hold = 0, rotor = 0;
        for (int i = 0; i < n; i++) {
            Part part = installed.get(i).part();
            for (CollisionSphere s : part.collision()) { collision.add(new CollisionSphere(apply(position, rotation, i, s.centerLocal()), s.radiusM())); owners.add(i); }
            for (Vec3d p : part.radiationPoints()) { radiation.add(apply(position, rotation, i, p)); }
            for (Vec3d p : part.seats()) { seats.add(apply(position, rotation, i, p)); }
            for (WorldBox b : part.deck()) { deck.add(box(b, position[i], rotation[i])); }
            for (Part.Tank t : part.tanks()) { tanks[tank++] = apply(position, rotation, i, t.centre()); }
            for (Part.Hold h : part.holds()) { holds[hold++] = apply(position, rotation, i, h.centre()); }
            for (Part.Rotor r : part.rotors()) { axes[rotor] = rotation[i].rotate(r.axis()); at[rotor++] = position[i]; }
        }
        int[] owner = new int[owners.size()];
        for (int k = 0; k < owner.length; k++) { owner[k] = owners.get(k); }
        Vec3d[] origin = new Vec3d[v.joints().size()], axis = new Vec3d[origin.length];
        for (Vehicle.Joint j : v.joints()) {
            // the joint's own axis turns with the parts above it, not with itself: place it in the parent's frame
            Part.Joint pj = j.joint();
            Vec3d o = pj.kind() == Part.Joint.Kind.REVOLUTE ? pj.origin() : Vec3d.ZERO;
            Mount base = j.base();
            Vec3d inParentPos; Orientation inParentRot;
            if (j.parent() < 0) { inParentPos = base.position(); inParentRot = base.rotation(); }
            else { inParentPos = apply(position, rotation, j.parent(), base.position()); inParentRot = rotation[j.parent()].multiply(base.rotation()); }
            origin[j.index()] = inParentPos.add(inParentRot.rotate(o));
            axis[j.index()] = inParentRot.rotate(pj.axis());
        }
        return new VehiclePose(v, values.clone(), position, rotation, masses, total, properties, List.copyOf(collision), owner, List.copyOf(radiation),
                List.copyOf(seats), List.copyOf(deck), tanks, holds, axes, at, origin, axis);
    }

    private static Vec3d apply(Vec3d[] position, Orientation[] rotation, int i, Vec3d p) { return position[i].add(rotation[i].rotate(p)); }

    /** Places part i (and first its parent) at the joint values. */
    private static void place(Vehicle v, double[] values, int i, Vec3d[] position, Orientation[] rotation) {
        if (position[i] != null) { return; }
        Vehicle.Installed installed = v.installed().get(i);
        int parent = v.parent(i);
        Mount m = v.relative(i);
        Vec3d p; Orientation r;
        if (parent < 0) { p = m.position(); r = m.rotation(); }
        else {
            place(v, values, parent, position, rotation);
            p = position[parent].add(rotation[parent].rotate(m.position()));
            r = rotation[parent].multiply(m.rotation());
        }
        int j = v.jointOf(i);
        if (j >= 0) {
            Part.Joint joint = installed.part().joint();
            double q = values[j];
            if (joint.kind() == Part.Joint.Kind.REVOLUTE) {
                Orientation turn = Orientation.rotationVector(joint.axis().scale(q));
                Vec3d o = joint.origin();
                p = p.add(r.rotate(o.subtract(turn.rotate(o))));
                r = r.multiply(turn);
            } else {
                p = p.add(r.rotate(joint.axis().scale(q)));
            }
        }
        position[i] = p; rotation[i] = r;
    }

    /** The axis-aligned box (vehicle frame) around a part-frame box placed at position, turned by rotation. */
    private static WorldBox box(WorldBox b, Vec3d position, Orientation rotation) {
        double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (int c = 0; c < 8; c++) {
            Vec3d corner = new Vec3d((c & 1) == 0 ? b.min().x() : b.max().x(), (c & 2) == 0 ? b.min().y() : b.max().y(), (c & 4) == 0 ? b.min().z() : b.max().z());
            Vec3d w = position.add(rotation.rotate(corner));
            double[] xyz = {w.x(), w.y(), w.z()};
            for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], xyz[k]); hi[k] = Math.max(hi[k], xyz[k]); }
        }
        return new WorldBox(new Vec3d(lo[0], lo[1], lo[2]), new Vec3d(hi[0], hi[1], hi[2]), b.material());
    }

    /**
     * Velocity (vehicle frame) a vehicle-frame point fixed on installed part i has against the vehicle frame when the
     * joints move at rates: the sum over the joints above the part (its own included) of rate x (axis x (point -
     * origin)) for a turning one, rate x axis for a sliding one.
     */
    public Vec3d relativeVelocity(int part, Vec3d vehiclePoint, double[] rates) {
        Vec3d v = Vec3d.ZERO;
        for (int j : vehicle.chain(part)) {
            double rate = rates[j];
            if (rate == 0) { continue; }
            Part.Joint joint = vehicle.joints().get(j).joint();
            v = v.add(joint.kind() == Part.Joint.Kind.REVOLUTE ? jointAxis[j].cross(vehiclePoint.subtract(jointOrigin[j])).scale(rate) : jointAxis[j].scale(rate));
        }
        return v;
    }

    /** Angular velocity (vehicle frame) of installed part i against the vehicle frame when the joints move at rates. */
    public Vec3d relativeSpin(int part, double[] rates) {
        Vec3d w = Vec3d.ZERO;
        for (int j : vehicle.chain(part)) {
            if (rates[j] != 0 && vehicle.joints().get(j).joint().kind() == Part.Joint.Kind.REVOLUTE) { w = w.add(jointAxis[j].scale(rates[j])); }
        }
        return w;
    }
}
