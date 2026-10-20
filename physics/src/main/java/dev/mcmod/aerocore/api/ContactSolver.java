package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * One rigid body against static boxes: velocity-level sequential impulses with an isotropic Coulomb
 * cone, speculative contacts and a separate position projection (Catto, "Iterative Dynamics with
 * Temporal Coherence", GDC 2005; Box2D). The projection moves the pose only, so it adds no kinetic energy.
 * Contact geometry is sphere-based; sample spacing limits how well flat faces and edges are resolved.
 */
public final class ContactSolver {
    /** Penetration (m) tolerated before position projection acts; prevents resting jitter. */
    public static final double SLOP = 0.005;
    /** Approach speed (m/s) below which restitution is ignored, so resting contacts stay at rest. */
    public static final double RESTITUTION_THRESHOLD = 0.5;
    private static final double SPECULATIVE_DISTANCE = 0.02, PROJECTION = 0.2, MAX_PROJECTION = 0.2;
    private static final int VELOCITY_ITERATIONS = 24, POSITION_ITERATIONS = 3;
    /**
     * 0.36, joints: a point gap left alone (m), the share of the rest closed per step through velocity and the most it
     * may ask for (m/s); separateAll's share and most per iteration (m, and rad for a held roll); a stop's push-back share.
     */
    private static final double JOINT_SLOP = 0.0002, JOINT_BAUMGARTE = 0.2, MAX_JOINT_BIAS = 0.5,
            JOINT_PROJECTION = 0.8, MAX_JOINT_PROJECTION = 0.02, ANGLE_SLOP = 0.001, MAX_ANGULAR_BIAS = 0.5;
    private static final Vec3d[] FACES = {Vec3d.X, Vec3d.X.scale(-1), Vec3d.Y, Vec3d.Y.scale(-1), Vec3d.Z, Vec3d.Z.scale(-1)};
    private ContactSolver() {}

    /**
     * Offset is a world vector from the COM to the body's point nearest the obstacle. Normal is a unit
     * world vector from the obstacle toward the body. Negative separation is penetration depth.
     */
    public record Contact(Vec3d offset, Vec3d normal, double separationM, ContactMaterial material) {
        public Contact(Vec3d offset, Vec3d normal, double separationM) { this(offset, normal, separationM, null); }
        public Contact {
            Objects.requireNonNull(offset, "offset");
            Objects.requireNonNull(normal, "normal");
            if (Math.abs(normal.length() - 1) > 1e-9) { throw new IllegalArgumentException("Contact normal must be a unit vector"); }
            if (!Double.isFinite(separationM)) { throw new IllegalArgumentException("Separation must be finite"); }
        }
    }
    /**
     * Contact forces are each contact's impulse divided by dt, reported at its offset from the step's COM.
     * frictionDissipationW = sum of -f_t . u_t with the final friction force and final slip velocity: the
     * power turned into heat at sliding interfaces (zero while sticking or rolling). Normal-impact losses
     * from restitution below 1 are not included.
     */
    public record Result(RigidBodyState state, Wrench wrench, List<AppliedForce> forces, double frictionDissipationW) {}

    /**
     * Contacts at the current pose. Each sphere also keeps speculative contacts within the distance it
     * can travel in dt, so fast bodies stop at a surface instead of tunnelling through thin obstacles.
     */
    public static List<Contact> generate(RigidBodyState s, RigidBodyProperties p, List<CollisionSphere> shape,
                                         List<WorldBox> obstacles, double dt) {
        Checks.nonnegative(dt, "Time step");
        Vec3d omega = s.angularVelocity(p);
        List<Contact> contacts = new ArrayList<>();
        BoxIndex index = BoxIndex.of(obstacles);
        for (CollisionSphere sphere : shape) {
            Vec3d r = s.worldOffset(p, sphere.centerLocal());
            Vec3d center = s.position().add(r);
            double margin = s.velocity().add(omega.cross(r)).length() * dt + SPECULATIVE_DISTANCE;
            double reach = sphere.radiusM() + margin;
            Vec3d extent = new Vec3d(reach, reach, reach);
            WorldBox bounds = new WorldBox(center.subtract(extent), center.add(extent));
            if (index == null) {
                for (WorldBox box : obstacles) {
                    if (!box.intersects(bounds)) { continue; }
                    Contact contact = sphereBox(center, r, sphere.radiusM(), box, obstacles, null, margin);
                    if (contact != null) { contacts.add(contact); }
                }
            } else {
                // 0.30: only the boxes in the cells the sphere's bounds touch, in the list's order (the same contacts, in
                // the same order, as looking at every box)
                for (int i : index.near(bounds)) {
                    WorldBox box = obstacles.get(i);
                    if (!box.intersects(bounds)) { continue; }
                    Contact contact = sphereBox(center, r, sphere.radiusM(), box, obstacles, index, margin);
                    if (contact != null) { contacts.add(contact); }
                }
            }
        }
        return contacts;
    }

    /**
     * 0.30: a list of obstacles bucketed by 1 m cells: each box in every cell it touches (one spanning more than
     * MOST_CELLS kept apart and always looked at). Built once per list (the lists bodies pass are immutable: a few
     * recent ones are kept per thread by identity); a short list is simply scanned. Queries give list indices in
     * ascending order, so callers meet the boxes in the order a scan would.
     */
    static final class BoxIndex {
        private static final int SMALL = 24, MOST_CELLS = 64, MOST_QUERY_CELLS = 4096, RECENT = 8;
        private static final ThreadLocal<Object[][]> CACHE = ThreadLocal.withInitial(() -> new Object[RECENT][]);

        private final java.util.HashMap<Long, int[]> cells = new java.util.HashMap<>();
        private final int[] big;
        private final int size;

        static BoxIndex of(List<WorldBox> boxes) {
            if (boxes.size() < SMALL) { return null; }
            Object[][] recent = CACHE.get();
            for (Object[] e : recent) {
                if (e != null && ((java.lang.ref.WeakReference<?>) e[0]).get() == boxes) { return (BoxIndex) e[1]; }
            }
            BoxIndex index = new BoxIndex(boxes);
            System.arraycopy(recent, 0, recent, 1, RECENT - 1);
            recent[0] = new Object[]{new java.lang.ref.WeakReference<>(boxes), index};
            return index;
        }

        private BoxIndex(List<WorldBox> boxes) {
            size = boxes.size();
            java.util.HashMap<Long, java.util.ArrayList<Integer>> building = new java.util.HashMap<>();
            java.util.ArrayList<Integer> large = new java.util.ArrayList<>();
            for (int i = 0; i < size; i++) {
                WorldBox b = boxes.get(i);
                long x0 = cell(b.min().x()), y0 = cell(b.min().y()), z0 = cell(b.min().z()), x1 = cell(b.max().x()), y1 = cell(b.max().y()), z1 = cell(b.max().z());
                if (x0 == Long.MIN_VALUE || x1 == Long.MIN_VALUE || (x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > MOST_CELLS) { large.add(i); continue; }
                for (long x = x0; x <= x1; x++) {
                    for (long y = y0; y <= y1; y++) {
                        for (long z = z0; z <= z1; z++) { building.computeIfAbsent(key(x, y, z), k -> new java.util.ArrayList<>()).add(i); }
                    }
                }
            }
            for (var e : building.entrySet()) { cells.put(e.getKey(), e.getValue().stream().mapToInt(Integer::intValue).toArray()); }
            big = large.stream().mapToInt(Integer::intValue).toArray();
        }

        /** A coordinate's cell (Long.MIN_VALUE where it is not finite or far out). */
        private static long cell(double v) { return Double.isFinite(v) && Math.abs(v) < 1e9 ? (long) Math.floor(v) : Long.MIN_VALUE; }

        private static long key(long x, long y, long z) { return ((x & 0x1FFFFFL) << 42) | ((y & 0x1FFFFFL) << 21) | (z & 0x1FFFFFL); }

        /** The indices of the boxes that may touch a box, ascending (every index where the box is too big to bucket). */
        int[] near(WorldBox bounds) {
            long x0 = cell(bounds.min().x()), y0 = cell(bounds.min().y()), z0 = cell(bounds.min().z());
            long x1 = cell(bounds.max().x()), y1 = cell(bounds.max().y()), z1 = cell(bounds.max().z());
            if (x0 == Long.MIN_VALUE || y0 == Long.MIN_VALUE || z0 == Long.MIN_VALUE || x1 == Long.MIN_VALUE || y1 == Long.MIN_VALUE || z1 == Long.MIN_VALUE
                    || (x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > MOST_QUERY_CELLS) {
                int[] all = new int[size];
                for (int i = 0; i < size; i++) { all[i] = i; }
                return all;
            }
            int[] found = new int[16];
            int n = 0;
            for (long x = x0; x <= x1; x++) {
                for (long y = y0; y <= y1; y++) {
                    for (long z = z0; z <= z1; z++) {
                        int[] in = cells.get(key(x, y, z));
                        if (in == null) { continue; }
                        if (n + in.length > found.length) { found = java.util.Arrays.copyOf(found, Math.max(2 * found.length, n + in.length)); }
                        System.arraycopy(in, 0, found, n, in.length);
                        n += in.length;
                    }
                }
            }
            if (n + big.length > found.length) { found = java.util.Arrays.copyOf(found, n + big.length); }
            System.arraycopy(big, 0, found, n, big.length);
            n += big.length;
            java.util.Arrays.sort(found, 0, n);
            int unique = 0;
            for (int i = 0; i < n; i++) { if (unique == 0 || found[i] != found[unique - 1]) { found[unique++] = found[i]; } }
            return java.util.Arrays.copyOf(found, unique);
        }

        /** The indices of the boxes whose interior may hold a point (any order). */
        int[] at(Vec3d point) {
            long x = cell(point.x()), y = cell(point.y()), z = cell(point.z());
            if (x == Long.MIN_VALUE || y == Long.MIN_VALUE || z == Long.MIN_VALUE) { return near(new WorldBox(point, point)); }
            int[] in = cells.get(key(x, y, z));
            if (in == null) { return big; }
            if (big.length == 0) { return in; }
            int[] both = java.util.Arrays.copyOf(in, in.length + big.length);
            System.arraycopy(big, 0, both, in.length, big.length);
            return both;
        }
    }

    private static Contact sphereBox(Vec3d center, Vec3d r, double radius, WorldBox box, List<WorldBox> all, BoxIndex index, double margin) {
        Vec3d closest = box.closestPoint(center);
        Vec3d delta = center.subtract(closest);
        double distance = delta.length();
        Vec3d normal;
        double separation;
        if (distance > 1e-9) {
            separation = distance - radius;
            if (separation > margin) { return null; }
            normal = delta.scale(1 / distance);
            // A face covered by a neighbouring obstacle is internal; that neighbour reports the real surface.
            if (insideOther(closest.add(normal.scale(1e-6)), box, all, index)) { return null; }
        } else {
            // Center inside: leave through the nearest face that is not shared with another obstacle,
            // so adjacent blocks do not push the body into each other.
            double[] exits = {box.max().x() - center.x(), center.x() - box.min().x(), box.max().y() - center.y(),
                    center.y() - box.min().y(), box.max().z() - center.z(), center.z() - box.min().z()};
            int best = -1, fallback = 0;
            for (int i = 0; i < 6; i++) {
                if (exits[i] < exits[fallback]) { fallback = i; }
                if (insideOther(center.add(FACES[i].scale(exits[i] + 1e-6)), box, all, index)) { continue; }
                if (best < 0 || exits[i] < exits[best]) { best = i; }
            }
            if (best < 0) { best = fallback; }
            normal = FACES[best];
            separation = -(exits[best] + radius);
        }
        return new Contact(r.subtract(normal.scale(radius)), normal, separation, box.material());
    }

    private static boolean insideOther(Vec3d point, WorldBox self, List<WorldBox> all, BoxIndex index) {
        if (index != null) {
            for (int i : index.at(point)) {
                WorldBox box = all.get(i);
                if (box != self && box.containsInterior(point)) { return true; }
            }
            return false;
        }
        for (WorldBox box : all) {
            if (box != self && box.containsInterior(point)) { return true; }
        }
        return false;
    }

    /**
     * Projected Gauss-Seidel on non-negative normal impulses and friction inside the cone |f| <= mu N.
     * Target normal velocity is zero for touching contacts and -gap/dt for speculative ones. Restitution
     * is applied afterwards to contacts that carried load, using the pre-solve approach speed. A speculative
     * contact therefore rebounds in the step its impact is predicted, up to |v| dt before the face (the same
     * trade-off as Box2D v3); it never penetrates.
     */
    public static Result solve(RigidBodyState s, RigidBodyProperties p, List<Contact> contacts,
                               ContactMaterial material, double dt) {
        Checks.positive(dt, "Time step");
        Objects.requireNonNull(material, "material");
        if (contacts.isEmpty()) { return new Result(s, Wrench.ZERO, List.of(), 0); }
        int count = contacts.size();
        MovingBody body = new MovingBody(s, p);
        Vec3d[] tangent1 = new Vec3d[count], tangent2 = new Vec3d[count];
        double[] normalMass = new double[count], tangentMass1 = new double[count], tangentMass2 = new double[count];
        double[] approach = new double[count], bias = new double[count];
        double[] normalImpulse = new double[count], tangentImpulse1 = new double[count], tangentImpulse2 = new double[count];
        for (int i = 0; i < count; i++) {
            Contact c = contacts.get(i);
            Vec3d n = c.normal();
            Vec3d helper = Math.abs(n.x()) < 0.57735 ? Vec3d.X : Math.abs(n.y()) < 0.57735 ? Vec3d.Y : Vec3d.Z;
            tangent1[i] = n.cross(helper).normalized();
            tangent2[i] = n.cross(tangent1[i]);
            normalMass[i] = 1 / body.inverseMass(c.offset(), n);
            tangentMass1[i] = 1 / body.inverseMass(c.offset(), tangent1[i]);
            tangentMass2[i] = 1 / body.inverseMass(c.offset(), tangent2[i]);
            approach[i] = n.dot(body.velocityAt(c.offset()));
            bias[i] = Math.max(c.separationM(), 0) / dt;
        }
        for (int iteration = 0; iteration < VELOCITY_ITERATIONS; iteration++) {
            for (int i = 0; i < count; i++) {
                Contact c = contacts.get(i);
                Vec3d r = c.offset();
                // Each obstacle may carry its own material (a block class); otherwise the body's.
                double mu = (c.material() != null ? c.material() : material).friction();
                if (mu > 0) {
                    Vec3d v = body.velocityAt(r);
                    double candidate1 = tangentImpulse1[i] - tangent1[i].dot(v) * tangentMass1[i];
                    double candidate2 = tangentImpulse2[i] - tangent2[i].dot(v) * tangentMass2[i];
                    double limit = mu * normalImpulse[i], magnitude = Math.hypot(candidate1, candidate2);
                    if (magnitude > limit) {
                        double scale = magnitude > 0 ? limit / magnitude : 0;
                        candidate1 *= scale;
                        candidate2 *= scale;
                    }
                    body.applyImpulse(r, tangent1[i].scale(candidate1 - tangentImpulse1[i])
                            .add(tangent2[i].scale(candidate2 - tangentImpulse2[i])));
                    tangentImpulse1[i] = candidate1;
                    tangentImpulse2[i] = candidate2;
                }
                double vn = c.normal().dot(body.velocityAt(r));
                double candidate = Math.max(normalImpulse[i] - (vn + bias[i]) * normalMass[i], 0);
                body.applyImpulse(r, c.normal().scale(candidate - normalImpulse[i]));
                normalImpulse[i] = candidate;
            }
        }
        for (int i = 0; i < count; i++) {
            Contact c = contacts.get(i);
            ContactMaterial m = c.material() != null ? c.material() : material;
            if (m.restitution() > 0) {
                if (normalImpulse[i] <= 0 || approach[i] >= -RESTITUTION_THRESHOLD) { continue; }
                double vn = c.normal().dot(body.velocityAt(c.offset()));
                double candidate = Math.max(normalImpulse[i] - (vn + m.restitutionAt(approach[i]) * approach[i]) * normalMass[i], 0);
                body.applyImpulse(c.offset(), c.normal().scale(candidate - normalImpulse[i]));
                normalImpulse[i] = candidate;
            }
        }
        List<AppliedForce> forces = new ArrayList<>();
        Wrench total = Wrench.ZERO;
        double friction = 0;
        for (int i = 0; i < count; i++) {
            Contact c = contacts.get(i);
            Vec3d tangential = tangent1[i].scale(tangentImpulse1[i]).add(tangent2[i].scale(tangentImpulse2[i]));
            Vec3d impulse = c.normal().scale(normalImpulse[i]).add(tangential);
            if (impulse.lengthSquared() == 0) { continue; }
            AppliedForce force = new AppliedForce(AppliedForce.Kind.CONTACT, c.offset(), impulse.scale(1 / dt));
            forces.add(force);
            total = total.add(force.wrench());
            Vec3d v = body.velocityAt(c.offset());
            // Power removed by this friction force from the final slip velocity, as for implicit drag.
            friction -= tangential.dot(v.subtract(c.normal().scale(c.normal().dot(v)))) / dt;
        }
        return new Result(body.state(), total, List.copyOf(forces), friction);
    }

    /**
     * Nonlinear Gauss-Seidel position projection for penetration beyond SLOP. Changes position and
     * orientation only; velocity and world angular momentum are kept, so no kinetic energy is injected.
     */
    public static RigidBodyState separate(RigidBodyState s, RigidBodyProperties p, List<CollisionSphere> shape,
                                          List<WorldBox> obstacles) {
        RigidBodyState current = s;
        double inverseMass = 1 / p.massKg();
        for (int iteration = 0; iteration < POSITION_ITERATIONS; iteration++) {
            Vec3d shift = Vec3d.ZERO, turn = Vec3d.ZERO;
            boolean moved = false;
            for (Contact c : generate(current, p, shape, obstacles, 0)) {
                // Linearized separation after the corrections already made in this sweep.
                double separation = c.separationM() + c.normal().dot(shift.add(turn.cross(c.offset())));
                double correction = Math.clamp(PROJECTION * (separation + SLOP), -MAX_PROJECTION, 0);
                if (correction == 0) { continue; }
                Vec3d rn = c.offset().cross(c.normal());
                Vec3d angular = p.inertia().solveWorld(current.orientation(), rn);
                double impulse = -correction / (inverseMass + rn.dot(angular));
                shift = shift.add(c.normal().scale(impulse * inverseMass));
                turn = turn.add(angular.scale(impulse));
                moved = true;
            }
            if (!moved) { break; }
            current = new RigidBodyState(current.position().add(shift), current.velocity(),
                    Orientation.rotationVector(turn).multiply(current.orientation()), current.angularMomentum());
        }
        return current;
    }

    // ---- Several moving bodies ----

    /** A moving rigid body taking part in a joint contact solve. */
    public record Body(RigidBodyState state, RigidBodyProperties properties) {
        public Body { Objects.requireNonNull(state, "state"); Objects.requireNonNull(properties, "properties"); }
    }
    /**
     * Contact between body a and body b, or a static obstacle when b is -1. Offsets are world vectors from each
     * body's COM to its contact point (offsetB is ignored for static contacts). The unit normal points from b toward
     * a; negative separation is penetration. The material is the pair's combined friction and restitution.
     */
    public record PairContact(int a, int b, Vec3d offsetA, Vec3d offsetB, Vec3d normal, double separationM, ContactMaterial material) {
        public PairContact {
            if (a < 0 || b < -1 || a == b) { throw new IllegalArgumentException("Contact needs a moving body and a distinct second body or -1"); }
            Objects.requireNonNull(offsetA, "offsetA"); Objects.requireNonNull(offsetB, "offsetB");
            Objects.requireNonNull(normal, "normal"); Objects.requireNonNull(material, "material");
            if (Math.abs(normal.length() - 1) > 1e-9) { throw new IllegalArgumentException("Contact normal must be a unit vector"); }
            if (!Double.isFinite(separationM)) { throw new IllegalArgumentException("Separation must be finite"); }
        }
    }
    /**
     * Per-body result of a joint solve. forces are the impulses this body received divided by dt.
     * staticFrictionW is the frictional power at its static contacts (to be shared with the ground by the caller);
     * pairFrictionW is half the frictional power at its body-body contacts (equal shares, like materials).
     * touches (0.37) are the contacts that pushed on it this step, for whoever listens (sounds, effects).
     */
    public record BodyResult(RigidBodyState state, Wrench wrench, List<AppliedForce> forces, double staticFrictionW, double pairFrictionW, List<Touch> touches) {
        public BodyResult { touches = List.copyOf(touches); }
        public BodyResult(RigidBodyState state, Wrench wrench, List<AppliedForce> forces, double staticFrictionW, double pairFrictionW) {
            this(state, wrench, forces, staticFrictionW, pairFrictionW, List.of());
        }
    }

    /**
     * 0.37: how one contact went, read after the solve (it changes nothing in it): the contact point as a world vector
     * from the body's COM, the normal pushing on the body, the normal speed it met at before the step (negative:
     * closing), the normal impulse it took (N s), how fast the surfaces slid at the end (m/s), the other body (its index
     * in the solve, -1 for a static obstacle) and the pair's material.
     */
    public record Touch(Vec3d offset, Vec3d normal, double approachMps, double normalImpulseNs, double slideMps, int other, ContactMaterial material) {}

    /** 0.36: body follower hangs on body leader by a coupling (indices into the bodies of the solve). */
    public record Joint(int leader, int follower, dev.mcmod.aerocore.api.vehicle.Coupling coupling) {
        public Joint {
            if (leader < 0 || follower < 0 || leader == follower) { throw new IllegalArgumentException("A joint needs two distinct bodies"); }
            Objects.requireNonNull(coupling, "coupling");
        }
    }
    /**
     * What a joint carried in a solve: the force on the follower at its point and the couple of the stops on it (world;
     * the leader got the opposite), and where the joint stood at the start (angles as Coupling.geometry, gap m).
     */
    public record JointResult(Vec3d forceN, Vec3d coupleNm, double pitchRad, double yawRad, double rollRad, double gapM) {}
    /** A solve with joints: the bodies' results as solveAll gives them, and each joint's. */
    public record Solution(List<BodyResult> bodies, List<JointResult> joints) {}

    /** Box2D-style mixing for a body pair: geometric-mean friction, the larger restitution, the lower yield velocity. */
    public static ContactMaterial combine(ContactMaterial a, ContactMaterial b) {
        return new ContactMaterial(Math.sqrt(a.friction() * b.friction()), Math.max(a.restitution(), b.restitution()),
                Math.min(a.yieldVelocityMps(), b.yieldVelocityMps()));
    }

    /** Static contacts of body index a, from generate(), tagged for solveAll(). */
    public static List<PairContact> tag(int a, List<Contact> contacts, ContactMaterial material) {
        List<PairContact> tagged = new ArrayList<>();
        for (Contact c : contacts) { tagged.add(new PairContact(a, -1, c.offset(), Vec3d.ZERO, c.normal(), c.separationM(), c.material() != null ? c.material() : material)); }
        return tagged;
    }

    /**
     * Sphere-sphere contacts between bodies a and b at their current poses, with the same speculative margin as
     * generate() (relative point speed times dt plus 2 cm). Shapes are model-local; spheres must not be nested.
     * <p>
     * Two hulls of about a hundred spheres each test ten thousand pairs per call, once per substep and again in every
     * separation iteration, so B's spheres are placed in the world once, and a pair farther apart than the radii plus
     * the largest margin it could have (the sum of both point speeds, times dt) is skipped before anything else; the
     * pairs kept and their contacts are the same as testing every pair in full.
     */
    public static List<PairContact> generatePair(int a, Body bodyA, List<CollisionSphere> shapeA, int b, Body bodyB,
                                                 List<CollisionSphere> shapeB, ContactMaterial material, double dt) {
        Checks.nonnegative(dt, "Time step");
        List<PairContact> contacts = new ArrayList<>();
        Vec3d omegaA = bodyA.state().angularVelocity(bodyA.properties()), omegaB = bodyB.state().angularVelocity(bodyB.properties());
        int countB = shapeB.size();
        Vec3d[] offsetB = new Vec3d[countB], velocityB = new Vec3d[countB];
        double[] xB = new double[countB], yB = new double[countB], zB = new double[countB], speedB = new double[countB];
        for (int j = 0; j < countB; j++) {
            Vec3d rb = bodyB.state().worldOffset(bodyB.properties(), shapeB.get(j).centerLocal()), cb = bodyB.state().position().add(rb);
            offsetB[j] = rb; velocityB[j] = bodyB.state().velocity().add(omegaB.cross(rb));
            xB[j] = cb.x(); yB[j] = cb.y(); zB[j] = cb.z(); speedB[j] = velocityB[j].length();
        }
        // 0.30: a sphere around all of B's spheres and how far they reach this step: a sphere of A beyond it touches none
        // of them (the bound is generous by a part in a billion, so it never skips a pair the test below would take)
        double mx = 0, my = 0, mz = 0;
        for (int j = 0; j < countB; j++) { mx += xB[j]; my += yB[j]; mz += zB[j]; }
        if (countB > 0) { mx /= countB; my /= countB; mz /= countB; }
        double boundB = 0;
        for (int j = 0; j < countB; j++) {
            double ex = xB[j] - mx, ey = yB[j] - my, ez = zB[j] - mz;
            boundB = Math.max(boundB, Math.sqrt(ex * ex + ey * ey + ez * ez) + shapeB.get(j).radiusM() + speedB[j] * dt);
        }
        for (CollisionSphere sa : shapeA) {
            Vec3d ra = bodyA.state().worldOffset(bodyA.properties(), sa.centerLocal()), ca = bodyA.state().position().add(ra);
            Vec3d va = bodyA.state().velocity().add(omegaA.cross(ra));
            double speedA = va.length();
            double fx = ca.x() - mx, fy = ca.y() - my, fz = ca.z() - mz, limit = (boundB + sa.radiusM() + speedA * dt + SPECULATIVE_DISTANCE) * (1 + 1e-9) + 1e-9;
            if (Double.isFinite(limit) && fx * fx + fy * fy + fz * fz > limit * limit) { continue; }
            for (int j = 0; j < countB; j++) {
                CollisionSphere sb = shapeB.get(j);
                double dx = ca.x() - xB[j], dy = ca.y() - yB[j], dz = ca.z() - zB[j];
                double reach = sa.radiusM() + sb.radiusM() + (speedA + speedB[j]) * dt + SPECULATIVE_DISTANCE;
                if (dx * dx + dy * dy + dz * dz > reach * reach * (1 + 1e-12)) { continue; }
                Vec3d rb = offsetB[j];
                Vec3d delta = new Vec3d(dx, dy, dz);
                double distance = delta.length();
                double margin = va.subtract(velocityB[j]).length() * dt + SPECULATIVE_DISTANCE;
                double separation = distance - sa.radiusM() - sb.radiusM();
                if (separation > margin) { continue; }
                // Coincident centres have no defined normal; push along +Y, which only happens in degenerate setups.
                Vec3d n = distance > 1e-9 ? delta.scale(1 / distance) : Vec3d.Y;
                contacts.add(new PairContact(a, b, ra.subtract(n.scale(sa.radiusM())), rb.add(n.scale(sb.radiusM())), n, separation, material));
            }
        }
        return contacts;
    }

    /**
     * Joint projected Gauss-Seidel over static and body-body contacts, the multi-body form of solve(): equal and
     * opposite impulses on each pair, so the bodies' total momentum changes only through static contacts.
     */
    public static List<BodyResult> solveAll(List<Body> bodies, List<PairContact> contacts, double dt) {
        return solveAll(bodies, contacts, List.of(), dt).bodies();
    }

    /**
     * solveAll with joints between bodies (0.36). Every iteration takes the contacts and then the joints: each joint's
     * stops (roll, pitch, yaw) and last its points, held together as an exact 3x3 block (so a light tractor on a heavy
     * trailer stays stiff). A gap beyond JOINT_SLOP is closed a fifth per step (at most MAX_JOINT_BIAS); separateAll with
     * the joints takes out what is left. The joints' impulses are in the bodies' forces (CONTACT: the points' forces, the
     * stops' couples). With no joints the result is solveAll's, bit for bit.
     */
    public static Solution solveAll(List<Body> bodies, List<PairContact> contacts, List<Joint> joints, double dt) {
        Checks.positive(dt, "Time step");
        MovingBody[] moving = new MovingBody[bodies.size()];
        for (int i = 0; i < moving.length; i++) { moving[i] = new MovingBody(bodies.get(i).state(), bodies.get(i).properties()); }
        int count = contacts.size();
        Vec3d[] tangent1 = new Vec3d[count], tangent2 = new Vec3d[count];
        double[] normalMass = new double[count], tangentMass1 = new double[count], tangentMass2 = new double[count];
        double[] approach = new double[count], bias = new double[count];
        double[] normalImpulse = new double[count], tangentImpulse1 = new double[count], tangentImpulse2 = new double[count];
        for (int i = 0; i < count; i++) {
            PairContact c = contacts.get(i);
            if (c.a() >= moving.length || c.b() >= moving.length) { throw new IllegalArgumentException("Contact refers to a missing body"); }
            Vec3d n = c.normal();
            Vec3d helper = Math.abs(n.x()) < 0.57735 ? Vec3d.X : Math.abs(n.y()) < 0.57735 ? Vec3d.Y : Vec3d.Z;
            tangent1[i] = n.cross(helper).normalized();
            tangent2[i] = n.cross(tangent1[i]);
            normalMass[i] = 1 / inverseMass(moving, c, n);
            tangentMass1[i] = 1 / inverseMass(moving, c, tangent1[i]);
            tangentMass2[i] = 1 / inverseMass(moving, c, tangent2[i]);
            approach[i] = n.dot(relativeVelocity(moving, c));
            bias[i] = Math.max(c.separationM(), 0) / dt;
        }
        JointRows[] rows = new JointRows[joints.size()];
        for (int j = 0; j < rows.length; j++) {
            Joint joint = joints.get(j);
            if (joint.leader() >= moving.length || joint.follower() >= moving.length) { throw new IllegalArgumentException("Joint refers to a missing body"); }
            rows[j] = new JointRows(joint, moving[joint.leader()], moving[joint.follower()], dt);
        }
        for (int iteration = 0; iteration < VELOCITY_ITERATIONS; iteration++) {
            for (int i = 0; i < count; i++) {
                PairContact c = contacts.get(i);
                double mu = c.material().friction();
                if (mu > 0) {
                    Vec3d v = relativeVelocity(moving, c);
                    double candidate1 = tangentImpulse1[i] - tangent1[i].dot(v) * tangentMass1[i];
                    double candidate2 = tangentImpulse2[i] - tangent2[i].dot(v) * tangentMass2[i];
                    double limit = mu * normalImpulse[i], magnitude = Math.hypot(candidate1, candidate2);
                    if (magnitude > limit) {
                        double scale = magnitude > 0 ? limit / magnitude : 0;
                        candidate1 *= scale;
                        candidate2 *= scale;
                    }
                    apply(moving, c, tangent1[i].scale(candidate1 - tangentImpulse1[i]).add(tangent2[i].scale(candidate2 - tangentImpulse2[i])));
                    tangentImpulse1[i] = candidate1;
                    tangentImpulse2[i] = candidate2;
                }
                double vn = c.normal().dot(relativeVelocity(moving, c));
                double candidate = Math.max(normalImpulse[i] - (vn + bias[i]) * normalMass[i], 0);
                apply(moving, c, c.normal().scale(candidate - normalImpulse[i]));
                normalImpulse[i] = candidate;
            }
            for (JointRows r : rows) { r.iterate(); }
        }
        for (int i = 0; i < count; i++) {
            PairContact c = contacts.get(i);
            double e = c.material().restitutionAt(approach[i]);
            if (e <= 0 || normalImpulse[i] <= 0 || approach[i] >= -RESTITUTION_THRESHOLD) { continue; }
            double vn = c.normal().dot(relativeVelocity(moving, c));
            double candidate = Math.max(normalImpulse[i] - (vn + e * approach[i]) * normalMass[i], 0);
            apply(moving, c, c.normal().scale(candidate - normalImpulse[i]));
            normalImpulse[i] = candidate;
        }
        List<List<AppliedForce>> forces = new ArrayList<>();
        Wrench[] totals = new Wrench[moving.length];
        double[] staticFriction = new double[moving.length], pairFriction = new double[moving.length];
        List<List<Touch>> touches = new ArrayList<>();
        for (int k = 0; k < moving.length; k++) { forces.add(new ArrayList<>()); totals[k] = Wrench.ZERO; touches.add(new ArrayList<>()); }
        for (int i = 0; i < count; i++) {
            PairContact c = contacts.get(i);
            Vec3d tangential = tangent1[i].scale(tangentImpulse1[i]).add(tangent2[i].scale(tangentImpulse2[i]));
            Vec3d impulse = c.normal().scale(normalImpulse[i]).add(tangential);
            if (impulse.lengthSquared() == 0) { continue; }
            AppliedForce onA = new AppliedForce(AppliedForce.Kind.CONTACT, c.offsetA(), impulse.scale(1 / dt));
            forces.get(c.a()).add(onA); totals[c.a()] = totals[c.a()].add(onA.wrench());
            Vec3d v = relativeVelocity(moving, c);
            Vec3d slip = v.subtract(c.normal().scale(c.normal().dot(v)));
            double power = -tangential.dot(slip) / dt;
            if (normalImpulse[i] > 0) {
                double slide = slip.length();
                touches.get(c.a()).add(new Touch(c.offsetA(), c.normal(), approach[i], normalImpulse[i], slide, c.b(), c.material()));
                if (c.b() >= 0) { touches.get(c.b()).add(new Touch(c.offsetB(), c.normal().scale(-1), approach[i], normalImpulse[i], slide, c.a(), c.material())); }
            }
            if (c.b() < 0) { staticFriction[c.a()] += power; continue; }
            AppliedForce onB = new AppliedForce(AppliedForce.Kind.CONTACT, c.offsetB(), impulse.scale(-1 / dt));
            forces.get(c.b()).add(onB); totals[c.b()] = totals[c.b()].add(onB.wrench());
            pairFriction[c.a()] += power / 2; pairFriction[c.b()] += power / 2;
        }
        List<JointResult> carried = new ArrayList<>();
        for (int j = 0; j < rows.length; j++) {
            JointRows r = rows[j];
            int l = joints.get(j).leader(), f = joints.get(j).follower();
            Vec3d couple = r.couple();
            if (r.point.lengthSquared() > 0) {
                AppliedForce onF = new AppliedForce(AppliedForce.Kind.CONTACT, r.rf, r.point.scale(1 / dt));
                AppliedForce onL = new AppliedForce(AppliedForce.Kind.CONTACT, r.rl, r.point.scale(-1 / dt));
                forces.get(f).add(onF); totals[f] = totals[f].add(onF.wrench());
                forces.get(l).add(onL); totals[l] = totals[l].add(onL.wrench());
            }
            if (couple.lengthSquared() > 0) {
                AppliedForce onF = new AppliedForce(AppliedForce.Kind.CONTACT, Vec3d.ZERO, Vec3d.ZERO, couple.scale(1 / dt));
                AppliedForce onL = new AppliedForce(AppliedForce.Kind.CONTACT, Vec3d.ZERO, Vec3d.ZERO, couple.scale(-1 / dt));
                forces.get(f).add(onF); totals[f] = totals[f].add(onF.wrench());
                forces.get(l).add(onL); totals[l] = totals[l].add(onL.wrench());
            }
            carried.add(new JointResult(r.point.scale(1 / dt), couple.scale(1 / dt), r.g.pitchRad(), r.g.yawRad(), r.g.rollRad(), r.g.gap().length()));
        }
        List<BodyResult> results = new ArrayList<>();
        for (int k = 0; k < moving.length; k++) {
            results.add(new BodyResult(moving[k].state(), totals[k], List.copyOf(forces.get(k)), staticFriction[k], pairFriction[k], touches.get(k)));
        }
        return new Solution(results, List.copyOf(carried));
    }

    /**
     * One joint's rows in a solve: its stops (each an angular row along an axis a, the angle's rate being a . (omega of the
     * follower - omega of the leader)) and its point block. Impulses go +a / +P on the follower and the opposite on the leader.
     */
    private static final class JointRows {
        private final MovingBody leader, follower;
        private final dev.mcmod.aerocore.api.vehicle.Coupling.Geometry g;
        private final Vec3d rl, rf, bias;
        private final double[] inverse;
        private final List<Angular> stops = new ArrayList<>();
        /** The impulse on the follower at its point so far. */
        private Vec3d point = Vec3d.ZERO;

        private JointRows(Joint joint, MovingBody leader, MovingBody follower, double dt) {
            this.leader = leader; this.follower = follower;
            var c = joint.coupling();
            g = c.geometry(leader.initial, leader.properties, follower.initial, follower.properties);
            rl = g.leaderOffset(); rf = g.followerOffset();
            inverse = invert(block(leader, rl, follower, rf));
            double gap = g.gap().length();
            Vec3d b = gap > JOINT_SLOP ? g.gap().scale(JOINT_BAUMGARTE * (gap - JOINT_SLOP) / gap / dt) : Vec3d.ZERO;
            double speed = b.length();
            bias = speed > MAX_JOINT_BIAS ? b.scale(MAX_JOINT_BIAS / speed) : b;
            if (c.rollStopped()) {
                // the roll's rate along the kingpin x hinge (unit): d asin(p . k)/dt = (omega_F - omega_L) . (k x p)/|k x p|
                Vec3d n = g.yawAxis().cross(g.pitchAxis());
                if (n.length() > 1e-6) {
                    Vec3d a = n.normalized();
                    double k = c.rollStiffnessNmPerRad(), d = c.rollDampingNms();
                    if (c.rollHeld() && k == Double.POSITIVE_INFINITY) { stops.add(Angular.held(a, g.rollRad(), dt)); }
                    else if (k > 0 || d > 0) { stops.add(Angular.spring(a, g.rollRad(), k, d, dt)); }
                    if (!c.rollHeld()) { stops.add(Angular.upper(a, g.rollRad(), c.rollLimitRad(), dt)); stops.add(Angular.lower(a, g.rollRad(), c.rollLimitRad(), dt)); }
                }
            }
            if (c.pitchLimitRad() < Math.PI) {
                stops.add(Angular.upper(g.pitchAxis(), g.pitchRad(), c.pitchLimitRad(), dt));
                stops.add(Angular.lower(g.pitchAxis(), g.pitchRad(), c.pitchLimitRad(), dt));
            }
            if (c.yawLimitRad() < Math.PI) {
                stops.add(Angular.upper(g.yawAxis(), g.yawRad(), c.yawLimitRad(), dt));
                stops.add(Angular.lower(g.yawAxis(), g.yawRad(), c.yawLimitRad(), dt));
            }
            for (Angular s : stops) { s.mass(leader, follower); }
        }

        private void iterate() {
            for (Angular s : stops) { s.iterate(leader, follower); }
            Vec3d v = follower.velocityAt(rf).subtract(leader.velocityAt(rl)).add(bias);
            Vec3d impulse = times(inverse, v).scale(-1);
            follower.applyImpulse(rf, impulse);
            leader.applyImpulse(rl, impulse.scale(-1));
            point = point.add(impulse);
        }

        /** The stops' angular impulse on the follower so far. */
        private Vec3d couple() {
            Vec3d sum = Vec3d.ZERO;
            for (Angular s : stops) { if (s.impulse != 0) { sum = sum.add(s.axis.scale(s.impulse)); } }
            return sum;
        }
    }

    /**
     * An angular row: held (an equality with a Baumgarte target), a spring (Catto's soft constraint: gamma and the
     * target from stiffness and damping), or a one-sided stop (upper: rate at most target, lower: at least; speculative
     * short of the stop, pushing back past it).
     */
    private static final class Angular {
        private enum Type { HELD, SPRING, UPPER, LOWER }
        private final Vec3d axis;
        private final Type type;
        private final double target, gamma;
        private double mass, impulse;

        private Angular(Vec3d axis, Type type, double target, double gamma) { this.axis = axis; this.type = type; this.target = target; this.gamma = gamma; }

        static Angular held(Vec3d a, double angle, double dt) {
            return new Angular(a, Type.HELD, Math.clamp(-JOINT_BAUMGARTE * angle / dt, -MAX_ANGULAR_BIAS, MAX_ANGULAR_BIAS), 0);
        }
        static Angular spring(Vec3d a, double angle, double stiffness, double damping, double dt) {
            double denominator = damping + dt * stiffness;
            return new Angular(a, Type.SPRING, -dt * stiffness / denominator * angle / dt, 1 / (dt * denominator));
        }
        static Angular upper(Vec3d a, double angle, double limit, double dt) {
            double room = limit - angle;
            return new Angular(a, Type.UPPER, room >= 0 ? room / dt : Math.max(JOINT_BAUMGARTE * room / dt, -MAX_ANGULAR_BIAS), 0);
        }
        static Angular lower(Vec3d a, double angle, double limit, double dt) {
            double room = -limit - angle;
            return new Angular(a, Type.LOWER, room <= 0 ? room / dt : Math.min(JOINT_BAUMGARTE * room / dt, MAX_ANGULAR_BIAS), 0);
        }

        void mass(MovingBody leader, MovingBody follower) {
            double inverse = axis.dot(leader.spin(axis)) + axis.dot(follower.spin(axis));
            mass = 1 / (inverse + gamma);
        }

        void iterate(MovingBody leader, MovingBody follower) {
            double rate = axis.dot(follower.omega().subtract(leader.omega()));
            double candidate = switch (type) {
                case HELD -> impulse - (rate - target) * mass;
                case SPRING -> impulse - (rate - target + gamma * impulse) * mass;
                case UPPER -> Math.min(impulse - (rate - target) * mass, 0);
                case LOWER -> Math.max(impulse - (rate - target) * mass, 0);
            };
            if (candidate == impulse) { return; }
            Vec3d d = axis.scale(candidate - impulse);
            follower.applyAngular(d);
            leader.applyAngular(d.scale(-1));
            impulse = candidate;
        }
    }

    /** The 3x3 matrix taking an impulse +P at rb on b and -P at ra on a to the change of b's point velocity less a's (row-major). */
    private static double[] block(MovingBody a, Vec3d ra, MovingBody b, Vec3d rb) {
        double[] k = new double[9];
        Vec3d[] axes = {Vec3d.X, Vec3d.Y, Vec3d.Z};
        for (int column = 0; column < 3; column++) {
            Vec3d c = a.pointResponse(ra, axes[column]).add(b.pointResponse(rb, axes[column]));
            k[column] = c.x(); k[3 + column] = c.y(); k[6 + column] = c.z();
        }
        return k;
    }
    private static double[] invert(double[] m) {
        double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
        double ca = e * i - f * h, cb = -(d * i - f * g), cc = d * h - e * g;
        double det = a * ca + b * cb + c * cc;
        if (!(Math.abs(det) > 0) || !Double.isFinite(det)) { throw new IllegalArgumentException("A joint between bodies with no effective mass"); }
        double s = 1 / det;
        return new double[]{ca * s, -(b * i - c * h) * s, (b * f - c * e) * s,
                cb * s, (a * i - c * g) * s, -(a * f - c * d) * s,
                cc * s, -(a * h - b * g) * s, (a * e - b * d) * s};
    }
    private static Vec3d times(double[] m, Vec3d v) {
        return new Vec3d(m[0] * v.x() + m[1] * v.y() + m[2] * v.z(), m[3] * v.x() + m[4] * v.y() + m[5] * v.z(), m[6] * v.x() + m[7] * v.y() + m[8] * v.z());
    }

    /**
     * Position projection for static and body-body penetration beyond SLOP, the multi-body form of separate().
     * contacts is regenerated by the caller at the current poses; only poses change.
     */
    public static List<RigidBodyState> separateAll(List<Body> bodies, java.util.function.Function<List<Body>, List<PairContact>> contacts) {
        return separateAll(bodies, contacts, List.of());
    }

    /**
     * separateAll with joints (0.36): after the contacts, each iteration moves the joined bodies to close their points'
     * gap beyond JOINT_SLOP (a share JOINT_PROJECTION, at most MAX_JOINT_PROJECTION per iteration, shared by the bodies'
     * effective masses) and to stand a rigidly held roll square again (or a stopped one back within its play). Only poses
     * change. With no joints it is separateAll as before, bit for bit.
     */
    public static List<RigidBodyState> separateAll(List<Body> bodies, java.util.function.Function<List<Body>, List<PairContact>> contacts, List<Joint> joints) {
        for (Joint j : joints) {
            if (j.leader() >= bodies.size() || j.follower() >= bodies.size()) { throw new IllegalArgumentException("Joint refers to a missing body"); }
        }
        List<Body> current = new ArrayList<>(bodies);
        for (int iteration = 0; iteration < POSITION_ITERATIONS; iteration++) {
            Vec3d[] shift = new Vec3d[current.size()], turn = new Vec3d[current.size()];
            java.util.Arrays.fill(shift, Vec3d.ZERO); java.util.Arrays.fill(turn, Vec3d.ZERO);
            boolean moved = false;
            for (PairContact c : contacts.apply(current)) {
                Body a = current.get(c.a());
                Body b = c.b() < 0 ? null : current.get(c.b());
                Vec3d moveA = shift[c.a()].add(turn[c.a()].cross(c.offsetA()));
                Vec3d moveB = b == null ? Vec3d.ZERO : shift[c.b()].add(turn[c.b()].cross(c.offsetB()));
                double separation = c.separationM() + c.normal().dot(moveA.subtract(moveB));
                double correction = Math.clamp(PROJECTION * (separation + SLOP), -MAX_PROJECTION, 0);
                if (correction == 0) { continue; }
                Vec3d rnA = c.offsetA().cross(c.normal()), angularA = a.properties().inertia().solveWorld(a.state().orientation(), rnA);
                double inverse = 1 / a.properties().massKg() + rnA.dot(angularA);
                Vec3d rnB = Vec3d.ZERO, angularB = Vec3d.ZERO;
                if (b != null) {
                    rnB = c.offsetB().cross(c.normal());
                    angularB = b.properties().inertia().solveWorld(b.state().orientation(), rnB);
                    inverse += 1 / b.properties().massKg() + rnB.dot(angularB);
                }
                double impulse = -correction / inverse;
                shift[c.a()] = shift[c.a()].add(c.normal().scale(impulse / a.properties().massKg()));
                turn[c.a()] = turn[c.a()].add(angularA.scale(impulse));
                if (b != null) {
                    shift[c.b()] = shift[c.b()].subtract(c.normal().scale(impulse / b.properties().massKg()));
                    turn[c.b()] = turn[c.b()].subtract(angularB.scale(impulse));
                }
                moved = true;
            }
            for (Joint j : joints) { moved |= separateJoint(current, j, shift, turn); }
            if (!moved) { break; }
            for (int k = 0; k < current.size(); k++) {
                RigidBodyState s = current.get(k).state();
                if (shift[k].lengthSquared() == 0 && turn[k].lengthSquared() == 0) { continue; }
                current.set(k, new Body(new RigidBodyState(s.position().add(shift[k]), s.velocity(),
                        Orientation.rotationVector(turn[k]).multiply(s.orientation()), s.angularMomentum()), current.get(k).properties()));
            }
        }
        return current.stream().map(Body::state).toList();
    }

    /** One joint's share of a separation iteration on top of what shift and turn hold already; whether it moved anything. */
    private static boolean separateJoint(List<Body> current, Joint j, Vec3d[] shift, Vec3d[] turn) {
        int l = j.leader(), f = j.follower();
        Body bl = current.get(l), bf = current.get(f);
        var c = j.coupling();
        var g = c.geometry(bl.state(), bl.properties(), bf.state(), bf.properties());
        MovingBody ml = new MovingBody(bl.state(), bl.properties()), mf = new MovingBody(bf.state(), bf.properties());
        boolean moved = false;
        Vec3d gap = g.gap().add(shift[f].add(turn[f].cross(g.followerOffset()))).subtract(shift[l].add(turn[l].cross(g.leaderOffset())));
        double length = gap.length();
        if (length > JOINT_SLOP) {
            double close = Math.min(JOINT_PROJECTION * (length - JOINT_SLOP), MAX_JOINT_PROJECTION);
            Vec3d impulse = times(invert(block(ml, g.leaderOffset(), mf, g.followerOffset())), gap.scale(-close / length));
            shift[f] = shift[f].add(impulse.scale(1 / bf.properties().massKg()));
            turn[f] = turn[f].add(mf.spin(g.followerOffset().cross(impulse)));
            shift[l] = shift[l].subtract(impulse.scale(1 / bl.properties().massKg()));
            turn[l] = turn[l].subtract(ml.spin(g.leaderOffset().cross(impulse)));
            moved = true;
        }
        boolean rigid = c.rollHeld() && c.rollStiffnessNmPerRad() == Double.POSITIVE_INFINITY;
        if (rigid || c.rollStopped() && !c.rollHeld()) {
            Vec3d n = g.yawAxis().cross(g.pitchAxis());
            if (n.length() > 1e-6) {
                Vec3d a = n.normalized();
                double roll = g.rollRad() + a.dot(turn[f].subtract(turn[l]));
                double excess = rigid ? roll : roll - Math.clamp(roll, -c.rollLimitRad(), c.rollLimitRad());
                if (Math.abs(excess) > ANGLE_SLOP) {
                    double inverse = a.dot(ml.spin(a)) + a.dot(mf.spin(a));
                    double impulse = -Math.clamp(JOINT_PROJECTION * excess, -MAX_JOINT_PROJECTION, MAX_JOINT_PROJECTION) / inverse;
                    turn[f] = turn[f].add(mf.spin(a).scale(impulse));
                    turn[l] = turn[l].subtract(ml.spin(a).scale(impulse));
                    moved = true;
                }
            }
        }
        return moved;
    }

    private static Vec3d relativeVelocity(MovingBody[] moving, PairContact c) {
        Vec3d v = moving[c.a()].velocityAt(c.offsetA());
        return c.b() < 0 ? v : v.subtract(moving[c.b()].velocityAt(c.offsetB()));
    }
    private static double inverseMass(MovingBody[] moving, PairContact c, Vec3d d) {
        double inverse = moving[c.a()].inverseMass(c.offsetA(), d);
        return c.b() < 0 ? inverse : inverse + moving[c.b()].inverseMass(c.offsetB(), d);
    }
    /** Impulse on a along +impulse, and the reaction on b. */
    private static void apply(MovingBody[] moving, PairContact c, Vec3d impulse) {
        moving[c.a()].applyImpulse(c.offsetA(), impulse);
        if (c.b() >= 0) { moving[c.b()].applyImpulse(c.offsetB(), impulse.scale(-1)); }
    }

    private static final class MovingBody {
        private final RigidBodyProperties properties;
        private final RigidBodyState initial;
        /** 0.30: the orientation's inverse and the angular velocity of the momentum as it is (null after an impulse), kept. */
        private final Orientation inverse;
        private Vec3d velocity, momentum, omega;
        private MovingBody(RigidBodyState s, RigidBodyProperties p) {
            properties = p; initial = s; velocity = s.velocity(); momentum = s.angularMomentum(); inverse = s.orientation().inverse();
        }
        private Vec3d omega() {
            if (omega == null) { omega = properties.inertia().solveWorld(initial.orientation(), inverse, momentum); }
            return omega;
        }
        private Vec3d velocityAt(Vec3d r) { return velocity.add(omega().cross(r)); }
        /** 1/m + (r x d) . I^-1 (r x d): inverse effective mass for a unit impulse along d at r. */
        private double inverseMass(Vec3d r, Vec3d d) {
            Vec3d rd = r.cross(d);
            return 1 / properties.massKg() + rd.dot(properties.inertia().solveWorld(initial.orientation(), inverse, rd));
        }
        private void applyImpulse(Vec3d r, Vec3d impulse) {
            velocity = velocity.add(impulse.scale(1 / properties.massKg()));
            momentum = momentum.add(r.cross(impulse));
            omega = null;
        }
        /** 0.36: an angular impulse (a couple times dt). */
        private void applyAngular(Vec3d impulse) {
            momentum = momentum.add(impulse);
            omega = null;
        }
        /** I^-1 v (world), at the solve's orientation. */
        private Vec3d spin(Vec3d v) { return properties.inertia().solveWorld(initial.orientation(), inverse, v); }
        /** The change of the velocity at r from a unit impulse along e there: e/m + (I^-1 (r x e)) x r. */
        private Vec3d pointResponse(Vec3d r, Vec3d e) { return e.scale(1 / properties.massKg()).add(spin(r.cross(e)).cross(r)); }
        private RigidBodyState state() {
            return new RigidBodyState(initial.position(), velocity, initial.orientation(), momentum);
        }
    }
}
