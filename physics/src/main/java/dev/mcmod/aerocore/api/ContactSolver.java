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
        for (CollisionSphere sphere : shape) {
            Vec3d r = s.worldOffset(p, sphere.centerLocal());
            Vec3d center = s.position().add(r);
            double margin = s.velocity().add(omega.cross(r)).length() * dt + SPECULATIVE_DISTANCE;
            double reach = sphere.radiusM() + margin;
            Vec3d extent = new Vec3d(reach, reach, reach);
            WorldBox bounds = new WorldBox(center.subtract(extent), center.add(extent));
            for (WorldBox box : obstacles) {
                if (!box.intersects(bounds)) { continue; }
                Contact contact = sphereBox(center, r, sphere.radiusM(), box, obstacles, margin);
                if (contact != null) { contacts.add(contact); }
            }
        }
        return contacts;
    }

    private static Contact sphereBox(Vec3d center, Vec3d r, double radius, WorldBox box, List<WorldBox> all, double margin) {
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
            if (insideOther(closest.add(normal.scale(1e-6)), box, all)) { return null; }
        } else {
            // Center inside: leave through the nearest face that is not shared with another obstacle,
            // so adjacent blocks do not push the body into each other.
            double[] exits = {box.max().x() - center.x(), center.x() - box.min().x(), box.max().y() - center.y(),
                    center.y() - box.min().y(), box.max().z() - center.z(), center.z() - box.min().z()};
            int best = -1, fallback = 0;
            for (int i = 0; i < 6; i++) {
                if (exits[i] < exits[fallback]) { fallback = i; }
                if (insideOther(center.add(FACES[i].scale(exits[i] + 1e-6)), box, all)) { continue; }
                if (best < 0 || exits[i] < exits[best]) { best = i; }
            }
            if (best < 0) { best = fallback; }
            normal = FACES[best];
            separation = -(exits[best] + radius);
        }
        return new Contact(r.subtract(normal.scale(radius)), normal, separation, box.material());
    }

    private static boolean insideOther(Vec3d point, WorldBox self, List<WorldBox> all) {
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
     */
    public record BodyResult(RigidBodyState state, Wrench wrench, List<AppliedForce> forces, double staticFrictionW, double pairFrictionW) {}

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
        for (CollisionSphere sa : shapeA) {
            Vec3d ra = bodyA.state().worldOffset(bodyA.properties(), sa.centerLocal()), ca = bodyA.state().position().add(ra);
            Vec3d va = bodyA.state().velocity().add(omegaA.cross(ra));
            double speedA = va.length();
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
        for (int k = 0; k < moving.length; k++) { forces.add(new ArrayList<>()); totals[k] = Wrench.ZERO; }
        for (int i = 0; i < count; i++) {
            PairContact c = contacts.get(i);
            Vec3d tangential = tangent1[i].scale(tangentImpulse1[i]).add(tangent2[i].scale(tangentImpulse2[i]));
            Vec3d impulse = c.normal().scale(normalImpulse[i]).add(tangential);
            if (impulse.lengthSquared() == 0) { continue; }
            AppliedForce onA = new AppliedForce(AppliedForce.Kind.CONTACT, c.offsetA(), impulse.scale(1 / dt));
            forces.get(c.a()).add(onA); totals[c.a()] = totals[c.a()].add(onA.wrench());
            Vec3d v = relativeVelocity(moving, c);
            double power = -tangential.dot(v.subtract(c.normal().scale(c.normal().dot(v)))) / dt;
            if (c.b() < 0) { staticFriction[c.a()] += power; continue; }
            AppliedForce onB = new AppliedForce(AppliedForce.Kind.CONTACT, c.offsetB(), impulse.scale(-1 / dt));
            forces.get(c.b()).add(onB); totals[c.b()] = totals[c.b()].add(onB.wrench());
            pairFriction[c.a()] += power / 2; pairFriction[c.b()] += power / 2;
        }
        List<BodyResult> results = new ArrayList<>();
        for (int k = 0; k < moving.length; k++) {
            results.add(new BodyResult(moving[k].state(), totals[k], List.copyOf(forces.get(k)), staticFriction[k], pairFriction[k]));
        }
        return results;
    }

    /**
     * Position projection for static and body-body penetration beyond SLOP, the multi-body form of separate().
     * contacts is regenerated by the caller at the current poses; only poses change.
     */
    public static List<RigidBodyState> separateAll(List<Body> bodies, java.util.function.Function<List<Body>, List<PairContact>> contacts) {
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
        private Vec3d velocity, momentum;
        private MovingBody(RigidBodyState s, RigidBodyProperties p) {
            properties = p; initial = s; velocity = s.velocity(); momentum = s.angularMomentum();
        }
        private Vec3d omega() { return properties.inertia().solveWorld(initial.orientation(), momentum); }
        private Vec3d velocityAt(Vec3d r) { return velocity.add(omega().cross(r)); }
        /** 1/m + (r x d) . I^-1 (r x d): inverse effective mass for a unit impulse along d at r. */
        private double inverseMass(Vec3d r, Vec3d d) {
            Vec3d rd = r.cross(d);
            return 1 / properties.massKg() + rd.dot(properties.inertia().solveWorld(initial.orientation(), rd));
        }
        private void applyImpulse(Vec3d r, Vec3d impulse) {
            velocity = velocity.add(impulse.scale(1 / properties.massKg()));
            momentum = momentum.add(r.cross(impulse));
        }
        private RigidBodyState state() {
            return new RigidBodyState(initial.position(), velocity, initial.orientation(), momentum);
        }
    }
}
