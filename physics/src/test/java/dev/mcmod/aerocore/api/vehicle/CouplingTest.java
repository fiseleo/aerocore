package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.Wrench;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/** 0.36: two bodies joined by a Coupling in ContactSolver.solveAll and separateAll. */
class CouplingTest {
    private static final double DT = 0.005, G = 9.80665;
    /** A tractor (8 t, 6 x 2 x 2.5 m) and a trailer (20 t, 13 x 2 x 2.5 m), each with its COM at its model origin. */
    private static final RigidBodyProperties TRACTOR = new RigidBodyProperties(8000, Vec3d.ZERO, InertiaTensor.box(8000, new Vec3d(6, 2, 2.5)));
    private static final RigidBodyProperties TRAILER = new RigidBodyProperties(20000, Vec3d.ZERO, InertiaTensor.box(20000, new Vec3d(13, 2, 2.5)));
    /** An anchor heavy enough to stand for the world. */
    private static final RigidBodyProperties ANCHOR = new RigidBodyProperties(1e12, Vec3d.ZERO, new InertiaTensor(1e13, 1e13, 1e13, 0, 0, 0));
    /** The fifth wheel 1 m ahead of the tractor's rear axle line, the kingpin 5.5 m ahead of the trailer's COM. */
    private static final Coupling FIFTH = Coupling.fifthWheel(new Vec3d(-1, 1.2, 0), new Vec3d(5.5, 0.5, 0), 0.2, 1.5);

    /** One step: loads as kicks, the joint solve, a torque-free drift, the joints' separation. */
    private static List<RigidBodyState> step(List<ContactSolver.Body> bodies, List<ContactSolver.Joint> joints, List<Wrench> loads,
                                             ContactSolver.Solution[] solved) {
        List<ContactSolver.Body> kicked = new ArrayList<>();
        for (int i = 0; i < bodies.size(); i++) {
            ContactSolver.Body b = bodies.get(i);
            kicked.add(new ContactSolver.Body(RigidBodyDynamics.kick(b.state(), b.properties(), loads.get(i), DT), b.properties()));
        }
        ContactSolver.Solution s = ContactSolver.solveAll(kicked, List.of(), joints, DT);
        if (solved != null) { solved[0] = s; }
        List<ContactSolver.Body> drifted = new ArrayList<>();
        for (int i = 0; i < bodies.size(); i++) {
            drifted.add(new ContactSolver.Body(RigidBodyDynamics.drift(s.bodies().get(i).state(), bodies.get(i).properties(), DT), bodies.get(i).properties()));
        }
        return ContactSolver.separateAll(drifted, b -> List.of(), joints);
    }
    private static List<ContactSolver.Body> with(List<ContactSolver.Body> bodies, List<RigidBodyState> states) {
        List<ContactSolver.Body> out = new ArrayList<>();
        for (int i = 0; i < bodies.size(); i++) { out.add(new ContactSolver.Body(states.get(i), bodies.get(i).properties())); }
        return out;
    }
    /** The follower placed so that its point is on the leader's (leader at the origin, both level). */
    private static Vec3d follower(Coupling c, Vec3d leaderAt) { return leaderAt.add(c.leaderPoint()).subtract(c.followerPoint()); }
    private static double energy(List<ContactSolver.Body> bodies) {
        double e = 0;
        for (ContactSolver.Body b : bodies) { e += b.state().kineticEnergy(b.properties()); }
        return e;
    }
    private static Vec3d momentum(List<ContactSolver.Body> bodies) {
        Vec3d p = Vec3d.ZERO;
        for (ContactSolver.Body b : bodies) { p = p.add(b.state().velocity().scale(b.properties().massKg())); }
        return p;
    }
    private static double gap(Coupling c, List<ContactSolver.Body> bodies) {
        return c.geometry(bodies.get(0).state(), bodies.get(0).properties(), bodies.get(1).state(), bodies.get(1).properties()).gap().length();
    }

    @Test void inZeroGravityMomentumIsKeptAndEnergyNeverRises() {
        List<ContactSolver.Body> bodies = List.of(
                new ContactSolver.Body(new RigidBodyState(Vec3d.ZERO, new Vec3d(3, 0.5, -0.2), Orientation.IDENTITY, new Vec3d(1000, 5000, -2000)), TRACTOR),
                new ContactSolver.Body(new RigidBodyState(follower(FIFTH, Vec3d.ZERO), new Vec3d(2, -0.3, 0.4), Orientation.IDENTITY, new Vec3d(-3000, 2000, 8000)), TRAILER));
        List<ContactSolver.Joint> joints = List.of(new ContactSolver.Joint(0, 1, FIFTH));
        List<Wrench> none = List.of(Wrench.ZERO, Wrench.ZERO);
        Vec3d p0 = momentum(bodies);
        // the first step takes up the velocities the joint does not allow (an inelastic catch)
        bodies = with(bodies, step(bodies, joints, none, null));
        double e1 = energy(bodies), most = e1, worstGap = 0;
        for (int i = 0; i < 2000; i++) {
            bodies = with(bodies, step(bodies, joints, none, null));
            most = Math.max(most, energy(bodies));
            if (i > 20) { worstGap = Math.max(worstGap, gap(FIFTH, bodies)); }
        }
        Vec3d p = momentum(bodies);
        assertEquals(0, p.subtract(p0).length() / p0.length(), 1e-9, "momentum " + p0 + " -> " + p);
        assertTrue(most <= e1 * 1.001, "energy " + e1 + " rose to " + most);
        assertTrue(worstGap < 0.001, "the points apart by " + worstGap + " m");
        var g = FIFTH.geometry(bodies.get(0).state(), TRACTOR, bodies.get(1).state(), TRAILER);
        assertTrue(Math.abs(g.rollRad()) < 0.002, "roll held: " + g.rollRad());
        assertTrue(Math.abs(g.pitchRad()) < 0.2 + 0.01, "pitch within its stop: " + g.pitchRad());
    }

    @Test void aBallMakesAPendulumOfTheRightPeriod() {
        // a 10 kg ball (r 0.1 m) hanging 2 m under a ball joint on an anchor, let go from 0.1 rad
        RigidBodyProperties bob = new RigidBodyProperties(10, Vec3d.ZERO, new InertiaTensor(0.04, 0.04, 0.04, 0, 0, 0));
        double length = 2, start = 0.1;
        Coupling ball = Coupling.ball(Vec3d.ZERO, new Vec3d(0, length, 0));
        List<ContactSolver.Body> bodies = List.of(
                new ContactSolver.Body(new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), ANCHOR),
                new ContactSolver.Body(new RigidBodyState(new Vec3d(length * Math.sin(start), -length * Math.cos(start), 0), Vec3d.ZERO,
                        Orientation.rotationVector(Vec3d.Z.scale(start)), Vec3d.ZERO), bob));
        List<ContactSolver.Joint> joints = List.of(new ContactSolver.Joint(0, 1, ball));
        List<Wrench> weight = List.of(Wrench.ZERO, new Wrench(Vec3d.Y.scale(-G * 10), Vec3d.ZERO));
        List<Double> crossings = new ArrayList<>();
        double lastX = bodies.get(1).state().position().x();
        for (int i = 0; i < (int) (30 / DT); i++) {
            bodies = with(bodies, step(bodies, joints, weight, null));
            double x = bodies.get(1).state().position().x();
            if (lastX < 0 && x >= 0) { crossings.add((i + lastX / (lastX - x)) * DT); }
            lastX = x;
        }
        double period = (crossings.getLast() - crossings.getFirst()) / (crossings.size() - 1);
        // a physical pendulum, with the first amplitude correction
        double expected = 2 * Math.PI * Math.sqrt((0.04 + 10 * length * length) / (10 * G * length)) * (1 + start * start / 16);
        assertEquals(expected, period, 0.01 * expected, "period over " + (crossings.size() - 1) + " swings");
        assertTrue(gap(ball, bodies) < 0.001, "on its string: " + gap(ball, bodies));
    }

    @Test void aFifthWheelCarriesTheTrailersRollButLetsItPitchToTheStop() {
        List<ContactSolver.Body> bodies = List.of(
                new ContactSolver.Body(new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), ANCHOR),
                new ContactSolver.Body(new RigidBodyState(follower(FIFTH, Vec3d.ZERO), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), TRAILER));
        List<ContactSolver.Joint> joints = List.of(new ContactSolver.Joint(0, 1, FIFTH));
        // a roll couple on the trailer: the joint takes it all, the trailer does not lean
        double torque = 50_000;
        List<Wrench> roll = List.of(Wrench.ZERO, new Wrench(Vec3d.ZERO, Vec3d.X.scale(torque)));
        ContactSolver.Solution[] solved = new ContactSolver.Solution[1];
        for (int i = 0; i < 200; i++) { bodies = with(bodies, step(bodies, joints, roll, solved)); }
        var g = FIFTH.geometry(bodies.get(0).state(), ANCHOR, bodies.get(1).state(), TRAILER);
        assertTrue(Math.abs(g.rollRad()) < 1e-3, "roll " + g.rollRad());
        ContactSolver.JointResult r = solved[0].joints().getFirst();
        // the couple and the point's force together hold the trailer: about the point they cancel the applied couple
        Vec3d held = r.coupleNm().add(g.followerOffset().cross(r.forceN()));
        assertEquals(-torque, held.x(), 0.05 * torque, "held " + held);
        // a pitch couple: the trailer turns about the hinge until the stop at 0.2 rad
        bodies = List.of(bodies.get(0), new ContactSolver.Body(new RigidBodyState(follower(FIFTH, Vec3d.ZERO), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), TRAILER));
        List<Wrench> pitch = List.of(Wrench.ZERO, new Wrench(Vec3d.ZERO, Vec3d.Z.scale(200_000)));
        double most = 0;
        for (int i = 0; i < 800; i++) {
            bodies = with(bodies, step(bodies, joints, pitch, null));
            most = Math.max(most, FIFTH.geometry(bodies.get(0).state(), ANCHOR, bodies.get(1).state(), TRAILER).pitchRad());
        }
        assertTrue(most > 0.19 && most < 0.2 + 0.01, "pitched to the stop: " + most);
        assertTrue(gap(FIFTH, bodies) < 0.001, "points together: " + gap(FIFTH, bodies));
    }

    @Test void aLightLeaderPullsAHeavyFollowerStiffly() {
        for (double ratio : new double[]{5, 10}) {
            RigidBodyProperties light = new RigidBodyProperties(20000 / ratio, Vec3d.ZERO, InertiaTensor.box(20000 / ratio, new Vec3d(6, 2, 2.5)));
            List<ContactSolver.Body> bodies = List.of(
                    new ContactSolver.Body(new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), light),
                    new ContactSolver.Body(new RigidBodyState(follower(FIFTH, Vec3d.ZERO), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), TRAILER));
            List<ContactSolver.Joint> joints = List.of(new ContactSolver.Joint(0, 1, FIFTH));
            // 50 kN at the leader's COM, forward and a little to the side
            List<Wrench> pull = List.of(new Wrench(new Vec3d(50_000, 0, 5_000), Vec3d.ZERO), Wrench.ZERO);
            double worst = 0;
            for (int i = 0; i < 1000; i++) {
                bodies = with(bodies, step(bodies, joints, pull, null));
                if (i > 20) { worst = Math.max(worst, gap(FIFTH, bodies)); }
            }
            assertTrue(worst < 0.001, "1:" + ratio + " apart by " + worst);
            // together the two take the force: the total momentum is the impulse
            Vec3d p = momentum(bodies);
            assertEquals(50_000 * 1000 * DT, p.x(), 1e-6 * 50_000 * 1000 * DT, "momentum " + p);
        }
    }

    @Test void withoutJointsTheSolveAndTheSeparationAreAsBefore() {
        Random random = new Random(36);
        RigidBodyProperties box = new RigidBodyProperties(10, Vec3d.ZERO, InertiaTensor.box(10, new Vec3d(1, 0.5, 1)));
        List<CollisionSphere> shape = new ArrayList<>();
        for (int i = 0; i < 8; i++) { shape.add(new CollisionSphere(new Vec3d((i & 1) - 0.5, ((i >> 1) & 1) * 0.5 - 0.25, ((i >> 2) & 1) - 0.5).scale(0.9), 0.05)); }
        List<WorldBox> floor = List.of(new WorldBox(new Vec3d(-100, -1, -100), new Vec3d(100, 0, 100)));
        for (int trial = 0; trial < 20; trial++) {
            List<ContactSolver.Body> bodies = new ArrayList<>();
            for (int k = 0; k < 3; k++) {
                bodies.add(new ContactSolver.Body(new RigidBodyState(new Vec3d(k * 0.9 + random.nextDouble() * 0.1, 0.2 + random.nextDouble() * 0.1, random.nextDouble() * 0.1),
                        new Vec3d(random.nextGaussian(), -1 - random.nextDouble(), random.nextGaussian()),
                        Orientation.rotationVector(new Vec3d(random.nextGaussian(), random.nextGaussian(), random.nextGaussian()).scale(0.1)),
                        new Vec3d(random.nextGaussian(), random.nextGaussian(), random.nextGaussian())), box));
            }
            List<ContactSolver.PairContact> contacts = new ArrayList<>();
            for (int k = 0; k < 3; k++) {
                contacts.addAll(ContactSolver.tag(k, ContactSolver.generate(bodies.get(k).state(), box, shape, floor, DT), ContactMaterial.STEEL_ON_STEEL));
                for (int j = k + 1; j < 3; j++) { contacts.addAll(ContactSolver.generatePair(k, bodies.get(k), shape, j, bodies.get(j), shape, ContactMaterial.STEEL_ON_STEEL, DT)); }
            }
            List<ContactSolver.BodyResult> old = ContactSolver.solveAll(bodies, contacts, DT);
            ContactSolver.Solution now = ContactSolver.solveAll(bodies, contacts, List.of(), DT);
            assertEquals(old, now.bodies());
            assertEquals(List.of(), now.joints());
            java.util.function.Function<List<ContactSolver.Body>, List<ContactSolver.PairContact>> regenerate = states -> {
                List<ContactSolver.PairContact> list = new ArrayList<>();
                for (int k = 0; k < 3; k++) for (int j = k + 1; j < 3; j++) {
                    list.addAll(ContactSolver.generatePair(k, states.get(k), shape, j, states.get(j), shape, ContactMaterial.STEEL_ON_STEEL, 0));
                }
                return list;
            };
            assertEquals(ContactSolver.separateAll(bodies, regenerate), ContactSolver.separateAll(bodies, regenerate, List.of()));
        }
    }

    @Test void aCouplingRoundTripsThroughJson() {
        Coupling rigid = FIFTH.withForwarded(List.of("brake", "lights"));
        assertEquals(rigid, Coupling.parse(rigid.write()));
        Coupling oscillating = FIFTH.withRoll(0.05, 2e5, 1e4).withCapture(0.2).withCollide(true);
        assertEquals(oscillating, Coupling.parse(oscillating.write()));
        Coupling ball = Coupling.ball(new Vec3d(-2.5, 0.5, 0), new Vec3d(4, 0.5, 0));
        assertEquals(ball, Coupling.parse(ball.write()));
        assertFalse(rigid.write().contains("rollStiffness"), "a rigid roll leaves its stiffness out");
        assertThrows(IllegalArgumentException.class, () -> FIFTH.withRoll(0.05, Double.POSITIVE_INFINITY, 0));
        assertThrows(IllegalArgumentException.class, () -> new Coupling(Coupling.Kind.BALL, Vec3d.ZERO, Vec3d.ZERO, Vec3d.Y, Vec3d.Y,
                1, 1, 1, 0, 0, 0.1, List.of(), false));
    }
}
