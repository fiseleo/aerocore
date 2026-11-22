package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** 0.43: a rope between two bodies (Coupling.Kind.ROPE in ContactSolver). */
class RopeTest {
    private static final double DT = 0.005, G = 9.80665;
    private static final RigidBodyProperties ANCHOR = new RigidBodyProperties(1e12, Vec3d.ZERO, new InertiaTensor(1e13, 1e13, 1e13, 0, 0, 0));
    /** A 1 t load, small for its rope (a 0.3 m ball's inertia). */
    private static final RigidBodyProperties LOAD = new RigidBodyProperties(1000, Vec3d.ZERO, new InertiaTensor(36, 36, 36, 0, 0, 0));

    private static List<ContactSolver.Body> step(List<ContactSolver.Body> bodies, List<ContactSolver.Joint> joints) {
        List<ContactSolver.Body> kicked = new ArrayList<>();
        for (ContactSolver.Body b : bodies) {
            Wrench w = b.properties() == ANCHOR ? Wrench.ZERO : new Wrench(Vec3d.Y.scale(-G * b.properties().massKg()), Vec3d.ZERO);
            kicked.add(new ContactSolver.Body(RigidBodyDynamics.kick(b.state(), b.properties(), w, DT), b.properties()));
        }
        ContactSolver.Solution s = ContactSolver.solveAll(kicked, List.of(), joints, DT);
        List<ContactSolver.Body> drifted = new ArrayList<>();
        for (int i = 0; i < bodies.size(); i++) {
            drifted.add(new ContactSolver.Body(RigidBodyDynamics.drift(s.bodies().get(i).state(), bodies.get(i).properties(), DT), bodies.get(i).properties()));
        }
        List<RigidBodyState> separated = ContactSolver.separateAll(drifted, b -> List.of(), joints);
        List<ContactSolver.Body> out = new ArrayList<>();
        for (int i = 0; i < bodies.size(); i++) { out.add(new ContactSolver.Body(separated.get(i), bodies.get(i).properties())); }
        return out;
    }

    private static List<ContactSolver.Body> hanging(Vec3d at) {
        return List.of(new ContactSolver.Body(new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), ANCHOR),
                new ContactSolver.Body(new RigidBodyState(at, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), LOAD));
    }

    @Test
    void aLoadSwingsOnItsRopeAsAPendulumAndHangsAtItsLength() {
        double length = 5, start = 0.15;
        Coupling rope = Coupling.rope(Vec3d.ZERO, Vec3d.ZERO, length, 1e8, 2e4);
        List<ContactSolver.Body> bodies = hanging(new Vec3d(length * Math.sin(start), -length * Math.cos(start), 0));
        List<ContactSolver.Joint> joints = List.of(new ContactSolver.Joint(0, 1, rope));
        List<Double> crossings = new ArrayList<>();
        double lastX = bodies.get(1).state().position().x(), longest = 0;
        for (int i = 0; i < (int) (40 / DT); i++) {
            bodies = step(bodies, joints);
            Vec3d p = bodies.get(1).state().position();
            if (lastX < 0 && p.x() >= 0) { crossings.add((i + lastX / (lastX - p.x())) * DT); }
            lastX = p.x();
            if (i > 400) { longest = Math.max(longest, p.length()); }
        }
        double period = (crossings.getLast() - crossings.getFirst()) / (crossings.size() - 1);
        double expected = 2 * Math.PI * Math.sqrt((36.0 + 1000 * length * length) / (1000 * G * length)) * (1 + start * start / 16);
        assertEquals(expected, period, 0.01 * expected, "a pendulum of the rope's length");
        // the rope stretches by m g L / E A (0.5 mm) and a little more from the swing: never a slack or a long rope
        assertEquals(length, longest, 0.01, "it hangs at the rope's length");
    }

    @Test
    void aSlackRopePullsNothing() {
        Coupling rope = Coupling.rope(Vec3d.ZERO, Vec3d.ZERO, 5, 1e8, 2e4);
        List<ContactSolver.Body> bodies = hanging(new Vec3d(0, -2, 0));
        List<ContactSolver.Joint> joints = List.of(new ContactSolver.Joint(0, 1, rope));
        for (int i = 0; i < 100; i++) { bodies = step(bodies, joints); }
        double t = 100 * DT, fall = 0.5 * G * t * t;
        // free fall (semi-implicit Euler drops g dt^2 / 2 more per step)
        assertEquals(-2 - fall, bodies.get(1).state().position().y(), G * DT * t, "the rope never pushes and pulls nothing while slack");
    }

    @Test
    void aWinchLiftsTheLoadAsItReelsIn() {
        double length = 6;
        Coupling rope = Coupling.rope(Vec3d.ZERO, Vec3d.ZERO, length, 1e8, 2e4);
        List<ContactSolver.Body> bodies = hanging(new Vec3d(0, -length, 0));
        for (int i = 0; i < 200; i++) { bodies = step(bodies, List.of(new ContactSolver.Joint(0, 1, rope))); }
        double reel = -0.5;
        for (int i = 0; i < 400; i++) {
            length += reel * DT;
            rope = rope.withRope(Vec3d.ZERO, length);
            bodies = step(bodies, List.of(new ContactSolver.Joint(0, 1, rope, reel)));
        }
        assertEquals(-5, bodies.get(1).state().position().y(), 0.02, "two seconds at 0.5 m/s take in a metre of the 6 m");
        assertTrue(Math.abs(bodies.get(1).state().velocity().y() - 0.5) < 0.05, "and it rises at the reel's speed");
    }

    @Test
    void aRopeRoundTripsThroughJson() {
        Coupling rope = Coupling.rope(new Vec3d(1, 2, 3), new Vec3d(0, 0.5, 0), 12.5, 3e7, 1e4);
        Coupling back = Coupling.parse(rope.write());
        assertEquals(rope, back);
        Coupling ball = Coupling.ball(Vec3d.ZERO, Vec3d.Y);
        assertTrue(!ball.write().contains("rope"), "only ropes write rope values");
        assertEquals(ball, Coupling.parse(ball.write()));
    }
}
