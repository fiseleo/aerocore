package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.AxlePart;
import dev.mcmod.aerocore.api.vehicle.part.FramePart;
import dev.mcmod.aerocore.api.vehicle.part.MassPart;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.Tyre;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A twisting frame (0.33, FrameFlex): rigid it is the old rigid body to the bit; a warp of the ground is shared by the
 * suspensions' roll stiffnesses and the frame's as series springs; with no frame stiffness the ends carry their own
 * masses' roll moments; stiff frames stay stable; three axles carry the weight; the step converges.
 */
class FrameFlexTest {
    static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    static final double G = TestTrack.G, BODY = 1200, AXLE = 100, FRONT = 1.1, REAR = -1.6, TRACK = 1.55, RATE = 30000;
    static final Tyre TYRE = TestParts.carTyre();
    static final Ground FLAT = Ground.flat(0, GroundContact.Surface.TARMAC);

    static double corner(boolean front) {
        double mass = BODY + 2 * AXLE, comX = (AXLE * FRONT + AXLE * REAR) / mass;
        double share = (comX - REAR) / (FRONT - REAR);
        return mass * G * (front ? share : 1 - share) / 2;
    }

    /** The body: a 1200 kg box at the origin, a twisting frame between the axles (null: rigid), or a plain mass. */
    static PartDesign body(FrameTwist frame, boolean plain) {
        MassProperties m = new MassProperties(BODY, Vec3d.ZERO, BODY * (1 + 1.7 * 1.7) / 12, BODY * (4.2 * 4.2 + 1.7 * 1.7) / 12, BODY * (4.2 * 4.2 + 1) / 12, 0, 0, 0);
        if (plain) { return new TestParts.Fixture("body", "plain body", d -> new MassPart(d, m, List.of())); }
        String key = frame == null ? "rigid" : java.util.Arrays.toString(frame.stationX()) + java.util.Arrays.deepToString(frame.twistNmPerRad());
        return new TestParts.Fixture("body", "frame " + key, d -> new FramePart(d, m, Map.of(), List.of(), frame, List.of()));
    }

    static VehicleBlueprint.Entry axle(String id, double x, boolean front, double antiRoll) {
        double load = corner(front), c = 2 * 0.3 * Math.sqrt(RATE * load / G);
        return new VehicleBlueprint.Entry(id, TestParts.axle(id, TRACK, RATE, load, c, c, Suspension.Type.TRAILING_ARM, true, null, null, antiRoll, 0),
                Mount.at(new Vec3d(x, TYRE.radiusM() - TYRE.deflectionM(load), 0)), List.of(Binding.of("brake", "brake")));
    }

    static Vehicle car(FrameTwist frame, boolean plain, double frontAntiRoll) {
        return new VehicleBlueprint("frame car", List.of(new VehicleBlueprint.Entry("body", body(frame, plain), Mount.at(new Vec3d(0, 0.6, 0))),
                axle("front", FRONT, true, frontAntiRoll), axle("rear", REAR, false, 0)), List.of()).compile();
    }

    /** Level ground with a block of a height under the front left wheel (x 0.6..3, z < 0). */
    static Ground block(double h) {
        return (from, dir, max) -> {
            double ground = from.x() > 0.6 && from.x() < 3 && from.z() < 0 ? h : 0;
            if (from.y() <= ground) { return new Ground.Hit(from, Vec3d.Y, 0, GroundContact.Surface.TARMAC); }
            if (dir.y() >= 0) { return null; }
            double t = (from.y() - ground) / -dir.y();
            return t > max ? null : new Ground.Hit(from.add(dir.scale(t)), Vec3d.Y, t, GroundContact.Surface.TARMAC);
        };
    }

    /** The corner loads {front left, front right, rear left, rear right} after settling (N). */
    static double[] loads(Vehicle v, Ground ground) {
        VehicleState st = TestTrack.settle(v, ground, AIR, Vec3d.ZERO, 0, Map.of("brake", 1.0), 3);
        List<TestTrack.Sample> s = TestTrack.drive(v, st, ground, AIR, (t, x) -> Map.of("brake", 1.0), 0.5, x -> false, Integer.MAX_VALUE);
        Map<String, Map<String, Double>> d = s.get(s.size() - 1).diagnostics();
        return new double[]{d.get("front").get("leftLoadN"), d.get("front").get("rightLoadN"), d.get("rear").get("leftLoadN"), d.get("rear").get("rightLoadN")};
    }

    /** A corner's rate: the wheel rate in series with the tyre at its load, N/m. */
    static double cornerRate(boolean front) {
        double kt = TYRE.verticalStiffness(TYRE.deflectionM(corner(front)));
        return RATE * kt / (RATE + kt);
    }

    @Test void aRigidFrameIsTheRigidBody() {
        Vehicle framed = car(null, false, 0), plain = car(null, true, 0);
        assertEquals(null, framed.flex());
        VehicleState a = TestTrack.settle(framed, block(0.05), AIR, Vec3d.ZERO, 0, Map.of(), 2), b = TestTrack.settle(plain, block(0.05), AIR, Vec3d.ZERO, 0, Map.of(), 2);
        TestTrack.drive(framed, a, block(0.05), AIR, (t, s) -> Map.of("brake", 0.3), 2, s -> false, Integer.MAX_VALUE);
        TestTrack.drive(plain, b, block(0.05), AIR, (t, s) -> Map.of("brake", 0.3), 2, s -> false, Integer.MAX_VALUE);
        assertEquals(0, a.body().position().subtract(b.body().position()).length(), 1e-9);
        assertEquals(0, a.body().angularMomentum().subtract(b.body().angularMomentum()).length(), 1e-9);
        // a very stiff frame comes to the same loads
        double[] rigid = loads(plain, block(0.02)), stiff = loads(car(FrameTwist.between(REAR, FRONT, 1e9), false, 0), block(0.02));
        for (int i = 0; i < 4; i++) { assertEquals(rigid[i], stiff[i], 1e-3 * rigid[i], "corner " + i); }
    }

    /**
     * The block under the front left wheel twists the ground between the axles by psi = h / t. Each end rolls by phi:
     * its suspension (corner rate, through the tyre) resists phi against the ground's, the frame K the difference of
     * the ends', and the weight overturns the body, which leans by the ends' mean (their mass shares w), by
     * m g h_cg phi_body shared by w (the roll centres are on the ground: trailing arms):
     * C_f (psi - phi_f) - K (phi_f - phi_r) + w_f G phi_b = 0 and -C_r phi_r + K (phi_f - phi_r) + w_r G phi_b = 0.
     */
    @Test void aWarpIsSharedLikeSeriesSprings() {
        double h = 0.02, psi = h / TRACK, cf = cornerRate(true) * TRACK * TRACK / 2, cr = cornerRate(false) * TRACK * TRACK / 2;
        for (double k : new double[]{Double.POSITIVE_INFINITY, 3e5, 3e4}) {
            Vehicle v = Double.isInfinite(k) ? car(null, true, 0) : car(FrameTwist.between(REAR, FRONT, k), false, 0);
            double big = (BODY + 2 * AXLE) * G * v.mass().centre().y();
            double wf = v.flex() == null ? 0.5 : v.flex().weights()[1], wr = 1 - wf;
            double phiF, phiR;
            if (Double.isInfinite(k)) {
                phiF = phiR = cf * psi / (cf + cr - big);
            } else {
                double a = cf + k - wf * wf * big, b = -k - wf * wr * big, c = -k - wr * wf * big, d = cr + k - wr * wr * big, det = a * d - b * c;
                phiF = d * cf * psi / det; phiR = -c * cf * psi / det;
            }
            double[] flat = loads(v, FLAT), warped = loads(v, block(h));
            // the warp's part of each axle's change: half the difference across it (the heave and pitch the block also
            // brings are the same on both sides)
            double front = ((warped[0] - flat[0]) - (warped[1] - flat[1])) / 2, rear = ((warped[3] - flat[3]) - (warped[2] - flat[2])) / 2;
            assertEquals(cf * (psi - phiF) / TRACK, front, 0.03 * cf * (psi - phiF) / TRACK, "front at " + k + " N m/rad");
            assertEquals(cr * phiR / TRACK, rear, 0.03 * cr * phiR / TRACK, "rear at " + k + " N m/rad");
            double sum = 0, weight = (BODY + 2 * AXLE) * G;
            for (double f : warped) { sum += f; }
            assertEquals(weight, sum, 0.005 * weight, "the wheels carry the weight");
        }
    }

    @Test void withoutFrameStiffnessEachEndCarriesItsOwnMass() {
        // on a cross slope the weight's sideways part rolls the car; a stiff anti-roll bar in front takes most of it on a
        // rigid body, but a frame that does not resist twisting lets each end carry the roll moment of its own mass
        double slope = Math.toRadians(4);
        Ground cross = Ground.plane(Vec3d.ZERO, new Vec3d(0, Math.cos(slope), Math.sin(slope)), GroundContact.Surface.TARMAC);
        double arb = 60000;
        Vehicle rigid = car(null, true, arb), soft = car(FrameTwist.between(REAR, FRONT, 1), false, arb);
        double[] r = loads(rigid, cross), s = loads(soft, cross);
        double frontShareRigid = transfer(r, true) / (transfer(r, true) + transfer(r, false));
        double frontShareSoft = transfer(s, true) / (transfer(s, true) + transfer(s, false));
        // the bar works with the spring (each wheel 2 x its rate against the other's opposite travel), both through the tyre
        double kt = TYRE.verticalStiffness(TYRE.deflectionM(corner(true))), roll = (RATE + 2 * arb) * kt / (RATE + 2 * arb + kt);
        double cf = roll * TRACK * TRACK / 2, cr = cornerRate(false) * TRACK * TRACK / 2;
        assertEquals(cf / (cf + cr), frontShareRigid, 0.03, "rigid: by the ends' roll stiffness");
        assertEquals(soft.flex().weights()[1], frontShareSoft, 0.02, "free to twist: by the ends' mass");
    }

    /** An axle's load transfer across it (N; the downhill wheel's load less the uphill's, halved). */
    static double transfer(double[] loads, boolean front) {
        int i = front ? 0 : 2;
        return Math.abs(loads[i] - loads[i + 1]) / 2;
    }

    @Test void stiffFramesStayStableOnRoughGround() {
        Random random = new Random(7);
        double[][] bumps = new double[24][4];
        for (double[] b : bumps) { b[0] = 0.5 + 3 * random.nextDouble(); b[1] = 2 * Math.PI * random.nextDouble(); b[2] = random.nextDouble() - 0.5; b[3] = 0.012 * random.nextDouble(); }
        Ground rough = (from, dir, max) -> {
            double ground = 0;
            for (double[] b : bumps) { ground += b[3] * Math.sin(2 * Math.PI * (from.x() + b[2] * from.z()) / b[0] + b[1]); }
            if (from.y() <= ground) { return new Ground.Hit(from, Vec3d.Y, 0, GroundContact.Surface.TARMAC); }
            if (dir.y() >= 0) { return null; }
            double t = (from.y() - ground) / -dir.y();
            return t > max ? null : new Ground.Hit(from.add(dir.scale(t)), Vec3d.Y, t, GroundContact.Surface.TARMAC);
        };
        for (double k : new double[]{1e5, 1e6, 1e7}) {
            Vehicle v = car(FrameTwist.between(REAR, FRONT, k), false, 20000);
            VehicleState st = TestTrack.settle(v, rough, AIR, Vec3d.ZERO, 0, Map.of(), 1);
            RigidBodyStateHelper.moving(st, 15);
            List<TestTrack.Sample> trace = TestTrack.drive(v, st, rough, AIR, (t, s) -> Map.of(), 20, s -> false, 50);
            for (TestTrack.Sample s : trace) {
                assertTrue(s.speedMps() < 20 && s.speedMps() > 5, k + ": speed " + s.speedMps() + " at " + s.timeS());
                Vec3d up = st.body().orientation().rotate(Vec3d.Y);
                assertTrue(up.y() > 0.95, k + ": upright");
            }
            int at = v.installed().get(v.flex().part()).firstInternal();
            assertTrue(Math.abs(st.internals[at + 1] - st.internals[at]) < 0.05, k + ": twist " + (st.internals[at + 1] - st.internals[at]));
        }
    }

    /** Sets a state moving along +x with its wheels rolling with it (no axle is driven here). */
    static final class RigidBodyStateHelper {
        static void moving(VehicleState st, double speed) {
            var b = st.body();
            st.setBody(new dev.mcmod.aerocore.api.RigidBodyState(b.position(), new Vec3d(speed, 0, 0), b.orientation(), Vec3d.ZERO));
            double re = TYRE.radiusM() - TYRE.deflectionM(4000) / 3;
            for (Vehicle.Installed i : st.vehicle().installed()) {
                if (!(i.part() instanceof AxlePart)) { continue; }
                for (int k = 0; k < 2; k++) {
                    Vehicle.Rotor r = st.vehicle().rotors().get(i.firstRotor() + k);
                    st.setTrainSpeed(r.train(), speed / re / r.ratio());
                }
            }
        }
    }

    @Test void threeAxlesCarryTheWeightOnAWarpedGround() {
        double mid = -0.25;
        List<VehicleBlueprint.Entry> parts = new ArrayList<>(List.of(new VehicleBlueprint.Entry("body",
                body(FrameTwist.chain(new double[]{REAR, mid, FRONT}, new double[]{5e4, 5e4}), false), Mount.at(new Vec3d(0, 0.6, 0))),
                axle("front", FRONT, true, 0), axle("middle", mid, false, 0), axle("rear", REAR, false, 0)));
        Vehicle v = new VehicleBlueprint("three", parts, List.of()).compile();
        VehicleState st = TestTrack.settle(v, block(0.03), AIR, Vec3d.ZERO, 0, Map.of("brake", 1.0), 3);
        List<TestTrack.Sample> s = TestTrack.drive(v, st, block(0.03), AIR, (t, x) -> Map.of("brake", 1.0), 0.5, x -> false, Integer.MAX_VALUE);
        Map<String, Map<String, Double>> d = s.get(s.size() - 1).diagnostics();
        double sum = 0;
        for (String axle : List.of("front", "middle", "rear")) { sum += d.get(axle).get("leftLoadN") + d.get(axle).get("rightLoadN"); }
        double weight = (BODY + 3 * AXLE) * G;
        assertEquals(weight, sum, 0.005 * weight);
        for (String axle : List.of("front", "middle", "rear")) { assertTrue(d.get(axle).get("leftLoadN") > 1000 && d.get(axle).get("rightLoadN") > 1000, axle + " on the ground"); }
    }

    @Test void theStepConverges() {
        double[] coarse = landing(TestTrack.DT), fine = landing(TestTrack.DT / 4);
        for (int i = 0; i < coarse.length; i++) { assertEquals(fine[i], coarse[i], 0.01 * Math.abs(fine[i]) + 20, "corner " + i); }
    }

    /** Dropped 3 cm onto the block, the corner loads 0.3 s later (N) with a frame of 1e5 N m/rad. */
    static double[] landing(double dt) {
        Vehicle v = car(FrameTwist.between(REAR, FRONT, 1e5), false, 0);
        Ground ground = block(0.04);
        VehicleEnvironment env = VehicleEnvironment.land(ground, AIR);
        VehicleState st = v.newState(new Vec3d(0, 0.03 + 0.04, 0), Orientation.IDENTITY, dt);
        Map<String, Map<String, Double>> last = null;
        int steps = (int) Math.round(0.3 / dt);
        for (int i = 0; i < steps; i++) {
            last = VehicleDynamics.step(v, st, env, Map.of("brake", 1.0), Vec3d.Y.scale(-G), List.of(), null, i * dt, dt).diagnostics();
        }
        return new double[]{last.get("front").get("leftLoadN"), last.get("front").get("rightLoadN"), last.get("rear").get("leftLoadN"), last.get("rear").get("rightLoadN")};
    }
}
