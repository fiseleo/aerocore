package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.BoxGround;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.AxlePart;
import dev.mcmod.aerocore.api.vehicle.part.LegPart;
import dev.mcmod.aerocore.api.wheel.Brake;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.Tyre;
import dev.mcmod.aerocore.api.wheel.WheelCorner;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Wheels on the ground (0.24): ride, braking, cornering, split friction, kerbs, slopes, brakes and a landing strut. */
class WheelTest {
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final double G = TestTrack.G;
    private static final Ground FLAT = Ground.flat(0, GroundContact.Surface.TARMAC);
    private static final Tyre TYRE = TestParts.carTyre();
    private static final double BODY = 1200, AXLE = 100, ENGINE = 10;

    /** A car's layout: axles at x = front and -rear (m), body centre at x = bodyX, 0.6 m up; vehicle origin on the ground. */
    private record Layout(double front, double rear, double bodyX, double track) {
        double mass() { return BODY + 2 * AXLE + ENGINE; }
        double comX() { return (BODY * bodyX + AXLE * front - AXLE * rear + ENGINE * front) / mass(); }
        /** Static load on one front (true) or rear wheel, N. */
        double corner(boolean front) {
            double frontShare = (comX() + rear) / (front() + rear);
            return mass() * G * (front ? frontShare : 1 - frontShare) / 2;
        }
        double hubHeight(boolean front) { return TYRE.radiusM() - TYRE.deflectionM(corner(front)); }
    }

    private static final Layout CAR = new Layout(1.1, 1.6, 0, 1.55);

    /** A car on the layout: rear axle driven (final 3.5) by a 0.15 kg m^2 "engine" on "torque", front steered on "steer" (+-0.6 rad), brakes on "brake". */
    private static Vehicle car(String name, Layout l, double rideHz, double zeta, Suspension.Type type, AxlePart.Drive drive, double antiRoll) {
        return car(name, l, rideHz, zeta, type, drive, antiRoll, 0);
    }

    /** The same with a share of Ackermann steering. */
    private static Vehicle car(String name, Layout l, double rideHz, double zeta, Suspension.Type type, AxlePart.Drive drive, double antiRoll, double ackermann) {
        return car(name, l, rideHz, zeta, type, drive, antiRoll, ackermann, null);
    }

    /** 0.33: the same with the front axle on hardpoints (null: straight up). */
    private static Vehicle car(String name, Layout l, double rideHz, double zeta, Suspension.Type type, AxlePart.Drive drive, double antiRoll, double ackermann,
                               dev.mcmod.aerocore.api.wheel.kinematics.Hardpoints frontHardpoints) {
        double front = l.corner(true), rear = l.corner(false);
        // wheel rate for the ride frequency in series with the tyre, and damping as a share of critical at the wheel
        double kf = rate(front, rideHz), kr = rate(rear, rideHz);
        double cf = 2 * zeta * Math.sqrt(kf * front / G), cr = 2 * zeta * Math.sqrt(kr * rear / G);
        List<VehicleBlueprint.Entry> parts = new ArrayList<>(List.of(
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4.2, 1.0, 1.7), 0, false, 1, MassSpec.given(BODY)), Mount.at(new Vec3d(l.bodyX(), 0.6, 0))),
                new VehicleBlueprint.Entry("front", TestParts.axle("f", l.track(), kf, front, cf, cf, type, true, null, new AxlePart.Steering(0.6, ackermann, l.front() + l.rear()),
                        antiRoll, 0, frontHardpoints), Mount.at(new Vec3d(l.front(), l.hubHeight(true), 0)), List.of(Binding.of("steer", "steer"), Binding.of("brake", "brake"))),
                new VehicleBlueprint.Entry("rear", TestParts.axle("r", l.track(), kr, rear, cr, cr, type, true, drive, null, antiRoll, 0),
                        Mount.at(new Vec3d(-l.rear(), l.hubHeight(false), 0)), List.of(Binding.of("brake", "brake"))),
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.15, true), Mount.at(new Vec3d(l.front(), 0.45, 0)), List.of(Binding.of("torque", "torque")))));
        List<Connection> shafts = drive == null ? List.of() : List.of(Connection.Shaft.ports("engine", null, "rear", "input", 1, 1));
        return new VehicleBlueprint(name, parts, shafts).compile();
    }

    private static AxlePart.Drive drive(AxlePart.Differential d) { return new AxlePart.Drive(3.5, 1, 0.05, d, 30, 0.3, 3, 50, false, false); }

    /** The wheel rate that, in series with the tyre, gives a corner of a load this ride frequency, N/m. */
    private static double rate(double load, double hz) {
        double ride = load / G * Math.pow(2 * Math.PI * hz, 2), kt = TYRE.verticalStiffness(TYRE.deflectionM(load));
        return ride * kt / (kt - ride);
    }

    private static VehicleState parked(Vehicle v, Ground ground, Map<String, Double> controls) {
        return TestTrack.settle(v, ground, AIR, Vec3d.ZERO, 0, controls, 2);
    }

    private static void moving(VehicleState st, double speed) {
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(speed, 0, 0), b.orientation(), Vec3d.ZERO));
        // wheels rolling with it
        for (Vehicle.Installed i : st.vehicle().installed()) {
            if (i.part() instanceof AxlePart a) {
                int first = i.firstRotor() + (a.drive() == null ? 0 : 1);
                double re = TYRE.radiusM() - TYRE.deflectionM(4000) / 3;
                for (int k = 0; k < 2; k++) {
                    Vehicle.Rotor r = st.vehicle().rotors().get(first + k);
                    st.setTrainSpeed(r.train(), speed / re / r.ratio());
                }
                if (a.drive() != null) {
                    Vehicle.Rotor input = st.vehicle().rotors().get(i.firstRotor());
                    st.setTrainSpeed(input.train(), a.drive().finalRatio() * speed / re / input.ratio());
                }
            }
        }
    }

    @Test void aCarRidesAtItsFrequencyAndDamping() {
        // symmetric, so heave is its own mode: m s^3 c + m (k_t + k_w) s^2 + k_t c s + k_t k_w = 0 per corner
        Layout l = new Layout(1.35, 1.35, 0, 1.55);
        double hz = 1.3, zeta = 0.3;
        Vehicle v = car("ride", l, hz, zeta, Suspension.Type.TRAILING_ARM, null, 0);
        VehicleState st = parked(v, FLAT, Map.of());
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(0, -0.25, 0), b.orientation(), Vec3d.ZERO));
        List<TestTrack.Sample> trace = TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of(), 4, s -> false, 1);
        List<double[]> peaks = new ArrayList<>();
        for (int i = 1; i < trace.size() - 1; i++) {
            double y0 = trace.get(i - 1).origin().y(), y1 = trace.get(i).origin().y(), y2 = trace.get(i + 1).origin().y();
            if (y1 < y0 && y1 <= y2) { peaks.add(new double[]{trace.get(i).timeS(), y1}); }
        }
        assertTrue(peaks.size() >= 2, "oscillates: " + peaks.size());
        double settled = trace.get(trace.size() - 1).origin().y();
        double period = peaks.get(1)[0] - peaks.get(0)[0];
        double a0 = settled - peaks.get(0)[1], a1 = settled - peaks.get(1)[1];
        double decrement = Math.log(a0 / a1), measured = decrement / Math.sqrt(4 * Math.PI * Math.PI + decrement * decrement);
        // the linear model's root
        double m = l.mass() / 4, load = l.corner(true), kt = TYRE.verticalStiffness(TYRE.deflectionM(load)), kw = rate(load, hz);
        double c = 2 * zeta * Math.sqrt(kw * load / G);
        double[] root = cubicRoot(m * c, m * (kt + kw), kt * c, kt * kw);
        double damped = root[1], expectedZeta = -root[0] / Math.hypot(root[0], root[1]);
        assertEquals(2 * Math.PI / damped, period, 0.05 * 2 * Math.PI / damped, "damped period");
        assertEquals(expectedZeta, measured, 0.12 * expectedZeta, "damping ratio");
    }

    /** The complex root (re, im > 0) of a s^3 + b s^2 + c s + d nearest the ride mode (Newton from the undamped guess). */
    private static double[] cubicRoot(double a, double b, double c, double d) {
        double re = 0, im = Math.sqrt(d / b);
        for (int i = 0; i < 100; i++) {
            // p(s) and p'(s) for complex s
            double s2r = re * re - im * im, s2i = 2 * re * im, s3r = s2r * re - s2i * im, s3i = s2r * im + s2i * re;
            double pr = a * s3r + b * s2r + c * re + d, pi = a * s3i + b * s2i + c * im;
            double dr = 3 * a * s2r + 2 * b * re + c, di = 3 * a * s2i + 2 * b * im;
            double den = dr * dr + di * di;
            re -= (pr * dr + pi * di) / den;
            im -= (pi * dr - pr * di) / den;
        }
        return new double[]{re, im};
    }

    @Test void lockedWheelsStopInVSquaredOverTwoGMu() {
        Vehicle v = car("brake", CAR, 1.3, 0.3, Suspension.Type.DOUBLE_WISHBONE, null, 0);
        VehicleState st = parked(v, FLAT, Map.of());
        moving(st, 20);
        double x0 = st.origin().x();
        List<TestTrack.Sample> trace = TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 1.0), 10, s -> s.body().velocity().length() < 0.02, 1);
        double distance = st.origin().x() - x0;
        // the tyre's locked friction at the static wheel loads, averaged by load
        double f = CAR.corner(true), r = CAR.corner(false);
        double lockF = -TYRE.forces(GroundContact.Surface.TARMAC, f, -1, 0, 0, 10).longitudinalN(), lockR = -TYRE.forces(GroundContact.Surface.TARMAC, r, -1, 0, 0, 10).longitudinalN();
        double mu = (lockF + lockR) / (f + r);
        assertEquals(400 / (2 * G * mu), distance, 0.08 * 400 / (2 * G * mu), "stopping distance at mu " + mu);
        // and then it stays: the pitch settles (1.3 Hz, a third of critical) and nothing creeps or jitters
        TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 1.0), 4, s -> false, Integer.MAX_VALUE);
        Vec3d stopped = st.origin();
        List<TestTrack.Sample> still = TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 1.0), 4, s -> false, 10);
        assertTrue(st.origin().subtract(stopped).length() < 1e-3, "crept " + st.origin().subtract(stopped).length());
        for (TestTrack.Sample s : still) { assertTrue(s.speedMps() < 1e-3, "still: " + s.speedMps()); }
        assertTrue(trace.size() > 10);
    }

    @Test void steadyCorneringMatchesTheBicycleModelsUndersteer() {
        // full Ackermann, so the front tyres share one slip angle as the single-track model's one front tyre does (parallel
        // steer gives the inner and outer wheels slip angles 0.08 degrees apart here, on unequal loads)
        Vehicle v = car("corner", CAR, 1.3, 0.3, Suspension.Type.TRAILING_ARM, drive(AxlePart.Differential.OPEN), 0, 1);
        double radius = 50, speed = 7, wheelbase = CAR.front() + CAR.rear();
        VehicleState st = parked(v, FLAT, Map.of());
        moving(st, speed);
        double[] steer = {wheelbase / radius / 0.6};
        List<TestTrack.Sample> trace = TestTrack.drive(v, st, FLAT, AIR, (t, s) -> {
            double u = s.body().velocity().length(), yaw = s.body().angularVelocity(v.properties()).y();
            steer[0] += 0.5 * (speed / radius - yaw) * TestTrack.DT;
            return Map.of("steer", steer[0], "torque", 400 * (speed - u) + 20);
        }, 25, s -> false, 50);
        TestTrack.Sample last = trace.get(trace.size() - 1);
        double u = last.speedMps(), yaw = st.body().angularVelocity(v.properties()).y(), lateral = u * yaw, path = u / yaw;
        assertEquals(speed, u, 0.02 * speed, "held speed");
        double[] angles = ((AxlePart) v.installed("front").part()).steerAngles(steer[0]);
        double measured = ((angles[0] + angles[1]) / 2 - wheelbase / path) / lateral;
        // single-track model at a tenth of g: delta = L / R + (alpha_f - alpha_r) with each axle's force F = C alpha; camber
        // leans both axles alike (trailing arms), adding the same slip to both, and drops out
        Map<String, Double> fd = last.diagnostics().get("front"), rd = last.diagnostics().get("rear");
        double slipF = -Math.toRadians(fd.get("leftSlipAngleDeg") + fd.get("rightSlipAngleDeg")) / 2;
        double slipR = -Math.toRadians(rd.get("leftSlipAngleDeg") + rd.get("rightSlipAngleDeg")) / 2;
        assertEquals((slipF - slipR) / lateral, measured, 0.05 * measured, "kinematics: steer beyond Ackermann is the axles' slip difference");
        double fl = fd.get("leftLoadN"), fr = fd.get("rightLoadN"), rl = rd.get("leftLoadN"), rr = rd.get("rightLoadN");
        double sideF = fd.get("leftSideForceN") + fd.get("rightSideForceN"), sideR = rd.get("leftSideForceN") + rd.get("rightSideForceN");
        double cf = TYRE.corneringStiffness(fl) + TYRE.corneringStiffness(fr), cr = TYRE.corneringStiffness(rl) + TYRE.corneringStiffness(rr);
        assertEquals((sideF / cf - sideR / cr) / lateral, measured, 0.05 * measured, "tyres: each axle's slip is its force over its stiffness");
        // from the design alone (static loads, forces split by the moment balance with the pneumatic trail) within 15 %: the
        // gradient is the small difference of two slips (each about 6.5 times it), so the 1 % the split moves by load
        // transfer and the outer wheels' larger rolling resistance across the track shows ten times over
        double f = CAR.corner(true), r = CAR.corner(false), a = CAR.front() - CAR.comX(), b = CAR.rear() + CAR.comX(), m = CAR.mass();
        double tf = TYRE.contactLengthM(f) / 6, tr = TYRE.contactLengthM(r) / 6;
        double front = m * (b + tr) / (a - tf + b + tr), rear = m - front;
        double expected = front / (2 * TYRE.corneringStiffness(f)) - rear / (2 * TYRE.corneringStiffness(r));
        assertEquals(expected, measured, 0.15 * expected, "understeer gradient (rad per m/s^2) from the design");
        assertTrue(measured > 0, "understeers with 59 % on the front");
    }

    @Test void splitFrictionTractionFollowsTheDifferential() {
        Ground split = Ground.split(0, GroundContact.Surface.ICE, GroundContact.Surface.TARMAC);
        double[] accel = new double[3], bias = new double[3];
        AxlePart.Differential[] kinds = {AxlePart.Differential.OPEN, AxlePart.Differential.TORSEN, AxlePart.Differential.SPOOL};
        for (int k = 0; k < 3; k++) {
            Vehicle v = car("split " + kinds[k], CAR, 1.3, 0.3, Suspension.Type.TRAILING_ARM, drive(kinds[k]), 0);
            VehicleState st = parked(v, split, Map.of());
            // the engine governed at 150 rad/s with at most 200 N m, so a spinning wheel turns steadily (its torque is its friction alone)
            int engine = v.installed("engine").firstRotor();
            List<TestTrack.Sample> trace = TestTrack.drive(v, st, split, AIR,
                    (t, s) -> Map.of("torque", Math.clamp(3 * (150 - s.rotorSpeed(engine)), 0, 200)), 2, s -> false, 10);
            TestTrack.Sample a = trace.get(trace.size() / 2), b = trace.get(trace.size() - 1);
            accel[k] = (b.velocity().x() - a.velocity().x()) / (b.timeS() - a.timeS());
            Map<String, Double> rear = b.diagnostics().get("rear");
            bias[k] = rear.get("rightForceN") / rear.get("leftForceN");
        }
        // the gripping (tarmac) wheel's push over the icy wheel's: the same torque through an open differential, up to the
        // bias ratio through a Torsen (3), and all the engine gives through a spool; the car goes faster in that order
        assertEquals(1, bias[0], 0.15, "open");
        assertEquals(3, bias[1], 0.15 * 3, "Torsen");
        assertTrue(bias[2] > 4, "spool " + bias[2]);
        assertTrue(accel[0] < accel[1] && accel[1] < accel[2], "open " + accel[0] + " < Torsen " + accel[1] + " < spool " + accel[2]);
        double iceLock = -TYRE.forces(GroundContact.Surface.ICE, CAR.corner(false), -1, 0, 0, 5).longitudinalN();
        assertEquals(2 * iceLock, accel[0] * CAR.mass(), 0.3 * 2 * iceLock, "open: about twice the spinning icy wheel's sliding force");
    }

    @Test void aKerbWithinTheRadiusIsClimbedAndAStepAboveItStops() {
        Vehicle v = car("kerb", CAR, 1.3, 0.3, Suspension.Type.DOUBLE_WISHBONE, drive(AxlePart.Differential.OPEN), 0);
        for (double height : new double[]{0.12, 0.45}) {
            Ground track = Ground.nearest(FLAT, Ground.boxes(List.of(new BoxGround.Box(new Vec3d(4, 0, -3), new Vec3d(30, height, 3), GroundContact.Surface.TARMAC))));
            VehicleState st = parked(v, track, Map.of());
            moving(st, 4);
            TestTrack.drive(v, st, track, AIR, (t, s) -> Map.of("torque", 60.0), 8, s -> false, 100);
            double x = st.origin().x();
            if (height < TYRE.radiusM()) { assertTrue(x > 8, "climbed the " + height + " m kerb: x " + x); }
            else { assertTrue(x < 4 + CAR.front(), "stopped at the " + height + " m step: x " + x); }
        }
    }

    @Test void aBrakedCarHoldsOnAThirtyPercentSlope() {
        double grade = Math.atan(0.3);
        Ground hill = Ground.plane(Vec3d.ZERO, new Vec3d(-Math.sin(grade), Math.cos(grade), 0), GroundContact.Surface.TARMAC);
        Vehicle v = car("slope", CAR, 1.3, 0.3, Suspension.Type.DOUBLE_WISHBONE, null, 0);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.rotationVector(Vec3d.Z.scale(grade)), TestTrack.DT);
        TestTrack.drive(v, st, hill, AIR, (t, s) -> Map.of("brake", 1.0), 3, s -> false, Integer.MAX_VALUE);
        Vec3d start = st.origin();
        List<TestTrack.Sample> trace = TestTrack.drive(v, st, hill, AIR, (t, s) -> Map.of("brake", 1.0), 30, s -> false, 20);
        assertTrue(st.origin().subtract(start).length() < 0.01, "crept " + st.origin().subtract(start).length() + " m in 30 s");
        for (TestTrack.Sample s : trace.subList(trace.size() / 2, trace.size())) { assertTrue(s.speedMps() < 1e-3, "no jitter: " + s.speedMps()); }
    }

    /** The deepest nose-down pitch braking at half the pedal from 20 m/s for a second, rad. */
    private static double dive(Vehicle v) {
        VehicleState st = parked(v, FLAT, Map.of());
        moving(st, 20);
        double start = Math.asin(st.body().orientation().rotate(Vec3d.X).y()), most = 0;
        for (int i = 0; i < 50; i++) {
            TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 0.5), 0.02, s -> false, Integer.MAX_VALUE);
            most = Math.min(most, Math.asin(st.body().orientation().rotate(Vec3d.X).y()) - start);
        }
        return -most;
    }

    @Test void hardpointsBehaveAsTheSimpleGeometryAndPitchedPivotsHoldTheNoseUp() {
        Vec3d hub = new Vec3d(0, 0, CAR.track() / 2);
        var simple = dev.mcmod.aerocore.api.wheel.kinematics.Hardpoints.fromSimple(Suspension.Type.DOUBLE_WISHBONE, CAR.track(), hub, TYRE.radiusM(), 0, -1, 0);
        var d = (dev.mcmod.aerocore.api.wheel.kinematics.Hardpoints.DoubleWishbone) simple;
        // both arms' pivot axes pitched nose down 12 degrees: the contact patch moves forward as the wheel rises
        double beta = Math.toRadians(12);
        Vec3d[] lower = pitched(d.lowerFront(), d.lowerRear(), beta), upper = pitched(d.upperFront(), d.upperRear(), beta);
        var anti = new dev.mcmod.aerocore.api.wheel.kinematics.Hardpoints.DoubleWishbone(d.hub(), lower[0], lower[1], d.lowerBall(), upper[0], upper[1], d.upperBall(),
                d.tieInner(), d.tieOuter(), d.springBody(), d.springArm());
        double plain = dive(car("plain", CAR, 1.3, 0.3, Suspension.Type.DOUBLE_WISHBONE, null, 0, 0));
        double table = dive(car("table", CAR, 1.3, 0.3, Suspension.Type.DOUBLE_WISHBONE, null, 0, 0, simple));
        double held = dive(car("anti", CAR, 1.3, 0.3, Suspension.Type.DOUBLE_WISHBONE, null, 0, 0, anti));
        assertTrue(plain > 0.005, "it dives: " + Math.toDegrees(plain));
        assertEquals(plain, table, 0.1 * plain, "hardpoints made from the simple geometry dive alike");
        assertTrue(held < 0.85 * table, "anti-dive: " + Math.toDegrees(held) + " against " + Math.toDegrees(table) + " degrees");
    }

    private static Vec3d[] pitched(Vec3d front, Vec3d rear, double beta) {
        Vec3d mid = front.add(rear).scale(0.5);
        Vec3d u = new Vec3d(Math.cos(beta), -Math.sin(beta), 0).scale(front.subtract(rear).length() / 2);
        return new Vec3d[]{mid.add(u), mid.subtract(u)};
    }

    @Test void ackermannSteeringMeetsAtTheRearAxle() {
        AxlePart front = (AxlePart) car("ack", CAR, 1.3, 0.3, Suspension.Type.MACPHERSON, null, 0).installed("front").part();
        double l = 2.7, t = 1.55;
        AxlePart full = new AxlePart(front.design(), front.left(), front.right(), null, new AxlePart.Steering(0.6, 1, l), 0, 0, null, front.mass(), List.of());
        double[] a = full.steerAngles(0.5);
        assertTrue(a[0] > a[1] && a[1] > 0, "turning left the left wheel turns more");
        assertEquals(t / l, 1 / Math.tan(a[1]) - 1 / Math.tan(a[0]), 1e-9, "cot outer - cot inner = track / wheelbase");
        // 0.33: a rear axle 1.1 m behind the zero-steer line, counter-steering (right, d < 0) while the car turns left:
        // the turn centre is still to the left, so its left wheel is the inner one and turns more
        AxlePart rear = new AxlePart(front.design(), front.left(), front.right(), null, new AxlePart.Steering(0.1, 1, -1.1), 0, 0, null, front.mass(), List.of());
        double[] b = rear.steerAngles(-0.5);
        assertTrue(b[0] < b[1] && b[1] < 0, "counter-steering, the left (inner) wheel turns more: " + b[0] + ", " + b[1]);
        assertEquals(t / 1.1, 1 / Math.tan(-b[1]) - 1 / Math.tan(-b[0]), 1e-9, "cot outer - cot inner = track / distance");
        // the same axle steering with the front (in phase) turns about a centre to the right of it
        double[] c = rear.steerAngles(0.5);
        assertTrue(c[1] > c[0] && c[0] > 0, "in phase, the right wheel is the inner one: " + c[0] + ", " + c[1]);
        // the front axle is unchanged for a positive distance; 0 is parallel
        AxlePart parallel = new AxlePart(front.design(), front.left(), front.right(), null, new AxlePart.Steering(0.6, 1, 0), 0, 0, null, front.mass(), List.of());
        assertEquals(parallel.steerAngles(0.5)[0], parallel.steerAngles(0.5)[1], 0);
    }

    @Test void brakesHaveLimpertsCoolingDarcomsFactorsAndFade() {
        Brake disc = new Brake(Brake.Type.DISC, 1000, 10, 0.381, 0.2);
        // Limpert: 381 mm solid disc at 97 km/h, 113.6 W/m^2 K
        assertEquals(113.6, disc.convection(97 / 3.6), 0.12 * 113.6);
        // DARCOM table 2-1 at mu 0.35
        assertEquals(0.7, Brake.Type.DISC.factor(0.35), 1e-12);
        assertTrue(Brake.Type.DRUM_LEADING_TRAILING.factor(0.35) >= 2.0 && Brake.Type.DRUM_LEADING_TRAILING.factor(0.35) <= 2.8);
        assertTrue(Brake.Type.DRUM_TWO_LEADING.factor(0.35) >= 2.5 && Brake.Type.DRUM_TWO_LEADING.factor(0.35) <= 3.5);
        assertTrue(Brake.Type.DRUM_DUO_SERVO.factor(0.35) >= 3.0 && Brake.Type.DRUM_DUO_SERVO.factor(0.35) <= 7.0);
        // fade: a hot disc gives less; a drum's torque falls more than a disc's for the same lining
        double hot = 550 + 273.15;
        assertTrue(disc.torqueNm(1, hot) < 0.8 * disc.torqueNm(1, 300));
        Brake drum = new Brake(Brake.Type.DRUM_DUO_SERVO, 1000, 10, 0.28, 0.15);
        assertTrue(drum.torqueNm(1, hot) / drum.torqueNm(1, 300) < disc.torqueNm(1, hot) / disc.torqueNm(1, 300));
        // a stop's energy heats the lump by 0.9 E / (m c) when there is no time to cool
        double e = 0.5 * 1500 * 30 * 30 * 0.3, t = disc.temperatureK(300, e / 0.01, 0, 300, 0.01);
        assertEquals(0.9 * e / (10 * 419), t - 300, 0.01 * 0.9 * e / (10 * 419));
    }

    /**
     * An orifice's force is K v |v| (Milwitzky and Cook): a strut with no spring pushed at a steady rate through a tyre
     * ends up carrying exactly that, at the game's step and at a fine one (0.24.0 first took the coefficient once at the
     * ground's rate, as a tangent: about a fifth soft in a landing).
     */
    @Test void anOrificeDampsWithItsSquareLaw() {
        Suspension.Damper d = Suspension.Damper.orifice(0.0024, 6.4e-5, 850, 4, 100);
        Suspension strut = new Suspension(new Suspension.Geometry(0, 0, false), Suspension.Spring.coil(0, 0), d, 2.0, 0, 0);
        for (double dt : new double[]{TestTrack.DT, 2e-4}) {
            for (double rate : new double[]{0.5, 2.0}) {
                double c = 0, own = 0, force = 0;
                for (int i = 0; i * dt < 0.4; i++) {
                    double p = rate * i * dt;
                    Suspension.Step s = strut.step(TYRE, p, rate, c, own, 0, dt);
                    double next = s.compression(rate, dt);
                    own = (next - c) / dt;
                    c = next;
                    force = s.forceN() + s.slopeNsPerM() * rate;
                }
                double expected = d.orificeNs2PerM2() * rate * rate + 100 * rate;
                assertEquals(expected, force, 0.03 * expected, "rate " + rate + " m/s, step " + dt + " s");
            }
        }
    }

    @Test void anOleoStrutTakesAFar23LandingWithoutBottoming() {
        // a main leg carrying 5 kN: 0.15 m stroke, gas 4:1 static over extended (Currey's typical), orifice for about 2 g, and
        // a linear part (seal friction and the metering pin's bleed) of about a sixth of critical at the static position
        double load = 5000, stroke = 0.3, area = 0.0042, pStatic = load / area + 101325, staticStroke = 0.84 * stroke;
        double extended = pStatic / 4, volume = area * staticStroke / (1 - Math.pow(0.25, 1 / Suspension.POLYTROPIC));
        Suspension strut = new Suspension(new Suspension.Geometry(0, 0, false), Suspension.Spring.oleo(area, volume, extended),
                Suspension.Damper.orifice(area, 1.45e-4, 850, 3, 1800), stroke, 0, staticStroke);
        Tyre tyre = new Tyre(new Tyre.Spec(Tyre.Construction.AIRCRAFT, 0.16, 0.91, 0.1524, 290e3, 290e3, Tyre.Tread.AIRCRAFT_RIB, 1));
        WheelCorner wheel = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, 0, 0), 0, 0, tyre, 0.05, strut, null, 0));
        PartDesign leg = new TestParts.Fixture("leg", "oleo leg", d -> new LegPart(d, wheel, LegPart.Steer.FIXED, 0, 0.1, null,
                MassProperties.point(20, Vec3d.ZERO), List.of()));
        double mass = load / G;
        Vehicle v = new VehicleBlueprint("drop", List.of(
                new VehicleBlueprint.Entry("leg", leg, Mount.ORIGIN),
                new VehicleBlueprint.Entry("mass", TestParts.box("m", new Vec3d(0.5, 0.5, 0.5), 0, false, 1, MassSpec.given(mass - 20)), Mount.at(new Vec3d(0, 0.5, 0)))),
                List.of()).compile();
        // FAR 23.473: descent 4.4 (W/S)^(1/4) ft/s, 8.62 ft/s for a 172S; wing lift 2/3 of the weight through the impact
        double descent = 8.62 * 0.3048, height = tyre.radiusM() + 0.02;
        VehicleState st = v.newState(new Vec3d(0, height, 0), Orientation.IDENTITY, TestTrack.DT);
        st.internals[v.installed("leg").firstInternal()] = 0;   // the strut hangs fully extended before touchdown
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(0, -descent, 0), Orientation.IDENTITY, Vec3d.ZERO));
        VehicleEnvironment env = VehicleEnvironment.land(FLAT, AIR);
        double peak = 0, deepest = 0;
        for (int i = 0; i < 400; i++) {
            VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, Map.of(), Vec3d.Y.scale(-G),
                    List.of(new dev.mcmod.aerocore.api.AppliedForce(dev.mcmod.aerocore.api.AppliedForce.Kind.LIFT, Vec3d.ZERO, Vec3d.Y.scale(2.0 / 3 * mass * G))),
                    null, i * TestTrack.DT, TestTrack.DT);
            peak = Math.max(peak, r.load(dev.mcmod.aerocore.api.AppliedForce.Kind.CONTACT).force().y());
            deepest = Math.max(deepest, st.shown("leg", 2));
        }
        double factor = peak / (mass * G);
        assertTrue(deepest < stroke, "no bottoming: " + deepest + " of " + stroke + " m");
        assertTrue(factor > 1.2 && factor < 3.0, "ground reaction load factor " + factor + " (FAR 23.473: designed for at least 2.0)");
        assertTrue(Math.abs(st.body().velocity().y()) < 0.05, "settled: " + st.body().velocity().y());
    }
}
