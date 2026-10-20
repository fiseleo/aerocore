package dev.mcmod.aerocore.api.wheel.kinematics;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.wheel.Suspension;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Hardpoint kinematics (0.33) against geometry worked by hand and the simple swing-arm model. */
class KinematicsTest {
    static final double TRACK = 1.55, R = 0.31, DESIGN = 0.1, TRAVEL = 0.2;
    static final Vec3d HUB = new Vec3d(0, -0.05, TRACK / 2);

    /** The camber gain at the design position, rad/m (from the samples either side). */
    static double camberGain(KinematicTable t) {
        double h = 0.01;
        return (t.at(DESIGN + h).camberRad() - t.at(DESIGN - h).camberRad()) / (2 * h);
    }

    @Test void equalParallelArmsKeepTheWheelUpright() {
        Vec3d lowerBall = HUB.add(new Vec3d(0, -0.13, -0.02)), upperBall = HUB.add(new Vec3d(0, 0.17, -0.02));
        Vec3d lowerInner = lowerBall.add(new Vec3d(0, 0, -0.38)), upperInner = upperBall.add(new Vec3d(0, 0, -0.38));
        Vec3d tieOuter = HUB.add(new Vec3d(-0.13, 0, -0.02)), tieInner = tieOuter.add(new Vec3d(0, 0, -0.38));
        Hardpoints.DoubleWishbone d = new Hardpoints.DoubleWishbone(HUB, lowerInner.add(new Vec3d(0.14, 0, 0)), lowerInner.add(new Vec3d(-0.14, 0, 0)), lowerBall,
                upperInner.add(new Vec3d(0.11, 0, 0)), upperInner.add(new Vec3d(-0.11, 0, 0)), upperBall, tieInner, tieOuter,
                lowerInner.add(new Vec3d(0, 0.35, 0.2)), lowerInner.add(new Vec3d(0, 0, 0.2)));
        KinematicTable t = KinematicTable.of(d, R, DESIGN, TRAVEL);
        for (KinematicTable.Sample s : t.samples()) {
            assertEquals(0, s.camberRad(), 1e-9, "a parallelogram keeps the upright upright at " + s.compressionM());
            assertEquals(0, s.toeRad(), 1e-9, "and a parallel tie rod of the arms' length keeps the toe");
        }
        assertEquals(0, t.rollCentreHeightM(), 1e-6, "level parallel arms: the roll centre on the ground");
        assertEquals(0, t.at(DESIGN).sideTanContact(), 1e-9, "level pivots: no anti");
        // the hub swings in on the arms' arc away from the design position
        assertTrue(t.hubOffset(DESIGN + 0.1).z() < -0.01, "in by " + t.hubOffset(DESIGN + 0.1).z());
        assertEquals(DESIGN + 0.1 - DESIGN, t.at(DESIGN + 0.1).hub().y() - HUB.y(), 1e-9, "the table is indexed by the hub's rise");
        // the spring on the lower arm: the motion ratio is its share of the arm's length (0.2 of 0.38) times the sine of its angle to the arm
        assertEquals(0.2 / 0.38 * 0.35 / Math.hypot(0.35, 0), t.at(DESIGN).motionRatio(), 0.01);
    }

    @Test void theSimpleLayoutsMatchTheirSwingArmGeometry() {
        for (Suspension.Type type : Suspension.Type.values()) {
            Suspension.Geometry g = Suspension.Geometry.of(type, TRACK, R, 0, -1, 0);
            KinematicTable t = KinematicTable.of(Hardpoints.fromSimple(type, TRACK, HUB, R, 0, -1, 0), R, DESIGN, TRAVEL);
            assertEquals(g.rollCentreHeightM(), t.rollCentreHeightM(), 1e-3, type + ": roll centre");
            if (g.solid()) {
                assertEquals(0, camberGain(t), 1e-12, type + ": a solid axle stays square");
            } else {
                assertEquals(g.camberGainPerM(), camberGain(t), Math.max(0.05 * Math.abs(g.camberGainPerM()), 2e-3), type + ": camber gain");
            }
        }
    }

    @Test void aTieRodOnTheInstantCentresLineHasLittleBumpSteer() {
        // the second-order term levelled out, a cubic of about 0.01 degree is left at 50 mm (road cars: 0.1-0.25)
        for (Suspension.Type type : List.of(Suspension.Type.DOUBLE_WISHBONE, Suspension.Type.MULTI_LINK, Suspension.Type.MACPHERSON)) {
            KinematicTable t = KinematicTable.of(Hardpoints.fromSimple(type, TRACK, HUB, R, 0, -1, 0), R, DESIGN, TRAVEL);
            for (double dc : new double[]{-0.05, 0.05}) {
                assertTrue(Math.abs(Math.toDegrees(t.at(DESIGN + dc).toeRad())) < 0.02, type + ": " + Math.toDegrees(t.at(DESIGN + dc).toeRad()) + " deg at " + dc);
            }
        }
    }

    @Test void aTrailingArmsHubTravelsOnACircle() {
        Hardpoints.TrailingArm a = (Hardpoints.TrailingArm) Hardpoints.fromSimple(Suspension.Type.TRAILING_ARM, TRACK, HUB, R, 0, -1, 0);
        KinematicTable t = KinematicTable.of(a, R, DESIGN, TRAVEL);
        double radius = Math.hypot(HUB.x() - a.pivotA().x(), HUB.y() - a.pivotA().y());
        for (KinematicTable.Sample s : t.samples()) {
            assertEquals(radius, Math.hypot(s.hub().x() - a.pivotA().x(), s.hub().y() - a.pivotA().y()), 1e-9);
            assertEquals(0, s.camberRad(), 1e-12);
            assertEquals(0, s.toeRad(), 1e-12);
        }
        // the hub swings forward as it rises behind its pivot: its path's side-view slope is the arm's
        assertEquals(-(HUB.y() - a.pivotA().y()) / (HUB.x() - a.pivotA().x()), t.at(DESIGN).sideTanHub(), 1e-6);
    }

    @Test void pivotsPitchedNoseDownGiveAntiDive() {
        Hardpoints.DoubleWishbone d = (Hardpoints.DoubleWishbone) Hardpoints.fromSimple(Suspension.Type.DOUBLE_WISHBONE, TRACK, HUB, R, 0, -1, 0);
        double beta = Math.toRadians(10);
        Hardpoints.DoubleWishbone pitched = pitch(d, beta);
        KinematicTable level = KinematicTable.of(d, R, DESIGN, TRAVEL), anti = KinematicTable.of(pitched, R, DESIGN, TRAVEL);
        assertEquals(0, level.at(DESIGN).sideTanContact(), 1e-6);
        assertEquals(Math.tan(beta), anti.at(DESIGN).sideTanContact(), 0.02, "the contact patch moves forward as the wheel rises");
        // front view unchanged
        assertEquals(level.rollCentreHeightM(), anti.rollCentreHeightM(), 0.01);
    }

    /** The double wishbone's pivot axes pitched nose down by an angle about their middles. */
    static Hardpoints.DoubleWishbone pitch(Hardpoints.DoubleWishbone d, double beta) {
        Vec3d[] lower = pitchPair(d.lowerFront(), d.lowerRear(), beta), upper = pitchPair(d.upperFront(), d.upperRear(), beta);
        return new Hardpoints.DoubleWishbone(d.hub(), lower[0], lower[1], d.lowerBall(), upper[0], upper[1], d.upperBall(), d.tieInner(), d.tieOuter(), d.springBody(), d.springArm());
    }

    static Vec3d[] pitchPair(Vec3d front, Vec3d rear, double beta) {
        Vec3d mid = front.add(rear).scale(0.5);
        double half = front.subtract(rear).length() / 2;
        Vec3d u = new Vec3d(Math.cos(beta), -Math.sin(beta), 0).scale(half);
        return new Vec3d[]{mid.add(u), mid.subtract(u)};
    }

    @Test void aStrutSlidesAndLinksThatCannotReachBind() {
        KinematicTable t = KinematicTable.of(Hardpoints.fromSimple(Suspension.Type.MACPHERSON, TRACK, HUB, R, 0, -1, 0), R, DESIGN, TRAVEL);
        double ratio = t.at(DESIGN).motionRatio();
        assertTrue(ratio > 0.85 && ratio < 1.05, "a strut's motion ratio is about the cosine of its lean: " + ratio);
        // the steering axis from the lower ball joint to the strut's top (the simple model's short default swing arm leans
        // the strut a long way in: a 1.5 m arm with its centre 0.1 m up)
        KinematicTable.SteeringAxis axis = t.steeringAxis();
        Hardpoints.MacPherson mp = (Hardpoints.MacPherson) t.hardpoints();
        Vec3d s = mp.strutTop().subtract(mp.lowerBall());
        assertEquals(Math.atan2(-s.z(), s.y()), axis.kingpinRad(), 1e-12);
        assertEquals(Math.atan2(-s.x(), s.y()), axis.casterRad(), 1e-12);
        assertTrue(axis.kingpinRad() > 0 && axis.kingpinRad() < Math.toRadians(45), "kingpin " + Math.toDegrees(axis.kingpinRad()));
        // a lower arm too short to follow the wheel through its travel
        Hardpoints.DoubleWishbone d = (Hardpoints.DoubleWishbone) Hardpoints.fromSimple(Suspension.Type.DOUBLE_WISHBONE, TRACK, HUB, R, 0, -1, 0);
        Vec3d inner = d.lowerBall().add(new Vec3d(0, 0.02, -0.03));
        Hardpoints.DoubleWishbone stub = new Hardpoints.DoubleWishbone(d.hub(), inner.add(new Vec3d(0.01, 0, 0)), inner.add(new Vec3d(-0.01, 0, 0)), d.lowerBall(),
                d.upperFront(), d.upperRear(), d.upperBall(), d.tieInner(), d.tieOuter(), d.springBody(), d.springArm());
        assertThrows(LinkageLockException.class, () -> KinematicTable.of(stub, R, DESIGN, TRAVEL));
    }
}
