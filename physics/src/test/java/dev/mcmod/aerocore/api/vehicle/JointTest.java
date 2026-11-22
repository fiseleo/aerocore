package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** 0.43: joints inside one vehicle (Part.joint, VehiclePose, JointDrive). */
class JointTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    /** Water far below and air of no density: nothing but the joints and gravity acts. */
    private static final VehicleEnvironment VACUUM = VehicleEnvironment.calm(-1e4, WATER, AIR);

    /** A 2 t base with a 500 kg, 3 m arm hinged at its top front edge about +z (q > 0 lifts the arm's tip). */
    private static VehicleBlueprint arm(double reachNm, List<CollisionSphere> feet) {
        Part.Joint hinge = Part.Joint.revolute(Vec3d.ZERO, Vec3d.Z, -1.2, 1.2, 0);
        PartDesign base = TestParts.solid("base", new Vec3d(4, 1, 2), 2000, Vec3d.ZERO, Map.of("pin", Mount.at(new Vec3d(2, 0.5, 0))), null, 0, feet);
        PartDesign link = TestParts.solid("arm", new Vec3d(3, 0.3, 0.3), 500, new Vec3d(1.5, 0, 0), Map.of(), hinge, reachNm, List.of());
        return new VehicleBlueprint("arm", List.of(
                new VehicleBlueprint.Entry("base", base, Mount.at(Vec3d.ZERO)),
                new VehicleBlueprint.Entry("arm", link, Mount.on("base", "pin"), List.of(Binding.of("rate", "rate")))), List.of());
    }

    @Test
    void aVehicleWithoutJointsKeepsItsCompiledGeometry() {
        Vehicle v = new VehicleBlueprint("plain", List.of(new VehicleBlueprint.Entry("base",
                TestParts.solid("base", new Vec3d(4, 1, 2), 2000, Vec3d.ZERO, Map.of(), null, 0, List.of()), Mount.at(Vec3d.ZERO))), List.of()).compile();
        VehicleState s = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        assertTrue(v.joints().isEmpty());
        assertSame(v.properties(), s.properties());
        assertSame(v.collision(), s.pose().collision());
        assertTrue(!s.toJson().containsKey("joints"));
    }

    @Test
    void aSwingingArmKeepsTheMomentumOfTheWhole() {
        Vehicle v = arm(Double.POSITIVE_INFINITY, List.of()).compile();
        VehicleState s = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        Vec3d com0 = s.body().position();
        double maxTurn = 0;
        for (int i = 0; i < 800; i++) {
            double t = i * DT, rate = t < 1 ? 0.5 : t < 2 ? -0.5 : 0;
            VehicleDynamics.step(v, s, VACUUM, Map.of("rate", rate), Vec3d.ZERO, List.of(), null, t, DT);
            // the base turns the other way about z while the arm lifts
            double turn = 2 * Math.atan2(s.body().orientation().z(), s.body().orientation().w());
            if (t < 1) { assertTrue(turn <= 1e-12, "the base turns against the arm: " + turn); }
            maxTurn = Math.max(maxTurn, Math.abs(turn));
            Vec3d momentum = s.body().velocity().scale(s.properties().massKg()).add(s.relativeLinear);
            assertEquals(0, momentum.length(), 1e-9, "linear momentum at " + t);
            Vec3d angular = s.body().angularMomentum().add(s.body().orientation().rotate(s.relativeAngular));
            assertEquals(0, angular.length(), 1e-6, "angular momentum at " + t);
        }
        assertEquals(0, s.jointValue(0), 1e-9);
        assertTrue(maxTurn > 0.01, "the base turned back while the arm swung: " + maxTurn);
        double back = 2 * Math.atan2(s.body().orientation().z(), s.body().orientation().w());
        assertEquals(0, back, 1e-4, "out and back again, the base faces as it did");
        // the step takes the parts' momentum at the start pose and the centre's move by the pose's chord: a first-order
        // drift (0.15 mm in 4 s of swinging 3 m of arm about)
        assertEquals(0, s.body().position().subtract(com0).length(), 1e-3, "the centre of mass stays put");
    }

    /** A base standing on four feet on a floor at y = -0.5 - radius. */
    private static FloatingBodyDynamics.Collision floor(List<CollisionSphere> feet) {
        WorldBox ground = new WorldBox(new Vec3d(-50, -10, -50), new Vec3d(50, -0.8, 50), ContactMaterial.STEEL_ON_ROCK);
        return new FloatingBodyDynamics.Collision(feet, List.of(ground), ContactMaterial.STEEL_ON_ROCK);
    }

    private static final List<CollisionSphere> FEET = List.of(new CollisionSphere(new Vec3d(-1.8, -0.5, -0.8), 0.3), new CollisionSphere(new Vec3d(-1.8, -0.5, 0.8), 0.3),
            new CollisionSphere(new Vec3d(1.8, -0.5, -0.8), 0.3), new CollisionSphere(new Vec3d(1.8, -0.5, 0.8), 0.3));

    @Test
    void aWeakDriveCannotHoldTheArmUpAndAStrongOneLiftsIt() {
        // the arm's weight about the pin: 500 kg x g x 1.5 m = 7355 N m at q = 0
        double weight = 500 * Atmosphere.GRAVITY_MPS2 * 1.5;
        // 0.3 of it cannot hold the arm anywhere above its -1.2 rad stop (there the weight's moment is cos 1.2 = 0.36 of it)
        for (double reach : new double[]{0.3 * weight, 2 * weight}) {
            Vehicle v = arm(reach, FEET).compile();
            VehicleState s = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            for (int i = 0; i < 400; i++) {
                VehicleDynamics.step(v, s, VACUUM, Map.of("rate", 0.3), Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2), List.of(), floor(FEET), i * DT, DT);
            }
            if (reach < weight) { assertEquals(-1.2, s.jointValue(0), 1e-9, "a weak drive lets the arm fall to its stop"); }
            else { assertEquals(0.3 * 2.0, s.jointValue(0), 0.02, "a strong drive lifts it at the rate"); }
        }
    }

    @Test
    void reachingOutPastTheFrontFeetTipsTheMachine() {
        // a 4 t mass on a 6 m arm: the centre of mass passes the front feet (x = 1.8) at a computable angle
        Part.Joint hinge = Part.Joint.revolute(Vec3d.ZERO, Vec3d.Z, -1.5, 1.5, 1.2);
        PartDesign base = TestParts.solid("base", new Vec3d(4, 1, 2), 6000, Vec3d.ZERO, Map.of("pin", Mount.at(new Vec3d(1, 0.5, 0))), null, 0, FEET);
        // a drive of finite reach (the arm's weight is 1.4e5 N m at the tipping angle): it cannot jerk the arm to speed
        PartDesign link = TestParts.solid("arm", new Vec3d(6, 0.3, 0.3), 4000, new Vec3d(6, 0, 0), Map.of(), hinge, 4e5, List.of());
        Vehicle v = new VehicleBlueprint("reach", List.of(new VehicleBlueprint.Entry("base", base, Mount.at(Vec3d.ZERO)),
                new VehicleBlueprint.Entry("arm", link, Mount.on("base", "pin"), List.of(Binding.of("rate", "rate")))), List.of()).compile();
        // the tipping angle: total COM x = (6000 x 0 + 4000 x (1 + 6 cos q)) / 10000 = 1.8  ->  cos q = 3.5 / 6
        double tip = Math.acos(3.5 / 6);
        VehicleState s = v.newState(new Vec3d(0, 0.02, 0), Orientation.IDENTITY, DT);
        for (int i = 0; i < 200; i++) {   // settle at 1.2 rad (COM well inside)
            VehicleDynamics.step(v, s, VACUUM, Map.of("rate", 0.0), Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2), List.of(), floor(FEET), i * DT, DT);
        }
        double rearBefore = s.body().worldPoint(s.properties(), new Vec3d(-1.8, -0.5, 0)).y();
        double liftedAt = Double.NaN;
        for (int i = 0; i < 4000 && Double.isNaN(liftedAt); i++) {
            double rate = -0.05 * Math.min(1, i * DT);   // the operator eases the arm down
            VehicleDynamics.step(v, s, VACUUM, Map.of("rate", rate), Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2), List.of(), floor(FEET), (200 + i) * DT, DT);
            double rear = s.body().worldPoint(s.properties(), new Vec3d(-1.8, -0.5, 0)).y();
            if (rear - rearBefore > 0.003) { liftedAt = s.jointValue(0); }
        }
        assertTrue(!Double.isNaN(liftedAt), "the machine tipped");
        assertEquals(tip, liftedAt, 0.04, "it tips where its centre of mass passes the front feet");
    }

    @Test
    void jointsAreSavedAndCarried() {
        Vehicle v = arm(Double.POSITIVE_INFINITY, List.of()).compile();
        VehicleState s = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        for (int i = 0; i < 100; i++) { VehicleDynamics.step(v, s, VACUUM, Map.of("rate", 0.4), Vec3d.ZERO, List.of(), null, i * DT, DT); }
        double q = s.jointValue(0);
        assertEquals(0.2, q, 1e-9);
        VehicleState read = VehicleState.fromJson(v, s.toJson(), DT);
        assertEquals(q, read.jointValue(0), 0);
        assertEquals(s.body().position().x(), read.body().position().x(), 0);
        // a save without joints (before 0.43) starts them where a new vehicle has them
        Map<String, Object> old = new java.util.LinkedHashMap<>(s.toJson());
        old.remove("joints");
        assertEquals(0, VehicleState.fromJson(v, old, DT).jointValue(0), 0);
        // placed elsewhere: the joint stays as it stood, the frame at the new origin
        VehicleState moved = s.placed(new Vec3d(10, 0, 0), Orientation.IDENTITY);
        assertEquals(q, moved.jointValue(0), 0);
        assertEquals(10, moved.origin().x(), 1e-9);
        assertNotSame(s.pose(), moved.pose());
        assertEquals(s.properties().centerOfMassLocal().x(), moved.properties().centerOfMassLocal().x(), 1e-12);
    }
}
