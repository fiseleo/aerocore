package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.ArrayList;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

/** Plausibility of the demonstrator boat (no measured data exists for it): flotation, stability, speed and turning. */
class TestHullTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, DT = 0.005;
    private static final FluidState WATER = FluidState.FRESH_WATER_20C, AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final HullDynamics.Environment CALM = point -> HullDynamics.Water.calm(0, WATER, AIR);

    @Test void boatFloatsWithADisplacementHullsProportions() {
        SlenderHull.Particulars d = TestHull.model().design();
        assertEquals(TestHull.MASS / WATER.densityKgPerM3(), d.volumeM3(), 1e-3);
        assertTrue(d.draftM() > 0.2 && d.draftM() < 0.4, "Draft " + d.draftM());
        assertTrue(d.blockCoefficient() > 0.35 && d.blockCoefficient() < 0.65, "C_B " + d.blockCoefficient());
        // Propeller and rudder sit below the design waterline.
        assertTrue(TestHull.PROPELLER.positionLocal().y() + 0.5 * TestHull.PROPELLER.diameterM() < d.waterlineY());
        assertTrue(TestHull.RUDDER.positionLocal().y() + 0.5 * TestHull.RUDDER.spanM() < d.waterlineY() + 0.05);
    }
    @Test void heeledBoatRightsItself() {
        RigidBodyProperties p = TestHull.properties();
        double waterline = TestHull.model().design().waterlineY();
        RigidBodyState s = TestHull.atModelOrigin(new Vec3d(0, -waterline, 0), Orientation.rotationVector(new Vec3d(0.15, 0, 0)));
        Wrench load = HullDynamics.captiveLoad(s, p, TestHull.model(), CALM, Vec3d.Y.scale(-G));
        assertTrue(load.torque().x() < 0, "Heel to +x rotation must produce a restoring moment: " + load.torque());
    }
    @Test void propellerDrivesTheBoatToAPlausibleSpeedAndTheRudderTurnsItToStarboard() {
        RigidBodyProperties p = TestHull.properties();
        double waterline = TestHull.model().design().waterlineY();
        RigidBodyState s = TestHull.atModelOrigin(new Vec3d(0, -waterline, 0), Orientation.IDENTITY);
        double rps = 12;
        for (int i = 0; i < 8000; i++) { s = step(s, p, rps, 0); }
        // At the time of writing: 3.25 m/s at 12 rev/s; 20 deg rudder then turns on an 18 m circle (3.0 L) at 3.0 m/s.
        double speed = s.velocity().dot(s.orientation().rotate(Vec3d.X));
        assertTrue(speed > 2 && speed < 6, "Steady speed " + speed);
        // Rudder +20 deg: the bow turns to starboard, i.e. negative yaw rate about +y.
        for (int i = 0; i < 6000; i++) { s = step(s, p, rps, Math.toRadians(20)); }
        double yawRate = s.angularVelocity(p).y(), turnSpeed = s.velocity().length(), diameter = 2 * turnSpeed / Math.abs(yawRate);
        assertTrue(yawRate < 0, "Positive rudder must turn the bow to starboard: " + yawRate);
        assertTrue(diameter > 2 * TestHull.LENGTH && diameter < 12 * TestHull.LENGTH, "Turning diameter " + diameter);
        assertTrue(turnSpeed < speed, "Turning costs speed");
    }

    private static RigidBodyState step(RigidBodyState s, RigidBodyProperties p, double rps, double rudder) {
        PropellerSolver.Load screw = PropellerSolver.load(s, p, TestHull.PROPELLER, rps, WATER);
        List<AppliedForce> loads = new ArrayList<>();
        loads.add(screw.force());
        SurfaceForces r = TestHull.RUDDER.load(s, p, rudder, WATER, screw.thrustN(), TestHull.PROPELLER.diameterM());
        loads.add(new AppliedForce(AppliedForce.Kind.LIFT, s.worldOffset(p, TestHull.RUDDER.positionLocal()), r.wrench().force()));
        return HullDynamics.step(s, p, TestHull.model(), CALM, Vec3d.Y.scale(-G), loads, null, DT).state();
    }
}
