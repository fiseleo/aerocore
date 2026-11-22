package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.DrumPart;
import dev.mcmod.aerocore.api.vehicle.part.TrackPart;
import dev.mcmod.aerocore.api.wheel.RigidRoller;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** 0.43: crawler tracks and roller drums (TrackPart, DrumPart, RigidRoller). */
class TrackTest {
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final double G = TestTrack.G;

    /** A 20 t-class crawler's tracks: 2.4 m gauge, 3.6 m on the ground, 600 mm steel shoes, 7 road wheels. */
    private static final TrackPart.Spec SPEC = new TrackPart.Spec(2.4, 3.6, 0.6, 7, 0.1, 0.05, 0.3, 0.35, 0.12, TrackPart.Shoe.STEEL,
            1.5e6, 2.5e4, 0.6, 0.05, 40, 60000, 60000);

    private static PartDesign tracks(TrackPart.Spec spec, double massKg) {
        MassProperties m = MassShapes.box(new Vec3d(spec.contactLengthM() + 1, 0.8, spec.gaugeM() + 0.6), 1e9);
        MassProperties mass = m.scaled(massKg / m.massKg());
        return new TestParts.Fixture("tracks", "tracks " + spec + " " + massKg, d -> new TrackPart(d, spec, mass, List.of()));
    }

    /** A 25 t crawler: 20 t of body above 5 t of tracks, each sprocket driven by its own motor through 40 : 1. */
    private static Vehicle crawler() {
        PartDesign body = TestParts.solid("body", new Vec3d(5, 1.5, 3), 20000, new Vec3d(0, 1.2, 0), Map.of(), null, 0, List.of());
        PartDesign motor = TestParts.motor("motor", 2000, 150e3, 3000, 0.5, 0.95, 200);
        return new VehicleBlueprint("crawler", List.of(
                new VehicleBlueprint.Entry("body", body, Mount.at(Vec3d.ZERO)),
                new VehicleBlueprint.Entry("tracks", tracks(SPEC, 5000), Mount.at(Vec3d.ZERO),
                        List.of(Binding.of("brake", "brake"), Binding.of("park", "park"))),
                new VehicleBlueprint.Entry("left", motor, Mount.at(new Vec3d(-1.5, 0.8, -0.8)), List.of(Binding.of("throttle", "left"))),
                new VehicleBlueprint.Entry("right", motor, Mount.at(new Vec3d(-1.5, 0.8, 0.8)), List.of(Binding.of("throttle", "right")))),
                List.of(Connection.Shaft.ports("left", "", "tracks", "left", 40, 0.95), Connection.Shaft.ports("right", "", "tracks", "right", 40, 0.95))).compile();
    }

    private static double total(Map<String, Map<String, Double>> d) {
        Map<String, Double> t = d.get("tracks");
        return t.get("leftLoadN") + t.get("rightLoadN");
    }

    @Test
    void aCrawlerRestsOnItsRollers() {
        Vehicle v = crawler();
        VehicleState st = TestTrack.settle(v, Ground.flat(0, GroundContact.Surface.TARMAC), AIR, new Vec3d(0, 0.2, 0), 0, Map.of("park", 1.0), 3);
        var samples = TestTrack.drive(v, st, Ground.flat(0, GroundContact.Surface.TARMAC), AIR, (t, s) -> Map.of("park", 1.0), 0.5, s -> false, 1);
        double weight = v.properties().massKg() * G;
        assertEquals(weight, total(samples.getLast().diagnostics()), 0.01 * weight, "the rollers carry the weight");
        assertTrue(st.body().velocity().length() < 1e-3, "it stands still: " + st.body().velocity().length());
    }

    @Test
    void theShearStateRisesAsWongsSlipCurveDoesAtLowSlip() {
        // steady rolling at slip i: Wong's tractive effort F/Fmax = 1 - K/(iL) (1 - exp(-iL/K)); the shear state gives
        // 1 - exp(-u/K) with u = i sigma / (1 - i), sigma = L/2: the same slope at small slip
        double l = 3.6, k = 0.025, belt = 2;
        for (double i : new double[]{0.01, 0.02, 0.05}) {
            double vx = belt * (1 - i), u = 0;
            for (int n = 0; n < 200000; n++) { u = RigidRoller.relaxed(vx - belt, vx, l / 2, k, u, 0.005); }
            double ours = RigidRoller.shear(1, vx - belt, vx, l / 2, k, u, 0.005).forceN();
            double wong = 1 - k / (i * l) * (1 - Math.exp(-i * l / k));
            assertEquals(wong, ours, 0.35 * wong, "slip " + i);
        }
        // and both reach the full capacity at large slip
        double vx = 0.5, u = 0;
        for (int n = 0; n < 200000; n++) { u = RigidRoller.relaxed(vx - 2, vx, l / 2, k, u, 0.005); }
        assertTrue(RigidRoller.shear(1, vx - 2, vx, l / 2, k, u, 0.005).forceN() > 0.99);
    }

    @Test
    void itParksOnASlopeItsFrictionHoldsAndSlidesOnIce() {
        double slope = Math.toRadians(15);
        Vec3d normal = new Vec3d(-Math.sin(slope), Math.cos(slope), 0);
        for (GroundContact.Surface surface : new GroundContact.Surface[]{GroundContact.Surface.TARMAC, GroundContact.Surface.ICE}) {
            Ground ground = Ground.plane(Vec3d.ZERO, normal, surface);
            Vehicle v = crawler();
            VehicleState st = v.newState(new Vec3d(0, 0.25, 0), Orientation.rotationVector(Vec3d.Z.scale(slope)), TestTrack.DT);
            TestTrack.drive(v, st, ground, AIR, (t, s) -> Map.of("park", 1.0), 4, s -> false, Integer.MAX_VALUE);
            double speed = st.body().velocity().length();
            if (surface == GroundContact.Surface.TARMAC) { assertTrue(speed < 0.01, "steel shoes hold 15 degrees of asphalt: " + speed); }
            else { assertTrue(speed > 0.5, "on ice it slides: " + speed); }
        }
    }

    @Test
    void itDrivesOnSandNoHarderThanTheSoilGives() {
        Ground sand = Ground.flat(0, GroundContact.Surface.SAND);
        Vehicle v = crawler();
        VehicleState st = TestTrack.settle(v, sand, AIR, new Vec3d(0, 0.2, 0), 0, Map.of("park", 1.0), 2);
        var samples = TestTrack.drive(v, st, sand, AIR, (t, s) -> Map.of("left", 1.0, "right", 1.0), 8, s -> false, 20);
        double speed = st.body().velocity().x();
        assertTrue(speed > 1, "it drives off: " + speed);
        GroundContact.Soil soil = GroundContact.Soil.DRY_SAND;
        for (var s : samples) {
            Map<String, Double> d = s.diagnostics().get("tracks");
            double w = d.get("leftLoadN");
            double limit = 0.6 * 3.6 * soil.cohesionKpa() * 1000 + w * Math.tan(soil.frictionAngle());
            assertTrue(d.get("leftTractionN") <= limit * 1.001, "within the soil's shear strength");
        }
        assertTrue(samples.getLast().diagnostics().get("tracks").get("leftSinkageM") > 0, "the shoes sink into sand");
    }

    @Test
    void aPivotTurnMeetsTheShoesSlidingResistance() {
        Ground ground = Ground.flat(0, GroundContact.Surface.TARMAC);
        Vehicle v = crawler();
        double w = v.properties().massKg() * G;
        // the road wheels' loads spread evenly: the moment is mu_t x (W / n) x sum |x_i| (0.286 mu W L for seven rollers; Wong's continuous 0.25)
        double moment = 0.6 * w / 7 * 2 * (1.8 + 1.2 + 0.6);
        for (double share : new double[]{0.6, 1.6}) {
            VehicleState st = TestTrack.settle(v, ground, AIR, new Vec3d(0, 0.2, 0), 0, Map.of("park", 1.0), 2);
            AppliedForce couple = new AppliedForce(AppliedForce.Kind.THRUST, Vec3d.ZERO, Vec3d.ZERO, Vec3d.Y.scale(share * moment));
            VehicleEnvironment env = VehicleEnvironment.land(ground, AIR);
            for (int i = 0; i < 400; i++) {
                VehicleDynamics.step(v, st, env, Map.of(), Vec3d.Y.scale(-G), List.of(couple), null, i * TestTrack.DT, TestTrack.DT);
            }
            double yawRate = st.body().angularVelocity(st.properties()).y();
            if (share < 1) { assertTrue(Math.abs(yawRate) < 0.02, "below the resisting moment it holds: " + yawRate); }
            else { assertTrue(yawRate > 0.1, "above it the crawler turns: " + yawRate); }
        }
    }

    @Test
    void aTandemRollerRollsOnAndShakesWhenAsked() {
        DrumPart.Spec drum = new DrumPart.Spec(2.1, 0.75, 3, DrumPart.Shell.SMOOTH, 2e6, 3e4, 400, 3.0, 45, 30000, 30000);
        MassProperties m = MassShapes.box(new Vec3d(1.5, 1.5, 2.1), 1e9);
        MassProperties dm = m.scaled(2000 / m.massKg());
        PartDesign drums = new TestParts.Fixture("drum", "drum " + drum, d -> new DrumPart(d, drum, dm, List.of()));
        PartDesign body = TestParts.solid("body", new Vec3d(4, 1, 2), 6000, new Vec3d(0, 1, 0), Map.of(), null, 0, List.of());
        Vehicle v = new VehicleBlueprint("roller", List.of(new VehicleBlueprint.Entry("body", body, Mount.at(Vec3d.ZERO)),
                new VehicleBlueprint.Entry("front", drums, Mount.at(new Vec3d(1.6, 0, 0)), List.of(Binding.of("vibration", "vibration"))),
                new VehicleBlueprint.Entry("rear", drums, Mount.at(new Vec3d(-1.6, 0, 0)))), List.of()).compile();
        Ground ground = Ground.flat(0, GroundContact.Surface.TARMAC);
        VehicleState st = TestTrack.settle(v, ground, AIR, new Vec3d(0, 0.76, 0), 0, Map.of("park", 1.0), 2);
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(2, 0, 0), b.orientation(), Vec3d.ZERO));
        // the drums start rolling with the body
        for (int k = 0; k < v.trains(); k++) { st.setTrainSpeed(k, 2 / 0.75); }
        var samples = TestTrack.drive(v, st, ground, AIR, (t, s) -> Map.of("vibration", 1.0), 2, s -> false, 1);
        double expected = 2 - DrumPart.HARD_ROLLING * G * 2;   // 0.02 g of rolling resistance, a little less with the drums' inertia
        assertEquals(expected, st.body().velocity().x(), 0.15, "it rolls on, slowed by steel on asphalt");
        assertEquals(drum.dynamicForceN(), samples.getLast().diagnostics().get("front").get("dynamicForceN"), 1e-6);
        assertEquals(3.0 * Math.pow(2 * Math.PI * 45, 2), drum.dynamicForceN(), 1e-6);
    }
}
