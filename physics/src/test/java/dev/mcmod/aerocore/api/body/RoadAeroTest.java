package dev.mcmod.aerocore.api.body;

import dev.mcmod.aerocore.api.FlightAtmosphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.aero.Airfoil;
import dev.mcmod.aerocore.api.aero.Wing;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * A road vehicle's aero kit (0.34): the floor channel's trends (Cooper et al. 1998; Katz 1995), the rear wing as the
 * lifting line it is built on, and the plain pieces. Coefficients are on a 2 m^2 frontal area at 30 m/s. 0.35.1: the
 * wheels (Breslouer &amp; George 2008), a rough floor ahead of a diffuser (Marklund 2013) and production cars' published
 * figures (Porsche 718 Cayman GT4, Mercedes-AMG GT R, 911 GT3 RS) within 30 %.
 */
class RoadAeroTest {
    private static final double SPEED = 30, RHO = 1.225, Q = 0.5 * RHO * SPEED * SPEED, FRONTAL = 2;
    private static final Vec3d DRIVING = new Vec3d(-SPEED, 0, 0);

    /** A 3.5 m floor 1.6 m wide (a sedan's), from 4.0 m back to 0.5 m, a diffuser over its last 0.6 m at an angle. */
    private static RoadAero.Underbody floor(double angleDeg, double sealing, double damM, double splitterM, boolean smooth) {
        return new RoadAero.Underbody(4.0, 0.5, 1.6, 0.15, angleDeg > 0 ? 0.6 : 0, angleDeg, sealing, damM, splitterM, splitterM > 0 ? 1.6 : 0, smooth);
    }

    private static RoadAero.Loads at(RoadAero.Element e, double heightM) {
        return new RoadAero(List.of(e)).loads(DRIVING, RHO, p -> heightM);
    }

    private static double cl(RoadAero.Element e, double heightM) { return at(e, heightM).downforceN() / (Q * FRONTAL); }

    @Test void aPlainFloorAddsNothing() {
        RoadAero.Loads l = at(floor(0, 0, 0, 0, false), 0.14);
        assertEquals(Vec3d.ZERO, l.force(), "the kit's floor is the road car's floor: exactly nothing");
        assertEquals(Vec3d.ZERO, l.moment());
        assertSame(RoadAero.Loads.NONE, new RoadAero(List.of()).loads(DRIVING, RHO, p -> 0.1));
    }

    @Test void aDiffuserPullsDownUntilTheCarIsTooLow() {
        RoadAero.Underbody d = floor(10, 0, 0, 0, true);
        double best = 0, bestH = 0;
        for (double h = 0.01; h <= 0.4; h += 0.01) {
            double c = cl(d, h);
            if (c > best) { best = c; bestH = h; }
        }
        assertTrue(bestH > 0.05 && bestH < 0.2, "the downforce peaks near the ground, not on it: " + bestH);
        assertTrue(cl(d, 0.015) < 0.3 * best, "lower still, the diffuser stalls and the floor loses it (force reduction)");
        assertTrue(cl(d, 0.4) < best && cl(d, 0.4) > 0, "higher, less ground effect");
        // a road car's diffuser and undertray, open sides, wheels at the corners: a few hundredths off the lift
        // coefficient (0.35.1; the 718 Cayman GT4's diffuser is some 0.04 on its 2 m^2, aProductionDiffuserAsTheCaymanGt4s)
        assertTrue(best > 0.03 && best < 0.2, "road car kit: " + best);
    }

    @Test void theBestRampIsModeratelySteep() {
        double best = 0, bestAngle = 0;
        for (double a = 2; a <= 30; a += 1) {
            double c = cl(floor(a, 0, 0, 0, true), 0.12);
            if (c > best) { best = c; bestAngle = a; }
        }
        assertTrue(bestAngle >= 7 && bestAngle <= 16, "Cooper et al. find the best angles around 10 to 15 degrees: " + bestAngle);
        assertTrue(cl(floor(30, 0, 0, 0, true), 0.12) < 0.5 * best, "a steep ramp stalls");
    }

    @Test void skirtsSealTheFloor() {
        double open = cl(floor(10, 0, 0, 0, true), 0.08), sealed = cl(floor(10, 1, 0, 0, true), 0.08);
        assertTrue(sealed > 2.5 * open, "sealed " + sealed + " against open " + open);
        // without wheels in it, a sealed ground-effect floor is in the race car range (wings apart); a road car's wheels take half of that
        double bare = RoadAero.floorAlone(floor(10, 1, 0, 0, true), SPEED, RHO, p -> 0.08, false).downforceN() / (Q * FRONTAL);
        assertTrue(bare > 0.8 && bare < 3, "a sealed ground-effect floor without wheels: " + bare);
        assertTrue(sealed > 0.3 && sealed < 1.5, "with a road car's wheels: " + sealed);
        assertTrue(cl(floor(0, 1, 0, 0, false), 0.12) > 0, "skirts alone hold the floor at the base pressure");
        // sealing closes a share of the sides' leak area: half of it gives less than half of the gain (the leak goes as the open length)
        double half = cl(floor(10, 0.5, 0, 0, true), 0.08);
        assertTrue(half > open && half - open < 0.5 * (sealed - open), "half sealed " + half);
    }

    @Test void wheelsTakeAboutHalfAFloorsDownforce() {
        // Breslouer & George (SAE 2008-01-0326): wheels beside a diffuser body take as much as 65 % of its downforce
        for (double sealing : new double[]{0, 1}) {
            for (double h : new double[]{0.06, 0.08, 0.12}) {
                RoadAero.Underbody u = floor(10, sealing, 0, 0, true);
                double with = RoadAero.floorAlone(u, SPEED, RHO, p -> h, true).downforceN(), without = RoadAero.floorAlone(u, SPEED, RHO, p -> h, false).downforceN();
                double taken = 1 - with / without;
                assertTrue(taken > 0.35 && taken < 0.65, "sealing " + sealing + " at " + h + " m: the wheels take " + taken);
            }
        }
    }

    @Test void aDiffuserBehindARoughFloorRecoversLittle() {
        // Marklund (Chalmers 2013): with the underbody covers off, the diffuser's pressure recovery is lost
        double smooth = cl(floor(10, 0, 0, 0, true), 0.12) - cl(floor(0, 0, 0, 0, true), 0.12),
                rough = cl(floor(10, 0, 0, 0, false), 0.12) - cl(floor(0, 0, 0, 0, false), 0.12);
        assertTrue(rough > 0 && rough < 0.5 * smooth, "rough " + rough + " against an undertray's " + smooth);
    }

    /** Production cars' published figures at 200 km/h (0.35.1 calibration): within 30 %. */
    private static final double V200 = 200 / 3.6;

    @Test void aProductionDiffuserAsTheCaymanGt4s() {
        // Porsche 718 Cayman GT4 (2019 press kit): the diffuser behind the encased underbody gives 30 % of the 122 kg
        // rear downforce at its 304 km/h top speed, so 155 N at 200 km/h. Its floor as estimated from the car (4.46 m
        // long, 1.80 m wide): 4.1 m from behind the bumper, 1.2 m between the wheels, a 0.6 m ramp at about 10 degrees, 0.11 m up.
        RoadAero.Underbody gt4 = new RoadAero.Underbody(4.1, 0, 1.2, 0.15, 0.6, 10, 0, 0, 0, 0, true);
        double n = new RoadAero(List.of(gt4)).loads(new Vec3d(-V200, 0, 0), RHO, p -> 0.11).downforceN(), ref = 0.3 * 122 * 9.81 * Math.pow(200.0 / 304, 2);
        assertEquals(ref, n, 0.3 * ref, "the GT4's diffuser at 200 km/h");
    }

    @Test void anUnderbodyDamAsTheAmgGtRs() {
        // Mercedes-AMG GT R (2016): a carbon profile under the front drops 40 mm at speed and takes 40 kg off the front
        // axle's lift at 250 km/h. A 4.0 m by 1.4 m road car floor 0.10 m up with a 40 mm dam at its front.
        RoadAero.Underbody amg = new RoadAero.Underbody(4.0, 0, 1.4, 0.15, 0, 0, 0, 0.04, 0, 0, false);
        double v = 250 / 3.6, n = new RoadAero(List.of(amg)).loads(new Vec3d(-v, 0, 0), RHO, p -> 0.10).downforceN(), ref = 40 * 9.81;
        assertEquals(ref, n, 0.3 * ref, "the GT R's underbody profile at 250 km/h");
        assertTrue(new RoadAero(List.of(amg)).loads(new Vec3d(-v, 0, 0), RHO, p -> 0.10).moment().z() < 0, "at the front: nose down");
    }

    @Test void aTrackDayKitIsAProductionTrackCars() {
        // hdfv's sample sedan with a mild kit (a 50 mm dam, an 80 mm splitter, skirts sealing about a fifth, an undertray
        // and a 10 degree diffuser, a 30 mm lip, a 1.5 m by 0.25 m wing at 8 degrees with 150 mm plates), 0.12 m up at 200 km/h.
        // 0.34 gave 4.0 kN, 3.0 kN of it from the floor. Production track cars' whole-car downforce at 200 km/h: Porsche
        // 911 GT3 RS (991.2) 345 kg at 312 km/h, so 1.4 kN; Ferrari 488 Pista 240 kg, 2.4 kN; a sedan's own body lifts
        // some 0.5 to 1 kN that the kit has to take off first.
        RoadAero kit = new RoadAero(List.of(
                new RoadAero.Underbody(4.55, 0, 1.46, 0.15, 0.5, 10, 0.22, 0.05, 0.08, 1.37, true),
                new RoadAero.Fence(new Vec3d(0.02, 1.12, 0), 1.58, 0.03),
                new RoadAero.RearWing(new Vec3d(0.24, 1.36, 0), 1.5, 0.25, 8, 0.06, 0.12, 0.15, 1.36)));
        RoadAero.Loads all = kit.loads(new Vec3d(-V200, 0, 0), RHO, p -> 0.12);
        double floor = new RoadAero(List.of(kit.elements().getFirst())).loads(new Vec3d(-V200, 0, 0), RHO, p -> 0.12).downforceN();
        assertTrue(all.downforceN() > 1000 && all.downforceN() < 2000, "the whole kit: " + all.downforceN());
        assertTrue(floor > 300 && floor < 1000, "its floor, dam and splitter: " + floor);
    }

    @Test void anAirDamTakesFrontLiftOffAndASplitterMore() {
        RoadAero.Loads none = at(floor(0, 0, 0, 0, false), 0.14), dam = at(floor(0, 0, 0.06, 0, false), 0.14),
                split = at(floor(0, 0, 0.06, 0.08, false), 0.14);
        assertEquals(0, none.downforceN(), 1e-9);
        double damCl = dam.downforceN() / (Q * FRONTAL), splitCl = split.downforceN() / (Q * FRONTAL);
        assertTrue(damCl > 0.05 && damCl < 0.25, "an air dam: some 0.1 to 0.2 off the front lift (Hucho): " + damCl);
        assertTrue(splitCl > damCl, "the splitter adds to it");
        assertTrue(dam.dragN() > 0, "the dam has drag");
        // the dam's downforce is at the front: the moment about the origin pitches the nose down
        assertTrue(dam.moment().z() < 0, "nose down: " + dam.moment());
        assertTrue(cl(floor(0, 0, 0.06, 0.08, false), 0.10) > splitCl, "lower, the splitter pulls harder");
    }

    @Test void theFloorFadesOffTheGroundAndDoesNothingBackwards() {
        RoadAero.Underbody d = floor(10, 1, 0.04, 0.05, true);
        assertTrue(cl(d, 0.3) > 0);
        assertEquals(0, cl(d, RoadAero.FADE_TO), 0, "airborne or over water: no floor effect");
        assertEquals(0, new RoadAero(List.of(d)).loads(DRIVING, RHO, p -> Double.POSITIVE_INFINITY).downforceN(), 0);
        RoadAero.Loads back = new RoadAero(List.of(d)).loads(DRIVING.scale(-1), RHO, p -> 0.1);
        assertEquals(Vec3d.ZERO, back.force(), "reversing: the floor, dam and splitter give nothing");
        assertEquals(0.75, RoadAero.fade(0.625), 1e-12);
    }

    @Test void downforceGoesWithTheDynamicPressure() {
        RoadAero r = new RoadAero(List.of(floor(10, 0.5, 0.04, 0.05, true), new RoadAero.Fence(new Vec3d(0.4, 1.0, 0), 1.4, 0.05),
                new RoadAero.Bluff(new Vec3d(2, 1.5, 0), 0.08, RoadAero.CD_TUBE)));
        RoadAero.Loads slow = r.loads(DRIVING.scale(0.5), RHO, p -> 0.1), fast = r.loads(DRIVING, RHO, p -> 0.1);
        assertEquals(4 * slow.downforceN(), fast.downforceN(), 1e-9 * fast.downforceN());
        assertEquals(4 * slow.dragN(), fast.dragN(), 1e-9 * fast.dragN());
    }

    @Test void aFenceAndABluffPiece() {
        RoadAero.Fence lip = new RoadAero.Fence(new Vec3d(0.3, 1.0, 0), 1.4, 0.05);
        RoadAero.Loads l = at(lip, 0.1);
        assertEquals(RoadAero.FENCE_LIFT * Q * 0.07, l.downforceN(), 1e-9);
        assertEquals(RoadAero.FENCE_DRAG * Q * 0.07, l.dragN(), 1e-9);
        double rearLift = l.downforceN() / (Q * FRONTAL);
        assertTrue(rearLift > 0.05 && rearLift < 0.2, "a 5 cm lip: some 0.1 off the rear lift (Hucho): " + rearLift);
        // roof bars: two 3 cm tubes 1.2 m long
        RoadAero.Bluff bars = new RoadAero.Bluff(new Vec3d(2, 1.5, 0), 2 * 0.03 * 1.2, RoadAero.CD_TUBE);
        RoadAero.Loads b = at(bars, 0.1);
        assertEquals(Q * 0.072 * RoadAero.CD_TUBE, b.dragN(), 1e-9);
        assertEquals(0, b.downforceN(), 1e-12);
        assertEquals(new Vec3d(-1, 0, 0), b.force().normalized(), "along the flow");
    }

    /** The test wing: 1.4 m by 0.3 m, NACA 6412-like (6 % camber, 12 % thick). */
    private static RoadAero.RearWing wing(double angleDeg, double plates, double heightM) {
        return new RoadAero.RearWing(new Vec3d(0.2, 1.2, 0), 1.4, 0.3, angleDeg, 0.06, 0.12, plates, heightM);
    }

    @Test void theRearWingIsTheLiftingLineUpsideDown() {
        RoadAero.RearWing w = wing(8, 0, 10);   // far above the ground: free air
        RoadAero.Loads l = at(w, 10);
        Wing free = new Wing(new Wing.Planform(1.4, 0.3, 0.3, 0, 0, 0, 0, true), Airfoil.naca4(0.06, 0.4, 0.12), Airfoil.naca4(0.06, 0.4, 0.12), List.of());
        double alpha = Math.toRadians(8);
        Wing.Solution s = free.solve(RoadAero.TABLE_SPEED, alpha, 0, Wing.Air.SEA_LEVEL, null);
        double lift = s.force().dot(new Vec3d(Math.sin(alpha), Math.cos(alpha), 0)), q = 0.5 * Wing.Air.SEA_LEVEL.densityKgM3() * RoadAero.TABLE_SPEED * RoadAero.TABLE_SPEED;
        assertEquals(lift / q, l.downforceN() / Q, 1e-3 * lift / q, "the free wing's lift, as downforce");
        assertTrue(l.force().y() < 0, "down");
        Vec3d arm = w.quarterChordM().cross(l.force());
        assertEquals(0, arm.subtract(l.moment()).length(), 1e-9, "acting at its quarter chord");
        double clw = l.downforceN() / (Q * w.areaM2());
        assertTrue(clw > 0.8 && clw < 1.2, "a cambered wing at 8 degrees: " + clw);
    }

    @Test void endPlatesAndStall() {
        double plain = at(wing(8, 0, 10), 10).downforceN(), plated = at(wing(8, 0.15, 10), 10).downforceN();
        assertTrue(plated > 1.02 * plain && plated < 1.15 * plain, "end plates lift the effective aspect ratio by 1.9 h / b: " + plated / plain);
        assertEquals(1 / (1 + 1.9 * 0.15 / 1.4), RoadAero.endplateFactor(0.15, 1.4), 1e-12);
        double best = 0;
        for (int a = -4; a <= 30; a++) { best = Math.max(best, at(wing(a, 0, 10), 10).downforceN()); }
        assertTrue(at(wing(28, 0, 10), 10).downforceN() < best, "past the stall it falls");
        assertTrue(at(wing(28, 0, 10), 10).dragN() > 3 * at(wing(8, 0, 10), 10).dragN(), "and the drag climbs");
        // close to the ground the image helps (Wieselsberger), high above it does not
        assertTrue(at(wing(8, 0, 0.3), 0.3).downforceN() > plain);
        assertEquals(0, FlightAtmosphere.groundInterference(10, 1.4), 0);
    }

    @Test void theWingSeesTheFlowAngle() {
        RoadAero r = new RoadAero(List.of(wing(8, 0, 10)));
        double level = r.loads(DRIVING, RHO, p -> 10).downforceN();
        // air rising past the car (its nose dropping) takes angle off an inverted wing
        double rising = r.loads(new Vec3d(-SPEED, SPEED * Math.tan(Math.toRadians(4)), 0), RHO, p -> 10).force().y();
        assertTrue(-rising < level, "less downforce with the flow 4 degrees up");
    }
}
