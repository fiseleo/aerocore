package dev.mcmod.aerocore.api.wheel;

import dev.mcmod.aerocore.api.GroundContact;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Tyres (0.24) against published data (docs/WHEELS.md lists the sources). */
class TyreTest {
    private static final GroundContact.Surface TARMAC = GroundContact.Surface.TARMAC;

    private static Tyre road(double width, double aspect, double rimInches, double kpa) {
        return new Tyre(new Tyre.Spec(Tyre.Construction.RADIAL, width, aspect, rimInches * 0.0254, kpa * 1000, 300e3, Tyre.Tread.ROAD, 1));
    }

    private static void assertWithin(double lo, double hi, double value, String what) {
        assertTrue(value >= lo && value <= hi, what + ": " + value + " not in [" + lo + ", " + hi + "]");
    }

    @Test void aPassengerTyreHasPublishedStiffnesses() {
        Tyre t = road(0.205, 0.55, 16, 230);
        // Rhyne gives 238 N/mm for 205/55R16 at 230 kPa; Krmela et al. measured 251 N/mm
        assertEquals(238e3, t.verticalStiffness(0.01), 0.01 * 238e3, "Rhyne");
        assertEquals(251e3, t.verticalStiffness(0.01), 0.1 * 251e3, "measured");
        // nominal load where published Magic Formula sets for this size put F_z0 (3.3 and 4.25 kN; 4.85 kN for 245/40R18)
        assertWithin(3300, 4900, t.nominalLoadN(), "nominal load");
        // cornering stiffness at 4.25 kN: Continental 82.1, Michelin 63.7 kN/rad (Aertssen et al.)
        assertWithin(60e3, 90e3, t.corneringStiffness(4250), "cornering stiffness");
        // cornering coefficient 0.2-0.3 per degree near rated load, 0.3-0.4 at light load
        double rated = t.maxLoadN(), light = 0.35 * t.nominalLoadN();
        assertWithin(0.18, 0.32, Math.toRadians(t.corneringStiffness(rated) / rated), "CC at rated load");
        assertWithin(0.27, 0.45, Math.toRadians(t.corneringStiffness(light) / light), "CC at light load");
        // it saturates at 1.0-1.3 x rated load (TPT): no more than a few percent rise beyond 1.3 x rated
        assertTrue(t.corneringStiffness(1.6 * rated) < 1.05 * t.corneringStiffness(1.3 * rated), "saturation");
        // slip stiffness 18.5-25 F_z in published sets
        assertWithin(18, 25, t.slipStiffness(t.nominalLoadN()) / t.nominalLoadN(), "C_k / F_z");
        // lateral relaxation: Pacejka measured 0.56 / 0.66 / 0.69 m at 2.9 / 4.5 / 5.9 kN on a 195/60R14
        Tyre p = road(0.195, 0.60, 14, 200);
        double[][] relaxation = {{2900, 0.56}, {4500, 0.66}, {5900, 0.69}};
        for (double[] r : relaxation) { assertEquals(r[1], p.lateralRelaxationM(r[0]), 0.3 * r[1], "relaxation at " + r[0] + " N"); }
        assertTrue(p.longitudinalRelaxationM(4500) < p.lateralRelaxationM(4500), "braking relaxation shorter (TPT)");
        // mass of a 205/55R16 tyre is about 9 kg
        assertWithin(7.5, 10.5, t.massKg(), "mass");
    }

    @Test void frictionPeaksAndLocksAsWongsTableAndTheTyreBooksSay() {
        Tyre t = road(0.205, 0.55, 16, 230);
        double fz = t.nominalLoadN();
        // {surface, peak low, peak high, lock / peak low, high, peak slip low, high}: the peak slip is smaller where the friction
        // is low (the brush model's mu F_z / C_k); lab data put it at 0.07-0.15 on tarmac
        Object[][] cases = {
                {TARMAC, 0.85, 1.15, 0.66, 0.80, 0.07, 0.15},                    // Wong dry 0.8-0.9 (older tyres), good tyres above 1.0; TPT lock 0.70, Balkwill 0.75
                {GroundContact.Surface.WET_TARMAC, 0.5, 0.85, 0.75, 0.92, 0.05, 0.15}, // Wong wet asphalt 0.5-0.7 / 0.45-0.6
                {GroundContact.Surface.GRAVEL, 0.5, 0.65, 0.85, 0.97, 0.04, 0.15},     // Wong 0.6 / 0.55
                {GroundContact.Surface.SNOW, 0.18, 0.37, 0.68, 0.82, 0.01, 0.08},      // Wong hard-packed 0.2 / 0.15; NASA 0.24-0.37
                {GroundContact.Surface.ICE, 0.07, 0.13, 0.62, 0.78, 0.003, 0.04}};     // Wong 0.1 / 0.07
        for (Object[] c : cases) {
            GroundContact.Surface s = (GroundContact.Surface) c[0];
            double peak = 0, peakSlip = 0;
            for (double k = 0.0005; k <= 0.5; k += 0.0005) {
                double f = t.forces(s, fz, k, 0, 0, 10).longitudinalN() / fz;
                if (f > peak) { peak = f; peakSlip = k; }
            }
            double lock = -t.forces(s, fz, -1, 0, 0, 10).longitudinalN() / fz;
            assertWithin((double) c[1], (double) c[2], peak, s.name() + " peak");
            assertWithin((double) c[3], (double) c[4], lock / peak, s.name() + " lock over peak");
            assertWithin((double) c[5], (double) c[6], peakSlip, s.name() + " peak slip ratio");
        }
        // peak slip angle 6-10 degrees for road tyres (Blundell and Harty)
        double peak = 0, angle = 0;
        for (double a = 0.5; a <= 20; a += 0.25) {
            double f = t.forces(TARMAC, fz, 0, -Math.tan(Math.toRadians(a)), 0, 10).lateralN();
            if (-f > peak) { peak = -f; angle = a; }
        }
        assertWithin(5, 10, angle, "peak slip angle");
        // load sensitivity: published sets lose 3.4 %/kN near 4 kN
        double perKn = (t.peakFriction(TARMAC, fz - 500, 10) - t.peakFriction(TARMAC, fz + 500, 10)) / t.peakFriction(TARMAC, fz, 10);
        assertWithin(0.02, 0.05, perKn, "friction loss per kN");
    }

    @Test void combinedSlipStaysInsideTheFrictionEllipse() {
        Tyre t = road(0.225, 0.45, 17, 250);
        Random random = new Random(3);
        double fz = 5000, mu = t.peakFriction(TARMAC, fz, 20);
        for (int i = 0; i < 2000; i++) {
            double sx = 2 * random.nextGaussian() * 0.2, sy = random.nextGaussian() * 0.3;
            Tyre.Forces f = t.forces(TARMAC, fz, sx, sy, 0, 20);
            assertTrue(Math.hypot(f.longitudinalN(), f.lateralN()) <= mu * fz * (1 + 1e-9), "inside the friction circle at " + sx + ", " + sy);
            assertTrue(f.longitudinalN() * sx >= 0 && f.lateralN() * sy >= 0, "forces along the slips");
        }
        // small slips: the stiffnesses, unchanged by the other slip
        Tyre.Forces small = t.forces(TARMAC, fz, 1e-5, -2e-5, 0, 20);
        assertEquals(t.slipStiffness(fz) * 1e-5, small.longitudinalN(), 1e-3 * t.slipStiffness(fz) * 1e-5);
        assertEquals(-t.corneringStiffness(fz) * 2e-5, small.lateralN(), 1e-3 * t.corneringStiffness(fz) * 2e-5);
        // the aligning moment's trail is a sixth of the contact length at small slip
        assertEquals(-t.contactLengthM(fz) / 6, small.aligningNm() / small.lateralN(), 1e-3 * t.contactLengthM(fz), "behind the centre: it turns the wheel toward its path");
    }

    @Test void rollingResistanceFollowsSaeJ2452AndSoftGround() {
        Tyre t = road(0.205, 0.65, 15, 240);
        // NREL's check of Michelin's coefficients: 8.8 N/kN at 4 kN, 240 kPa, 80 km/h
        assertEquals(8.8, t.rollingResistanceN(TARMAC, 4000, 80 / 3.6) / 4.0, 0.1);
        // EU-labelled passenger tyres span 6.5-12 N/kN
        assertWithin(6.5, 12, t.rollingResistanceN(TARMAC, 4000, 20) / 4.0, "label range");
        // under-inflation raises it about as (P / P0)^-0.4 (TPT)
        Tyre soft = road(0.205, 0.65, 15, 160);
        assertEquals(Math.pow(160.0 / 240, -0.3947), soft.rollingResistanceN(TARMAC, 4000, 20) / t.rollingResistanceN(TARMAC, 4000, 20), 1e-9);
        // a passenger car in sand: Wong's 0.30 of the load
        assertWithin(0.2, 0.4, t.rollingResistanceN(GroundContact.Surface.SAND, 4000, 5) / 4000, "sand");
    }

    @Test void wetGripFallsAwayAtHornesHydroplaningSpeed() {
        Tyre slick = new Tyre(new Tyre.Spec(Tyre.Construction.RADIAL, 0.205, 0.55, 16 * 0.0254, 220e3, 300e3, Tyre.Tread.SLICK, 1));
        // 9 sqrt(p psi) knots: 220 kPa is 94 km/h
        assertEquals(94 / 3.6, slick.hydroplaningSpeedMps(), 0.01 * 94 / 3.6);
        GroundContact.Surface wet = GroundContact.Surface.WET_TARMAC;
        double vp = slick.hydroplaningSpeedMps(), fz = slick.nominalLoadN();
        assertEquals(slick.peakFriction(wet, fz, 1), slick.peakFriction(wet, fz, 0.5 * vp), 1e-12, "no loss well below");
        assertTrue(slick.peakFriction(wet, fz, 1.05 * vp) < 0.1 * slick.peakFriction(wet, fz, 1), "almost nothing above V_p");
        assertEquals(slick.peakFriction(TARMAC, fz, 1), slick.peakFriction(TARMAC, fz, 1.05 * vp), 1e-12, "dry ground does not hydroplane");
        assertTrue(road(0.205, 0.55, 16, 220).hydroplaningSpeedMps() > slick.hydroplaningSpeedMps(), "grooves drain");
    }

    @Test void aCessnaMainTyreMatchesSmileyAndHorne() {
        // 6.00-6 Type III: section width 6.3 in, outside diameter 17.5 in, rim 6 in, 42 psi rated and used (Goodyear)
        double width = 6.3 * 0.0254, rim = 6 * 0.0254, height = (17.5 * 0.0254 - rim) / 2, psi = 6894.76;
        Tyre t = new Tyre(new Tyre.Spec(Tyre.Construction.AIRCRAFT, width, height / width, rim, 42 * psi, 42 * psi, Tyre.Tread.AIRCRAFT_RIB, 1));
        double lb = 4.44822;
        // Goodyear rates it at 1750 lb; Smiley and Horne's rated relation gives 1584 lb
        assertEquals(1750 * lb, t.nominalLoadN(), 0.12 * 1750 * lb);
        // the worked example: 1.15 in at 1100 lb, cornering power 30 kN/rad, relaxation length 8.9 in
        assertEquals(1.15 * 0.0254, t.deflectionM(1100 * lb), 0.1 * 1.15 * 0.0254);
        assertEquals(30e3, t.corneringStiffness(1100 * lb), 0.15 * 30e3);
        assertEquals(1.72 * 0.0254, t.deflectionM(1750 * lb), 0.1 * 1.72 * 0.0254, "1.72 in at rated load");
        assertWithin(4, 6.5, t.massKg(), "mass");
    }

    @Test void forcesBuildUpOverTheRelaxationLength() {
        Tyre t = road(0.205, 0.55, 16, 230);
        double fz = 4000, speed = 10, dt = 0.001, angle = Math.toRadians(0.25), vy = speed * Math.tan(angle);
        double sigma = t.lateralRelaxationM(fz), steady = TyreContact.step(t, TARMAC, fz, speed, vy, speed, 0, 0, -sigma * vy / speed, dt).lateralN();
        double u = 0, v = 0, travelled = 0;
        while (travelled < sigma) {
            double[] next = TyreContact.deflections(t, fz, speed, vy, speed, u, v, dt);
            u = next[0]; v = next[1];
            travelled += speed * dt;
        }
        double force = TyreContact.step(t, TARMAC, fz, speed, vy, speed, 0, u, v, 0).lateralN();
        assertEquals(1 - Math.exp(-1), force / steady, 0.01, "63 % after one relaxation length");
    }

    @Test void aBrakedWheelHoldsOnASlopeAsASpringWithoutJitter() {
        // a quarter of a 1500 kg car on a 30 % slope, its wheel held by the brake: 30 s, as the vehicle step takes it
        Tyre t = road(0.205, 0.55, 16, 230);
        double m = 375, g = 9.80665, slope = Math.atan(0.3), fz = m * g * Math.cos(slope), pull = -m * g * Math.sin(slope), dt = 0.005;
        double x = 0, v = 0, u = 0, peak = 0;
        for (int i = 0; i < 6000; i++) {
            TyreContact.Step s = TyreContact.step(t, TARMAC, fz, v, 0, 0, 0, u, 0, dt);
            // implicit in the body's speed through the line damper: m (v' - v) = dt (F0 + pull - k (v' - v))
            double next = v + dt * (s.longitudinalN() + pull) / (m + dt * s.longitudinalPerMps());
            u = TyreContact.deflections(t, fz, next, 0, 0, u, 0, dt)[0];
            x += next * dt;
            v = next;
            if (i > 3000) { peak = Math.max(peak, Math.abs(v)); }
        }
        assertTrue(Math.abs(x) < 0.01, "crept " + x + " m");
        assertTrue(peak < 1e-4, "still: " + peak + " m/s");
        // on ice it slides
        double xi = 0, vi = 0, ui = 0;
        for (int i = 0; i < 400; i++) {
            TyreContact.Step s = TyreContact.step(t, GroundContact.Surface.ICE, fz, vi, 0, 0, 0, ui, 0, dt);
            double next = vi + dt * (s.longitudinalN() + pull) / (m + dt * s.longitudinalPerMps());
            ui = TyreContact.deflections(t, fz, next, 0, 0, ui, 0, dt)[0];
            xi += next * dt;
            vi = next;
        }
        assertTrue(xi < -0.5, "slides on ice: " + xi);
    }

    @Test void theContactStaysFiniteForAnyInput() {
        Tyre t = road(0.265, 0.75, 16, 300);
        Random random = new Random(11);
        GroundContact.Surface[] surfaces = {TARMAC, GroundContact.Surface.WET_TARMAC, GroundContact.Surface.SAND, GroundContact.Surface.MUD,
                GroundContact.Surface.SNOW, GroundContact.Surface.ICE, GroundContact.Surface.GRAVEL, GroundContact.Surface.SOIL};
        for (int i = 0; i < 5000; i++) {
            GroundContact.Surface s = surfaces[random.nextInt(surfaces.length)];
            double fz = random.nextInt(5) == 0 ? 0 : 20000 * random.nextDouble(), vx = random.nextInt(4) == 0 ? 0 : 80 * random.nextGaussian();
            TyreContact.Step c = TyreContact.step(t, s, fz, vx, 10 * random.nextGaussian(), 80 * random.nextGaussian(), 0.2 * random.nextGaussian(),
                    random.nextGaussian(), random.nextGaussian(), 0.005);
            for (double d : new double[]{c.longitudinalN(), c.lateralN(), c.aligningNm(), c.longitudinalPerMps(), c.lateralPerMps(), c.rollingResistanceN()}) {
                assertTrue(Double.isFinite(d), "finite: " + c);
            }
            assertTrue(c.longitudinalPerMps() >= 0 && c.lateralPerMps() >= 0, "damping slopes only");
        }
    }
}
