package dev.mcmod.aerocore.api.earthmoving;

import dev.mcmod.aerocore.api.GroundContact;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Materials, soil and rock cutting, buckets and blades against closed forms, hand sums and the handbook figures. */
class EarthmovingTest {
    private static final double G = Material.G;

    private static double rad(double deg) { return Math.toRadians(deg); }

    @Test void materialsSwellAndWeighAsTheHandbookSays() {
        // Caterpillar: earth, loam 23 % swell; broken granite 64 %; sand and gravel 12 %
        assertEquals(1.25, Material.LOAM.swell(), 0.03);
        assertTrue(Material.HARD_ROCK.swell() >= 1.5 && Material.HARD_ROCK.swell() <= 1.7);
        assertTrue(Material.BLASTED_ROCK.swell() >= 1.5 && Material.BLASTED_ROCK.swell() <= 1.7);
        assertEquals(1.13, Material.WET_SAND.swell(), 0.01);
        assertEquals(1540 * 9.80665, Material.LOAM.unitWeightNPerM3(), 1e-9);
        assertEquals(1.0 * Material.LOAM.swell(), Material.LOAM.looseM3(1.0), 0);
        // the soils are Wong's, the same as the wheels see
        assertEquals(GroundContact.Soil.SANDY_LOAM.cohesionKpa() * 1000, Material.LOAM.cohesionPa(), 1e-9);
        assertEquals(GroundContact.Soil.FIRM_CLAY.frictionAngle(), Material.STIFF_CLAY.frictionRad(), 0);
        assertEquals(GroundContact.Soil.DRY_SAND.cohesionKpa() * 1000, Material.DRY_SAND.cohesionPa(), 1e-9);
        // rock: c from Mohr-Coulomb gives back the strength
        Material rock = Material.HARD_ROCK;
        assertEquals(rock.compressivePa(), 2 * rock.cohesionPa() * Math.cos(rock.frictionRad()) / (1 - Math.sin(rock.frictionRad())), 1);
        for (Material m : Material.TYPICAL) {
            assertTrue(m.swell() >= 1 && m.swell() <= 1.8, m.name());
            assertTrue(m.wallFrictionRad() <= m.frictionRad(), m.name());
            // nothing wetter than saturated, nothing denser than its Proctor maximum
            if (m.moisture() > 0) { assertTrue(m.dryKgM3() <= Compaction.zeroAirVoidsKgM3(m.moisture(), Material.SOLIDS_KG_M3) + 1e-9, m.name()); }
            if (m.proctorKgM3() > 0) { assertTrue(m.relativeCompaction() <= 1, m.name()); }
        }
        // dumped material keeps its volume when dug again, and its mass
        Material fill = Material.LOAM.dumped();
        assertEquals(1, fill.swell(), 0);
        assertEquals(Material.LOAM.massKg(1), fill.massKg(Material.LOAM.looseM3(1)), 1e-9);
        assertEquals(fill, fill.dumped());
        // compacting raises the density and with it the swell
        Material rolled = Material.LOOSE_FILL.compacted(0.95);
        assertEquals(0.95, rolled.relativeCompaction(), 1e-12);
        assertTrue(rolled.swell() > 1.4);
        assertThrows(IllegalArgumentException.class, () -> Material.HARD_ROCK.compacted(0.9));
        assertThrows(IllegalArgumentException.class, () -> new Material("x", 1000, 1200, 0.5, 0, 0.5, 0.3, 0, 0, 0, 0, 0, 0));
    }

    @Test void aVerticalWallGivesCoulombsPassiveThrust() {
        // The case of the wedge (McKyes 1985, ch. 4) with a closed form: a vertical wall (alpha 90 deg) pushed into
        // cohesionless soil of 18 kN/m^3, phi 30 deg, wall friction 20 deg, 1 m deep and wide. The least force over the
        // failure angle is Coulomb's passive thrust 0.5 gamma d^2 K_p, K_p = cos^2 phi / (cos delta (1 - sqrt(sin(phi + delta) sin phi / cos delta))^2) = 6.105.
        double phi = rad(30), delta = rad(20);
        double kp = Math.pow(Math.cos(phi), 2) / (Math.cos(delta) * Math.pow(1 - Math.sqrt(Math.sin(phi + delta) * Math.sin(phi) / Math.cos(delta)), 2));
        assertEquals(6.105, kp, 1e-3);
        Cut wall = SoilCutting.wide(18e3, 0, phi, delta, 0, 0, rad(90), 1, 1, 0);
        assertEquals(0.5 * 18e3 * kp, wall.horizontalN() / Math.sin(rad(110)), 1e-6 * 0.5 * 18e3 * kp);
        // the thrust leans at delta: the soil pushes the wall up
        assertEquals(0.5 * 18e3 * kp * Math.cos(rad(70)), wall.liftN(), 1e-3);
        // smooth and cohesive: Rankine, 0.5 gamma d^2 K_p + 2 c d sqrt(K_p) on the plane at 45 - phi / 2
        double phi2 = rad(20), kp2 = Math.pow(Math.tan(Math.PI / 4 + phi2 / 2), 2);
        Cut rankine = SoilCutting.wide(18e3, 10e3, phi2, 0, 0, 0, rad(90), 1, 1, 0);
        assertEquals(0.5 * 18e3 * kp2 + 2 * 10e3 * Math.sqrt(kp2), rankine.horizontalN(), 1e-3);
        assertEquals(rad(45 - 10), rankine.failureAngleRad(), 1e-5);
        // the N factors on that plane: N_gamma = K_p / 2, N_c = 2 sqrt(K_p), N_q = K_p
        SoilCutting.Factors n = SoilCutting.factors(rad(90), 0, phi2, rad(35));
        assertEquals(kp2 / 2, n.nGamma(), 1e-12);
        assertEquals(2 * Math.sqrt(kp2), n.nCohesion(), 1e-12);
        assertEquals(kp2, n.nSurcharge(), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> SoilCutting.factors(rad(120), rad(20), rad(30), rad(20)));
    }

    @Test void aBladeInLoamTakesTheLeastForceAndIsDrawnInWhenSharp() {
        Material loam = Material.LOAM;
        double alpha = rad(45), d = 0.3, w = 1;
        Cut cut = SoilCutting.wide(loam, alpha, d, w, 0, 0);
        // by the factors at the angle found: H = F sin(alpha + delta) + c_a d w cot(alpha)
        double delta = loam.wallFrictionRad(), beta = cut.failureAngleRad();
        SoilCutting.Factors n = SoilCutting.factors(alpha, delta, loam.frictionRad(), beta);
        double f = (loam.unitWeightNPerM3() * d * d * n.nGamma() + loam.cohesionPa() * d * n.nCohesion() + loam.adhesionPa() * d * n.nAdhesion()) * w;
        assertEquals(f * Math.sin(alpha + delta) + loam.adhesionPa() * d * w / Math.tan(alpha), cut.horizontalN(), 1e-6 * cut.horizontalN());
        assertEquals(loam.adhesionPa() * d * w - f * Math.cos(alpha + delta), cut.liftN(), 1e-6 * cut.horizontalN());
        // a few kN per metre at 0.3 m: and the found plane is the weakest one
        assertTrue(cut.horizontalN() > 2e3 && cut.horizontalN() < 10e3, "H " + cut.horizontalN());
        for (double off : new double[]{-rad(2), rad(2)}) {
            SoilCutting.Factors m = SoilCutting.factors(alpha, delta, loam.frictionRad(), beta + off);
            double g = (loam.unitWeightNPerM3() * d * d * m.nGamma() + loam.cohesionPa() * d * m.nCohesion() + loam.adhesionPa() * d * m.nAdhesion()) * w;
            assertTrue(g > f);
        }
        // a low rake is drawn into the ground, a vertical blade pushed out; the steeper the blade, the more it takes
        assertTrue(SoilCutting.wide(loam, rad(30), d, w, 0, 0).liftN() < 0);
        assertTrue(SoilCutting.wide(loam, rad(90), d, w, 0, 0).liftN() > 0);
        double last = 0;
        for (int deg = 30; deg <= 130; deg += 10) {
            double h = SoilCutting.wide(loam, rad(deg), d, w, 0, 0).horizontalN();
            assertTrue(h > last, "rake " + deg);
            last = h;
        }
        // past 170 deg - delta - phi (136.7 deg for loam) the blade is held there: large, finite, the same
        double held = SoilCutting.wide(loam, rad(150), d, w, 0, 0).horizontalN();
        assertTrue(held > last && Double.isFinite(held));
        assertEquals(held, SoilCutting.wide(loam, rad(180), d, w, 0, 0).horizontalN(), 0);
        // the volume cut
        assertEquals(0.3 * 1 * 2, SoilCutting.wide(loam, alpha, d, w, 2, 0).bankM3PerS(), 1e-12);
    }

    @Test void narrowTinesPushMoreSoilThanTheirWidth() {
        Material loam = Material.LOAM;
        for (double deg : new double[]{30, 45, 60, 90}) {
            Cut wide = SoilCutting.wide(loam, rad(deg), 0.3, 1, 0, 0);
            Cut tine = SoilCutting.narrow(loam, rad(deg), 0.3, 0.1, 0, 0);
            // McKyes and Ali's side crescents: far more than a tenth of the metre-wide blade
            assertTrue(tine.horizontalN() > 1.5 * 0.1 * wide.horizontalN(), "rake " + deg);
            assertTrue(tine.disturbedWidthM() > 0.1);
        }
        // a 3 m "tine" is nearly a blade: the crescents add a little at the ends
        double wide = SoilCutting.wide(loam, rad(45), 0.3, 1, 0, 0).horizontalN();
        double broad = SoilCutting.narrow(loam, rad(45), 0.3, 3, 0, 0).horizontalN();
        assertTrue(broad > 3 * wide && broad < 1.2 * 3 * wide);
        // a vertical tine's crescents reach r = d (cot alpha + cot beta) sideways
        Cut vertical = SoilCutting.narrow(loam, rad(90), 0.3, 0.1, 0, 0);
        assertEquals(0.1 + 2 * 0.3 / Math.tan(vertical.failureAngleRad()), vertical.disturbedWidthM(), 1e-9);
    }

    @Test void harderGroundResistsMore() {
        double alpha = rad(50), d = 0.2, w = 2;
        double last = 0;
        for (double c = 0; c <= 100e3; c += 10e3) {
            double h = SoilCutting.wide(18e3, c, rad(30), rad(20), c / 2, 0, alpha, d, w, 0).horizontalN();
            assertTrue(h > last);
            last = h;
        }
        last = 0;
        for (double phi = 0; phi <= 60; phi += 5) {
            double h = SoilCutting.narrow(18e3, 5e3, rad(phi), rad(phi * 2 / 3), 0, 0, alpha, d, 0.1, 0).horizontalN();
            assertTrue(h > last, "phi " + phi);
            last = h;
        }
        last = 0;
        for (double gamma = 5e3; gamma <= 30e3; gamma += 5e3) {
            double h = SoilCutting.wide(gamma, 5e3, rad(30), rad(20), 0, 0, alpha, d, w, 0).horizontalN();
            assertTrue(h > last);
            last = h;
        }
        // the table in order of hardness, rock by specific energy: finite but beyond any machine
        double loam = Digging.edge(Material.LOAM, alpha, d, w, 1, 0).horizontalN();
        double clay = Digging.edge(Material.STIFF_CLAY, alpha, d, w, 1, 0).horizontalN();
        double soft = Digging.edge(Material.SOFT_ROCK, alpha, d, w, 1, 0).horizontalN();
        double hard = Digging.edge(Material.HARD_ROCK, alpha, d, w, 1, 0).horizontalN();
        assertTrue(loam < clay && clay < soft && soft < hard);
        assertEquals(150e6 / RockCutting.TYPICAL_EFFICIENCY * d * w, hard, 1e-6 * hard);
        assertTrue(Double.isFinite(hard) && hard > 1e8);
        // jointed rock is far easier (Hoek-Brown)
        assertTrue(Digging.edge(Material.HARD_ROCK.jointed(50), alpha, d, w, 1, 0).horizontalN() < 0.1 * hard);
        // a dumped heap is easier than the bank it came from
        assertTrue(Digging.edge(Material.STIFF_CLAY.dumped(), alpha, d, w, 1, 0).horizontalN() < clay);
    }

    @Test void speedAddsTheSoilsAcceleration() {
        Material loam = Material.LOAM;
        double alpha = rad(45);
        Cut still = SoilCutting.wide(loam, alpha, 0.3, 1, 0, 0);
        double last = still.horizontalN();
        for (double v : new double[]{0.5, 1, 2, 3, 5, 10}) {
            Cut moving = SoilCutting.wide(loam, alpha, 0.3, 1, v, 0);
            assertTrue(moving.horizontalN() > last);
            last = moving.horizontalN();
            // no more than the still failure plane with McKyes' N_a added (the least force can only be lower)
            SoilCutting.Factors n = SoilCutting.factors(alpha, loam.wallFrictionRad(), loam.frictionRad(), still.failureAngleRad());
            double bound = still.horizontalN() + loam.bankKgM3() * v * v * 0.3 * n.nAcceleration() * Math.sin(alpha + loam.wallFrictionRad());
            assertTrue(moving.horizontalN() <= bound * (1 + 1e-9));
        }
        // reversing is the same resistance
        assertEquals(SoilCutting.wide(loam, alpha, 0.3, 1, 2, 0).horizontalN(), SoilCutting.wide(loam, alpha, 0.3, 1, -2, 0).horizontalN(), 0);
    }

    @Test void aDozerBladeHoldsItsRatedLoad() {
        // SAE J1265: 0.8 W H^2, the prism at a repose of 32 deg; a D6-class straight blade 3.4 m x 1.3 m: 4 to 5 m^3 loose
        double rated = Blade.saeJ1265M3(3.4, 1.3);
        assertEquals(0.8 * 3.4 * 1.69, rated, 1e-12);
        assertTrue(rated > 4 && rated < 5);
        assertEquals(rated, Blade.prismM3(3.4, 1.3, rad(32)), 1e-3 * rated);
        double sand = Blade.prismM3(3.4, 1.3, Material.DRY_SAND.reposeRad());
        assertTrue(sand > 4 && sand < 5);
        // the prism's height comes back from its volume
        assertEquals(1.3, Blade.prismHeightM(Material.DRY_SAND, sand, 3.4), 1e-12);
        // pushing it and cutting 0.1 m of loam under it at 0.8 m/s: a fraction of a 20 t dozer's traction (0.9 of its weight)
        double push = Blade.pushN(Material.LOAM, rated);
        assertEquals(rated * 1250 * G * Math.tan(rad(37)), push, 1e-6);
        double q = Blade.surchargePa(Material.LOAM, 1.3);
        Cut cut = Digging.edge(Material.LOAM, rad(55), 0.1, 3.4, 0.8, q);
        assertTrue(cut.horizontalN() > Digging.edge(Material.LOAM, rad(55), 0.1, 3.4, 0.8, 0).horizontalN());
        assertTrue(push + cut.horizontalN() < 0.9 * 20e3 * G);
        // production: a 30 m push at 0.8 m/s, back at 2 m/s, 6 s of shifting: a few hundred loose m^3 an hour,
        // the scale of Caterpillar's dozing production curves for the class
        double cycle = 30 / 0.8 + 30 / 2.0 + 6;
        double perHour = rated * 3600 / cycle;
        assertTrue(perHour > 200 && perHour < 600, "m^3/h " + perHour);
        // cutting 0.1 m fills it in a dozen metres; past capacity it spills
        assertEquals(rated / (0.1 * 3.4 * Material.LOAM.swell()), Bucket.cutToFillM(Material.LOAM, rated, 1, 0.1, 3.4), 1e-9);
        assertEquals(0.4, Blade.spilledM3(rated + 0.4, rated), 1e-12);
        assertEquals(0, Blade.spilledM3(rated - 0.4, rated), 0);
    }

    @Test void bucketsAreRatedAsSaeJ296() {
        // a 1 m x 1.5 m opening with the SAE 1:1 heap: b^2 W / 4 - b^3 / 12
        assertEquals(1.5 / 4 - 1.0 / 12, Bucket.heapM3(1, 1.5, Bucket.SAE_HEAP_SLOPE), 1e-12);
        assertEquals(Bucket.heapM3(1, 1.5, 1), Bucket.heapM3(1.5, 1, 1), 1e-15);
        // the CECE 1:2 heap is half
        assertEquals(1.5 / 8 - 1.0 / 24, Bucket.heapM3(1, 1.5, Bucket.CECE_HEAP_SLOPE), 1e-12);
        // a square opening heaps to a pyramid of height b / 2: b^3 / 6
        assertEquals(1.0 / 6, Bucket.heapM3(1, 1, 1), 1e-12);
        double struck = Bucket.struckM3(0.8, 1.4);
        assertEquals(1.12 + Bucket.heapM3(1.1, 1.4, 1), Bucket.heapedM3(struck, 1.1, 1.4, 1), 1e-12);
        // loam fills it in a few metres of a 0.2 m cut; blasted rock carries less (Caterpillar fill factors)
        double cap = Bucket.heapedM3(struck, 1.1, 1.4, 1);
        assertEquals(cap * Bucket.FILL_LOAM / (0.2 * 1.4 * Material.LOAM.swell()), Bucket.cutToFillM(Material.LOAM, cap, Bucket.FILL_LOAM, 0.2, 1.4), 1e-9);
        assertTrue(Bucket.FILL_BLASTED_ROCK < Bucket.FILL_HARD_CLAY && Bucket.FILL_HARD_CLAY < Bucket.FILL_SAND_GRAVEL);
        assertEquals(Double.POSITIVE_INFINITY, Bucket.cutToFillM(Material.LOAM, cap, 1, 0, 1.4), 0);
        assertEquals(0, Bucket.cutToFillM(Material.LOAM, 0, 1, 0, 1.4), 0);
        // a scraper lifts its chip into the bowl on top of the cut
        double edge = Digging.edge(Material.LOAM, rad(30), 0.15, 3, 1.5, 0).horizontalN();
        assertEquals(edge + 1540 * G * 0.15 * 3 * 1.2, Bucket.scraperLoadingN(Material.LOAM, rad(30), 0.15, 3, 1.5, 1.2), 1e-6);
        // breakout: the cut plus a 2 t load 1.5 m from the pivot on a 1.6 m tip radius
        assertEquals(50e3 + 2000 * G * 1.5 / 1.6, Bucket.breakoutN(50e3, 2000, 1.5, 1.6), 1e-9);
    }

    @Test void rockIsCutBySpecificEnergy() {
        // Teale: efficient cutting needs about the compressive strength per unit volume; real tools about 1 / 0.35 of it
        assertEquals(150e6, RockCutting.specificEnergyPa(150e6, 1), 0);
        assertEquals(150e6 / 0.35, Digging.specificEnergyPa(Material.HARD_ROCK, rad(50), 0.05, 1), 1e-3);
        // soil's specific energy is that of its wedge: kPa, not MPa
        double soilSe = Digging.specificEnergyPa(Material.LOAM, rad(30), 0.1, 1);
        assertTrue(soilSe > 1e3 && soilSe < 1e5, "SE " + soilSe);
        // a cold planer drum 2 m wide, 0.1 m deep, advancing 10 m/min in asphalt: power = SE Q
        double se = RockCutting.specificEnergyPa(Material.ASPHALT.compressivePa(), RockCutting.TYPICAL_EFFICIENCY);
        RockCutting.Rotary drum = RockCutting.rotary(se, 2, 0.1, 10 / 60.0, 0.55, 2 * Math.PI * 100 / 60);
        assertEquals(se * 2 * 0.1 * 10 / 60.0, drum.powerW(), 1e-6);
        assertEquals(drum.powerW() / (2 * Math.PI * 100 / 60 * 0.55), drum.rimForceN(), 1e-6);
        assertEquals(drum.rimForceN() * 0.55, drum.torqueNm(), 1e-9);
        // hundreds of kW, as cold planers of that size have
        assertTrue(drum.powerW() > 100e3 && drum.powerW() < 1e6, "W " + drum.powerW());
        // a stopped drum ploughs like a blade of the cut's cross-section; a drum spinning in place cuts nothing
        assertEquals(se * 2 * 0.1, RockCutting.rotary(se, 2, 0.1, 0.2, 0.55, 0).rimForceN(), 1e-6);
        assertEquals(0, RockCutting.rotary(se, 2, 0.1, 0, 0.55, 10).powerW(), 0);
        // Evans' pick: 16 pi sigma_t^2 d^2 / (sigma_c cos^2(theta / 2)), granite at 10 mm with an 80 deg tip: about 8 kN
        double pick = RockCutting.pickForceN(12e6, 150e6, 0.01, rad(80));
        assertEquals(16 * Math.PI * 144e12 * 1e-4 / (150e6 * Math.pow(Math.cos(rad(40)), 2)), pick, 1e-6);
        assertEquals(4 * pick, RockCutting.pickForceN(12e6, 150e6, 0.02, rad(80)), 1e-6);
        // a 6 kJ hammer blow; jointed rock (GSI 50) breaks about 16 times more per blow
        double massive = RockCutting.blowM3(6000, Digging.specificEnergyPa(Material.HARD_ROCK, rad(60), 0.05, 0));
        double jointed = RockCutting.blowM3(6000, Digging.specificEnergyPa(Material.HARD_ROCK.jointed(50), rad(60), 0.05, 0));
        assertEquals(6000 * 0.35 / 150e6, massive, 1e-15);
        assertEquals(1 / RockCutting.rockMassFactor(50, 0), jointed / massive, 1e-9);
        // Hoek-Brown: s^a, 1 intact; by hand at GSI 75: s = exp(-25 / 9), a = 0.5 + (exp(-5) - exp(-20 / 3)) / 6
        assertEquals(1, RockCutting.rockMassFactor(100, 0), 1e-15);
        assertEquals(Math.pow(Math.exp(-25.0 / 9), 0.5 + (Math.exp(-5) - Math.exp(-20.0 / 3)) / 6), RockCutting.rockMassFactor(75, 0), 1e-15);
        assertTrue(RockCutting.rockMassFactor(50, 1) < RockCutting.rockMassFactor(50, 0));
        // the edge is pushed out of rock at a steep rake and drawn in at a sharp one, as in soil
        assertTrue(RockCutting.edge(se, rad(80), rad(20), rad(30), 0.05, 1, 1).liftN() > 0);
        assertTrue(RockCutting.edge(se, rad(30), rad(20), rad(30), 0.05, 1, 1).liftN() < 0);
    }

    @Test void aTbmDiscMeetsItsRatedLoad() {
        // CSM: a 432 mm (17 in) disc, 19 mm tip, 89 mm spacing, 5 mm a revolution in 150 MPa rock of 12 MPa Brazilian
        // strength: the normal force falls in the 150 to 300 kN of hard-rock boring (17 in discs are rated about
        // 250 to 270 kN; Bruland 1998, NTNU 1B-98), rolling coefficient about 0.1
        RockCutting.Disc disc = RockCutting.disc(150e6, 12e6, 0.432, 0.019, 0.089, 0.005, 0.1);
        assertTrue(disc.normalN() > 150e3 && disc.normalN() < 300e3, "Fn " + disc.normalN());
        assertEquals(Math.tan(disc.contactAngleRad() / 2), disc.rollingCoefficient(), 1e-12);
        assertTrue(disc.rollingCoefficient() > 0.08 && disc.rollingCoefficient() < 0.15);
        // by hand: phi = acos((R - p) / R), P' = C cbrt(S sc^2 st / (phi sqrt(R T))), F_t = T R phi P' / (1 + psi)
        double r = 0.216, phi = Math.acos((r - 0.005) / r);
        double p0 = 2.12 * Math.cbrt(0.089 * 150e6 * 150e6 * 12e6 / (phi * Math.sqrt(r * 0.019)));
        assertEquals(0.019 * r * phi * p0 / 1.1, disc.totalN(), 1e-6 * disc.totalN());
        // the disc's specific energy (rolling force over the groove's section) is of the order of the strength (Teale)
        double se = disc.rollingN() / (0.005 * 0.089);
        assertTrue(se > 0.1 * 150e6 && se < 3 * 150e6, "SE " + se);
        // no penetration, no force; deeper, more
        assertEquals(0, RockCutting.disc(150e6, 12e6, 0.432, 0.019, 0.089, 0, 0.1).totalN(), 0);
        assertTrue(RockCutting.disc(150e6, 12e6, 0.432, 0.019, 0.089, 0.008, 0.1).normalN() > disc.normalN());
        // a 7 m head with 45 discs: 0.3 D N F_r
        assertEquals(0.3 * 7 * 45 * disc.rollingN(), RockCutting.cutterheadTorqueNm(45, disc.rollingN(), 7), 1e-6);
        assertThrows(IllegalArgumentException.class, () -> RockCutting.disc(150e6, 12e6, 0.432, 0.019, 0.089, 0.3, 0.1));
    }
}
