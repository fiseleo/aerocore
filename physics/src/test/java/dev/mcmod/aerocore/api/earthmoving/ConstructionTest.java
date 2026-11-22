package dev.mcmod.aerocore.api.earthmoving;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Compaction, paving, concrete and piling against their sources' figures; every function finite and continuous over sweeps. */
class ConstructionTest {
    private static final double G = Material.G, C = 273.15;

    private static double rad(double deg) { return Math.toRadians(deg); }

    @Test void rollingApproachesButNeverPassesProctor() {
        // dumped loam (64 % of Proctor) under the reference roller: 95 % after the typical 4 passes, then ever closer to 100 %
        Material fill = Material.LOOSE_FILL;
        double loose = Compaction.looseRelative(Material.LOAM);
        assertEquals(fill.relativeCompaction(), loose, 1e-12);
        double nc = Compaction.passConstant(loose, Compaction.PASSES_TO_95_GRANULAR), limit = Compaction.limit(fill);
        assertEquals(1, limit, 0);
        assertEquals(0.95, Compaction.afterPasses(loose, 4, nc, limit), 1e-12);
        double last = loose;
        for (double n = 0.25; n <= 40; n += 0.25) {
            double rc = Compaction.afterPasses(loose, n, nc, limit);
            assertTrue(rc > last && rc < 1, "passes " + n);
            last = rc;
        }
        // a pass in parts is the same as the whole: substeps can roll a share of a pass each
        assertEquals(Compaction.afterPasses(loose, 4, nc, limit), Compaction.afterPasses(Compaction.afterPasses(loose, 1.5, nc, limit), 2.5, nc, limit), 1e-12);
        // never undone
        assertEquals(0.98, Compaction.afterPasses(0.98, 3, nc, 0.9), 0);
        // wet clay and mud cannot be squeezed past saturation: rho_zav = rho_s / (1 + w rho_s / rho_w)
        assertEquals(2700 / 1.54, Compaction.zeroAirVoidsKgM3(0.2, 2700), 1e-9);
        double clay = Compaction.limit(Material.SOFT_CLAY);
        assertEquals(2700 / (1 + 0.38 * 2.7) / 1440, clay, 1e-12);
        assertTrue(Compaction.afterPasses(Material.SOFT_CLAY.relativeCompaction(), 50, nc, clay) <= clay);
        assertTrue(Compaction.limit(Material.MUD) - Material.MUD.relativeCompaction() < 0.01);
        // the reference roller on its lift needs the typical passes
        double vibrating = Compaction.linearLoadNPerM(28e3, 266e3, 2.13, 31, 1.0);
        assertEquals(Compaction.REFERENCE_LINEAR_LOAD_N_PER_M, vibrating, 1e-9);
        assertEquals(4, Compaction.passesTo95(4, Compaction.LIFT_GRANULAR_M, 0.3, vibrating), 1e-12);
        // without vibration, travelling fast or on a thicker lift it takes more passes: never less density for the same passes
        double still = Compaction.linearLoadNPerM(28e3, 266e3, 2.13, 0, 1.0);
        double fast = Compaction.linearLoadNPerM(28e3, 266e3, 2.13, 31, 3.0);
        assertEquals(28e3, still, 0);
        assertEquals(28e3 + 266e3 / 2.13 * 31 / 90, fast, 1e-6);
        double withVibration = Compaction.afterPasses(loose, 4, nc, limit);
        double without = Compaction.afterPasses(loose, 4, Compaction.passConstant(loose, Compaction.passesTo95(4, 0.3, 0.3, still)), limit);
        double thick = Compaction.afterPasses(loose, 4, Compaction.passConstant(loose, Compaction.passesTo95(4, 0.3, 0.5, vibrating)), limit);
        double quick = Compaction.afterPasses(loose, 4, Compaction.passConstant(loose, Compaction.passesTo95(4, 0.3, 0.3, fast)), limit);
        assertTrue(without < withVibration && thick < withVibration && quick < withVibration && without < quick);
        assertThrows(IllegalArgumentException.class, () -> Compaction.passConstant(0.96, 4));
    }

    @Test void aThinMatCoolsToCessationInMinutes() {
        // 50 mm laid at 145 C on a 10 C base in 10 C air and a 4 m/s wind: the cold-weather guides after Corlew and
        // Dickson allow about 15 minutes for a 50 mm lift on a 10 to 15 C base; this gives about 20
        double t50 = Asphalt.secondsToK(0.05, 145 + C, 10 + C, 10 + C, 4, 0, Asphalt.CESSATION_K);
        assertTrue(t50 > 10 * 60 && t50 < 25 * 60, "minutes " + t50 / 60);
        // thinner lifts cool much faster (the guides: 4 to 8 minutes for 25 mm), thicker slower
        double t25 = Asphalt.secondsToK(0.025, 145 + C, 10 + C, 10 + C, 4, 0, Asphalt.CESSATION_K);
        double t75 = Asphalt.secondsToK(0.075, 145 + C, 10 + C, 10 + C, 4, 0, Asphalt.CESSATION_K);
        assertTrue(t25 > 4 * 60 && t25 < 10 * 60, "minutes " + t25 / 60);
        assertTrue(t75 > 1.5 * t50);
        // wind and a cold base shorten it, sunshine lengthens it
        assertTrue(Asphalt.secondsToK(0.05, 145 + C, 10 + C, 10 + C, 8, 0, Asphalt.CESSATION_K) < t50);
        assertTrue(Asphalt.secondsToK(0.05, 145 + C, C, 10 + C, 4, 0, Asphalt.CESSATION_K) < t50);
        assertTrue(Asphalt.secondsToK(0.05, 145 + C, 10 + C, 10 + C, 4, 800, Asphalt.CESSATION_K) > t50);
        // continuous: a tenth of a millimetre or a hundredth of a degree moves it a little
        assertEquals(t50, Asphalt.secondsToK(0.0501, 145 + C, 10 + C, 10 + C, 4, 0, Asphalt.CESSATION_K), 0.01 * t50);
        assertEquals(t50, Asphalt.secondsToK(0.05, 145 + C, 10 + C, 10.01 + C, 4, 0, Asphalt.CESSATION_K), 0.001 * t50);
        assertEquals(0, Asphalt.secondsToK(0.05, 70 + C, 10 + C, 10 + C, 4, 0, Asphalt.CESSATION_K), 0);
        // the step: 5 s is within a few tenths of a kelvin of 1 s after 10 minutes, 30 s within a kelvin
        double[] mean = new double[3];
        double[] steps = {1, 5, 30};
        for (int i = 0; i < 3; i++) {
            Asphalt.Layer mat = Asphalt.mat(0.05, 145 + C, 10 + C);
            double lastMean = mat.meanK();
            for (int s = 0; s < 600 / steps[i]; s++) {
                mat.step(steps[i], 10 + C, 4, 0);
                assertTrue(mat.meanK() < lastMean);
                assertTrue(mat.surfaceK() < mat.meanK());
                lastMean = mat.meanK();
            }
            assertEquals(600, mat.elapsedS(), 1e-9);
            mean[i] = mat.meanK();
        }
        assertEquals(mean[0], mean[1], 0.2);
        assertEquals(mean[0], mean[2], 1);
        assertEquals(mean[1], Asphalt.matMeanK(0.05, 145 + C, 10 + C, 10 + C, 4, 0, 600), 1e-9);
    }

    @Test void aTruckLoadStaysHotForAnHour() {
        // a metre-deep load at 160 C loses its heat through the crust: still above 145 C after an hour on a bare bed,
        // warmer under a tarp, while a 50 mm mat is down to about 40 C
        double bare = Asphalt.loadMeanK(1, 160 + C, 0, 0, 10 + C, 5, 0, 3600);
        double tarped = Asphalt.loadMeanK(1, 160 + C, 0.1, 0, 10 + C, 5, 0, 3600);
        double insulated = Asphalt.loadMeanK(1, 160 + C, 0.1, 0.2, 10 + C, 5, 0, 3600);
        assertTrue(bare > 145 + C && bare < 160 + C, "C " + (bare - C));
        assertTrue(tarped > bare && insulated > tarped);
        double mat = Asphalt.matMeanK(0.05, 145 + C, 10 + C, 10 + C, 4, 0, 3600);
        assertTrue(mat < 60 + C, "C " + (mat - C));
        assertEquals(160 + C, Asphalt.loadMeanK(1, 160 + C, 0, 0, 10 + C, 5, 0, 0), 1e-9);
    }

    @Test void concreteSetsInHoursAndSlowerWhenCold() {
        // Nurse-Saul with the -10 C datum (ASTM C1074): initial set at 4 h at 20 C, twice that at 5 C, never below -10 C
        assertEquals(4 * 3600, Concrete.secondsTo(Concrete.INITIAL_SET_KS, 20 + C), 1e-6);
        assertEquals(8 * 3600, Concrete.secondsTo(Concrete.INITIAL_SET_KS, 5 + C), 1e-6);
        assertEquals(Double.POSITIVE_INFINITY, Concrete.secondsTo(Concrete.INITIAL_SET_KS, Concrete.DATUM_K), 0);
        assertEquals(Double.POSITIVE_INFINITY, Concrete.secondsTo(Concrete.INITIAL_SET_KS, -20 + C), 0);
        double m = 0;
        for (int minute = 0; minute < 234; minute++) { m = Concrete.maturityKs(m, 20 + C, 60); }
        assertTrue(!Concrete.initiallySet(m));
        for (int minute = 0; minute < 12; minute++) { m = Concrete.maturityKs(m, 20 + C, 60); }
        assertTrue(Concrete.initiallySet(m) && !Concrete.finallySet(m));
        assertEquals(0, Concrete.strengthPa(m, 30e6), 0);
        // frozen below the datum: nothing added
        assertEquals(m, Concrete.maturityKs(m, -15 + C, 3600), 0);
        // ACI 209 (type I, moist): 28 days at 20 C within 1 % of f_28, 7 days about 70 %, colder less, always growing
        double day = 30 * 86400;
        assertEquals(30e6, Concrete.strengthPa(28 * day, 30e6), 0.01 * 30e6);
        double week = Concrete.strengthPa(7 * day, 30e6) / 30e6;
        assertTrue(week > 0.6 && week < 0.75, "7 days " + week);
        assertTrue(Concrete.strengthPa(Concrete.maturityKs(0, 5 + C, 7 * 86400), 30e6) < 0.6 * 30e6);
        double last = -1;
        for (double d = 0; d <= 90; d += 0.5) {
            double f = Concrete.strengthPa(d * day, 30e6);
            assertTrue(f >= last && f < 30e6 / 0.85);
            last = f;
        }
        assertEquals(day, Concrete.equivalentAgeS(day) * 30, 1e-6);
    }

    @Test void pilesGoInLessAgainstMoreResistance() {
        // Hiley by hand: a 5 t ram dropping 1 m at 0.8 on a 3 t pile, restitution 0.4, 15 mm temporary compression,
        // against 1 MN: eta = (5 + 0.16 x 3) / 8 = 0.685, s = 0.8 x 5000 g x 0.685 / 1e6 - 0.0075 = 19.4 mm
        assertEquals(0.685, Piling.blowEfficiency(5000, 3000, 0.4), 1e-12);
        double set = Piling.hileySetM(1e6, 5000, 1, 0.8, 3000, 0.4, 0.015);
        assertEquals(0.8 * 5000 * G * 0.685 / 1e6 - 0.0075, set, 1e-12);
        assertEquals(1e6, Piling.hileyResistanceN(set, 5000, 1, 0.8, 3000, 0.4, 0.015), 1e-3);
        double last = Double.POSITIVE_INFINITY;
        for (double r = 0.1e6; r <= 5e6; r += 0.1e6) {
            double s = Piling.hileySetM(r, 5000, 1, 0.8, 3000, 0.4, 0.015);
            assertTrue(s < last || s == 0);
            last = s;
        }
        assertEquals(0, Piling.hileySetM(5e6, 5000, 1, 0.8, 3000, 0.4, 0.015), 0);
        assertEquals(Double.POSITIVE_INFINITY, Piling.hileySetM(0, 5000, 1, 0.8, 3000, 0.4, 0.015), 0);
        // the blow efficiency's two branches meet where the ram weighs n times the pile
        assertEquals(Piling.blowEfficiency(1200 - 1e-6, 3000, 0.4), Piling.blowEfficiency(1200 + 1e-6, 3000, 0.4), 1e-9);
        assertTrue(Piling.blowEfficiency(600, 3000, 0.4) < Piling.blowEfficiency(1200, 3000, 0.4));
        // ENR: R = W h / (s + 25.4 mm)
        assertEquals(5000 * G / 1e6 - 0.0254, Piling.enrSetM(1e6, 5000, 1, Piling.ENR_DROP_M), 1e-12);
        // the textbook bearing factors at 30 deg: N_q 18.40, N_c 30.14; N_c 5.14 at 0
        assertEquals(18.40, Piling.bearingNq(rad(30)), 0.01);
        assertEquals(30.14, Piling.bearingNc(rad(30)), 0.01);
        assertEquals(Math.PI + 2, Piling.bearingNc(0), 1e-12);
        assertEquals(Piling.bearingNc(1e-4 - 1e-12), Piling.bearingNc(1e-4), 1e-6);
        // a 0.3 m concrete pile 10 m into loam: a few hundred kN, a 3 t hammer drives it some 50 mm a blow; granite stops it
        double loam = Piling.staticResistanceN(Material.LOAM, 1.2, 0.09, 10);
        assertTrue(loam > 150e3 && loam < 500e3, "N " + loam);
        assertTrue(Piling.staticResistanceN(Material.LOAM, 1.2, 0.09, 12) > loam);
        double pile = 0.09 * 10 * 2400;
        double s = Piling.hileySetM(loam, 3000, 1, 0.8, pile, 0.4, 0.015);
        assertTrue(s > 0.02 && s < 0.1, "m " + s);
        assertEquals(0, Piling.hileySetM(Piling.staticResistanceN(Material.HARD_ROCK, 1.2, 0.09, 10), 3000, 1, 0.8, pile, 0.4, 0.015), 0);
        assertEquals(0, Piling.staticResistanceN(Material.GRAVEL, 1.2, 0.09, 0), 0);
        // elastic shortening R L / (A E)
        assertEquals(1e6 * 10 / (0.09 * 30e9), Piling.elasticCompressionM(1e6, 10, 0.09, 30e9), 1e-15);
    }

    @Test void nothingIsNaNAnywhere() {
        double[] rakes = {0, 1, 5, 30, 45, 60, 90, 120, 150, 179, 180};
        double[] depths = {0, 1e-9, 1e-3, 0.05, 0.3, 2};
        double[] widths = {0, 1e-3, 0.05, 1, 4};
        double[] speeds = {0, -1, 0.5, 5, 30};
        for (Material m : Material.TYPICAL) {
            for (double a : rakes) {
                for (double d : depths) {
                    for (double w : widths) {
                        for (double v : speeds) {
                            for (Cut c : new Cut[]{Digging.edge(m, rad(a), d, w, v, 5e3), Digging.tooth(m, rad(a), d, w, v),
                                    SoilCutting.wide(m, rad(a), d, w, v, 0), SoilCutting.narrow(m, rad(a), d, w, v, 0)}) {
                                assertTrue(Double.isFinite(c.horizontalN()) && c.horizontalN() >= 0, m.name());
                                assertTrue(Double.isFinite(c.liftN()) && Double.isFinite(c.failureAngleRad()) && Double.isFinite(c.bankM3PerS())
                                        && Double.isFinite(c.disturbedWidthM()), m.name());
                                if (d == 0) { assertEquals(0, c.horizontalN(), 0); }
                            }
                        }
                    }
                }
                assertTrue(Double.isFinite(Digging.specificEnergyPa(m, rad(a), 0, 0)));
                assertTrue(Double.isFinite(Bucket.scraperLoadingN(m, rad(a), 0.1, 3, 0, 1)));
            }
            assertTrue(Double.isFinite(Blade.pushN(m, 0)) && Double.isFinite(Blade.surchargePa(m, 0)) && Double.isFinite(Blade.prismHeightM(m, 0, 1)));
            assertTrue(Double.isFinite(Piling.staticResistanceN(m, 1.2, 0.09, 0)) && Double.isFinite(Piling.staticResistanceN(m, 1.2, 0.09, 30)));
            assertTrue(Double.isFinite(m.dumped().swell()) && Double.isFinite(m.jointed(10).compressivePa()));
            if (m.proctorKgM3() > 0) { assertTrue(Double.isFinite(Compaction.limit(m))); }
        }
        RockCutting.Rotary zero = RockCutting.rotary(0, 0, 0, 0, 0, 0);
        assertEquals(0, zero.powerW() + zero.torqueNm() + zero.rimForceN() + zero.removalM3PerS(), 0);
        assertTrue(Double.isFinite(RockCutting.rotary(1e8, 2, 0.1, 0.2, 0, 0).rimForceN()));
        RockCutting.Disc none = RockCutting.disc(0, 0, 0.432, 0.019, 0.089, 0, 0);
        assertEquals(0, none.totalN() + none.rollingCoefficient(), 0);
        assertTrue(Double.isFinite(RockCutting.disc(300e6, 20e6, 0.432, 0.019, 0.089, 0.216, -0.2).normalN()));
        assertTrue(Double.isFinite(Asphalt.secondsToK(0.3, 160 + C, -20 + C, -20 + C, 20, 0, Asphalt.CESSATION_K)));
        assertTrue(Double.isFinite(Asphalt.secondsToK(0.01, 160 + C, 40 + C, 40 + C, 0, 1000, Asphalt.CESSATION_K)));
        assertTrue(Double.isFinite(Compaction.afterPasses(0.7, 3, Double.POSITIVE_INFINITY, 1)));
        assertTrue(Double.isFinite(Concrete.strengthPa(0, 30e6)));
        // what is not a number is refused
        assertThrows(IllegalArgumentException.class, () -> Digging.edge(Material.LOAM, Double.NaN, 0.1, 1, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> Digging.edge(Material.LOAM, 1, -0.1, 1, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> Digging.edge(Material.HARD_ROCK, 1, 0.1, Double.POSITIVE_INFINITY, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> Digging.tooth(Material.LOAM, 1, 0.1, 0.1, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> SoilCutting.wide(Material.LOAM, 1, 0.1, 1, 1, -1));
        assertThrows(IllegalArgumentException.class, () -> Asphalt.mat(0, 400, 300));
        assertThrows(IllegalArgumentException.class, () -> Concrete.maturityKs(0, Double.NaN, 1));
    }

    @Test void cuttingIsContinuous() {
        // a rake swept by a tenth of a degree, from flat to past vertical, never jumps (5 % at most where the force
        // climbs steeply towards the steepest blade held)
        for (Material m : new Material[]{Material.LOAM, Material.DRY_SAND, Material.STIFF_CLAY, Material.SNOW}) {
            double wide = 0, tine = 0;
            for (int i = 0; i <= 1800; i++) {
                double a = rad(i / 10.0);
                double w = Digging.edge(m, a, 0.3, 1, 1, 2e3).horizontalN(), t = Digging.tooth(m, a, 0.3, 0.1, 1).horizontalN();
                if (i > 0) {
                    assertTrue(w / wide < 1.05 && wide / w < 1.05, m.name() + " wide at " + i / 10.0);
                    assertTrue(t / tine < 1.05 && tine / t < 1.05, m.name() + " tine at " + i / 10.0);
                }
                wide = w;
                tine = t;
            }
        }
        // depth and speed by small steps: small changes
        double top = Digging.edge(Material.LOAM, rad(45), 0.5, 1, 1, 0).horizontalN(), last = 0;
        for (int i = 0; i <= 500; i++) {
            double h = Digging.edge(Material.LOAM, rad(45), i / 1000.0, 1, 1, 0).horizontalN();
            assertTrue(Math.abs(h - last) < 0.01 * top && h >= last);
            last = h;
        }
        last = Digging.tooth(Material.WET_SAND, rad(60), 0.3, 0.1, 0).horizontalN();
        for (int i = 1; i <= 500; i++) {
            double h = Digging.tooth(Material.WET_SAND, rad(60), 0.3, 0.1, i / 100.0).horizontalN();
            assertTrue(h >= last && h - last < 0.01 * last);
            last = h;
        }
        // the rock edge's lift turns over smoothly with the rake
        double lift = RockCutting.edge(1e8, 0, rad(20), rad(30), 0.05, 1, 1).liftN();
        for (int i = 1; i <= 1800; i++) {
            double l = RockCutting.edge(1e8, rad(i / 10.0), rad(20), rad(30), 0.05, 1, 1).liftN();
            assertTrue(Math.abs(l - lift) < 0.05 * 5e6, "at " + i / 10.0);
            lift = l;
        }
    }
}
