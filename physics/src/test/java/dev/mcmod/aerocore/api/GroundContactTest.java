package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class GroundContactTest {
    @Test void magicFormulaGripPeaksAtTheSurfacesFriction() {
        for (GroundContact.MagicFormula mf : new GroundContact.MagicFormula[]{GroundContact.MagicFormula.DRY, GroundContact.MagicFormula.WET,
                GroundContact.MagicFormula.SNOW, GroundContact.MagicFormula.ICE}) {
            double best = 0, at = 0;
            for (double s = 0; s <= 1; s += 1e-4) { double r = mf.ratio(s); if (r > best) { best = r; at = s; } }
            assertEquals(mf.d(), best, 0.01 * mf.d(), "Peak friction");
            assertTrue(at > 0.02 && at < 0.5, "Peak slip " + at);
            assertEquals(0, mf.ratio(0), 0);
            assertEquals(-mf.ratio(0.1), mf.ratio(-0.1), 1e-12, "Odd in slip");
            assertTrue(mf.ratio(1) < best, "Sliding below the peak");
        }
        // Locked dry wheel (slip 1) still grips about 0.9, a typical skid on dry asphalt.
        assertEquals(0.9, GroundContact.MagicFormula.DRY.ratio(1), 0.05);
    }

    @Test void combinedSlipSharesTheEllipse() {
        GroundContact.MagicFormula mf = GroundContact.MagicFormula.DRY;
        double[] pure = GroundContact.tyreForces(mf, 0.05, 0, 4000);
        assertEquals(mf.ratio(0.05) * 4000, pure[0], 1e-9);
        assertEquals(0, pure[1], 1e-12);
        double[] lateral = GroundContact.tyreForces(mf, 0, Math.toRadians(3), 4000);
        assertTrue(lateral[1] < 0 && lateral[0] == 0, "Lateral force opposes the slip angle");
        double[] both = GroundContact.tyreForces(mf, 0.08, Math.toRadians(4), 4000);
        double resultant = Math.hypot(0.08, Math.tan(Math.toRadians(4)));
        assertEquals(mf.ratio(resultant) * 4000, Math.hypot(both[0], both[1]), 1e-9, "On the friction ellipse");
        assertTrue(Math.hypot(both[0], both[1]) <= 4000 * mf.d() + 1e-9);
    }

    @Test void rigidWheelSinksUntilTheSoilCarriesItsLoad() {
        double load = 4000, diameter = 0.65, width = 0.2;
        for (GroundContact.Soil soil : new GroundContact.Soil[]{GroundContact.Soil.DRY_SAND, GroundContact.Soil.SANDY_LOAM,
                GroundContact.Soil.CLAYEY, GroundContact.Soil.SNOW, GroundContact.Soil.FIRM_CLAY}) {
            GroundContact.Sinkage s = GroundContact.rigidWheel(soil, load, diameter, width);
            // The pressure over the parabolic contact z = z0 - x^2 / D carries the load.
            double carried = 0, length = Math.sqrt(diameter * s.sinkageM()), dx = length / 20000;
            for (double x = dx / 2; x < length; x += dx) { carried += width * soil.pressure(s.sinkageM() - x * x / diameter, width) * dx; }
            assertEquals(load, carried, 0.002 * load, "Load carried on " + soil);
            // Compaction resistance is the work of pressing the soil down to z0, per metre rolled.
            double work = 0, dz = s.sinkageM() / 20000;
            for (double z = dz / 2; z < s.sinkageM(); z += dz) { work += width * soil.pressure(z, width) * dz; }
            assertEquals(work, s.compactionN(), 0.002 * work);
        }
        // A car wheel in dry sand: about a tenth of a metre down and rolling resistance near Wong's 0.3 of the load.
        GroundContact.Sinkage sand = GroundContact.rigidWheel(GroundContact.Soil.DRY_SAND, load, diameter, width);
        System.out.printf("Car wheel in dry sand: sinkage %.3f m, compaction %.0f N (%.2f W), traction at most %.0f N%n",
                sand.sinkageM(), sand.compactionN(), sand.compactionN() / load, sand.maxTractionN());
        assertTrue(sand.sinkageM() > 0.05 && sand.sinkageM() < 0.15);
        assertTrue(sand.compactionN() / load > 0.15 && sand.compactionN() / load < 0.4, "Wong: sand 0.15-0.3");
        assertEquals(load * Math.tan(Math.toRadians(28)), sand.maxTractionN(), 0.05 * load, "Friction-dominated sand");
        // Firm clay gives a little: Wong's car tyre on medium hard soil is 0.08, on concrete 0.015.
        double firm = GroundContact.rollingResistance(GroundContact.Surface.SOIL, load, diameter, width) / load;
        assertTrue(firm > 0.03 && firm < 0.1, "Firm soil " + firm);
        assertEquals(0.013, GroundContact.rollingResistance(GroundContact.Surface.TARMAC, load, diameter, width) / load, 1e-12);
    }

    @Test void soilShearBuildsUpWithSlip() {
        GroundContact.Soil sand = GroundContact.Soil.DRY_SAND;
        double p = 30000, full = sand.cohesionKpa() * 1000 + p * Math.tan(sand.frictionAngle());
        assertEquals(0, GroundContact.shear(sand, p, 0), 0);
        assertEquals(full * (1 - Math.exp(-1)), GroundContact.shear(sand, p, sand.shearModulusM()), 1e-9);
        assertEquals(full, GroundContact.shear(sand, p, 10), 1e-6);
    }
}
