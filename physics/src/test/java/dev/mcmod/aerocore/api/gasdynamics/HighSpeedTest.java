package dev.mcmod.aerocore.api.gasdynamics;

import dev.mcmod.aerocore.api.aero.Airfoil;
import dev.mcmod.aerocore.api.aero.Wing;
import dev.mcmod.aerocore.api.body.BodyAerodynamics;
import dev.mcmod.aerocore.api.body.BodyModule;
import dev.mcmod.aerocore.api.body.BodyShape;
import dev.mcmod.aerocore.api.body.SectionShape;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** Supersonic bodies and wings, and aerodynamic heating (0.28). */
class HighSpeedTest {
    static BodyShape cone(double halfAngle, double length) {
        double rb = length * Math.tan(halfAngle);
        return new BodyShape(SectionShape.Rounded.ellipse(1e-3, 1e-3, 0), List.of(BodyModule.straight("cone", length, SectionShape.Rounded.ellipse(2 * rb, 2 * rb, -rb))));
    }

    /** A Sears-Haack body of a length and largest diameter: r = r_max (4 xi (1 - xi))^(3/4), in straight modules. */
    static BodyShape searsHaack(double length, double diameter, int modules) {
        List<BodyModule> m = new ArrayList<>();
        for (int k = 1; k <= modules; k++) {
            double xi = (double) k / modules, r = 0.5 * diameter * Math.pow(Math.max(4 * xi * (1 - xi), 1e-6), 0.75);
            m.add(BodyModule.straight("m" + k, length / modules, SectionShape.Rounded.ellipse(2 * r, 2 * r, -r)));
        }
        return new BodyShape(SectionShape.Rounded.ellipse(1e-3, 1e-3, 0), m);
    }

    @Test void theSurfacePressureLimitsAreTheTextbooks() {
        assertEquals(1.276, SurfacePressure.stagnation(1.0), 0.002, "Rayleigh pitot at Mach 1");
        assertEquals(1.839, SurfacePressure.stagnation(20), 0.003, "modified Newtonian's Cp_max for gamma 1.4");
        assertEquals(-2 / (1.4 * 9), SurfacePressure.expansion(3, Math.toRadians(80)), 1e-12, "no lower than a vacuum");
        // tangent cone is Taylor-Maccoll between the table's rows
        for (double m : new double[]{1.7, 2.6, 4.2, 9}) {
            for (double deg : new double[]{5, 10, 20}) {
                ConicalShock c = ConicalShock.of(m, Math.toRadians(deg), 1.4);
                double exact = SurfacePressure.cp(c.surfacePressureRatio(), m);
                assertEquals(exact, SurfacePressure.tangentCone(m, Math.toRadians(deg)), 0.03 * exact + 1e-4, "M " + m + " " + deg + " deg");
            }
        }
    }

    @Test void aConesDragIsTaylorMaccollsPlusItsBase() {
        double half = Math.toRadians(10), length = 2;
        BodyShape shape = cone(half, length);
        double base = shape.particulars().baseAreaM2();   // the outline's polygon, a little under pi r^2
        BodyAerodynamics b = new BodyAerodynamics(shape, 0);
        for (double m : new double[]{1.5, 2, 3, 5}) {
            double fore = SurfacePressure.cp(ConicalShock.of(m, half, 1.4).surfacePressureRatio(), m) * base;
            double total = fore - SurfacePressure.base(m) * base;
            assertEquals(total, b.supersonicPressureDragAreaM2(m), 0.01 * total, "M " + m);
        }
    }

    @Test void aSearsHaackBodyComesNearSlenderBodyTheoryAndFallsWithMach() {
        double l = 10, d = 1, a = Math.PI * d * d / 4, theory = 4.5 * Math.PI * a * a / (l * l);
        BodyAerodynamics b = new BodyAerodynamics(searsHaack(l, d, 40), 0);
        // local-inclination methods overestimate slender bodies just above Mach 1 (1.7 times at 1.2), match near 2-3
        for (double m : new double[]{1.5, 2, 3}) {
            double ratio = b.supersonicPressureDragAreaM2(m) / theory;
            assertTrue(ratio > 0.8 && ratio < 1.6, "M " + m + ": " + ratio + " of slender-body theory");
        }
        // Raymer's empirical fall of wave drag from Mach 1.2 to 2 (1 - 0.386 (M - 1.2)^0.57 for an unswept body): 0.66
        double fall = b.supersonicPressureDragAreaM2(2) / b.supersonicPressureDragAreaM2(1.2);
        assertEquals(1 - 0.386 * Math.pow(0.8, 0.57), fall, 0.08);
    }

    @Test void theDragRisesThroughTransonicSmoothly() {
        BodyAerodynamics b = new BodyAerodynamics(searsHaack(10, 1, 20), 0);
        Wing.Air air = Wing.Air.SEA_LEVEL;
        double last = 0;
        for (double m = 0.5; m <= 1.3; m += 0.02) {
            double d = b.zeroLiftDragAreaM2(m * air.soundMps(), air);
            if (m > b.criticalMach() + 0.02 && m < 1.19) { assertTrue(d > last, "rising at M " + m); }
            if (m > 0.52) { assertTrue(Math.abs(d - last) < 0.02, "no jump at M " + m); }
            last = d;
        }
        assertTrue(b.zeroLiftDragAreaM2(1.2 * air.soundMps(), air) > 3 * b.zeroLiftDragAreaM2(0.6 * air.soundMps(), air), "the wave drag dominates");
    }

    @Test void aSupersonicWingFollowsLinearTheory() {
        // the Mach cone's tip loss averages to a quarter over the cone's footprint: C_L,alpha = 4/beta (1 - 1/(2 beta A))
        double sum = 0;
        for (int k = 0; k < 10000; k++) { sum += Wing.machConeShare((k + 0.5) / 10000); }
        assertEquals(0.75, sum / 10000, 1e-6);
        Airfoil thin = Airfoil.naca4(0, 0, 0.04);
        Wing.Air air = new Wing.Air(0.4, 1.5e-5, 300);
        for (double span : new double[]{3, 20}) {
            Wing w = new Wing(new Wing.Planform(span, 1, 1, 0, 0, 0, 0, true), thin, thin, List.of(), 24);
            double v = 600, beta = Math.sqrt(3), a = Math.toRadians(2), q = 0.5 * 0.4 * v * v * span;
            Wing.Solution up = w.solve(v, a, 0, air, Map.of()), down = w.solve(v, -a, 0, air, Map.of());
            double lift = (up.force().y() * Math.cos(a) + up.force().x() * Math.sin(a)) - (down.force().y() * Math.cos(a) - down.force().x() * Math.sin(a));
            double slope = lift / q / (2 * a), theory = 4 / beta * (1 - 1 / (2 * beta * span));
            assertEquals(theory, slope, 0.03 * theory, "aspect ratio " + span);
        }
    }

    @Test void skinsHeatAsTheSupersonicAircraftDid() {
        // SR-71 at Mach 3.2, 24 km (ISA 220.6 K, 0.0469 kg/m^3), black paint: fuselage skin 230-315 C (Lockheed)
        double srSpeed = 3.2 * Math.sqrt(1.4 * 287.05287 * 220.6);
        double sr = AeroHeating.surfaceTemperature(0.0469, 220.6, srSpeed, 5, 0.9);
        assertTrue(sr > 503 && sr < 600, "SR-71 skin " + sr + " K");
        // Concorde at Mach 2.02, 18 km (216.65 K, 0.1207 kg/m^3): skin about 90-100 C, nose tip 127 C (BAC/Aerospatiale)
        double cSpeed = 2.02 * Math.sqrt(1.4 * 287.05287 * 216.65);
        double skin = AeroHeating.surfaceTemperature(0.1207, 216.65, cSpeed, 10, 0.85), nose = AeroHeating.noseTemperature(0.1207, 216.65, cSpeed, 0.05, 0.85);
        assertTrue(skin > 355 && skin < 380, "Concorde skin " + skin + " K");
        assertTrue(nose > 385 && nose < 405, "Concorde nose " + nose + " K");
        // X-15 at Mach 6.7, 31 km (226.5 K, 0.0158 kg/m^3): lower skin to about 1300 F (978 K)
        double xSpeed = 6.7 * Math.sqrt(1.4 * 287.05287 * 226.5);
        double x15 = AeroHeating.surfaceTemperature(0.0158, 226.5, xSpeed, 2, 0.8);
        assertEquals(978, x15, 0.2 * 978, "X-15 lower skin");
        assertTrue(AeroHeating.recoveryTemperature(220, 0.1) < 221, "no heating at low speed");
    }
}
