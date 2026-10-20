package dev.mcmod.aerocore.api.body;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** A body's structure against thin-walled beam and shell formulas. */
class BodyStructureTest {
    private static final double R = 1, T = 0.002, L = 10, RHO = 2780, G = 9.81;

    private static BodyStructure.Module skin(double stringers, double pressure) {
        return new BodyStructure.Module(BodyStructure.Build.SEMI_MONOCOQUE, RHO, 345e6, 73e9, T, stringers, 0.5, 0, 0, 0, 0, 0, 0, pressure);
    }

    private static BodyShape tube() {
        SectionShape circle = SectionShape.Rounded.ellipse(2 * R, 2 * R, 0);
        return new BodyShape(circle, List.of(BodyModule.straight("tube", L, circle)));
    }

    @Test void aThinTubeBendsAndHoldsPressureByTheBook() {
        BodyShape b = tube();
        BodyShape.Station s = b.stations().get(0);
        double[] section = BodyStructure.bendingSection(s, skin(0, 0));
        // thin-walled circle: I = pi r^3 t (the 32-gon is a hair smaller), farthest fibre r
        assertEquals(Math.PI * R * R * R * T, section[0], 0.01 * Math.PI * R * R * R * T);
        assertEquals(R, section[1], 1e-9);
        // its skin's mass: 2 pi r L t rho, stringers adding their share
        double shell = 2 * Math.PI * R * L * T * RHO;
        assertEquals(1.5 * shell, BodyStructure.masses(b, List.of(skin(0.5, 0))).get(0).skinKg(), 0.01 * shell);
        // simply supported at the ends under its own weight: M = w L^2 / 8 in the middle
        BodyStructure.Result r = BodyStructure.analyze(b, List.of(skin(0, 0)), List.of(), 1, G, List.of(new double[]{0, 1}, new double[]{L, 1}));
        double w = r.massKg() * G / L, mid = 0;
        for (int i = 0; i < r.xM().length; i++) { if (Math.abs(r.xM()[i] - L / 2) < Math.abs(mid - L / 2)) { mid = r.xM()[i]; } }
        double max = 0;
        for (double m : r.momentNm()) { max = Math.max(max, Math.abs(m)); }
        assertEquals(w * L * L / 8, max, 0.05 * w * L * L / 8, "mid-span moment");
        assertEquals(L / 2, r.maxBendingAtM(), 0.1 * L);
        assertEquals(max * R / section[0], r.maxBendingPa(), 0.02 * max * R / section[0]);
        // a cabin at 55 kPa: p r / t (r of the polygon's area)
        double p = 55e3, rArea = Math.sqrt(s.areaM2() / Math.PI);
        assertEquals(p * rArea / T, BodyStructure.analyze(b, List.of(skin(0, p)), List.of(), 1, G, null).maxHoopPa(), 1e-6 * p * rArea / T);
    }

    @Test void doorsWeakenAndAddFrames() {
        BodyShape b = tube();
        BodyStructure.Module plain = skin(0.3, 0), doors = new BodyStructure.Module(BodyStructure.Build.SEMI_MONOCOQUE, RHO, 345e6, 73e9, T, 0.3, 0.5,
                4e-4, 0, 2, 1.0, 1.8, 0.1, 0);
        BodyShape.Station s = b.stations().get(0);
        assertTrue(BodyStructure.bendingSection(s, doors)[0] < BodyStructure.bendingSection(s, plain)[0], "openings cut the side skin");
        assertTrue(BodyStructure.masses(b, List.of(doors)).get(0).openingsKg() > 0, "and need frames round them");
        // buoyancy holds a boat's hull up along its length: a uniform tube in still water is unloaded
        BodyStructure.Result r = BodyStructure.analyze(b, List.of(plain), List.of(), 1, G, null);
        double max = 0;
        for (double m : r.momentNm()) { max = Math.max(max, Math.abs(m)); }
        assertEquals(0, max, 1e-6 * r.massKg() * G * L);
    }

    /** ISO 12215-5's worked 24 m yacht (Tivari 2022, tables 16-19): L_WL 22.06 m, B_C 6.83 m, 20.1 deg, 25 kn, 95.4 t. */
    @Test void isoBottomPressureMatchesTheWorkedYacht() {
        assertEquals(1.19, BodyStructure.dynamicLoadFactor(95_400, 22.06, 6.83, 20.1, 25), 0.005);
        assertEquals(138.7e3, BodyStructure.bottomPressurePa(95_400, 22.06, 6.83, 20.1, 25, 1.0), 0.5e3);
        assertEquals(130.7e3, BodyStructure.bottomPressurePa(95_400, 22.06, 6.83, 20.1, 25, 0.8), 0.5e3);
        // a light, long, slow hull is held to the minimum (its base pressure is 1.5 kPa)
        assertEquals(1000 * Math.max(0.45 * Math.pow(300, 0.33) + 0.9 * 10, 7), BodyStructure.bottomPressurePa(300, 10, 2, 20, 1, 1.0), 1e-6);
        // plating: 0.5 m frames at 100 kPa in a 100 MPa design stress need 0.5 sqrt(0.5 * 1e5 / 1e8) = 11.2 mm
        assertEquals(0.5 * Math.sqrt(0.5e-3), BodyStructure.plateThicknessM(1e5, 0.5, 1e8), 1e-12);
    }
}
