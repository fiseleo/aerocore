package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/** Published check values for the empirical coefficients that replaced the earlier demonstration constants. */
class CoefficientDataTest {
    @Test void sphereDragFollowsTheStandardCurve() {
        // Clift, Grace and Weber standard curve values (also tabulated by the fluids package): 0.471, 0.405, 0.502.
        assertEquals(0.471, SphereDrag.coefficient(1e3), 0.003);
        assertEquals(0.405, SphereDrag.coefficient(1e4), 0.003);
        assertEquals(0.502, SphereDrag.coefficient(1e5), 0.003);
        // Creeping flow is Stokes drag; neighbouring ranges meet within 1 % (the drag crisis at 4e5 excepted).
        assertEquals(24 / 1e-3, SphereDrag.coefficient(1e-3), 0.2);
        for (double re : new double[]{0.01, 20, 260, 1.5e3, 1.2e4, 4.4e4, 3.38e5, 1e6}) {
            assertEquals(SphereDrag.coefficient(re * (1 - 1e-9)), SphereDrag.coefficient(re * (1 + 1e-9)), 0.01 * SphereDrag.coefficient(re), "Re " + re);
        }
        FluidState water = FluidState.FRESH_WATER_20C;
        assertEquals(6 * Math.PI * water.dynamicViscosityPaS() * 0.5, SphereDrag.resistance(water, 0.5, 0), 1e-12);
    }

    @Test void convectionCorrelations() {
        // Churchill natural convection, the EES example: Ra 1.5e6, Pr 2 gives Nu 19.52.
        assertEquals(19.52, HeatTransfer.sphereNaturalNusselt(1.5e6, 2), 0.01);
        // Whitaker: conduction limit Nu = 2 at rest, and a hand value at Re 1e4, Pr 0.71.
        assertEquals(2, HeatTransfer.sphereForcedNusselt(0, 0.71), 1e-12);
        assertEquals(2 + (0.4 * 100 + 0.06 * Math.pow(1e4, 2.0 / 3)) * Math.pow(0.71, 0.4), HeatTransfer.sphereForcedNusselt(1e4, 0.71), 1e-9);
    }

    @Test void thorntonRestitutionAndYieldVelocity() {
        assertEquals(1, ContactMaterial.thornton(1), 1e-3);
        assertEquals(1.185 * Math.pow(1e-6, 0.25), ContactMaterial.thornton(1e-6), 0.01 * 1.185 * Math.pow(1e-6, 0.25));
        ContactMaterial steel = ContactMaterial.STEEL_ON_ROCK;
        // Mild steel on granite yields at a few centimetres per second (the coefficient report's 0.02 m/s).
        assertTrue(steel.yieldVelocityMps() > 0.015 && steel.yieldVelocityMps() < 0.035, "V_y " + steel.yieldVelocityMps());
        assertEquals(1, steel.restitutionAt(0.5 * steel.yieldVelocityMps()));
        assertTrue(steel.restitutionAt(1) > 0.4 && steel.restitutionAt(1) < 0.5, "e(1 m/s) " + steel.restitutionAt(1));
        assertTrue(steel.restitutionAt(5) < steel.restitutionAt(1), "restitution falls with impact speed");
        assertEquals(0.2, new ContactMaterial(0.3, 0.2).restitutionAt(10), 1e-12);
    }

    @Test void roundedRectangleAndRudderTables() {
        assertEquals(2.5, HullManoeuvring.roundedRectangleDrag(0.5, 0), 1e-12);
        assertEquals(1.2, HullManoeuvring.roundedRectangleDrag(1.0, 0.167), 1e-12);
        assertEquals(0.7, HullManoeuvring.roundedRectangleDrag(2.0, 0.5), 1e-12);
        // Whicker and Fehlner lift slopes per radian at effective aspect ratios 1, 2, 3: 1.40, 2.44, 3.14.
        double[] expected = {1.40, 2.44, 3.14};
        for (int a = 1; a <= 3; a++) {
            AirfoilPolar polar = Rudder.lowAspectPolar(1, a / 2.0);
            double lift = polar.sample(Math.toRadians(1)).lift() / Math.toRadians(1);
            assertEquals(expected[a - 1], lift, 0.03, "a_e " + a);
        }
    }
}
