package dev.mcmod.aerocore.api.manufacture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Electric discharge, electrochemical, laser, plasma and waterjet machining, and the gases they use (0.35). */
class NontraditionalTest {
    private static final double MM = 1e-3, MM3 = 1e-9;

    @Test void edmRemovalWireAndHoles() {
        // Weller: steel melting at 1500 C at 20 A: 664 x 20 / 1500^1.23 = 1.65 mm^3/s; a higher melting point removes less
        double r = Edm.removalRate(20, 1500 + 273.15);
        assertEquals(664 * 20 / Math.pow(1500, 1.23), r / MM3, 1e-9);
        assertEquals(1.646, r / MM3, 0.002);
        assertTrue(Edm.removalRate(20, 3410 + 273.15) < r);
        assertEquals(1000 * MM3 / r, Edm.sinkS(1000 * MM3, r), 1e-6);
        // a 0.25 mm wire with 0.02 mm overcut each side cuts 0.29 mm; through 20 mm at that rate: 17 mm/min
        double kerf = Edm.kerfM(0.25 * MM, 0.02 * MM);
        assertEquals(0.29 * MM, kerf, 1e-12);
        assertEquals(r / (kerf * 0.02), Edm.wireFeedMps(r, kerf, 0.02), 1e-15);
        assertEquals(17.0, Edm.wireFeedMps(r, kerf, 0.02) / MM * 60, 0.1);
        // four 0.2 mm orifices 1 mm deep at 0.1 mm^3/s: 1.26 s; electrode wear 5 % of 10 mm^3; generator 25 V x 20 A at half duty
        assertEquals(4 * Math.PI / 4 * 0.04 * 1 / 0.1, Edm.drillS(4, 0.2 * MM, 1 * MM, 0.1 * MM3), 1e-9);
        assertEquals(0.5 * MM3, Edm.electrodeWearM3(10 * MM3, 0.05), 1e-18);
        assertEquals(250, Edm.powerW(25, 20, 0.5), 1e-9);
        assertEquals(600, Edm.wireUsedM(0.15, 4000), 1e-9);
    }

    @Test void ecmFollowsFaradayAndOhm() {
        // iron dissolving as Fe2+: 0.055845 / (2 x 96485 x 7870) = 3.68e-2 mm^3 a coulomb (Groover's table for iron)
        double c = Ecm.specificRemoval(0.055845, 2, 7870);
        assertEquals(3.677e-2, c / MM3, 0.002e-2);
        // 1000 A at 90 %: 33 mm^3/s; over 10 cm^2 the tool advances 2 mm/min
        double rate = Ecm.removalRate(1000, c, 0.9);
        assertEquals(33.1, rate / MM3, 0.1);
        double f = Ecm.feedMps(rate, 1e-3);
        assertEquals(1.99, f / MM * 60, 0.01);
        // the gap at that feed in 20 S/m brine at 12 V (2 V overpotential): Ohm's law over the face, kappa (V - dV) A / I = 0.2 mm
        assertEquals(20 * 10 * 1e-3 / 1000, Ecm.gapM(c, 0.9, 20, 12, 2, f), 1e-12);
        assertEquals(500 * MM3 / rate, Ecm.machineS(500 * MM3, rate), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> Ecm.gapM(c, 0.9, 20, 2, 2, f));
    }

    @Test void laserAndPlasmaCutFromTheHeatBalance() {
        // steel: 500 J/kg K from 20 C to 1527 C, 270 kJ/kg to melt: 1.02 MJ/kg
        double e = ThermalCutting.meltJPerKg(500, 293.15, 1800.15, 270e3, 0, 0);
        assertEquals(1.0235e6, e, 1);
        assertEquals(4.871e6, ThermalCutting.IRON_OXIDATION_J_PER_KG, 1e3);
        // burning a tenth of the iron supplies 0.49 MJ/kg of it
        double ox = ThermalCutting.withOxygenJPerKg(e, 0.1);
        assertEquals(e - 0.1 * ThermalCutting.IRON_OXIDATION_J_PER_KG, ox, 1e-6);
        // 4 kW at 15 % through 6 mm with a 0.3 mm kerf: 4.8 m/min with oxygen, half that with nitrogen
        double v = ThermalCutting.speedMps(4000, 0.15, 0.3 * MM, 6 * MM, 7850, ox);
        assertEquals(0.15 * 4000 / (7850 * 0.3 * MM * 6 * MM * ox), v, 1e-12);
        assertEquals(4.75, v * 60, 0.05);
        assertTrue(ThermalCutting.speedMps(4000, 0.15, 0.3 * MM, 6 * MM, 7850, e) < 0.55 * v);
        assertEquals(1 / v + 3 * 0.5, ThermalCutting.cutS(1, v, 3, 0.5), 1e-9);
        // a 260 A plasma torch at 150 V: 39 kW
        assertEquals(39e3, ThermalCutting.arcW(150, 260), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> ThermalCutting.withOxygenJPerKg(e, 0.5));
    }

    @Test void waterjetFromBernoulliMixingAndErosion() {
        // 380 MPa: 872 m/s ideal, 785 at C_v 0.9; a 0.33 mm orifice at C_d 0.7 passes 3.1 L/min and takes 19.8 kW
        double v = Waterjet.jetMps(380e6, 1000, 0.9);
        assertEquals(0.9 * Math.sqrt(2 * 380e6 / 1000), v, 1e-9);
        double q = Waterjet.waterM3PerS(0.33 * MM, 380e6, 1000, 0.7);
        assertEquals(3.13, q * 60e3, 0.01);
        assertEquals(19.8e3, Waterjet.hydraulicW(380e6, q), 0.1e3);
        // 0.4 kg/min of garnet: loading 0.128, the particles leave at 0.75 v / 1.128 = 522 m/s
        double abrasive = 0.4 / 60, va = Waterjet.abrasiveMps(v, abrasive / (q * 1000), 0.75);
        assertEquals(521.8, va, 0.5);
        // steel eroding at 45 J/mm^3, a 1 mm kerf through 10 mm: about 120 mm/min, the order of the shop tables
        double removal = Waterjet.removalRate(abrasive, va, 45e9);
        assertEquals(121, Waterjet.cutMps(removal, 1 * MM, 10 * MM) / MM * 60, 1);
        // twice as thick, half as fast
        assertEquals(Waterjet.cutMps(removal, 1 * MM, 10 * MM) / 2, Waterjet.cutMps(removal, 1 * MM, 20 * MM), 1e-15);
    }

    @Test void assistGasCylindersAndAirSeparation() {
        // nitrogen through a 1.5 mm nozzle from 16 bar (abs) at 20 C, C_d 0.85: choked, 20.1 kg/h
        double m = Gases.chokedKgPerS(Gases.Gas.NITROGEN, 1.5 * MM, 16e5, 293, 0.85);
        assertEquals(20.1, m * 3600, 0.1);
        assertTrue(Gases.choked(Gases.Gas.NITROGEN, 16e5, 1.01325e5));
        assertFalse(Gases.choked(Gases.Gas.NITROGEN, 1.5e5, 1.01325e5));
        // a 50 L cylinder at 200 bar and 15 C holds 11.7 kg of nitrogen (Z 1); argon, heavier, more
        double n2 = Gases.cylinderKg(Gases.Gas.NITROGEN, 0.05, 2e7, 288.15, 1);
        assertEquals(11.69, n2, 0.01);
        assertTrue(Gases.cylinderKg(Gases.Gas.ARGON, 0.05, 2e7, 288.15, 1) > n2);
        // compressing 1 kg of air from 1 to 200 bar at 20 C takes at least 446 kJ
        assertEquals(445.6e3, Gases.isothermalJ(Gases.Gas.AIR, 1, 293.15, 1e5, 2e7), 0.5e3);
        // the least work for oxygen is 0.034 kWh/kg; a plant at 15 % of it about 0.22; a kilogram at 90 % recovery takes 4.8 kg of air
        double least = AirSeparation.minimumJPerKg(AirSeparation.O2_MOLE, 0.031999, 298.15);
        assertEquals(0.0336, least / 3.6e6, 0.0005);
        assertEquals(least / 0.15, AirSeparation.plantJPerKg(least, 0.15), 1e-6);
        assertEquals(1 / (0.2314 * 0.9), AirSeparation.airKg(1, AirSeparation.O2_MASS, 0.9), 1e-9);
        // the mass fractions follow from the mole fractions
        double air = AirSeparation.N2_MOLE * 0.028014 + AirSeparation.O2_MOLE * 0.031999 + AirSeparation.AR_MOLE * 0.039948 + 0.0004 * 0.04401;
        assertEquals(AirSeparation.O2_MASS, AirSeparation.O2_MOLE * 0.031999 / air, 2e-4);
        assertEquals(AirSeparation.N2_MASS, AirSeparation.N2_MOLE * 0.028014 / air, 2e-4);
    }
}
