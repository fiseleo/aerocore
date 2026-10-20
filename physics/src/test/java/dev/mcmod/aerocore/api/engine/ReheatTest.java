package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ReheatTest {
    /** J79-GE-17's cycle as TurbojetTest has it (pressure ratio 13.5, 77 kg/s, T4 1210 K): 52.8 kN dry. */
    private static final Turbojet.Design J79 = new Turbojet.Design(13.5, 77, 0.87, 1210, 0.95, 0.98, 0.88, 0.99, 0, 0.98, false, 1, 0.99,
            Turbojet.Inlet.PITOT, Fuel.KEROSENE);
    private static final Reheat.Design AFTERBURNER = new Reheat.Design(1950, 0.9, 0.25, 1.8);

    @Test void theJ79sAfterburnerGivesItsWetThrustAndFuelBurn() {
        Turbojet.Sizing s = Turbojet.size(J79, Turbojet.Flight.sea(0));
        Turbojet.Point dry = s.point();
        Reheat.Point wet = Reheat.turbojet(s, dry, Turbojet.Flight.sea(0), AFTERBURNER, 1950);
        // J79-GE-17: 79.6 kN (17 900 lbf) with afterburner, SFC 1.965 lb/(lbf h) = 55.7 g/(kN s) (GE data as in Jane's)
        assertEquals(79.6e3, wet.thrustN(), 0.1 * 79.6e3, "wet thrust");
        assertEquals(55.7e-6, (dry.fuelKgPerS() + wet.fuelKgPerS()) / wet.thrustN(), 0.12 * 55.7e-6, "wet SFC");
        assertTrue(wet.throatRatio() > 1.3 && wet.throatRatio() < AFTERBURNER.maxThroatRatio(), "the nozzle opens: " + wet.throatRatio());
        assertFalse(wet.thermallyLimited() || wet.throatLimited());
    }

    @Test void unlitItIsTheDryEngineAndHotterPushesHarder() {
        Turbojet.Flight sea = Turbojet.Flight.sea(0);
        Turbojet.Sizing s = Turbojet.size(J79, sea);
        Turbojet.Point dry = s.point();
        Reheat.Point cold = Reheat.turbojet(s, dry, sea, AFTERBURNER, 0);
        assertEquals(dry.thrustN(), cold.thrustN(), 0.002 * dry.thrustN(), "no reheat: the dry thrust");
        assertEquals(0, cold.fuelKgPerS(), 1e-12);
        assertEquals(1, cold.throatRatio(), 0.002);
        double last = cold.thrustN();
        for (double t : new double[]{1200, 1500, 1800, 1950}) {
            Reheat.Point p = Reheat.turbojet(s, dry, sea, AFTERBURNER, t);
            assertTrue(p.thrustN() > last, "more thrust at " + t + " K");
            last = p.thrustN();
        }
        // the throat's travel holds it down
        Reheat.Design tight = new Reheat.Design(1950, 0.9, 0.25, 1.1);
        Reheat.Point held = Reheat.turbojet(s, dry, sea, tight, 1950);
        assertTrue(held.throatLimited() && held.temperatureK() < 1950 && held.throatRatio() <= 1.1 + 1e-6, "throat-limited: " + held);
        // a duct entered fast chokes thermally first
        Reheat.Point fast = Reheat.turbojet(s, dry, sea, new Reheat.Design(2400, 0.9, 0.45, 3), 2400);
        assertTrue(fast.thermallyLimited(), "thermal choke at Mach 0.45 in");
        // at Mach 2 the afterburner adds a larger share
        Turbojet.Flight m2 = new Turbojet.Flight(216.65, 22632, 2 * Math.sqrt(1.4 * 287.05 * 216.65));
        Turbojet.Point dry2 = Turbojet.equilibrium(s, m2, 1210, Double.NaN);
        Reheat.Point wet2 = Reheat.turbojet(s, dry2, m2, AFTERBURNER, 1950);
        assertTrue(wet2.thrustN() / dry2.thrustN() > wet(s, sea) / dry.thrustN(), "reheat matters more in fast flight");
    }

    private static double wet(Turbojet.Sizing s, Turbojet.Flight f) { return Reheat.turbojet(s, s.point(), f, AFTERBURNER, 1950).thrustN(); }

    @Test void aMixedTurbofanReheatsItsMixedStream() {
        Turbojet.Flight sea = Turbojet.Flight.sea(0);
        Turbofan.Sizing s = Turbofan.size(TurbofanTest.FJ44_CLASS, sea);
        Turbofan.Point dry = s.point();
        Reheat.Point cold = Reheat.turbofan(s, dry, sea, AFTERBURNER, 0);
        assertEquals(dry.thrustN(), cold.thrustN(), 0.01 * dry.thrustN(), "no reheat: the dry thrust");
        Reheat.Point wet = Reheat.turbofan(s, dry, sea, AFTERBURNER, 1900);
        // a low-bypass mixed fan gains more from reheat than a turbojet: its cool mixed stream takes more heat
        assertTrue(wet.thrustN() > 1.4 * dry.thrustN(), "wet " + wet.thrustN() + " dry " + dry.thrustN());
        assertThrows(IllegalArgumentException.class, () -> Reheat.turbofan(Turbofan.size(TurbofanTest.CFM56_CLASS, sea),
                Turbofan.size(TurbofanTest.CFM56_CLASS, sea).point(), sea, AFTERBURNER, 1900), "separate exhausts do not reheat");
    }
}
