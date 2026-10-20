package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TurboRamjetTest {
    /** J58-class core: pressure ratio 8.5, 148 kg/s (326 lb/s), turbine at about 2000 F. */
    static final Turbojet.Design J58_CORE = new Turbojet.Design(8.5, 148, 0.87, 1366, 0.95, 0.98, 0.88, 0.99, 0, 0.98, false, 1, 1,
            Turbojet.Inlet.PITOT, Fuel.KEROSENE);
    /** Its afterburner to about 3400 F (2144 K), the nozzle opening widely. */
    static final Reheat.Design J58_REHEAT = new Reheat.Design(2144, 0.9, 0.25, 2.5);

    static TurboRamjet.Design j58(double maxBypass) {
        return new TurboRamjet.Design(J58_CORE, J58_REHEAT, new SupersonicInlet.Design(SupersonicInlet.Kind.CONE, 3.2, new double[]{Math.toRadians(26)}, 1),
                1.0, 2.0, 3.0, maxBypass, 0.4, 0.95);
    }

    @Test void theJ58sCoreAndAfterburnerGiveItsSeaLevelThrust() {
        Turbojet.Sizing core = Turbojet.size(J58_CORE, Turbojet.Flight.sea(0));
        // Lockheed/P&W J58 characteristics: 23 900 lbf (106.3 kN) military, 34 000 lbf (151.2 kN) with afterburner. Its
        // turbine temperature is not published (2000 F is the figure usually quoted); with J79-class efficiencies the model
        // gives about 14 % more dry thrust.
        assertEquals(106.3e3, core.point().thrustN(), 0.15 * 106.3e3, "dry");
        Reheat.Point wet = Reheat.turbojet(core, core.point(), Turbojet.Flight.sea(0), J58_REHEAT, 2144);
        assertEquals(151.2e3, wet.thrustN(), 0.15 * 151.2e3, "with afterburner");
        // as a turbo-ramjet at rest the bypass is shut: the same engine
        TurboRamjet.Sizing s = TurboRamjet.size(j58(0.4));
        TurboRamjet.Point p = TurboRamjet.at(s, Turbojet.Flight.sea(0), 1366, 2144);
        assertEquals(0, p.bypassAirflowKgPerS(), 1e-12);
        assertEquals(wet.thrustN(), p.thrustN(), 0.01 * wet.thrustN());
    }

    @Test void steppedOnItsRunningLinesItIsTheEquilibrium() {
        TurboRamjet.Sizing s = TurboRamjet.size(j58(0.4));
        Turbojet.RunningLines lines = Turbojet.runningLines(s.core());
        Turbojet.Flight sea = Turbojet.Flight.sea(0);
        TurboRamjet.Point eq = TurboRamjet.at(s, sea, 1366, 2144);
        double speed = eq.core().speedFraction();
        TurboRamjet.Point q = TurboRamjet.quasiSteady(s, lines, sea, speed, Turbojet.lineFuel(s.core(), lines, sea, speed), 2144);
        assertEquals(eq.thrustN(), q.thrustN(), 0.01 * eq.thrustN(), "the running line's fuel is the equilibrium");
    }

    @Test void aStoppedEngineGivesNothingAndNoNaN() {
        TurboRamjet.Sizing s = TurboRamjet.size(j58(0.4));
        Turbojet.RunningLines lines = Turbojet.runningLines(s.core());
        TurboRamjet.Point q = TurboRamjet.quasiSteady(s, lines, Turbojet.Flight.sea(0), 0, 0, 0);
        assertEquals(0, q.thrustN(), 1e-9);
        assertEquals(0, q.fuelKgPerS(), 1e-12);
    }

    @Test void atMach3TheBypassFeedsTheAfterburnerLikeARamjet() {
        // 24 km ISA: 220.65 K, 2972 Pa
        double t = 220.65, a = Math.sqrt(1.4 * 287.05287 * t);
        Turbojet.Flight m32 = new Turbojet.Flight(t, 2972, 3.2 * a);
        TurboRamjet.Point with = TurboRamjet.at(TurboRamjet.size(j58(0.4)), m32, 1366, 2144);
        TurboRamjet.Point without = TurboRamjet.at(TurboRamjet.size(j58(0)), m32, 1366, 2144);
        assertFalse(with.bypassBlocked(), "the bleed stage's pressure reaches the afterburner");
        assertEquals(0.4 * with.coreAirflowKgPerS(), with.bypassAirflowKgPerS(), 1e-9 * with.coreAirflowKgPerS());
        assertTrue(with.thrustN() > without.thrustN(), "the bypass adds thrust at Mach 3.2: " + with.thrustN() + " against " + without.thrustN());
        assertTrue(with.thrustN() > 0, "it pushes at Mach 3.2");
        // the bypass opens between Mach 2 and 3
        assertEquals(0, j58(0.4).bypassRatio(1.9), 1e-12);
        assertEquals(0.2, j58(0.4).bypassRatio(2.5), 1e-12);
        assertEquals(0.4, j58(0.4).bypassRatio(3.2), 1e-12);
    }
}
