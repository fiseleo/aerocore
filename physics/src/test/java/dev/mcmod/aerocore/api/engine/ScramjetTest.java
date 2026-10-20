package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.gasdynamics.GasState;
import dev.mcmod.aerocore.api.gasdynamics.NormalShock;
import dev.mcmod.aerocore.api.gasdynamics.Thermo;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ScramjetTest {
    /** 27 km ISA: 223.5 K, 1847 Pa. */
    private static final double T27 = 223.5, P27 = 1847;

    static Turbojet.Flight at27(double mach) { return new Turbojet.Flight(T27, P27, mach * Math.sqrt(1.4 * 287.05287 * T27)); }

    static Scramjet.Design engine(double internalContraction, double isolator, Fuel fuel) {
        return new Scramjet.Design(0.1, 7, new double[]{Math.toRadians(5.5), Math.toRadians(5.5)}, 0.01, internalContraction, 0.97, isolator, 1.0, 2.5, 0.0015,
                0.9, 3, 0.98, 0.5, fuel);
    }

    @Test void theCompressionIsHeiserAndPrattsForAPerfectGas() {
        Thermo g = Thermo.PERFECT_AIR;
        for (double m : new double[]{5, 7, 10}) {
            GasState free = new GasState(T27, P27, m * g.speedOfSound(T27));
            for (double psi : new double[]{4, 6, 8}) {
                double closed = Math.pow(psi / (1 + 0.2 * m * m * (1 - 0.97)), 3.5);
                assertEquals(closed, Scramjet.compressionPressureRatio(g, free, psi * T27, 0.97), 1e-9 * closed, "M" + m + " psi " + psi);
            }
        }
    }

    @Test void theInletStartsByKantrowitzAndStaysStartedBelowIt() {
        assertNotEquals(Scramjet.Mode.UNSTART, Scramjet.at(engine(1.5, 10, Fuel.HYDROGEN), at27(7), 1, true, false).mode(), "a mild internal contraction starts");
        Scramjet.Design tight = engine(2.5, 10, Fuel.HYDROGEN);
        assertEquals(Scramjet.Mode.UNSTART, Scramjet.at(tight, at27(7), 1, true, false).mode(), "too much internal contraction does not start itself");
        assertNotEquals(Scramjet.Mode.UNSTART, Scramjet.at(tight, at27(7), 1, true, true).mode(), "once started it stays started");
        // too slow for the contraction: the throat chokes
        assertEquals(Scramjet.Mode.UNSTART, Scramjet.at(engine(1.5, 10, Fuel.HYDROGEN), at27(3.5), 1, true, true).mode());
    }

    @Test void hydrogenBurnsSupersonicAtMach7AndInADualModeAt4Point5() {
        Scramjet.Design d = engine(1.5, 10, Fuel.HYDROGEN);
        Scramjet.Point m7 = Scramjet.at(d, at27(7), 1, true, true);
        assertEquals(Scramjet.Mode.SCRAM, m7.mode());
        assertTrue(m7.combustorEntryMach() > 1 && m7.combustorExitMach() > 1, "supersonic through the combustor");
        // a hydrogen scramjet's fuel impulse at Mach 7: 1500-4000 s (Heiser and Pratt fig. 1.14)
        assertTrue(m7.specificImpulse() > 1500 && m7.specificImpulse() < 4000, "Isp " + m7.specificImpulse());
        double q = 0.5 * 1.4 * P27 * 49;
        double ct = m7.thrustN() / (q * 0.1);
        assertTrue(ct > 0.3 && ct < 1.2, "thrust coefficient " + ct);
        // this mildly compressing engine reaches only about 820 K at Mach 7: hydrogen needs a pilot there, but lights by itself at Mach 10 (about 1040 K)
        assertTrue(m7.ignitionDelayS() > m7.residenceTimeS(), "no autoignition at Mach 7: " + m7.ignitionDelayS() + " s in " + m7.residenceTimeS() + " s");
        Scramjet.Point m10 = Scramjet.at(d, at27(10), 1, true, true);
        assertTrue(m10.ignitionDelayS() < m10.residenceTimeS(), "autoignition at Mach 10: " + m10.ignitionDelayS() + " s in " + m10.residenceTimeS() + " s");
        Scramjet.Point m45 = Scramjet.at(d, at27(4.5), 1, true, true);
        assertEquals(Scramjet.Mode.DUAL, m45.mode());
        assertTrue(m45.combustorEntryMach() < 1, "a shock train slows it to subsonic");
        GasState s3 = Scramjet.throat(Gas.AIR, new GasState(T27, P27, at27(4.5).speedMps()), 0.97, at27(4.5).pressurePa() / (287.05287 * T27)
                * at27(4.5).speedMps() * 0.1 * SupersonicInlet.captureRatio(d.ramps(), 4.5) / 0.01);
        double normal = NormalShock.behind(Gas.AIR, s3).pressurePa() / s3.pressurePa();
        assertTrue(m45.isolatorPressureRatio() > 1 && m45.isolatorPressureRatio() <= normal * 1.0001, "the isolator holds less than a normal shock's rise");
        assertTrue(m45.thrustN() > 0);
        // a short isolator cannot hold that rise: unstart
        assertEquals(Scramjet.Mode.UNSTART, Scramjet.at(engine(1.5, 2, Fuel.HYDROGEN), at27(4.5), 1, true, true).mode());
        // unlit it only drags
        assertTrue(Scramjet.at(d, at27(7), 1, false, true).thrustN() < 0);
    }

    @Test void theMapScalesWithAmbientPressureAndMatchesTheEngine() {
        Scramjet.Design d = engine(1.5, 10, Fuel.HYDROGEN);
        Scramjet.Map map = Scramjet.Map.build(d, 4, 8, 1.0);
        // on a row (Mach 7, 250 K, phi 1) at another pressure: the engine's numbers exactly scaled
        Turbojet.Flight f = new Turbojet.Flight(250, 3000, 7 * Math.sqrt(1.4 * 287.05287 * 250));
        Scramjet.Point direct = Scramjet.at(d, f, 1, true, true), looked = map.at(f, 1, true);
        assertEquals(direct.mode(), looked.mode());
        // (to the solvers' tolerances, which are relative to the pressure)
        assertEquals(direct.thrustN(), looked.thrustN(), 1e-5 * Math.abs(direct.thrustN()), "thrust per pascal");
        assertEquals(direct.fuelKgPerS(), looked.fuelKgPerS(), 1e-5 * direct.fuelKgPerS());
        assertEquals(direct.combustorEntryK(), looked.combustorEntryK(), 1e-5 * direct.combustorEntryK());
        assertEquals(direct.ignitionDelayS(), looked.ignitionDelayS(), 1e-4 * direct.ignitionDelayS());
        // unlit row
        assertEquals(Scramjet.at(d, f, 0, false, true).thrustN(), map.at(f, 1, false).thrustN(), 1e-5 * Math.abs(Scramjet.at(d, f, 0, false, true).thrustN()));
        // between rows: between the neighbours
        Turbojet.Flight between = new Turbojet.Flight(250, 3000, 7.1 * Math.sqrt(1.4 * 287.05287 * 250));
        double t = map.at(between, 0.9, true).thrustN();
        assertTrue(Double.isFinite(t) && t > 0);
        assertTrue(Scramjet.started(d, 7, false) && !Scramjet.started(engine(2.5, 10, Fuel.HYDROGEN), 7, false) && Scramjet.started(engine(2.5, 10, Fuel.HYDROGEN), 7, true));
    }

    @Test void kerosenesIgnitionNeedsTheHotDualModeOrAPilot() {
        Scramjet.Design d = engine(1.5, 10, Fuel.KEROSENE);
        Scramjet.Point m7 = Scramjet.at(d, at27(7), 1, true, true);
        assertEquals(Scramjet.Mode.SCRAM, m7.mode());
        assertTrue(m7.ignitionDelayS() > 10 * m7.residenceTimeS(), "kerosene does not light by itself in a Mach 7 scram combustor");
        Scramjet.Point m5 = Scramjet.at(d, at27(5), 1, true, true);
        assertEquals(Scramjet.Mode.DUAL, m5.mode());
        assertTrue(m5.ignitionDelayS() < m5.residenceTimeS(), "behind the shock train it does (as the X-51 flew, at Mach 5)");
        // hydrogen's delay is far shorter than kerosene's at the same state
        assertTrue(Fuel.HYDROGEN.ignitionDelayS(900, 1e5) < 0.05 * Fuel.KEROSENE.ignitionDelayS(900, 1e5));
        // Jet A at 1000 K and 10 atm: about 0.3 ms (Spadaccini and TeVelde's fit)
        assertEquals(0.3e-3, Fuel.KEROSENE.ignitionDelayS(1000, 10 * 101325), 0.05e-3);
    }

    /** An X-51A-class kerosene engine's table across Mach 4-7 builds (0.28: a dual-mode march once overflowed past choking and threw). */
    @Test void aKeroseneEnginesTableBuildsAcrossItsRange() {
        Scramjet.Design d = new Scramjet.Design(0.06, 6, SupersonicInlet.optimalRamps(6, 2), 0.06 / 8, 1.4, 0.97, 10, 1.0, 2.0, 0.0015, 0.85, 2.5, 0.98, 0.3, Fuel.KEROSENE);
        Scramjet.Map map = Scramjet.Map.build(d, 4, 7, 1);
        for (double m = 4; m <= 7; m += 0.5) {
            Scramjet.Point p = map.at(at27(m), 1, true);
            assertTrue(Double.isFinite(p.thrustN()) && Double.isFinite(p.fuelKgPerS()), "Mach " + m);
        }
    }
}
