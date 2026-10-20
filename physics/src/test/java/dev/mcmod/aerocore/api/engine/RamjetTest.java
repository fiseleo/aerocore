package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.gasdynamics.NormalShock;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RamjetTest {
    /** ISA at 20 km. */
    private static final double T20 = 216.65, P20 = 5474.9, A20 = Math.sqrt(1.4 * 287.05287 * T20);

    private static Turbojet.Flight at20(double mach) { return new Turbojet.Flight(T20, P20, mach * A20); }

    static double mil(double m) { return m <= 1 ? 1 : 1 - 0.075 * Math.pow(m - 1, 1.35); }

    @Test void inletsRecoverAsTheShockTablesAndTheMilSpecSay() {
        SupersonicInlet.Design pitot = new SupersonicInlet.Design(SupersonicInlet.Kind.PITOT, 2, null, 1);
        assertEquals(NormalShock.of(2, 1.4).totalPressureRatio(), SupersonicInlet.critical(pitot, 2).recovery(), 1e-12, "a pitot inlet loses a normal shock");
        // Oswatitsch's optimum: equal-strength shocks; more of them recover more
        double previous = SupersonicInlet.critical(pitot, 3).recovery();
        for (int n = 1; n <= 3; n++) {
            SupersonicInlet.Design d = new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, 3, SupersonicInlet.optimalRamps(3, n), 1);
            double r = SupersonicInlet.critical(d, 3).recovery();
            assertTrue(r > previous, n + " ramps recover more: " + r);
            previous = r;
        }
        // a well-designed inlet with a 0.97 subsonic diffuser comes within a few percent of MIL-E-5008B
        SupersonicInlet.Design two = new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, 2, SupersonicInlet.optimalRamps(2, 2), 0.97);
        assertEquals(mil(2), SupersonicInlet.critical(two, 2).recovery(), 0.05 * mil(2), "two ramps at Mach 2");
        SupersonicInlet.Design three = new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, 3, SupersonicInlet.optimalRamps(3, 3), 0.97);
        assertEquals(mil(3), SupersonicInlet.critical(three, 3).recovery(), 0.08 * mil(3), "three ramps at Mach 3");
        // the equal-strength ramps leave the terminal shock as strong as each of them
        double[] ramps = SupersonicInlet.optimalRamps(3, 2);
        SupersonicInlet.Critical c = SupersonicInlet.critical(new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, 3, ramps, 1), 3);
        double firstNormal = 3 * Math.sin(dev.mcmod.aerocore.api.gasdynamics.ObliqueShock.of(3, ramps[0], 1.4, false).waveAngle());
        assertEquals(firstNormal, c.terminalMach(), 1e-6, "Oswatitsch: the terminal shock as strong as the oblique ones");
    }

    @Test void belowItsDesignMachTheInletSpills() {
        SupersonicInlet.Design d = new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, 3, SupersonicInlet.optimalRamps(3, 2), 0.97);
        assertEquals(1, SupersonicInlet.captureRatio(d, 3), 1e-12, "shock on lip");
        assertEquals(1, SupersonicInlet.captureRatio(d, 3.5), 1e-12, "all captured above design");
        double last = 1;
        for (double m : new double[]{2.6, 2.2, 1.8, 1.5}) {
            double r = SupersonicInlet.captureRatio(d, m);
            assertTrue(r < last && r > 0.3, "spills more at Mach " + m + ": " + r);
            last = r;
        }
        SupersonicInlet.Design spike = new SupersonicInlet.Design(SupersonicInlet.Kind.CONE, 2.5, new double[]{Math.toRadians(20)}, 0.97);
        assertTrue(SupersonicInlet.captureRatio(spike, 2) < 1 && SupersonicInlet.captureRatio(spike, 2.5) == 1);
        assertTrue(SupersonicInlet.critical(spike, 2.5).recovery() > SupersonicInlet.critical(new SupersonicInlet.Design(SupersonicInlet.Kind.PITOT, 2.5, null, 0.97), 2.5).recovery());
    }

    static Ramjet.Design mach3(double margin) {
        return new Ramjet.Design(new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, 3, SupersonicInlet.optimalRamps(3, 3), 0.97), 0.1, 0.18, 1.5, 0.95,
                2300, 0.97, true, 2.5, margin, Fuel.KEROSENE);
    }

    @Test void aMach3RamjetMatchesItsInletAndPushes() {
        Ramjet.Sizing s = Ramjet.size(mach3(1.03), at20(3), 2000);
        Ramjet.Point p = s.point();
        assertEquals(Ramjet.Mode.SUPERCRITICAL, p.mode(), "sized a little supercritical");
        SupersonicInlet.Critical c = SupersonicInlet.critical(s.design().inlet(), 3);
        assertEquals(c.recovery() / 1.03, p.recovery(), 0.002, "the margin comes off the recovery");
        assertTrue(p.thrustN() > 0 && p.combustorMach() > 0.1 && p.combustorMach() < 0.5, "thrust " + p.thrustN() + ", M3 " + p.combustorMach());
        // the gross-minus-ram thrust per airflow equals a constant-gamma hand calculation within the gas's property spread
        double f = p.fuelKgPerS() / p.airflowKgPerS(), g = 1.30, r = 8.314462618 / 0.02897;
        double cp = g * r / (g - 1), pt9 = p.pt4() * 0.97;
        double v9 = Math.sqrt(2 * cp * p.tt4() * (1 - Math.pow(P20 / pt9, (g - 1) / g)));
        double hand = (1 + f) * v9 - 3 * A20;
        assertEquals(hand, (p.thrustN() + p.spillageDragN()) / p.airflowKgPerS(), 0.08 * hand, "specific thrust");
        // a hydrocarbon ramjet's fuel impulse at Mach 3: 1000-2000 s (Heiser and Pratt, Hypersonic Airbreathing Propulsion, fig. 1.14)
        assertTrue(p.specificImpulse() > 1000 && p.specificImpulse() < 2000, "Isp " + p.specificImpulse());
        // the fuel given back
        Ramjet.Point back = Ramjet.withFuel(s, at20(3), p.fuelKgPerS());
        assertEquals(p.tt4(), back.tt4(), 0.5);
    }

    @Test void colderBurnsGoSupercriticalHotterSpillAndBuzz() {
        Ramjet.Sizing s = Ramjet.size(mach3(1.03), at20(3), 2000);
        Ramjet.Point design = s.point(), cool = Ramjet.at(s, at20(3), 1500), hot = Ramjet.at(s, at20(3), 2300);
        assertTrue(cool.recovery() < design.recovery(), "a cooler burn draws the shock deeper: " + cool.recovery());
        assertEquals(Ramjet.Mode.SUBCRITICAL, hot.mode(), "hotter than design: the shock is pushed out");
        assertTrue(hot.spillageDragN() > 0 && hot.airflowKgPerS() < design.airflowKgPerS());
        Ramjet.Sizing cold = Ramjet.size(mach3(1.03), at20(3), 1300);
        assertEquals(Ramjet.Mode.BUZZ, Ramjet.at(cold, at20(3), 2300).mode(), "far subcritical: buzz");
        // no fuel: only drag
        assertTrue(Ramjet.at(s, at20(3), 0).thrustN() < 0, "unlit it is drag");
        // slower, less thrust (it spills and compresses less)
        double previous = Double.MAX_VALUE;
        for (double m : new double[]{3, 2.5, 2, 1.6}) {
            double t = Ramjet.at(s, at20(m), 2000).thrustN();
            assertTrue(t < previous, "less at Mach " + m + ": " + t);
            previous = t;
        }
        // subsonic flight: no shocks, it takes what it asks
        Ramjet.Point sub = Ramjet.at(s, new Turbojet.Flight(288.15, 101325, 0.7 * 340.3), 2000);
        assertEquals(Ramjet.Mode.SUBSONIC, sub.mode());
        assertEquals(0.97, sub.recovery(), 1e-12);
    }
}
