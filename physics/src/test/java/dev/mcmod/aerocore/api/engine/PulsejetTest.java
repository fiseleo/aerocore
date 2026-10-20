package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PulsejetTest {
    /**
     * The Argus As 014 (V-1): 3.65 m, chamber 0.58 m, tailpipe 0.38 x 1.75 m (Simpson's measurements of a surviving
     * engine); the reeds' open area is not published (0.04 m^2 here, a sixth of the grid's face).
     */
    static Pulsejet.Design argus() {
        return new Pulsejet.Design(Pulsejet.Kind.VALVED, 0, 0, 0.04, 5000, 200, 0.58, 0.9, 1.0, 0.38, 1.75, false, 3e-3, 0.9, Fuel.gasoline(70, 60));
    }

    /** A hobby-size Lockwood-Hiller: 1.2 m, chamber 0.16 m, intake bent rearward, on propane. */
    static Pulsejet.Design lockwood(double scale) {
        return new Pulsejet.Design(Pulsejet.Kind.VALVELESS, 0.08 * scale, 0.15 * scale, 0, 0, 0, 0.16 * scale, 0.30 * scale, 0.15 * scale, 0.08 * scale,
                0.60 * scale, true, 3e-3 * scale, 0.9, Fuel.PROPANE);
    }

    @Test void theArgusGivesItsPublishedThrustFuelAndFrequency() {
        // static: 500 lbf (2.22 kN) on 3.4 lb/(lbf h) (96 mg/(N s): 0.21 kg/s), about 45 Hz
        Pulsejet.Cycle still = Pulsejet.run(argus(), Turbojet.Flight.sea(0), 0.21);
        assertTrue(still.running());
        assertEquals(2224, still.thrustN(), 0.25 * 2224, "static thrust");
        assertEquals(96e-6, still.tsfc(), 0.25 * 96e-6, "fuel consumption");
        assertEquals(45, still.frequencyHz(), 0.25 * 45, "frequency");
        // in flight at 640 km/h about 3.3 kN on 3-4 lb/(lbf h)
        Pulsejet.Cycle flying = Pulsejet.run(argus(), Turbojet.Flight.sea(178), 0.30);
        assertTrue(flying.running());
        assertEquals(3300, flying.thrustN(), 0.25 * 3300, "thrust at 640 km/h");
        assertTrue(flying.thrustN() > still.thrustN(), "the ram adds thrust");
        assertEquals(0.21, argus().nominalFuelKgPerS(), 0.02, "the map's scale is the engine's fuel");
    }

    @Test void withoutFuelTheTubeDiesAndGivesNoThrust() {
        Pulsejet.Cycle c = Pulsejet.run(argus(), Turbojet.Flight.sea(0), 0);
        assertFalse(c.running());
        assertEquals(0, c.thrustN(), 1);
        assertEquals(0, c.frequencyHz());
    }

    @Test void aStraightTubeRingsAtItsQuarterWave() {
        // a chamber the tailpipe's width: the cycle is the quarter-wave of the tube's own mean sound speed
        for (double tail : new double[]{1.0, 2.0}) {
            Pulsejet.Design tube = new Pulsejet.Design(Pulsejet.Kind.VALVED, 0, 0, 0.004, 3000, 400, 0.12, 0.25, 0, 0.12, tail, false, 2e-3, 0.9, Fuel.PROPANE);
            Pulsejet.Solver s = new Pulsejet.Solver(tube, Turbojet.Flight.sea(0), 0.004, 100);
            double t = 0, span = 0;
            double[] flux = new double[3], c = new double[s.n];
            while (t < 0.3) {
                double dt = s.timeStep();
                s.step(dt, flux);
                t += dt;
                if (t > 0.15) {
                    span += dt;
                    for (int i = 0; i < s.n; i++) { c[i] += Math.sqrt(Pulsejet.GAMMA * s.p(i) / s.rho(i)) * dt; }
                }
            }
            double travel = 0;
            for (int i = 0; i < s.n; i++) { travel += s.dx * span / c[i]; }
            Pulsejet.Cycle cycle = Pulsejet.run(tube, Turbojet.Flight.sea(0), 0.004, 100, 0.3);
            assertTrue(cycle.running());
            assertEquals(1 / (4 * travel), cycle.frequencyHz(), 0.1 / (4 * travel), "tail " + tail);
        }
    }

    @Test void massIsConservedThroughTheCycle() {
        Pulsejet.Solver s = new Pulsejet.Solver(argus(), Turbojet.Flight.sea(0), 0.21, 120);
        double start = s.mass(), t = 0, through = 0;
        double[] flux = new double[3];
        while (t < 0.2) {
            double dt = s.timeStep();
            double before = s.boundaryMass;
            s.step(dt, flux);
            through += Math.abs(s.boundaryMass - before);
            t += dt;
        }
        assertEquals(start + s.boundaryMass + 0.21 * t, s.mass(), 1e-6 * through);
    }

    @Test void theGridConverges() {
        Pulsejet.Cycle coarse = Pulsejet.run(argus(), Turbojet.Flight.sea(0), 0.21, 60, 0.5), fine = Pulsejet.run(argus(), Turbojet.Flight.sea(0), 0.21, 240, 0.5);
        assertEquals(fine.thrustN(), coarse.thrustN(), 0.03 * fine.thrustN());
        assertEquals(fine.frequencyHz(), coarse.frequencyHz(), 0.02 * fine.frequencyHz());
    }

    @Test void aValvelessEngineRunsAndScalesWithItsSize() {
        Pulsejet.Cycle small = Pulsejet.run(lockwood(1), Turbojet.Flight.sea(0), 0.008);
        assertTrue(small.running());
        assertTrue(small.frequencyHz() > 100 && small.frequencyHz() < 250, "hobby valveless engines ring at 100-250 Hz: " + small.frequencyHz());
        assertTrue(small.thrustN() > 20 && small.thrustN() < 120, "tens of newtons: " + small.thrustN());
        assertTrue(small.tsfc() > 90e-6 && small.tsfc() < 200e-6, "valveless engines burn 0.35-0.7 kg/(N h): " + small.tsfc());
        // twice the size, its mixing time and four times the fuel: half the frequency, four times the thrust
        Pulsejet.Cycle big = Pulsejet.run(lockwood(2), Turbojet.Flight.sea(0), 0.032);
        assertTrue(big.running());
        assertEquals(small.frequencyHz() / 2, big.frequencyHz(), 0.1 * small.frequencyHz() / 2);
        assertEquals(4 * small.thrustN(), big.thrustN(), 0.25 * 4 * small.thrustN());
    }

    @Test void theMapIsItsRunsAndInterpolatesBetweenThem() {
        Pulsejet.Map map = Pulsejet.Map.build(argus(), 250, 0.35);
        Pulsejet.Cycle corner = map.cycle(2, 4), again = Pulsejet.run(argus(), Turbojet.Flight.sea(100), 0.35 * 4 / 6, 80, 0.4);
        assertEquals(again.thrustN(), corner.thrustN(), 1e-9 * Math.abs(again.thrustN()));
        assertEquals(again.thrustN(), map.at(Turbojet.Flight.sea(100), 0.35 * 4 / 6).thrustN(), 1e-6 * Math.abs(again.thrustN()));
        Pulsejet.Cycle between = map.at(Turbojet.Flight.sea(120), 0.25), direct = Pulsejet.run(argus(), Turbojet.Flight.sea(120), 0.25);
        assertTrue(between.running());
        assertEquals(direct.thrustN(), between.thrustN(), 0.1 * direct.thrustN());
        assertFalse(map.at(Turbojet.Flight.sea(0), 0).running(), "no fuel, no cycle");
        // altitude by similarity: half the pressure at the same temperature, half the fuel: half the thrust
        Turbojet.Flight high = new Turbojet.Flight(288.15, 101325 / 2.0, 100);
        assertEquals(map.at(Turbojet.Flight.sea(100), 0.25).thrustN() / 2, map.at(high, 0.125).thrustN(), 1e-6);
    }
}
