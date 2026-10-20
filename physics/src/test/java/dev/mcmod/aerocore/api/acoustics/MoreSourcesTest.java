package dev.mcmod.aerocore.api.acoustics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** The other sound sources (0.37): horns, impacts, gas jets, electric machines, turbines, rotors, water and machining. */
class MoreSourcesTest {
    private static final int RATE = Voice.RATE;

    private static Spectrum.Power spectrum(Voice v, double seconds) {
        Voice.render(v, 0.3);
        return Spectrum.welch(Voice.render(v, seconds), RATE, 16384);
    }

    /** Whether a narrow band around f holds ten times the power of a band beside it. */
    private static boolean line(Spectrum.Power p, double f) {
        return p.band(f - 4, f + 4) > 10 * p.band(f + 40, f + 48);
    }

    @Test void hornsSoundTheirPitchesAndAShipsWhistleItsLength() {
        Horn car = new Horn(Horn.car(1));
        car.sound(true);
        Spectrum.Power p = spectrum(car, 2);
        assertTrue(line(p, 420) && line(p, 510), "a car's two horns, a major third apart");
        // COLREGs Annex III: 70-200 Hz at 200 m and over, 130-350 from 75 m, 250-700 below
        assertTrue(Horn.ship(250, 1).fundamentalsHz()[0] >= 70 && Horn.ship(250, 1).fundamentalsHz()[0] <= 200);
        assertTrue(Horn.ship(120, 1).fundamentalsHz()[0] >= 130 && Horn.ship(120, 1).fundamentalsHz()[0] <= 350);
        assertTrue(Horn.ship(12, 1).fundamentalsHz()[0] >= 250 && Horn.ship(12, 1).fundamentalsHz()[0] <= 700);
        assertTrue(Horn.ship(12, 1).fundamentalsHz()[0] > Horn.ship(60, 1).fundamentalsHz()[0], "a shorter vessel whistles higher");
        car.sound(false);
        Voice.render(car, 0.5);
        assertEquals(0, Dsp.rms(Voice.render(car, 0.2), 0, (int) (0.2 * RATE)), 1e-4, "and stops");
        assertEquals(-100, Horn.soundPowerDb(Horn.car(1), false));
    }

    @Test void aStruckPanelRingsAtItsPlateModesAndSoftGroundOnlyThuds() {
        Modal steel = ImpactNoise.modes(ImpactNoise.Spec.steelPart(0.5), 3);
        assertTrue(steel.longestDecayS() > 0.3, "lightly damped steel rings: " + steel.longestDecayS());
        float[] clang = ImpactNoise.render(ImpactNoise.Spec.steelPart(0.5), 1, 3);
        float[] thud = ImpactNoise.render(ImpactNoise.Spec.carPanel(Surface.SOIL), 1, 3);
        Spectrum.Power pc = Spectrum.welch(clang, RATE, 4096), pt = Spectrum.welch(thud, RATE, 4096);
        double clangHigh = pc.band(1000, 8000) / pc.total(), thudHigh = pt.band(1000, 8000) / pt.total();
        assertTrue(clangHigh > 5 * thudHigh, "steel rings high (" + clangHigh + "), soil thuds low (" + thudHigh + ")");
        double a = ImpactNoise.soundPowerDb(ImpactNoise.Spec.carPanel(Surface.ROCK), 1000, 5), b = ImpactNoise.soundPowerDb(ImpactNoise.Spec.carPanel(Surface.ROCK), 4000, 10);
        assertEquals(10 * Math.log10(8), b - a, 1e-9, "the radiated energy follows J v");
    }

    @Test void escapingAirHissesByLighthillAndRunsDown() {
        double u = GasJetNoise.jetSpeed(8.5e5, 101_325, 293);
        assertTrue(u > 480 && u < 560, "8.5 bar into the air expands to about 520 m/s: " + u);
        assertEquals(80 * Math.log10(2), Dsp.powerLevelDb(GasJetNoise.powerW(200, 0.01)) - Dsp.powerLevelDb(GasJetNoise.powerW(100, 0.01)), 1e-9);
        GasJetNoise.Spec brake = new GasJetNoise.Spec(0.006, 8.5e5, 0.015, 293, 1);   // a brake chamber's few litres through a relay valve
        double later = GasJetNoise.pressureAt(brake, 0.5, 101_325);
        assertTrue(later < 8.5e5 && later > 101_325, "the chamber empties: " + later);
        assertTrue(GasJetNoise.soundPowerDb(brake, 8.5e5, 101_325) > GasJetNoise.soundPowerDb(brake, later, 101_325));
        GasJetNoise g = new GasJetNoise(brake);
        g.release();
        float[] s = Voice.render(g, 2);
        assertTrue(Dsp.rms(s, 0, RATE / 10) > 3 * Dsp.rms(s, RATE / 2, RATE / 2 + RATE / 10), "loud at first, fading as it empties");
    }

    @Test void anElectricMachineHumsAtTwiceItsElectricalFrequency() {
        MotorNoise m = new MotorNoise(new MotorNoise.Spec(4, 36, 15_000, 0, 0, 2));
        m.set(new MotorNoise.State(25, 1, 0));
        Spectrum.Power p = spectrum(m, 2);
        assertTrue(line(p, 2 * 2 * 25), "4 poles at 25 rev/s: 50 Hz electrical, 100 Hz hum");
        assertTrue(line(p, 36 * 25), "and its slots at 900 Hz");
        MotorNoise grid = new MotorNoise(new MotorNoise.Spec(4, 36, 50_000, 0, 0, 2));
        grid.set(new MotorNoise.State(25, 0.5, 50));
        assertTrue(line(spectrum(grid, 2), 100), "on the grid: 100 Hz");
        assertTrue(MotorNoise.soundPowerDb(m.spec(), new MotorNoise.State(25, 1, 0)) > 80);
    }

    @Test void aTurbinesJetFollowsTheEighthPowerAndItsFanSingsAtItsBladePassing() {
        double a = Dsp.powerLevelDb(TurbineNoise.jetPowerW(300, 288, 0.5)), b = Dsp.powerLevelDb(TurbineNoise.jetPowerW(600, 288, 0.5));
        assertEquals(80 * Math.log10(2), b - a, 1e-9);
        double j79 = TurbineNoise.soundPowerDb(new TurbineNoise.Spec(TurbineNoise.Kind.TURBOJET, 0.7, 0, 0, 0, 120, 1),
                new TurbineNoise.State(120, 600, 900, 0, Math.toRadians(35), 0, true));
        assertTrue(j79 > 160 && j79 < 180, "a J79 at full: 160-180 dB of sound power: " + j79);
        TurbineNoise fan = new TurbineNoise(new TurbineNoise.Spec(TurbineNoise.Kind.TURBOFAN, 1.2, 24, 1.5, 0, 60, 4));
        fan.set(new TurbineNoise.State(40, 200, 600, 0, Math.toRadians(120), 0, true));
        assertTrue(line(spectrum(fan, 2), 24 * 40), "24 blades at 40 rev/s: 960 Hz");
        assertTrue(TurbineNoise.jetDirectivityDb(Math.toRadians(35)) > TurbineNoise.jetDirectivityDb(Math.toRadians(150)) + 10, "loudest aft at 35 degrees");
        TurbineNoise pulse = new TurbineNoise(new TurbineNoise.Spec(TurbineNoise.Kind.PULSEJET, 0.4, 0, 0, 0, 0, 5));
        pulse.set(new TurbineNoise.State(0, 0, 900, 1, 1, 45, true));
        List<Spectrum.Peak> peaks = Spectrum.peaks(spectrum(pulse, 3), 20, 300, 1, 10);
        assertEquals(45, peaks.getFirst().hz(), 3, "a pulsejet buzzes at its firing frequency");
    }

    @Test void propellersSingAtTheirBladePassingAndGrowWithTipMach() {
        RotorNoise prop = new RotorNoise(new RotorNoise.Spec(RotorNoise.Kind.PROPELLER, 2, 1.9, 6));
        prop.set(new RotorNoise.State(40, 120_000, 0, 0));
        Spectrum.Power p = spectrum(prop, 2);
        assertTrue(line(p, 80) && line(p, 160), "two blades at 40 rev/s: 80 Hz and its harmonics");
        RotorNoise.Spec s = new RotorNoise.Spec(RotorNoise.Kind.PROPELLER, 2, 1.9, 6);
        double light = RotorNoise.soundPowerDb(s, new RotorNoise.State(40, 120_000, 0, 0));
        assertTrue(light > 105 && light < 125, "a light aircraft's propeller at full: " + light);
        assertTrue(RotorNoise.soundPowerDb(s, new RotorNoise.State(30, 120_000, 0, 0)) < light - 5, "slower tips are quieter");
        assertTrue(RotorNoise.soundPowerDb(s, new RotorNoise.State(40, 120_000, 1, 0)) > light, "and a slap louder");
    }

    @Test void aBoatHissesFasterAndSlamsLikeWagnerSays() {
        WaterNoise.Spec boat = new WaterNoise.Spec(2, 20, 7);
        assertEquals(40 * Math.log10(2), WaterNoise.soundPowerDb(boat, new WaterNoise.State(20, 10, 0.3, 1)) - WaterNoise.soundPowerDb(boat, new WaterNoise.State(10, 10, 0.3, 1)), 1e-9);
        assertEquals(-100, WaterNoise.soundPowerDb(boat, new WaterNoise.State(10, 0, 0, 0)), "out of the water: silent");
        // Wagner: 1/2 rho v^2 (pi^2 / (4 tan^2 beta) + 1); at 20 deg and 5 m/s about 236 kPa
        assertEquals(0.5 * 1000 * 25 * (Math.PI * Math.PI / (4 * Math.pow(Math.tan(Math.toRadians(20)), 2)) + 1), WaterNoise.wagnerPressurePa(5, 20), 1e-6);
        assertTrue(WaterNoise.wagnerPressurePa(5, 10) > WaterNoise.wagnerPressurePa(5, 25), "a flatter bottom slams harder");
        WaterNoise w = new WaterNoise(boat);
        w.set(new WaterNoise.State(12, 8, 0.4, 1));
        w.slam(1);
        float[] s = Voice.render(w, 1);
        assertTrue(Dsp.rms(s, 0, RATE / 10) > Dsp.rms(s, RATE / 2, RATE / 2 + RATE / 10), "the slam's thud first");
    }

    @Test void machinesSoundByTheirProcessAndToothPassing() {
        ProcessNoise mill = new ProcessNoise(new ProcessNoise.Spec(ProcessNoise.Process.MILLING, 11_000, 4, 50, 0, 8));
        mill.set(new ProcessNoise.State(1, true));
        assertTrue(line(spectrum(mill, 3), 200), "four teeth at 50 rev/s: 200 Hz");
        for (ProcessNoise.Process p : ProcessNoise.Process.values()) {
            ProcessNoise n = new ProcessNoise(new ProcessNoise.Spec(p, 10_000, 4, 30, 1, 9));
            n.set(new ProcessNoise.State(1, true));
            float[] s = Voice.render(n, 1.5);
            double rms = Dsp.gainToDb(Dsp.rms(s, s.length / 3, s.length));
            assertTrue(rms > -45 && rms < -1, p + " at work: " + rms + " dBFS");
            for (float x : s) { assertTrue(Float.isFinite(x)); }
        }
        ProcessNoise.Spec lathe = new ProcessNoise.Spec(ProcessNoise.Process.TURNING, 10_000, 1, 20, 0, 1);
        assertEquals(80, ProcessNoise.soundPowerDb(lathe, new ProcessNoise.State(1, true)), 1e-9, "a 10 kW lathe turning");
        assertTrue(ProcessNoise.soundPowerDb(new ProcessNoise.Spec(ProcessNoise.Process.WATERJET, 10_000, 0, 0, 0, 1), new ProcessNoise.State(1, true)) > 95);
        assertEquals(-100, ProcessNoise.soundPowerDb(lathe, ProcessNoise.State.OFF));
    }
}
