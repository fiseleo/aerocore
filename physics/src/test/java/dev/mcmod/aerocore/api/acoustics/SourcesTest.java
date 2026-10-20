package dev.mcmod.aerocore.api.acoustics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Sound sources (0.37): what they sound like (their spectra) and how loud they are, against engine and road facts. */
class SourcesTest {
    private static final int RATE = Voice.RATE;

    private static PistonEngineNoise.Spec engine(double[] tdc, int[] bank, int banks, double litres) {
        return new PistonEngineNoise.Spec(tdc.length, PistonEngineNoise.firingAngles(tdc), bank, banks, litres, 0.086, 2.5,
                600, 0, 120, false, 4, 0, 0, false, 11);
    }

    private static float[] run(Voice v, double seconds) { return Voice.render(v, seconds); }

    private static double rmsDb(float[] s) { return Dsp.gainToDb(Dsp.rms(s, s.length / 4, s.length)); }

    @Test void cylindersFireAsEvenlyAsTheirCrankAllows() {
        assertArrayEquals(new double[]{0, 540, 180, 360}, PistonEngineNoise.firingAngles(new double[]{0, 180, 180, 0}), 1e-9, "inline four, 1-3-4-2");
        double[] three = PistonEngineNoise.firingAngles(new double[]{0, 120, 240});
        double[] s3 = three.clone();
        java.util.Arrays.sort(s3);
        assertArrayEquals(new double[]{0, 240, 480}, s3, 1e-9, "inline three: every 240 degrees");
        // a flat-plane V8 at 90 degrees: every 90 degrees
        double[] v8 = PistonEngineNoise.firingAngles(new double[]{0, 180, 180, 0, 90, 270, 270, 90});
        double[] s8 = v8.clone();
        java.util.Arrays.sort(s8);
        for (int i = 0; i < 8; i++) { assertEquals(90 * i, s8[i], 1e-9); }
        // a 45-degree V-twin cannot be even: 405 and 315
        double[] twin = PistonEngineNoise.firingAngles(new double[]{0, 45});
        assertEquals(405, Math.abs(twin[1] - twin[0]), 1e-9);
    }

    /** Power in narrow bands at multiples of f (k = 1..count), over the power halfway between them. */
    private static double harmonicContrastDb(Spectrum.Power p, double f, int count) {
        double on = 0, off = 0;
        for (int k = 1; k <= count; k++) { on += p.band(k * f - 2, k * f + 2); off += p.band((k + 0.5) * f - 2, (k + 0.5) * f + 2); }
        return 10 * Math.log10(on / off);
    }

    @Test void anInlineFourSingsAtItsFiringFrequencyAndAVTwinAtHalfOrders() {
        PistonEngineNoise four = new PistonEngineNoise(engine(new double[]{0, 180, 180, 0}, new int[]{0, 0, 0, 0}, 1, 2));
        four.set(new PistonEngineNoise.State(3000, 0.6, 0.5, true, false, 1, 0));
        Spectrum.Power p = Spectrum.welch(run(four, 4), RATE, 16384);
        assertTrue(harmonicContrastDb(p, 100, 6) > 10, "3000 rpm, four cylinders: lines at 100 Hz multiples, " + harmonicContrastDb(p, 100, 6) + " dB above between");
        // an even twin (360 degrees apart) has only whole orders of its 50 Hz firing; a 45-degree V-twin has the cycle's 25 Hz
        PistonEngineNoise even = new PistonEngineNoise(engine(new double[]{0, 0}, new int[]{0, 0}, 1, 1.2));
        PistonEngineNoise vee = new PistonEngineNoise(engine(new double[]{0, 45}, new int[]{0, 0}, 1, 1.2));
        PistonEngineNoise.State st = new PistonEngineNoise.State(3000, 0.6, 0.5, true, false, 1, 0);
        even.set(st);
        vee.set(st);
        Spectrum.Power pe = Spectrum.welch(run(even, 4), RATE, 16384), pv = Spectrum.welch(run(vee, 4), RATE, 16384);
        double halfEven = pe.band(73, 77) / pe.band(98, 102), halfVee = pv.band(73, 77) / pv.band(98, 102);
        assertTrue(halfVee > 10 * halfEven, "the V-twin's 75 Hz half order against the even twin's: " + halfVee + " vs " + halfEven);
    }

    @Test void theEnginesVoiceKeepsItsLevelAndItsLoudnessComesFromTheModel() {
        PistonEngineNoise.Spec spec = engine(new double[]{0, 180, 180, 0}, new int[]{0, 0, 0, 0}, 1, 2);
        PistonEngineNoise idle = new PistonEngineNoise(spec), redline = new PistonEngineNoise(spec);
        idle.set(new PistonEngineNoise.State(800, 0.2, 0.02, true, false, 1, 0));
        redline.set(new PistonEngineNoise.State(6500, 1, 1, true, false, 1, 0));
        double a = rmsDb(run(idle, 2)), b = rmsDb(run(redline, 2));
        assertTrue(a > -28 && a < -4 && b > -28 && b < -4, "about -12 dBFS at idle " + a + " and at the redline " + b);
        double full = PistonEngineNoise.soundPowerDb(spec, new PistonEngineNoise.State(3000, 1, 1, true, false, 1, 0));
        assertEquals(100.5, full, 2.5, "a 2-litre road car at full load, 3000 rpm");
        assertEquals(71.5, Propagation.levelAt(full, 0, 7.5), 3, "about the ISO 362 pass-by at 7.5 m");
        double atIdle = PistonEngineNoise.soundPowerDb(spec, new PistonEngineNoise.State(800, 0.2, 0.02, true, false, 1, 0));
        assertTrue(atIdle < full - 20, "idle much quieter: " + atIdle);
        assertEquals(-100, PistonEngineNoise.soundPowerDb(spec, PistonEngineNoise.State.OFF));
        PistonEngineNoise one = new PistonEngineNoise(spec), two = new PistonEngineNoise(spec);
        one.set(new PistonEngineNoise.State(3000, 0.6, 0.5, true, false, 1, 0));
        two.set(new PistonEngineNoise.State(3000, 0.6, 0.5, true, false, 1, 0));
        float[] x = run(one, 0.5), y = run(two, 0.5);
        assertArrayEquals(x, y, "the same seed gives the same sound");
    }

    @Test void aTurbochargerWhistlesAtItsSecondShaftOrder() {
        double rev = PistonEngineNoise.turboRevPerS(1.8, 0.05);
        assertTrue(rev * 60 > 80_000 && rev * 60 < 200_000, "a 50 mm wheel at 1.8 bar absolute: " + rev * 60 + " rpm");
        PistonEngineNoise.Spec spec = new PistonEngineNoise.Spec(4, PistonEngineNoise.firingAngles(new double[]{0, 180, 180, 0}), new int[4], 1, 2, 0.086,
                2.5, 600, 0, 120, false, 4, 11, 0.05, true, 3);
        PistonEngineNoise e = new PistonEngineNoise(spec);
        e.set(new PistonEngineNoise.State(4000, 1, 1, true, false, 1.8, 0));
        Spectrum.Power p = Spectrum.welch(run(e, 3), RATE, 16384);
        double whistle = PistonEngineNoise.turboWhistleHz(1.8, 0.05);
        assertEquals(2 * rev, whistle, 1e-9);
        assertTrue(whistle > 2000 && whistle < 8000, "audible: " + whistle);
        assertTrue(p.band(whistle - 30, whistle + 30) > 10 * p.band(whistle + 300, whistle + 360), "a line at " + whistle + " Hz");
    }

    @Test void tyresRollLouderWithSpeedAndSquealWhenTheySlide() {
        TyreNoise.Spec road = new TyreNoise.Spec(TyreNoise.Tread.ROAD, 0.31, 0.205, 5);
        assertEquals(714.3, TyreNoise.blockHz(TyreNoise.Tread.ROAD, 20), 0.1);
        TyreNoise.State at80 = new TyreNoise.State(22.2, 0, 4000, 5000, Surface.ROCK, false, false);
        TyreNoise.State at160 = new TyreNoise.State(44.4, 0, 4000, 5000, Surface.ROCK, false, false);
        double slope = TyreNoise.soundPowerDb(road, at160) - TyreNoise.soundPowerDb(road, at80);
        assertEquals(33 * Math.log10(2), slope, 0.01, "the CPX slope");
        assertEquals(94, TyreNoise.soundPowerDb(road, at80), 1);
        assertEquals(0, TyreNoise.squeal(at80));
        TyreNoise.State sliding = new TyreNoise.State(15, 4, 4000, 5000, Surface.ROCK, false, false);
        assertTrue(TyreNoise.squeal(sliding) > 0.9);
        assertEquals(0, TyreNoise.squeal(new TyreNoise.State(15, 4, 4000, 5000, Surface.GRAVEL, false, false)), "gravel crunches, not squeals");
        assertTrue(TyreNoise.soundPowerDb(road, sliding) > TyreNoise.soundPowerDb(road, new TyreNoise.State(15, 0, 4000, 5000, Surface.ROCK, false, false)) + 5);
        TyreNoise t = new TyreNoise(road);
        t.set(sliding);
        float[] s = run(t, 3);
        List<Spectrum.Peak> peaks = Spectrum.peaks(Spectrum.welch(s, RATE, 16384), 500, 1500, 1, 50);
        double expected = 700 + 400 * (4 / 6.0) + 150 * (4000 / 5000.0);
        assertEquals(expected, peaks.getFirst().hz(), 40, "the squeal's pitch");
        TyreNoise rolling = new TyreNoise(road);
        rolling.set(at80);
        double level = rmsDb(run(rolling, 2));
        assertTrue(level > -32 && level < -4, "rolling at -12 dBFS or so: " + level);
    }

    @Test void windGrowsWithTheSixthPowerOfSpeed() {
        FlowNoise.Spec car = new FlowNoise.Spec(2.2, 0.04, 1);
        assertEquals(85, FlowNoise.soundPowerDb(car, new FlowNoise.State(30, 0, 0)), 1e-9);
        assertEquals(60 * Math.log10(2), FlowNoise.soundPowerDb(car, new FlowNoise.State(60, 0, 0)) - FlowNoise.soundPowerDb(car, new FlowNoise.State(30, 0, 0)), 1e-9);
        FlowNoise w = new FlowNoise(car);
        w.set(new FlowNoise.State(30, 0, 0.3));
        double level = rmsDb(run(w, 2));
        assertTrue(level > -36 && level < -4, "wind at " + level);
    }
}
