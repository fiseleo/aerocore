package dev.mcmod.aerocore.api.acoustics;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Construction machinery's sound sources (0.43): a hydraulic pump with its relief valve and cylinders, a crawler's track. */
class MachineryNoiseTest {
    private static final int RATE = Voice.RATE;
    /** rad/s per rpm. */
    private static final double RPM = 2 * Math.PI / 60;

    private static Spectrum.Power spectrum(Voice v, double seconds) {
        Voice.render(v, 0.3);
        return Spectrum.welch(Voice.render(v, seconds), RATE, 16384);
    }

    private static double rmsDb(float[] s) { return Dsp.gainToDb(Dsp.rms(s, s.length / 4, s.length)); }

    private static HydraulicNoise pump(HydraulicNoise.Spec spec, HydraulicNoise.State st) {
        HydraulicNoise h = new HydraulicNoise(spec);
        h.set(st);
        return h;
    }

    private static TrackNoise track(TrackNoise.Spec spec, TrackNoise.State st) {
        TrackNoise t = new TrackNoise(spec);
        t.set(st);
        return t;
    }

    private static double pumpDb(double shaftRadPerS, double pressurePa, double flowM3PerS) {
        return HydraulicNoise.pumpPowerDb(HydraulicNoise.Spec.axialPiston(35e6, 1), new HydraulicNoise.State(shaftRadPerS, pressurePa, flowM3PerS, 0));
    }

    private static double peakAbs(float[] s) {
        double m = 0;
        for (float x : s) { m = Math.max(m, Math.abs(x)); }
        return m;
    }

    @Test void theSameSeedGivesTheSameSamples() {
        HydraulicNoise.Spec ps = HydraulicNoise.Spec.axialPiston(35e6, 7);
        HydraulicNoise.State relieving = new HydraulicNoise.State(1800 * RPM, 30e6, 3e-3, 0.5);
        HydraulicNoise a = pump(ps, relieving), b = pump(ps, relieving), other = pump(HydraulicNoise.Spec.axialPiston(35e6, 8), relieving);
        a.bottomOut(0.7);
        b.bottomOut(0.7);
        other.bottomOut(0.7);
        float[] x = Voice.render(a, 0.5);
        assertArrayEquals(x, Voice.render(b, 0.5), "a pump with its relief and a clunk");
        assertFalse(Arrays.equals(x, Voice.render(other, 0.5)), "another seed, another hiss");
        TrackNoise.Spec ts = TrackNoise.Spec.steel(0.19, 7);
        TrackNoise.State turning = new TrackNoise.State(2.5, 0.8, 1.2e5);
        float[] y = Voice.render(track(ts, turning), 0.5);
        assertArrayEquals(y, Voice.render(track(ts, turning), 0.5), "a turning track");
        assertFalse(Arrays.equals(y, Voice.render(track(TrackNoise.Spec.steel(0.19, 9), turning), 0.5)));
    }

    @Test void aPumpSingsAtItsPumpingFrequencyWithHarmonicsFallingSixDecibelsAnOctave() {
        assertEquals(300, HydraulicNoise.pumpingHz(9, 2000 * RPM), 1e-9, "nine pistons at 2000 rpm: f = n z");
        HydraulicNoise.Spec spec = HydraulicNoise.Spec.axialPiston(35e6, 3);
        Spectrum.Power p = spectrum(pump(spec, new HydraulicNoise.State(2000 * RPM, 20e6, 2e-3, 0)), 3);
        List<Spectrum.Peak> peaks = Spectrum.peaks(p, 50, 3000, 1, 20);
        assertEquals(300, peaks.getFirst().hz(), p.binHz(), "the strongest line is the pumping frequency");
        Spectrum.Power pg = spectrum(pump(HydraulicNoise.Spec.gear(11, 25e6, 2), new HydraulicNoise.State(1800 * RPM, 12e6, 1e-3, 0)), 3);
        assertEquals(330, Spectrum.peaks(pg, 50, 3000, 1, 20).getFirst().hz(), pg.binHz(), "eleven gear teeth at 1800 rpm");
        // at rated pressure the ripple's harmonics fall as 1/k
        HydraulicNoise full = pump(spec, new HydraulicNoise.State(2000 * RPM, 35e6, 2e-3, 0));
        Spectrum.Power pr = spectrum(full, 3);
        double first = pr.band(292, 308);
        assertEquals(-6.02, 10 * Math.log10(pr.band(592, 608) / first), 1, "the second harmonic, an octave up");
        assertEquals(-12.04, 10 * Math.log10(pr.band(1192, 1208) / first), 1.5, "the fourth, two octaves up");
        double level = rmsDb(Voice.render(full, 1));
        assertTrue(level > -20 && level < -6, "the voice near -12 dBFS: " + level);
    }

    @Test void thePumpGrowsSixDecibelsADoublingOfPressure() {
        double n = 1500 * RPM;
        assertEquals(20 * Math.log10(2), pumpDb(n, 20e6, 1e-3) - pumpDb(n, 10e6, 1e-3), 1e-9, "the compression ripple grows as p");
        assertEquals(10 * Math.log10(2), pumpDb(n, 20e6, 2e-3) - pumpDb(n, 20e6, 1e-3), 1e-9, "twice the displacement");
        assertEquals(20 * Math.log10(2), pumpDb(2 * n, 20e6, 2e-3) - pumpDb(n, 20e6, 1e-3), 1e-9, "twice the speed, and so the flow");
        assertEquals(85, pumpDb(n, 28e6, 71e-6 * 25), 0.5, "a 71 cm3 piston pump at 1500 rpm and 280 bar");
        assertEquals(pumpDb(n, 2.5e6, 1e-3), pumpDb(n, 0, 1e-3), 1e-9, "unloaded it keeps its standby level");
        HydraulicNoise.Spec spec = HydraulicNoise.Spec.axialPiston(35e6, 1);
        HydraulicNoise.State working = new HydraulicNoise.State(n, 20e6, 1e-3, 0);
        assertEquals(HydraulicNoise.pumpPowerDb(spec, working), HydraulicNoise.soundPowerDb(spec, working), 1e-9, "no relief: the circuit is its pump");
        assertTrue(HydraulicNoise.soundPowerDb(HydraulicNoise.Spec.gear(11, 25e6, 1), working) > HydraulicNoise.soundPowerDb(new HydraulicNoise.Spec(HydraulicNoise.Kind.VANE, 11, 25e6, 1), working),
                "a gear pump whines louder than a vane pump");
        assertEquals(-100, HydraulicNoise.soundPowerDb(spec, HydraulicNoise.State.OFF));
    }

    @Test void aReliefValveAddsAHissAboveTwoKilohertz() {
        HydraulicNoise.Spec spec = HydraulicNoise.Spec.axialPiston(35e6, 3);
        HydraulicNoise.State working = new HydraulicNoise.State(2000 * RPM, 20e6, 2e-3, 0), relieving = new HydraulicNoise.State(2000 * RPM, 20e6, 2e-3, 1);
        Spectrum.Power quiet = spectrum(pump(spec, working), 3), hiss = spectrum(pump(spec, relieving), 3);
        double ratio = hiss.band(2000, 10_000) / quiet.band(2000, 10_000);
        assertTrue(ratio > 10, "all the flow over the relief: " + 10 * Math.log10(ratio) + " dB more above 2 kHz");
        assertEquals(-100, HydraulicNoise.reliefPowerDb(spec, working));
        HydraulicNoise.State stalled = new HydraulicNoise.State(1800 * RPM, 35e6, 4e-3, 1), half = new HydraulicNoise.State(1800 * RPM, 35e6, 4e-3, 0.5);
        assertEquals(93.5, HydraulicNoise.reliefPowerDb(spec, stalled), 1, "4 l/s over a 35 MPa relief");
        assertEquals(10 * Math.log10(2), HydraulicNoise.reliefPowerDb(spec, stalled) - HydraulicNoise.reliefPowerDb(spec, half), 1e-9, "half the flow, half the stream's power");
        assertTrue(HydraulicNoise.soundPowerDb(spec, relieving) > HydraulicNoise.pumpPowerDb(spec, relieving) + 2, "the relief is heard over the pump");
    }

    @Test void aCylinderBottomingClunksAndDiesAway() {
        HydraulicNoise h = new HydraulicNoise(HydraulicNoise.Spec.axialPiston(35e6, 3));
        h.bottomOut(1);
        float[] s = Voice.render(h, 1);
        double clunk = Dsp.rms(s, 0, RATE / 10), later = Dsp.rms(s, 6 * RATE / 10, 7 * RATE / 10);
        assertTrue(clunk > 0.05 && clunk > 100 * later, "a clunk (" + clunk + ") that rings out (" + later + ")");
        assertEquals(20 * Math.log10(2), HydraulicNoise.bottomOutPowerDb(2000, 0.6) - HydraulicNoise.bottomOutPowerDb(2000, 0.3), 1e-9, "its energy is 1/2 m v^2");
        assertEquals(-100, HydraulicNoise.bottomOutPowerDb(2000, 0));
    }

    @Test void trackLinksStrikeAtBeltSpeedOverPitch() {
        assertEquals(50, TrackNoise.linkHz(0.16, 8), 1e-9);
        assertEquals(17.6 * 10 / 6, TrackNoise.linkHz(6 * 0.0254, 10 * 0.44704), 0.05, "an M113 at 10 mph on 6 inch shoes");
        Spectrum.Power p = spectrum(track(TrackNoise.Spec.steel(0.16, 5), new TrackNoise.State(8, 0, 6e4)), 3);
        double f = 50, on = 0, off = 0;
        for (int k = 1; k <= 6; k++) { on += p.band(k * f - 2, k * f + 2); off += p.band((k + 0.5) * f - 2, (k + 0.5) * f + 2); }
        assertTrue(10 * Math.log10(on / off) > 10, "lines at multiples of v / p: " + 10 * Math.log10(on / off) + " dB above between");
        assertTrue(p.band(f - 2, f + 2) > 5 * p.band(1.5 * f - 2, 1.5 * f + 2), "once a link (50 Hz), not once every other link");
        double level = rmsDb(Voice.render(track(TrackNoise.Spec.steel(0.16, 5), new TrackNoise.State(8, 0, 6e4)), 1.5));
        assertTrue(level > -24 && level < -6, "the voice near -12 dBFS: " + level);
        TrackNoise.Spec steel = TrackNoise.Spec.steel(0.16, 5);
        assertEquals(112, TrackNoise.soundPowerDb(steel, new TrackNoise.State(10, 0, 6e4)), 1e-9);
        assertEquals(30 * Math.log10(2), TrackNoise.soundPowerDb(steel, new TrackNoise.State(20, 0, 6e4)) - TrackNoise.soundPowerDb(steel, new TrackNoise.State(10, 0, 6e4)), 1e-9,
                "impact energy v^2 times v / p impacts a second");
        assertEquals(5 * Math.log10(2), TrackNoise.soundPowerDb(steel, new TrackNoise.State(10, 0, 1.2e5)) - TrackNoise.soundPowerDb(steel, new TrackNoise.State(10, 0, 6e4)), 1e-9,
                "heavier, mildly louder");
    }

    @Test void rubberIsQuieterWithoutTheClankAndSlippingSqueals() {
        TrackNoise.State run = new TrackNoise.State(8, 0, 6e4);
        TrackNoise.Spec steel = TrackNoise.Spec.steel(0.16, 5), rubber = TrackNoise.Spec.rubber(0.16, 5);
        assertEquals(10, TrackNoise.soundPowerDb(steel, run) - TrackNoise.soundPowerDb(rubber, run), 1e-9, "a rubber band 10 dB below");
        Spectrum.Power ps = spectrum(track(steel, run), 3), pr = spectrum(track(rubber, run), 3);
        double steelHigh = ps.band(1000, 8000) / ps.total(), rubberHigh = pr.band(1000, 8000) / pr.total();
        assertTrue(steelHigh > 10 * rubberHigh, "steel clanks above 1 kHz (" + steelHigh + "), rubber only thumps (" + rubberHigh + ")");
        TrackNoise.Spec dozer = TrackNoise.Spec.steel(0.2, 7);
        TrackNoise.State straight = new TrackNoise.State(3, 0, 6e4), pivoting = new TrackNoise.State(3, 1.2, 6e4);
        assertEquals(0, TrackNoise.squeal(straight));
        assertEquals(1, TrackNoise.squeal(pivoting), 1e-12);
        assertTrue(TrackNoise.soundPowerDb(dozer, pivoting) > TrackNoise.soundPowerDb(dozer, straight) + 10, "a pivot turn squeals over the rolling");
        Spectrum.Power a = spectrum(track(dozer, straight), 2), b = spectrum(track(dozer, pivoting), 2);
        assertTrue(b.band(1500, 3000) > 3 * a.band(1500, 3000), "the squeal's band");
    }

    @Test void standingStillIsSilence() {
        HydraulicNoise h = pump(HydraulicNoise.Spec.axialPiston(35e6, 1), HydraulicNoise.State.OFF);
        TrackNoise t = track(TrackNoise.Spec.steel(0.2, 1), TrackNoise.State.STILL);
        assertEquals(0, peakAbs(Voice.render(h, 0.5)), 0, "a pump standing");
        assertEquals(0, peakAbs(Voice.render(t, 0.5)), 0, "a track standing");
        h.set(new HydraulicNoise.State(2000 * RPM, 20e6, 2e-3, 0.3));
        t.set(new TrackNoise.State(8, 0.5, 6e4));
        Voice.render(h, 0.5);
        Voice.render(t, 0.5);
        h.set(HydraulicNoise.State.OFF);
        t.set(TrackNoise.State.STILL);
        Voice.render(h, 2);
        Voice.render(t, 2);
        assertTrue(peakAbs(Voice.render(h, 0.2)) < 1e-4, "and the pump falls quiet when it stops");
        assertTrue(peakAbs(Voice.render(t, 0.2)) < 1e-4, "and the track");
        assertEquals(-100, TrackNoise.soundPowerDb(TrackNoise.Spec.steel(0.2, 1), TrackNoise.State.STILL));
        assertEquals(0, TrackNoise.linkHz(0.2, 0));
    }

    @Test void wildStatesNeverGiveNaNOrInfinity() {
        double[] wild = {0, -0.0, Double.MIN_VALUE, 1e-300, -1, 1, 37, 2.5e7, -1e6, 1e300, Double.MAX_VALUE, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        int w = wild.length;
        float[] buffer = new float[256];
        HydraulicNoise.Spec[] pumps = {HydraulicNoise.Spec.axialPiston(35e6, 1), HydraulicNoise.Spec.gear(11, 25e6, 2),
                new HydraulicNoise.Spec(HydraulicNoise.Kind.VANE, 0, Double.NaN, 3), new HydraulicNoise.Spec(null, -4, -1, 4),
                new HydraulicNoise.Spec(HydraulicNoise.Kind.GEAR, 1000, 1e300, 5)};
        for (HydraulicNoise.Spec spec : pumps) {
            HydraulicNoise h = new HydraulicNoise(spec);
            for (int i = 0; i < w; i++) {
                for (int j = 0; j < w; j++) {
                    HydraulicNoise.State st = new HydraulicNoise.State(wild[i], wild[j], wild[(i + j) % w], wild[(3 * i + j) % w]);
                    h.set(st);
                    if (j == 0) { h.bottomOut(wild[i]); }
                    h.render(buffer, 0, buffer.length);
                    for (float x : buffer) { assertTrue(Float.isFinite(x) && Math.abs(x) <= 1, () -> "pump " + spec + " at " + st + ": " + x); }
                    for (double db : new double[]{HydraulicNoise.soundPowerDb(spec, st), HydraulicNoise.pumpPowerDb(spec, st), HydraulicNoise.reliefPowerDb(spec, st),
                            HydraulicNoise.bottomOutPowerDb(wild[i], wild[j]), HydraulicNoise.pumpingHz(spec.pistons(), wild[i])}) {
                        assertTrue(Double.isFinite(db), () -> "a level at " + st);
                    }
                }
            }
        }
        TrackNoise.Spec[] tracks = {TrackNoise.Spec.steel(0.2, 1), TrackNoise.Spec.rubber(0.08, 2), new TrackNoise.Spec(0, false, 0, 3),
                new TrackNoise.Spec(Double.NaN, true, -3, 4), new TrackNoise.Spec(1e300, false, 100, 5), new TrackNoise.Spec(-0.2, false, 5, 6),
                new TrackNoise.Spec(Double.MIN_VALUE, false, 5, 7)};
        for (TrackNoise.Spec spec : tracks) {
            TrackNoise t = new TrackNoise(spec);
            for (int i = 0; i < w; i++) {
                for (int j = 0; j < w; j++) {
                    TrackNoise.State st = new TrackNoise.State(wild[i], wild[j], wild[(i + j) % w]);
                    t.set(st);
                    t.render(buffer, 0, buffer.length);
                    for (float x : buffer) { assertTrue(Float.isFinite(x) && Math.abs(x) <= 1, () -> "track " + spec + " at " + st + ": " + x); }
                    assertTrue(Double.isFinite(TrackNoise.soundPowerDb(spec, st)) && Double.isFinite(TrackNoise.linkHz(spec.pitchM(), wild[i]))
                            && Double.isFinite(TrackNoise.squeal(st)), () -> "a level at " + st);
                }
            }
        }
    }
}
