package dev.mcmod.aerocore.api.acoustics;

import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Synthesis building blocks (0.37) against their textbook responses, and the analysis tools against known signals. */
class DspTest {
    private static final int RATE = Voice.RATE;

    private static float[] sine(double hz, double amplitude, double seconds) {
        float[] s = new float[(int) (seconds * RATE)];
        for (int i = 0; i < s.length; i++) { s[i] = (float) (amplitude * Math.sin(2 * Math.PI * hz * i / RATE)); }
        return s;
    }

    /** Gain (dB) of a filter for a sine, measured after it settles. */
    private static double gainDb(java.util.function.DoubleUnaryOperator filter, double hz) {
        float[] in = sine(hz, 1, 1);
        double si = 0, so = 0;
        for (int i = 0; i < in.length; i++) {
            double y = filter.applyAsDouble(in[i]);
            if (i > in.length / 2) { si += in[i] * in[i]; so += y * y; }
        }
        return 10 * Math.log10(so / si);
    }

    @Test void theSameSeedGivesTheSameNoiseAndPinkFallsThreeDecibelsAnOctave() {
        Noise a = new Noise(42), b = new Noise(42), c = new Noise(43);
        for (int i = 0; i < 1000; i++) { assertEquals(a.white(), b.white()); }
        assertNotEquals(new Noise(42).white(), c.white());
        Noise n = new Noise(7);
        float[] pink = new float[RATE * 8], white = new float[RATE * 8];
        for (int i = 0; i < pink.length; i++) { pink[i] = (float) n.pink(); white[i] = (float) n.white(); }
        Spectrum.Power p = Spectrum.welch(pink, RATE, 8192), w = Spectrum.welch(white, RATE, 8192);
        // four octaves apart: 12 dB less in a band of the same relative width
        double drop = p.bandDb(100, 200) - p.bandDb(1600, 3200);
        assertEquals(0, drop, 1.0, "pink: equal power per octave (" + drop + " dB)");
        double whiteDrop = w.bandDb(100, 200) - w.bandDb(1600, 3200);
        assertEquals(-12.04, whiteDrop, 1.0, "white: 3 dB more per octave (" + whiteDrop + ")");
        assertEquals(1 / 3.0, Dsp.rms(white, 0, white.length) * Dsp.rms(white, 0, white.length), 0.01, "white uniform noise has variance 1/3");
        assertEquals(Dsp.rms(white, 0, white.length), Dsp.rms(pink, 0, pink.length), 0.15, "pink about as loud as white");
    }

    @Test void filtersAreThreeDecibelsDownAtTheirCutoff() {
        Filters.Biquad lp = Filters.Biquad.lowpass(1000, Math.sqrt(0.5));
        assertEquals(-3.01, gainDb(lp::process, 1000), 0.2);
        double far = gainDb(Filters.Biquad.lowpass(1000, Math.sqrt(0.5))::process, 10000);
        assertTrue(far < -38 && far > -48, "12 dB per octave above, a little more near Nyquist (bilinear): " + far);
        Filters.Biquad hp = Filters.Biquad.highpass(200, Math.sqrt(0.5));
        assertEquals(-3.01, gainDb(hp::process, 200), 0.2);
        assertEquals(0, gainDb(Filters.Biquad.bandpass(500, 4)::process, 500), 0.2, "band-pass is 0 dB at its centre");
        assertEquals(6, gainDb(Filters.Biquad.peaking(800, 2, 6)::process, 800), 0.2);
        Filters.Svf svf = new Filters.Svf(2000, Math.sqrt(0.5));
        assertEquals(-3.01, gainDb(x -> { svf.process(x); return svf.low; }, 2000), 0.2);
        Filters.OnePole one = new Filters.OnePole(300);
        assertEquals(-3.01, gainDb(one::lowpass, 300), 0.4, "the one-pole matches an RC low-pass well below Nyquist");
    }

    @Test void aDelayLineReadsBetweenSamples() {
        DelayLine d = new DelayLine(4096);
        float[] s = sine(440, 1, 0.5);
        double maxError = 0;
        for (int i = 0; i < s.length; i++) {
            d.write(s[i]);
            if (i > 200) {
                double expected = Math.sin(2 * Math.PI * 440 * (i - 99.37) / RATE);
                maxError = Math.max(maxError, Math.abs(d.read(100.37) - expected));
            }
        }
        // read(1) is the last sample written, so read(100.37) is the sample at i - 99.37
        assertTrue(maxError < 1e-3, "cubic interpolation of a 440 Hz sine: " + maxError);
    }

    @Test void aBandLimitedSawHasHarmonicsFallingAsOneOverN() {
        Oscillator o = new Oscillator();
        float[] s = new float[RATE * 2];
        for (int i = 0; i < s.length; i++) { s[i] = (float) (0.5 * o.saw(220)); }
        Spectrum.Power p = Spectrum.welch(s, RATE, 16384);
        double h1 = p.bandDb(210, 230), h2 = p.bandDb(430, 450), h4 = p.bandDb(870, 890);
        assertEquals(-6.02, h2 - h1, 0.5);
        assertEquals(-12.04, h4 - h1, 0.5);
        // and almost nothing between harmonics (aliases of a naive saw would land there)
        assertTrue(p.bandDb(9300, 9400) < h1 - 45, "between the 42nd and 43rd harmonic: " + (p.bandDb(9300, 9400) - h1));
    }

    @Test void aModeRingsAtItsFrequencyAndDecaysSixtyDecibelsInItsTime() {
        Modal m = new Modal(new double[]{700}, new double[]{0.5}, new double[]{1});
        m.strike(1);
        float[] s = new float[RATE];
        for (int i = 0; i < s.length; i++) { s[i] = (float) m.next(0); }
        double early = Dsp.rms(s, 0, 2205), late = Dsp.rms(s, RATE / 2, RATE / 2 + 2205);
        assertEquals(-60, Dsp.gainToDb(late / early), 2.5, "60 dB down after 0.5 s");
        double peak = 0;
        for (int i = 0; i < 200; i++) { peak = Math.max(peak, Math.abs(s[i])); }
        assertEquals(1, peak, 0.05, "the strike's peak is the mode's gain");
        List<Spectrum.Peak> peaks = Spectrum.peaks(Spectrum.welch(s, RATE, 8192), 100, 5000, 1, 50);
        assertEquals(700, peaks.getFirst().hz(), 2);
    }

    @Test void theSpectrumFindsASineAndItsPower() {
        float[] s = sine(1000, 0.5, 2);
        Spectrum.Power p = Spectrum.welch(s, RATE, 8192);
        assertEquals(0.125, p.total(), 0.002, "mean square of a 0.5 sine");
        Spectrum.Peak peak = Spectrum.peaks(p, 50, 20000, 1, 10).getFirst();
        assertEquals(1000, peak.hz(), 1.0);
        assertEquals(-9.03, peak.db(), 0.8);
        assertEquals(-19.1, Spectrum.aWeightDb(100), 0.1);
        assertEquals(0, Spectrum.aWeightDb(1000), 0.05);
        assertEquals(-2.5, Spectrum.aWeightDb(10000), 0.1);
        double[] bands = Spectrum.thirdOctaves(p);
        int k = java.util.Arrays.binarySearch(Spectrum.THIRD_OCTAVES, 1000);
        assertTrue(bands[k] > bands[k - 1] + 20 && bands[k] > bands[k + 1] + 20, "the 1 kHz band holds the tone");
    }

    @Test void aWavFileReadsBackItsSamples() throws Exception {
        float[] s = sine(300, 0.7, 0.1);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        Wav.write(out, s, RATE, 1);
        Wav.Sound back = Wav.read(new ByteArrayInputStream(out.toByteArray()));
        assertEquals(RATE, back.rate());
        assertEquals(1, back.channels());
        assertEquals(s.length, back.samples().length);
        for (int i = 0; i < s.length; i++) { assertEquals(s[i], back.samples()[i], 1.0 / 32767 + 1e-6); }
        assertEquals(1f, Wav.read(new ByteArrayInputStream(writeOne(3f))).samples()[0], 1e-4, "clipped at full scale");
    }

    private static byte[] writeOne(float v) throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        Wav.write(out, new float[]{v}, RATE, 1);
        return out.toByteArray();
    }

    @Test void aMixRampsItsGainsAndKeepsSilentVoicesTurning() {
        Mix mix = new Mix();
        int[] calls = {0};
        Mix.Input a = mix.add((out, off, n) -> { calls[0]++; java.util.Arrays.fill(out, off, off + n, 1f); }, 0);
        float[] buf = new float[256];
        mix.render(buf, 0, 256);
        assertEquals(0, buf[255], 0, "silent");
        assertEquals(1, calls[0], "but rendered");
        a.gain(1);
        mix.render(buf, 0, 256);
        assertTrue(buf[0] > 0 && buf[0] < 0.01 && Math.abs(buf[255] - 1) < 1e-6, "ramped up over the block");
        a.remove();
        mix.render(buf, 0, 256);
        assertEquals(0, mix.size());
    }
}
