package dev.mcmod.aerocore.api.acoustics;

/**
 * A delay line with fractional reads (0.37; cubic Hermite interpolation): pipe waveguides (an exhaust's
 * reflections), combs and all-passes, and reading a signal at a changing delay (the Doppler of a moving source).
 * Its length is rounded up to a power of two.
 */
public final class DelayLine {
    private final float[] buffer;
    private final int mask;
    private int write;

    public DelayLine(int maxSamples) {
        int n = Integer.highestOneBit(Math.max(4, maxSamples + 4) - 1) << 1;
        buffer = new float[n];
        mask = n - 1;
    }

    /** The longest delay that can be read, in samples. */
    public int capacity() { return buffer.length - 4; }

    public void write(double x) {
        buffer[write] = (float) x;
        write = (write + 1) & mask;
    }

    /** The sample written delay samples ago (1 = the last one written), interpolated between whole samples. */
    public double read(double delay) {
        double d = Math.max(1, Math.min(capacity(), delay));
        int i = (int) Math.floor(d);
        double f = d - i;
        double xm1 = at(i - 1), x0 = at(i), x1 = at(i + 1), x2 = at(i + 2);
        double c1 = 0.5 * (x1 - xm1), c2 = xm1 - 2.5 * x0 + 2 * x1 - 0.5 * x2, c3 = 0.5 * (x2 - xm1) + 1.5 * (x0 - x1);
        return ((c3 * f + c2) * f + c1) * f + x0;
    }

    private double at(int samplesAgo) { return buffer[(write - Math.max(1, samplesAgo)) & mask]; }

    /** A feedback comb: writes x + feedback times the sample delay ago, returns that delayed sample. */
    public double comb(double x, double delay, double feedback) {
        double y = read(delay);
        write(x + feedback * y);
        return y;
    }

    /** A Schroeder all-pass of gain g around this line. */
    public double allpass(double x, double delay, double g) {
        double d = read(delay), v = x + g * d;
        write(v);
        return d - g * v;
    }

    public void clear() { java.util.Arrays.fill(buffer, 0f); }
}
