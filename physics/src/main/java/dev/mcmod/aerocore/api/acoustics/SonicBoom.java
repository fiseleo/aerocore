package dev.mcmod.aerocore.api.acoustics;

/**
 * The sonic boom of a body in steady level supersonic flight (0.37), by Carlson's simplified method (NASA TP-1122,
 * 1978): the N-wave's peak overpressure
 * dp = K_p K_r sqrt(p_v p_g) (M^2 - 1)^(1/8) h^(-3/4) l^(3/4) K_s
 * and its duration dt = K_t (3.42 / a_v) M (M^2 - 1)^(-3/8) h^(1/4) l^(3/4) K_s, with p_v, a_v the ambient pressure and
 * sound speed at the body, p_g at the listener, h the height above the listener, l the body length, K_s its shape
 * factor (about 0.1 for transport and fighter shapes, lower for very slender ones), K_r the reflection factor (1.9 on
 * the ground; 1 for a listener in the air) and K_p, K_t Carlson's atmosphere factors (near 1 for flight below about 20
 * km and Mach 1.3-3, taken as 1 here). For Concorde (l 62 m, M 2, 16.8 km) it gives about 2 psf (96 Pa), what was
 * measured. Near the ground (h of a few hundred metres, as in the game) it is an extrapolation of the far-field law.
 */
public final class SonicBoom {
    private SonicBoom() {}

    /** Carlson's typical shape factor. */
    public static final double SHAPE = 0.1;
    /** Reflection factor of a hard ground (the incident and reflected waves arrive together). */
    public static final double GROUND = 1.9;

    /** An N-wave: peak overpressure (Pa) and duration (s). */
    public record NWave(double overpressurePa, double durationS) {}

    /**
     * The boom of a body of length l (m) at Mach m, height h (m) above the listener, ambient pressures at the body and
     * at the listener (Pa), sound speed at the body (m/s), shape factor and reflection factor. Below Mach 1 there is none.
     */
    public static NWave nWave(double lengthM, double mach, double heightM, double bodyPressurePa, double listenerPressurePa,
                              double bodySoundMps, double shape, double reflection) {
        if (!(mach > 1) || !(lengthM > 0)) { return new NWave(0, 0); }
        double h = Math.max(heightM, lengthM), m2 = mach * mach - 1;
        double scale = Math.pow(lengthM, 0.75) * shape;
        double dp = reflection * Math.sqrt(bodyPressurePa * listenerPressurePa) * Math.pow(m2, 0.125) * Math.pow(h, -0.75) * scale;
        double dt = 3.42 / bodySoundMps * mach * Math.pow(m2, -0.375) * Math.pow(h, 0.25) * scale;
        return new NWave(dp, dt);
    }

    /** The Mach cone's half angle (rad), asin(1/M). */
    public static double coneHalfAngle(double mach) { return mach > 1 ? Math.asin(1 / mach) : Math.PI / 2; }

    /**
     * An N-wave as samples (full scale is the given overpressure): a jump to +1 over the shock's rise time, a straight
     * fall to -1 over the duration, a jump back; the rise is the shock thickness of a weak boom (a millisecond or so).
     */
    public static float[] samples(NWave wave, double riseS, double fullScalePa) {
        int n = (int) Math.max(8, Math.round((wave.durationS() + 4 * riseS) * Voice.RATE));
        float[] out = new float[n];
        double amp = fullScalePa > 0 ? wave.overpressurePa() / fullScalePa : 0;
        int rise = (int) Math.max(1, Math.round(riseS * Voice.RATE));
        int body = (int) Math.max(1, Math.round(wave.durationS() * Voice.RATE));
        for (int i = 0; i < n; i++) {
            double v;
            if (i < rise) { v = (double) i / rise; }
            else if (i < rise + body) { v = 1 - 2.0 * (i - rise) / body; }
            else if (i < 2 * rise + body) { v = -1 + (double) (i - rise - body) / rise; }
            else { v = 0; }
            out[i] = (float) (amp * v);
        }
        return out;
    }
}
