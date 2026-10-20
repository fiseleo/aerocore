package dev.mcmod.aerocore.api.acoustics;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;

/**
 * Sound from a source to a listener in the open air (0.37):
 * <ul>
 * <li>Spreading: a point source of sound power level Lw gives Lp = Lw + DI - 20 log10(r) - 11 dB at r (free field,
 *     ISO 9613-2 eq. 7 with the directivity index DI).</li>
 * <li>Air absorption, ISO 9613-1:1993 eq. 3-5 (pure tones, dB/m) from temperature, relative humidity and pressure.</li>
 * <li>Speed of sound in dry air, c = 331.3 sqrt(T / 273.15) m/s.</li>
 * <li>Doppler: f'/f = (c - v_L . n) / (c - v_S . n), n the unit vector from the source where it sounded to the
 *     listener where it is heard (still air).</li>
 * <li>Retarded time: the sound heard now left the source when |x_L(t) - x_S(t_e)| = c (t - t_e). A subsonic source
 *     has one such time; a supersonic one has two inside its Mach cone (it is heard coming and going) and none ahead
 *     of it. {@link #emissions} finds all of them on a source's recorded track.</li>
 * </ul>
 * Ground reflection, refraction by wind and temperature gradients, and barriers are not modelled (a block in the way
 * is the game side's occlusion).
 */
public final class Propagation {
    private Propagation() {}

    /** ISO 9613-1 reference conditions. */
    static final double T0 = 293.15, T01 = 273.16, PR = 101_325;

    public static double speedOfSound(double temperatureK) { return 331.3 * Math.sqrt(Math.max(1, temperatureK) / 273.15); }

    /** Free-field level at r (m) of a source of sound power level lw (dB) and directivity index di (dB). */
    public static double levelAt(double lw, double di, double r) {
        return lw + di - 20 * Math.log10(Math.max(r, 0.1)) - 10.99;
    }

    /**
     * Pure-tone attenuation coefficient of air, dB per metre (ISO 9613-1): f Hz, temperature K, relative humidity in
     * percent, pressure Pa. Valid 50 Hz-10 kHz, -20 to 50 C, 10-100 % (the standard quotes +/-10 % there).
     */
    public static double absorptionDbPerM(double f, double temperatureK, double humidityPercent, double pressurePa) {
        double t = Math.max(200, temperatureK), pa = Math.max(1000, pressurePa) / PR;
        double c = -6.8346 * Math.pow(T01 / t, 1.261) + 4.6151;
        double h = Math.max(0, humidityPercent) * Math.pow(10, c) / pa;   // molar concentration of water vapour, percent
        double frO = pa * (24 + 4.04e4 * h * (0.02 + h) / (0.391 + h));
        double frN = pa * Math.pow(t / T0, -0.5) * (9 + 280 * h * Math.exp(-4.170 * (Math.pow(t / T0, -1.0 / 3) - 1)));
        double f2 = f * f;
        return 8.686 * f2 * (1.84e-11 / pa * Math.sqrt(t / T0) + Math.pow(t / T0, -2.5)
                * (0.01275 * Math.exp(-2239.1 / t) / (frO + f2 / frO) + 0.1068 * Math.exp(-3352.0 / t) / (frN + f2 / frN)));
    }

    /**
     * The frequency above which a distance of air takes more than 3 dB off (Hz, bisection on the ISO 9613-1
     * coefficient; 20 kHz when even that loses less): the cutoff of one low-pass standing in for air absorption.
     */
    public static double airCutoffHz(double distanceM, double temperatureK, double humidityPercent, double pressurePa) {
        if (absorptionDbPerM(20_000, temperatureK, humidityPercent, pressurePa) * distanceM <= 3) { return 20_000; }
        double lo = 20, hi = 20_000;
        for (int i = 0; i < 40; i++) {
            double mid = Math.sqrt(lo * hi);
            if (absorptionDbPerM(mid, temperatureK, humidityPercent, pressurePa) * distanceM > 3) { hi = mid; } else { lo = mid; }
        }
        return lo;
    }

    /** The Doppler factor (heard over emitted frequency); infinite or negative when the source outruns the sound toward the listener. */
    public static double doppler(Vec3d sourceVelocity, Vec3d listenerVelocity, Vec3d fromSourceToListener, double c) {
        double len = fromSourceToListener.length();
        if (len < 1e-9) { return 1; }
        Vec3d n = fromSourceToListener.scale(1 / len);
        double den = c - sourceVelocity.dot(n);
        return den <= 1e-9 ? Double.POSITIVE_INFINITY : (c - listenerVelocity.dot(n)) / den;
    }

    /** A sound that reaches the listener now: when it left (s), from where, how far it went, and the source's velocity then. */
    public record Emission(double time, Vec3d position, Vec3d velocity, double distanceM) {
        public double delayS(double now) { return now - time; }
    }

    /**
     * When sounds heard now left a source whose positions at increasing times are recorded (straight lines between
     * the records), heard at the listener's position now: the newest first. Empty when none reached it within the
     * record (the record is too short, or the listener is ahead of a supersonic source).
     */
    public static List<Emission> emissions(double[] times, Vec3d[] positions, int count, Vec3d listener, double now, double c) {
        List<Emission> out = new ArrayList<>();
        if (count < 1) { return out; }
        // g(te) = |xL - xS(te)| - c (now - te): positive for recent times (not yet heard), negative for old ones (passed)
        double gNext = g(positions[count - 1], listener, now, times[count - 1], c);
        if (Math.abs(gNext) < 1e-9) { out.add(emission(times, positions, count - 1, 0, listener)); }
        for (int k = count - 2; k >= 0; k--) {
            double gk = g(positions[k], listener, now, times[k], c);
            if ((gk < 0) != (gNext < 0)) {
                double s = segmentRoot(positions[k], positions[k + 1], times[k], times[k + 1], listener, now, c);
                if (!Double.isNaN(s)) {
                    Emission e = emission(times, positions, k, s, listener);
                    if (out.isEmpty() || Math.abs(out.getLast().time() - e.time()) > 1e-9) { out.add(e); }
                }
            }
            gNext = gk;
        }
        return out;
    }

    private static double g(Vec3d xs, Vec3d xl, double now, double te, double c) { return xl.subtract(xs).length() - c * (now - te); }

    /** The fraction s in [0, 1] of the segment k..k+1 where the emission condition holds (the root of a quadratic). */
    private static double segmentRoot(Vec3d a, Vec3d b, double ta, double tb, Vec3d xl, double now, double c) {
        // |xl - a - s d| = c (now - ta - s dt): square both sides
        Vec3d d = b.subtract(a), e = xl.subtract(a);
        double dt = tb - ta, r0 = c * (now - ta), r1 = -c * dt;
        double qa = d.dot(d) - r1 * r1, qb = -2 * e.dot(d) - 2 * r0 * r1, qc = e.dot(e) - r0 * r0;
        double best = Double.NaN;
        if (Math.abs(qa) < 1e-12) {
            if (Math.abs(qb) > 1e-15) { best = check(-qc / qb, r0, r1); }
        } else {
            double disc = qb * qb - 4 * qa * qc;
            if (disc >= 0) {
                double sq = Math.sqrt(disc), s1 = (-qb - sq) / (2 * qa), s2 = (-qb + sq) / (2 * qa);
                double c1 = check(s1, r0, r1), c2 = check(s2, r0, r1);
                best = Double.isNaN(c1) ? c2 : Double.isNaN(c2) ? c1 : Math.max(c1, c2);
            }
        }
        return best;
    }

    private static double check(double s, double r0, double r1) {
        return s >= -1e-9 && s <= 1 + 1e-9 && r0 + r1 * s >= -1e-9 ? Math.max(0, Math.min(1, s)) : Double.NaN;
    }

    private static Emission emission(double[] times, Vec3d[] positions, int k, double s, Vec3d listener) {
        Vec3d a = positions[k];
        Vec3d b = k + 1 < positions.length && positions[k + 1] != null ? positions[k + 1] : a;
        double ta = times[k], tb = k + 1 < times.length ? times[k + 1] : ta;
        Vec3d p = a.add(b.subtract(a).scale(s));
        Vec3d v = tb > ta ? b.subtract(a).scale(1 / (tb - ta)) : Vec3d.ZERO;
        return new Emission(ta + s * (tb - ta), p, v, listener.subtract(p).length());
    }
}
