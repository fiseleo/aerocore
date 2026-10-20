package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.gasdynamics.ConicalShock;
import dev.mcmod.aerocore.api.gasdynamics.NormalShock;
import dev.mcmod.aerocore.api.gasdynamics.ObliqueShock;

import java.util.Objects;

/**
 * An external-compression supersonic inlet (0.28) at its critical point (the terminal normal shock at the cowl lip;
 * Seddon and Goldsmith, Intake Aerodynamics, ch. 8-11; Mattingly, Elements of Propulsion, ch. 10): its total pressure
 * recovery and the share of the capture area's stream it takes in.
 * <ul>
 * <li>PITOT: a normal shock at the lip.</li>
 * <li>RAMPS: two-dimensional ramps, each turning the flow by its angle through an oblique shock (perfect gas, gamma 1.4),
 * then the terminal normal shock. optimalRamps gives Oswatitsch's equal-strength shocks for a design Mach number.</li>
 * <li>CONE: an axisymmetric spike of a half-angle: the conical shock (Taylor-Maccoll), the conical field compressing
 * isentropically to the surface Mach number, then the normal shock there.</li>
 * </ul>
 * Shock-on-lip at the design Mach number: slower, the first shock stands off the lip and the stream between it and the
 * lip spills (the mass-flow ratio of a wedge, from where the streamline into the lip crosses the shock; the spike's is
 * taken as its square); faster, all the capture stream enters. A detached shock (ramp too steep for the Mach number) is
 * a normal shock. The subsonic diffuser keeps a given share of total pressure. Below Mach 1 there are no shocks and the
 * inlet takes what the engine asks.
 */
public final class SupersonicInlet {
    private SupersonicInlet() {}

    public enum Kind { PITOT, RAMPS, CONE }

    private static final double G = 1.4;

    /** Kind, design Mach number, ramp deflections (radians; one cone half-angle for CONE; none for PITOT), the subsonic diffuser's recovery. */
    public record Design(Kind kind, double designMach, double[] angles, double subsonicRecovery) {
        public Design {
            Objects.requireNonNull(kind, "kind");
            angles = angles == null ? new double[0] : angles.clone();
            if (!(designMach >= 1) || !(subsonicRecovery > 0 && subsonicRecovery <= 1)) { throw new IllegalArgumentException("Design Mach >= 1 and a recovery in (0, 1]"); }
            if (kind == Kind.CONE && angles.length != 1) { throw new IllegalArgumentException("A spike has one half-angle"); }
            if (kind == Kind.RAMPS && angles.length < 1) { throw new IllegalArgumentException("A ramp inlet has at least one ramp"); }
            for (double a : angles) { if (!(a > 0 && a < Math.toRadians(40))) { throw new IllegalArgumentException("Ramp and cone angles in (0, 40) degrees"); } }
        }
    }

    /** The inlet at its critical point: total pressure recovery, captured over capture-area stream, the Mach number ahead of the terminal shock. */
    public record Critical(double recovery, double captureRatio, double terminalMach) {}

    /**
     * The inlet's critical points tabulated every 0.05 in Mach number from 1 up (the cone's Taylor-Maccoll solution
     * costs milliseconds; a part looks its inlet up every step), linear between; past the table its last row.
     */
    public record Table(Design design, double[] mach, double[] recovery, double[] capture, double[] terminal) {
        public static Table of(Design d, double maxMach) {
            int n = (int) Math.ceil((Math.max(1.5, maxMach) - 1) / 0.05) + 1;
            double[] m = new double[n], r = new double[n], c = new double[n], t = new double[n];
            for (int i = 0; i < n; i++) {
                m[i] = 1 + 0.05 * i;
                Critical x = critical(d, i == 0 ? 1.0001 : m[i]);
                r[i] = x.recovery(); c[i] = x.captureRatio(); t[i] = x.terminalMach();
            }
            return new Table(d, m, r, c, t);
        }

        /** The critical point at a flight Mach number. */
        public Critical at(double flightMach) {
            if (flightMach <= 1) { return new Critical(design.subsonicRecovery(), Double.NaN, flightMach); }
            double x = Math.min((flightMach - 1) / 0.05, mach.length - 1.000001);
            int i = (int) x;
            double w = x - i;
            return new Critical(recovery[i] + w * (recovery[i + 1] - recovery[i]), capture[i] + w * (capture[i + 1] - capture[i]),
                    terminal[i] + w * (terminal[i + 1] - terminal[i]));
        }
    }

    /** The inlet at a flight Mach number. */
    public static Critical critical(Design d, double mach) {
        if (!(mach >= 0) || !Double.isFinite(mach)) { throw new IllegalArgumentException("Mach must be finite and nonnegative"); }
        if (mach <= 1) { return new Critical(d.subsonicRecovery(), Double.NaN, mach); }
        double recovery = 1, m = mach;
        switch (d.kind()) {
            case PITOT -> { }
            case RAMPS -> {
                for (double a : d.angles()) {
                    if (m <= 1) { break; }
                    double[] max = ObliqueShock.maxDeflection(m, G);
                    if (a > max[0]) {   // detached: a bow shock, normal ahead of the ramp
                        NormalShock n = NormalShock.of(m, G);
                        recovery *= n.totalPressureRatio();
                        m = n.machBehind();
                        break;
                    }
                    ObliqueShock s = ObliqueShock.of(m, a, G, false);
                    recovery *= s.totalPressureRatio();
                    m = s.machBehind();
                }
            }
            case CONE -> {
                try {
                    ConicalShock c = ConicalShock.of(mach, d.angles()[0], G);
                    recovery *= c.totalPressureRatio();
                    m = c.surfaceMach();
                } catch (IllegalArgumentException detached) {
                    NormalShock n = NormalShock.of(mach, G);
                    recovery *= n.totalPressureRatio();
                    m = n.machBehind();
                }
            }
        }
        double terminal = m;
        if (m > 1) { recovery *= NormalShock.of(m, G).totalPressureRatio(); }
        return new Critical(recovery * d.subsonicRecovery(), captureRatio(d, mach), terminal);
    }

    /** Captured over capture-area stream at a supersonic Mach number (1 at and above the design Mach number). */
    public static double captureRatio(Design d, double mach) {
        if (mach >= d.designMach() || d.kind() == Kind.PITOT) { return 1; }
        double theta = d.angles()[0];
        double beta, betaDesign;
        if (d.kind() == Kind.CONE) {
            try { beta = ConicalShock.of(mach, theta, G).waveAngle(); } catch (IllegalArgumentException e) { return wedge(Math.PI / 2, ConicalShock.of(d.designMach(), theta, G).waveAngle(), theta) ; }
            betaDesign = ConicalShock.of(d.designMach(), theta, G).waveAngle();
            double r = wedge(beta, betaDesign, theta);
            return r * r;
        }
        try { beta = ObliqueShock.of(mach, theta, G, false).waveAngle(); } catch (IllegalArgumentException e) { beta = Math.PI / 2; }
        betaDesign = ObliqueShock.of(d.designMach(), theta, G, false).waveAngle();
        return wedge(beta, betaDesign, theta);
    }

    /** Mass-flow ratio of a wedge of deflection theta whose shock at beta misses a lip placed on the design shock at betaD. */
    private static double wedge(double beta, double betaDesign, double theta) {
        double tb = Math.tan(beta), tt = Math.tan(theta), td = Math.tan(betaDesign);
        if (!Double.isFinite(tb)) { return 1 - tt / td; }
        return Math.clamp(tb * (1 - tt / td) / (tb - tt), 0, 1);
    }

    /**
     * Oswatitsch's optimum for n ramps at a design Mach number: oblique shocks of equal strength (the same normal Mach
     * number) whose last leaves a Mach number equal to that normal Mach number for the terminal shock; the ramp
     * deflections, radians.
     */
    public static double[] optimalRamps(double designMach, int ramps) {
        if (!(designMach > 1) || ramps < 1) { throw new IllegalArgumentException("Design Mach > 1 and at least one ramp"); }
        double lo = 1 + 1e-9, hi = designMach;
        double[] angles = new double[ramps];
        for (int it = 0; it < 200 && hi - lo > 1e-12; it++) {
            double mn = 0.5 * (lo + hi), m = designMach;
            boolean ok = true;
            for (int i = 0; i < ramps; i++) {
                if (m <= mn) { ok = false; break; }
                double beta = Math.asin(mn / m);
                angles[i] = ObliqueShock.deflection(m, beta, G);
                m = ObliqueShock.ofWaveAngle(m, beta, G).machBehind();
            }
            // too strong shocks leave the flow slower than mn: weaken them
            if (!ok || m < mn) { hi = mn; } else { lo = mn; }
        }
        double mn = 0.5 * (lo + hi), m = designMach;
        for (int i = 0; i < ramps; i++) {
            double beta = Math.asin(mn / m);
            angles[i] = ObliqueShock.deflection(m, beta, G);
            m = ObliqueShock.ofWaveAngle(m, beta, G).machBehind();
        }
        return angles;
    }
}
