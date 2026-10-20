package dev.mcmod.aerocore.api.gasdynamics;

/**
 * A plane oblique shock in a calorically perfect gas (NACA Report 1135 eqs. 138-150; Anderson, Modern Compressible
 * Flow, ch. 4): wave angle beta from the deflection theta by the theta-beta-M relation
 * tan(theta) = 2 cot(beta) (M^2 sin^2 beta - 1) / (M^2 (gamma + cos 2 beta) + 2), weak or strong branch, and the ratios
 * of a normal shock at the normal Mach number M sin(beta). Angles in radians; ratios are behind over ahead.
 */
public record ObliqueShock(double machAhead, double deflection, double waveAngle, double machBehind, double pressureRatio,
                           double temperatureRatio, double densityRatio, double totalPressureRatio) {

    /** Deflection of the flow by a shock of wave angle beta (0 at the Mach angle and at a normal shock). */
    public static double deflection(double mach, double waveAngle, double gamma) {
        double s = Math.sin(waveAngle), m2 = mach * mach;
        double num = 2 / Math.tan(waveAngle) * (m2 * s * s - 1), den = m2 * (gamma + Math.cos(2 * waveAngle)) + 2;
        return Math.atan(Math.max(0, num) / den);
    }

    /** {largest deflection, its wave angle}: past it the shock detaches. */
    public static double[] maxDeflection(double mach, double gamma) {
        if (!(mach > 1)) { throw new IllegalArgumentException("An oblique shock needs M > 1"); }
        double lo = Math.asin(1 / mach), hi = Math.PI / 2;
        // golden-section search: theta(beta) has one maximum between the Mach angle and 90 degrees
        double r = 0.5 * (Math.sqrt(5) - 1), a = hi - r * (hi - lo), b = lo + r * (hi - lo);
        double fa = deflection(mach, a, gamma), fb = deflection(mach, b, gamma);
        for (int i = 0; i < 200 && hi - lo > 1e-14; i++) {
            if (fa < fb) { lo = a; a = b; fa = fb; b = lo + r * (hi - lo); fb = deflection(mach, b, gamma); }
            else { hi = b; b = a; fb = fa; a = hi - r * (hi - lo); fa = deflection(mach, a, gamma); }
        }
        double beta = 0.5 * (lo + hi);
        return new double[]{deflection(mach, beta, gamma), beta};
    }

    /** The shock that turns flow at Mach M through theta; weak (the one nature usually picks) or strong; throws if theta detaches it. */
    public static ObliqueShock of(double mach, double deflection, double gamma, boolean strong) {
        if (!(mach > 1) || !Double.isFinite(mach) || !(gamma > 1)) { throw new IllegalArgumentException("An oblique shock needs M > 1 and gamma > 1"); }
        if (!(deflection >= 0)) { throw new IllegalArgumentException("Deflection must be nonnegative"); }
        double[] max = maxDeflection(mach, gamma);
        if (deflection > max[0]) { throw new IllegalArgumentException("Deflection " + Math.toDegrees(deflection) + " deg detaches the shock at Mach " + mach); }
        double lo = strong ? max[1] : Math.asin(1 / mach), hi = strong ? Math.PI / 2 : max[1];
        for (int i = 0; i < 200 && hi - lo > 1e-15; i++) {
            double beta = 0.5 * (lo + hi);
            // the weak branch's deflection rises with beta, the strong branch's falls
            if ((deflection(mach, beta, gamma) < deflection) != strong) { lo = beta; } else { hi = beta; }
        }
        return ofWaveAngle(mach, 0.5 * (lo + hi), gamma);
    }

    /** The shock of a given wave angle (between the Mach angle and 90 degrees). */
    public static ObliqueShock ofWaveAngle(double mach, double waveAngle, double gamma) {
        double mn = mach * Math.sin(waveAngle);
        NormalShock n = NormalShock.of(Math.max(1, mn), gamma);
        double theta = deflection(mach, waveAngle, gamma);
        double behind = n.machBehind() / Math.sin(waveAngle - theta);
        return new ObliqueShock(mach, theta, waveAngle, behind, n.pressureRatio(), n.temperatureRatio(), n.densityRatio(), n.totalPressureRatio());
    }

    /**
     * An oblique shock in a thermally perfect gas turning the stream ahead through theta (weak branch): the normal
     * component goes through NormalShock.behind, the tangential one is kept; the wave angle is found by bisection
     * between the Mach angle and the angle of largest deflection. Returns {wave angle, state behind} as an Oblique.
     */
    public static Thermal behind(Thermo gas, GasState ahead, double deflection) {
        double mach = ahead.mach(gas);
        if (!(mach > 1)) { throw new IllegalArgumentException("An oblique shock needs supersonic flow"); }
        if (deflection <= 0) { return new Thermal(Math.asin(1 / mach), ahead); }
        double lo = Math.asin(1 / mach) + 1e-12, hi = Math.PI / 2;
        // largest deflection of this gas (golden section), then the weak root below it
        double r = 0.5 * (Math.sqrt(5) - 1), a = hi - r * (hi - lo), b = lo + r * (hi - lo);
        double fa = thermalDeflection(gas, ahead, a), fb = thermalDeflection(gas, ahead, b);
        double l = lo, h = hi;
        for (int i = 0; i < 120 && h - l > 1e-12; i++) {
            if (fa < fb) { l = a; a = b; fa = fb; b = l + r * (h - l); fb = thermalDeflection(gas, ahead, b); }
            else { h = b; b = a; fb = fa; a = h - r * (h - l); fa = thermalDeflection(gas, ahead, a); }
        }
        double betaMax = 0.5 * (l + h);
        if (deflection > thermalDeflection(gas, ahead, betaMax)) { throw new IllegalArgumentException("The deflection detaches the shock"); }
        hi = betaMax;
        for (int i = 0; i < 120 && hi - lo > 1e-13; i++) {
            double beta = 0.5 * (lo + hi);
            if (thermalDeflection(gas, ahead, beta) < deflection) { lo = beta; } else { hi = beta; }
        }
        double beta = 0.5 * (lo + hi);
        return new Thermal(beta, thermalBehind(gas, ahead, beta));
    }

    /** A thermally perfect oblique shock: its wave angle and the state behind it. */
    public record Thermal(double waveAngle, GasState behind) {}

    private static GasState thermalBehind(Thermo gas, GasState ahead, double beta) {
        double v = ahead.speedMps(), un = v * Math.sin(beta), ut = v * Math.cos(beta);
        double a = gas.speedOfSound(ahead.temperatureK());
        if (un <= a) { return ahead; }
        GasState n = NormalShock.behind(gas, new GasState(ahead.temperatureK(), ahead.pressurePa(), un));
        return new GasState(n.temperatureK(), n.pressurePa(), Math.hypot(n.speedMps(), ut));
    }

    private static double thermalDeflection(Thermo gas, GasState ahead, double beta) {
        double v = ahead.speedMps(), un = v * Math.sin(beta), ut = v * Math.cos(beta);
        if (un <= gas.speedOfSound(ahead.temperatureK())) { return 0; }
        GasState n = NormalShock.behind(gas, new GasState(ahead.temperatureK(), ahead.pressurePa(), un));
        return beta - Math.atan2(n.speedMps(), ut);
    }
}
