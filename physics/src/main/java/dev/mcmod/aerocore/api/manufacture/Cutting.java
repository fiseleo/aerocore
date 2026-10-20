package dev.mcmod.aerocore.api.manufacture;

/**
 * Machining times (0.25), after Groover (Fundamentals of Modern Manufacturing, ch. 21-22) and Kalpakjian and Schmid
 * (Manufacturing Engineering and Technology, ch. 21-23).
 *
 * <ul>
 * <li>Turning a length L of a diameter D at cutting speed v and feed f per revolution: N = v / (pi D), the tool moves
 *     f N along, T = L / (f N) = pi D L / (f v) (Groover eq. 22.3-22.5).</li>
 * <li>Drilling a hole of diameter D and depth d with point angle theta: the approach A = (D / 2) tan(90 - theta / 2)
 *     is added to the depth, T = (d + A) / (f N) (Groover eq. 22.7-22.9).</li>
 * <li>Roughing by power: the material removal rate is the spindle's power (times the drive's efficiency) over the
 *     material's specific cutting energy u (J/m^3), at most what the tool and the machine's rigidity allow (a rate
 *     limit), so T = V / min(eta P / u, MRR_max) (Groover eq. 21.16).</li>
 * <li>Finishing: the ideal peak-to-valley roughness of a tool of nose radius r at feed f is f^2 / (8 r), its average
 *     Ra = f^2 / (32 r) (Groover eq. 24.1); the feed that gives a roughness is f = sqrt(32 r Ra), and a finished area
 *     A takes A / (v f) a pass.</li>
 * </ul>
 */
public final class Cutting {
    private Cutting() {}

    /** Time to turn a length (m) of a diameter (m) at a cutting speed (m/s) and feed (m/rev), s. */
    public static double turnS(double lengthM, double diameterM, double speedMps, double feedM) {
        positive(speedMps, feedM, diameterM);
        return Math.PI * diameterM * Math.max(lengthM, 0) / (feedM * speedMps);
    }

    /** Time to drill holes of a diameter and depth (m) at a speed (m/s) and feed (m/rev), with a point angle (rad), s. */
    public static double drillS(int holes, double diameterM, double depthM, double speedMps, double feedM, double pointAngleRad) {
        positive(speedMps, feedM, diameterM);
        double approach = diameterM / 2 * Math.tan(Math.PI / 2 - pointAngleRad / 2);
        double revsPerS = speedMps / (Math.PI * diameterM);
        return Math.max(holes, 0) * (Math.max(depthM, 0) + approach) / (feedM * revsPerS);
    }

    /** Material removal rate of a power (W) at an efficiency over a specific cutting energy (J/m^3), at most a limit (m^3/s). */
    public static double removalRate(double powerW, double efficiency, double specificEnergyJPerM3, double limitM3PerS) {
        positive(powerW, efficiency, specificEnergyJPerM3);
        return Math.min(efficiency * powerW / specificEnergyJPerM3, limitM3PerS > 0 ? limitM3PerS : Double.POSITIVE_INFINITY);
    }

    /** Time to rough out a volume (m^3) at that removal rate, s. */
    public static double roughS(double volumeM3, double powerW, double efficiency, double specificEnergyJPerM3, double limitM3PerS) {
        return Math.max(volumeM3, 0) / removalRate(powerW, efficiency, specificEnergyJPerM3, limitM3PerS);
    }

    /** The average roughness Ra (m) a tool of nose radius r (m) leaves at a feed (m/rev). */
    public static double roughness(double feedM, double noseRadiusM) { return feedM * feedM / (32 * noseRadiusM); }

    /** The feed (m/rev) that leaves a roughness Ra (m) with a nose radius (m). */
    public static double feedFor(double roughnessM, double noseRadiusM) { return Math.sqrt(32 * noseRadiusM * Math.max(roughnessM, 0)); }

    /** Time to finish an area (m^2) at a speed (m/s) and feed (m) in some passes, s. */
    public static double finishS(double areaM2, double speedMps, double feedM, int passes) {
        positive(speedMps, feedM, 1);
        return Math.max(areaM2, 0) / (speedMps * feedM) * Math.max(passes, 1);
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Speeds, feeds, sizes and powers must be positive"); }
    }
}
