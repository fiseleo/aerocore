package dev.mcmod.aerocore.api.manufacture;

/**
 * Making threads (0.35): tapping, single-point cutting, thread milling and rolling, from the tools' kinematics
 * (Groover sec. 22.2 and 21; Kalpakjian and Schmid ch. 23 and 13).
 *
 * <ul>
 * <li>Tapping: the tap advances a pitch p a revolution, so a depth L takes L / (p n) in and L / (p n r) out at r times
 *     the speed in reverse.</li>
 * <li>Single-point threading on a lathe: each pass runs the length at a pitch a revolution, L / (p n), and the passes
 *     step the depth in.</li>
 * <li>Thread milling: the cutter follows a helix of one pitch a turn round the thread, its centre on a circle of
 *     D - d (internal) or D + d (external); a thread of length L is L / p turns. The feed speed is given at the cutting
 *     edge, so the centre moves at v_f (D -+ d) / D.</li>
 * <li>Rolling: cylindrical dies form a thread in a number of the work's revolutions (in-feed), flat dies in one stroke
 *     of the moving die: no metal is cut away.</li>
 * </ul>
 */
public final class Threading {
    private Threading() {}

    /** Time to tap a depth (m) of a pitch (m) at a speed (rev/s), reversing out at a ratio of that speed, s. */
    public static double tappingS(double depthM, double pitchM, double revPerS, double reverseRatio) {
        positive(pitchM, revPerS, reverseRatio);
        double in = Math.max(depthM, 0) / (pitchM * revPerS);
        return in + in / reverseRatio;
    }

    /** Time to single-point thread a length (m) of a pitch (m) at a speed (rev/s) in some passes (the cutting strokes), s. */
    public static double singlePointS(double lengthM, double pitchM, double revPerS, int passes) {
        positive(pitchM, revPerS, 1);
        return Math.max(passes, 1) * Math.max(lengthM, 0) / (pitchM * revPerS);
    }

    /**
     * Time to thread-mill a thread of a diameter, length and pitch (m) with a cutter of a diameter (m) at a feed speed at
     * the edge (m/s), inside a bore or outside a shaft, s.
     */
    public static double threadMillingS(double threadDiameterM, double cutterDiameterM, double lengthM, double pitchM, double edgeFeedMps, boolean internal) {
        positive(threadDiameterM, cutterDiameterM, pitchM);
        positive(edgeFeedMps, 1, 1);
        double centre = internal ? threadDiameterM - cutterDiameterM : threadDiameterM + cutterDiameterM;
        if (!(centre > 0)) { throw new IllegalArgumentException("The cutter must be smaller than the bore"); }
        double turns = Math.max(lengthM, 0) / pitchM;
        double path = turns * Math.hypot(Math.PI * centre, pitchM);
        return path / (edgeFeedMps * centre / threadDiameterM);
    }

    /** Time to roll a thread in cylindrical dies: the work's revolutions to form it at its speed (rev/s), s. */
    public static double inFeedRollingS(double workRevolutions, double workRevPerS) {
        positive(workRevPerS, 1, 1);
        return Math.max(workRevolutions, 0) / workRevPerS;
    }

    /** Time to roll a thread between flat dies: the moving die's travel (m) at its speed (m/s), forward and back, s. */
    public static double flatDieS(double travelM, double dieMps) {
        positive(dieMps, 1, 1);
        return 2 * Math.max(travelM, 0) / dieMps;
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Pitches, speeds, sizes and ratios must be positive"); }
    }
}
