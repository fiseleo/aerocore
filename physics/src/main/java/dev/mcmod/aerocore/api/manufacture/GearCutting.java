package dev.mcmod.aerocore.api.manufacture;

/**
 * Gear cutting times (0.35): hobbing, shaping, shaving and grinding by generation, from the machines' kinematics (the
 * usual shop formulas, e.g. Machinery's Handbook's gear-cutting chapters; Kalpakjian and Schmid ch. 24).
 *
 * <ul>
 * <li>Hobbing: a hob of k starts turning n_h a second turns the gear k n_h / z a second (z teeth) and feeds f_a along
 *     the face a gear revolution, so it crosses the face width b plus the approach A and overrun in
 *     z (b + A + overrun) / (k n_h f_a). A hob of diameter D_h cutting a tooth depth h starts A = sqrt(h (D_h - h)) before
 *     the face (the chord of its circle at that depth). Its speed n_h = v / (pi D_h). Generating grinding with a
 *     threaded wheel (worm) moves the same way.</li>
 * <li>Shaping: the cutter plunges to depth and then rolls the gear's pitch circle round once a cut, a rotary feed f_r a
 *     stroke, at N strokes a second: pi d / (f_r N) a cut.</li>
 * <li>Shaving (traverse): the gear turns N_w a second and traverses its face at f_a a revolution, a pass b / (f_a N_w).</li>
 * <li>Profile grinding: the wheel grinds one gap at a time, table strokes of the face plus overtravel at v_t, so
 *     z strokes (b + overtravel) / v_t.</li>
 * </ul>
 */
public final class GearCutting {
    private GearCutting() {}

    /** How far before the face a hob or worm of a diameter (m) starts cutting a tooth depth (m), m. */
    public static double approachM(double toolDiameterM, double depthM) {
        positive(toolDiameterM, depthM, 1);
        if (depthM >= toolDiameterM) { throw new IllegalArgumentException("The depth is less than the tool's diameter"); }
        return Math.sqrt(depthM * (toolDiameterM - depthM));
    }

    /** A hob's speed for a cutting speed (m/s) and its diameter (m), rev/s. */
    public static double hobRevPerS(double cuttingMps, double hobDiameterM) {
        positive(cuttingMps, hobDiameterM, 1);
        return cuttingMps / (Math.PI * hobDiameterM);
    }

    /**
     * Time to hob (or generate-grind) a gear of some teeth across a face width (m) with an approach and overrun (m),
     * a hob of some starts at a speed (rev/s) and an axial feed (m a gear revolution), s.
     */
    public static double hobbingS(int teeth, double faceWidthM, double approachM, double overrunM, int starts, double hobRevPerS, double axialFeedM) {
        positive(hobRevPerS, axialFeedM, 1);
        if (teeth < 1 || starts < 1) { throw new IllegalArgumentException("At least a tooth and a start"); }
        return teeth * (Math.max(faceWidthM, 0) + Math.max(approachM, 0) + Math.max(overrunM, 0)) / (starts * hobRevPerS * axialFeedM);
    }

    /** Time to shape a gear of a pitch diameter (m) at a rotary feed (m a stroke) and strokes a second, in some cuts, s. */
    public static double shapingS(double pitchDiameterM, double rotaryFeedM, double strokesPerS, int cuts) {
        positive(pitchDiameterM, rotaryFeedM, strokesPerS);
        return Math.max(cuts, 1) * Math.PI * pitchDiameterM / (rotaryFeedM * strokesPerS);
    }

    /** Time to traverse-shave a face width (m) at an axial feed (m a gear revolution) and the gear's speed (rev/s), in some passes, s. */
    public static double shavingS(double faceWidthM, double axialFeedM, double gearRevPerS, int passes) {
        positive(axialFeedM, gearRevPerS, 1);
        return Math.max(passes, 1) * Math.max(faceWidthM, 0) / (axialFeedM * gearRevPerS);
    }

    /** Time to profile-grind a gear of some teeth across a face (m) with an overtravel (m) at a table speed (m/s), some strokes a gap, s. */
    public static double profileGrindingS(int teeth, double faceWidthM, double overtravelM, double tableMps, int strokesPerGap) {
        positive(tableMps, 1, 1);
        if (teeth < 1) { throw new IllegalArgumentException("At least a tooth"); }
        return teeth * Math.max(strokesPerGap, 1) * (Math.max(faceWidthM, 0) + 2 * Math.max(overtravelM, 0)) / tableMps;
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Speeds, feeds and sizes must be positive"); }
    }
}
