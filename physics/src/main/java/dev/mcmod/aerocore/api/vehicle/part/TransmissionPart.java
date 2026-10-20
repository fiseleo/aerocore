package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.drive.TorqueConverter;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * A gearbox (0.24): the mechanics of a manual box, a torque-converter automatic, a dual-clutch box and a CVT. When to
 * shift, how hard to slip a launch clutch, which ratio a CVT should aim for are the vehicle mod's rules, given as
 * inputs; this part does what the hardware does with them. Rotors (a shaft joins "input" to the engine and "output"
 * to an axle or transfer case), with their rows (PartStep):
 * <ul>
 * <li>Manual: "input" (flywheel side of the clutch), "primary" (clutch disc and primary shaft), "output". The clutch
 *     between input and primary holds up to its capacity times (1 - "clutch"), the pedal 0 up to 1 pressed. "gear"
 *     (-1 reverse, 0 neutral, 1..n) moves the lever: the engaged gear comes out at once, and the new one engages only
 *     when its synchroniser (a friction cone of synchroniserNm between primary shaft and gear; reverse has a quarter of
 *     it, being unsynchronised in most boxes) has matched the speeds within 3 rad/s. With the clutch engaged the cone
 *     has to swing the engine too and may never match: the box grinds (shown state 2 after 0.6 s) and stays in
 *     neutral.</li>
 * <li>Automatic: "input" (converter pump), "turbine", "output"; the converter (TorqueConverter) between pump and turbine,
 *     its lock-up clutch on "lockup" (0..1), and each gear a shift element (a clutch of clutchNm on the turbine) to the
 *     output at its ratio. "gear" (-2 park, -1 reverse, 0 neutral, 1..n) starts a clutch-to-clutch shift: over shiftTimeS
 *     the off-going element's capacity falls as the on-coming one's rises, the inertia phase and torque handover coming
 *     out of the solve. Park holds the output with a pawl once below 3 rad/s (it ratchets faster).</li>
 * <li>Dual clutch: "input", "odd" (gears 1, 3, 5, 7), "even" (2, 4, 6 and reverse), "output". Each shaft's gear engages
 *     by its synchroniser while that shaft's clutch is open; "gear" picks the gear to drive in and "preselect" the one
 *     to ready on the other shaft. When the wanted gear is ready on the other shaft the clutches hand over in shiftTimeS
 *     (one ramps down while the other ramps up, no break in drive). "clutch" (0..1) scales the driving clutch for launch
 *     and creep.</li>
 * <li>CVT: "input" (converter pump), "turbine" (primary pulley), "output"; converter and lock-up as an automatic, then
 *     the variator: a ratio row whose torque the belt's capacity bounds (beyond it the belt slips). The ratio moves
 *     toward "ratio" (between the box's lowest and highest) at most ratioRatePerS. "gear": -2 park, -1 reverse (times the
 *     reverse set's ratio), 0 neutral, 1 drive.</li>
 * </ul>
 * Gears pass power with the box's efficiency. Shown values: the engaged gear (CVT: its ratio), the state (0 steady,
 * 1 synchronising or shifting, 2 grinding), the ratio in use.
 */
public final class TransmissionPart implements Part {
    /** Kinds of gearbox. */
    public enum Kind { MANUAL, AUTOMATIC, DUAL_CLUTCH, CVT }

    /**
     * A gearbox: kind, forward ratios (input over output speed, 1st first; a CVT's lowest and highest), reverse ratio
     * (negative), efficiency, clutch or shift element capacity (N m), synchroniser torque (N m), converter (automatic,
     * CVT), lock-up capacity (N m), shift time (s), CVT ratio rate (per s) and belt capacity (N m), inertias of the input
     * side, the inner shafts and the output (kg m^2), and whether its shafts lie across the vehicle (along -z) rather than
     * along it (+x).
     */
    public record Spec(Kind kind, double[] ratios, double reverseRatio, double efficiency, double clutchNm, double synchroniserNm, TorqueConverter converter,
                       double lockupNm, double shiftTimeS, double ratioRatePerS, double beltNm, double inputInertiaKgM2, double shaftInertiaKgM2,
                       double outputInertiaKgM2, boolean transverse) {
        public Spec {
            Objects.requireNonNull(kind, "kind");
            ratios = ratios.clone();
            if (ratios.length == 0 || (kind == Kind.CVT && ratios.length != 2)) { throw new IllegalArgumentException("Ratios: the gears', or a CVT's lowest and highest"); }
            for (double r : ratios) { if (!(r > 0) || !Double.isFinite(r)) { throw new IllegalArgumentException("Forward ratios must be positive"); } }
            if (!(reverseRatio < 0) || !(efficiency > 0 && efficiency <= 1)) { throw new IllegalArgumentException("Reverse ratio negative, efficiency in (0, 1]"); }
            if ((kind == Kind.AUTOMATIC || kind == Kind.CVT) && converter == null) { throw new IllegalArgumentException("An automatic or CVT needs its converter"); }
            if (!(inputInertiaKgM2 > 0) || !(shaftInertiaKgM2 > 0) || !(outputInertiaKgM2 > 0) || !(shiftTimeS > 0)) { throw new IllegalArgumentException("Inertias and shift time must be positive"); }
        }
        @Override public double[] ratios() { return ratios.clone(); }
    }

    /** Speed match at which a synchroniser lets the gear in, rad/s. */
    public static final double SYNCED = 3;
    /** Time a synchroniser may take before the box is taken to grind, s. */
    public static final double GRIND = 0.6;
    /** The park pawl catches below this output speed, rad/s. */
    public static final double PAWL = 3;
    /** Churning drag on the inner shafts, N m s/rad (rule; lets a declutched primary shaft spin down). */
    static final double CHURN = 0.002;

    private final PartDesign design;
    private final Spec spec;
    private final MassProperties mass;
    private final List<DesignIssue> issues;

    public TransmissionPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.spec = Objects.requireNonNull(spec, "spec");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public Spec spec() { return spec; }
    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }

    @Override public List<Rotor> rotors() {
        Vec3d axis = spec.transverse() ? Vec3d.Z.scale(-1) : Vec3d.X;
        return switch (spec.kind()) {
            case MANUAL -> List.of(new Rotor("input", axis, spec.inputInertiaKgM2(), false), new Rotor("primary", axis, spec.shaftInertiaKgM2(), false),
                    new Rotor("output", axis, spec.outputInertiaKgM2(), false));
            case AUTOMATIC, CVT -> List.of(new Rotor("input", axis, spec.inputInertiaKgM2(), false), new Rotor("turbine", axis, spec.shaftInertiaKgM2(), false),
                    new Rotor("output", axis, spec.outputInertiaKgM2(), false));
            case DUAL_CLUTCH -> List.of(new Rotor("input", axis, spec.inputInertiaKgM2(), false), new Rotor("odd", axis, spec.shaftInertiaKgM2(), false),
                    new Rotor("even", axis, spec.shaftInertiaKgM2(), false), new Rotor("output", axis, spec.outputInertiaKgM2(), false));
        };
    }

    @Override public List<Link> links() {
        return spec.kind() == Kind.DUAL_CLUTCH ? List.of(new Link(0, 1), new Link(0, 2), new Link(1, 3), new Link(2, 3), new Link(0, 3))
                : List.of(new Link(0, 1), new Link(1, 2), new Link(0, 2));
    }

    @Override public List<String> inputs() {
        return switch (spec.kind()) {
            case MANUAL -> List.of("gear", "clutch");
            case AUTOMATIC -> List.of("gear", "lockup");
            case DUAL_CLUTCH -> List.of("gear", "preselect", "clutch");
            case CVT -> List.of("gear", "ratio", "lockup");
        };
    }

    /** The box's own internal states; a part wrapping it keeps its own after these. */
    public static final int STATES = 7;

    @Override public int internalStates() { return STATES; }
    @Override public double[] initialInternals() {
        double[] s = new double[STATES];
        if (spec.kind() == Kind.AUTOMATIC) { s[2] = 1; }
        if (spec.kind() == Kind.DUAL_CLUTCH) { s[5] = 1; }
        if (spec.kind() == Kind.CVT) { s[0] = spec.ratios()[0]; }
        return s;
    }
    @Override public int shownValues() { return 3; }

    /**
     * The gear the box drives in at the step's start, for a control rule: a manual's engaged gear (0 while the lever is
     * between gears), an automatic's or dual-clutch box's gear (-2 park), a CVT's mode (-2 park, -1 reverse, 0 neutral,
     * 1 drive).
     */
    public int engaged(PartStep step) {
        return switch (spec.kind()) {
            case MANUAL, AUTOMATIC -> (int) step.internal(0);
            case DUAL_CLUTCH -> {
                int active = (int) step.internal(4);
                yield active == 0 ? 0 : (int) step.internal(active - 1);
            }
            case CVT -> (int) step.internal(1);
        };
    }

    /** Whether a shift is under way at the step's start (synchronising, or elements or clutches handing over). */
    public boolean shifting(PartStep step) {
        return switch (spec.kind()) {
            case MANUAL -> step.internal(1) != 0;
            case AUTOMATIC -> step.internal(2) < 1;
            case DUAL_CLUTCH -> step.internal(5) < 1;
            case CVT -> false;
        };
    }

    /** A CVT's ratio at the step's start (NaN for the others). */
    public double cvtRatio(PartStep step) { return spec.kind() == Kind.CVT ? step.internal(0) : Double.NaN; }

    /** Forward gears (a CVT: 1). */
    public int gears() { return spec.kind() == Kind.CVT ? 1 : spec.ratios().length; }

    /** The ratio of a gear (1..n forward, -1 reverse). */
    public double ratio(int gear) { return gear < 0 ? spec.reverseRatio() : spec.ratios()[gear - 1]; }

    private int wanted(PartStep step, int lowest) {
        double g = step.input("gear", 0);
        return (int) Math.round(Math.clamp(Double.isFinite(g) ? g : 0, lowest, gears()));
    }

    @Override public void loads(PartStep step) {
        switch (spec.kind()) {
            case MANUAL -> manual(step);
            case AUTOMATIC -> automatic(step);
            case DUAL_CLUTCH -> dualClutch(step);
            case CVT -> cvt(step);
        }
    }

    /** A gear between two rotors at a ratio, passing power with the box's efficiency, able to carry cap (N m on a). */
    private void gearRow(PartStep step, int a, int b, double ratio, double cap) {
        step.row(new int[]{a, b}, new double[]{1, -ratio}, 0, -cap, cap, spec.efficiency());
    }

    /**
     * The same carrying only drive from a to b (the row loads a: torque on a at most zero) when oneWay: an off-going
     * element in an upshift lets go as soon as the on-coming one pulls the input below its speed, and an on-coming one in
     * a downshift takes the drive only once the input has run up to its speed, as the shift's control does.
     */
    private void elementRow(PartStep step, int a, int b, double ratio, double cap, boolean oneWay) {
        step.row(new int[]{a, b}, new double[]{1, -ratio}, 0, -cap, oneWay ? 0 : cap, spec.efficiency());
    }

    /** Whether going from one gear to another is an upshift (a smaller forward ratio). */
    private boolean upshift(int from, int to) { return from >= 1 && to >= 1 && ratio(to) < ratio(from); }

    private void manual(PartStep step) {
        int want = wanted(step, -1), engaged = (int) step.internal(0), target = (int) step.internal(1);
        double time = step.internal(2), dt = step.dt();
        int state = 0;
        if (want != engaged) {
            engaged = 0;
            if (want != 0) {
                double ratio = ratio(want), slip = step.rotorSpeed(1) - ratio * step.rotorSpeed(2);
                if (target != want) { target = want; time = 0; }
                if (Math.abs(slip) < SYNCED) { engaged = want; target = 0; time = 0; }
                else {
                    step.clutch(1, 2, ratio, want < 0 ? 0.25 * spec.synchroniserNm() : spec.synchroniserNm());
                    time += dt;
                    state = time > GRIND ? 2 : 1;
                }
            } else { target = 0; time = 0; }
        }
        if (engaged != 0) { gearRow(step, 1, 2, ratio(engaged), Double.POSITIVE_INFINITY); }
        double pedal = Math.clamp(step.input("clutch", 0), 0, 1);
        step.clutch(0, 1, 1, (1 - pedal) * spec.clutchNm());
        step.rotorTorque(1, -CHURN * step.rotorSpeed(1), -CHURN, false);
        step.setInternal(0, engaged);
        step.setInternal(1, target);
        step.setInternal(2, time);
        report(step, engaged, state, engaged == 0 ? 0 : ratio(engaged));
    }

    private void converter(PartStep step) {
        double p = step.rotorSpeed(0), t = step.rotorSpeed(1), hp = Math.max(0.5, 1e-3 * Math.abs(p)), ht = Math.max(0.5, 1e-3 * Math.abs(t));
        TorqueConverter c = spec.converter();
        double[] f = c.torques(p, t), dp = slope(c, p + hp, t, p - hp, t, hp), dt = slope(c, p, t + ht, p, t - ht, ht);
        step.torquePair(0, 1, f[0], f[1], dp[0], dt[0], dp[1], dt[1]);
        double lockup = Math.clamp(step.input("lockup", 0), 0, 1);
        if (lockup > 0) { step.clutch(0, 1, 1, lockup * spec.lockupNm()); }
        step.diagnostic("converterSpeedRatio", p != 0 ? t / p : 0);
    }

    private static double[] slope(TorqueConverter c, double p1, double t1, double p0, double t0, double h) {
        double[] a = c.torques(p1, t1), b = c.torques(p0, t0);
        return new double[]{(a[0] - b[0]) / (2 * h), (a[1] - b[1]) / (2 * h)};
    }

    private void park(PartStep step) {
        step.hold(2, Math.abs(step.rotorSpeed(2)) < PAWL ? Double.POSITIVE_INFINITY : 0);
    }

    private void automatic(PartStep step) {
        converter(step);
        int want = wanted(step, -2), current = (int) step.internal(0), previous = (int) step.internal(1);
        double progress = step.internal(2);
        if (want != current) { previous = progress >= 1 ? current : previous; current = want; progress = 0; }
        progress = Math.min(1, progress + step.dt() / spec.shiftTimeS());
        boolean shifting = progress < 1 && previous != 0 && previous != -2 && previous != current;
        boolean up = shifting && upshift(previous, current), down = shifting && upshift(current, previous);
        if (current != 0 && current != -2) { elementRow(step, 1, 2, ratio(current), spec.clutchNm() * progress, down); }
        if (shifting) { elementRow(step, 1, 2, ratio(previous), spec.clutchNm() * (1 - progress), up); }
        if (current == -2) { park(step); }
        step.rotorTorque(1, -CHURN * step.rotorSpeed(1), -CHURN, false);
        step.setInternal(0, current);
        step.setInternal(1, previous);
        step.setInternal(2, progress);
        report(step, current, progress < 1 ? 1 : 0, current == 0 || current == -2 ? 0 : ratio(current));
    }

    /** The shaft a dual-clutch box carries a gear on: 1 odd, 2 even (and reverse), 0 none. */
    private static int shaft(int gear) { return gear == 0 || gear == -2 ? 0 : gear < 0 || gear % 2 == 0 ? 2 : 1; }

    private void dualClutch(PartStep step) {
        int want = wanted(step, -2);
        int[] gear = {0, (int) step.internal(0), (int) step.internal(1)};
        int active = (int) step.internal(4), before = (int) step.internal(6);
        double progress = step.internal(5), dt = step.dt();
        double pre = step.input("preselect", want >= 1 ? want + 1 : 0);
        int preselect = (int) Math.round(Math.clamp(Double.isFinite(pre) ? pre : 0, -1, gears()));
        int driving = shaft(want), other = driving == 1 ? 2 : 1;
        int[] target = {0, gear[1], gear[2]};
        if (driving != 0) { target[driving] = want; }
        if (driving != 0 && shaft(preselect) == other) { target[other] = preselect; }
        int state = 0;
        for (int s = 1; s <= 2; s++) {
            // a shaft changes gear only while its clutch is open: not driving, and not handing over
            boolean open = !(s == active && progress > 0) && !(s == before && progress < 1);
            if (target[s] == gear[s] || !open) { continue; }
            gear[s] = 0;
            if (target[s] != 0) {
                double ratio = ratio(target[s]), slip = step.rotorSpeed(s) - ratio * step.rotorSpeed(3);
                if (Math.abs(slip) < SYNCED) { gear[s] = target[s]; }
                else { step.clutch(s, 3, ratio, spec.synchroniserNm()); state = 1; }
            }
        }
        // hand over to the shaft that has the wanted gear ready
        if (driving != 0 && gear[driving] == want && active != driving) { before = active; active = driving; progress = 0; }
        if (driving == 0) { before = active; active = 0; progress = 1; }
        progress = Math.min(1, progress + dt / spec.shiftTimeS());
        double command = driving == 0 ? 0 : Math.clamp(step.input("clutch", 1), 0, 1);
        double[] cap = {0, 0, 0};
        boolean[] oneWay = {false, false, false};
        if (active != 0) { cap[active] += spec.clutchNm() * command * progress; }
        if (before != 0 && before != active && progress < 1) {
            cap[before] += spec.clutchNm() * command * (1 - progress);
            boolean up = upshift(gear[before], gear[active]);
            oneWay[up ? before : active] = true;
        }
        step.row(new int[]{0, 1}, new double[]{1, -1}, 0, -cap[1], oneWay[1] ? 0 : cap[1], 1);
        step.row(new int[]{0, 2}, new double[]{1, -1}, 0, -cap[2], oneWay[2] ? 0 : cap[2], 1);
        for (int s = 1; s <= 2; s++) {
            if (gear[s] != 0) { gearRow(step, s, 3, ratio(gear[s]), Double.POSITIVE_INFINITY); }
            step.rotorTorque(s, -CHURN * step.rotorSpeed(s), -CHURN, false);
        }
        if (want == -2) { step.hold(3, Math.abs(step.rotorSpeed(3)) < PAWL ? Double.POSITIVE_INFINITY : 0); }
        step.setInternal(0, gear[1]);
        step.setInternal(1, gear[2]);
        step.setInternal(4, active);
        step.setInternal(5, progress);
        step.setInternal(6, before);
        int shown = active == 0 ? 0 : gear[active];
        report(step, shown, progress < 1 || state == 1 ? 1 : 0, shown == 0 ? 0 : ratio(shown));
    }

    private void cvt(PartStep step) {
        converter(step);
        int mode = wanted(step, -2);
        double low = spec.ratios()[0], high = spec.ratios()[1], r = step.internal(0);
        double aim = step.input("ratio", r);
        aim = Math.clamp(Double.isFinite(aim) ? aim : r, Math.min(low, high), Math.max(low, high));
        double move = spec.ratioRatePerS() * step.dt();
        r += Math.clamp(aim - r, -move, move);
        double total = mode == 1 ? r : mode == -1 ? spec.reverseRatio() * r : 0;
        if (total != 0) { gearRow(step, 1, 2, total, spec.beltNm()); }
        if (mode == -2) { park(step); }
        step.rotorTorque(1, -CHURN * step.rotorSpeed(1), -CHURN, false);
        step.setInternal(0, r);
        step.setInternal(1, mode);
        report(step, mode == 1 ? r : mode, 0, total);
    }

    private static void report(PartStep step, double gear, int state, double ratio) {
        step.show(0, gear);
        step.show(1, state);
        step.show(2, ratio);
        step.diagnostic("gear", gear);
        step.diagnostic("state", state);
        step.diagnostic("ratio", ratio);
    }
}
