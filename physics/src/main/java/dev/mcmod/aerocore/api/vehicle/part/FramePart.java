package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.FrameTwist;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Mount;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A vehicle's frame (0.33: a ladder, tube or backbone chassis): mass, mount points for what it carries (axles, the
 * engine, the body), seats, and its twist (FrameTwist; null: rigid) that VehicleDynamics solves each step (FrameFlex). Its
 * internal states are the stations' twists (rad), its shown values the same to a tenth of a milliradian.
 *
 * <p>0.42: a frame given a Wearing (wearing) has one condition, STRUCTURE (0), which only the game changes (a crash,
 * corrosion, a repair; it does not wear itself). A damaged frame twists more easily: its twist stiffness is times
 * 1 - stiffnessLoss x used (Part.frameStiffness: the whole matrix scaled, so it stays symmetric with rows summing to
 * zero); a rigid frame (no FrameTwist) only carries the condition. Diagnostic "structureUsed". Without a Wearing: no
 * conditions, as before 0.42.
 */
public final class FramePart implements Part {
    /** How finely the twist is shown, rad. */
    public static final double SHOWN_TWIST_RAD = 1e-4;

    private final PartDesign design;
    private final MassProperties mass;
    private final Map<String, Mount> mountPoints;
    private final List<Vec3d> seats;
    private final FrameTwist frame;
    private final List<DesignIssue> issues;
    private final Wearing wearing;

    /** 0.42: the name of a wearing frame's condition (index 0). */
    public static final String STRUCTURE = "structure";
    private static final List<String> CONDITIONS = List.of(STRUCTURE);

    /** 0.42: what damage does to a frame (the caller's rule): the share of its twist stiffness lost with its structure used up (in proportion before). */
    public record Wearing(double stiffnessLoss) {
        public Wearing {
            if (!(stiffnessLoss >= 0 && stiffnessLoss <= 1)) { throw new IllegalArgumentException("Stiffness loss 0..1"); }
        }
        /** Rule: a frame cracked and bent to scrap keeps 0.4 of its twist stiffness (rails with cracked flanges and loose cross-members still carry torque). */
        public static Wearing typical() { return new Wearing(0.6); }
    }

    public FramePart(PartDesign design, MassProperties mass, Map<String, Mount> mountPoints, List<Vec3d> seats, FrameTwist frame, List<DesignIssue> issues) {
        this(design, mass, mountPoints, seats, frame, issues, null);
    }

    private FramePart(PartDesign design, MassProperties mass, Map<String, Mount> mountPoints, List<Vec3d> seats, FrameTwist frame, List<DesignIssue> issues,
                      Wearing wearing) {
        this.design = Objects.requireNonNull(design, "design");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.mountPoints = Map.copyOf(mountPoints);
        this.seats = List.copyOf(seats);
        this.frame = frame;
        this.issues = List.copyOf(issues);
        this.wearing = wearing;
    }

    /** 0.42: this frame damageable by this rule, with the condition STRUCTURE; null: one that is not. A copy. */
    public FramePart wearing(Wearing wearing) { return new FramePart(design, mass, mountPoints, seats, frame, issues, wearing); }
    /** 0.42: what damage does to this frame, or null. */
    public Wearing wearing() { return wearing; }
    @Override public List<String> conditions() { return wearing == null ? List.of() : CONDITIONS; }

    /** 0.42: the share of its twist stiffness a damaged frame keeps (1 while undamaged or not wearing). */
    @Override public double frameStiffness(PartStep step) {
        if (wearing == null) { return 1; }
        double used = Math.min(step.used(0), 1);
        return used > 0 ? 1 - wearing.stiffnessLoss() * used : 1;
    }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public Map<String, Mount> mountPoints() { return mountPoints; }
    @Override public List<Vec3d> seats() { return seats; }
    @Override public FrameTwist frame() { return frame; }
    @Override public int internalStates() { return frame == null ? 0 : frame.stations(); }
    @Override public int shownValues() { return frame == null ? 0 : frame.stations(); }

    @Override public void loads(PartStep step) {
        if (wearing != null) { step.diagnostic("structureUsed", step.used(0)); }
        if (frame == null) { return; }
        double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < frame.stations(); i++) {
            double t = step.internal(i);
            step.show(i, PartStep.stepped(step.shown(i), t, SHOWN_TWIST_RAD));
            low = Math.min(low, t); high = Math.max(high, t);
        }
        step.diagnostic("twistDeg", Math.toDegrees(high - low));
    }
}
