package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Mount;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * 0.43: a hook block hanging on a rope from a point outside any vehicle: a tower crane's trolley, a gantry's crab.
 * The point (world, m), its velocity (m/s: the trolley and the slewing jib) and the rope's length (m, the hoist's) come
 * on control channels each tick ("anchor_x", "anchor_y", "anchor_z", "anchor_vx", "anchor_vy", "anchor_vz",
 * "rope_length"; "rope_on" above 0.5 hangs it) and are carried on with the velocity through the tick's substeps. The
 * rope pulls the part's rope point (its frame's origin, or its "rope" mount point) toward the anchor when longer than
 * its length: E A / L times the stretch, with a damper on the stretching speed (implicit); slack, it pulls nothing.
 * The structure is taken as rigid and does not move for it (a tower crane's load is the game's to check against its
 * chart). Diagnostics: "tensionN", "ropeLengthM", "stretchM".
 */
public final class HangPart implements Part {
    private final PartDesign design;
    private final MassProperties mass;
    private final List<CollisionSphere> collision;
    private final Map<String, Mount> points;
    private final double stiffnessN, dampingNs;
    private final List<DesignIssue> issues;

    /** A hook of a mass and contact spheres with mount points (a load's sling hangs on them), on a rope of E A stiffnessN (N) and damping (N s/m). */
    public HangPart(PartDesign design, MassProperties mass, List<CollisionSphere> collision, Map<String, Mount> points, double stiffnessN, double dampingNs,
                    List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design"); this.mass = Objects.requireNonNull(mass, "mass");
        this.collision = List.copyOf(collision); this.points = Map.copyOf(points);
        if (!(stiffnessN > 0) || !Double.isFinite(stiffnessN) || !(dampingNs >= 0)) { throw new IllegalArgumentException("A rope has a positive finite E A and a damping"); }
        this.stiffnessN = stiffnessN; this.dampingNs = dampingNs; this.issues = List.copyOf(issues);
    }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return collision; }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public Map<String, Mount> mountPoints() { return points; }
    @Override public List<String> inputs() {
        return List.of("anchor_x", "anchor_y", "anchor_z", "anchor_vx", "anchor_vy", "anchor_vz", "anchor_t", "rope_length", "rope_on");
    }

    @Override public void loads(PartStep step) {
        if (step.input("rope_on", 0) <= 0.5) { step.diagnostic("tensionN", 0); return; }
        Vec3d anchor = new Vec3d(step.input("anchor_x", 0), step.input("anchor_y", 0), step.input("anchor_z", 0));
        Vec3d velocity = new Vec3d(step.input("anchor_vx", 0), step.input("anchor_vy", 0), step.input("anchor_vz", 0));
        // the anchor moves on through the tick: the game sets it once a tick ("anchor_t", the step time it was taken at)
        double since = step.input("anchor_t", step.time());
        anchor = anchor.add(velocity.scale(Math.clamp(step.time() - since, 0, 0.1)));
        double length = Math.max(step.input("rope_length", 0), 0.05);
        Mount rope = points.get("rope");
        Vec3d at = step.point(rope == null ? Vec3d.ZERO : rope.position());
        Vec3d span = anchor.subtract(at);
        double d = span.length();
        step.diagnostic("ropeLengthM", length);
        if (!(d > length) || d < 1e-9) { step.diagnostic("tensionN", 0); step.diagnostic("stretchM", 0); return; }
        Vec3d e = span.scale(1 / d);
        double k = stiffnessN / length, stretch = d - length;
        RigidBodyState s = step.state();
        Vec3d offset = at.subtract(s.position());
        double pull = k * stretch;
        step.force(new AppliedForce(AppliedForce.Kind.CONTACT, offset, e.scale(pull)));
        // the rope's damping on how fast it stretches: d/dt (d) = -(v_point - v_anchor) . e
        step.damper(offset, e, dampingNs, velocity.dot(e));
        step.diagnostic("tensionN", pull);
        step.diagnostic("stretchM", stretch);
    }

    /**
     * The game's control channels for an anchor at a world point moving at a velocity, taken at a step time (s: the
     * vehicle's clock, PartStep.time, which the game keeps as its tick count x the tick), with a rope of a length.
     */
    public static Map<String, Double> controls(Vec3d anchor, Vec3d velocity, double timeS, double ropeLengthM) {
        Map<String, Double> m = new java.util.LinkedHashMap<>();
        m.put("anchor_x", anchor.x()); m.put("anchor_y", anchor.y()); m.put("anchor_z", anchor.z());
        m.put("anchor_vx", velocity.x()); m.put("anchor_vy", velocity.y()); m.put("anchor_vz", velocity.z());
        m.put("anchor_t", timeS); m.put("rope_length", ropeLengthM); m.put("rope_on", 1.0);
        return m;
    }
}
