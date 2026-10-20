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

    public FramePart(PartDesign design, MassProperties mass, Map<String, Mount> mountPoints, List<Vec3d> seats, FrameTwist frame, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.mountPoints = Map.copyOf(mountPoints);
        this.seats = List.copyOf(seats);
        this.frame = frame;
        this.issues = List.copyOf(issues);
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
