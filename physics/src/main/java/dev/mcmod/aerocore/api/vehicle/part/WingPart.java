package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import dev.mcmod.aerocore.api.aero.Airfoil;
import dev.mcmod.aerocore.api.aero.Wing;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A wing (or tailplane, or a fin as a half wing on its side) whose root quarter-chord point is the part origin, in the
 * Wing frame (x forward, y up, z starboard). Loads each step from Wing.solve with the air at the root (its wind at
 * every strip's own velocity), the control surfaces' roles as inputs, and ground effect over the water surface; the
 * vortex strengths are kept as internal states to warm-start the next step. The section polars are built on the
 * first step or in warmUp.
 * Not modelled: water on the wing (a float or hull carries the vehicle on water), a propeller's slipstream over it,
 * other wings' downwash; ground effect over land arrives with the ground query.
 */
public final class WingPart implements Part {
    private final PartDesign design;
    private final Wing.Planform planform;
    private final Airfoil root, tip;
    private final List<Wing.ControlSurface> surfaces;
    private final int stripsPerHalf, strips;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final List<CollisionSphere> collision;
    private final List<String> inputs;
    private volatile Wing wing;

    public WingPart(PartDesign design, Wing.Planform planform, Airfoil root, Airfoil tip, List<Wing.ControlSurface> surfaces, int stripsPerHalf,
                    MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.planform = Objects.requireNonNull(planform, "planform");
        this.root = Objects.requireNonNull(root, "root"); this.tip = Objects.requireNonNull(tip, "tip");
        this.surfaces = List.copyOf(surfaces);
        if (stripsPerHalf < 2 || stripsPerHalf > 64) { throw new IllegalArgumentException("Strips per half must be 2..64"); }
        this.stripsPerHalf = stripsPerHalf;
        this.strips = (planform.mirrored() ? 2 : 1) * stripsPerHalf;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
        LinkedHashSet<String> roles = new LinkedHashSet<>();
        for (Wing.ControlSurface s : surfaces) { roles.add(s.role()); }
        this.inputs = List.copyOf(roles);
        this.collision = spheres(planform);
    }

    /** Spheres along the mid-chord line, about a chord apart, a third of the local chord in radius. */
    private static List<CollisionSphere> spheres(Wing.Planform p) {
        List<CollisionSphere> out = new ArrayList<>();
        double half = p.semiSpanM(), step = Math.max(Math.min(p.rootChordM(), p.tipChordM()), half / 12);
        int n = Math.max(1, (int) Math.ceil(half / step));
        double sweep = Math.tan(Math.toRadians(p.sweepDeg())), dihedral = Math.tan(Math.toRadians(p.dihedralDeg()));
        for (int side : p.mirrored() ? new int[]{-1, 1} : new int[]{1}) {
            for (int k = 0; k <= n; k++) {
                if (side < 0 && k == 0) { continue; }
                double y = half * k / n, c = p.rootChordM() + (p.tipChordM() - p.rootChordM()) * k / n;
                out.add(new CollisionSphere(new Vec3d(-y * sweep - 0.25 * c, y * dihedral, side * y), Math.max(0.05, c / 3)));
            }
        }
        return List.copyOf(out);
    }

    public Wing.Planform planform() { return planform; }
    public Airfoil rootSection() { return root; }
    public Airfoil tipSection() { return tip; }
    public List<Wing.ControlSurface> surfaces() { return surfaces; }

    /** The wing with its polars, built on first use. */
    public Wing wing() {
        Wing w = wing;
        if (w == null) {
            synchronized (this) {
                w = wing;
                if (w == null) { wing = w = new Wing(planform, root, tip, surfaces, stripsPerHalf); }
            }
        }
        return w;
    }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return collision; }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<String> inputs() { return inputs; }
    @Override public int internalStates() { return strips; }
    @Override public void warmUp() { wing(); }

    @Override public void loads(PartStep step) {
        Map<String, Double> in = new LinkedHashMap<>();
        for (String role : inputs) { in.put(role, step.input(role, 0)); }
        loads(step, in);
    }

    /** The loads with the control surfaces at these inputs (by role, -1..1), for parts that wrap a wing with their own rules. */
    public void loads(PartStep step, Map<String, Double> controls) {
        Wing w = wing();
        List<Wing.Strip> list = w.strips();
        Vec3d rootWorld = step.point(Vec3d.ZERO);
        HullDynamics.Water water = step.environment().point(rootWorld);
        FluidState air = water.air();
        Vec3d[] flow = new Vec3d[strips];
        for (int i = 0; i < strips; i++) { flow[i] = step.partDirection(air.velocity().subtract(step.velocity(list.get(i).point()))); }
        double height = water.hasWater() ? rootWorld.y() - water.surfaceY() : Double.POSITIVE_INFINITY;
        double[] warm = new double[strips];
        boolean any = false;
        for (int i = 0; i < strips; i++) { warm[i] = step.internal(i); any |= warm[i] != 0; }
        Wing.Solution s = w.solve(flow, new Wing.Air(air.densityKgPerM3(), air.dynamicViscosityPaS(), air.speedOfSoundMps()), controls,
                height, any ? warm : null);
        for (int i = 0; i < strips; i++) { step.setInternal(i, s.strength()[i]); }
        Vec3d force = step.axis(s.force()), origin = step.offset(Vec3d.ZERO);
        Vec3d moment = step.axis(s.moment()).add(origin.cross(force));
        if (force.lengthSquared() > 0 || moment.lengthSquared() > 0) {
            step.load(new Wrench(force, moment), AppliedForce.Kind.LIFT);
            step.display(new AppliedForce(AppliedForce.Kind.LIFT, origin, force));
        }
        Vec3d stream = flow[strips / 2];
        double speed = stream.length();
        Vec3d along = speed > 1e-6 ? stream.scale(1 / speed) : Vec3d.X.scale(-1);
        double drag = s.force().dot(along);
        step.diagnostic("airspeedMps", speed);
        step.diagnostic("liftN", s.force().subtract(along.scale(drag)).length());
        step.diagnostic("dragN", drag);
        step.diagnostic("alphaDeg", Math.toDegrees(Math.atan2(-stream.y(), -stream.x())));
        step.diagnostic("stalledShare", s.stalledShare());
        step.diagnostic("heightM", Double.isFinite(height) ? height : -1);
    }
}
