package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.hydraulic.Cylinder;
import dev.mcmod.aerocore.api.hydraulic.Motor;
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
 * 0.43: a link of a machine's work equipment or frame that moves on its joint (Part.joint), driven by a hydraulic
 * actuator: an excavator's boom, stick and bucket, a loader's lift arm, a slewing ring, a loader's front frame on its
 * centre hinge, a telescopic section, a dump body, a forklift's mast stage. What is mounted on its mount points rides
 * on it; it may carry holds (a bucket's, a dump body's), a deck and seats (a work platform's basket), contact spheres.
 *
 * <p>Actuators (the valve on "command", -1..1: the share of the valve's full flow, positive moving q up):
 * <ul>
 * <li>Ram: cylinders from a pin on what the link is mounted on (the mount's frame, which the joint does not move) to a
 *     pin on the link (its own frame). Their length L(q) and lever dL/dq follow the joint; the rate is the flow over the
 *     side's area over the lever (Cylinder), the reach the relief pressure's push or pull times the lever (Cylinder.maxPushN,
 *     maxPullN). Centred, the load-holding valves hold up to their own pressure; on "float" (above 0.5) the ram gives
 *     nothing either way (a dozer blade riding the ground). It asks the circuit for the valve's flow at the pressure the
 *     last step's load on the joint needed on the side it fills.</li>
 * <li>Slew: a hydraulic motor through a gear (ratio, efficiency) turning a revolute joint (a slewing ring); centred, the
 *     swing brake holds with its torque.</li>
 * <li>Free: a pin with friction (N m or N): a grapple's rotator left free, a dragline's bucket hitch.</li>
 * <li>Held (null): the joint holds where it is (moved by the game only).</li>
 * </ul>
 * Diagnostics: "q" (rad or m), "rate", "effort" (the actuator's generalized force), "load" (outside it), "pressurePa";
 * for a ram "lengthM" and "forceN".
 */
public final class LinkPart implements Part {
    /** How the joint is driven. */
    public sealed interface Actuator permits Ram, Slew, Free {}

    /**
     * Cylinders from basePin (the mount's frame) to rodPin (the link's frame), on a circuit, relieved at reliefPa, held
     * centred by load-holding valves up to holdPa, through a valve that passes at most valveFlowM3PerS.
     */
    public record Ram(Cylinder cylinder, Vec3d basePin, Vec3d rodPin, String circuit, double reliefPa, double holdPa, double valveFlowM3PerS) implements Actuator {
        public Ram {
            Objects.requireNonNull(cylinder); Objects.requireNonNull(basePin); Objects.requireNonNull(rodPin); Objects.requireNonNull(circuit);
            if (!(reliefPa > 0) || !(holdPa >= reliefPa) || !(valveFlowM3PerS > 0)) { throw new IllegalArgumentException("Ram values out of range"); }
        }
    }

    /** A motor through a gear of ratio (motor turns per link turn) and efficiency, on a circuit, relieved at reliefPa, through a valve of valveFlowM3PerS, braked centred (N m at the link). */
    public record Slew(Motor motor, double ratio, double efficiency, String circuit, double reliefPa, double valveFlowM3PerS, double brakeNm) implements Actuator {
        public Slew {
            Objects.requireNonNull(motor); Objects.requireNonNull(circuit);
            if (!(ratio > 0) || !(efficiency > 0 && efficiency <= 1) || !(reliefPa > 0) || !(valveFlowM3PerS > 0) || !(brakeNm >= 0)) {
                throw new IllegalArgumentException("Slew values out of range");
            }
        }
    }

    /** A free pin with friction (N m for a turning joint, N for a sliding one). */
    public record Free(double friction) implements Actuator {
        public Free { if (!(friction >= 0)) { throw new IllegalArgumentException("Friction must not be negative"); } }
    }

    /**
     * A cutter turning on its own rotor "cutter" for the link's MILL or FACE tools (a cold planer's drum, a cutterhead,
     * a bucket wheel): its radius (m) to the picks, its inertia (kg m^2), its axis (link frame). A shaft or a hydraulic
     * motor drives the rotor.
     */
    public record Cutter(double radiusM, double inertiaKgM2, Vec3d axis) {
        public Cutter {
            if (!(radiusM > 0) || !(inertiaKgM2 > 0)) { throw new IllegalArgumentException("A cutter has a radius and an inertia"); }
            axis = axis.normalized();
        }
    }

    /** Below this share of the valve the lever is centred. */
    public static final double DEADBAND = 0.02;

    private final PartDesign design;
    private final MassProperties mass;
    private final List<CollisionSphere> collision;
    private final Map<String, Mount> points;
    private final Part.Joint joint;
    private final Actuator actuator;
    private final List<Part.Hold> holds;
    private final List<WorldBox> deck;
    private final List<Vec3d> seats;
    private final List<DesignIssue> issues;
    private final List<Part.Tool> tools;
    private final Cutter cutter;

    public LinkPart(PartDesign design, MassProperties mass, List<CollisionSphere> collision, Map<String, Mount> points, Part.Joint joint, Actuator actuator,
                    List<Part.Hold> holds, List<WorldBox> deck, List<Vec3d> seats, List<DesignIssue> issues) {
        this(design, mass, collision, points, joint, actuator, holds, deck, seats, issues, List.of(), null);
    }

    /** A link with cutting tools (Part.Tool: a bucket's lip and teeth, a blade's edge) and, for mills and cutterheads, their cutter. */
    public LinkPart(PartDesign design, MassProperties mass, List<CollisionSphere> collision, Map<String, Mount> points, Part.Joint joint, Actuator actuator,
                    List<Part.Hold> holds, List<WorldBox> deck, List<Vec3d> seats, List<DesignIssue> issues, List<Part.Tool> tools, Cutter cutter) {
        this.tools = List.copyOf(tools);
        this.cutter = cutter;
        for (Part.Tool t : this.tools) {
            if ((t.kind() == Part.Tool.Kind.MILL || t.kind() == Part.Tool.Kind.FACE) && cutter == null) { throw new IllegalArgumentException("A mill or a cutterhead turns on a cutter"); }
        }
        this.design = Objects.requireNonNull(design, "design"); this.mass = Objects.requireNonNull(mass, "mass");
        this.collision = List.copyOf(collision); this.points = Map.copyOf(points); this.joint = Objects.requireNonNull(joint, "joint");
        this.actuator = actuator; this.holds = List.copyOf(holds); this.deck = List.copyOf(deck); this.seats = List.copyOf(seats);
        this.issues = List.copyOf(issues);
        if (actuator instanceof Slew && joint.kind() != Part.Joint.Kind.REVOLUTE) { throw new IllegalArgumentException("A slew turns a revolute joint"); }
    }

    public Actuator actuator() { return actuator; }
    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return collision; }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public Map<String, Mount> mountPoints() { return points; }
    @Override public Part.Joint joint() { return joint; }
    @Override public List<Part.Hold> holds() { return holds; }
    @Override public List<WorldBox> deck() { return deck; }
    @Override public List<Vec3d> seats() { return seats; }
    @Override public List<Part.Tool> tools() { return tools; }
    @Override public List<Rotor> rotors() { return cutter == null ? List.of() : List.of(new Rotor("cutter", cutter.axis(), cutter.inertiaKgM2(), false)); }
    @Override public List<String> inputs() {
        List<String> in = new java.util.ArrayList<>();
        if (actuator instanceof Ram) { in.add("command"); in.add("float"); }
        else if (actuator instanceof Slew) { in.add("command"); }
        if (!tools.isEmpty()) { in.add("full"); in.add("pushed"); }
        return List.copyOf(in);
    }
    public Cutter cutter() { return cutter; }

    /** Where the joint at q puts a point of the link's own frame, in its mount's frame. */
    public Vec3d moved(Vec3d partPoint, double q) {
        if (joint.kind() == Part.Joint.Kind.PRISMATIC) { return partPoint.add(joint.axis().scale(q)); }
        Vec3d o = joint.origin();
        return o.add(Orientation.rotationVector(joint.axis().scale(q)).rotate(partPoint.subtract(o)));
    }

    /** A ram's length at q (m). */
    public double ramLength(Ram ram, double q) { return moved(ram.rodPin(), q).subtract(ram.basePin()).length(); }

    /** dL/dq of a ram at q (m per rad or per m). */
    public double lever(Ram ram, double q) {
        double h = 1e-4;
        return (ramLength(ram, q + h) - ramLength(ram, q - h)) / (2 * h);
    }

    @Override public void loads(PartStep step) {
        double q = step.jointValue(), load = step.jointLoad();
        step.diagnostic("q", q);
        step.diagnostic("rate", step.jointRate());
        step.diagnostic("effort", step.jointEffort());
        step.diagnostic("load", load);
        switch (actuator) {
            case null -> { }
            case Free free -> step.driveJoint(0, free.friction(), free.friction());
            case Ram ram -> ram(step, ram, q, load);
            case Slew slew -> slew(step, slew, load);
        }
        if (!tools.isEmpty()) {
            boolean full = step.input("full", 0) > 0.5;
            double pushed = Math.max(step.input("pushed", 0), 0);
            ToolCutter.Drive drive = cutter == null ? null : new ToolCutter.Drive(0, cutter.radiusM());
            for (Part.Tool t : tools) { ToolCutter.work(step, t, drive, full, pushed); }
        }
    }

    private void ram(PartStep step, Ram ram, double q, double load) {
        Cylinder cyl = ram.cylinder();
        double lever = lever(ram, q);
        if (Math.abs(lever) < 1e-6) { lever = lever < 0 ? -1e-6 : 1e-6; }
        double arm = Math.abs(lever);
        step.diagnostic("lengthM", ramLength(ram, q));
        step.diagnostic("forceN", step.jointEffort() / lever);
        if (step.input("float", 0) > 0.5) {
            step.driveJoint(0, 0, 0);
            step.diagnostic("pressurePa", 0);
            return;
        }
        double command = Math.clamp(step.input("command", 0), -1, 1);
        // extending the ram moves q the way the lever's sign says
        boolean upExtends = lever > 0;
        if (Math.abs(command) < DEADBAND) {
            double up = (upExtends ? cyl.maxPushN(ram.holdPa()) : cyl.maxPullN(ram.holdPa())) * arm;
            double down = (upExtends ? cyl.maxPullN(ram.holdPa()) : cyl.maxPushN(ram.holdPa())) * arm;
            step.driveJoint(0, up, down);
            // the held load's pressure on the side that carries it
            double force = -load / lever;
            step.diagnostic("pressurePa", force >= 0 ? force / cyl.headAreaM2() : -force / cyl.rodSideAreaM2());
            return;
        }
        double flow = Math.abs(command) * ram.valveFlowM3PerS() * step.hydraulicShare(ram.circuit());
        boolean extending = (command > 0) == upExtends;
        double speed = extending ? cyl.extendingVelocityMps(flow) : cyl.retractingVelocityMps(flow);
        double rate = Math.signum(command) * speed / arm;
        double up = (upExtends ? cyl.maxPushN(ram.reliefPa()) : cyl.maxPullN(ram.reliefPa())) * arm;
        double down = (upExtends ? cyl.maxPullN(ram.reliefPa()) : cyl.maxPushN(ram.reliefPa())) * arm;
        step.driveJoint(rate, up, down);
        // the pressure the move needs: the load on the joint (last step) as a ram force on the side being filled
        double force = -load / lever;   // + pushing
        double pressure = extending ? Math.max(force, 0) / cyl.headAreaM2() : Math.max(-force, 0) / cyl.rodSideAreaM2();
        pressure = Math.min(pressure, ram.reliefPa());
        step.hydraulicDemand(ram.circuit(), Math.abs(command) * ram.valveFlowM3PerS(), pressure);
        step.diagnostic("pressurePa", pressure);
    }

    private void slew(PartStep step, Slew slew, double load) {
        double command = Math.clamp(step.input("command", 0), -1, 1);
        if (Math.abs(command) < DEADBAND) {
            step.driveJoint(0, slew.brakeNm(), slew.brakeNm());
            step.diagnostic("pressurePa", 0);
            return;
        }
        double flow = Math.abs(command) * slew.valveFlowM3PerS() * step.hydraulicShare(slew.circuit());
        double motorSpeed = slew.motor().speedRadPerS(flow, 0);
        double rate = Math.signum(command) * motorSpeed / slew.ratio();
        double reach = slew.motor().maxTorqueNm(slew.reliefPa()) * slew.ratio() * slew.efficiency();
        step.driveJoint(rate, reach, reach);
        double torque = Math.abs(step.jointEffort()) / (slew.ratio() * slew.efficiency());
        double pressure = Math.min(slew.motor().pressureDropPa(Math.abs(motorSpeed), torque), slew.reliefPa());
        step.hydraulicDemand(slew.circuit(), Math.abs(command) * slew.valveFlowM3PerS(), Math.max(pressure, 0));
        step.diagnostic("pressurePa", Math.max(pressure, 0));
    }
}
