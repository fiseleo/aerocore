package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.hydraulic.Motor;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * 0.43: a hydraulic motor on a circuit (by name) turning its rotor "output" (about the part's axis): a crawler's
 * travel motor, a winch, a cutter head's drive. Its valve, on "command" (-1..1), asks the circuit for the flow to turn
 * the motor at that share of its top speed, and the motor holds its rotor to that speed with at most the torque its
 * relief pressure gives (a drivetrain row: the speed is the flow's, the torque what the load takes, both ways, so a
 * counterbalance valve holds a hanging load and the motor stalls at the relief). A short circuit (flow sharing) slows
 * it with the others. "brake" (0..1) is a spring-applied brake's share. The pressure it asks for is the one its last
 * step's torque needed.
 *
 * <p>With a Drum (a winch): the rope it pays out is an internal state, positive speed paying out (lowering): the rope
 * length grows by speed x drum radius / parts of line, within the drum's least and most; "tension" (N, the hook's load
 * as the game finds it on the rope) loads the drum by tension x radius / parts of line. Diagnostics "ropeLengthM" and
 * "ropeRateMps" give the hook's length and speed for the rope coupling.
 *
 * <p>Diagnostics: "speedRadPerS", "torqueNm", "pressurePa", "flowM3PerS". Internal states: the last torque (N m), then
 * the rope length (m) with a drum.
 */
public final class HydraulicMotorPart implements Part {
    /** A winch drum: radius to the rope's middle layer (m), parts of line (falls), the rope length at the start, least and most (m). */
    public record Drum(double radiusM, int partsOfLine, double initialLengthM, double minLengthM, double maxLengthM) {
        public Drum {
            if (!(radiusM > 0) || partsOfLine < 1 || !(minLengthM >= 0) || !(maxLengthM >= minLengthM) || initialLengthM < minLengthM || initialLengthM > maxLengthM) {
                throw new IllegalArgumentException("Drum values out of range");
            }
        }
    }

    private final PartDesign design;
    private final Motor motor;
    private final String circuit;
    private final double reliefPa, maxSpeedRadPerS, brakeNm, inertiaKgM2;
    private final Vec3d axis;
    private final Drum drum;
    private final MassProperties mass;
    private final List<DesignIssue> issues;

    /**
     * A motor of a circuit relieved at reliefPa (the line's relief), turning at most maxSpeedRadPerS (its valve's
     * flow over its displacement), with a holding brake (N m) and its rotating group's inertia, on a shaft along axis.
     */
    public HydraulicMotorPart(PartDesign design, Motor motor, String circuit, double reliefPa, double maxSpeedRadPerS, double brakeNm, double inertiaKgM2,
                              Vec3d axis, Drum drum, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design"); this.motor = Objects.requireNonNull(motor, "motor");
        this.circuit = Objects.requireNonNull(circuit, "circuit");
        if (!(reliefPa > 0) || !(maxSpeedRadPerS > 0) || !(brakeNm >= 0) || !(inertiaKgM2 > 0)) { throw new IllegalArgumentException("Motor values out of range"); }
        this.reliefPa = reliefPa; this.maxSpeedRadPerS = maxSpeedRadPerS; this.brakeNm = brakeNm; this.inertiaKgM2 = inertiaKgM2;
        this.axis = axis.normalized(); this.drum = drum; this.mass = Objects.requireNonNull(mass, "mass"); this.issues = List.copyOf(issues);
    }

    public Motor motor() { return motor; }
    public String circuit() { return circuit; }
    public Drum drum() { return drum; }
    /** The most torque the relief lets it give, N m. */
    public double maxTorqueNm() { return motor.maxTorqueNm(reliefPa); }
    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor("output", axis, inertiaKgM2, false)); }
    @Override public List<String> inputs() { return drum == null ? List.of("command", "brake") : List.of("command", "brake", "tension"); }
    @Override public int internalStates() { return drum == null ? 1 : 2; }
    @Override public double[] initialInternals() { return drum == null ? new double[1] : new double[]{0, drum.initialLengthM()}; }

    @Override public void loads(PartStep step) {
        double command = Math.clamp(step.input("command", 0), -1, 1), share = step.hydraulicShare(circuit);
        double target = command * maxSpeedRadPerS * share;
        if (drum != null) {
            double length = step.internal(1);
            // the drum stops at its rope's ends
            if ((length <= drum.minLengthM() && target < 0) || (length >= drum.maxLengthM() && target > 0)) { target = 0; }
            double tension = Math.max(step.input("tension", 0), 0);
            if (tension > 0) { step.rotorTorque(0, -tension * drum.radiusM() / drum.partsOfLine(), 0, true); }
            step.diagnostic("ropeLengthM", length);
            step.diagnostic("ropeRateMps", target * drum.radiusM() / drum.partsOfLine());
        }
        double torque = maxTorqueNm();
        int row = step.row(new int[]{0}, new double[]{1}, target, -torque, torque, motor.hydromechanicalEfficiency());
        step.carry(row);
        double brake = Math.clamp(step.input("brake", 0), 0, 1) * brakeNm;
        if (brake > 0) { step.hold(0, brake); }
        double last = Math.abs(step.internal(0)), pressure = Math.min(motor.pressureDropPa(Math.abs(target), last), reliefPa);
        double flow = Math.abs(command) > 0 ? motor.flowM3PerS(Math.abs(command) * maxSpeedRadPerS, Math.max(pressure, 0)) : 0;
        step.hydraulicDemand(circuit, Math.max(flow, 0), Math.max(pressure, 0));
        step.diagnostic("speedRadPerS", step.rotorSpeed(0));
        step.diagnostic("torqueNm", step.internal(0));
        step.diagnostic("pressurePa", Math.max(pressure, 0));
        step.diagnostic("flowM3PerS", Math.max(flow, 0));
    }

    @Override public void after(PartAfter a) {
        if (a.carried() instanceof Integer row) { a.setInternal(0, a.rowTorque(row)); }
        if (drum != null) {
            double length = a.internal(1) + a.rotorSpeed(0) * drum.radiusM() / drum.partsOfLine() * a.dt();
            a.setInternal(1, Math.clamp(length, drum.minLengthM(), drum.maxLengthM()));
        }
    }
}
