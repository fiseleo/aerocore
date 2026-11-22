package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.hydraulic.Pump;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * 0.43: a hydraulic pump on an engine's shaft feeding a circuit (by name): its rotor "input" (about the part's axis)
 * is what a shaft joins. Each step it offers the circuit the flow it delivers at its shaft speed and the circuit's
 * last pressure (Pump.flowM3PerS: a compensated pump gives none at its set pressure, a power-limited one less as
 * the pressure rises) and loads its shaft with the torque that pressure takes (Pump.torqueNm), so the engine feels
 * the work: lugging down with the boom against the relief, idling light with the levers centred. The circuit's pressure
 * is the one its actuators asked for in the last step (VehicleDynamics, flow sharing), at most this pump's relief.
 * Diagnostics: "pressurePa", "flowM3PerS", "torqueNm", "powerW" (hydraulic, given), "share".
 * <p>
 * Speed sensing (optional, the anti-stall control of a mobile machine's pump): between a low and a high shaft speed
 * the pump strokes from none to its full displacement, so an engine pulled down below the high speed sheds the pump's
 * load instead of stalling, and an engine being started turns it free. Without it (NaN) the pump is as above.
 */
public final class HydraulicPumpPart implements Part {
    private final PartDesign design;
    private final Pump pump;
    private final String circuit;
    private final Vec3d axis;
    private final double inertiaKgM2;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final double senseLowRadPerS, senseHighRadPerS;

    /** A pump of a circuit on a shaft along axis (part frame), with its rotating group's inertia (kg m^2). */
    public HydraulicPumpPart(PartDesign design, Pump pump, String circuit, Vec3d axis, double inertiaKgM2, MassProperties mass, List<DesignIssue> issues) {
        this(design, pump, circuit, axis, inertiaKgM2, mass, issues, Double.NaN, Double.NaN);
    }

    /** A pump with speed sensing: no stroke at or below senseLowRadPerS, full from senseHighRadPerS (rad/s of its shaft). */
    public HydraulicPumpPart(PartDesign design, Pump pump, String circuit, Vec3d axis, double inertiaKgM2, MassProperties mass, List<DesignIssue> issues,
                             double senseLowRadPerS, double senseHighRadPerS) {
        if (!Double.isNaN(senseLowRadPerS) && !(senseHighRadPerS > senseLowRadPerS && senseLowRadPerS >= 0)) {
            throw new IllegalArgumentException("Speed sensing needs a low speed under its high one");
        }
        this.senseLowRadPerS = senseLowRadPerS; this.senseHighRadPerS = senseHighRadPerS;
        this.design = Objects.requireNonNull(design, "design"); this.pump = Objects.requireNonNull(pump, "pump");
        this.circuit = Objects.requireNonNull(circuit, "circuit"); this.axis = axis.normalized();
        if (!(inertiaKgM2 > 0)) { throw new IllegalArgumentException("A pump's inertia must be positive"); }
        this.inertiaKgM2 = inertiaKgM2; this.mass = Objects.requireNonNull(mass, "mass"); this.issues = List.copyOf(issues);
    }

    public Pump pump() { return pump; }
    public String circuit() { return circuit; }
    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor("input", axis, inertiaKgM2, false)); }

    @Override public void loads(PartStep step) {
        double speed = Math.abs(step.rotorSpeed(0)), pressure = step.hydraulicPressure(circuit);
        double stroke = Double.isNaN(senseLowRadPerS) ? 1 : Math.clamp((speed - senseLowRadPerS) / (senseHighRadPerS - senseLowRadPerS), 0, 1);
        double flow = pump.flowM3PerS(speed, pressure);
        if (stroke < 1) { flow *= stroke; }
        step.hydraulicSupply(circuit, Math.max(flow, 0), pump.maxPressurePa());
        double torque = pump.torqueNm(speed, pressure), sense = Math.signum(step.rotorSpeed(0));
        // the torque against the shaft's turning, with its slope (drag and displacement against speed), from inside the vehicle
        double h = Math.max(1e-3 * speed, 1e-2), slope = (pump.torqueNm(speed + h, pressure) - torque) / h;
        if (stroke < 1) { torque *= stroke; slope *= stroke; }
        if (sense != 0) { step.rotorTorque(0, -sense * torque, -slope, false); }
        step.diagnostic("pressurePa", pressure);
        step.diagnostic("flowM3PerS", Math.max(flow, 0));
        step.diagnostic("torqueNm", sense * torque);
        step.diagnostic("powerW", pressure * Math.max(flow, 0));
        step.diagnostic("share", step.hydraulicShare(circuit));
    }
}
