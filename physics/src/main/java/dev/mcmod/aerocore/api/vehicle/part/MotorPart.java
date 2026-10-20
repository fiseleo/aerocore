package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ElectricMotor;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * An electric motor whose rotor turns about the part's +X axis: its ElectricMotor envelope times the "throttle" input
 * (-1..1; negative drives in reverse, against the rotation it brakes). Electrical power is shaft power over efficiency
 * when driving and times it when braking (regeneration); no battery, heating or iron losses.
 */
public final class MotorPart implements Part {
    private final PartDesign design;
    private final ElectricMotor motor;
    private final double rotorInertiaKgM2, efficiency;
    private final MassProperties mass;
    private final List<DesignIssue> issues;

    public MotorPart(PartDesign design, ElectricMotor motor, double rotorInertiaKgM2, double efficiency, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.motor = Objects.requireNonNull(motor, "motor");
        this.rotorInertiaKgM2 = rotorInertiaKgM2; this.efficiency = efficiency;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public ElectricMotor motor() { return motor; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, rotorInertiaKgM2, true)); }
    @Override public List<String> inputs() { return List.of("throttle"); }

    @Override public void loads(PartStep step) {
        double u = step.input("throttle", 0), w = step.rotorSpeed(0);
        double torque = motor.torque(u, w), delta = Math.max(1e-3, 1e-4 * Math.abs(w));
        double slope = (motor.torque(u, w + delta) - motor.torque(u, w - delta)) / (2 * delta);
        step.rotorTorque(0, torque, slope, false);
        double shaft = torque * w;
        step.diagnostic("throttle", Math.clamp(u, -1, 1));
        step.diagnostic("torqueNm", torque);
        step.diagnostic("speedRpm", w * 30 / Math.PI);
        step.diagnostic("shaftPowerW", shaft);
        step.diagnostic("electricPowerW", ElectricMotor.electricalPower(shaft, efficiency));
    }
}
