package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ElectricMotor;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.damage.Wear;
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
 *
 * <p>0.42: a motor given a Wearing (wearing) has one condition, MOTOR (0): windings and bearings, worn in loads() by
 * Wear.running with the Wearing's life at its rated speed and torque, from the rotor's speed and the torque it gives.
 * Its torque (and the torque's slope) is times 1 - used: a winding burning out phase by phase, none left at 1 (the rotor
 * then turns free). Diagnostic "motorUsed". Without a Wearing: no conditions, as before 0.42.
 */
public final class MotorPart implements Part {
    private final PartDesign design;
    private final ElectricMotor motor;
    private final double rotorInertiaKgM2, efficiency;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final Wearing wearing;

    /** 0.42: the name of a wearing motor's condition (index 0). */
    public static final String MOTOR = "motor";
    private static final List<String> CONDITIONS = List.of(MOTOR);

    /**
     * 0.42: what wears a motor (the caller's numbers): its life (s) at a rated speed (rad/s) and torque (N m), with
     * Wear.running's exponents on speed and load.
     */
    public record Wearing(double lifeS, double ratedSpeedRadPerS, double ratedTorqueNm, double speedExponent, double loadExponent) {
        public Wearing {
            if (!(lifeS > 0) || !(ratedTorqueNm > 0) || !(ratedSpeedRadPerS > 0)) { throw new IllegalArgumentException("Life and rated speed and torque must be positive"); }
            if (!(speedExponent >= 0) || !Double.isFinite(speedExponent) || !(loadExponent >= 0) || !Double.isFinite(loadExponent)) {
                throw new IllegalArgumentException("Exponents must be finite and not negative");
            }
        }

        /**
         * Typical exponents: the bearings' (ISO 281, as Wear.running says: a life in revolutions, 1, falling with the cube
         * of the load, 3). A winding's heat goes as the torque squared and its insulation ages faster than any power of
         * it once hot: a caller that lets its motors be overloaded may raise the load exponent.
         */
        public static Wearing typical(double lifeS, double ratedSpeedRadPerS, double ratedTorqueNm) { return new Wearing(lifeS, ratedSpeedRadPerS, ratedTorqueNm, 1, 3); }
    }

    public MotorPart(PartDesign design, ElectricMotor motor, double rotorInertiaKgM2, double efficiency, MassProperties mass, List<DesignIssue> issues) {
        this(design, motor, rotorInertiaKgM2, efficiency, mass, issues, null);
    }

    private MotorPart(PartDesign design, ElectricMotor motor, double rotorInertiaKgM2, double efficiency, MassProperties mass, List<DesignIssue> issues,
                      Wearing wearing) {
        this.wearing = wearing;
        this.design = Objects.requireNonNull(design, "design");
        this.motor = Objects.requireNonNull(motor, "motor");
        this.rotorInertiaKgM2 = rotorInertiaKgM2; this.efficiency = efficiency;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public ElectricMotor motor() { return motor; }

    /** 0.42: this motor wearing by these numbers, with the condition MOTOR; null: one that does not wear. A copy. */
    public MotorPart wearing(Wearing wearing) { return new MotorPart(design, motor, rotorInertiaKgM2, efficiency, mass, issues, wearing); }
    /** 0.42: what wears this motor, or null. */
    public Wearing wearing() { return wearing; }
    @Override public List<String> conditions() { return wearing == null ? List.of() : CONDITIONS; }

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
        if (wearing != null) {
            // 0.42: what is left of the winding gives its share of the torque; what it gives and how fast it turns wear it
            double used = step.used(0);
            if (used > 0) { double left = 1 - Math.min(used, 1); torque *= left; slope *= left; }
            double load = Math.abs(torque), speed = Math.abs(w);
            if (load < Double.POSITIVE_INFINITY && speed < Double.POSITIVE_INFINITY) {
                double worn = Wear.running(step.dt(), wearing.lifeS(), speed / wearing.ratedSpeedRadPerS(), wearing.speedExponent(), load / wearing.ratedTorqueNm(),
                        wearing.loadExponent());
                if (worn > 0) { step.wear(0, Math.min(worn, 1)); }
            }
            step.diagnostic("motorUsed", used);
        }
        step.rotorTorque(0, torque, slope, false);
        double shaft = torque * w;
        step.diagnostic("throttle", Math.clamp(u, -1, 1));
        step.diagnostic("torqueNm", torque);
        step.diagnostic("speedRpm", w * 30 / Math.PI);
        step.diagnostic("shaftPowerW", shaft);
        step.diagnostic("electricPowerW", ElectricMotor.electricalPower(shaft, efficiency));
    }
}
