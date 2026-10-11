package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * An electric motor whose rotor turns about the part's +X axis: the textbook torque-speed envelope of a field-weakened
 * machine -- constant torque up to the base speed where torque times speed reaches the rated power, constant power
 * above it, nothing beyond the top speed -- times the "throttle" input (-1..1; negative drives in reverse, against the
 * rotation it brakes with the same envelope). Electrical power is shaft power over efficiency when driving and times
 * it when braking (regeneration); no battery, heating or iron losses. Its housing mass is given; the housing's
 * inertia takes it as a cube of 4000 kg/m^3 (a rough overall density for motors, an estimate).
 * Specific power above 5.2 kW/kg (Siemens SP260D, 2015, an aircraft motor of record) is reported as an extrapolation.
 */
public record MotorDesign(String name, double maxTorqueNm, double maxPowerW, double maxSpeedRpm, double rotorInertiaKgM2,
                          double efficiency, double massKg) implements PartDesign {
    public static final String TYPE = "aerocore:electric_motor";
    /** Best published specific power of a production-intent electric aircraft motor (Siemens SP260D, 2015), W/kg. */
    public static final double RECORD_SPECIFIC_POWER = 5200;
    private static final double HOUSING_DENSITY = 4000;

    public MotorDesign { Objects.requireNonNull(name, "name"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        positive(issues, "maxTorqueNm", maxTorqueNm);
        positive(issues, "maxPowerW", maxPowerW);
        positive(issues, "maxSpeedRpm", maxSpeedRpm);
        positive(issues, "rotorInertiaKgM2", rotorInertiaKgM2);
        positive(issues, "massKg", massKg);
        if (!(efficiency > 0 && efficiency <= 1)) { issues.add(DesignIssue.invalid("efficiency", "Efficiency must be in (0, 1]")); }
        if (massKg > 0 && maxPowerW > 0) { DesignIssue.range(issues, "specificPowerWPerKg", maxPowerW / massKg, 0, RECORD_SPECIFIC_POWER, "Siemens SP260D 5.2 kW/kg"); }
        return issues;
    }

    static void positive(List<DesignIssue> issues, String name, double value) {
        if (!(value > 0) || !Double.isFinite(value)) { issues.add(DesignIssue.invalid(name, name + " must be positive")); }
    }

    @Override public Part compile() {
        List<DesignIssue> issues = check();
        DesignException.throwIfInvalid(issues);
        return new Compiled(this, issues);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("maxTorqueNm", maxTorqueNm, "maxPowerW", maxPowerW, "maxSpeedRpm", maxSpeedRpm,
                "rotorInertiaKgM2", rotorInertiaKgM2, "efficiency", efficiency, "massKg", massKg);
    }

    static PartDesign read(Fields f) {
        return new MotorDesign(f.string("name", "motor"), f.number("maxTorqueNm"), f.number("maxPowerW"), f.number("maxSpeedRpm"),
                f.number("rotorInertiaKgM2"), f.number("efficiency", 0.9), f.number("massKg"));
    }

    /** Torque magnitude available at speed (rad/s) in the direction of rotation (driving), N m. */
    double available(double speed) {
        double w = Math.abs(speed), top = maxSpeedRpm * Math.PI / 30;
        if (w >= top) { return 0; }
        return Math.min(maxTorqueNm, maxPowerW / Math.max(w, 1e-9));
    }

    /** Torque on the rotor (N m) for a throttle -1..1 at rotor speed (rad/s). */
    public double torque(double throttle, double speed) {
        double u = Math.clamp(throttle, -1, 1);
        if (u == 0) { return 0; }
        boolean driving = speed == 0 || Math.signum(u) == Math.signum(speed);
        // Braking is limited by the power the machine can absorb, not by its top speed.
        double magnitude = driving ? available(speed) : Math.min(maxTorqueNm, maxPowerW / Math.max(Math.abs(speed), 1e-9));
        return u * magnitude;
    }

    private static final class Compiled implements Part {
        private final MotorDesign design;
        private final List<DesignIssue> issues;
        private final MassProperties mass;

        Compiled(MotorDesign design, List<DesignIssue> issues) {
            this.design = design;
            this.issues = issues.stream().filter(i -> i.kind() == DesignIssue.Kind.EXTRAPOLATION).toList();
            double side = Math.cbrt(design.massKg / HOUSING_DENSITY);
            this.mass = MassProperties.of(design.massKg, Vec3d.ZERO, InertiaTensor.box(design.massKg, new Vec3d(side, side, side)));
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<CollisionSphere> collision() { return List.of(); }
        @Override public List<DesignIssue> issues() { return issues; }
        @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, design.rotorInertiaKgM2, true)); }
        @Override public List<String> inputs() { return List.of("throttle"); }

        @Override public void loads(PartStep step) {
            double u = step.input("throttle", 0), w = step.rotorSpeed(0);
            double torque = design.torque(u, w), delta = Math.max(1e-3, 1e-4 * Math.abs(w));
            double slope = (design.torque(u, w + delta) - design.torque(u, w - delta)) / (2 * delta);
            step.rotorTorque(0, torque, slope, false);
            double shaft = torque * w;
            step.diagnostic("throttle", Math.clamp(u, -1, 1));
            step.diagnostic("torqueNm", torque);
            step.diagnostic("speedRpm", w * 30 / Math.PI);
            step.diagnostic("shaftPowerW", shaft);
            step.diagnostic("electricPowerW", shaft >= 0 ? shaft / design.efficiency : shaft * design.efficiency);
        }
    }
}
