package dev.mcmod.aerocore.api.hydraulic;

import java.util.Objects;

/**
 * A closed-loop hydrostatic drive: a pump whose stroke is commanded (-1..1, the sign choosing the direction) feeding a
 * motor whose stroke is 0..1 (1 for a fixed motor; bent-axis variable motors go down to about 0.2 to 0.3). The pump's
 * control kind is not used; its maxPressurePa is the loop's high-pressure relief. The pump is taken as driven
 * forward (an engine). Pressures are the loop's A side over its B side (positive when the pump drives forward).
 *
 * <p>The textbook relations (Merritt, Hydraulic Control Systems, 1967; Ivantysyn and Ivantysynova, Hydrostatic Pumps
 * and Motors, 2001): w_motor / w_pump = (s_p D_p eta_vp) eta_vm / (s_m D_m), the torque limit eta_hm,m s_m D_m p_relief
 * / (2 pi), and the input torque T_in = s_p D_p dp / (2 pi eta_hm,p) driving, s_p D_p dp eta_hm,p / (2 pi) braking.
 * Leakage is taken as a share of each unit's own flow, as in Pump and Motor at full stroke; near neutral a real pump's
 * leakage, set by pressure alone, is a larger share and the motor creeps under load (not modelled). The pump's drag
 * c w_pump is not included: add pump().dragNmPerRadPerS() x its speed.
 */
public record Hydrostat(Pump pump, Motor motor) {
    /** Checks that both are given. */
    public Hydrostat {
        Objects.requireNonNull(pump, "pump");
        Objects.requireNonNull(motor, "motor");
    }

    private static double pumpStroke(double s) {
        if (!(s >= -1 && s <= 1)) { throw new IllegalArgumentException("Pump stroke must be in [-1, 1]"); }
        return s;
    }

    private static double motorStroke(double s) {
        if (!(s > 0 && s <= 1)) { throw new IllegalArgumentException("Motor stroke must be in (0, 1]"); }
        return s;
    }

    /**
     * Motor speed over pump speed at the strokes and a loop pressure (Pa, either sign; leakage by its size, taken up to
     * the relief): (s_p D_p eta_vp(p)) eta_vm(p) / (s_m D_m), signed with the pump stroke.
     */
    public double speedRatio(double pumpStroke, double motorStroke, double pressurePa) {
        double p = Math.abs(Hydraulics.finite(pressurePa, "Pressure"));
        return pumpStroke(pumpStroke) * pump.displacementM3PerRev() * pump.volumetricEfficiencyAt(p) * motor.volumetricEfficiencyAt(Math.min(p, pump.maxPressurePa()))
                / (motorStroke(motorStroke) * motor.displacementM3PerRev());
    }

    /** The largest output torque (N m) at low speed at a motor stroke: the relief pressure across the motor, eta_hm s_m D_m p / (2 pi). */
    public double outputTorqueLimitNm(double motorStroke) {
        return motorStroke(motorStroke) * motor.maxTorqueNm(pump.maxPressurePa());
    }

    /**
     * The loop pressure (Pa) that gives an output torque (N m, positive forward) at a motor speed (rad/s) and stroke:
     * Motor.pressureDropPa at the stroke's displacement (its friction and drag included), held within +-relief (beyond
     * it the relief opens and the torque is the limit).
     */
    public double loopPressurePa(double motorSpeedRadPerS, double motorStroke, double outputTorqueNm) {
        double d = motorStroke(motorStroke) * motor.displacementM3PerRev();
        double p = motor.pressure(d, Hydraulics.finite(motorSpeedRadPerS, "Motor speed"), Hydraulics.finite(outputTorqueNm, "Torque"));
        double relief = pump.maxPressurePa();
        return Math.max(-relief, Math.min(relief, p));
    }

    /**
     * The torque (N m) the pump takes from the engine at a pump stroke for an output torque (N m) at a motor speed
     * (rad/s) and stroke: the loop pressure on the pump, s_p D_p dp / (2 pi eta_hm) while the pump drives the loop and
     * s_p D_p dp eta_hm / (2 pi) while the loop drives the pump (engine braking, negative). Continuous: the two meet at
     * zero.
     */
    public double inputTorqueNm(double pumpStroke, double motorSpeedRadPerS, double motorStroke, double outputTorqueNm) {
        double dp = loopPressurePa(motorSpeedRadPerS, motorStroke, outputTorqueNm);
        double ideal = pumpStroke(pumpStroke) * pump.displacementM3PerRev() * dp / (2 * Math.PI);
        double eta = pump.hydromechanicalEfficiency();
        return ideal >= 0 ? ideal / eta : ideal * eta;
    }
}
