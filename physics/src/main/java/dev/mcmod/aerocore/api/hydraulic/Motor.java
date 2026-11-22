package dev.mcmod.aerocore.api.hydraulic;

/**
 * A hydraulic motor of fixed displacement (m^3/rev; a two-speed or variable motor is one record per displacement, or
 * {@link Hydrostat}'s stroke): its volumetric and hydromechanical efficiencies at its rated pressure (Pa) and its
 * viscous drag coefficient (N m per rad/s).
 *
 * <p>The pressure drop dp is the A port over the B port and the flow is into A; both and the speed are signed, a
 * positive dp driving the motor forward. Losses as for {@link Pump} (Wilson's model, simplified; Merritt, Hydraulic
 * Control Systems, 1967; Ivantysyn and Ivantysynova, Hydrostatic Pumps and Motors, 2001):
 * <ul>
 * <li>Leakage from the high-pressure port past the rotating group, in proportion to the pressure drop and to the
 * geometric flow: Q = D w / (2 pi) + lambda D |w| / (2 pi), lambda = k dp, k = (1 / eta_v - 1) / p_rated (capped at
 * +-MAX_LEAK), so the volumetric efficiency at the rated pressure is eta_v. Taking it with the speed means a stalled
 * motor holding a load does not creep (not modelled).</li>
 * <li>Friction in proportion to the pressure, opposing the rotation: driven by the oil (dp and w the same sign) the
 * motor gives eta_hm D dp / (2 pi); driven by its load as a pump it takes D dp / (2 pi eta_hm); minus the drag c w.
 * The friction is ramped in over Hydraulics.STANDSTILL_RAD_PER_S, so at standstill it gives the ideal D dp / (2 pi)
 * (breakaway torque, lower in real motors, is not modelled).</li>
 * </ul>
 */
public record Motor(double displacementM3PerRev, double volumetricEfficiency, double hydromechanicalEfficiency,
                    double ratedPressurePa, double dragNmPerRadPerS) {

    /** The largest leakage share of the geometric flow (both signs), so the flow and speed relations stay invertible. */
    public static final double MAX_LEAK = 0.5;

    /** Checks the values: displacement and rated pressure positive, efficiencies in (0, 1], drag not negative. */
    public Motor {
        Hydraulics.positive(displacementM3PerRev, "Displacement");
        Hydraulics.efficiency(volumetricEfficiency, "Volumetric efficiency");
        Hydraulics.efficiency(hydromechanicalEfficiency, "Hydromechanical efficiency");
        Hydraulics.positive(ratedPressurePa, "Rated pressure");
        Hydraulics.nonNegative(dragNmPerRadPerS, "Drag");
    }

    /** An axial piston motor with the typical efficiencies at 280 bar (Hydraulics.AXIAL_PISTON_*) and the typical drag. */
    public static Motor axialPiston(double displacementM3PerRev) {
        return new Motor(displacementM3PerRev, Hydraulics.AXIAL_PISTON_VOLUMETRIC, Hydraulics.AXIAL_PISTON_HYDROMECHANICAL,
                Hydraulics.AXIAL_PISTON_RATED_PA, Hydraulics.typicalDragNmPerRadPerS(displacementM3PerRev, Hydraulics.AXIAL_PISTON_RATED_PA));
    }

    /** An external gear motor with the typical efficiencies at 200 bar (Hydraulics.GEAR_*) and the typical drag. */
    public static Motor gear(double displacementM3PerRev) {
        return new Motor(displacementM3PerRev, Hydraulics.GEAR_VOLUMETRIC, Hydraulics.GEAR_HYDROMECHANICAL,
                Hydraulics.GEAR_RATED_PA, Hydraulics.typicalDragNmPerRadPerS(displacementM3PerRev, Hydraulics.GEAR_RATED_PA));
    }

    private double leak(double dp) {
        double k = (1 / volumetricEfficiency - 1) / ratedPressurePa;
        return Math.max(-MAX_LEAK, Math.min(MAX_LEAK, k * dp));
    }

    /** Volumetric efficiency at a pressure drop (Pa, either sign): 1 / (1 + |lambda|), eta_v at the rated pressure. */
    public double volumetricEfficiencyAt(double pressureDropPa) {
        return 1 / (1 + Math.abs(leak(Hydraulics.finite(pressureDropPa, "Pressure drop"))));
    }

    /** The flow (m^3/s, into A) the motor takes to turn at a speed (rad/s) under a pressure drop (Pa): see the class. */
    public double flowM3PerS(double speedRadPerS, double pressureDropPa) {
        return flow(displacementM3PerRev, Hydraulics.finite(speedRadPerS, "Speed"), Hydraulics.finite(pressureDropPa, "Pressure drop"));
    }

    /**
     * The speed (rad/s) a flow (m^3/s, into A) turns the motor at under a pressure drop (Pa): the exact inverse of
     * flowM3PerS, w = 2 pi Q / (D (1 + lambda sign Q)); 2 pi Q eta_v / D when the oil drives it.
     */
    public double speedRadPerS(double flowM3PerS, double pressureDropPa) {
        return speed(displacementM3PerRev, Hydraulics.finite(flowM3PerS, "Flow"), Hydraulics.finite(pressureDropPa, "Pressure drop"));
    }

    /**
     * The torque (N m, positive forward) the motor gives its shaft at a speed (rad/s) under a pressure drop (Pa):
     * eta_hm D dp / (2 pi) - c w when the oil drives it, D dp / (2 pi eta_hm) - c w when its load drives it (see the class).
     */
    public double torqueNm(double speedRadPerS, double pressureDropPa) {
        return torque(displacementM3PerRev, Hydraulics.finite(speedRadPerS, "Speed"), Hydraulics.finite(pressureDropPa, "Pressure drop"));
    }

    /** The pressure drop (Pa) the motor needs to give a torque (N m) at a speed (rad/s): the exact inverse of torqueNm. */
    public double pressureDropPa(double speedRadPerS, double torqueNm) {
        return pressure(displacementM3PerRev, Hydraulics.finite(speedRadPerS, "Speed"), Hydraulics.finite(torqueNm, "Torque"));
    }

    /** The largest torque (N m) at low speed with a relief pressure (Pa) across it: eta_hm D p / (2 pi). */
    public double maxTorqueNm(double reliefPa) {
        return hydromechanicalEfficiency * displacementM3PerRev * Hydraulics.nonNegative(reliefPa, "Relief pressure") / (2 * Math.PI);
    }

    /** Hydraulic power into the motor (W): dp x Q. */
    public double hydraulicPowerW(double speedRadPerS, double pressureDropPa) {
        return pressureDropPa * flowM3PerS(speedRadPerS, pressureDropPa);
    }

    /** Shaft power out of the motor (W): torque x speed. */
    public double shaftPowerW(double speedRadPerS, double pressureDropPa) {
        return torqueNm(speedRadPerS, pressureDropPa) * speedRadPerS;
    }

    /** Heat the motor makes (W): hydraulic power in - shaft power out, not negative in any quadrant. */
    public double heatW(double speedRadPerS, double pressureDropPa) {
        return hydraulicPowerW(speedRadPerS, pressureDropPa) - shaftPowerW(speedRadPerS, pressureDropPa);
    }

    double flow(double d, double speed, double dp) {
        return d / (2 * Math.PI) * (speed + leak(dp) * Math.abs(speed));
    }

    double speed(double d, double flow, double dp) {
        return 2 * Math.PI * flow / (d * (1 + leak(dp) * Math.signum(flow)));
    }

    double torque(double d, double speed, double dp) {
        double ideal = d * dp / (2 * Math.PI);
        double sigma = Hydraulics.ramp(speed, Hydraulics.STANDSTILL_RAD_PER_S);
        double loss = dp * sigma >= 0 ? 1 - hydromechanicalEfficiency : 1 / hydromechanicalEfficiency - 1;
        return ideal - Math.abs(ideal) * loss * sigma - dragNmPerRadPerS * speed;
    }

    double pressure(double d, double speed, double torque) {
        // torque + c w = ideal (1 - loss sign(ideal) sigma), the factor positive, so sign(ideal) = sign(torque + c w)
        double y = torque + dragNmPerRadPerS * speed;
        double s = (y >= 0 ? 1 : -1) * Hydraulics.ramp(speed, Hydraulics.STANDSTILL_RAD_PER_S);
        double loss = s >= 0 ? 1 - hydromechanicalEfficiency : 1 / hydromechanicalEfficiency - 1;
        return 2 * Math.PI * y / (d * (1 - loss * s));
    }
}
