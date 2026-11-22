package dev.mcmod.aerocore.api.hydraulic;

import java.util.Objects;

/**
 * A positive-displacement pump for an open circuit: its largest displacement (m^3/rev), how its displacement is
 * controlled, the pressure its relief or compensator holds (Pa), a power limit (W, used by POWER_LIMITED only, 0
 * otherwise), its volumetric and hydromechanical efficiencies at its rated pressure (Pa), and its viscous drag
 * coefficient (N m per rad/s, see {@link Hydraulics#typicalDragNmPerRadPerS}).
 *
 * <p>Losses follow Wilson's model as Merritt (Hydraulic Control Systems, 1967) and Ivantysyn and Ivantysynova
 * (Hydrostatic Pumps and Motors, 2001) give it, simplified:
 * <ul>
 * <li>Leakage is proportional to pressure: a share lambda(p) = (1 - eta_v) p / p_rated of the full displacement's
 * flow (capped at 1), so a full-stroke pump's volumetric efficiency falls linearly with pressure,
 * eta_v(p) = 1 - lambda(p). It is taken in proportion to speed as well (real gap leakage is set by pressure alone), so
 * a stopped pump neither delivers nor takes back anything, the way a check valve at its outlet would have it.</li>
 * <li>Friction torque is proportional to pressure, T = D_eff p / (2 pi eta_hm) at speed, plus a viscous drag
 * c w that is there at zero pressure too.</li>
 * </ul>
 *
 * <p>Pressures are taken in [0, maxPressurePa]: below zero as zero (an open-circuit pump's outlet cannot usefully
 * fall below its inlet; cavitation is not modelled), above the setting as the setting (the relief valve or the
 * compensator holds it). The flow a fixed pump delivers at its relief setting goes over the relief valve; how much of
 * it the functions take is the caller's (FlowShare), the rest is heat (Hydraulics.powerW).
 *
 * <p>Turned backwards (negative speed) the pump runs as a motor: the flow reverses, leakage adds to the backflow,
 * and the pressure still pushes on the shaft (positive torque, which now drives the backward rotation) with the
 * friction reversed. Continuous through zero speed; an engine should never turn it so, but a time-stepper may.
 */
public record Pump(double displacementM3PerRev, Control control, double maxPressurePa, double powerLimitW,
                   double volumetricEfficiency, double hydromechanicalEfficiency, double ratedPressurePa,
                   double dragNmPerRadPerS) {

    /** How the pump's displacement is controlled. */
    public enum Control {
        /** Full displacement always (gear pumps, fixed piston pumps); a relief valve caps the pressure. */
        FIXED,
        /**
         * The compensator strokes the pump back as the pressure nears its setting: the delivered flow falls linearly
         * to zero over the last COMPENSATOR_BAND of the setting, where the pump only makes up its own leakage.
         */
        PRESSURE_COMPENSATED,
        /**
         * Pressure compensated as above, and the displacement also held down so that p x the geometric flow stays
         * within the power limit (a hyperbolic or "horsepower" controller). A mechanical controller (Bosch Rexroth LR)
         * balances p x displacement, that is the torque, so it holds the power only at the speed it was set for; an
         * electronic limit-load control follows the shaft speed as this does.
         */
        POWER_LIMITED
    }

    /**
     * The pressure-compensator's droop, as a share of its setting: the stroke collapses over the last 3 %. A rule:
     * catalogue characteristics of pressure-compensated axial piston pumps show the flow vanishing within a few per
     * cent of the setting.
     */
    public static final double COMPENSATOR_BAND = 0.03;

    /**
     * Checks the values: displacement, maximum and rated pressures positive, efficiencies in (0, 1], drag not negative,
     * the power limit positive for POWER_LIMITED and not negative otherwise; all finite.
     */
    public Pump {
        Hydraulics.positive(displacementM3PerRev, "Displacement");
        Objects.requireNonNull(control, "control");
        Hydraulics.positive(maxPressurePa, "Maximum pressure");
        if (control == Control.POWER_LIMITED) { Hydraulics.positive(powerLimitW, "Power limit"); } else { Hydraulics.nonNegative(powerLimitW, "Power limit"); }
        Hydraulics.efficiency(volumetricEfficiency, "Volumetric efficiency");
        Hydraulics.efficiency(hydromechanicalEfficiency, "Hydromechanical efficiency");
        Hydraulics.positive(ratedPressurePa, "Rated pressure");
        Hydraulics.nonNegative(dragNmPerRadPerS, "Drag");
    }

    /**
     * An axial piston pump with the typical efficiencies at 280 bar (Hydraulics.AXIAL_PISTON_*) and the typical drag;
     * powerLimitW is used by POWER_LIMITED only (pass 0 otherwise).
     */
    public static Pump axialPiston(double displacementM3PerRev, Control control, double maxPressurePa, double powerLimitW) {
        return new Pump(displacementM3PerRev, control, maxPressurePa, powerLimitW, Hydraulics.AXIAL_PISTON_VOLUMETRIC,
                Hydraulics.AXIAL_PISTON_HYDROMECHANICAL, Hydraulics.AXIAL_PISTON_RATED_PA,
                Hydraulics.typicalDragNmPerRadPerS(displacementM3PerRev, Hydraulics.AXIAL_PISTON_RATED_PA));
    }

    /** A fixed external gear pump with the typical efficiencies at 200 bar (Hydraulics.GEAR_*) and the typical drag. */
    public static Pump gear(double displacementM3PerRev, double maxPressurePa) {
        return new Pump(displacementM3PerRev, Control.FIXED, maxPressurePa, 0, Hydraulics.GEAR_VOLUMETRIC,
                Hydraulics.GEAR_HYDROMECHANICAL, Hydraulics.GEAR_RATED_PA,
                Hydraulics.typicalDragNmPerRadPerS(displacementM3PerRev, Hydraulics.GEAR_RATED_PA));
    }

    private double working(double pressurePa) {
        return Math.max(0, Math.min(Hydraulics.finite(pressurePa, "Pressure"), maxPressurePa));
    }

    private double leak(double p) { return Math.min(1, (1 - volumetricEfficiency) * p / ratedPressurePa); }

    /** Volumetric efficiency at full stroke and a pressure (Pa): 1 - (1 - eta_v) p / p_rated, not below 0. */
    public double volumetricEfficiencyAt(double pressurePa) { return 1 - leak(working(pressurePa)); }

    /**
     * The share of the largest displacement the pump runs at (0..1) at a shaft speed (rad/s) and pressure (Pa).
     * FIXED: 1. PRESSURE_COMPENSATED: lambda + (1 - lambda) c, c falling linearly from 1 at (1 - COMPENSATOR_BAND)
     * p_max to 0 at p_max, so the delivered flow vanishes at the setting while the pump still makes up its leakage
     * lambda. POWER_LIMITED: that, but not above P / (p D |w| / (2 pi)). Continuous, not increasing in pressure.
     */
    public double displacementFraction(double speedRadPerS, double pressurePa) {
        Hydraulics.finite(speedRadPerS, "Speed");
        return stroke(Math.abs(speedRadPerS), working(pressurePa));
    }

    private double stroke(double absSpeedRadPerS, double p) {
        if (control == Control.FIXED) { return 1; }
        double lambda = leak(p);
        double c = (maxPressurePa - p) / (COMPENSATOR_BAND * maxPressurePa);
        // the ends exactly: full stroke below the band, only the leakage at the setting
        double s = c >= 1 ? 1 : c <= 0 ? lambda : lambda + (1 - lambda) * c;
        if (control == Control.POWER_LIMITED) {
            double fullPowerW = p * displacementM3PerRev * absSpeedRadPerS / (2 * Math.PI);
            if (fullPowerW > powerLimitW) { s = Math.min(s, powerLimitW / fullPowerW); }
        }
        return s;
    }

    /**
     * The flow the pump delivers (m^3/s) at a shaft speed (rad/s) and pressure (Pa): Q = D w / (2 pi) (s - lambda),
     * not below zero, s the displacement fraction; a fixed pump gives eta_v(p) D w / (2 pi). Backwards (w below
     * zero) D w / (2 pi) (s + lambda), negative. Continuous; not increasing in pressure.
     */
    public double flowM3PerS(double speedRadPerS, double pressurePa) {
        Hydraulics.finite(speedRadPerS, "Speed");
        double p = working(pressurePa);
        double s = stroke(Math.abs(speedRadPerS), p);
        double lambda = leak(p);
        double full = displacementM3PerRev * speedRadPerS / (2 * Math.PI);
        return speedRadPerS >= 0 ? full * Math.max(0, s - lambda) : full * (s + lambda);
    }

    /**
     * The torque the pump takes from its shaft (N m; positive resists forward rotation) at a speed (rad/s) and
     * pressure (Pa): T = s D p / (2 pi eta_hm) + c w once turning faster than Hydraulics.STANDSTILL_RAD_PER_S; at
     * standstill the ideal s D p / (2 pi); backwards s D p eta_hm / (2 pi) + c w (the pressure drives it, less its
     * friction). The friction share is ramped linearly between, so the torque is continuous in speed.
     */
    public double torqueNm(double speedRadPerS, double pressurePa) {
        Hydraulics.finite(speedRadPerS, "Speed");
        double p = working(pressurePa);
        double ideal = stroke(Math.abs(speedRadPerS), p) * displacementM3PerRev * p / (2 * Math.PI);
        double sigma = Hydraulics.ramp(speedRadPerS, Hydraulics.STANDSTILL_RAD_PER_S);
        double friction = sigma >= 0 ? (1 / hydromechanicalEfficiency - 1) * sigma : (1 - hydromechanicalEfficiency) * sigma;
        return ideal * (1 + friction) + dragNmPerRadPerS * speedRadPerS;
    }

    /** Shaft power the pump takes (W): torque x speed (negative when it runs backwards as a motor). */
    public double shaftPowerW(double speedRadPerS, double pressurePa) {
        return torqueNm(speedRadPerS, pressurePa) * speedRadPerS;
    }

    /** Hydraulic power the pump delivers (W): p x Q, p taken in [0, maxPressurePa]. */
    public double hydraulicPowerW(double speedRadPerS, double pressurePa) {
        return working(pressurePa) * flowM3PerS(speedRadPerS, pressurePa);
    }

    /**
     * Heat the pump makes (W): shaft power - hydraulic power, not negative at any speed or pressure (leakage, friction
     * and drag; zero but for rounding for a lossless pump at zero speed).
     */
    public double heatW(double speedRadPerS, double pressurePa) {
        return shaftPowerW(speedRadPerS, pressurePa) - hydraulicPowerW(speedRadPerS, pressurePa);
    }
}
