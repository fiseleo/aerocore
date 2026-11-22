package dev.mcmod.aerocore.api.hydraulic;

/**
 * Hydraulic drives for heavy machinery at "medium" fidelity: a pump delivers displacement x speed less a leakage, a
 * cylinder moves at flow / area and pushes with pressure x area, a pump that cannot supply every function shares its
 * flow ({@link FlowShare}), and the pump's shaft torque goes back on what drives it. There are no line pressure
 * dynamics (the pressure at a function is the caller's: set by its load, capped by the relief), no oil temperature and
 * no valve spool details. Formulas and typical values with their sources only; which machine uses what is the caller's.
 *
 * <p>Every method is cheap enough for each 5 ms substep, allocates nothing unless it says so, never returns NaN for
 * finite inputs and is continuous in its inputs (Coulomb-like friction changes sign over a small speed,
 * {@link #STANDSTILL_RAD_PER_S}, instead of jumping), so a time-stepper does not chatter. Non-finite inputs throw
 * IllegalArgumentException.
 *
 * <p>Sign conventions: speeds are signed (positive forward); a pump's pressure is its outlet over its inlet; a motor's
 * pressure drop is its A port over its B port (positive drives it forward) and its flow is into A; a cylinder extends
 * with positive velocity and force. Displacements are per revolution (m^3/rev), as catalogues give them.
 *
 * <p>This class holds the unit factors, the ideal relations of a displacement machine, the typical values of pumps and
 * motors and the argument checks of the package.
 */
public final class Hydraulics {
    private Hydraulics() {}

    /** One bar in Pa. */
    public static final double BAR_PA = 1e5;

    /** One litre per minute in m^3/s. */
    public static final double LITRE_PER_MIN_M3_PER_S = 1e-3 / 60;

    /** One cm^3 (a catalogue's cc/rev) in m^3. */
    public static final double CM3_M3 = 1e-6;

    /** One revolution per minute in rad/s. */
    public static final double RPM_RAD_PER_S = 2 * Math.PI / 60;

    /**
     * Volumetric efficiency of an axial piston pump or motor at its rated pressure: 0.92 to 0.98 for swash-plate and
     * bent-axis units in good condition (Ivantysyn and Ivantysynova, Hydrostatic Pumps and Motors, 2001); 0.95 taken.
     */
    public static final double AXIAL_PISTON_VOLUMETRIC = 0.95;

    /**
     * Hydromechanical (torque) efficiency of an axial piston unit at its rated pressure: about 0.9 to 0.95
     * (Ivantysyn and Ivantysynova 2001); 0.92 taken.
     */
    public static final double AXIAL_PISTON_HYDROMECHANICAL = 0.92;

    /** Rated (continuous) pressure of common open-circuit axial piston pumps, Pa: 280 bar (e.g. Bosch Rexroth A10VO). */
    public static final double AXIAL_PISTON_RATED_PA = 280 * BAR_PA;

    /**
     * Volumetric efficiency of an external gear pump or motor at its rated pressure: about 0.85 to 0.95, lower than a
     * piston unit's because of the tooth-tip and side-plate gaps (Ivantysyn and Ivantysynova 2001); 0.90 taken.
     */
    public static final double GEAR_VOLUMETRIC = 0.90;

    /** Hydromechanical efficiency of an external gear unit at its rated pressure: about 0.8 to 0.9; 0.85 taken. */
    public static final double GEAR_HYDROMECHANICAL = 0.85;

    /** Rated (continuous) pressure of an external gear pump, Pa: 200 bar (catalogue ratings run from 160 to 280 bar). */
    public static final double GEAR_RATED_PA = 200 * BAR_PA;

    /**
     * No-load drag of a displacement machine at {@link #DRAG_REFERENCE_RAD_PER_S}, as a share of its ideal torque at
     * rated pressure: a rule of the order the torque-loss maps at zero pressure show (viscous drag of the running
     * gaps and churning; Ivantysyn and Ivantysynova 2001), not a measurement of one unit.
     */
    public static final double DRAG_SHARE = 0.02;

    /** The speed DRAG_SHARE is taken at: 1500 rpm, in rad/s. */
    public static final double DRAG_REFERENCE_RAD_PER_S = 1500 * RPM_RAD_PER_S;

    /**
     * Speed over which friction torque changes sign, rad/s (about 10 rpm): friction is ramped linearly between
     * -STANDSTILL and +STANDSTILL instead of jumping at zero speed. A regularisation for time-stepping, not a measured
     * stiction curve; at standstill the machine gives its ideal torque.
     */
    public static final double STANDSTILL_RAD_PER_S = 1.0;

    /**
     * A typical viscous drag coefficient, N m per rad/s: DRAG_SHARE of the ideal torque D p_rated / (2 pi) at the
     * reference speed, so the drag torque c w is linear in speed (Wilson's viscous loss term, as in Merritt,
     * Hydraulic Control Systems, 1967).
     */
    public static double typicalDragNmPerRadPerS(double displacementM3PerRev, double ratedPressurePa) {
        positive(displacementM3PerRev, "Displacement");
        positive(ratedPressurePa, "Rated pressure");
        return DRAG_SHARE * displacementM3PerRev * ratedPressurePa / (2 * Math.PI) / DRAG_REFERENCE_RAD_PER_S;
    }

    /** Geometric flow of a displacement machine, m^3/s: Q = D w / (2 pi), D in m^3/rev, w in rad/s (signed). */
    public static double flowM3PerS(double speedRadPerS, double displacementM3PerRev) {
        nonNegative(displacementM3PerRev, "Displacement");
        return displacementM3PerRev * finite(speedRadPerS, "Speed") / (2 * Math.PI);
    }

    /** Geometric speed of a displacement machine fed a flow, rad/s: w = 2 pi Q / D (signed with the flow). */
    public static double speedRadPerS(double flowM3PerS, double displacementM3PerRev) {
        positive(displacementM3PerRev, "Displacement");
        return 2 * Math.PI * finite(flowM3PerS, "Flow") / displacementM3PerRev;
    }

    /** Ideal torque of a displacement machine, N m: T = D dp / (2 pi), D in m^3/rev (signed with the pressure). */
    public static double torqueNm(double pressurePa, double displacementM3PerRev) {
        nonNegative(displacementM3PerRev, "Displacement");
        return displacementM3PerRev * finite(pressurePa, "Pressure") / (2 * Math.PI);
    }

    /** Ideal pressure a displacement machine needs for a torque, Pa: dp = 2 pi T / D. */
    public static double pressureForTorquePa(double torqueNm, double displacementM3PerRev) {
        positive(displacementM3PerRev, "Displacement");
        return 2 * Math.PI * finite(torqueNm, "Torque") / displacementM3PerRev;
    }

    /** Ideal pressure on an area for a force, Pa: p = F / A (signed with the force). */
    public static double pressureForForcePa(double forceN, double areaM2) {
        positive(areaM2, "Area");
        return finite(forceN, "Force") / areaM2;
    }

    /**
     * Hydraulic power of a flow at a pressure, W: P = p Q. Also the heat a flow makes when it drops the pressure
     * through a valve (a relief, a throttle) without doing work.
     */
    public static double powerW(double pressurePa, double flowM3PerS) {
        return finite(pressurePa, "Pressure") * finite(flowM3PerS, "Flow");
    }

    static double finite(double value, String name) {
        if (!Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be finite"); }
        return value;
    }

    static double positive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException(name + " must be finite and positive"); }
        return value;
    }

    static double nonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0) { throw new IllegalArgumentException(name + " must be finite and not negative"); }
        return value;
    }

    static double efficiency(double value, String name) {
        if (!(value > 0 && value <= 1)) { throw new IllegalArgumentException(name + " must be in (0, 1]"); }
        return value;
    }

    /** x / scale clamped to [-1, 1]: the regularised sign of friction. */
    static double ramp(double x, double scale) {
        return Math.max(-1, Math.min(1, x / scale));
    }
}
