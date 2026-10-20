package dev.mcmod.aerocore.api.petroleum;

/**
 * Pumping and drilling (0.26).
 * <ul>
 * <li>Rod (beam) pump: displacement (pi / 4) D^2 S N times a volumetric efficiency (about 0.8); the work to lift a flow
 *     from a depth is rho g H q over the system efficiency (about 0.5 from motor to fluid) (Economides et al., ch. 12).</li>
 * <li>Drilling: Teale's mechanical specific energy at the bit, the energy to remove a volume of rock, is close to the
 *     rock's compressive strength when drilling is efficient; at the rig's power P a bit of area A drills eta P / (A
 *     UCS) (eta about 0.35 for practical drilling) (Teale, Int. J. Rock Mech. 2 (1965) 57).</li>
 * </ul>
 */
public final class Lift {
    private Lift() {}

    public static final double PUMP_VOLUMETRIC = 0.8, LIFT_EFFICIENCY = 0.5, DRILLING_EFFICIENCY = 0.35;

    /** A rod pump's rate: plunger diameter (m), stroke (m), strokes per minute, m^3/s. */
    public static double rodPumpM3PerS(double plungerM, double strokeM, double strokesPerMinute) {
        return Math.PI / 4 * plungerM * plungerM * strokeM * strokesPerMinute / 60 * PUMP_VOLUMETRIC;
    }

    /** Power to lift a flow (m^3/s) of a density from a depth (m) against a wellhead pressure (Pa), W. */
    public static double liftPowerW(double flowM3PerS, double density, double depthM, double wellheadPa) {
        return flowM3PerS * (density * 9.81 * depthM + wellheadPa) / LIFT_EFFICIENCY;
    }

    /** Rate of penetration of a bit (diameter m) driven with a power (W) into rock of a compressive strength (Pa), m/s. */
    public static double penetrationMPerS(double powerW, double bitM, double rockStrengthPa) {
        return DRILLING_EFFICIENCY * powerW / (Math.PI / 4 * bitM * bitM * rockStrengthPa);
    }
}
