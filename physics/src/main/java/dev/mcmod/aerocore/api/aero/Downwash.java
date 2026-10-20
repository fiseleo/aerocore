package dev.mcmod.aerocore.api.aero;

/**
 * How hard a rotor's wake blows below it and along the ground (0.30), for drawing dust and spray: a display
 * approximation, not a wake model.
 * <ul>
 * <li>Below a hovering rotor the slipstream speeds up as it contracts, from the induced velocity at the disk toward
 * twice it far down (momentum theory): v(z) = v_i (1 + (z/R) / sqrt(1 + (z/R)^2)), the form of the far-wake
 * contraction used in simple wake models (Leishman, Principles of Helicopter Aerodynamics, 2nd ed., ch. 2).</li>
 * <li>Where it meets the ground it turns into a radial wall jet whose peak speed beyond about one radius decays roughly
 * as r^-1 (Leishman ch. 10, outwash surveys); taken here as 0.9 v(h) (R/r)^1.1 from r = R, rising from the axis to R.</li>
 * <li>A rotor high above the ground stirs it less as the wake mixes out: past two radii up the outwash falls off as
 * exp(-(h - 2R)/(2R)), so at about four diameters there is little left.</li>
 * </ul>
 */
public final class Downwash {
    private Downwash() {}

    /** The wake's speed (m/s) a depth below the rotor disk (m), for a rotor of a radius (m) and induced velocity (m/s). */
    public static double wakeSpeed(double inducedMps, double radiusM, double depthM) {
        if (!(radiusM > 0)) { return 0; }
        double z = Math.max(0, depthM) / radiusM;
        return Math.max(0, inducedMps) * (1 + z / Math.sqrt(1 + z * z));
    }

    /** The outwash's peak speed (m/s) over the ground at a distance from the rotor's axis (m), the rotor a height above it (m). */
    public static double outwash(double inducedMps, double radiusM, double heightM, double radialM) {
        if (!(radiusM > 0) || !(heightM >= 0)) { return 0; }
        double w = wakeSpeed(inducedMps, radiusM, heightM), r = Math.max(0, radialM) / radiusM;
        double shape = r <= 1 ? 0.9 * r : 0.9 * Math.pow(1 / r, 1.1);
        double high = heightM <= 2 * radiusM ? 1 : Math.exp(-(heightM - 2 * radiusM) / (2 * radiusM));
        return w * shape * high;
    }
}
