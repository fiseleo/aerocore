package dev.mcmod.aerocore.api.manufacture;

/**
 * Setting solid rivets (0.35).
 *
 * <ul>
 * <li>A rivet of shank diameter d through a grip g sticks out 1.5 d to form its shop head (common practice: the head
 *     comes out about 1.5 d across), so it is g + 1.5 d long.</li>
 * <li>Squeezing it upsets that protrusion, a cylinder d across and 1.5 d high, into a head 1.5 d across (its height from
 *     the volume, 2/3 d): the force at the end of the squeeze is the upsetting force of that cylinder
 *     (Forming.upsetForceN: flow stress K e^n with the friction factor 1 + 0.4 mu D / h; Groover eq. 19.12-19.13).</li>
 * </ul>
 */
public final class Riveting {
    private Riveting() {}

    /** Protrusion and head diameter over the shank's diameter. */
    public static final double PROTRUSION = 1.5, HEAD = 1.5;

    /** A rivet's length for a grip (m) and its diameter (m), m. */
    public static double lengthM(double gripM, double diameterM) {
        if (!(diameterM > 0)) { throw new IllegalArgumentException("The diameter must be positive"); }
        return Math.max(gripM, 0) + PROTRUSION * diameterM;
    }

    /** The squeeze at the end of forming the head of a rivet of a diameter (m): strength coefficient (Pa), hardening exponent and friction, N. */
    public static double squeezeForceN(double diameterM, double strengthPa, double hardeningExponent, double friction) {
        if (!(diameterM > 0) || !(strengthPa > 0)) { throw new IllegalArgumentException("Diameter and strength must be positive"); }
        double h0 = PROTRUSION * diameterM, volume = Math.PI / 4 * diameterM * diameterM * h0;
        double head = HEAD * diameterM, h = volume / (Math.PI / 4 * head * head);
        return Forming.upsetForceN(volume, h0, h, friction, strengthPa, hardeningExponent);
    }
}
