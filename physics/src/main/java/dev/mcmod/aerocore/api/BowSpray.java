package dev.mcmod.aerocore.api;

/**
 * Spray thrown at a moving bow: whether the bow wave breaks, how high it reaches and the velocity of the water leaving
 * it in a frame moving with the water (forward along the motion, lateral outward, up), m/s.
 * <ul>
 * <li>Displacement bow (Noblesse et al. 2008, 2013; review in J. Hydrodynamics 25(4) 2013, eqs. 5-10): crest height
 * Z_b = (U^2/g) min(2.2 tan a / ((1 + F) cos a), 1/2), F = U / sqrt(g D), a the waterline entrance half-angle, D the
 * draft. The crest spills unsteadily (spray) when 4.4 tan a / cos a > 1 + F. Water deflected along the bow surface at
 * speed U leaves at U (1 - cos a) forward and U sin a outward; its vertical speed sqrt(2 g Z_b) reaches the crest.</li>
 * <li>Planing bottom (Savitsky 1964 spray root; whisker spray after Savitsky, DeLorme and Datla 2007; apex after
 * Savitsky and Morabito, as restated by Ghadimi et al. 2015): spray root at a_s = atan(pi tan t / (2 tan b)) to the
 * keel; the whisker sheet leaves at 2 a_s from the keel with the boat speed V; vertical speed
 * V_V = V sin a_s + V cos a_s sin a_s sin b_E, b_E = atan((1 - 2/pi) tan b); apex V_V^2 / (2 g).</li>
 * </ul>
 * Empirical relations with their stated scatter; droplet break-up, wind and the spray's mass are not modelled.
 */
public record BowSpray(boolean breaking, double crestHeightM, double forwardMps, double lateralMps, double verticalMps) {
    public static final BowSpray NONE = new BowSpray(false, 0, 0, 0, 0);

    /** Noblesse bow wave; entranceHalfAngle in rad (pi/2 for a blunt, flat bow). */
    public static BowSpray displacementBow(double speed, double entranceHalfAngle, double draft, double gravity) {
        Checks.positive(draft, "Draft"); Checks.positive(gravity, "Gravity");
        if (!(speed > 0)) { return NONE; }
        double a = Math.clamp(entranceHalfAngle, 1e-3, Math.PI / 2), froude = speed / Math.sqrt(gravity * draft);
        double ratio = a >= Math.PI / 2 - 1e-9 ? Double.POSITIVE_INFINITY : Math.tan(a) / Math.cos(a);
        double crest = speed * speed / gravity * Math.min(2.2 * ratio / (1 + froude), 0.5);
        boolean breaking = 4.4 * ratio > 1 + froude;
        return new BowSpray(breaking, crest, speed * (1 - Math.cos(a)), speed * Math.sin(a), Math.sqrt(2 * gravity * crest));
    }

    /** Spray-root angle to the keel in plan view, rad (trim and deadrise in rad). */
    public static double sprayRootAngle(double trim, double deadrise) {
        return Math.atan(Math.PI * Math.tan(trim) / (2 * Math.tan(deadrise)));
    }

    /** Whisker spray of a planing bottom at trim t and deadrise b (rad), speed V. */
    public static BowSpray planing(double speed, double trim, double deadrise, double gravity) {
        Checks.positive(gravity, "Gravity");
        if (!(speed > 0) || !(trim > 0) || !(deadrise > 0)) { return NONE; }
        double root = sprayRootAngle(trim, deadrise), edge = 2 * root;
        double effective = Math.atan((1 - 2 / Math.PI) * Math.tan(deadrise));
        double vertical = speed * Math.sin(root) + speed * Math.cos(root) * Math.sin(root) * Math.sin(effective);
        return new BowSpray(true, vertical * vertical / (2 * gravity), speed * (1 - Math.cos(edge)), speed * Math.sin(edge), vertical);
    }
}
