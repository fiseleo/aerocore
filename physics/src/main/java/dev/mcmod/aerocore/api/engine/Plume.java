package dev.mcmod.aerocore.api.engine;

/**
 * The shape and look of an exhaust plume (0.29), for drawing it: from the nozzle exit (Mach, static pressure,
 * temperature, diameter, gas) and the ambient pressure, the fully expanded jet (Mach and diameter reached when the
 * exit pressure has expanded or compressed to ambient), the shock-cell ("diamond") spacing, the Mach disk of a
 * strongly under-expanded jet, the potential core's length, and the flame's colours by propellant family.
 * <p>
 * Formulas (display approximations, axisymmetric):
 * <ul>
 * <li>fully expanded state: isentropic from the exit total pressure to ambient; area ratio from A/A*;</li>
 * <li>shock-cell spacing L = 1.306 D_j sqrt(M_j^2 - 1) (Prandtl's and Pack's vortex-sheet result, Pack 1950;
 * Tam 1987 reviews it against measurements: within about 10 % for moderate mismatch);</li>
 * <li>Mach disk distance x / D_e = 0.67 sqrt(p_0 / p_a) for an under-expanded jet with p_e / p_a over about 2
 * (Ashkenas and Sherman 1966, sonic nozzles; used here with the exit total pressure);</li>
 * <li>potential core L_c / D_j = 4.2 + 1.1 M_j^2 (Lau, Morris and Fisher 1979, unheated jets), shortened for hot
 * jets by (T_j / T_a)^-0.2 (a fit of the trend: the core of a heated jet mixes out sooner).</li>
 * </ul>
 * A scramjet's two-dimensional nozzle is drawn with the same formulas on its equivalent diameter.
 */
public final class Plume {
    private Plume() {}

    /** What burns in the plume, which sets its colour and smoke. */
    public enum Family { HYDROGEN, HYDROCARBON, ALUMINISED_SOLID, SUGAR_SOLID, METHANE }

    /** The nozzle exit and the air it leaves into. */
    public record Exit(double mach, double staticPressurePa, double staticTemperatureK, double diameterM, double gamma, double ambientPressurePa,
                       double ambientTemperatureK) {}

    /**
     * The plume's shape: the fully expanded jet's Mach and diameter (m), the spacing between shock cells (m, 0 for a
     * subsonic jet), how strong the cells look (0-1, from the pressure mismatch), the Mach disk's distance from the exit
     * (m, NaN without one) and diameter (m), and the potential core's length (m).
     */
    public record Shape(double jetMach, double jetDiameterM, double cellSpacingM, double cellStrength, double machDiskM, double machDiskDiameterM,
                        double coreLengthM) {
        public boolean hasCells() { return cellSpacingM > 0 && cellStrength > 0.02; }
        public boolean hasMachDisk() { return Double.isFinite(machDiskM); }
    }

    /** The plume's shape from a nozzle exit. */
    public static Shape shape(Exit e) {
        double g = e.gamma(), me = Math.max(0, e.mach());
        double pe = Math.max(1, e.staticPressurePa()), pa = Math.max(1, e.ambientPressurePa());
        double p0 = pe * Math.pow(1 + 0.5 * (g - 1) * me * me, g / (g - 1));
        double ratio = p0 / pa;
        // fully expanded Mach: from the total-to-ambient pressure ratio (subsonic if it is below the critical ratio)
        double mj2 = 2 / (g - 1) * (Math.pow(Math.max(ratio, 1), (g - 1) / g) - 1);
        double mj = Math.sqrt(Math.max(0, mj2));
        double dj = e.diameterM() * Math.sqrt(areaRatio(Math.max(mj, 1e-3), g) / areaRatio(Math.max(me, 1e-3), g));
        if (me < 1e-3 || mj < 1e-3) { dj = e.diameterM(); }
        double spacing = mj > 1 ? 1.306 * dj * Math.sqrt(mj * mj - 1) : 0;
        double mismatch = Math.abs(pe / pa - 1);
        // a perfectly expanded supersonic jet still shows faint cells (nozzle lip waves); mismatch makes them strong
        double strength = mj > 1 ? Math.min(1, 0.08 + 0.9 * mismatch / (1 + mismatch)) : 0;
        double disk = Double.NaN, diskD = 0;
        if (pe / pa > 2 && me >= 1 - 1e-9) {
            disk = 0.67 * e.diameterM() * Math.sqrt(ratio);
            diskD = Math.min(0.9 * dj, 0.25 * e.diameterM() * Math.sqrt(pe / pa - 1));   // a display estimate of its size
        }
        double tj = e.staticTemperatureK() * (1 + 0.5 * (g - 1) * me * me) / (1 + 0.5 * (g - 1) * mj * mj);
        double core = dj * (4.2 + 1.1 * mj * mj) * Math.pow(Math.max(1, tj / Math.max(1, e.ambientTemperatureK())), -0.2);
        return new Shape(mj, dj, spacing, strength, disk, diskD, core);
    }

    /** A / A* of isentropic flow at a Mach number. */
    static double areaRatio(double m, double g) {
        return 1 / m * Math.pow(2 / (g + 1) * (1 + 0.5 * (g - 1) * m * m), (g + 1) / (2 * (g - 1)));
    }

    /**
     * How a plume looks: the colour of its hot core and of its outer envelope (ARGB, alpha their strength), how far
     * it shows in exit diameters at full power, the colour of the shock diamonds, and its smoke (0-1 how much, ARGB).
     */
    public record Look(int core, int envelope, double visibleDiameters, int diamonds, double smoke, int smokeColour) {}

    /**
     * The look of a plume of a propellant family at a gas temperature (K, the hottest in the plume: the combustor or
     * the afterburner): soot glows as a black body (Blackbody), hydrogen burns nearly invisibly (its OH band is in the
     * ultraviolet; a faint blue), a lean hydrocarbon flame is blue and short, aluminium oxide particles glow white,
     * potassium in sugar propellant tints the flame violet and its carbonate makes white smoke.
     */
    public static Look look(Family family, double kelvin, boolean afterburning) {
        double t = Math.max(600, kelvin);
        return switch (family) {
            case HYDROGEN -> new Look(0x553a6cff, 0x18406aff, 2.5, 0x40c8d8ff, 0, 0);
            case HYDROCARBON -> afterburning || t > 1700
                    ? new Look(withAlpha(Blackbody.argb(Math.min(t, 2300), 255), 0xd0), withAlpha(Blackbody.argb(Math.min(t, 2300) - 350, 255), 0x70), 6,
                            withAlpha(Blackbody.argb(Math.min(t + 300, 2600), 255), 0xe0), 0.05, 0x60404040)
                    : new Look(0x404a66ff, 0x14403050, 1.5, 0x20a0b0ff, 0.02, 0x30303030);
            case ALUMINISED_SOLID -> new Look(withAlpha(Blackbody.argb(3200, 255), 0xff), withAlpha(Blackbody.argb(2600, 255), 0xb0), 10,
                    withAlpha(Blackbody.argb(3600, 255), 0xf0), 0.9, 0xd0f0f0f0);
            case SUGAR_SOLID -> new Look(0xe0ffb0e0, 0x90ff8ab0, 8, 0xe0ffd0f0, 0.8, 0xc0f4f4f4);
            case METHANE -> new Look(withAlpha(Blackbody.argb(1900, 255), 0xd0), withAlpha(Blackbody.argb(1500, 255), 0x80), 5, 0, 0.35, 0x80303030);
        };
    }

    private static int withAlpha(int argb, int alpha) { return (alpha & 255) << 24 | (argb & 0xffffff); }
}
