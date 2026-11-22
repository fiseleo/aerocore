package dev.mcmod.aerocore.api.lifting;

import dev.mcmod.aerocore.api.Atmosphere;

/**
 * Wind on cranes and their hook loads, after ISO 4302:2016 (Cranes, wind load assessment): the design wind pressure,
 * the in-service design wind speeds, the wind on a suspended load and how far it leans the rope. Constants with their
 * sources; whether a crane must stop or weathervane is the caller's rule.
 *
 * <p>Out-of-service (storm) wind depends on the region and the height and is the caller's (ISO 4302 section 6;
 * EN 14439 for tower cranes refers to regional storm wind maps).
 */
public final class Wind {
    private Wind() {}

    private static final double G = Atmosphere.GRAVITY_MPS2;

    /** In-service design wind speed (m/s) of cranes easily secured and meant for light winds only, such as booms that are readily lowered: 14 (ISO 4302 Table 2, 125 N/m^2). */
    public static final double IN_SERVICE_LIGHT_MPS = 14;
    /**
     * In-service design wind speed (m/s) of all normal cranes installed in the open: 20 (ISO 4302 Table 2, 250 N/m^2).
     * Tower cranes to EN 14439 are normally designed for the same 20 m/s (72 km/h) in service (Construction Plant-hire
     * Association, Crane Interest Group technical information note TIN 101).
     */
    public static final double IN_SERVICE_MPS = 20;
    /** In-service design wind speed (m/s) of cranes in process applications that must keep working in high winds: 28.5 (ISO 4302 Table 2, 500 N/m^2). */
    public static final double IN_SERVICE_PROCESS_MPS = 28.5;
    /**
     * The 3 s gust at the boom head (m/s) for which mobile crane load charts generally hold, with the load's wind area
     * at 1.2 m^2 per tonne (c_w x A, the EN 13000 assumption): 9 (Liebherr, customer magazine "Safe to use in high
     * winds"; some charts give more).
     */
    public static final double MOBILE_CHART_GUST_MPS = 9;
    /** Shape coefficient c_H of a hoist load of unknown form: 2.4 (ISO 4302 5.2). */
    public static final double LOAD_SHAPE = 2.4;
    /** Wind area of a hoist load of unknown form per kg of load (m^2/kg): 0.0005, so c_H A_H = 1.2 m^2 per tonne (ISO 4302 5.2). */
    public static final double LOAD_AREA_M2_PER_KG = 0.0005;

    /** Design wind pressure (Pa): p = 0.625 v^2 (ISO 4302 eq. 2, which rounds 0.5 rho v^2 with its rho = 1.225 kg/m^3 up to 0.625). */
    public static double pressurePa(double windMps) {
        double v = Check.finite(windMps, "Wind speed");
        return 0.625 * v * v;
    }

    /** Wind force (N) on a hoist load: F = c_H A_H p (ISO 4302 eq. 3), shape coefficient and area (m^2) the load's. */
    public static double loadForceN(double shapeCoefficient, double areaM2, double windMps) {
        Check.nonNegative(shapeCoefficient, "Shape coefficient"); Check.nonNegative(areaM2, "Area");
        return shapeCoefficient * areaM2 * pressurePa(windMps);
    }

    /** Wind force (N) on a hoist load (kg) of unknown form: LOAD_SHAPE x LOAD_AREA_M2_PER_KG x m x p. */
    public static double loadForceN(double loadKg, double windMps) {
        return loadForceN(LOAD_SHAPE, LOAD_AREA_M2_PER_KG * Check.nonNegative(loadKg, "Load"), windMps);
    }

    /** How far a steady side force (N) leans a freely hanging load (kg) on its rope (rad): atan(F / (m g)), the rope's own weight and wind left out. */
    public static double swingAngleRad(double forceN, double loadKg) {
        Check.finite(forceN, "Force"); Check.positive(loadKg, "Load");
        return Math.atan(forceN / (loadKg * G));
    }

    /**
     * The gust (m/s) a chart rated for ratedMps at 1.2 m^2 per tonne allows for a load (kg) of a larger wind area: the
     * same force, v = v_rated sqrt(1.2 m^2/t x m / (c_w A)) (the conversion Liebherr's manuals describe; ISO 4302 5.2
     * names reducing the wind speed for loads above a stated area). Never more than the rated gust.
     */
    public static double permissibleMps(double ratedMps, double loadKg, double shapeCoefficient, double areaM2) {
        Check.positive(ratedMps, "Rated wind speed"); Check.positive(loadKg, "Load");
        Check.positive(shapeCoefficient, "Shape coefficient"); Check.positive(areaM2, "Area");
        double ratio = LOAD_SHAPE * LOAD_AREA_M2_PER_KG * loadKg / (shapeCoefficient * areaM2);
        return ratedMps * Math.sqrt(Math.min(1, ratio));
    }
}
