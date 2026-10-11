package dev.mcmod.aerocore.api.engine;

/**
 * Sub-models of a spark-ignition cycle: in-cylinder heat transfer, burn rate and knock.
 * <ul>
 * <li>Heat transfer: Woschni, SAE 670931 (1967), as given by Heywood (1988) eq. 12.19-12.20:
 *   h = 3.26 B^-0.2 p^0.8 T^-0.55 w^0.8 (W/m^2 K; B m, p kPa, T K, w m/s), w = C1 S_p + C2 (V_d T_r / (p_r V_r)) (p - p_mot),
 *   C1 = 6.18 during gas exchange and 2.28 otherwise (no swirl), C2 = 3.24e-3 m/(s K) during combustion and
 *   expansion, else 0.</li>
 * <li>Burn rate: Wiebe function x_b = 1 - exp(-a ((theta - theta_ST) / dtheta90)^n), a = ln 10 (90 % burned at
 *   dtheta90), with dtheta90 and n from Bonatesta, Waters and Shayler (2010) as given by Bonatesta (2013), "Premixed
 *   combustion in spark ignition engines and the influence of operating variables", InTech, doi 10.5772/55495,
 *   eq. 22 and 24: dtheta90 = 178 rho_ST^-0.34 (1 - 1.164 / sqrt(S_p)) (1 - 2.06 x_b^0.77)^-0.85
 *   (0.00033 theta_ST^2 - 0.0263 theta_ST + 1) and n = 3.46 S_p^-0.225 (1 + sqrt(theta_ST))^0.35 (1 - 1.28 x_b),
 *   with rho_ST the charge density at spark (kg/m^3), S_p the mean piston speed (m/s), x_b the burned-gas dilution and
 *   theta_ST the spark advance (crank degrees before TDC). Fitted on one 1.6 l four-valve pent-roof engine with a
 *   central plug, 1500-3500 rpm, 2-7 bar IMEP, stoichiometric gasoline, dilution 6-26 %, spark 8-35 degrees
 *   (errors within 11 %). This model adds two scalings of its own, labelled as assumptions: the burn angle scales with
 *   the flame's travel over the stroke relative to that engine (79 mm bore, 81.4 mm stroke, central plug), since
 *   turbulence and hence flame speed scale with mean piston speed (Bonatesta: u' about 0.38 S_p); and with
 *   (S_L,ref / S_L)^0.85 for other fuels and mixtures, the exponent of the correlation's own dilution term, which comes
 *   from the laminar burning velocity.</li>
 * <li>Knock: Livengood-Wu integral of 1/tau over the end gas, tau = 17.68 (ON/100)^3.402 p^-1.7 exp(3800/T) ms
 *   (p atm, T K) from Douaud and Eyzat, SAE 780080 (1978), as given by Heywood; ON is taken as the research octane
 *   number. The end gas is the unburned charge compressed isentropically from its state at spark.</li>
 * </ul>
 */
public final class Combustion {
    private Combustion() {}

    /** ln 10: the Wiebe efficiency factor that puts 90 % burned at dtheta90. */
    public static final double WIEBE_A = Math.log(10);
    /** Bonatesta's engine: half its bore (a central plug's flame travel) over its stroke. */
    public static final double REFERENCE_TRAVEL_OVER_STROKE = 0.0395 / 0.0814;

    /** Woschni heat transfer coefficient, W/(m^2 K). */
    public static double woschni(double boreM, double pressurePa, double temperatureK, double gasSpeedMps) {
        return 3.26 * Math.pow(boreM, -0.2) * Math.pow(pressurePa / 1000, 0.8) * Math.pow(temperatureK, -0.55) * Math.pow(Math.max(gasSpeedMps, 0.1), 0.8);
    }

    /** Bonatesta's 0-90 % burn angle, crank degrees (before the scalings this model adds). */
    public static double burnAngle90(double chargeDensityAtSpark, double meanPistonSpeed, double dilution, double sparkAdvanceDeg) {
        double rho = Math.max(chargeDensityAtSpark, 0.2), sp = Math.max(meanPistonSpeed, 1.5), x = Math.clamp(dilution, 0, 0.45);
        double speed = Math.max(0.05, 1 - 1.164 / Math.sqrt(sp));
        double diluted = Math.max(0.05, 1 - 2.06 * Math.pow(x, 0.77));
        double spark = 0.00033 * sparkAdvanceDeg * sparkAdvanceDeg - 0.0263 * sparkAdvanceDeg + 1;
        return 178 * Math.pow(rho, -0.34) * speed * Math.pow(diluted, -0.85) * Math.max(0.3, spark);
    }

    /** Bonatesta's Wiebe form factor n (the exponent). */
    public static double wiebeExponent(double meanPistonSpeed, double dilution, double sparkAdvanceDeg) {
        double sp = Math.max(meanPistonSpeed, 1.5), x = Math.clamp(dilution, 0, 0.45);
        return 3.46 * Math.pow(sp, -0.225) * Math.pow(1 + Math.sqrt(Math.max(0, sparkAdvanceDeg)), 0.35) * Math.max(0.3, 1 - 1.28 * x);
    }

    /** Wiebe mass fraction burned at crank angle (degrees after spark) for a burn angle and exponent. */
    public static double burned(double degreesAfterSpark, double burnAngle90, double exponent) {
        if (degreesAfterSpark <= 0) { return 0; }
        return 1 - Math.exp(-WIEBE_A * Math.pow(degreesAfterSpark / burnAngle90, exponent));
    }

    /** Douaud-Eyzat auto-ignition delay, seconds (p in Pa, T in K). */
    public static double ignitionDelay(double octane, double pressurePa, double temperatureK) {
        return 17.68e-3 * Math.pow(octane / 100, 3.402) * Math.pow(pressurePa / 101325, -1.7) * Math.exp(3800 / temperatureK);
    }
}
