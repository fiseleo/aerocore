package dev.mcmod.aerocore.api.engine;

/**
 * Sub-models of the engine cycle: in-cylinder heat transfer, burn rate and knock; for compression ignition (0.26)
 * ignition delay and Watson's diesel burn (see the methods).
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

    // ---------------------------------------------------------------- compression ignition (0.26)

    /** Watson's normalizing combustion duration, crank degrees. */
    public static final double DIESEL_DURATION_DEG = 125;

    /**
     * Ignition delay of a diesel spray, crank degrees: Hardenberg and Hase, SAE 790493 (1979), as given by Heywood
     * (1988) eq. 10.37: (0.36 + 0.22 S_p) exp[E_A (1 / (R T) - 1 / 17190) (21.2 / (p - 12.4))^0.63], E_A = 618840 /
     * (CN + 25) J/mol, with the charge's temperature (K) and pressure (bar) at TDC (polytropic compression, n = 1.35).
     * Below 13 bar the correlation's pressure term blows up: the charge does not light.
     */
    public static double dieselIgnitionDelayDeg(double meanPistonSpeed, double temperatureK, double pressurePa, double cetane) {
        double bar = pressurePa / 1e5;
        if (bar <= 13) { return Double.POSITIVE_INFINITY; }
        double ea = 618840 / (Math.max(cetane, 1) + 25);
        double arg = ea * (1 / (8.3143 * temperatureK) - 1 / 17190.0) * Math.pow(21.2 / (bar - 12.4), 0.63);
        return (0.36 + 0.22 * meanPistonSpeed) * Math.exp(arg);
    }

    /**
     * The share of a diesel's fuel burned premixed (Watson, Pilley and Marzouk, SAE 800029, 1980, as given by
     * Heywood section 14.4.3): 1 - 0.926 phi^0.37 / tau_id^0.26 with tau_id the ignition delay in ms; a longer delay
     * lets more fuel mix before it lights. Kept within 0.02-0.95.
     */
    public static double premixedFraction(double phi, double delayMs) {
        return Math.clamp(1 - 0.926 * Math.pow(Math.max(phi, 1e-3), 0.37) / Math.pow(Math.max(delayMs, 1e-3), 0.26), 0.02, 0.95);
    }

    /**
     * Watson's diesel burn, the fraction burned at tau (time since ignition over the 125-degree duration): a premixed
     * part 1 - (1 - tau^C1)^C2 and a diffusion part 1 - exp(-C3 tau^C4), weighted by the premixed fraction; C1 = 2 +
     * 1.25e-8 (tau_id N)^2.4 (ms, rpm), C2 = 5000, C3 = 14.2 / phi^0.644, C4 = 0.79 C3^0.25.
     */
    public record DieselBurn(double premixed, double c1, double c3, double c4) {
        public static final double C2 = 5000;

        public static DieselBurn of(double phi, double delayMs, double rpm) {
            double c3 = 14.2 / Math.pow(Math.max(phi, 1e-3), 0.644);
            return new DieselBurn(premixedFraction(phi, delayMs), 2 + 1.25e-8 * Math.pow(delayMs * rpm, 2.4), c3, 0.79 * Math.pow(c3, 0.25));
        }

        /** Fraction burned at tau. */
        public double burned(double tau) {
            if (tau <= 0) { return 0; }
            double t = Math.min(tau, 1);
            return premixed * (1 - Math.pow(1 - Math.pow(t, c1), C2)) + (1 - premixed) * (1 - Math.exp(-c3 * Math.pow(t, c4)));
        }

        /** d(burned)/d(tau). */
        public double rate(double tau) {
            if (tau <= 0 || tau >= 1) { return 0; }
            double premixedRate = C2 * Math.pow(1 - Math.pow(tau, c1), C2 - 1) * c1 * Math.pow(tau, c1 - 1);
            double diffusionRate = c3 * c4 * Math.pow(tau, c4 - 1) * Math.exp(-c3 * Math.pow(tau, c4));
            return premixed * premixedRate + (1 - premixed) * diffusionRate;
        }
    }
}
