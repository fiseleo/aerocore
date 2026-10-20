package dev.mcmod.aerocore.api.manufacture;

/**
 * Industrial gases (0.35): assist gas through a cutting nozzle, what a cylinder holds and the work to fill it
 * (perfect gas with a compressibility factor; Anderson, Modern Compressible Flow, ch. 5).
 *
 * <ul>
 * <li>A nozzle fed from p0, T0 above about 1.9 times the outside pressure is choked: it passes
 *     m = C_d A p0 sqrt(gamma / (R T0)) (2 / (gamma + 1))^((gamma + 1) / (2 (gamma - 1))) whatever the outside pressure.</li>
 * <li>A cylinder of volume V at p and T holds p V / (Z R T) (Z the compressibility factor, near 1 at 200 bar).</li>
 * <li>Compressing a mass isothermally from p1 to p2 takes at least m R T ln(p2 / p1).</li>
 * </ul>
 */
public final class Gases {
    private Gases() {}

    /** A gas: specific gas constant (J/kg K), ratio of specific heats and molar mass (kg/mol). */
    public enum Gas {
        AIR(287.05, 1.4, 0.028965), NITROGEN(296.80, 1.4, 0.028014), OXYGEN(259.84, 1.395, 0.031999), ARGON(208.13, 1.667, 0.039948);

        public final double r, gamma, molarMass;

        Gas(double r, double gamma, double molarMass) { this.r = r; this.gamma = gamma; this.molarMass = molarMass; }
    }

    /** Choked mass flow through a nozzle of a diameter (m) from a supply pressure (Pa abs) and temperature (K), discharge coefficient, kg/s. */
    public static double chokedKgPerS(Gas g, double nozzleM, double supplyPa, double supplyK, double dischargeCoefficient) {
        if (!(nozzleM > 0) || !(supplyPa > 0) || !(supplyK > 0) || !(dischargeCoefficient > 0)) { throw new IllegalArgumentException("Sizes, pressures and temperatures positive"); }
        double y = g.gamma, area = Math.PI / 4 * nozzleM * nozzleM;
        return dischargeCoefficient * area * supplyPa * Math.sqrt(y / (g.r * supplyK)) * Math.pow(2 / (y + 1), (y + 1) / (2 * (y - 1)));
    }

    /** Whether a nozzle fed at a pressure is choked into an outside pressure (Pa). */
    public static boolean choked(Gas g, double supplyPa, double outsidePa) {
        return outsidePa / supplyPa <= Math.pow(2 / (g.gamma + 1), g.gamma / (g.gamma - 1));
    }

    /** What a cylinder of a volume (m^3) holds at a pressure (Pa abs) and temperature (K) with a compressibility factor, kg. */
    public static double cylinderKg(Gas g, double volumeM3, double pressurePa, double temperatureK, double compressibility) {
        if (!(volumeM3 > 0) || !(pressurePa > 0) || !(temperatureK > 0) || !(compressibility > 0)) { throw new IllegalArgumentException("Positive"); }
        return pressurePa * volumeM3 / (compressibility * g.r * temperatureK);
    }

    /** Least work to compress a mass (kg) isothermally at a temperature (K) between two pressures (Pa), J. */
    public static double isothermalJ(Gas g, double kg, double temperatureK, double fromPa, double toPa) {
        if (!(fromPa > 0) || !(toPa > 0) || !(temperatureK > 0)) { throw new IllegalArgumentException("Positive"); }
        return Math.max(kg, 0) * g.r * temperatureK * Math.log(toPa / fromPa);
    }
}
