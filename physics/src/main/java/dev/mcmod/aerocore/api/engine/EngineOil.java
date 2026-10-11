package dev.mcmod.aerocore.api.engine;

import java.util.List;
import java.util.Objects;

/**
 * Engine oil viscosity against temperature by the Vogel equation nu_0 = k exp(theta1 / (theta2 + T)) (cSt, T in
 * degrees C), times the high-shear ratio mu_inf / mu_0 (multigrade oils thin under shear), with the constants of
 * Sandoval (2002), "An Improved Friction Model for Spark Ignition Engines", MIT, appendix A.2 (after Taylor et al.,
 * SAE 941981). The friction model scales its hydrodynamic terms by sqrt(nu / nu_ref), nu_ref being the reference
 * 10W-30 at 90 C that calibrated Patton, Nitschke and Heywood's model.
 */
public record EngineOil(String grade, double kCst, double theta1C, double theta2C, double shearRatio) {
    public EngineOil {
        Objects.requireNonNull(grade, "grade");
        if (!(kCst > 0) || !(theta1C > 0) || !(theta2C > 0) || !(shearRatio > 0)) { throw new IllegalArgumentException("Oil constants must be positive"); }
    }

    public static final EngineOil W0_40 = new EngineOil("0W-40", 0.01341, 1986.4, 189.7, 0.67);
    public static final EngineOil W5_20 = new EngineOil("5W-20", 0.04576, 1224, 134.1, 0.94);
    public static final EngineOil W5_40 = new EngineOil("5W-40", 0.15, 1018.74, 125.91, 0.8);
    public static final EngineOil W10_30 = new EngineOil("10W-30", 0.1403, 869.72, 104.4, 0.76);
    public static final EngineOil W10_50 = new EngineOil("10W-50", 0.0352, 1658.88, 163.54, 0.49);
    public static final EngineOil W15_40 = new EngineOil("15W-40", 0.1223, 933.46, 103.89, 0.9);
    public static final EngineOil W20_50 = new EngineOil("20W-50", 0.0639, 1255.46, 117.7, 0.84);
    public static final EngineOil SAE10 = new EngineOil("SAE 10", 0.0258, 1345.42, 144.58, 1);
    public static final EngineOil SAE30 = new EngineOil("SAE 30", 0.0246, 1432.29, 132.94, 1);
    public static final EngineOil SAE50 = new EngineOil("SAE 50", 0.0384, 1349.94, 115.16, 1);
    public static final List<EngineOil> GRADES = List.of(W0_40, W5_20, W5_40, W10_30, W10_50, W15_40, W20_50, SAE10, SAE30, SAE50);

    /** The reference viscosity of the friction model: 10W-30 at 90 C, high shear, cSt. */
    public static final double REFERENCE_CST = W10_30.viscosity(90);

    public static EngineOil grade(String name) {
        for (EngineOil oil : GRADES) { if (oil.grade.equalsIgnoreCase(name)) { return oil; } }
        throw new IllegalArgumentException("Unknown oil grade " + name);
    }

    /** High-shear kinematic viscosity at an oil temperature (degrees C), cSt. */
    public double viscosity(double temperatureC) {
        return kCst * Math.exp(theta1C / (theta2C + Math.max(temperatureC, -theta2C + 5))) * shearRatio;
    }

    /** sqrt(nu / nu_ref), the factor on the hydrodynamic friction terms. */
    public double frictionScaling(double temperatureC) { return Math.sqrt(viscosity(temperatureC) / REFERENCE_CST); }
}
