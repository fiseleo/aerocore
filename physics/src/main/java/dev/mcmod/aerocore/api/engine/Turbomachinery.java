package dev.mcmod.aerocore.api.engine;

/**
 * Design relations of gas-turbine compressors and turbines (SI units): work and stage loading, the impeller's slip,
 * the flow a face passes, rotor stress, cooled blade temperature, spool torque; fan stages, a turbine away from its
 * best speed, reduction gearboxes.
 */
public final class Turbomachinery {
    private Turbomachinery() {}

    /** Corrected mass flux (kg/(s m^2) at sea-level reference) at which a face chokes (Mach 1), and at Mach 0.7. */
    public static final double CHOKED_FLUX = 241, MACH_07_FLUX = 219;
    /** Eye diameter of a centrifugal impeller as a fraction of its tip diameter (taken as 0.6). */
    public static final double EYE_RATIO = 0.6;

    /** Compressor delivery temperature for a pressure ratio at a polytropic efficiency from an inlet temperature (air, variable properties), K. */
    public static double compressorExitTemperature(double pressureRatio, double polytropicEfficiency, double inletK) {
        Gas air = Gas.AIR;
        return air.temperatureAtEntropy(air.entropyFunction(inletK) + air.gasConstant() * Math.log(pressureRatio) / polytropicEfficiency);
    }

    /** Work per kilogram of air to compress it by a pressure ratio at a polytropic efficiency from an inlet temperature, J/kg. */
    public static double compressorWork(double pressureRatio, double polytropicEfficiency, double inletK) {
        return Gas.AIR.enthalpy(compressorExitTemperature(pressureRatio, polytropicEfficiency, inletK)) - Gas.AIR.enthalpy(inletK);
    }

    /** Blade tip speed (m/s) of a rotor of a tip diameter (m) at a speed (rpm). */
    public static double tipSpeed(double tipDiameter, double rpm) { return Math.PI * tipDiameter * rpm / 60; }

    /** Flow area of a compressor face (m^2): the annulus of a hub-to-tip ratio, or a centrifugal impeller's eye. */
    public static double faceArea(boolean centrifugal, double tipDiameter, double hubTipRatio) {
        double d = centrifugal ? EYE_RATIO * tipDiameter : tipDiameter;
        return Math.PI / 4 * d * d * (1 - hubTipRatio * hubTipRatio);
    }

    /** Stanitz's slip factor of a radial-bladed impeller with Z blades: 1 - 0.63 pi / Z. */
    public static double slipFactor(int blades) { return 1 - 0.63 * Math.PI / blades; }

    /**
     * Stage loading, work per stage over the tip speed squared. Axial compressor stages run at about 0.2-0.45 (Dixon
     * and Hall, Fluid Mechanics and Thermodynamics of Turbomachinery); turbine stages to about 2.5 (Smith's chart).
     */
    public static double stageLoading(double workJPerKg, int stages, double tipSpeed) { return workJPerKg / (stages * tipSpeed * tipSpeed); }

    /** Most work radial-bladed impellers without inlet swirl give, n sigma U^2 (Euler's equation with slip), J/kg. */
    public static double slipLimitedWork(int stages, int blades, double tipSpeed) { return stages * slipFactor(blades) * tipSpeed * tipSpeed; }

    /** Stress at the centre of a solid disc spinning at a tip speed: (3 + nu) / 8 rho U^2 (Roark, rotating discs, nu = 0.3), Pa. */
    public static double discStress(double density, double tipSpeed) { return (3 + 0.3) / 8 * density * tipSpeed * tipSpeed; }

    /**
     * Blade metal temperature with a cooling effectiveness epsilon = (T_gas - T_metal) / (T_gas - T_coolant)
     * (Saravanamuttoo et al., Gas Turbine Theory): T_gas - epsilon (T_gas - T_coolant), K.
     */
    public static double bladeMetalTemperature(double gasK, double coolantK, double effectiveness) { return gasK - effectiveness * (gasK - coolantK); }

    /** Hottest gas a blade of a service temperature takes at a cooling effectiveness and coolant temperature, K. */
    public static double maxGasTemperature(double serviceK, double coolantK, double effectiveness) {
        return (serviceK - effectiveness * coolantK) / (1 - effectiveness);
    }

    /** Net torque on a spool (N m) from turbine and compressor power at a speed (rad/s): (P_t - P_c) / omega. */
    public static double spoolTorque(double turbineW, double compressorW, double omega) { return (turbineW - compressorW) / omega; }

    /**
     * Efficiency of a turbine relative to its best at a blade-to-spouting speed ratio U / C0 given as a fraction x of
     * the best one. Below the best, the ideal impulse stage's parabola 2x - x^2 (Dixon and Hall, Fluid Mechanics and
     * Thermodynamics of Turbomachinery: efficiency against blade speed ratio): zero with the rotor held, where its
     * torque is twice the best point's, the free power turbine's stall torque. Above it a flatter fall, as reaction
     * stages have, 1 - (x - 1)^2 / 2: zero at 2.4 times the best ratio and braking beyond (a rule, not a stage
     * calculation).
     */
    public static double turbineSpeedEfficiency(double speedRatio) {
        double x = speedRatio;
        return x <= 1 ? 2 * x - x * x : 1 - 0.5 * (x - 1) * (x - 1);
    }

    /** Pressure ratio of a fan stage of a tip speed (m/s) at a stage loading (work over tip speed squared) and polytropic efficiency, from an inlet total temperature. */
    public static double fanStagePressureRatio(double tipSpeed, double loading, double polytropicEfficiency, double inletK) {
        Gas air = Gas.AIR;
        double t = air.temperatureAtEnthalpy(air.enthalpy(inletK) + loading * tipSpeed * tipSpeed);
        return Math.exp(polytropicEfficiency * (air.entropyFunction(t) - air.entropyFunction(inletK)) / air.gasConstant());
    }

    /** Stage loading a fan stage needs for a pressure ratio at a tip speed (the inverse of fanStagePressureRatio). */
    public static double fanStageLoading(double pressureRatio, double tipSpeed, double polytropicEfficiency, double inletK) {
        return compressorWork(pressureRatio, polytropicEfficiency, inletK) / (tipSpeed * tipSpeed);
    }

    /** Relative Mach number at a blade tip with no inlet swirl: the tip speed and an axial inflow of an axial Mach number, from a total temperature (K). */
    public static double relativeTipMach(double tipSpeed, double axialMach, double totalK) {
        double t = totalK / (1 + 0.2 * axialMach * axialMach), a = Math.sqrt(1.4 * Gas.AIR.gasConstant() * t), cx = axialMach * a;
        return Math.sqrt(tipSpeed * tipSpeed + cx * cx) / a;
    }

    /**
     * Axial Mach number at a face of an area (m^2) passing a corrected flow (kg/s at sea-level reference): the inverse
     * of the mass flow function sqrt(gamma / R) M (1 + 0.2 M^2)^-3 (gamma 1.4) on its subsonic branch; 1 if the flow
     * would choke the face.
     */
    public static double faceMach(double correctedFlow, double area) {
        double target = correctedFlow / area, lo = 0, hi = 1;
        if (target >= CHOKED_FLUX) { return 1; }
        for (int i = 0; i < 60; i++) {
            double m = 0.5 * (lo + hi);
            if (fluxAt(m) < target) { lo = m; } else { hi = m; }
        }
        return 0.5 * (lo + hi);
    }
    private static double fluxAt(double mach) {
        return Turbojet.P_REF / Math.sqrt(Turbojet.T_REF) * Math.sqrt(1.4 / Gas.AIR.gasConstant()) * mach * Math.pow(1 + 0.2 * mach * mach, -3);
    }

    /** Efficiency of a gear train of a number of meshes at 98.5 % each (spur and helical meshes run at 98-99 %, Dudley's Gear Handbook). */
    public static double gearboxEfficiency(int meshes) { return Math.pow(0.985, Math.max(0, meshes)); }

    /** Output torque of a reduction gearbox, N m: the input torque times the reduction ratio and the efficiency. */
    public static double gearboxOutputTorque(double inputTorqueNm, double ratio, double efficiency) { return inputTorqueNm * ratio * efficiency; }
}
