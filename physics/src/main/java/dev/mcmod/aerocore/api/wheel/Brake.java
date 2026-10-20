package dev.mcmod.aerocore.api.wheel;

import java.util.Objects;

/**
 * A wheel brake (0.24): its torque from the demand, the lining's friction at its temperature and the brake's factor,
 * and its temperature from the heat it takes and loses.
 *
 * <ul>
 * <li>Brake factor BF (drag force over actuating force; DARCOM-P 706-358 (Limpert 1976) ch. 2): disc 2 mu; drums
 *     a mu / (1 - (mu / mu*)^2), the self-energising shoes' form (DARCOM eq. 2-23), with (a, mu*) set so that at mu 0.35
 *     they give DARCOM table 2-1's middle values: leading-trailing 2.47 (table 2.0-2.8; a 6, mu* 0.9), two-leading 3.1
 *     (2.5-3.5; a 7, mu* 0.75), duo-servo 5.3 (3.0-7.0; a 9, mu* 0.55). The torque is the design's maximum at mu 0.40
 *     scaled by BF(mu) / BF(0.40): a drum's torque swings much more with the lining's friction than a disc's.</li>
 * <li>Lining friction: 0.40 cold (passenger pads 0.35-0.45), flat to 250 C, fading linearly to 0.75 of it at 450 C and
 *     0.55 at 650 C, no lower (rule from the published fade: onset 230-320 C, resin pads 0.47 to 0.36 over a fade test).</li>
 * <li>Temperature: the disc or drum is one lump (cast iron, c 419 J/kg K, Limpert's table 3-1) taking 0.9 of the heat
 *     (Limpert's example 3-1) and losing h A (T - T_air) plus radiation (emissivity 0.55, machined cast iron) over both
 *     faces. Convection by Limpert's solid-rotor relations with Re on the rotor diameter at the vehicle's speed,
 *     h = (k / D) max(0.70 Re^0.55, 0.04 Re^0.8) (eqs. 3-40, 3-41; taking the larger is continuous and switches at
 *     Re 9.4e4 instead of the book's 2.4e5, where it jumps 27 %), twice that for vented discs (Limpert: "approximately
 *     twice"), at least 8 W/m^2 K (still air).</li>
 * </ul>
 */
public final class Brake {
    /** Brake kinds. */
    public enum Type {
        DISC(2, 0), VENTED_DISC(2, 0), DRUM_LEADING_TRAILING(6, 0.9), DRUM_TWO_LEADING(7, 0.75), DRUM_DUO_SERVO(9, 0.55);

        final double gain, lock;

        Type(double gain, double lock) { this.gain = gain; this.lock = lock; }

        /** Brake factor at a lining friction. */
        public double factor(double mu) {
            if (lock == 0) { return gain * mu; }
            double r = Math.min(mu / lock, 0.95);
            return gain * mu / (1 - r * r);
        }
    }

    /** Cold lining friction the maximum torque is quoted at. */
    public static final double COLD_MU = 0.40;
    /** Iron's specific heat, J/kg K; the share of the heat the rotor takes. */
    static final double IRON_C = 419, ROTOR_SHARE = 0.9, EMISSIVITY = 0.55, SIGMA = 5.670374e-8;
    static final double AIR_K = 0.0263, AIR_NU = 1.57e-5;

    private final Type type;
    private final double maxTorqueNm, rotorMassKg, rotorDiameterM, areaM2;

    /**
     * A brake: kind, torque at full demand with cold linings (N m), rotor (disc or drum) mass (kg) and outside diameter
     * (m), and its cooled area (both faces of a disc, the drum's surface; m^2).
     */
    public Brake(Type type, double maxTorqueNm, double rotorMassKg, double rotorDiameterM, double areaM2) {
        this.type = Objects.requireNonNull(type, "type");
        if (!(maxTorqueNm >= 0) || !(rotorMassKg > 0) || !(rotorDiameterM > 0) || !(areaM2 > 0)) { throw new IllegalArgumentException("Brake values must be positive"); }
        this.maxTorqueNm = maxTorqueNm; this.rotorMassKg = rotorMassKg; this.rotorDiameterM = rotorDiameterM; this.areaM2 = areaM2;
    }

    public Type type() { return type; }
    public double maxTorqueNm() { return maxTorqueNm; }
    public double rotorMassKg() { return rotorMassKg; }

    /** Lining friction at a rotor temperature (K). */
    public static double liningFriction(double temperatureK) {
        double c = temperatureK - 273.15;
        double f = c <= 250 ? 1 : c <= 450 ? 1 - 0.25 * (c - 250) / 200 : Math.max(0.75 - 0.20 * (c - 450) / 200, 0.55);
        return COLD_MU * f;
    }

    /** Torque capacity (N m) at a demand (0..1) and rotor temperature (K). */
    public double torqueNm(double demand, double temperatureK) {
        double d = Math.clamp(demand, 0, 1);
        if (d == 0) { return 0; }
        return d * maxTorqueNm * type.factor(liningFriction(temperatureK)) / type.factor(COLD_MU);
    }

    /** Convective coefficient at a vehicle speed (m/s), W/m^2 K. */
    public double convection(double speedMps) {
        double re = Math.abs(speedMps) * rotorDiameterM / AIR_NU;
        double h = AIR_K / rotorDiameterM * Math.max(0.70 * Math.pow(re, 0.55), 0.04 * Math.pow(re, 0.8));
        return Math.max(type == Type.VENTED_DISC ? 2 * h : h, 8);
    }

    /**
     * The rotor's temperature after a step: heat (W) taken by the brake, vehicle speed (m/s), air temperature (K),
     * step (s); cooling taken implicitly.
     */
    public double temperatureK(double temperatureK, double heatW, double speedMps, double airK, double dt) {
        double t = Math.max(temperatureK, 1);
        double radiation = EMISSIVITY * SIGMA * (t * t + airK * airK) * (t + airK);
        double loss = (convection(speedMps) + radiation) * areaM2, capacity = rotorMassKg * IRON_C;
        return (t * capacity + dt * (ROTOR_SHARE * Math.max(heatW, 0) + loss * airK)) / (capacity + dt * loss);
    }
}
