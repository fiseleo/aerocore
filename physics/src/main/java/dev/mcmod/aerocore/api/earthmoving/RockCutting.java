package dev.mcmod.aerocore.api.earthmoving;

/**
 * Cutting rock and other hard materials (rock, ore, concrete, asphalt): specific energy, picks, disc cutters, rotary
 * cutters and hammers. Formulas only; the tools' sizes and the efficiencies of a given machine are the caller's.
 *
 * <ul>
 * <li>Specific energy SE, J per m^3 removed (= Pa): Teale (1965, The concept of specific energy in rock drilling,
 *     Int. J. Rock Mech. Min. Sci. 2) found the least SE of efficient cutting close to the rock's uniaxial compressive
 *     strength. Real cutting and drilling run at a mechanical efficiency of about 0.35 (Dupriest and Koederitz 2005,
 *     SPE/IADC 92194), SE = UCS / 0.35; blunt tools and fine grinding more. Force of an edge = SE x cut cross-section,
 *     power = SE x removal rate.</li>
 * <li>Point-attack picks (roadheaders, cold planer drums, trenchers): Evans (1984, A theory of the cutting force for
 *     point-attack picks, Int. J. Min. Eng. 2) F_c = 16 pi sigma_t^2 d^2 / (sigma_c cos^2(theta / 2)), the peak
 *     cutting force at depth d for a cone of tip angle theta; for rock with sigma_c / sigma_t about 8 to 15.</li>
 * <li>Disc cutters (TBMs): the Colorado School of Mines model (Rostami and Ozdemir 1993, RETC; Rostami 1997, PhD
 *     thesis, CSM): crushed-zone pressure P' = C cbrt(S sigma_c^2 sigma_t / (phi sqrt(R T))), C = 2.12, contact angle
 *     phi = acos((R - p) / R), total force F_t = T R phi P' / (1 + psi) with the pressure distribution exponent psi
 *     (about 0.2 for sharp, to -0.2 for blunt tips), normal force F_t cos(phi / 2), rolling force F_t sin(phi / 2);
 *     disc radius R, tip width T, spacing S, penetration p per revolution. Cutterhead torque about 0.3 D N F_r for N
 *     discs on a head of diameter D (the same papers).</li>
 * <li>Rock mass: Hoek, Carranza-Torres and Corkum (2002, Hoek-Brown failure criterion, 2002 edition): the jointed mass
 *     has the uniaxial strength sigma_ci s^a, s = exp((GSI - 100) / (9 - 3 D)), a = 1/2 + (exp(-GSI / 15) -
 *     exp(-20 / 3)) / 6 (D the blast damage, 0 undisturbed to 1).</li>
 * </ul>
 */
public final class RockCutting {
    private RockCutting() {}

    /** Mechanical efficiency of real cutting and drilling (Dupriest and Koederitz 2005): SE about UCS / 0.35. Teale's limit is 1. */
    public static final double TYPICAL_EFFICIENCY = 0.35;

    /** The CSM model's dimensionless constant C (Rostami 1997). */
    public static final double CSM_C = 2.12;

    /** Specific energy (J/m^3 = Pa) of cutting a material of uniaxial compressive strength (Pa) at a mechanical efficiency (0..1]: UCS / efficiency (Teale). */
    public static double specificEnergyPa(double compressivePa, double efficiency) {
        if (!(compressivePa >= 0) || !Double.isFinite(compressivePa) || !(efficiency > 0 && efficiency <= 1)) {
            throw new IllegalArgumentException("Strength not negative, efficiency within (0, 1]");
        }
        return compressivePa / efficiency;
    }

    /**
     * An edge cutting rock by specific energy (Pa): horizontal force SE x depth x width (independent of speed: the power
     * goes as the speed); the resultant leans at the soil-metal friction delta from the rake face's normal, as a
     * cut's does (lift = -H cot(alpha + delta)), with the rake held as in {@link SoilCutting} for the friction phi
     * (rad). A finite, large resistance for strong rock: what makes an actuator stall.
     */
    public static Cut edge(double specificEnergyPa, double rakeRad, double wallFrictionRad, double frictionRad, double depthM, double widthM, double speedMps) {
        if (!(specificEnergyPa >= 0) || !Double.isFinite(specificEnergyPa) || !(depthM >= 0) || !(widthM >= 0) || !Double.isFinite(depthM)
                || !Double.isFinite(widthM) || !Double.isFinite(speedMps)) {
            throw new IllegalArgumentException("Specific energy, depth and width finite and not negative, speed finite");
        }
        if (!(frictionRad >= 0 && frictionRad < Math.toRadians(80)) || !(wallFrictionRad >= 0 && wallFrictionRad < Math.PI / 2) || !(rakeRad >= 0 && rakeRad <= Math.PI)) {
            throw new IllegalArgumentException("Friction within [0, 80) deg, wall friction within [0, 90) deg, rake within [0, 180] deg");
        }
        double delta = Math.min(wallFrictionRad, frictionRad);
        double a = Math.clamp(rakeRad, SoilCutting.MIN_RAKE_RAD, Math.PI - delta - frictionRad - SoilCutting.CLOSURE_RAD);
        double h = specificEnergyPa * depthM * widthM;
        return new Cut(h, -h * Math.cos(a + delta) / Math.sin(a + delta), 0, depthM * widthM * Math.abs(speedMps), widthM);
    }

    /**
     * Evans' peak cutting force (N) of a point-attack pick at depth of cut d (m) with a tip cone angle theta (rad,
     * 0..180 deg) in rock of tensile and compressive strength (Pa): 16 pi sigma_t^2 d^2 / (sigma_c cos^2(theta / 2)).
     */
    public static double pickForceN(double tensilePa, double compressivePa, double depthM, double tipAngleRad) {
        if (!(tensilePa >= 0) || !(compressivePa > 0) || !(depthM >= 0) || !(tipAngleRad > 0 && tipAngleRad < Math.toRadians(170))
                || !Double.isFinite(tensilePa) || !Double.isFinite(compressivePa) || !Double.isFinite(depthM)) {
            throw new IllegalArgumentException("Strengths and depth finite, compressive positive, tip angle within (0, 170) deg");
        }
        double cos = Math.cos(tipAngleRad / 2);
        return 16 * Math.PI * tensilePa * tensilePa * depthM * depthM / (compressivePa * cos * cos);
    }

    /** One disc cutter's forces (N) by the CSM model, and its contact angle (rad). */
    public record Disc(double normalN, double rollingN, double totalN, double contactAngleRad) {
        /** Rolling coefficient: rolling over normal force (tan(phi / 2)). */
        public double rollingCoefficient() { return normalN > 0 ? rollingN / normalN : 0; }
    }

    /**
     * A disc cutter by the CSM model (class note): rock compressive and Brazilian tensile strength (Pa), disc diameter,
     * tip width, spacing and penetration per revolution (m; penetration up to the disc radius), pressure distribution
     * exponent psi (-0.2..0.2; 0 uniform). Written as F_t = T R C cbrt(S sigma_c^2 sigma_t / sqrt(R T)) phi^(2/3) / (1 + psi),
     * so zero penetration gives zero force.
     */
    public static Disc disc(double compressivePa, double tensilePa, double diameterM, double tipWidthM, double spacingM, double penetrationM, double psi) {
        if (!(compressivePa >= 0) || !(tensilePa >= 0) || !(diameterM > 0) || !(tipWidthM > 0) || !(spacingM > 0) || !(penetrationM >= 0)
                || !(penetrationM <= diameterM / 2) || !(psi > -0.5 && psi < 1) || !Double.isFinite(compressivePa) || !Double.isFinite(tensilePa)
                || !Double.isFinite(diameterM) || !Double.isFinite(tipWidthM) || !Double.isFinite(spacingM)) {
            throw new IllegalArgumentException("Strengths not negative, sizes positive, penetration within [0, R], psi within (-0.5, 1)");
        }
        double r = diameterM / 2;
        double phi = Math.acos((r - penetrationM) / r);
        double total = tipWidthM * r * CSM_C * Math.cbrt(spacingM * compressivePa * compressivePa * tensilePa / Math.sqrt(r * tipWidthM))
                * Math.pow(phi, 2.0 / 3) / (1 + psi);
        return new Disc(total * Math.cos(phi / 2), total * Math.sin(phi / 2), total, phi);
    }

    /** Cutterhead torque (N m) of discs (count) each rolling against a force (N) on a head of diameter D (m): 0.3 D N F_r (Rostami and Ozdemir 1993). */
    public static double cutterheadTorqueNm(double discs, double rollingN, double headDiameterM) {
        if (!(discs >= 0) || !(rollingN >= 0) || !(headDiameterM >= 0)) { throw new IllegalArgumentException("Count, force and diameter not negative"); }
        return 0.3 * headDiameterM * discs * rollingN;
    }

    /** A rotary cutter's power (W), torque (N m), tangential force at the rim (N) and removal rate (bank m^3/s). */
    public record Rotary(double powerW, double torqueNm, double rimForceN, double removalM3PerS) {}

    /**
     * A rotary cutter (milling drum, cutterhead, bucket wheel, trencher chain, auger) of radius R (m) turning at omega
     * (rad/s) while it advances at v (m/s) into a cut of width w and depth d (m), in a material of specific energy SE
     * (Pa): removal Q = w d v, power SE Q, rim force SE Q / max(omega R, v) (when the rim is slower than the advance the
     * cutter ploughs like a blade of the cut's cross-section, SE w d), torque rim force x R. Signs ignored.
     */
    public static Rotary rotary(double specificEnergyPa, double widthM, double depthM, double advanceMps, double radiusM, double omegaRadPerS) {
        if (!(specificEnergyPa >= 0) || !(widthM >= 0) || !(depthM >= 0) || !(radiusM >= 0) || !Double.isFinite(specificEnergyPa)
                || !Double.isFinite(widthM) || !Double.isFinite(depthM) || !Double.isFinite(radiusM) || !Double.isFinite(advanceMps)
                || !Double.isFinite(omegaRadPerS)) {
            throw new IllegalArgumentException("Specific energy and sizes finite and not negative, speeds finite");
        }
        double v = Math.abs(advanceMps), q = widthM * depthM * v, power = specificEnergyPa * q;
        double rim = power > 0 ? power / Math.max(Math.abs(omegaRadPerS) * radiusM, v) : 0;
        return new Rotary(power, rim * radiusM, rim, q);
    }

    /** Bank volume (m^3) broken by one hammer blow of an impact energy (J) in a material of specific energy (Pa): E / SE. */
    public static double blowM3(double impactJ, double specificEnergyPa) {
        if (!(impactJ >= 0) || !(specificEnergyPa > 0) || !Double.isFinite(impactJ) || !Double.isFinite(specificEnergyPa)) {
            throw new IllegalArgumentException("Energy finite and not negative, specific energy positive");
        }
        return impactJ / specificEnergyPa;
    }

    /**
     * Hoek-Brown factor of a jointed rock mass's uniaxial strength over the intact rock's: s^a (class note) for a
     * geological strength index (0..100) and blast damage D (0..1). 1 at GSI 100 and D 0; about 0.25 at GSI 75, 0.06
     * at 50.
     */
    public static double rockMassFactor(double gsi, double disturbance) {
        if (!(gsi >= 0 && gsi <= 100) || !(disturbance >= 0 && disturbance <= 1)) { throw new IllegalArgumentException("GSI within [0, 100], D within [0, 1]"); }
        double s = Math.exp((gsi - 100) / (9 - 3 * disturbance));
        double a = 0.5 + (Math.exp(-gsi / 15) - Math.exp(-20.0 / 3)) / 6;
        return Math.pow(s, a);
    }
}
