package dev.mcmod.aerocore.api.earthmoving;

/**
 * One call per tool per step: the resistance a cutting edge or tooth meets in a material, by the model that fits it.
 * Soils (no compressive strength) fail as a wedge ({@link SoilCutting}); rock-like materials are cut by specific energy
 * at the typical mechanical efficiency ({@link RockCutting#edge}, SE = UCS / {@link RockCutting#TYPICAL_EFFICIENCY}).
 * Jointed rock is weaker: pass {@link Material#jointed}. Each call costs about 60 evaluations of a few trigonometric
 * functions for soil and a handful of operations for rock; never NaN for valid input, continuous in every number.
 */
public final class Digging {
    private Digging() {}

    /**
     * A wide edge (bucket lip, dozer blade, grader moldboard, scraper cutting edge): rake (rad, 0..180 deg from the
     * horizontal in the direction of travel), depth and width (m), speed (m/s), surcharge on the surface ahead (Pa;
     * {@link Blade#surchargePa} for a dozer's prism; it matters only in soil). For a blade set at an angle in plan, the
     * caller passes the width across the travel.
     */
    public static Cut edge(Material m, double rakeRad, double depthM, double widthM, double speedMps, double surchargePa) {
        if (!(surchargePa >= 0) || !Double.isFinite(surchargePa)) { throw new IllegalArgumentException("Surcharge finite and not negative"); }
        if (m.isRock()) {
            return RockCutting.edge(RockCutting.specificEnergyPa(m.compressivePa(), RockCutting.TYPICAL_EFFICIENCY), rakeRad, m.wallFrictionRad(),
                    m.frictionRad(), depthM, widthM, speedMps);
        }
        return SoilCutting.wide(m, rakeRad, depthM, widthM, speedMps, surchargePa);
    }

    /**
     * A narrow tooth or tine (ripper shank, bucket tooth, trencher tooth): as {@link #edge} with the side crescents in
     * soil (McKyes and Ali), the tooth's own cross-section in rock (no surcharge).
     */
    public static Cut tooth(Material m, double rakeRad, double depthM, double widthM, double speedMps) {
        if (m.isRock()) {
            return RockCutting.edge(RockCutting.specificEnergyPa(m.compressivePa(), RockCutting.TYPICAL_EFFICIENCY), rakeRad, m.wallFrictionRad(),
                    m.frictionRad(), depthM, widthM, speedMps);
        }
        return SoilCutting.narrow(m, rakeRad, depthM, widthM, speedMps, 0);
    }

    /**
     * Specific energy (J per bank m^3 = Pa) of cutting a material with a wide edge at a rake, depth (m) and speed (m/s):
     * the horizontal force per unit cut cross-section, for rotary cutters ({@link RockCutting#rotary}) in soil as well
     * as rock. Depths below 1 mm are taken at 1 mm (the limit as the depth goes to zero is finite).
     */
    public static double specificEnergyPa(Material m, double rakeRad, double depthM, double speedMps) {
        if (!(depthM >= 0) || !Double.isFinite(depthM)) { throw new IllegalArgumentException("Depth finite and not negative"); }
        double d = Math.max(depthM, 1e-3);
        return edge(m, rakeRad, d, 1, speedMps, 0).horizontalN() / d;
    }
}
