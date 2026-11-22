package dev.mcmod.aerocore.api.hydraulic;

/**
 * Single-rod double-acting hydraulic cylinders: bore and rod diameters (m, 0 &lt; rod &lt; bore), stroke (m) and how many
 * act together side by side (a boom's pair); areas and flows are for all of them together. Velocity and force are
 * positive extending; the head side is the full bore, the rod side the annulus.
 *
 * <p>Ideal relations: v = Q / A (flow into the head side to extend, into the rod side to retract) and
 * F = p_head A_head - p_rod A_rod. Seal friction is taken in proportion to the pressures on the seals,
 * (1 - SEAL_EFFICIENCY)(|p_head| A_head + |p_rod| A_rod), opposing the motion and ramped in over STANDSTILL_MPS, so
 * at standstill the ideal force acts and a push with the rod side at tank gives SEAL_EFFICIENCY of p A_head.
 */
public record Cylinder(double boreM, double rodM, double strokeM, int count) {

    /**
     * Force efficiency of a cylinder for seal friction: the usual design allowance of 0.95 (friction of 3 to 10 % of
     * the force with ordinary rod and piston seals, as cylinder manufacturers' sizing guides take it).
     */
    public static final double SEAL_EFFICIENCY = 0.95;

    /** Speed over which seal friction changes sign, m/s (1 mm/s): a regularisation for time-stepping, not a measurement. */
    public static final double STANDSTILL_MPS = 1e-3;

    /** Checks the values: bore and stroke positive, the rod positive and thinner than the bore, at least one cylinder. */
    public Cylinder {
        Hydraulics.positive(boreM, "Bore");
        Hydraulics.positive(rodM, "Rod diameter");
        if (!(rodM < boreM)) { throw new IllegalArgumentException("The rod must be thinner than the bore"); }
        Hydraulics.positive(strokeM, "Stroke");
        if (count < 1) { throw new IllegalArgumentException("At least one cylinder"); }
    }

    /** One cylinder. */
    public static Cylinder of(double boreM, double rodM, double strokeM) { return new Cylinder(boreM, rodM, strokeM, 1); }

    /** Head-side (full bore) area of all the cylinders, m^2: n pi bore^2 / 4. */
    public double headAreaM2() { return count * Math.PI / 4 * boreM * boreM; }

    /** Rod-side (annulus) area of all the cylinders, m^2: n pi (bore^2 - rod^2) / 4. */
    public double rodSideAreaM2() { return count * Math.PI / 4 * (boreM * boreM - rodM * rodM); }

    /** Rod cross-section of all the cylinders, m^2: n pi rod^2 / 4, the head area less the rod-side area. */
    public double rodAreaM2() { return count * Math.PI / 4 * rodM * rodM; }

    /** Extending velocity (m/s) from a flow into the head side (m^3/s, signed): v = Q / A_head. */
    public double extendingVelocityMps(double headFlowM3PerS) {
        return Hydraulics.finite(headFlowM3PerS, "Flow") / headAreaM2();
    }

    /**
     * Retracting speed (m/s, positive retracting) from a flow into the rod side (m^3/s, signed): v = Q / A_rod side,
     * faster than the same flow extends by A_head / A_rod side.
     */
    public double retractingVelocityMps(double rodSideFlowM3PerS) {
        return Hydraulics.finite(rodSideFlowM3PerS, "Flow") / rodSideAreaM2();
    }

    /**
     * Extending velocity (m/s) in a regenerative circuit, the rod side's outflow joining the supply into the head:
     * only the difference of the areas (the rod's cross-section) takes supply, v = Q / (A_head - A_rod side), at
     * a push of only p A_rod.
     */
    public double regenerativeVelocityMps(double supplyFlowM3PerS) {
        return Hydraulics.finite(supplyFlowM3PerS, "Flow") / rodAreaM2();
    }

    /**
     * The flow the supply must give to move at a velocity (m/s, signed), m^3/s, not negative: v A_head extending,
     * -v A_rod side retracting.
     */
    public double supplyFlowM3PerS(double velocityMps) {
        double v = Hydraulics.finite(velocityMps, "Velocity");
        return v >= 0 ? v * headAreaM2() : -v * rodSideAreaM2();
    }

    /**
     * The flow the other side returns to tank at a velocity (m/s, signed), m^3/s, not negative: v A_rod side extending,
     * -v A_head retracting (more than the supply by A_head / A_rod side, which the return line and valve must pass).
     */
    public double returnFlowM3PerS(double velocityMps) {
        double v = Hydraulics.finite(velocityMps, "Velocity");
        return v >= 0 ? v * rodSideAreaM2() : -v * headAreaM2();
    }

    /**
     * The force the cylinders give (N, positive extending) at the head and rod-side pressures (Pa, gauge) while moving
     * at a velocity (m/s): p_head A_head - p_rod A_rod less the seal friction (see the class); at standstill the ideal.
     */
    public double forceN(double headPressurePa, double rodPressurePa, double velocityMps) {
        double head = Hydraulics.finite(headPressurePa, "Head pressure") * headAreaM2();
        double rod = Hydraulics.finite(rodPressurePa, "Rod pressure") * rodSideAreaM2();
        double sigma = Hydraulics.ramp(Hydraulics.finite(velocityMps, "Velocity"), STANDSTILL_MPS);
        return head - rod - (1 - SEAL_EFFICIENCY) * (Math.abs(head) + Math.abs(rod)) * sigma;
    }

    /**
     * The head-side pressure (Pa) that gives a force (N, positive extending) with a rod-side pressure (Pa) at a velocity
     * (m/s): the exact inverse of forceN in the head pressure. Below zero the load overruns (it would pull the oil
     * out faster than the supply gives it: a meter-out or counterbalance valve's business, not modelled).
     */
    public double headPressurePa(double forceN, double rodPressurePa, double velocityMps) {
        double ar = rodSideAreaM2();
        double sigma = Hydraulics.ramp(Hydraulics.finite(velocityMps, "Velocity"), STANDSTILL_MPS);
        double pr = Hydraulics.finite(rodPressurePa, "Rod pressure");
        double numerator = Hydraulics.finite(forceN, "Force") + ar * (pr + (1 - SEAL_EFFICIENCY) * Math.abs(pr) * sigma);
        double sign = numerator >= 0 ? 1 : -1;
        return numerator / (headAreaM2() * (1 - (1 - SEAL_EFFICIENCY) * sigma * sign));
    }

    /**
     * The rod-side pressure (Pa) that gives a force (N, positive extending, so a pull is negative) with a head-side
     * pressure (Pa) at a velocity (m/s): the exact inverse of forceN in the rod pressure.
     */
    public double rodPressurePa(double forceN, double headPressurePa, double velocityMps) {
        double ah = headAreaM2();
        double sigma = Hydraulics.ramp(Hydraulics.finite(velocityMps, "Velocity"), STANDSTILL_MPS);
        double ph = Hydraulics.finite(headPressurePa, "Head pressure");
        double numerator = ah * (ph - (1 - SEAL_EFFICIENCY) * Math.abs(ph) * sigma) - Hydraulics.finite(forceN, "Force");
        double sign = numerator >= 0 ? 1 : -1;
        return numerator / (rodSideAreaM2() * (1 + (1 - SEAL_EFFICIENCY) * sigma * sign));
    }

    /** The largest push (N) with a relief pressure (Pa) on the head side and the rod side at tank: SEAL_EFFICIENCY p A_head. */
    public double maxPushN(double reliefPa) {
        return SEAL_EFFICIENCY * Hydraulics.nonNegative(reliefPa, "Relief pressure") * headAreaM2();
    }

    /** The largest pull (N, positive) with a relief pressure (Pa) on the rod side and the head side at tank: SEAL_EFFICIENCY p A_rod side. */
    public double maxPullN(double reliefPa) {
        return SEAL_EFFICIENCY * Hydraulics.nonNegative(reliefPa, "Relief pressure") * rodSideAreaM2();
    }

    /**
     * Stiffness (N/m) of the cylinders with both sides shut in (a held load): the oil springs
     * k = B (A_head^2 / V_head + A_rod side^2 / V_rod side), V_head = A_head x + V_0, V_rod side = A_rod side (L - x) + V_0
     * (Merritt, Hydraulic Control Systems, 1967: 4 B A^2 / V_t for a symmetric actuator at mid stroke). x is the
     * extension (m, taken in [0, stroke]), B the effective bulk modulus (Pa), V_0 (m^3, positive) the oil between
     * each side's pistons and its valve, lines included, for all the cylinders together.
     */
    public double stiffnessNPerM(double extensionM, double bulkModulusPa, double deadVolumeM3) {
        double x = Math.max(0, Math.min(strokeM, Hydraulics.finite(extensionM, "Extension")));
        Hydraulics.positive(bulkModulusPa, "Bulk modulus");
        Hydraulics.positive(deadVolumeM3, "Dead volume");
        double ah = headAreaM2(), ar = rodSideAreaM2();
        return bulkModulusPa * (ah * ah / (ah * x + deadVolumeM3) + ar * ar / (ar * (strokeM - x) + deadVolumeM3));
    }
}
