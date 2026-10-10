package dev.mcmod.aerocore.api;

/**
 * Quasi-steady propeller loads on a rigid body. Advance speed is the hub velocity relative to the local
 * fluid along the thrust axis; cross-flow forces, hull wake and slipstream interaction are not modeled.
 */
public final class PropellerSolver {
    private PropellerSolver() {}

    /**
     * Force is thrust along the axis at the hub, with the shaft reaction as its couple (the engine turns
     * the propeller with +Q about the axis, so the body receives -Q). advanceRatio is NaN when n = 0.
     */
    public record Load(AppliedForce force, double thrustN, double torqueNm, double advanceSpeedMps,
                       double advanceRatio, double shaftPowerW) {}

    public static Load load(RigidBodyState s, RigidBodyProperties p, Propeller propeller,
                            double revolutionsPerSecond, FluidState fluid) {
        if (!Double.isFinite(revolutionsPerSecond)) { throw new IllegalArgumentException("Propeller speed must be finite"); }
        Vec3d r = s.worldOffset(p, propeller.positionLocal());
        Vec3d axis = s.orientation().rotate(propeller.axisLocal());
        double advance = s.velocity().add(s.angularVelocity(p).cross(r)).subtract(fluid.velocity()).dot(axis);
        double d = propeller.diameterM(), rotation = 0.7 * Math.PI * revolutionsPerSecond * d;
        double speedSquared = advance * advance + rotation * rotation;
        double ratio = revolutionsPerSecond == 0 ? Double.NaN : advance / (revolutionsPerSecond * d);
        if (speedSquared == 0 || fluid.densityKgPerM3() == 0) {
            return new Load(new AppliedForce(AppliedForce.Kind.THRUST, r, Vec3d.ZERO), 0, 0, advance, ratio, 0);
        }
        PropellerCoefficients c = propeller.model().sample(Math.atan2(advance, rotation));
        double reference = 0.5 * fluid.densityKgPerM3() * speedSquared * Math.PI * d * d / 4;
        double thrust = c.thrust() * reference, torque = c.torque() * reference * d;
        AppliedForce force = new AppliedForce(AppliedForce.Kind.THRUST, r, axis.scale(thrust), axis.scale(-torque));
        double power = revolutionsPerSecond == 0 ? 0 : 2 * Math.PI * revolutionsPerSecond * torque;
        return new Load(force, thrust, torque, advance, ratio, power);
    }
}
