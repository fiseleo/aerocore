package dev.mcmod.aerocore.api;

/**
 * Translational sphere demonstrator: gravity, displaced-fluid buoyancy, area-weighted quadratic drag.
 * Wet fraction is supplied by geometry. Constant Cd and wet area weighting are explicit approximations.
 * Does not model rotation, water entry/slamming, added mass or waves.
 */
public final class SphereDynamics {
    private SphereDynamics() {}

    public record Properties(double massKg, double radiusM, double dragCoefficient) {
        public Properties {
            Checks.positive(massKg, "Mass");
            Checks.positive(radiusM, "Radius");
            Checks.nonnegative(dragCoefficient, "Drag coefficient");
        }
        public double volumeM3() { return 4.0 / 3 * Math.PI * radiusM * radiusM * radiusM; }
        public double projectedAreaM2() { return Math.PI * radiusM * radiusM; }
    }

    /** Drag is the average force represented by this step's drag impulse, before any collision. */
    public record Result(TranslationState state, Vec3d weight, Vec3d buoyancy, Vec3d averageDrag) {
        public Vec3d netForce() { return weight.add(buoyancy).add(averageDrag); }
    }

    public static Result step(TranslationState state, Properties properties, FluidState air, FluidState water,
                              double wetFraction, Vec3d gravityMps2, double dtSeconds) {
        Checks.positive(dtSeconds, "Time step");
        if (!Double.isFinite(wetFraction) || wetFraction < 0 || wetFraction > 1) {
            throw new IllegalArgumentException("Wet fraction must be in [0, 1]");
        }
        double mass = properties.massKg();
        double displacedMass = properties.volumeM3()
                * (air.densityKgPerM3() * (1 - wetFraction) + water.densityKgPerM3() * wetFraction);
        Vec3d weight = gravityMps2.scale(mass);
        Vec3d buoyancy = gravityMps2.scale(-displacedMass);
        Vec3d halfKick = weight.add(buoyancy).scale(dtSeconds / (2 * mass));
        Vec3d beforeDrag = state.velocity().add(halfKick);
        double area = properties.projectedAreaM2(), cd = properties.dragCoefficient();
        // Symmetric splitting of air/water drag. Each update dissipates relative kinetic energy.
        Vec3d velocity = LinearDynamics.quadraticDrag(beforeDrag, air, area * (1 - wetFraction), cd, mass, dtSeconds / 2);
        velocity = LinearDynamics.quadraticDrag(velocity, water, area * wetFraction, cd, mass, dtSeconds);
        velocity = LinearDynamics.quadraticDrag(velocity, air, area * (1 - wetFraction), cd, mass, dtSeconds / 2);
        Vec3d averageDrag = velocity.subtract(beforeDrag).scale(mass / dtSeconds);
        velocity = velocity.add(halfKick);
        // Kick then drift (symplectic Euler in the no-drag limit): unlike using the old-force
        // parabolic position estimate, this does not continually add energy to buoyant oscillation.
        Vec3d position = state.position().add(velocity.scale(dtSeconds));
        return new Result(new TranslationState(position, velocity), weight, buoyancy, averageDrag);
    }
}
