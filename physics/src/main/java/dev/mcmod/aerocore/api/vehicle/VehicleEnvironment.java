package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.BuoyantSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.HydroSolver;
import dev.mcmod.aerocore.api.Immersion;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;

/**
 * What a vehicle's parts sample during one step (the caller builds it for the step's time): a sphere's immersion in
 * water and air (as FloatingBodyDynamics floats use) and the water and air at a point (as hulls use; Water carries
 * the surface, current, wave acceleration and the air with its wind). waves() is the wave field for the floats'
 * exact heave diffraction when their contact asks for it (Contact.surfaceDiffraction).
 */
public interface VehicleEnvironment {
    FloatingBodyDynamics.Contact sphere(Vec3d centre, double radiusM);
    HullDynamics.Water point(Vec3d worldPoint);
    default WaveField waves() { return WaveField.NONE; }

    static VehicleEnvironment of(FloatingBodyDynamics.Environment spheres, HullDynamics.Environment points, WaveField waves) {
        return new VehicleEnvironment() {
            @Override public FloatingBodyDynamics.Contact sphere(Vec3d centre, double radiusM) { return spheres.sample(centre, radiusM); }
            @Override public HullDynamics.Water point(Vec3d worldPoint) { return points.sample(worldPoint); }
            @Override public WaveField waves() { return waves; }
        };
    }

    /** Still water with its surface at waterY (exact spherical-cap immersion) and the given air (offline benches and tests). */
    static VehicleEnvironment calm(double waterY, FluidState water, FluidState air) {
        return of((centre, radius) -> {
            Immersion immersion = HydroSolver.sphereImmersion(new BuoyantSphere(Vec3d.ZERO, radius), waterY - centre.y(), Vec3d.Y);
            return new FloatingBodyDynamics.Contact(immersion.fraction(), immersion.centerOfBuoyancy(), air, water);
        }, point -> HullDynamics.Water.calm(waterY, water, air), WaveField.NONE);
    }
}
