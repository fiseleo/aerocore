package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Position is world COM. Linear velocity and angular momentum about COM use world coordinates. */
public record RigidBodyState(Vec3d position, Vec3d velocity, Orientation orientation, Vec3d angularMomentum) {
    public RigidBodyState {
        Objects.requireNonNull(position); Objects.requireNonNull(velocity);
        Objects.requireNonNull(orientation); Objects.requireNonNull(angularMomentum);
    }
    public Vec3d angularVelocity(RigidBodyProperties properties) { return properties.inertia().solveWorld(orientation, angularMomentum); }
    public Vec3d worldOffset(RigidBodyProperties properties, Vec3d localPoint) {
        return orientation.rotate(localPoint.subtract(properties.centerOfMassLocal()));
    }
    public Vec3d worldPoint(RigidBodyProperties properties, Vec3d localPoint) { return position.add(worldOffset(properties, localPoint)); }
    public double kineticEnergy(RigidBodyProperties properties) {
        return 0.5 * properties.massKg() * velocity.lengthSquared() + 0.5 * angularMomentum.dot(angularVelocity(properties));
    }
}
