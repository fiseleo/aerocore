package dev.mcmod.aerocore.api;

import java.util.Objects;

/**
 * Solver force at a world-frame offset from the step's starting COM. Couple is an additional pure
 * torque in N m (for example a propeller's shaft reaction); it has no application point.
 */
public record AppliedForce(Kind kind,Vec3d offset,Vec3d force,Vec3d couple) {
    /**
     * INERTIA is hydrodynamic inertia: added-mass reaction plus horizontal Froude-Krylov force from wave acceleration.
     * RADIATION is the wave-radiation memory force (WaveRadiation), supplied by the caller as an external load.
     */
    /** LIFT is circulation lift on a hull's trailing station or a control surface such as a rudder. */
    public enum Kind { WEIGHT, BUOYANCY, DRAG, CONTACT, THRUST, INERTIA, RADIATION, LIFT }
    public AppliedForce(Kind kind,Vec3d offset,Vec3d force) {this(kind,offset,force,Vec3d.ZERO);}
    public AppliedForce {
        Objects.requireNonNull(kind);Objects.requireNonNull(offset);Objects.requireNonNull(force);Objects.requireNonNull(couple);
    }
    public Wrench wrench() {
        Wrench point=Wrench.atOffset(force,offset);
        return new Wrench(point.force(),point.torque().add(couple));
    }
}
