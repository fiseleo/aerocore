package dev.mcmod.aerocore.api;

import java.util.Objects;

/**
 * A spade rudder at model-local position (centre of pressure), turning about the local vertical axis. Positive
 * angles move the trailing edge to starboard, which pushes the stern to port and turns the bow to starboard.
 * Lift and drag come from AeroSolver with the given polar; behind a propeller the axial inflow is raised by the
 * slipstream of an actuator disk, u^2 = u_a |u_a| + eta 8 T / (rho pi D^2), with eta the fraction of the span
 * inside the race (D / span, at most 1). Slipstream contraction, swirl, hull wake and rudder stock are not modelled.
 */
public record Rudder(Vec3d positionLocal, double chordM, double spanM, AirfoilPolar polar) {
    public static final double MAX_ANGLE_RADIANS = Math.toRadians(35);

    public Rudder {
        Objects.requireNonNull(positionLocal, "positionLocal");
        Checks.positive(chordM, "Rudder chord"); Checks.positive(spanM, "Rudder span");
        Objects.requireNonNull(polar, "polar");
    }

    /**
     * Polar of a low-aspect-ratio rudder from Whicker and Fehlner (1958, DTMB Report 933): effective aspect ratio a_e
     * twice the geometric one (root against the hull, the model being mounted on a ground plane), lift slope
     * 0.9 * 2 pi a_e / (sqrt(a_e^2 + 4) + 1.8) per radian (no sweep), C_D0 0.0065 of a NACA 0015 section and induced drag
     * with span efficiency 0.9. Stall at 17.5 deg, the middle of the 15-20 deg open-water range of Molland and Turnock
     * (2007), blended over 10 deg; their 30-40 deg stall in a propeller slipstream and the a^2 cross-flow lift term
     * are not modelled. Whicker and Fehlner's tests ran at Re about 1e6 to 3e6.
     */
    public static AirfoilPolar lowAspectPolar(double chordM, double spanM) {
        double aspect = 2 * spanM / chordM, slope = 0.9 * 2 * Math.PI * aspect / (Math.sqrt(aspect * aspect + 4) + 1.8);
        return new SymmetricAirfoil(slope, Math.toRadians(17.5), Math.toRadians(10), 0.0065, aspect, 0.9);
    }

    /**
     * Rudder load at angle (rad, clamped to +-35 deg) in water whose velocity is the fluid's. thrustN and diameterM
     * describe the propeller ahead of it (thrust 0 for none); its slipstream only acts while the thrust is positive.
     */
    public SurfaceForces load(RigidBodyState s, RigidBodyProperties p, double angleRadians, FluidState water, double thrustN, double diameterM) {
        double angle = Math.clamp(angleRadians, -MAX_ANGLE_RADIANS, MAX_ANGLE_RADIANS);
        Orientation q = s.orientation();
        Vec3d offset = s.worldOffset(p, positionLocal);
        Vec3d ex = q.rotate(Vec3d.X);
        Vec3d chord = q.rotate(new Vec3d(Math.cos(angle), 0, -Math.sin(angle)));
        Vec3d normal = q.rotate(new Vec3d(Math.sin(angle), 0, Math.cos(angle)));
        Vec3d relative = s.velocity().add(s.angularVelocity(p).cross(offset)).subtract(water.velocity());
        double axial = relative.dot(ex);
        if (thrustN > 0 && diameterM > 0 && water.densityKgPerM3() > 0) {
            double race = Math.min(1, diameterM / spanM);
            double squared = axial * Math.abs(axial) + race * 8 * thrustN / (water.densityKgPerM3() * Math.PI * diameterM * diameterM);
            axial = Math.signum(squared) * Math.sqrt(Math.abs(squared));
            relative = relative.subtract(ex.scale(relative.dot(ex))).add(ex.scale(axial));
        }
        // Evaluate as a body moving with the relative velocity through still water.
        BodyMotion motion = new BodyMotion(relative, Vec3d.ZERO);
        LiftingSurface surface = new LiftingSurface(offset, chord, normal, chordM * spanM, chordM, polar);
        return AeroSolver.surface(motion, water.withVelocity(Vec3d.ZERO), surface);
    }
}
