package dev.mcmod.aerocore.api;

import java.util.List;

/** Stateless, quasi-steady force solver. It does not integrate motion or interact with Minecraft. */
public final class AeroSolver {
    private static final double MIN_SPEED_SQUARED = 1e-12;
    private AeroSolver() {}

    public static SurfaceForces surface(BodyMotion body, AirState air, LiftingSurface surface) {
        return surface(body, air.asFluid(), surface);
    }

    /** Also supports fully submerged hydrofoils. Free-surface and cavitation effects are excluded. */
    public static SurfaceForces surface(BodyMotion body, FluidState fluid, LiftingSurface surface) {
        Vec3d relativeVelocity = body.velocityAt(surface.offset()).subtract(fluid.velocity());
        Vec3d span = surface.span();
        // Sectional model: pure spanwise flow contributes no wing force. Use body drag for that axis.
        Vec3d sectionVelocity = relativeVelocity.subtract(span.scale(relativeVelocity.dot(span)));
        double speedSquared = sectionVelocity.lengthSquared();
        if (speedSquared < MIN_SPEED_SQUARED || fluid.densityKgPerM3() == 0) {
            return SurfaceForces.ZERO;
        }
        Vec3d direction = sectionVelocity.scale(1 / Math.sqrt(speedSquared));
        double alpha = Math.atan2(-sectionVelocity.dot(surface.normal()), sectionVelocity.dot(surface.chord()));
        FlowConditions flow = FlowConditions.of(fluid, Math.sqrt(speedSquared), surface.chordLengthM());
        AeroCoefficients coefficients = surface.polar().sample(alpha, flow);
        double pressure = 0.5 * fluid.densityKgPerM3() * speedSquared;
        double scale = pressure * surface.areaM2();
        Vec3d lift = span.cross(direction).scale(scale * coefficients.lift());
        Vec3d drag = direction.scale(-scale * coefficients.drag());
        Wrench wrench = Wrench.atOffset(lift.add(drag), surface.offset());
        Vec3d intrinsicMoment = span.scale(scale * surface.chordLengthM() * coefficients.pitchingMoment());
        return new SurfaceForces(new Wrench(wrench.force(), wrench.torque().add(intrinsicMoment)),
                lift, drag, alpha, pressure, coefficients, flow);
    }

    public static Wrench drag(BodyMotion body, AirState air, DragBody dragBody) {
        return drag(body, air.asFluid(), dragBody);
    }

    public static Wrench drag(BodyMotion body, FluidState fluid, DragBody dragBody) {
        Vec3d velocity = body.velocityAt(dragBody.offset()).subtract(fluid.velocity());
        double speedSquared = velocity.lengthSquared();
        if (speedSquared < MIN_SPEED_SQUARED || fluid.densityKgPerM3() == 0) { return Wrench.ZERO; }
        double scale = -0.5 * fluid.densityKgPerM3() * Math.sqrt(speedSquared)
                * dragBody.areaM2() * dragBody.dragCoefficient();
        return Wrench.atOffset(velocity.scale(scale), dragBody.offset());
    }

    /** All offsets and angular velocity must refer to the same center of mass. */
    public static Wrench solve(BodyMotion body, AirState air,
                               List<LiftingSurface> surfaces, List<DragBody> dragBodies) {
        return solve(body, air.asFluid(), surfaces, dragBodies);
    }

    public static Wrench solve(BodyMotion body, FluidState fluid,
                               List<LiftingSurface> surfaces, List<DragBody> dragBodies) {
        Wrench total = Wrench.ZERO;
        for (LiftingSurface surface : surfaces) { total = total.add(surface(body, fluid, surface).wrench()); }
        for (DragBody dragBody : dragBodies) { total = total.add(drag(body, fluid, dragBody)); }
        return total;
    }

    /** Samples each element's application point. Split surfaces crossing a fluid interface yourself. */
    public static Wrench solve(BodyMotion body, Vec3d centerOfMassM, FluidField field, double timeSeconds,
                               List<LiftingSurface> surfaces, List<DragBody> dragBodies) {
        if (!Double.isFinite(timeSeconds)) { throw new IllegalArgumentException("Time must be finite"); }
        Wrench total = Wrench.ZERO;
        for (LiftingSurface surface : surfaces) {
            total = total.add(surface(body, field.sample(centerOfMassM.add(surface.offset()), timeSeconds), surface).wrench());
        }
        for (DragBody dragBody : dragBodies) {
            total = total.add(drag(body, field.sample(centerOfMassM.add(dragBody.offset()), timeSeconds), dragBody));
        }
        return total;
    }
}
