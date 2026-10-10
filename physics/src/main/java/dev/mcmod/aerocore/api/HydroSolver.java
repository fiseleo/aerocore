package dev.mcmod.aerocore.api;

/** Hydrostatic buoyancy and an isolated, fully submerged sphere drag correlation. SI units. */
public final class HydroSolver {
    private HydroSolver() {}

    /**
     * Exact spherical cap below a plane. Depth is positive below the surface; up points out of it.
     * For hydrostatics, up must oppose gravity. Displacement elements must not overlap.
     */
    public static Immersion sphereImmersion(BuoyantSphere sphere, double centerDepthM, Vec3d up) {
        if (!Double.isFinite(centerDepthM)) { throw new IllegalArgumentException("Depth must be finite"); }
        Vec3d unitUp = up.normalized();
        double r = sphere.radiusM();
        double h = Math.max(0, Math.min(2 * r, r + centerDepthM));
        if (h == 0) { return new Immersion(0, sphere.offset(), 0); }
        double volume = Math.PI * h * h * (r - h / 3);
        // Cap centroid relative to the sphere center; -3r/8 for a half-immersed sphere.
        double centroidHeight = -3 * (2 * r - h) * (2 * r - h) / (4 * (3 * r - h));
        return new Immersion(volume, sphere.offset().add(unitUp.scale(centroidHeight)),
                Math.min(1, volume / sphere.volumeM3()));
    }

    /**
     * Sphere in open water carrying a prescribed linear wave field, by columns x columns vertical-chord quadrature.
     * Each wet chord contributes its length and its linear pressure drop P(bottom) - P(top), P = head - (y - still),
     * with P = 0 on the instantaneous surface. Summed, these give the vertical Froude-Krylov force including the
     * depth-attenuated dynamic pressure, rather than hydrostatics on the instantaneous surface. Horizontal
     * pressure forces are not integrated here; use rho V_wet times the horizontal acceleration for them.
     */
    public static WaveImmersion sphereInWaves(Vec3d center, double radiusM, WaveField waves, double stillWaterY,
                                              double timeSeconds, int columns) {
        Checks.positive(radiusM, "Radius");
        if (columns < 3 || columns > 101) { throw new IllegalArgumentException("Column count must be 3..101"); }
        if (!Double.isFinite(stillWaterY) || !Double.isFinite(timeSeconds)) { throw new IllegalArgumentException("Level and time must be finite"); }
        double total = 0, wet = 0, pressure = 0, spacing = 2 * radiusM / columns;
        Vec3d centroid = Vec3d.ZERO, pressureCentroid = Vec3d.ZERO, velocity = Vec3d.ZERO, acceleration = Vec3d.ZERO;
        for (int ix = 0; ix < columns; ix++) {
            double x = -radiusM + (ix + 0.5) * spacing;
            for (int iz = 0; iz < columns; iz++) {
                double z = -radiusM + (iz + 0.5) * spacing;
                double radial = radiusM * radiusM - x * x - z * z;
                if (radial <= 0) { continue; }
                double half = Math.sqrt(radial), bottom = center.y() - half, top = center.y() + half;
                total += 2 * half;
                double cx = center.x() + x, cz = center.z() + z;
                double surface = stillWaterY + waves.elevation(cx, cz, timeSeconds);
                double upper = Math.min(top, surface);
                if (upper <= bottom) { continue; }
                double length = upper - bottom, middle = 0.5 * (bottom + upper);
                double drop = pressure(waves, cx, bottom, cz, stillWaterY, timeSeconds)
                        - (upper >= surface ? 0 : pressure(waves, cx, upper, cz, stillWaterY, timeSeconds));
                Vec3d offset = new Vec3d(x, middle - center.y(), z), point = new Vec3d(cx, middle, cz);
                wet += length; pressure += drop;
                centroid = centroid.add(offset.scale(length));
                pressureCentroid = pressureCentroid.add(offset.scale(drop));
                velocity = velocity.add(waves.velocity(point, stillWaterY, timeSeconds).scale(length));
                acceleration = acceleration.add(waves.acceleration(point, stillWaterY, timeSeconds).scale(length));
            }
        }
        if (wet == 0) { return new WaveImmersion(0, Vec3d.ZERO, 0, Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO); }
        Vec3d wetCentroid = centroid.scale(1 / wet);
        return new WaveImmersion(Math.min(1, wet / total), wetCentroid, pressure / total,
                pressure > 0 ? pressureCentroid.scale(1 / pressure) : wetCentroid,
                velocity.scale(1 / wet), acceleration.scale(1 / wet));
    }

    /** Linear total pressure head relative to still water, P = p/(rho g) = head - (y - still). */
    private static double pressure(WaveField waves, double x, double y, double z, double stillWaterY, double t) {
        return waves.dynamicPressureHead(new Vec3d(x, y, z), stillWaterY, t) - (y - stillWaterY);
    }

    /** Archimedes' force only. Add weight separately using the body's mass and gravity. */
    public static Wrench buoyancy(FluidState fluid, Immersion immersion, Vec3d gravityMps2) {
        if (immersion.displacedVolumeM3() == 0 || fluid.densityKgPerM3() == 0) { return Wrench.ZERO; }
        Vec3d force = gravityMps2.scale(-fluid.densityKgPerM3() * immersion.displacedVolumeM3());
        return Wrench.atOffset(force, immersion.centerOfBuoyancy());
    }

    /**
     * Schiller-Naumann below Re=1000, Cd=0.44 up to Re=200000. No free surface/drag crisis.
     * The sphere must be fully immersed. Use calibrated DragBody coefficients for other shapes.
     */
    public static Wrench sphereDrag(BodyMotion body, FluidState fluid, BuoyantSphere sphere) {
        Vec3d velocity = body.velocityAt(sphere.offset()).subtract(fluid.velocity());
        double speed = velocity.length();
        if (speed == 0 || fluid.densityKgPerM3() == 0) { return Wrench.ZERO; }
        double reynolds = FlowConditions.of(fluid, speed, 2 * sphere.radiusM()).reynoldsNumber();
        if (reynolds > 200000) { throw new IllegalArgumentException("Sphere drag model requires Re <= 200000"); }
        double factor;
        if (reynolds <= 1000) {
            // Algebraic Stokes form avoids a singular Cd at very small nonzero speeds.
            factor = 6 * Math.PI * fluid.dynamicViscosityPaS() * sphere.radiusM()
                    * (1 + 0.15 * Math.pow(reynolds, 0.687));
        } else {
            factor = 0.5 * fluid.densityKgPerM3() * speed * Math.PI * sphere.radiusM() * sphere.radiusM() * 0.44;
        }
        return Wrench.atOffset(velocity.scale(-factor), sphere.offset());
    }
}
