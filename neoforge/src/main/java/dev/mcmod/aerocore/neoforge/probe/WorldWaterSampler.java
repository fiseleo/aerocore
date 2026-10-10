package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CompiledWaves;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.tags.FluidTags;
import net.minecraft.util.Mth;
import net.minecraft.world.level.material.FluidState;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * 7x7 horizontal quadrature of spherical water displacement, using actual water block heights.
 * Integrates each vertical chord continuously. Approximate at banks/curved boundaries.
 * An optional prescribed wave field raises or lowers each column's free surface (the top of a water run
 * with open space above) and adds orbital velocity; water blocks themselves never change.
 * Cache is valid for one server entity tick only and never loads absent chunks.
 */
public final class WorldWaterSampler {
    /**
     * fraction and wetCentroidOffset describe the wet geometry; pressureFraction and pressureCentroidOffset the
     * vertical pressure force (linear wave pressure, which decays with depth), equal to the geometry without waves.
     * accelerationMps2 is the wet-averaged wave acceleration; vanilla flow is treated as steady.
     */
    public record Contact(double fraction, Vec3d currentMps, Vec3d wetCentroidOffset,
                          double pressureFraction, Vec3d pressureCentroidOffset, Vec3d accelerationMps2) {}
    private record WaterCell(boolean water, boolean open, double surfaceY, Vec3d flow) {}
    private static final WaterCell SOLID = new WaterCell(false, false, Double.NEGATIVE_INFINITY, Vec3d.ZERO);
    private static final WaterCell AIR = new WaterCell(false, true, Double.NEGATIVE_INFINITY, Vec3d.ZERO);
    private static final int COLUMNS = 7, SURFACE_SEARCH = 16;
    /** Water speed of one unit of vanilla flow, m/s (see read()). */
    public static final double VANILLA_FLOW_MPS = 0.014 * 0.8 / (1 - 0.8) / MinecraftUnits.TICK_SECONDS;
    private final ServerLevel level;
    private final Map<BlockPos, WaterCell> cells = new HashMap<>();
    /** The last field compiled; bodies pass the same field object for every substep of a tick. */
    private WaveField compiledField = WaveField.NONE;
    private CompiledWaves compiled = CompiledWaves.NONE;
    private double[] cos = new double[0], sin = new double[0];
    private final double[] kinematics = new double[6], segment = new double[8], wake = new double[3], second = new double[3];
    /** Entity id whose own wake this sampler ignores (-1: none). */
    private int ownWake = -1;
    /** Sea region whose wind-driven drift moves the water under its free surfaces (null: none). */
    private WaveRegion drift;

    public WorldWaterSampler(ServerLevel level) { this.level = level; }

    /** Leaves out the wake of the given entity (a body's own wake is part of its resistance, not its surroundings). */
    public WorldWaterSampler excludingWakeOf(int entityId) { ownWake = entityId; return this; }

    /** Adds the region's wind-driven drift (WaveRegion.windDrift) under its free surfaces; null for none. */
    public WorldWaterSampler drifting(WaveRegion region) { drift = region == null || region.wind().calm() ? null : region; return this; }

    /** Wake elevation and slopes at (x, z) on the free surface at still level y, into wake[]. */
    private void wakeAt(double still, double x, double z) {
        wake[0] = 0; wake[1] = 0; wake[2] = 0;
        Wakes.sample(level, still, x, z, ownWake, wake);
    }

    public Contact sample(Vec3d center, double radiusM) { return sample(center, radiusM, WaveField.NONE, 0); }

    public Contact sample(Vec3d center, double radiusM, WaveField waves, double timeSeconds) {
        return sample(center, radiusM, waves, Double.NaN, timeSeconds);
    }

    /**
     * As above, but the waves move only free surfaces within WaveRegion.LEVEL_TOLERANCE of waveLevelY (a wave
     * region's still level); other water inside the region stays calm. NaN moves every free surface.
     */
    public Contact sample(Vec3d center, double radiusM, WaveField waves, double waveLevelY, double timeSeconds) {
        Objects.requireNonNull(waves, "waves");
        return sample(center, radiusM, compile(waves), waveLevelY, timeSeconds);
    }

    /** sample with waves already compiled (a body's WaveRegions.Local.compiled, the local view of its sea). */
    public Contact sample(Vec3d center, double radiusM, CompiledWaves fast, double waveLevelY, double timeSeconds) {
        if (!Double.isFinite(radiusM) || radiusM <= 0 || radiusM > 10) {
            throw new IllegalArgumentException("Water sampler radius must be in (0, 10] m");
        }
        Objects.requireNonNull(fast, "waves");
        if (!Double.isFinite(timeSeconds)) { throw new IllegalArgumentException("Time must be finite"); }
        ensurePhases(fast.size());
        double reach = fast.maxAmplitude();
        double wetLength = 0, totalLength = 0, pressure = 0;
        Vec3d weightedCurrent = Vec3d.ZERO, weightedAcceleration = Vec3d.ZERO;
        Vec3d weightedCentroid = Vec3d.ZERO, pressureCentroid = Vec3d.ZERO;
        double spacing = 2 * radiusM / COLUMNS;
        boolean phased = false;
        double correction = 0;
        int components = fast.size();
        for (int ix = 0; ix < COLUMNS; ix++) {
            double x = -radiusM + (ix + 0.5) * spacing;
            for (int iz = 0; iz < COLUMNS; iz++) {
                double z = -radiusM + (iz + 0.5) * spacing;
                double radialSquared = radiusM * radiusM - x * x - z * z;
                if (radialSquared <= 0) { continue; }
                double halfHeight = Math.sqrt(radialSquared);
                totalLength += 2 * halfHeight;
                double bottom = center.y() - halfHeight, top = center.y() + halfHeight;
                double columnX = center.x() + x, columnZ = center.z() + z;
                int blockX = Mth.floor(columnX), blockZ = Mth.floor(columnZ);
                int high = Mth.floor(top + reach);
                // Walk contiguous water runs; a wave can only move a run's free surface.
                for (int y = Mth.floor(bottom - reach); y <= high; y++) {
                    if (!cell(blockX, y, blockZ).water()) { continue; }
                    int start = y;
                    while (y < high + SURFACE_SEARCH && cell(blockX, y + 1, blockZ).water()) { y++; }
                    WaterCell above = cell(blockX, y + 1, blockZ);
                    double still = cell(blockX, y, blockZ).surfaceY();
                    boolean open = !above.water() && above.open();
                    boolean free = !fast.isEmpty() && open
                            && (Double.isNaN(waveLevelY) || Math.abs(still - waveLevelY) <= WaveRegion.LEVEL_TOLERANCE);
                    // Wakes raise any open surface at their still level, hydrostatically.
                    if (open) { wakeAt(still, columnX, columnZ); } else { wake[0] = 0; }
                    double wakeHeight = wake[0];
                    if (free && !phased) {
                        // All 7 x 7 column phases at once by phasor recurrence; the second-order correction, which varies
                        // over half a dominant wavelength, once at the centre column for the whole sphere.
                        double first = -radiusM + 0.5 * spacing;
                        fast.gridPhases(center.x() + first, center.z() + first, spacing, spacing, COLUMNS, COLUMNS, timeSeconds, cos, sin);
                        correction = fast.correctionHeight(cos, sin, (COLUMNS / 2 * COLUMNS + COLUMNS / 2) * components);
                        phased = true;
                    }
                    int phaseOffset = (ix * COLUMNS + iz) * components;
                    if (free) { second[0] = fast.cheapSecondOrderHeight(sin, phaseOffset) + correction; }
                    double surface = (free ? still + fast.elevation(cos, phaseOffset) + second[0] : still) + wakeHeight;
                    boolean drifts = open && drift != null && drift.ownsSurface(still);
                    double lower = Math.max(bottom, start), upper = Math.min(top, surface);
                    for (int cellY = start; cellY <= y && upper > lower; cellY++) {
                        // A crest above the top block is attributed to that block's flow.
                        double segmentLow = Math.max(lower, cellY), segmentHigh = cellY == y ? upper : Math.min(upper, cellY + 1);
                        double length = segmentHigh - segmentLow;
                        if (length <= 0) { continue; }
                        double middleY = (segmentLow + segmentHigh) / 2;
                        Vec3d offset = new Vec3d(x, middleY - center.y(), z);
                        Vec3d flow = cell(blockX, cellY, blockZ).flow();
                        if (drifts) { flow = flow.add(drift.windDrift(columnX, columnZ, surface - (segmentLow + segmentHigh) / 2)); }
                        // Linear pressure head relative to still water, zero on the instantaneous surface.
                        double drop = length;
                        if (free) {
                            fast.segment(cos, sin, phaseOffset, segmentLow - still, segmentHigh - still, segment);
                            flow = flow.add(new Vec3d(segment[2], segment[3], segment[4]));
                            weightedAcceleration = weightedAcceleration.add(new Vec3d(segment[5], segment[6], segment[7]).scale(length));
                            // Total linear pressure head relative to still water: dynamic head minus height.
                            drop = segment[0] + wakeHeight - (segmentLow - still) - (segmentHigh >= surface ? 0 : segment[1] + wakeHeight - (segmentHigh - still));
                        }
                        wetLength += length;
                        pressure += drop;
                        weightedCurrent = weightedCurrent.add(flow.scale(length));
                        weightedCentroid = weightedCentroid.add(offset.scale(length));
                        pressureCentroid = pressureCentroid.add(offset.scale(drop));
                    }
                }
            }
        }
        // Normalization gives exactly zero/full displacement and symmetric half immersion.
        double fraction = Math.clamp(wetLength / totalLength, 0, 1);
        Vec3d wetCentroid = wetLength == 0 ? Vec3d.ZERO : weightedCentroid.scale(1 / wetLength);
        return new Contact(fraction, wetLength == 0 ? Vec3d.ZERO : weightedCurrent.scale(1 / wetLength), wetCentroid,
                pressure / totalLength, pressure > 0 ? pressureCentroid.scale(1 / pressure) : wetCentroid,
                wetLength == 0 ? Vec3d.ZERO : weightedAcceleration.scale(1 / wetLength));
    }

    private CompiledWaves compile(WaveField waves) {
        if (waves != compiledField) {
            compiledField = waves;
            compiled = new CompiledWaves(waves);
        }
        return compiled;
    }

    private void ensurePhases(int components) {
        if (cos.length < COLUMNS * COLUMNS * components) { cos = new double[COLUMNS * COLUMNS * components]; sin = new double[COLUMNS * COLUMNS * components]; }
    }

    /**
     * Water column at a point, for hull strips: the top of the water run within 3 blocks above to 8 below the point
     * (NaN when there is none), raised by the waves when it is a free surface at the wave level, with the surface
     * slope, the still level and an amplitude-weighted wavenumber for the pressure decay (NaN and 0 when calm), and the
     * vanilla flow plus orbital velocity and wave acceleration at the point.
     */
    public record Column(double surfaceY, Vec3d slope, double stillY, double decayWavenumber, Vec3d currentMps, Vec3d accelerationMps2) {
        public static final Column NONE = new Column(Double.NaN, Vec3d.ZERO, Double.NaN, 0, Vec3d.ZERO, Vec3d.ZERO);
    }

    public Column column(Vec3d point, WaveField waves, double waveLevelY, double timeSeconds) {
        return column(point, waves.isEmpty() ? CompiledWaves.NONE : compile(waves), waveLevelY, timeSeconds);
    }

    /** column with waves already compiled (a body's local view of its sea). */
    public Column column(Vec3d point, CompiledWaves fast, double waveLevelY, double timeSeconds) {
        int x = Mth.floor(point.x()), z = Mth.floor(point.z()), base = Mth.floor(point.y());
        for (int y = base + 3; y >= base - 8; y--) {
            if (!cell(x, y, z).water() || cell(x, y + 1, z).water()) { continue; }
            double still = cell(x, y, z).surfaceY();
            WaterCell above = cell(x, y + 1, z);
            boolean moving = above.open() && !fast.isEmpty() && (Double.isNaN(waveLevelY) || Math.abs(still - waveLevelY) <= WaveRegion.LEVEL_TOLERANCE);
            int flowY = Math.clamp(base, y - 64, y);
            Vec3d flow = cell(x, flowY, z).water() ? cell(x, flowY, z).flow() : cell(x, y, z).flow();
            if (above.open()) { wakeAt(still, point.x(), point.z()); } else { wake[0] = 0; wake[1] = 0; wake[2] = 0; }
            Vec3d wakeSlope = new Vec3d(wake[1], 0, wake[2]);
            if (above.open() && drift != null && drift.ownsSurface(still)) { flow = flow.add(drift.windDrift(point.x(), point.z(), still - point.y())); }
            if (!moving) { return new Column(still + wake[0], wakeSlope, Double.NaN, 0, flow, Vec3d.ZERO); }
            ensurePhases(fast.size());
            fast.phases(point.x(), point.z(), timeSeconds, cos, sin);
            fast.kinematics(cos, sin, point.y() - still, kinematics);
            double[] slope = fast.slope(sin);
            fast.secondOrder(cos, sin, 0, second);
            return new Column(still + fast.elevation(cos) + second[0] + wake[0], new Vec3d(slope[0] + second[1], 0, slope[1] + second[2]).add(wakeSlope), still, fast.meanWavenumber(),
                    flow.add(new Vec3d(kinematics[0], kinematics[1], kinematics[2])), new Vec3d(kinematics[3], kinematics[4], kinematics[5]));
        }
        return Column.NONE;
    }

    /** Free-surface height of the water column through the given block, or NaN if there is none nearby. */
    public double stillSurface(BlockPos start) {
        for (int dy = 3; dy >= -8; dy--) {
            int y = start.getY() + dy;
            WaterCell here = cell(start.getX(), y, start.getZ()), above = cell(start.getX(), y + 1, start.getZ());
            if (here.water() && !above.water() && above.open()) { return here.surfaceY(); }
        }
        return Double.NaN;
    }

    /** Depth from the given surface to the first non-water block below; +infinity (deep water) beyond 64 m. */
    public double depthBelow(BlockPos column, double surfaceY) {
        int top = Mth.floor(surfaceY - 1e-6);
        for (int y = top; y > top - 64; y--) {
            if (!cell(column.getX(), y, column.getZ()).water()) { return surfaceY - (y + 1); }
        }
        return Double.POSITIVE_INFINITY;
    }

    private WaterCell cell(int x, int y, int z) {
        return cells.computeIfAbsent(new BlockPos(x, y, z), this::read);
    }

    private WaterCell read(BlockPos position) {
        if (!level.hasChunkAt(position)) { return SOLID; }
        boolean open = level.getBlockState(position).getCollisionShape(level, position).isEmpty();
        FluidState fluid = level.getFluidState(position);
        if (!fluid.is(FluidTags.WATER)) { return open ? AIR : SOLID; }
        // Vanilla flow is dimensionless; its speed is set by what vanilla itself does with it: a mob (water damping
        // f = 0.8 per tick) pushed by a full-strength flow at 0.014 blocks/tick^2 drifts at 0.014 f / (1 - f) = 0.056
        // blocks/tick, so one flow unit is taken as 1.12 m/s.
        return new WaterCell(true, open, position.getY() + fluid.getHeight(level, position),
                MinecraftUnits.fromMinecraft(fluid.getFlow(level, position)).scale(VANILLA_FLOW_MPS));
    }
}
