package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.tags.FluidTags;
import net.minecraft.util.Mth;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.material.FluidState;

import java.util.ArrayList;
import java.util.List;

/**
 * The one free surface every consumer reads inside sea regions: AeroCore bodies (through WaveRegions.Local and the
 * wake sampler), vanilla entities and boats (fluid height, eye and buoyancy mixins), the camera fog and the client
 * renderer. On the server the persistent regions come from the world's saved data, on the client from the synced
 * mirror; both sides add the calm zones that follow wakes (Wakes), and evaluate the same field at the same game time
 * (game ticks x 0.05 s). A column belongs to a region when the water source block at the region's still level has an
 * open, non-fluid block above it; other water (pools at other levels, waterfalls) keeps vanilla behaviour. Where
 * regions overlap (a wake zone touching a persistent region), the first in the list, persistent ones first, owns it.
 */
public final class SeaSurface {
    private record Mirror(ResourceLocation dimension, List<WaveRegion> regions) {}
    private static volatile Mirror client = new Mirror(null, List.of());

    private SeaSurface() {}

    /** Called by the client when a region list arrives; the dedicated server never sets it. */
    public static void setClientRegions(ResourceLocation dimension, List<WaveRegion> regions) { client = new Mirror(dimension, List.copyOf(regions)); }
    public static void clearClient() { client = new Mirror(null, List.of()); }

    /** Saved sea regions of the level (the synced copy on the client). */
    public static List<WaveRegion> persistentRegions(Level level) {
        if (level instanceof ServerLevel server) { return WaveRegions.of(server).regions(); }
        Mirror m = client;
        return m.dimension() != null && m.dimension().equals(level.dimension().location()) ? m.regions() : List.of();
    }

    /** Persistent regions followed by the wake zones outside them. */
    public static List<WaveRegion> regions(Level level) {
        List<WaveRegion> persistent = persistentRegions(level), zones = Wakes.zones(level);
        if (zones.isEmpty()) { return persistent; }
        List<WaveRegion> all = new ArrayList<>(persistent);
        all.addAll(zones);
        return all;
    }

    public static WaveRegion regionAt(List<WaveRegion> regions, double x, double z) {
        for (WaveRegion region : regions) { if (region.contains(x, z)) { return region; } }
        return null;
    }

    /**
     * The region whose moving surface replaces the water at block column (x, z): the first region containing the
     * column centre whose still level is this column's free surface. Null where vanilla water rules apply.
     */
    public static WaveRegion owner(BlockGetter level, List<WaveRegion> regions, int x, int z) {
        for (WaveRegion region : regions) {
            if (region.contains(x + 0.5, z + 0.5) && ownsColumn(level, region, x, z)) { return region; }
        }
        return null;
    }

    /** Wave time at the start of the level's current tick, s. */
    public static double time(Level level) { return level.getGameTime() * MinecraftUnits.TICK_SECONDS; }

    /** How far the surface can move from the still level in this region, m (ambient waves plus wakes present). */
    public static double reach(Level level, WaveRegion region) { return region.compiled().maxAmplitude() + Wakes.maxElevation(level); }

    /**
     * True when the column through block (x, z) has its free surface at the region's still level: a still water
     * source block (not waterlogged) at floor(stillY) with an open, non-fluid block above.
     */
    public static boolean ownsColumn(BlockGetter level, WaveRegion region, int x, int z) {
        int y = Mth.floor(region.stillWaterY() - 1e-6);
        BlockPos pos = new BlockPos(x, y, z);
        BlockState state = level.getBlockState(pos);
        FluidState fluid = state.getFluidState();
        if (!state.is(Blocks.WATER) || !fluid.isSource()) { return false; }
        BlockPos abovePos = pos.above();
        BlockState above = level.getBlockState(abovePos);
        if (!above.getFluidState().isEmpty() || !above.getCollisionShape(level, abovePos).isEmpty()) { return false; }
        return region.ownsSurface(y + fluid.getHeight(level, pos));
    }

    /**
     * Still free-surface height of the water column through (x, y, z): the top of a water block with an open,
     * non-fluid block above, searched from 3 blocks above to 8 below; NaN when there is none.
     */
    public static double stillSurface(BlockGetter level, double x, double y, double z) {
        int bx = Mth.floor(x), by = Mth.floor(y), bz = Mth.floor(z);
        for (int cy = by + 3; cy >= by - 8; cy--) {
            BlockPos pos = new BlockPos(bx, cy, bz);
            FluidState fluid = level.getFluidState(pos);
            if (!fluid.is(FluidTags.WATER)) { continue; }
            BlockPos abovePos = pos.above();
            BlockState above = level.getBlockState(abovePos);
            if (above.getFluidState().isEmpty() && above.getCollisionShape(level, abovePos).isEmpty()) { return cy + fluid.getHeight(level, pos); }
            return Double.NaN;
        }
        return Double.NaN;
    }

    /** Instantaneous surface height of a region-owned column: still level plus tapered waves plus wakes. */
    public static double surfaceY(Level level, WaveRegion region, double x, double z, double t) { return surfaceY(level, region, x, z, t, -1); }
    /** As above without the wake of entity id exclude (a boat does not float on its own wake). */
    public static double surfaceY(Level level, WaveRegion region, double x, double z, double t, int exclude) {
        return region.stillWaterY() + region.elevation(x, z, t) + Wakes.elevation(level, region.stillWaterY(), x, z, exclude);
    }

    /**
     * Surface height at (x, z) when that column belongs to a sea region and the vertical span [minY, maxY] reaches
     * the band the surface can move through; NaN when vanilla water rules apply.
     */
    public static double surfaceNear(Level level, double x, double z, double minY, double maxY, double t) {
        WaveRegion region = governing(level, x, z, minY, maxY);
        return region == null ? Double.NaN : surfaceY(level, region, x, z, t);
    }

    /** The region governing column (x, z) for a span [minY, maxY], or null. */
    public static WaveRegion governing(Level level, double x, double z, double minY, double maxY) {
        List<WaveRegion> regions = regions(level);
        if (regions.isEmpty()) { return null; }
        WaveRegion region = owner(level, regions, Mth.floor(x), Mth.floor(z));
        if (region == null) { return null; }
        double still = region.stillWaterY(), reach = reach(level, region);
        return minY > still + reach || maxY < still - reach - 1 ? null : region;
    }

    /**
     * Whether the point is under the sea surface, for eyes and the camera: null where vanilla rules apply (no region,
     * far from the surface band, a column no region owns, or a solid / non-water block at the point).
     */
    public static Boolean submerged(Level level, double x, double y, double z, double t) {
        WaveRegion region = governing(level, x, z, y, y);
        if (region == null) { return null; }
        int bx = Mth.floor(x), by = Mth.floor(y), bz = Mth.floor(z), surfaceBlock = Mth.floor(region.stillWaterY() - 1e-6);
        // Below the surface block the point must be in the same water column; above it, in open air.
        for (int cy = Math.min(by, surfaceBlock); cy <= Math.max(by, surfaceBlock); cy++) {
            BlockPos pos = new BlockPos(bx, cy, bz);
            BlockState state = level.getBlockState(pos);
            boolean ok = cy <= surfaceBlock ? state.getFluidState().is(FluidTags.WATER)
                    : state.getFluidState().isEmpty() && state.getCollisionShape(level, pos).isEmpty();
            if (!ok) { return null; }
        }
        return y < surfaceY(level, region, x, z, t);
    }

    /** Ambient-wave orbital velocity (m/s) at a point of a governed column, tapered like the surface; wakes add none. */
    public static Vec3d orbitalVelocity(Level level, WaveRegion region, double x, double y, double z, double t) {
        if (region.compiled().isEmpty()) { return Vec3d.ZERO; }
        double w = region.weight(x, z);
        return w == 0 ? Vec3d.ZERO : region.compiled().velocity(new Vec3d(x, y, z), region.stillWaterY(), t).scale(w);
    }
}
