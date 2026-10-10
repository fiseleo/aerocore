package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CompiledWaves;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import net.minecraft.core.BlockPos;
import net.minecraft.core.HolderLookup;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.nbt.ListTag;
import net.minecraft.nbt.Tag;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.tags.FluidTags;
import net.minecraft.util.Mth;
import net.minecraft.world.level.saveddata.SavedData;

import java.util.ArrayList;
import java.util.List;

/**
 * The wave regions of one dimension, saved with the world and mirrored to its players. Regions never overlap;
 * adding one replaces those it touches. Water blocks are never changed.
 */
public final class WaveRegions extends SavedData {
    public static final int MAX_REGIONS = 16;
    private static final String NAME = "aerocore_wave_regions";
    private static final Factory<WaveRegions> FACTORY = new Factory<>(WaveRegions::new, WaveRegions::load, null);
    private List<WaveRegion> regions = List.of();

    /**
     * The sea a body at (x, z) samples (WaveRegion.localWaves): compiled for the water sampler, the same waves as a
     * WaveField for frequency-domain loads, and the still level; calm water outside every region.
     */
    public record Local(WaveField waves, CompiledWaves compiled, double stillWaterY, WaveRegion region) {
        public static final Local CALM = new Local(WaveField.NONE, CompiledWaves.NONE, Double.NaN, null);

        /** The region's wind at a point (log profile above the still level, tapered like the waves), m/s; zero outside. */
        public Vec3d windAt(double x, double y, double z) {
            if (region == null || region.wind().calm()) { return Vec3d.ZERO; }
            return region.wind().at(y - stillWaterY).scale(region.weight(x, z));
        }
    }

    public static WaveRegions of(ServerLevel level) { return level.getDataStorage().computeIfAbsent(FACTORY, NAME); }

    public List<WaveRegion> regions() { return regions; }

    public Local at(double x, double z) {
        for (WaveRegion region : regions) {
            if (region.contains(x, z)) {
                CompiledWaves local = region.localWaves(x, z);
                return new Local(local.field(), local, region.stillWaterY(), region);
            }
        }
        return Local.CALM;
    }

    public WaveRegion regionAt(double x, double z) {
        for (WaveRegion region : regions) { if (region.contains(x, z)) { return region; } }
        return null;
    }

    /** Adds a region, removing any it overlaps; returns how many were replaced. */
    public int add(ServerLevel level, WaveRegion region) {
        List<WaveRegion> next = new ArrayList<>();
        for (WaveRegion old : regions) { if (!old.overlaps(region)) { next.add(old); } }
        int replaced = regions.size() - next.size();
        if (next.size() >= MAX_REGIONS) { throw new IllegalStateException("Wave region limit reached"); }
        next.add(region);
        update(level, next);
        return replaced;
    }

    /** Puts next in the place of old (a new wind or survey for the same sea). */
    public void replace(ServerLevel level, WaveRegion old, WaveRegion next) {
        List<WaveRegion> list = new ArrayList<>(regions);
        int index = list.indexOf(old);
        if (index < 0) { throw new IllegalStateException("Wave region not found"); }
        list.set(index, next);
        update(level, list);
    }

    /** Removes the regions containing (x, z); returns how many were removed. */
    public int removeAt(ServerLevel level, double x, double z) {
        List<WaveRegion> next = regions.stream().filter(region -> !region.contains(x, z)).toList();
        int removed = regions.size() - next.size();
        if (removed > 0) { update(level, next); }
        return removed;
    }

    public int clear(ServerLevel level) {
        int removed = regions.size();
        if (removed > 0) { update(level, List.of()); }
        return removed;
    }

    private void update(ServerLevel level, List<WaveRegion> next) {
        regions = List.copyOf(next);
        setDirty();
        WaveNetwork.sync(level);
    }

    /**
     * Surveys the bed under a region's square every Survey.SPACING m: at each node the column at the still level is
     * land unless it is water, and the depth runs down to the first block that is not water (DEEP below
     * Survey.MAX_DEPTH, and where the chunk is not loaded, which is taken as open sea). Blocks are only read.
     */
    public static WaveRegion.Survey survey(ServerLevel level, double centerX, double centerZ, double radius, double stillY) {
        double spacing = WaveRegion.Survey.SPACING;
        double x0 = Math.floor((centerX - radius) / spacing) * spacing, z0 = Math.floor((centerZ - radius) / spacing) * spacing;
        int nx = (int) Math.ceil((centerX + radius - x0) / spacing) + 1, nz = (int) Math.ceil((centerZ + radius - z0) / spacing) + 1;
        short[] depth = new short[nx * nz];
        int top = Mth.floor(stillY - 1e-6);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int bx = Mth.floor(x0 + i * spacing), bz = Mth.floor(z0 + j * spacing), index = i * nz + j;
            pos.set(bx, top, bz);
            if (!level.hasChunkAt(pos)) { depth[index] = WaveRegion.Survey.DEEP; continue; }
            if (!level.getFluidState(pos).is(FluidTags.WATER)) { depth[index] = WaveRegion.Survey.LAND; continue; }
            int y = top;
            while (y > top - WaveRegion.Survey.MAX_DEPTH && level.getFluidState(pos.setY(y - 1)).is(FluidTags.WATER)) { y--; }
            depth[index] = y <= top - WaveRegion.Survey.MAX_DEPTH ? WaveRegion.Survey.DEEP : (short) Math.clamp(Math.round((stillY - y) * 10), 1, 32766);
        }
        return new WaveRegion.Survey(x0, z0, spacing, nx, nz, depth);
    }

    @Override public CompoundTag save(CompoundTag tag, HolderLookup.Provider registries) {
        ListTag list = new ListTag();
        for (WaveRegion region : regions) {
            CompoundTag entry = new CompoundTag();
            entry.putDouble("X", region.centerX()); entry.putDouble("Z", region.centerZ());
            entry.putDouble("Radius", region.radiusM()); entry.putDouble("StillY", region.stillWaterY());
            ListTag components = new ListTag();
            for (LinearWave wave : region.waves().components()) {
                CompoundTag c = new CompoundTag();
                c.putDouble("Amplitude", wave.amplitudeM()); c.putDouble("Wavelength", wave.wavelengthM());
                c.putDouble("Direction", wave.directionRadians()); c.putDouble("Phase", wave.phaseRadians());
                // NBT doubles keep +infinity for deep water.
                c.putDouble("Depth", wave.depthM());
                components.add(c);
            }
            entry.put("Waves", components);
            WaveRegion.Sea sea = region.sea();
            if (sea != null) {
                CompoundTag spectrum = new CompoundTag();
                spectrum.putDouble("Hs", sea.significantHeightM()); spectrum.putDouble("Tp", sea.peakPeriodS());
                spectrum.putDouble("Gamma", sea.gamma()); spectrum.putDouble("Direction", sea.directionRadians());
                spectrum.putDouble("Spreading", sea.spreading());
                entry.put("Sea", spectrum);
            }
            if (!region.wind().calm()) {
                entry.putDouble("WindSpeed", region.wind().speedMps()); entry.putDouble("WindDirection", region.wind().directionRadians());
            }
            WaveRegion.Survey s = region.survey();
            if (s != null) {
                CompoundTag survey = new CompoundTag();
                survey.putDouble("X0", s.x0()); survey.putDouble("Z0", s.z0()); survey.putDouble("Spacing", s.spacing());
                survey.putInt("Nx", s.nx()); survey.putInt("Nz", s.nz());
                int[] depth = new int[s.nx() * s.nz()];
                for (int i = 0; i < depth.length; i++) { depth[i] = s.depthDm(i); }
                survey.putIntArray("DepthDm", depth);
                entry.put("Survey", survey);
            }
            list.add(entry);
        }
        tag.put("Regions", list);
        return tag;
    }

    static WaveRegions load(CompoundTag tag, HolderLookup.Provider registries) {
        WaveRegions result = new WaveRegions();
        List<WaveRegion> loaded = new ArrayList<>();
        for (Tag element : tag.getList("Regions", Tag.TAG_COMPOUND)) {
            CompoundTag entry = (CompoundTag) element;
            try {
                List<LinearWave> waves = new ArrayList<>();
                for (Tag w : entry.getList("Waves", Tag.TAG_COMPOUND)) {
                    CompoundTag c = (CompoundTag) w;
                    waves.add(new LinearWave(c.getDouble("Amplitude"), c.getDouble("Wavelength"), c.getDouble("Direction"),
                            c.getDouble("Depth"), c.getDouble("Phase")));
                }
                WaveRegion.Sea sea = null;
                if (entry.contains("Sea", Tag.TAG_COMPOUND)) {
                    CompoundTag s = entry.getCompound("Sea");
                    sea = new WaveRegion.Sea(s.getDouble("Hs"), s.getDouble("Tp"), s.getDouble("Gamma"), s.getDouble("Direction"), s.getDouble("Spreading"));
                }
                WaveRegion.Wind wind = entry.contains("WindSpeed", Tag.TAG_DOUBLE)
                        ? new WaveRegion.Wind(entry.getDouble("WindSpeed"), entry.getDouble("WindDirection")) : WaveRegion.Wind.CALM;
                WaveRegion.Survey survey = null;
                if (entry.contains("Survey", Tag.TAG_COMPOUND)) {
                    CompoundTag s = entry.getCompound("Survey");
                    int[] stored = s.getIntArray("DepthDm");
                    short[] depth = new short[stored.length];
                    for (int i = 0; i < stored.length; i++) { depth[i] = (short) Math.clamp(stored[i], WaveRegion.Survey.LAND, WaveRegion.Survey.DEEP); }
                    survey = new WaveRegion.Survey(s.getDouble("X0"), s.getDouble("Z0"), s.getDouble("Spacing"), s.getInt("Nx"), s.getInt("Nz"), depth);
                }
                WaveRegion region = new WaveRegion(entry.getDouble("X"), entry.getDouble("Z"), entry.getDouble("Radius"),
                        entry.getDouble("StillY"), new WaveField(waves), sea, survey, wind);
                if (loaded.size() < MAX_REGIONS && loaded.stream().noneMatch(region::overlaps)) { loaded.add(region); }
            } catch (IllegalArgumentException invalid) {
                // A corrupted or hand-edited entry is dropped rather than loaded outside the validated range.
            }
        }
        result.regions = List.copyOf(loaded);
        return result;
    }
}
