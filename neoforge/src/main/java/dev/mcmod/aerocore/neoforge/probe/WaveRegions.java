package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.WaveField;
import net.minecraft.core.HolderLookup;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.nbt.ListTag;
import net.minecraft.nbt.Tag;
import net.minecraft.server.level.ServerLevel;
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

    /** Field and still level a body at (x, z) should use; the field is still water outside every region. */
    public record Local(WaveField waves, double stillWaterY, WaveRegion region) {
        public static final Local CALM = new Local(WaveField.NONE, Double.NaN, null);
    }

    public static WaveRegions of(ServerLevel level) { return level.getDataStorage().computeIfAbsent(FACTORY, NAME); }

    public List<WaveRegion> regions() { return regions; }

    public Local at(double x, double z) {
        for (WaveRegion region : regions) {
            if (region.contains(x, z)) { return new Local(region.fieldAt(x, z), region.stillWaterY(), region); }
        }
        return Local.CALM;
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
                WaveRegion region = new WaveRegion(entry.getDouble("X"), entry.getDouble("Z"), entry.getDouble("Radius"),
                        entry.getDouble("StillY"), new WaveField(waves), sea);
                if (loaded.size() < MAX_REGIONS && loaded.stream().noneMatch(region::overlaps)) { loaded.add(region); }
            } catch (IllegalArgumentException invalid) {
                // A corrupted or hand-edited entry is dropped rather than loaded outside the validated range.
            }
        }
        result.regions = List.copyOf(loaded);
        return result;
    }
}
