package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.SeaSurface;
import dev.mcmod.aerocore.neoforge.probe.WaveNetwork;
import dev.mcmod.aerocore.neoforge.probe.WaveRegion;
import net.minecraft.client.Minecraft;
import net.minecraft.core.BlockPos;
import net.minecraft.core.SectionPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.util.Mth;
import net.minecraft.world.level.BlockGetter;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.fml.event.lifecycle.FMLClientSetupEvent;
import net.neoforged.neoforge.client.event.ClientPlayerNetworkEvent;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.List;

/**
 * Client copy of the current dimension's wave regions. Chunk meshing threads read it to drop the vanilla top face
 * of the water surfaces that WaveSurfaceRenderer redraws, so both use {@link #surfaceRegion}.
 */
public final class ClientWaveRegions {
    private record Snapshot(ResourceLocation dimension, List<WaveRegion> regions) {}
    private static final Snapshot EMPTY = new Snapshot(null, List.of());
    private static volatile Snapshot snapshot = EMPTY;
    private static volatile ResourceLocation currentDimension;

    private ClientWaveRegions() {}

    /** Wake zones of the client level last tick (Wakes), kept here so chunk-meshing threads see a stable list. */
    private static volatile List<WaveRegion> zones = List.of();

    /** Saved regions of the dimension the client is in (none while a payload for another dimension is pending), then wake zones. */
    public static List<WaveRegion> current() {
        Snapshot s = snapshot;
        List<WaveRegion> persistent = s.dimension() != null && s.dimension().equals(currentDimension) ? s.regions() : List.of();
        List<WaveRegion> z = zones;
        if (z.isEmpty()) { return persistent; }
        List<WaveRegion> all = new java.util.ArrayList<>(persistent);
        all.addAll(z);
        return all;
    }

    /**
     * The region whose moving surface replaces this block's top face (SeaSurface.ownsColumn at the region's still
     * level, column centre inside the region), or null.
     */
    public static WaveRegion surfaceRegion(BlockGetter level, BlockPos pos) {
        List<WaveRegion> regions = current();
        if (regions.isEmpty()) { return null; }
        WaveRegion owner = SeaSurface.owner(level, regions, pos.getX(), pos.getZ());
        return owner != null && Mth.floor(owner.stillWaterY() - 1e-6) == pos.getY() ? owner : null;
    }

    static void accept(WaveNetwork.RegionsPayload payload) {
        long started = System.nanoTime();
        Snapshot previous = snapshot;
        currentDimension = Minecraft.getInstance().level == null ? null : Minecraft.getInstance().level.dimension().location();
        boolean same = previous.dimension() != null && previous.dimension().equals(payload.dimension()) && payload.dimension().equals(currentDimension);
        List<WaveRegion> regions = same ? intern(payload.regions(), previous.regions()) : payload.regions();
        snapshot = new Snapshot(payload.dimension(), regions);
        SeaSurface.setClientRegions(payload.dimension(), regions);
        updates++;
        if (same) {
            // A natural sea moving with a player changes coverage only along a crescent: rebuild those chunk sections and
            // hand the cached tiles and foam on (rebuilding both whole regions and every tile stuttered at each move).
            List<WaveRegion> z = zones;
            migrate(concat(previous.regions(), z), concat(regions, z));
            remeshChanges(previous.regions(), regions);
        } else {
            WaveSurfaceRenderer.invalidate();
            if (previous.dimension() != null && previous.dimension().equals(currentDimension)) { previous.regions().forEach(ClientWaveRegions::remesh); }
            regions.forEach(ClientWaveRegions::remesh);
        }
        acceptNanos += System.nanoTime() - started;
    }

    /**
     * The received regions with each one equal to a region already held replaced by that object (taking the new
     * height): caches keyed by region then match by identity, not by comparing whole depth surveys every lookup.
     */
    private static List<WaveRegion> intern(List<WaveRegion> received, List<WaveRegion> held) {
        List<WaveRegion> out = new java.util.ArrayList<>(received.size());
        for (WaveRegion r : received) {
            int i = held.indexOf(r);
            if (i >= 0 && held.get(i) != r) { held.get(i).setScale(r.scale()); out.add(held.get(i)); } else { out.add(r); }
        }
        return List.copyOf(out);
    }

    /** Region updates received, chunk sections queued for rebuilding and time spent accepting updates (ns), for AutoShot. */
    static int updates, remeshed;
    static long acceptNanos;

    private static void migrate(List<WaveRegion> before, List<WaveRegion> after) {
        WaveSurfaceRenderer.migrate(before, after);
        SeaFoam.migrate(before, after);
    }

    private static List<WaveRegion> concat(List<WaveRegion> a, List<WaveRegion> b) {
        if (b.isEmpty()) { return a; }
        List<WaveRegion> all = new java.util.ArrayList<>(a);
        all.addAll(b);
        return all;
    }

    /**
     * Rebuilds the chunk sections at a still level where some column's coverage (inside a region of that still level)
     * differs between the two region lists; regions in both lists change nothing.
     */
    private static void remeshChanges(List<WaveRegion> before, List<WaveRegion> after) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.levelRenderer == null || minecraft.level == null) { return; }
        List<WaveRegion> changed = new java.util.ArrayList<>();
        for (WaveRegion r : before) { if (!after.contains(r)) { changed.add(r); } }
        for (WaveRegion r : after) { if (!before.contains(r)) { changed.add(r); } }
        java.util.Set<Integer> levels = new java.util.HashSet<>();
        for (WaveRegion r : changed) { levels.add(Mth.floor(r.stillWaterY() - 1e-6)); }
        for (int y : levels) {
            double minX = Double.MAX_VALUE, minZ = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxZ = -Double.MAX_VALUE;
            for (WaveRegion r : changed) {
                if (Mth.floor(r.stillWaterY() - 1e-6) != y) { continue; }
                minX = Math.min(minX, r.centerX() - r.radiusM()); maxX = Math.max(maxX, r.centerX() + r.radiusM());
                minZ = Math.min(minZ, r.centerZ() - r.radiusM()); maxZ = Math.max(maxZ, r.centerZ() + r.radiusM());
            }
            for (int cx = Mth.floor(minX) >> 4; cx <= Mth.floor(maxX) >> 4; cx++) for (int cz = Mth.floor(minZ) >> 4; cz <= Mth.floor(maxZ) >> 4; cz++) {
                boolean dirty = false;
                for (int i = 0; i < 16 && !dirty; i++) for (int k = 0; k < 16 && !dirty; k++) {
                    double x = (cx << 4) + i + 0.5, z = (cz << 4) + k + 0.5;
                    dirty = covered(before, y, x, z) != covered(after, y, x, z);
                }
                if (dirty) { queue(cx, y, cz); }
            }
        }
    }

    private static boolean covered(List<WaveRegion> regions, int y, double x, double z) {
        for (WaveRegion r : regions) { if (Mth.floor(r.stillWaterY() - 1e-6) == y && r.contains(x, z)) { return true; } }
        return false;
    }

    /** The present sea heights of the dimension's regions, in list order. */
    static void scales(WaveNetwork.ScalesPayload payload) {
        Snapshot s = snapshot;
        if (s.dimension() == null || !s.dimension().equals(payload.dimension())) { return; }
        for (int i = 0; i < Math.min(s.regions().size(), payload.scales().size()); i++) { s.regions().get(i).setScale(payload.scales().get(i)); }
    }

    private static void remesh(WaveRegion region) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.levelRenderer == null || minecraft.level == null) { return; }
        int y = Mth.floor(region.stillWaterY() - 1e-6);
        for (int cx = Mth.floor(region.centerX() - region.radiusM()) >> 4; cx <= Mth.floor(region.centerX() + region.radiusM()) >> 4; cx++) {
            for (int cz = Mth.floor(region.centerZ() - region.radiusM()) >> 4; cz <= Mth.floor(region.centerZ() + region.radiusM()) >> 4; cz++) { queue(cx, y, cz); }
        }
    }

    /**
     * Chunk sections waiting to be rebuilt (SectionPos longs). The hidden top faces are those of the blocks at the still
     * level, so only the section holding that level is rebuilt (LevelRenderer.setBlocksDirty would add a block of margin
     * on every side, up to 18 sections per chunk). Client thread only.
     */
    private static final java.util.Set<Long> PENDING = new java.util.HashSet<>();
    /**
     * Sections handed to the game per client tick, nearest the camera first: the game copies each section's blocks on
     * the render thread to rebuild it, about 0.2 ms each, and a moved natural sea changes a hundred or more at once.
     */
    private static final int REMESH_PER_TICK = 32;

    private static void queue(int chunkX, int y, int chunkZ) {
        if (PENDING.add(SectionPos.asLong(chunkX, SectionPos.blockToSectionCoord(y), chunkZ))) { remeshed++; }
    }

    private static void drain(Minecraft minecraft) {
        if (PENDING.isEmpty()) { return; }
        if (minecraft.levelRenderer == null || minecraft.level == null) { PENDING.clear(); return; }
        net.minecraft.world.phys.Vec3 eye = minecraft.gameRenderer.getMainCamera().getPosition();
        List<Long> order = new java.util.ArrayList<>(PENDING);
        if (order.size() > REMESH_PER_TICK) {
            order.sort(java.util.Comparator.comparingDouble(s -> {
                double dx = SectionPos.sectionToBlockCoord(SectionPos.x(s), 8) - eye.x, dz = SectionPos.sectionToBlockCoord(SectionPos.z(s), 8) - eye.z;
                return dx * dx + dz * dz;
            }));
        }
        for (int i = 0; i < Math.min(REMESH_PER_TICK, order.size()); i++) {
            long s = order.get(i);
            minecraft.levelRenderer.setSectionDirty(SectionPos.x(s), SectionPos.y(s), SectionPos.z(s));
            PENDING.remove(s);
        }
    }

    @EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
    public static final class Setup {
        private Setup() {}
        @SubscribeEvent public static void clientSetup(FMLClientSetupEvent event) {
            WaveNetwork.setClientHandler(ClientWaveRegions::accept);
            WaveNetwork.setClientScales(ClientWaveRegions::scales);
        }
    }

    @EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
    public static final class Events {
        private Events() {}
        @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
            Minecraft minecraft = Minecraft.getInstance();
            currentDimension = minecraft.level == null ? null : minecraft.level.dimension().location();
            drain(minecraft);
            // Wake zones appear, move in 16 m steps and vanish with the boats: rebuild the chunks whose top faces change.
            List<WaveRegion> next = minecraft.level == null ? List.of() : dev.mcmod.aerocore.neoforge.probe.Wakes.zones(minecraft.level);
            List<WaveRegion> previous = zones;
            if (!next.equals(previous)) {
                zones = next;
                Snapshot s = snapshot;
                List<WaveRegion> persistent = s.dimension() != null && s.dimension().equals(currentDimension) ? s.regions() : List.of();
                migrate(concat(persistent, previous), concat(persistent, next));
                remeshChanges(previous, next);
            }
        }
        @SubscribeEvent public static void logout(ClientPlayerNetworkEvent.LoggingOut event) {
            snapshot = EMPTY; currentDimension = null; zones = List.of(); PENDING.clear(); SeaSurface.clearClient(); WaveSurfaceRenderer.invalidate();
            dev.mcmod.aerocore.neoforge.probe.WorldWind.clearClient();
        }
    }
}
