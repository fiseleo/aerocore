package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.SeaSurface;
import dev.mcmod.aerocore.neoforge.probe.WaveNetwork;
import dev.mcmod.aerocore.neoforge.probe.WaveRegion;
import net.minecraft.client.Minecraft;
import net.minecraft.core.BlockPos;
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
        Snapshot previous = snapshot;
        snapshot = new Snapshot(payload.dimension(), payload.regions());
        SeaSurface.setClientRegions(payload.dimension(), payload.regions());
        WaveSurfaceRenderer.invalidate();
        currentDimension = Minecraft.getInstance().level == null ? null : Minecraft.getInstance().level.dimension().location();
        // Rebuild the chunk sections whose surface faces appear or disappear.
        if (previous.dimension() != null && previous.dimension().equals(currentDimension)) { previous.regions().forEach(ClientWaveRegions::remesh); }
        payload.regions().forEach(ClientWaveRegions::remesh);
    }

    private static void remesh(WaveRegion region) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.levelRenderer == null || minecraft.level == null) { return; }
        int y = Mth.floor(region.stillWaterY() - 1e-6);
        minecraft.levelRenderer.setBlocksDirty(Mth.floor(region.centerX() - region.radiusM()), y - 1, Mth.floor(region.centerZ() - region.radiusM()),
                Mth.floor(region.centerX() + region.radiusM()), y + 1, Mth.floor(region.centerZ() + region.radiusM()));
    }

    @EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
    public static final class Setup {
        private Setup() {}
        @SubscribeEvent public static void clientSetup(FMLClientSetupEvent event) { WaveNetwork.setClientHandler(ClientWaveRegions::accept); }
    }

    @EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
    public static final class Events {
        private Events() {}
        @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
            Minecraft minecraft = Minecraft.getInstance();
            currentDimension = minecraft.level == null ? null : minecraft.level.dimension().location();
            // Wake zones appear, move in 16 m steps and vanish with the boats: rebuild the chunks whose top faces change.
            List<WaveRegion> next = minecraft.level == null ? List.of() : dev.mcmod.aerocore.neoforge.probe.Wakes.zones(minecraft.level);
            List<WaveRegion> previous = zones;
            if (!next.equals(previous)) {
                zones = next;
                for (WaveRegion zone : previous) { if (!next.contains(zone)) { remesh(zone); } }
                for (WaveRegion zone : next) { if (!previous.contains(zone)) { remesh(zone); } }
                WaveSurfaceRenderer.invalidate();
            }
        }
        @SubscribeEvent public static void logout(ClientPlayerNetworkEvent.LoggingOut event) {
            snapshot = EMPTY; currentDimension = null; zones = List.of(); SeaSurface.clearClient(); WaveSurfaceRenderer.invalidate();
        }
    }
}
