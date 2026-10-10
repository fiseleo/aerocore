package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.server.level.ServerLevel;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.tick.LevelTickEvent;

import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Wind seas growing and calming with the wind over time (server authority, synced to players and saved). A wind sea's
 * height relative to as synthesised (WaveRegion.scale) relaxes toward Hs(U, F) / Hs(U_sea, F): U the region's mean wind
 * now (weather included), U_sea the wind that raised it, F its representative fetch (SeaWind.sea, same depth and caps).
 * The e-folding time is a third of the CEM's time to fetch-limited growth (II-2-35, SeaWind.growthTime) at the
 * present wind, so the sea is within 5 % of its target after that time; while calming the same time at the sea's own
 * wind is used (the decay of a wind sea has no comparably simple measured law; an assumption). A small lake follows a
 * storm within minutes, open sea over hours. The spectrum's shape stays the synthesised one (its peak period does not
 * shift), and the sea never exceeds its synthesised height: the windsea command's wind is the strongest it can raise.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class SeaState {
    private static final int SYNC_TICKS = 20;
    private static final double SYNC_CHANGE = 0.005;
    private static final Map<ServerLevel, double[]> SENT = new WeakHashMap<>();

    private SeaState() {}

    /** The height (relative to as synthesised) the present wind raises on a wind sea, at most 1. */
    static double target(WaveRegion.Sea sea, double wind, double depthM) {
        double fetch = sea.fetchM(), design = SeaWind.sea(sea.windMps(), fetch, depthM)[0];
        if (!(design > 0)) { return 1; }
        return Math.min(1, SeaWind.sea(Math.clamp(wind, 0, SeaWind.MAX_WIND), fetch, depthM)[0] / design);
    }

    /** The present-wind target and next scale of a wind sea after dt seconds. */
    static double step(WaveRegion.Sea sea, double wind, double scale, double depthM, double dt) {
        double fetch = sea.fetchM(), design = SeaWind.sea(sea.windMps(), fetch, depthM)[0];
        if (!(design > 0)) { return scale; }
        double target = target(sea, wind, depthM);
        double tau = SeaWind.growthTime(target >= scale ? Math.max(wind, 0.5) : sea.windMps(), fetch) / 3;
        return target + (scale - target) * Math.exp(-dt / tau);
    }

    @SubscribeEvent public static void tick(LevelTickEvent.Post event) {
        if (!(event.getLevel() instanceof ServerLevel level)) { return; }
        WaveRegions saved = WaveRegions.of(level);
        List<WaveRegion> regions = saved.regions();
        if (regions.isEmpty()) { return; }
        boolean changed = false;
        double[] sent = SENT.get(level);
        if (sent == null || sent.length != regions.size()) { sent = new double[regions.size()]; java.util.Arrays.fill(sent, Double.NaN); SENT.put(level, sent); }
        for (int i = 0; i < regions.size(); i++) {
            WaveRegion region = regions.get(i);
            WaveRegion.Sea sea = region.sea();
            if (sea == null || !sea.windSea() || !Double.isFinite(sea.fetchM()) || region.waves().isEmpty()) { continue; }
            // Natural seas grow with the level's wind over open water; commanded ones with their own (none: calm).
            double wind = region.natural() ? NaturalSeas.seaWind(level) : region.wind().calm() ? 0 : region.wind().mean(level);
            region.setScale(step(sea, wind, region.scale(), region.waves().components().getFirst().depthM(), MinecraftUnits.TICK_SECONDS));
            if (!(Math.abs(region.scale() - sent[i]) < SYNC_CHANGE)) { changed = true; }
        }
        if (changed && level.getGameTime() % SYNC_TICKS == 0) {
            for (int i = 0; i < regions.size(); i++) { sent[i] = regions.get(i).scale(); }
            WaveNetwork.syncScales(level);
            saved.setDirty();
        }
    }
}
