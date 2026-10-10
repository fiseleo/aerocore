package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CoastalWaves;
import dev.mcmod.aerocore.api.SeaSpectrum;
import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.api.SurfaceWind;
import dev.mcmod.aerocore.api.WaveField;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.tags.BiomeTags;
import net.minecraft.tags.FluidTags;
import net.minecraft.util.Mth;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.tick.LevelTickEvent;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Waves on natural water by themselves: around every player near open water at the level's sea level, a natural sea
 * region (WaveRegion.natural, not saved) of up to 128 m radius, kept clear of the commanded regions and of each other,
 * replaced by a new one when the player has moved 64 m off its centre. Each is a wind sea of the level's wind
 * (WorldWind): the bed and banks are surveyed, every water point's straight-line fetch measured upwind to land (and on
 * beyond the survey over loaded water; where that runs into unloaded chunks, unlimited over an ocean biome, else what was
 * measured), and the sea synthesised for the strongest weather wind (thunder, 15 m/s at 10 m over open terrain, over
 * water by SurfaceWind.overWater) at the fetch near the player, so a pond gets short chop and the ocean long waves;
 * each point then grows with its own fetch (CoastalWaves.WindSea), and SeaState keeps the sea's height at what the
 * present wind raises (its CEM growth and decay times). The spectrum is chosen from classes - fetch in powers of two from
 * 16 m to 16 km, depth 1, 2, 4 ... 32 m or deep, wind direction in 15 degree sectors - each synthesised once with a
 * seed from the world seed and the class, so moving on within the same water keeps the same waves; a new direction
 * sector is taken only when the wind has turned 30 degrees from the sea's. Off in GameTests by default;
 * /aerocore waves natural switches it per level.
 * <p>
 * Not modelled: swell from distant storms, the sea's own period lengthening with fetch inside one region (one spectrum
 * per region, grown in height only), and terrain sheltering the wind over a lake (the open-water wind is used).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class NaturalSeas {
    private static final int CHECK_TICKS = 40, SEA_COMPONENTS = 32;
    private static final double MIN_RADIUS = 48, RECENTRE = 64, KEEP = 64, SEARCH = 32, NEAR = 32, MAX_ABOVE = 64;
    /** Deepest bed a natural survey looks for, m: deeper water is the sea's own depth (its waves barely feel it). */
    private static final int SURVEY_DEPTH = 32;
    private static final double MAX_FETCH = 16000, TURN = Math.toRadians(30), SECTOR = Math.toRadians(15), SPREADING = 4;
    /** The design wind: the thunder speed of the weather rule, at 10 m over open terrain. */
    private static final double DESIGN_WIND = WaveRegion.Wind.WEATHER_DEFAULTS[2];
    private static final Map<String, WaveField> SPECTRA = new LinkedHashMap<>(16, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<String, WaveField> eldest) { return size() > 64; }
    };

    private NaturalSeas() {}

    /** The 10 m wind over open water that natural seas grow and calm with now, m/s. */
    public static double seaWind(ServerLevel level) { return SurfaceWind.overWater(WorldWind.reference(level).speedMps()); }

    @SubscribeEvent public static void tick(LevelTickEvent.Post event) {
        if (!(event.getLevel() instanceof ServerLevel level) || level.getGameTime() % CHECK_TICKS != 0) { return; }
        WaveRegions seas = WaveRegions.of(level);
        List<WaveRegion> natural = seas.regions().stream().filter(WaveRegion::natural).toList();
        if (!seas.naturalSeas() || !level.dimensionType().hasSkyLight() || level.dimensionType().hasCeiling()) {
            if (!natural.isEmpty()) { seas.setNatural(level, List.of()); }
            return;
        }
        WorldWind.Reference wind = WorldWind.reference(level);
        List<WaveRegion> keep = new ArrayList<>();
        for (WaveRegion r : natural) {
            boolean near = level.players().stream().anyMatch(p -> Math.hypot(p.getX() - r.centerX(), p.getZ() - r.centerZ()) < r.radiusM() + KEEP);
            boolean aligned = wind.speedMps() < 1 || r.sea() == null || Math.abs(Mth.wrapDegrees(Math.toDegrees(wind.directionRadians() - r.sea().directionRadians()))) <= Math.toDegrees(TURN);
            if (near && aligned) { keep.add(r); }
        }
        for (ServerPlayer player : level.players()) {
            double px = player.getX(), pz = player.getZ();
            boolean inside = false;
            WaveRegion own = null;
            for (WaveRegion r : seas.regions()) {
                if (!r.natural() || keep.contains(r)) {
                    double d = Math.hypot(px - r.centerX(), pz - r.centerZ());
                    if (d < r.radiusM() - RECENTRE) { inside = true; break; }
                    if (r.natural() && d < r.radiusM()) { own = r; }
                }
            }
            if (inside) { continue; }
            double still = stillSurface(level, player);
            if (Double.isNaN(still)) { continue; }
            if (own != null) { keep.remove(own); }
            WaveRegion made = build(level, seas, keep, px, pz, still, wind, own);
            if (made != null) { keep.add(made); } else if (own != null) { keep.add(own); }
        }
        seas.setNatural(level, keep);
    }

    /** The still surface of open water at the sea level within SEARCH of the player, or NaN. */
    private static double stillSurface(ServerLevel level, ServerPlayer player) {
        int y = level.getSeaLevel() - 1;
        if (Math.abs(player.getY() - y) > MAX_ABOVE) { return Double.NaN; }
        WorldWaterSampler sampler = new WorldWaterSampler(level);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        for (double dx = -SEARCH; dx <= SEARCH; dx += 8) for (double dz = -SEARCH; dz <= SEARCH; dz += 8) {
            pos.set(Mth.floor(player.getX() + dx), y, Mth.floor(player.getZ() + dz));
            if (!level.hasChunkAt(pos) || !level.getFluidState(pos).is(FluidTags.WATER) || !level.getFluidState(pos).isSource()) { continue; }
            if (!level.canSeeSky(pos.above())) { continue; }
            double surface = sampler.stillSurface(pos.immutable());
            if (!Double.isNaN(surface)) { return surface; }
        }
        return Double.NaN;
    }

    private static WaveRegion build(ServerLevel level, WaveRegions seas, List<WaveRegion> keep, double px, double pz, double still,
                                    WorldWind.Reference wind, WaveRegion old) {
        double cx = Math.round(px / 8) * 8.0, cz = Math.round(pz / 8) * 8.0, radius = WaveRegion.MAX_RADIUS;
        for (WaveRegion r : seas.regions()) {
            if (r == old || (r.natural() && !keep.contains(r))) { continue; }
            double d = Math.hypot(cx - r.centerX(), cz - r.centerZ());
            radius = Math.min(radius, d - r.radiusM() - 1);
        }
        if (radius < MIN_RADIUS) { return null; }
        // The direction: the old sea's while the wind stays within 30 degrees of it, else the wind's 15 degree sector.
        double direction = old != null && old.sea() != null
                && Math.abs(Mth.wrapDegrees(Math.toDegrees(wind.directionRadians() - old.sea().directionRadians()))) <= Math.toDegrees(TURN)
                ? old.sea().directionRadians() : Math.round(wind.directionRadians() / SECTOR) * SECTOR;
        WaveRegion.Survey survey = WaveRegions.survey(level, cx, cz, radius, still, SURVEY_DEPTH);
        double beyond = openWaterBeyond(level, survey, px, pz, still, direction);
        float[] bed = survey.depths(1e4);
        double[] fetch = CoastalWaves.fetches(survey.spacing(), survey.nx(), survey.nz(), bed, direction, beyond);
        double rep = near(survey, fetch, px, pz, NEAR), depth = nearDepth(survey, px, pz, 16);
        if (Double.isNaN(rep)) { return null; }
        int fetchClass = (int) Math.clamp(Math.round(Math.log(Math.max(rep, 1)) / Math.log(2)), 4, 14);
        int depthClass = Double.isFinite(depth) && depth < 48 ? (int) Math.clamp(Math.round(Math.log(Math.max(depth, 1)) / Math.log(2)), 0, 5) : 6;
        double classFetch = Math.pow(2, fetchClass), classDepth = depthClass == 6 ? Double.POSITIVE_INFINITY : Math.pow(2, depthClass);
        double design = SurfaceWind.overWater(DESIGN_WIND);
        double[] hsTp = SeaWind.sea(design, classFetch, classDepth);
        if (!(hsTp[0] > 0.005)) { return null; }
        double gamma = SeaSpectrum.recommendedGamma(hsTp[0], hsTp[1]);
        int sector = Math.floorMod((int) Math.round(direction / SECTOR), (int) Math.round(2 * Math.PI / SECTOR));
        String key = level.getSeed() + "/" + fetchClass + "/" + depthClass + "/" + sector + "/" + Double.doubleToLongBits(direction);
        WaveField waves;
        synchronized (SPECTRA) {
            waves = SPECTRA.computeIfAbsent(key, k -> SeaSpectrum.synthesize(hsTp[0], hsTp[1], gamma, direction, SPREADING, classDepth, SEA_COMPONENTS, k.hashCode() * 6364136223846793005L + 1442695040888963407L));
        }
        WaveRegion.Sea sea = new WaveRegion.Sea(hsTp[0], hsTp[1], gamma, direction, SPREADING, design, beyond, classFetch);
        WaveRegion region;
        try {
            region = new WaveRegion(cx, cz, radius, still, waves, sea, survey, WaveRegion.Wind.CALM).markNatural();
        } catch (IllegalArgumentException invalid) {
            return null;
        }
        double target = SeaState.target(sea, seaWind(level), classDepth);
        region.setScale(old != null && old.waves().equals(waves) ? old.scale() : target);
        return region;
    }

    /** Median fetch over the water nodes within reach of (x, z), or over all water nodes; NaN without water. */
    private static double near(WaveRegion.Survey survey, double[] fetch, double x, double z, double reach) {
        List<Double> close = new ArrayList<>(), all = new ArrayList<>();
        for (int i = 0; i < survey.nx(); i++) for (int j = 0; j < survey.nz(); j++) {
            double f = fetch[i * survey.nz() + j];
            if (Double.isNaN(f)) { continue; }
            all.add(f);
            if (Math.hypot(survey.x0() + i * survey.spacing() - x, survey.z0() + j * survey.spacing() - z) <= reach) { close.add(f); }
        }
        List<Double> use = close.isEmpty() ? all : close;
        if (use.isEmpty()) { return Double.NaN; }
        double[] sorted = use.stream().mapToDouble(Double::doubleValue).sorted().toArray();
        return sorted[sorted.length / 2];
    }

    /** Median water depth within reach of (x, z), m (infinite where deeper than the survey reaches); NaN without water. */
    private static double nearDepth(WaveRegion.Survey survey, double x, double z, double reach) {
        List<Double> depths = new ArrayList<>();
        for (int i = 0; i < survey.nx(); i++) for (int j = 0; j < survey.nz(); j++) {
            if (Math.hypot(survey.x0() + i * survey.spacing() - x, survey.z0() + j * survey.spacing() - z) > reach) { continue; }
            short d = survey.depthDm(i * survey.nz() + j);
            if (d == WaveRegion.Survey.LAND) { continue; }
            depths.add(d == WaveRegion.Survey.DEEP ? Double.POSITIVE_INFINITY : d / 10.0);
        }
        if (depths.isEmpty()) { return Double.NaN; }
        double[] sorted = depths.stream().mapToDouble(Double::doubleValue).sorted().toArray();
        return sorted[sorted.length / 2];
    }

    /**
     * Open water upwind of the survey along the wind line through (x, z), m: over water at the still level to land;
     * where it runs into unloaded chunks, unlimited (MAX_FETCH) if the last loaded water is an ocean biome, else what was
     * measured; at most MAX_FETCH.
     */
    private static double openWaterBeyond(ServerLevel level, WaveRegion.Survey survey, double x, double z, double stillY, double heading) {
        double ex = -Math.cos(heading), ez = -Math.sin(heading);
        double maxX = survey.x0() + (survey.nx() - 1) * survey.spacing(), maxZ = survey.z0() + (survey.nz() - 1) * survey.spacing();
        double t = 0;
        while (x + t * ex >= survey.x0() && x + t * ex <= maxX && z + t * ez >= survey.z0() && z + t * ez <= maxZ) { t += 0.5; }
        int y = Mth.floor(stillY - 1e-6);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        BlockPos last = null;
        for (double d = 0; d < MAX_FETCH; d += 2) {
            pos.set(Mth.floor(x + (t + d) * ex), y, Mth.floor(z + (t + d) * ez));
            if (!level.hasChunkAt(pos)) {
                return last != null && level.getBiome(last).is(BiomeTags.IS_OCEAN) ? MAX_FETCH : d;
            }
            if (!level.getFluidState(pos).is(FluidTags.WATER)) { return d; }
            last = pos.immutable();
        }
        return MAX_FETCH;
    }

    /** A natural sea at (x, z) on water of this still level, as the tick would make it for a player there (tests). */
    static WaveRegion buildAt(ServerLevel level, double x, double z, double still) {
        return build(level, WaveRegions.of(level), List.of(), x, z, still, WorldWind.reference(level), null);
    }

    /** Spectra cached for the classes, for tests. */
    static int cachedSpectra() { synchronized (SPECTRA) { return SPECTRA.size(); } }
}
