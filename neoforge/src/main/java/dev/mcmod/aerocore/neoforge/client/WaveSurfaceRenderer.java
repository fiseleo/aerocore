package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.platform.GlStateManager;
import com.mojang.blaze3d.shaders.Uniform;
import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.blaze3d.vertex.VertexFormat;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.CoastalWaves;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.WaterOptics;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.probe.SeaSurface;
import dev.mcmod.aerocore.neoforge.probe.Wakes;
import dev.mcmod.aerocore.neoforge.probe.WaveRegion;
import net.minecraft.client.Camera;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderStateShard;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.ShaderInstance;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.util.Mth;
import net.minecraft.world.level.material.FluidState;
import net.minecraft.world.level.material.FogType;
import net.minecraft.world.level.material.Fluids;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RegisterShadersEvent;
import net.neoforged.neoforge.client.event.RenderLevelStageEvent;
import net.neoforged.neoforge.client.extensions.common.IClientFluidTypeExtensions;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL20;
import org.slf4j.Logger;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.WeakHashMap;
import java.util.stream.IntStream;

/**
 * Draws each sea region's surface in place of the vanilla water top faces that ClientWaveRegions hides, from the same
 * second-order field (CompiledWaves, over the region's coast where it has one), still level, wakes and time base as
 * the server physics (game time x 0.05 s), whitened by SeaFoam's foam. The whole region is drawn out to
 * the render distance in 16 x 16 tiles. Vertex spacing grows with distance from the camera (0.25 / 0.5 m near, 1 m to
 * 48 m, 2 m beyond, where full 2 x 2 cells merge into one quad); heights are always the exact field at the vertices,
 * and a finer tile's edge vertices are moved onto its coarser neighbour's edge so the levels meet without cracks.
 * Which columns are sea surface (plus their tint and light) is cached per tile and refreshed every half second.
 * Components shorter than four vertex spacings are left out of a tile's mesh (WaveRegion.drawnWaves): it would alias
 * them into false long patterns.
 * <p>
 * Those waves are drawn per pixel instead, by the sea shader (shaders/core/rendertype_aerocore_sea): it adds their
 * slopes to the mesh's, and beyond them the wind ripples down to the capillary limit (WaterOptics.ripples: Cox and
 * Munk's slope variance less the spectrum's), each wave fading out where it spans fewer than about five pixels into
 * the variance of the sun's glitter. So a small lake's centimetre waves, which no mesh can carry, still shape its
 * reflections, and the glitter turns from single sparkles near the eye into Cox and Munk's mean path far away. The
 * shader takes the spectrum and ripples as uniforms (phases at the region origin now) and per vertex: the foam, the
 * light returned by the bed (exp(-2Kd)), the coast's gain on the waves the mesh leaves out (CoastalWaves envelopes,
 * RMS), the wind sheltering of the ripples (WaveRegion.shelter), and the elevation of the surveyed terrain along the
 * reflected ray (so banks and trees darken the reflection where they would be seen in it).
 * <p>
 * Each frame the visible tiles' surfaces are computed in parallel (the common fork-join pool; the field evaluation
 * is thread-confined), then their vertices are emitted on the render thread, one batch per region.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class WaveSurfaceRenderer {
    private static final Logger LOGGER = LogUtils.getLogger();
    private static final int TILE = 16, CACHE_TICKS = 10, FORGET_TICKS = 200, LATTICE = TILE / 2 + 1;
    /**
     * Tile refreshes per frame: cached tiles going stale, and tiles seen for the first time (drawn once refreshed). A
     * refresh reads 256 columns of blocks and light, about 0.3 ms; a natural sea moving on brings 80 or more new tiles
     * at once, which refreshed together stalled a frame by 60 to 80 ms.
     */
    private static final int REFRESH_PER_FRAME = 8, NEW_PER_FRAME = 16;
    /**
     * Water tint lattice, blocks: the biome colour is read every TINT_STEP blocks and interpolated between, once per
     * tile (biomes do not change). The game's blend already smooths it over 5 or more blocks, and its tint cache holds
     * 256 chunks, fewer than a normal view, so reading every column missed it and cost half of each refresh.
     */
    private static final int TINT_STEP = 4, TINT_POINTS = TILE / TINT_STEP + 1;
    /** Ripple waves drawn beyond a spectrum, and the most waves the shader takes (WaveRegion.MAX_COMPONENTS + RIPPLES). */
    private static final int RIPPLES = 32, MAX_WAVES = 96;
    /** Below this wind (m/s) the surface is drawn without ripples (a glassy calm). */
    private static final double RIPPLE_WIND = 0.5;
    /** Terrain seen in a reflection: the biome foliage colour times this and the daylight, a display choice. */
    private static final double SHORE_SHADE = 0.3;
    /** How far a reflected ray is followed over the surveyed terrain, m. */
    private static final double REFLECTION_REACH = 96;
    /** Deepest water column the depth scan reports, blocks (beyond it the bed no longer shows). */
    private static final int DEPTH_SCAN = 16;
    /** Per vertex: foam, light returned by the bed, spectrum gain, ripple gain, terrain horizon (rad), slope x, slope z. */
    private static final int ATTRIBUTES = 7;

    private static volatile ShaderInstance shader;
    private static final RenderType SEA = RenderType.create("aerocore_sea", DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 786432, false, true,
            RenderType.CompositeState.builder()
                    .setShaderState(new RenderStateShard.ShaderStateShard(() -> shader))
                    .setTransparencyState(RenderStateShard.TRANSLUCENT_TRANSPARENCY)
                    .setLightmapState(RenderStateShard.LIGHTMAP)
                    .setOutputState(RenderStateShard.TRANSLUCENT_TARGET)
                    .createCompositeState(false));
    private static final FloatBuffer WAVES = BufferUtils.createFloatBuffer(4 * MAX_WAVES);

    private record TileKey(WaveRegion region, int x, int z) {}
    private static final class Tile {
        long refreshed = Long.MIN_VALUE, used;
        boolean any, tinted;
        final boolean[] surface = new boolean[TILE * TILE];
        final int[] tint = new int[TILE * TILE], light = new int[TILE * TILE];
        /** Water depth below the still level per column, m (0 off the sea surface). */
        final float[] depth = new float[TILE * TILE];
    }
    private static final Map<TileKey, Tile> TILES = new HashMap<>();
    private static volatile boolean invalidated;

    /**
     * One region's waves for this frame: the spectrum's current amplitudes and wavenumbers, the wind of its ripples
     * (m/s), the slope variance the spectrum carries and the ripples carry (open water), the ripples
     * (WaterOptics.ripples rows), and the surveyed terrain tops above the still level (m, or null).
     */
    private record Look(double[] amplitude, double[] wavenumber, double wind, double resolvedVariance, double rippleVariance,
                        double[][] ripples, float[] tops, double topMax) {}
    private record Ripples(double wind, double resolvedVariance, double kStart, double[][] rows) {}
    private static final Map<WaveRegion, Ripples> RIPPLE_CACHE = new WeakHashMap<>();
    private static final Map<WaveRegion.Survey, float[]> TOPS = new WeakHashMap<>();

    /** One visible tile's work this frame: its inputs, and the vertex fields a worker fills. Reused across frames. */
    private static final class Job {
        WaveRegion region;
        Look look;
        Tile tile;
        int tx, tz, points;
        /** Mesh spacing, and the coarser spacing the ambient sea is evaluated at (then interpolated) when a wake needs the finer one. */
        double s, ambient, eyeX, eyeY, eyeZ;
        double[][] coarseGrids = new double[6][0];
        boolean back, corrected;
        final double[] neighbours = new double[4];
        double[] height = new double[0], slopeX = new double[0], slopeZ = new double[0], height2 = new double[0], slope2X = new double[0], slope2Z = new double[0];
        float[] attributes = new float[0];
        final double[] latticeH = new double[LATTICE * LATTICE], latticeX = new double[LATTICE * LATTICE], latticeZ = new double[LATTICE * LATTICE];
        final double[] latticeGain = new double[LATTICE * LATTICE], latticeRipple = new double[LATTICE * LATTICE], latticeHorizon = new double[LATTICE * LATTICE];
        /** Mean foam on the lattice, drawn where SeaFoam computes no patch (beyond its range). */
        final double[] latticeFoam = new double[LATTICE * LATTICE];
        ClientLevel level;
        final double[] coarseWind = new double[9];
        double[][] envelope = new double[6][0], centreEnvelope = new double[2][0];

        void ensure(int count) {
            if (height.length >= count) { return; }
            height = new double[count]; slopeX = new double[count]; slopeZ = new double[count];
            height2 = new double[count]; slope2X = new double[count]; slope2Z = new double[count];
            attributes = new float[count * ATTRIBUTES];
        }
    }
    private static final List<Job> JOBS = new ArrayList<>();
    /** Tiles nearer than this get the full second-order surface; farther ones its cheap part (a drawing budget). */
    private static final double CORRECTION_RANGE = 24;
    /** How much an envelope or the depth may change across a tile for the tile to take its centre's waves (fraction). */
    private static final double UNIFORM = 0.02;

    /** Development view of the sea shader's inputs (AutoShot's debug step; 0 the picture, see rendertype_aerocore_sea.fsh). */
    public static volatile int debugView;
    /** Render-thread time spent here (ns), frames and tiles drawn, since the last read (AutoShot's fps step). */
    private static long spentNanos, computeNanos, frames, tilesDrawn;
    /** Render-thread time of the last frame's sea, ns, and of its tile refreshes (how many), looks and compute (AutoShot's slow-frame log). */
    static long lastDrawNanos;
    static final long[] phase = new long[3];
    static int refreshes;
    /** Worker time per part of compute (ns): mesh, correction, wakes, lattice, attributes; for timing(). */
    private static final java.util.concurrent.atomic.AtomicLongArray PARTS = new java.util.concurrent.atomic.AtomicLongArray(9);

    /** The mean milliseconds per frame spent drawing the sea (total and in the parallel compute) and tiles per frame since the last call. */
    public static String timing() {
        String out = frames == 0 ? "no frames" : String.format("%.2f ms per frame (compute %.2f ms; worker ms: mesh %.2f, correction %.2f, wakes %.2f, lattice %.2f, attributes %.2f), %.0f tiles",
                spentNanos / 1e6 / frames, computeNanos / 1e6 / frames, PARTS.get(0) / 1e6 / frames, PARTS.get(1) / 1e6 / frames, PARTS.get(2) / 1e6 / frames,
                PARTS.get(3) / 1e6 / frames, PARTS.get(4) / 1e6 / frames, (double) tilesDrawn / frames)
                + String.format(" (coastal %.0f, uniform %.0f, vertices %.0f, components drawn %.0f per tile)", (double) PARTS.get(5) / frames, (double) PARTS.get(6) / frames,
                (double) PARTS.get(7) / Math.max(1, tilesDrawn), (double) PARTS.get(8) / Math.max(1, tilesDrawn));
        spentNanos = computeNanos = frames = tilesDrawn = 0;
        for (int i = 0; i < 9; i++) { PARTS.set(i, 0); }
        return out;
    }

    private WaveSurfaceRenderer() {}

    /** Drops the column cache; safe to call from any thread. */
    public static void invalidate() { invalidated = true; }

    private record Migration(List<WaveRegion> before, List<WaveRegion> after) {}
    private static final java.util.Queue<Migration> MIGRATIONS = new java.util.concurrent.ConcurrentLinkedQueue<>();

    /**
     * The region list changed within a dimension (safe from any thread): at the next frame, tiles of regions in both
     * lists stay, tiles of a region that went are handed to a new region of the same still level covering them (a natural
     * sea moving with its player) and refreshed within the normal budget; the rest are dropped.
     */
    public static void migrate(List<WaveRegion> before, List<WaveRegion> after) { MIGRATIONS.add(new Migration(List.copyOf(before), List.copyOf(after))); }

    private static void applyMigration(Migration m, long tick) {
        Map<TileKey, Tile> next = new HashMap<>();
        for (Map.Entry<TileKey, Tile> e : TILES.entrySet()) {
            TileKey key = e.getKey();
            Tile tile = e.getValue();
            int kept = m.after().indexOf(key.region());
            if (kept >= 0) { next.put(new TileKey(m.after().get(kept), key.x(), key.z()), tile); continue; }
            for (WaveRegion n : m.after()) {
                if (m.before().contains(n) || Math.abs(n.stillWaterY() - key.region().stillWaterY()) > WaveRegion.LEVEL_TOLERANCE
                        || rectDistance(n.centerX(), n.centerZ(), key.x(), key.z()) >= n.radiusM()) { continue; }
                if (tile.refreshed != Long.MIN_VALUE) { tile.refreshed = Math.min(tile.refreshed, tick - CACHE_TICKS); }
                next.putIfAbsent(new TileKey(n, key.x(), key.z()), tile);
                break;
            }
        }
        TILES.clear();
        TILES.putAll(next);
    }

    @EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
    public static final class Shaders {
        private Shaders() {}
        @SubscribeEvent public static void register(RegisterShadersEvent event) {
            try {
                event.registerShader(new ShaderInstance(event.getResourceProvider(), ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "rendertype_aerocore_sea"),
                        DefaultVertexFormat.NEW_ENTITY), loaded -> shader = loaded);
            } catch (java.io.IOException | RuntimeException e) {
                LOGGER.error("AeroCore sea shader failed to load; sea surfaces are not drawn", e);
                shader = null;
            }
        }
    }

    /**
     * Which block columns of the 16 m tile at (tx, tz) are this region's drawn sea surface (index cx * 16 + cz), or null
     * when the tile is not cached (not visible). Render thread only.
     */
    public static boolean[] surfaceMask(WaveRegion region, int tx, int tz) {
        Tile tile = TILES.get(new TileKey(region, tx, tz));
        return tile == null || tile.refreshed == Long.MIN_VALUE || !tile.any ? null : tile.surface;
    }

    @SubscribeEvent public static void render(RenderLevelStageEvent event) {
        if (event.getStage() != RenderLevelStageEvent.Stage.AFTER_TRANSLUCENT_BLOCKS) { return; }
        long started = System.nanoTime();
        try { draw(event); } finally { lastDrawNanos = System.nanoTime() - started; spentNanos += lastDrawNanos; frames++; }
    }

    private static void draw(RenderLevelStageEvent event) {
        phase[0] = phase[1] = phase[2] = 0; refreshes = 0;
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        if (invalidated) { TILES.clear(); invalidated = false; MIGRATIONS.clear(); }
        for (Migration m; level != null && (m = MIGRATIONS.poll()) != null; ) { applyMigration(m, level.getGameTime()); }
        ShaderInstance program = shader;
        if (level == null || ClientWaveRegions.current().isEmpty() || program == null) { return; }
        Camera camera = event.getCamera();
        Vec3 eye = camera.getPosition();
        long tick = level.getGameTime();
        float partial = event.getPartialTick().getGameTimeDeltaPartialTick(false);
        double time = (tick + partial) * MinecraftUnits.TICK_SECONDS;
        double maxDistance = minecraft.options.getEffectiveRenderDistance() * 16.0;
        int[] budget = {REFRESH_PER_FRAME, NEW_PER_FRAME};
        List<WaveRegion> regions = ClientWaveRegions.current();
        int count = 0;
        for (WaveRegion region : regions) {
            double still = region.stillWaterY(), reach = SeaSurface.reach(level, region), r = region.radiusM();
            Look look = null;
            boolean back = eye.y < still + reach + 0.5;
            int x0 = Math.floorDiv(Mth.floor(Math.max(region.centerX() - r, eye.x - maxDistance)), TILE) * TILE;
            int x1 = Mth.floor(Math.min(region.centerX() + r, eye.x + maxDistance));
            int z0 = Math.floorDiv(Mth.floor(Math.max(region.centerZ() - r, eye.z - maxDistance)), TILE) * TILE;
            int z1 = Mth.floor(Math.min(region.centerZ() + r, eye.z + maxDistance));
            for (int tx = x0; tx <= x1; tx += TILE) for (int tz = z0; tz <= z1; tz += TILE) {
                if (rectDistance(region.centerX(), region.centerZ(), tx, tz) >= r) { continue; }
                double distance = rectDistance(eye.x, eye.z, tx, tz);
                if (distance > maxDistance) { continue; }
                if (!event.getFrustum().isVisible(new AABB(tx, still - reach - 0.1, tz, tx + TILE, still + reach + 0.1, tz + TILE))) { continue; }
                Tile tile = TILES.computeIfAbsent(new TileKey(region, tx, tz), key -> new Tile());
                tile.used = tick;
                if (tile.refreshed == Long.MIN_VALUE ? budget[1]-- > 0 : tick - tile.refreshed >= CACHE_TICKS && budget[0]-- > 0) {
                    long r0 = System.nanoTime();
                    refresh(level, regions, region, tile, tx, tz, tick);
                    phase[0] += System.nanoTime() - r0; refreshes++;
                }
                if (!tile.any) { continue; }
                if (look == null) { long l0 = System.nanoTime(); look = look(region, level); phase[1] += System.nanoTime() - l0; }
                if (JOBS.size() <= count) { JOBS.add(new Job()); }
                Job job = JOBS.get(count++);
                job.region = region; job.look = look; job.tile = tile; job.tx = tx; job.tz = tz; job.s = step(region, eye, tx, tz);
                job.ambient = Math.max(job.s, meshNeed(region));
                job.neighbours[0] = step(region, eye, tx - TILE, tz); job.neighbours[1] = step(region, eye, tx + TILE, tz);
                job.neighbours[2] = step(region, eye, tx, tz - TILE); job.neighbours[3] = step(region, eye, tx, tz + TILE);
                job.back = back && distance < 48; job.corrected = distance < CORRECTION_RANGE;
                job.eyeX = eye.x; job.eyeY = eye.y; job.eyeZ = eye.z; job.level = level;
            }
        }
        if (count > 0) {
            long computing = System.nanoTime();
            IntStream.range(0, count).parallel().forEach(i -> compute(JOBS.get(i), level, time));
            phase[2] = System.nanoTime() - computing;
            computeNanos += phase[2];
            tilesDrawn += count;
            lighting(program, level, camera, partial);
            MultiBufferSource.BufferSource buffers = minecraft.renderBuffers().bufferSource();
            for (WaveRegion region : regions) {
                Look look = null;
                for (int i = 0; i < count && look == null; i++) { if (JOBS.get(i).region == region) { look = JOBS.get(i).look; } }
                if (look == null) { continue; }
                double ox = Math.floor(region.centerX()), oz = Math.floor(region.centerZ());
                uploadWaves(program, region, look, ox, oz, time);
                VertexConsumer out = buffers.getBuffer(SEA);
                for (int i = 0; i < count; i++) { if (JOBS.get(i).region == region) { emit(out, eye, JOBS.get(i), ox, oz); } }
                buffers.endBatch(SEA);
            }
            for (int i = 0; i < count; i++) { JOBS.get(i).region = null; JOBS.get(i).tile = null; JOBS.get(i).look = null; JOBS.get(i).level = null; }
        }
        if (tick % FORGET_TICKS == 0) {
            for (Iterator<Tile> it = TILES.values().iterator(); it.hasNext(); ) { if (tick - it.next().used > FORGET_TICKS) { it.remove(); } }
            while (JOBS.size() > count) { JOBS.removeLast(); }
        }
    }

    /**
     * Wind for the unresolved slopes: the region's own mean wind, else the level's (WorldWind) 10 m above the region's
     * centre, m/s. Local sheltering scales the ripples per vertex.
     */
    private static double glitterWind(WaveRegion region, ClientLevel level) {
        if (!region.wind().calm()) { return region.wind().mean(level); }
        return region.meanWind(level, region.centerX(), region.centerZ()).length();
    }

    private static Look look(WaveRegion region, ClientLevel level) {
        List<LinearWave> waves = region.waves().components();
        int n = Math.min(waves.size(), MAX_WAVES - RIPPLES);
        double[] amplitude = new double[n], wavenumber = new double[n];
        double resolved = 0, kMax = 0;
        for (int i = 0; i < n; i++) {
            amplitude[i] = waves.get(i).amplitudeM() * region.scale();
            wavenumber[i] = waves.get(i).wavenumber();
            resolved += 0.5 * Math.pow(amplitude[i] * wavenumber[i], 2);
            if (amplitude[i] > 0) { kMax = Math.max(kMax, wavenumber[i]); }
        }
        double wind = glitterWind(region, level);
        double kStart = kMax > 0 ? 1.1 * kMax : 2 * Math.PI / 2;
        Ripples ripples = RIPPLE_CACHE.get(region);
        if (ripples == null || Math.abs(ripples.wind() - wind) > 0.01 * wind + 0.02 || Math.abs(ripples.resolvedVariance() - resolved) > 0.01 * resolved + 1e-6
                || ripples.kStart() != kStart) {
            double[][] rows = wind < RIPPLE_WIND ? new double[0][] : WaterOptics.ripples(wind, region.wind().calm() ? region.sea() == null ? 0 : region.sea().directionRadians()
                    : region.wind().directionRadians(), resolved, kStart, RIPPLES, Objects.hash(region.centerX(), region.centerZ(), region.stillWaterY()));
            ripples = new Ripples(wind, resolved, kStart, rows);
            RIPPLE_CACHE.put(region, ripples);
        }
        double rippleVariance = ripples.rows().length == 0 ? 0 : Math.max(0, WaterOptics.slopeVariance(wind) - resolved);
        WaveRegion.Survey survey = region.survey();
        float[] tops = survey == null || !survey.barriers() ? null : TOPS.computeIfAbsent(survey, WaveSurfaceRenderer::tops);
        double topMax = 0;
        if (tops != null) { for (float t : tops) { topMax = Math.max(topMax, t); } }
        return new Look(amplitude, wavenumber, wind, resolved, rippleVariance, ripples.rows(), tops, topMax);
    }

    /** Height of the ground or the tree tops above the still level at each survey node, m (0 on water). */
    private static float[] tops(WaveRegion.Survey survey) {
        float[] out = new float[survey.nx() * survey.nz()];
        for (int i = 0; i < out.length; i++) {
            if (survey.depthDm(i) == WaveRegion.Survey.LAND) { out[i] = Math.max(survey.groundDm(i), survey.canopyDm(i)) / 10f; }
        }
        return out;
    }

    /** The frame's sky, horizon and shore colours and the sun, from the game's own sky (render thread). */
    private static void lighting(ShaderInstance program, ClientLevel level, Camera camera, float partial) {
        Vec3 sky = level.getSkyColor(camera.getPosition(), partial);
        float[] fog = RenderSystem.getShaderFogColor();
        boolean underwater = camera.getFluidInCamera() == FogType.WATER;
        double angle = level.getTimeOfDay(partial) * 2 * Math.PI, sunY = Math.cos(angle);
        // Minecraft's sun crosses the zenith from east (+x) to west; dimmed toward the horizon and by rain.
        double sunStrength = Math.clamp(sunY * 4, 0, 1) * (1 - level.getRainLevel(partial));
        int foliage = level.getBiome(camera.getBlockPosition()).value().getFoliageColor();
        double shade = SHORE_SHADE * level.getSkyDarken(partial);
        float[] physical = SkyRenderer.zenithColour(level, partial);
        if (physical != null) { sky = new Vec3(physical[0], physical[1], physical[2]); }
        program.safeGetUniform("SkyColor").set((float) sky.x, (float) sky.y, (float) sky.z);
        if (underwater) { program.safeGetUniform("HorizonColor").set((float) sky.x, (float) sky.y, (float) sky.z); }
        else { program.safeGetUniform("HorizonColor").set(fog[0], fog[1], fog[2]); }
        program.safeGetUniform("ShoreColor").set((float) ((foliage >> 16 & 0xFF) / 255.0 * shade), (float) ((foliage >> 8 & 0xFF) / 255.0 * shade), (float) ((foliage & 0xFF) / 255.0 * shade));
        program.safeGetUniform("SunDirection").set((float) -Math.sin(angle), (float) sunY, 0f);
        program.safeGetUniform("SunStrength").set((float) sunStrength);
        program.safeGetUniform("DebugView").set(debugView);
        // The foam lace's clock (rendertype_aerocore_sea.fsh), wrapped where all its rates complete whole turns.
        double seconds = (level.getGameTime() + partial) * MinecraftUnits.TICK_SECONDS;
        program.safeGetUniform("FoamTime").set((float) (seconds - 2000 * Math.PI * Math.floor(seconds / (2000 * Math.PI))));
    }

    /**
     * The region's spectrum (current amplitudes) and ripples as the shader's Waves array: kx, kz, amplitude and the
     * phase at the origin (ox, oz) at time t, wrapped to [0, 2 pi) in double precision so the GPU's floats stay exact.
     */
    private static void uploadWaves(ShaderInstance program, WaveRegion region, Look look, double ox, double oz, double t) {
        WAVES.clear();
        List<LinearWave> waves = region.waves().components();
        int spectrum = look.amplitude().length;
        for (int i = 0; i < spectrum; i++) {
            LinearWave w = waves.get(i);
            double k = look.wavenumber()[i], d = w.directionRadians();
            WAVES.put((float) (k * Math.cos(d))).put((float) (k * Math.sin(d))).put((float) look.amplitude()[i]).put((float) wrap(w.phaseAt(ox, oz, t)));
        }
        for (double[] r : look.ripples()) { WAVES.put((float) r[0]).put((float) r[1]).put((float) r[2]).put((float) wrap(r[4] - r[3] * t)); }
        WAVES.flip();
        program.safeGetUniform("SpectrumCount").set(spectrum);
        program.safeGetUniform("WaveCount").set(spectrum + look.ripples().length);
        int location = Uniform.glGetUniformLocation(program.getId(), "Waves");
        if (location < 0 || !WAVES.hasRemaining()) { return; }
        // The array is not a JSON uniform: set it on the program directly and give the binding back.
        int previous = GlStateManager._getInteger(GL20.GL_CURRENT_PROGRAM);
        GlStateManager._glUseProgram(program.getId());
        GL20.glUniform4fv(location, WAVES);
        GlStateManager._glUseProgram(previous);
    }

    private static double wrap(double phase) { return phase - 2 * Math.PI * Math.floor(phase / (2 * Math.PI)); }

    /** Horizontal distance from (x, z) to the tile's square, 0 inside. */
    private static double rectDistance(double x, double z, int tx, int tz) {
        double dx = Math.max(0, Math.max(tx - x, x - (tx + TILE))), dz = Math.max(0, Math.max(tz - z, z - (tz + TILE)));
        return Math.hypot(dx, dz);
    }

    /**
     * Vertex spacing for a tile, m: a power of two from 0.25 to 2 by distance from the eye, but no finer than the mesh
     * has waves to draw (MeshNeed) unless a wake runs through the tile (its grid's spacing).
     */
    private static double distanceStep(WaveRegion region, Vec3 eye, int tx, int tz) {
        double d = Math.hypot(rectDistance(eye.x, eye.z, tx, tz), eye.y - region.stillWaterY());
        return d < 8 ? 0.25 : d < 16 ? 0.5 : d < 48 ? 1 : 2;
    }

    private static double step(WaveRegion region, Vec3 eye, int tx, int tz) {
        double s = distanceStep(region, eye, tx, tz);
        if (s >= 2) { return s; }
        double need = meshNeed(region);
        if (need > s) {
            ClientLevel level = Minecraft.getInstance().level;
            double wake = level == null ? Double.POSITIVE_INFINITY : Wakes.finestSpacing(level, region.stillWaterY(), tx, tz, tx + TILE, tz + TILE);
            // Tile spacings are powers of two: a wake grid's spacing rounds down to one.
            if (Double.isFinite(wake)) { wake = Math.max(0.25, Math.pow(2, Math.floor(Math.log(wake) / Math.log(2)))); }
            s = Math.min(need, Math.max(s, wake));
        }
        return s;
    }

    /** Amplitude below which a component is not worth a finer mesh, m: its height is left to the shader's normals. */
    private static final double VISIBLE_AMPLITUDE = 0.02;
    private record Need(double scale, double spacing) {}
    private static final Map<WaveRegion, Need> NEEDS = new WeakHashMap<>();

    /**
     * The coarsest spacing (0.25 to 2 m) whose mesh still draws every component of at least VISIBLE_AMPLITUDE at the
     * present scale (wavelength at least four spacings); shorter or smaller ones are drawn per pixel only.
     */
    private static double meshNeed(WaveRegion region) {
        Need need = NEEDS.get(region);
        if (need != null && Math.abs(need.scale() - region.scale()) < 0.05) { return need.spacing(); }
        double shortest = Double.POSITIVE_INFINITY;
        for (LinearWave w : region.waves().components()) {
            if (w.amplitudeM() * region.scale() >= VISIBLE_AMPLITUDE) { shortest = Math.min(shortest, w.wavelengthM()); }
        }
        double spacing = 2;
        for (double s : new double[]{1, 0.5, 0.25}) { if (shortest < 4 * spacing) { spacing = s; } }
        NEEDS.put(region, new Need(region.scale(), spacing));
        return spacing;
    }

    private static void refresh(ClientLevel level, List<WaveRegion> regions, WaveRegion region, Tile tile, int tx, int tz, long tick) {
        int y = Mth.floor(region.stillWaterY() - 1e-6);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        tile.any = false;
        if (!tile.tinted) { tint(level, tile, tx, y, tz); }
        for (int cx = 0; cx < TILE; cx++) for (int cz = 0; cz < TILE; cz++) {
            int i = cx * TILE + cz, x = tx + cx, z = tz + cz;
            pos.set(x, y, z);
            boolean surface = SeaSurface.owner(level, regions, x, z) == region;
            tile.surface[i] = surface;
            // Every column gets a tint and light: edge vertices are shared with neighbouring columns.
            tile.light[i] = maxLight(level, pos);
            tile.depth[i] = 0;
            if (!surface) { continue; }
            int bed = y;
            while (bed > y - DEPTH_SCAN && level.getFluidState(pos.setY(bed - 1)).is(net.minecraft.tags.FluidTags.WATER)) { bed--; }
            tile.depth[i] = (float) (region.stillWaterY() - bed);
            tile.any = true;
        }
        tile.refreshed = tick;
    }

    /** Every column's water tint, bilinear between the biome colours on the TINT_STEP lattice (shared with neighbouring tiles). */
    private static void tint(ClientLevel level, Tile tile, int tx, int y, int tz) {
        FluidState water = Fluids.WATER.defaultFluidState();
        IClientFluidTypeExtensions fluid = IClientFluidTypeExtensions.of(water);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        int[] lattice = new int[TINT_POINTS * TINT_POINTS];
        for (int a = 0; a < TINT_POINTS; a++) for (int b = 0; b < TINT_POINTS; b++) {
            lattice[a * TINT_POINTS + b] = fluid.getTintColor(water, level, pos.set(tx + a * TINT_STEP, y, tz + b * TINT_STEP));
        }
        for (int cx = 0; cx < TILE; cx++) for (int cz = 0; cz < TILE; cz++) {
            int a = cx / TINT_STEP, b = cz / TINT_STEP, q = a * TINT_POINTS + b;
            float fa = (cx - a * TINT_STEP) / (float) TINT_STEP, fb = (cz - b * TINT_STEP) / (float) TINT_STEP;
            int c00 = lattice[q], c01 = lattice[q + 1], c10 = lattice[q + TINT_POINTS], c11 = lattice[q + TINT_POINTS + 1], out = 0xFF000000;
            for (int shift = 0; shift <= 16; shift += 8) {
                float v = ((c00 >> shift & 0xFF) * (1 - fb) + (c01 >> shift & 0xFF) * fb) * (1 - fa)
                        + ((c10 >> shift & 0xFF) * (1 - fb) + (c11 >> shift & 0xFF) * fb) * fa;
                out |= Math.min(255, Math.round(v)) << shift;
            }
            tile.tint[cx * TILE + cz] = out;
        }
        tile.tinted = true;
    }

    /** A tile's vertex heights, slopes and attributes (worker thread: touches only the job and thread-safe readers). */
    private static void compute(Job j, ClientLevel level, double time) {
        WaveRegion region = j.region;
        int tx = j.tx, tz = j.tz, points = (int) Math.round(TILE / j.s) + 1, count = points * points;
        double s = j.s;
        j.points = points;
        j.ensure(count);
        // The second-order surface the physics reads (CompiledWaves): the linear part times the taper w, the bound part w^2.
        CoastalWaves coast = region.coast();
        boolean coastal = coast != null && coast.changedIn(tx, tz, tx + TILE, tz + TILE);
        double ambient = j.ambient;
        dev.mcmod.aerocore.api.CompiledWaves waves = region.drawnWaves(ambient);
        long t0 = System.nanoTime();
        // Where the coast's envelopes and depth hardly change across the tile (open water of a wind sea, whose growth
        // differs from 1 everywhere), the waves at its centre (CompiledWaves.at: the local amplitude, phase and
        // wavenumber of each component) stand for the whole tile on the fast open-sea path.
        if (coastal && uniform(j, coast, waves)) { waves = waves.at(coast, tx + TILE / 2.0, tz + TILE / 2.0, 1); coastal = false; PARTS.incrementAndGet(6); }
        if (coastal) { PARTS.incrementAndGet(5); }
        PARTS.addAndGet(7, count);
        int drawn = 0;
        for (int i = 0; i < waves.size(); i++) { if (waves.amplitude(i) != 0) { drawn++; } }
        PARTS.addAndGet(8, drawn);
        if (ambient > s) {
            // A wake needs the fine mesh but the sea does not: the sea on its own coarser grid, bilinear onto the fine one.
            double a = ambient;
            int pa = (int) Math.round(TILE / a) + 1, ca = pa * pa;
            if (j.coarseGrids[0].length < ca) { j.coarseGrids = new double[6][ca]; }
            double[][] c = j.coarseGrids;
            if (coastal) { waves.gridCoastal(coast, tx, tz, a, a, pa, pa, time, c[0], c[1], c[2], c[3], c[4], c[5]); }
            else { waves.grid(tx, tz, a, a, pa, pa, time, c[0], c[1], c[2]); waves.gridSecondOrder(tx, tz, a, a, pa, pa, time, c[3], c[4], c[5]); }
            double[][] fine = {j.height, j.slopeX, j.slopeZ, j.height2, j.slope2X, j.slope2Z};
            for (int i = 0; i < points; i++) for (int k = 0; k < points; k++) {
                double gx = i * s / a, gz = k * s / a;
                int ia = Math.min((int) gx, pa - 2), ib = Math.min((int) gz, pa - 2), q = ia * pa + ib;
                double fx = gx - ia, fz = gz - ib;
                for (int f = 0; f < 6; f++) {
                    double[] v = c[f];
                    fine[f][i * points + k] = (v[q] * (1 - fz) + v[q + 1] * fz) * (1 - fx) + (v[q + pa] * (1 - fz) + v[q + pa + 1] * fz) * fx;
                }
            }
        } else if (coastal) {
            waves.gridCoastal(coast, tx, tz, s, s, points, points, time, j.height, j.slopeX, j.slopeZ, j.height2, j.slope2X, j.slope2Z);
        } else {
            waves.grid(tx, tz, s, s, points, points, time, j.height, j.slopeX, j.slopeZ);
            waves.gridSecondOrder(tx, tz, s, s, points, points, time, j.height2, j.slope2X, j.slope2Z);
        }
        long t1 = System.nanoTime();
        if (j.corrected) { addCorrection(j, waves, coastal ? coast : null, time); }
        long t2 = System.nanoTime();
        double still = region.stillWaterY();
        boolean core = Math.hypot(Math.max(Math.abs(tx - region.centerX()), Math.abs(tx + TILE - region.centerX())),
                Math.max(Math.abs(tz - region.centerZ()), Math.abs(tz + TILE - region.centerZ()))) <= region.radiusM() - region.taperM();
        double[] height = j.height, slopeX = j.slopeX, slopeZ = j.slopeZ;
        for (int i = 0; i < points; i++) for (int k = 0; k < points; k++) {
            int p = i * points + k;
            double w = core ? 1 : region.weight(tx + i * s, tz + k * s), w2 = w * w;
            height[p] = w * height[p] + w2 * j.height2[p];
            slopeX[p] = w * slopeX[p] + w2 * j.slope2X[p];
            slopeZ[p] = w * slopeZ[p] + w2 * j.slope2Z[p];
        }
        // Wakes are not tapered: SeaSurface.surfaceY adds them to the tapered ambient sea in the same way.
        Wakes.addToGrid(level, region, tx, tz, s, points, time, height, slopeX, slopeZ);
        long t3 = System.nanoTime();
        float[] foamGrid = SeaFoam.patch(region, tx, tz);
        lattice(j, coastal ? coast : null, foamGrid == null);
        long t4 = System.nanoTime();
        float[] a = j.attributes;
        double highest = Double.NEGATIVE_INFINITY;
        for (int p = 0; p < count; p++) {
            height[p] += still;
            highest = Math.max(highest, height[p]);
            double lx = (p / points) * s, lz = (p % points) * s;
            int column = Math.min(TILE - 1, (int) Math.floor(lx)) * TILE + Math.min(TILE - 1, (int) Math.floor(lz));
            double gx = lx / 2, gz = lz / 2;
            int ia = Math.min((int) gx, LATTICE - 2), ib = Math.min((int) gz, LATTICE - 2), q = ia * LATTICE + ib;
            double fx = gx - ia, fz = gz - ib;
            int o = p * ATTRIBUTES;
            a[o] = foamGrid == null ? (float) bilinear(j.latticeFoam, q, fx, fz) : foamAt(foamGrid, lx, lz);
            a[o + 1] = (float) WaterOptics.transmittance(j.tile.depth[column]);
            a[o + 2] = (float) bilinear(j.latticeGain, q, fx, fz);
            a[o + 3] = (float) bilinear(j.latticeRipple, q, fx, fz);
            a[o + 4] = (float) bilinear(j.latticeHorizon, q, fx, fz);
            a[o + 5] = (float) slopeX[p];
            a[o + 6] = (float) slopeZ[p];
        }
        long t5 = System.nanoTime();
        PARTS.addAndGet(0, t1 - t0); PARTS.addAndGet(1, t2 - t1); PARTS.addAndGet(2, t3 - t2); PARTS.addAndGet(3, t4 - t3); PARTS.addAndGet(4, t5 - t4);
        // The underside shows only where the eye can be below the surface (a margin for the snapped edges).
        j.back &= j.eyeY < highest + 0.1;
        // West, east, north, south edges onto coarser neighbours.
        snap(j, j.neighbours[0], 0, 0, 0, 1);
        snap(j, j.neighbours[1], points - 1, 0, 0, 1);
        snap(j, j.neighbours[2], 0, 0, 1, 0);
        snap(j, j.neighbours[3], 0, points - 1, 1, 0);
    }

    /**
     * Whether every drawn component's envelope at the tile's corners is within UNIFORM (of the largest gain) of its
     * value at the centre, and the depth within UNIFORM of the centre's (or all deeper than its waves feel).
     */
    private static boolean uniform(Job j, CoastalWaves coast, dev.mcmod.aerocore.api.CompiledWaves waves) {
        int n = waves.size();
        if (coast.components() != n) { return false; }
        if (j.envelope[0].length < n) { j.envelope = new double[6][n]; }
        if (j.centreEnvelope[0].length < n) { j.centreEnvelope = new double[2][n]; }
        double cx = j.tx + TILE / 2.0, cz = j.tz + TILE / 2.0, depth = coast.depth(cx, cz);
        double[][] e = j.envelope, c = j.centreEnvelope;
        if (!coast.envelopes(cx, cz, c[0], c[1], null, null, null, null)) { return false; }
        double scale = Math.max(coast.maxGain(), 1e-6);
        for (int k = 0; k < 4; k++) {
            double x = j.tx + (k / 2) * TILE, z = j.tz + (k % 2) * TILE;
            if (!coast.envelopes(x, z, e[0], e[1], null, null, null, null)) { return false; }
            for (int i = 0; i < n; i++) {
                if (waves.amplitude(i) == 0) { continue; }
                if (Math.hypot(e[0][i] - c[0][i], e[1][i] - c[1][i]) > UNIFORM * scale) { return false; }
            }
            double d = coast.depth(x, z);
            if (Double.isFinite(depth) && depth < 1e3 && !(Math.abs(d - depth) <= UNIFORM * depth)) { return false; }
        }
        return true;
    }

    /**
     * The per-vertex shader inputs on the tile's 2 m lattice: the gain on the waves the mesh leaves out (the coast's
     * RMS envelope over them, times the region's taper), the ripples' share of the open water's (Cox-Munk at the
     * sheltered, tapered wind), and the elevation of the surveyed terrain seen along the reflected ray.
     */
    private static void lattice(Job j, CoastalWaves coast, boolean meanFoam) {
        WaveRegion region = j.region;
        Look look = j.look;
        int n = look.amplitude().length;
        double kMax = 2 * Math.PI / (4 * j.ambient), unresolved = 0;
        for (int i = 0; i < n; i++) { if (look.wavenumber()[i] > kMax) { unresolved += look.amplitude()[i] * look.amplitude()[i]; } }
        boolean envelopes = coast != null && unresolved > 0 && coast.components() == n;
        if (envelopes && j.envelope[0].length < n) { j.envelope = new double[6][n]; }
        WaveRegion.Survey survey = region.survey();
        // The 10 m wind (sheltered, tapered) every 8 m, bilinear between: it varies over tens of metres, and the level's
        // wind costs an upwind terrain lookup per point.
        double[] wind = j.coarseWind;
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) {
            double x = j.tx + 8 * a, z = j.tz + 8 * b;
            wind[a * 3 + b] = region.wind().calm() ? region.meanWind(j.level, x, z).length() : look.wind() * region.weight(x, z) * region.shelter(x, z);
        }
        for (int ia = 0; ia < LATTICE; ia++) for (int ib = 0; ib < LATTICE; ib++) {
            int q = ia * LATTICE + ib;
            double x = j.tx + 2 * ia, z = j.tz + 2 * ib, w = region.weight(x, z), gain = w;
            int ca = Math.min(ia / 4, 1), cb = Math.min(ib / 4, 1);
            double fa = (ia - 4 * ca) / 4.0, fb = (ib - 4 * cb) / 4.0;
            double localWind = (wind[ca * 3 + cb] * (1 - fb) + wind[ca * 3 + cb + 1] * fb) * (1 - fa)
                    + (wind[(ca + 1) * 3 + cb] * (1 - fb) + wind[(ca + 1) * 3 + cb + 1] * fb) * fa;
            double[][] e = j.envelope;
            if (envelopes && coast.envelopes(x, z, e[0], e[1], e[2], e[3], e[4], e[5])) {
                double sum = 0;
                for (int i = 0; i < n; i++) {
                    if (look.wavenumber()[i] > kMax) { sum += look.amplitude()[i] * look.amplitude()[i] * (e[0][i] * e[0][i] + e[1][i] * e[1][i]); }
                }
                gain = w * Math.sqrt(sum / unresolved);
            }
            j.latticeGain[q] = Math.min(gain, 3);
            double ripple = 0;
            if (look.rippleVariance() > 0) {
                double local = WaterOptics.slopeVariance(localWind) - look.resolvedVariance();
                ripple = Math.sqrt(Math.clamp(local / look.rippleVariance(), 0, 1));
            }
            j.latticeRipple[q] = ripple;
            j.latticeHorizon[q] = look.tops() == null ? 0 : horizon(survey, look.tops(), look.topMax(), x, z, x - j.eyeX, z - j.eyeZ);
            j.latticeFoam[q] = meanFoam ? SeaFoam.meanFoam(region, x, z, localWind) : 0;
        }
    }

    /**
     * Elevation (rad) of the highest surveyed ground or tree top seen from the still surface at (x, z) looking along the
     * horizontal direction (dx, dz) away from the eye, where a mirror reflection of the view continues; 0 when none.
     */
    private static double horizon(WaveRegion.Survey survey, float[] tops, double topMax, double x, double z, double dx, double dz) {
        double length = Math.hypot(dx, dz);
        if (!(length > 0.1) || topMax <= 0) { return 0; }
        dx /= length; dz /= length;
        double spacing = survey.spacing(), best = 0;
        int nx = survey.nx(), nz = survey.nz();
        for (double t = spacing; t <= REFLECTION_REACH; t += t < 16 ? spacing : 2 * spacing) {
            if (topMax / t <= best) { break; }
            int i = (int) Math.round((x + dx * t - survey.x0()) / spacing), k = (int) Math.round((z + dz * t - survey.z0()) / spacing);
            if (i < 0 || k < 0 || i >= nx || k >= nz) { break; }
            best = Math.max(best, tops[i * nz + k] / t);
        }
        return Math.atan(best);
    }

    /** A computed tile's quads (render thread). */
    private static void emit(VertexConsumer out, Vec3 eye, Job j, double ox, double oz) {
        Tile tile = j.tile;
        int points = j.points, tx = j.tx, tz = j.tz, code = (int) Math.round(Math.log(4 * j.ambient) / Math.log(2));
        double s = j.s;
        if (s <= 1) {
            int n = (int) Math.round(1 / s);
            for (int cx = 0; cx < TILE; cx++) for (int cz = 0; cz < TILE; cz++) {
                int c = cx * TILE + cz;
                if (!tile.surface[c]) { continue; }
                int tint = packTint(tile.tint[c], code);
                for (int i = 0; i < n; i++) for (int k = 0; k < n; k++) {
                    int gi = cx * n + i, gk = cz * n + k;
                    quad(out, eye, j, tile.light[c], tint, tx + gi * s, tz + gk * s, s, s, ox, oz,
                            gi * points + gk, gi * points + gk + 1, (gi + 1) * points + gk + 1, (gi + 1) * points + gk);
                }
            }
            return;
        }
        // 2 m spacing: whole 2 x 2 cells as one quad, partial cells per column with interpolated corners.
        for (int ax = 0; ax < TILE / 2; ax++) for (int az = 0; az < TILE / 2; az++) {
            int c00 = 2 * ax * TILE + 2 * az, c10 = c00 + TILE, c01 = c00 + 1, c11 = c00 + TILE + 1;
            if (tile.surface[c00] && tile.surface[c10] && tile.surface[c01] && tile.surface[c11]) {
                quad(out, eye, j, tile.light[c00], packTint(tile.tint[c00], code), tx + 2 * ax, tz + 2 * az, 2, 2, ox, oz,
                        ax * points + az, ax * points + az + 1, (ax + 1) * points + az + 1, (ax + 1) * points + az);
                continue;
            }
            for (int c : new int[]{c00, c10, c01, c11}) {
                if (!tile.surface[c]) { continue; }
                columnQuad(out, eye, j, tile.light[c], packTint(tile.tint[c], code), c / TILE, c % TILE, ox, oz);
            }
        }
    }

    /** The biome tint and the mesh's cutoff wavelength (2^code m) as the shader's UV1 (two non-negative shorts). */
    private static int packTint(int tint, int code) {
        int r = tint >> 16 & 0xFF, g = tint >> 8 & 0xFF, b = tint & 0xFF;
        return (r * 128 + (g >> 1)) | ((b >> 1) * 256 + code) << 16;
    }

    /**
     * Moves the vertices along one tile edge that the coarser neighbour does not have onto the straight line between
     * the ones it does, so both sides of the edge draw the same polyline. (i0, k0) is the edge start, (di, dk) its step.
     */
    private static void snap(Job j, double neighbourStep, int i0, int k0, int di, int dk) {
        int points = j.points, ratio = (int) Math.round(neighbourStep / j.s);
        if (ratio <= 1) { return; }
        float[] a = j.attributes;
        for (int q = 0; q < points; q++) {
            int rest = q % ratio;
            if (rest == 0) { continue; }
            int pa = (i0 + di * (q - rest)) * points + k0 + dk * (q - rest), pb = (i0 + di * (q - rest + ratio)) * points + k0 + dk * (q - rest + ratio);
            int p = (i0 + di * q) * points + k0 + dk * q;
            double f = (double) rest / ratio;
            j.height[p] = j.height[pa] + (j.height[pb] - j.height[pa]) * f;
            for (int c = 0; c < ATTRIBUTES; c++) { a[p * ATTRIBUTES + c] = (float) (a[pa * ATTRIBUTES + c] + (a[pb * ATTRIBUTES + c] - a[pa * ATTRIBUTES + c]) * f); }
        }
    }

    /**
     * The rest of the second-order surface (CompiledWaves.gridCorrection), evaluated on a 2 m lattice and bilinear in
     * between: its shortest scales are the bound waves of the dominant components, half their length.
     */
    private static void addCorrection(Job j, dev.mcmod.aerocore.api.CompiledWaves waves, CoastalWaves coast, double time) {
        java.util.Arrays.fill(j.latticeH, 0); java.util.Arrays.fill(j.latticeX, 0); java.util.Arrays.fill(j.latticeZ, 0);
        waves.gridCorrection(coast, j.tx, j.tz, 2, 2, LATTICE, LATTICE, time, j.latticeH, j.latticeX, j.latticeZ);
        int points = j.points;
        for (int i = 0; i < points; i++) for (int k = 0; k < points; k++) {
            double gx = i * j.s / 2, gz = k * j.s / 2;
            int a = Math.min((int) gx, LATTICE - 2), b = Math.min((int) gz, LATTICE - 2);
            double fx = gx - a, fz = gz - b;
            int q = a * LATTICE + b, p = i * points + k;
            j.height2[p] += bilinear(j.latticeH, q, fx, fz); j.slope2X[p] += bilinear(j.latticeX, q, fx, fz); j.slope2Z[p] += bilinear(j.latticeZ, q, fx, fz);
        }
    }

    private static double bilinear(double[] v, int q, double fx, double fz) {
        return (v[q] * (1 - fz) + v[q + 1] * fz) * (1 - fx) + (v[q + LATTICE] * (1 - fz) + v[q + LATTICE + 1] * fz) * fx;
    }

    /** Foam (0 to 1) at a tile-local point, bilinear on SeaFoam's grid. */
    private static float foamAt(float[] grid, double x, double z) {
        double gx = x / SeaFoam.SPACING, gz = z / SeaFoam.SPACING;
        int i = Math.min((int) gx, SeaFoam.POINTS - 2), k = Math.min((int) gz, SeaFoam.POINTS - 2);
        double fx = gx - i, fz = gz - k;
        int p = i * SeaFoam.POINTS + k;
        return (float) ((grid[p] * (1 - fz) + grid[p + 1] * fz) * (1 - fx) + (grid[p + SeaFoam.POINTS] * (1 - fz) + grid[p + SeaFoam.POINTS + 1] * fz) * fx);
    }

    /** Render-thread scratch for one quad: corner x, z, heights, attribute offsets, and interpolated corner attributes. */
    private static final double[] QX = new double[4], QZ = new double[4], QH = new double[4];
    private static final int[] QO = new int[4], CORNER_I = {0, 0, 1, 1}, CORNER_K = {0, 1, 1, 0}, BACK = {0, 3, 2, 1};
    private static final float[] CORNER = new float[4 * ATTRIBUTES];

    /** One block column on a 2 m grid: corner heights and attributes bilinear in the coarse cell, matching the merged neighbours. */
    private static void columnQuad(VertexConsumer out, Vec3 eye, Job j, int light, int tint, int cx, int cz, double ox, double oz) {
        int points = j.points;
        double[] h = QH;
        float[] corner = CORNER, a = j.attributes;
        for (int c = 0; c < 4; c++) {
            double gx = (cx + CORNER_I[c]) / 2.0, gz = (cz + CORNER_K[c]) / 2.0;
            int i = Math.min((int) gx, points - 2), k = Math.min((int) gz, points - 2);
            double fx = gx - i, fz = gz - k;
            int p00 = i * points + k, p10 = p00 + points, p01 = p00 + 1, p11 = p00 + points + 1;
            h[c] = (j.height[p00] * (1 - fx) + j.height[p10] * fx) * (1 - fz) + (j.height[p01] * (1 - fx) + j.height[p11] * fx) * fz;
            for (int q = 0; q < ATTRIBUTES; q++) {
                corner[c * ATTRIBUTES + q] = (float) ((a[p00 * ATTRIBUTES + q] * (1 - fx) + a[p10 * ATTRIBUTES + q] * fx) * (1 - fz)
                        + (a[p01 * ATTRIBUTES + q] * (1 - fx) + a[p11 * ATTRIBUTES + q] * fx) * fz);
            }
        }
        for (int c = 0; c < 4; c++) { QX[c] = j.tx + cx + CORNER_I[c]; QZ[c] = j.tz + cz + CORNER_K[c]; QO[c] = c * ATTRIBUTES; }
        emit(out, eye, light, tint, j.back, corner, ox, oz);
    }

    private static void quad(VertexConsumer out, Vec3 eye, Job j, int light, int tint, double x, double z, double sx, double sz, double ox, double oz,
                             int p00, int p01, int p11, int p10) {
        QH[0] = j.height[p00]; QH[1] = j.height[p01]; QH[2] = j.height[p11]; QH[3] = j.height[p10];
        QO[0] = p00 * ATTRIBUTES; QO[1] = p01 * ATTRIBUTES; QO[2] = p11 * ATTRIBUTES; QO[3] = p10 * ATTRIBUTES;
        for (int c = 0; c < 4; c++) { QX[c] = x + CORNER_I[c] * sx; QZ[c] = z + CORNER_K[c] * sz; }
        emit(out, eye, light, tint, j.back, j.attributes, ox, oz);
    }

    /**
     * The quad in QX, QZ, QH (corners (x, z), (x, z + sz), (x + sx, z + sz), (x + sx, z)) with attributes at QO in a;
     * the back face (reversed) is the underside seen from under the water, drawn by the same shader.
     */
    private static void emit(VertexConsumer out, Vec3 eye, int light, int tint, boolean back, float[] a, double ox, double oz) {
        for (int c = 0; c < 4; c++) { vertex(out, eye, QX[c], QH[c], QZ[c], a, QO[c], tint, light, ox, oz); }
        if (!back) { return; }
        for (int c : BACK) { vertex(out, eye, QX[c], QH[c], QZ[c], a, QO[c], tint, light, ox, oz); }
    }

    /** Vanilla's water top-face light: the brighter of the block and the block above, per channel. */
    private static int maxLight(ClientLevel level, BlockPos pos) {
        int here = LevelRenderer.getLightColor(level, pos), above = LevelRenderer.getLightColor(level, pos.above());
        return Math.max(here & 0xFF, above & 0xFF) | Math.max(here >> 16 & 0xFF, above >> 16 & 0xFF) << 16;
    }

    /** One vertex: its attributes packed as the sea shader reads them (see rendertype_aerocore_sea.vsh). */
    private static void vertex(VertexConsumer out, Vec3 eye, double x, double y, double z, float[] a, int o, int tint, int light, double ox, double oz) {
        double sx = a[o + 5], sz = a[o + 6], l = Math.sqrt(sx * sx + 1 + sz * sz);
        out.addVertex((float) (x - eye.x), (float) (y - eye.y), (float) (z - eye.z))
                .setColor(unit(a[o]), unit(a[o + 1]), unit(a[o + 2] / 3), unit(a[o + 4] / (Math.PI / 2)))
                .setUv((float) (x - ox), (float) (z - oz))
                .setUv1(tint & 0xFFFF, tint >>> 16)
                .setLight(light)
                .setNormal((float) (-sx / l), unit(a[o + 3]), (float) (-sz / l));
    }

    private static float unit(double v) { return (float) Math.clamp(v, 0, 1); }
}
