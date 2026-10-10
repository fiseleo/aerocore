package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.Vec3d;
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
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.texture.TextureAtlasSprite;
import net.minecraft.core.BlockPos;
import net.minecraft.util.Mth;
import net.minecraft.world.inventory.InventoryMenu;
import net.minecraft.world.level.material.FluidState;
import net.minecraft.world.level.material.Fluids;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RenderLevelStageEvent;
import net.neoforged.neoforge.client.extensions.common.IClientFluidTypeExtensions;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Draws each sea region's surface in place of the vanilla water top faces that ClientWaveRegions hides, from the same
 * field, still level, wakes and time base as the server physics (game time x 0.05 s). The whole region is drawn out to
 * the render distance in 16 x 16 tiles. Vertex spacing grows with distance from the camera (0.25 / 0.5 m near, 1 m to
 * 48 m, 2 m beyond, where full 2 x 2 cells merge into one quad); heights are always the exact field at the vertices,
 * and a finer tile's edge vertices are moved onto its coarser neighbour's edge so the levels meet without cracks.
 * Which columns are sea surface (plus their tint and light) is cached per tile and refreshed every half second.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class WaveSurfaceRenderer {
    private static final int TILE = 16, CACHE_TICKS = 10, REFRESH_PER_FRAME = 24, FORGET_TICKS = 200;
    /** Flat-surface sun direction for slope shading; a presentation choice, not a lighting model. */
    private static final Vec3d LIGHT = new Vec3d(0.35, 1, 0.55).normalized();

    private record TileKey(WaveRegion region, int x, int z) {}
    private static final class Tile {
        long refreshed = Long.MIN_VALUE, used;
        boolean any;
        final boolean[] surface = new boolean[TILE * TILE];
        final int[] tint = new int[TILE * TILE], light = new int[TILE * TILE];
    }
    private static final Map<TileKey, Tile> TILES = new HashMap<>();
    private static volatile boolean invalidated;
    private static double[] height = new double[0], slopeX = new double[0], slopeZ = new double[0];
    private static float[] shade = new float[0];

    private WaveSurfaceRenderer() {}

    /** Drops the column cache; safe to call from any thread. */
    public static void invalidate() { invalidated = true; }

    @SubscribeEvent public static void render(RenderLevelStageEvent event) {
        if (event.getStage() != RenderLevelStageEvent.Stage.AFTER_TRANSLUCENT_BLOCKS) { return; }
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        if (invalidated) { TILES.clear(); invalidated = false; }
        if (level == null || ClientWaveRegions.current().isEmpty()) { return; }
        Camera camera = event.getCamera();
        Vec3 eye = camera.getPosition();
        long tick = level.getGameTime();
        double time = (tick + event.getPartialTick().getGameTimeDeltaPartialTick(false)) * MinecraftUnits.TICK_SECONDS;
        double maxDistance = minecraft.options.getEffectiveRenderDistance() * 16.0;
        MultiBufferSource.BufferSource buffers = minecraft.renderBuffers().bufferSource();
        VertexConsumer out = buffers.getBuffer(RenderType.translucent());
        TextureAtlasSprite sprite = minecraft.getTextureAtlas(InventoryMenu.BLOCK_ATLAS)
                .apply(IClientFluidTypeExtensions.of(Fluids.WATER).getStillTexture());
        int[] budget = {REFRESH_PER_FRAME};
        java.util.List<WaveRegion> regions = ClientWaveRegions.current();
        for (WaveRegion region : regions) {
            double still = region.stillWaterY(), reach = SeaSurface.reach(level, region), r = region.radiusM();
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
                if (tile.refreshed == Long.MIN_VALUE || (tick - tile.refreshed >= CACHE_TICKS && budget[0]-- > 0)) { refresh(level, regions, region, tile, tx, tz, tick); }
                if (!tile.any) { continue; }
                double[] neighbours = {step(region, eye, tx - TILE, tz), step(region, eye, tx + TILE, tz), step(region, eye, tx, tz - TILE), step(region, eye, tx, tz + TILE)};
                draw(out, level, region, tile, sprite, eye, time, tx, tz, step(region, eye, tx, tz), neighbours, back && distance < 48);
            }
        }
        buffers.endBatch(RenderType.translucent());
        if (tick % FORGET_TICKS == 0) {
            for (Iterator<Tile> it = TILES.values().iterator(); it.hasNext(); ) { if (tick - it.next().used > FORGET_TICKS) { it.remove(); } }
        }
    }

    /** Horizontal distance from (x, z) to the tile's square, 0 inside. */
    private static double rectDistance(double x, double z, int tx, int tz) {
        double dx = Math.max(0, Math.max(tx - x, x - (tx + TILE))), dz = Math.max(0, Math.max(tz - z, z - (tz + TILE)));
        return Math.hypot(dx, dz);
    }

    /** Vertex spacing for a tile, m: a power of two from 0.25 to 2 by distance (0.25 only for short waves). */
    private static double step(WaveRegion region, Vec3 eye, int tx, int tz) {
        double d = rectDistance(eye.x, eye.z, tx, tz);
        if (d < 8 && region.waves().shortestWavelength() < 4) { return 0.25; }
        if (d < 16) { return 0.5; }
        return d < 48 ? 1 : 2;
    }

    private static void refresh(ClientLevel level, java.util.List<WaveRegion> regions, WaveRegion region, Tile tile, int tx, int tz, long tick) {
        int y = Mth.floor(region.stillWaterY() - 1e-6);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        tile.any = false;
        for (int cx = 0; cx < TILE; cx++) for (int cz = 0; cz < TILE; cz++) {
            int i = cx * TILE + cz, x = tx + cx, z = tz + cz;
            pos.set(x, y, z);
            boolean surface = SeaSurface.owner(level, regions, x, z) == region;
            tile.surface[i] = surface;
            if (!surface) { continue; }
            FluidState fluid = level.getFluidState(pos);
            tile.tint[i] = IClientFluidTypeExtensions.of(fluid).getTintColor(fluid, level, pos);
            tile.light[i] = maxLight(level, pos);
            tile.any = true;
        }
        tile.refreshed = tick;
    }

    private static void draw(VertexConsumer out, ClientLevel level, WaveRegion region, Tile tile, TextureAtlasSprite sprite, Vec3 eye,
                             double time, int tx, int tz, double s, double[] neighbours, boolean back) {
        int points = (int) Math.round(TILE / s) + 1, count = points * points;
        if (height.length < count) { height = new double[count]; slopeX = new double[count]; slopeZ = new double[count]; shade = new float[count]; }
        region.compiled().grid(tx, tz, s, s, points, points, time, height, slopeX, slopeZ);
        double still = region.stillWaterY();
        boolean core = Math.hypot(Math.max(Math.abs(tx - region.centerX()), Math.abs(tx + TILE - region.centerX())),
                Math.max(Math.abs(tz - region.centerZ()), Math.abs(tz + TILE - region.centerZ()))) <= region.radiusM() - region.taperM();
        if (!core) {
            for (int i = 0; i < points; i++) for (int k = 0; k < points; k++) {
                int p = i * points + k;
                double w = region.weight(tx + i * s, tz + k * s);
                height[p] *= w; slopeX[p] *= w; slopeZ[p] *= w;
            }
        }
        // Wakes are not tapered: SeaSurface.surfaceY adds them to the tapered ambient sea in the same way.
        Wakes.addToGrid(level, region, tx, tz, s, points, time, height, slopeX, slopeZ);
        for (int p = 0; p < points * points; p++) {
            height[p] += still;
            Vec3d normal = new Vec3d(-slopeX[p], 1, -slopeZ[p]).normalized();
            shade[p] = (float) Math.clamp(normal.dot(LIGHT) / LIGHT.y(), 0.7, 1.0);
        }
        // West, east, north, south edges onto coarser neighbours.
        snap(points, s, neighbours[0], 0, 0, 0, 1);
        snap(points, s, neighbours[1], points - 1, 0, 0, 1);
        snap(points, s, neighbours[2], 0, 0, 1, 0);
        snap(points, s, neighbours[3], 0, points - 1, 1, 0);
        if (s <= 1) {
            int n = (int) Math.round(1 / s);
            for (int cx = 0; cx < TILE; cx++) for (int cz = 0; cz < TILE; cz++) {
                int c = cx * TILE + cz;
                if (!tile.surface[c]) { continue; }
                for (int i = 0; i < n; i++) for (int k = 0; k < n; k++) {
                    int gi = cx * n + i, gk = cz * n + k;
                    quad(out, eye, tile.tint[c], tile.light[c], sprite, back,
                            tx + gi * s, tz + gk * s, s, s, (float) i / n, (float) k / n, 1f / n,
                            gi * points + gk, gi * points + gk + 1, (gi + 1) * points + gk + 1, (gi + 1) * points + gk);
                }
            }
            return;
        }
        // 2 m spacing: whole 2 x 2 cells as one quad, partial cells per column with interpolated corners.
        for (int ax = 0; ax < TILE / 2; ax++) for (int az = 0; az < TILE / 2; az++) {
            int c00 = 2 * ax * TILE + 2 * az, c10 = c00 + TILE, c01 = c00 + 1, c11 = c00 + TILE + 1;
            if (tile.surface[c00] && tile.surface[c10] && tile.surface[c01] && tile.surface[c11]) {
                quad(out, eye, tile.tint[c00], tile.light[c00], sprite, back, tx + 2 * ax, tz + 2 * az, 2, 2, 0, 0, 1,
                        ax * points + az, ax * points + az + 1, (ax + 1) * points + az + 1, (ax + 1) * points + az);
                continue;
            }
            for (int c : new int[]{c00, c10, c01, c11}) {
                if (!tile.surface[c]) { continue; }
                int cx = c / TILE, cz = c % TILE;
                columnQuad(out, eye, tile.tint[c], tile.light[c], sprite, back, tx, tz, cx, cz, points);
            }
        }
    }

    /**
     * Moves the vertices along one tile edge that the coarser neighbour does not have onto the straight line between
     * the ones it does, so both sides of the edge draw the same polyline. (i0, k0) is the edge start, (di, dk) its step.
     */
    private static void snap(int points, double s, double neighbourStep, int i0, int k0, int di, int dk) {
        int ratio = (int) Math.round(neighbourStep / s);
        if (ratio <= 1) { return; }
        for (int j = 0; j < points; j++) {
            int rest = j % ratio;
            if (rest == 0) { continue; }
            int a = (i0 + di * (j - rest)) * points + k0 + dk * (j - rest), b = (i0 + di * (j - rest + ratio)) * points + k0 + dk * (j - rest + ratio);
            int p = (i0 + di * j) * points + k0 + dk * j;
            double f = (double) rest / ratio;
            height[p] = height[a] + (height[b] - height[a]) * f;
            shade[p] = (float) (shade[a] + (shade[b] - shade[a]) * f);
        }
    }

    /** One block column on a 2 m grid: corner heights bilinear in the coarse cell, matching the merged neighbours. */
    private static void columnQuad(VertexConsumer out, Vec3 eye, int tint, int light, TextureAtlasSprite sprite, boolean back,
                                   int tx, int tz, int cx, int cz, int points) {
        double[] h = new double[4];
        float[] sh = new float[4];
        int[][] corners = {{cx, cz}, {cx, cz + 1}, {cx + 1, cz + 1}, {cx + 1, cz}};
        for (int c = 0; c < 4; c++) {
            double gx = corners[c][0] / 2.0, gz = corners[c][1] / 2.0;
            int i = Math.min((int) gx, points - 2), k = Math.min((int) gz, points - 2);
            double fx = gx - i, fz = gz - k;
            int p00 = i * points + k, p10 = p00 + points, p01 = p00 + 1, p11 = p00 + points + 1;
            h[c] = (height[p00] * (1 - fx) + height[p10] * fx) * (1 - fz) + (height[p01] * (1 - fx) + height[p11] * fx) * fz;
            sh[c] = (float) ((shade[p00] * (1 - fx) + shade[p10] * fx) * (1 - fz) + (shade[p01] * (1 - fx) + shade[p11] * fx) * fz);
        }
        emit(out, eye, tint, light, sprite, back, tx + cx, tz + cz, 1, 1, 0, 0, 1, h, sh);
    }

    private static void quad(VertexConsumer out, Vec3 eye, int tint, int light, TextureAtlasSprite sprite, boolean back,
                             double x, double z, double sx, double sz, float u, float v, float span, int p00, int p01, int p11, int p10) {
        emit(out, eye, tint, light, sprite, back, x, z, sx, sz, u, v, span,
                new double[]{height[p00], height[p01], height[p11], height[p10]}, new float[]{shade[p00], shade[p01], shade[p11], shade[p10]});
    }

    /** Corners in order (x, z), (x, z + sz), (x + sx, z + sz), (x + sx, z); the back face is the vanilla underside. */
    private static void emit(VertexConsumer out, Vec3 eye, int tint, int light, TextureAtlasSprite sprite, boolean back,
                             double x, double z, double sx, double sz, float u, float v, float span, double[] h, float[] sh) {
        float r = (tint >> 16 & 0xFF) / 255f, g = (tint >> 8 & 0xFF) / 255f, b = (tint & 0xFF) / 255f, a = (tint >>> 24) / 255f;
        float u0 = sprite.getU(u), u1 = sprite.getU(u + span), v0 = sprite.getV(v), v1 = sprite.getV(v + span);
        vertex(out, eye, x, h[0], z, r, g, b, a, sh[0], u0, v0, light);
        vertex(out, eye, x, h[1], z + sz, r, g, b, a, sh[1], u0, v1, light);
        vertex(out, eye, x + sx, h[2], z + sz, r, g, b, a, sh[2], u1, v1, light);
        vertex(out, eye, x + sx, h[3], z, r, g, b, a, sh[3], u1, v0, light);
        if (!back) { return; }
        vertex(out, eye, x, h[0], z, r, g, b, a, sh[0], u0, v0, light);
        vertex(out, eye, x + sx, h[3], z, r, g, b, a, sh[3], u1, v0, light);
        vertex(out, eye, x + sx, h[2], z + sz, r, g, b, a, sh[2], u1, v1, light);
        vertex(out, eye, x, h[1], z + sz, r, g, b, a, sh[1], u0, v1, light);
    }

    /** Vanilla's water top-face light: the brighter of the block and the block above, per channel. */
    private static int maxLight(ClientLevel level, BlockPos pos) {
        int here = LevelRenderer.getLightColor(level, pos), above = LevelRenderer.getLightColor(level, pos.above());
        return Math.max(here & 0xFF, above & 0xFF) | Math.max(here >> 16 & 0xFF, above >> 16 & 0xFF) << 16;
    }

    private static void vertex(VertexConsumer out, Vec3 eye, double x, double y, double z, float r, float g, float b, float a,
                               float shade, float u, float v, int light) {
        out.addVertex((float) (x - eye.x), (float) (y - eye.y), (float) (z - eye.z))
                .setColor(r * shade, g * shade, b * shade, a).setUv(u, v).setLight(light).setNormal(0, 1, 0);
    }
}
