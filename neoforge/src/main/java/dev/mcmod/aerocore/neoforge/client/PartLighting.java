package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.renderer.ShaderInstance;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RenderFrameEvent;

/**
 * The part shader's surroundings for a frame (0.29): the sun's direction and colour (warmer and weaker low in the
 * sky, gone below the horizon, the moon's faint light then; dimmed by rain and thunder), and the sky, horizon and
 * ground colours reflected, from Minecraft's sky and fog colours at the camera. Worked out once a frame.
 * 0.39, while beautified shading is on (Beautify): the ground reflected takes the colour of the land round the camera
 * (looked at once a second), and the shader gets how much day it is (block light warms parts at night).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class PartLighting {
    private PartLighting() {}

    private static float[] sun = {0, 1, 0}, sunColor = {1, 0.97f, 0.9f}, sky = {0.45f, 0.62f, 0.95f}, horizon = {0.75f, 0.82f, 0.92f}, ground = {0.3f, 0.29f, 0.26f};

    @SubscribeEvent public static void frame(RenderFrameEvent.Pre event) {
        Minecraft mc = Minecraft.getInstance();
        ClientLevel level = mc.level;
        if (level == null) { return; }
        float pt = event.getPartialTick().getGameTimeDeltaPartialTick(false);
        double angle = 2 * Math.PI * level.getTimeOfDay(pt);
        // celestial angle 0 is noon (sun overhead), 0.75 sunrise in the east (+x)
        double sx = -Math.sin(angle), sy = Math.cos(angle);
        float weather = 1 - 0.75f * level.getRainLevel(pt) - 0.2f * level.getThunderLevel(pt);
        if (sy > -0.05) {
            double height = Math.max(0, sy);
            double strength = Math.min(1, height * 4 + 0.05) * weather;
            double warm = 1 - Math.min(1, height * 2.5);   // low sun: redder
            sun = new float[]{(float) sx, (float) Math.max(sy, 0.02), 0};
            sunColor = new float[]{(float) (strength * 1.0), (float) (strength * (0.95 - 0.25 * warm)), (float) (strength * (0.88 - 0.45 * warm))};
        } else {
            sun = new float[]{(float) -sx, (float) -sy, 0};   // the moon, opposite
            float m = 0.12f * weather;
            sunColor = new float[]{m * 0.8f, m * 0.85f, m};
        }
        normalize(sun);
        Vec3 camera = mc.gameRenderer.getMainCamera().getPosition();
        Vec3 skyColour = level.getSkyColor(camera, pt);
        float[] fog = com.mojang.blaze3d.systems.RenderSystem.getShaderFogColor();   // the last frame's horizon
        sky = new float[]{(float) skyColour.x, (float) skyColour.y, (float) skyColour.z};
        horizon = new float[]{(float) (0.5 * fog[0] + 0.5 * skyColour.x), (float) (0.5 * fog[1] + 0.5 * skyColour.y), (float) (0.5 * fog[2] + 0.5 * skyColour.z)};
        ground = new float[]{0.3f * fog[0] + 0.05f, 0.29f * fog[1] + 0.05f, 0.26f * fog[2] + 0.05f};
        // 0.39 beautified: how much day there is (for the block light's warmth), and the ground reflected takes the colour
        // of the land round the camera (grass, sand, snow, water), lit as the fog is
        day = (float) Math.clamp(sy * 4 + 0.2, 0, 1) * weather;
        if (Beautify.on()) {
            long now = level.getGameTime();
            if (now - landAt >= 20 || now < landAt) { landAt = now; land = land(level, camera); }
            float lit = (fog[0] + fog[1] + fog[2]) / 3;
            for (int i = 0; i < 3; i++) {
                landSeen[i] += (land[i] - landSeen[i]) * 0.05f;   // about a second to settle at 60 frames
                ground[i] = 0.35f * ground[i] + 0.65f * (0.45f * landSeen[i] * lit + 0.03f);
            }
        }
    }

    /** 0.39: what share of the day it is (0 at night, 1 by full day; rain dims it). */
    private static float day = 1;
    private static long landAt = Long.MIN_VALUE / 2;
    private static float[] land = {0.35f, 0.42f, 0.25f}, landSeen = {0.35f, 0.42f, 0.25f};

    /** The mean colour of the land round a point: 5 x 5 columns 8 m apart, their top block's colour (grass, leaves and water by the biome). */
    private static float[] land(ClientLevel level, Vec3 at) {
        float[] sum = new float[3];
        int n = 0;
        net.minecraft.core.BlockPos.MutableBlockPos p = new net.minecraft.core.BlockPos.MutableBlockPos();
        for (int i = -2; i <= 2; i++) {
            for (int j = -2; j <= 2; j++) {
                int x = (int) Math.floor(at.x) + 8 * i, z = (int) Math.floor(at.z) + 8 * j;
                if (!level.hasChunk(x >> 4, z >> 4)) { continue; }
                int top = level.getHeight(net.minecraft.world.level.levelgen.Heightmap.Types.WORLD_SURFACE, x, z) - 1;
                p.set(x, top, z);
                net.minecraft.world.level.block.state.BlockState s = level.getBlockState(p);
                int rgb;
                if (s.is(net.minecraft.world.level.block.Blocks.GRASS_BLOCK)) { rgb = net.minecraft.client.renderer.BiomeColors.getAverageGrassColor(level, p); }
                else if (s.is(net.minecraft.tags.BlockTags.LEAVES)) { rgb = net.minecraft.client.renderer.BiomeColors.getAverageFoliageColor(level, p); }
                else if (!s.getFluidState().isEmpty()) { rgb = net.minecraft.client.renderer.BiomeColors.getAverageWaterColor(level, p); }
                else { rgb = s.getMapColor(level, p).col; }
                if (rgb == 0) { continue; }
                sum[0] += ((rgb >> 16) & 255) / 255f;
                sum[1] += ((rgb >> 8) & 255) / 255f;
                sum[2] += (rgb & 255) / 255f;
                n++;
            }
        }
        if (n == 0) { return land; }
        return new float[]{sum[0] / n, sum[1] / n, sum[2] / n};
    }

    private static void normalize(float[] v) {
        float l = (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (l > 1e-6f) { v[0] /= l; v[1] /= l; v[2] /= l; }
    }

    /** Sets the frame's surroundings on the part shader. */
    static void apply(ShaderInstance s) {
        s.safeGetUniform("SunDirection").set(sun[0], sun[1], sun[2]);
        s.safeGetUniform("SunColor").set(sunColor[0], sunColor[1], sunColor[2]);
        s.safeGetUniform("SkyColor").set(sky[0], sky[1], sky[2]);
        s.safeGetUniform("HorizonColor").set(horizon[0], horizon[1], horizon[2]);
        s.safeGetUniform("GroundColor").set(ground[0], ground[1], ground[2]);
        // 0.39: beautified shading on or off, the block light's sheen, the share of the day
        s.safeGetUniform("PartGlobal").set(Beautify.on() ? 1f : 0f, 0.8f, day, 0f);
        s.safeGetUniform("BlockLightColor").set(1f, 0.74f, 0.46f);
    }
}
