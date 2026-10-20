package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.platform.NativeImage;
import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.blaze3d.vertex.VertexFormat;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderStateShard;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.texture.DynamicTexture;
import net.minecraft.client.renderer.texture.OverlayTexture;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RenderLevelStageEvent;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;

/**
 * Marks left on the ground (0.30): a tyre's skid mark or rut, laid as a strip of quads just over the surface where a
 * mod says a wheel touched (each key one strip, continued while it keeps coming within a short gap), fading over its
 * life. At most 4096 segments (the oldest go first); kept on the client only and lost with the level. Drawn after the
 * world's translucent blocks, darkening what is under them (blended, a depth offset, no depth written; depth tested, so
 * what stands in front hides them). Not after the cutout blocks: at that stage the camera's turn is not yet in the
 * model-view matrix and camera-relative vertices land elsewhere (0.30 development: no mark showed). Render thread.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class GroundMarks {
    private GroundMarks() {}

    private static final int MAX_SEGMENTS = 4096;
    private static final double STEP_M = 0.2, BREAK_M = 3, BREAK_S = 0.5, LIFT_M = 0.004;

    private record Segment(double x0, double y0, double z0, double x1, double y1, double z1, double sx, double sz, int argb, long born, double lifeS, int light) {}

    private record Last(double x, double y, double z, long at) {}

    private static final Deque<Segment> SEGMENTS = new ArrayDeque<>();
    private static final Map<Object, Last> LAST = new HashMap<>();
    private static RenderType type;
    private static Object level;

    /**
     * A wheel (a key of the caller's: a vehicle and a wheel) touching the ground at a point (world, on the surface)
     * leaves a mark of a width (m) and colour (ARGB: alpha its darkness) that fades over a life (s).
     */
    public static void mark(Object key, Vec3 point, double widthM, int argb, double lifeS) {
        var mc = Minecraft.getInstance();
        if (mc.level == null) { return; }
        if (level != mc.level) { SEGMENTS.clear(); LAST.clear(); level = mc.level; }
        long now = System.nanoTime();
        Last last = LAST.get(key);
        if (last == null || (now - last.at()) / 1e9 > BREAK_S || Math.hypot(point.x - last.x(), point.z - last.z()) > BREAK_M) {
            LAST.put(key, new Last(point.x, point.y, point.z, now));
            return;
        }
        double dx = point.x - last.x(), dz = point.z - last.z(), d = Math.hypot(dx, dz);
        if (d < STEP_M) { return; }
        double half = widthM / 2, sx = -dz / d * half, sz = dx / d * half;
        int light = LevelRenderer.getLightColor(mc.level, BlockPos.containing(point.x, point.y + 0.1, point.z));
        SEGMENTS.addLast(new Segment(last.x(), last.y() + LIFT_M, last.z(), point.x, point.y + LIFT_M, point.z, sx, sz, argb, now, lifeS, light));
        while (SEGMENTS.size() > MAX_SEGMENTS) { SEGMENTS.removeFirst(); }
        LAST.put(key, new Last(point.x, point.y, point.z, now));
    }

    /** The wheel left the ground (or stopped marking): its next mark starts a new strip. */
    public static void lift(Object key) { LAST.remove(key); }

    /** Segments held now (a development count). */
    public static int count() { return SEGMENTS.size(); }

    @SubscribeEvent public static void render(RenderLevelStageEvent event) {
        if (event.getStage() != RenderLevelStageEvent.Stage.AFTER_TRANSLUCENT_BLOCKS || SEGMENTS.isEmpty()) { return; }
        Minecraft mc = Minecraft.getInstance();
        if (mc.level == null || mc.level != level) { return; }
        Vec3 eye = event.getCamera().getPosition();
        long now = System.nanoTime();
        SEGMENTS.removeIf(s -> (now - s.born()) / 1e9 > s.lifeS());
        MultiBufferSource.BufferSource buffers = mc.renderBuffers().bufferSource();
        VertexConsumer out = buffers.getBuffer(type());
        for (Segment s : SEGMENTS) {
            double ex = s.x0() - eye.x, ez = s.z0() - eye.z;
            if (ex * ex + ez * ez > 160 * 160) { continue; }
            double fade = 1 - (now - s.born()) / 1e9 / s.lifeS();
            int a = (int) (((s.argb() >>> 24) & 255) * fade), r = (s.argb() >> 16) & 255, g = (s.argb() >> 8) & 255, b = s.argb() & 255;
            vertex(out, s.x0() - s.sx() - eye.x, s.y0() - eye.y, s.z0() - s.sz() - eye.z, r, g, b, a, s.light());
            vertex(out, s.x1() - s.sx() - eye.x, s.y1() - eye.y, s.z1() - s.sz() - eye.z, r, g, b, a, s.light());
            vertex(out, s.x1() + s.sx() - eye.x, s.y1() - eye.y, s.z1() + s.sz() - eye.z, r, g, b, a, s.light());
            vertex(out, s.x0() + s.sx() - eye.x, s.y0() - eye.y, s.z0() + s.sz() - eye.z, r, g, b, a, s.light());
        }
        buffers.endBatch(type());
    }

    private static void vertex(VertexConsumer out, double x, double y, double z, int r, int g, int b, int a, int light) {
        out.addVertex((float) x, (float) y, (float) z).setColor(r, g, b, a).setUv(0.5f, 0.5f).setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(0, 1, 0);
    }

    private static RenderType type() {
        if (type == null) {
            NativeImage white = new NativeImage(4, 4, false);
            for (int y = 0; y < 4; y++) { for (int x = 0; x < 4; x++) { white.setPixelRGBA(x, y, 0xffffffff); } }
            ResourceLocation key = ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "ground_marks");
            Minecraft.getInstance().getTextureManager().register(key, new DynamicTexture(white));
            type = RenderType.create("aerocore_ground_marks", DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 4096, false, true,
                    RenderType.CompositeState.builder().setShaderState(RenderStateShard.RENDERTYPE_ENTITY_TRANSLUCENT_SHADER)
                            .setTextureState(new RenderStateShard.TextureStateShard(key, false, false))
                            .setTransparencyState(RenderStateShard.TRANSLUCENT_TRANSPARENCY).setCullState(RenderStateShard.NO_CULL)
                            .setLightmapState(RenderStateShard.LIGHTMAP).setOverlayState(RenderStateShard.OVERLAY)
                            .setLayeringState(RenderStateShard.POLYGON_OFFSET_LAYERING).setWriteMaskState(RenderStateShard.COLOR_WRITE)
                            .createCompositeState(false));
        }
        return type;
    }
}
