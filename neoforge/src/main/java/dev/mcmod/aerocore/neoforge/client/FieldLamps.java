package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.blaze3d.vertex.VertexFormat;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.light.Allard;
import dev.mcmod.aerocore.api.light.FieldLamp;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.WorldAir;
import net.minecraft.client.Camera;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderStateShard;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.resources.ResourceKey;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientPlayerNetworkEvent;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Lights fixed in the world (0.38): runway, approach and taxiway lights, PAPIs, obstruction lights and beacons that
 * a mod hands over as lists ({@link FieldLamp}), drawn each frame in one batch after the world's translucent blocks,
 * as far as {@link #MAX_M} whether or not their chunks are loaded (a runway is seen from far beyond view distance).
 * <ul>
 *   <li>How strongly each shows is a display rule from Allard's law ({@link Allard}): the illuminance at the eye
 *   through the weather's visibility ({@link WorldAir#visibilityM}) over the eye's threshold against the sky (bright
 *   by day, ICAO's night floor at night), s = log10(E / E_t) / 3, 0 to 1. A light at its threshold is a faint dot.</li>
 *   <li>A dot is never smaller than about a pixel and a half wide nor than its lens; brighter lights spread a little
 *   more (glare).</li>
 *   <li>Lights beyond the projection's far plane (four times the view distance) are drawn on a sphere just inside it,
 *   scaled to keep their size on screen: there is nothing nearer than them to hide them out there anyway.</li>
 *   <li>Flashes ask their signal for the peak since the last frame, so a flash shorter than a frame shows in one.</li>
 *   <li>Added light, no fog (a light is seen through fog by Allard's law, not vanilla's fog), depth tested, no depth
 *   written; alpha 1/255 as LampRenderer's (Fabulous).</li>
 * </ul>
 * Owners are any object the caller keeps (an airfield's id); a circuit's level scales its lights' intensity (0 off,
 * 1 full; circuits never set are full). Client thread only (the render thread).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class FieldLamps {
    private FieldLamps() {}

    /** Farthest a field light is drawn (m). */
    public static final double MAX_M = 8_000;
    /** Half the width of the faintest dot and how much the brightest spreads beyond it (pixels). */
    private static final float MIN_PX = 1.5f, GLARE_PX = 2.5f;

    private record Group(ResourceKey<Level> dimension, FieldLamp[] lamps, Map<Integer, Float> circuits) {}

    private static final Map<Object, Group> GROUPS = new ConcurrentHashMap<>();
    private static double lastTimeS = Double.NaN;
    private static RenderType type;

    /** Sets (or replaces) an owner's lights in a dimension; its circuits' levels are kept. */
    public static void put(ResourceKey<Level> dimension, Object owner, List<FieldLamp> lamps) {
        Group old = GROUPS.get(owner);
        GROUPS.put(owner, new Group(dimension, lamps.toArray(FieldLamp[]::new), old == null ? new ConcurrentHashMap<>() : old.circuits()));
    }

    /** Forgets an owner's lights. */
    public static void remove(Object owner) { GROUPS.remove(owner); }

    /** Sets how strongly a circuit of an owner's lights shines (0 off, 1 full). */
    public static void circuit(Object owner, int circuit, float level) {
        Group g = GROUPS.get(owner);
        if (g != null) { g.circuits().put(circuit, Math.max(0f, level)); }
    }

    /** How many lights are held (all owners). */
    public static int count() { return GROUPS.values().stream().mapToInt(g -> g.lamps().length).sum(); }

    /** Forgets all. */
    public static void clear() { GROUPS.clear(); lastTimeS = Double.NaN; }

    @SubscribeEvent public static void loggedOut(ClientPlayerNetworkEvent.LoggingOut event) { clear(); }

    /** Draws the lights of the camera's dimension (the late stage: after the world's translucent blocks). */
    static void draw(MultiBufferSource.BufferSource buffers, Camera camera, Matrix4f projection) {
        Minecraft minecraft = Minecraft.getInstance();
        net.minecraft.client.multiplayer.ClientLevel level = minecraft.level;
        if (level == null || GROUPS.isEmpty() || shader == null) { return; }
        long start = System.nanoTime();
        float partial = minecraft.getTimer().getGameTimeDeltaPartialTick(false);
        double now = (level.getGameTime() + partial) * 0.05;
        double from = Double.isNaN(lastTimeS) || now < lastTimeS || now - lastTimeS > 0.25 ? now - 1 / 60.0 : lastTimeS;
        lastTimeS = now;
        // the eye's threshold against the sky: bright day to ICAO's night floor, through the sky's darkening
        float dark = Math.clamp((1f - level.getSkyDarken(partial)) / 0.8f, 0f, 1f);
        double background = Math.exp(Math.log(Allard.DAY_BACKGROUND) + dark * (Math.log(Allard.NIGHT_BACKGROUND) - Math.log(Allard.DAY_BACKGROUND)));
        double threshold = Allard.thresholdLux(background), visibility = WorldAir.visibilityM(level, partial);
        Vec3 eye = camera.getPosition();
        Vector3f look = camera.getLookVector();
        // a pixel's angle and the screen's corner off the axis, from the projection
        float tanY = 1f / projection.m11(), tanX = 1f / projection.m00();
        int height = Math.max(1, minecraft.getWindow().getHeight());
        double pixel = 2 * tanY / height, cosCorner = 1 / Math.sqrt(1 + tanX * tanX + tanY * tanY);
        double far = minecraft.gameRenderer.getDepthFar() * 0.9;
        VertexConsumer out = null;
        int considered = 0, drawn = 0;
        for (Group g : GROUPS.values()) {
            if (!g.dimension().equals(level.dimension())) { continue; }
            for (FieldLamp lamp : g.lamps()) {
                considered++;
                double dx = lamp.at().x() - eye.x, dy = lamp.at().y() - eye.y, dz = lamp.at().z() - eye.z;
                double d = Math.sqrt(dx * dx + dy * dy + dz * dz);
                if (d > MAX_M || d < 0.05) { continue; }
                double cos = (dx * look.x + dy * look.y + dz * look.z) / d;
                if (d > 8 && cos < cosCorner - 0.02) { continue; }   // off screen (with a margin for the dot)
                Float c = g.circuits().get(lamp.circuit());
                double on = (c == null ? 1 : c) * lamp.signal().peak(from, now);
                if (on <= 0) { continue; }
                Vec3d toEye = new Vec3d(-dx, -dy, -dz);
                double cover = lamp.coverage(toEye);
                if (cover <= 0) { continue; }
                double e = Allard.illuminanceLux(lamp.intensityCd() * on * cover, d, visibility);
                double s = Math.clamp(Math.log10(e / threshold) / 3, 0, 1);
                if (s <= 0.01) { continue; }
                int argb = lamp.colour(toEye, now);
                double half = Math.max(0.8 * lamp.lensM(), (MIN_PX + GLARE_PX * s) * pixel * d);
                // beyond the far plane: drawn on a sphere just inside it, scaled to keep its size on screen
                double scale = d > far ? far / d : 1;
                if (out == null) { out = buffers.getBuffer(type()); }
                billboard(out, (float) (dx * scale), (float) (dy * scale), (float) (dz * scale), (float) (half * scale), argb, (float) s);
                drawn++;
            }
        }
        if (out != null) { buffers.endBatch(type()); }
        PartStats.fieldLamps(considered, drawn, System.nanoTime() - start);
    }

    private static void billboard(VertexConsumer out, float x, float y, float z, float size, int argb, float strength) {
        float len = (float) Math.sqrt(x * x + y * y + z * z);
        // toward the eye from the light
        float tx = -x / len, ty = -y / len, tz = -z / len;
        // right = toEye x up, up' = right x toEye
        float rx = ty * 0 - tz * 1, ry = tz * 0 - tx * 0, rz = tx * 1 - ty * 0;
        float rl = (float) Math.sqrt(rx * rx + ry * ry + rz * rz);
        if (rl < 1e-4f) { rx = 1; ry = 0; rz = 0; rl = 1; }
        rx = rx / rl * size; ry = ry / rl * size; rz = rz / rl * size;
        float ux = ry * tz - rz * ty, uy = rz * tx - rx * tz, uz = rx * ty - ry * tx;
        float ul = (float) Math.sqrt(ux * ux + uy * uy + uz * uz);
        ux = ux / ul * size; uy = uy / ul * size; uz = uz / ul * size;
        int r = scaled(argb >> 16, strength), g = scaled(argb >> 8, strength), b = scaled(argb, strength);
        vertex(out, x - rx - ux, y - ry - uy, z - rz - uz, 0, 1, r, g, b);
        vertex(out, x + rx - ux, y + ry - uy, z + rz - uz, 1, 1, r, g, b);
        vertex(out, x + rx + ux, y + ry + uy, z + rz + uz, 1, 0, r, g, b);
        vertex(out, x - rx + ux, y - ry + uy, z - rz + uz, 0, 0, r, g, b);
    }

    private static int scaled(int channel, float strength) { return Math.clamp((int) ((channel & 255) * strength + 0.5f), 0, 255); }

    private static void vertex(VertexConsumer out, float x, float y, float z, float u, float v, int r, int g, int b) {
        out.addVertex(x, y, z).setUv(u, v).setColor(r, g, b, 1);
    }

    private static volatile net.minecraft.client.renderer.ShaderInstance shader;

    /** Registers the lights' shader (no fog, no alpha test: vanilla's position_tex_color drops alpha below 0.1). */
    @EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
    public static final class Shaders {
        private Shaders() {}

        @SubscribeEvent public static void register(net.neoforged.neoforge.client.event.RegisterShadersEvent event) throws java.io.IOException {
            event.registerShader(new net.minecraft.client.renderer.ShaderInstance(event.getResourceProvider(),
                    ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "rendertype_aerocore_field_lamp"), DefaultVertexFormat.POSITION_TEX_COLOR),
                    loaded -> shader = loaded);
        }
    }

    /** Added light with the lights' own shader, the lamps' soft dot, no depth written. */
    private static RenderType type() {
        if (type == null) {
            ResourceLocation halo = LampRenderer.haloTexture();
            type = RenderType.create("aerocore_field_lamps", DefaultVertexFormat.POSITION_TEX_COLOR, VertexFormat.Mode.QUADS, 4096, false, true,
                    RenderType.CompositeState.builder()
                            .setTextureState(new RenderStateShard.TextureStateShard(halo, true, false))
                            .setShaderState(new RenderStateShard.ShaderStateShard(() -> shader))
                            .setTransparencyState(RenderStateShard.ADDITIVE_TRANSPARENCY)
                            .setCullState(RenderStateShard.NO_CULL)
                            .setWriteMaskState(RenderStateShard.COLOR_WRITE)
                            .createCompositeState(false));
        }
        return type;
    }
}
