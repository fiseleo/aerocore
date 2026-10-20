package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.platform.NativeImage;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.mesh.Lamp;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.LightTexture;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.texture.DynamicTexture;
import net.minecraft.client.renderer.texture.OverlayTexture;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.Vec3;
import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Lamps' light outside their lenses (0.30), drawn after the world's translucent blocks: a halo facing the camera (a
 * soft dot no smaller than a few pixels, so a navigation light shows from afar), shown within the lamp's sector and
 * brighter at night; and for a beam, in the dark, a faint cone of light and a pool where it lands (a ray each frame,
 * for the 8 nearest beams within 48 m). All added light (no depth written, depth tested). Display only: no block is
 * lit (Minecraft has no moving light sources). Render thread.
 * 0.39, beautified shading on: a lens-only lamp's lit patches (LensPatches: a tower's windows, a tail lamp's lens) get
 * a soft halo too, in the dark, strongest seen square on (within 256 m, at most 384 a frame).
 */
final class LampRenderer {
    private LampRenderer() {}

    private static final int MAX_POOLS = 8;
    private static final double POOL_RANGE = 48, CONE_LENGTH = 14;
    /** 0.39: lens halos (beautified) within this range, at most this many a frame. */
    private static final double GLINT_RANGE = 256;
    private static final int MAX_GLINTS = 384;
    private static ResourceLocation halo, white;

    static void draw(MultiBufferSource.BufferSource buffers, List<PartLayers.LampDraw> lamps) {
        Minecraft minecraft = Minecraft.getInstance();
        var level = minecraft.level;
        if (level == null) { return; }
        float partial = minecraft.getTimer().getGameTimeDeltaPartialTick(false);
        // 0 by day, 1 at night: lamps show most in the dark
        float dark = Math.clamp((1f - level.getSkyDarken(partial)) / 0.8f, 0f, 1f);
        Vec3 eye = minecraft.gameRenderer.getMainCamera().getPosition();
        RenderType haloType = type(texture(true)), plainType = type(texture(false));
        VertexConsumer out = buffers.getBuffer(haloType);
        List<PartLayers.LampDraw> beams = new ArrayList<>();
        int glints = 0;
        for (PartLayers.LampDraw d : lamps) {
            Lamp lamp = d.lamp();
            Matrix4f m = d.pose().pose();
            Matrix3f n = d.pose().normal();
            Vector3f at = m.transformPosition((float) lamp.at().x(), (float) lamp.at().y(), (float) lamp.at().z(), new Vector3f());   // camera-relative
            Vector3f axis = n.transform((float) lamp.axis().x(), (float) lamp.axis().y(), (float) lamp.axis().z(), new Vector3f()).normalize();
            float distance = at.length();
            if (distance < 1e-3f || distance > 512) { continue; }
            Vector3f toEye = new Vector3f(at).negate().div(distance);
            float facing = axis.dot(toEye), mask = sector(lamp.sectorDeg(), facing);
            if (d.glint()) {
                // 0.39 beautified: a lit lens patch's soft halo (a window at night, a tail lamp), strongest seen square on
                if (distance > GLINT_RANGE || glints >= MAX_GLINTS || facing <= 0) { continue; }
                // (0.35 and 1.4 radii at first: in the airport world a lit tower's windows showed only a few faint dots)
                float strength = Math.min(1.5f, d.level()) * 0.5f * dark * (float) Math.pow(facing, 0.5);
                if (strength > 0.01f) {
                    glints++;
                    float size = (float) Math.max(3 * lamp.radiusM(), 0.002 * distance);
                    billboard(out, new Vector3f(at).add(new Vector3f(toEye).mul(0.05f)), toEye, size, lamp.argb(), strength);
                }
                continue;
            }
            float strength = Math.min(1.5f, d.level()) * (0.25f + 0.75f * dark) * mask;
            if (strength > 0.01f) {
                float size = (float) Math.max(5 * lamp.radiusM() * Math.max(1, d.level()), 0.0025 * distance);
                Vector3f centre = new Vector3f(at).add(new Vector3f(toEye).mul((float) (1.5 * lamp.radiusM())));
                billboard(out, centre, toEye, size, lamp.argb(), strength);
            }
            if (lamp.kind() == Lamp.Kind.BEAM && dark > 0.2f && distance < POOL_RANGE) { beams.add(d); }
        }
        buffers.endBatch(haloType);
        if (!beams.isEmpty()) {
            beams.sort(Comparator.comparingDouble(b -> b.pose().pose().transformPosition((float) b.lamp().at().x(), (float) b.lamp().at().y(), (float) b.lamp().at().z(), new Vector3f()).length()));
            VertexConsumer cone = buffers.getBuffer(plainType);
            for (int i = 0; i < Math.min(MAX_POOLS, beams.size()); i++) { cone(cone, beams.get(i), dark); }
            buffers.endBatch(plainType);
            VertexConsumer pool = buffers.getBuffer(haloType);
            for (int i = 0; i < Math.min(MAX_POOLS, beams.size()); i++) { pool(pool, beams.get(i), dark, eye, level); }
            buffers.endBatch(haloType);
        }
    }

    /** How much of a lamp shows from a direction (cosine of the angle off its axis): all within its sector, fading 5 degrees past it. */
    static float sector(double sectorDeg, float cos) {
        if (sectorDeg >= 359) { return 1; }
        double half = Math.toRadians(sectorDeg / 2), feather = Math.toRadians(5);
        double inner = Math.cos(Math.min(Math.PI, half - feather)), outer = Math.cos(Math.min(Math.PI, half + feather));
        if (cos >= inner) { return 1; }
        if (cos <= outer) { return 0; }
        return (float) ((cos - outer) / (inner - outer));
    }

    private static void billboard(VertexConsumer out, Vector3f c, Vector3f toEye, float size, int argb, float strength) {
        Vector3f right = new Vector3f(toEye).cross(0, 1, 0);
        if (right.lengthSquared() < 1e-6f) { right.set(1, 0, 0); }
        right.normalize().mul(size);
        Vector3f up = new Vector3f(right).cross(toEye).normalize().mul(size);
        int r = scaled(argb >> 16, strength), g = scaled(argb >> 8, strength), b = scaled(argb, strength);
        vertex(out, c.x - right.x - up.x, c.y - right.y - up.y, c.z - right.z - up.z, 0, 1, r, g, b);
        vertex(out, c.x + right.x - up.x, c.y + right.y - up.y, c.z + right.z - up.z, 1, 1, r, g, b);
        vertex(out, c.x + right.x + up.x, c.y + right.y + up.y, c.z + right.z + up.z, 1, 0, r, g, b);
        vertex(out, c.x - right.x + up.x, c.y - right.y + up.y, c.z - right.z + up.z, 0, 0, r, g, b);
    }

    /** A beam's cone of light in the dark: bright at the lamp, gone at its end. */
    private static void cone(VertexConsumer out, PartLayers.LampDraw d, float dark) {
        Lamp lamp = d.lamp();
        Matrix4f m = d.pose().pose();
        Matrix3f n = d.pose().normal();
        Vector3f at = m.transformPosition((float) lamp.at().x(), (float) lamp.at().y(), (float) lamp.at().z(), new Vector3f());
        Vector3f axis = n.transform((float) lamp.axis().x(), (float) lamp.axis().y(), (float) lamp.axis().z(), new Vector3f()).normalize();
        float length = (float) Math.min(CONE_LENGTH, lamp.rangeM()), spread = (float) (length * Math.tan(Math.toRadians(lamp.beamDeg())));
        Vector3f u = new Vector3f(axis).cross(0, 1, 0);
        if (u.lengthSquared() < 1e-6f) { u.set(1, 0, 0); }
        u.normalize();
        Vector3f w = new Vector3f(axis).cross(u).normalize();
        float strength = 0.22f * dark * Math.min(1f, d.level());
        int r = scaled(lamp.argb() >> 16, strength), g = scaled(lamp.argb() >> 8, strength), b = scaled(lamp.argb(), strength);
        Vector3f end = new Vector3f(axis).mul(length).add(at);
        int sides = 12;
        for (int k = 0; k < sides; k++) {
            double a0 = 2 * Math.PI * k / sides, a1 = 2 * Math.PI * (k + 1) / sides;
            Vector3f p0 = new Vector3f(u).mul((float) Math.cos(a0) * spread).add(new Vector3f(w).mul((float) Math.sin(a0) * spread)).add(end);
            Vector3f p1 = new Vector3f(u).mul((float) Math.cos(a1) * spread).add(new Vector3f(w).mul((float) Math.sin(a1) * spread)).add(end);
            vertex(out, at.x, at.y, at.z, 0.5f, 0.5f, r, g, b);
            vertex(out, at.x, at.y, at.z, 0.5f, 0.5f, r, g, b);
            vertex(out, p0.x, p0.y, p0.z, 0.5f, 0.5f, 0, 0, 0);
            vertex(out, p1.x, p1.y, p1.z, 0.5f, 0.5f, 0, 0, 0);
        }
    }

    /** Where a beam lands (a ray along it): a soft spot on the face it hits, dimmer the farther it is. */
    private static void pool(VertexConsumer out, PartLayers.LampDraw d, float dark, Vec3 eye, net.minecraft.world.level.Level level) {
        Lamp lamp = d.lamp();
        Matrix4f m = d.pose().pose();
        Matrix3f n = d.pose().normal();
        Vector3f at = m.transformPosition((float) lamp.at().x(), (float) lamp.at().y(), (float) lamp.at().z(), new Vector3f());
        Vector3f axis = n.transform((float) lamp.axis().x(), (float) lamp.axis().y(), (float) lamp.axis().z(), new Vector3f()).normalize();
        Vec3 from = eye.add(at.x, at.y, at.z), to = from.add(axis.x * lamp.rangeM(), axis.y * lamp.rangeM(), axis.z * lamp.rangeM());
        BlockHitResult hit = level.clip(new ClipContext(from, to, ClipContext.Block.VISUAL, ClipContext.Fluid.ANY, net.minecraft.world.phys.shapes.CollisionContext.empty()));
        if (hit.getType() != HitResult.Type.BLOCK) { return; }
        double travelled = hit.getLocation().distanceTo(from);
        if (travelled < 0.3) { return; }
        float strength = (float) (0.9 * dark * Math.min(1f, d.level()) * Math.pow(1 - travelled / lamp.rangeM(), 2));
        if (strength < 0.01f) { return; }
        Vector3f face = new Vector3f(hit.getDirection().getStepX(), hit.getDirection().getStepY(), hit.getDirection().getStepZ());
        Vec3 c = hit.getLocation().subtract(eye).add(face.x * 0.02, face.y * 0.02, face.z * 0.02);
        float size = (float) (travelled * Math.tan(Math.toRadians(lamp.beamDeg())) * 1.4 + 0.3);
        Vector3f u = new Vector3f(face).cross(0, 1, 0);
        if (u.lengthSquared() < 1e-6f) { u.set(1, 0, 0); }
        u.normalize().mul(size);
        Vector3f w = new Vector3f(face).cross(u).normalize().mul(size);
        int r = scaled(lamp.argb() >> 16, strength), g = scaled(lamp.argb() >> 8, strength), b = scaled(lamp.argb(), strength);
        float x = (float) c.x, y = (float) c.y, z = (float) c.z;
        vertex(out, x - u.x - w.x, y - u.y - w.y, z - u.z - w.z, 0, 1, r, g, b);
        vertex(out, x + u.x - w.x, y + u.y - w.y, z + u.z - w.z, 1, 1, r, g, b);
        vertex(out, x + u.x + w.x, y + u.y + w.y, z + u.z + w.z, 1, 0, r, g, b);
        vertex(out, x - u.x + w.x, y - u.y + w.y, z - u.z + w.z, 0, 0, r, g, b);
    }

    private static final java.util.Map<ResourceLocation, RenderType> TYPES = new java.util.HashMap<>();

    /** Added light with a texture without mipmaps (the halo is made here, at one size): Minecraft's eyes shader, no depth written. */
    private static RenderType type(ResourceLocation texture) {
        return TYPES.computeIfAbsent(texture, t -> RenderType.create("aerocore_lamp", com.mojang.blaze3d.vertex.DefaultVertexFormat.NEW_ENTITY,
                com.mojang.blaze3d.vertex.VertexFormat.Mode.QUADS, 1536, false, true,
                RenderType.CompositeState.builder().setTextureState(new net.minecraft.client.renderer.RenderStateShard.TextureStateShard(t, true, false))
                        .setShaderState(net.minecraft.client.renderer.RenderStateShard.RENDERTYPE_EYES_SHADER)
                        .setTransparencyState(net.minecraft.client.renderer.RenderStateShard.ADDITIVE_TRANSPARENCY)
                        .setCullState(net.minecraft.client.renderer.RenderStateShard.NO_CULL)
                        .setWriteMaskState(net.minecraft.client.renderer.RenderStateShard.COLOR_WRITE).createCompositeState(false)));
    }

    private static int scaled(int channel, float strength) { return Math.clamp((int) ((channel & 255) * strength + 0.5f), 0, 255); }

    /**
     * 0.30: the alpha light is written with. With Fabulous graphics the late stage draws into the translucent target, which
     * the transparency pass lays over the world by its alpha (dst * (1 - a) + rgb, a layer of alpha 0 left out): an alpha
     * of 255 made each halo an opaque black square. One step of alpha keeps the layer and adds the light, darkening what
     * is behind by 0.4 %; other graphics modes do not look at it.
     */
    private static final int LIGHT_ALPHA = 1;

    private static void vertex(VertexConsumer out, float x, float y, float z, float u, float v, int r, int g, int b) {
        out.addVertex(x, y, z).setColor(r, g, b, LIGHT_ALPHA).setUv(u, v).setOverlay(OverlayTexture.NO_OVERLAY).setLight(LightTexture.FULL_BRIGHT).setNormal(0, 1, 0);
    }

    /** 0.38: the halo's soft dot, for FieldLamps. */
    static ResourceLocation haloTexture() { return texture(true); }

    /** The halo's soft dot (made once), or plain white for the cones. */
    private static ResourceLocation texture(boolean soft) {
        if (soft && halo != null) { return halo; }
        if (!soft && white != null) { return white; }
        int size = soft ? 64 : 4;
        NativeImage image = new NativeImage(size, size, false);
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                double dx = (x + 0.5) / size - 0.5, dy = (y + 0.5) / size - 0.5, r = 2 * Math.sqrt(dx * dx + dy * dy);
                int v = soft ? (int) Math.round(255 * Math.max(0, Math.exp(-Math.pow(r / 0.42, 2)) - 0.0067) / (1 - 0.0067)) : 255;
                image.setPixelRGBA(x, y, 0xff000000 | (v << 16) | (v << 8) | v);
            }
        }
        ResourceLocation key = ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, soft ? "lamp_halo" : "lamp_white");
        Minecraft.getInstance().getTextureManager().register(key, new DynamicTexture(image));
        if (soft) { halo = key; } else { white = key; }
        return key;
    }
}
