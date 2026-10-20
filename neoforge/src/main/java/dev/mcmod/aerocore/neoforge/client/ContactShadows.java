package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.platform.NativeImage;
import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.neoforge.vehicle.PartVehicle;
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

import java.util.Map;
import java.util.WeakHashMap;

/**
 * Contact shadows (0.39, beautified shading): a soft dark patch on the ground under a vehicle, the size of what is
 * near its bottom, following the ground (a 3 x 3 grid of points, each a ray down, looked again only when the vehicle
 * has moved 5 cm or every 4 ticks), darker the closer the vehicle sits and fading as it rises (gone 3 m up), weaker in
 * the shade (it is the sky's light being shut out) and on water. Drawn with the vehicle as Minecraft draws its round
 * shadows (which it replaces while on). Display only. Render thread.
 */
final class ContactShadows {
    private ContactShadows() {}

    /** How far down rays look (metres), and the height at which a shadow has faded out. */
    private static final double REACH = 4, FADE = 3;
    private static ResourceLocation texture;

    /** A vehicle's footprint in its own frame (metres from its centre of mass: x forward-back, z side to side) and its bottom's height below it. */
    record Footprint(double x0, double x1, double z0, double z1, double bottom) {}

    private static final class Ground {
        double x, y, z, yaw;
        long tick = Long.MIN_VALUE / 2;
        final double[] height = new double[9];
        final boolean[] water = new boolean[9], hit = new boolean[9];
    }

    private static final Map<PartVehicle, Ground> GROUND = new WeakHashMap<>();

    /**
     * Draws a vehicle's shadow: its footprint about a world position (where its centre of mass is drawn) and heading
     * (its frame's x axis at (cos, sin) in the world's x, z), in a pose whose origin is at a world point (the entity's
     * position: the pose before it is turned).
     */
    static void draw(PartVehicle entity, Footprint f, Vec3 at, double yaw, Vec3 origin, PoseStack pose, MultiBufferSource buffers) {
        var level = entity.level();
        Ground g = GROUND.computeIfAbsent(entity, e -> new Ground());
        long now = level.getGameTime();
        double c = Math.cos(yaw), s = Math.sin(yaw);
        // the soft edge reaches a third of the way in: the patch a little larger than the footprint
        double px = 0.12 * (f.x1() - f.x0()) + 0.1, pz = 0.12 * (f.z1() - f.z0()) + 0.1;
        double x0 = f.x0() - px, x1 = f.x1() + px, z0 = f.z0() - pz, z1 = f.z1() + pz;
        double[][] points = new double[9][];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double lx = x0 + (x1 - x0) * i / 2, lz = z0 + (z1 - z0) * j / 2;
                points[3 * i + j] = new double[]{at.x + lx * c - lz * s, at.z + lx * s + lz * c};
            }
        }
        boolean moved = Math.abs(at.x - g.x) + Math.abs(at.y - g.y) + Math.abs(at.z - g.z) > 0.05 || Math.abs(yaw - g.yaw) > 0.01;
        if (moved || now - g.tick >= 4 || now < g.tick) {
            g.x = at.x; g.y = at.y; g.z = at.z; g.yaw = yaw; g.tick = now;
            double top = at.y + f.bottom() + 0.5;
            for (int k = 0; k < 9; k++) {
                Vec3 from = new Vec3(points[k][0], top, points[k][1]);
                BlockHitResult hit = level.clip(new ClipContext(from, from.add(0, -REACH - 0.5, 0), ClipContext.Block.COLLIDER, ClipContext.Fluid.ANY,
                        net.minecraft.world.phys.shapes.CollisionContext.empty()));
                g.hit[k] = hit.getType() == HitResult.Type.BLOCK;
                g.height[k] = g.hit[k] ? hit.getLocation().y : Double.NaN;
                g.water[k] = g.hit[k] && !level.getFluidState(hit.getBlockPos()).isEmpty();
            }
        }
        double ground = Double.NEGATIVE_INFINITY;
        int water = 0;
        for (int k = 0; k < 9; k++) {
            if (!g.hit[k]) { return; }   // over a drop: no shadow
            ground = Math.max(ground, g.height[k]);
            if (g.water[k]) { water++; }
        }
        double h = Math.max(0, at.y + f.bottom() - ground);
        if (h >= FADE) { return; }
        float sky = level.getBrightness(net.minecraft.world.level.LightLayer.SKY, net.minecraft.core.BlockPos.containing(at.x, ground + 0.5, at.z)) / 15f;
        float strength = (float) (0.5 * Math.pow(1 - h / FADE, 2) * (0.35 + 0.65 * sky) * (water > 4 ? 0.5 : 1));
        if (strength < 0.01f) { return; }
        VertexConsumer out = buffers.getBuffer(RenderType.entityShadow(texture()));
        PoseStack.Pose p = pose.last();
        int alpha = Math.clamp((int) (255 * strength), 0, 255);
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                // wound as Minecraft's shadows (facing up; the type culls back faces)
                int[] corner = {3 * i + j, 3 * i + j + 1, 3 * (i + 1) + j + 1, 3 * (i + 1) + j};
                for (int k : corner) {
                    int ci = k / 3, cj = k % 3;
                    out.addVertex(p, (float) (points[k][0] - origin.x), (float) (g.height[k] + 0.015 - origin.y), (float) (points[k][1] - origin.z))
                            .setColor(255, 255, 255, alpha).setUv(ci / 2f, cj / 2f).setOverlay(OverlayTexture.NO_OVERLAY)
                            .setLight(LightTexture.FULL_BRIGHT).setNormal(p, 0, 1, 0);
                }
            }
        }
    }

    /** The soft rectangle (made once): black, its alpha falling off over the outer quarter on each side. */
    private static ResourceLocation texture() {
        if (texture != null) { return texture; }
        int size = 64;
        NativeImage image = new NativeImage(size, size, false);
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                double u = (x + 0.5) / size, v = (y + 0.5) / size;
                double a = edge(u) * edge(v);
                image.setPixelRGBA(x, y, (int) Math.round(255 * a) << 24);
            }
        }
        texture = Minecraft.getInstance().getTextureManager().register("aerocore_contact_shadow", new DynamicTexture(image));
        return texture;
    }

    private static double edge(double t) {
        double d = Math.min(t, 1 - t) / 0.3;   // 0 at the edge, 1 a third of the way in
        d = Math.clamp(d, 0, 1);
        return d * d * (3 - 2 * d);
    }
}
