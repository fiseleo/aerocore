package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.math.Axis;
import dev.mcmod.aerocore.api.*;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.PlaningBoat;
import dev.mcmod.aerocore.neoforge.probe.ProbeEntities;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.entity.EntityRenderer;
import net.minecraft.client.renderer.entity.EntityRendererProvider;
import net.minecraft.client.renderer.texture.OverlayTexture;
import net.minecraft.resources.ResourceLocation;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.EntityRenderersEvent;
import org.joml.Quaternionf;

import java.util.ArrayList;
import java.util.List;

/** Lofted planing hull from the planing demonstrator's sections, with propeller and rudder; all generated geometry. */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public final class PlaningRenderer extends EntityRenderer<PlaningBoat> {
    private static final ResourceLocation TEXTURE = ResourceLocation.withDefaultNamespace("textures/block/white_concrete.png");
    private static final int AROUND = 32, BOTTOM = 0xff9b3b30, TOPSIDE = 0xffe9e9e2, DECK = 0xffb5895a, METAL = 0xff8a8f96, BLADE = 0xffc8913a;
    /** Quads in model-local coordinates: 4 corners, a colour and a face normal each. */
    private record Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal) {}
    private static List<Quad> mesh;

    public PlaningRenderer(EntityRendererProvider.Context context) { super(context); shadowRadius = 2f; }
    @SubscribeEvent public static void register(EntityRenderersEvent.RegisterRenderers event) {
        event.registerEntityRenderer(ProbeEntities.PLANING_BOAT.get(), PlaningRenderer::new);
    }

    @Override public void render(PlaningBoat hull, float yaw, float partialTick, PoseStack stack, MultiBufferSource buffers, int light) {
        stack.pushPose();
        Orientation q = hull.visualOrientation(partialTick);
        stack.mulPose(new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
        Vec3d com = TestPlaningBoat.CENTER_OF_MASS;
        stack.translate(-com.x(), -com.y(), -com.z());
        VertexConsumer out = buffers.getBuffer(RenderType.entitySolid(TEXTURE));
        for (Quad quad : mesh()) {
            for (Vec3d corner : new Vec3d[]{quad.a(), quad.b(), quad.c(), quad.d()}) {
                vertex(out, stack, corner, quad.color(), quad.normal(), light);
            }
        }
        Propeller p = TestPlaningBoat.PROPELLER;
        Vec3d hub = p.positionLocal();
        box(out, stack, hub.x() - 0.04, hub.y() - 0.04, -0.04, hub.x() + 0.04, hub.y() + 0.04, 0.04, METAL, light);
        double turns = (hull.tickCount + partialTick) * 0.05 * hull.propellerRps();
        float angle = (float) (2 * Math.PI * (turns - Math.floor(turns)));
        for (int blade = 0; blade < 3; blade++) {
            stack.pushPose();
            stack.translate(hub.x(), hub.y(), hub.z());
            stack.mulPose(Axis.XP.rotation(angle + blade * (float) (2 * Math.PI / 3)));
            stack.mulPose(Axis.YP.rotationDegrees(35));
            box(out, stack, -0.008, 0.04, -0.045, 0.008, p.diameterM() / 2, 0.045, BLADE, light);
            stack.popPose();
        }
        Rudder r = TestPlaningBoat.RUDDER;
        stack.pushPose();
        stack.translate(r.positionLocal().x(), r.positionLocal().y(), r.positionLocal().z());
        // Positive angles move the trailing edge (aft, -x) to starboard (+z).
        stack.mulPose(Axis.YP.rotation((float) hull.rudderRadians()));
        box(out, stack, -0.75 * r.chordM(), -0.5 * r.spanM(), -0.012, 0.25 * r.chordM(), 0.5 * r.spanM(), 0.012, METAL, light);
        box(out, stack, -0.015, 0.5 * r.spanM(), -0.015, 0.015, TestPlaningBoat.DEPTH - r.positionLocal().y(), 0.015, METAL, light);
        stack.popPose();
        stack.popPose();
        super.render(hull, yaw, partialTick, stack, buffers, light);
    }

    /** Built once: each section resampled at AROUND rays from its centroid, lofted station to station, ends closed. */
    private static synchronized List<Quad> mesh() {
        if (mesh != null) { return mesh; }
        SlenderHull hull = TestPlaningBoat.HULL.hull();
        // Bottom colour up to the prismatic chine line.
        double waterline = TestPlaningBoat.BEAM / 2 * Math.tan(TestPlaningBoat.DEADRISE) + 1e-3;
        List<Vec3d[]> rings = new ArrayList<>();
        for (HullSection s : hull.sections()) { rings.add(ring(s)); }
        List<Quad> quads = new ArrayList<>();
        for (int i = 0; i + 1 < rings.size(); i++) {
            Vec3d[] a = rings.get(i), b = rings.get(i + 1);
            for (int k = 0; k < AROUND; k++) {
                int n = (k + 1) % AROUND;
                // Rings turn from straight up towards starboard, so this order is counter-clockwise seen from outside.
                Vec3d normal = a[n].subtract(a[k]).cross(b[k].subtract(a[k]));
                quads.add(new Quad(a[k], a[n], b[n], b[k], colour(a[k], a[n], waterline), unit(normal)));
            }
        }
        for (int end = 0; end < 2; end++) {
            Vec3d[] ring = rings.get(end == 0 ? 0 : rings.size() - 1);
            Vec3d centre = Vec3d.ZERO;
            for (Vec3d v : ring) { centre = centre.add(v.scale(1.0 / AROUND)); }
            Vec3d normal = end == 0 ? Vec3d.X.scale(-1) : Vec3d.X;
            for (int k = 0; k < AROUND; k++) {
                int n = (k + 1) % AROUND;
                Vec3d first = end == 0 ? ring[n] : ring[k], second = end == 0 ? ring[k] : ring[n];
                quads.add(new Quad(centre, first, second, second, colour(ring[k], ring[n], waterline), normal));
            }
        }
        mesh = List.copyOf(quads);
        return mesh;
    }
    /** Boundary points of a convex section on rays from its centroid, starting from straight up and turning to starboard. */
    private static Vec3d[] ring(HullSection s) {
        HullSection.Wet full = s.clip(1, 0, 1e9);
        Vec3d[] ring = new Vec3d[AROUND];
        for (int k = 0; k < AROUND; k++) {
            double angle = 2 * Math.PI * k / AROUND, dy = Math.cos(angle), dz = Math.sin(angle), t = Double.POSITIVE_INFINITY;
            List<double[]> v = s.vertices();
            for (int i = 0; i < v.size(); i++) {
                double[] p = v.get(i), q = v.get((i + 1) % v.size());
                double ey = q[0] - p[0], ez = q[1] - p[1], denominator = dy * ez - dz * ey;
                if (Math.abs(denominator) < 1e-12) { continue; }
                double oy = p[0] - full.centroidY(), oz = p[1] - full.centroidZ();
                double along = (oy * ez - oz * ey) / denominator, edge = (oy * dz - oz * dy) / denominator;
                if (along > 0 && edge >= -1e-9 && edge <= 1 + 1e-9) { t = Math.min(t, along); }
            }
            ring[k] = new Vec3d(s.xM(), full.centroidY() + t * dy, full.centroidZ() + t * dz);
        }
        return ring;
    }
    private static int colour(Vec3d a, Vec3d b, double waterline) {
        if (Math.abs(a.y() - TestPlaningBoat.DEPTH) < 1e-6 && Math.abs(b.y() - TestPlaningBoat.DEPTH) < 1e-6) { return DECK; }
        return 0.5 * (a.y() + b.y()) < waterline ? BOTTOM : TOPSIDE;
    }
    private static Vec3d unit(Vec3d v) { return v.lengthSquared() < 1e-18 ? Vec3d.Y : v.normalized(); }
    private static void vertex(VertexConsumer out, PoseStack stack, Vec3d p, int color, Vec3d n, int light) {
        out.addVertex(stack.last(), (float) p.x(), (float) p.y(), (float) p.z()).setColor(color).setUv(0, 0)
                .setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(stack.last(), (float) n.x(), (float) n.y(), (float) n.z());
    }
    private static void box(VertexConsumer out, PoseStack s, double x0, double y0, double z0, double x1, double y1, double z1, int color, int light) {
        double[][] v = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
        int[][] faces = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 1, 5, 4}, {3, 7, 6, 2}};
        Vec3d[] normals = {Vec3d.Z.scale(-1), Vec3d.Z, Vec3d.X.scale(-1), Vec3d.X, Vec3d.Y.scale(-1), Vec3d.Y};
        for (int face = 0; face < 6; face++) for (int i = 0; i < 4; i++) {
            double[] p = v[faces[face][i]];
            vertex(out, s, new Vec3d(p[0], p[1], p[2]), color, normals[face], light);
        }
    }
    @Override public ResourceLocation getTextureLocation(PlaningBoat hull) { return TEXTURE; }
}
