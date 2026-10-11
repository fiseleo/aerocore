package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.math.Axis;
import dev.mcmod.aerocore.api.HullSection;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.PlaningHull;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.BallastDesign;
import dev.mcmod.aerocore.api.vehicle.BoxDesign;
import dev.mcmod.aerocore.api.vehicle.ControlSurfaceDesign;
import dev.mcmod.aerocore.api.vehicle.FloatDesign;
import dev.mcmod.aerocore.api.vehicle.HullDesign;
import dev.mcmod.aerocore.api.vehicle.MotorDesign;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PlaningHullDesign;
import dev.mcmod.aerocore.api.vehicle.PropellerDesign;
import dev.mcmod.aerocore.api.vehicle.SeatDesign;
import dev.mcmod.aerocore.api.vehicle.Vehicle;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.vehicle.PartVehicle;
import dev.mcmod.aerocore.neoforge.vehicle.VehicleRegistry;
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
import java.util.Map;
import java.util.WeakHashMap;

/**
 * A plain look for any part-built vehicle, generated from its blueprint: lofted hulls, spheres for floats, boxes,
 * turning propellers, deflecting control surfaces. AeroCore uses it for its own vehicle type; a vehicle mod normally
 * draws its parts its own way, but may register this for its types too (new PartVehicleRenderer(context)).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public class PartVehicleRenderer extends EntityRenderer<PartVehicle> {
    private static final ResourceLocation TEXTURE = ResourceLocation.withDefaultNamespace("textures/block/white_concrete.png");
    private static final int AROUND = 24, HULL = 0xffd9d6cc, BOTTOM = 0xff9b3b30, FLOAT = 0xffc9502e, BOX = 0xffb5895a, METAL = 0xff8a8f96,
            BLADE = 0xffc8913a, MOTOR = 0xff3d4a5c, BALLAST = 0xff2f2f2f, SEAT = 0xff5a7a3a;
    private record Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal) {}
    /** The parts that do not move, in the vehicle frame, by compiled vehicle. */
    private static final Map<Vehicle, List<Quad>> STATIC = new WeakHashMap<>();

    public PartVehicleRenderer(EntityRendererProvider.Context context) { super(context); shadowRadius = 1f; }

    @SubscribeEvent public static void register(EntityRenderersEvent.RegisterRenderers event) {
        event.registerEntityRenderer(VehicleRegistry.PART_VEHICLE.get(), PartVehicleRenderer::new);
    }

    @Override public void render(PartVehicle entity, float yaw, float partialTick, PoseStack stack, MultiBufferSource buffers, int light) {
        Vehicle vehicle = entity.vehicle();
        if (vehicle != null) {
            stack.pushPose();
            Orientation q = entity.visualOrientation(partialTick);
            stack.mulPose(new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
            Vec3d com = vehicle.properties().centerOfMassLocal();
            stack.translate(-com.x(), -com.y(), -com.z());
            VertexConsumer out = buffers.getBuffer(RenderType.entitySolid(TEXTURE));
            for (Quad quad : staticMesh(vehicle)) {
                for (Vec3d corner : new Vec3d[]{quad.a(), quad.b(), quad.c(), quad.d()}) { vertex(out, stack, corner, quad.color(), quad.normal(), light); }
            }
            int rotor = 0;
            for (Vehicle.Installed installed : vehicle.installed()) {
                PartDesign design = installed.part().design();
                if (design instanceof PropellerDesign p) {
                    stack.pushPose();
                    mount(stack, installed);
                    stack.mulPose(Axis.XP.rotation((float) entity.rotorAngle(rotor, partialTick)));
                    double r = p.diameterM() / 2, hub = p.hubRatio() * r, chord = p.chordRatio() * p.diameterM();
                    box(out, stack, -0.15 * p.diameterM(), -hub, -hub, 0.15 * p.diameterM(), hub, hub, METAL, light);
                    for (int b = 0; b < p.blades(); b++) {
                        stack.pushPose();
                        stack.mulPose(Axis.XP.rotation((float) (2 * Math.PI * b / p.blades())));
                        stack.mulPose(Axis.YP.rotation((float) Math.atan(p.pitchRatio() / (0.7 * Math.PI))));
                        box(out, stack, -0.01, hub, -chord / 2, 0.01, r, chord / 2, BLADE, light);
                        stack.popPose();
                    }
                    stack.popPose();
                } else if (design instanceof ControlSurfaceDesign s) {
                    stack.pushPose();
                    mount(stack, installed);
                    double angle = Math.clamp(entity.input(installed.id(), "deflection"), -1, 1) * Math.toRadians(s.maxDeflectionDeg());
                    // Positive turns the trailing edge (aft, -x) toward +z.
                    stack.mulPose(Axis.YP.rotation((float) angle));
                    double t = s.thicknessRatio() * s.chordM() / 2;
                    box(out, stack, -0.75 * s.chordM(), -s.spanM() / 2, -t, 0.25 * s.chordM(), s.spanM() / 2, t, METAL, light);
                    stack.popPose();
                }
                rotor += installed.part().rotors().size();
            }
            stack.popPose();
        }
        super.render(entity, yaw, partialTick, stack, buffers, light);
    }

    private static void mount(PoseStack stack, Vehicle.Installed installed) {
        Vec3d p = installed.mount().position();
        Orientation r = installed.rotation();
        stack.translate(p.x(), p.y(), p.z());
        stack.mulPose(new Quaternionf((float) r.x(), (float) r.y(), (float) r.z(), (float) r.w()));
    }

    private static synchronized List<Quad> staticMesh(Vehicle vehicle) {
        List<Quad> cached = STATIC.get(vehicle);
        if (cached != null) { return cached; }
        List<Quad> quads = new ArrayList<>();
        for (Vehicle.Installed installed : vehicle.installed()) {
            PartDesign design = installed.part().design();
            List<Quad> local = new ArrayList<>();
            if (design instanceof HullDesign h) { loft(local, h.geometry(), Double.NaN); }
            else if (design instanceof PlaningHullDesign p) {
                PlaningHull hull = PlaningHull.fridsma(p.lengthM(), p.chineBeamM(), Math.toRadians(p.deadriseDeg()), p.depthM(), Math.min(p.stations(), 48));
                loft(local, hull.hull(), p.chineBeamM() / 2 * Math.tan(Math.toRadians(p.deadriseDeg())));
            } else if (design instanceof FloatDesign f) { sphere(local, f.radiusM(), FLOAT); }
            else if (design instanceof BoxDesign b) { box(local, b.sizeM().scale(-0.5), b.sizeM().scale(0.5), BOX); }
            else if (design instanceof MotorDesign m) {
                double side = Math.cbrt(m.massKg() / 4000) / 2;
                box(local, new Vec3d(-side, -side, -side), new Vec3d(side, side, side), MOTOR);
            } else if (design instanceof BallastDesign b) {
                double side = Math.cbrt(b.massKg() / 11340) / 2;
                box(local, new Vec3d(-side, -side, -side), new Vec3d(side, side, side), BALLAST);
            } else if (design instanceof SeatDesign) {
                box(local, new Vec3d(-0.2, 0, -0.2), new Vec3d(0.2, 0.08, 0.2), SEAT);
            }
            for (Quad quad : local) {
                quads.add(new Quad(installed.mount().apply(quad.a()), installed.mount().apply(quad.b()), installed.mount().apply(quad.c()),
                        installed.mount().apply(quad.d()), quad.color(), installed.rotation().rotate(quad.normal())));
            }
        }
        cached = List.copyOf(quads);
        STATIC.put(vehicle, cached);
        return cached;
    }

    /** Sections resampled on AROUND rays from their centroids, lofted station to station, ends closed. bottomBelow colours the bottom. */
    private static void loft(List<Quad> quads, SlenderHull hull, double bottomBelow) {
        List<Vec3d[]> rings = new ArrayList<>();
        for (HullSection s : hull.sections()) { rings.add(ring(s)); }
        double low = Double.MAX_VALUE;
        for (HullSection s : hull.sections()) { low = Math.min(low, s.minY()); }
        double line = Double.isNaN(bottomBelow) ? low + 0.4 * (hull.sections().get(hull.stations() / 2).maxY() - low) : bottomBelow + 1e-3;
        for (int i = 0; i + 1 < rings.size(); i++) {
            Vec3d[] a = rings.get(i), b = rings.get(i + 1);
            for (int k = 0; k < AROUND; k++) {
                int n = (k + 1) % AROUND;
                Vec3d normal = a[n].subtract(a[k]).cross(b[k].subtract(a[k]));
                quads.add(new Quad(a[k], a[n], b[n], b[k], 0.5 * (a[k].y() + a[n].y()) < line ? BOTTOM : HULL, unit(normal)));
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
                quads.add(new Quad(centre, first, second, second, HULL, normal));
            }
        }
    }

    /** Boundary points of a convex section on rays from its centroid, from straight up turning to starboard. */
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
            ring[k] = new Vec3d(s.xM(), full.centroidY() + (Double.isFinite(t) ? t : 0) * dy, full.centroidZ() + (Double.isFinite(t) ? t : 0) * dz);
        }
        return ring;
    }

    private static void sphere(List<Quad> quads, double r, int color) {
        int bands = 8, around = 16;
        for (int i = 0; i < bands; i++) for (int j = 0; j < around; j++) {
            Vec3d a = polar(r, i, j, bands, around), b = polar(r, i, j + 1, bands, around), c = polar(r, i + 1, j + 1, bands, around), d = polar(r, i + 1, j, bands, around);
            quads.add(new Quad(a, d, c, b, color, unit(a.add(b).add(c).add(d))));
        }
    }

    private static Vec3d polar(double r, int i, int j, int bands, int around) {
        double theta = Math.PI * i / bands, phi = 2 * Math.PI * j / around;
        return new Vec3d(r * Math.sin(theta) * Math.cos(phi), r * Math.cos(theta), r * Math.sin(theta) * Math.sin(phi));
    }

    private static void box(List<Quad> quads, Vec3d lo, Vec3d hi, int color) {
        Vec3d[] v = {new Vec3d(lo.x(), lo.y(), lo.z()), new Vec3d(hi.x(), lo.y(), lo.z()), new Vec3d(hi.x(), hi.y(), lo.z()), new Vec3d(lo.x(), hi.y(), lo.z()),
                new Vec3d(lo.x(), lo.y(), hi.z()), new Vec3d(hi.x(), lo.y(), hi.z()), new Vec3d(hi.x(), hi.y(), hi.z()), new Vec3d(lo.x(), hi.y(), hi.z())};
        int[][] faces = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 1, 5, 4}, {3, 7, 6, 2}};
        Vec3d[] normals = {Vec3d.Z.scale(-1), Vec3d.Z, Vec3d.X.scale(-1), Vec3d.X, Vec3d.Y.scale(-1), Vec3d.Y};
        for (int f = 0; f < 6; f++) { quads.add(new Quad(v[faces[f][0]], v[faces[f][1]], v[faces[f][2]], v[faces[f][3]], color, normals[f])); }
    }

    private static Vec3d unit(Vec3d v) { return v.lengthSquared() < 1e-18 ? Vec3d.Y : v.normalized(); }

    private static void vertex(VertexConsumer out, PoseStack stack, Vec3d p, int color, Vec3d n, int light) {
        out.addVertex(stack.last(), (float) p.x(), (float) p.y(), (float) p.z()).setColor(color).setUv(0, 0)
                .setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(stack.last(), (float) n.x(), (float) n.y(), (float) n.z());
    }

    private static void box(VertexConsumer out, PoseStack s, double x0, double y0, double z0, double x1, double y1, double z1, int color, int light) {
        List<Quad> quads = new ArrayList<>();
        box(quads, new Vec3d(x0, y0, z0), new Vec3d(x1, y1, z1), color);
        for (Quad q : quads) { for (Vec3d c : new Vec3d[]{q.a(), q.b(), q.c(), q.d()}) { vertex(out, s, c, q.color(), q.normal(), light); } }
    }

    @Override public ResourceLocation getTextureLocation(PartVehicle entity) { return TEXTURE; }
}
