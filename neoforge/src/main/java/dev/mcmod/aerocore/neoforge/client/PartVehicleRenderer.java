package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.math.Axis;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.Vehicle;
import dev.mcmod.aerocore.api.vehicle.part.BoxPart;
import dev.mcmod.aerocore.api.vehicle.part.ControlSurfacePart;
import dev.mcmod.aerocore.api.vehicle.part.HullPart;
import dev.mcmod.aerocore.api.vehicle.part.MassPart;
import dev.mcmod.aerocore.api.vehicle.part.MotorPart;
import dev.mcmod.aerocore.api.vehicle.part.PlaningHullPart;
import dev.mcmod.aerocore.api.vehicle.part.PropellerPart;
import dev.mcmod.aerocore.api.vehicle.part.SpherePart;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.vehicle.PartVehicle;
import dev.mcmod.aerocore.neoforge.vehicle.VehicleRegistry;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.entity.EntityRenderer;
import net.minecraft.client.renderer.entity.EntityRendererProvider;
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
 * A plain look for any part-built vehicle, drawn part by part with the PartRenderer registered for each part's class:
 * AeroCore's own parts as lofted hulls, spheres for floats, boxes, turning propellers and deflecting control surfaces;
 * a vehicle mod registers renderers for its part classes, or draws its vehicles its own way. AeroCore uses it for its
 * vehicle type; a vehicle mod may register it for its types too (new PartVehicleRenderer(context)).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public class PartVehicleRenderer extends EntityRenderer<PartVehicle> {
    private static final ResourceLocation TEXTURE = PartMesh.TEXTURE;
    private static final int FLOAT = 0xffc9502e, BOX = 0xffb5895a, METAL = 0xff8a8f96, BLADE = 0xffc8913a, MOTOR = 0xff3d4a5c, BALLAST = 0xff2f2f2f,
            SEAT = 0xff5a7a3a;
    /**
     * 0.29: the level of detail each part was last drawn at (changes with some hysteresis, so it does not flicker);
     * 0.30: by vehicle and part id (a compiled part is shared by every vehicle of a design, which would fight over one).
     */
    private static final Map<PartVehicle, Map<String, Integer>> LEVEL = new WeakHashMap<>();
    /** Projected size (radius over distance) above which a part is drawn at level 0, and at level 1. */
    private static final double LEVEL_0 = 0.06, LEVEL_1 = 0.02, HYSTERESIS = 1.2;

    static {
        PartRenderer.registerDefault(HullPart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) { out.loft(((HullPart) part).hull(), Double.NaN); }
        });
        PartRenderer.registerDefault(PlaningHullPart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) {
                PlaningHullPart p = (PlaningHullPart) part;
                out.loft(p.planing().hull(), p.chineBeamM() / 2 * Math.tan(Math.toRadians(p.deadriseDeg())));
            }
        });
        PartRenderer.registerDefault(SpherePart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) { out.sphere(((SpherePart) part).radiusM(), FLOAT); }
        });
        PartRenderer.registerDefault(BoxPart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) { Vec3d s = ((BoxPart) part).sizeM(); out.box(s.scale(-0.5), s.scale(0.5), BOX); }
        });
        PartRenderer.registerDefault(MotorPart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) { cube(out, Math.cbrt(part.mass().massKg() / 4000) / 2, MOTOR); }   // a motor's overall density, roughly
        });
        PartRenderer.registerDefault(MassPart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) {
                if (!part.seats().isEmpty()) { out.box(new Vec3d(-0.2, 0, -0.2), new Vec3d(0.2, 0.08, 0.2), SEAT); }
                else { cube(out, Math.cbrt(part.mass().massKg() / 11340) / 2, BALLAST); }   // drawn as lead
            }
        });
        // 0.30: a propeller and a control surface are one baked piece each (MeshBuilder.piece 1), turned by the renderer
        PartRenderer.registerDefault(PropellerPart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) {
                PropellerPart p = (PropellerPart) part;
                double d = p.diameterM(), r = d / 2, hub = p.hubRatio() * r, chord = p.chordRatio() * d;
                out.piece(1).box(new Vec3d(-0.15 * d, -hub, -hub), new Vec3d(0.15 * d, hub, hub), METAL);
                PartMesh blade = new PartMesh().piece(1).box(new Vec3d(-0.01, hub, -chord / 2), new Vec3d(0.01, r, chord / 2), BLADE);
                for (int b = 0; b < p.blades(); b++) {
                    Orientation turn = Orientation.rotationVector(Vec3d.X.scale(2 * Math.PI * b / p.blades()))
                            .multiply(Orientation.rotationVector(Vec3d.Y.scale(Math.atan(p.pitchRatio() / (0.7 * Math.PI)))));
                    out.add(blade, turn, Vec3d.ZERO);
                }
                out.piece(0);
            }

            @Override public void animate(Part part, PartVehicle entity, String partId, int firstRotor, float partialTick, PoseStack stack, PartLayers layers, int light, Drawn drawn) {
                if (drawn == null) { return; }
                stack.pushPose();
                stack.mulPose(Axis.XP.rotation((float) entity.rotorAngle(firstRotor, partialTick)));
                layers.draw(drawn.piece(1), stack, light);
                stack.popPose();
            }
        });
        PartRenderer.registerDefault(ControlSurfacePart.class, new PartRenderer() {
            @Override public void mesh(Part part, PartMesh out) {
                ControlSurfacePart s = (ControlSurfacePart) part;
                double t = s.thicknessRatio() * s.chordM() / 2;
                out.piece(1).box(new Vec3d(-0.75 * s.chordM(), -s.spanM() / 2, -t), new Vec3d(0.25 * s.chordM(), s.spanM() / 2, t), METAL).piece(0);
            }

            @Override public void animate(Part part, PartVehicle entity, String partId, int firstRotor, float partialTick, PoseStack stack, PartLayers layers, int light, Drawn drawn) {
                if (drawn == null) { return; }
                ControlSurfacePart s = (ControlSurfacePart) part;
                double angle = Math.clamp(entity.input(partId, "deflection"), -1, 1) * Math.toRadians(s.maxDeflectionDeg());
                // Positive turns the trailing edge (aft, -x) toward +z.
                stack.pushPose();
                stack.mulPose(Axis.YP.rotation((float) angle));
                layers.draw(drawn.piece(1), stack, light);
                stack.popPose();
            }
        });
    }

    private static void cube(PartMesh out, double half, int color) { out.box(new Vec3d(-half, -half, -half), new Vec3d(half, half, half), color); }

    public PartVehicleRenderer(EntityRendererProvider.Context context) { super(context); shadowRadius = 1f; }

    @SubscribeEvent public static void register(EntityRenderersEvent.RegisterRenderers event) {
        event.registerEntityRenderer(VehicleRegistry.PART_VEHICLE.get(), PartVehicleRenderer::new);
    }

    @Override public void render(PartVehicle entity, float yaw, float partialTick, PoseStack stack, MultiBufferSource buffers, int light) {
        Vehicle vehicle = entity.vehicle();
        if (vehicle != null) {
            stack.pushPose();
            // 0.36: a follower drawn with its coupling point on its leader's as that is drawn
            Vec3d shift = entity.drawnShift(partialTick);
            if (shift.lengthSquared() > 0) { stack.translate(shift.x(), shift.y(), shift.z()); }
            Orientation q = entity.visualOrientation(partialTick);
            stack.mulPose(new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
            Vec3d com = vehicle.properties().centerOfMassLocal();
            stack.translate(-com.x(), -com.y(), -com.z());
            PartLayers layers = new PartLayers().gpu(true).late(true);   // the parts' bakes drawn from the GPU (PartGpu); glass after water (0.30)
            net.minecraft.world.phys.Vec3 at = entity.getPosition(partialTick).add(shift.x(), shift.y(), shift.z());
            net.minecraft.world.phys.Vec3 camera = net.minecraft.client.Minecraft.getInstance().gameRenderer.getMainCamera().getPosition();
            int rotor = 0;
            boolean beautify = Beautify.on();
            shadowRadius = beautify ? 0 : 1f;   // 0.39: the contact shadow takes the place of Minecraft's round one
            List<double[]> near = new ArrayList<>();   // parts' boxes (vehicle frame from the centre of mass), for the shadow
            for (Vehicle.Installed installed : vehicle.installed()) {
                Part part = installed.part();
                PartRenderer renderer = PartRenderer.of(part);
                if (renderer != null) {
                    stack.pushPose();
                    mount(stack, installed);
                    MeshBakery.Baked baked = MeshBakery.get(part, () -> {
                        PartMesh local = new PartMesh();
                        renderer.mesh(part, local);
                        return local;
                    }, renderer.bake(part));
                    int partLight = light;
                    PartRenderer.Drawn drawnNow = null;
                    if (baked != null && baked.radius() > 0) {
                        // where the part is in the world: its own light and how big it looks
                        Vec3d local = installed.mount().position().add(installed.rotation().rotate(baked.centre())).subtract(com);
                        if (beautify) { near.add(box(installed, baked, com)); }
                        Vec3d offset = q.rotate(local);
                        double wx = at.x + offset.x(), wy = at.y + offset.y(), wz = at.z + offset.z();
                        partLight = lightAt(entity, wx, wy, wz, light);
                        double dx = wx - camera.x, dy = wy - camera.y, dz = wz - camera.z;
                        double distance = Math.max(1e-3, Math.sqrt(dx * dx + dy * dy + dz * dz));
                        int levelNow = level(entity, installed.id(), baked.radius() / distance, baked.levels().length - 1);
                        var drawn = baked.level(levelNow);
                        float[] lampLevels = new float[baked.lamps().size()];
                        if (lampLevels.length > 0) { renderer.lamps(part, entity, installed.id(), partialTick, baked.lamps(), lampLevels); }
                        drawnNow = new PartRenderer.Drawn(baked, levelNow, lampLevels);
                        layers.draw(drawn, stack, partLight);
                        layers.lamps(drawnNow, 0, stack);   // the fixed part's lamps (pieces' lamps their renderers draw)
                        int glow = renderer.glow(part, entity, installed.id(), partialTick);
                        if ((glow >>> 24) > 0) { layers.glow(drawn, stack, glow, true); }   // hot metal, over the solid drawing
                    } else if (baked != null) {
                        drawnNow = new PartRenderer.Drawn(baked, 0);
                    }
                    renderer.animate(part, entity, installed.id(), rotor, partialTick, stack, layers, partLight, drawnNow);
                    stack.popPose();
                }
                rotor += part.rotors().size();
            }
            stack.popPose();
            layers.flush(buffers);
            ContactShadows.Footprint f = beautify ? footprint(near) : null;
            if (f != null) {
                Vec3d x = q.rotate(new Vec3d(1, 0, 0));
                if (x.x() * x.x() + x.z() * x.z() > 1e-6) {
                    ContactShadows.draw(entity, f, at, Math.atan2(x.z(), x.x()), entity.getPosition(partialTick), stack, buffers);
                }
            }
        }
        super.render(entity, yaw, partialTick, stack, buffers, light);
    }

    /** 0.39: baked meshes' boxes (their farthest level's), kept while the mesh is. */
    private static final Map<dev.mcmod.aerocore.api.mesh.PackedMesh, float[]> BOUNDS = new WeakHashMap<>();

    /** A part's box in the vehicle's frame (from its centre of mass): x0, y0, z0, x1, y1, z1. */
    private static double[] box(Vehicle.Installed installed, MeshBakery.Baked baked, Vec3d com) {
        dev.mcmod.aerocore.api.mesh.PackedMesh far = baked.level(baked.levels().length - 1);
        float[] b = BOUNDS.computeIfAbsent(far, dev.mcmod.aerocore.api.mesh.PackedMesh::bounds);
        double[] out = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (int i = 0; i < 8; i++) {
            Vec3d c = new Vec3d(b[(i & 1) == 0 ? 0 : 3], b[(i & 2) == 0 ? 1 : 4], b[(i & 4) == 0 ? 2 : 5]);
            Vec3d v = installed.mount().position().add(installed.rotation().rotate(c)).subtract(com);
            out[0] = Math.min(out[0], v.x()); out[1] = Math.min(out[1], v.y()); out[2] = Math.min(out[2], v.z());
            out[3] = Math.max(out[3], v.x()); out[4] = Math.max(out[4], v.y()); out[5] = Math.max(out[5], v.z());
        }
        return out;
    }

    /**
     * 0.39: the shadow's footprint: the parts near the vehicle's bottom (reaching down within 1.2 m of its lowest
     * point: a rotor's disc, a mast, a high wing left out unless low).
     */
    private static ContactShadows.Footprint footprint(List<double[]> parts) {
        double bottom = Double.POSITIVE_INFINITY;
        for (double[] p : parts) { bottom = Math.min(bottom, p[1]); }
        if (!Double.isFinite(bottom)) { return null; }
        double x0 = Double.POSITIVE_INFINITY, x1 = Double.NEGATIVE_INFINITY, z0 = Double.POSITIVE_INFINITY, z1 = Double.NEGATIVE_INFINITY;
        for (double[] p : parts) {
            if (p[1] > bottom + 1.2) { continue; }
            x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[3]); z0 = Math.min(z0, p[2]); z1 = Math.max(z1, p[5]);
        }
        if (!(x1 > x0) || !(z1 > z0)) { return null; }
        return new ContactShadows.Footprint(x0, x1, z0, z1, bottom);
    }

    /** The packed light at a point: the block's there, or the vehicle's own where the point is inside a solid block. */
    private static int lightAt(PartVehicle entity, double x, double y, double z, int fallback) {
        net.minecraft.core.BlockPos pos = net.minecraft.core.BlockPos.containing(x, y, z);
        var level = entity.level();
        if (level.getBlockState(pos).isSolidRender(level, pos)) { return fallback; }
        return net.minecraft.client.renderer.LevelRenderer.getLightColor(level, pos);
    }

    /** The level of detail for a part that looks a size (radius over distance), moving a level only past some hysteresis. */
    private static int level(PartVehicle entity, String partId, double size, int last) {
        Map<String, Integer> levels = LEVEL.computeIfAbsent(entity, e -> new java.util.HashMap<>());
        int was = levels.getOrDefault(partId, -1), now;
        if (was < 0) { now = size > LEVEL_0 ? 0 : size > LEVEL_1 ? 1 : 2; }
        else {
            now = was;
            if (was == 0 && size < LEVEL_0 / HYSTERESIS) { now = size > LEVEL_1 ? 1 : 2; }
            else if (was == 1 && size > LEVEL_0 * HYSTERESIS) { now = 0; }
            else if (was == 1 && size < LEVEL_1 / HYSTERESIS) { now = 2; }
            else if (was >= 2 && size > LEVEL_1 * HYSTERESIS) { now = size > LEVEL_0 ? 0 : 1; }
        }
        levels.put(partId, now);
        return Math.min(now, last);
    }

    private static void mount(PoseStack stack, Vehicle.Installed installed) {
        Vec3d p = installed.mount().position();
        Orientation r = installed.rotation();
        stack.translate(p.x(), p.y(), p.z());
        stack.mulPose(new Quaternionf((float) r.x(), (float) r.y(), (float) r.z(), (float) r.w()));
    }

    @Override public ResourceLocation getTextureLocation(PartVehicle entity) { return TEXTURE; }
}
