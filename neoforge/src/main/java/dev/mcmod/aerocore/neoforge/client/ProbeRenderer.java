package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.PhysicsProbe;
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

/** Procedural sphere with latitude bands; no external texture assets required. */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public final class ProbeRenderer extends EntityRenderer<PhysicsProbe> {
    private static final ResourceLocation TEXTURE = ResourceLocation.withDefaultNamespace("textures/block/white_concrete.png");
    private static final int LATITUDES = 12, LONGITUDES = 24;

    public ProbeRenderer(EntityRendererProvider.Context context) {
        super(context);
        shadowRadius = 0.5f;
    }

    @SubscribeEvent public static void register(EntityRenderersEvent.RegisterRenderers event) {
        event.registerEntityRenderer(ProbeEntities.PHYSICS_PROBE.get(), ProbeRenderer::new);
    }

    @Override public void render(PhysicsProbe probe, float yaw, float partialTick, PoseStack stack,
                                 MultiBufferSource buffers, int light) {
        stack.pushPose();
        stack.translate(0, PhysicsProbe.RADIUS_M, 0);
        // Synced rigid-body orientation, so rolling and spin are visible on the bands.
        Orientation q = probe.visualOrientation(partialTick);
        stack.mulPose(new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
        VertexConsumer vertices = buffers.getBuffer(RenderType.entitySolid(TEXTURE));
        boolean heavy = probe.massKg() > 998.2 * 4.0 / 3 * Math.PI * Math.pow(PhysicsProbe.RADIUS_M, 3);
        for (int latitude = 0; latitude < LATITUDES; latitude++) {
            double a = Math.PI * latitude / LATITUDES, b = Math.PI * (latitude + 1) / LATITUDES;
            boolean stripe = latitude == LATITUDES / 2 || latitude == 2;
            for (int longitude = 0; longitude < LONGITUDES; longitude++) {
                // A dark meridian quarter makes rotation about the vertical axis visible too.
                boolean meridian = !stripe && longitude < LONGITUDES / 8;
                int red = stripe ? 245 : meridian ? 40 : heavy ? 228 : 48;
                int green = stripe ? 245 : meridian ? 40 : heavy ? 130 : 190;
                int blue = stripe ? 245 : meridian ? 60 : heavy ? 55 : 220;
                double c = 2 * Math.PI * longitude / LONGITUDES, d = 2 * Math.PI * (longitude + 1) / LONGITUDES;
                // Outward winding for x=sin(theta)cos(phi), y=cos(theta), z=sin(theta)sin(phi).
                vertex(vertices, stack.last(), a, c, red, green, blue, light);
                vertex(vertices, stack.last(), a, d, red, green, blue, light);
                vertex(vertices, stack.last(), b, d, red, green, blue, light);
                vertex(vertices, stack.last(), b, c, red, green, blue, light);
            }
        }
        stack.popPose();
        super.render(probe, yaw, partialTick, stack, buffers, light);
    }

    private static void vertex(VertexConsumer consumer, PoseStack.Pose pose, double theta, double phi,
                               int red, int green, int blue, int light) {
        float x = (float) (Math.sin(theta) * Math.cos(phi)), y = (float) Math.cos(theta), z = (float) (Math.sin(theta) * Math.sin(phi));
        consumer.addVertex(pose, x * 0.5f, y * 0.5f, z * 0.5f).setColor(red, green, blue, 255)
                .setUv((float) (phi / (2 * Math.PI)), (float) (theta / Math.PI))
                .setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(pose, x, y, z);
    }

    @Override public ResourceLocation getTextureLocation(PhysicsProbe probe) { return TEXTURE; }
}
