package dev.mcmod.aerocore.neoforge.mixin;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.math.Axis;
import dev.mcmod.aerocore.neoforge.probe.RowingBoats;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.entity.BoatRenderer;
import net.minecraft.world.entity.vehicle.Boat;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Boats are drawn with the pitch and roll RowingBoats integrates, turned about their bottom centre (the entity
 * position) right after vanilla's heading rotation. Riders are placed by vanilla and do not tilt with the hull.
 */
@Mixin(BoatRenderer.class)
abstract class BoatRendererMixin {
    @Inject(method = "render(Lnet/minecraft/world/entity/vehicle/Boat;FFLcom/mojang/blaze3d/vertex/PoseStack;Lnet/minecraft/client/renderer/MultiBufferSource;I)V",
            at = @At(value = "INVOKE", target = "Lcom/mojang/blaze3d/vertex/PoseStack;mulPose(Lorg/joml/Quaternionf;)V", ordinal = 0, shift = At.Shift.AFTER))
    private void aerocore$tilt(Boat boat, float yaw, float partialTick, PoseStack pose, MultiBufferSource buffers, int light, CallbackInfo ci) {
        float[] attitude = RowingBoats.attitude(boat, partialTick);
        if (attitude[0] == 0 && attitude[1] == 0) { return; }
        // After the heading rotation local -Z is forward and +X is the boat's right: +X rotation raises the bow,
        // -Z rotation lowers the right side.
        pose.translate(0.0F, -0.375F, 0.0F);
        pose.mulPose(Axis.XP.rotation(attitude[0]));
        pose.mulPose(Axis.ZP.rotation(-attitude[1]));
        pose.translate(0.0F, 0.375F, 0.0F);
    }
}
