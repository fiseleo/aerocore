package dev.mcmod.aerocore.neoforge.mixin;

import com.mojang.blaze3d.vertex.PoseStack;
import dev.mcmod.aerocore.neoforge.probe.RowingBoats;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.entity.LivingEntityRenderer;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.vehicle.Boat;
import org.joml.Quaternionf;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * A living rider of a boat is drawn turned with the hull's pitch and roll about its feet, which BoatSeatMixin has
 * already moved to the tilted seat: together the rider moves with the hull as one body.
 */
@Mixin(LivingEntityRenderer.class)
abstract class RiderTiltMixin {
    @Inject(method = "render(Lnet/minecraft/world/entity/LivingEntity;FFLcom/mojang/blaze3d/vertex/PoseStack;Lnet/minecraft/client/renderer/MultiBufferSource;I)V",
            at = @At(value = "INVOKE", target = "Lnet/minecraft/client/renderer/entity/LivingEntityRenderer;setupRotations(Lnet/minecraft/world/entity/LivingEntity;Lcom/mojang/blaze3d/vertex/PoseStack;FFFF)V"))
    private void aerocore$tilt(LivingEntity entity, float yaw, float partialTick, PoseStack pose, MultiBufferSource buffers, int light, CallbackInfo ci) {
        if (!(entity.getVehicle() instanceof Boat boat)) { return; }
        Quaternionf q = RowingBoats.tilt(boat, partialTick);
        if (q != null) { pose.mulPose(q); }
    }
}
