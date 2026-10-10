package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.probe.RowingBoats;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityDimensions;
import net.minecraft.world.entity.vehicle.Boat;
import net.minecraft.world.phys.Vec3;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** Riders sit where the tilted hull carries their seat (RowingBoats.tiltSeat). */
@Mixin(Boat.class)
abstract class BoatSeatMixin {
    @Inject(method = "getPassengerAttachmentPoint", at = @At("RETURN"), cancellable = true)
    private void aerocore$tiltSeat(Entity passenger, EntityDimensions dimensions, float scale, CallbackInfoReturnable<Vec3> result) {
        Vec3 seat = RowingBoats.tiltSeat((Boat) (Object) this, result.getReturnValue());
        if (seat != null) { result.setReturnValue(seat); }
    }
}
