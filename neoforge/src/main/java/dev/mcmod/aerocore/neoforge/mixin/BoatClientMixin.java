package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.probe.RowingBoats;
import net.minecraft.world.entity.vehicle.Boat;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.Constant;
import org.spongepowered.asm.mixin.injection.ModifyConstant;

/** The paddle animation interpolates over one tick's advance, which RowingBoats sets (client-only method). */
@Mixin(Boat.class)
abstract class BoatClientMixin {
    @ModifyConstant(method = "getRowingTime", constant = @Constant(floatValue = (float) (Math.PI / 8)))
    private float aerocore$paddleStep(float vanilla) { return RowingBoats.PADDLE_STEP; }
}
