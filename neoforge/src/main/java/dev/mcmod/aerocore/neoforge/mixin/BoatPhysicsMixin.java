package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.probe.RowingBoats;
import net.minecraft.world.entity.vehicle.Boat;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Constant;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.ModifyConstant;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Vanilla boats at the water move by RowingBoats (a rowed pram) instead of vanilla's floatBoat and controlBoat; the
 * paddles turn at the pram's stroke rate.
 */
@Mixin(Boat.class)
abstract class BoatPhysicsMixin {
    @Inject(method = "floatBoat", at = @At("HEAD"), cancellable = true)
    private void aerocore$float(CallbackInfo ci) {
        if (RowingBoats.floatBoat((Boat) (Object) this)) { ci.cancel(); }
    }

    @Inject(method = "controlBoat", at = @At("HEAD"), cancellable = true)
    private void aerocore$control(CallbackInfo ci) {
        if (RowingBoats.controlsBoat((Boat) (Object) this)) { ci.cancel(); }
    }

    @Inject(method = "tick", at = @At("TAIL"))
    private void aerocore$afterTick(CallbackInfo ci) { RowingBoats.afterTick((Boat) (Object) this); }

    /** Vanilla advances the paddles pi/8 per tick (75 strokes a minute). */
    @ModifyConstant(method = "tick", constant = @Constant(floatValue = (float) (Math.PI / 8)))
    private float aerocore$paddleStep(float vanilla) { return RowingBoats.PADDLE_STEP; }
}
