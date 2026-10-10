package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.probe.VanillaWater;
import net.minecraft.world.entity.vehicle.Boat;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Vanilla boats float on the sea-region surface: their water level, in-water and under-water checks use the moving
 * surface instead of the static block tops, so vanilla's own boat buoyancy follows the waves and wakes.
 */
@Mixin(Boat.class)
abstract class BoatWaterMixin {
    @Shadow private double waterLevel;

    @Inject(method = "checkInWater", at = @At("RETURN"), cancellable = true)
    private void aerocore$seaInWater(CallbackInfoReturnable<Boolean> result) {
        Boat self = (Boat) (Object) this;
        double surface = VanillaWater.boatSurface(self, result.getReturnValueZ() ? waterLevel : Double.NaN);
        if (Double.isNaN(surface)) { return; }
        waterLevel = surface;
        result.setReturnValue(self.getBoundingBox().minY < surface);
    }

    @Inject(method = "isUnderwater", at = @At("RETURN"), cancellable = true)
    private void aerocore$seaUnderwater(CallbackInfoReturnable<Boat.Status> result) {
        if (result.getReturnValue() == Boat.Status.UNDER_FLOWING_WATER) { return; }
        Boat self = (Boat) (Object) this;
        double surface = VanillaWater.boatSurface(self, Double.NaN);
        if (Double.isNaN(surface)) { return; }
        // A crest washing over the boat is left to vanilla's in-water buoyancy, which grows with submergence; vanilla's
        // under-water rule (sink, eject after 3 s) applies only once the boat is a whole hull height under the surface.
        result.setReturnValue(self.getBoundingBox().maxY + self.getBbHeight() < surface ? Boat.Status.UNDER_WATER : null);
    }

    @Inject(method = "getWaterLevelAbove", at = @At("RETURN"), cancellable = true)
    private void aerocore$seaLevelAbove(CallbackInfoReturnable<Float> result) {
        double surface = VanillaWater.boatSurface((Boat) (Object) this, Double.NaN);
        if (!Double.isNaN(surface)) { result.setReturnValue((float) surface); }
    }
}
