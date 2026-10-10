package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.probe.VanillaWater;
import it.unimi.dsi.fastutil.objects.Object2DoubleMap;
import net.minecraft.world.entity.Entity;
import net.neoforged.neoforge.fluids.FluidType;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Lets vanilla entities feel the sea-region surface: after vanilla's block scan, the water height and the eye fluid
 * are re-evaluated against the moving surface (see VanillaWater). Both sides run it, so client-moved players agree.
 */
@Mixin(Entity.class)
abstract class EntityWaterMixin {
    @Shadow protected Object2DoubleMap<FluidType> forgeFluidTypeHeight;
    @Shadow private FluidType forgeFluidTypeOnEyes;

    @Inject(method = "updateFluidHeightAndDoFluidPushing()V", at = @At("TAIL"))
    private void aerocore$seaSurface(CallbackInfo info) {
        VanillaWater.afterFluidScan((Entity) (Object) this, forgeFluidTypeHeight);
    }

    @Inject(method = "updateFluidOnEyes", at = @At("TAIL"))
    private void aerocore$seaEyes(CallbackInfo info) {
        forgeFluidTypeOnEyes = VanillaWater.eyeFluid((Entity) (Object) this, forgeFluidTypeOnEyes);
    }
}
