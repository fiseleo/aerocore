package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.client.SkyRenderer;
import net.minecraft.client.Camera;
import net.minecraft.client.renderer.DimensionSpecialEffects;
import net.minecraft.client.renderer.LevelRenderer;
import org.joml.Matrix4f;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.Redirect;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/** Paints the physical sky (SkyRenderer) over vanilla's flat sky colour, and leaves out vanilla's sunrise fan with it. */
@Mixin(LevelRenderer.class)
public abstract class SkyDomeMixin {
    @Inject(method = "renderSky", at = @At(value = "INVOKE", target = "Lcom/mojang/blaze3d/vertex/VertexBuffer;unbind()V", ordinal = 0, shift = At.Shift.AFTER))
    private void aerocore$physicalSky(Matrix4f modelView, Matrix4f projection, float partial, Camera camera, boolean foggy, Runnable setupFog, CallbackInfo ci) {
        SkyRenderer.drawDome(modelView, projection, partial);
    }

    @Redirect(method = "renderSky", at = @At(value = "INVOKE", target = "Lnet/minecraft/client/renderer/DimensionSpecialEffects;getSunriseColor(FF)[F"))
    private float[] aerocore$noSunriseFan(DimensionSpecialEffects effects, float timeOfDay, float partial) {
        return SkyRenderer.active() ? null : effects.getSunriseColor(timeOfDay, partial);
    }
}
