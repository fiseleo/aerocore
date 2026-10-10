package dev.mcmod.aerocore.neoforge.mixin;

import com.mojang.blaze3d.systems.RenderSystem;
import dev.mcmod.aerocore.neoforge.client.SkyRenderer;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.client.renderer.LightTexture;
import org.joml.Matrix4f;
import org.joml.Matrix4fStack;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Rain and snow lean with the wind (SkyRenderer.precipitationLean): vanilla draws its falling columns camera-relative,
 * so a shear x' = x - (U / v) y, z' = z - (W / v) y of the model view, applied first, tilts them the way drops fall
 * downwind.
 */
@Mixin(LevelRenderer.class)
public abstract class WeatherWindMixin {
    @Inject(method = "renderSnowAndRain", at = @At("HEAD"))
    private void aerocore$lean(LightTexture light, float partial, double camX, double camY, double camZ, CallbackInfo ci) {
        float[] lean = SkyRenderer.precipitationLean(camX, camY, camZ, partial);
        Matrix4fStack view = RenderSystem.getModelViewStack();
        view.pushMatrix();
        view.mul(new Matrix4f().m10(-lean[0]).m12(-lean[1]));
        RenderSystem.applyModelViewMatrix();
    }

    @Inject(method = "renderSnowAndRain", at = @At("RETURN"))
    private void aerocore$unlean(LightTexture light, float partial, double camX, double camY, double camZ, CallbackInfo ci) {
        RenderSystem.getModelViewStack().popMatrix();
        RenderSystem.applyModelViewMatrix();
    }
}
