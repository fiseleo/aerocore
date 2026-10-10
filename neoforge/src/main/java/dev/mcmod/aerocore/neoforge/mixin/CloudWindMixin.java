package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.client.SkyRenderer;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.LevelRenderer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.ModifyVariable;

/**
 * Clouds drift with the wind (SkyRenderer): the camera position the cloud pattern is laid out from is shifted back by the
 * wind's drift, and vanilla's own drift (0.03 blocks a tick toward -x) is taken out.
 */
@Mixin(LevelRenderer.class)
public abstract class CloudWindMixin {
    @Shadow private int ticks;

    @ModifyVariable(method = "renderClouds", at = @At("HEAD"), argsOnly = true, ordinal = 0)
    private double aerocore$cloudX(double camX) {
        if (!SkyRenderer.active()) { return camX; }
        float partial = Minecraft.getInstance().getTimer().getGameTimeDeltaPartialTick(false);
        return camX - SkyRenderer.cloudOffsetX(partial) - (ticks + partial) * 0.03;
    }

    @ModifyVariable(method = "renderClouds", at = @At("HEAD"), argsOnly = true, ordinal = 2)
    private double aerocore$cloudZ(double camZ) {
        if (!SkyRenderer.active()) { return camZ; }
        return camZ - SkyRenderer.cloudOffsetZ(Minecraft.getInstance().getTimer().getGameTimeDeltaPartialTick(false));
    }
}
