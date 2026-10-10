package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.probe.SeaSurface;
import net.minecraft.client.Camera;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.world.level.material.FogType;
import net.minecraft.world.phys.Vec3;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Underwater fog follows the drawn sea surface: inside a sea region the camera is in water exactly when it is below
 * the moving surface at the frame's time, the same rule the renderer and the entity eye check use. Lava and powder
 * snow results are left alone.
 */
@Mixin(Camera.class)
abstract class CameraFogMixin {
    @Shadow private Vec3 position;
    @Shadow private boolean initialized;

    @Inject(method = "getFluidInCamera", at = @At("RETURN"), cancellable = true)
    private void aerocore$seaFog(CallbackInfoReturnable<FogType> result) {
        FogType vanilla = result.getReturnValue();
        if (!initialized || vanilla == FogType.LAVA || vanilla == FogType.POWDER_SNOW) { return; }
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        if (level == null) { return; }
        double time = (level.getGameTime() + minecraft.getTimer().getGameTimeDeltaPartialTick(false)) * MinecraftUnits.TICK_SECONDS;
        Boolean under = SeaSurface.submerged(level, position.x, position.y, position.z, time);
        if (under != null) { result.setReturnValue(under ? FogType.WATER : FogType.NONE); }
    }
}
