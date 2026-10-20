package dev.mcmod.aerocore.neoforge.mixin;

import net.minecraft.client.sounds.SoundEngine;
import net.minecraft.client.sounds.SoundEngineExecutor;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/**
 * The sound engine's executor (the "Sound engine" thread, which owns every OpenAL call) and whether a device is open
 * (0.37, client.sound.SoundHooks). The executor object outlives a reload; its thread does not.
 */
@Mixin(SoundEngine.class)
public interface SoundEngineAccessor {
    @Accessor("executor") SoundEngineExecutor aerocore$executor();
    @Accessor("loaded") boolean aerocore$loaded();
}
