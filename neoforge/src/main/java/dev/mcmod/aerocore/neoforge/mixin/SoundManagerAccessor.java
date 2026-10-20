package dev.mcmod.aerocore.neoforge.mixin;

import net.minecraft.client.sounds.SoundEngine;
import net.minecraft.client.sounds.SoundManager;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/** The sound engine behind the manager (0.37, client.sound.SoundHooks): its executor runs on the sound thread. */
@Mixin(SoundManager.class)
public interface SoundManagerAccessor {
    @Accessor("soundEngine") SoundEngine aerocore$soundEngine();
}
