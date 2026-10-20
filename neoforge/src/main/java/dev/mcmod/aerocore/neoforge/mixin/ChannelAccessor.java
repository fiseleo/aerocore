package dev.mcmod.aerocore.neoforge.mixin;

import com.mojang.blaze3d.audio.Channel;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/** A channel's OpenAL source (0.37, client.sound.Efx): read on the sound thread while the channel is alive. */
@Mixin(Channel.class)
public interface ChannelAccessor {
    @Accessor("source") int aerocore$source();
}
