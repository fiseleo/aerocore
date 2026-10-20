package dev.mcmod.aerocore.neoforge.mixin;

import com.mojang.blaze3d.audio.SoundBuffer;
import dev.mcmod.aerocore.neoforge.client.sound.RuntimeSounds;
import net.minecraft.client.sounds.SoundBufferLibrary;
import net.minecraft.resources.ResourceLocation;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

import java.util.Map;
import java.util.concurrent.CompletableFuture;

/**
 * Sounds made in code (0.37, client.sound.RuntimeSounds): a static sound whose file would be
 * {@code <namespace>:sounds/runtime/...ogg} is built from the samples kept in Java instead of read from a resource
 * pack. It goes into the library's own cache, so a resource reload discards its OpenAL buffer with vanilla's and the
 * next use builds it again.
 */
@Mixin(SoundBufferLibrary.class)
abstract class SoundBufferLibraryMixin {
    @Shadow @Final private Map<ResourceLocation, CompletableFuture<SoundBuffer>> cache;

    @Inject(method = "getCompleteBuffer", at = @At("HEAD"), cancellable = true)
    private void aerocore$runtime(ResourceLocation file, CallbackInfoReturnable<CompletableFuture<SoundBuffer>> result) {
        if (!RuntimeSounds.isRuntimeFile(file)) { return; }
        SoundBuffer made = RuntimeSounds.buffer(file);
        if (made == null) { return; }
        result.setReturnValue(cache.computeIfAbsent(file, f -> CompletableFuture.completedFuture(made)));
    }
}
