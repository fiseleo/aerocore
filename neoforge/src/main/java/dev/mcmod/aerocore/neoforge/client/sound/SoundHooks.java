package dev.mcmod.aerocore.neoforge.client.sound;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.mixin.GuiAccessor;
import dev.mcmod.aerocore.neoforge.mixin.SoundEngineAccessor;
import dev.mcmod.aerocore.neoforge.mixin.SoundManagerAccessor;
import net.minecraft.client.Minecraft;
import net.minecraft.client.resources.sounds.SimpleSoundInstance;
import net.minecraft.client.sounds.SoundEngine;
import net.minecraft.client.sounds.SoundEngineExecutor;
import net.minecraft.client.sounds.WeighedSoundEvents;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.sounds.SoundEvent;
import net.minecraft.sounds.SoundSource;
import net.minecraft.util.RandomSource;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.sound.PlaySoundSourceEvent;
import net.neoforged.neoforge.client.event.sound.PlayStreamingSourceEvent;
import net.neoforged.neoforge.client.event.sound.SoundEngineLoadEvent;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * AeroCore's hold on Minecraft's sound engine (0.37): its executor (the "Sound engine" thread, where every OpenAL
 * call belongs), a generation that changes whenever the engine is (re)loaded (F3+T, a resource pack, a new audio
 * device: every channel and OpenAL object is gone then), how many streaming channels are free, and subtitles for
 * events inside a synthesized mix.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class SoundHooks {
    private SoundHooks() {}

    private static volatile int generation;
    private static final Pattern DEBUG = Pattern.compile("(\\d+)/(\\d+) \\+ (\\d+)/(\\d+)");

    /** The sound engine, or null before the game has one. */
    public static SoundEngine engine() {
        Minecraft mc = Minecraft.getInstance();
        return mc == null || mc.getSoundManager() == null ? null : ((SoundManagerAccessor) mc.getSoundManager()).aerocore$soundEngine();
    }

    /** The sound thread's executor, or null when no device is open. Safe to call from any thread. */
    public static SoundEngineExecutor executor() {
        SoundEngine engine = engine();
        if (engine == null) { return null; }
        SoundEngineAccessor a = (SoundEngineAccessor) engine;
        return a.aerocore$loaded() ? a.aerocore$executor() : null;
    }

    /** Changes each time the sound engine is loaded again (everything played before is gone). */
    public static int generation() { return generation; }

    /** Free streaming channels (Minecraft has at most 8, shared with music and jukeboxes); -1 when unknown. */
    public static int freeStreams() {
        Minecraft mc = Minecraft.getInstance();
        if (mc == null || mc.getSoundManager() == null) { return -1; }
        Matcher m = DEBUG.matcher(mc.getSoundManager().getDebugString());
        if (!m.find()) { return -1; }
        return Integer.parseInt(m.group(4)) - Integer.parseInt(m.group(3));
    }

    /** Whether a category (and the master volume) is audible. */
    public static boolean audible(SoundSource source) {
        Minecraft mc = Minecraft.getInstance();
        return mc.options.getSoundSourceVolume(SoundSource.MASTER) > 0 && (source == SoundSource.MASTER || mc.options.getSoundSourceVolume(source) > 0);
    }

    /**
     * Shows a subtitle as if a sound had started at a point (heard within range blocks), when subtitles are on:
     * for events mixed into a stream, which Minecraft never sees start.
     */
    public static void subtitle(String key, double x, double y, double z, float range) {
        Minecraft mc = Minecraft.getInstance();
        if (mc.gui == null || !mc.options.showSubtitles().get() || mc.player == null) { return; }
        if (mc.player.distanceToSqr(x, y, z) > (double) range * range) { return; }
        ResourceLocation id = ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "subtitle");
        SimpleSoundInstance at = new SimpleSoundInstance(SoundEvent.createVariableRangeEvent(id), SoundSource.NEUTRAL, 1, 1, RandomSource.create(), x, y, z);
        ((GuiAccessor) mc.gui).aerocore$subtitleOverlay().onPlaySound(at, new WeighedSoundEvents(id, key), range);
    }

    @SubscribeEvent static void streaming(PlayStreamingSourceEvent event) {
        if (event.getSound() instanceof SynthInstance synth) {
            SynthThread.playing(event.getChannel());
            synth.started(event.getChannel());
        }
    }

    @SubscribeEvent static void playing(PlaySoundSourceEvent event) {
        if (event.getSound() instanceof SynthInstance synth) { synth.started(event.getChannel()); }
    }

    /** The engine's (re)load, on the mod bus. */
    @EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
    public static final class Load {
        private Load() {}
        @SubscribeEvent static void loaded(SoundEngineLoadEvent event) { generation++; }
    }
}
