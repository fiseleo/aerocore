package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.blaze3d.audio.Channel;
import dev.mcmod.aerocore.api.acoustics.Voice;
import net.minecraft.client.resources.sounds.AbstractSoundInstance;
import net.minecraft.client.resources.sounds.Sound;
import net.minecraft.client.resources.sounds.SoundInstance;
import net.minecraft.client.resources.sounds.TickableSoundInstance;
import net.minecraft.client.sounds.AudioStream;
import net.minecraft.client.sounds.SoundBufferLibrary;
import net.minecraft.client.sounds.SoundManager;
import net.minecraft.client.sounds.WeighedSoundEvents;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.sounds.SoundSource;
import net.minecraft.util.valueproviders.ConstantFloat;

import java.util.concurrent.CompletableFuture;

/**
 * A sound played from code, not from sounds.json (0.37): either a {@link Voice} streamed live ({@link SynthStream})
 * or a {@link RuntimeSounds} buffer on a static channel. The owner moves it and sets its volume and pitch on the game
 * thread each tick (Minecraft applies them on its next tick) and calls {@link #finish} to end it. It may start
 * silent (a fade-in from zero), and its attenuation is NONE by default: the owner computes the gain itself.
 * <p>
 * A streamed instance loops "manually" (looping with a one-tick delay): should its source run dry anyway, Minecraft
 * starts it again a tick later instead of dropping it; {@link #finish} turns that off.
 */
public class SynthInstance extends AbstractSoundInstance implements TickableSoundInstance {
    private final Voice voice;
    private final int distance;
    private boolean finished;
    private volatile SynthStream stream;
    private volatile Channel channel;
    private volatile long startedNanos;

    /** A live voice on a streaming channel. */
    public static SynthInstance streamed(ResourceLocation id, SoundSource source, Voice voice) {
        return new SynthInstance(id, source, voice, true);
    }

    /** A runtime buffer on a static channel, played once or looped. */
    public static SynthInstance buffered(ResourceLocation runtimeId, SoundSource source, boolean loop) {
        if (!RuntimeSounds.isRuntime(runtimeId)) { throw new IllegalArgumentException("not a runtime sound id: " + runtimeId); }
        return new SynthInstance(runtimeId, source, null, loop);
    }

    protected SynthInstance(ResourceLocation id, SoundSource source, Voice voice, boolean loop) {
        super(id, source, SoundInstance.createUnseededRandom());
        this.voice = voice;
        this.distance = 16;
        this.looping = loop;
        this.delay = voice != null && loop ? 1 : 0;
        this.attenuation = Attenuation.NONE;
        this.volume = 1;
        this.pitch = 1;
    }

    @Override public WeighedSoundEvents resolve(SoundManager manager) {
        this.sound = new Sound(location, ConstantFloat.of(1), ConstantFloat.of(1), 1, Sound.Type.FILE, voice != null, false, distance);
        return new WeighedSoundEvents(location, null);
    }

    @Override public CompletableFuture<AudioStream> getStream(SoundBufferLibrary buffers, Sound sound, boolean looping) {
        if (voice == null) { return buffers.getStream(sound.getPath(), looping); }
        SynthStream s = new SynthStream(voice, location.toString());
        SynthStream was = stream;
        if (was != null) { was.close(); }
        stream = s;
        SynthThread.add(s);
        return CompletableFuture.completedFuture(s);
    }

    @Override public boolean canStartSilent() { return true; }
    @Override public boolean isStopped() { return finished; }
    @Override public void tick() {}

    /** Ends the sound (Minecraft stops it on its next tick). */
    public void finish() {
        finished = true;
        looping = false;
        SynthStream s = stream;
        if (s != null) { s.boost(0); }
    }

    public boolean finished() { return finished; }
    public boolean streamedLive() { return voice != null; }
    public Voice voice() { return voice; }
    /** The stream now playing (null before it starts, and for a buffered sound). */
    public SynthStream stream() { return stream; }

    public void at(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    public void volume(float volume) { this.volume = Float.isFinite(volume) ? Math.max(0, volume) : 0; SynthStream s = stream; if (s != null) { s.heardGain = this.volume; } }
    public void pitch(float pitch) { this.pitch = Float.isFinite(pitch) ? pitch : 1; }
    /** Whether the position is relative to the listener (head-locked: a sound inside the listener's own vehicle). */
    public void relative(boolean relative) { this.relative = relative; }
    public void linear(boolean linear) { this.attenuation = linear ? Attenuation.LINEAR : Attenuation.NONE; }

    /** Sound thread: the channel that plays this now. */
    void started(Channel channel) {
        this.channel = channel;
        this.startedNanos = System.nanoTime();
    }

    /** The channel playing this (sound thread use only), or null. */
    public Channel channel() { return channel; }
    /** When it last started on a channel (System.nanoTime), 0 before. */
    public long startedNanos() { return startedNanos; }
}
