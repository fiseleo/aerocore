package dev.mcmod.aerocore.neoforge.client.sound;

import dev.mcmod.aerocore.api.acoustics.Voice;
import net.minecraft.client.Minecraft;
import net.minecraft.client.sounds.SoundManager;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.sounds.SoundSource;

/**
 * A sound that keeps going until its owner stops it (0.37): a live voice on a streaming channel, or a looped runtime
 * buffer. The owner calls {@link #update} on the game thread every tick with where it is and how loud; LiveSound
 * starts it, and starts it again when Minecraft has dropped it (a reload, a stream that ran dry, a channel that never
 * came, a category turned down to zero and back), while the category is muted it stays off. Streams start only while
 * at least {@link #reservedStreams} streaming channels stay free for music and jukeboxes.
 */
public final class LiveSound {
    /** Streaming channels a new live stream leaves free (Minecraft has at most 8 in all). */
    public static volatile int reservedStreams = 2;
    private static final long GRACE_NANOS = 600_000_000L;

    private final ResourceLocation id;
    private final SoundSource source;
    private final Voice voice;
    private SynthInstance current;
    private int generation = Integer.MIN_VALUE;
    private long playedNanos;
    private boolean stopped;
    private int restarts;

    private LiveSound(ResourceLocation id, SoundSource source, Voice voice) {
        this.id = id;
        this.source = source;
        this.voice = voice;
    }

    /** A voice made live on a streaming channel. */
    public static LiveSound streamed(ResourceLocation id, SoundSource source, Voice voice) { return new LiveSound(id, source, voice); }

    /** A runtime buffer looped on a static channel (its pitch and volume follow updates). */
    public static LiveSound looped(ResourceLocation runtimeId, SoundSource source) {
        if (!RuntimeSounds.isRuntime(runtimeId)) { throw new IllegalArgumentException("not a runtime sound id: " + runtimeId); }
        return new LiveSound(runtimeId, source, null);
    }

    /**
     * Game thread, each tick: the sound's position (relative to the listener's head when relative), its volume
     * (0-1, before the category) and pitch (Minecraft keeps it within 0.5-2).
     */
    public void update(double x, double y, double z, float volume, float pitch, boolean relative) {
        if (stopped) { return; }
        Minecraft mc = Minecraft.getInstance();
        SoundManager manager = mc.getSoundManager();
        if (!SoundHooks.audible(source)) {
            if (current != null) { current.finish(); current = null; }
            return;
        }
        if (current == null || current.finished() || generation != SoundHooks.generation() || dropped(mc, manager)) {
            if (!start(manager, current != null)) { return; }
        }
        current.at(x, y, z);
        current.volume(volume);
        current.pitch(pitch);
        current.relative(relative);
    }

    private boolean dropped(Minecraft mc, SoundManager manager) {
        long age = System.nanoTime() - playedNanos;
        if (age < GRACE_NANOS || mc.isPaused()) { return false; }
        if (voice == null) { return !manager.isActive(current); }
        SynthStream s = current.stream();
        return s == null || s.closed() || System.nanoTime() - s.lastReadNanos > GRACE_NANOS;
    }

    private boolean start(SoundManager manager, boolean again) {
        if (current != null) { current.finish(); current = null; }
        if (voice != null && !again) {
            int free = SoundHooks.freeStreams();
            if (free >= 0 && free <= reservedStreams) { return false; }
        }
        current = voice != null ? SynthInstance.streamed(id, source, voice) : SynthInstance.buffered(id, source, true);
        manager.play(current);
        generation = SoundHooks.generation();
        playedNanos = System.nanoTime();
        if (again) { restarts++; }
        return true;
    }

    /** Ends it for good. */
    public void stop() {
        stopped = true;
        if (current != null) { current.finish(); current = null; }
    }

    public boolean stopped() { return stopped; }
    /** Whether an instance is out now (it may still be waiting for its channel). */
    public boolean playing() { return current != null && !current.finished(); }
    /** How many times it had to be started again. */
    public int restarts() { return restarts; }
    public SynthInstance instance() { return current; }
    public ResourceLocation id() { return id; }
}
