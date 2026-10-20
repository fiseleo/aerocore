package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.blaze3d.audio.SoundBuffer;
import dev.mcmod.aerocore.api.acoustics.Voice;
import dev.mcmod.aerocore.api.acoustics.Wav;
import net.minecraft.resources.ResourceLocation;

import javax.sound.sampled.AudioFormat;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Static sounds made in code (0.37): a mod synthesizes the samples (a door's slam, a machine's loop) and defines them
 * under {@code <namespace>:runtime/<name>}; a {@link SynthInstance} with that id plays them on an ordinary (static)
 * channel. The samples stay in Java; OpenAL gets its own copy the first time the sound plays (through a mixin on the
 * buffer library), and again after a resource reload, which discards every buffer.
 * <p>
 * An id is defined once: a buffer already handed to OpenAL is not replaced while it might be playing, so a sound that
 * changes gets a new id (a version or a parameter bin in the name). {@link #undefine} drops the Java copy only.
 */
public final class RuntimeSounds {
    private RuntimeSounds() {}

    /** Mono 16-bit at the voices' rate: the format of every synthesized sound. */
    public static final AudioFormat FORMAT = new AudioFormat(Voice.RATE, 16, 1, true, false);
    private static final String PREFIX = "runtime/", FILE_PREFIX = "sounds/runtime/", FILE_SUFFIX = ".ogg";

    private static final Map<ResourceLocation, ByteBuffer> PCM = new ConcurrentHashMap<>();
    private static final AtomicLong BYTES = new AtomicLong();

    /** The id of a runtime sound: {@code namespace:runtime/name}. */
    public static ResourceLocation id(String namespace, String name) {
        return ResourceLocation.fromNamespaceAndPath(namespace, PREFIX + name);
    }

    /** Whether an id names a runtime sound (its path starts with runtime/). */
    public static boolean isRuntime(ResourceLocation id) { return id.getPath().startsWith(PREFIX); }

    /**
     * Defines a runtime sound from its samples (full scale 1, at {@link Voice#RATE}); false (and nothing changes) when
     * the id is taken or not a runtime id.
     */
    public static boolean define(ResourceLocation id, float[] samples) {
        if (!isRuntime(id) || samples.length == 0) { return false; }
        if (PCM.containsKey(id)) { return false; }
        ByteBuffer pcm = ByteBuffer.allocateDirect(samples.length * 2).order(ByteOrder.LITTLE_ENDIAN);
        for (float s : samples) { pcm.putShort(Wav.toShort(s)); }
        pcm.flip();
        if (PCM.putIfAbsent(id, pcm) != null) { return false; }
        BYTES.addAndGet(pcm.capacity());
        return true;
    }

    public static boolean defined(ResourceLocation id) { return PCM.containsKey(id); }

    /** Drops a sound's samples (a buffer OpenAL already has lasts until the next resource reload). */
    public static void undefine(ResourceLocation id) {
        ByteBuffer was = PCM.remove(id);
        if (was != null) { BYTES.addAndGet(-was.capacity()); }
    }

    /** How many runtime sounds are defined, and their bytes kept in Java. */
    public static int count() { return PCM.size(); }
    public static long bytes() { return BYTES.get(); }

    /** Mixin: whether a sound file path is a runtime sound's ({@code ns:sounds/runtime/...ogg}). */
    public static boolean isRuntimeFile(ResourceLocation file) {
        String p = file.getPath();
        return p.startsWith(FILE_PREFIX) && p.endsWith(FILE_SUFFIX);
    }

    /** Mixin: a new buffer for a runtime sound's file (null when nothing is defined there). */
    public static SoundBuffer buffer(ResourceLocation file) {
        String p = file.getPath();
        ResourceLocation id = ResourceLocation.fromNamespaceAndPath(file.getNamespace(), p.substring("sounds/".length(), p.length() - FILE_SUFFIX.length()));
        ByteBuffer pcm = PCM.get(id);
        return pcm == null ? null : new SoundBuffer(pcm.duplicate().order(ByteOrder.LITTLE_ENDIAN), FORMAT);
    }
}
