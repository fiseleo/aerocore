package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.acoustics.Voice;
import dev.mcmod.aerocore.api.acoustics.Wav;
import net.minecraft.client.sounds.AudioStream;
import org.slf4j.Logger;

import javax.sound.sampled.AudioFormat;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * A voice played as a stream (0.37). Minecraft queues four buffers on a streaming channel and refills only those that
 * finished, once per client tick; when the queue runs dry the source stops and the sound is dropped for good. So the
 * samples are made ahead on AeroCore's synthesis thread ({@link SynthThread}) into a ring, the same thread asks the
 * sound thread to refill every ~10 ms (a stalled game thread no longer starves the stream), and {@link #read} only
 * copies: {@link #CHUNK} frames per buffer, about 0.1 s queued in all. Should the ring still be short when a buffer
 * is asked for (the synthesis thread not woken in time: Windows wakes a sleeping thread on its 15.6 ms timer unless
 * asked for better), read renders the rest itself, under the same lock the synthesis thread renders with, so the
 * voice is never rendered twice at once and OpenAL never gets a gap.
 * <p>
 * read never throws and never returns null (an exception there stops every channel's refill for that tick; a null
 * loses a queue slot for good); when the ring is short it fades the last sample out instead of clicking.
 */
public final class SynthStream implements AudioStream {
    private static final Logger LOGGER = LogUtils.getLogger();

    /** Frames per buffer handed to OpenAL (23 ms). */
    public static final int CHUNK = 1024;
    /** Frames rendered at a time on the synthesis thread. */
    static final int BLOCK = 256;
    /** Frames the synthesis thread keeps ready beyond what OpenAL has queued. */
    static final int AHEAD = 2048;

    private final Voice voice;
    private final String name;
    private final short[] ring = new short[1 << 14];
    private final int mask = ring.length - 1;
    private volatile long written, taken;
    private final ByteBuffer out = ByteBuffer.allocateDirect(CHUNK * 2).order(ByteOrder.LITTLE_ENDIAN);
    private final float[] block = new float[BLOCK];
    private final java.util.concurrent.locks.ReentrantLock rendering = new java.util.concurrent.locks.ReentrantLock();
    volatile int rescued;
    private volatile boolean closed;
    private volatile boolean failed;
    private volatile float boost = 1;
    private float boostNow = 1;
    private float last;

    volatile long lastReadNanos = System.nanoTime();
    volatile int underruns;
    volatile long sentFrames;
    /** The gain OpenAL applies (set by the owner each tick), for recording what is heard. */
    volatile float heardGain = 1;

    public SynthStream(Voice voice, String name) {
        this.voice = voice;
        this.name = name;
    }

    public String name() { return name; }

    /** A gain applied in the stream before it is clipped (above 1: louder than OpenAL's own gain allows). */
    public void boost(float gain) { this.boost = Float.isFinite(gain) ? Math.max(0, gain) : 1; }

    public boolean closed() { return closed; }
    public int underruns() { return underruns; }
    /** How many times read had to render a chunk itself. */
    public int rescued() { return rescued; }
    /** Frames ready in the ring (not yet handed to OpenAL). */
    public int ready() { return (int) (written - taken); }

    @Override public AudioFormat getFormat() { return RuntimeSounds.FORMAT; }

    /** Synthesis thread: renders until the ring holds {@link #AHEAD} frames. */
    void fill() {
        if (closed) { return; }
        while (written - taken + BLOCK <= Math.min(AHEAD, ring.length) && !closed) {
            rendering.lock();
            try { renderBlock(); } finally { rendering.unlock(); }
        }
    }

    /** Renders one block into the ring (with the rendering lock held). */
    private void renderBlock() {
        if (failed) {
            java.util.Arrays.fill(block, 0f);
        } else {
            try {
                voice.render(block, 0, BLOCK);
            } catch (Throwable t) {
                failed = true;
                LOGGER.error("AeroCore sound '{}' failed to render; it goes silent", name, t);
                java.util.Arrays.fill(block, 0f);
            }
        }
        float target = boost, step = (target - boostNow) / BLOCK;
        long w = written;
        for (int i = 0; i < BLOCK; i++) {
            boostNow += step;
            ring[(int) (w + i) & mask] = Wav.toShort(block[i] * boostNow);
        }
        boostNow = target;
        written = w + BLOCK;
    }

    /** Sound thread: makes sure a chunk is ready, rendering it here if the synthesis thread is behind. */
    private void rescue() {
        if (written - taken >= CHUNK || closed) { return; }
        try {
            if (!rendering.tryLock(5, java.util.concurrent.TimeUnit.MILLISECONDS)) { return; }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
        }
        try {
            boolean any = false;
            while (written - taken < CHUNK && written - taken + BLOCK <= ring.length) { renderBlock(); any = true; }
            if (any) { rescued++; }
        } finally {
            rendering.unlock();
        }
        SynthThread.wake();
    }

    @Override public ByteBuffer read(int size) {
        try {
            lastReadNanos = System.nanoTime();
            rescue();
            long t = taken, available = written - t;
            int n = CHUNK, have = (int) Math.min(available, n);
            out.clear();
            for (int i = 0; i < have; i++) {
                short s = ring[(int) (t + i) & mask];
                out.putShort(s);
                last = s / 32767f;
            }
            if (have < n) {
                underruns++;
                for (int i = have; i < n; i++) {
                    last *= 0.995f;
                    out.putShort(Wav.toShort(last));
                }
            }
            taken = t + have;
            sentFrames += n;
            out.flip();
            if (SoundRecorder.recording()) { SoundRecorder.tap(this, out.duplicate().order(ByteOrder.LITTLE_ENDIAN), heardGain); }
            return out;
        } catch (Throwable e) {
            if (!failed) { LOGGER.error("AeroCore sound '{}' failed to stream", name, e); }
            failed = true;
            out.clear();
            for (int i = 0; i < CHUNK; i++) { out.putShort((short) 0); }
            out.flip();
            return out;
        }
    }

    @Override public void close() { closed = true; }
}
