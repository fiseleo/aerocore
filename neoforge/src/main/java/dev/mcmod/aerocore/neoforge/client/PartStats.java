package dev.mcmod.aerocore.neoforge.client;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Counters for drawing parts (0.29), summed until read: quads drawn, render types asked for, time spent emitting
 * (render thread), meshes baked and the time baking took (background). AutoShot's fps step prints and resets them.
 */
public final class PartStats {
    private PartStats() {}

    private static long quads, types, emitNanos, flushes, calls;
    private static final AtomicInteger BAKES = new AtomicInteger(), QUEUED = new AtomicInteger();
    private static final AtomicLong BAKE_NANOS = new AtomicLong();

    static synchronized void drawn(int q, int t, long nanos) { quads += q; types += t; emitNanos += nanos; flushes++; }

    static void queued(int delta) { QUEUED.addAndGet(delta); }

    /** 0.30: draw calls made from buffers kept on the GPU. */
    static synchronized void calls(int n) { calls += n; }

    static void baked(long nanos) { BAKES.incrementAndGet(); BAKE_NANOS.addAndGet(nanos); }

    private static long lampsHeld, lampsDrawn, lampNanos;

    /** 0.38: field lights looked at and drawn in a frame, and the time it took (render thread). */
    static synchronized void fieldLamps(int held, int drawn, long nanos) { lampsHeld += held; lampsDrawn += drawn; lampNanos += nanos; }

    /** A line for the log: per frame over a number of frames, then cleared. */
    public static synchronized String report(int frames) {
        int f = Math.max(1, frames);
        String s = String.format(java.util.Locale.ROOT, "parts %d quads, %d render types and %d GPU draw calls per frame, %.2f ms emitting per frame; %d meshes baked (%.0f ms each), %d waiting",
                quads / f, types / f, calls / f, emitNanos / 1e6 / f, BAKES.get(), BAKES.get() == 0 ? 0 : BAKE_NANOS.get() / 1e6 / BAKES.get(), QUEUED.get());
        if (lampsHeld > 0) {
            s += String.format(java.util.Locale.ROOT, "; field lights %d held, %d drawn, %.3f ms per frame", lampsHeld / f, lampsDrawn / f, lampNanos / 1e6 / f);
        }
        lampsHeld = lampsDrawn = lampNanos = 0;
        quads = types = emitNanos = flushes = calls = 0;
        BAKES.set(0);
        BAKE_NANOS.set(0);
        return s;
    }
}
