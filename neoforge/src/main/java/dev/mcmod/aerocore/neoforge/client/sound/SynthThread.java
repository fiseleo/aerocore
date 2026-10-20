package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.blaze3d.audio.Channel;
import com.mojang.logging.LogUtils;
import net.minecraft.client.sounds.SoundEngineExecutor;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.LockSupport;

/**
 * AeroCore's synthesis thread (0.37): renders every live {@link SynthStream} ahead into its ring, and about every
 * {@link #PUMP_MS} ms asks the sound thread to refill the streams' OpenAL queues (Minecraft does that only once per
 * client tick, so a stalled game thread would let a stream run dry and be dropped). The thread starts with the first
 * stream and runs as a daemon.
 */
final class SynthThread {
    private SynthThread() {}

    private static final Logger LOGGER = LogUtils.getLogger();
    static final long PUMP_MS = 10;

    private static final List<SynthStream> STREAMS = new CopyOnWriteArrayList<>();
    /** The streaming channels playing our streams: touched only on the sound thread. */
    private static final List<Channel> CHANNELS = new ArrayList<>();
    private static final AtomicBoolean PUMP_QUEUED = new AtomicBoolean();
    /** When the pending refill was posted: a reload drops queued tasks, so one that never ran is posted again. */
    private static volatile long pumpPostedNanos;
    private static volatile Thread thread;
    private static volatile long busyNanos, loops;

    static void add(SynthStream stream) {
        STREAMS.add(stream);
        if (thread == null) { start(); }
        wake();
    }

    /** Wakes the thread early (a stream was read: refill it now). */
    static void wake() {
        Thread t = thread;
        if (t != null) { LockSupport.unpark(t); }
    }

    /** Sound thread: a channel now plays one of our streams. */
    static void playing(Channel channel) { if (!CHANNELS.contains(channel)) { CHANNELS.add(channel); } }

    static int streams() { return STREAMS.size(); }
    /** The share of its time the synthesis thread spends rendering (0-1, smoothed). */
    static double load() { return loops == 0 ? 0 : busyShare; }
    private static volatile double busyShare;

    private static synchronized void start() {
        if (thread != null) { return; }
        Thread t = new Thread(SynthThread::run, "AeroCore synth");
        t.setDaemon(true);
        t.setPriority(Thread.NORM_PRIORITY + 1);
        thread = t;
        t.start();
    }

    private static final Runnable PUMP = () -> {
        PUMP_QUEUED.set(false);
        for (Iterator<Channel> it = CHANNELS.iterator(); it.hasNext(); ) {
            Channel c = it.next();
            try {
                if (c.stopped()) { it.remove(); continue; }
                c.updateStream();
            } catch (Throwable t) {
                it.remove();
                LOGGER.warn("AeroCore sound: could not refill a stream", t);
            }
        }
    };

    private static void run() {
        long nextPump = System.nanoTime();
        while (true) {
            try {
                long start = System.nanoTime();
                for (SynthStream s : STREAMS) {
                    if (s.closed()) { STREAMS.remove(s); continue; }
                    s.fill();
                }
                long now = System.nanoTime();
                busyNanos = now - start;
                loops++;
                busyShare += (Math.min(1, busyNanos / 2e6) - busyShare) * 0.01;
                if (now >= nextPump) {
                    nextPump = now + PUMP_MS * 1_000_000;
                    SoundEngineExecutor executor = SoundHooks.executor();
                    if (PUMP_QUEUED.get() && now - pumpPostedNanos > 250_000_000L) { PUMP_QUEUED.set(false); }
                    if (executor != null && !STREAMS.isEmpty() && PUMP_QUEUED.compareAndSet(false, true)) {
                        pumpPostedNanos = now;
                        executor.execute(PUMP);
                    }
                }
                // Thread.sleep(1) asks Windows for its 1 ms timer (HotSpot does for sleeps not a multiple of 10 ms);
                // parkNanos alone waits for the 15.6 ms tick there. A read wakes it early.
                if (!Thread.interrupted()) { LockSupport.parkNanos(200_000); }
                Thread.sleep(1);
            } catch (InterruptedException e) {
                // woken: go round again
            } catch (Throwable t) {
                LOGGER.error("AeroCore synth thread", t);
                LockSupport.parkNanos(50_000_000);
            }
        }
    }
}
