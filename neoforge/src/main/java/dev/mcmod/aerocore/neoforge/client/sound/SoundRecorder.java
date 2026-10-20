package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.acoustics.Voice;
import dev.mcmod.aerocore.api.acoustics.Wav;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import org.slf4j.Logger;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Development recording of synthesized sound (0.37; the game's audio cannot be screenshot): for a few seconds every
 * stream's samples are kept exactly as handed to OpenAL (and the gain OpenAL applies to them), together with one-shot
 * sounds reported by their owners; then each is written as a WAV file, with a mix of all of them at their gains, to
 * {@code %USERPROFILE%\.mcmod\exports\sound\<label>-<time>\}.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class SoundRecorder {
    private SoundRecorder() {}

    private static final Logger LOGGER = LogUtils.getLogger();
    private static volatile boolean on;
    private static volatile long startNanos, endNanos;
    private static String label = "sound";
    private static Consumer<Path> done;
    private static final Map<Object, Track> TRACKS = new LinkedHashMap<>();

    private static final class Track {
        final String name;
        final long startFrame;
        float[] samples = new float[Voice.RATE];
        float[] gains = new float[Voice.RATE];
        int length;
        int underrunsAtStart = -1, underruns;
        Track(String name, long startFrame) { this.name = name; this.startFrame = startFrame; }
        void add(float s, float g) {
            if (length == samples.length) {
                samples = java.util.Arrays.copyOf(samples, length * 2);
                gains = java.util.Arrays.copyOf(gains, length * 2);
            }
            samples[length] = s;
            gains[length++] = g;
        }
    }

    public static boolean recording() { return on; }

    /** Starts recording for a time; when written, the folder is handed to the callback (game thread). Null when busy. */
    public static synchronized boolean start(double seconds, String name, Consumer<Path> whenWritten) {
        if (on) { return false; }
        TRACKS.clear();
        label = name.replaceAll("[^A-Za-z0-9_.-]", "_");
        done = whenWritten;
        startNanos = System.nanoTime();
        endNanos = startNanos + (long) (seconds * 1e9);
        on = true;
        return true;
    }

    private static long frameNow() { return (long) ((System.nanoTime() - startNanos) * 1e-9 * Voice.RATE); }

    /** Sound thread: a stream's chunk as sent. */
    static synchronized void tap(SynthStream stream, ByteBuffer pcm, float gain) {
        if (!on) { return; }
        Track t = TRACKS.computeIfAbsent(stream, s -> new Track(stream.name(), frameNow()));
        if (t.underrunsAtStart < 0) { t.underrunsAtStart = stream.underruns(); }
        t.underruns = stream.underruns() - t.underrunsAtStart;
        while (pcm.remaining() >= 2) { t.add(pcm.getShort() / 32767f, gain); }
    }

    /** A one-shot sound its owner played now (its samples, at the gain it plays with). */
    public static synchronized void oneShot(String name, float[] samples, float gain) {
        if (!on) { return; }
        Track t = new Track(name, frameNow());
        for (float s : samples) { t.add(s, gain); }
        TRACKS.put(new Object(), t);
    }

    @SubscribeEvent static void tick(ClientTickEvent.Post event) {
        if (!on || System.nanoTime() < endNanos) { return; }
        List<Track> tracks;
        String name;
        Consumer<Path> then;
        synchronized (SoundRecorder.class) {
            on = false;
            tracks = new ArrayList<>(TRACKS.values());
            TRACKS.clear();
            name = label;
            then = done;
        }
        Path dir = Path.of(System.getProperty("user.home"), ".mcmod", "exports", "sound",
                name + "-" + LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss")));
        Thread writer = new Thread(() -> write(dir, tracks, then), "AeroCore sound recorder");
        writer.setDaemon(true);
        writer.start();
    }

    private static void write(Path dir, List<Track> tracks, Consumer<Path> then) {
        try {
            Files.createDirectories(dir);
            long end = 0;
            for (Track t : tracks) { end = Math.max(end, t.startFrame + t.length); }
            float[] mix = new float[(int) Math.min(end, Integer.MAX_VALUE - 8)];
            StringBuilder index = new StringBuilder("[\n");
            Map<String, Integer> names = new LinkedHashMap<>();
            for (Track t : tracks) {
                String base = t.name.replaceAll("[^A-Za-z0-9_.-]", "_");
                int k = names.merge(base, 1, Integer::sum);
                String file = (k == 1 ? base : base + "_" + k) + ".wav";
                Wav.write(dir.resolve(file), java.util.Arrays.copyOf(t.samples, t.length), Voice.RATE);
                double sumGain = 0;
                for (int i = 0; i < t.length; i++) {
                    int at = (int) (t.startFrame + i);
                    if (at < mix.length) { mix[at] += t.samples[i] * t.gains[i]; }
                    sumGain += t.gains[i];
                }
                if (index.length() > 2) { index.append(",\n"); }
                index.append(String.format(java.util.Locale.ROOT, "  {\"file\": \"%s\", \"startS\": %.4f, \"seconds\": %.4f, \"meanGain\": %.4f, \"underruns\": %d}",
                        file, t.startFrame / (double) Voice.RATE, t.length / (double) Voice.RATE, t.length == 0 ? 0 : sumGain / t.length, Math.max(0, t.underruns)));
            }
            index.append("\n]\n");
            Wav.write(dir.resolve("mix.wav"), mix, Voice.RATE);
            Files.writeString(dir.resolve("index.json"), index);
            LOGGER.info("AeroCore sound recording: {} tracks written to {}", tracks.size(), dir);
            if (then != null) { net.minecraft.client.Minecraft.getInstance().execute(() -> then.accept(dir)); }
        } catch (IOException | RuntimeException e) {
            LOGGER.error("AeroCore sound recording could not be written to {}", dir, e);
        }
    }
}
