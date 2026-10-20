package dev.mcmod.aerocore.api.acoustics;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * 16-bit PCM WAV files (0.37), for looking at synthesized sound outside the game: writing what a voice renders,
 * reading it back in tests. Samples are floats with full scale 1, clipped when written.
 */
public final class Wav {
    private Wav() {}

    /** A sound read from a file: its samples (channels interleaved), rate and channel count. */
    public record Sound(float[] samples, int rate, int channels) {
        public double seconds() { return samples.length / (double) (rate * channels); }
    }

    /** 16-bit little-endian PCM of the samples (clipped to -1..1). */
    public static byte[] pcm16(float[] samples) {
        ByteBuffer b = ByteBuffer.allocate(samples.length * 2).order(ByteOrder.LITTLE_ENDIAN);
        for (float s : samples) { b.putShort(toShort(s)); }
        return b.array();
    }

    /** One sample as 16-bit (clipped). */
    public static short toShort(float s) {
        float c = Float.isFinite(s) ? Math.max(-1f, Math.min(1f, s)) : 0f;
        return (short) Math.round(c * 32767f);
    }

    /** Writes a mono file at the given rate. */
    public static void write(Path file, float[] samples, int rate) throws IOException { write(file, samples, rate, 1); }

    /** Writes a file (channels interleaved in the samples). */
    public static void write(Path file, float[] samples, int rate, int channels) throws IOException {
        if (file.getParent() != null) { Files.createDirectories(file.getParent()); }
        try (OutputStream out = Files.newOutputStream(file)) { write(out, samples, rate, channels); }
    }

    public static void write(OutputStream out, float[] samples, int rate, int channels) throws IOException {
        if (rate <= 0 || channels <= 0) { throw new IllegalArgumentException("rate and channels must be positive"); }
        byte[] data = pcm16(samples);
        ByteBuffer h = ByteBuffer.allocate(44).order(ByteOrder.LITTLE_ENDIAN);
        h.put(new byte[]{'R', 'I', 'F', 'F'}).putInt(36 + data.length).put(new byte[]{'W', 'A', 'V', 'E'});
        h.put(new byte[]{'f', 'm', 't', ' '}).putInt(16).putShort((short) 1).putShort((short) channels)
                .putInt(rate).putInt(rate * channels * 2).putShort((short) (channels * 2)).putShort((short) 16);
        h.put(new byte[]{'d', 'a', 't', 'a'}).putInt(data.length);
        out.write(h.array());
        out.write(data);
    }

    public static Sound read(Path file) throws IOException {
        try (InputStream in = Files.newInputStream(file)) { return read(in); }
    }

    /** Reads a 16-bit PCM file (other chunks are skipped). */
    public static Sound read(InputStream in) throws IOException {
        ByteArrayOutputStream all = new ByteArrayOutputStream();
        in.transferTo(all);
        ByteBuffer b = ByteBuffer.wrap(all.toByteArray()).order(ByteOrder.LITTLE_ENDIAN);
        if (b.remaining() < 12) { throw new IOException("not a WAV file"); }
        int riff = b.getInt();
        b.getInt();   // the file's length
        if (riff != 0x46464952 || b.getInt() != 0x45564157) { throw new IOException("not a WAV file"); }
        int rate = 0, channels = 0, bits = 0;
        while (b.remaining() >= 8) {
            int id = b.getInt(), size = b.getInt();
            if (size < 0 || size > b.remaining()) { size = b.remaining(); }
            int next = b.position() + size + (size & 1);
            if (id == 0x20746d66) {   // "fmt "
                int format = b.getShort() & 0xffff;
                channels = b.getShort() & 0xffff;
                rate = b.getInt();
                b.getInt(); b.getShort();
                bits = b.getShort() & 0xffff;
                if (format != 1 || bits != 16) { throw new IOException("only 16-bit PCM is read"); }
            } else if (id == 0x61746164) {   // "data"
                if (rate == 0) { throw new IOException("data before format"); }
                float[] s = new float[size / 2];
                for (int i = 0; i < s.length; i++) { s[i] = b.getShort() / 32767f; }
                return new Sound(s, rate, channels);
            }
            b.position(Math.min(next, b.limit()));
        }
        throw new IOException("no data chunk");
    }
}
