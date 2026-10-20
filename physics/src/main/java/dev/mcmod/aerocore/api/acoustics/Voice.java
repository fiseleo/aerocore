package dev.mcmod.aerocore.api.acoustics;

/**
 * A sound made sample by sample (0.37): mono, at {@link #RATE}, full scale 1. A voice keeps its own state (phases,
 * filters, noise generators) and reads the parameters its owner last set. {@link #render} is called from one thread at
 * a time (a synthesis thread, or a test); the owner may set parameters from another thread, so a voice keeps them in
 * volatile fields or immutable snapshots and never reads game objects while rendering.
 */
public interface Voice {
    /** Samples per second of every voice. */
    int RATE = 44_100;

    /** Writes n samples (about -1 to 1) into out from offset on. */
    void render(float[] out, int offset, int n);

    /** A voice that renders silence. */
    Voice SILENT = (out, offset, n) -> java.util.Arrays.fill(out, offset, offset + n, 0f);

    /** Renders a voice for a time into a new array (offline: tests, exports). */
    static float[] render(Voice voice, double seconds) {
        float[] out = new float[(int) Math.round(seconds * RATE)];
        for (int i = 0; i < out.length; i += 1024) { voice.render(out, i, Math.min(1024, out.length - i)); }
        return out;
    }
}
