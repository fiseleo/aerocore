package dev.mcmod.aerocore.api.acoustics;

/**
 * Real sound pressure levels to playback gain (0.37). A loudspeaker cannot give the 140 dB of a jet next to it, and a
 * game's world is louder than ours (a cow is heard across a field), so the levels are compressed: below
 * {@link #KNEE_DB} every {@link #RATIO} dB of real level is one dB of playback, with the knee at {@link #KNEE_PLAYBACK_DB};
 * above it a soft knee bends toward full scale, which it never passes (the slopes match at the knee). So a car idling
 * 7.5 m away (about 50 dB) plays at -22 dBFS, one pulling hard (75 dB) at -6, a jet close by at nearly 0; doubling the
 * distance takes 3.75 dB off below the knee. The constants are a listening choice, set against Minecraft's own sounds.
 */
public final class Loudness {
    private Loudness() {}

    public static final double KNEE_DB = 75, KNEE_PLAYBACK_DB = -6, RATIO = 1.6;

    /** Playback level, dB re full scale, of a sound of splDb at the listener. */
    public static double playbackDb(double splDb) {
        if (!Double.isFinite(splDb)) { return splDb < 0 ? -200 : 0; }
        if (splDb <= KNEE_DB) { return KNEE_PLAYBACK_DB + (splDb - KNEE_DB) / RATIO; }
        double room = -KNEE_PLAYBACK_DB;
        return KNEE_PLAYBACK_DB + room * (1 - Math.exp(-(splDb - KNEE_DB) / (room * RATIO)));
    }

    /** Playback gain (0-1) of a sound of splDb at the listener. */
    public static double gain(double splDb) { return Dsp.dbToGain(playbackDb(splDb)); }
}
