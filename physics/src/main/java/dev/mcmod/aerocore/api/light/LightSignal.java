package dev.mcmod.aerocore.api.light;

import java.util.Arrays;

/**
 * How a fixed light shows over time (0.38): steady, flashing, one of a sequence of flashers (an approach light
 * system's "rabbit": the flashers fire one after another toward the runway, twice a second) or alternating colours
 * (an aerodrome beacon's white and green). A pure function of game time, so every client shows the same flash at
 * the same moment without anything being sent.
 * <p>
 * A flash is shorter than a frame (a real capacitor-discharge flash lasts well under a millisecond), so a renderer
 * asks for the {@link #peak} over the time since its last frame: each flash then shows in exactly one frame whatever
 * the frame rate. {@link #level} is the value at one instant (a flash's own on time).
 */
public record LightSignal(Kind kind, double periodS, double onS, double startS, int[] colours) {
    public enum Kind { STEADY, FLASH, ALTERNATE }

    private static final LightSignal STEADY = new LightSignal(Kind.STEADY, 1, 1, 0, new int[0]);

    public LightSignal {
        if (!(periodS > 0) || !(onS > 0) || onS > periodS || !Double.isFinite(startS)) {
            throw new IllegalArgumentException("A light signal needs 0 < on <= period and a finite start");
        }
        colours = colours.clone();
        if (kind == Kind.ALTERNATE && colours.length == 0) { throw new IllegalArgumentException("Alternating needs colours"); }
    }

    /** Always on. */
    public static LightSignal steady() { return STEADY; }

    /** On for onS at the start of every period, the first starting at startS (an obstruction light, a strobe). */
    public static LightSignal flash(double periodS, double onS, double startS) { return new LightSignal(Kind.FLASH, periodS, onS, startS, new int[0]); }

    /**
     * The index-th of count flashers fired in turn every period (index 0 first). Each fires once a period, an equal
     * share of the period after the one before; its flash lasts a fortieth of a second or its share, if shorter.
     */
    public static LightSignal sequence(double periodS, int index, int count) {
        if (count < 1 || index < 0 || index >= count) { throw new IllegalArgumentException("Flasher " + index + " of " + count); }
        double slot = periodS / count;
        return new LightSignal(Kind.FLASH, periodS, Math.min(slot, 0.025), index * slot, new int[0]);
    }

    /** Flashes of the colours in turn, one every period, each on for onS (a beacon: white, green, white...). */
    public static LightSignal alternate(double periodS, double onS, int... argb) { return new LightSignal(Kind.ALTERNATE, periodS, onS, 0, argb); }

    /** 1 when on at that moment, else 0. */
    public double level(double timeS) { return peak(timeS, timeS); }

    /** 1 when the light was on at any moment from fromS to toS (inclusive), else 0. */
    public double peak(double fromS, double toS) {
        if (kind == Kind.STEADY) { return 1; }
        if (toS < fromS) { double t = fromS; fromS = toS; toS = t; }
        if (toS - fromS >= periodS) { return 1; }
        // the on windows are [startS + k period, startS + k period + onS): one could end in the interval or begin in it
        double k = Math.floor((fromS - startS) / periodS);
        for (double j = k; j <= k + 2; j++) {
            double on = startS + j * periodS, off = on + onS;
            if (on <= toS && off > fromS) { return 1; }
        }
        return 0;
    }

    /** The colour shown at a moment: the turn's colour when alternating, else the light's own. */
    public int colour(double timeS, int own) {
        if (kind != Kind.ALTERNATE) { return own; }
        long turn = (long) Math.floor((timeS - startS) / periodS);
        return colours[(int) Math.floorMod(turn, colours.length)];
    }

    @Override public boolean equals(Object o) {
        return o instanceof LightSignal s && s.kind == kind && s.periodS == periodS && s.onS == onS && s.startS == startS && Arrays.equals(s.colours, colours);
    }

    @Override public int hashCode() { return java.util.Objects.hash(kind, periodS, onS, startS, Arrays.hashCode(colours)); }

    @Override public String toString() { return "LightSignal[" + kind + ", " + periodS + " s, on " + onS + " s, start " + startS + " s]"; }
}
