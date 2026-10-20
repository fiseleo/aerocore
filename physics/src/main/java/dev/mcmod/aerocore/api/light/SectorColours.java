package dev.mcmod.aerocore.api.light;

import java.util.Arrays;

/**
 * A light whose colour depends on where it is seen from (0.38): by the elevation above the horizontal through the
 * light (a precision approach path indicator, PAPI: red below its setting angle, white above), or by the bearing off
 * its axis (a sector light: green one way, red the other). Bounds ascend; colour i shows between bound i-1 and bound
 * i (the first below the first bound, the last above the last). Within half the feather of a bound the two colours
 * mix: a PAPI's transition from red to white is no wider than 3 minutes of arc (ICAO Annex 14, 5.3.5).
 */
public record SectorColours(Plane plane, double[] boundsRad, int[] argb, double featherRad) {
    public enum Plane { ELEVATION, AZIMUTH }

    public SectorColours {
        if (argb.length != boundsRad.length + 1) { throw new IllegalArgumentException("One colour more than bounds"); }
        for (int i = 1; i < boundsRad.length; i++) {
            if (!(boundsRad[i] > boundsRad[i - 1])) { throw new IllegalArgumentException("Bounds must ascend"); }
        }
        if (!(featherRad >= 0)) { throw new IllegalArgumentException("A feather is not negative"); }
        boundsRad = boundsRad.clone();
        argb = argb.clone();
    }

    /** A PAPI unit: red below its setting (radians above the horizontal), white above, mixing over the transition. */
    public static SectorColours papi(double settingRad, double transitionRad, int red, int white) {
        return new SectorColours(Plane.ELEVATION, new double[]{settingRad}, new int[]{red, white}, transitionRad);
    }

    /** The colour seen from that bearing off the light's axis (radians, -pi to pi, positive to the left) and that elevation. */
    public int colour(double azimuthRad, double elevationRad) {
        double a = plane == Plane.ELEVATION ? elevationRad : azimuthRad;
        int i = 0;
        while (i < boundsRad.length && a >= boundsRad[i]) { i++; }
        // the nearest bound and how far past it (signed): mix within half the feather
        double half = featherRad / 2;
        if (half > 0) {
            if (i > 0 && a - boundsRad[i - 1] < half) { return mix(argb[i - 1], argb[i], 0.5 + 0.5 * (a - boundsRad[i - 1]) / half); }
            if (i < boundsRad.length && boundsRad[i] - a < half) { return mix(argb[i], argb[i + 1], 0.5 - 0.5 * (boundsRad[i] - a) / half); }
        }
        return argb[i];
    }

    /** a (share 0) to b (share 1), channel by channel. */
    static int mix(int a, int b, double share) {
        double s = Math.clamp(share, 0, 1);
        int out = 0;
        for (int shift = 0; shift <= 24; shift += 8) {
            int ca = (a >>> shift) & 255, cb = (b >>> shift) & 255;
            out |= ((int) Math.round(ca + (cb - ca) * s) & 255) << shift;
        }
        return out;
    }

    @Override public boolean equals(Object o) {
        return o instanceof SectorColours s && s.plane == plane && Arrays.equals(s.boundsRad, boundsRad) && Arrays.equals(s.argb, argb) && s.featherRad == featherRad;
    }

    @Override public int hashCode() { return java.util.Objects.hash(plane, Arrays.hashCode(boundsRad), Arrays.hashCode(argb), featherRad); }
}
