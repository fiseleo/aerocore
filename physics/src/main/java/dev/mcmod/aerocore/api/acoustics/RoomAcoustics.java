package dev.mcmod.aerocore.api.acoustics;

import java.util.List;

/**
 * The room around a listener from rays cast out of it (0.37). Each ray reports how far it went and the surface it hit
 * (OPEN when it left within the range: sky, or a space too large to matter) and the cosine between the ray and the
 * surface's normal. Treating the rays as equal solid angles, the room's volume is 4 pi / 3 mean(r^3) and its boundary
 * 4 pi mean(r^2 / cos) (both exact for any room the listener sees all of), its absorption the area-weighted mean of the
 * hits with open rays absorbing all. From them Eyring's reverberation time
 * T60 = 0.161 V / (-S ln(1 - a) + 4 m V) per octave band, m the air's intensity attenuation (ISO 9613-1), the mean free
 * path 4 V / S and the critical distance 0.057 sqrt(Q V / T60) (Kuttruff, Room Acoustics, eq. 5.42).
 * <p>
 * {@link #reverb} turns a room into the parameters an artificial reverberator wants (EFX names). That mapping is a
 * listening rule, not acoustics: the decay and its high-frequency ratio are the room's, the gains rise with how
 * closed the room is, the reflection delays follow the mean free path.
 */
public final class RoomAcoustics {
    private RoomAcoustics() {}

    /** One ray: how far it went (m), what it hit (OPEN: nothing within range) and the cosine of its incidence on it. */
    public record Ray(double distanceM, Surface surface, double cosine) {
        public Ray(double distanceM, Surface surface) { this(distanceM, surface, 1); }
    }

    /** The room as estimated: per octave band (Surface.BANDS) absorption and T60. */
    public record Room(double volumeM3, double surfaceM2, double[] absorption, double openShare, double[] rt60S, double meanFreePathM) {
        /** How closed it is: 0 outdoors, 1 sealed. */
        public double enclosure() { return 1 - openShare; }
        /** Mean T60 over 500 Hz-1 kHz. */
        public double rt60() { return 0.5 * (rt60S[2] + rt60S[3]); }
        /** T60 at 4 kHz over the mean. */
        public double hfRatio() { return rt60S[5] / Math.max(1e-3, rt60()); }
        /** Distance (m) where a source of directivity q sounds as loud direct as reverberant. */
        public double criticalDistanceM(double q) { return 0.057 * Math.sqrt(q * volumeM3 / Math.max(0.05, rt60())); }
    }

    /** What the open air is: no room. */
    public static final Room OUTDOORS = new Room(0, 0, new double[]{1, 1, 1, 1, 1, 1}, 1, new double[]{0, 0, 0, 0, 0, 0}, Double.POSITIVE_INFINITY);

    public static Room estimate(List<Ray> rays, double rangeM, double temperatureK, double humidityPercent, double pressurePa) {
        if (rays.isEmpty()) { return OUTDOORS; }
        double r2 = 0, r3 = 0, open = 0;
        double[] a = new double[Surface.BANDS.length];
        for (Ray ray : rays) {
            boolean escaped = ray.surface() == Surface.OPEN || !(ray.distanceM() < rangeM);
            double r = escaped ? rangeM : Math.max(0.25, ray.distanceM());
            double w = r * r / (escaped ? 1 : Math.max(0.05, Math.min(1, Math.abs(ray.cosine()))));
            r2 += w;
            r3 += r * r * r;
            if (escaped) { open += w; }
            for (int b = 0; b < a.length; b++) { a[b] += w * (escaped ? 1 : ray.surface().absorption(b)); }
        }
        int n = rays.size();
        for (int b = 0; b < a.length; b++) { a[b] /= r2; }
        double volume = 4 * Math.PI / 3 * r3 / n, surface = 4 * Math.PI * r2 / n, openShare = open / r2;
        double[] rt = new double[a.length];
        for (int b = 0; b < a.length; b++) {
            double m = Propagation.absorptionDbPerM(Surface.BANDS[b], temperatureK, humidityPercent, pressurePa) / 4.343;
            double alpha = Math.min(0.995, a[b]);
            rt[b] = 0.161 * volume / (-surface * Math.log(1 - alpha) + 4 * m * volume);
        }
        return new Room(volume, surface, a, openShare, rt, 4 * volume / surface);
    }

    /** An artificial reverberator's settings (EFX standard reverb names and ranges). */
    public record Reverb(double decayS, double hfRatio, double density, double diffusion, double gain, double gainHf,
                         double reflectionsGain, double reflectionsDelayS, double lateGain, double lateDelayS) {}

    /** No reverberation (the reverberator's output gain 0). */
    public static final Reverb DRY = new Reverb(0.1, 1, 1, 1, 0, 1, 0, 0, 0, 0);

    public static Reverb reverb(Room room, double soundMps) {
        double closed = room.enclosure();
        if (closed < 0.05 || room.volumeM3() <= 0) { return DRY; }
        double rt = Math.max(0.1, Math.min(20, room.rt60()));
        double path = room.meanFreePathM() / soundMps;
        double hf = 0.5 * (room.absorption()[4] + room.absorption()[5]);
        return new Reverb(rt, Math.max(0.1, Math.min(2, room.hfRatio())), 1, 0.5 + 0.5 * closed,
                0.32 * closed, Math.max(0.1, Math.min(1, 1 - hf)),
                Math.min(3.16, 0.05 + closed * (1 - room.absorption()[3])), Math.min(0.3, path),
                Math.min(10, 1.26 * closed), Math.min(0.1, 1.5 * path));
    }

    /**
     * How much more of a source a listener hears through the room than directly, as an amplitude ratio (the
     * reverberant field over the direct one at distance r, sqrt((r / r_c)^2)), clamped to [0.1, 4].
     */
    public static double wetOverDry(Room room, double distanceM, double q) {
        if (room.enclosure() < 0.05) { return 0; }
        double rc = room.criticalDistanceM(q);
        return Math.max(0.1, Math.min(4, room.enclosure() * distanceM / Math.max(0.3, rc)));
    }
}
