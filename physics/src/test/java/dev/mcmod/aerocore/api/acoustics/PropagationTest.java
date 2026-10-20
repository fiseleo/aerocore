package dev.mcmod.aerocore.api.acoustics;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Sound in the open, in rooms and through walls (0.37) against standards, textbook cases and measured booms. */
class PropagationTest {
    private static final double C20 = Propagation.speedOfSound(293.15);

    @Test void airAbsorbsAsIso9613Tabulates() {
        // ISO 9613-2 table 2 (from ISO 9613-1), dB/km at 20 C and 70 %, and at 10 C and 70 %
        double[] f = {63, 125, 250, 500, 1000, 2000, 4000, 8000};
        double[] at20 = {0.1, 0.3, 1.1, 2.8, 5.0, 9.0, 22.9, 76.6}, at10 = {0.1, 0.4, 1.0, 1.9, 3.7, 9.7, 32.8, 117};
        for (int i = 0; i < f.length; i++) {
            assertEquals(at20[i], 1000 * Propagation.absorptionDbPerM(f[i], 293.15, 70, 101_325), Math.max(0.05, 0.03 * at20[i]), f[i] + " Hz, 20 C");
            assertEquals(at10[i], 1000 * Propagation.absorptionDbPerM(f[i], 283.15, 70, 101_325), Math.max(0.05, 0.03 * at10[i]), f[i] + " Hz, 10 C");
        }
        assertEquals(343.2, C20, 0.2);
    }

    @Test void aPointSourceFallsSixDecibelsADoubling() {
        assertEquals(100 - 20 - 11, Propagation.levelAt(100, 0, 10), 0.05, "Lw 100 dB at 10 m");
        assertEquals(-20 * Math.log10(2), Propagation.levelAt(100, 0, 20) - Propagation.levelAt(100, 0, 10), 1e-9);
    }

    @Test void dopplerShiftsByTheSpeedsAlongTheLine() {
        Vec3d toListener = new Vec3d(100, 0, 0);
        assertEquals(2, Propagation.doppler(new Vec3d(C20 / 2, 0, 0), Vec3d.ZERO, toListener, C20), 1e-9, "approaching at Mach 0.5");
        assertEquals(2 / 3.0, Propagation.doppler(new Vec3d(-C20 / 2, 0, 0), Vec3d.ZERO, toListener, C20), 1e-9, "receding");
        assertEquals(1.5, Propagation.doppler(Vec3d.ZERO, new Vec3d(-C20 / 2, 0, 0), toListener, C20), 1e-9, "listener coming toward it");
        assertEquals(1, Propagation.doppler(new Vec3d(0, 0, 50), Vec3d.ZERO, toListener, C20), 1e-9, "passing across: no shift");
        assertTrue(Double.isInfinite(Propagation.doppler(new Vec3d(2 * C20, 0, 0), Vec3d.ZERO, toListener, C20)));
    }

    /** A track of n points at dt apart moving at v from start. */
    private static Vec3d[] track(Vec3d start, Vec3d v, double[] times) {
        Vec3d[] p = new Vec3d[times.length];
        for (int i = 0; i < times.length; i++) { p[i] = start.add(v.scale(times[i])); }
        return p;
    }

    private static double[] times(int n, double dt) {
        double[] t = new double[n];
        for (int i = 0; i < n; i++) { t[i] = i * dt; }
        return t;
    }

    @Test void soundHeardNowLeftWhenTheDistanceIsTheTravel() {
        double[] t = times(121, 0.05);   // six seconds recorded, "now" at the last
        double now = t[120];
        Vec3d listener = new Vec3d(0, 0, 0);
        // a still source 343 m away: heard one second late
        List<Propagation.Emission> still = Propagation.emissions(t, track(new Vec3d(C20, 0, 0), Vec3d.ZERO, t), 121, listener, now, C20);
        assertEquals(1, still.size());
        assertEquals(1, still.getFirst().delayS(now), 1e-6);
        // a subsonic plane passing at 100 m: one emission, and its distance is c times its delay
        Vec3d v = new Vec3d(150, 0, 0);
        List<Propagation.Emission> plane = Propagation.emissions(t, track(new Vec3d(-450, 100, 0), v, t), 121, listener, now, C20);
        assertEquals(1, plane.size());
        Propagation.Emission e = plane.getFirst();
        assertEquals(C20 * e.delayS(now), e.distanceM(), 1e-6);
        assertEquals(150, e.velocity().x(), 1e-6);
        assertTrue(e.position().x() < 450 && e.position().x() > 0, "heard from where it was, behind where it is (450 m on): " + e.position());
        // a supersonic one, Mach 2 at 300 m: none ahead of the cone, two inside it (coming and going)
        Vec3d fast = new Vec3d(2 * C20, 0, 0);
        double crossing = 300 * Math.sqrt(3);   // the cone (half angle 30 deg) reaches the listener when the plane is this far past
        double[] t2 = times(241, 0.05);
        double now2 = t2[240];
        Vec3d start = new Vec3d(-2 * C20 * now2 - 50, 300, 0);   // at "now" the plane is 50 m short of overhead: cone not yet here
        assertEquals(0, Propagation.emissions(t2, track(start, fast, t2), 241, listener, now2, C20).size(), "ahead of the cone: silence");
        Vec3d later = new Vec3d(-2 * C20 * now2 + crossing + 200, 300, 0);   // now well past the crossing
        List<Propagation.Emission> both = Propagation.emissions(t2, track(later, fast, t2), 241, listener, now2, C20);
        assertEquals(2, both.size(), "inside the cone: two arrivals");
        for (Propagation.Emission b : both) { assertEquals(C20 * b.delayS(now2), b.distanceM(), 1e-4); }
        assertTrue(both.get(0).position().x() > 0 && both.get(1).position().x() < 0, "one from after it passed overhead, one from before: " + both);
    }

    @Test void aTrackIsHeardWhereItWasAndBoomsOnceWhenItsConePasses() {
        SourceTrack slow = new SourceTrack(), fast = new SourceTrack();
        Vec3d listener = Vec3d.ZERO;
        int booms = 0;
        double firstHeard = -1;
        for (int tick = 0; tick <= 200; tick++) {
            double t = tick * 0.05;
            slow.record(t, new Vec3d(-300 + 100 * t, 50, 0));
            fast.record(t, new Vec3d(-2000 + 2 * C20 * t, 300, 0));
            SourceTrack.Hearing h = slow.hear(listener, Vec3d.ZERO, t, C20, true, true);
            if (tick == 60) {
                // at 3 s the plane is overhead (x 0); it is heard from where it was about a sixth of a second before
                assertTrue(h.apparent().x() < 0 && h.apparent().x() > -40, "heard behind: " + h.apparent());
                assertEquals(h.distanceM() / C20, h.delayS(), 1e-6);
                assertEquals(1, h.doppler(), 0.15, "nearly overhead, little shift");
            }
            if (tick == 20) { assertTrue(h.doppler() > 1.2, "coming: higher " + h.doppler()); }
            if (tick == 100) { assertTrue(h.doppler() < 0.85, "going: lower " + h.doppler()); }
            SourceTrack.Hearing f = fast.hear(listener, Vec3d.ZERO, t, C20, true, true);
            if (f.boom()) { booms++; }
            if (f.heard() && firstHeard < 0) { firstHeard = t; }
        }
        assertEquals(1, booms, "one boom");
        // the cone (half angle 30 deg) reaches the listener when the plane is 300 / tan 30 = 520 m past
        double expected = (2000 + 300 * Math.sqrt(3)) / (2 * C20);
        assertEquals(expected, firstHeard, 0.06, "silent until the cone arrives, then heard at once");
    }

    @Test void aSourceLaunchedPastTheListenerBoomsWhileItsStandingSoundIsStillArriving() {
        // standing 64 m off (40 m aside, 50 m back) for 2 s, then sent off at 600 m/s past the listener (hdfv's launch
        // test): what it sent standing arrives until 2 + 64/c = 2.187 s, its cone at 2.179 s, so the arrivals go from one
        // to three (within a tick, from one to two): the boom is the first sound sent while closing faster than sound
        SourceTrack track = new SourceTrack();
        Vec3d listener = Vec3d.ZERO;
        int booms = 0;
        double boomAt = -1;
        for (int tick = 0; tick <= 120; tick++) {
            double t = tick * 0.05;
            track.record(t, new Vec3d(-40, 0, t <= 2 ? -50 : -50 + 600 * (t - 2)));
            SourceTrack.Hearing h = track.hear(listener, Vec3d.ZERO, t, C20, true, true);
            if (tick == 43) { assertTrue(h.heard(), "still hearing it as it stood"); }
            if (h.boom()) { booms++; boomAt = t; }
        }
        assertEquals(1, booms, "one boom");
        // the cone: the sound sent where 600 m/s along the line of sight is c, 40 / sqrt(M^2 - 1) before abeam
        double z = -40 / Math.sqrt(Math.pow(600 / C20, 2) - 1);
        double expected = 2 + (z + 50) / 600 + Math.hypot(40, z) / C20;
        assertEquals(expected, boomAt, 0.06, "when the cone arrives");
        // put 200 m nearer in a tick (a jump, not flight), and passing at 100 m/s: no boom
        SourceTrack jumped = new SourceTrack(), slow = new SourceTrack();
        for (int tick = 0; tick <= 80; tick++) {
            double t = tick * 0.05;
            jumped.record(t, new Vec3d(-40, 0, tick < 20 ? -250 : -50));
            slow.record(t, new Vec3d(-40, 0, -200 + 100 * t));
            assertFalse(jumped.hear(listener, Vec3d.ZERO, t, C20, true, true).boom(), "a jump does not boom");
            assertFalse(slow.hear(listener, Vec3d.ZERO, t, C20, true, true).boom(), "a subsonic source does not boom");
        }
    }

    @Test void airTakesTheHighsFirst() {
        double near = Propagation.airCutoffHz(10, 293.15, 70, 101_325), mid = Propagation.airCutoffHz(100, 293.15, 70, 101_325);
        double far = Propagation.airCutoffHz(1000, 293.15, 70, 101_325);
        assertTrue(near > 12_000, "10 m: only the top octave, above " + near + " Hz");
        assertTrue(mid > 4000 && mid < 8000, "100 m: 3 dB gone above " + mid + " Hz (30 dB/km between 4 and 8 kHz)");
        assertTrue(far > 400 && far < 1000, "1 km: only a rumble below " + far + " Hz");
        assertEquals(3, Propagation.absorptionDbPerM(far, 293.15, 70, 101_325) * 1000, 0.01);
    }

    @Test void concordesBoomIsAboutTwoPoundsPerSquareFoot() {
        // Concorde at Mach 2, 16.8 km (55,000 ft), 62 m: measured 1.9-2.5 psf (91-120 Pa), about 0.3 s
        SonicBoom.NWave w = SonicBoom.nWave(62, 2, 16_800, 9_100, 101_325, 295, SonicBoom.SHAPE, SonicBoom.GROUND);
        assertEquals(96, w.overpressurePa(), 24);
        assertTrue(w.durationS() > 0.2 && w.durationS() < 0.5, "duration " + w.durationS());
        SonicBoom.NWave twice = SonicBoom.nWave(62, 2, 33_600, 9_100, 101_325, 295, SonicBoom.SHAPE, SonicBoom.GROUND);
        assertEquals(Math.pow(2, -0.75), twice.overpressurePa() / w.overpressurePa(), 1e-9, "h^-3/4");
        assertEquals(0, SonicBoom.nWave(62, 0.95, 10_000, 30_000, 101_325, 300, 0.1, 1.9).overpressurePa());
        assertEquals(Math.toRadians(30), SonicBoom.coneHalfAngle(2), 1e-12);
        float[] n = SonicBoom.samples(w, 0.002, 2 * w.overpressurePa());
        double max = 0, min = 0;
        for (float s : n) { max = Math.max(max, s); min = Math.min(min, s); }
        assertEquals(0.5, max, 0.01);
        assertEquals(-0.5, min, 0.01);
    }

    /** Rays from a point inside a box (lx, ly, lz) with the point at (px, py, pz), evenly over the sphere. */
    private static List<RoomAcoustics.Ray> boxRays(double lx, double ly, double lz, double px, double py, double pz, Surface s, int n) {
        java.util.ArrayList<RoomAcoustics.Ray> rays = new java.util.ArrayList<>();
        double golden = Math.PI * (3 - Math.sqrt(5));
        for (int i = 0; i < n; i++) {
            double y = 1 - 2 * (i + 0.5) / n, r = Math.sqrt(1 - y * y), th = golden * i;
            double dx = Math.cos(th) * r, dz = Math.sin(th) * r;
            double tx = dx > 0 ? (lx - px) / dx : dx < 0 ? -px / dx : Double.POSITIVE_INFINITY;
            double ty = y > 0 ? (ly - py) / y : y < 0 ? -py / y : Double.POSITIVE_INFINITY;
            double tz = dz > 0 ? (lz - pz) / dz : dz < 0 ? -pz / dz : Double.POSITIVE_INFINITY;
            double t = Math.min(tx, Math.min(ty, tz)), cos = t == tx ? Math.abs(dx) : t == ty ? Math.abs(y) : Math.abs(dz);
            rays.add(new RoomAcoustics.Ray(t, s, cos));
        }
        return rays;
    }

    @Test void aRoomsReverberationIsEyringsFromItsRays() {
        // a sphere about the listener is exact: V, S and Eyring's T60 for concrete (alpha 0.015 at 500 Hz)
        List<RoomAcoustics.Ray> sphere = java.util.Collections.nCopies(200, new RoomAcoustics.Ray(5, Surface.ROCK));
        RoomAcoustics.Room r = RoomAcoustics.estimate(sphere, 32, 293.15, 50, 101_325);
        assertEquals(4 * Math.PI / 3 * 125, r.volumeM3(), 1e-6);
        assertEquals(4 * Math.PI * 25, r.surfaceM2(), 1e-6);
        double m = Propagation.absorptionDbPerM(500, 293.15, 50, 101_325) / 4.343;
        assertEquals(0.161 * r.volumeM3() / (-r.surfaceM2() * Math.log(1 - 0.015) + 4 * m * r.volumeM3()), r.rt60S()[2], 1e-9);
        assertEquals(0, r.openShare(), 0);
        // a 12 x 4 x 8 m stone hall seen from inside: its volume and boundary (352 m^2), T60 of seconds; wool-lined, far shorter
        RoomAcoustics.Room hall = RoomAcoustics.estimate(boxRays(12, 4, 8, 6, 1.6, 4, Surface.ROCK, 4000), 32, 293.15, 50, 101_325);
        assertEquals(384, hall.volumeM3(), 384 * 0.05);
        assertEquals(352, hall.surfaceM2(), 352 * 0.08);
        assertTrue(hall.rt60() > 3 && hall.rt60() < 15, "stone hall T60 " + hall.rt60());
        RoomAcoustics.Room padded = RoomAcoustics.estimate(boxRays(12, 4, 8, 6, 1.6, 4, Surface.WOOL, 4000), 32, 293.15, 50, 101_325);
        assertTrue(padded.rt60() < 0.4, "wool-lined T60 " + padded.rt60());
        assertTrue(hall.hfRatio() < 1, "air takes the highs first: " + hall.hfRatio());
        RoomAcoustics.Reverb rev = RoomAcoustics.reverb(hall, C20);
        assertEquals(Math.min(20, hall.rt60()), rev.decayS(), 1e-9);
        assertTrue(rev.gain() > 0.3 && rev.reflectionsDelayS() > 0.005 && rev.reflectionsDelayS() < 0.05, rev.toString());
        // outdoors: all rays leave, nothing to reverberate
        List<RoomAcoustics.Ray> open = java.util.Collections.nCopies(64, new RoomAcoustics.Ray(Double.POSITIVE_INFINITY, Surface.OPEN));
        RoomAcoustics.Room outside = RoomAcoustics.estimate(open, 32, 293.15, 50, 101_325);
        assertEquals(1, outside.openShare(), 1e-12);
        assertSame(RoomAcoustics.DRY, RoomAcoustics.reverb(outside, C20));
        // farther from a source than the critical distance, the room is heard more than the source
        assertTrue(RoomAcoustics.wetOverDry(hall, 3 * hall.criticalDistanceM(1), 1) > 1);
        assertEquals(0, RoomAcoustics.wetOverDry(outside, 10, 1));
    }

    @Test void wallsFollowTheMassLawAndALeakLimitsThem() {
        assertEquals(27.0, Transmission.massLawDb(10, 500), 0.05, "10 kg/m^2 at 500 Hz");
        assertEquals(20 * Math.log10(2), Transmission.massLawDb(20, 500) - Transmission.massLawDb(10, 500), 1e-9, "doubling the mass");
        assertEquals(20 * Math.log10(2), Transmission.massLawDb(10, 1000) - Transmission.massLawDb(10, 500), 1e-9, "and the frequency");
        // a car cabin: 3 m^2 of 0.8 mm steel, 2 m^2 of 4 mm glass, a 5 cm^2 seal leak
        List<Transmission.Element> cabin = List.of(new Transmission.Element(3, 6.3), new Transmission.Element(2, 10), Transmission.Element.opening(5e-4));
        double tl500 = Transmission.compositeDb(cabin, 500);
        assertTrue(tl500 > 22 && tl500 < 32, "cabin TL at 500 Hz " + tl500);
        assertTrue(Transmission.compositeDb(cabin, 4000) < 41, "the leak caps the highs");
        Transmission.Insulation in = Transmission.insulation(cabin);
        assertTrue(in.gain() < 0.08 && in.gainHf() < 1, in.toString());
        assertEquals(20, Transmission.compositeDb(List.of(new Transmission.Element(99, 1000), Transmission.Element.opening(1)), 1000), 0.1, "a 1 % opening: 20 dB");
    }

    @Test void loudnessIsOneToOneBelowTheKneeAndNeverPassesFullScale() {
        assertEquals(-6, Loudness.playbackDb(75), 1e-12);
        assertEquals(-6 - 25 / 1.6, Loudness.playbackDb(50), 1e-12, "a car idling at 7.5 m");
        assertEquals(-3.75, Loudness.playbackDb(44) - Loudness.playbackDb(50), 0.02, "twice as far");
        double prev = -1e9;
        for (double l = 0; l <= 200; l += 0.5) {
            double p = Loudness.playbackDb(l);
            assertTrue(p > prev && p < 0, "monotone and below full scale at " + l);
            prev = p;
        }
        assertEquals(1 / 1.6, (Loudness.playbackDb(75.01) - Loudness.playbackDb(74.99)) / 0.02, 1e-3, "smooth at the knee");
        assertTrue(Loudness.gain(150) > 0.99);
    }
}
