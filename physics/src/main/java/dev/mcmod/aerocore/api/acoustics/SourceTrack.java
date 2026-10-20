package dev.mcmod.aerocore.api.acoustics;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;

/**
 * A moving source's recent positions and what a listener hears of it now (0.37): the sound left it at the retarded
 * time ({@link Propagation#emissions}), so it is heard from where the source was then (the apparent position), late
 * by the travel time, shifted by the Doppler factor of the source's velocity then and the listener's now. A
 * supersonic source ahead of its cone is not heard at all; when the cone sweeps over the listener two arrivals are
 * born together, all the sound sent while the source closed on the listener faster than sound arriving at once: the
 * boom, which {@link Hearing#boom} reports once (whatever else is still arriving, e.g. the sound a car made standing
 * before it was launched; {@link #foldArrives}).
 * <p>
 * Positions are recorded once a tick (or so); the track keeps the last {@link #CAPACITY} of them, enough for sound to
 * travel 2 km. Not thread-safe: record and hear on one thread.
 */
public final class SourceTrack {
    /** Positions kept: 6 s at 20 a second. */
    public static final int CAPACITY = 121;
    /** Segments faster than this (m/s) are jumps (a vehicle put somewhere else), not flight: no boom comes of them (Mach 9). */
    public static final double JUMP_MPS = 3000;

    private final double[] times = new double[CAPACITY];
    private final Vec3d[] positions = new Vec3d[CAPACITY];
    private int count;
    /** When the track was last heard (NaN: not yet): a boom arriving since then is reported. */
    private double lastNow = Double.NaN;

    /** Records where the source is at a time (times must increase). */
    public void record(double time, Vec3d position) {
        if (count > 0 && time <= times[count - 1]) {
            positions[count - 1] = position;   // the same tick again: keep the newest
            return;
        }
        if (count == CAPACITY) {
            System.arraycopy(times, 1, times, 0, CAPACITY - 1);
            System.arraycopy(positions, 1, positions, 0, CAPACITY - 1);
            count--;
        }
        times[count] = time;
        positions[count] = position;
        count++;
    }

    public int size() { return count; }
    public Vec3d latest() { return count == 0 ? null : positions[count - 1]; }

    /**
     * What is heard now: arrivals (0, 1, 2 or more), the newest arrival's apparent position, its delay (s), the distance
     * it travelled, its Doppler factor, the source's velocity then, and whether a sonic boom arrived since the last
     * hearing (what the source sent while closing faster than sound, all at once).
     */
    public record Hearing(int arrivals, Vec3d apparent, double delayS, double distanceM, double doppler, Vec3d sourceVelocity, boolean boom) {
        public boolean heard() { return arrivals > 0; }
    }

    /**
     * Hears the track at the listener now; without propagation delay (delay false) it is heard where it is, Doppler
     * still from its recorded velocity when doppler is true.
     */
    public Hearing hear(Vec3d listener, Vec3d listenerVelocity, double now, double soundMps, boolean delay, boolean doppler) {
        if (count == 0) { return new Hearing(0, listener, 0, 0, 1, Vec3d.ZERO, false); }
        Vec3d here = positions[count - 1];
        Vec3d velocity = count >= 2 ? here.subtract(positions[count - 2]).scale(1 / Math.max(1e-6, times[count - 1] - times[count - 2])) : Vec3d.ZERO;
        if (!delay) {
            double f = doppler ? clampDoppler(Propagation.doppler(velocity, listenerVelocity, listener.subtract(here), soundMps)) : 1;
            return new Hearing(1, here, 0, listener.subtract(here).length(), f, velocity, false);
        }
        if (count < 2) { return new Hearing(0, here, 0, listener.subtract(here).length(), 1, velocity, false); }
        // the record ends at the newest sample; extend it to now along the last velocity so a source keeps sounding between records
        boolean extended = now > times[count - 1] + 1e-9;
        if (extended) { record(now, here.add(velocity.scale(now - times[count - 1]))); }
        List<Propagation.Emission> e = Propagation.emissions(times, positions, count, listener, now, soundMps);
        // the cone's edge: a fold of the arrival time, its sound arriving all at once, since the last hearing (not on the
        // first: the listener may already be inside the cone)
        boolean boom = !Double.isNaN(lastNow) && foldArrives(listener, lastNow, now, soundMps);
        if (extended) { count--; }   // the extension was only for this question
        lastNow = now;
        int arrivals = e.size();
        if (arrivals == 0) {
            // not reached yet (supersonic, ahead of the cone), or the record is too short for how far away it is: silent
            return new Hearing(0, here, 0, listener.subtract(here).length(), 1, velocity, false);
        }
        Propagation.Emission newest = e.getFirst();
        double f = doppler ? clampDoppler(Propagation.doppler(newest.velocity(), listenerVelocity, listener.subtract(newest.position()), soundMps)) : 1;
        return new Hearing(arrivals, newest.position(), newest.delayS(now), newest.distanceM(), f, newest.velocity(), boom);
    }

    /**
     * Whether a boom arrives within (after, upTo]: a fold of the arrival time T(t) = t + |listener - source(t)| / c over
     * the record, where T stops falling (the source stops closing on the listener faster than sound). The sound sent
     * while T fell all arrives at T's minimum; checking that moment rather than what arrives at the hearing's instant
     * catches a burst shorter than a tick (a car launched at 600 m/s from 64 m off: 8 ms), and it does not matter what
     * else is arriving then (what it sent standing, before the launch). On a straight segment T is convex, its minimum
     * where the closing speed along the line of sight is c: 1 / sqrt(M^2 - 1) of the miss distance short of abeam.
     * Segments faster than {@link #JUMP_MPS} are jumps (a vehicle put somewhere else), not flight.
     */
    private boolean foldArrives(Vec3d listener, double after, double upTo, double c) {
        double before = Double.NaN;   // dT/ds at the end of the last segment
        for (int k = 0; k + 1 < count; k++) {
            double dt = times[k + 1] - times[k];
            Vec3d d = positions[k + 1].subtract(positions[k]), u = listener.subtract(positions[k]);
            double len = d.length();
            if (!(dt > 0) || len > JUMP_MPS * dt) { before = Double.NaN; continue; }
            double start = dt - closing(u, d) / c, end = dt - closing(u.subtract(d), d) / c;
            double fold = Double.NaN;
            if (before < 0 && start >= 0) {
                fold = times[k] + u.length() / c;   // at the vertex (the source turned or stopped)
            } else if (start < 0 && end >= 0) {
                double along = u.dot(d) / len, across = u.subtract(d.scale(along / len)).length();
                double mach = len / (c * dt);   // over 1 where it closes faster than sound
                double s = Math.clamp((along - across / Math.sqrt(mach * mach - 1)) / len, 0, 1);
                fold = times[k] + s * dt + u.subtract(d.scale(s)).length() / c;
            }
            if (fold > after && fold <= upTo) { return true; }
            before = end;
        }
        return false;
    }

    /** A segment's displacement along the line of sight toward the listener (offset u from the source). */
    private static double closing(Vec3d u, Vec3d d) {
        double r = u.length();
        return r < 1e-9 ? 0 : u.dot(d) / r;
    }

    /** Keeps a Doppler factor finite (a source arriving with its own sound would be infinite): 1/8 to 8. */
    static double clampDoppler(double f) { return Double.isFinite(f) && f > 0 ? Math.max(0.125, Math.min(8, f)) : 8; }

    public void clear() { count = 0; lastNow = Double.NaN; }
}
