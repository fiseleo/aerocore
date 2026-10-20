package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

/**
 * A vehicle's clearance over the ground in side view (0.33): the underside as points (x forward, y up, in the vehicle's
 * frame standing on its wheels), the wheels as circles (centre and loaded radius); the ground is the line under the
 * frontmost and rearmost wheels. Gives the least clearance and where, and the ramp angles of SAE J1100: approach (a
 * ramp tangent to the front tyres that just touches the underside ahead of them), departure (the same behind the rear
 * tyres) and breakover between each two neighbouring axles (two ramps tangent to their tyres meeting at the underside
 * between them: the angle each makes with the ground, summed). Angles in radians; PI / 2 where nothing limits them.
 * Pure geometry: the caller decides what the underside is (a body's bottom line, a frame's members, an engine's sump).
 */
public final class Clearance {
    private Clearance() {}

    /** A wheel in side view: its centre (x, y) and loaded radius (m). */
    public record Wheel(double xM, double yM, double radiusM) {
        public Wheel {
            if (!(radiusM > 0) || !Double.isFinite(xM) || !Double.isFinite(yM)) { throw new IllegalArgumentException("A wheel needs a finite centre and a positive radius"); }
        }
    }

    /**
     * The least clearance over the ground (m; negative: the underside is below the ground line) and the x where it is,
     * the approach and departure angles and the breakover angle between each two neighbouring axles (breakoversRad, front to back).
     */
    public record Result(double minClearanceM, double minAtXM, double approachRad, double departureRad, double[] breakoversRad) {
        /** The smallest breakover angle (PI / 2 with one axle). */
        public double breakoverRad() {
            double out = Math.PI / 2;
            for (double b : breakoversRad) { out = Math.min(out, b); }
            return out;
        }
    }

    /** The clearance of an underside (points {x, y}) over wheels (two or more axles' x, or one). */
    public static Result of(List<double[]> underside, List<Wheel> wheels) {
        Objects.requireNonNull(underside, "underside");
        if (wheels.isEmpty()) { throw new IllegalArgumentException("Clearance needs a wheel"); }
        List<Wheel> sorted = new ArrayList<>(wheels);
        sorted.sort(Comparator.comparingDouble(Wheel::xM).reversed());
        Wheel front = sorted.get(0), rear = sorted.get(sorted.size() - 1);
        // the ground: under the front and rear wheels (level under one axle)
        double gx0 = front.xM(), gy0 = front.yM() - front.radiusM(), gx1 = rear.xM(), gy1 = rear.yM() - rear.radiusM();
        double slope = gx0 - gx1 > 1e-9 ? (gy0 - gy1) / (gx0 - gx1) : 0, cos = 1 / Math.sqrt(1 + slope * slope), sin = slope * cos;
        // into the ground's frame: u along it (forward), v up from it
        List<double[]> pts = new ArrayList<>();
        for (double[] p : underside) {
            double dx = p[0] - gx0, dy = p[1] - gy0;
            pts.add(new double[]{dx * cos + dy * sin, -dx * sin + dy * cos});
        }
        List<double[]> centres = new ArrayList<>();
        for (Wheel w : sorted) {
            double dx = w.xM() - gx0, dy = w.yM() - gy0;
            centres.add(new double[]{dx * cos + dy * sin, -dx * sin + dy * cos, w.radiusM()});
        }
        double min = Double.POSITIVE_INFINITY, at = Double.NaN;
        for (int i = 0; i < pts.size(); i++) {
            if (pts.get(i)[1] < min) { min = pts.get(i)[1]; at = underside.get(i)[0]; }
        }
        double[] f = centres.get(0), r = centres.get(centres.size() - 1);
        double approach = Math.PI / 2, departure = Math.PI / 2;
        for (double[] p : pts) {
            if (p[0] > f[0]) { approach = Math.min(approach, ramp(p, f, 1)); }
            if (p[0] < r[0]) { departure = Math.min(departure, ramp(p, r, -1)); }
        }
        double[] breakover = new double[centres.size() - 1];
        for (int k = 0; k + 1 < centres.size(); k++) {
            double[] ahead = centres.get(k), behind = centres.get(k + 1);
            double b = Math.PI / 2;
            for (double[] p : pts) {
                if (p[0] < ahead[0] && p[0] > behind[0]) { b = Math.min(b, ramp(p, behind, 1) + ramp(p, ahead, -1)); }
            }
            breakover[k] = b;
        }
        return new Result(min, at, approach, departure, breakover);
    }

    /**
     * The angle to the ground of a ramp tangent to a wheel (centre {u, v, r} in the ground's frame) under it and through
     * a point on the side dir (+1: the point ahead of the wheel, -1: behind): 0 when the point is on or below the
     * ground, at most PI / 2.
     */
    static double ramp(double[] p, double[] c, int dir) {
        if (p[1] <= 0) { return 0; }
        // the ramp runs from the wheel up to the point: a line through p at angle a (rising toward dir) with the wheel's
        // centre r above it: r = a's normal . (c - p), i.e. R sin(a + psi) = r with a the distance along and h the height
        double along = dir * (p[0] - c[0]), h = c[1] - p[1], big = Math.hypot(along, h);
        if (big <= c[2]) { return Math.PI / 2; }
        double psi = Math.atan2(h, along), a = Math.asin(c[2] / big) - psi;
        return Math.clamp(a, 0, Math.PI / 2);
    }
}
