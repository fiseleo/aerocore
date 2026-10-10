package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * One transverse hull section at model-local station x (m, +X forward): a convex polygon in the section plane with
 * vertices (y, z) in metres, y up and z to starboard (z = x cross y), counter-clockwise seen from the bow.
 * Clipping it by a straight waterline gives the exact wet area, centroid, waterline breadth, draft and wetted girth
 * at any heel; sections must be convex (concave hollows and tunnels are not represented).
 */
public record HullSection(double xM, List<double[]> vertices) {
    /** Wet part of the section below a line a y + b z < d (a, b the line's normal in section coordinates). */
    public record Wet(double areaM2, double centroidY, double centroidZ, double waterlineBreadthM, double draftM, double girthM,
                      double depthBelowLineM) {
        public static final Wet DRY = new Wet(0, 0, 0, 0, 0, 0, 0);
        /** True when the cut line crosses the section (a free surface pierces it). */
        public boolean piercing() { return waterlineBreadthM > 0; }
    }

    public HullSection {
        if (!Double.isFinite(xM)) { throw new IllegalArgumentException("Station must be finite"); }
        Objects.requireNonNull(vertices, "vertices");
        if (vertices.size() < 3) { throw new IllegalArgumentException("A section needs at least three vertices"); }
        List<double[]> copy = new ArrayList<>();
        for (double[] v : vertices) {
            if (v.length != 2 || !Double.isFinite(v[0]) || !Double.isFinite(v[1])) { throw new IllegalArgumentException("Vertices are finite (y, z) pairs"); }
            copy.add(v.clone());
        }
        double area = signedArea(copy);
        if (!(area > 1e-9)) { throw new IllegalArgumentException("Section must be counter-clockwise with positive area"); }
        for (int i = 0; i < copy.size(); i++) {
            double[] p = copy.get(i), q = copy.get((i + 1) % copy.size()), r = copy.get((i + 2) % copy.size());
            if ((q[0] - p[0]) * (r[1] - q[1]) - (q[1] - p[1]) * (r[0] - q[0]) < -1e-12) { throw new IllegalArgumentException("Section must be convex"); }
        }
        vertices = List.copyOf(copy);
    }

    /**
     * Wall-sided section with a circular bilge: half-beam b, keel at keelY, top (deck edge) at topY, bilge radius
     * 0..min(b, topY - keelY), bilge arc split into the given number of segments. Radius 0 gives a box.
     */
    public static HullSection rounded(double x, double halfBeam, double keelY, double topY, double bilgeRadius, int segments) {
        Checks.positive(halfBeam, "Half-beam");
        if (!(topY > keelY)) { throw new IllegalArgumentException("Top must be above keel"); }
        Checks.nonnegative(bilgeRadius, "Bilge radius");
        double r = Math.min(bilgeRadius, Math.min(halfBeam, topY - keelY));
        List<double[]> points = new ArrayList<>();
        // Counter-clockwise seen from the bow (+X towards the viewer): port top -> port bilge -> keel -> starboard.
        // With y up and z to starboard (to the viewer's left), that order is (topY, -b), down, across, up.
        points.add(new double[]{topY, halfBeam});
        if (r > 0) {
            int n = Math.max(1, segments);
            for (int i = 0; i <= n; i++) {
                double angle = 0.5 * Math.PI * i / n;
                points.add(new double[]{keelY + r - r * Math.sin(angle), halfBeam - r + r * Math.cos(angle)});
            }
            for (int i = 0; i <= n; i++) {
                double angle = 0.5 * Math.PI * i / n;
                points.add(new double[]{keelY + r - r * Math.cos(angle), -halfBeam + r - r * Math.sin(angle)});
            }
        } else {
            points.add(new double[]{keelY, halfBeam});
            points.add(new double[]{keelY, -halfBeam});
        }
        points.add(new double[]{topY, -halfBeam});
        return new HullSection(x, dedupe(orient(points)));
    }

    /** Ellipse with half-beam b and half-height c about (centerY, 0), as a polygon of the given vertex count. */
    public static HullSection ellipse(double x, double halfBeam, double centerY, double halfHeight, int count) {
        Checks.positive(halfBeam, "Half-beam"); Checks.positive(halfHeight, "Half-height");
        if (count < 8) { throw new IllegalArgumentException("Use at least eight vertices"); }
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double angle = 2 * Math.PI * i / count;
            points.add(new double[]{centerY + halfHeight * Math.sin(angle), halfBeam * Math.cos(angle)});
        }
        return new HullSection(x, orient(points));
    }

    public double area() { return signedArea(vertices); }
    /** Largest half-breadth |z| of the section. */
    public double maxHalfBreadth() { double m = 0; for (double[] v : vertices) { m = Math.max(m, Math.abs(v[1])); } return m; }
    public double minY() { double m = Double.POSITIVE_INFINITY; for (double[] v : vertices) { m = Math.min(m, v[0]); } return m; }
    public double maxY() { double m = Double.NEGATIVE_INFINITY; for (double[] v : vertices) { m = Math.max(m, v[0]); } return m; }

    /** Half-breadth of the section at height y (0 outside it): half the chord of the horizontal line through y. */
    public double halfBreadthAt(double y) {
        double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < vertices.size(); i++) {
            double[] p = vertices.get(i), q = vertices.get((i + 1) % vertices.size());
            if ((p[0] - y) * (q[0] - y) > 0 || p[0] == q[0]) { continue; }
            double z = p[1] + (y - p[0]) / (q[0] - p[0]) * (q[1] - p[1]);
            low = Math.min(low, z); high = Math.max(high, z);
        }
        return high > low ? 0.5 * (high - low) : 0;
    }

    /**
     * Part of the section with a y + b z < d (water side), where (a, b) is the upward normal of the waterline in
     * section coordinates. Returns the wet area, its centroid, the waterline length (cut chord), the draft (deepest
     * wet point below the line, measured along the normal), the wetted girth (perimeter excluding the cut) and the
     * depth of the wet centroid below the line.
     */
    public Wet clip(double a, double b, double d) {
        double norm = Math.hypot(a, b);
        if (!(norm > 0)) { throw new IllegalArgumentException("Waterline normal must be nonzero"); }
        a /= norm; b /= norm; d /= norm;
        List<double[]> wet = new ArrayList<>();
        double girth = 0, cut = 0, draft = 0;
        int n = vertices.size();
        for (int i = 0; i < n; i++) {
            double[] p = vertices.get(i), q = vertices.get((i + 1) % n);
            double sp = a * p[0] + b * p[1] - d, sq = a * q[0] + b * q[1] - d;
            if (sp < 0) { wet.add(p); draft = Math.max(draft, -sp); }
            if ((sp < 0) != (sq < 0)) {
                double t = sp / (sp - sq);
                wet.add(new double[]{p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])});
            }
            // Wetted length of this edge: the part below the line.
            double length = Math.hypot(q[0] - p[0], q[1] - p[1]);
            if (sp < 0 && sq < 0) { girth += length; }
            else if (sp < 0 || sq < 0) { girth += length * (sp < 0 ? sp / (sp - sq) : sq / (sq - sp)); }
        }
        if (wet.size() < 3) { return Wet.DRY; }
        double area = 0, cy = 0, cz = 0;
        for (int i = 0; i < wet.size(); i++) {
            double[] p = wet.get(i), q = wet.get((i + 1) % wet.size());
            double cross = p[0] * q[1] - q[0] * p[1];
            area += cross; cy += (p[0] + q[0]) * cross; cz += (p[1] + q[1]) * cross;
            // Cut edges lie on the line: their combined length is the waterline breadth.
            boolean onP = Math.abs(a * p[0] + b * p[1] - d) < 1e-12, onQ = Math.abs(a * q[0] + b * q[1] - d) < 1e-12;
            if (onP && onQ) { cut += Math.hypot(q[0] - p[0], q[1] - p[1]); }
        }
        area *= 0.5;
        if (!(area > 0)) { return Wet.DRY; }
        cy /= 6 * area; cz /= 6 * area;
        return new Wet(area, cy, cz, cut, draft, girth, d - (a * cy + b * cz));
    }

    private static double signedArea(List<double[]> points) {
        double sum = 0;
        for (int i = 0; i < points.size(); i++) {
            double[] p = points.get(i), q = points.get((i + 1) % points.size());
            sum += p[0] * q[1] - q[0] * p[1];
        }
        return 0.5 * sum;
    }
    private static List<double[]> orient(List<double[]> points) {
        if (signedArea(points) < 0) { List<double[]> reversed = new ArrayList<>(points); java.util.Collections.reverse(reversed); return reversed; }
        return points;
    }
    private static List<double[]> dedupe(List<double[]> points) {
        List<double[]> result = new ArrayList<>();
        for (double[] p : points) {
            double[] last = result.isEmpty() ? null : result.get(result.size() - 1);
            if (last == null || Math.hypot(last[0] - p[0], last[1] - p[1]) > 1e-12) { result.add(p); }
        }
        if (result.size() > 1 && Math.hypot(result.get(0)[0] - result.get(result.size() - 1)[0], result.get(0)[1] - result.get(result.size() - 1)[1]) < 1e-12) {
            result.remove(result.size() - 1);
        }
        return result;
    }
}
