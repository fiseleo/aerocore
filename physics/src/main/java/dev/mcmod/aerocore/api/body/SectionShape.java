package dev.mcmod.aerocore.api.body;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * A body's cross-section at one station, from a few shape parameters: its width, height and the height of its lowest
 * point above the body's reference line. Section coordinates as HullSection: (y up, z to starboard). Every family
 * gives a convex outline, so a body's stations are valid hull sections in water as well as in air.
 */
public sealed interface SectionShape permits SectionShape.Rounded, SectionShape.Chined, SectionShape.Cabin {
    /** Vertices of an outline (the families' own count is about this many). */
    int POINTS = 32;

    double widthM();
    double heightM();
    /** Height of the lowest point (the keel or floor) above the body's reference line. */
    double bottomM();

    /** The outline as convex (y, z) vertices, counter-clockwise as HullSection wants them. */
    List<double[]> outline();

    /** The same shape scaled about its own centre (for the point of a nose or a tail). */
    SectionShape scaled(double factor);

    /** The same shape with its bottom raised by a height (behind a step), the top where it was. */
    SectionShape raised(double byM);

    default double topM() { return bottomM() + heightM(); }
    default double centreM() { return bottomM() + 0.5 * heightM(); }

    /**
     * A superellipse |2z/w|^n + |2(y - yc)/h|^n = 1 with its own exponent above and below the centre: 2 is an ellipse
     * (a round fuselage), larger is squarer (about 8 a rounded box), a lower exponent of 1 a vee.
     */
    record Rounded(double widthM, double heightM, double bottomM, double upperExponent, double lowerExponent) implements SectionShape {
        public Rounded {
            Require.positive(widthM, "Width"); Require.positive(heightM, "Height"); Require.finite(bottomM, "Bottom");
            if (!(upperExponent >= 1 && upperExponent <= 20) || !(lowerExponent >= 1 && lowerExponent <= 20)) {
                throw new IllegalArgumentException("Superellipse exponents are 1..20");
            }
        }
        /** An ellipse. */
        public static Rounded ellipse(double widthM, double heightM, double bottomM) { return new Rounded(widthM, heightM, bottomM, 2, 2); }

        @Override public List<double[]> outline() {
            List<double[]> p = new ArrayList<>();
            double yc = centreM();
            for (int i = 0; i < POINTS; i++) {
                double t = 2 * Math.PI * i / POINTS, c = Math.cos(t), s = Math.sin(t);
                double n = s >= 0 ? upperExponent : lowerExponent;
                p.add(new double[]{yc + 0.5 * heightM * Math.signum(s) * Math.pow(Math.abs(s), 2 / n), 0.5 * widthM * Math.signum(c) * Math.pow(Math.abs(c), 2 / n)});
            }
            return convex(p);
        }
        @Override public SectionShape scaled(double f) {
            return new Rounded(widthM * f, heightM * f, centreM() - 0.5 * heightM * f, upperExponent, lowerExponent);
        }
        @Override public SectionShape raised(double by) {
            return new Rounded(widthM, Math.max(heightM - by, 0.05 * heightM), Math.min(bottomM + by, topM() - 0.05 * heightM), upperExponent, lowerExponent);
        }
    }

    /**
     * A hard-chine boat section: a vee of the deadrise angle from the keel out to the chines (width apart), sides
     * flaring outward at the flare angle up to the sheer (height above the keel).
     */
    record Chined(double widthM, double heightM, double bottomM, double deadriseDeg, double flareDeg) implements SectionShape {
        public Chined {
            Require.positive(widthM, "Chine beam"); Require.positive(heightM, "Depth"); Require.finite(bottomM, "Keel");
            if (!(deadriseDeg >= 0 && deadriseDeg < 60) || !(flareDeg >= -30 && flareDeg <= 45)) {
                throw new IllegalArgumentException("Deadrise is 0..60 deg, flare -30..45 deg");
            }
        }
        /** Height of the chines above the keel (capped just below the sheer). */
        public double chineHeightM() { return Math.min(0.5 * widthM * Math.tan(Math.toRadians(deadriseDeg)), 0.95 * heightM); }
        /** Half-breadth at the sheer. */
        public double sheerHalfBreadthM() { return Math.max(0.5 * widthM + (heightM - chineHeightM()) * Math.tan(Math.toRadians(flareDeg)), 0.05 * widthM); }

        @Override public List<double[]> outline() {
            double k = bottomM, hc = k + chineHeightM(), top = k + heightM, b = 0.5 * widthM, bs = sheerHalfBreadthM();
            List<double[]> p = new ArrayList<>();
            p.add(new double[]{k, 0});
            p.add(new double[]{hc, b}); p.add(new double[]{top, bs});
            p.add(new double[]{top, -bs}); p.add(new double[]{hc, -b});
            return convex(p);
        }
        @Override public SectionShape scaled(double f) {
            return new Chined(widthM * f, heightM * f, centreM() - 0.5 * heightM * f, deadriseDeg, flareDeg);
        }
        @Override public SectionShape raised(double by) {
            return new Chined(widthM, Math.max(heightM - by, 0.05 * heightM), Math.min(bottomM + by, topM() - 0.05 * heightM), deadriseDeg, flareDeg);
        }
    }

    /**
     * A car body's section: a flat floor whose corners are rounded by the given radius, vertical sides up to the belt
     * line (a fraction of the height), then the glasshouse leaning in to a roof of the given share of the width.
     */
    record Cabin(double widthM, double heightM, double bottomM, double beltFraction, double roofWidthRatio, double cornerRadiusM) implements SectionShape {
        public Cabin {
            Require.positive(widthM, "Width"); Require.positive(heightM, "Height"); Require.finite(bottomM, "Floor");
            if (!(beltFraction > 0.2 && beltFraction <= 1) || !(roofWidthRatio > 0.2 && roofWidthRatio <= 1) || !(cornerRadiusM >= 0)) {
                throw new IllegalArgumentException("Belt line 0.2..1 of the height, roof 0.2..1 of the width, corner radius >= 0");
            }
        }
        @Override public List<double[]> outline() {
            double b = 0.5 * widthM, y0 = bottomM, belt = y0 + beltFraction * heightM, top = y0 + heightM, roof = b * roofWidthRatio;
            double r = Math.min(cornerRadiusM, Math.min(0.45 * widthM, 0.45 * (belt - y0)));
            List<double[]> p = new ArrayList<>();
            int arc = 4;
            for (int side : new int[]{1, -1}) {
                for (int i = 0; i <= arc; i++) {
                    double a = 0.5 * Math.PI * i / arc;
                    p.add(new double[]{y0 + r - r * Math.cos(a), side * (b - r + r * Math.sin(a))});
                }
                p.add(new double[]{belt, side * b});
                p.add(new double[]{top, side * roof});
            }
            return convex(p);
        }
        @Override public SectionShape scaled(double f) {
            return new Cabin(widthM * f, heightM * f, centreM() - 0.5 * heightM * f, beltFraction, roofWidthRatio, cornerRadiusM * f);
        }
        @Override public SectionShape raised(double by) {
            double h = Math.max(heightM - by, 0.05 * heightM);
            return new Cabin(widthM, h, Math.min(bottomM + by, topM() - 0.05 * heightM), Math.min(1, beltFraction * heightM / h), roofWidthRatio, cornerRadiusM);
        }
    }

    /**
     * The convex hull of points, counter-clockwise in HullSection's sense (positive signed area of y_i z_i+1 - y_i+1 z_i),
     * without repeated points.
     */
    static List<double[]> convex(List<double[]> points) {
        // snapped to a nanometre and without repeats: a blend gives each corner a dozen copies differing in the last bit,
        // and copies of two corners at one height interleaved in the sort dropped a real corner (a lopsided station)
        List<double[]> pts = new ArrayList<>();
        for (double[] v : points) { pts.add(new double[]{Math.rint(v[0] * 1e9) / 1e9, Math.rint(v[1] * 1e9) / 1e9}); }
        pts.sort(Comparator.<double[]>comparingDouble(v -> v[0]).thenComparingDouble(v -> v[1]));
        for (int i = pts.size() - 1; i > 0; i--) {
            if (pts.get(i)[0] == pts.get(i - 1)[0] && pts.get(i)[1] == pts.get(i - 1)[1]) { pts.remove(i); }
        }
        if (pts.size() < 3) { return pts; }
        List<double[]> hull = new ArrayList<>();
        for (int pass = 0; pass < 2; pass++) {
            int start = hull.size();
            for (double[] v : pts) {
                while (hull.size() >= start + 2 && cross(hull.get(hull.size() - 2), hull.get(hull.size() - 1), v) <= 1e-14) { hull.remove(hull.size() - 1); }
                hull.add(v);
            }
            hull.remove(hull.size() - 1);
            java.util.Collections.reverse(pts);
        }
        return hull;
    }

    private static double cross(double[] o, double[] a, double[] b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
}
