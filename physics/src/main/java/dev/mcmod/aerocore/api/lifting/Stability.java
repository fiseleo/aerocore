package dev.mcmod.aerocore.api.lifting;

import dev.mcmod.aerocore.api.Atmosphere;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

/**
 * Static tipping of a machine standing on a support polygon (outrigger pads, the corners of the track or tyre
 * contacts): the moments about the governing tipping edge, the hook load that tips it at a radius and slewing angle,
 * the least stable slewing direction, and rated loads as a share of the tipping load. Formulas and the sourced rating
 * rules; the machine's masses and supports are the caller's. The physics engine simulates the actual tipping; these
 * are the numbers a load moment indicator works with.
 *
 * <p>Plan coordinates (m) in the carrier's frame: x forward, y to the left, z up (out of the plan). The superstructure
 * slews about the slewing centre; its masses are given in its own frame, x along the boom from the slewing centre and y
 * to the left, and turn with the slewing angle (rad, counter-clockwise seen from above, 0 over the carrier's x). The
 * hook hangs at a radius from the slewing centre along the boom.
 *
 * <p>The machine tips about an edge of the convex hull of its supports when the combined centre of gravity crosses it.
 * About an edge with outward normal n at offset c (inside n.p &lt; c) a mass m at p has the arm n.p - c: outside it
 * overturns, inside it stabilises. Rigid machine on rigid ground; deflections of tyres, outriggers and the ground
 * (which move the centre of gravity outward) and dynamic loads are left out.
 */
public final class Stability {
    private Stability() {}

    private static final double G = Atmosphere.GRAVITY_MPS2;
    /** Slewing angles tried before refining the least stable one. */
    private static final int SWEEP = 720;

    /** A point in plan (m). */
    public record Point(double xM, double yM) {
        public Point {
            Check.finite(xM, "x"); Check.finite(yM, "y");
        }
    }

    /** A mass (kg, not negative) at a point in plan (m). */
    public record Mass(double massKg, double xM, double yM) {
        public Mass {
            Check.nonNegative(massKg, "Mass"); Check.finite(xM, "x"); Check.finite(yM, "y");
        }
    }

    /**
     * A slewing machine: its supports in plan (at least three, not all on a line; any order), the carrier's masses
     * (carrier frame), the slewing centre (carrier frame) and the superstructure's masses (its own frame: x along the
     * boom from the slewing centre): turntable, cab, winches, counterweight, and the boom at its centre of gravity.
     */
    public record Crane(List<Point> support, List<Mass> carrier, Point slewCentre, List<Mass> upper) {
        public Crane {
            support = List.copyOf(support);
            carrier = List.copyOf(carrier);
            Objects.requireNonNull(slewCentre, "slewCentre");
            upper = List.copyOf(upper);
            supportPolygon(support);
        }

        /** The same machine with one more superstructure mass (its own frame). */
        public Crane withUpper(Mass mass) {
            List<Mass> more = new ArrayList<>(upper);
            more.add(Objects.requireNonNull(mass, "mass"));
            return new Crane(support, carrier, slewCentre, more);
        }
    }

    /**
     * Moments (N m) about a tipping edge running from one support to the next (counter-clockwise): the overturning
     * moment of the masses outside it and the stabilising moment of those inside.
     */
    public record Moments(Point from, Point to, double overturningNm, double stabilisingNm) {
        /** Stabilising less overturning moment (N m): negative when the machine tips about this edge. */
        public double marginNm() { return stabilisingNm - overturningNm; }

        /** Stabilising over overturning moment (infinite with nothing overturning). */
        public double ratio() { return overturningNm == 0 ? Double.POSITIVE_INFINITY : stabilisingNm / overturningNm; }
    }

    /** The least stable slewing angle (rad, 0..2 pi) at a radius and the tipping load there (kg). */
    public record Worst(double slewRad, double tippingKg) {}

    /**
     * Rated load from a tipping load. The share rules are those of the US crane standard: rated loads "shall not exceed"
     * 75 % of the tipping load for crawler cranes and 85 % on outriggers fully extended, with the boom in the least stable
     * direction (OSHA 29 CFR 1910.180(c)(1)(i), which reproduces ANSI B30.5-1968 and refers to SAE J765; ASME B30.5
     * 5-1.1.1 is the current home of the rule). The ISO rules ask that the crane does not tip under 1.25 P + 0.1 F, P the rated load and F
     * the boom's (and fly jib's) mass reduced to the boom head (ISO 4305 on outriggers or crawlers), and 1.33 P + 0.1 F
     * on tyres (as summarised from ISO 4305:2014; the standard's text was not at hand).
     */
    public enum Rule {
        /** Crawler cranes without outriggers: 75 % of tipping. */
        CRAWLER(0.75, 0),
        /** On outriggers fully extended and set: 85 % of tipping. */
        OUTRIGGERS(0.85, 0),
        /**
         * Wheel-mounted cranes lifting on their tyres: 75 % of tipping, as later ASME B30.5 editions rate them (quoted
         * from secondary sources); the 1968 table in OSHA 1910.180 allowed 85 % for truck and wheel cranes either way.
         */
        TYRES(0.75, 0),
        /** ISO 4305 on outriggers or crawlers: 1.25 P + 0.1 F must not tip the crane. */
        ISO_4305(1 / 1.25, 0.1),
        /** ISO 4305 on tyres: 1.33 P + 0.1 F must not tip the crane. */
        ISO_4305_TYRES(1 / 1.33, 0.1);

        private final double share, boomFactor;

        Rule(double share, double boomFactor) {
            this.share = share;
            this.boomFactor = boomFactor;
        }

        /**
         * Rated load (kg, not negative) from the tipping load at the hook (kg) and the boom's mass reduced to its head
         * (kg; the boom's moment about its foot over its length, mass x centroid share): share x (tipping - factor x F).
         * An infinite tipping load gives an infinite rating.
         */
        public double ratedKg(double tippingKg, double boomHeadKg) {
            if (!(tippingKg >= 0)) { throw new IllegalArgumentException("Tipping load must not be negative"); }
            Check.nonNegative(boomHeadKg, "Boom head mass");
            if (tippingKg == Double.POSITIVE_INFINITY) { return Double.POSITIVE_INFINITY; }
            return Math.max(0, share * (tippingKg - boomFactor * boomHeadKg));
        }
    }

    /** The convex hull (counter-clockwise from above) of the supports; at least three points not on one line. */
    public static List<Point> supportPolygon(List<Point> points) {
        Objects.requireNonNull(points, "points");
        List<Point> sorted = new ArrayList<>(points);
        for (Point p : sorted) { Objects.requireNonNull(p, "support point"); }
        sorted.sort(Comparator.comparingDouble(Point::xM).thenComparingDouble(Point::yM));
        int n = sorted.size();
        Point[] hull = new Point[2 * n + 1];
        int k = 0;
        for (Point p : sorted) {
            while (k >= 2 && turn(hull[k - 2], hull[k - 1], p) <= 0) { k--; }
            hull[k++] = p;
        }
        for (int i = n - 2, lower = k + 1; i >= 0; i--) {
            Point p = sorted.get(i);
            while (k >= lower && turn(hull[k - 2], hull[k - 1], p) <= 0) { k--; }
            hull[k++] = p;
        }
        if (k - 1 < 3) { throw new IllegalArgumentException("The supports must span an area"); }
        return List.of(Arrays.copyOf(hull, k - 1));
    }

    /** The edge with the least margin (stabilising less overturning moment) for masses placed in plan (carrier frame). */
    public static Moments governing(List<Point> support, List<Mass> masses) {
        List<Point> hull = supportPolygon(support);
        Objects.requireNonNull(masses, "masses");
        Moments worst = null;
        for (int i = 0; i < hull.size(); i++) {
            Point a = hull.get(i), b = hull.get((i + 1) % hull.size());
            double[] e = edge(a, b);
            double over = 0, stab = 0;
            for (Mass m : masses) {
                double arm = e[0] * m.xM() + e[1] * m.yM() - e[2];
                if (arm > 0) { over += m.massKg() * G * arm; } else { stab -= m.massKg() * G * arm; }
            }
            Moments here = new Moments(a, b, over, stab);
            if (worst == null || here.marginNm() < worst.marginNm()) { worst = here; }
        }
        return worst;
    }

    /** All the machine's masses in the carrier frame at a slewing angle (rad): the carrier's, then the superstructure's turned about the slewing centre. */
    public static List<Mass> placed(Crane crane, double slewRad) {
        Objects.requireNonNull(crane, "crane");
        Check.finite(slewRad, "Slewing angle");
        double c = Math.cos(slewRad), s = Math.sin(slewRad);
        List<Mass> out = new ArrayList<>(crane.carrier());
        for (Mass m : crane.upper()) {
            out.add(new Mass(m.massKg(), crane.slewCentre().xM() + m.xM() * c - m.yM() * s, crane.slewCentre().yM() + m.xM() * s + m.yM() * c));
        }
        return out;
    }

    /** Where the hook hangs (carrier frame) at a radius (m) from the slewing centre and a slewing angle (rad). */
    public static Point hook(Crane crane, double radiusM, double slewRad) {
        Objects.requireNonNull(crane, "crane");
        Check.nonNegative(radiusM, "Radius"); Check.finite(slewRad, "Slewing angle");
        return new Point(crane.slewCentre().xM() + radiusM * Math.cos(slewRad), crane.slewCentre().yM() + radiusM * Math.sin(slewRad));
    }

    /**
     * The hook load (kg, block and slings included) at a point that brings the machine to balance about its least
     * stable edge: for each edge the load sits outside of, (sum of m (c - n.p)) / (n.h - c), the least of them. Positive
     * infinity when the hook is inside the support polygon (no load tips it); 0 when the machine already tips (or
     * balances) without a load about some edge, as an over-ballasted crane does backward.
     */
    public static double tippingLoadKg(List<Point> support, List<Mass> masses, Point hook) {
        List<Point> hull = supportPolygon(support);
        Objects.requireNonNull(masses, "masses");
        Objects.requireNonNull(hook, "hook");
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < hull.size(); i++) {
            double[] e = edge(hull.get(i), hull.get((i + 1) % hull.size()));
            double stabilising = 0;
            for (Mass m : masses) { stabilising += m.massKg() * (e[2] - e[0] * m.xM() - e[1] * m.yM()); }
            if (!(stabilising > 0)) { return 0; }
            double arm = e[0] * hook.xM() + e[1] * hook.yM() - e[2];
            if (arm > 0) { best = Math.min(best, stabilising / arm); }
        }
        return best;
    }

    /** The tipping load (kg) at a radius (m) and slewing angle (rad): tippingLoadKg of the placed machine at the hook. */
    public static double tippingLoadKg(Crane crane, double radiusM, double slewRad) {
        return tippingLoadKg(crane.support(), placed(crane, slewRad), hook(crane, radiusM, slewRad));
    }

    /**
     * The least stable slewing direction at a radius (m): the slewing angle (rad, 0..2 pi) with the least tipping load,
     * from a sweep of 720 angles refined by a golden-section search between the neighbours of the least. With the
     * masses at the centre of a rectangle of supports it is square to the nearest edge, not over a pad: a corner lies
     * farther out from the centre than the edges either side of it (a pad's own load, on the other hand, is greatest
     * with the boom over it).
     */
    public static Worst worstSlew(Crane crane, double radiusM) {
        Objects.requireNonNull(crane, "crane");
        Check.nonNegative(radiusM, "Radius");
        double best = Double.POSITIVE_INFINITY, bestAngle = 0;
        double step = 2 * Math.PI / SWEEP;
        for (int i = 0; i < SWEEP; i++) {
            double t = tippingLoadKg(crane, radiusM, i * step);
            if (t < best) { best = t; bestAngle = i * step; }
        }
        if (best == Double.POSITIVE_INFINITY || best == 0) { return new Worst(bestAngle, best); }
        double lo = bestAngle - step, hi = bestAngle + step, golden = (Math.sqrt(5) - 1) / 2;
        double x1 = hi - golden * (hi - lo), x2 = lo + golden * (hi - lo);
        double f1 = tippingLoadKg(crane, radiusM, x1), f2 = tippingLoadKg(crane, radiusM, x2);
        for (int i = 0; i < 60; i++) {
            if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - golden * (hi - lo); f1 = tippingLoadKg(crane, radiusM, x1); }
            else { lo = x1; x1 = x2; f1 = f2; x2 = lo + golden * (hi - lo); f2 = tippingLoadKg(crane, radiusM, x2); }
        }
        if (f1 < best) { best = f1; bestAngle = x1; }
        if (f2 < best) { best = f2; bestAngle = x2; }
        double turn = 2 * Math.PI;
        return new Worst(((bestAngle % turn) + turn) % turn, best);
    }

    /** z of (b - a) x (c - a): positive when a, b, c turn counter-clockwise. */
    private static double turn(Point a, Point b, Point c) {
        return (b.xM() - a.xM()) * (c.yM() - a.yM()) - (b.yM() - a.yM()) * (c.xM() - a.xM());
    }

    /** The outward unit normal (nx, ny) and offset c of a counter-clockwise edge from a to b: inside is n.p &lt; c. */
    private static double[] edge(Point a, Point b) {
        double ex = b.xM() - a.xM(), ey = b.yM() - a.yM(), len = Math.hypot(ex, ey);
        double nx = ey / len, ny = -ex / len;
        return new double[]{nx, ny, nx * a.xM() + ny * a.yM()};
    }
}
