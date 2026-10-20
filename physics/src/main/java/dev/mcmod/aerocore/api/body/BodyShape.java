package dev.mcmod.aerocore.api.body;

import dev.mcmod.aerocore.api.HullSection;
import dev.mcmod.aerocore.api.PlaningHull;
import dev.mcmod.aerocore.api.SlenderHull;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A body made of modules strung from the nose back: its stations and what follows from them. Body axes as SlenderHull:
 * x forward with x = 0 at the rear end, y up, z to starboard; heights are measured from the body's reference line
 * (y = 0, usually the keel or the floor). The modules are listed nose first; each starts from the rear section of the
 * one ahead of it (or the nose section), so the joints always match, and a step raises the bottom where a module starts.
 */
public final class BodyShape {
    /** One station: its outline (convex, as a hull section), the module it belongs to and its measures. */
    public record Station(double xM, List<double[]> outline, int module, double areaM2, double perimeterM, double widthM,
                          double bottomM, double topM, double centroidYM) {
        public Station { outline = List.copyOf(outline); }
    }

    /**
     * Overall measures: volume and its centroid, wetted area (the sides, without the end faces), the largest section (frontal) area and where it
     * is, width, height, planform (top view) and side areas, fineness (length over the diameter of a circle of the
     * frontal area), the areas of the blunt rear and nose ends, the tail's upsweep (the rise of the section centres from
     * the largest section to the rear end, degrees) and the rear slant (degrees): the line from the roof's trailing edge (the
     * rearmost point within 5 mm of the greatest height) to the top of the rear end, which is the last module's own slope
     * for a fastback or the Ahmed body and, for a notchback, the effective backlight angle across the rear window to the
     * trunk lid's edge (Hucho, Aerodynamics of Road Vehicles; over the last module alone, before 0.23, a notchback's flat
     * trunk read as a square back).
     */
    public record Particulars(double lengthM, double volumeM3, double centroidXM, double centroidYM, double wettedAreaM2,
                              double frontalAreaM2, double xMaxAreaM, double maxWidthM, double maxHeightM, double planformAreaM2,
                              double sideAreaM2, double finenessRatio, double baseAreaM2, double noseAreaM2, double upsweepDeg,
                              double rearSlantDeg) {}

    private final SectionShape nose;
    private final List<BodyModule> modules;
    private final List<Station> stations;
    private final double[] front, rear;
    private final double length;
    private Particulars particulars;

    public BodyShape(SectionShape nose, List<BodyModule> modules) {
        this.nose = Objects.requireNonNull(nose, "nose");
        this.modules = List.copyOf(modules);
        if (this.modules.isEmpty()) { throw new IllegalArgumentException("A body needs at least one module"); }
        int n = this.modules.size();
        front = new double[n]; rear = new double[n];
        double total = 0;
        for (BodyModule m : this.modules) { total += m.lengthM(); }
        length = total;
        double x = total;
        for (int i = 0; i < n; i++) { front[i] = x; x -= this.modules.get(i).lengthM(); rear[i] = Math.max(x, 0); }
        // stations nose first, then reversed into increasing x
        List<Station> fromNose = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            BodyModule m = this.modules.get(i);
            SectionShape start = i == 0 ? nose : this.modules.get(i - 1).rear();
            if (m.stepM() > 0 && i > 0) { start = start.raised(m.stepM()); }
            List<double[]> a = start.outline(), b = m.rear().outline();
            boolean growing = area(b) >= area(a);
            int intervals = Math.max(3, Math.min(16, (int) Math.ceil(m.lengthM() / (total / 48)) + (m.fullness() != 1 ? 4 : 0)));
            for (int j = 0; j <= intervals; j++) {
                if (j == 0 && i > 0 && !(m.stepM() > 0)) { continue; }   // the joint is the module ahead's rear station
                double t = 0.5 * (1 - Math.cos(Math.PI * j / intervals));
                // a superellipse profile from the larger end: fullness 2 an exact ellipse, 1 a straight cone
                double p = m.fullness();
                double g = growing ? Math.pow(1 - Math.pow(1 - t, p), 1 / p) : 1 - Math.pow(1 - Math.pow(t, p), 1 / p);
                double xs = front[i] - t * m.lengthM();
                if (j == 0 && i > 0) { xs -= 1e-3; }   // behind a step: just aft of the forebody's last station
                fromNose.add(station(Math.max(xs, 0), blend(a, b, g), i));
            }
        }
        List<Station> out = new ArrayList<>();
        for (int k = fromNose.size() - 1; k >= 0; k--) {
            Station s = fromNose.get(k);
            if (!out.isEmpty() && !(s.xM() > out.get(out.size() - 1).xM() + 1e-6)) { continue; }
            out.add(s);
        }
        stations = List.copyOf(out);
    }

    public SectionShape nose() { return nose; }
    public List<BodyModule> modules() { return modules; }
    public List<Station> stations() { return stations; }
    public double lengthM() { return length; }
    /** Where a module starts (its forward end) and ends, x from the rear. */
    public double moduleFrontX(int i) { return front[i]; }
    public double moduleRearX(int i) { return rear[i]; }

    /** The outer shape as hull sections. */
    public SlenderHull hull() {
        List<HullSection> sections = new ArrayList<>();
        for (Station s : stations) { sections.add(new HullSection(s.xM(), s.outline())); }
        return new SlenderHull(sections);
    }

    /**
     * The part of the shape that keeps the water out, as hull sections: an open module's sections cut at its sill (water
     * above it pours in, so it carries nothing); a section wholly above its sill keeps a sliver at the bottom.
     */
    public SlenderHull waterHull() {
        List<HullSection> sections = new ArrayList<>();
        for (Station s : stations) {
            double sill = modules.get(s.module()).sillM();
            List<double[]> outline = s.outline();
            if (sill < s.topM()) { outline = below(outline, Math.max(sill, s.bottomM() + 0.02 * Math.max(s.topM() - s.bottomM(), 1e-3))); }
            sections.add(new HullSection(s.xM(), outline));
        }
        return new SlenderHull(sections);
    }

    /** A convex outline cut by the horizontal line y = h, the part below (Sutherland-Hodgman, so it stays convex). */
    static List<double[]> below(List<double[]> outline, double h) {
        List<double[]> out = new ArrayList<>();
        int n = outline.size();
        for (int i = 0; i < n; i++) {
            double[] p = outline.get(i), q = outline.get((i + 1) % n);
            boolean pin = p[0] <= h, qin = q[0] <= h;
            if (pin) { out.add(p.clone()); }
            if (pin != qin) {
                double t = (h - p[0]) / (q[0] - p[0]);
                out.add(new double[]{h, p[1] + t * (q[1] - p[1])});
            }
        }
        return SectionShape.convex(out);
    }

    /**
     * The shape as a hard-chine planing hull (PlaningDynamics): at each station the keel (lowest point), the chine
     * (where the bottom turns up into the side, the first edge steeper than 45 degrees going out from the keel) and the
     * deadrise between them; a step where a module starts with one.
     */
    public PlaningHull planingHull() {
        int n = stations.size();
        double[] keel = new double[n], chine = new double[n], deadrise = new double[n];
        for (int i = 0; i < n; i++) {
            Station st = stations.get(i);
            double[] c = chine(st.outline());
            keel[i] = st.bottomM();
            chine[i] = Math.max(c[1], 0.01 * Math.max(st.widthM(), 1e-3));
            deadrise[i] = Math.clamp(Math.atan2(c[0] - st.bottomM(), chine[i]), Math.toRadians(1), Math.toRadians(59));
        }
        List<Double> steps = new ArrayList<>();
        for (int i = 1; i < modules.size(); i++) { if (modules.get(i).stepM() > 0) { steps.add(front[i]); } }
        return PlaningHull.of(waterHull(), length, keel, chine, deadrise, steps.stream().mapToDouble(Double::doubleValue).toArray());
    }

    /** Whether the body's bottom is hard-chined (most of its length in modules ending in a chined section). */
    public boolean hardChined() {
        double chined = 0;
        for (BodyModule m : modules) { if (m.rear() instanceof SectionShape.Chined) { chined += m.lengthM(); } }
        return chined > 0.5 * length;
    }

    /** The chine of an outline: (height, half-breadth) of the starboard vertex where the bottom ends. */
    private static double[] chine(List<double[]> outline) {
        List<double[]> starboard = new ArrayList<>();
        for (double[] v : outline) { if (v[1] >= -1e-9) { starboard.add(v); } }
        starboard.sort((a, b) -> a[0] != b[0] ? Double.compare(a[0], b[0]) : Double.compare(b[1], a[1]));
        double[] at = starboard.get(0);
        for (int k = 1; k < starboard.size(); k++) {
            double[] next = starboard.get(k);
            double dy = next[0] - at[0], dz = next[1] - at[1];
            if (dz <= 0 || dy > dz) { break; }   // the side: going in, or steeper than 45 degrees
            at = next;
        }
        return new double[]{at[0], Math.abs(at[1])};
    }

    /** Bottom, top, width and section centre height at x, interpolated between stations (0 outside the body). */
    public double bottomAt(double x) { return interpolate(x, 0); }
    public double topAt(double x) { return interpolate(x, 1); }
    public double widthAt(double x) { return interpolate(x, 2); }
    public double centreAt(double x) { return interpolate(x, 3); }

    private double interpolate(double x, int what) {
        if (x <= stations.get(0).xM()) { return value(stations.get(0), what); }
        for (int i = 1; i < stations.size(); i++) {
            Station a = stations.get(i - 1), b = stations.get(i);
            if (x <= b.xM()) {
                double t = (x - a.xM()) / (b.xM() - a.xM());
                return value(a, what) + t * (value(b, what) - value(a, what));
            }
        }
        return value(stations.get(stations.size() - 1), what);
    }

    private static double value(Station s, int what) {
        return switch (what) { case 0 -> s.bottomM(); case 1 -> s.topM(); case 2 -> s.widthM(); default -> 0.5 * (s.bottomM() + s.topM()); };
    }

    public Particulars particulars() {
        if (particulars != null) { return particulars; }
        double volume = 0, mx = 0, my = 0, wetted = 0, planform = 0, side = 0, maxArea = 0, xMax = 0, maxWidth = 0, maxHeight = 0;
        for (Station s : stations) {
            if (s.areaM2() > maxArea) { maxArea = s.areaM2(); xMax = s.xM(); }
            maxWidth = Math.max(maxWidth, s.widthM()); maxHeight = Math.max(maxHeight, s.topM() - s.bottomM());
        }
        for (int i = 1; i < stations.size(); i++) {
            Station a = stations.get(i - 1), b = stations.get(i);
            double dx = b.xM() - a.xM();
            double v = 0.5 * (a.areaM2() + b.areaM2()) * dx;
            volume += v; mx += v * 0.5 * (a.xM() + b.xM()); my += v * 0.5 * (a.centroidYM() + b.centroidYM());
            double dr = (b.perimeterM() - a.perimeterM()) / (2 * Math.PI);
            wetted += 0.5 * (a.perimeterM() + b.perimeterM()) * Math.sqrt(dx * dx + dr * dr);
            planform += 0.5 * (a.widthM() + b.widthM()) * dx;
            side += 0.5 * (a.topM() - a.bottomM() + b.topM() - b.bottomM()) * dx;
        }
        Station tail = stations.get(0), tip = stations.get(stations.size() - 1);
        double base = tail.areaM2(), noseArea = tip.areaM2();
        double fineness = length / Math.sqrt(4 * Math.max(maxArea, 1e-12) / Math.PI);
        // the tail's rise: the section centres from the largest section to the rear end
        Station big = stations.get(0);
        for (Station s : stations) { if (s.areaM2() >= maxArea) { big = s; break; } }
        double run = big.xM() - tail.xM();
        double upsweep = run > 1e-6 ? Math.toDegrees(Math.atan2(tail.centroidYM() - big.centroidYM(), run)) : 0;
        double highest = Double.NEGATIVE_INFINITY;
        for (Station s : stations) { highest = Math.max(highest, s.topM()); }
        Station roofEnd = tail;
        for (Station s : stations) { if (s.topM() >= highest - 0.005) { roofEnd = s; break; } }
        double runRoof = roofEnd.xM() - tail.xM();
        double slant = runRoof > 1e-6 ? Math.toDegrees(Math.atan2(roofEnd.topM() - tail.topM(), runRoof)) : 0;
        particulars = new Particulars(length, volume, volume > 0 ? mx / volume : 0, volume > 0 ? my / volume : 0, wetted,
                maxArea, xMax, maxWidth, maxHeight, planform, side, fineness, base, noseArea, upsweep, slant);
        return particulars;
    }

    // ---------------------------------------------------------------- fitting (0.32)

    /**
     * How far something sticks out of the body, m, through its top, its port (-z) and starboard (+z) sides, past the
     * nose (front) and past the rear end. Below the section counts as inside: a bay is open underneath (an engine's sump
     * may hang below the body's bottom line).
     */
    public record Overhang(double topM, double portM, double starboardM, double frontM, double rearM) {
        public static final Overhang NONE = new Overhang(0, 0, 0, 0, 0);

        /** The farthest it sticks out on any side. */
        public double maxM() { return Math.max(Math.max(topM, Math.max(portM, starboardM)), Math.max(frontM, rearM)); }

        /** The side it sticks out farthest ("top", "port", "starboard", "front", "rear"), "" when inside. */
        public String worst() {
            double m = maxM();
            if (!(m > 0)) { return ""; }
            return m == topM ? "top" : m == portM ? "port" : m == starboardM ? "starboard" : m == frontM ? "front" : "rear";
        }

        /** Side by side the larger of the two. */
        public Overhang max(Overhang o) {
            return new Overhang(Math.max(topM, o.topM), Math.max(portM, o.portM), Math.max(starboardM, o.starboardM), Math.max(frontM, o.frontM), Math.max(rearM, o.rearM));
        }
    }

    /** The section at x (from the rear), (y, z) points counter-clockwise: the two stations either side blended; an end station beyond either end. */
    public List<double[]> outlineAt(double x) {
        if (x <= stations.get(0).xM()) { return copy(stations.get(0).outline()); }
        for (int i = 1; i < stations.size(); i++) {
            Station a = stations.get(i - 1), b = stations.get(i);
            if (x <= b.xM()) { return blend(a.outline(), b.outline(), (x - a.xM()) / (b.xM() - a.xM())); }
        }
        return copy(stations.get(stations.size() - 1).outline());
    }

    /** How far these points (body frame: x from the rear end, y up, z starboard) stick out of the body, each side the most of any. */
    public Overhang overhang(Iterable<dev.mcmod.aerocore.api.Vec3d> points) {
        java.util.Map<Long, List<double[]>> sections = new java.util.HashMap<>();
        double top = 0, port = 0, starboard = 0, fore = 0, aft = 0;
        for (dev.mcmod.aerocore.api.Vec3d p : points) {
            fore = Math.max(fore, p.x() - length);
            aft = Math.max(aft, -p.x());
            double x = Math.clamp(p.x(), 0, length);
            List<double[]> outline = sections.computeIfAbsent(Math.round(x * 1e4), k -> outlineAt(x));
            double zMin = Double.POSITIVE_INFINITY, zMax = Double.NEGATIVE_INFINITY;
            for (double[] q : outline) { zMin = Math.min(zMin, q[1]); zMax = Math.max(zMax, q[1]); }
            port = Math.max(port, zMin - p.z());
            starboard = Math.max(starboard, p.z() - zMax);
            top = Math.max(top, p.y() - upper(outline, Math.clamp(p.z(), zMin, zMax)));
        }
        return new Overhang(Math.max(top, 0), Math.max(port, 0), Math.max(starboard, 0), Math.max(fore, 0), Math.max(aft, 0));
    }

    /**
     * How far a box sticks out of the body: min to max in a part's frame, the part turned by rotation and its origin at
     * at (body frame). Its six faces are sampled on a grid no coarser than stepM (not only its corners: a body lower in the
     * middle than at both corners' x would let a box's middle through).
     */
    public Overhang overhang(dev.mcmod.aerocore.api.Vec3d min, dev.mcmod.aerocore.api.Vec3d max, dev.mcmod.aerocore.api.Orientation rotation,
                             dev.mcmod.aerocore.api.Vec3d at, double stepM) {
        if (!(stepM > 0)) { throw new IllegalArgumentException("Step must be positive"); }
        double[] lo = {min.x(), min.y(), min.z()}, hi = {max.x(), max.y(), max.z()};
        int[] n = new int[3];
        for (int k = 0; k < 3; k++) { n[k] = Math.max(1, (int) Math.ceil((hi[k] - lo[k]) / stepM)); }
        List<dev.mcmod.aerocore.api.Vec3d> points = new ArrayList<>();
        for (int face = 0; face < 3; face++) {
            int u = (face + 1) % 3, v = (face + 2) % 3;
            for (double fixed : new double[]{lo[face], hi[face]}) {
                for (int i = 0; i <= n[u]; i++) {
                    for (int j = 0; j <= n[v]; j++) {
                        double[] c = new double[3];
                        c[face] = fixed;
                        c[u] = lo[u] + (hi[u] - lo[u]) * i / n[u];
                        c[v] = lo[v] + (hi[v] - lo[v]) * j / n[v];
                        points.add(at.add(rotation.rotate(new dev.mcmod.aerocore.api.Vec3d(c[0], c[1], c[2]))));
                    }
                }
            }
        }
        return overhang(points);
    }

    /** The highest y of a section at z (its upper edge there). */
    private static double upper(List<double[]> outline, double z) {
        double best = Double.NEGATIVE_INFINITY;
        int n = outline.size();
        for (int i = 0; i < n; i++) {
            double[] p = outline.get(i), q = outline.get((i + 1) % n);
            double z0 = Math.min(p[1], q[1]), z1 = Math.max(p[1], q[1]);
            if (z < z0 - 1e-12 || z > z1 + 1e-12) { continue; }
            double y = z1 - z0 < 1e-12 ? Math.max(p[0], q[0]) : p[0] + (q[0] - p[0]) * (z - p[1]) / (q[1] - p[1]);
            best = Math.max(best, y);
        }
        return best;
    }

    // ---------------------------------------------------------------- sections

    private static Station station(double x, List<double[]> outline, int module) {
        double perimeter = 0, bottom = Double.POSITIVE_INFINITY, top = Double.NEGATIVE_INFINITY, zMin = Double.POSITIVE_INFINITY, zMax = Double.NEGATIVE_INFINITY;
        double a = 0, cy = 0;
        int n = outline.size();
        for (int i = 0; i < n; i++) {
            double[] p = outline.get(i), q = outline.get((i + 1) % n);
            perimeter += Math.hypot(q[0] - p[0], q[1] - p[1]);
            double c = p[0] * q[1] - q[0] * p[1];
            a += c; cy += (p[0] + q[0]) * c;
            bottom = Math.min(bottom, p[0]); top = Math.max(top, p[0]); zMin = Math.min(zMin, p[1]); zMax = Math.max(zMax, p[1]);
        }
        a *= 0.5;
        return new Station(x, outline, module, a, perimeter, zMax - zMin, bottom, top, a > 0 ? cy / (6 * a) : 0.5 * (bottom + top));
    }

    static double area(List<double[]> outline) {
        double a = 0;
        for (int i = 0; i < outline.size(); i++) {
            double[] p = outline.get(i), q = outline.get((i + 1) % outline.size());
            a += p[0] * q[1] - q[0] * p[1];
        }
        return 0.5 * a;
    }

    /**
     * The Minkowski combination (1 - t) A + t B of two convex outlines, from their support functions over every edge
     * normal of both (so corners and flat sides come out exact) and a uniform fan between them.
     */
    static List<double[]> blend(List<double[]> a, List<double[]> b, double t) {
        if (t <= 0) { return copy(a); }
        if (t >= 1) { return copy(b); }
        List<Double> angles = new ArrayList<>();
        for (int k = 0; k < 64; k++) { angles.add(2 * Math.PI * k / 64); }
        for (List<double[]> poly : List.of(a, b)) {
            for (int i = 0; i < poly.size(); i++) {
                double[] p = poly.get(i), q = poly.get((i + 1) % poly.size());
                // the outward normal of a counter-clockwise edge in (y, z): (dz, -dy)
                double angle = Math.atan2(-(q[0] - p[0]), q[1] - p[1]);
                angles.add(angle < 0 ? angle + 2 * Math.PI : angle);
            }
        }
        angles.sort(Double::compare);
        List<double[]> dirs = new ArrayList<>();
        List<Double> h = new ArrayList<>();
        double last = Double.NEGATIVE_INFINITY;
        for (double angle : angles) {
            if (angle - last < 1e-9) { continue; }
            last = angle;
            double[] n = {Math.cos(angle), Math.sin(angle)};
            dirs.add(n);
            h.add((1 - t) * support(a, n) + t * support(b, n));
        }
        List<double[]> out = new ArrayList<>();
        int m = dirs.size();
        for (int k = 0; k < m; k++) {
            double[] n1 = dirs.get(k), n2 = dirs.get((k + 1) % m);
            double det = n1[0] * n2[1] - n1[1] * n2[0];
            if (Math.abs(det) < 1e-12) { continue; }
            double h1 = h.get(k), h2 = h.get((k + 1) % m);
            out.add(new double[]{(h1 * n2[1] - h2 * n1[1]) / det, (n1[0] * h2 - n2[0] * h1) / det});
        }
        return SectionShape.convex(out);
    }

    private static double support(List<double[]> poly, double[] n) {
        double best = Double.NEGATIVE_INFINITY;
        for (double[] p : poly) { best = Math.max(best, p[0] * n[0] + p[1] * n[1]); }
        return best;
    }

    private static List<double[]> copy(List<double[]> poly) {
        List<double[]> out = new ArrayList<>(poly.size());
        for (double[] p : poly) { out.add(p.clone()); }
        return out;
    }
}
