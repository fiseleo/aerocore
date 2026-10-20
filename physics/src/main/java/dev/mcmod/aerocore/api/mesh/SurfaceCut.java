package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Openings cut in a gridded surface (0.30): a hull or a fuselage lofted as rings of points (stations along u, rays
 * round v), with holes given in (u, v): a window taken out, a door kept apart as a plug that can swing, a wheel arch.
 * Each cell is cut exactly: a hole is a union of convex parts, and a convex region minus a convex part is a few
 * convex pieces (inside the part's earlier edges and outside one more), so nothing is approximated in (u, v). Points
 * are placed on the cell's own bilinear facet, not on the smooth surface the rings were sampled from: a point on a
 * cell's edge then lies on the straight edge both cells share, and the cut skin stays watertight with its uncut
 * neighbours (their shared edges gain points in line with them, as with other subdivision). Corner normals and
 * colours are interpolated the same way.
 * <p>
 * Holes are worked innermost first (in the order given): a window in a door is taken out, the rest of the door is the
 * door's plug. Each hole's outline is returned as a loop on the facet, with points added where it crosses the grid,
 * so a reveal from the skin to a liner cut with the same holes ({@link #strip}) meets both edge to edge.
 * <p>
 * Pure geometry: the caller decides what is a window and draws the glass, frames and the inside.
 */
public final class SurfaceCut {
    private SurfaceCut() {}

    /** What a hole does to the skin: taken out (a window, an arch), or kept apart as a plug (a door, a canopy). */
    public enum Kind { DISCARD, PLUG }

    /**
     * A hole: an id (plugs and loops are keyed by it), its kind, its convex parts (each a polygon in (u, v), either
     * winding) and its outline (the boundary of their union, in order round it).
     */
    public record Hole(int id, Kind kind, List<double[][]> parts, double[][] outline) {
        public Hole {
            // repeated corners dropped (a part narrowing to a point at its end, as a round window's first strip): an
            // edge of no length has no inside or outside, and clipping by it would keep the whole cell twice
            List<double[][]> kept = new ArrayList<>();
            for (double[][] part : parts) { double[][] d = distinct(part); if (d.length >= 3) { kept.add(d); } }
            outline = distinct(outline);
            if (kept.isEmpty() || outline.length < 3) { throw new IllegalArgumentException("A hole needs parts and an outline"); }
            parts = List.copyOf(kept);
        }

        /** A convex hole: one part, which is also its outline. */
        public static Hole convex(int id, Kind kind, double[][] polygon) { return new Hole(id, kind, List.<double[][]>of(polygon), polygon); }
    }

    /**
     * The grid: knots u (stations) and v (rays, increasing; with periodic, v[rays] is v[0] plus one period), and per cell
     * (i between stations i and i + 1, k between rays k and k + 1) its four corners in the order (u_i, v_k),
     * (u_i, v_k+1), (u_i+1, v_k+1), (u_i+1, v_k), their normals (null: the cell's face), and a colour; a null cell is
     * absent (cut away by the caller). Knots u may repeat (a step: a cell of no width, drawn as it is). Optionally a
     * finish per cell (null, or a null entry: the pen's), so one cut can lay paint, carpet and trim.
     */
    public record Grid(double[] u, double[] v, boolean periodic, Vec3d[][][] corner, Vec3d[][][] normal, int[][] colour, Finish[][] finish) {
        public Grid(double[] u, double[] v, boolean periodic, Vec3d[][][] corner, Vec3d[][][] normal, int[][] colour) {
            this(u, v, periodic, corner, normal, colour, null);
        }

        public int cellsU() { return u.length - 1; }
        public int cellsV() { return v.length - 1; }
        double period() { return v[v.length - 1] - v[0]; }
    }

    /** A hole's outline on the facet: points in order round it, their normals, and their (u, v). */
    public record Loop(List<Vec3d> points, List<Vec3d> normals, List<double[]> uv) {}

    /** What a cut made: the plugs' quads by hole id, and each hole's outline on the facet. */
    public record Result(Map<Integer, Mesh> plugs, Map<Integer, Loop> loops) {}

    /**
     * Cuts the grid's cells with the holes into a skin (drawn into skin, with its pens) and plugs; cells no hole
     * touches are drawn as they are.
     */
    public static Result cut(Grid grid, List<Hole> holes, MeshBuilder<?> skin) {
        Map<Integer, Mesh> plugs = new LinkedHashMap<>();
        for (Hole h : holes) { if (h.kind() == Kind.PLUG) { plugs.put(h.id(), new Mesh().pens(skin.finish(), skin.tags())); } }
        double[][] box = new double[holes.size()][];
        for (int h = 0; h < holes.size(); h++) { box[h] = bounds(holes.get(h).outline()); }
        for (int i = 0; i < grid.cellsU(); i++) {
            for (int k = 0; k < grid.cellsV(); k++) {
                Vec3d[] c = grid.corner()[i][k];
                if (c == null) { continue; }
                double u0 = grid.u()[i], u1 = grid.u()[i + 1], v0 = grid.v()[k], v1 = grid.v()[k + 1];
                if (u1 - u0 < 1e-9) {
                    // two stations at one u (a step in a hull's bottom): the cell is the face between their rings, no
                    // hole can have area in it; drawn from its corners as it is
                    whole(skin, grid, i, k);
                    continue;
                }
                // the holes touching this cell, with the period shift that brings them over it
                List<Hole> here = new ArrayList<>();
                List<Double> shifts = new ArrayList<>();
                for (int h = 0; h < holes.size(); h++) {
                    for (double shift : grid.periodic() ? new double[]{0, -grid.period(), grid.period()} : new double[]{0}) {
                        double[] b = box[h];
                        if (b[0] < u1 && b[2] > u0 && b[1] + shift < v1 && b[3] + shift > v0) { here.add(holes.get(h)); shifts.add(shift); }
                    }
                }
                if (here.isEmpty()) {
                    emit(skin, grid, i, k, new double[][]{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}});
                    continue;
                }
                List<double[][]> free = new ArrayList<>();
                free.add(new double[][]{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}});
                for (int h = 0; h < here.size(); h++) {
                    Hole hole = here.get(h);
                    double shift = shifts.get(h);
                    for (double[][] part : hole.parts()) {
                        double[][] p = ccw(shifted(part, shift));
                        List<double[][]> next = new ArrayList<>();
                        for (double[][] f : free) {
                            double[][] in = clip(f, p);
                            if (in != null && hole.kind() == Kind.PLUG) { emit(plugs.get(hole.id()), grid, i, k, in); }
                            if (in == null) { next.add(f); } else { next.addAll(subtract(f, p)); }
                        }
                        free = next;
                    }
                }
                for (double[][] f : free) { emit(skin, grid, i, k, f); }
            }
        }
        Map<Integer, Loop> loops = new LinkedHashMap<>();
        for (Hole h : holes) { loops.put(h.id(), loop(grid, h.outline())); }
        return new Result(plugs, loops);
    }

    /**
     * A strip between two loops with as many points (a reveal from a skin's opening to a liner's), facing the way a
     * normal-picking function says at each point (null: as wound).
     */
    public static void strip(MeshBuilder<?> out, Loop a, Loop b, int colour, java.util.function.Function<Vec3d, Vec3d> facing) {
        int n = a.points().size();
        if (n != b.points().size()) { throw new IllegalArgumentException("Loops of " + n + " and " + b.points().size() + " points"); }
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            Vec3d p0 = a.points().get(i), p1 = a.points().get(j), q1 = b.points().get(j), q0 = b.points().get(i);
            Vec3d face = q1.subtract(p0).cross(q0.subtract(p1));
            if (face.lengthSquared() < 1e-20) { continue; }
            Vec3d want = facing == null ? face : facing.apply(p0.add(p1).add(q0).add(q1).scale(0.25));
            Vec3d n0 = face.normalized();
            if (want != null && n0.dot(want) < 0) { out.quad(p0, q0, q1, p1, new Vec3d[]{n0.scale(-1), n0.scale(-1), n0.scale(-1), n0.scale(-1)}, new int[]{colour, colour, colour, colour}); }
            else { out.quad(p0, p1, q1, q0, new Vec3d[]{n0, n0, n0, n0}, new int[]{colour, colour, colour, colour}); }
        }
    }

    // ---------------------------------------------------------------- facets

    /** A point on cell (i, k)'s bilinear facet at (u, v), its normal and colour. */
    private static Vec3d at(Grid g, int i, int k, double u, double v, Vec3d[] corner) {
        double du = g.u()[i + 1] - g.u()[i], dv = g.v()[k + 1] - g.v()[k];
        double a = du > 0 ? (u - g.u()[i]) / du : 0, b = dv > 0 ? (v - g.v()[k]) / dv : 0;
        a = Math.clamp(a, 0, 1); b = Math.clamp(b, 0, 1);
        // corners: 0 (u0, v0), 1 (u0, v1), 2 (u1, v1), 3 (u1, v0)
        return corner[0].scale((1 - a) * (1 - b)).add(corner[1].scale((1 - a) * b)).add(corner[2].scale(a * b)).add(corner[3].scale(a * (1 - b)));
    }

    private static Vec3d face(Vec3d[] c) { return c[2].subtract(c[0]).cross(c[3].subtract(c[1])); }

    /** The way a cell faces out: its corner normals' mean where given (they say which side is out), else its corners' face; null when degenerate. */
    private static Vec3d outward(Vec3d[] c, Vec3d[] normals) {
        Vec3d f = normals == null ? face(c) : normals[0].add(normals[1]).add(normals[2]).add(normals[3]);
        return f.lengthSquared() < 1e-24 ? null : f.normalized();
    }

    /** Draws a convex polygon in (u, v) on cell (i, k): fanned into quads (a last triangle as a quad with a repeated corner). */
    private static void emit(MeshBuilder<?> out, Grid g, int i, int k, double[][] polygon) {
        int n = polygon.length;
        if (n < 3 || Math.abs(area(polygon)) < 1e-18) { return; }
        Vec3d[] c = g.corner()[i][k];
        Vec3d[] cn = g.normal() == null || g.normal()[i][k] == null ? null : g.normal()[i][k];
        Vec3d facing = outward(c, cn);
        if (facing == null) { return; }
        int colour = g.colour()[i][k];
        Finish pen = out.finish(), own = g.finish() == null ? null : g.finish()[i][k];
        if (own != null) { out.finish(own); }
        Vec3d[] p = new Vec3d[n], nn = new Vec3d[n];
        for (int t = 0; t < n; t++) {
            p[t] = at(g, i, k, polygon[t][0], polygon[t][1], c);
            nn[t] = cn == null ? facing : unit(at(g, i, k, polygon[t][0], polygon[t][1], cn), facing);
        }
        for (int t = 1; t + 1 < n; t += 2) {
            int t2 = Math.min(t + 2, n - 1);
            Vec3d a = p[0], b = p[t], cc = p[t + 1], d = t2 == t + 1 ? p[t + 1] : p[t2];
            Vec3d na = nn[0], nb = nn[t], nc = nn[t + 1], nd = t2 == t + 1 ? nn[t + 1] : nn[t2];
            Vec3d tri = cc.subtract(a).cross(d.subtract(b));
            if (tri.lengthSquared() < 1e-24) { tri = b.subtract(a).cross(cc.subtract(a)); }
            int[] colours = {colour, colour, colour, colour};
            if (tri.dot(facing) >= 0) { out.quad(a, b, cc, d, new Vec3d[]{na, nb, nc, nd}, colours); }
            else { out.quad(a, d, cc, b, new Vec3d[]{na, nd, nc, nb}, colours); }
        }
        out.finish(pen);
    }

    private static Vec3d unit(Vec3d v, Vec3d fallback) { return v.lengthSquared() > 1e-24 ? v.normalized() : fallback; }

    /** A cell drawn from its four corners (one of no width in u, where the facet has no (u, v) to place points by). */
    private static void whole(MeshBuilder<?> out, Grid g, int i, int k) {
        Vec3d[] c = g.corner()[i][k];
        if (face(c).lengthSquared() < 1e-24) { return; }   // the two rings meet here: no face
        Vec3d[] cn = g.normal() == null || g.normal()[i][k] == null ? null : g.normal()[i][k];
        Vec3d facing = outward(c, cn);
        if (facing == null) { return; }
        Vec3d[] nn = cn == null ? new Vec3d[]{facing, facing, facing, facing} : cn;
        int colour = g.colour()[i][k];
        int[] colours = {colour, colour, colour, colour};
        Finish pen = out.finish(), own = g.finish() == null ? null : g.finish()[i][k];
        if (own != null) { out.finish(own); }
        if (face(c).dot(facing) >= 0) { out.quad(c[0], c[1], c[2], c[3], nn, colours); }
        else { out.quad(c[0], c[3], c[2], c[1], new Vec3d[]{nn[0], nn[3], nn[2], nn[1]}, colours); }
        out.finish(pen);
    }

    // ---------------------------------------------------------------- the outline on the facet

    private static Loop loop(Grid g, double[][] outline) {
        List<Vec3d> points = new ArrayList<>(), normals = new ArrayList<>();
        List<double[]> uv = new ArrayList<>();
        int n = outline.length;
        for (int e = 0; e < n; e++) {
            double[] a = outline[e], b = outline[(e + 1) % n];
            // the edge's crossings with grid lines, in order along it
            List<Double> ts = new ArrayList<>();
            ts.add(0.0);
            for (double gu : g.u()) { double t = (gu - a[0]) / (b[0] - a[0]); if (b[0] != a[0] && t > 1e-9 && t < 1 - 1e-9) { ts.add(t); } }
            double lo = Math.min(a[1], b[1]), hi = Math.max(a[1], b[1]);
            for (double shift : g.periodic() ? new double[]{0, -g.period(), g.period()} : new double[]{0}) {
                for (double gv : g.v()) {
                    double vv = gv - shift;
                    if (vv <= lo || vv >= hi || b[1] == a[1]) { continue; }
                    double t = (vv - a[1]) / (b[1] - a[1]);
                    if (t > 1e-9 && t < 1 - 1e-9) { ts.add(t); }
                }
            }
            ts.sort(Double::compare);
            for (double t : ts) {
                double u = a[0] + (b[0] - a[0]) * t, v = a[1] + (b[1] - a[1]) * t;
                Vec3d[] pn = onGrid(g, u, v);
                if (pn == null) { continue; }
                if (!points.isEmpty() && points.get(points.size() - 1).subtract(pn[0]).lengthSquared() < 1e-18) { continue; }
                points.add(pn[0]); normals.add(pn[1]); uv.add(new double[]{u, v});
            }
        }
        if (points.size() > 1 && points.get(0).subtract(points.get(points.size() - 1)).lengthSquared() < 1e-18) {
            points.remove(points.size() - 1); normals.remove(normals.size() - 1); uv.remove(uv.size() - 1);
        }
        return new Loop(points, normals, uv);
    }

    /** The point and normal at (u, v) on the facet of the cell holding it (v brought into the period), or null off the grid. */
    private static Vec3d[] onGrid(Grid g, double u, double v) {
        if (g.periodic()) {
            double v0 = g.v()[0], p = g.period();
            v = v0 + ((v - v0) % p + p) % p;
        }
        int i = cellOf(g.u(), u), k = cellOf(g.v(), v);
        if (i < 0 || k < 0) { return null; }
        Vec3d[] c = g.corner()[i][k];
        if (c == null) { return null; }
        Vec3d[] cn = g.normal() == null || g.normal()[i][k] == null ? null : g.normal()[i][k];
        Vec3d facing = outward(c, cn);
        if (facing == null) { facing = Vec3d.Y; }
        Vec3d n = cn == null ? facing : unit(at(g, i, k, u, v, cn), facing);
        return new Vec3d[]{at(g, i, k, u, v, c), n};
    }

    private static int cellOf(double[] knots, double x) {
        if (x < knots[0] - 1e-12 || x > knots[knots.length - 1] + 1e-12) { return -1; }
        int i = Arrays.binarySearch(knots, x);
        if (i < 0) { i = -i - 2; }
        int last = knots.length - 2;
        i = Math.clamp(i, 0, last);
        if (knots[i + 1] - knots[i] >= 1e-12) { return i; }
        // on a repeated knot (a step) the cell of no width has no facet to place by: the next one with width, else the one before
        for (int j = i + 1; j <= last; j++) { if (knots[j + 1] - knots[j] >= 1e-12) { return j; } }
        for (int j = i - 1; j >= 0; j--) { if (knots[j + 1] - knots[j] >= 1e-12) { return j; } }
        return i;
    }

    // ---------------------------------------------------------------- convex polygons in (u, v)

    /** A polygon without repeated corners (consecutive, or the last one back at the first). */
    static double[][] distinct(double[][] polygon) {
        List<double[]> out = new ArrayList<>();
        for (double[] p : polygon) {
            if (!out.isEmpty()) { double[] q = out.get(out.size() - 1); if (Math.abs(p[0] - q[0]) < 1e-12 && Math.abs(p[1] - q[1]) < 1e-12) { continue; } }
            out.add(p);
        }
        while (out.size() > 1) {
            double[] a = out.get(0), b = out.get(out.size() - 1);
            if (Math.abs(a[0] - b[0]) < 1e-12 && Math.abs(a[1] - b[1]) < 1e-12) { out.remove(out.size() - 1); } else { break; }
        }
        return out.toArray(new double[0][]);
    }

    private static double[] bounds(double[][] polygon) {
        double[] b = {Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (double[] p : polygon) { b[0] = Math.min(b[0], p[0]); b[1] = Math.min(b[1], p[1]); b[2] = Math.max(b[2], p[0]); b[3] = Math.max(b[3], p[1]); }
        return b;
    }

    private static double[][] shifted(double[][] polygon, double dv) {
        if (dv == 0) { return polygon; }
        double[][] out = new double[polygon.length][];
        for (int i = 0; i < polygon.length; i++) { out[i] = new double[]{polygon[i][0], polygon[i][1] + dv}; }
        return out;
    }

    static double area(double[][] p) {
        double s = 0;
        for (int i = 0; i < p.length; i++) { double[] a = p[i], b = p[(i + 1) % p.length]; s += a[0] * b[1] - b[0] * a[1]; }
        return 0.5 * s;
    }

    /** The polygon wound counter-clockwise in (u, v). */
    static double[][] ccw(double[][] p) {
        if (area(p) >= 0) { return p; }
        double[][] out = new double[p.length][];
        for (int i = 0; i < p.length; i++) { out[i] = p[p.length - 1 - i]; }
        return out;
    }

    /** Which side of the directed edge a to b a point lies (positive: left, inside a counter-clockwise polygon). */
    private static double side(double[] a, double[] b, double[] p) { return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]); }

    /** A convex subject clipped to one side of the line through a and b (inside: the left side; else the right). */
    private static double[][] half(double[][] subject, double[] a, double[] b, boolean inside) {
        List<double[]> out = new ArrayList<>();
        int n = subject.length;
        for (int i = 0; i < n; i++) {
            double[] p = subject[i], q = subject[(i + 1) % n];
            double sp = side(a, b, p), sq = side(a, b, q);
            if (!inside) { sp = -sp; sq = -sq; }
            if (sp >= 0) { out.add(p); }
            if ((sp > 0 && sq < 0) || (sp < 0 && sq > 0)) {
                double t = sp / (sp - sq);
                out.add(new double[]{p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t});
            }
        }
        return out.size() < 3 ? null : out.toArray(new double[0][]);
    }

    /** A convex subject inside a convex counter-clockwise clipper, or null where they do not overlap (by area). */
    static double[][] clip(double[][] subject, double[][] clipper) {
        double[][] s = subject;
        for (int e = 0; e < clipper.length && s != null; e++) { s = half(s, clipper[e], clipper[(e + 1) % clipper.length], true); }
        return s == null || Math.abs(area(s)) < 1e-14 ? null : s;
    }

    /** A convex subject minus a convex counter-clockwise clipper: disjoint convex pieces (inside the earlier edges, outside this one). */
    static List<double[][]> subtract(double[][] subject, double[][] clipper) {
        List<double[][]> out = new ArrayList<>();
        double[][] rest = subject;
        for (int e = 0; e < clipper.length && rest != null; e++) {
            double[] a = clipper[e], b = clipper[(e + 1) % clipper.length];
            double[][] outside = half(rest, a, b, false);
            if (outside != null && Math.abs(area(outside)) > 1e-14) { out.add(outside); }
            rest = half(rest, a, b, true);
        }
        return out;
    }
}
