package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.HullSection;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;

/**
 * Quads for drawing parts (moved from the client's PartMesh in 0.29, which now extends it): boxes, rounded boxes,
 * spheres, cylinders and cones on any axis, solids of revolution (lathe), tubes along paths (bends, coils, belts),
 * extruded outlines, lofted hulls, and meshes moved into others, collected in a part's frame. Curved surfaces carry a
 * normal per corner, so they shade smoothly; a tube may change colour along its path. Colours are ARGB; an alpha below
 * 0xff marks a quad for a translucent pass (a design screen's cutaway).
 * <p>
 * 0.29: every builder also gives its quads texture coordinates in metres (planar on flat faces, arc length round and
 * along turned and drawn surfaces, circumferences rounded to whole repeats of the finish's tile), and stamps them with
 * the builder's pens: a {@link Finish} and tags (a level of detail, interior, hot, decal), which drawing uses to pick
 * textures, drop small details far away, leave out what cannot be seen from outside, and glow by temperature.
 * Pure Java: model code builds here, and a client draws the result.
 */
public abstract class MeshBuilder<M extends MeshBuilder<M>> {
    /** Detail level in the low two bits: 0 always drawn, 1 dropped far away, 2 drawn only close, 3 only the closest. */
    public static final int DETAIL = 3;
    /** Not seen from outside the part (pistons, blade rows inside a casing): left out where a part is drawn whole. */
    public static final int INTERIOR = 4;
    /** Hot metal that glows by the part's temperature (exhaust pipes, nozzles). */
    public static final int HOT = 8;
    /** A surface lying on another (a stencil, rivet heads, a panel line): drawn with a depth offset. */
    public static final int DECAL = 16;
    /**
     * A stand-in (0.29): a coarse shape of the whole (a few dozen quads round an engine) drawn only at the farthest
     * level of a part baked with two or more levels, where it replaces everything else; never drawn otherwise.
     */
    public static final int STAND_IN = 32;
    /**
     * 0.30: seen only through openings in the part (a cabin behind its windows): never left out as enclosed at the
     * nearest level (glass leaves no path for the outside air on the voxel grid's scale), and left out of farther
     * levels (where the windows are drawn shut).
     */
    public static final int CABIN = 64;
    /**
     * 0.30: see-through (glazing): it neither shades nor hides what lies behind it, and where a mesh's cabin is left
     * out (farther levels) it is drawn opaque and darker.
     */
    public static final int CLEAR = 128;
    /** 0.30: a lamp's lens, its lamp's number in {@link #LAMP_ID}: drawn in a group of its own, lit by the part's lamp rules. */
    public static final int LAMP = 256;
    /**
     * 0.30: the nearest level a quad is drawn at (0-3, bits 9-10): with the detail level, a quad is drawn from this
     * level to 3 - detail (a far variant: a hull drawn uncut with its windows painted on, where the cut one stops).
     */
    public static final int NEAR = 3 << 9;
    /** 0.30: a lamp's number (0-254) in bits 12-19, with {@link #LAMP}. */
    public static final int LAMP_ID = 0xff << 12;
    /** 0.30: the piece a quad belongs to (0 the fixed part, 1-255 pieces that move: a door, an arm, a blade), bits 20-27. */
    public static final int PIECE = 0xff << 20;
    private static final int NEAR_SHIFT = 9, LAMP_SHIFT = 12, PIECE_SHIFT = 20;

    /**
     * One quad: corners, colour, face normal, and optionally a normal and a colour per corner (null: the face's);
     * texture coordinates in metres (u, v per corner; null: none, projected from the corners where drawn), a finish
     * (null: the drawing's default) and tags. Degenerate quads (a repeated corner) draw triangles.
     */
    public record Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal, Vec3d[] normals, int[] colors,
                       float[] uv, Finish finish, int tags) {
        public Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal, Vec3d[] normals, int[] colors) {
            this(a, b, c, d, color, normal, normals, colors, null, null, 0);
        }

        public Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal) { this(a, b, c, d, color, normal, null, null); }

        public boolean translucent() { return (color >>> 24) < 0xff; }

        public int detail() { return tags & DETAIL; }

        /** 0.30: the nearest level it is drawn at. */
        public int near() { return (tags & NEAR) >>> NEAR_SHIFT; }

        /** 0.30: its lamp's number, or -1 when it is not a lamp's lens. */
        public int lamp() { return (tags & LAMP) == 0 ? -1 : (tags & LAMP_ID) >>> LAMP_SHIFT; }

        /** 0.30: the piece it belongs to (0 the fixed part). */
        public int piece() { return (tags & PIECE) >>> PIECE_SHIFT; }

        /** 0.30: whether it lets what is behind it be seen (glazing, or any translucent quad). */
        public boolean seeThrough() { return translucent() || (tags & CLEAR) != 0; }

        public boolean has(int tag) { return (tags & tag) != 0; }

        public Vec3d corner(int i) { return switch (i) { case 0 -> a; case 1 -> b; case 2 -> c; default -> d; }; }

        public Vec3d normalAt(int i) { return normals == null ? normal : normals[i]; }

        public int colorAt(int i) { return colors == null ? color : colors[i]; }

        /** The quad turned by a rotation and moved by an offset (texture coordinates, finish and tags kept). */
        public Quad moved(Orientation rotation, Vec3d offset) {
            Vec3d[] n = null;
            if (normals != null) { n = new Vec3d[4]; for (int i = 0; i < 4; i++) { n[i] = rotation.rotate(normals[i]); } }
            return new Quad(offset.add(rotation.rotate(a)), offset.add(rotation.rotate(b)), offset.add(rotation.rotate(c)), offset.add(rotation.rotate(d)),
                    color, rotation.rotate(normal), n, colors, uv, finish, tags);
        }

        public Quad withFinish(Finish f) { return new Quad(a, b, c, d, color, normal, normals, colors, uv, f, tags); }

        public Quad withTags(int t) { return new Quad(a, b, c, d, color, normal, normals, colors, uv, finish, t); }

        public Quad withColor(int argb) { return new Quad(a, b, c, d, argb, normal, normals, colors == null ? null : tint(colors, argb, color), uv, finish, tags); }

        private static int[] tint(int[] colors, int to, int from) {
            int[] out = colors.clone();
            for (int i = 0; i < 4; i++) { if (out[i] == from) { out[i] = to; } }
            return out;
        }
    }

    private static final int AROUND = 24;
    public static final int HULL = 0xffd9d6cc, BOTTOM = 0xff9b3b30;
    private final List<Quad> quads = new ArrayList<>();
    /** 0.30: the mesh's lamps (in its frame), carried along when it is added to another, placed, mirrored or cut. */
    private final List<Lamp> lamps = new ArrayList<>();
    private Finish finish;
    private int tags;

    /** A new empty mesh of this kind (for {@link #clipped} and parts built apart and moved in). */
    protected abstract M create();

    @SuppressWarnings("unchecked")
    protected final M self() { return (M) this; }

    public List<Quad> quads() { return quads; }

    /** 0.30: the lamps (see Lamp; their lenses are the quads built with the lamp pen). */
    public List<Lamp> lamps() { return lamps; }

    /** 0.30: adds a lamp. */
    public M addLamp(Lamp lamp) { lamps.add(lamp); return self(); }

    // ---------------------------------------------------------------- pens

    /** The finish quads built from now on get (null: none, the drawing's default). */
    public M finish(Finish f) { finish = f; return self(); }

    public Finish finish() { return finish; }

    /** The detail level quads built from now on get (0 always drawn … 3 only closest). */
    public M detail(int level) { tags = (tags & ~DETAIL) | Math.max(0, Math.min(DETAIL, level)); return self(); }

    public M interior(boolean on) { return tag(INTERIOR, on); }

    public M hot(boolean on) { return tag(HOT, on); }

    public M decal(boolean on) { return tag(DECAL, on); }

    public M tag(int tag, boolean on) { tags = on ? tags | tag : tags & ~tag; return self(); }

    /** 0.30: cabin quads from now on (see {@link #CABIN}). */
    public M cabin(boolean on) { return tag(CABIN, on); }

    /** 0.30: glazing from now on (see {@link #CLEAR}). */
    public M clear(boolean on) { return tag(CLEAR, on); }

    /** 0.30: quads from now on are drawn from a level (nearest) to another (farthest), 0-3 (sets the detail level too). */
    public M levels(int nearest, int farthest) {
        int n = Math.max(0, Math.min(3, nearest)), f = Math.max(n, Math.min(3, farthest));
        tags = (tags & ~(NEAR | DETAIL)) | (n << NEAR_SHIFT) | (DETAIL - f);
        return self();
    }

    /** 0.30: quads from now on are a lamp's lens (0-254), or not (-1). */
    public M lamp(int id) {
        if (id > 254) { throw new IllegalArgumentException("At most 255 lamps in a mesh"); }
        tags = id < 0 ? tags & ~(LAMP | LAMP_ID) : (tags & ~LAMP_ID) | LAMP | (id << LAMP_SHIFT);
        return self();
    }

    /** 0.30: quads from now on belong to a piece (0 the fixed part, 1-255 moving ones). */
    public M piece(int id) {
        if (id < 0 || id > 255) { throw new IllegalArgumentException("Pieces are 0-255"); }
        tags = (tags & ~PIECE) | (id << PIECE_SHIFT);
        return self();
    }

    /** The tags quads built from now on get. */
    public int tags() { return tags; }

    /** Both pens at once (to restore them after a component). */
    public M pens(Finish f, int t) { finish = f; tags = t; return self(); }

    /** An empty mesh of this kind with the same pens. */
    protected M sub() { return create().pens(finish, tags); }

    private double tile() { return finish == null ? Finish.DEFAULT_TILE_M : finish.tileM(); }

    private void put(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal, Vec3d[] normals, int[] colors, float[] uv) {
        quads.add(new Quad(a, b, c, d, color, normal, normals, colors, uv, finish, tags));
    }

    public M quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal) { put(a, b, c, d, color, unit(normal), null, null, null); return self(); }

    /** A quad with a normal and a colour per corner (smooth surfaces built outside this class). */
    public M quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, Vec3d[] normals, int[] colors) {
        Vec3d n = unit(normals[0].add(normals[1]).add(normals[2]).add(normals[3]));
        put(a, b, c, d, colors[0], n, normals.clone(), colors.clone(), null);
        return self();
    }

    /** A quad with everything given (texture coordinates in metres, may be null), stamped with the pens. */
    public M quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal, Vec3d[] normals, int[] colors, float[] uv) {
        put(a, b, c, d, color, unit(normal), normals == null ? null : normals.clone(), colors == null ? null : colors.clone(), uv == null ? null : uv.clone());
        return self();
    }

    // ---------------------------------------------------------------- boxes and simple solids

    /** An axis-aligned box between two corners. */
    public M box(Vec3d lo, Vec3d hi, int color) {
        Vec3d[] v = {new Vec3d(lo.x(), lo.y(), lo.z()), new Vec3d(hi.x(), lo.y(), lo.z()), new Vec3d(hi.x(), hi.y(), lo.z()), new Vec3d(lo.x(), hi.y(), lo.z()),
                new Vec3d(lo.x(), lo.y(), hi.z()), new Vec3d(hi.x(), lo.y(), hi.z()), new Vec3d(hi.x(), hi.y(), hi.z()), new Vec3d(lo.x(), hi.y(), hi.z())};
        int[][] faces = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 1, 5, 4}, {3, 7, 6, 2}};
        Vec3d[] normals = {Vec3d.Z.scale(-1), Vec3d.Z, Vec3d.X.scale(-1), Vec3d.X, Vec3d.Y.scale(-1), Vec3d.Y};
        for (int f = 0; f < 6; f++) {
            Vec3d a = v[faces[f][0]], b = v[faces[f][1]], c = v[faces[f][2]], d = v[faces[f][3]];
            put(a, b, c, d, color, normals[f], null, null, planar(normals[f], a, b, c, d));
        }
        return self();
    }

    /** A box of half-sizes about a centre, turned by a rotation. */
    public M box(Vec3d centre, Vec3d half, Orientation rotation, int color) {
        return add(sub().box(half.scale(-1), half, color), rotation, centre);
    }

    /**
     * An axis-aligned box between two corners with its edges and corners rounded to a radius (a casting, a machined
     * block, a cabinet): flat faces, quarter-round edges and eighth-sphere corners of a number of segments each,
     * normals per corner. A radius of zero (or segments below 1) is a plain box.
     */
    public M roundedBox(Vec3d lo, Vec3d hi, double radius, int segments, int color) {
        double r = Math.min(radius, 0.5 * Math.min(hi.x() - lo.x(), Math.min(hi.y() - lo.y(), hi.z() - lo.z())));
        if (!(r > 1e-9) || segments < 1) { return box(lo, hi, color); }
        double[][] grid = {knots(lo.x(), hi.x(), r, segments), knots(lo.y(), hi.y(), r, segments), knots(lo.z(), hi.z(), r, segments)};
        Vec3d inLo = new Vec3d(lo.x() + r, lo.y() + r, lo.z() + r), inHi = new Vec3d(hi.x() - r, hi.y() - r, hi.z() - r);
        // each face: its axis, side, and two in-plane axes (e1 x e2 = outward normal)
        int[][] faces = {{0, 1, 1, 2}, {0, -1, 2, 1}, {1, 1, 2, 0}, {1, -1, 0, 2}, {2, 1, 0, 1}, {2, -1, 1, 0}};
        for (int[] f : faces) {
            int axis = f[0], e1 = f[2], e2 = f[3];
            double plane = f[1] > 0 ? component(hi, axis) : component(lo, axis);
            Vec3d faceNormal = axisVector(axis, f[1]);
            double[] g1 = grid[e1], g2 = grid[e2];
            Vec3d[][] p = new Vec3d[g1.length][g2.length], n = new Vec3d[g1.length][g2.length];
            for (int i = 0; i < g1.length; i++) for (int j = 0; j < g2.length; j++) {
                double[] c = new double[3];
                c[axis] = plane; c[e1] = g1[i]; c[e2] = g2[j];
                Vec3d onCube = new Vec3d(c[0], c[1], c[2]);
                Vec3d inner = new Vec3d(clamp(c[0], inLo.x(), inHi.x()), clamp(c[1], inLo.y(), inHi.y()), clamp(c[2], inLo.z(), inHi.z()));
                Vec3d dir = onCube.subtract(inner);
                Vec3d normal = dir.lengthSquared() < 1e-24 ? faceNormal : dir.normalized();
                p[i][j] = inner.add(normal.scale(r));
                n[i][j] = normal;
            }
            for (int i = 0; i + 1 < g1.length; i++) for (int j = 0; j + 1 < g2.length; j++) {
                Vec3d a = p[i][j], b = p[i + 1][j], c = p[i + 1][j + 1], d = p[i][j + 1];
                if (c.subtract(a).cross(d.subtract(b)).lengthSquared() < 1e-30) { continue; }   // a flat span of zero width
                Vec3d[] ns = {n[i][j], n[i + 1][j], n[i + 1][j + 1], n[i][j + 1]};
                Vec3d face = unit(ns[0].add(ns[1]).add(ns[2]).add(ns[3]));
                put(a, b, c, d, color, face, ns, null, planar(faceNormal, a, b, c, d));
            }
        }
        return self();
    }

    /** Grid lines along one axis of a rounded box: the rounded ends at even angles, one flat span between. */
    private static double[] knots(double lo, double hi, double r, int segments) {
        double[] out = new double[2 * (segments + 1)];
        for (int k = 0; k <= segments; k++) {
            double t = Math.tan(0.25 * Math.PI * (segments - k) / segments);
            out[k] = lo + r - r * t;
            out[2 * segments + 1 - k] = hi - r + r * t;
        }
        return out;
    }

    /**
     * A six-faced solid from eight corners (one end face 0-3, the other 4-7, corner i + 4 opposite corner i): a tapered
     * beam, a wedge, a skewed block. Faces are wound to face away from the solid's centre whichever way the corners go.
     */
    public M hexahedron(Vec3d[] c, int color) {
        int[][] faces = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
        Vec3d centre = Vec3d.ZERO;
        for (Vec3d v : c) { centre = centre.add(v.scale(0.125)); }
        for (int[] f : faces) {
            Vec3d n = c[f[1]].subtract(c[f[0]]).cross(c[f[2]].subtract(c[f[1]])).add(c[f[2]].subtract(c[f[0]]).cross(c[f[3]].subtract(c[f[2]])));
            Vec3d mid = c[f[0]].add(c[f[1]]).add(c[f[2]]).add(c[f[3]]).scale(0.25);
            if (n.dot(mid.subtract(centre)) >= 0) {
                Vec3d un = unit(n);
                put(c[f[0]], c[f[1]], c[f[2]], c[f[3]], color, un, null, null, planar(un, c[f[0]], c[f[1]], c[f[2]], c[f[3]]));
            } else {
                Vec3d un = unit(n.scale(-1));
                put(c[f[0]], c[f[3]], c[f[2]], c[f[1]], color, un, null, null, planar(un, c[f[0]], c[f[3]], c[f[2]], c[f[1]]));
            }
        }
        return self();
    }

    /**
     * A surface between two rows of points (a blade, a fin, a belt): quads between point i and i + 1 of each row;
     * two-sided adds the back face, so it shows from both sides where faces are culled. Texture u runs along row A,
     * v across to row B.
     */
    public M sheet(List<Vec3d> rowA, List<Vec3d> rowB, int color, boolean twoSided) {
        int m = Math.min(rowA.size(), rowB.size());
        double[] along = new double[Math.max(m, 1)];
        for (int i = 1; i < m; i++) { along[i] = along[i - 1] + rowA.get(i).subtract(rowA.get(i - 1)).length(); }
        for (int i = 0; i + 1 < m; i++) {
            Vec3d a = rowA.get(i), b = rowA.get(i + 1), c = rowB.get(i + 1), d = rowB.get(i);
            float u0 = (float) along[i], u1 = (float) along[i + 1], w0 = (float) d.subtract(a).length(), w1 = (float) c.subtract(b).length();
            // the diagonals' cross product (0.23: the sum of corner normals used before cancelled to zero on a flat parallelogram)
            Vec3d n = unit(c.subtract(a).cross(d.subtract(b)));
            put(a, d, c, b, color, n.scale(-1), null, null, new float[]{u0, 0, u0, w0, u1, w1, u1, 0});
            if (twoSided) { put(a, b, c, d, color, n, null, null, new float[]{u0, 0, u1, 0, u1, w1, u0, w0}); }
        }
        return self();
    }

    /** A sphere of a radius at the origin. */
    public M sphere(double r, int color) {
        int bands = 8, around = 16;
        double circumference = Finish.snapped(2 * Math.PI * r, tile());
        for (int i = 0; i < bands; i++) for (int j = 0; j < around; j++) {
            Vec3d a = polar(r, i, j, bands, around), b = polar(r, i, j + 1, bands, around), c = polar(r, i + 1, j + 1, bands, around), d = polar(r, i + 1, j, bands, around);
            float u0 = (float) (circumference * j / around), u1 = (float) (circumference * (j + 1) / around);
            float v0 = (float) (Math.PI * r * i / bands), v1 = (float) (Math.PI * r * (i + 1) / bands);
            put(a, d, c, b, color, unit(a.add(b).add(c).add(d)), new Vec3d[]{unit(a), unit(d), unit(c), unit(b)}, null,
                    new float[]{u0, v0, u0, v1, u1, v1, u1, v0});
        }
        return self();
    }

    /** A cylinder along x from x0 to x1 of a radius, ends closed. */
    public M cylinderX(double x0, double x1, double r, int color) {
        return cylinder(new Vec3d(x0, 0, 0), new Vec3d(x1, 0, 0), r, r, 16, color);
    }

    /**
     * A cylinder from one point to another, or a truncated cone when the end radii differ (a cone when one is zero),
     * ends closed; around 16 sides (a minimum of 3).
     */
    public M cylinder(Vec3d from, Vec3d to, double r0, double r1, int color) { return cylinder(from, to, r0, r1, 16, color); }

    public M cylinder(Vec3d from, Vec3d to, double r0, double r1, int sides, int color) {
        double length = to.subtract(from).length();
        if (length < 1e-9) { return self(); }
        return lathe(from, to, new double[]{0, 0, 1, 1}, new double[]{0, r0, r1, 0}, null, sides, color, true);
    }

    // ---------------------------------------------------------------- solids of revolution

    /**
     * A solid of revolution about the axis from one point to another: a profile of (fraction along the axis, radius)
     * points, swept around with smooth normals; points of radius 0 close the ends. Colours per profile point are
     * optional (null: one colour). With sharp edges, each profile segment has its own normals (crisp steps and caps);
     * without, normals are averaged across profile points (a smooth spinner). Texture u runs round (each ring's
     * circumference rounded to whole tiles), v along the profile (slant length, so end caps run radially).
     */
    public M lathe(Vec3d from, Vec3d to, double[] along, double[] radius, int[] colors, int sides, int color, boolean sharpEdges) {
        Vec3d axis = to.subtract(from);
        double length = axis.length();
        if (length < 1e-12 || along.length < 2) { return self(); }
        Vec3d u = axis.scale(1 / length);
        Vec3d[] frame = perpendiculars(u);
        int n = Math.max(3, sides), m = along.length;
        double tile = tile();
        double[] slant = new double[m], round = new double[m];
        for (int i = 0; i < m; i++) {
            round[i] = Finish.snapped(2 * Math.PI * Math.max(0, radius[i]), tile);
            if (i > 0) { slant[i] = slant[i - 1] + Math.hypot((along[i] - along[i - 1]) * length, Math.max(0, radius[i]) - Math.max(0, radius[i - 1])); }
        }
        for (int i = 0; i + 1 < m; i++) {
            double t0 = along[i], t1 = along[i + 1], q0 = Math.max(0, radius[i]), q1 = Math.max(0, radius[i + 1]);
            if (q0 == 0 && q1 == 0) { continue; }
            // profile normal of this segment in (axial, radial) components: perpendicular to the segment, pointing out
            double da = (t1 - t0) * length, dr = q1 - q0, l = Math.hypot(da, dr);
            if (l < 1e-12) { continue; }
            double na = -dr / l, nr = da / l;
            double[] n0 = {na, nr}, n1 = {na, nr};
            if (!sharpEdges) {
                n0 = profileNormal(along, radius, i, length, na, nr);
                n1 = profileNormal(along, radius, i + 1, length, na, nr);
            }
            int c0 = colors == null ? color : colors[i], c1 = colors == null ? color : colors[i + 1];
            float v0 = (float) slant[i], v1 = (float) slant[i + 1];
            for (int k = 0; k < n; k++) {
                double a0 = 2 * Math.PI * k / n, a1 = 2 * Math.PI * (k + 1) / n;
                Vec3d d0 = frame[0].scale(Math.cos(a0)).add(frame[1].scale(Math.sin(a0))), d1 = frame[0].scale(Math.cos(a1)).add(frame[1].scale(Math.sin(a1)));
                Vec3d s0 = from.add(u.scale(t0 * length)).add(d0.scale(q0)), s1 = from.add(u.scale(t0 * length)).add(d1.scale(q0));
                Vec3d e0 = from.add(u.scale(t1 * length)).add(d0.scale(q1)), e1 = from.add(u.scale(t1 * length)).add(d1.scale(q1));
                Vec3d ns0 = unit(u.scale(n0[0]).add(d0.scale(n0[1]))), ns1 = unit(u.scale(n0[0]).add(d1.scale(n0[1])));
                Vec3d ne0 = unit(u.scale(n1[0]).add(d0.scale(n1[1]))), ne1 = unit(u.scale(n1[0]).add(d1.scale(n1[1])));
                Vec3d face = unit(ns0.add(ns1).add(ne0).add(ne1));
                float us0 = (float) (round[i] * k / n), us1 = (float) (round[i] * (k + 1) / n), ue0 = (float) (round[i + 1] * k / n), ue1 = (float) (round[i + 1] * (k + 1) / n);
                put(s0, s1, e1, e0, c0, face, new Vec3d[]{ns0, ns1, ne1, ne0}, new int[]{c0, c0, c1, c1}, new float[]{us0, v0, us1, v0, ue1, v1, ue0, v1});
            }
        }
        return self();
    }

    /** The averaged profile normal at a profile point (the segment's own where the point is an end or a corner sharper than 60 degrees). */
    private static double[] profileNormal(double[] along, double[] radius, int i, double length, double na, double nr) {
        double sa = 0, sr = 0;
        for (int j = Math.max(0, i - 1); j <= Math.min(along.length - 2, i); j++) {
            double da = (along[j + 1] - along[j]) * length, dr = radius[j + 1] - radius[j], l = Math.hypot(da, dr);
            if (l < 1e-12) { continue; }
            sa += -dr / l; sr += da / l;
        }
        double l = Math.hypot(sa, sr);
        if (l < 1e-9 || (sa * na + sr * nr) / l < 0.5) { return new double[]{na, nr}; }
        return new double[]{sa / l, sr / l};
    }

    // ---------------------------------------------------------------- tubes along paths

    /**
     * A tube along a path of points (a bend, a coil, a belt), its cross-section carried along without twisting
     * (parallel transport); radii and colours per point are optional (null: the single value). Open ends are capped
     * unless the path is a closed loop. Texture u runs round, v along the path.
     */
    public M tube(List<Vec3d> path, double radius, double[] radii, int color, int[] colors, int sides, boolean loop) {
        int m = path.size();
        if (m < 2) { return self(); }
        int n = Math.max(3, sides);
        Vec3d[] tangent = new Vec3d[m];
        for (int i = 0; i < m; i++) {
            Vec3d prev = path.get(i == 0 ? (loop ? m - 1 : 0) : i - 1), next = path.get(i == m - 1 ? (loop ? 0 : m - 1) : i + 1);
            tangent[i] = unit(next.subtract(prev));
        }
        Vec3d[] p = new Vec3d[m], q = new Vec3d[m];
        Vec3d[] start = perpendiculars(tangent[0]);
        p[0] = start[0];
        for (int i = 1; i < m; i++) {
            Vec3d t = tangent[i];
            Vec3d carried = p[i - 1].subtract(t.scale(p[i - 1].dot(t)));   // project the previous frame onto the new normal plane
            p[i] = carried.lengthSquared() < 1e-12 ? perpendiculars(t)[0] : carried.normalized();
        }
        for (int i = 0; i < m; i++) { q[i] = tangent[i].cross(p[i]); }
        double tile = tile();
        double[] run = new double[m + 1], round = new double[m];
        for (int i = 1; i <= m; i++) { run[i] = run[i - 1] + path.get(i % m).subtract(path.get(i - 1)).length(); }
        for (int i = 0; i < m; i++) { round[i] = Finish.snapped(2 * Math.PI * (radii == null ? radius : radii[i]), tile); }
        int segments = loop ? m : m - 1;
        for (int i = 0; i < segments; i++) {
            int j = (i + 1) % m;
            double r0 = radii == null ? radius : radii[i], r1 = radii == null ? radius : radii[j];
            int c0 = colors == null ? color : colors[i], c1 = colors == null ? color : colors[j];
            float v0 = (float) run[i], v1 = (float) run[i + 1];
            for (int k = 0; k < n; k++) {
                double a0 = 2 * Math.PI * k / n, a1 = 2 * Math.PI * (k + 1) / n;
                Vec3d d00 = p[i].scale(Math.cos(a0)).add(q[i].scale(Math.sin(a0))), d01 = p[i].scale(Math.cos(a1)).add(q[i].scale(Math.sin(a1)));
                Vec3d d10 = p[j].scale(Math.cos(a0)).add(q[j].scale(Math.sin(a0))), d11 = p[j].scale(Math.cos(a1)).add(q[j].scale(Math.sin(a1)));
                Vec3d s0 = path.get(i).add(d00.scale(r0)), s1 = path.get(i).add(d01.scale(r0)), e0 = path.get(j).add(d10.scale(r1)), e1 = path.get(j).add(d11.scale(r1));
                float us0 = (float) (round[i] * k / n), us1 = (float) (round[i] * (k + 1) / n), ue0 = (float) (round[j] * k / n), ue1 = (float) (round[j] * (k + 1) / n);
                put(s0, s1, e1, e0, c0, unit(d00.add(d01).add(d10).add(d11)), new Vec3d[]{d00, d01, d11, d10}, new int[]{c0, c0, c1, c1},
                        new float[]{us0, v0, us1, v0, ue1, v1, ue0, v1});
            }
        }
        if (!loop) {
            for (int end = 0; end < 2; end++) {
                int i = end == 0 ? 0 : m - 1;
                double r = radii == null ? radius : radii[i];
                int c = colors == null ? color : colors[i];
                Vec3d centre = path.get(i), out = end == 0 ? tangent[i].scale(-1) : tangent[i];
                for (int k = 0; k < n; k++) {
                    double a0 = 2 * Math.PI * k / n, a1 = 2 * Math.PI * (k + 1) / n;
                    Vec3d v0 = centre.add(p[i].scale(r * Math.cos(a0))).add(q[i].scale(r * Math.sin(a0)));
                    Vec3d v1 = centre.add(p[i].scale(r * Math.cos(a1))).add(q[i].scale(r * Math.sin(a1)));
                    float x0 = (float) (r * Math.cos(a0)), y0 = (float) (r * Math.sin(a0)), x1 = (float) (r * Math.cos(a1)), y1 = (float) (r * Math.sin(a1));
                    if (end == 0) { put(centre, v1, v0, v0, c, out, null, null, new float[]{0, 0, x1, y1, x0, y0, x0, y0}); }
                    else { put(centre, v0, v1, v1, c, out, null, null, new float[]{0, 0, x0, y0, x1, y1, x1, y1}); }
                }
            }
        }
        return self();
    }

    public M tube(List<Vec3d> path, double radius, int sides, int color) { return tube(path, radius, null, color, null, sides, false); }

    /** Points on a cubic Bezier curve, both ends included. */
    public static List<Vec3d> bezier(Vec3d p0, Vec3d p1, Vec3d p2, Vec3d p3, int segments) {
        List<Vec3d> out = new ArrayList<>();
        for (int i = 0; i <= segments; i++) {
            double t = (double) i / segments, s = 1 - t;
            out.add(p0.scale(s * s * s).add(p1.scale(3 * s * s * t)).add(p2.scale(3 * s * t * t)).add(p3.scale(t * t * t)));
        }
        return out;
    }

    /** Points on a coil about an axis from a base point: a radius, a length, a number of turns. */
    public static List<Vec3d> helix(Vec3d base, Vec3d axis, double radius, double length, double turns, int perTurn) {
        Vec3d u = unit(axis);
        Vec3d[] f = perpendiculars(u);
        int count = Math.max(2, (int) Math.ceil(turns * perTurn));
        List<Vec3d> out = new ArrayList<>();
        for (int i = 0; i <= count; i++) {
            double t = (double) i / count, a = 2 * Math.PI * turns * t;
            out.add(base.add(u.scale(length * t)).add(f[0].scale(radius * Math.cos(a))).add(f[1].scale(radius * Math.sin(a))));
        }
        return out;
    }

    /** Points on a circle about an axis (not repeating the first), for closed tubes: rings, lips, a belt round one pulley. */
    public static List<Vec3d> circle(Vec3d centre, Vec3d axis, double radius, int count) {
        Vec3d[] f = perpendiculars(unit(axis));
        List<Vec3d> out = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double a = 2 * Math.PI * i / count;
            out.add(centre.add(f[0].scale(radius * Math.cos(a))).add(f[1].scale(radius * Math.sin(a))));
        }
        return out;
    }

    // ---------------------------------------------------------------- extrusions

    /**
     * An outline swept along a depth: outline points (u, v) in the plane through origin spanned by the unit vectors
     * uAxis and vAxis, extruded along depth; the caps are fanned from the outline's centroid (so the outline should be
     * star-shaped about it, as a cam lobe or a crank web is). Smooth sides average the normals at each point. Texture
     * coordinates: round the outline and along the depth on the sides, the outline's own (u, v) on the caps.
     */
    public M extrude(double[][] outline, Vec3d origin, Vec3d uAxis, Vec3d vAxis, Vec3d depth, int color, boolean smooth) {
        int m = outline.length;
        if (m < 3) { return self(); }
        Vec3d[] bottom = new Vec3d[m], top = new Vec3d[m];
        double cu = 0, cv = 0;
        for (int i = 0; i < m; i++) {
            bottom[i] = origin.add(uAxis.scale(outline[i][0])).add(vAxis.scale(outline[i][1]));
            top[i] = bottom[i].add(depth);
            cu += outline[i][0] / m; cv += outline[i][1] / m;
        }
        Vec3d along = unit(depth);
        // which way the outline turns, so side normals point out
        double area = 0;
        for (int i = 0; i < m; i++) { double[] a = outline[i], b = outline[(i + 1) % m]; area += a[0] * b[1] - b[0] * a[1]; }
        double sign = Math.signum(area) * Math.signum(uAxis.cross(vAxis).dot(along));
        if (sign == 0) { sign = 1; }
        Vec3d[] edgeNormal = new Vec3d[m];
        for (int i = 0; i < m; i++) {
            Vec3d e = bottom[(i + 1) % m].subtract(bottom[i]);
            edgeNormal[i] = unit(e.cross(along).scale(sign));
        }
        double[] perimeter = new double[m + 1];
        for (int i = 0; i < m; i++) { perimeter[i + 1] = perimeter[i] + bottom[(i + 1) % m].subtract(bottom[i]).length(); }
        float h = (float) depth.length();
        for (int i = 0; i < m; i++) {
            int j = (i + 1) % m;
            Vec3d ni = smooth ? unit(edgeNormal[i].add(edgeNormal[(i - 1 + m) % m])) : edgeNormal[i];
            Vec3d nj = smooth ? unit(edgeNormal[i].add(edgeNormal[j])) : edgeNormal[i];
            float pi = (float) perimeter[i], pj = (float) perimeter[i + 1];
            if (sign > 0) { put(bottom[i], bottom[j], top[j], top[i], color, edgeNormal[i], new Vec3d[]{ni, nj, nj, ni}, null, new float[]{pi, 0, pj, 0, pj, h, pi, h}); }
            else { put(bottom[j], bottom[i], top[i], top[j], color, edgeNormal[i], new Vec3d[]{nj, ni, ni, nj}, null, new float[]{pj, 0, pi, 0, pi, h, pj, h}); }
        }
        Vec3d c0 = origin.add(uAxis.scale(cu)).add(vAxis.scale(cv)), c1 = c0.add(depth);
        float fu = (float) cu, fv = (float) cv;
        for (int i = 0; i < m; i++) {
            int j = (i + 1) % m;
            float ui = (float) outline[i][0], vi = (float) outline[i][1], uj = (float) outline[j][0], vj = (float) outline[j][1];
            if (sign > 0) {
                put(c0, bottom[j], bottom[i], bottom[i], color, along.scale(-1), null, null, new float[]{fu, fv, uj, vj, ui, vi, ui, vi});
                put(c1, top[i], top[j], top[j], color, along, null, null, new float[]{fu, fv, ui, vi, uj, vj, uj, vj});
            } else {
                put(c0, bottom[i], bottom[j], bottom[j], color, along.scale(-1), null, null, new float[]{fu, fv, ui, vi, uj, vj, uj, vj});
                put(c1, top[j], top[i], top[i], color, along, null, null, new float[]{fu, fv, uj, vj, ui, vi, ui, vi});
            }
        }
        return self();
    }

    // ---------------------------------------------------------------- hulls

    /** Sections resampled on rays from their centroids, lofted station to station, ends closed; quads below bottomBelow (NaN: 40 % of the depth) get the bottom colour. */
    public M loft(SlenderHull hull, double bottomBelow) {
        List<Vec3d[]> rings = new ArrayList<>();
        for (HullSection s : hull.sections()) { rings.add(ring(s)); }
        double low = Double.MAX_VALUE;
        for (HullSection s : hull.sections()) { low = Math.min(low, s.minY()); }
        double line = Double.isNaN(bottomBelow) ? low + 0.4 * (hull.sections().get(hull.stations() / 2).maxY() - low) : bottomBelow + 1e-3;
        List<double[]> girths = new ArrayList<>();
        for (Vec3d[] r : rings) {
            double[] g = new double[AROUND + 1];
            for (int k = 0; k < AROUND; k++) { g[k + 1] = g[k] + r[(k + 1) % AROUND].subtract(r[k]).length(); }
            girths.add(g);
        }
        for (int i = 0; i + 1 < rings.size(); i++) {
            Vec3d[] a = rings.get(i), b = rings.get(i + 1);
            double[] ga = girths.get(i), gb = girths.get(i + 1);
            float xa = (float) a[0].x(), xb = (float) b[0].x();
            for (int k = 0; k < AROUND; k++) {
                int n = (k + 1) % AROUND;
                Vec3d normal = a[n].subtract(a[k]).cross(b[k].subtract(a[k]));
                put(a[k], a[n], b[n], b[k], 0.5 * (a[k].y() + a[n].y()) < line ? BOTTOM : HULL, unit(normal), null, null,
                        new float[]{(float) ga[k], xa, (float) ga[k + 1], xa, (float) gb[k + 1], xb, (float) gb[k], xb});
            }
        }
        for (int end = 0; end < 2; end++) {
            Vec3d[] ring = rings.get(end == 0 ? 0 : rings.size() - 1);
            Vec3d centre = Vec3d.ZERO;
            for (Vec3d v : ring) { centre = centre.add(v.scale(1.0 / AROUND)); }
            Vec3d normal = end == 0 ? Vec3d.X.scale(-1) : Vec3d.X;
            for (int k = 0; k < AROUND; k++) {
                int n = (k + 1) % AROUND;
                Vec3d first = end == 0 ? ring[n] : ring[k], second = end == 0 ? ring[k] : ring[n];
                put(centre, first, second, second, HULL, normal, null, null, planar(normal, centre, first, second, second));
            }
        }
        return self();
    }

    /** Boundary points of a convex section on rays from its centroid, from straight up turning to starboard. */
    private static Vec3d[] ring(HullSection s) {
        HullSection.Wet full = s.clip(1, 0, 1e9);
        Vec3d[] ring = new Vec3d[AROUND];
        for (int k = 0; k < AROUND; k++) {
            double angle = 2 * Math.PI * k / AROUND, dy = Math.cos(angle), dz = Math.sin(angle), t = Double.POSITIVE_INFINITY;
            List<double[]> v = s.vertices();
            for (int i = 0; i < v.size(); i++) {
                double[] p = v.get(i), q = v.get((i + 1) % v.size());
                double ey = q[0] - p[0], ez = q[1] - p[1], denominator = dy * ez - dz * ey;
                if (Math.abs(denominator) < 1e-12) { continue; }
                double oy = p[0] - full.centroidY(), oz = p[1] - full.centroidZ();
                double along = (oy * ez - oz * ey) / denominator, edge = (oy * dz - oz * dy) / denominator;
                if (along > 0 && edge >= -1e-9 && edge <= 1 + 1e-9) { t = Math.min(t, along); }
            }
            ring[k] = new Vec3d(s.xM(), full.centroidY() + (Double.isFinite(t) ? t : 0) * dy, full.centroidZ() + (Double.isFinite(t) ? t : 0) * dz);
        }
        return ring;
    }

    // ---------------------------------------------------------------- combining

    /** Another mesh's quads, turned by a rotation and moved by an offset (p to offset + rotation p), added to this one. */
    public M add(MeshBuilder<?> other, Orientation rotation, Vec3d offset) {
        for (Quad q : other.quads) { quads.add(q.moved(rotation, offset)); }
        for (Lamp l : other.lamps) { lamps.add(l.moved(rotation, offset)); }
        return self();
    }

    /**
     * This mesh cut by a plane: what lies on the side the normal points away from (where (p - point) . normal <= 0)
     * is kept, quads crossing the plane are cut along it (corner normals, colours and texture coordinates
     * interpolated); the cut itself is left open, as in a cutaway drawing.
     */
    public M clipped(Vec3d point, Vec3d normal) {
        M out = create();
        out.lamps().addAll(lamps);
        List<Quad> kept = out.quads();
        for (Quad q : quads) {
            Vec3d[] v = {q.a(), q.b(), q.c(), q.d()};
            double[] d = new double[4];
            boolean any = false, all = true;
            for (int i = 0; i < 4; i++) { d[i] = v[i].subtract(point).dot(normal); any |= d[i] <= 0; all &= d[i] <= 0; }
            if (all) { kept.add(q); continue; }
            if (!any) { continue; }
            List<Vec3d> pv = new ArrayList<>(), pn = new ArrayList<>();
            List<Integer> pc = new ArrayList<>();
            List<float[]> pt = new ArrayList<>();
            float[] uv = q.uv();
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                Vec3d ni = q.normals() == null ? q.normal() : q.normals()[i], nj = q.normals() == null ? q.normal() : q.normals()[j];
                int ci = q.colors() == null ? q.color() : q.colors()[i], cj = q.colors() == null ? q.color() : q.colors()[j];
                if (d[i] <= 0) { pv.add(v[i]); pn.add(ni); pc.add(ci); pt.add(uv == null ? null : new float[]{uv[2 * i], uv[2 * i + 1]}); }
                if ((d[i] <= 0) != (d[j] <= 0)) {
                    double t = d[i] / (d[i] - d[j]);
                    pv.add(v[i].add(v[j].subtract(v[i]).scale(t)));
                    pn.add(unit(ni.scale(1 - t).add(nj.scale(t))));
                    pc.add(lerpColor(ci, cj, t));
                    pt.add(uv == null ? null : new float[]{(float) (uv[2 * i] + (uv[2 * j] - uv[2 * i]) * t), (float) (uv[2 * i + 1] + (uv[2 * j + 1] - uv[2 * i + 1]) * t)});
                }
            }
            int n = pv.size();
            if (n < 3) { continue; }
            // a convex polygon as a fan of quads (the last of an odd count is a triangle)
            for (int i = 1; i + 1 < n; i += 2) {
                int k = Math.min(i + 2, n - 1);
                int[] idx = {0, i, i + 1, k};
                Vec3d[] corners = new Vec3d[4], normals = new Vec3d[4];
                int[] colors = new int[4];
                float[] tex = uv == null ? null : new float[8];
                for (int c = 0; c < 4; c++) {
                    corners[c] = pv.get(idx[c]); normals[c] = pn.get(idx[c]); colors[c] = pc.get(idx[c]);
                    if (tex != null) { tex[2 * c] = pt.get(idx[c])[0]; tex[2 * c + 1] = pt.get(idx[c])[1]; }
                }
                kept.add(new Quad(corners[0], corners[1], corners[2], corners[3], q.color(), q.normal(),
                        q.normals() == null ? null : normals, q.colors() == null ? null : colors, tex, q.finish(), q.tags()));
            }
        }
        return out;
    }

    static int lerpColor(int a, int b, double t) {
        int r = 0;
        for (int shift = 0; shift < 32; shift += 8) {
            int ca = (a >>> shift) & 255, cb = (b >>> shift) & 255;
            r |= ((int) Math.round(ca + (cb - ca) * t) & 255) << shift;
        }
        return r;
    }

    /** The corners of the box around every quad (min, max); null for an empty mesh. */
    public Vec3d[] bounds() { return bounds(quads); }

    public static Vec3d[] bounds(List<Quad> quads) {
        if (quads.isEmpty()) { return null; }
        double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (Quad q : quads) {
            for (int i = 0; i < 4; i++) {
                Vec3d v = q.corner(i);
                lo[0] = Math.min(lo[0], v.x()); lo[1] = Math.min(lo[1], v.y()); lo[2] = Math.min(lo[2], v.z());
                hi[0] = Math.max(hi[0], v.x()); hi[1] = Math.max(hi[1], v.y()); hi[2] = Math.max(hi[2], v.z());
            }
        }
        return new Vec3d[]{new Vec3d(lo[0], lo[1], lo[2]), new Vec3d(hi[0], hi[1], hi[2])};
    }

    // ---------------------------------------------------------------- helpers

    /**
     * Texture coordinates of four corners projected on the plane most facing a normal (metres): x and y on a face
     * looking along z, z and y along x, x and z along y, turned so a face is not seen mirrored from outside.
     */
    public static float[] planar(Vec3d normal, Vec3d a, Vec3d b, Vec3d c, Vec3d d) {
        float[] out = new float[8];
        Vec3d[] v = {a, b, c, d};
        double ax = Math.abs(normal.x()), ay = Math.abs(normal.y()), az = Math.abs(normal.z());
        for (int i = 0; i < 4; i++) {
            Vec3d p = v[i];
            double u, w;
            if (ax >= ay && ax >= az) { u = normal.x() > 0 ? -p.z() : p.z(); w = p.y(); }
            else if (az >= ay) { u = normal.z() > 0 ? p.x() : -p.x(); w = p.y(); }
            else { u = p.x(); w = normal.y() > 0 ? -p.z() : p.z(); }
            out[2 * i] = (float) u;
            out[2 * i + 1] = (float) w;
        }
        return out;
    }

    private static Vec3d polar(double r, int i, int j, int bands, int around) {
        double theta = Math.PI * i / bands, phi = 2 * Math.PI * j / around;
        return new Vec3d(r * Math.sin(theta) * Math.cos(phi), r * Math.cos(theta), r * Math.sin(theta) * Math.sin(phi));
    }

    /** Two unit vectors P, Q perpendicular to a unit vector u with P x Q = u (as Y x Z = X). */
    public static Vec3d[] perpendiculars(Vec3d u) {
        Vec3d p = unit(u.cross(Math.abs(u.x()) < 0.9 ? Vec3d.X : Vec3d.Y).cross(u));
        return new Vec3d[]{p, u.cross(p)};
    }

    public static Vec3d unit(Vec3d v) { return v.lengthSquared() < 1e-18 ? Vec3d.Y : v.normalized(); }

    private static double clamp(double v, double lo, double hi) { return v < lo ? lo : Math.min(v, hi); }

    private static double component(Vec3d v, int axis) { return axis == 0 ? v.x() : axis == 1 ? v.y() : v.z(); }

    private static Vec3d axisVector(int axis, int sign) {
        return axis == 0 ? Vec3d.X.scale(sign) : axis == 1 ? Vec3d.Y.scale(sign) : Vec3d.Z.scale(sign);
    }
}
