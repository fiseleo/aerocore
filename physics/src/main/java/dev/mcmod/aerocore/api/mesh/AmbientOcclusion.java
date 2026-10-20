package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Ambient occlusion and enclosed faces for a mesh (0.29), on a voxel grid of its surfaces: each corner's share of a
 * hemisphere of rays (cosine-weighted, a fixed Hammersley set, so the result is the same every time) that hit
 * another surface within a radius darkens it, as in a crevice, a fillet or where a part sits on another; and a face
 * none of whose surroundings connect to the air outside the mesh's box (flood fill from the box's faces through
 * empty voxels) is enclosed: a piston inside a block, a blade row inside a closed casing. An opening (an intake, a
 * nozzle) lets the outside air in, so what is seen through it stays drawn. Surfaces thinner than a voxel are walls
 * either side of which counts as outside, which keeps the test on the safe side. See-through quads (glazing,
 * translucent ones; 0.30) are not on the grid and cover nothing: a cabin behind its windows is shaded and kept.
 * <p>
 * This is a display approximation: no light transport, one bounce of nothing. Cost grows with corners times rays
 * times radius over voxel; meshes are baked on a background thread.
 */
public final class AmbientOcclusion {
    private AmbientOcclusion() {}

    /**
     * How fine and how far: voxel edge (metres), most cells allowed (the voxel grows to keep within it), rays per
     * corner, the radius rays look out to, how dark full occlusion is (0-1), and the darkest a corner can be.
     */
    public record Settings(double voxelM, int maxCells, int rays, double radiusM, double strength, double floor, Refine refine) {
        public Settings {
            if (!(voxelM > 0) || maxCells < 1000 || rays < 1 || !(radiusM > 0) || !(strength >= 0 && strength <= 1) || !(floor >= 0 && floor <= 1) || refine == null) {
                throw new IllegalArgumentException("Bad ambient occlusion settings");
            }
        }

        /** As before 0.39: quads split by their corners' contrast only. */
        public Settings(double voxelM, int maxCells, int rays, double radiusM, double strength, double floor) {
            this(voxelM, maxCells, rays, radiusM, strength, floor, Refine.CORNERS);
        }

        /** These settings refined another way (0.39). */
        public Settings withRefine(Refine r) { return new Settings(voxelM, maxCells, rays, radiusM, strength, floor, r); }

        /**
         * Settings for a part of a size (its largest extent, metres): about a hundredth of it per voxel, rays out to
         * 8 % of it (3 to 30 cm: contact shading, not a sky's), at most 80 % dark and never below 35 %.
         */
        public static Settings forSize(double extentM) {
            double e = Math.max(0.05, extentM);
            return new Settings(e / 128, 3_000_000, 16, Math.clamp(0.08 * e, 0.03, 0.3), 0.8, 0.35);
        }
    }

    /**
     * Brightness 0-1 per quad corner (4 per quad, in the order of quads) and which quads are hidden (enclosed, or
     * covered by a surface just in front of them); quads are the ones given, or after {@link #refined} the given ones
     * with some split.
     */
    public record Result(List<MeshBuilder.Quad> quads, float[] corner, boolean[] enclosed) {
        public float at(int quad, int corner) { return this.corner[4 * quad + corner]; }
    }

    /** The voxel grid of a mesh's surfaces and the air connected to the outside. */
    public static final class Field {
        final double x0, y0, z0, voxel;
        final int nx, ny, nz;
        final long[] solid, outside;

        Field(List<MeshBuilder.Quad> quads, double voxelM, int maxCells) {
            Vec3d[] b = MeshBuilder.bounds(quads);
            if (b == null) { b = new Vec3d[]{Vec3d.ZERO, Vec3d.ZERO}; }
            double ex = b[1].x() - b[0].x(), ey = b[1].y() - b[0].y(), ez = b[1].z() - b[0].z();
            double v = voxelM;
            while ((ex / v + 5) * (ey / v + 5) * (ez / v + 5) > maxCells) { v *= 1.25; }
            voxel = v;
            x0 = b[0].x() - 2 * v; y0 = b[0].y() - 2 * v; z0 = b[0].z() - 2 * v;
            nx = (int) Math.ceil(ex / v) + 5; ny = (int) Math.ceil(ey / v) + 5; nz = (int) Math.ceil(ez / v) + 5;
            long cells = (long) nx * ny * nz;
            solid = new long[(int) ((cells + 63) >>> 6)];
            outside = new long[solid.length];
            for (MeshBuilder.Quad q : quads) {
                if (q.seeThrough()) { continue; }   // 0.30: glass lets the air (and the rays) through
                triangle(q.a(), q.b(), q.c()); if (!q.d().equals(q.c())) { triangle(q.a(), q.c(), q.d()); }
            }
            flood();
        }

        private void triangle(Vec3d a, Vec3d b, Vec3d c) {
            double longest = Math.max(b.subtract(a).length(), Math.max(c.subtract(b).length(), a.subtract(c).length()));
            int n = Math.max(1, (int) Math.ceil(longest / (0.5 * voxel)));
            for (int i = 0; i <= n; i++) for (int j = 0; i + j <= n; j++) {
                double s = (double) i / n, t = (double) j / n;
                double x = a.x() + (b.x() - a.x()) * s + (c.x() - a.x()) * t;
                double y = a.y() + (b.y() - a.y()) * s + (c.y() - a.y()) * t;
                double z = a.z() + (b.z() - a.z()) * s + (c.z() - a.z()) * t;
                int k = cell(x, y, z);
                if (k >= 0) { solid[k >>> 6] |= 1L << k; }
            }
        }

        int cell(double x, double y, double z) {
            int i = (int) Math.floor((x - x0) / voxel), j = (int) Math.floor((y - y0) / voxel), k = (int) Math.floor((z - z0) / voxel);
            if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) { return -1; }
            return (i * ny + j) * nz + k;
        }

        boolean solidAt(int k) { return (solid[k >>> 6] & (1L << k)) != 0; }

        boolean outsideAt(int k) { return (outside[k >>> 6] & (1L << k)) != 0; }

        /** Air reachable from the grid's faces through empty cells (6-connected). */
        private void flood() {
            int[] queue = new int[1 << 16];
            int head = 0, tail = 0;
            for (int i = 0; i < nx; i++) for (int j = 0; j < ny; j++) for (int k = 0; k < nz; k++) {
                if (i != 0 && j != 0 && k != 0 && i != nx - 1 && j != ny - 1 && k != nz - 1) { continue; }
                int c = (i * ny + j) * nz + k;
                if (solidAt(c) || outsideAt(c)) { continue; }
                outside[c >>> 6] |= 1L << c;
                if (tail == queue.length) { queue = grow(queue, head, tail); tail -= head; head = 0; }
                queue[tail++] = c;
            }
            while (head < tail) {
                int c = queue[head++];
                int i = c / (ny * nz), j = (c / nz) % ny, k = c % nz;
                for (int d = 0; d < 6; d++) {
                    int ii = i + (d == 0 ? 1 : d == 1 ? -1 : 0), jj = j + (d == 2 ? 1 : d == 3 ? -1 : 0), kk = k + (d == 4 ? 1 : d == 5 ? -1 : 0);
                    if (ii < 0 || jj < 0 || kk < 0 || ii >= nx || jj >= ny || kk >= nz) { continue; }
                    int n = (ii * ny + jj) * nz + kk;
                    if (solidAt(n) || outsideAt(n)) { continue; }
                    outside[n >>> 6] |= 1L << n;
                    if (tail == queue.length) { queue = grow(queue, head, tail); tail -= head; head = 0; }
                    queue[tail++] = n;
                }
            }
        }

        private static int[] grow(int[] q, int head, int tail) {
            int[] out = new int[Math.max(q.length, 2 * (tail - head) + 1024)];
            System.arraycopy(q, head, out, 0, tail - head);
            return out;
        }

        /** Whether outside air touches the cell holding a point or any of its 26 neighbours. */
        boolean nearOutside(Vec3d p) {
            int i = (int) Math.floor((p.x() - x0) / voxel), j = (int) Math.floor((p.y() - y0) / voxel), k = (int) Math.floor((p.z() - z0) / voxel);
            for (int di = -1; di <= 1; di++) for (int dj = -1; dj <= 1; dj++) for (int dk = -1; dk <= 1; dk++) {
                int ii = i + di, jj = j + dj, kk = k + dk;
                if (ii < 0 || jj < 0 || kk < 0 || ii >= nx || jj >= ny || kk >= nz) { return true; }
                if (outsideAt((ii * ny + jj) * nz + kk)) { return true; }
            }
            return false;
        }

        /**
         * The occluded share (0-1) of a cosine-weighted hemisphere of rays from a point about a normal, out to a
         * radius: rays start two voxels off the surface (a slanted surface fills voxels up to the diagonal, 1.73 voxels, from itself), a hit counts the more the closer it is.
         */
        public double occlusion(Vec3d p, Vec3d normal, int rays, double radius) {
            Vec3d n = MeshBuilder.unit(normal);
            Vec3d[] f = MeshBuilder.perpendiculars(n);
            Vec3d start = p.add(n.scale(SURFACE_VOXELS * voxel));
            double step = 0.5 * voxel, hits = 0;
            for (int r = 0; r < rays; r++) {
                double u = (r + 0.5) / rays, w = radicalInverse(r);
                double rad = Math.sqrt(u), phi = 2 * Math.PI * w, h = Math.sqrt(1 - u);
                Vec3d d = f[0].scale(rad * Math.cos(phi)).add(f[1].scale(rad * Math.sin(phi))).add(n.scale(h));
                for (double s = 0; s <= radius; s += step) {
                    int k = cell(start.x() + d.x() * s, start.y() + d.y() * s, start.z() + d.z() * s);
                    if (k < 0) { break; }
                    if (solidAt(k)) { double t = s / radius; hits += 1 - t * t; break; }
                }
            }
            return hits / rays;
        }

        public double voxel() { return voxel; }
    }

    private static double radicalInverse(int i) {
        int bits = Integer.reverse(i);
        return (bits >>> 1) / (double) (1L << 31) * 1.0;
    }

    /** The grid of a mesh's surfaces, to ask occlusion at any point. */
    public static Field field(List<MeshBuilder.Quad> quads, Settings s) { return new Field(quads, s.voxelM(), s.maxCells()); }

    /** Brightness per corner and enclosed quads for a mesh. Corners at the same point with the same normal are worked out once. */
    public static Result compute(List<MeshBuilder.Quad> quads, Settings s) {
        Field field = field(quads, s);
        float[] corner = new float[4 * quads.size()];
        boolean[] enclosed = new boolean[quads.size()];
        Map<Key, Float> done = new HashMap<>();
        Cover cover = new Cover(quads);
        for (int qi = 0; qi < quads.size(); qi++) {
            MeshBuilder.Quad q = quads.get(qi);
            for (int c = 0; c < 4; c++) { corner[4 * qi + c] = brightness(field, s, q.corner(c), q.normalAt(c), done); }
            enclosed[qi] = !visible(field, cover, qi, q);
        }
        return new Result(quads, corner, enclosed);
    }

    private static float brightness(Field field, Settings s, Vec3d p, Vec3d n, Map<Key, Float> done) {
        Key key = Key.of(p, n);
        Float value = done.get(key);
        if (value == null) {
            value = (float) Math.max(s.floor(), 1 - s.strength() * field.occlusion(p, n, s.rays(), s.radiusM()));
            done.put(key, value);
        }
        return value;
    }

    /** How far off a surface (voxels) rays start: past the voxels a slanted surface itself fills (up to the diagonal, 1.73). */
    static final double SURFACE_VOXELS = 2.0;

    /** Corners of one quad differing more than this get the quad split (where its edges are long enough to split). */
    public static final double SPLIT_CONTRAST = 0.12;

    /**
     * What decides a quad is split (0.39): its corners differing (CORNERS, as before), or that or its middle differing
     * from its corners' mean (CENTRE). Brightness is interpolated between corners, so a long face whose four corners
     * all sit in concave edges (a room's wall between floor and ceiling, a channel's floor) comes out as dark all over
     * as its corners under CORNERS; CENTRE sees its open middle and splits it.
     */
    public enum Refine { CORNERS, CENTRE }

    /**
     * As {@link #compute}, then quads whose corners differ much in brightness and whose edges are longer than a few
     * voxels are split into a grid of about three voxels per cell and their new corners worked out: shade from a
     * flange or a pipe then stays near it instead of spreading along a long casing panel (brightness is interpolated
     * between corners, so one dark corner of a 20 cm quad darkens all of it). Twice at most.
     */
    public static Result refined(List<MeshBuilder.Quad> quads, Settings s) { return both(quads, s).fine(); }

    /** The occlusion of a mesh as it is (coarse) and refined (fine), from one voxel grid. */
    public record Both(Result coarse, Result fine) {}

    /**
     * Both results: coarse on the quads given (for far levels of detail, where splitting would cost more than it
     * shows), fine with quads split (see refined), sharing one grid and the corners worked out.
     */
    public static Both both(List<MeshBuilder.Quad> quads, Settings s) {
        Field field = field(quads, s);
        Map<Key, Float> done = new HashMap<>();
        float[] coarseCorner = new float[4 * quads.size()];
        boolean[] coarseEnclosed = new boolean[quads.size()];
        Cover coarseCover = new Cover(quads);
        for (int qi = 0; qi < quads.size(); qi++) {
            MeshBuilder.Quad q = quads.get(qi);
            for (int c = 0; c < 4; c++) { coarseCorner[4 * qi + c] = brightness(field, s, q.corner(c), q.normalAt(c), done); }
            coarseEnclosed[qi] = !visible(field, coarseCover, qi, q);
        }
        Result coarse = new Result(quads, coarseCorner, coarseEnclosed);
        List<MeshBuilder.Quad> current = quads;
        for (int pass = 0; pass < 2; pass++) {
            List<MeshBuilder.Quad> next = new ArrayList<>(current.size());
            boolean any = false;
            double cell = 3 * field.voxel();
            for (MeshBuilder.Quad q : current) {
                float lo = 1, hi = 0, sum = 0;
                for (int c = 0; c < 4; c++) { float b = brightness(field, s, q.corner(c), q.normalAt(c), done); lo = Math.min(lo, b); hi = Math.max(hi, b); sum += b; }
                double longest = Math.max(Math.max(q.b().subtract(q.a()).length(), q.c().subtract(q.b()).length()), Math.max(q.d().subtract(q.c()).length(), q.a().subtract(q.d()).length()));
                boolean split = hi - lo > SPLIT_CONTRAST;
                if (!split && s.refine() == Refine.CENTRE && longest > 2 * cell && !q.d().equals(q.c())) {
                    // 0.39: the middle against the corners' mean
                    Vec3d n = MeshOps.faceNormal(q);
                    if (n.lengthSquared() > 0) {
                        Vec3d middle = q.a().add(q.b()).add(q.c()).add(q.d()).scale(0.25);
                        split = Math.abs(brightness(field, s, middle, n.normalized(), done) - sum / 4) > SPLIT_CONTRAST;
                    }
                }
                if (split && longest > 2 * cell && !q.d().equals(q.c())) {
                    next.addAll(MeshOps.subdivide(single(q), cell).quads());
                    any = true;
                } else {
                    next.add(q);
                }
            }
            current = next;
            if (!any) { break; }
        }
        if (current == quads) { return new Both(coarse, coarse); }
        float[] corner = new float[4 * current.size()];
        boolean[] enclosed = new boolean[current.size()];
        Cover cover = new Cover(current);
        for (int qi = 0; qi < current.size(); qi++) {
            MeshBuilder.Quad q = current.get(qi);
            for (int c = 0; c < 4; c++) { corner[4 * qi + c] = brightness(field, s, q.corner(c), q.normalAt(c), done); }
            enclosed[qi] = !visible(field, cover, qi, q);
        }
        return new Both(coarse, new Result(current, corner, enclosed));
    }

    private static Mesh single(MeshBuilder.Quad q) {
        Mesh m = new Mesh();
        m.quads().add(q);
        return m;
    }

    /**
     * Whether a quad can be seen: outside air reaches one of a few points on it (its centre, its corners pulled a
     * little inward), and it is not covered: another quad lies over all of those points within a small gap, facing
     * the same way or back at it (a pipe drawn just inside a casing wall, a liner under a shell). A covered quad
     * would otherwise be shaded black (its rays start inside the cover's voxels) and flicker against the cover where
     * depth is coarse. The cover test is exact geometry, not voxels: a manifold a few centimetres off a casing must
     * not count, however coarse the grid of a large part is.
     */
    private static boolean visible(Field field, Cover cover, int self, MeshBuilder.Quad q) {
        Vec3d centre = q.a().add(q.b()).add(q.c()).add(q.d()).scale(0.25);
        Vec3d[] points = new Vec3d[5];
        points[4] = centre;
        for (int c = 0; c < 4; c++) { points[c] = q.corner(c).add(centre.subtract(q.corner(c)).scale(0.2)); }
        boolean reached = false;
        for (Vec3d p : points) { if (field.nearOutside(p)) { reached = true; break; } }
        if (!reached) { return false; }
        Vec3d n = MeshOps.faceNormal(q);
        if (n.lengthSquared() == 0) { return true; }
        for (Vec3d p : points) { if (!cover.covers(self, p, n)) { return true; } }
        return false;
    }

    /** The largest gap (metres) between a quad and one lying over it for the quad to count as covered, for a part of an extent. */
    static double coverGap(double extentM) { return Math.clamp(0.003 * extentM, 0.001, 0.006); }

    /** Quads by the cells of a coarse grid that their boxes (grown by the gap) touch: to find a quad lying just over a point. */
    static final class Cover {
        private final List<MeshBuilder.Quad> quads;
        private final Vec3d[] normals;
        private final double gap, cell;
        private final Map<Long, int[]> cells = new HashMap<>();

        Cover(List<MeshBuilder.Quad> quads) {
            this.quads = quads;
            Vec3d[] b = MeshBuilder.bounds(quads);
            double extent = b == null ? 1 : Math.max(b[1].x() - b[0].x(), Math.max(b[1].y() - b[0].y(), b[1].z() - b[0].z()));
            gap = coverGap(extent);
            cell = Math.max(4 * gap, extent / 64);
            normals = new Vec3d[quads.size()];
            for (int i = 0; i < quads.size(); i++) {
                MeshBuilder.Quad q = quads.get(i);
                // 0.30: glass covers nothing (a window frame, a seat behind it stays drawn)
                if (q.seeThrough()) { normals[i] = Vec3d.ZERO; continue; }
                normals[i] = MeshOps.faceNormal(q);
                double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
                for (int c = 0; c < 4; c++) {
                    Vec3d v = q.corner(c);
                    lo[0] = Math.min(lo[0], v.x()); lo[1] = Math.min(lo[1], v.y()); lo[2] = Math.min(lo[2], v.z());
                    hi[0] = Math.max(hi[0], v.x()); hi[1] = Math.max(hi[1], v.y()); hi[2] = Math.max(hi[2], v.z());
                }
                int i0 = index(lo[0] - gap), i1 = index(hi[0] + gap), j0 = index(lo[1] - gap), j1 = index(hi[1] + gap), k0 = index(lo[2] - gap), k1 = index(hi[2] + gap);
                for (int x = i0; x <= i1; x++) for (int y = j0; y <= j1; y++) for (int z = k0; z <= k1; z++) {
                    cells.merge(key(x, y, z), new int[]{1, i}, (old, add) -> {
                        int[] a = old[0] + 1 < old.length ? old : java.util.Arrays.copyOf(old, 2 * old.length);
                        a[++a[0]] = add[1];
                        return a;
                    });
                }
            }
        }

        private int index(double v) { return (int) Math.floor(v / cell); }

        private static long key(int x, int y, int z) {
            return ((long) (x + (1 << 20)) << 42) | ((long) (y + (1 << 20)) << 21) | (z + (1 << 20));
        }

        /** Whether another quad, facing along or against a normal, lies over a point within the gap. */
        boolean covers(int self, Vec3d p, Vec3d n) {
            int[] list = cells.get(key(index(p.x()), index(p.y()), index(p.z())));
            if (list == null) { return false; }
            for (int at = 1; at <= list[0]; at++) {
                int k = list[at];
                if (k == self) { continue; }
                Vec3d m = normals[k];
                if (m.lengthSquared() == 0 || Math.abs(m.dot(n)) < 0.9) { continue; }
                MeshBuilder.Quad other = quads.get(k);
                double ahead = other.a().subtract(p).dot(m) / m.dot(n);   // along n from p to the other's plane
                if (!(ahead > 1e-7 && ahead <= gap)) { continue; }
                Vec3d hit = p.add(n.scale(ahead));
                if (inside(hit, other.a(), other.b(), other.c(), m) || (!other.d().equals(other.c()) && inside(hit, other.a(), other.c(), other.d(), m))) { return true; }
            }
            return false;
        }

        /** Whether a point on a triangle's plane lies inside it (a small tolerance at the edges). */
        private static boolean inside(Vec3d p, Vec3d a, Vec3d b, Vec3d c, Vec3d n) {
            double e = -1e-9;
            return b.subtract(a).cross(p.subtract(a)).dot(n) >= e * n.lengthSquared()
                    && c.subtract(b).cross(p.subtract(b)).dot(n) >= e * n.lengthSquared()
                    && a.subtract(c).cross(p.subtract(c)).dot(n) >= e * n.lengthSquared()
                    || b.subtract(a).cross(p.subtract(a)).dot(n) <= -e * n.lengthSquared()
                    && c.subtract(b).cross(p.subtract(b)).dot(n) <= -e * n.lengthSquared()
                    && a.subtract(c).cross(p.subtract(c)).dot(n) <= -e * n.lengthSquared();
        }
    }

    /** A corner: its position to 10 micrometres and its normal to a thousandth, exactly (no hashing them together). */
    private record Key(long x, long y, long z, int nx, int ny, int nz) {
        static Key of(Vec3d p, Vec3d n) {
            return new Key(Math.round(p.x() * 1e5), Math.round(p.y() * 1e5), Math.round(p.z() * 1e5),
                    (int) Math.round(n.x() * 1e3), (int) Math.round(n.y() * 1e3), (int) Math.round(n.z() * 1e3));
        }
    }
}
