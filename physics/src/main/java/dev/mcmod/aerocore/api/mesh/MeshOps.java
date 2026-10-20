package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Work on meshes (0.29): smoothing normals across edges flatter than a crease angle, mirroring, placing a mesh with a
 * non-uniform scale (detail parts stretched to a design's sizes), and subdividing large quads. Each returns a new
 * {@link Mesh} and keeps colours, texture coordinates, finishes and tags.
 */
public final class MeshOps {
    private MeshOps() {}

    /** Corners closer than this (metres) are the same point for smoothing. */
    public static final double WELD_M = 1e-5;

    /** A quad's face normal from its diagonals (zero for a degenerate quad). */
    public static Vec3d faceNormal(MeshBuilder.Quad q) {
        Vec3d n = q.c().subtract(q.a()).cross(q.d().subtract(q.b()));
        // below Vec3d.normalized's own limit (1e-12 in length): a sliver quad has no normal (0.30: was 1e-30, and a
        // quad between the two threw, which left a whole part mesh empty)
        return n.lengthSquared() < 1e-24 ? Vec3d.ZERO : n.normalized();
    }

    /** A quad's area (two triangles; a degenerate quad is its triangle). */
    public static double area(MeshBuilder.Quad q) {
        return 0.5 * (q.b().subtract(q.a()).cross(q.c().subtract(q.a())).length() + q.c().subtract(q.a()).cross(q.d().subtract(q.a())).length());
    }

    /**
     * Normals per corner averaged (by area) over the quads meeting at that point whose faces turn less than a crease
     * angle from this one: curved surfaces shade smoothly, edges sharper than the angle stay crisp. Faces are
     * matched by position only, so separate parts touching at a point are smoothed together only where they are also
     * nearly parallel.
     */
    public static Mesh smooth(MeshBuilder<?> mesh, double creaseDegrees) {
        List<MeshBuilder.Quad> quads = mesh.quads();
        int n = quads.size();
        Vec3d[] face = new Vec3d[n];
        double[] weight = new double[n];
        Map<Point, List<int[]>> at = new HashMap<>();
        for (int i = 0; i < n; i++) {
            MeshBuilder.Quad q = quads.get(i);
            face[i] = faceNormal(q);
            weight[i] = area(q);
            for (int c = 0; c < 4; c++) {
                if (c == 3 && q.d().equals(q.c())) { continue; }
                at.computeIfAbsent(key(q.corner(c)), k -> new ArrayList<>()).add(new int[]{i, c});
            }
        }
        double cos = Math.cos(Math.toRadians(creaseDegrees));
        Mesh out = new Mesh();
        for (int i = 0; i < n; i++) {
            MeshBuilder.Quad q = quads.get(i);
            if (face[i] == Vec3d.ZERO) { out.quads().add(q); continue; }
            Vec3d[] normals = new Vec3d[4];
            for (int c = 0; c < 4; c++) {
                Vec3d sum = Vec3d.ZERO;
                List<int[]> shared = at.get(key(q.corner(c)));
                if (shared != null) {
                    for (int[] e : shared) {
                        if (face[e[0]] == Vec3d.ZERO || face[e[0]].dot(face[i]) < cos) { continue; }
                        sum = sum.add(face[e[0]].scale(weight[e[0]]));
                    }
                }
                normals[c] = sum.lengthSquared() < 1e-24 ? face[i] : sum.normalized();
            }
            out.quads().add(new MeshBuilder.Quad(q.a(), q.b(), q.c(), q.d(), q.color(), face[i], normals, q.colors(), q.uv(), q.finish(), q.tags()));
        }
        return out;
    }

    /** A position rounded to the weld distance, compared exactly (a spatial hash folded into one long collides). */
    private record Point(long x, long y, long z) {}

    private static Point key(Vec3d p) {
        return new Point(Math.round(p.x() / WELD_M), Math.round(p.y() / WELD_M), Math.round(p.z() / WELD_M));
    }

    /**
     * The mirror image across the plane through the origin square to an axis (0 x, 1 y, 2 z): corners and normals
     * reflected, corners taken in the opposite order so faces still face out.
     */
    public static Mesh mirror(MeshBuilder<?> mesh, int axis) {
        Mesh out = new Mesh();
        for (Lamp l : mesh.lamps()) { out.lamps().add(l.mirrored(axis)); }
        for (MeshBuilder.Quad q : mesh.quads()) {
            Vec3d[] p = new Vec3d[4], nn = null;
            int[] order = {0, 3, 2, 1};
            for (int i = 0; i < 4; i++) { p[i] = reflect(q.corner(order[i]), axis); }
            if (q.normals() != null) { nn = new Vec3d[4]; for (int i = 0; i < 4; i++) { nn[i] = reflect(q.normals()[order[i]], axis); } }
            int[] colors = null;
            if (q.colors() != null) { colors = new int[4]; for (int i = 0; i < 4; i++) { colors[i] = q.colors()[order[i]]; } }
            float[] uv = null;
            if (q.uv() != null) { uv = new float[8]; for (int i = 0; i < 4; i++) { uv[2 * i] = q.uv()[2 * order[i]]; uv[2 * i + 1] = q.uv()[2 * order[i] + 1]; } }
            out.quads().add(reordered(p, nn, colors, uv, q, reflect(q.normal(), axis)));
        }
        return out;
    }

    private static Vec3d reflect(Vec3d v, int axis) {
        return axis == 0 ? new Vec3d(-v.x(), v.y(), v.z()) : axis == 1 ? new Vec3d(v.x(), -v.y(), v.z()) : new Vec3d(v.x(), v.y(), -v.z());
    }

    /**
     * A mesh scaled along its own axes, then turned and moved (p to offset + rotation (scale p)): a unit detail part
     * stretched to the size it is placed at. Normals use the inverse scale (they stay square to the surface);
     * a negative scale on an odd number of axes reverses the corners so faces still face out; texture coordinates are
     * scaled by the mean size change so the texture keeps its scale on the part.
     */
    public static Mesh placed(MeshBuilder<?> mesh, Vec3d scale, Orientation rotation, Vec3d offset) {
        boolean flip = scale.x() * scale.y() * scale.z() < 0;
        double mean = Math.cbrt(Math.abs(scale.x() * scale.y() * scale.z()));
        Mesh out = new Mesh();
        for (Lamp l : mesh.lamps()) { out.lamps().add(l.placed(scale, rotation, offset)); }
        for (MeshBuilder.Quad q : mesh.quads()) {
            int[] order = flip ? new int[]{0, 3, 2, 1} : new int[]{0, 1, 2, 3};
            Vec3d[] p = new Vec3d[4], nn = null;
            for (int i = 0; i < 4; i++) { p[i] = offset.add(rotation.rotate(mul(q.corner(order[i]), scale))); }
            if (q.normals() != null) { nn = new Vec3d[4]; for (int i = 0; i < 4; i++) { nn[i] = rotation.rotate(MeshBuilder.unit(div(q.normals()[order[i]], scale))); } }
            int[] colors = null;
            if (q.colors() != null) { colors = new int[4]; for (int i = 0; i < 4; i++) { colors[i] = q.colors()[order[i]]; } }
            float[] uv = null;
            if (q.uv() != null) { uv = new float[8]; for (int i = 0; i < 4; i++) { uv[2 * i] = (float) (q.uv()[2 * order[i]] * mean); uv[2 * i + 1] = (float) (q.uv()[2 * order[i] + 1] * mean); } }
            Vec3d normal = rotation.rotate(MeshBuilder.unit(div(q.normal(), scale)));
            out.quads().add(reordered(p, nn, colors, uv, q, normal));
        }
        return out;
    }

    /**
     * A quad from corners taken in a new order; a degenerate quad (triangle a b c c) taken backwards is a c c b, so
     * its repeat is moved back to the end (a c b b), where the rest of the code looks for it.
     */
    private static MeshBuilder.Quad reordered(Vec3d[] p, Vec3d[] nn, int[] colors, float[] uv, MeshBuilder.Quad q, Vec3d normal) {
        if (p[1].equals(p[2]) && !p[2].equals(p[3])) {
            p = new Vec3d[]{p[0], p[2], p[3], p[3]};
            if (nn != null) { nn = new Vec3d[]{nn[0], nn[2], nn[3], nn[3]}; }
            if (colors != null) { colors = new int[]{colors[0], colors[2], colors[3], colors[3]}; }
            if (uv != null) { uv = new float[]{uv[0], uv[1], uv[4], uv[5], uv[6], uv[7], uv[6], uv[7]}; }
        }
        return new MeshBuilder.Quad(p[0], p[1], p[2], p[3], q.color(), normal, nn, colors, uv, q.finish(), q.tags());
    }

    private static Vec3d mul(Vec3d v, Vec3d s) { return new Vec3d(v.x() * s.x(), v.y() * s.y(), v.z() * s.z()); }

    private static Vec3d div(Vec3d v, Vec3d s) { return new Vec3d(v.x() / s.x(), v.y() / s.y(), v.z() / s.z()); }

    /**
     * Quads with an edge longer than a length split into a grid (bilinear, normals, colours and texture coordinates
     * interpolated), so per-corner shading such as ambient occlusion has corners to vary over. Triangles are kept.
     */
    public static Mesh subdivide(MeshBuilder<?> mesh, double maxEdgeM) {
        Mesh out = new Mesh();
        for (MeshBuilder.Quad q : mesh.quads()) {
            if (q.d().equals(q.c())) { out.quads().add(q); continue; }
            double lu = Math.max(q.b().subtract(q.a()).length(), q.c().subtract(q.d()).length());
            double lv = Math.max(q.d().subtract(q.a()).length(), q.c().subtract(q.b()).length());
            int nu = Math.max(1, (int) Math.ceil(lu / maxEdgeM)), nv = Math.max(1, (int) Math.ceil(lv / maxEdgeM));
            if (nu == 1 && nv == 1) { out.quads().add(q); continue; }
            for (int i = 0; i < nu; i++) for (int j = 0; j < nv; j++) {
                double[][] st = {{(double) i / nu, (double) j / nv}, {(double) (i + 1) / nu, (double) j / nv},
                        {(double) (i + 1) / nu, (double) (j + 1) / nv}, {(double) i / nu, (double) (j + 1) / nv}};
                Vec3d[] p = new Vec3d[4], nn = new Vec3d[4];
                int[] colors = new int[4];
                float[] uv = q.uv() == null ? null : new float[8];
                for (int c = 0; c < 4; c++) {
                    double s = st[c][0], t = st[c][1];
                    double wa = (1 - s) * (1 - t), wb = s * (1 - t), wc = s * t, wd = (1 - s) * t;
                    p[c] = q.a().scale(wa).add(q.b().scale(wb)).add(q.c().scale(wc)).add(q.d().scale(wd));
                    nn[c] = MeshBuilder.unit(q.normalAt(0).scale(wa).add(q.normalAt(1).scale(wb)).add(q.normalAt(2).scale(wc)).add(q.normalAt(3).scale(wd)));
                    colors[c] = bilerpColor(q, wa, wb, wc, wd);
                    if (uv != null) {
                        float[] u = q.uv();
                        uv[2 * c] = (float) (u[0] * wa + u[2] * wb + u[4] * wc + u[6] * wd);
                        uv[2 * c + 1] = (float) (u[1] * wa + u[3] * wb + u[5] * wc + u[7] * wd);
                    }
                }
                out.quads().add(new MeshBuilder.Quad(p[0], p[1], p[2], p[3], q.color(), q.normal(), q.normals() == null ? null : nn,
                        q.colors() == null ? null : colors, uv, q.finish(), q.tags()));
            }
        }
        return out;
    }

    private static int bilerpColor(MeshBuilder.Quad q, double wa, double wb, double wc, double wd) {
        int r = 0;
        for (int shift = 0; shift < 32; shift += 8) {
            double v = ((q.colorAt(0) >>> shift) & 255) * wa + ((q.colorAt(1) >>> shift) & 255) * wb + ((q.colorAt(2) >>> shift) & 255) * wc + ((q.colorAt(3) >>> shift) & 255) * wd;
            r |= ((int) Math.round(v) & 255) << shift;
        }
        return r;
    }

    /** Quads without a finish given one (a part's default), others unchanged. */
    public static Mesh withDefaultFinish(MeshBuilder<?> mesh, Finish finish) {
        Mesh out = new Mesh();
        for (MeshBuilder.Quad q : mesh.quads()) { out.quads().add(q.finish() == null ? q.withFinish(finish) : q); }
        return out;
    }

    /**
     * Only the quads drawn at a level of detail (0 nearest): those whose detail level plus the level is at most 3
     * (stand-ins never: MeshBaker.levels puts them at a part's farthest level).
     */
    public static Mesh atLevel(MeshBuilder<?> mesh, int level, boolean interiorToo) {
        Mesh out = new Mesh();
        for (MeshBuilder.Quad q : mesh.quads()) {
            if (q.detail() + level > MeshBuilder.DETAIL || q.has(MeshBuilder.STAND_IN)) { continue; }
            if (!interiorToo && q.has(MeshBuilder.INTERIOR)) { continue; }
            out.quads().add(q);
        }
        return out;
    }
}
