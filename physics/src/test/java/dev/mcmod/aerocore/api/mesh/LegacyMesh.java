package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.HullSection;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;

/**
 * A frozen copy of the 0.28 client PartMesh's geometry (its drawing left out), so {@code MeshCompatTest} can check
 * that the builders moved to {@link MeshBuilder} give the same corners, normals and colours. Do not change it.
 */
final class LegacyMesh {
    record Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal, Vec3d[] normals, int[] colors) {
        Quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal) { this(a, b, c, d, color, normal, null, null); }
        Quad moved(Orientation rotation, Vec3d offset) {
            Vec3d[] n = null;
            if (normals != null) { n = new Vec3d[4]; for (int i = 0; i < 4; i++) { n[i] = rotation.rotate(normals[i]); } }
            return new Quad(offset.add(rotation.rotate(a)), offset.add(rotation.rotate(b)), offset.add(rotation.rotate(c)), offset.add(rotation.rotate(d)),
                    color, rotation.rotate(normal), n, colors);
        }
    }

    private static final int AROUND = 24;
    static final int HULL = 0xffd9d6cc, BOTTOM = 0xff9b3b30;
    final List<Quad> quads = new ArrayList<>();

    LegacyMesh quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, int color, Vec3d normal) { quads.add(new Quad(a, b, c, d, color, unit(normal))); return this; }

    LegacyMesh quad(Vec3d a, Vec3d b, Vec3d c, Vec3d d, Vec3d[] normals, int[] colors) {
        Vec3d n = unit(normals[0].add(normals[1]).add(normals[2]).add(normals[3]));
        quads.add(new Quad(a, b, c, d, colors[0], n, normals.clone(), colors.clone()));
        return this;
    }

    LegacyMesh box(Vec3d lo, Vec3d hi, int color) {
        Vec3d[] v = {new Vec3d(lo.x(), lo.y(), lo.z()), new Vec3d(hi.x(), lo.y(), lo.z()), new Vec3d(hi.x(), hi.y(), lo.z()), new Vec3d(lo.x(), hi.y(), lo.z()),
                new Vec3d(lo.x(), lo.y(), hi.z()), new Vec3d(hi.x(), lo.y(), hi.z()), new Vec3d(hi.x(), hi.y(), hi.z()), new Vec3d(lo.x(), hi.y(), hi.z())};
        int[][] faces = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 1, 5, 4}, {3, 7, 6, 2}};
        Vec3d[] normals = {Vec3d.Z.scale(-1), Vec3d.Z, Vec3d.X.scale(-1), Vec3d.X, Vec3d.Y.scale(-1), Vec3d.Y};
        for (int f = 0; f < 6; f++) { quads.add(new Quad(v[faces[f][0]], v[faces[f][1]], v[faces[f][2]], v[faces[f][3]], color, normals[f])); }
        return this;
    }

    LegacyMesh box(Vec3d centre, Vec3d half, Orientation rotation, int color) {
        return add(new LegacyMesh().box(half.scale(-1), half, color), rotation, centre);
    }

    LegacyMesh hexahedron(Vec3d[] c, int color) {
        int[][] faces = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
        Vec3d centre = Vec3d.ZERO;
        for (Vec3d v : c) { centre = centre.add(v.scale(0.125)); }
        for (int[] f : faces) {
            Vec3d n = c[f[1]].subtract(c[f[0]]).cross(c[f[2]].subtract(c[f[1]])).add(c[f[2]].subtract(c[f[0]]).cross(c[f[3]].subtract(c[f[2]])));
            Vec3d mid = c[f[0]].add(c[f[1]]).add(c[f[2]]).add(c[f[3]]).scale(0.25);
            if (n.dot(mid.subtract(centre)) >= 0) { quads.add(new Quad(c[f[0]], c[f[1]], c[f[2]], c[f[3]], color, unit(n))); }
            else { quads.add(new Quad(c[f[0]], c[f[3]], c[f[2]], c[f[1]], color, unit(n.scale(-1)))); }
        }
        return this;
    }

    LegacyMesh sheet(List<Vec3d> rowA, List<Vec3d> rowB, int color, boolean twoSided) {
        int m = Math.min(rowA.size(), rowB.size());
        for (int i = 0; i + 1 < m; i++) {
            Vec3d a = rowA.get(i), b = rowA.get(i + 1), c = rowB.get(i + 1), d = rowB.get(i);
            Vec3d n = unit(c.subtract(a).cross(d.subtract(b)));
            quads.add(new Quad(a, d, c, b, color, n.scale(-1)));
            if (twoSided) { quads.add(new Quad(a, b, c, d, color, n)); }
        }
        return this;
    }

    LegacyMesh sphere(double r, int color) {
        int bands = 8, around = 16;
        for (int i = 0; i < bands; i++) for (int j = 0; j < around; j++) {
            Vec3d a = polar(r, i, j, bands, around), b = polar(r, i, j + 1, bands, around), c = polar(r, i + 1, j + 1, bands, around), d = polar(r, i + 1, j, bands, around);
            quads.add(new Quad(a, d, c, b, color, unit(a.add(b).add(c).add(d)), new Vec3d[]{unit(a), unit(d), unit(c), unit(b)}, null));
        }
        return this;
    }

    LegacyMesh cylinderX(double x0, double x1, double r, int color) {
        return cylinder(new Vec3d(x0, 0, 0), new Vec3d(x1, 0, 0), r, r, 16, color);
    }

    LegacyMesh cylinder(Vec3d from, Vec3d to, double r0, double r1, int sides, int color) {
        double length = to.subtract(from).length();
        if (length < 1e-9) { return this; }
        return lathe(from, to, new double[]{0, 0, 1, 1}, new double[]{0, r0, r1, 0}, null, sides, color, true);
    }

    LegacyMesh lathe(Vec3d from, Vec3d to, double[] along, double[] radius, int[] colors, int sides, int color, boolean sharpEdges) {
        Vec3d axis = to.subtract(from);
        double length = axis.length();
        if (length < 1e-12 || along.length < 2) { return this; }
        Vec3d u = axis.scale(1 / length);
        Vec3d[] frame = perpendiculars(u);
        int n = Math.max(3, sides), m = along.length;
        for (int i = 0; i + 1 < m; i++) {
            double t0 = along[i], t1 = along[i + 1], q0 = Math.max(0, radius[i]), q1 = Math.max(0, radius[i + 1]);
            if (q0 == 0 && q1 == 0) { continue; }
            double da = (t1 - t0) * length, dr = q1 - q0, l = Math.hypot(da, dr);
            if (l < 1e-12) { continue; }
            double na = -dr / l, nr = da / l;
            double[] n0 = {na, nr}, n1 = {na, nr};
            if (!sharpEdges) {
                n0 = profileNormal(along, radius, i, length, na, nr);
                n1 = profileNormal(along, radius, i + 1, length, na, nr);
            }
            int c0 = colors == null ? color : colors[i], c1 = colors == null ? color : colors[i + 1];
            for (int k = 0; k < n; k++) {
                double a0 = 2 * Math.PI * k / n, a1 = 2 * Math.PI * (k + 1) / n;
                Vec3d d0 = frame[0].scale(Math.cos(a0)).add(frame[1].scale(Math.sin(a0))), d1 = frame[0].scale(Math.cos(a1)).add(frame[1].scale(Math.sin(a1)));
                Vec3d s0 = from.add(u.scale(t0 * length)).add(d0.scale(q0)), s1 = from.add(u.scale(t0 * length)).add(d1.scale(q0));
                Vec3d e0 = from.add(u.scale(t1 * length)).add(d0.scale(q1)), e1 = from.add(u.scale(t1 * length)).add(d1.scale(q1));
                Vec3d ns0 = unit(u.scale(n0[0]).add(d0.scale(n0[1]))), ns1 = unit(u.scale(n0[0]).add(d1.scale(n0[1])));
                Vec3d ne0 = unit(u.scale(n1[0]).add(d0.scale(n1[1]))), ne1 = unit(u.scale(n1[0]).add(d1.scale(n1[1])));
                Vec3d face = unit(ns0.add(ns1).add(ne0).add(ne1));
                quads.add(new Quad(s0, s1, e1, e0, c0, face, new Vec3d[]{ns0, ns1, ne1, ne0}, new int[]{c0, c0, c1, c1}));
            }
        }
        return this;
    }

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

    LegacyMesh tube(List<Vec3d> path, double radius, double[] radii, int color, int[] colors, int sides, boolean loop) {
        int m = path.size();
        if (m < 2) { return this; }
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
            Vec3d carried = p[i - 1].subtract(t.scale(p[i - 1].dot(t)));
            p[i] = carried.lengthSquared() < 1e-12 ? perpendiculars(t)[0] : carried.normalized();
        }
        for (int i = 0; i < m; i++) { q[i] = tangent[i].cross(p[i]); }
        int segments = loop ? m : m - 1;
        for (int i = 0; i < segments; i++) {
            int j = (i + 1) % m;
            double r0 = radii == null ? radius : radii[i], r1 = radii == null ? radius : radii[j];
            int c0 = colors == null ? color : colors[i], c1 = colors == null ? color : colors[j];
            for (int k = 0; k < n; k++) {
                double a0 = 2 * Math.PI * k / n, a1 = 2 * Math.PI * (k + 1) / n;
                Vec3d d00 = p[i].scale(Math.cos(a0)).add(q[i].scale(Math.sin(a0))), d01 = p[i].scale(Math.cos(a1)).add(q[i].scale(Math.sin(a1)));
                Vec3d d10 = p[j].scale(Math.cos(a0)).add(q[j].scale(Math.sin(a0))), d11 = p[j].scale(Math.cos(a1)).add(q[j].scale(Math.sin(a1)));
                Vec3d s0 = path.get(i).add(d00.scale(r0)), s1 = path.get(i).add(d01.scale(r0)), e0 = path.get(j).add(d10.scale(r1)), e1 = path.get(j).add(d11.scale(r1));
                quads.add(new Quad(s0, s1, e1, e0, c0, unit(d00.add(d01).add(d10).add(d11)), new Vec3d[]{d00, d01, d11, d10}, new int[]{c0, c0, c1, c1}));
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
                    quads.add(end == 0 ? new Quad(centre, v1, v0, v0, c, out) : new Quad(centre, v0, v1, v1, c, out));
                }
            }
        }
        return this;
    }

    LegacyMesh extrude(double[][] outline, Vec3d origin, Vec3d uAxis, Vec3d vAxis, Vec3d depth, int color, boolean smooth) {
        int m = outline.length;
        if (m < 3) { return this; }
        Vec3d[] bottom = new Vec3d[m], top = new Vec3d[m];
        double cu = 0, cv = 0;
        for (int i = 0; i < m; i++) {
            bottom[i] = origin.add(uAxis.scale(outline[i][0])).add(vAxis.scale(outline[i][1]));
            top[i] = bottom[i].add(depth);
            cu += outline[i][0] / m; cv += outline[i][1] / m;
        }
        Vec3d along = unit(depth);
        double area = 0;
        for (int i = 0; i < m; i++) { double[] a = outline[i], b = outline[(i + 1) % m]; area += a[0] * b[1] - b[0] * a[1]; }
        double sign = Math.signum(area) * Math.signum(uAxis.cross(vAxis).dot(along));
        if (sign == 0) { sign = 1; }
        Vec3d[] edgeNormal = new Vec3d[m];
        for (int i = 0; i < m; i++) {
            Vec3d e = bottom[(i + 1) % m].subtract(bottom[i]);
            edgeNormal[i] = unit(e.cross(along).scale(sign));
        }
        for (int i = 0; i < m; i++) {
            int j = (i + 1) % m;
            Vec3d ni = smooth ? unit(edgeNormal[i].add(edgeNormal[(i - 1 + m) % m])) : edgeNormal[i];
            Vec3d nj = smooth ? unit(edgeNormal[i].add(edgeNormal[j])) : edgeNormal[i];
            quads.add(sign > 0 ? new Quad(bottom[i], bottom[j], top[j], top[i], color, edgeNormal[i], new Vec3d[]{ni, nj, nj, ni}, null)
                    : new Quad(bottom[j], bottom[i], top[i], top[j], color, edgeNormal[i], new Vec3d[]{nj, ni, ni, nj}, null));
        }
        Vec3d c0 = origin.add(uAxis.scale(cu)).add(vAxis.scale(cv)), c1 = c0.add(depth);
        for (int i = 0; i < m; i++) {
            int j = (i + 1) % m;
            if (sign > 0) {
                quads.add(new Quad(c0, bottom[j], bottom[i], bottom[i], color, along.scale(-1)));
                quads.add(new Quad(c1, top[i], top[j], top[j], color, along));
            } else {
                quads.add(new Quad(c0, bottom[i], bottom[j], bottom[j], color, along.scale(-1)));
                quads.add(new Quad(c1, top[j], top[i], top[i], color, along));
            }
        }
        return this;
    }

    LegacyMesh loft(SlenderHull hull, double bottomBelow) {
        List<Vec3d[]> rings = new ArrayList<>();
        for (HullSection s : hull.sections()) { rings.add(ring(s)); }
        double low = Double.MAX_VALUE;
        for (HullSection s : hull.sections()) { low = Math.min(low, s.minY()); }
        double line = Double.isNaN(bottomBelow) ? low + 0.4 * (hull.sections().get(hull.stations() / 2).maxY() - low) : bottomBelow + 1e-3;
        for (int i = 0; i + 1 < rings.size(); i++) {
            Vec3d[] a = rings.get(i), b = rings.get(i + 1);
            for (int k = 0; k < AROUND; k++) {
                int n = (k + 1) % AROUND;
                Vec3d normal = a[n].subtract(a[k]).cross(b[k].subtract(a[k]));
                quads.add(new Quad(a[k], a[n], b[n], b[k], 0.5 * (a[k].y() + a[n].y()) < line ? BOTTOM : HULL, unit(normal)));
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
                quads.add(new Quad(centre, first, second, second, HULL, normal));
            }
        }
        return this;
    }

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

    LegacyMesh add(LegacyMesh other, Orientation rotation, Vec3d offset) {
        for (Quad q : other.quads) { quads.add(q.moved(rotation, offset)); }
        return this;
    }

    LegacyMesh clipped(Vec3d point, Vec3d normal) {
        LegacyMesh out = new LegacyMesh();
        for (Quad q : quads) {
            Vec3d[] v = {q.a(), q.b(), q.c(), q.d()};
            double[] d = new double[4];
            boolean any = false, all = true;
            for (int i = 0; i < 4; i++) { d[i] = v[i].subtract(point).dot(normal); any |= d[i] <= 0; all &= d[i] <= 0; }
            if (all) { out.quads.add(q); continue; }
            if (!any) { continue; }
            List<Vec3d> pv = new ArrayList<>(), pn = new ArrayList<>();
            List<Integer> pc = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                Vec3d ni = q.normals() == null ? q.normal() : q.normals()[i], nj = q.normals() == null ? q.normal() : q.normals()[j];
                int ci = q.colors() == null ? q.color() : q.colors()[i], cj = q.colors() == null ? q.color() : q.colors()[j];
                if (d[i] <= 0) { pv.add(v[i]); pn.add(ni); pc.add(ci); }
                if ((d[i] <= 0) != (d[j] <= 0)) {
                    double t = d[i] / (d[i] - d[j]);
                    pv.add(v[i].add(v[j].subtract(v[i]).scale(t)));
                    pn.add(unit(ni.scale(1 - t).add(nj.scale(t))));
                    pc.add(MeshBuilder.lerpColor(ci, cj, t));
                }
            }
            int n = pv.size();
            if (n < 3) { continue; }
            for (int i = 1; i + 1 < n; i += 2) {
                int k = Math.min(i + 2, n - 1);
                Vec3d[] corners = {pv.get(0), pv.get(i), pv.get(i + 1), pv.get(k)};
                Vec3d[] normals = {pn.get(0), pn.get(i), pn.get(i + 1), pn.get(k)};
                int[] colors = {pc.get(0), pc.get(i), pc.get(i + 1), pc.get(k)};
                out.quads.add(new Quad(corners[0], corners[1], corners[2], corners[3], q.color(), q.normal(),
                        q.normals() == null ? null : normals, q.colors() == null ? null : colors));
            }
        }
        return out;
    }

    private static Vec3d polar(double r, int i, int j, int bands, int around) {
        double theta = Math.PI * i / bands, phi = 2 * Math.PI * j / around;
        return new Vec3d(r * Math.sin(theta) * Math.cos(phi), r * Math.cos(theta), r * Math.sin(theta) * Math.sin(phi));
    }

    static Vec3d[] perpendiculars(Vec3d u) {
        Vec3d p = unit(u.cross(Math.abs(u.x()) < 0.9 ? Vec3d.X : Vec3d.Y).cross(u));
        return new Vec3d[]{p, u.cross(p)};
    }

    static Vec3d unit(Vec3d v) { return v.lengthSquared() < 1e-18 ? Vec3d.Y : v.normalized(); }
}
