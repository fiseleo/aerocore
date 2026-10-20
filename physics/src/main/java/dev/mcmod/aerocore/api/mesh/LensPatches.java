package dev.mcmod.aerocore.api.mesh;

import java.util.ArrayList;
import java.util.List;

/**
 * Where a lens-only lamp's light comes from (0.39): its lens quads (a baked mesh's groups of that lamp number) gathered
 * into patches, each a centre, a radius (of the circle of its area) and the way it faces. One lamp number may light
 * several apart (a tower's windows round its base, a car's two tail lamps): quads within {@link #MERGE_M} of a patch's
 * centre and facing much the same way join it, others start their own. For drawing a halo over each lit patch.
 */
public final class LensPatches {
    private LensPatches() {}

    /** A patch: centre, radius (metres), unit facing, in the mesh's frame. */
    public record Patch(double x, double y, double z, double radiusM, double nx, double ny, double nz) {}

    /** Quads whose centres are within this of a patch's (and face within 45 degrees of it) join that patch. */
    public static final double MERGE_M = 0.3;

    /** The patches of a lamp's lens quads in a mesh (empty when it has none). */
    public static List<Patch> of(PackedMesh mesh, int lamp) {
        float[] p = mesh.positions();
        List<double[]> acc = new ArrayList<>();   // sum of area times centre (3), area, sum of area times normal (3)
        for (PackedMesh.Group g : mesh.groups()) {
            if (g.lamp() != lamp) { continue; }
            for (int q = g.first(); q < g.first() + g.count(); q++) {
                int v = 12 * q;
                double ax = p[v], ay = p[v + 1], az = p[v + 2], bx = p[v + 3], by = p[v + 4], bz = p[v + 5];
                double cx = p[v + 6], cy = p[v + 7], cz = p[v + 8], dx = p[v + 9], dy = p[v + 10], dz = p[v + 11];
                // the quad's area normal: half the cross product of its diagonals
                double ux = cx - ax, uy = cy - ay, uz = cz - az, wx = dx - bx, wy = dy - by, wz = dz - bz;
                double nx = 0.5 * (uy * wz - uz * wy), ny = 0.5 * (uz * wx - ux * wz), nz = 0.5 * (ux * wy - uy * wx);
                double area = Math.sqrt(nx * nx + ny * ny + nz * nz);
                if (area < 1e-8) { continue; }
                // a long quad (a window strip) in pieces along its longer side, so its light comes from along it
                double abx = bx - ax, aby = by - ay, abz = bz - az, adx = dx - ax, ady = dy - ay, adz = dz - az;
                double lab = Math.sqrt(abx * abx + aby * aby + abz * abz), lad = Math.sqrt(adx * adx + ady * ady + adz * adz);
                int n = (int) Math.min(64, Math.max(1, Math.ceil(Math.max(lab, lad) / (2 * MERGE_M))));
                boolean alongAb = lab >= lad;
                for (int i = 0; i < n; i++) {
                    double t = (i + 0.5) / n;
                    double mx, my, mz;
                    if (alongAb) { mx = ax + abx * t + adx * 0.5; my = ay + aby * t + ady * 0.5; mz = az + abz * t + adz * 0.5; }
                    else { mx = ax + adx * t + abx * 0.5; my = ay + ady * t + aby * 0.5; mz = az + adz * t + abz * 0.5; }
                    add(acc, mx, my, mz, area / n, nx / n, ny / n, nz / n);
                }
            }
        }
        List<Patch> out = new ArrayList<>(acc.size());
        for (double[] a : acc) {
            double len = Math.sqrt(a[4] * a[4] + a[5] * a[5] + a[6] * a[6]);
            double nx = len < 1e-12 ? 0 : a[4] / len, ny = len < 1e-12 ? 1 : a[5] / len, nz = len < 1e-12 ? 0 : a[6] / len;
            out.add(new Patch(a[0] / a[3], a[1] / a[3], a[2] / a[3], Math.sqrt(a[3] / Math.PI), nx, ny, nz));
        }
        return out;
    }

    /** Adds a piece (centre, area, area normal) to the patch it is near and faces with, or starts one. */
    private static void add(List<double[]> acc, double mx, double my, double mz, double area, double nx, double ny, double nz) {
        double[] into = null;
        for (double[] a : acc) {
            double ex = a[0] / a[3] - mx, ey = a[1] / a[3] - my, ez = a[2] / a[3] - mz;
            double len = Math.sqrt(a[4] * a[4] + a[5] * a[5] + a[6] * a[6]);
            double cos = len < 1e-12 ? 1 : (a[4] * nx + a[5] * ny + a[6] * nz) / (len * area);
            if (ex * ex + ey * ey + ez * ez <= MERGE_M * MERGE_M && cos > 0.7) { into = a; break; }
        }
        if (into == null) { into = new double[7]; acc.add(into); }
        into[0] += area * mx; into[1] += area * my; into[2] += area * mz; into[3] += area;
        into[4] += nx; into[5] += ny; into[6] += nz;
    }
}
