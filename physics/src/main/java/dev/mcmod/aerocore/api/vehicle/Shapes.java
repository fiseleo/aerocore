package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.HullSection;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;

/** Unit-density mass distributions (kg at 1 kg/m^3, so a material's density scales them) and contact shapes of simple solids. */
final class Shapes {
    private Shapes() {}

    /** Hollow sphere of outer radius r and wall t (t >= r: solid), centred at the origin. */
    static MassProperties sphere(double r, double t) {
        double ri = Math.max(r - t, 0), outer = r * r * r, inner = ri * ri * ri;
        double m = 4.0 / 3 * Math.PI * (outer - inner), i = outer > inner ? 0.4 * m * (r * r * outer - ri * ri * inner) / (outer - inner) : 0;
        return new MassProperties(m, Vec3d.ZERO, i, i, i, 0, 0, 0);
    }

    /** Hollow box of outer size s and wall t (outer box minus inner box; t at least half a side: solid), centred at the origin. */
    static MassProperties box(Vec3d s, double t) {
        Vec3d in = new Vec3d(Math.max(s.x() - 2 * t, 0), Math.max(s.y() - 2 * t, 0), Math.max(s.z() - 2 * t, 0));
        double mo = s.x() * s.y() * s.z(), mi = in.x() * in.y() * in.z();
        double xx = (mo * (s.y() * s.y() + s.z() * s.z()) - mi * (in.y() * in.y() + in.z() * in.z())) / 12;
        double yy = (mo * (s.x() * s.x() + s.z() * s.z()) - mi * (in.x() * in.x() + in.z() * in.z())) / 12;
        double zz = (mo * (s.x() * s.x() + s.y() * s.y()) - mi * (in.x() * in.x() + in.y() * in.y())) / 12;
        return new MassProperties(mo - mi, Vec3d.ZERO, xx, yy, zz, 0, 0, 0);
    }

    /**
     * Thin shell over a hull's sections (thickness t, thin-wall approximation): each strip's section outline as point
     * masses at the edge midpoints, spread along the strip, plus the end sections as plates (transom and stem faces).
     */
    static MassProperties hullShell(SlenderHull hull, double t) {
        List<MassProperties> pieces = new ArrayList<>();
        for (int i = 0; i < hull.stations(); i++) {
            HullSection s = hull.sections().get(i);
            double dx = hull.stripLength(i);
            List<double[]> v = s.vertices();
            for (int k = 0; k < v.size(); k++) {
                double[] a = v.get(k), b = v.get((k + 1) % v.size());
                double length = Math.hypot(b[0] - a[0], b[1] - a[1]), m = t * length * dx;
                if (m <= 0) { continue; }
                // A rod along x (length dx) at the edge's midpoint.
                double rod = m * dx * dx / 12;
                pieces.add(new MassProperties(m, new Vec3d(s.xM(), 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])), 0, rod, rod, 0, 0, 0));
            }
        }
        for (HullSection end : List.of(hull.sections().getFirst(), hull.sections().getLast())) {
            double m = t * end.area();
            if (m > 0) { pieces.add(MassProperties.point(m, new Vec3d(end.xM(), centroidY(end), 0))); }
        }
        return MassProperties.combine(pieces);
    }

    private static double centroidY(HullSection s) {
        HullSection.Wet all = s.clip(1, 0, 1e9);
        return all.centroidY();
    }

    /** Spheres of radius r filling a box of size s centred at the origin (a rounded box). */
    static List<CollisionSphere> boxSpheres(Vec3d s) {
        double r = 0.5 * Math.min(s.x(), Math.min(s.y(), s.z()));
        List<CollisionSphere> out = new ArrayList<>();
        int nx = count(s.x(), r), ny = count(s.y(), r), nz = count(s.z(), r);
        for (int i = 0; i < nx; i++) for (int j = 0; j < ny; j++) for (int k = 0; k < nz; k++) {
            out.add(new CollisionSphere(new Vec3d(spot(s.x(), r, i, nx), spot(s.y(), r, j, ny), spot(s.z(), r, k, nz)), r));
        }
        return out;
    }

    /** Spheres at most one radius apart along a side (centres inset by r). */
    private static int count(double side, double r) { return Math.max(1, (int) Math.ceil((side - 2 * r) / r - 1e-9) + 1); }

    private static double spot(double side, double r, int i, int n) { return n == 1 ? 0 : -side / 2 + r + (side - 2 * r) * i / (n - 1); }
}
