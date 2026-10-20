package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** 0.30: openings cut in a gridded surface (a unit cylinder: its cells are flat, so areas add up exactly). */
class SurfaceCutTest {
    private static final int STATIONS = 6, RAYS = 16, PAINT = 0xffc0c0c0;

    /** A unit cylinder along x from 0 to 2: rings at u = 0, 0.4 … 2, rays at v = 2 pi k / 16, normals outward. */
    private static SurfaceCut.Grid cylinder() {
        double[] u = new double[STATIONS], v = new double[RAYS + 1];
        for (int i = 0; i < STATIONS; i++) { u[i] = 2.0 * i / (STATIONS - 1); }
        for (int k = 0; k <= RAYS; k++) { v[k] = 2 * Math.PI * k / RAYS; }
        Vec3d[][][] corner = new Vec3d[STATIONS - 1][RAYS][], normal = new Vec3d[STATIONS - 1][RAYS][];
        int[][] colour = new int[STATIONS - 1][RAYS];
        for (int i = 0; i < STATIONS - 1; i++) {
            for (int k = 0; k < RAYS; k++) {
                corner[i][k] = new Vec3d[]{p(u[i], v[k]), p(u[i], v[k + 1]), p(u[i + 1], v[k + 1]), p(u[i + 1], v[k])};
                normal[i][k] = new Vec3d[]{n(v[k]), n(v[k + 1]), n(v[k + 1]), n(v[k])};
                colour[i][k] = PAINT;
            }
        }
        return new SurfaceCut.Grid(u, v, true, corner, normal, colour);
    }

    private static Vec3d p(double u, double v) { return new Vec3d(u, Math.cos(v), Math.sin(v)); }
    private static Vec3d n(double v) { return new Vec3d(0, Math.cos(v), Math.sin(v)); }

    private static double area(MeshBuilder<?> m) {
        double a = 0;
        for (MeshBuilder.Quad q : m.quads()) {
            a += 0.5 * q.b().subtract(q.a()).cross(q.c().subtract(q.a())).length();
            if (!q.d().equals(q.c())) { a += 0.5 * q.c().subtract(q.a()).cross(q.d().subtract(q.a())).length(); }
        }
        return a;
    }

    private static double[][] rect(double u0, double u1, double v0, double v1) { return new double[][]{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}; }

    @Test void skinAndPlugsMakeUpTheWholeSurface() {
        SurfaceCut.Grid g = cylinder();
        Mesh whole = new Mesh();
        SurfaceCut.cut(g, List.of(), whole);
        double total = area(whole);
        // every hole a plug: nothing is lost (one across the seam at v = 0, one over several cells, one inside a cell)
        List<SurfaceCut.Hole> holes = List.of(
                SurfaceCut.Hole.convex(1, SurfaceCut.Kind.PLUG, rect(0.3, 0.9, -0.5, 0.45)),
                SurfaceCut.Hole.convex(2, SurfaceCut.Kind.PLUG, new double[][]{{1.0, 2.0}, {1.7, 2.2}, {1.5, 3.1}, {1.1, 2.9}}),
                SurfaceCut.Hole.convex(3, SurfaceCut.Kind.PLUG, rect(0.05, 0.15, 4.0, 4.1)));
        Mesh skin = new Mesh();
        SurfaceCut.Result r = SurfaceCut.cut(g, holes, skin);
        double sum = area(skin);
        for (Mesh plug : r.plugs().values()) { sum += area(plug); }
        assertEquals(total, sum, 1e-9 * total);
        assertTrue(area(r.plugs().get(1)) > 0 && area(r.plugs().get(2)) > 0 && area(r.plugs().get(3)) > 0);
    }

    @Test void aStepOfNoLengthIsKeptAndARoundEndCutsCleanly() {
        // a step: stations 2 and 3 at one u, the ring at 3 a smaller cylinder (radius 0.8): the cell between is the
        // annulus face, which a cut must keep as it is
        SurfaceCut.Grid g = cylinder();
        double[] u = g.u().clone();
        for (int i = 3; i < u.length; i++) { u[i] = u[i - 1] + (i == 3 ? 0 : 0.4); }
        Vec3d[][][] corner = new Vec3d[STATIONS - 1][RAYS][];
        for (int i = 0; i < STATIONS - 1; i++) {
            for (int k = 0; k < RAYS; k++) {
                double r0 = i >= 3 ? 0.8 : 1, r1 = i + 1 >= 3 ? 0.8 : 1, va = g.v()[k], vb = g.v()[k + 1];
                corner[i][k] = new Vec3d[]{ring(u[i], va, r0), ring(u[i], vb, r0), ring(u[i + 1], vb, r1), ring(u[i + 1], va, r1)};
            }
        }
        SurfaceCut.Grid stepped = new SurfaceCut.Grid(u, g.v(), true, corner, g.normal(), g.colour());
        Mesh whole = new Mesh();
        SurfaceCut.cut(stepped, List.of(), whole);
        // the annulus between radius 0.8 and 1 (as 16 flat trapezoids)
        double annulus = RAYS * 0.5 * Math.sin(2 * Math.PI / RAYS) * (1 - 0.64);
        Mesh rest = new Mesh();
        SurfaceCut.cut(stepped, List.of(SurfaceCut.Hole.convex(1, SurfaceCut.Kind.DISCARD, rect(0.1, 0.3, 1, 2))), rest);
        double step = 0;
        for (MeshBuilder.Quad q : whole.quads()) { if (Math.abs(q.a().x() - u[2]) < 1e-12 && Math.abs(q.c().x() - u[2]) < 1e-12) { step += 0.5 * q.b().subtract(q.a()).cross(q.c().subtract(q.a())).length() + 0.5 * q.c().subtract(q.a()).cross(q.d().subtract(q.a())).length(); } }
        assertEquals(annulus, step, 1e-9, "the step's face is drawn");
        // an opening ending on the step's u: its outline is placed on a cell with width (finite points)
        SurfaceCut.Result onStep = SurfaceCut.cut(stepped, List.of(SurfaceCut.Hole.convex(2, SurfaceCut.Kind.DISCARD, rect(0.5, 0.8, 1, 2))), new Mesh());
        for (Vec3d q : onStep.loops().get(2).points()) { assertTrue(Double.isFinite(q.x() + q.y() + q.z()), "a finite outline point " + q); }
        // a finish per cell: the cells of the first station strip carpet, the rest the pen's
        Finish carpet = new Finish("test:carpet", 0.5), paint = new Finish("test:paint", 1);
        Finish[][] finishes = new Finish[STATIONS - 1][RAYS];
        for (int k = 0; k < RAYS; k++) { finishes[0][k] = carpet; }
        Mesh laid = new Mesh().finish(paint);
        SurfaceCut.cut(new SurfaceCut.Grid(g.u(), g.v(), true, g.corner(), g.normal(), g.colour(), finishes), List.of(SurfaceCut.Hole.convex(3, SurfaceCut.Kind.DISCARD, rect(0.1, 0.6, 1, 2))), laid);
        for (MeshBuilder.Quad q : laid.quads()) {
            double x = (q.a().x() + q.b().x() + q.c().x() + q.d().x()) / 4;
            assertEquals(x < 0.4 ? carpet : paint, q.finish(), "the cell's finish at x " + x);
        }
        assertEquals(paint, laid.finish(), "the pen is put back");
        // a hole narrowing to a point at each end (a round window cut as strips) given with its repeated corners
        double[][] lens = {{0.5, 1.5}, {0.5, 1.5}, {0.7, 1.3}, {0.7, 1.7}};
        double[][] lens2 = {{0.7, 1.3}, {0.9, 1.5}, {0.9, 1.5}, {0.7, 1.7}};
        Mesh skin = new Mesh();
        SurfaceCut.Result r = SurfaceCut.cut(g, List.of(new SurfaceCut.Hole(1, SurfaceCut.Kind.PLUG, List.of(lens, lens2),
                new double[][]{{0.5, 1.5}, {0.7, 1.3}, {0.9, 1.5}, {0.9, 1.5}, {0.7, 1.7}})), skin);
        Mesh full = new Mesh();
        SurfaceCut.cut(g, List.of(), full);
        assertEquals(area(full), area(skin) + area(r.plugs().get(1)), 1e-9, "nothing kept twice");
        assertEquals(4, r.loops().get(1).uv().stream().filter(p -> Math.abs(p[0] - 0.9) < 1e-12 || Math.abs(p[0] - 0.5) < 1e-12 || Math.abs(p[0] - 0.7) < 1e-12).count(),
                "the outline without its repeated corner");
    }

    private static Vec3d ring(double u, double v, double r) { return new Vec3d(u, r * Math.cos(v), r * Math.sin(v)); }

    @Test void aWindowInADoorIsTakenOutOfTheDoorsPlug() {
        SurfaceCut.Grid g = cylinder();
        // the window first (innermost), then the door round it
        List<SurfaceCut.Hole> holes = List.of(
                SurfaceCut.Hole.convex(1, SurfaceCut.Kind.DISCARD, rect(0.8, 1.2, 1.2, 1.6)),
                SurfaceCut.Hole.convex(2, SurfaceCut.Kind.PLUG, rect(0.5, 1.5, 0.9, 2.0)));
        Mesh skin = new Mesh();
        SurfaceCut.Result r = SurfaceCut.cut(g, holes, skin);
        // on a unit cylinder an (u, v) rectangle's area is its u span times its arc, less the chord's sag per cell: compare to
        // the same plugs cut alone
        Mesh doorAlone = new Mesh();
        SurfaceCut.Result door = SurfaceCut.cut(g, List.of(SurfaceCut.Hole.convex(2, SurfaceCut.Kind.PLUG, rect(0.5, 1.5, 0.9, 2.0))), doorAlone);
        Mesh windowAlone = new Mesh();
        SurfaceCut.Result window = SurfaceCut.cut(g, List.of(SurfaceCut.Hole.convex(1, SurfaceCut.Kind.PLUG, rect(0.8, 1.2, 1.2, 1.6))), windowAlone);
        assertEquals(area(door.plugs().get(2)) - area(window.plugs().get(1)), area(r.plugs().get(2)), 1e-9);
        assertEquals(area(doorAlone), area(skin), 1e-9, "the skin is the same with or without the window in the door");
        assertFalse(r.plugs().containsKey(1), "a window makes no plug");
    }

    @Test void outlinesAreClosedLoopsOnTheFacetCrossingTheGrid() {
        SurfaceCut.Grid g = cylinder();
        SurfaceCut.Result r = SurfaceCut.cut(g, List.of(SurfaceCut.Hole.convex(1, SurfaceCut.Kind.DISCARD, rect(0.3, 0.9, -0.5, 0.45))), new Mesh());
        SurfaceCut.Loop loop = r.loops().get(1);
        // corners 4, grid lines crossed: u = 0.4, 0.8 twice each, v = 0, +-0.39 twice each
        assertEquals(4 + 4 + 6, loop.points().size());
        for (int i = 0; i < loop.points().size(); i++) {
            Vec3d a = loop.points().get(i), b = loop.points().get((i + 1) % loop.points().size());
            assertTrue(b.subtract(a).length() > 1e-6, "no repeated point");
            // on a flat cell of the unit cylinder: radius between the chord's and 1, x = u
            double[] uv = loop.uv().get(i);
            assertEquals(uv[0], a.x(), 1e-9);
            double r0 = Math.hypot(a.y(), a.z());
            assertTrue(r0 <= 1 + 1e-9 && r0 >= Math.cos(Math.PI / RAYS) - 1e-9, "on the facet: " + r0);
            assertTrue(loop.normals().get(i).dot(new Vec3d(0, a.y(), a.z())) > 0, "normals point out");
        }
    }

    @Test void theSkinFacesOutAndAStripJoinsTwoLoops() {
        SurfaceCut.Grid g = cylinder();
        List<SurfaceCut.Hole> holes = List.of(SurfaceCut.Hole.convex(1, SurfaceCut.Kind.DISCARD, new double[][]{{1.0, 2.0}, {1.7, 2.2}, {1.5, 3.1}, {1.1, 2.9}}));
        Mesh skin = new Mesh();
        SurfaceCut.Result outer = SurfaceCut.cut(g, holes, skin);
        for (MeshBuilder.Quad q : skin.quads()) {
            Vec3d face = q.c().subtract(q.a()).cross(q.d().equals(q.c()) ? q.c().subtract(q.b()) : q.d().subtract(q.b()));
            Vec3d mid = q.a().add(q.b()).add(q.c()).add(q.d()).scale(0.25);
            assertTrue(face.dot(new Vec3d(0, mid.y(), mid.z())) >= -1e-12, "outward");
        }
        // a liner 5 cm in, cut with the same hole: its loop has as many points, so a reveal joins them
        SurfaceCut.Grid in = scaled(g, 0.95);
        SurfaceCut.Result inner = SurfaceCut.cut(in, holes, new Mesh());
        assertEquals(outer.loops().get(1).points().size(), inner.loops().get(1).points().size());
        Mesh reveal = new Mesh();
        SurfaceCut.strip(reveal, outer.loops().get(1), inner.loops().get(1), PAINT, null);
        assertEquals(outer.loops().get(1).points().size(), reveal.quads().size());
        assertThrows(IllegalArgumentException.class, () -> SurfaceCut.strip(new Mesh(), outer.loops().get(1),
                SurfaceCut.cut(g, List.of(SurfaceCut.Hole.convex(1, SurfaceCut.Kind.DISCARD, rect(0.3, 0.9, 1, 2))), new Mesh()).loops().get(1), PAINT, null));
    }

    private static SurfaceCut.Grid scaled(SurfaceCut.Grid g, double k) {
        Vec3d[][][] c = new Vec3d[g.cellsU()][g.cellsV()][];
        for (int i = 0; i < g.cellsU(); i++) {
            for (int j = 0; j < g.cellsV(); j++) {
                Vec3d[] o = g.corner()[i][j];
                c[i][j] = new Vec3d[4];
                for (int t = 0; t < 4; t++) { c[i][j][t] = new Vec3d(o[t].x(), k * o[t].y(), k * o[t].z()); }
            }
        }
        return new SurfaceCut.Grid(g.u(), g.v(), g.periodic(), c, g.normal(), g.colour());
    }

    @Test void convexPiecesAreExact() {
        double[][] square = rect(0, 1, 0, 1), hole = rect(0.25, 0.5, 0.25, 0.75);
        double sum = 0;
        for (double[][] p : SurfaceCut.subtract(square, SurfaceCut.ccw(hole))) { sum += SurfaceCut.area(p); }
        assertEquals(1 - 0.125, sum, 1e-12);
        assertEquals(0.125, SurfaceCut.area(SurfaceCut.clip(square, SurfaceCut.ccw(hole))), 1e-12);
        assertNull(SurfaceCut.clip(square, SurfaceCut.ccw(rect(2, 3, 2, 3))));
    }
}
