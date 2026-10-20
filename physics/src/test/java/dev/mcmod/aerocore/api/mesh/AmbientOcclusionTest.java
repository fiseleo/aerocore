package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Ambient occlusion and enclosed faces, and baking into draw groups and levels of detail (0.29). */
class AmbientOcclusionTest {
    private static final int GREY = 0xff808080, GLASS = 0x60a0c0e0;
    private static final AmbientOcclusion.Settings SETTINGS = AmbientOcclusion.Settings.forSize(2);

    /** A thin plate (two-sided so it reads as a solid slab) of half-size h at height 0, split into small quads. */
    private static Mesh plate(double h) {
        return MeshOps.subdivide(new Mesh().box(new Vec3d(-h, -0.01, -h), new Vec3d(h, 0, h), GREY), 0.05);
    }

    private static float brightnessNear(Mesh m, AmbientOcclusion.Result r, Vec3d point) {
        double best = Double.MAX_VALUE;
        float value = -1;
        for (int i = 0; i < m.quads().size(); i++) {
            MeshBuilder.Quad q = m.quads().get(i);
            if (q.normal().y() < 0.99) { continue; }   // the plate's top
            for (int c = 0; c < 4; c++) {
                double d = q.corner(c).subtract(point).length();
                if (d < best) { best = d; value = r.at(i, c); }
            }
        }
        return value;
    }

    @Test void aPartOnAPlateDarkensTheContact() {
        Mesh m = plate(1);
        m.box(new Vec3d(-0.2, 0, -0.2), new Vec3d(0.2, 0.4, 0.2), GREY);
        AmbientOcclusion.Result r = AmbientOcclusion.compute(m.quads(), SETTINGS);
        float far = brightnessNear(m, r, new Vec3d(0.9, 0, 0.9)), near = brightnessNear(m, r, new Vec3d(0.25, 0, 0));
        assertEquals(1, far, 1e-6, "an open plate far from anything is unshaded");
        assertTrue(near < far - 0.15, "beside the box the plate is shaded: " + near);
        // a lone plate is open everywhere
        Mesh lone = plate(1);
        AmbientOcclusion.Result open = AmbientOcclusion.compute(lone.quads(), SETTINGS);
        for (int i = 0; i < lone.quads().size(); i++) {
            if (lone.quads().get(i).normal().y() > 0.99) { for (int c = 0; c < 4; c++) { assertEquals(1, open.at(i, c), 1e-6); } }
        }
    }

    @Test void whatIsShutInsideIsEnclosedAndWhatShowsThroughAnOpeningIsNot() {
        Mesh shell = new Mesh().box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY);
        int outer = shell.quads().size();
        shell.box(new Vec3d(-0.2, -0.2, -0.2), new Vec3d(0.2, 0.2, 0.2), GREY);
        AmbientOcclusion.Result r = AmbientOcclusion.compute(shell.quads(), SETTINGS);
        for (int i = 0; i < shell.quads().size(); i++) { assertEquals(i >= outer, r.enclosed()[i], "quad " + i); }
        // an open pipe with a box deep inside: air comes in at the ends, so the box stays drawn
        Mesh pipe = new Mesh().lathe(new Vec3d(-1, 0, 0), new Vec3d(1, 0, 0), new double[]{0, 1}, new double[]{0.15, 0.15}, null, 24, GREY, true);
        int walls = pipe.quads().size();
        pipe.box(new Vec3d(-0.05, -0.05, -0.05), new Vec3d(0.05, 0.05, 0.05), GREY);
        AmbientOcclusion.Result p = AmbientOcclusion.compute(pipe.quads(), SETTINGS);
        for (int i = walls; i < pipe.quads().size(); i++) { assertFalse(p.enclosed()[i]); }
        // deep inside the pipe a point looking out along it is far more occluded than at the mouth
        AmbientOcclusion.Field field = AmbientOcclusion.field(pipe.quads(), SETTINGS);
        double mouth = field.occlusion(new Vec3d(-0.98, 0, 0), new Vec3d(-1, 0, 0), 64, 0.5);
        double deep = field.occlusion(new Vec3d(-0.4, 0, 0), new Vec3d(-1, 0, 0), 64, 0.5);
        assertTrue(deep > mouth + 0.3, "mouth " + mouth + ", deep " + deep);
    }

    @Test void everyCornerGetsItsOwnOcclusionNotAnotherWithTheSameHash() {
        // a casing round a shaft: many corners in symmetric places, outside open, inside shaded (0.29: a key that
        // folded the position into one hashed long gave outside corners the inside's darkness)
        Mesh m = new Mesh().lathe(Vec3d.ZERO, new Vec3d(1, 0, 0), new double[]{0, 1}, new double[]{0.3, 0.3}, null, 64, GREY, true)
                .lathe(Vec3d.ZERO, new Vec3d(1, 0, 0), new double[]{0, 0, 1, 1}, new double[]{0, 0.1, 0.1, 0}, null, 64, GREY, true);
        AmbientOcclusion.Settings s = AmbientOcclusion.Settings.forSize(1);
        AmbientOcclusion.Result r = AmbientOcclusion.compute(m.quads(), s);
        AmbientOcclusion.Field field = AmbientOcclusion.field(m.quads(), s);
        for (int i = 0; i < m.quads().size(); i++) {
            MeshBuilder.Quad q = m.quads().get(i);
            for (int c = 0; c < 4; c++) {
                double direct = Math.max(s.floor(), 1 - s.strength() * field.occlusion(q.corner(c), q.normalAt(c), s.rays(), s.radiusM()));
                assertEquals(direct, r.at(i, c), 1e-6, "quad " + i + " corner " + c);
            }
        }
        // the casing's outside is open: no darker than a few per cent
        for (int i = 0; i < 64; i++) { for (int c = 0; c < 4; c++) { assertTrue(r.at(i, c) > 0.95, "outside corner " + r.at(i, c)); } }
    }

    @Test void shadeStaysNearWhatCastsItOnALongPanel() {
        // one 1 m panel quad with a wall standing at one end: unrefined, its dark corners shade the whole panel
        Mesh m = new Mesh().quad(new Vec3d(0, 0, 0), new Vec3d(0, 0, 0.2), new Vec3d(1, 0, 0.2), new Vec3d(1, 0, 0), GREY, Vec3d.Y)
                .box(new Vec3d(-0.06, 0, -0.05), new Vec3d(0.0, 0.3, 0.25), GREY);   // a wall along its near end
        AmbientOcclusion.Settings s = AmbientOcclusion.Settings.forSize(1);
        AmbientOcclusion.Result plain = AmbientOcclusion.compute(m.quads(), s), fine = AmbientOcclusion.refined(m.quads(), s);
        assertTrue(plain.at(0, 0) < 0.9, "the panel corner by the wall is shaded: " + plain.at(0, 0));
        assertTrue(fine.quads().size() > m.quads().size(), "the panel is split");
        for (int i = 0; i < fine.quads().size(); i++) {
            MeshBuilder.Quad q = fine.quads().get(i);
            if (q.normal().y() < 0.99 || q.a().y() > 1e-9) { continue; }
            double x = (q.a().x() + q.c().x()) / 2;
            if (x > 0.4) { for (int c = 0; c < 4; c++) { assertEquals(1, fine.at(i, c), 1e-6, "far along the panel it is open"); } }
        }
        assertEquals(fine.quads().size(), fine.enclosed().length);
    }

    /**
     * 0.39: a floor quad walled round (a room's floor, a channel's), each corner in two concave edges, its middle open.
     * Split by its corners' contrast only (CORNERS, as before), it stays one quad dark all over; CENTRE sees the open
     * middle and splits it, so the floor is light away from the walls and still dark along them.
     */
    @Test void aWalledFloorDarkAtItsCornersIsSplitAtItsOpenMiddle() {
        Mesh m = new Mesh().quad(new Vec3d(0, 0, 0), new Vec3d(0, 0, 1), new Vec3d(1, 0, 1), new Vec3d(1, 0, 0), GREY, Vec3d.Y)
                .box(new Vec3d(-0.05, 0, -0.05), new Vec3d(0, 0.3, 1.05), GREY).box(new Vec3d(1, 0, -0.05), new Vec3d(1.05, 0.3, 1.05), GREY)
                .box(new Vec3d(0, 0, -0.05), new Vec3d(1, 0.3, 0), GREY).box(new Vec3d(0, 0, 1), new Vec3d(1, 0.3, 1.05), GREY);
        AmbientOcclusion.Settings corners = AmbientOcclusion.Settings.forSize(1);
        assertEquals(AmbientOcclusion.Refine.CORNERS, corners.refine(), "as before unless asked");
        AmbientOcclusion.Result old = AmbientOcclusion.refined(m.quads(), corners), centre = AmbientOcclusion.refined(m.quads(), corners.withRefine(AmbientOcclusion.Refine.CENTRE));
        List<Integer> oldFloor = floor(old), newFloor = floor(centre);
        assertEquals(1, oldFloor.size(), "corners alone: the floor stays one quad");
        for (int c = 0; c < 4; c++) { assertTrue(old.at(oldFloor.get(0), c) < 0.9, "dark all over: " + old.at(oldFloor.get(0), c)); }
        assertTrue(newFloor.size() > 1, "centre: the floor is split");
        float middle = 0, edge = 1;
        for (int i : newFloor) {
            for (int c = 0; c < 4; c++) {
                Vec3d p = centre.quads().get(i).corner(c);
                if (Math.abs(p.x() - 0.5) < 0.1 && Math.abs(p.z() - 0.5) < 0.1) { middle = Math.max(middle, centre.at(i, c)); }
                if (p.x() < 1e-9 && p.z() < 1e-9) { edge = Math.min(edge, centre.at(i, c)); }
            }
        }
        assertEquals(1, middle, 1e-6, "its middle is open");
        assertTrue(edge < 0.9, "its corner by the walls is still shaded: " + edge);
    }

    /** The floor quads (facing up at height 0) of a result. */
    private static List<Integer> floor(AmbientOcclusion.Result r) {
        List<Integer> out = new java.util.ArrayList<>();
        for (int i = 0; i < r.quads().size(); i++) {
            MeshBuilder.Quad q = r.quads().get(i);
            if (q.normal().y() > 0.99 && Math.abs(q.a().y()) < 1e-9 && Math.abs(q.c().y()) < 1e-9) { out.add(i); }
        }
        return out;
    }

    @Test void aSlantedSurfaceDoesNotShadeItself() {
        // a cone and a plate tilted to every axis fill voxels up to the diagonal away from themselves: rays must start
        // past those (0.29: from a voxel and a half, slanted casings came out blotched)
        Mesh m = new Mesh().lathe(Vec3d.ZERO, new Vec3d(0.7, 0.2, 0.1), new double[]{0, 1}, new double[]{0.3, 0.12}, null, 48, GREY, true);
        Vec3d u = new Vec3d(1, 0.4, -0.3).normalized(), v = new Vec3d(0.2, 1, 0.7);
        v = v.subtract(u.scale(v.dot(u))).normalized();
        Vec3d o = new Vec3d(0, -1, 0);
        m.quad(o, o.add(v.scale(0.6)), o.add(v.scale(0.6)).add(u.scale(0.6)), o.add(u.scale(0.6)), GREY, u.cross(v).scale(-1));
        Mesh fine = MeshOps.subdivide(m, 0.03);
        AmbientOcclusion.Result r = AmbientOcclusion.compute(fine.quads(), AmbientOcclusion.Settings.forSize(1.2));
        for (int i = 0; i < fine.quads().size(); i++) {
            for (int c = 0; c < 4; c++) { assertEquals(1, r.at(i, c), 1e-6, "quad " + i + " corner " + c); }
            assertFalse(r.enclosed()[i]);
        }
    }

    @Test void onlyASurfaceLyingRightOverAnotherCoversIt() {
        // a 5 m casing (voxels about 4 cm) with a manifold ring 5 cm out: the casing stays (0.29: a voxel test took the
        // ring for a cover and cut holes in large engines)
        Mesh casing = new Mesh().lathe(Vec3d.ZERO, new Vec3d(5, 0, 0), new double[]{0, 1}, new double[]{0.5, 0.5}, null, 48, GREY, true);
        int walls = casing.quads().size();
        casing.tube(MeshBuilder.circle(new Vec3d(2.5, 0, 0), Vec3d.X, 0.55, 48), 0.02, null, GREY, null, 8, true);
        AmbientOcclusion.Result r = AmbientOcclusion.compute(casing.quads(), AmbientOcclusion.Settings.forSize(5));
        for (int i = 0; i < walls; i++) { assertFalse(r.enclosed()[i], "casing quad " + i + " is not covered"); }
        // two shells a millimetre apart: the inner one is covered; a pipe 0.8 mm inside a casing's inner wall too
        Mesh shells = new Mesh().lathe(Vec3d.ZERO, new Vec3d(1, 0, 0), new double[]{0, 1}, new double[]{0.300, 0.300}, null, 32, GREY, true);
        int outer = shells.quads().size();
        shells.lathe(Vec3d.ZERO, new Vec3d(1, 0, 0), new double[]{0, 1}, new double[]{0.299, 0.299}, null, 32, GREY, true);
        int inner = shells.quads().size();
        shells.lathe(new Vec3d(1, 0, 0), Vec3d.ZERO, new double[]{0, 1}, new double[]{0.35, 0.35}, null, 32, GREY, true)   // facing in
                .lathe(Vec3d.ZERO, new Vec3d(1, 0, 0), new double[]{0, 1}, new double[]{0.3492, 0.3492}, null, 32, GREY, true);
        AmbientOcclusion.Result s = AmbientOcclusion.compute(shells.quads(), AmbientOcclusion.Settings.forSize(1));
        for (int i = 0; i < outer; i++) { assertFalse(s.enclosed()[i], "the outer shell shows"); }
        for (int i = outer; i < inner; i++) { assertTrue(s.enclosed()[i], "the shell a millimetre under it is covered"); }
        for (int i = shells.quads().size() - 32; i < shells.quads().size(); i++) { assertTrue(s.enclosed()[i], "the pipe under the inner wall is covered"); }
    }

    @Test void theSameMeshGivesTheSameResult() {
        Mesh m = plate(0.5).cylinder(Vec3d.ZERO, new Vec3d(0, 0.5, 0), 0.1, 0.05, 16, GREY);
        AmbientOcclusion.Result a = AmbientOcclusion.compute(m.quads(), SETTINGS), b = AmbientOcclusion.compute(m.quads(), SETTINGS);
        assertArrayEquals(a.corner(), b.corner());
        assertArrayEquals(a.enclosed(), b.enclosed());
    }

    @Test void bakingGroupsByPassFinishAndHeatAndScalesTextureToTheTile() {
        Finish steel = new Finish("steel", 0.25), brass = new Finish("brass", 0.1), plain = new Finish("plain", 0.5);
        Mesh m = new Mesh().finish(steel).box(Vec3d.ZERO, new Vec3d(0.5, 0.5, 0.5), GREY)
                .finish(brass).sphere(0.1, GREY)
                .finish(steel).hot(true).cylinder(Vec3d.ZERO, Vec3d.X, 0.1, 0.1, 8, GREY).hot(false)
                .finish(null).box(Vec3d.ZERO, new Vec3d(0.1, 0.1, 0.1), GREY)
                .finish(steel).box(Vec3d.ZERO, new Vec3d(1, 1, 1), GLASS)
                .decal(true).box(Vec3d.ZERO, new Vec3d(0.2, 0.2, 0.2), GREY);
        PackedMesh p = MeshBaker.bake(m, plain);
        assertEquals(m.quads().size(), p.quads());
        List<PackedMesh.Group> g = p.groups();
        // SOLID first, by finish id, cold before hot; then TRANSLUCENT; then DECAL
        assertEquals(List.of("brass/SOLID/false", "plain/SOLID/false", "steel/SOLID/false", "steel/SOLID/true", "steel/TRANSLUCENT/false", "steel/DECAL/false"),
                g.stream().map(x -> x.finish().id() + "/" + x.pass() + "/" + x.hot()).toList());
        int sum = 0;
        for (PackedMesh.Group x : g) { assertEquals(sum, x.first()); sum += x.count(); }
        assertEquals(p.quads(), sum);
        // the first steel box face: planar metres over a 0.25 m tile
        PackedMesh.Group steelSolid = g.get(2);
        int v = 4 * steelSolid.first();
        float[] uv = p.uvs(), pos = p.positions();
        float[] metres = MeshBuilder.planar(m.quads().get(0).normal(), m.quads().get(0).a(), m.quads().get(0).b(), m.quads().get(0).c(), m.quads().get(0).d());
        assertEquals(metres[0] / 0.25, uv[2 * v], 1e-6);
        assertEquals(m.quads().get(0).a().x(), pos[3 * v], 1e-6);
        assertEquals(255, p.light(v));
    }

    @Test void anEngineSizedMeshBakesInAboutASecond() {
        // about 100k quads on a 1.2 m part: a casing of fine rings and four hundred bolts
        Mesh m = new Mesh();
        for (int i = 0; i < 40; i++) {
            double x = 0.03 * i;
            m.lathe(new Vec3d(x, 0, 0), new Vec3d(x + 0.03, 0, 0), new double[]{0, 0.5, 1}, new double[]{0.3, 0.31, 0.3}, null, 800, GREY, false);
        }
        for (int i = 0; i < 400; i++) {
            double a = 2 * Math.PI * i / 400;
            Vec3d at = new Vec3d(0.6, 0.32 * Math.cos(a), 0.32 * Math.sin(a));
            m.cylinder(at, at.add(new Vec3d(0.01, 0.02 * Math.cos(a), 0.02 * Math.sin(a))), 0.008, 0.008, 12, GREY)
                    .lathe(at, at.add(new Vec3d(0.02, 0.04 * Math.cos(a), 0.04 * Math.sin(a))), new double[]{0, 1}, new double[]{0.01, 0.01}, null, 36, GREY, true);
        }
        int quads = m.quads().size();
        assertTrue(quads > 90_000, "quads " + quads);
        long t0 = System.nanoTime();
        PackedMesh[] levels = MeshBaker.levels(m, null, AmbientOcclusion.Settings.forSize(1.2), 2, true);
        double seconds = (System.nanoTime() - t0) * 1e-9;
        System.out.printf("ambient occlusion and 3 levels for %d quads: %.2f s%n", quads, seconds);
        assertTrue(levels[0].quads() > 0);
        assertTrue(seconds < 6, "far over the 1.5 s per 100k quads budget: " + seconds);
    }

    @Test void levelsDropDetailAndCullWhatCannotBeSeen() {
        Mesh m = new Mesh().box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY)          // a closed casing
                .box(new Vec3d(-0.3, -0.3, -0.3), new Vec3d(0.3, 0.3, 0.3), GREY)        // something inside it, found enclosed
                .detail(2).sphere(0.05, GREY).detail(0)                                  // a bolt (inside too)
                .interior(true).box(new Vec3d(1.5, 0, 0), new Vec3d(1.6, 0.1, 0.1), GREY); // tagged interior though outside
        PackedMesh[] world = MeshBaker.levels(m, null, SETTINGS, 3, true), cut = MeshBaker.levels(m, null, SETTINGS, 3, false);
        assertEquals(6, world[0].quads(), "only the casing is drawn whole");
        assertEquals(6 + 6 + 128 + 6, cut[0].quads());
        assertEquals(6 + 6 + 128 + 6, cut[1].quads());
        assertEquals(6 + 6 + 6, cut[2].quads(), "the bolt goes at level 2");
        for (int level = 1; level < 4; level++) { assertTrue(cut[level].quads() <= cut[level - 1].quads()); }
        // the casing's outside is lit by the open air, the inside box shaded by the casing around it
        assertTrue(world[0].light(0) > 200);
    }
}
