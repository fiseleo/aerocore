package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** Smoothing, mirroring, placing, subdividing, levels of detail and OBJ reading and writing (0.29). */
class MeshOpsTest {
    private static final int GREY = 0xff808080;

    private static Vec3d centroid(MeshBuilder<?> m) {
        Vec3d[] b = m.bounds();
        return b[0].add(b[1]).scale(0.5);
    }

    private static void facesOut(MeshBuilder<?> m) {
        Vec3d c = centroid(m);
        for (MeshBuilder.Quad q : m.quads()) {
            Vec3d mid = q.a().add(q.b()).add(q.c()).add(q.d()).scale(0.25);
            Vec3d n = MeshOps.faceNormal(q);
            if (n.lengthSquared() == 0) { continue; }
            assertTrue(n.dot(mid.subtract(c)) > -1e-9, "a face points inward");
            assertTrue(q.normal().dot(n) > 0.5, "the stored normal disagrees with the winding");
        }
    }

    @Test void mirroredSolidsStillFaceOut() {
        Mesh m = new Mesh().box(new Vec3d(0.1, 0, 0), new Vec3d(0.6, 0.3, 0.2), GREY)
                .cylinder(new Vec3d(1, 0, 0), new Vec3d(1.5, 0.3, 0), 0.1, 0.05, 12, GREY);
        for (int axis = 0; axis < 3; axis++) {
            Mesh mirrored = MeshOps.mirror(m, axis);
            assertEquals(m.quads().size(), mirrored.quads().size());
            facesOut(MeshOps.mirror(new Mesh().box(new Vec3d(0.1, 0, 0), new Vec3d(0.6, 0.3, 0.2), GREY), axis));
            facesOut(MeshOps.mirror(new Mesh().cylinder(new Vec3d(1, 0, 0), new Vec3d(1.5, 0.3, 0), 0.1, 0.05, 12, GREY), axis));
            for (MeshBuilder.Quad q : mirrored.quads()) { assertFalse(q.b().equals(q.c()) && !q.c().equals(q.d()), "a triangle keeps its repeat at the end"); }
        }
    }

    @Test void smoothingFollowsTheCreaseAngle() {
        // a 32-sided cylinder: neighbouring sides turn 11.25 degrees, so they smooth at 30 but the caps stay sharp
        Mesh cylinder = new Mesh();
        int sides = 32;
        for (int k = 0; k < sides; k++) {
            double a0 = 2 * Math.PI * k / sides, a1 = 2 * Math.PI * (k + 1) / sides;
            Vec3d p0 = new Vec3d(Math.cos(a0), 0, Math.sin(a0)), p1 = new Vec3d(Math.cos(a1), 0, Math.sin(a1));
            cylinder.quad(p0, p0.add(Vec3d.Y), p1.add(Vec3d.Y), p1, GREY, p0.add(p1));   // counter-clockwise seen from outside
        }
        Mesh smooth = MeshOps.smooth(cylinder, 30);
        for (MeshBuilder.Quad q : smooth.quads()) {
            for (int i = 0; i < 4; i++) {
                Vec3d radial = new Vec3d(q.corner(i).x(), 0, q.corner(i).z()).normalized();
                assertEquals(1, q.normals()[i].dot(radial), 1e-9, "the corner normal is the surface's");
            }
        }
        Mesh box = MeshOps.smooth(new Mesh().box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY), 30);
        for (MeshBuilder.Quad q : box.quads()) { for (int i = 0; i < 4; i++) { assertEquals(1, q.normals()[i].dot(q.normal()), 1e-12, "a box stays flat"); } }
        Mesh rounded = MeshOps.smooth(new Mesh().box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY), 95);
        Vec3d corner = rounded.quads().get(0).normals()[0];
        assertEquals(1 / Math.sqrt(3), Math.abs(corner.x()), 1e-9, "past 90 degrees a cube's corners round off");
    }

    @Test void placingStretchesNormalsCorrectlyAndFlipsMirroredScales() {
        // a sphere stretched into an ellipsoid: normals stay square to the surface (the gradient of the ellipsoid)
        Vec3d scale = new Vec3d(2, 0.5, 1);
        Mesh ellipsoid = MeshOps.placed(new Mesh().sphere(1, GREY), scale, Orientation.IDENTITY, Vec3d.ZERO);
        for (MeshBuilder.Quad q : ellipsoid.quads()) {
            for (int i = 0; i < 4; i++) {
                Vec3d p = q.corner(i);
                if (p.lengthSquared() < 1e-12 || Math.abs(Math.abs(p.y()) - 0.5) < 1e-9) { continue; }
                Vec3d gradient = new Vec3d(p.x() / 4, p.y() / 0.25, p.z()).normalized();
                assertEquals(1, q.normals()[i].dot(gradient), 1e-9);
            }
        }
        Mesh flipped = MeshOps.placed(new Mesh().box(new Vec3d(0.1, 0.1, 0.1), new Vec3d(0.5, 0.4, 0.3), GREY), new Vec3d(-1, 2, 1),
                Orientation.rotationVector(new Vec3d(0, 0.6, 0)), new Vec3d(3, 0, 0));
        facesOut(flipped);
    }

    @Test void subdividingKeepsTheSurfaceAndItsMapping() {
        Mesh box = new Mesh().box(Vec3d.ZERO, new Vec3d(1, 0.5, 0.25), GREY);
        Mesh fine = MeshOps.subdivide(box, 0.1);
        double a0 = 0, a1 = 0;
        for (MeshBuilder.Quad q : box.quads()) { a0 += MeshOps.area(q); }
        for (MeshBuilder.Quad q : fine.quads()) {
            a1 += MeshOps.area(q);
            assertArrayEquals(MeshBuilder.planar(q.normal(), q.a(), q.b(), q.c(), q.d()), q.uv(), 1e-6f);
        }
        assertEquals(a0, a1, 1e-12);
        assertEquals(2 * (10 * 5 + 5 * 3 + 10 * 3), fine.quads().size());
    }

    @Test void levelsOfDetailDropSmallPartsFarAwayAndInteriorsOutside() {
        Mesh m = new Mesh().box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY)
                .detail(1).sphere(0.1, GREY).detail(3).sphere(0.05, GREY).detail(0).interior(true).box(Vec3d.ZERO, new Vec3d(0.5, 0.5, 0.5), GREY);
        int[] counts = new int[4];
        for (int level = 0; level < 4; level++) { counts[level] = MeshOps.atLevel(m, level, false).quads().size(); }
        assertEquals(6 + 128 + 128, counts[0]);
        assertEquals(6 + 128, counts[1]);
        assertEquals(6 + 128, counts[2]);
        assertEquals(6, counts[3]);
        assertEquals(6 + 128 + 128 + 6, MeshOps.atLevel(m, 0, true).quads().size());
    }

    @Test void objRoundTripsAndReadsWhatBlenderWrites() {
        Finish brass = new Finish("brass", 0.1);
        Mesh m = new Mesh().finish(brass).cylinder(Vec3d.ZERO, new Vec3d(0, 0.2, 0), 0.03, 0.03, 12, 0xffc8a040)
                .finish(null).box(new Vec3d(0.1, 0, 0), new Vec3d(0.2, 0.1, 0.1), GREY);
        ObjWriter.Files files = ObjWriter.write(Map.of("bolt", m), "bolt.mtl");
        assertTrue(files.mtl().contains("newmtl brass_c8a040") && files.mtl().contains("Ka 0 0 0"));
        ObjReader.Result back = ObjReader.read(files.obj(), name -> new ObjReader.Material(name.startsWith("brass") ? brass : null, 0xffffffff), 1);
        assertEquals(m.quads().size(), back.mesh().quads().size());
        assertEquals(0, back.polygons());
        assertFalse(back.vertexColours() || back.missingMaterial());
        for (int i = 0; i < m.quads().size(); i++) {
            MeshBuilder.Quad a = m.quads().get(i), b = back.mesh().quads().get(i);
            for (int k = 0; k < 4; k++) {
                assertEquals(0, a.corner(k).subtract(b.corner(k)).length(), 2e-6);
                assertEquals(1, a.normalAt(k).dot(b.normalAt(k)), 1e-6);
            }
            assertEquals(a.normal().dot(b.normal()), 1, 1e-6, "the winding comes back the same");
            // written in tiles, read back with a scale of 1: metres over the tile
            double tile = a.finish() == null ? Finish.DEFAULT_TILE_M : a.finish().tileM();
            // a triangle (repeated last corner) is written with three corners, so its fourth comes back as the third
            int written = a.d().equals(a.c()) ? 6 : 8;
            for (int k = 0; k < written; k++) { assertEquals(a.uv()[k] / tile, b.uv()[k], 1e-5); }
        }
        assertSame(brass, back.mesh().quads().get(0).finish());

        String blender = """
                # Blender 5.2
                mtllib x.mtl
                o Washer
                v 0 0 0
                v 1 0 0
                v 1 1 0
                v 0.5 1.5 0
                v 0 1 0
                v 0 0 1 1 0 0
                vt 0 0
                vn 0 0 1
                usemtl steel
                f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
                f -6/-1/-1 -5/-1/-1 -4/-1/-1
                g other
                f 1 2 3
                """;
        ObjReader.Result r = ObjReader.read(blender, ObjReader.plain(0.2), 0.2);
        assertEquals(1, r.polygons(), "a five-cornered face is counted");
        assertTrue(r.vertexColours(), "a v line with colours is noticed");
        assertEquals(2 + 1 + 1, r.mesh().quads().size(), "five corners fan into a quad and a triangle");
        assertEquals(3, r.group("Washer").quads().size());
        assertEquals(1, r.group("other").quads().size());
        assertEquals(List.of("steel"), List.copyOf(r.materials()));
        MeshBuilder.Quad tri = r.group("Washer").quads().get(1);
        assertEquals(tri.c(), tri.d(), "the last of an odd fan is a triangle");
        assertEquals(1, r.mesh().quads().get(0).normal().z(), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> ObjReader.read("v 0 0 0\nf 1 2 3\n", ObjReader.plain(1), 1));
    }

    @Test void aStandInShowsAloneAtTheFarthestLevelOnly() {
        Mesh m = new Mesh();
        m.box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), 0xffffffff);                 // the whole: 6 quads
        m.detail(3).box(new Vec3d(1, -0.1, -0.1), new Vec3d(1.2, 0.1, 0.1), 0xffffffff).detail(0);   // a close detail: 6 more
        int whole = m.quads().size();
        Mesh standIn = new Mesh().box(new Vec3d(-1.1, -1.1, -1.1), new Vec3d(1.1, 1.1, 1.1), 0xff808080);
        for (MeshBuilder.Quad q : standIn.quads()) { m.quads().add(q.withTags(q.tags() | MeshBuilder.STAND_IN)); }
        PackedMesh[] levels = MeshBaker.levels(m, null, null, 2, true);
        assertEquals(whole, levels[0].quads(), "near: the whole and its details, no stand-in");
        assertEquals(6, levels[1].quads(), "middle: the whole without its details");
        assertEquals(6, levels[2].quads(), "far: the stand-in alone");
        assertEquals(0x80, levels[2].colors()[0] & 0xff, "and it is the stand-in");
        PackedMesh[] one = MeshBaker.levels(m, null, null, 0, true);
        assertEquals(whole, one[0].quads(), "a single level (a design screen) never shows it");
        assertEquals(whole, MeshOps.atLevel(m, 0, true).quads().size());
        // the stand-in wraps the whole: occlusion and culling must not see it, or everything inside it would be enclosed
        PackedMesh[] shaded = MeshBaker.levels(m, null, AmbientOcclusion.Settings.forSize(2.4), 2, true);
        assertTrue(shaded[0].quads() >= whole, "near, with occlusion: the whole is kept, " + shaded[0].quads());
        assertEquals(6, shaded[2].quads(), "far, with occlusion: the stand-in alone");
    }
}
