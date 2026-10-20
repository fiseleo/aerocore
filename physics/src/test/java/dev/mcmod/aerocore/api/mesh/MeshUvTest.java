package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Texture coordinates (metres), pens and rounded boxes (0.29). */
class MeshUvTest {
    private static final int GREY = 0xff808080;
    private static final Finish STEEL = new Finish("steel", 0.2);

    @Test void boxFacesAreMappedFlatAtTrueScale() {
        Mesh m = new Mesh().box(new Vec3d(0, 0, 0), new Vec3d(0.6, 0.4, 0.2), GREY);
        for (MeshBuilder.Quad q : m.quads()) {
            assertNotNull(q.uv());
            // the mapping keeps lengths: each edge's length in texture space equals its length in space
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                double du = q.uv()[2 * j] - q.uv()[2 * i], dv = q.uv()[2 * j + 1] - q.uv()[2 * i + 1];
                assertEquals(q.corner(j).subtract(q.corner(i)).length(), Math.hypot(du, dv), 1e-6);
            }
        }
        // side faces share v (height) along their common vertical edges, so a texture runs on round a corner
        MeshBuilder.Quad east = face(m, Vec3d.X), south = face(m, Vec3d.Z);
        for (int i = 0; i < 4; i++) for (int k = 0; k < 4; k++) {
            if (east.corner(i).subtract(south.corner(k)).lengthSquared() < 1e-18) { assertEquals(east.uv()[2 * i + 1], south.uv()[2 * k + 1], 1e-6); }
        }
    }

    private static MeshBuilder.Quad face(Mesh m, Vec3d n) {
        for (MeshBuilder.Quad q : m.quads()) { if (q.normal().subtract(n).lengthSquared() < 1e-12) { return q; } }
        throw new AssertionError("no face " + n);
    }

    @Test void turnedSurfacesWrapWholeTilesAroundAndRunTheProfileLength() {
        double r = 0.13;
        Mesh m = new Mesh().finish(STEEL).lathe(Vec3d.ZERO, new Vec3d(0, 0, 0.5), new double[]{0, 0, 1, 1}, new double[]{0, r, r, 0}, null, 24, GREY, true);
        double expectedRound = Finish.snapped(2 * Math.PI * r, STEEL.tileM());
        assertEquals(0, Math.IEEEremainder(expectedRound, STEEL.tileM()), 1e-12);
        double maxU = 0, maxV = 0;
        for (MeshBuilder.Quad q : m.quads()) {
            assertSame(STEEL, q.finish());
            for (int i = 0; i < 4; i++) { maxU = Math.max(maxU, q.uv()[2 * i]); maxV = Math.max(maxV, q.uv()[2 * i + 1]); }
        }
        assertEquals(expectedRound, maxU, 1e-6, "the side wraps a whole number of tiles");
        assertEquals(r + 0.5 + r, maxV, 1e-6, "v runs the profile: across the cap, along the side, across the other cap");
    }

    @Test void tubesRunTheirPathLength() {
        List<Vec3d> path = List.of(new Vec3d(0, 0, 0), new Vec3d(1, 0, 0), new Vec3d(1, 2, 0));
        Mesh m = new Mesh().tube(path, 0.05, 8, GREY);
        double maxV = 0;
        for (MeshBuilder.Quad q : m.quads()) { for (int i = 0; i < 4; i++) { maxV = Math.max(maxV, q.uv()[2 * i + 1]); } }
        assertEquals(3, maxV, 1e-6);
    }

    @Test void sheetsRunAlongTheirFirstRowAndAcross() {
        Mesh m = new Mesh().sheet(List.of(new Vec3d(0, 0, 0), new Vec3d(0.3, 0, 0), new Vec3d(0.7, 0, 0)),
                List.of(new Vec3d(0, 0.2, 0), new Vec3d(0.3, 0.2, 0), new Vec3d(0.7, 0.25, 0)), GREY, false);
        MeshBuilder.Quad last = m.quads().get(1);
        assertArrayEquals(new float[]{0.3f, 0, 0.3f, 0.2f, 0.7f, 0.25f, 0.7f, 0}, last.uv(), 1e-6f);
    }

    @Test void pensStampTheQuadsBuiltWhileTheyAreSet() {
        Mesh m = new Mesh().finish(STEEL).detail(2).hot(true).box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY)
                .finish(null).detail(0).hot(false).interior(true).sphere(0.2, GREY);
        for (int i = 0; i < 6; i++) {
            MeshBuilder.Quad q = m.quads().get(i);
            assertSame(STEEL, q.finish());
            assertEquals(2, q.detail());
            assertTrue(q.has(MeshBuilder.HOT));
            assertFalse(q.has(MeshBuilder.INTERIOR));
        }
        MeshBuilder.Quad ball = m.quads().get(6);
        assertNull(ball.finish());
        assertEquals(0, ball.detail());
        assertTrue(ball.has(MeshBuilder.INTERIOR) && !ball.has(MeshBuilder.HOT));
        // a part built apart and moved in keeps the pens it was built with
        Mesh moved = new Mesh().finish(STEEL).box(Vec3d.ZERO, new Vec3d(0.1, 0.1, 0.1), dev.mcmod.aerocore.api.Orientation.IDENTITY, GREY);
        for (MeshBuilder.Quad q : moved.quads()) { assertSame(STEEL, q.finish()); }
    }

    @Test void cuttingInterpolatesTextureCoordinates() {
        Mesh m = new Mesh().box(new Vec3d(-1, 0, 0), new Vec3d(1, 1, 1), GREY);
        Mesh cut = m.clipped(new Vec3d(0.25, 0, 0), Vec3d.X);
        for (MeshBuilder.Quad q : cut.quads()) {
            float[] expected = MeshBuilder.planar(q.normal(), q.a(), q.b(), q.c(), q.d());
            assertArrayEquals(expected, q.uv(), 1e-6f, "a cut face keeps its flat mapping");
            for (int i = 0; i < 4; i++) { assertTrue(q.corner(i).x() <= 0.25 + 1e-12); }
        }
    }

    @Test void roundedBoxesAreClosedAndRoundedOutward() {
        Vec3d lo = new Vec3d(-0.3, 0, -0.1), hi = new Vec3d(0.5, 0.4, 0.2);
        double r = 0.05;
        Mesh m = new Mesh().roundedBox(lo, hi, r, 6, GREY);
        Vec3d[] b = m.bounds();
        assertEquals(lo.x(), b[0].x(), 1e-12); assertEquals(hi.y(), b[1].y(), 1e-12); assertEquals(hi.z(), b[1].z(), 1e-12);
        Vec3d centre = lo.add(hi).scale(0.5);
        double area = 0;
        for (MeshBuilder.Quad q : m.quads()) {
            Vec3d mid = q.a().add(q.b()).add(q.c()).add(q.d()).scale(0.25);
            assertTrue(MeshOps.faceNormal(q).dot(mid.subtract(centre)) > 0, "faces face out");
            for (int i = 0; i < 4; i++) { assertEquals(1, q.normalAt(i).length(), 1e-9); }
            area += MeshOps.area(q);
        }
        double x = hi.x() - lo.x() - 2 * r, y = hi.y() - lo.y() - 2 * r, z = hi.z() - lo.z() - 2 * r;
        double exact = 2 * (x * y + y * z + z * x) + 2 * Math.PI * r * (x + y + z) + 4 * Math.PI * r * r;
        assertEquals(exact, area, 0.01 * exact, "flat faces, quarter-round edges, eighth-sphere corners");
        // no radius: a plain box
        assertEquals(6, new Mesh().roundedBox(lo, hi, 0, 6, GREY).quads().size());
    }
}
