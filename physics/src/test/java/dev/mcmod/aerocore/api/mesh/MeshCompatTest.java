package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The builders moved from the client's PartMesh to {@link MeshBuilder} (0.29) give the same quads as the frozen 0.28
 * copy: corners, face normal, corner normals and colours, to 1e-12 (texture coordinates, finishes and tags are new).
 */
class MeshCompatTest {
    private static final int RED = 0xffc23a2e, BLUE = 0xff3a5ec2, GLASS = 0x5578a0c0;
    private static final Orientation TURN = Orientation.rotationVector(new Vec3d(0.3, -0.7, 0.45));

    private static void same(LegacyMesh legacy, MeshBuilder<?> mesh) {
        List<LegacyMesh.Quad> a = legacy.quads;
        List<MeshBuilder.Quad> b = mesh.quads();
        assertEquals(a.size(), b.size(), "quad count");
        for (int i = 0; i < a.size(); i++) {
            LegacyMesh.Quad x = a.get(i);
            MeshBuilder.Quad y = b.get(i);
            near(x.a(), y.a()); near(x.b(), y.b()); near(x.c(), y.c()); near(x.d(), y.d());
            near(x.normal(), y.normal());
            assertEquals(x.color(), y.color(), "colour of quad " + i);
            assertEquals(x.normals() == null, y.normals() == null, "corner normals of quad " + i);
            if (x.normals() != null) { for (int k = 0; k < 4; k++) { near(x.normals()[k], y.normals()[k]); } }
            assertEquals(x.colors() == null, y.colors() == null, "corner colours of quad " + i);
            if (x.colors() != null) { assertArrayEquals(x.colors(), y.colors(), "corner colours of quad " + i); }
            if (y.uv() != null) { assertEquals(8, y.uv().length); for (float f : y.uv()) { assertTrue(Float.isFinite(f)); } }
        }
    }

    private static void near(Vec3d p, Vec3d q) {
        assertEquals(p.x(), q.x(), 1e-12); assertEquals(p.y(), q.y(), 1e-12); assertEquals(p.z(), q.z(), 1e-12);
    }

    private static void check(Consumer<LegacyMesh> legacy, Consumer<Mesh> mesh) {
        LegacyMesh l = new LegacyMesh();
        Mesh m = new Mesh();
        legacy.accept(l);
        mesh.accept(m);
        same(l, m);
    }

    @Test void boxesAndSolidsAreUnchanged() {
        Vec3d lo = new Vec3d(-0.2, 0.1, -0.4), hi = new Vec3d(0.5, 0.9, 0.3);
        check(l -> l.box(lo, hi, RED), m -> m.box(lo, hi, RED));
        check(l -> l.box(new Vec3d(1, 2, 3), new Vec3d(0.1, 0.2, 0.3), TURN, BLUE), m -> m.box(new Vec3d(1, 2, 3), new Vec3d(0.1, 0.2, 0.3), TURN, BLUE));
        Vec3d[] corners = {new Vec3d(0, 0, 0), new Vec3d(1, 0, 0), new Vec3d(1, 1, 0), new Vec3d(0, 1, 0),
                new Vec3d(0.2, 0.1, 2), new Vec3d(0.8, 0.1, 2), new Vec3d(0.7, 0.6, 2), new Vec3d(0.3, 0.6, 2)};
        check(l -> l.hexahedron(corners, RED), m -> m.hexahedron(corners, RED));
        // the same corners given end-first, so faces are wound the other way round
        Vec3d[] reversed = {corners[4], corners[5], corners[6], corners[7], corners[0], corners[1], corners[2], corners[3]};
        check(l -> l.hexahedron(reversed, RED), m -> m.hexahedron(reversed, RED));
        check(l -> l.sphere(0.37, BLUE), m -> m.sphere(0.37, BLUE));
        check(l -> l.cylinderX(-0.3, 0.8, 0.12, RED), m -> m.cylinderX(-0.3, 0.8, 0.12, RED));
        check(l -> l.cylinder(new Vec3d(0, 0, 0), new Vec3d(0.3, 1, -0.2), 0.2, 0.05, 7, RED), m -> m.cylinder(new Vec3d(0, 0, 0), new Vec3d(0.3, 1, -0.2), 0.2, 0.05, 7, RED));
        check(l -> l.cylinder(new Vec3d(0, 0, 0), new Vec3d(0, 0, 1e-12), 0.2, 0.05, 7, RED), m -> m.cylinder(new Vec3d(0, 0, 0), new Vec3d(0, 0, 1e-12), 0.2, 0.05, 7, RED));
    }

    @Test void sheetsAndQuadsAreUnchanged() {
        List<Vec3d> a = List.of(new Vec3d(0, 0, 0), new Vec3d(0.4, 0.05, 0), new Vec3d(0.8, 0.2, 0.1)),
                b = List.of(new Vec3d(0, 0, 0.3), new Vec3d(0.45, 0.1, 0.3), new Vec3d(0.85, 0.3, 0.35));
        check(l -> l.sheet(a, b, RED, true), m -> m.sheet(a, b, RED, true));
        check(l -> l.sheet(a, b, RED, false), m -> m.sheet(a, b, RED, false));
        Vec3d p = new Vec3d(0, 0, 0), q = new Vec3d(1, 0, 0), r = new Vec3d(1, 1, 0.2), s = new Vec3d(0, 1, 0);
        check(l -> l.quad(p, q, r, s, BLUE, new Vec3d(0, 0, 3)), m -> m.quad(p, q, r, s, BLUE, new Vec3d(0, 0, 3)));
        Vec3d[] n = {Vec3d.Z, Vec3d.Z, new Vec3d(0, 0.2, 1).normalized(), Vec3d.Z};
        int[] c = {RED, BLUE, RED, BLUE};
        check(l -> l.quad(p, q, r, s, n, c), m -> m.quad(p, q, r, s, n, c));
    }

    @Test void lathesAndTubesAreUnchanged() {
        double[] along = {0, 0, 0.2, 0.25, 0.7, 1, 1}, radius = {0, 0.1, 0.12, 0.2, 0.2, 0.08, 0};
        int[] colors = {RED, RED, BLUE, BLUE, RED, RED, BLUE};
        Vec3d from = new Vec3d(0.1, 0.2, 0.3), to = new Vec3d(-0.4, 0.9, 1.2);
        check(l -> l.lathe(from, to, along, radius, null, 20, RED, true), m -> m.lathe(from, to, along, radius, null, 20, RED, true));
        check(l -> l.lathe(from, to, along, radius, colors, 20, RED, false), m -> m.lathe(from, to, along, radius, colors, 20, RED, false));
        List<Vec3d> path = MeshBuilder.bezier(new Vec3d(0, 0, 0), new Vec3d(0.5, 0, 0), new Vec3d(0.5, 0.5, 0.2), new Vec3d(0.5, 1, 0.4), 12);
        double[] radii = new double[path.size()];
        int[] pc = new int[path.size()];
        for (int i = 0; i < radii.length; i++) { radii[i] = 0.03 + 0.002 * i; pc[i] = i % 2 == 0 ? RED : BLUE; }
        check(l -> l.tube(path, 0.04, null, RED, null, 10, false), m -> m.tube(path, 0.04, null, RED, null, 10, false));
        check(l -> l.tube(path, 0.04, radii, RED, pc, 10, false), m -> m.tube(path, 0.04, radii, RED, pc, 10, false));
        List<Vec3d> ring = MeshBuilder.circle(new Vec3d(0, 1, 0), new Vec3d(0.2, 1, 0.1), 0.5, 18);
        check(l -> l.tube(ring, 0.02, null, BLUE, null, 8, true), m -> m.tube(ring, 0.02, null, BLUE, null, 8, true));
        List<Vec3d> coil = MeshBuilder.helix(Vec3d.ZERO, Vec3d.Y, 0.1, 0.4, 3.5, 12);
        check(l -> l.tube(coil, 0.01, null, RED, null, 6, false), m -> m.tube(coil, 0.01, null, RED, null, 6, false));
    }

    @Test void extrusionsAndHullsAreUnchanged() {
        double[][] cam = {{0.05, 0}, {0.03, 0.04}, {0, 0.07}, {-0.03, 0.04}, {-0.05, 0}, {-0.03, -0.04}, {0, -0.05}, {0.03, -0.04}};
        Vec3d o = new Vec3d(0.1, 0, 0.2);
        check(l -> l.extrude(cam, o, Vec3d.X, Vec3d.Y, new Vec3d(0, 0, 0.02), RED, true), m -> m.extrude(cam, o, Vec3d.X, Vec3d.Y, new Vec3d(0, 0, 0.02), RED, true));
        check(l -> l.extrude(cam, o, Vec3d.Y, Vec3d.X, new Vec3d(0, 0, 0.02), RED, false), m -> m.extrude(cam, o, Vec3d.Y, Vec3d.X, new Vec3d(0, 0, 0.02), RED, false));
        SlenderHull hull = SlenderHull.tapered(4, 0.6, -0.3, 0.4, 0.15, 9, s -> 1 - s * s * s * s);
        check(l -> l.loft(hull, Double.NaN), m -> m.loft(hull, Double.NaN));
        check(l -> l.loft(hull, -0.1), m -> m.loft(hull, -0.1));
    }

    @Test void combiningAndCuttingAreUnchanged() {
        LegacyMesh la = new LegacyMesh().cylinder(Vec3d.ZERO, new Vec3d(1, 0, 0), 0.3, 0.25, 16, RED).box(new Vec3d(-0.1, -0.4, -0.4), new Vec3d(0.1, 0.4, 0.4), GLASS);
        Mesh ma = new Mesh().cylinder(Vec3d.ZERO, new Vec3d(1, 0, 0), 0.3, 0.25, 16, RED).box(new Vec3d(-0.1, -0.4, -0.4), new Vec3d(0.1, 0.4, 0.4), GLASS);
        same(new LegacyMesh().add(la, TURN, new Vec3d(2, 0, -1)), new Mesh().add(ma, TURN, new Vec3d(2, 0, -1)));
        Vec3d point = new Vec3d(0.4, 0.05, 0), normal = new Vec3d(0.2, 0.1, 1).normalized();
        same(la.clipped(point, normal), ma.clipped(point, normal));
        double[] along = {0, 0, 0.5, 1, 1}, radius = {0, 0.2, 0.25, 0.1, 0};
        int[] colors = {RED, RED, BLUE, RED, RED};
        LegacyMesh lb = new LegacyMesh().lathe(Vec3d.ZERO, Vec3d.Y, along, radius, colors, 24, RED, false);
        Mesh mb = new Mesh().lathe(Vec3d.ZERO, Vec3d.Y, along, radius, colors, 24, RED, false);
        same(lb.clipped(Vec3d.ZERO, Vec3d.Z), mb.clipped(Vec3d.ZERO, Vec3d.Z));
        same(lb.clipped(new Vec3d(0.05, 0, 0), new Vec3d(1, 0.3, 0).normalized()), mb.clipped(new Vec3d(0.05, 0, 0), new Vec3d(1, 0.3, 0).normalized()));
    }
}
