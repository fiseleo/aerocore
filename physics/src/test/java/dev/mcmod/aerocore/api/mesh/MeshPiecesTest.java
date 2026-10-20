package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

/** 0.30: cabins behind glazing, far variants, moving pieces and lamps in occlusion and baking. */
class MeshPiecesTest {
    private static final int GREY = 0xff808080, GLASS = 0x5a283846;
    private static final AmbientOcclusion.Settings SETTINGS = AmbientOcclusion.Settings.forSize(2);

    /** A closed 2 m box with its +z face taken out (a window opening). */
    private static Mesh shellWithOpening() {
        Mesh m = new Mesh().box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY);
        m.quads().removeIf(q -> q.normal().z() > 0.99);
        return m;
    }

    @Test void aSliverQuadHasNoNormalAndDoesNotStopTheBake() {
        // a quad whose diagonals' cross product is 1e-13 long: under Vec3d.normalized's limit, over the old 1e-30
        // threshold on its square, it threw and left the whole part mesh empty (seen on hdfv's cut-away bodies)
        Mesh m = new Mesh().box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY);
        m.quad(new Vec3d(0, 1.2, 0), new Vec3d(1e-7, 1.2, 0), new Vec3d(1e-7, 1.2, 1e-6), new Vec3d(0, 1.2, 1e-6), GREY, Vec3d.Y);
        assertEquals(Vec3d.ZERO, MeshOps.faceNormal(m.quads().get(m.quads().size() - 1)));
        assertTrue(MeshBaker.levels(m, null, SETTINGS, 1, true)[0].quads() >= 6, "the box is baked");
        assertEquals(7, MeshOps.smooth(m, 40).quads().size(), "and smoothed");
    }

    @Test void aCabinBehindGlassIsSeenAndBehindAnOpaquePaneIsShut() {
        Mesh glazed = shellWithOpening().clear(true).quad(new Vec3d(-1, -1, 1), new Vec3d(1, -1, 1), new Vec3d(1, 1, 1), new Vec3d(-1, 1, 1), GLASS, Vec3d.Z)
                .clear(false);
        int before = glazed.quads().size();
        glazed.box(new Vec3d(-0.3, -0.9, -0.3), new Vec3d(0.3, -0.4, 0.3), GREY);   // a seat inside (off the floor: the shell is its only skin)
        AmbientOcclusion.Result r = AmbientOcclusion.compute(glazed.quads(), SETTINGS);
        for (int i = before; i < glazed.quads().size(); i++) { assertFalse(r.enclosed()[i], "the seat shows through the glass: " + i); }
        // the same pane opaque: the seat is shut in
        Mesh shut = shellWithOpening().quad(new Vec3d(-1, -1, 1), new Vec3d(1, -1, 1), new Vec3d(1, 1, 1), new Vec3d(-1, 1, 1), GREY, Vec3d.Z);
        shut.box(new Vec3d(-0.3, -0.9, -0.3), new Vec3d(0.3, -0.4, 0.3), GREY);
        AmbientOcclusion.Result s = AmbientOcclusion.compute(shut.quads(), SETTINGS);
        for (int i = before; i < shut.quads().size(); i++) { assertTrue(s.enclosed()[i], "behind an opaque pane the seat is enclosed: " + i); }
    }

    @Test void glassCoversNothingButAnOpaquePlateDoes() {
        Mesh frame = new Mesh().quad(new Vec3d(-0.2, -0.2, 0), new Vec3d(0.2, -0.2, 0), new Vec3d(0.2, 0.2, 0), new Vec3d(-0.2, 0.2, 0), GREY, Vec3d.Z);
        Mesh glazed = new Mesh(); glazed.quads().addAll(frame.quads());
        glazed.clear(true).quad(new Vec3d(-0.3, -0.3, 0.001), new Vec3d(0.3, -0.3, 0.001), new Vec3d(0.3, 0.3, 0.001), new Vec3d(-0.3, 0.3, 0.001), GLASS, Vec3d.Z);
        assertFalse(AmbientOcclusion.compute(glazed.quads(), SETTINGS).enclosed()[0], "a pane a millimetre over a frame leaves it drawn");
        Mesh plated = new Mesh(); plated.quads().addAll(frame.quads());
        plated.quad(new Vec3d(-0.3, -0.3, 0.001), new Vec3d(0.3, -0.3, 0.001), new Vec3d(0.3, 0.3, 0.001), new Vec3d(-0.3, 0.3, 0.001), GREY, Vec3d.Z);
        assertTrue(AmbientOcclusion.compute(plated.quads(), SETTINGS).enclosed()[0], "an opaque plate a millimetre over it covers it");
    }

    @Test void aCabinIsKeptNearestAndLeftOutFartherWhereGlassTurnsOpaque() {
        // a closed shell (the windows drawn shut): what is in it would be culled, but a cabin is kept at level 0
        Mesh m = new Mesh().box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY)
                .cabin(true).box(new Vec3d(-0.3, -1, -0.3), new Vec3d(0.3, -0.4, 0.3), GREY).cabin(false)
                .clear(true).quad(new Vec3d(-0.5, -0.5, 1.002), new Vec3d(0.5, -0.5, 1.002), new Vec3d(0.5, 0.5, 1.002), new Vec3d(-0.5, 0.5, 1.002), GLASS, Vec3d.Z)
                .clear(false);
        PackedMesh[] levels = MeshBaker.levels(m, null, SETTINGS, 2, true);
        // level 0 splits quads where shade changes, so look for the cabin's top (y -0.4) rather than count
        boolean cabinTop = false;
        float[] pos = levels[0].positions();
        for (int v = 0; v < pos.length / 3; v++) { cabinTop |= Math.abs(pos[3 * v + 1] + 0.4) < 1e-6 && Math.abs(pos[3 * v]) <= 0.3 + 1e-6; }
        assertTrue(cabinTop, "the cabin is kept at the nearest level though shut in");
        Mesh untagged = new Mesh().box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY).box(new Vec3d(-0.3, -1, -0.3), new Vec3d(0.3, -0.4, 0.3), GREY);
        float[] up = MeshBaker.levels(untagged, null, SETTINGS, 0, true)[0].positions();
        for (int v = 0; v < up.length / 3; v++) { assertFalse(Math.abs(up[3 * v + 1] + 0.4) < 1e-6, "untagged, the same box is culled"); }
        assertEquals(6 + 1, levels[1].quads(), "no cabin farther");
        boolean translucent0 = false, solidGlass1 = false;
        for (PackedMesh.Group g : levels[0].groups()) { translucent0 |= g.pass() == PackedMesh.Pass.TRANSLUCENT; }
        for (PackedMesh.Group g : levels[1].groups()) { assertNotEquals(PackedMesh.Pass.TRANSLUCENT, g.pass(), "glass is opaque where the cabin is left out"); }
        for (int v = 0; v < 4 * levels[1].quads(); v++) { solidGlass1 |= levels[1].colors()[v] == (0xff000000 | (0x28 * 11 / 20 << 16) | (0x38 * 11 / 20 << 8) | (0x46 * 11 / 20)); }
        assertTrue(translucent0, "glass is translucent at the nearest level");
        assertTrue(solidGlass1, "and darker, opaque, farther");
    }

    @Test void aFarVariantAndTheNearOneDoNotHideEachOther() {
        // a cut hull (only the nearest level) and an uncut one a millimetre out (from level 1): shaded apart, neither is
        // taken for the other's cover
        Mesh m = new Mesh().levels(0, 0).box(new Vec3d(-1, -1, -1), new Vec3d(1, 1, 1), GREY)
                .levels(1, 3).box(new Vec3d(-1.001, -1.001, -1.001), new Vec3d(1.001, 1.001, 1.001), GREY).levels(0, 3);
        PackedMesh[] levels = MeshBaker.levels(m, null, SETTINGS, 2, true);
        assertEquals(6, levels[0].quads(), "the near hull alone at level 0");
        assertEquals(6, levels[1].quads(), "the far one alone at level 1");
        assertEquals(6, levels[2].quads());
        // a quad's range: levels(1, 2) is drawn at 1 and 2 only
        MeshBuilder.Quad q = new Mesh().levels(1, 2).box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY).quads().get(0);
        assertEquals(1, q.near());
        assertEquals(1, q.detail());
    }

    @Test void piecesAddUpToTheWholeWithTheSameShade() {
        Mesh m = new Mesh().box(new Vec3d(-1, -0.1, -1), new Vec3d(1, 0, 1), GREY)
                .piece(1).box(new Vec3d(-0.2, 0, -0.2), new Vec3d(0.2, 0.4, 0.2), GREY)
                .piece(2).cylinder(new Vec3d(0.6, 0, 0), new Vec3d(0.6, 0.3, 0), 0.1, 0.1, 12, GREY).piece(0);
        assertTrue(MeshBaker.hasPieces(m));
        PackedMesh[] whole = MeshBaker.levels(m, null, SETTINGS, 2, true);
        PackedMesh[][] pieces = MeshBaker.pieces(m, null, SETTINGS, 2, true);
        assertEquals(3, pieces.length);
        for (int level = 0; level <= 2; level++) {
            int quads = 0;
            long light = 0, wholeLight = 0;
            for (PackedMesh[] p : pieces) {
                quads += p[level].quads();
                for (int v = 0; v < 4 * p[level].quads(); v++) { light += p[level].light(v); }
            }
            for (int v = 0; v < 4 * whole[level].quads(); v++) { wholeLight += whole[level].light(v); }
            assertEquals(whole[level].quads(), quads, "level " + level);
            assertEquals(wholeLight, light, "the same shade at level " + level);
            assertTrue(pieces[1][level].quads() > 0 && pieces[2][level].quads() > 0);
        }
        assertFalse(MeshBaker.hasPieces(new Mesh().box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY)));
    }

    @Test void lampsTravelWithTheirMesh() {
        Mesh light = new Mesh().addLamp(Lamp.point(1, 0, new Vec3d(1, 0, 0.5), Vec3d.X, 0.05, 0xffff2020, 110, 3));
        Mesh car = new Mesh().add(light, dev.mcmod.aerocore.api.Orientation.rotationVector(Vec3d.Y.scale(Math.PI / 2)), new Vec3d(0, 1, 0));
        Lamp moved = car.lamps().get(0);
        assertEquals(0.5, moved.at().x(), 1e-12);
        assertEquals(1, moved.at().y(), 1e-12);
        assertEquals(-1, moved.at().z(), 1e-12);
        assertEquals(-1, moved.axis().z(), 1e-12, "the axis turned too");
        assertEquals(0.5 * -1, MeshOps.mirror(light, 2).lamps().get(0).at().z(), 1e-12, "mirrored");
        Lamp placed = MeshOps.placed(light, new Vec3d(2, 2, 2), dev.mcmod.aerocore.api.Orientation.IDENTITY, Vec3d.ZERO).lamps().get(0);
        assertEquals(2, placed.at().x(), 1e-12);
        assertEquals(0.1, placed.radiusM(), 1e-12, "sized with the mesh");
        assertEquals(1, light.clipped(Vec3d.ZERO, Vec3d.Y).lamps().size(), "kept by a cut");
        assertEquals(1, Mesh.of(light).lamps().size());
        assertThrows(IllegalArgumentException.class, () -> Lamp.point(255, 0, Vec3d.ZERO, Vec3d.X, 0.1, 0, 360, 0));
    }

    @Test void lampsGetGroupsOfTheirOwn() {
        Mesh m = new Mesh().box(Vec3d.ZERO, new Vec3d(1, 1, 1), GREY)
                .lamp(3).box(new Vec3d(1, 0.4, 0.4), new Vec3d(1.05, 0.6, 0.6), 0xfffff4d0)
                .lamp(5).box(new Vec3d(-0.05, 0.4, 0.4), new Vec3d(0, 0.6, 0.6), 0xffd02020).lamp(-1);
        assertEquals(-1, m.quads().get(0).lamp());
        assertEquals(3, m.quads().get(6).lamp());
        assertEquals(5, m.quads().get(12).lamp());
        PackedMesh baked = MeshBaker.bake(m, null);
        Set<Integer> lamps = new HashSet<>();
        int total = 0;
        for (PackedMesh.Group g : baked.groups()) { lamps.add(g.lamp()); total += g.count(); }
        assertEquals(Set.of(-1, 3, 5), lamps);
        assertEquals(18, total);
        assertThrows(IllegalArgumentException.class, () -> new Mesh().lamp(255));
    }
}
