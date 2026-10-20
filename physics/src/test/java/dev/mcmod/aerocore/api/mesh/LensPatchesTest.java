package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Where a lens-only lamp's halos go (0.39): its lens quads gathered into patches. */
class LensPatchesTest {
    private static final int WARM = 0xffffd9a0;

    private static Mesh window(Mesh m, double x, double z, double w, double h) {
        // a window facing -z at z, from x to x + w, 1 m up, h tall
        return m.quad(new Vec3d(x, 1, z), new Vec3d(x, 1 + h, z), new Vec3d(x + w, 1 + h, z), new Vec3d(x + w, 1, z), WARM, new Vec3d(0, 0, -1));
    }

    private static List<LensPatches.Patch> patches(Mesh m, int lamp) {
        PackedMesh baked = MeshBaker.levels(m, null, null, 0, false)[0];
        return LensPatches.of(baked, lamp);
    }

    @Test void twoWindowsApartAreTwoPatchesAndAMullionsTwoPanesOne() {
        Mesh m = new Mesh();
        m.lamp(3);
        window(m, 0, 0, 0.4, 0.4);
        window(m, 2, 0, 0.4, 0.4);
        // one window of two panes either side of a 5 cm mullion
        window(m, 5, 0, 0.2, 0.4);
        window(m, 5.25, 0, 0.2, 0.4);
        m.lamp(-1);
        window(m, 8, 0, 0.4, 0.4);   // not a lens
        List<LensPatches.Patch> p = patches(m, 3);
        assertEquals(3, p.size(), "two windows and one two-pane window: " + p);
        LensPatches.Patch first = p.stream().filter(x -> x.x() < 1).findFirst().orElseThrow();
        assertEquals(0.2, first.x(), 1e-6);
        assertEquals(1.2, first.y(), 1e-6);
        assertEquals(Math.sqrt(0.16 / Math.PI), first.radiusM(), 1e-6, "the radius of the circle of its area");
        assertEquals(-1, first.nz(), 1e-6, "facing out");
        assertTrue(patches(m, 7).isEmpty(), "no quads of that lamp, no patches");
    }

    @Test void aLongStripGivesARowOfPatches() {
        Mesh m = new Mesh();
        m.lamp(1);
        window(m, 0, 0, 0.7, 6);   // a stair window up a tower: 0.7 m wide, 6 m tall
        List<LensPatches.Patch> p = patches(m, 1);
        assertEquals(10, p.size(), "one about every 0.6 m");
        double area = 0;
        for (LensPatches.Patch x : p) {
            assertEquals(0.35, x.x(), 1e-6);
            area += Math.PI * x.radiusM() * x.radiusM();
        }
        assertEquals(4.2, area, 1e-6, "the strip's area shared out");
    }
}
