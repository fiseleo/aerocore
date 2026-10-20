package dev.mcmod.aerocore.api.ground;

import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

/** Ground rays (0.24): floors, slopes, boxes and steps against hand geometry. */
class GroundTest {
    private static final Vec3d DOWN = Vec3d.Y.scale(-1);
    private static final GroundContact.Surface TARMAC = GroundContact.Surface.TARMAC, ICE = GroundContact.Surface.ICE;

    private static void assertVec(Vec3d expected, Vec3d actual) { assertEquals(0, expected.subtract(actual).length(), 1e-12, expected + " vs " + actual); }

    @Test void aFloorIsHitFromAboveOnly() {
        Ground floor = Ground.flat(0.5, TARMAC);
        Ground.Hit hit = floor.cast(new Vec3d(3, 2, -1), DOWN, 5);
        assertEquals(1.5, hit.distanceM(), 1e-12);
        assertVec(new Vec3d(3, 0.5, -1), hit.point());
        assertVec(Vec3d.Y, hit.normal());
        assertSame(TARMAC, hit.surface());
        assertNull(floor.cast(new Vec3d(3, 2, -1), Vec3d.Y, 5), "looking up");
        assertNull(floor.cast(new Vec3d(3, 2, -1), DOWN, 1.4), "beyond reach");
        Ground.Hit buried = floor.cast(new Vec3d(0, 0.2, 0), DOWN, 5);
        assertEquals(0, buried.distanceM(), 0, "a ray from inside hits at once");
        assertVec(new Vec3d(0, 0.2, 0), buried.point());
    }

    @Test void aSlopeIsHitWhereItsPlaneIs() {
        double a = Math.toRadians(30);
        // rising toward -x: the normal leans toward +x
        Ground hill = Ground.plane(Vec3d.ZERO, new Vec3d(Math.sin(a), Math.cos(a), 0), GroundContact.Surface.SOIL);
        Ground.Hit hit = hill.cast(new Vec3d(-2, 5, 0), DOWN, 10);
        assertEquals(2 * Math.tan(a), hit.point().y(), 1e-12, "height of the slope 2 m uphill");
        assertEquals(Math.toDegrees(a), Math.toDegrees(Math.acos(hit.normal().y())), 1e-9, "slope angle from the normal");
        // a ray along the slope's normal comes in square
        Vec3d n = new Vec3d(Math.sin(a), Math.cos(a), 0);
        Ground.Hit square = hill.cast(n.scale(3), n.scale(-1), 10);
        assertEquals(3, square.distanceM(), 1e-12);
    }

    @Test void aSplitFloorHasTwoSurfaces() {
        Ground split = Ground.split(0, ICE, TARMAC);
        assertSame(ICE, split.cast(new Vec3d(0, 1, -0.8), DOWN, 2).surface());
        assertSame(TARMAC, split.cast(new Vec3d(0, 1, 0.8), DOWN, 2).surface());
    }

    @Test void boxesAreHitOnTheFaceTheRayEnters() {
        Ground step = Ground.boxes(List.of(new BoxGround.Box(new Vec3d(1, 0, -2), new Vec3d(3, 0.15, 2), GroundContact.Surface.GRAVEL)));
        Ground.Hit top = step.cast(new Vec3d(2, 1, 0), DOWN, 2);
        assertEquals(0.85, top.distanceM(), 1e-12);
        assertVec(Vec3d.Y, top.normal());
        Ground.Hit face = step.cast(new Vec3d(0, 0.1, 0), Vec3d.X, 2);
        assertEquals(1, face.distanceM(), 1e-12, "the kerb's face");
        assertVec(Vec3d.X.scale(-1), face.normal());
        assertNull(step.cast(new Vec3d(0, 0.2, 0), Vec3d.X, 5), "passing over the kerb");
        assertNull(step.cast(new Vec3d(2, 1, 3), DOWN, 5), "beside it");
        // from inside, the nearest face: 0.05 below the top, nearer than any side
        Ground.Hit inside = step.cast(new Vec3d(2, 0.1, 0), DOWN, 5);
        assertEquals(0, inside.distanceM(), 0);
        assertVec(Vec3d.Y, inside.normal());
    }

    @Test void stairsRiseStepByStepAndTheNearestGroundWins() {
        List<BoxGround.Box> stairs = BoxGround.stairs(0, 0, -1, 1, 1.0 / 16, 1.0 / 16, 16, TARMAC);
        assertEquals(16, stairs.size());
        Ground ramp = Ground.nearest(Ground.flat(0, ICE), Ground.boxes(stairs));
        List<Double> heights = new ArrayList<>();
        for (int i = 0; i < 18; i++) {
            Ground.Hit hit = ramp.cast(new Vec3d((i + 0.5) / 16, 2, 0), DOWN, 3);
            heights.add(hit.point().y());
            assertSame(i < 16 ? TARMAC : ICE, hit.surface(), "step " + i);
        }
        for (int i = 0; i < 16; i++) { assertEquals((i + 1) / 16.0, heights.get(i), 1e-12, "step " + i); }
        assertEquals(0, heights.get(16), 1e-12, "the floor beyond");
        assertNull(Ground.NONE.cast(Vec3d.ZERO, DOWN, 100));
    }
}
