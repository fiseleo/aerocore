package dev.mcmod.aerocore.api.ground;

import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

/** 0.42: a block whose top is a plane (a road ramp) as rays meet it. */
class SlopedCellTest {
    private static final GroundContact.Surface ROAD = GroundContact.Surface.TARMAC;
    /** The second part of a 1:4 ramp rising toward +x: from a quarter to a half of a block across its metre. */
    private static final SlopedCell PART = new SlopedCell(10, 64, -3, 0.25, 0.25, 0);

    @Test void aRayFromAboveMeetsTheTiltedTop() {
        Ground.Hit hit = PART.cast(new Vec3d(10.5, 66, -2.5), new Vec3d(0, -1, 0), 5, ROAD);
        assertNotNull(hit);
        // the top is at 64 + 0.25 + 0.25 x 0.5 over the middle
        assertEquals(64.375, hit.point().y(), 1e-12);
        assertEquals(66 - 64.375, hit.distanceM(), 1e-12);
        // the normal leans downhill by the slope: atan(1/4)
        Vec3d n = hit.normal();
        assertEquals(-Math.sin(Math.atan(0.25)), n.x(), 1e-12);
        assertEquals(Math.cos(Math.atan(0.25)), n.y(), 1e-12);
        assertEquals(0, n.z(), 0);
        assertEquals(64.25, PART.height(10, -3), 1e-12);
        assertEquals(64.5, PART.height(11, -2), 1e-12);
    }

    @Test void theTopEndsAtTheFootprintAndTheReach() {
        // beside the block, and beyond the ray's length
        assertNull(PART.cast(new Vec3d(11.5, 66, -2.5), new Vec3d(0, -1, 0), 5, ROAD));
        assertNull(PART.cast(new Vec3d(10.5, 66, -3.5), new Vec3d(0, -1, 0), 5, ROAD));
        assertNull(PART.cast(new Vec3d(10.5, 66, -2.5), new Vec3d(0, -1, 0), 1.5, ROAD));
        // a ray going up or along the slope never meets it from above
        assertNull(PART.cast(new Vec3d(10.5, 66, -2.5), new Vec3d(0, 1, 0), 5, ROAD));
        // the shared edge of two parts belongs to both: no gap between ramp blocks
        SlopedCell next = new SlopedCell(11, 64, -3, 0.5, 0.25, 0);
        Ground.Hit a = PART.cast(new Vec3d(11, 66, -2.5), new Vec3d(0, -1, 0), 5, ROAD), b = next.cast(new Vec3d(11, 66, -2.5), new Vec3d(0, -1, 0), 5, ROAD);
        assertEquals(a.point().y(), b.point().y(), 1e-12);
        // a slanting ray that would cross the plane outside the cell
        assertNull(PART.cast(new Vec3d(8, 66, -2.5), new Vec3d(-1, -1, 0).normalized(), 9, ROAD));
    }

    @Test void aRayThatStartsInTheWedgeHitsAtOnce() {
        Ground.Hit in = PART.cast(new Vec3d(10.5, 64.1, -2.5), new Vec3d(1, 0, 0), 5, ROAD);
        assertNotNull(in);
        assertEquals(0, in.distanceM(), 0);
        // under the plane but beside the block, or under its floor: not this cell's
        assertNull(PART.cast(new Vec3d(9.5, 64.1, -2.5), new Vec3d(1, 0, 0), 5, ROAD));
        assertNull(PART.cast(new Vec3d(10.5, 63.5, -2.5), new Vec3d(0, 1, 0), 5, ROAD));
    }

    @Test void aSlopeAlongZAndDownhill() {
        // rising toward -z: a quarter of a block at z0 + 1, a half at z0
        SlopedCell north = new SlopedCell(0, 0, 0, 0.5, 0, -0.25);
        assertEquals(0.5, north.height(0.5, 0), 1e-12);
        assertEquals(0.25, north.height(0.5, 1), 1e-12);
        Ground.Hit hit = north.cast(new Vec3d(0.5, 2, 0.5), new Vec3d(0, -1, 0), 5, ROAD);
        assertEquals(0.375, hit.point().y(), 1e-12);
        assertEquals(Math.sin(Math.atan(0.25)), hit.normal().z(), 1e-12);
    }
}
