package dev.mcmod.aerocore.api.body;

import dev.mcmod.aerocore.api.SlenderHull;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Bodies from modules against shapes with closed-form measures: an ellipsoid, a cylinder, a stepped vee hull. */
class BodyShapeTest {
    private static final double R = 0.8, A = 3;

    /** A prolate ellipsoid of semi-axes A (length) and R: two elliptical modules from a point to a circle and back. */
    private static BodyShape ellipsoid() {
        SectionShape circle = SectionShape.Rounded.ellipse(2 * R, 2 * R, 0);
        return new BodyShape(circle.scaled(1e-3), List.of(
                new BodyModule("front", A, circle, 2, 0, Double.POSITIVE_INFINITY),
                new BodyModule("rear", A, circle.scaled(1e-3), 2, 0, Double.POSITIVE_INFINITY)));
    }

    @Test void anEllipsoidHasItsVolumeAndArea() {
        BodyShape.Particulars p = ellipsoid().particulars();
        double volume = 4.0 / 3 * Math.PI * R * R * A;
        // a prolate spheroid's surface: 2 pi R^2 (1 + (A / (R e)) asin e), e = sqrt(1 - R^2 / A^2)
        double e = Math.sqrt(1 - R * R / (A * A)), area = 2 * Math.PI * R * R * (1 + A / (R * e) * Math.asin(e));
        assertEquals(volume, p.volumeM3(), 0.02 * volume, "volume");
        assertEquals(area, p.wettedAreaM2(), 0.02 * area, "surface");
        assertEquals(A, p.centroidXM(), 1e-3, "centroid at the middle");
        assertEquals(R, p.centroidYM(), 1e-3, "on the axis");
        assertEquals(Math.PI * R * R, p.frontalAreaM2(), 0.01 * Math.PI * R * R, "frontal area");
        assertEquals(2 * A / (2 * R), p.finenessRatio(), 0.02 * A / R, "fineness");
        assertEquals(0, p.baseAreaM2(), 1e-4, "no blunt base");
        assertEquals(0, p.upsweepDeg(), 1e-6);
    }

    @Test void aCylinderIsExactUpToItsPolygon() {
        SectionShape circle = SectionShape.Rounded.ellipse(2 * R, 2 * R, 0);
        BodyShape b = new BodyShape(circle, List.of(BodyModule.straight("tube", 5, circle)));
        BodyShape.Particulars p = b.particulars();
        double polygon = SectionShape.POINTS / 2.0 * Math.sin(2 * Math.PI / SectionShape.POINTS) * R * R;   // inscribed polygon's area
        // (outline corners are snapped to a nanometre: SectionShape.convex)
        assertEquals(polygon * 5, p.volumeM3(), 1e-6);
        assertEquals(polygon, p.baseAreaM2(), 1e-6, "a cut-off tube has a blunt base");
        assertEquals(2 * Math.PI * R * 5, p.wettedAreaM2(), 0.01 * 2 * Math.PI * R * 5);
        assertEquals(2 * R * 5, p.planformAreaM2(), 0.01);
    }

    @Test void aStepRaisesTheKeelAndHullSectionsStayValid() {
        SectionShape vee = new SectionShape.Chined(1.1, 0.8, 0, 25, 5);
        BodyShape b = new BodyShape(vee.scaled(0.05), List.of(
                new BodyModule("bow", 1.5, vee, 2, 0, Double.POSITIVE_INFINITY),
                BodyModule.straight("forebody", 2.5, vee),
                new BodyModule("afterbody", 3, new SectionShape.Chined(0.8, 0.6, 0.2, 18, 5), 1, 0.06, Double.POSITIVE_INFINITY)));
        assertEquals(7, b.lengthM(), 1e-12);
        double step = b.moduleFrontX(2);
        assertEquals(0, b.bottomAt(step + 0.01), 1e-9, "the forebody's keel");
        assertEquals(0.06, b.bottomAt(step - 0.01), 0.005, "behind the step the keel is 6 cm higher");
        SlenderHull hull = b.hull();   // every station a convex, counter-clockwise hull section
        assertTrue(hull.sections().size() > 10);
        for (int i = 1; i < b.stations().size(); i++) { assertTrue(b.stations().get(i).xM() > b.stations().get(i - 1).xM()); }
    }

    @Test void sectionsBlendAcrossFamiliesAndATailSweepsUp() {
        SectionShape nose = new SectionShape.Rounded(1.2, 1.4, 0.2, 2.5, 3);
        SectionShape cabin = new SectionShape.Cabin(1.6, 1.5, 0, 0.55, 0.75, 0.15);
        SectionShape tail = SectionShape.Rounded.ellipse(0.3, 0.3, 1.1);
        BodyShape b = new BodyShape(nose.scaled(0.2), List.of(
                new BodyModule("nose", 1.6, nose, 2, 0, Double.POSITIVE_INFINITY),
                BodyModule.straight("cabin", 3, cabin),
                new BodyModule("tail cone", 3.5, tail, 1.3, 0, Double.POSITIVE_INFINITY)));
        b.hull();
        BodyShape.Particulars p = b.particulars();
        assertTrue(p.upsweepDeg() > 5, "the tail cone's end is above the cabin's centre: " + p.upsweepDeg());
        // its top line falls from the roof's trailing edge (1.5 m) to the tail's (1.4 m) over the tail cone's 3.5 m, a little
        // steeper: the blend keeps the roof's height a short way into the cone
        assertTrue(p.rearSlantDeg() > Math.toDegrees(Math.atan2(0.1, 3.5)) - 0.01 && p.rearSlantDeg() < 1.15 * Math.toDegrees(Math.atan2(0.1, 3.5)), "slant " + p.rearSlantDeg());
        assertEquals(1.6, p.maxWidthM(), 1e-6);
        // the blend half way through the tail cone lies between the two sections
        double mid = b.moduleRearX(2) + 1.75;
        assertTrue(b.widthAt(mid) < 1.6 && b.widthAt(mid) > 0.3);
    }

    // ---------------------------------------------------------------- fitting (0.32)

    /** A box-section body 4 m long, 1.6 wide, 1 high from y 0.2 (square corners, belt at the top). */
    private static BodyShape box() {
        SectionShape s = new SectionShape.Cabin(1.6, 1.0, 0.2, 1.0, 1.0, 0);
        return new BodyShape(s, List.of(BodyModule.straight("box", 4, s)));
    }

    private static BodyShape.Overhang at(BodyShape b, double x0, double y0, double z0, double x1, double y1, double z1) {
        return b.overhang(new dev.mcmod.aerocore.api.Vec3d(x0, y0, z0), new dev.mcmod.aerocore.api.Vec3d(x1, y1, z1), dev.mcmod.aerocore.api.Orientation.IDENTITY,
                dev.mcmod.aerocore.api.Vec3d.ZERO, 0.05);
    }

    @Test void aBoxInsideFitsAndOneOutsideSaysWhere() {
        BodyShape b = box();
        assertEquals(BodyShape.Overhang.NONE, at(b, 1, 0.3, -0.5, 3, 1.1, 0.5), "inside");
        BodyShape.Overhang up = at(b, 1, 0.3, -0.5, 3, 1.25, 0.5);
        assertEquals(0.05, up.topM(), 1e-9, "5 cm through the top");
        assertEquals("top", up.worst());
        BodyShape.Overhang side = at(b, 1, 0.3, -0.83, 3, 1.0, 0.5);
        assertEquals(0.03, side.portM(), 1e-9, "3 cm out to port");
        assertEquals(0, side.starboardM(), 1e-12);
        assertEquals(0.2, at(b, 3.5, 0.3, -0.2, 4.2, 0.8, 0.2).frontM(), 1e-9, "past the nose");
        assertEquals(0.1, at(b, -0.1, 0.3, -0.2, 0.5, 0.8, 0.2).rearM(), 1e-9, "past the tail");
        assertEquals(BodyShape.Overhang.NONE, at(b, 1, -0.3, -0.5, 3, 0.9, 0.5), "below the bottom is open");
    }

    @Test void theOutlineBetweenStationsIsContinuous() {
        BodyShape b = ellipsoid();
        for (BodyShape.Station s : b.stations()) {
            List<double[]> o = b.outlineAt(s.xM());
            double top = Double.NEGATIVE_INFINITY;
            for (double[] q : o) { top = Math.max(top, q[0]); }
            assertEquals(s.topM(), top, 1e-9, "at a station its own outline");
        }
        double x = b.moduleRearX(0), dt = 1e-4;
        double before = 0, after = 0;
        for (double[] q : b.outlineAt(x - dt)) { before = Math.max(before, q[0]); }
        for (double[] q : b.outlineAt(x + dt)) { after = Math.max(after, q[0]); }
        assertEquals(before, after, 1e-3, "no jump at the module joint");
    }

    /** High, low, high: a box spanning the low part at the high parts' level has its corners inside but not its middle. */
    @Test void aDipBetweenTheCornersIsFound() {
        SectionShape high = new SectionShape.Cabin(1.6, 1.0, 0.2, 1.0, 1.0, 0), low = new SectionShape.Cabin(1.6, 0.6, 0.2, 1.0, 1.0, 0);
        BodyShape b = new BodyShape(high, List.of(new BodyModule("a", 1, low, 1, 0, Double.POSITIVE_INFINITY), new BodyModule("b", 1, high, 1, 0, Double.POSITIVE_INFINITY)));
        BodyShape.Overhang dip = at(b, 0.05, 0.3, -0.2, 1.95, 1.1, 0.2);
        assertTrue(dip.topM() > 0.25, "through the low middle: " + dip);
    }

    @Test void aTurnedBoxIsTurned() {
        BodyShape b = box();
        // 1.2 m long along its own x, 0.3 wide: across the body it sticks out of neither side; along it, fine
        dev.mcmod.aerocore.api.Vec3d lo = new dev.mcmod.aerocore.api.Vec3d(-0.6, 0, -0.15), hi = new dev.mcmod.aerocore.api.Vec3d(0.6, 0.3, 0.15);
        dev.mcmod.aerocore.api.Vec3d centre = new dev.mcmod.aerocore.api.Vec3d(2, 0.5, 0);
        assertEquals(BodyShape.Overhang.NONE, b.overhang(lo, hi, dev.mcmod.aerocore.api.Orientation.IDENTITY, centre, 0.05));
        BodyShape.Overhang turned = b.overhang(lo.scale(1.5), hi.scale(1.5), dev.mcmod.aerocore.api.Orientation.rotationVector(dev.mcmod.aerocore.api.Vec3d.Y.scale(Math.PI / 2)), centre, 0.05);
        assertEquals(0.1, turned.portM(), 1e-6, "0.9 m each side across a 0.8 m half-width");
        assertEquals(0.1, turned.starboardM(), 1e-6);
    }
}
