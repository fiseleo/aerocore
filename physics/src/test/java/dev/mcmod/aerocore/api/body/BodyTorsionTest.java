package dev.mcmod.aerocore.api.body;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/** A body's torsion against Bredt's closed thin-walled cell, framed openings and a real car body's stiffness. */
class BodyTorsionTest {
    private static final double INF = Double.POSITIVE_INFINITY;

    /** Aluminium skin with stringers smeared in, frames of 1.2 cm^2, openings as given. */
    private static BodyStructure.Module alu(BodyStructure.Build build, double skin, double stringers, int doors, double doorWidth, double doorHeight, double windows) {
        return new BodyStructure.Module(build, 2780, 345e6, 73.1e9, skin, stringers, 0.5, 1.2e-4, 0, doors, doorWidth, doorHeight, windows, 0);
    }

    /** A family car's steel (STEEL_1045's density, yield and modulus), 0.8 mm skin, 20 % stringers, frames of 2 cm^2 every 0.5 m. */
    private static BodyStructure.Module steel(int doors, double doorWidth, double doorHeight, double windows, double floor) {
        return new BodyStructure.Module(BodyStructure.Build.MONOCOQUE, 7850, 310e6, 205e9, 0.0008, 0.2, 0.5, 2e-4, floor, doors, doorWidth, doorHeight, windows, 0);
    }

    private static BodyShape prism(SectionShape section, double length) {
        return new BodyShape(section, List.of(BodyModule.straight("tube", length, section)));
    }

    /**
     * A notchback 4.85 m long, 1.82 m wide, 1.45 m high (hdfv's sample sedan): the hood, the windshield's rise, the
     * four-door cabin under its flat roof, the rear window down to the trunk's deck, the trunk.
     */
    static BodyShape sedan() {
        SectionShape hood = new SectionShape.Cabin(1.8, 0.78, 0.15, 1.0, 0.9, 0.22);
        SectionShape cell = new SectionShape.Cabin(1.82, 1.3, 0.15, 0.6, 0.72, 0.15);
        SectionShape rear = new SectionShape.Cabin(1.78, 0.95, 0.18, 0.95, 0.85, 0.13);
        SectionShape trunk = new SectionShape.Cabin(1.74, 0.9, 0.2, 1.0, 0.85, 0.2);
        return new BodyShape(hood.scaled(0.55), List.of(
                new BodyModule("hood", 1.0, hood, 1.8, 0, INF),
                new BodyModule("windshield", 0.8, cell, 1.4, 0, INF),
                new BodyModule("cabin", 1.65, cell, 1.0, 0, INF),
                new BodyModule("rear window", 0.6, rear, 1.0, 0, INF),
                new BodyModule("trunk", 0.8, trunk, 1.0, 0, INF)));
    }

    /** The sedan's structure: glazed windshield, cabin (four doors 0.8 by 1.05 m) and rear window, closed hood and trunk. */
    static List<BodyStructure.Module> sedanStructure() {
        return List.of(steel(0, 0.9, 1.1, 0, 0), steel(0, 0.95, 1.05, 0.35, 15), steel(4, 0.8, 1.05, 0.35, 15),
                steel(0, 0.95, 1.05, 0.35, 15), steel(0, 0.9, 1.0, 0, 10));
    }

    private static double perDegreeKNm(double nmPerRad) { return nmPerRad * Math.PI / 180 / 1000; }

    @Test void closedTubesMatchBredt() {
        // the shear flow runs in the skin: stringers (here 25 %) carry none of it
        double t = 0.0016, stringers = 0.25;
        BodyStructure.Module plain = alu(BodyStructure.Build.SEMI_MONOCOQUE, t, stringers, 0, 0.9, 1.1, 0);
        double g = BodyStructure.shearModulusPa(73.1e9);
        assertEquals(73.1e9 / 2.6, g, 1e-6);
        // a round tube (the 32-gon): Bredt's 4 A^2 t / P on the polygon, and within 1.5 % of the circle's 2 pi r^3 t
        double r = 0.8, length = 6;
        BodyShape round = prism(SectionShape.Rounded.ellipse(2 * r, 2 * r, 0), length);
        BodyShape.Station s = round.stations().get(0);
        BodyStructure.Torsion tr = BodyStructure.torsion(round, List.of(plain), new double[]{1, 1 + 4});
        double bredt = 4 * s.areaM2() * s.areaM2() * t / s.perimeterM();
        for (double gj : tr.gjNm2()) { assertEquals(bredt, gj / g, 1e-6 * bredt); }
        assertEquals(2 * Math.PI * r * r * r * t, bredt, 0.015 * 2 * Math.PI * r * r * r * t);
        // uniform along the length: the spring between stations 4 m apart is G J / 4 (the blended stations' polygons
        // differ in their last digits)
        assertEquals(g * bredt / 4, tr.springNmPerRad()[0], 1e-6 * g * bredt / 4);
        // a rectangular box 1.2 by 0.8 m: 4 (w h)^2 t / (2 (w + h))
        double w = 1.2, h = 0.8;
        BodyShape box = prism(new SectionShape.Cabin(w, h, 0, 1.0, 1.0, 0), length);
        double rect = 4 * w * w * h * h * t / (2 * (w + h));
        for (double gj : BodyStructure.torsion(box, List.of(plain), new double[]{3}).gjNm2()) { assertEquals(rect, gj / g, 1e-6 * rect); }
        // a space frame's shell the same skin (its tubes are not modelled)
        double frame = BodyStructure.torsion(box, List.of(alu(BodyStructure.Build.SPACE_FRAME, t, stringers, 0, 0, 0, 0)), new double[]{3}).gjNm2()[0] / g;
        assertEquals(rect, frame, 1e-6 * rect);
    }

    @Test void withoutOpeningsItIsTheClosedShell() {
        BodyShape box = prism(new SectionShape.Cabin(1.8, 1.3, 0.15, 0.6, 0.75, 0.12), 2.4);
        List<double[]> outline = box.stations().get(0).outline();
        BodyStructure.Module closed = steel(0, 0, 0, 0, 15);
        double j = BodyStructure.sectionTorsion(outline, closed, 2.4, 0, INF);
        BodyShape.Station s = box.stations().get(0);
        assertEquals(4 * s.areaM2() * s.areaM2() * closed.skinM() / s.perimeterM(), j, 1e-12 * j);
        // doors of no width, a door's size but no doors, a sill above the top: all the same closed cell, bit for bit
        assertEquals(j, BodyStructure.sectionTorsion(outline, steel(4, 0, 1.05, 0, 15), 2.4, 0, INF));
        assertEquals(j, BodyStructure.sectionTorsion(outline, steel(0, 0.95, 1.05, 0, 15), 2.4, 0, INF));
        assertEquals(j, BodyStructure.sectionTorsion(outline, closed, 2.4, 0, 1.5));
        // and the whole body: every sample, every spring
        BodyStructure.Torsion a = BodyStructure.torsion(box, List.of(closed), new double[]{0.4, 2.0});
        BodyStructure.Torsion b = BodyStructure.torsion(box, List.of(steel(0, 0.95, 1.05, 0, 15)), new double[]{0.4, 2.0});
        assertArrayEquals(a.gjNm2(), b.gjNm2());
        assertArrayEquals(a.springNmPerRad(), b.springNmPerRad());
        // a frame however heavy makes an opening no stiffer than the skin it replaces
        BodyStructure.Module heavy = new BodyStructure.Module(BodyStructure.Build.MONOCOQUE, 7850, 310e6, 205e9, 0.0008, 0.2, 0.5, 0.05, 15, 4, 0.2, 0.3, 0, 0);
        assertTrue(BodyStructure.sectionTorsion(outline, heavy, 2.4, 0, INF) <= j);
    }

    @Test void openingsLowerTheStiffness() {
        double length = 2.4;
        BodyShape box = prism(new SectionShape.Cabin(1.8, 1.3, 0.15, 0.6, 0.75, 0.12), length);
        List<double[]> outline = box.stations().get(0).outline();
        double closed = BodyStructure.sectionTorsion(outline, steel(0, 0, 0, 0, 15), length, 0, INF);
        double twoDoors = BodyStructure.sectionTorsion(outline, steel(2, 1.1, 1.05, 0, 15), length, 0, INF);
        double fourDoors = BodyStructure.sectionTorsion(outline, steel(4, 0.95, 1.05, 0, 15), length, 0, INF);
        double windows = BodyStructure.sectionTorsion(outline, steel(0, 0, 0, 0.35, 15), length, 0, INF);
        double both = BodyStructure.sectionTorsion(outline, steel(4, 0.95, 1.05, 0.35, 15), length, 0, INF);
        assertTrue(windows < closed, "windows cut the sides");
        assertTrue(twoDoors < windows, "doors cut more");
        assertTrue(fourDoors < twoDoors, "four doors leave thinner pillars");
        assertTrue(both <= fourDoors, "windows over the doors");
        // a door band is a frame, not a hole, but far softer than the skin it replaces
        assertTrue(fourDoors > 0.01 * closed && fourDoors < 0.2 * closed, "four doors: " + fourDoors / closed + " of the closed cell");
        // heavier frames round the doors stiffen them
        BodyStructure.Module light = new BodyStructure.Module(BodyStructure.Build.MONOCOQUE, 7850, 310e6, 205e9, 0.0008, 0.2, 0.5, 1e-4, 15, 4, 0.95, 1.05, 0, 0);
        assertTrue(BodyStructure.sectionTorsion(outline, light, length, 0, INF) < fourDoors);
        // a windscreen over the top and an open top (cut at the belt)
        assertTrue(BodyStructure.sectionTorsion(outline, steel(0, 0, 0, 0.35, 15), length, 0.8, INF) < windows, "a windscreen opens the top");
        double open = BodyStructure.sectionTorsion(outline, steel(0, 0, 0, 0, 15), length, 0, 0.15 + 0.6 * 1.3);
        assertTrue(open < 0.05 * closed, "an open top: " + open / closed);
        // the same through torsion(): an open module, and a glazed module's windscreen where its top slopes
        BodyStructure.Torsion roofed = BodyStructure.torsion(box, List.of(steel(0, 0, 0, 0, 15)), new double[]{0.2, 2.2});
        BodyShape convertible = new BodyShape(new SectionShape.Cabin(1.8, 1.3, 0.15, 0.6, 0.75, 0.12),
                List.of(new BodyModule("open", length, new SectionShape.Cabin(1.8, 1.3, 0.15, 0.6, 0.75, 0.12), 1, 0, 0.15 + 0.6 * 1.3)));
        assertTrue(BodyStructure.torsion(convertible, List.of(steel(0, 0, 0, 0, 15)), new double[]{0.2, 2.2}).springNmPerRad()[0] < 0.05 * roofed.springNmPerRad()[0]);
        BodyShape car = sedan();
        BodyStructure.Torsion sedan = BodyStructure.torsion(car, sedanStructure(), new double[]{1.2, 3.9});
        double g = BodyStructure.shearModulusPa(205e9);
        int screened = 0, flat = 0;
        for (int k = 0; k < sedan.xM().length; k++) {
            double x = sedan.xM()[k];
            BodyShape.Station s = null;
            for (BodyShape.Station c : car.stations()) { if (c.xM() == x) { s = c; } }
            if (x > car.moduleRearX(1) + 0.1 && x < car.moduleFrontX(1) - 0.2) {
                // inside the windshield's module, where its top rises steeply: less than its side windows alone
                double sides = g * BodyStructure.sectionTorsion(s.outline(), sedanStructure().get(1), car.moduleFrontX(1) - car.moduleRearX(1), 0, INF);
                if (sedan.gjNm2()[k] < 0.9 * sides) { screened++; }
            }
            if (x > car.moduleRearX(2) + 0.05 && x < car.moduleFrontX(2) - 0.05) {
                // the cabin's flat roof is no windscreen
                assertEquals(g * BodyStructure.sectionTorsion(s.outline(), sedanStructure().get(2), car.moduleFrontX(2) - car.moduleRearX(2), 0, INF), sedan.gjNm2()[k]);
                flat++;
            }
        }
        assertTrue(screened > 0 && flat > 0, screened + " windscreen sections, " + flat + " cabin sections");
    }

    @Test void theStationMatrixIsSymmetricSemiDefiniteAndFree() {
        BodyShape car = sedan();
        BodyStructure.Torsion t = BodyStructure.torsion(car, sedanStructure(), new double[]{3.9, 0.56, 2.4, 4.15});
        assertArrayEquals(new double[]{0.56, 2.4, 3.9, 4.15}, t.stationXM());
        double[][] k = t.stiffnessNmPerRad();
        double scale = 0;
        for (double[] row : k) for (double v : row) { scale = Math.max(scale, Math.abs(v)); }
        for (int i = 0; i < 4; i++) {
            double sum = 0;
            for (int j = 0; j < 4; j++) {
                assertEquals(k[i][j], k[j][i], "symmetric");
                sum += k[i][j];
                if (Math.abs(i - j) > 1) { assertEquals(0, k[i][j], "only neighbours are joined"); }
            }
            assertEquals(0, sum, 1e-12 * scale, "K 1 = 0: a free body turns without a torque");
            assertTrue(k[i][i] > 0);
        }
        Random random = new Random(7);
        for (int trial = 0; trial < 100; trial++) {
            double[] v = new double[4];
            for (int i = 0; i < 4; i++) { v[i] = random.nextGaussian(); }
            double q = 0;
            for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { q += v[i] * k[i][j] * v[j]; }
            assertTrue(q >= -1e-12 * scale, "positive semi-definite");
        }
        // the springs in series: the outer stations' stiffness through the two between
        double series = 1 / (1 / t.springNmPerRad()[0] + 1 / t.springNmPerRad()[1] + 1 / t.springNmPerRad()[2]);
        assertEquals(series, t.betweenNmPerRad(0, 3), 1e-12 * series);
        BodyStructure.Torsion two = BodyStructure.torsion(car, sedanStructure(), new double[]{0.56, 4.15});
        assertEquals(series, two.springNmPerRad()[0], 1e-9 * series, "the same length however it is split");
        assertEquals(two.springNmPerRad()[0], -two.stiffnessNmPerRad()[0][1]);
        // the sampled sections run along the body, each joint between modules twice (G J may step there)
        for (int i = 1; i < t.xM().length; i++) { assertTrue(t.xM()[i] >= t.xM()[i - 1]); }
        int repeats = 0;
        for (int i = 1; i < t.xM().length; i++) { if (t.xM()[i] == t.xM()[i - 1]) { repeats++; } }
        assertEquals(car.modules().size() - 1, repeats);
        // stations must be within the body and apart
        assertThrows(IllegalArgumentException.class, () -> BodyStructure.torsion(car, sedanStructure(), new double[]{-0.1, 2}));
        assertThrows(IllegalArgumentException.class, () -> BodyStructure.torsion(car, sedanStructure(), new double[]{2, 2}));
        assertThrows(IllegalArgumentException.class, () -> BodyStructure.torsion(car, sedanStructure(), new double[]{}));
        assertEquals(0, BodyStructure.torsion(car, sedanStructure(), new double[]{2}).stiffnessNmPerRad()[0][0]);
    }

    /**
     * A steel unibody saloon's body in white twists 10..40 kN m per degree between its axles (from about 10 for older or
     * open-structured bodies to about 40 for stiff modern ones).
     */
    @Test void aSedanTwistsLikeARealOne() {
        BodyShape car = sedan();
        assertEquals(4.85, car.lengthM(), 1e-9);
        assertEquals(1.82, car.particulars().maxWidthM(), 1e-6);
        assertEquals(1.45, car.particulars().maxHeightM() + 0.15, 1e-6);
        BodyStructure.Torsion t = BodyStructure.torsion(car, sedanStructure(), new double[]{1.2, 3.9});
        double perDeg = perDegreeKNm(t.springNmPerRad()[0]);
        // hdfv's sample puts its axles at 0.56 and 4.15 m (3.59 m apart)
        double samplePerDeg = perDegreeKNm(BodyStructure.torsion(car, sedanStructure(), new double[]{0.56, 4.15}).springNmPerRad()[0]);
        List<BodyStructure.Module> closed = List.of(steel(0, 0, 0, 0, 0), steel(0, 0, 0, 0, 15), steel(0, 0, 0, 0, 15), steel(0, 0, 0, 0, 15), steel(0, 0, 0, 0, 10));
        double closedPerDeg = perDegreeKNm(BodyStructure.torsion(car, closed, new double[]{1.2, 3.9}).springNmPerRad()[0]);
        System.out.printf("sedan torsion: %.1f kN m/deg over 2.7 m (1.2..3.9), %.1f between hdfv's axles, %.0f closed all round%n",
                perDeg, samplePerDeg, closedPerDeg);
        assertTrue(perDeg >= 10 && perDeg <= 40, perDeg + " kN m/deg");
        assertTrue(closedPerDeg > 10 * perDeg, "the openings are what makes a car body soft");
    }
}
