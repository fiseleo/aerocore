package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Plume shapes (0.29) against the gas-dynamic relations and measurements, and blackbody colours against Charity's table. */
class PlumeTest {
    private static final double PA = 101_325, TA = 288.15, AIR = 1.4;

    private static Plume.Exit sonic(double pressureRatio) {
        // a sonic exit whose total pressure is the ratio times ambient
        double pe = pressureRatio * PA / Math.pow(1 + 0.2, 3.5);
        return new Plume.Exit(1, pe, TA / 1.2, 1, AIR, PA, TA);
    }

    @Test void theAreaRelationAndTheExpandedJetAreRight() {
        assertEquals(1.6875, Plume.areaRatio(2, AIR), 1e-4, "A/A* at Mach 2");
        assertEquals(1.0, Plume.areaRatio(1, AIR), 1e-12);
        // a jet already at ambient pressure keeps its Mach and diameter
        Plume.Shape matched = Plume.shape(new Plume.Exit(1.5, PA, 200, 0.4, AIR, PA, TA));
        assertEquals(1.5, matched.jetMach(), 1e-9);
        assertEquals(0.4, matched.jetDiameterM(), 1e-9);
        // and its cells are Prandtl-Pack's 1.306 D sqrt(M^2 - 1), faint (only lip waves), with no Mach disk
        assertEquals(1.306 * 0.4 * Math.sqrt(1.25), matched.cellSpacingM(), 1e-9);
        assertTrue(matched.cellStrength() < 0.15 && matched.hasCells());
        assertFalse(matched.hasMachDisk());
    }

    @Test void aSonicJetsCellsMatchMeasuredSpacing() {
        // Powell (1953) and later sonic-jet measurements: about 1.1 to 1.3 exit diameters at a pressure ratio of 3
        Plume.Shape s = Plume.shape(sonic(3));
        assertEquals(1.358, s.jetMach(), 2e-3);
        assertTrue(s.jetDiameterM() > 1 && s.jetDiameterM() < 1.06, "expanded diameter " + s.jetDiameterM());
        assertTrue(s.cellSpacingM() > 1.1 && s.cellSpacingM() < 1.35, "cell spacing " + s.cellSpacingM());
        assertTrue(s.cellSpacingM() > Plume.shape(sonic(2.2)).cellSpacingM(), "cells lengthen with pressure ratio");
        assertFalse(s.hasMachDisk(), "p_e / p_a 1.6: no Mach disk yet");
    }

    @Test void aStronglyUnderExpandedJetHasAMachDiskWhereAshkenasAndShermanPutIt() {
        Plume.Shape s = Plume.shape(sonic(6));
        assertTrue(s.hasMachDisk());
        assertEquals(0.67 * Math.sqrt(6), s.machDiskM(), 1e-9);
        assertTrue(s.machDiskDiameterM() > 0 && s.machDiskDiameterM() < s.jetDiameterM());
        assertTrue(s.cellStrength() > 0.5);
    }

    @Test void aSubsonicJetHasNoCellsAndAHotJetsCoreIsShorter() {
        Plume.Shape sub = Plume.shape(new Plume.Exit(0.6, PA, 600, 0.5, AIR, PA, TA));
        assertEquals(0, sub.cellSpacingM());
        assertFalse(sub.hasCells() || sub.hasMachDisk());
        Plume.Shape cold = Plume.shape(new Plume.Exit(0.9, PA, TA, 0.5, AIR, PA, TA)), hot = Plume.shape(new Plume.Exit(0.9, PA, 900, 0.5, AIR, PA, TA));
        assertEquals(0.5 * (4.2 + 1.1 * 0.81), cold.coreLengthM(), 1e-9, "Lau, Morris and Fisher for an unheated jet");
        assertTrue(hot.coreLengthM() < cold.coreLengthM());
    }

    @Test void blackbodyColoursFollowCharitysTable() {
        // Mitchell Charity, "What color is a blackbody?" (sRGB, D65, 2 degree): 255 is the brightest channel
        int[][] table = {{1000, 255, 56, 0}, {1500, 255, 109, 0}, {2000, 255, 137, 18}, {3000, 255, 180, 107}, {4000, 255, 209, 163},
                {5000, 255, 228, 206}, {6500, 255, 249, 253}};
        for (int[] row : table) {
            double[] c = Blackbody.srgb(row[0]);
            for (int k = 0; k < 3; k++) { assertEquals(row[k + 1], 255 * c[k], 30, row[0] + " K channel " + k); }
        }
        assertTrue(Blackbody.radiance(2000, 2000) == 1 && Blackbody.radiance(900, 2000) < 1e-3, "steel at 900 K is dim beside a flame");
        assertTrue(Blackbody.radiance(3000, 2000) > 5);
    }

    @Test void flamesLookAsTheirPropellantsDo() {
        Plume.Look h2 = Plume.look(Plume.Family.HYDROGEN, 2500, false), dry = Plume.look(Plume.Family.HYDROCARBON, 1000, false),
                reheat = Plume.look(Plume.Family.HYDROCARBON, 2000, true), apcp = Plume.look(Plume.Family.ALUMINISED_SOLID, 3300, false),
                knsb = Plume.look(Plume.Family.SUGAR_SOLID, 1600, false);
        assertTrue((h2.core() >>> 24) < 0x80 && (h2.core() & 255) > ((h2.core() >> 16) & 255), "hydrogen: faint and blue");
        assertTrue(dry.visibleDiameters() < reheat.visibleDiameters(), "a dry jet barely shows; an afterburner's flame is long");
        assertTrue(((reheat.core() >> 16) & 255) > (reheat.core() & 255), "afterburner soot glows orange");
        assertTrue(apcp.smoke() > 0.5 && knsb.smoke() > 0.5 && h2.smoke() == 0, "solid motors smoke, hydrogen does not");
        assertTrue((knsb.core() & 255) > ((knsb.core() >> 8) & 255), "potassium tints the sugar motor's flame violet");
    }
}
