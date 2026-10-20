package dev.mcmod.aerocore.api.body;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.aero.Wing;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** A body's air loads: zero-lift drag of streamlined and blunt bodies, lift and Munk's moment at an angle, damping. */
class BodyAerodynamicsTest {
    private static final Wing.Air AIR = Wing.Air.SEA_LEVEL;

    /** An ellipsoid of revolution, diameter d, fineness f (the length is f d). */
    static BodyShape ellipsoid(double d, double f) {
        SectionShape circle = SectionShape.Rounded.ellipse(d, d, 0);
        return new BodyShape(circle.scaled(1e-3), List.of(
                new BodyModule("front", f * d / 2, circle, 2, 0, Double.POSITIVE_INFINITY),
                new BodyModule("rear", f * d / 2, circle.scaled(1e-3), 2, 0, Double.POSITIVE_INFINITY)));
    }

    @Test void aStreamlinedBodyHasHoernersDrag() {
        // Hoerner (Fluid-Dynamic Drag, ch. 6): C_D on the frontal area = C_f (3 f + 4.5 / sqrt f + 21 / f^2) for f 2..10
        double d = 1, f = 5, speed = 50;
        BodyAerodynamics a = new BodyAerodynamics(ellipsoid(d, f), 0);
        double re = AIR.densityKgM3() * speed * f * d / AIR.viscosityPaS();
        double cf = BodyAerodynamics.skinFriction(re, speed / AIR.soundMps());
        double hoerner = cf * (3 * f + 4.5 / Math.sqrt(f) + 21 / (f * f));
        double cd = a.zeroLiftDragAreaM2(speed, AIR) / (Math.PI * d * d / 4);
        assertEquals(hoerner, cd, 0.15 * hoerner, "C_D " + cd + " against Hoerner's " + hoerner);
        // straight through the air: drag only, along the body
        BodyAerodynamics.Loads l = a.loads(new Vec3d(speed, 0, 0), Vec3d.ZERO, AIR);
        assertEquals(0, l.liftN(), 1e-6);
        assertEquals(0.5 * AIR.densityKgM3() * speed * speed * a.zeroLiftDragAreaM2(speed, AIR), l.dragN(), 1e-6);
    }

    @Test void aBluntBaseAddsBaseDrag() {
        SectionShape circle = SectionShape.Rounded.ellipse(1, 1, 0);
        BodyShape cut = new BodyShape(circle.scaled(1e-3), List.of(new BodyModule("nose", 1.5, circle, 2, 0, Double.POSITIVE_INFINITY),
                BodyModule.straight("tube", 3, circle)));
        BodyShape closed = new BodyShape(circle.scaled(1e-3), List.of(new BodyModule("nose", 1.5, circle, 2, 0, Double.POSITIVE_INFINITY),
                BodyModule.straight("tube", 1.5, circle), new BodyModule("tail", 1.5, circle.scaled(1e-3), 2, 0, Double.POSITIVE_INFINITY)));
        double cutDrag = new BodyAerodynamics(cut, 0).zeroLiftDragAreaM2(50, AIR), closedDrag = new BodyAerodynamics(closed, 0).zeroLiftDragAreaM2(50, AIR);
        assertTrue(cutDrag > 1.5 * closedDrag, "a cut-off tail " + cutDrag + " against a closed one " + closedDrag);
    }

    @Test void anAngleGivesLiftAndAnUnstableMoment() {
        BodyShape b = ellipsoid(1, 6);
        BodyAerodynamics a = new BodyAerodynamics(b, 0);
        double speed = 40, alpha = Math.toRadians(8);
        BodyAerodynamics.Loads l = a.loads(new Vec3d(speed * Math.cos(alpha), -speed * Math.sin(alpha), 0), Vec3d.ZERO, AIR);
        assertTrue(l.liftN() > 0, "lift " + l.liftN());
        // about the centroid, Munk's moment turns the nose up (away from the wind): destabilising
        Vec3d c = new Vec3d(b.particulars().centroidXM(), b.particulars().centroidYM(), 0);
        double pitch = l.moment().subtract(c.cross(l.force())).z();
        double q = 0.5 * AIR.densityKgM3() * speed * speed;
        // Munk: M = q (k2 - k1) Vol sin 2a, (k2 - k1) about 0.9 at fineness 6
        double munk = q * 0.9 * b.particulars().volumeM3() * Math.sin(2 * alpha);
        assertEquals(munk, pitch, 0.2 * munk, "pitching moment " + pitch + " against Munk's " + munk);
        // side-slip mirrors it
        BodyAerodynamics.Loads side = a.loads(new Vec3d(speed * Math.cos(alpha), 0, -speed * Math.sin(alpha)), Vec3d.ZERO, AIR);
        assertEquals(l.liftN(), side.sideN(), 1e-6 * Math.abs(l.liftN()) + 1e-9);
    }

    @Test void turningIsDamped() {
        BodyShape b = ellipsoid(1, 6);
        BodyAerodynamics a = new BodyAerodynamics(b, 0);
        Vec3d c = new Vec3d(b.particulars().centroidXM(), b.particulars().centroidYM(), 0);
        // a quick turn (the ends' cross-flow well past the onset angle)
        for (double rate : new double[]{2, -2}) {
            BodyAerodynamics.Loads l = a.loads(new Vec3d(10, 0, 0).subtract(new Vec3d(0, 0, rate).cross(c)), new Vec3d(0, 0, rate), AIR);
            double pitch = l.moment().subtract(c.cross(l.force())).z();
            assertTrue(pitch * rate < 0, "pitching at " + rate + " rad/s is resisted: " + pitch);
        }
    }

    /**
     * Jorgensen (NASA TR R-474, fig. 19): body N2C1, a 3.5-calibre tangent-ogive nose on a 7-calibre cylinder, at Mach
     * 0.6 and Re_d 6.5e5; normal force on the base area read off at 10, 20 and 30 degrees (+-0.1). The ogive is taken as a
     * superellipse profile of fullness 1.6.
     */
    @Test void anOgiveCylindersNormalForceFollowsJorgensen() {
        double mach = 0.6, speed = mach * AIR.soundMps(), d = 6.5e5 * AIR.viscosityPaS() / (AIR.densityKgM3() * speed);
        SectionShape circle = SectionShape.Rounded.ellipse(d, d, 0);
        BodyAerodynamics a = new BodyAerodynamics(new BodyShape(circle.scaled(1e-3), List.of(
                new BodyModule("ogive", 3.5 * d, circle, 1.6, 0, Double.POSITIVE_INFINITY), BodyModule.straight("cylinder", 7 * d, circle))), 0);
        double q = 0.5 * AIR.densityKgM3() * speed * speed, base = Math.PI * d * d / 4;
        double[][] measured = {{10, 0.4}, {20, 1.3}, {30, 2.95}};
        for (double[] m : measured) {
            double alpha = Math.toRadians(m[0]);
            BodyAerodynamics.Loads l = a.loads(new Vec3d(speed * Math.cos(alpha), -speed * Math.sin(alpha), 0), Vec3d.ZERO, AIR);
            double cn = l.force().y() / (q * base);
            assertEquals(m[1], cn, 0.15 + 0.12 * m[1], "C_N at " + m[0] + " deg: " + cn);
        }
    }

    /** The Ahmed body (1.044 x 0.389 x 0.288 m, 100 mm front radius, a 222 mm slant) at 60 m/s with its slant angle. */
    static BodyShape ahmed(double slantDeg) {
        double w = 0.389, h = 0.288, slant = 0.222, run = slant * Math.cos(Math.toRadians(slantDeg)), drop = slant * Math.sin(Math.toRadians(slantDeg));
        SectionShape box = new SectionShape.Cabin(w, h, 0, 1, 1, 0);
        List<BodyModule> modules = new java.util.ArrayList<>(List.of(new BodyModule("front", 0.1, box, 2, 0, Double.POSITIVE_INFINITY),
                BodyModule.straight("body", 1.044 - 0.1 - run, box)));
        if (slantDeg > 0) { modules.add(BodyModule.straight("slant", run, new SectionShape.Cabin(w, h - drop, 0, 1, 1, 0))); }
        return new BodyShape(box.scaled(0.4), modules);
    }

    /**
     * Ahmed, Ramm and Faltin (1984): C_D 0.285 at 25 degrees (front 0.020, slant 0.140, base 0.070, friction 0.055), 0.378 in
     * the high-drag state at 30 and 0.26 past it at 35; 0.230 at 10 (secondary); lift 0.345 at 25 (Meile et al. 2011).
     * The base and slant terms are fitted to these, so this checks the fit holds across the angles with one set of terms.
     */
    @Test void theAhmedBodysDragFollowsItsSlant() {
        double[][] measured = {{10, 0.230}, {25, 0.285}, {30, 0.378}, {35, 0.26}};
        for (double[] m : measured) {
            BodyAerodynamics a = new BodyAerodynamics(ahmed(m[0]), 0);
            double cd = a.zeroLiftDragAreaM2(60, AIR) / (0.389 * 0.288);
            assertEquals(m[1], cd, 0.1 * m[1], "C_D at " + m[0] + " deg: " + cd);
        }
        BodyAerodynamics a25 = new BodyAerodynamics(ahmed(25), 0);
        assertEquals(0.345, a25.slantLiftAreaM2() / (0.389 * 0.288), 0.05);
        BodyAerodynamics.Loads l = a25.loads(new Vec3d(60, 0, 0), Vec3d.ZERO, AIR);
        assertEquals(0.345, l.liftN() / (0.5 * AIR.densityKgM3() * 3600 * 0.389 * 0.288), 0.05, "the slant lifts the rear");
    }

    /**
     * The NASA box van (TP-1999-206574: 5.08 x 2.03 x 1.61 m body): square corners 0.89, every corner rounded 0.41 on
     * the area down to the ground. Its wheels and underbody are not modelled, so only the rounding's effect is checked.
     */
    @Test void roundingAVansFrontCutsItsDrag() {
        SectionShape box = new SectionShape.Cabin(2.03, 1.61, 0, 1, 1, 0), rounded = new SectionShape.Cabin(2.03, 1.61, 0, 1, 1, 0.3);
        BodyAerodynamics square = new BodyAerodynamics(new BodyShape(box, List.of(BodyModule.straight("box", 5.08, box))), 0);
        BodyAerodynamics round = new BodyAerodynamics(new BodyShape(rounded.scaled(0.7), List.of(new BodyModule("front", 0.3, rounded, 2, 0, Double.POSITIVE_INFINITY),
                BodyModule.straight("box", 4.78, rounded))), 0);
        double ref = 2.03 * 2.04, cdSquare = square.zeroLiftDragAreaM2(25, AIR) / ref, cdRound = round.zeroLiftDragAreaM2(25, AIR) / ref;
        assertTrue(cdSquare > 0.6 && cdSquare < 0.95, "square " + cdSquare);
        assertTrue(cdSquare > 2 * cdRound, "rounding the corners more than halves it: " + cdSquare + " against " + cdRound);
    }
}
