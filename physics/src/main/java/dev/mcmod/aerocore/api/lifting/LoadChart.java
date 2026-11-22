package dev.mcmod.aerocore.api.lifting;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.frame.FrameMaterial;
import dev.mcmod.aerocore.api.frame.Section;

import java.util.Objects;

/**
 * Rated loads of a slewing crane with a luffing boom (a telescopic boom on its luffing cylinder, or a lattice boom on
 * its pendants): at each radius the least of the stability limit (Stability and a rating rule), the boom's structural
 * limit and the hoist's (rope, reeving, winch) limit, and a chart of them. What a load moment indicator compares the
 * hook load with. Formulas and documented simplifications; the crane is the caller's.
 *
 * <p>Luffing plane: x horizontal along the boom's direction from the slewing centre, z up from the ground (m). The boom
 * is pinned at its foot (x_f, z_f) and held at a distance a along it by a two-force member (the luffing cylinder, or
 * the pendants when a = L) whose other end sits on the superstructure at (x_s, z_s). The hook hangs from the boom head
 * at the radius r = x_f + L cos(alpha), alpha the boom angle above horizontal; the head sheave's offset from the boom
 * axis is left out. Loads are gross: the hook block, slings and rigging are part of the hook load.
 *
 * <p>Structure, a documented simplification (stressPa): one section along the whole boom (for a telescopic boom an
 * equivalent of its sections: the base section at the cylinder is the most loaded, but the thinner fly sections set
 * the stiffness and many real limits; telescoping overlaps, local plate buckling, the luffing cylinder itself and
 * fatigue are not checked); first-order statics in the luffing plane with a second-order amplification 1 / (1 - N / N_cr)
 * of the moments (Timoshenko and Gere, Theory of Elastic Stability, 2nd ed., ch. 1, beam-columns), N_cr the Euler load
 * pi^2 E I / L_e^2 over L_e = max(a, 2 (L - a)), the pinned span or the overhang as a cantilever (Euler's case 1);
 * an optional side force at the head (a share of the hook load) bending the boom sideways as a cantilever from the foot
 * (L_e = 2 L); the greatest axial force and the greatest moments are added as if at one section (on the safe side).
 * Allowable stress: yield over a safety factor (STRUCTURE_SAFETY), with the hook load raised by the hoist's dynamic
 * factor.
 */
public final class LoadChart {
    private LoadChart() {}

    private static final double G = Atmosphere.GRAVITY_MPS2;
    /** Bisection steps of structureKg. */
    private static final int BISECTIONS = 80;
    /** Below this the support's line passes through the boom foot and cannot hold the boom. */
    private static final double PARALLEL = 1e-9;

    /**
     * Safety factor on yield for the main loads: 1.5, as DIN 15018-1's allowable stress for crane steel in load case H
     * (240 N/mm^2 for St 52-3 of yield 355 to 360 N/mm^2). A round figure for any steel; EN 13001's limit-state
     * factors are the caller's if it wants them.
     */
    public static final double STRUCTURE_SAFETY = 1.5;

    /**
     * Hoisting classes of EN 13001-2 (HC1 stiff to HC4 soft) with the dynamic factor on the hoist load when a slack
     * load is lifted off the ground: phi2 = phi2min + beta2 v_h, v_h the hoisting speed (m/s): HC1 1.05 + 0.17 v,
     * HC2 1.10 + 0.34 v, HC3 1.15 + 0.51 v, HC4 1.20 + 0.68 v (EN 13001-2, phi2 for the hoist drive classes with a
     * steady hoisting speed; values as widely quoted).
     */
    public enum HoistClass {
        HC1(1.05, 0.17), HC2(1.10, 0.34), HC3(1.15, 0.51), HC4(1.20, 0.68);

        private final double minimum, slope;

        HoistClass(double minimum, double slope) {
            this.minimum = minimum;
            this.slope = slope;
        }

        /** phi2 = phi2min + beta2 v_h for a hoisting speed (m/s, not negative). */
        public double dynamicFactor(double hoistSpeedMps) {
            return minimum + slope * Check.nonNegative(hoistSpeedMps, "Hoisting speed");
        }
    }

    /**
     * A luffing boom: length L (m), mass (kg, not negative) with its centre of gravity at a share of L from the foot
     * (0..1, exclusive), the foot's x from the slewing centre (m, along the boom; negative behind it) and height above
     * the ground (m), where the luffing cylinder or pendants hold it (m from the foot, 0 &lt; a &lt;= L) and the other end
     * of that member on the superstructure (x_s, z_s; m), the section (frame.Section: y up in the luffing plane, z
     * sideways) and its material.
     */
    public record Boom(double lengthM, double massKg, double centroidShare, double footXM, double footHeightM,
                       double supportFromFootM, double supportXM, double supportZM, Section section, FrameMaterial material) {
        public Boom {
            Check.positive(lengthM, "Boom length"); Check.nonNegative(massKg, "Boom mass");
            if (!(centroidShare > 0 && centroidShare < 1)) { throw new IllegalArgumentException("The centroid share is between 0 and 1"); }
            Check.finite(footXM, "Foot x"); Check.finite(footHeightM, "Foot height");
            Check.positive(supportFromFootM, "Support distance");
            if (supportFromFootM > lengthM) { throw new IllegalArgumentException("The support is on the boom"); }
            Check.finite(supportXM, "Support x"); Check.finite(supportZM, "Support z");
            Objects.requireNonNull(section, "section");
            Objects.requireNonNull(material, "material");
        }
    }

    /**
     * The hoist: parts of line, the line pull the rope and winch allow (N: the lesser of WireRope.workingLoadLimitN and
     * the winch's pull at its working layer), sheave efficiency and fixed guide sheaves (Reeving).
     */
    public record Hoist(int parts, double linePullN, double sheaveEfficiency, int guideSheaves) {
        public Hoist {
            Check.atLeastOne(parts, "Parts of line"); Check.positive(linePullN, "Line pull");
            Check.share(sheaveEfficiency, "Sheave efficiency");
            if (guideSheaves < 0) { throw new IllegalArgumentException("Guide sheaves must not be negative"); }
        }
    }

    /**
     * How the crane is rated: the stability rule, the structural safety factor on yield (at least 1; STRUCTURE_SAFETY),
     * the dynamic factor on the hook load for the structure (at least 1; HoistClass) and the side force at the boom head
     * as a share of the hook load (not negative; 0 leaves out sideways bending).
     */
    public record Duty(Stability.Rule rule, double structureSafety, double dynamicFactor, double sideLoadShare) {
        public Duty {
            Objects.requireNonNull(rule, "rule");
            if (!(structureSafety >= 1) || !Double.isFinite(structureSafety)) { throw new IllegalArgumentException("Safety factor at least 1"); }
            if (!(dynamicFactor >= 1) || !Double.isFinite(dynamicFactor)) { throw new IllegalArgumentException("Dynamic factor at least 1"); }
            Check.nonNegative(sideLoadShare, "Side load share");
        }
    }

    /** What sets a rated load: tipping, the boom, the hoist, or a radius the boom cannot reach. */
    public enum Limit { STABILITY, STRUCTURE, ROPE, REACH }

    /**
     * A rated load (kg) at a radius (m) with the boom angle (rad above horizontal) and boom head height (m) there, the
     * three limits (kg; stability may be infinite when the hook is over the supports) and which one governs. Out of reach:
     * all zero, the angle at the nearer end (0 or pi / 2).
     */
    public record Rating(double radiusM, double boomAngleRad, double headHeightM, double stabilityKg, double structureKg,
                         double ropeKg, double ratedKg, Limit limit) {}

    /**
     * A load chart: radii (m, increasing) and rated loads (kg). ratedKg(r) interpolates linearly between the radii and is
     * zero outside them (no rating off the chart).
     */
    public record Table(double[] radiiM, double[] ratedKg) {
        public Table {
            Objects.requireNonNull(radiiM, "radiiM"); Objects.requireNonNull(ratedKg, "ratedKg");
            if (radiiM.length == 0 || radiiM.length != ratedKg.length) { throw new IllegalArgumentException("A chart has as many loads as radii, at least one"); }
            radiiM = radiiM.clone();
            ratedKg = ratedKg.clone();
            for (int i = 0; i < radiiM.length; i++) {
                Check.nonNegative(radiiM[i], "Radius"); Check.nonNegative(ratedKg[i], "Rated load");
                if (i > 0 && !(radiiM[i] > radiiM[i - 1])) { throw new IllegalArgumentException("Radii must increase"); }
            }
        }

        @Override public double[] radiiM() { return radiiM.clone(); }
        @Override public double[] ratedKg() { return ratedKg.clone(); }

        /** The rated load (kg) at a radius (m): linear between the chart's radii, 0 outside them. */
        public double ratedKg(double radiusM) {
            Check.finite(radiusM, "Radius");
            if (radiusM < radiiM[0] || radiusM > radiiM[radiiM.length - 1]) { return 0; }
            for (int i = 1; i < radiiM.length; i++) {
                if (radiusM <= radiiM[i]) {
                    double t = (radiusM - radiiM[i - 1]) / (radiiM[i] - radiiM[i - 1]);
                    return ratedKg[i - 1] + t * (ratedKg[i] - ratedKg[i - 1]);
                }
            }
            return ratedKg[0];
        }
    }

    /** Whether the boom reaches a radius (m): 0 &lt; r - x_f &lt;= L. */
    public static boolean reaches(Boom boom, double radiusM) {
        Check.finite(radiusM, "Radius");
        double horizontal = radiusM - boom.footXM();
        return horizontal > 0 && horizontal <= boom.lengthM();
    }

    /** Boom angle (rad above horizontal) for a radius (m) it reaches: acos((r - x_f) / L). */
    public static double boomAngleRad(Boom boom, double radiusM) {
        if (!reaches(boom, radiusM)) { throw new IllegalArgumentException("The boom does not reach " + radiusM + " m"); }
        return Math.acos((radiusM - boom.footXM()) / boom.lengthM());
    }

    /** The boom's mass reduced to its head (kg): its moment about the foot over its length, mass x centroid share (ISO 4305's F). */
    public static double boomHeadKg(Boom boom) {
        return boom.massKg() * boom.centroidShare();
    }

    /** The hoist's limit (kg): Reeving.hookLoadN of its parts, line pull and sheaves, over g. */
    public static double ropeKg(Hoist hoist) {
        return Reeving.hookLoadN(hoist.parts(), hoist.linePullN(), hoist.sheaveEfficiency(), hoist.guideSheaves()) / G;
    }

    /**
     * The machine with the boom at a boom angle: the boom's mass added to the superstructure at its centre of gravity,
     * x_f + c L cos(alpha) along the boom.
     */
    public static Stability.Crane withBoom(Stability.Crane machine, Boom boom, double boomAngleRad) {
        Objects.requireNonNull(machine, "machine");
        Check.finite(boomAngleRad, "Boom angle");
        return machine.withUpper(new Stability.Mass(boom.massKg(), boom.footXM() + boom.centroidShare() * boom.lengthM() * Math.cos(boomAngleRad), 0));
    }

    /**
     * The greatest stress (Pa) in the boom at a boom angle (rad) under a hook load (kg, not negative); positive infinity
     * when it buckles (the axial force reaches N_cr) or the support cannot hold it. With e_b the unit vector from foot to
     * head, W = phi2 m g the hook load, T = W / (n eta_F) the fast line running back along the boom, q = m_boom g / L:
     * <ul>
     * <li>support force F along u (from the attachment A = foot + a e_b toward (x_s, z_s); positive pulls), from
     * moments about the foot: F a (e_b x u) = (W L + m_boom g c L) cos(alpha);</li>
     * <li>axial compression: N1 = W sin(alpha) + T + q sin(alpha) (L - a) on the head side of A,
     * N2 = N1 - F u.e_b on its foot side, N3 = N2 + q sin(alpha) a at the foot;</li>
     * <li>luffing-plane moment: M_A = W cos(alpha) (L - a) + q cos(alpha) (L - a)^2 / 2 at A; between foot and A,
     * M(x) = R x - q cos(alpha) x^2 / 2 with R = (q cos(alpha) a^2 / 2 - M_A) / a, greatest R^2 / (2 q cos(alpha));</li>
     * <li>sideways: M_y = s W L at the foot, s the side load share;</li>
     * <li>sigma = max |N_i| / A + section.bendingPa(M_y / (1 - N / N_cr,y), M_z / (1 - N / N_cr,z)),
     * N = max(N_i, 0), N_cr,z = pi^2 E I_z / max(a, 2 (L - a))^2, N_cr,y = pi^2 E I_y / (2 L)^2.</li>
     * </ul>
     * A lattice boom on pendants (a = L) carries no overhang moment: compression and its own weight's bending, as it
     * should; its chords' buckling between lacing panels is not checked (latticeSection).
     */
    public static double stressPa(Boom boom, Hoist hoist, Duty duty, double boomAngleRad, double hookLoadKg) {
        Objects.requireNonNull(boom, "boom"); Objects.requireNonNull(hoist, "hoist"); Objects.requireNonNull(duty, "duty");
        if (!(boomAngleRad >= 0 && boomAngleRad <= Math.PI / 2)) { throw new IllegalArgumentException("Boom angle is 0..pi/2"); }
        Check.nonNegative(hookLoadKg, "Hook load");
        Section s = boom.section();
        double e = boom.material().modulusPa();
        double length = boom.lengthM(), a = boom.supportFromFootM(), over = length - a;
        double cos = Math.cos(boomAngleRad), sin = Math.sin(boomAngleRad);
        double w = duty.dynamicFactor() * hookLoadKg * G;
        double fast = w / (hoist.parts() * Reeving.efficiency(hoist.parts(), hoist.sheaveEfficiency()));
        double q = boom.massKg() * G / length, qPerp = q * cos, qAxial = q * sin;
        // the support's force from moments about the foot
        double ax = boom.footXM() + a * cos, az = boom.footHeightM() + a * sin;
        double ux = boom.supportXM() - ax, uz = boom.supportZM() - az, ul = Math.hypot(ux, uz);
        if (!(ul > PARALLEL)) { return Double.POSITIVE_INFINITY; }
        ux /= ul;
        uz /= ul;
        double lever = a * (cos * uz - sin * ux);
        double turning = (w * length + boom.massKg() * G * boom.centroidShare() * length) * cos;
        double support;
        if (Math.abs(lever) < PARALLEL * a) {
            if (turning > 0) { return Double.POSITIVE_INFINITY; }
            support = 0;
        } else {
            support = turning / lever;
        }
        double n1 = w * sin + fast + qAxial * over;
        double n2 = n1 - support * (ux * cos + uz * sin);
        double n3 = n2 + qAxial * a;
        double axial = Math.max(Math.abs(n1), Math.max(Math.abs(n2), Math.abs(n3)));
        double compression = Math.max(0, Math.max(n1, Math.max(n2, n3)));
        // luffing-plane moment: the overhang's at A, the span's sagging peak
        double atSupport = w * cos * over + qPerp * over * over / 2;
        double span = 0;
        if (qPerp > 0) {
            double reaction = (qPerp * a * a / 2 - atSupport) / a;
            if (reaction > 0 && reaction / qPerp < a) { span = reaction * reaction / (2 * qPerp); }
        }
        double mz = Math.max(atSupport, span);
        double criticalZ = Math.PI * Math.PI * e * s.izM4() / sq(Math.max(a, 2 * over));
        if (compression >= criticalZ) { return Double.POSITIVE_INFINITY; }
        double my = duty.sideLoadShare() * w * length;
        double ampY = 1;
        if (my > 0) {
            double criticalY = Math.PI * Math.PI * e * s.iyM4() / sq(2 * length);
            if (compression >= criticalY) { return Double.POSITIVE_INFINITY; }
            ampY = 1 / (1 - compression / criticalY);
        }
        return axial / s.areaM2() + s.bendingPa(my * ampY, mz / (1 - compression / criticalZ));
    }

    /**
     * The boom's structural limit (kg) at a boom angle (rad): the greatest hook load whose stressPa stays within
     * yield / safety, by bisection (to about 1e-9 of the load); 0 when the boom's own weight already exceeds it.
     */
    public static double structureKg(Boom boom, Hoist hoist, Duty duty, double boomAngleRad) {
        double allowable = boom.material().yieldPa() / duty.structureSafety();
        if (!(stressPa(boom, hoist, duty, boomAngleRad, 0) <= allowable)) { return 0; }
        double lo = 0, hi = 1000;
        while (stressPa(boom, hoist, duty, boomAngleRad, hi) <= allowable) {
            lo = hi;
            hi *= 2;
            if (hi > 1e12) { return lo; }
        }
        for (int i = 0; i < BISECTIONS && hi - lo > 1e-9 * hi; i++) {
            double mid = 0.5 * (lo + hi);
            if (stressPa(boom, hoist, duty, boomAngleRad, mid) <= allowable) { lo = mid; } else { hi = mid; }
        }
        return lo;
    }

    /** The rated load at a radius (m) with the boom slewed to an angle (rad): rate at that direction only. */
    public static Rating rate(Stability.Crane machine, Boom boom, Hoist hoist, Duty duty, double radiusM, double slewRad) {
        Check.finite(slewRad, "Slewing angle");
        return rate(machine, boom, hoist, duty, radiusM, slewRad, false);
    }

    /** The rated load at a radius (m) over the full slewing circle (the least stable direction; Stability.worstSlew). */
    public static Rating rate360(Stability.Crane machine, Boom boom, Hoist hoist, Duty duty, double radiusM) {
        return rate(machine, boom, hoist, duty, radiusM, 0, true);
    }

    /** A 360-degree chart at radii (m, increasing): rate360 at each. */
    public static Table chart(Stability.Crane machine, Boom boom, Hoist hoist, Duty duty, double[] radiiM) {
        Objects.requireNonNull(radiiM, "radiiM");
        double[] rated = new double[radiiM.length];
        for (int i = 0; i < radiiM.length; i++) { rated[i] = rate360(machine, boom, hoist, duty, radiiM[i]).ratedKg(); }
        return new Table(radiiM, rated);
    }

    /**
     * An equivalent section of a four-chord lattice boom: chords (each the given section, usually a round tube) at the
     * corners of a width b (along z) by height h (along y) between chord centres. A = 4 A_c, I_z = 4 (I_c + A_c h^2 / 4),
     * I_y = 4 (I_c + A_c b^2 / 4); torsion from the chords alone (4 J_c, the lacing's part left out); open, not warping.
     * The lacing is taken as making the chords act as one (its shear flexibility, Engesser's reduction of N_cr, is left
     * out) and the chords' buckling between lacing panels is not checked. The outline is the envelope, for drawing.
     */
    public static Section latticeSection(Section chord, double widthM, double heightM) {
        Objects.requireNonNull(chord, "chord");
        Check.positive(widthM, "Width"); Check.positive(heightM, "Height");
        double ac = chord.areaM2();
        double iz = 4 * (chord.izM4() + ac * heightM * heightM / 4), iy = 4 * (chord.iyM4() + ac * widthM * widthM / 4);
        double outerY = heightM / 2 + chord.outerYM(), outerZ = widthM / 2 + chord.outerZM();
        double[] envelope = {-outerZ, -outerY, outerZ, -outerY, outerZ, outerY, -outerZ, outerY};
        return new Section(Section.Kind.CUSTOM, 4 * ac, iy, iz, 4 * chord.torsionM4(), 0, outerY, outerZ, false, chord.wallM(), 0,
                new double[][]{envelope});
    }

    private static Rating rate(Stability.Crane machine, Boom boom, Hoist hoist, Duty duty, double radiusM, double slewRad, boolean worst) {
        Objects.requireNonNull(machine, "machine"); Objects.requireNonNull(boom, "boom");
        Objects.requireNonNull(hoist, "hoist"); Objects.requireNonNull(duty, "duty");
        Check.nonNegative(radiusM, "Radius");
        if (!reaches(boom, radiusM)) {
            double angle = radiusM - boom.footXM() > 0 ? 0 : Math.PI / 2;
            return new Rating(radiusM, angle, 0, 0, 0, 0, 0, Limit.REACH);
        }
        double angle = boomAngleRad(boom, radiusM);
        Stability.Crane crane = withBoom(machine, boom, angle);
        double tipping = worst ? Stability.worstSlew(crane, radiusM).tippingKg() : Stability.tippingLoadKg(crane, radiusM, slewRad);
        double stability = duty.rule().ratedKg(tipping, boomHeadKg(boom));
        double structure = structureKg(boom, hoist, duty, angle);
        double rope = ropeKg(hoist);
        double rated = Math.min(stability, Math.min(structure, rope));
        Limit limit = rated == stability ? Limit.STABILITY : rated == structure ? Limit.STRUCTURE : Limit.ROPE;
        return new Rating(radiusM, angle, boom.footHeightM() + boom.lengthM() * Math.sin(angle), stability, structure, rope, rated, limit);
    }

    private static double sq(double v) { return v * v; }
}
