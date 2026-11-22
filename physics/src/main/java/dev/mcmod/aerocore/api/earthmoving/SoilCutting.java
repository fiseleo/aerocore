package dev.mcmod.aerocore.api.earthmoving;

/**
 * Soil cutting by the wedge failure model: McKyes' form of Reece's (1964) "universal earthmoving equation" (McKyes,
 * Soil Cutting and Tillage, Elsevier 1985, ch. 4), with McKyes and Ali's (1977) side crescents for narrow tines.
 *
 * <p>A flat blade of rake alpha (from the horizontal, measured in the direction of travel), width w, working depth d,
 * moves at speed v through soil of unit weight gamma, cohesion c, internal friction phi, soil-metal friction delta and
 * adhesion c_a, under a surcharge q on the surface. The soil ahead fails along a plane from the tip at beta from the
 * horizontal. The force on the blade, at delta to its normal, is
 * <pre>F = (gamma d^2 N_gamma + c d N_c + q d N_q + c_a d N_ca + (gamma v^2 / g) d N_a) w</pre>
 * with D = cos(alpha + delta) + sin(alpha + delta) cot(beta + phi) and
 * <pre>
 * N_gamma = (cot alpha + cot beta) / (2 D)        N_c  = (1 + cot beta cot(beta + phi)) / D
 * N_q     = (cot alpha + cot beta) / D            N_ca = (1 - cot alpha cot(beta + phi)) / D
 * N_a     = sin alpha cos phi / (sin(alpha + beta) sin(alpha + delta + beta + phi))
 * </pre>
 * (N_a: the soil crossing the failure plane is brought to v sin alpha / sin(alpha + beta); McKyes 1985, section 4.6).
 * beta is the one that gives the least force (searched: 24 trial angles, then a golden-section refinement, about 60
 * evaluations). Horizontal force H = F sin(alpha + delta) + c_a d w cot alpha; downward force on the tool
 * V = F cos(alpha + delta) - c_a d w (returned as lift = -V).
 *
 * <p>Narrow tines (rippers, teeth): the soil also fails sideways in two crescents of radius r = d (cot alpha + cot beta)
 * about the top of the tine, each spanning rho, cos rho = cot alpha / (cot alpha + cot beta) (round to beside the tip);
 * N_gamma is multiplied by 1 + 2 r sin(rho) / (3 w), N_c and N_q by 1 + r sin(rho) / w, N_ca stays on the tine face
 * (McKyes 1985, section 4.4). The crescents get the acceleration term too (N_a times 1 + r sin(rho) / w): an
 * assumption, McKyes and Ali give it for the central wedge.
 *
 * <p>Validity: a flat blade and a planar failure surface. For a vertical wall the result is Coulomb's passive thrust
 * (Rankine's when smooth). The model needs alpha + delta + phi + beta below 180 deg: steeper blades (a curled bucket, a
 * blade tipped forward) are held at alpha = 170 deg - delta - phi, where the force is large but finite; rakes below
 * 5 deg are taken as 5 deg; delta is capped at phi (a rougher wall shears the soil instead). Below the critical depth of
 * a narrow tine (several tine widths; Godwin and Spoor 1977) the soil flows round it sideways and this overestimates.
 * Undrained, rate-dependent and dilatant behaviour is not modelled. All inputs are continuous: the result is.
 */
public final class SoilCutting {
    private SoilCutting() {}

    /** Smallest rake used, rad (5 deg): flatter blades are taken at this. */
    public static final double MIN_RAKE_RAD = Math.toRadians(5);

    /** How far below 180 deg alpha + delta + phi is held, rad (10 deg): the room left for the failure plane. */
    public static final double CLOSURE_RAD = Math.toRadians(10);

    private static final double BETA_LO = Math.toRadians(0.5), BETA_HI = Math.toRadians(89);
    private static final int COARSE = 24, REFINE = 32;
    private static final double GOLDEN = (Math.sqrt(5) - 1) / 2;

    /** McKyes' factors of a wide blade at one failure angle (dimensionless). */
    public record Factors(double nGamma, double nCohesion, double nSurcharge, double nAdhesion, double nAcceleration) {}

    /**
     * The N factors (formulas in the class note) of a wide blade of rake alpha with soil-metal friction delta, in soil
     * of friction phi, for a failure plane at beta (all rad). alpha in (0, 180) deg, beta in (0, 90] deg, and
     * alpha + delta + phi + beta below 180 deg.
     */
    public static Factors factors(double rakeRad, double wallFrictionRad, double frictionRad, double failureRad) {
        if (!(rakeRad > 0 && rakeRad < Math.PI) || !(wallFrictionRad >= 0) || !(frictionRad >= 0) || !(failureRad > 0 && failureRad <= Math.PI / 2)
                || !(rakeRad + wallFrictionRad + frictionRad + failureRad < Math.PI)) {
            throw new IllegalArgumentException("Angles out of the wedge's range");
        }
        double cotA = 1 / Math.tan(rakeRad), cotB = 1 / Math.tan(failureRad), cotBP = 1 / Math.tan(failureRad + frictionRad);
        double d = Math.cos(rakeRad + wallFrictionRad) + Math.sin(rakeRad + wallFrictionRad) * cotBP;
        double nAcc = Math.sin(rakeRad) * Math.cos(frictionRad)
                / (Math.sin(rakeRad + failureRad) * Math.sin(rakeRad + wallFrictionRad + failureRad + frictionRad));
        return new Factors((cotA + cotB) / (2 * d), (1 + cotB * cotBP) / d, (cotA + cotB) / d, (1 - cotA * cotBP) / d, nAcc);
    }

    /**
     * A wide blade (bucket lip, dozer blade, grader moldboard, scraper edge) of rake alpha (rad, 0..180 deg), cutting
     * depth d (m) and width w (m) at speed v (m/s) in a material, with a surcharge q (Pa) on the surface ahead (a soil
     * prism, {@link Blade#surchargePa}). Uses the material's soil strength even for rock (Mohr-Coulomb c); for the
     * specific-energy model of rock see {@link Digging#edge}.
     */
    public static Cut wide(Material m, double rakeRad, double depthM, double widthM, double speedMps, double surchargePa) {
        return wide(m.unitWeightNPerM3(), m.cohesionPa(), m.frictionRad(), m.wallFrictionRad(), m.adhesionPa(), surchargePa,
                rakeRad, depthM, widthM, speedMps);
    }

    /** A narrow tine or tooth (ripper, bucket tooth) in a material: {@link #wide(Material, double, double, double, double, double)} with the side crescents. */
    public static Cut narrow(Material m, double rakeRad, double depthM, double widthM, double speedMps, double surchargePa) {
        return narrow(m.unitWeightNPerM3(), m.cohesionPa(), m.frictionRad(), m.wallFrictionRad(), m.adhesionPa(), surchargePa,
                rakeRad, depthM, widthM, speedMps);
    }

    /**
     * A wide blade in soil given by numbers: unit weight (N/m^3), cohesion (Pa), friction and soil-metal friction (rad),
     * adhesion (Pa), surcharge (Pa); rake (rad), depth and width (m), speed (m/s, sign ignored). The mass density in the
     * acceleration term is the unit weight over {@link Material#G}.
     */
    public static Cut wide(double unitWeightNPerM3, double cohesionPa, double frictionRad, double wallFrictionRad, double adhesionPa,
                           double surchargePa, double rakeRad, double depthM, double widthM, double speedMps) {
        return cut(unitWeightNPerM3, cohesionPa, frictionRad, wallFrictionRad, adhesionPa, surchargePa, rakeRad, depthM, widthM, speedMps, false);
    }

    /** A narrow tine in soil given by numbers (as {@link #wide(double, double, double, double, double, double, double, double, double, double)}), with the side crescents. */
    public static Cut narrow(double unitWeightNPerM3, double cohesionPa, double frictionRad, double wallFrictionRad, double adhesionPa,
                             double surchargePa, double rakeRad, double depthM, double widthM, double speedMps) {
        return cut(unitWeightNPerM3, cohesionPa, frictionRad, wallFrictionRad, adhesionPa, surchargePa, rakeRad, depthM, widthM, speedMps, true);
    }

    private static Cut cut(double gamma, double c, double phi, double delta, double ca, double q, double rake, double d, double w,
                           double v, boolean narrow) {
        if (!(gamma >= 0) || !(c >= 0) || !(ca >= 0) || !(q >= 0) || !(d >= 0) || !(w >= 0) || !Double.isFinite(gamma) || !Double.isFinite(c)
                || !Double.isFinite(ca) || !Double.isFinite(q) || !Double.isFinite(d) || !Double.isFinite(w) || !Double.isFinite(v)) {
            throw new IllegalArgumentException("Unit weight, strengths, surcharge, depth and width finite and not negative, speed finite");
        }
        if (!(phi >= 0 && phi < Math.toRadians(80)) || !(delta >= 0 && delta < Math.PI / 2) || !(rake >= 0 && rake <= Math.PI)) {
            throw new IllegalArgumentException("Friction within [0, 80) deg, wall friction within [0, 90) deg, rake within [0, 180] deg");
        }
        Wedge f = new Wedge(gamma, c, phi, Math.min(delta, phi), ca, q, rake, d, w, Math.abs(v), narrow);
        double beta = f.leastForce();
        double p = f.perDepth(beta) * d;
        double h = p * f.sinAD + ca * d * w * f.cotA;
        double lift = ca * d * w - p * f.cosAD;
        return new Cut(Math.max(h, 0), lift, beta, d * w * Math.abs(v), w + 2 * f.spread(beta));
    }

    /** The wedge at one rake: the force per unit depth as a function of beta, and the search for its least value. */
    private static final class Wedge {
        final double gd, c, phi, delta, ca, q, rv2, a, w, d, cotA, sinA, sinAD, cosAD, cosPhi, betaHi;
        final boolean narrow;

        Wedge(double gamma, double c, double phi, double delta, double ca, double q, double rake, double d, double w, double v, boolean narrow) {
            this.c = c;
            this.phi = phi;
            this.delta = delta;
            this.ca = ca;
            this.q = q;
            this.w = w;
            this.d = d;
            this.narrow = narrow;
            gd = gamma * d;
            rv2 = gamma / Material.G * v * v;
            a = Math.clamp(rake, MIN_RAKE_RAD, Math.PI - delta - phi - CLOSURE_RAD);
            cotA = 1 / Math.tan(a);
            sinA = Math.sin(a);
            sinAD = Math.sin(a + delta);
            cosAD = Math.cos(a + delta);
            cosPhi = Math.cos(phi);
            betaHi = Math.min(BETA_HI, Math.PI - a - delta - phi - 1e-6);
        }

        /** Width the side crescents add on each side, m (0 for a wide blade). */
        double spread(double beta) {
            if (!narrow) { return 0; }
            double rd = cotA + 1 / Math.tan(beta);
            double ratio = Math.clamp(cotA / rd, 0.0, 1.0);
            return d * rd * Math.sqrt(1 - ratio * ratio);
        }

        /** Total force F (N) over the depth, at a failure angle. */
        double perDepth(double beta) {
            double cotB = 1 / Math.tan(beta), cotBP = 1 / Math.tan(beta + phi);
            double den = cosAD + sinAD * cotBP;
            double rd = cotA + cotB, s = spread(beta);
            double acc = sinA * cosPhi / (Math.sin(a + beta) * Math.sin(a + delta + beta + phi));
            return (gd * rd / 2 * (w + 2 * s / 3) + c * (1 + cotB * cotBP) * (w + s) + q * rd * (w + s) + ca * (1 - cotA * cotBP) * w) / den
                    + rv2 * acc * (w + s);
        }

        /** The failure angle of least force: the best of COARSE trial angles, then golden-section between its neighbours. */
        double leastForce() {
            double step = (betaHi - BETA_LO) / (COARSE - 1), best = Double.POSITIVE_INFINITY;
            int at = 0;
            for (int i = 0; i < COARSE; i++) {
                double value = perDepth(BETA_LO + i * step);
                if (value < best) { best = value; at = i; }
            }
            double lo = BETA_LO + Math.max(at - 1, 0) * step, hi = BETA_LO + Math.min(at + 1, COARSE - 1) * step;
            double x1 = hi - GOLDEN * (hi - lo), x2 = lo + GOLDEN * (hi - lo), f1 = perDepth(x1), f2 = perDepth(x2);
            for (int i = 0; i < REFINE; i++) {
                if (f1 < f2) {
                    hi = x2; x2 = x1; f2 = f1; x1 = hi - GOLDEN * (hi - lo); f1 = perDepth(x1);
                } else {
                    lo = x1; x1 = x2; f1 = f2; x2 = lo + GOLDEN * (hi - lo); f2 = perDepth(x2);
                }
            }
            return 0.5 * (lo + hi);
        }
    }
}
