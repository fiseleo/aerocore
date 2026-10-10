package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Blade-element momentum propeller with Prandtl tip and hub losses (Glauert; inflow-angle residual
 * after Ning 2014, Wind Energy 17:1327, DOI 10.1002/we.1636). Constant chord c/D and constant geometric
 * pitch P/D; annulus momentum assumes steady axisymmetric inflow with no hull wake.
 *
 * Sections are treated as fore-aft symmetric (bidirectional thruster blades), so reversing both rotation
 * and advance negates thrust and torque. Where the momentum equations have no root an annulus falls back
 * to blade elements without induction. Stalled inboard sections can have several momentum solutions; the
 * one nearest the uninduced inflow angle is used, so sample() can step by a few percent where a branch
 * vanishes (about 1 % near bollard pull, up to about 4 % near n = 0). tabulate() interpolates linearly and
 * is therefore continuous; there is no hysteresis or unsteady stall. With an angle-only section
 * polar the coefficients depend on beta alone: no Reynolds, cavitation, ventilation or oblique-flow effects.
 */
public record BladeElementPropeller(int blades, double hubRatio, double chordRatio, double pitchRatio,
                                    AirfoilPolar section, int annuli) implements PropellerModel {
    /** Thin-airfoil 2D section (lift slope 2 pi, CD0 0.01, no induced drag): illustrative, not measured. */
    public static final AirfoilPolar THIN_SECTION =
            new SymmetricAirfoil(2 * Math.PI, Math.toRadians(12), Math.toRadians(10), 0.01, 1e12, 1);
    private static final int SCAN = 96, BISECTIONS = 50;

    public BladeElementPropeller {
        if (blades < 2 || blades > 12) { throw new IllegalArgumentException("Blade count must be 2..12"); }
        if (!Double.isFinite(hubRatio) || hubRatio <= 0 || hubRatio >= 0.5) { throw new IllegalArgumentException("Hub ratio must be in (0, 0.5)"); }
        if (!Double.isFinite(chordRatio) || chordRatio <= 0 || chordRatio > 1) { throw new IllegalArgumentException("Chord ratio must be in (0, 1]"); }
        if (!Double.isFinite(pitchRatio) || pitchRatio <= 0 || pitchRatio > 3) { throw new IllegalArgumentException("Pitch ratio must be in (0, 3]"); }
        Objects.requireNonNull(section, "section");
        if (annuli < 4 || annuli > 200) { throw new IllegalArgumentException("Annulus count must be 4..200"); }
    }

    @Override public PropellerCoefficients sample(double beta) {
        if (!Double.isFinite(beta) || Math.abs(beta) > Math.PI) {
            throw new IllegalArgumentException("Hydrodynamic pitch angle must be in [-pi, pi]");
        }
        // (n, Va) -> (-n, -Va) maps beta -> beta +/- pi; symmetric blades then give negated loads.
        if (beta > Math.PI / 2) { return ahead(beta - Math.PI).negate(); }
        if (beta < -Math.PI / 2) { return ahead(beta + Math.PI).negate(); }
        return ahead(beta);
    }

    /** Uniformly spaced table over [-pi, pi] for fast in-game interpolation. */
    public PropellerTable tabulate(int points) {
        if (points < 3) { throw new IllegalArgumentException("Need at least three table points"); }
        List<PropellerTable.Point> table = new ArrayList<>();
        for (int i = 0; i < points; i++) {
            double beta = i == points - 1 ? Math.PI : -Math.PI + 2 * Math.PI * i / (points - 1);
            table.add(new PropellerTable.Point(beta, sample(beta)));
        }
        return new PropellerTable(table);
    }

    /** Non-dimensional solve: D = 1, rho = 1, Vr = 1, so Va = sin(beta) and 0.7 pi n D = cos(beta). */
    private PropellerCoefficients ahead(double beta) {
        double advance = Math.sin(beta), omega = 2 * Math.cos(beta) / 0.7;
        double tip = 0.5, hub = hubRatio * tip, width = (tip - hub) / annuli;
        double thrust = 0, torque = 0;
        for (int i = 0; i < annuli; i++) {
            double r = hub + (i + 0.5) * width;
            double[] load = annulus(r, advance, omega);
            thrust += load[0] * width;
            torque += load[1] * width;
        }
        double reference = 0.5 * Math.PI / 4;
        return new PropellerCoefficients(thrust / reference, torque / reference);
    }

    /** Returns dT/dr and dQ/dr for one annulus. */
    private double[] annulus(double r, double advance, double omega) {
        double tangential = omega * r;
        double pitch = Math.atan(pitchRatio / (2 * Math.PI * r));
        double solidity = blades * chordRatio / (2 * Math.PI * r);
        double phi = tangential > 1e-12 ? inflowAngle(r, advance, tangential, pitch, solidity) : Double.NaN;
        double axial, swirl;
        if (Double.isNaN(phi)) {
            axial = advance;
            swirl = tangential;
            phi = Math.atan2(axial, swirl);
        } else {
            swirl = tangential / (1 + factors(phi, r, pitch, solidity)[1]);
            axial = swirl * Math.tan(phi);
        }
        AeroCoefficients c = section.sample(wrap(pitch - phi));
        double normal = c.lift() * Math.cos(phi) - c.drag() * Math.sin(phi);
        double tangent = c.lift() * Math.sin(phi) + c.drag() * Math.cos(phi);
        double scale = 0.5 * (axial * axial + swirl * swirl) * blades * chordRatio;
        return new double[]{scale * normal, scale * tangent * r};
    }

    /**
     * Inflow angle phi in (-pi/2, pi/2) solving R(phi) = Vy (sin phi - sigma Cn / (4 F |sin phi|)) - Va cos phi = 0
     * with Vy = Omega r / (1 + k'), k' = sigma Ct / (4 F |sin phi| cos phi). It equates blade-element loads with
     * annulus momentum dT = 4 pi r rho u |Vx| F dr (and the matching torque). Using |Vx| extends momentum theory
     * continuously from the working state into the windmill-brake state; it is not valid inside the vortex-ring
     * state, where real flow is unsteady. R has a removable singularity at phi = 0, so static thrust is regular.
     * The root nearest the uninduced angle atan2(Va, Omega r) is chosen.
     */
    private double inflowAngle(double r, double advance, double tangential, double pitch, double solidity) {
        double target = Math.atan2(advance, tangential), best = Double.NaN;
        double previous = Double.NaN, previousPhi = 0;
        for (int i = 0; i < SCAN; i++) {
            // Offset grid never samples phi = 0 exactly.
            double phi = -Math.PI / 2 + (i + 0.5) * Math.PI / SCAN;
            double value = residual(phi, r, advance, tangential, pitch, solidity);
            if (!Double.isNaN(previous) && !Double.isNaN(value) && (previous < 0) != (value < 0)) {
                double root = bisect(previousPhi, phi, previous, r, advance, tangential, pitch, solidity);
                if (!Double.isNaN(root) && (Double.isNaN(best) || Math.abs(root - target) < Math.abs(best - target))) { best = root; }
            }
            previous = value;
            previousPhi = phi;
        }
        return best;
    }

    private double bisect(double low, double high, double lowValue, double r, double advance, double tangential, double pitch, double solidity) {
        for (int i = 0; i < BISECTIONS; i++) {
            double middle = 0.5 * (low + high);
            double value = residual(middle, r, advance, tangential, pitch, solidity);
            if (Double.isNaN(value)) { return Double.NaN; }
            if ((value < 0) == (lowValue < 0)) { low = middle; lowValue = value; } else { high = middle; }
        }
        double root = 0.5 * (low + high);
        // A sign change across a pole is not a root.
        double residual = residual(root, r, advance, tangential, pitch, solidity);
        return Math.abs(residual) <= 1e-7 * (tangential + Math.abs(advance)) ? root : Double.NaN;
    }

    private double residual(double phi, double r, double advance, double tangential, double pitch, double solidity) {
        double[] k = factors(phi, r, pitch, solidity);
        // Swirl may not reverse the tangential inflow; that branch is outside momentum theory.
        if (!(1 + k[1] > 0)) { return Double.NaN; }
        return tangential / (1 + k[1]) * (Math.sin(phi) - k[0]) - advance * Math.cos(phi);
    }

    /** k0 = sigma Cn / (4 F |sin phi|) and k1 = sigma Ct / (4 F |sin phi| cos phi). */
    private double[] factors(double phi, double r, double pitch, double solidity) {
        double sin = Math.sin(phi), cos = Math.cos(phi);
        if (sin == 0) { sin = Math.copySign(1e-12, phi); }
        AeroCoefficients c = section.sample(wrap(pitch - phi));
        double normal = c.lift() * cos - c.drag() * sin, tangent = c.lift() * sin + c.drag() * cos;
        double loss = loss(r, sin), denominator = 4 * loss * Math.abs(sin);
        return new double[]{solidity * normal / denominator, solidity * tangent / (denominator * cos)};
    }

    /** Prandtl tip and hub loss factors, F = (2/pi) acos(exp(-B (R - r) / (2 r sin phi))). */
    private double loss(double r, double sinPhi) {
        double tip = 0.5, hub = hubRatio * tip, s = Math.abs(sinPhi);
        double tipLoss = 2 / Math.PI * Math.acos(Math.exp(-blades * (tip - r) / (2 * r * s)));
        double hubLoss = 2 / Math.PI * Math.acos(Math.exp(-blades * (r - hub) / (2 * hub * s)));
        return Math.max(tipLoss * hubLoss, 1e-4);
    }

    private static double wrap(double angle) {
        double wrapped = Math.IEEEremainder(angle, 2 * Math.PI);
        return Math.clamp(wrapped, -Math.PI, Math.PI);
    }
}
