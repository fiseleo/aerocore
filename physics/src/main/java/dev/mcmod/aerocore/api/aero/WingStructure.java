package dev.mcmod.aerocore.api.aero;

import java.util.Objects;

/**
 * A wing's structure under a design lift, for strength checks and mass (Megson, Aircraft Structures, ch. 16-21;
 * Niu, Airframe Structural Design, ch. 7):
 * <ul>
 * <li>Spanwise lift by Schrenk's approximation: the mean of a distribution proportional to the chord and an elliptic
 *   one of the same total (good to a few percent for unswept wings, NACA TM 948).</li>
 * <li>Shear and bending moment from the tip inward along the plan-view half span (sweep and dihedral ignored).</li>
 * <li>A box spar between the front and rear spar stations: two caps (the skin panels over the box, of the cap
 *   thickness) at a depth of 0.9 of the local section thickness, two shear webs; the rest of the skin and the ribs only
 *   add mass. Cap stress M / (A h), web shear V / (2 h t), tip deflection from M / (E I), I = A h^2 / 2, clamped at
 *   the root.</li>
 * </ul>
 * Not checked: buckling of the caps and webs (thin skins buckle well below their material's strength), torsion,
 * fatigue, joints and cut-outs.
 */
public final class WingStructure {
    private WingStructure() {}

    /** Stations along the half span. */
    public static final int STATIONS = 40;

    /**
     * Spar box between front and rear spar (fractions of the chord), cap and web thickness at the root (m; both thin
     * linearly to taper of their root value at the tip), skin thickness outside the box, rib spacing and thickness (m).
     */
    public record Box(double frontSpar, double rearSpar, double capThicknessM, double webThicknessM, double tipTaper,
                      double skinThicknessM, double ribSpacingM, double ribThicknessM) {
        public Box {
            if (!(frontSpar >= 0.05 && rearSpar <= 0.85 && rearSpar > frontSpar + 0.1)) { throw new IllegalArgumentException("Spar stations out of range"); }
            if (!(capThicknessM > 0 && webThicknessM > 0 && skinThicknessM > 0 && ribThicknessM > 0)) { throw new IllegalArgumentException("Thicknesses must be positive"); }
            if (!(tipTaper > 0 && tipTaper <= 1) || !(ribSpacingM > 0.05)) { throw new IllegalArgumentException("Taper or rib spacing out of range"); }
        }
    }

    /** Densities (kg/m^3) and stiffness (Pa) of the spar box's material and of the skin's and ribs'. */
    public record Materials(double boxDensity, double boxModulusPa, double skinDensity) {}

    /**
     * Root shear and moment of one half; the largest cap stress and web shear and where (fraction of the half span);
     * masses of the whole wing (both halves when mirrored) by item; tip deflection.
     */
    public record Result(double rootShearN, double rootMomentNm, double capStressPa, double capStressAt, double webShearPa, double webShearAt,
                         double capsKg, double websKg, double skinKg, double ribsKg, double tipDeflectionM) {
        public double massKg() { return capsKg + websKg + skinKg + ribsKg; }
    }

    /** The structure of a planform with its root and tip thickness ratios carrying a lift (N, the whole wing: both halves when mirrored). */
    public static Result analyze(Wing.Planform p, double rootThickness, double tipThickness, Box box, Materials m, double liftN) {
        Objects.requireNonNull(p, "planform"); Objects.requireNonNull(box, "box"); Objects.requireNonNull(m, "materials");
        double half = p.semiSpanM(), lift = (p.mirrored() ? 0.5 : 1) * Math.max(liftN, 0);
        double area = half * (p.rootChordM() + p.tipChordM()) / 2;
        int n = STATIONS;
        double dy = half / n;
        double[] y = new double[n], load = new double[n], c = new double[n], t = new double[n];
        for (int i = 0; i < n; i++) {
            y[i] = (i + 0.5) * dy;
            double eta = y[i] / half;
            c[i] = p.rootChordM() + (p.tipChordM() - p.rootChordM()) * eta;
            t[i] = rootThickness + (tipThickness - rootThickness) * eta;
            double planform = lift * c[i] / area, elliptic = 4 * lift / (Math.PI * half) * Math.sqrt(Math.max(0, 1 - eta * eta));
            load[i] = 0.5 * (planform + elliptic);   // N per m of span
        }
        // shear and moment at the inboard edge of each strip, from the tip in
        double[] shear = new double[n], moment = new double[n];
        double v = 0, mo = 0;
        for (int i = n - 1; i >= 0; i--) {
            double f = load[i] * dy;
            mo += v * dy + f * dy / 2;
            v += f;
            shear[i] = v; moment[i] = mo;
        }
        double capStress = 0, capAt = 0, webShear = 0, webAt = 0, caps = 0, webs = 0, skin = 0, ribs = 0;
        double[] curvature = new double[n];
        for (int i = 0; i < n; i++) {
            double eta = (double) i / n, k = 1 + (box.tipTaper() - 1) * eta;
            double h = 0.9 * t[i] * c[i], w = (box.rearSpar() - box.frontSpar()) * c[i];
            double tc = box.capThicknessM() * k, tw = box.webThicknessM() * k;
            double capArea = w * tc;
            double sigma = moment[i] / (capArea * h), tau = shear[i] / (2 * h * tw);
            if (sigma > capStress) { capStress = sigma; capAt = eta; }
            if (tau > webShear) { webShear = tau; webAt = eta; }
            curvature[i] = moment[i] / (m.boxModulusPa() * capArea * h * h / 2);
            caps += 2 * capArea * dy * m.boxDensity();
            webs += 2 * h * tw * dy * m.boxDensity();
            double perimeter = 2.04 * c[i] * Math.sqrt(1 + 2.0 * t[i] * t[i]);   // both surfaces of the section
            skin += Math.max(0, perimeter - 2 * w) * box.skinThicknessM() * dy * m.skinDensity();
            ribs += 0.685 * t[i] * c[i] * c[i] * box.ribThicknessM() * m.skinDensity() * dy / box.ribSpacingM();
        }
        // deflection: integrate the curvature twice from the clamped root
        double slope = 0, deflection = 0;
        for (int i = 0; i < n; i++) { slope += curvature[i] * dy; deflection += slope * dy; }
        double both = p.mirrored() ? 2 : 1;
        return new Result(shear[0], moment[0], capStress, capAt, webShear, webAt, both * caps, both * webs, both * skin, both * ribs, deflection);
    }
}
