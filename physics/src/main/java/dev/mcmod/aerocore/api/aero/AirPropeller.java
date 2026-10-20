package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.PropellerCoefficients;
import dev.mcmod.aerocore.api.PropellerModel;

import java.util.Objects;

/**
 * An air propeller from its blades: blade-element momentum theory with Prandtl's tip and hub losses and swirl, the
 * inflow angle from Ning's single residual (Wind Energy 17, 2014; as BladeElementPropeller), the sections from
 * SectionPolar (root and tip sections blended along the radius) at each station's Reynolds and Mach number.
 * <ul>
 * <li>Blades: chord linear from hub to tip; blade angle of a constant geometric pitch P (beta = atan(P / (pi D x)) at
 *   radius fraction x) plus a linear extra twist, and a blade setting added to every station (a variable-pitch hub).</li>
 * <li>Coefficients in the four-quadrant form PropellerSolver uses: C_T* and C_Q* against the hydrodynamic pitch angle
 *   atan2(Va, 0.7 pi n D), so a stopped propeller (n = 0) is regular. Forward rotation only: for n &lt; 0 the loads are
 *   the negated forward ones at the mirrored angle (a rule).</li>
 * <li>The disk is axial: no normal force or P-factor at incidence, no slipstream contraction effects on the airframe,
 *   no blade flexing, no compressibility beyond the sections' own (SectionPolar's rough transonic drag).</li>
 * </ul>
 * Validation against NACA Report 640 in docs/WING.md.
 */
public final class AirPropeller {
    /** Radial stations of the blade-element sum. */
    public static final int STATIONS = 20;
    private static final int SEARCH = 90, FALSE_POSITION = 40;
    private static final double STEP = Math.toRadians(2);

    /**
     * Blade count, diameter (m), hub ratio, chord over diameter at the hub and the tip, geometric pitch over diameter
     * at 0.75 R (blade setting 0), extra twist from hub to tip (deg; negative washes out the tip), root and tip sections.
     */
    public record Geometry(int blades, double diameterM, double hubRatio, double rootChordRatio, double tipChordRatio, double pitchRatio,
                           double twistDeg, Airfoil root, Airfoil tip) {
        public Geometry {
            if (blades < 2 || blades > 8) { throw new IllegalArgumentException("Blade count must be 2..8"); }
            if (!(diameterM > 0) || !(hubRatio > 0.05 && hubRatio < 0.5)) { throw new IllegalArgumentException("Diameter or hub ratio out of range"); }
            if (!(rootChordRatio > 0 && rootChordRatio < 0.5 && tipChordRatio > 0 && tipChordRatio < 0.5)) { throw new IllegalArgumentException("Chord ratios must be in (0, 0.5)"); }
            if (!(pitchRatio > 0.1 && pitchRatio < 4) || !(Math.abs(twistDeg) <= 30)) { throw new IllegalArgumentException("Pitch ratio or twist out of range"); }
            Objects.requireNonNull(root, "root"); Objects.requireNonNull(tip, "tip");
        }
        /** Blade angle (rad) at radius fraction x for blade setting 0. */
        public double bladeAngle(double x) {
            return Math.atan(pitchRatio / (Math.PI * x)) + Math.toRadians(twistDeg) * (x - hubRatio) / (1 - hubRatio);
        }
        /** Chord over diameter at radius fraction x. */
        public double chordRatio(double x) { return rootChordRatio + (tipChordRatio - rootChordRatio) * (x - hubRatio) / (1 - hubRatio); }
        /** Blade activity factor 100000/16 integral (c/D) x^3 dx from the hub to the tip (per blade). */
        public double activityFactor() {
            double sum = 0; int k = 200;
            for (int i = 0; i < k; i++) { double x = hubRatio + (i + 0.5) * (1 - hubRatio) / k; sum += chordRatio(x) * x * x * x * (1 - hubRatio) / k; }
            return 100000.0 / 16 * sum;
        }
        /** Solidity: blade area over disk area. */
        public double solidity() {
            double sum = 0; int k = 200;
            for (int i = 0; i < k; i++) { double x = hubRatio + (i + 0.5) * (1 - hubRatio) / k; sum += chordRatio(x) * (1 - hubRatio) / k; }
            return blades * sum * 0.5 / (Math.PI / 4);
        }
    }

    private final Geometry g;
    private final SectionPolar rootPolar, tipPolar;
    private final double[] x = new double[STATIONS], chord = new double[STATIONS], angle = new double[STATIONS], weight = new double[STATIONS];
    private final double width;

    public AirPropeller(Geometry geometry) {
        this.g = Objects.requireNonNull(geometry, "geometry");
        rootPolar = SectionPolar.of(geometry.root());
        tipPolar = geometry.root().equals(geometry.tip()) ? rootPolar : SectionPolar.of(geometry.tip());
        width = (1 - geometry.hubRatio()) / STATIONS;
        for (int i = 0; i < STATIONS; i++) {
            x[i] = geometry.hubRatio() + (i + 0.5) * width;
            chord[i] = geometry.chordRatio(x[i]);
            angle[i] = geometry.bladeAngle(x[i]);
            weight[i] = (x[i] - geometry.hubRatio()) / (1 - geometry.hubRatio());
        }
    }

    public Geometry geometry() { return g; }

    /**
     * Four-quadrant coefficients at a hydrodynamic pitch angle (rad), blade setting (rad, added to every station), the
     * Mach number of the resultant speed at 0.7 R, and the Reynolds number of that speed on the diameter.
     */
    public PropellerCoefficients sample(double beta, double setting, double mach, double reynolds) {
        double b = Math.IEEEremainder(beta, 2 * Math.PI);
        if (b > Math.PI / 2) { return ahead(b - Math.PI, setting, mach, reynolds).negate(); }
        if (b < -Math.PI / 2) { return ahead(b + Math.PI, setting, mach, reynolds).negate(); }
        return ahead(b, setting, mach, reynolds);
    }

    /**
     * Classic coefficients at advance ratio J = V / (n D): {C_T = T / (rho n^2 D^4), C_P = P / (rho n^3 D^5), efficiency
     * J C_T / C_P}, for a tip speed (m/s), the air (density, viscosity, sound speed).
     */
    public double[] classic(double advanceRatio, double setting, double tipSpeedMps, Wing.Air air) {
        double nd = tipSpeedMps / Math.PI;                       // n D
        double v = advanceRatio * nd, vr = Math.hypot(v, 0.7 * Math.PI * nd);
        PropellerCoefficients c = sample(Math.atan2(v, 0.7 * Math.PI * nd), setting, vr / air.soundMps(),
                air.densityKgM3() * vr * g.diameterM() / air.viscosityPaS());
        double k = Math.PI * (advanceRatio * advanceRatio + 0.49 * Math.PI * Math.PI) / 8;
        double ct = c.thrust() * k, cp = 2 * Math.PI * c.torque() * k;
        return new double[]{ct, cp, cp > 0 && ct > 0 ? advanceRatio * ct / cp : 0};
    }

    /** Non-dimensional solve: D = 1, rho = 1, resultant at 0.7 R = 1, so Va = sin(beta) and 0.7 pi n = cos(beta). */
    private PropellerCoefficients ahead(double beta, double setting, double mach, double reynolds) {
        double va = Math.sin(beta), omega = 2 * Math.cos(beta) / 0.7;
        double thrust = 0, torque = 0;
        for (int i = 0; i < STATIONS; i++) {
            double r = 0.5 * x[i];
            double[] load = annulus(i, r, va, omega, setting, mach, reynolds);
            thrust += load[0] * 0.5 * width;
            torque += load[1] * 0.5 * width;
        }
        double reference = 0.5 * Math.PI / 4;
        return new PropellerCoefficients(thrust / reference, torque / reference);
    }

    /** dT/dr and dQ/dr at station i. */
    private double[] annulus(int i, double r, double va, double omega, double setting, double mach, double reynolds) {
        double tangential = omega * r, pitch = angle[i] + setting;
        double solidity = g.blades() * chord[i] / (2 * Math.PI * r);
        double phi = tangential > 1e-12 ? inflowAngle(i, r, va, tangential, pitch, solidity, mach, reynolds) : Double.NaN;
        double axial, swirl;
        if (Double.isNaN(phi)) {
            axial = va; swirl = tangential; phi = Math.atan2(axial, swirl);
        } else {
            swirl = tangential / (1 + factors(i, phi, r, pitch, solidity, Math.hypot(va, tangential), mach, reynolds)[1]);
            axial = swirl * Math.tan(phi);
        }
        double w = Math.hypot(axial, swirl);
        double[] c = section(i, pitch - phi, w, mach, reynolds);
        double normal = c[0] * Math.cos(phi) - c[1] * Math.sin(phi), tangent = c[0] * Math.sin(phi) + c[1] * Math.cos(phi);
        double scale = 0.5 * w * w * g.blades() * chord[i];
        return new double[]{scale * normal, scale * tangent * r};
    }

    /**
     * The inflow angle nearest the uninduced one with R(phi) = Vy (sin phi - k0) - Va cos phi = 0 (BladeElementPropeller):
     * brackets searched outward from the uninduced angle, then the Illinois false-position method.
     */
    private double inflowAngle(int i, double r, double va, double tangential, double pitch, double solidity, double mach, double reynolds) {
        double target = Math.atan2(va, tangential), w = Math.hypot(va, tangential);
        double limit = Math.PI / 2 - 1e-6;
        double center = Math.clamp(target, -limit, limit), fc = residual(i, center, r, va, tangential, pitch, solidity, w, mach, reynolds);
        if (fc == 0) { return center; }
        double upPrev = center, upVal = fc, downPrev = center, downVal = fc;
        for (int k = 1; k <= SEARCH; k++) {
            double d = STEP * k;
            // the nearer side first; each side keeps its last finite value
            for (int side = 0; side < 2; side++) {
                double phi = side == 0 ? center + d : center - d;
                if (phi > limit || phi < -limit) { continue; }
                double value = residual(i, phi, r, va, tangential, pitch, solidity, w, mach, reynolds);
                if (Double.isNaN(value)) { continue; }
                double prev = side == 0 ? upPrev : downPrev, prevVal = side == 0 ? upVal : downVal;
                if (!Double.isNaN(prevVal) && (prevVal < 0) != (value < 0)) {
                    double root = falsePosition(i, prev, prevVal, phi, value, r, va, tangential, pitch, solidity, w, mach, reynolds);
                    if (!Double.isNaN(root)) { return root; }
                }
                if (side == 0) { upPrev = phi; upVal = value; } else { downPrev = phi; downVal = value; }
            }
        }
        return Double.NaN;
    }

    private double falsePosition(int i, double a, double fa, double b, double fb, double r, double va, double tangential, double pitch,
                                 double solidity, double w, double mach, double reynolds) {
        int sideKept = 0;
        for (int k = 0; k < FALSE_POSITION; k++) {
            double c = (a * fb - b * fa) / (fb - fa), fc = residual(i, c, r, va, tangential, pitch, solidity, w, mach, reynolds);
            if (Double.isNaN(fc)) { return Double.NaN; }
            if (Math.abs(fc) <= 1e-9 * (tangential + Math.abs(va)) || Math.abs(b - a) < 1e-10) { return c; }
            if ((fc < 0) == (fb < 0)) { b = c; fb = fc; if (sideKept == -1) { fa /= 2; } sideKept = -1; }
            else { a = c; fa = fc; if (sideKept == 1) { fb /= 2; } sideKept = 1; }
        }
        // a sign change across a pole is not a root
        double c = 0.5 * (a + b), fc = residual(i, c, r, va, tangential, pitch, solidity, w, mach, reynolds);
        return Math.abs(fc) <= 1e-6 * (tangential + Math.abs(va)) ? c : Double.NaN;
    }

    private double residual(int i, double phi, double r, double va, double tangential, double pitch, double solidity, double w, double mach, double reynolds) {
        double[] k = factors(i, phi, r, pitch, solidity, w, mach, reynolds);
        if (!(1 + k[1] > 0)) { return Double.NaN; }
        return tangential / (1 + k[1]) * (Math.sin(phi) - k[0]) - va * Math.cos(phi);
    }

    /** k0 = sigma Cn / (4 F |sin phi|), k1 = sigma Ct / (4 F |sin phi| cos phi); w: the station's uninduced speed for Re and Mach. */
    private double[] factors(int i, double phi, double r, double pitch, double solidity, double w, double mach, double reynolds) {
        double sin = Math.sin(phi), cos = Math.cos(phi);
        if (sin == 0) { sin = Math.copySign(1e-12, phi); }
        double[] c = section(i, pitch - phi, w, mach, reynolds);
        double normal = c[0] * cos - c[1] * sin, tangent = c[0] * sin + c[1] * cos;
        double loss = loss(r, sin), denominator = 4 * loss * Math.abs(sin);
        return new double[]{solidity * normal / denominator, solidity * tangent / (denominator * cos)};
    }

    /** Prandtl's tip and hub loss factors. */
    private double loss(double r, double sinPhi) {
        double tip = 0.5, hub = g.hubRatio() * tip, s = Math.abs(sinPhi);
        double tipLoss = 2 / Math.PI * Math.acos(Math.exp(-g.blades() * (tip - r) / (2 * r * s)));
        double hubLoss = 2 / Math.PI * Math.acos(Math.exp(-g.blades() * (r - hub) / (2 * hub * s)));
        return Math.max(tipLoss * hubLoss, 1e-4);
    }

    /**
     * {cl, cd} of station i at an angle and local speed (in units of the 0.7 R resultant), with Snel's rotational stall
     * delay: a stalled section on a turning blade gets back 3 (c/r)^2 of its lost lift (Snel, Houwink and Bosschers,
     * ECN-C--94-107).
     */
    private double[] section(int i, double alpha, double w, double mach, double reynolds) {
        double re = reynolds * w * chord[i], m = mach * w;
        double[] a = rootPolar.coefficients(alpha, re, m);
        double cl, cd, attached;
        if (tipPolar == rootPolar) {
            cl = a[0]; cd = a[1]; attached = rootPolar.attachedLift(alpha, re, m);
        } else {
            double[] b = tipPolar.coefficients(alpha, re, m);
            double t = weight[i];
            cl = a[0] + t * (b[0] - a[0]); cd = a[1] + t * (b[1] - a[1]);
            attached = rootPolar.attachedLift(alpha, re, m) * (1 - t) + tipPolar.attachedLift(alpha, re, m) * t;
        }
        double cr = 2 * chord[i] / x[i];   // chord over radius
        if (Math.abs(attached) > Math.abs(cl) && Math.signum(attached) == Math.signum(cl)) { cl += Math.min(1, 3 * cr * cr) * (attached - cl); }
        return new double[]{cl, cd};
    }

    // ----- table for running in the game -----

    /** Blade settings, Mach numbers (resultant at 0.7 R) and pitch angles a table covers. */
    public record Grid(double minSetting, double maxSetting, int settings, double[] machs, double reynolds) {
        public Grid {
            if (settings < 1 || settings > 31 || maxSetting < minSetting) { throw new IllegalArgumentException("Bad setting range"); }
            if (machs.length < 1) { throw new IllegalArgumentException("Need a Mach number"); }
            machs = machs.clone();
        }
        double setting(int k) { return settings == 1 ? minSetting : minSetting + (maxSetting - minSetting) * k / (settings - 1); }
    }

    /** Pitch angles of a table: every 2.5 degrees from -45 to 90, every 15 beyond (radians). */
    static final double[] BETAS = betas();
    private static double[] betas() {
        java.util.List<Double> b = new java.util.ArrayList<>();
        for (int d = -180; d < -45; d += 15) { b.add((double) d); }
        for (double d = -45; d <= 90; d += 2.5) { b.add(d); }
        for (int d = 105; d <= 180; d += 15) { b.add((double) d); }
        double[] out = new double[b.size()];
        for (int i = 0; i < out.length; i++) { out[i] = Math.toRadians(b.get(i)); }
        return out;
    }

    /** C_T*, C_Q* over the grid's settings, Mach numbers and the pitch angles BETAS, for interpolation in the game. */
    public Table tabulate(Grid grid) {
        int ns = grid.settings(), nm = grid.machs().length, nb = BETAS.length;
        double[][][] t = new double[ns][nm][nb], q = new double[ns][nm][nb];
        for (int s = 0; s < ns; s++) {
            for (int m = 0; m < nm; m++) {
                for (int b = 0; b < nb; b++) {
                    PropellerCoefficients c = sample(BETAS[b], grid.setting(s), grid.machs()[m], grid.reynolds());
                    t[s][m][b] = c.thrust(); q[s][m][b] = c.torque();
                }
            }
        }
        return new Table(grid, t, q);
    }

    /** Tabulated coefficients; model() gives the four-quadrant model at one setting and Mach number. */
    public static final class Table {
        private final Grid grid;
        private final double[][][] thrust, torque;

        Table(Grid grid, double[][][] thrust, double[][][] torque) { this.grid = grid; this.thrust = thrust; this.torque = torque; }

        public Grid grid() { return grid; }

        /** The model at a blade setting (rad) and Mach number (held at the table's ends). */
        public PropellerModel model(double setting, double mach) {
            double s = grid.settings() == 1 ? 0 : Math.clamp((setting - grid.minSetting()) / (grid.maxSetting() - grid.minSetting()) * (grid.settings() - 1), 0, grid.settings() - 1);
            int s0 = Math.min((int) s, Math.max(grid.settings() - 2, 0)), s1 = Math.min(s0 + 1, grid.settings() - 1);
            double ws = s - s0;
            double[] machs = grid.machs();
            int mi = 0;
            while (mi < machs.length - 2 && mach > machs[mi + 1]) { mi++; }
            final int m0 = mi, m1 = Math.min(mi + 1, machs.length - 1);
            double wm = m1 == m0 ? 0 : Math.clamp((mach - machs[m0]) / (machs[m1] - machs[m0]), 0, 1);
            return beta -> {
                double b = Math.IEEEremainder(beta, 2 * Math.PI);
                int k = 0;
                while (k < BETAS.length - 2 && b > BETAS[k + 1]) { k++; }
                double wb = Math.clamp((b - BETAS[k]) / (BETAS[k + 1] - BETAS[k]), 0, 1);
                return new PropellerCoefficients(blend(thrust, s0, s1, ws, m0, m1, wm, k, wb), blend(torque, s0, s1, ws, m0, m1, wm, k, wb));
            };
        }

        private static double blend(double[][][] a, int s0, int s1, double ws, int m0, int m1, double wm, int k, double wb) {
            double v00 = lerp(a[s0][m0][k], a[s0][m0][k + 1], wb), v01 = lerp(a[s0][m1][k], a[s0][m1][k + 1], wb);
            double v10 = lerp(a[s1][m0][k], a[s1][m0][k + 1], wb), v11 = lerp(a[s1][m1][k], a[s1][m1][k + 1], wb);
            return lerp(lerp(v00, v01, wm), lerp(v10, v11, wm), ws);
        }
        private static double lerp(double a, double b, double w) { return a + w * (b - a); }
    }
}
