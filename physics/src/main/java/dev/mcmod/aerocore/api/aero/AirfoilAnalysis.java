package dev.mcmod.aerocore.api.aero;

/**
 * A section's aerodynamics from its shape (chord 1, free stream 1), incompressible.
 * <ul>
 * <li>Inviscid flow: Hess and Smith's panel method (constant-strength sources on each panel, one vortex strength on
 *   all of them, the Kutta condition of equal tangential velocities at the trailing edge's two panels), as coded by
 *   Moran (An Introduction to Theoretical and Computational Aerodynamics, ch. 4).</li>
 * <li>Boundary layers on both surfaces from the stagnation point on the inviscid velocities: Thwaites' laminar method
 *   with the Cebeci and Bradshaw fits of H(lambda) and l(lambda); transition by Michel's criterion
 *   Re_theta = 1.174 (1 + 22400 / Re_x) Re_x^0.46, or at laminar separation (lambda below -0.09, the bubble taken to
 *   reattach turbulent); Head's entrainment method for the turbulent layer with Ludwieg and Tillmann's skin friction,
 *   separated where the shape factor passes SEPARATION_H.</li>
 * <li>No viscous-inviscid coupling (this is not XFOIL): the layers' displacement lowers the lift slope by
 *   viscousFactor (a fit to measured slopes against thickness and Reynolds number). Coupling by transpiration was tried
 *   during 0.22's development and diverged near the trailing edge, where the inviscid flow stagnates.</li>
 * <li>Stall: the suction side separates from the trailing edge (TRAILING_EDGE_SCALE times as far from it as Head's
 *   separation point: Kirchhoff's f is an effective separation fitted to lift, and the uncoupled layer separates too
 *   late; calibrated on the NACA 4412's maximum lift) or from the leading edge when
 *   the inviscid suction peak's pressure difference to the trailing edge passes Valarezo and Chin's critical value
 *   (criticalPressureDifference), whichever is further forward. The separation f (fraction of chord from the leading
 *   edge) enters Kirchhoff's flow model as in the Beddoes-Leishman static stall (Leishman, Principles of Helicopter
 *   Aerodynamics, ch. 9): the normal force is the attached one times ((1 + sqrt f) / 2)^2, the leading-edge suction
 *   times sqrt f; it does not fall below (1 - f) times a flat plate's (Lindenburg, ECN-C--03-025:
 *   Cn = Cd90 sin a / (0.56 + 0.44 sin a), Cd90 = 2 in two dimensions, Hoerner's flat plate).</li>
 * <li>Drag: Squire and Young's formula from each surface's trailing-edge momentum thickness and shape factor, plus
 *   the pressure drag of separation from the Kirchhoff forces.</li>
 * </ul>
 * Not modelled: the wake's displacement, laminar separation bubbles' own drag and bursting, unsteady effects. Meant for
 * angles within about 25 degrees of zero lift; SectionPolar continues to 180 degrees with a flat plate and adds
 * compressibility. Validation against Abbott and von Doenhoff in docs/WING.md.
 */
public final class AirfoilAnalysis {
    /** Panels used by the analysis. */
    public static final int PANELS = 160;
    /** Shape factor at which a turbulent layer separates (Head's method: 1.8 to 2.8 in the literature). */
    public static final double SEPARATION_H = 2.4;
    /**
     * Valarezo and Chin's critical pressure difference at a Reynolds number of 6 million and its power of the Reynolds
     * number, and how far past it (a share of it) the separation runs to the leading edge.
     */
    public static final double PDR_AT_6E6 = 15, PDR_EXPONENT = 0.1, STALL_WIDTH = 0.3;
    /** Separation ahead of this chord fraction freezes the layer for the friction drag. */
    private static final double FREEZE_BEFORE = 0.95;
    /** Kirchhoff's f is an effective separation (fitted to lift); it moves this many times as far as Head's separation point. */
    public static final double TRAILING_EDGE_SCALE = 2;
    /** A flat plate's normal force at 90 degrees in two dimensions (Hoerner, Fluid-Dynamic Drag, 3-16: 1.98). */
    public static final double FLAT_PLATE_CD90 = 2.0;

    private final Airfoil airfoil;
    private final int n;
    private final double[] xn, yn, xc, yc, len, theta, sin, cos;
    private final double[][] an, at;   // normal and tangential influence of the sources; last column the vortex
    private final double[][] lu;
    private final int[] pivot;

    public AirfoilAnalysis(Airfoil airfoil) {
        this.airfoil = airfoil;
        double[][] nodes = airfoil.nodes(PANELS);
        n = nodes.length - 1;
        xn = new double[n + 1]; yn = new double[n + 1];
        for (int i = 0; i <= n; i++) { xn[i] = nodes[i][0]; yn[i] = nodes[i][1]; }
        xc = new double[n]; yc = new double[n]; len = new double[n]; theta = new double[n]; sin = new double[n]; cos = new double[n];
        for (int j = 0; j < n; j++) {
            xc[j] = 0.5 * (xn[j] + xn[j + 1]); yc[j] = 0.5 * (yn[j] + yn[j + 1]);
            double dx = xn[j + 1] - xn[j], dy = yn[j + 1] - yn[j];
            len[j] = Math.hypot(dx, dy); theta[j] = Math.atan2(dy, dx); sin[j] = Math.sin(theta[j]); cos[j] = Math.cos(theta[j]);
        }
        an = new double[n][n + 1]; at = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double flog, ftan;
                if (i == j) { flog = 0; ftan = Math.PI; }
                else {
                    double dxj = xc[i] - xn[j], dxjp = xc[i] - xn[j + 1], dyj = yc[i] - yn[j], dyjp = yc[i] - yn[j + 1];
                    flog = 0.5 * Math.log((dxjp * dxjp + dyjp * dyjp) / (dxj * dxj + dyj * dyj));
                    ftan = Math.atan2(dyjp * dxj - dxjp * dyj, dxjp * dxj + dyjp * dyj);
                }
                double ct = Math.cos(theta[i] - theta[j]), st = Math.sin(theta[i] - theta[j]);
                an[i][j] = (st * flog + ct * ftan) / (2 * Math.PI);
                at[i][j] = (st * ftan - ct * flog) / (2 * Math.PI);
                an[i][n] += (ct * flog - st * ftan) / (2 * Math.PI);
                at[i][n] += (st * flog + ct * ftan) / (2 * Math.PI);
            }
        }
        // the system: n normal-velocity rows and the Kutta row
        double[][] a = new double[n + 1][n + 1];
        for (int i = 0; i < n; i++) { System.arraycopy(an[i], 0, a[i], 0, n + 1); }
        for (int j = 0; j <= n; j++) { a[n][j] = at[0][j] + at[n - 1][j]; }
        pivot = new int[n + 1];
        lu = decompose(a, pivot);
    }

    public Airfoil airfoil() { return airfoil; }

    private static double[][] decompose(double[][] a, int[] p) {
        int n = a.length;
        for (int i = 0; i < n; i++) { p[i] = i; }
        for (int c = 0; c < n; c++) {
            int best = c;
            for (int i = c + 1; i < n; i++) { if (Math.abs(a[i][c]) > Math.abs(a[best][c])) { best = i; } }
            double[] t = a[c]; a[c] = a[best]; a[best] = t;
            int tp = p[c]; p[c] = p[best]; p[best] = tp;
            for (int i = c + 1; i < n; i++) {
                double f = a[i][c] /= a[c][c];
                for (int j = c + 1; j < n; j++) { a[i][j] -= f * a[c][j]; }
            }
        }
        return a;
    }

    private double[] solve(double[] b) {
        int m = n + 1;
        double[] x = new double[m];
        for (int i = 0; i < m; i++) { x[i] = b[pivot[i]]; }
        for (int i = 0; i < m; i++) { for (int j = 0; j < i; j++) { x[i] -= lu[i][j] * x[j]; } }
        for (int i = m - 1; i >= 0; i--) { for (int j = i + 1; j < m; j++) { x[i] -= lu[i][j] * x[j]; } x[i] /= lu[i][i]; }
        return x;
    }

    /** Tangential velocities at the control points (positive along the panel direction) at an angle. */
    private double[] tangential(double alpha) {
        double[] b = new double[n + 1];
        for (int i = 0; i < n; i++) { b[i] = -Math.sin(alpha - theta[i]); }
        b[n] = -Math.cos(alpha - theta[0]) - Math.cos(alpha - theta[n - 1]);
        double[] q = solve(b), vt = new double[n];
        for (int i = 0; i < n; i++) {
            double v = Math.cos(alpha - theta[i]);
            for (int j = 0; j <= n; j++) { v += at[i][j] * q[j]; }
            vt[i] = v;
        }
        return vt;
    }

    /** One surface's boundary layer, from the stagnation point to the trailing edge. */
    record Layer(double[] s, double[] ue, double[] deltaStar, double thetaTe, double hTe, double ueTe, double separationX, double transitionX, int separatedFrom) {}

    /** The boundary layer on velocities ue (positive, from the stagnation point) at arc lengths s (s[0] > 0), chordwise stations x. */
    static Layer layer(double[] s, double[] ue, double[] x, double reynolds) {
        int m = s.length;
        double nu = 1 / reynolds;
        double[] dstar = new double[m];
        double integral = 0, sPrev = 0, uPrev = 0, th = 0, h = 2.59;
        boolean turbulent = false, separated = false;
        double separationX = 1, transitionX = 1;
        int separatedFrom = m;
        double y = 0;   // Head's Ue theta H1
        for (int k = 0; k < m; k++) {
            double u = Math.max(ue[k], 1e-6), ds = s[k] - sPrev;
            if (!turbulent) {
                // Thwaites: theta^2 Ue^6 = 0.45 nu integral Ue^5 ds (trapezoid, Ue rising from 0 at the stagnation point)
                integral += 0.5 * (Math.pow(uPrev, 5) + Math.pow(u, 5)) * ds;
                th = Math.sqrt(0.45 * nu * integral / Math.pow(u, 6));
                double dudx = k == 0 ? u / s[0] : (u - uPrev) / ds;
                double lambda = Math.clamp(th * th / nu * dudx, -0.1, 0.25);
                h = lambda >= 0 ? 2.61 - 3.75 * lambda + 5.24 * lambda * lambda : 2.088 + 0.0731 / (lambda + 0.14);
                double reX = u * s[k] / nu, reTheta = u * th / nu;
                boolean michel = reX > 1e4 && reTheta > 1.174 * (1 + 22400 / reX) * Math.pow(reX, 0.46);
                if (michel || lambda <= -0.09) {
                    turbulent = true; transitionX = x[k];
                    h = 1.4;
                    y = u * th * h1(h);
                }
            } else if (!separated) {
                // Head's method over this interval in sub-steps (Ue linear)
                int sub = Math.max(1, (int) Math.ceil(ds / 0.002));
                double hs = ds / sub;
                for (int q = 0; q < sub; q++) {
                    double u0 = uPrev + (u - uPrev) * q / sub, u1 = uPrev + (u - uPrev) * (q + 1) / sub, du = (u1 - u0) / hs;
                    double[] d1 = head(th, y, u0, du, nu), mid = {th + 0.5 * hs * d1[0], y + 0.5 * hs * d1[1]};
                    double[] d2 = head(mid[0], mid[1], 0.5 * (u0 + u1), du, nu);
                    th = Math.max(1e-9, th + hs * d2[0]);
                    y = Math.max(1e-12, y + hs * d2[1]);
                    h = hOf(y / (u1 * th));
                    if (h > SEPARATION_H) { separated = true; separationX = x[k]; separatedFrom = k; h = SEPARATION_H; break; }
                }
            } else if (separationX >= FREEZE_BEFORE) {
                // separated in the trailing edge's last few percent (where the inviscid flow stagnates): the momentum
                // thickness grows in the adverse gradient at the separation shape factor, for Squire and Young
                double du = (u - uPrev) / ds;
                th = Math.max(th, th * (1 - (h + 2) * du / Math.max(u, 1e-6) * ds));
            }
            // separated further forward: the layer is frozen at the separation point (its drag is the Kirchhoff pressure drag)
            dstar[k] = h * th;
            sPrev = s[k]; uPrev = u;
        }
        double ueEnd = separated && separationX < FREEZE_BEFORE ? ue[Math.max(separatedFrom - 1, 0)] : ue[m - 1];
        return new Layer(s, ue, dstar, th, Math.min(h, SEPARATION_H), Math.max(ueEnd, 1e-6), separationX, transitionX, separatedFrom);
    }

    /** Head's equations: d theta/ds and d(Ue theta H1)/ds. */
    private static double[] head(double th, double y, double u, double dudx, double nu) {
        double h1 = Math.max(3.32, y / (u * th)), hh = hOf(h1);
        double reTheta = Math.max(u * th / nu, 50);
        double cf = 0.246 * Math.pow(10, -0.678 * hh) * Math.pow(reTheta, -0.268);
        double dth = cf / 2 - (hh + 2) * th / u * dudx;
        double dy = u * 0.0306 * Math.pow(h1 - 3, -0.6169);
        return new double[]{dth, dy};
    }
    /** Head's H1(H) (Cebeci and Bradshaw's fit) and its inverse. */
    static double h1(double h) { return h <= 1.6 ? 0.8234 * Math.pow(h - 1.1, -1.287) + 3.3 : 1.5501 * Math.pow(h - 0.6778, -3.064) + 3.3; }
    static double hOf(double h1) {
        if (h1 <= 3.3001) { return 3.0; }
        double h = h1 >= 5.3 ? 0.86 * Math.pow(h1 - 3.3, -0.777) + 1.1 : 1.1538 * Math.pow(h1 - 3.3, -0.326) + 0.6778;
        return h;
    }

    /**
     * The section at an angle of attack (rad) and Reynolds number (on chord): lift, drag and quarter-chord moment
     * coefficients, the lowest pressure coefficient (inviscid, for compressibility), the attached (f = 1) lift, both
     * surfaces' separation and transition stations (fraction of chord; 1 when attached or laminar to the trailing
     * edge) and the separation f the forces use.
     */
    public record Point(double alpha, double cl, double cd, double cm, double cpMin, double attachedCl, double upperSeparation, double lowerSeparation,
                        double upperTransition, double lowerTransition, double separation) {}

    /** The inviscid solution at one angle, shared by the Reynolds numbers analysed at it. */
    public final class Inviscid {
        final double alpha, clInviscid, moment, cpMinUpper, cpMinLower, cpTe;
        final double[] vt;
        final int stag;

        private Inviscid(double alpha) {
            this.alpha = alpha;
            vt = tangential(alpha);
            stag = stagnation(vt);
            double fx = 0, fy = 0, m = 0, minU = 0, minL = 0;
            for (int i = 0; i < n; i++) {
                double cp = 1 - vt[i] * vt[i], nx = -sin[i], ny = cos[i];
                fx += -cp * nx * len[i]; fy += -cp * ny * len[i];
                m += -cp * len[i] * ((xc[i] - 0.25) * ny - yc[i] * nx);
                if (i < n / 2) { minL = Math.min(minL, cp); } else { minU = Math.min(minU, cp); }
            }
            clInviscid = fy * Math.cos(alpha) - fx * Math.sin(alpha);
            moment = -m;
            cpMinUpper = minU; cpMinLower = minL;
            cpTe = 0.5 * ((1 - vt[0] * vt[0]) + (1 - vt[n - 1] * vt[n - 1]));
        }

        /** Inviscid lift coefficient. */
        public double cl() { return clInviscid; }
        /** Lowest inviscid pressure coefficient on either surface. */
        public double cpMin() { return Math.min(cpMinUpper, cpMinLower); }
        /** The section at this angle and a Reynolds number. */
        public Point at(double reynolds) { return point(this, reynolds); }
    }

    /** The inviscid solution at an angle of attack (rad), to analyse at several Reynolds numbers. */
    public Inviscid inviscid(double alpha) {
        if (!Double.isFinite(alpha)) { throw new IllegalArgumentException("Angle must be finite"); }
        return new Inviscid(alpha);
    }

    /** The section at an angle of attack (rad) and Reynolds number (on chord). */
    public Point analyze(double alpha, double reynolds) { return inviscid(alpha).at(reynolds); }

    private Point point(Inviscid inv, double reynolds) {
        if (!(reynolds > 0) || !Double.isFinite(reynolds)) { throw new IllegalArgumentException("Reynolds number must be positive"); }
        double alpha = inv.alpha;
        Layer[] layers = layers(inv.vt, inv.stag, reynolds);
        Layer upper = layers[0], lower = layers[1];
        double ca = Math.cos(alpha), sa = Math.sin(alpha);
        double kv = viscousFactor(airfoil.thickness(), reynolds);
        double clAttached = kv * inv.clInviscid, cmAttached = kv * inv.moment;
        // the suction side's separation: from its trailing edge (the turbulent layer's), or a leading-edge stall when its
        // suction peak recovers too steeply (Valarezo and Chin's pressure difference rule)
        boolean upperSide = inv.clInviscid >= 0;
        double fTe = Math.max(0, 1 - TRAILING_EDGE_SCALE * (1 - (upperSide ? upper.separationX() : lower.separationX())));
        double dcp = inv.cpTe - (upperSide ? inv.cpMinUpper : inv.cpMinLower), critical = criticalPressureDifference(reynolds);
        double fLe = Math.clamp(1 - (dcp - critical) / (STALL_WIDTH * critical), 0, 1);
        double f = Math.min(fTe, fLe);
        // Kirchhoff: the normal force times ((1 + sqrt f) / 2)^2, the leading-edge suction times sqrt f; not below the
        // separated share of a flat plate's normal force
        double cn = clAttached * ca, cc = clAttached * sa;
        double factor = Math.pow((1 + Math.sqrt(f)) / 2, 2);
        double cnSep = cn * factor, ccSep = cc * Math.sqrt(f);
        double plate = (1 - f) * flatPlateNormal(alpha);
        if (plate > Math.abs(cnSep)) { cnSep = Math.copySign(plate, cnSep == 0 ? alpha : cnSep); }
        double cl = cnSep * ca + ccSep * sa, cdPressure = Math.max(0, cnSep * sa - ccSep * ca);
        double cdFriction = squireYoung(upper) + squireYoung(lower);
        // the centre of pressure moves aft as the suction side separates (to mid-chord when fully separated)
        double cm = cmAttached * factor - cnSep * 0.25 * (1 - f) * (1 - f);
        return new Point(alpha, cl, cdFriction + cdPressure, cm, inv.cpMin(), clAttached, upper.separationX(), lower.separationX(),
                upper.transitionX(), lower.transitionX(), f);
    }

    /** A flat plate's normal force coefficient (magnitude) at an angle (Lindenburg's fit to Hoerner). */
    public static double flatPlateNormal(double alpha) {
        double s = Math.abs(Math.sin(alpha));
        return FLAT_PLATE_CD90 * s / (0.56 + 0.44 * s);
    }

    /**
     * Lift slope of the real section over the inviscid one, for a thickness and Reynolds number: the boundary layer's
     * displacement decambers it, more for thicker sections and thicker (lower Reynolds number) layers. A fit to NACA
     * 0012 and 2412 data (Abbott and von Doenhoff; Sheldahl and Klimas): 0.875 for 12 % at 6 million.
     */
    public static double viscousFactor(double thickness, double reynolds) {
        double f = 1.04 + 0.30 * Math.log10(6e6 / Math.clamp(reynolds, 1e4, 1e9));
        return Math.clamp(1 - thickness * f, 0.6, 1);
    }

    /**
     * Valarezo and Chin's critical pressure difference (AIAA J. 31, 1994: stall when the suction peak's Cp less the
     * trailing edge's reaches it, from an inviscid pressure distribution), at low Mach number against the Reynolds number.
     */
    public static double criticalPressureDifference(double reynolds) {
        return Math.clamp(PDR_AT_6E6 * Math.pow(Math.clamp(reynolds, 1e4, 1e9) / 6e6, PDR_EXPONENT), 4, 16);
    }

    private static double squireYoung(Layer l) { return 2 * l.thetaTe() * Math.pow(l.ueTe(), (l.hTe() + 5) / 2); }

    /** Index of the last control point before the stagnation point (tangential velocity turning from negative to positive). */
    private int stagnation(double[] vt) {
        int best = n / 2;
        for (int i = 0; i < n - 1; i++) { if (vt[i] < 0 && vt[i + 1] >= 0) { return i; } }
        return best;
    }

    /** Both layers from the stagnation point between control points stag and stag + 1: {upper, lower}. */
    private Layer[] layers(double[] vt, int stag, double reynolds) {
        // stagnation point by linear interpolation of vt between the two control points
        double t = vt[stag] / (vt[stag] - vt[stag + 1]);
        double sStag = t * dist(stag, stag + 1);
        int nu = n - 1 - stag, nl = stag + 1;
        double[] su = new double[nu], uu = new double[nu], xu = new double[nu], sl = new double[nl], ul = new double[nl], xl = new double[nl];
        double acc = dist(stag, stag + 1) - sStag;
        for (int k = 0; k < nu; k++) {
            int i = stag + 1 + k;
            if (k > 0) { acc += dist(i - 1, i); }
            su[k] = Math.max(acc, 1e-6); uu[k] = Math.max(vt[i], 1e-6); xu[k] = xc[i];
        }
        acc = sStag;
        for (int k = 0; k < nl; k++) {
            int i = stag - k;
            if (k > 0) { acc += dist(i + 1, i); }
            sl[k] = Math.max(acc, 1e-6); ul[k] = Math.max(-vt[i], 1e-6); xl[k] = xc[i];
        }
        return new Layer[]{layer(su, uu, xu, reynolds), layer(sl, ul, xl, reynolds)};
    }

    private double dist(int i, int j) { return Math.hypot(xc[i] - xc[j], yc[i] - yc[j]); }

    /** The inviscid lift (no boundary layer) at an angle, for checks. */
    public double inviscidCl(double alpha) { return inviscid(alpha).cl(); }
}
