package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.FlightAtmosphere;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A wing from its planform and sections: spanwise loading by Phillips and Snyder's numerical lifting line (J. Aircraft
 * 37(4), 2000: a horseshoe vortex on each strip's quarter-chord segment, trailing legs aft along the wing's -x; each
 * strength makes the Kutta-Joukowski force of the local velocity, induced velocities included, equal the section's
 * lift at its local angle), solved by Newton's method with the sections' nonlinear polars (SectionPolar; root and tip
 * sections blended linearly along the span), so sweep, dihedral, twist, taper, the spread of stall and flaps all come
 * from the geometry and the polars.
 * <ul>
 * <li>Frame: x forward, y up, z starboard; the root's quarter-chord point at the origin; a mirrored wing has both
 *   halves, otherwise one half along +z (a fin is a half wing mounted on its side). Span is the plan-view span (tip to
 *   tip when mirrored), sweep that of the quarter-chord line; the sections are given streamwise (parallel to x).</li>
 * <li>Sweep: each section works in the plane normal to its quarter-chord line (simple sweep theory: the velocity
 *   component normal to the line, the chord c cos(sweep), the Reynolds and Mach numbers of that component), which with
 *   the lifting line reproduces DATCOM's swept-wing lift slope. The section's thickness ratio is not rescaled.</li>
 * <li>Strips: cosine-spaced along each half (finer at root and tip).</li>
 * <li>Control surfaces: trailing-edge flaps (SectionPolar.flapped) over a span range; antisymmetric ones (ailerons)
 *   deflect the starboard side by the negative input.</li>
 * <li>Ground effect: Wieselsberger's factor on the induced velocities (FlightAtmosphere.inducedDragFactor) for the
 *   root's height over the surface.</li>
 * <li>Supersonic (0.28): above Mach 1 the trailing vortices' downwash does not reach ahead of the Mach cones, so the
 *   induced velocities fade out from Mach 1 to 1.2 and each strip works as its section (SectionPolar: Ackeret's linear
 *   theory) times the loss inside the tip's Mach cone: averaged along the chord, the conical-flow tip solution
 *   (2/pi) asin sqrt(d beta / x) gives k = (2/pi) (asin sqrt(a) + sqrt(a (1 - a))), a = d beta / c, d the strip's
 *   distance from the tip (1 beyond a = 1); for a rectangular wing that is exactly the linear theory's
 *   C_L,alpha = 4 / beta (1 - 1 / (2 beta A)) (Ashley and Landahl, Aerodynamics of Wings and Bodies, ch. 9).</li>
 * </ul>
 * Not modelled: other wings' downwash (a tail sees the free stream), a propeller's slipstream over the wing, the
 * fuselage, trailing legs following the actual free stream (they stay along -x), unsteady (Theodorsen) lift and stall
 * hysteresis.
 */
public final class Wing {
    /** Strips on each half of the wing by default. */
    public static final int STRIPS = 12;
    /**
     * Newton iterations from a warm start (the vehicle's last step: in a stall the solve carries on over the steps)
     * and from a cold one.
     */
    private static final int ITERATIONS = 12, COLD_ITERATIONS = 40;
    private static final double TOLERANCE = 1e-6, WARM_TOLERANCE = 1e-4, DERIVATIVE_STEP = Math.toRadians(0.25);

    /**
     * The planform: plan-view span (tip to tip when mirrored, root to tip otherwise, m), root and tip chords (m),
     * quarter-chord sweep, dihedral, the root section's incidence and the tip's twist from it (deg, positive nose up;
     * washout is negative).
     */
    public record Planform(double spanM, double rootChordM, double tipChordM, double sweepDeg, double dihedralDeg,
                           double incidenceDeg, double twistDeg, boolean mirrored) {
        public Planform {
            if (!(spanM > 0) || !(rootChordM > 0) || !(tipChordM > 0) || !Double.isFinite(spanM + rootChordM + tipChordM)) {
                throw new IllegalArgumentException("Span and chords must be positive");
            }
            if (!(Math.abs(sweepDeg) < 75) || !(Math.abs(dihedralDeg) <= 60) || !(Math.abs(incidenceDeg) <= 30) || !(Math.abs(twistDeg) <= 30)) {
                throw new IllegalArgumentException("Sweep, dihedral, incidence or twist out of range");
            }
        }
        /** Plan-view length of one half (m). */
        public double semiSpanM() { return mirrored ? spanM / 2 : spanM; }
        /** Plan-view area (m^2). */
        public double areaM2() { return (mirrored ? 2 : 1) * semiSpanM() * (rootChordM + tipChordM) / 2; }
        /** Aspect ratio span^2 / area (a half wing's is that of the wing it would make with its mirror image). */
        public double aspectRatio() { double b = 2 * semiSpanM(); return b * b / (2 * semiSpanM() * (rootChordM + tipChordM) / 2); }
        /** Taper ratio tip / root chord. */
        public double taper() { return tipChordM / rootChordM; }
        /** Mean aerodynamic chord (m). */
        public double meanChordM() {
            double l = taper();
            return 2.0 / 3 * rootChordM * (1 + l + l * l) / (1 + l);
        }
    }

    /**
     * A trailing-edge control surface: the input role that moves it, its kind, chord fraction, span range (fraction of
     * the half span from the root), how far its trailing edge goes down and up at input +1 and -1 (deg), and whether it
     * is antisymmetric (an aileron: the starboard side moves by the negative input).
     */
    public record ControlSurface(String role, SectionPolar.FlapType type, double chordFraction, double fromEta, double toEta,
                                 double downDeg, double upDeg, boolean antisymmetric) {
        public ControlSurface {
            Objects.requireNonNull(role, "role"); Objects.requireNonNull(type, "type");
            if (!(chordFraction > 0 && chordFraction <= 0.6)) { throw new IllegalArgumentException("Control surface chord fraction must be in (0, 0.6]"); }
            if (!(fromEta >= 0 && toEta <= 1 && toEta > fromEta)) { throw new IllegalArgumentException("Control surface span range must lie in [0, 1]"); }
            if (!(downDeg >= 0 && downDeg <= 80 && upDeg >= 0 && upDeg <= 80)) { throw new IllegalArgumentException("Control surface travel must be in [0, 80] deg"); }
        }
        /** Trailing-edge-down deflection (rad) for an input in [-1, 1]. */
        public double deflection(double input) {
            double x = Math.clamp(Double.isFinite(input) ? input : 0, -1, 1);
            return Math.toRadians(x >= 0 ? x * downDeg : x * upDeg);
        }
    }

    /**
     * One strip: its quarter-chord segment ends a, b and middle, streamwise chord, plan area, aft chord and normal
     * directions (in the plane normal to the segment), position along the half span (0 root, 1 tip) and side (+1
     * starboard, -1 port).
     */
    public record Strip(Vec3d a, Vec3d b, Vec3d point, double chordM, double areaM2, Vec3d aft, Vec3d normal, double eta, int side) {}

    private final Planform planform;
    private final SectionPolar root, tip;
    private final List<ControlSurface> surfaces;
    private final int n;
    private final double[][] p, dl, ua, un, es;   // per strip: point, segment, aft, normal, span direction
    private final double[] chordN, areaN, eta;     // chord and area normal to the quarter-chord line
    private final int[] side, surface;             // control surface index or -1
    // velocity at strip i per unit strength of horseshoe j (3 per pair), and its products with strip i's directions
    private final double[] v, vxdl, va, vn, vs;
    private final List<Strip> strips;

    public Wing(Planform planform, Airfoil root, Airfoil tip, List<ControlSurface> surfaces) { this(planform, root, tip, surfaces, STRIPS); }

    public Wing(Planform planform, Airfoil root, Airfoil tip, List<ControlSurface> surfaces, int stripsPerHalf) {
        this.planform = Objects.requireNonNull(planform, "planform");
        this.root = SectionPolar.of(Objects.requireNonNull(root, "root"));
        this.tip = root.equals(Objects.requireNonNull(tip, "tip")) ? this.root : SectionPolar.of(tip);
        this.surfaces = List.copyOf(surfaces);
        if (stripsPerHalf < 2 || stripsPerHalf > 64) { throw new IllegalArgumentException("Strips per half must be 2..64"); }
        int h = stripsPerHalf;
        n = planform.mirrored() ? 2 * h : h;
        p = new double[n][]; dl = new double[n][]; ua = new double[n][]; un = new double[n][]; es = new double[n][];
        chordN = new double[n]; areaN = new double[n]; eta = new double[n]; side = new int[n]; surface = new int[n];
        double[][] ends = new double[2 * n][];
        List<Strip> list = new ArrayList<>();
        int k = 0;
        // port tip to root, then root to starboard tip, so every segment runs toward +z
        for (int s : planform.mirrored() ? new int[]{-1, 1} : new int[]{1}) {
            for (int q = 0; q < h; q++) {
                int idx = s < 0 ? h - 1 - q : q;               // strip index along the half from the root
                double e0 = station(idx, h), e1 = station(idx + 1, h), em = 0.5 * (e0 + e1);
                double[] a = quarterChord(s < 0 ? e1 : e0, s), b = quarterChord(s < 0 ? e0 : e1, s), mid = quarterChord(em, s);
                double[] seg = sub(b, a), span = unit(seg);
                // aft and normal directions in the plane normal to the quarter-chord line, then the section's twist
                double[] back = {-1, 0, 0};
                double along = dot(back, span);
                double[] aft0 = unit(new double[]{back[0] - along * span[0], back[1] - along * span[1], back[2] - along * span[2]});
                double[] normal0 = unit(cross(aft0, span));
                double twist = Math.toRadians(planform.incidenceDeg() + planform.twistDeg() * em);
                double ct = Math.cos(twist), st = Math.sin(twist);
                double[] aft = {ct * aft0[0] - st * normal0[0], ct * aft0[1] - st * normal0[1], ct * aft0[2] - st * normal0[2]};
                double[] normal = {st * aft0[0] + ct * normal0[0], st * aft0[1] + ct * normal0[1], st * aft0[2] + ct * normal0[2]};
                double c = planform.rootChordM() + (planform.tipChordM() - planform.rootChordM()) * em;
                double cosSweep = Math.sqrt(Math.max(0, 1 - along * along));
                p[k] = mid; dl[k] = seg; ua[k] = aft; un[k] = normal; es[k] = span;
                chordN[k] = c * cosSweep; areaN[k] = chordN[k] * norm(seg); eta[k] = em; side[k] = s;
                ends[2 * k] = a; ends[2 * k + 1] = b;
                surface[k] = -1;
                for (int i = 0; i < this.surfaces.size(); i++) {
                    ControlSurface cs = this.surfaces.get(i);
                    if (em >= cs.fromEta() && em <= cs.toEta()) { surface[k] = i; break; }
                }
                double plan = c * Math.abs(seg[2]);
                list.add(new Strip(vec(a), vec(b), vec(mid), c, plan, vec(aft), vec(normal), em, s));
                k++;
            }
        }
        strips = List.copyOf(list);
        // influence of each horseshoe (unit strength, legs to x = -infinity) at each strip's point; a strip's own bound
        // segment induces nothing there
        v = new double[3 * n * n]; vxdl = new double[3 * n * n]; va = new double[n * n]; vn = new double[n * n]; vs = new double[n * n];
        double[] down = {-1, 0, 0};
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double[] w = horseshoe(p[i], ends[2 * j], ends[2 * j + 1], down, i == j), c = cross(w, dl[i]);
                int ij = i * n + j;
                System.arraycopy(w, 0, v, 3 * ij, 3);
                System.arraycopy(c, 0, vxdl, 3 * ij, 3);
                va[ij] = dot(w, ua[i]); vn[ij] = dot(w, un[i]); vs[ij] = dot(w, es[i]);
            }
        }
    }

    /** Cosine-spaced station k of h strips along a half (0 root, 1 tip). */
    private static double station(int k, int h) { return 0.5 * (1 - Math.cos(Math.PI * k / h)); }

    /**
     * The quarter-chord line at a fraction of the half span: swept, but blended over about one root chord at the root
     * (a hyperbola, the tip where the straight line puts it), as Reid and Hunsaker do for swept lifting lines (J. Aircraft
     * 58(2), 2021); a sharp kink makes the bound vortices of the two halves load the root unrealistically.
     */
    private double[] quarterChord(double e, int s) {
        double y = e * planform.semiSpanM(), half = planform.semiSpanM(), d = planform.rootChordM();
        double bend = (Math.sqrt(y * y + d * d) - d) / (Math.sqrt(half * half + d * d) - d) * half;
        return new double[]{-bend * Math.tan(Math.toRadians(planform.sweepDeg())), y * Math.tan(Math.toRadians(planform.dihedralDeg())), s * y};
    }

    /**
     * Velocity at point per unit strength of the horseshoe a-b with legs along u (Phillips and Snyder, eq. 6):
     * (u x r2) / (r2 (r2 - u.r2)) + (r1 + r2)(r1 x r2) / (r1 r2 (r1 r2 + r1.r2)) - (u x r1) / (r1 (r1 - u.r1)), over 4 pi.
     */
    private static double[] horseshoe(double[] point, double[] a, double[] b, double[] u, boolean own) {
        double[] r1 = sub(point, a), r2 = sub(point, b);
        double l1 = norm(r1), l2 = norm(r2);
        double[] out = new double[3];
        double core = 1e-9;
        double d2 = l2 * (l2 - dot(u, r2));
        if (d2 > core) { double[] c = cross(u, r2); for (int k = 0; k < 3; k++) { out[k] += c[k] / d2; } }
        if (!own) {
            double d = l1 * l2 * (l1 * l2 + dot(r1, r2));
            if (Math.abs(d) > core) { double[] c = cross(r1, r2); for (int k = 0; k < 3; k++) { out[k] += (l1 + l2) * c[k] / d; } }
        }
        double d1 = l1 * (l1 - dot(u, r1));
        if (d1 > core) { double[] c = cross(u, r1); for (int k = 0; k < 3; k++) { out[k] -= c[k] / d1; } }
        for (int k = 0; k < 3; k++) { out[k] /= 4 * Math.PI; }
        return out;
    }

    public Planform planform() { return planform; }
    public SectionPolar rootPolar() { return root; }
    public SectionPolar tipPolar() { return tip; }
    public List<ControlSurface> surfaces() { return surfaces; }
    public List<Strip> strips() { return strips; }
    /** Number of strips (both halves). */
    public int size() { return n; }

    /**
     * The loads in the wing frame: total force (N) and moment about the origin (N m); per strip the vortex strength
     * (m^2/s, for warm starts), local angle of attack (rad, in the plane normal to the quarter-chord line), lift and
     * profile drag coefficients, normal force per unit span (N/m, along the strip's normal) and whether it is stalled;
     * the remaining residual over the dynamic pressure times area. profileForce is the sum of the sections' drag
     * (the rest of the force is the lift, induced drag included).
     */
    public record Solution(Vec3d force, Vec3d moment, Vec3d profileForce, double[] strength, double[] alpha, double[] cl, double[] cd,
                           double[] normalLoad, boolean[] stalled, double residual) {
        /** Share of the strips that are stalled. */
        public double stalledShare() {
            int s = 0;
            for (boolean b : stalled) { if (b) { s++; } }
            return stalled.length == 0 ? 0 : (double) s / stalled.length;
        }
    }

    /** The air: density (kg/m^3), dynamic viscosity (Pa s), speed of sound (m/s). */
    public record Air(double densityKgM3, double viscosityPaS, double soundMps) {
        public static final Air SEA_LEVEL = new Air(1.225, 1.789e-5, 340.3);
        public Air {
            if (!(densityKgM3 >= 0) || !(viscosityPaS > 0) || !(soundMps > 0)) { throw new IllegalArgumentException("Invalid air"); }
        }
    }

    /**
     * Solves the loading for the air's velocity relative to the wing at each strip's point (wing frame; the flow, so a
     * wing flying forward sees -x), control inputs by role, and the height of the root over a surface below (m;
     * infinite for none). warm: the strengths of a previous solution to start from, or null.
     */
    public Solution solve(Vec3d[] flow, Air air, Map<String, Double> inputs, double heightM, double[] warm) {
        return solve(flow, air, inputs, heightM, warm, warm != null);
    }

    private Solution solve(Vec3d[] flow, Air air, Map<String, Double> inputs, double heightM, double[] warm, boolean quick) {
        double ground = Double.isFinite(heightM) ? FlightAtmosphere.inducedDragFactor(heightM, 2 * planform.semiSpanM()) : 1;
        return solveScaled(flow, air, inputs, ground, warm, quick);
    }

    private Solution solveScaled(Vec3d[] flow, Air air, Map<String, Double> inputs, double inducedFactor, double[] warm, boolean quick) {
        if (flow.length != n) { throw new IllegalArgumentException("Need one flow velocity per strip"); }
        double[] vinf = new double[3 * n];
        for (int i = 0; i < n; i++) { vinf[3 * i] = flow[i].x(); vinf[3 * i + 1] = flow[i].y(); vinf[3 * i + 2] = flow[i].z(); }
        double[] delta = new double[n];
        for (int i = 0; i < n; i++) {
            if (surface[i] < 0) { continue; }
            ControlSurface cs = surfaces.get(surface[i]);
            double input = inputs == null ? 0 : inputs.getOrDefault(cs.role(), 0.0);
            delta[i] = cs.deflection(cs.antisymmetric() && side[i] > 0 ? -input : input);
        }
        return solve(vinf, air, delta, inducedFactor, warm, quick);
    }

    /** A uniform free stream at a speed (m/s), angle of attack and sideslip (rad), for benches. */
    public Solution solve(double speedMps, double alpha, double beta, Air air, Map<String, Double> inputs) {
        return solve(speedMps, alpha, beta, air, inputs, null);
    }

    /**
     * The same, starting from the strengths of the previous point of a sweep (or null), converged as tightly as a cold
     * start. Past the stall the lifting line has more than one solution; a sweep continued this way follows one branch
     * (as a wind tunnel run with rising angle) instead of jumping between them from point to point.
     */
    public Solution solve(double speedMps, double alpha, double beta, Air air, Map<String, Double> inputs, double[] previous) {
        Vec3d f = new Vec3d(-Math.cos(alpha) * Math.cos(beta), Math.sin(alpha) * Math.cos(beta), -Math.sin(beta)).scale(speedMps);
        Vec3d[] flow = new Vec3d[n];
        Arrays.fill(flow, f);
        return solve(flow, air, inputs, Double.POSITIVE_INFINITY, previous, false);
    }

    /**
     * 0.34: the same with the induced velocities scaled by a factor (below 1: a surface below,
     * FlightAtmosphere.inducedDragFactor, and end plates, RoadAero.endplateFactor, multiplied), for a car's wing tables.
     */
    public Solution solve(double speedMps, double alpha, double beta, Air air, Map<String, Double> inputs, double inducedFactor, double[] previous) {
        if (!(inducedFactor > 0) || inducedFactor > 1) { throw new IllegalArgumentException("Induced factor is in (0, 1]"); }
        Vec3d f = new Vec3d(-Math.cos(alpha) * Math.cos(beta), Math.sin(alpha) * Math.cos(beta), -Math.sin(beta)).scale(speedMps);
        Vec3d[] flow = new Vec3d[n];
        Arrays.fill(flow, f);
        return solveScaled(flow, air, inputs, inducedFactor, previous, false);
    }

    /** quick: warm from the vehicle's last step (a looser tolerance and fewer iterations; the next step carries on). */
    private Solution solve(double[] vinf, Air air, double[] delta, double ground, double[] warm, boolean quick) {
        double[] g = new double[n], alphaOut = new double[n], clOut = new double[n], cdOut = new double[n], loadOut = new double[n];
        boolean[] stalledOut = new boolean[n];
        double vmax = 0;
        for (int i = 0; i < n; i++) { vmax = Math.max(vmax, Math.sqrt(dot3(vinf, i, vinf, i))); }
        if (!(vmax > 1e-6) || !(air.densityKgM3() > 0)) {
            return new Solution(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO, g, alphaOut, clOut, cdOut, loadOut, stalledOut, 0);
        }
        // above Mach 1 the downwash fades out and the tips' Mach cones take lift off instead
        double mach = vmax / air.soundMps(), sup = Math.clamp((mach - 1) / 0.2, 0, 1);
        double[] tip = tipFactors(mach, sup);
        ground *= 1 - sup;
        boolean warmed = warm != null && warm.length == n, fast = quick && warmed;
        if (warmed) { System.arraycopy(warm, 0, g, 0, n); }
        else {
            for (int i = 0; i < n; i++) {
                double a = vinf[3 * i] * ua[i][0] + vinf[3 * i + 1] * ua[i][1] + vinf[3 * i + 2] * ua[i][2];
                double b = vinf[3 * i] * un[i][0] + vinf[3 * i + 1] * un[i][1] + vinf[3 * i + 2] * un[i][2];
                double speed = Math.sqrt(a * a + b * b);
                g[i] = 0.5 * speed * chordN[i] * section(i, Math.atan2(b, a), speed, air, delta[i], tip[i])[0];
            }
        }
        // a warm start from the last step only needs the forces to a part in ten thousand
        double scale = vmax * vmax * planform.areaM2() / n, tolerance = (fast ? WARM_TOLERANCE : TOLERANCE) * scale;
        State st = new State(n);
        residuals(st, g, vinf, air, delta, ground, tip);
        double[] trial = new double[n];
        for (int it = 0; it < (fast ? ITERATIONS : COLD_ITERATIONS) && st.norm > tolerance; it++) {
            double[][] jac = jacobian(st, g, air, delta, ground, tip);
            double[] step = solveLinear(jac, st.residual);
            State next = null;
            double lambda = 1;
            for (int t = 0; step != null && t < 5 && next == null; t++) {
                State s2 = trial(g, step, lambda, trial, vinf, air, delta, ground, tip, vmax);
                if (s2.norm < st.norm) { next = s2; } else { lambda *= 0.5; }
            }
            // Past the stall the sections' lift falls with angle, the Jacobian turns nearly singular and Newton's step
            // stops lowering the residuals: Levenberg and Marquardt's damped least-squares step does.
            if (next == null) { next = marquardt(jac, st, g, trial, vinf, air, delta, ground, tip, vmax); }
            if (next == null) { break; }
            System.arraycopy(trial, 0, g, 0, n);
            st = next;
        }
        // forces from the sections at the local velocities (the coefficients of the last residuals)
        double[] force = new double[3], moment = new double[3], profile = new double[3];
        for (int i = 0; i < n; i++) {
            double[] vi = st.velocity[i];
            double vs2 = dot(vi, es[i]);
            double[] perp = {vi[0] - vs2 * es[i][0], vi[1] - vs2 * es[i][1], vi[2] - vs2 * es[i][2]};
            double pp = dot(perp, perp);
            if (pp < 1e-12) { continue; }
            double speed = Math.sqrt(pp);
            double[] c = st.coefficients[i];
            double q = 0.5 * air.densityKgM3() * pp * areaN[i];
            double[] liftDir = unit(cross(vi, dl[i]));
            double[] f = new double[3];
            for (int k = 0; k < 3; k++) {
                double drag = q * c[1] * perp[k] / speed;
                f[k] = q * c[0] * liftDir[k] + drag;
                force[k] += f[k]; profile[k] += drag;
            }
            double[] r = cross(p[i], f);
            for (int k = 0; k < 3; k++) { moment[k] += r[k] + q * chordN[i] * c[2] * es[i][k]; }
            alphaOut[i] = st.alpha[i]; clOut[i] = c[0]; cdOut[i] = c[1]; stalledOut[i] = c[3] < 0.7;
            loadOut[i] = dot(f, un[i]) / norm(dl[i]);
        }
        return new Solution(vec(force), vec(moment), vec(profile), g, alphaOut, clOut, cdOut, loadOut, stalledOut, st.norm / scale);
    }

    /** The residuals at g - lambda step (each strength held within three times its strip's top speed times chord). */
    private State trial(double[] g, double[] step, double lambda, double[] trial, double[] vinf, Air air, double[] delta, double ground, double[] tip, double vmax) {
        for (int i = 0; i < n; i++) {
            double limit = 3 * vmax * chordN[i];
            trial[i] = Math.clamp(g[i] - lambda * step[i], -limit, limit);
        }
        State s2 = new State(n);
        residuals(s2, trial, vinf, air, delta, ground, tip);
        return s2;
    }

    /** (J^T J + mu diag J^T J) step = J^T R, mu raised tenfold from 1e-3 until the residuals fall (null if they never do). */
    private State marquardt(double[][] jac, State st, double[] g, double[] trial, double[] vinf, Air air, double[] delta, double ground, double[] tip, double vmax) {
        double[][] jtj = new double[n][n];
        double[] jtr = new double[n];
        for (int k = 0; k < n; k++) {
            double[] row = jac[k];
            double r = st.residual[k];
            for (int i = 0; i < n; i++) {
                if (row[i] == 0) { continue; }
                jtr[i] += row[i] * r;
                for (int j = 0; j < n; j++) { jtj[i][j] += row[i] * row[j]; }
            }
        }
        for (double mu = 1e-3; mu <= 1e5; mu *= 10) {
            double[][] a = new double[n][];
            for (int i = 0; i < n; i++) { a[i] = jtj[i].clone(); a[i][i] += mu * Math.max(jtj[i][i], 1e-12); }
            double[] step = solveLinear(a, jtr);
            if (step == null) { continue; }
            State s2 = trial(g, step, 1, trial, vinf, air, delta, ground, tip, vmax);
            if (s2.norm < st.norm) { return s2; }
        }
        return null;
    }

    /**
     * Newton's residuals R_i = 2 |V_i x dl_i| G_i - |V_i,n|^2 dA_i,n cl_i (V_i,n the velocity normal to the
     * quarter-chord line, dA_i,n the strip area normal to it), with the local velocities and section coefficients.
     */
    private void residuals(State st, double[] g, double[] vinf, Air air, double[] delta, double ground, double[] tip) {
        double norm = 0;
        for (int i = 0; i < n; i++) {
            double x = vinf[3 * i], y = vinf[3 * i + 1], z = vinf[3 * i + 2];
            int base = 3 * i * n;
            for (int j = 0; j < n; j++) {
                double gj = ground * g[j];
                x += v[base + 3 * j] * gj; y += v[base + 3 * j + 1] * gj; z += v[base + 3 * j + 2] * gj;
            }
            double[] vi = {x, y, z};
            st.velocity[i] = vi;
            double[] w = cross(vi, dl[i]);
            double wn = norm(w), sv = dot(vi, es[i]), pp = Math.max(dot(vi, vi) - sv * sv, 0);
            double a = dot(vi, ua[i]), b = dot(vi, un[i]);
            double alpha = Math.atan2(b, a), speed = Math.sqrt(pp);
            double[] c = section(i, alpha, speed, air, delta[i], tip[i]);
            st.alpha[i] = alpha; st.speed[i] = speed; st.coefficients[i] = c;
            st.residual[i] = 2 * wn * g[i] - pp * areaN[i] * c[0];
            norm += st.residual[i] * st.residual[i];
        }
        st.norm = Math.sqrt(norm / n);
    }

    /** The residuals' Jacobian at the state's point (Phillips and Snyder, eqs. 10-14, with the normal velocity). */
    private double[][] jacobian(State st, double[] g, Air air, double[] delta, double ground, double[] tip) {
        double[][] jac = new double[n][n];
        for (int i = 0; i < n; i++) {
            double[] vi = st.velocity[i];
            double[] w = cross(vi, dl[i]);
            double wn = norm(w), sv = dot(vi, es[i]), pp = st.speed[i] * st.speed[i];
            double a = dot(vi, ua[i]), b = dot(vi, un[i]), cl = st.coefficients[i][0];
            double slope = (section(i, st.alpha[i] + DERIVATIVE_STEP, st.speed[i], air, delta[i], tip[i])[0] - cl) / DERIVATIVE_STEP;
            double ab = a * a + b * b;
            double[] row = jac[i];
            for (int j = 0; j < n; j++) {
                int ij = i * n + j;
                double dw = wn > 1e-12 ? ground * (w[0] * vxdl[3 * ij] + w[1] * vxdl[3 * ij + 1] + w[2] * vxdl[3 * ij + 2]) / wn : 0;
                double dpp = 2 * ground * (vi[0] * v[3 * ij] + vi[1] * v[3 * ij + 1] + vi[2] * v[3 * ij + 2] - sv * vs[ij]);
                double da = ab > 1e-12 ? ground * (a * vn[ij] - b * va[ij]) / ab : 0;
                row[j] = 2 * dw * g[i] - dpp * areaN[i] * cl - pp * areaN[i] * slope * da;
            }
            row[i] += 2 * wn;
        }
        return jac;
    }

    /**
     * {cl, cd, cm, separation} of strip i at an angle, local (normal) speed and flap deflection, its lift and moment
     * times a factor (the supersonic tip loss).
     */
    private double[] section(int i, double alpha, double speed, Air air, double delta, double liftFactor) {
        double re = air.densityKgM3() * speed * chordN[i] / air.viscosityPaS(), mach = speed / air.soundMps();
        ControlSurface cs = surface[i] < 0 ? null : surfaces.get(surface[i]);
        SectionPolar.FlapType type = cs == null ? null : cs.type();
        double cf = cs == null ? 0 : cs.chordFraction();
        double[] r = root.flappedCoefficients(alpha, re, mach, type, cf, delta);
        if (tip != root) {
            double[] t = tip.flappedCoefficients(alpha, re, mach, type, cf, delta);
            double w = eta[i];
            r = new double[]{r[0] + w * (t[0] - r[0]), r[1] + w * (t[1] - r[1]), r[2] + w * (t[2] - r[2]), Math.min(r[3], t[3])};
        }
        if (liftFactor != 1) { r = new double[]{r[0] * liftFactor, r[1], r[2] * liftFactor, r[3]}; }
        return r;
    }

    /**
     * Each strip's share of its section's lift in the tips' Mach cones at a free-stream Mach number, blended in by sup
     * (0 subsonic, 1 from Mach 1.2): k = (2/pi) (asin sqrt(a) + sqrt(a (1 - a))), a = d beta / c (1 when a >= 1).
     */
    double[] tipFactors(double mach, double sup) {
        double[] k = new double[n];
        Arrays.fill(k, 1);
        if (sup <= 0) { return k; }
        double beta = Math.sqrt(Math.max(mach * mach - 1, 0));
        for (int i = 0; i < n; i++) {
            double c = planform.rootChordM() + (planform.tipChordM() - planform.rootChordM()) * eta[i];
            k[i] = 1 - sup * (1 - machConeShare((1 - eta[i]) * planform.semiSpanM() * beta / c));
        }
        return k;
    }

    /** The chord-averaged share of the two-dimensional lift at a = d beta / c from a tip (supersonic conical flow). */
    public static double machConeShare(double a) {
        if (a >= 1) { return 1; }
        if (!(a > 0)) { return 0; }
        return 2 / Math.PI * (Math.asin(Math.sqrt(a)) + Math.sqrt(a * (1 - a)));
    }

    private static final class State {
        final double[][] velocity, coefficients;
        final double[] residual, alpha, speed;
        double norm;
        State(int n) { velocity = new double[n][]; coefficients = new double[n][]; residual = new double[n]; alpha = new double[n]; speed = new double[n]; }
    }

    // ----- small vector and matrix helpers -----

    private static double[] sub(double[] a, double[] b) { return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    private static double dot3(double[] a, int i, double[] b, int j) { return a[3 * i] * b[3 * j] + a[3 * i + 1] * b[3 * j + 1] + a[3 * i + 2] * b[3 * j + 2]; }
    private static double norm(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] cross(double[] a, double[] b) {
        return new double[]{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
    private static double[] unit(double[] a) { double l = norm(a); return l > 0 ? new double[]{a[0] / l, a[1] / l, a[2] / l} : new double[3]; }
    private static Vec3d vec(double[] a) { return new Vec3d(a[0], a[1], a[2]); }

    /** Solves a x = b by Gaussian elimination with partial pivoting, or null when singular. */
    private static double[] solveLinear(double[][] a0, double[] b0) {
        int n = b0.length;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++) { a[i] = a0[i].clone(); }
        double[] b = b0.clone();
        for (int c = 0; c < n; c++) {
            int piv = c;
            for (int i = c + 1; i < n; i++) { if (Math.abs(a[i][c]) > Math.abs(a[piv][c])) { piv = i; } }
            if (Math.abs(a[piv][c]) < 1e-300) { return null; }
            double[] t = a[c]; a[c] = a[piv]; a[piv] = t; double tb = b[c]; b[c] = b[piv]; b[piv] = tb;
            for (int i = c + 1; i < n; i++) {
                double f = a[i][c] / a[c][c];
                if (f == 0) { continue; }
                for (int j = c; j < n; j++) { a[i][j] -= f * a[c][j]; }
                b[i] -= f * b[c];
            }
        }
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double s = b[i];
            for (int j = i + 1; j < n; j++) { s -= a[i][j] * x[j]; }
            x[i] = s / a[i][i];
        }
        for (double d : x) { if (!Double.isFinite(d)) { return null; } }
        return x;
    }
}
