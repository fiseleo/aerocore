package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.Objects;

/**
 * A helicopter rotor (or tail rotor) from its blades: a blade-element sweep over azimuth and radius with the sections'
 * polars (SectionPolar: stall, reversed flow on the retreating side, compressibility on the advancing tip), first-harmonic
 * flapping, and momentum inflow (Leishman, Principles of Helicopter Aerodynamics, ch. 2-4, 10).
 * <ul>
 * <li>Frame: x forward, y up along the shaft, z starboard; the hub at the origin. Seen from above the blades turn
 *   counter-clockwise (clockwise rotors are computed mirrored). Azimuth psi is 0 over the tail and 90 degrees on the
 *   advancing (starboard) side.</li>
 * <li>Blade pitch theta0 + twist (r/R - 0.75) + theta1c cos psi + theta1s sin psi; linear chord, root and tip sections
 *   blended along the radius; tip loss by Prandtl's B = 1 - sqrt(2 C_T) / N_b (no lift outboard of B R), not below
 *   TIP_LOSS_MIN (the formula is for lightly loaded rotors; a slowly turning rotor's C_T from its tip speed is meaningless).</li>
 * <li>Flapping: beta0 + beta1c cos psi + beta1s sin psi about a hinge at offset e R with a spring K (a hingeless or
 *   teetering hub is an equivalent offset and spring), blade mass uniform from the hinge to the tip; harmonic balance of
 *   the aerodynamic flap moment (from the same sweep) against centrifugal, spring and gyroscopic moments
 *   (Johnson: beta'' + nu^2 beta = gamma M + 2/Omega (p cos psi - q sin psi)), one quasi-Newton step per call with the
 *   classic damping Jacobian (gamma / 8), so a vehicle stepping it converges over its steps.</li>
 * <li>Hub loads: the blades' aerodynamic forces, and the hub moment from the offset hinge's force and the spring; the
 *   shaft torque separately.</li>
 * <li>Inflow: uniform induced velocity from Glauert's momentum equation with the sweep's thrust, Drees' linear
 *   variation in forward flight (with the advance ratio held at DREES_MU_MAX, the model's range: a rotor barely turning
 *   has an advance ratio in the tens, which made the variation hundreds of times the mean and fed on itself), the empirical induced velocity curve through the vortex-ring state in descent
 *   (Leishman eq. 2.138), Cheeseman and Bennett's ground effect, and Pitt and Peters' first-order lag
 *   (tau = 0.849 R / (4 V_m)).</li>
 * </ul>
 * Not modelled: lag motion, blade elasticity, higher harmonics and vibration, the swashplate phase (with an offset hinge
 * the cyclic response is less than 90 degrees behind and couples the axes), interference with the fuselage and tail,
 * blade-vortex interaction, the rotor's own wake on other parts.
 */
public final class HelicopterRotor {
    /** Azimuth and radial stations of the sweep. */
    public static final int AZIMUTHS = 12, STATIONS = 10;
    /**
     * Induced power factor: the uniform inflow's induced velocity is raised by it (Leishman, ch. 3: 1.15 for a real
     * rotor's non-uniform inflow and tip losses beyond Prandtl's B).
     */
    public static final double INDUCED_FACTOR = 1.15;
    /** The advance ratio Drees' inflow variation is evaluated at, at most (helicopters fly below about 0.4). */
    public static final double DREES_MU_MAX = 0.5;
    /** Prandtl's tip loss factor, at least (about 0.97 at a hovering C_T of 0.006). */
    public static final double TIP_LOSS_MIN = 0.9;
    private static final double G = 9.81, TRIM_STEP = 0.05;

    /**
     * Blade count, radius (m), root cutout (fraction of R), chord at the cutout and the tip (m), linear twist from
     * root to tip (deg, negative washes out), root and tip sections, hinge offset (fraction of R), one blade's mass
     * (kg), flap spring (N m/rad), and whether it turns clockwise seen from above.
     */
    public record Geometry(int blades, double radiusM, double rootCutout, double rootChordM, double tipChordM, double twistDeg,
                           Airfoil root, Airfoil tip, double hingeOffset, double bladeMassKg, double flapSpringNmPerRad, boolean clockwise) {
        public Geometry {
            if (blades < 2 || blades > 8) { throw new IllegalArgumentException("Blade count must be 2..8"); }
            if (!(radiusM > 0) || !(rootCutout >= 0.05 && rootCutout < 0.5)) { throw new IllegalArgumentException("Radius or root cutout out of range"); }
            if (!(rootChordM > 0 && tipChordM > 0)) { throw new IllegalArgumentException("Chords must be positive"); }
            if (!(Math.abs(twistDeg) <= 30)) { throw new IllegalArgumentException("Twist out of range"); }
            if (!(hingeOffset >= 0 && hingeOffset < rootCutout)) { throw new IllegalArgumentException("Hinge offset must be in [0, cutout)"); }
            if (!(bladeMassKg > 0) || !(flapSpringNmPerRad >= 0)) { throw new IllegalArgumentException("Blade mass must be positive, spring non-negative"); }
            Objects.requireNonNull(root, "root"); Objects.requireNonNull(tip, "tip");
        }
        /** Chord at radius fraction x (m). */
        public double chordAt(double x) { return rootChordM + (tipChordM - rootChordM) * (x - rootCutout) / (1 - rootCutout); }
        /** Thrust-weighted solidity N_b c(0.75) / (pi R). */
        public double solidity() { return blades * chordAt(0.75) / (Math.PI * radiusM); }
        /** Disk area (m^2). */
        public double diskAreaM2() { return Math.PI * radiusM * radiusM; }
        /** One blade's flap inertia about the hinge (uniform from hinge to tip, kg m^2). */
        public double flapInertia() { double l = radiusM * (1 - hingeOffset); return bladeMassKg * l * l / 3; }
        /** One blade's first mass moment about the hinge (kg m). */
        public double flapMassMoment() { return bladeMassKg * radiusM * (1 - hingeOffset) / 2; }
    }

    /**
     * Blade pitch inputs (rad): collective and the cyclic terms on cos psi and sin psi, with psi the rotor's own azimuth
     * (from over the tail in its direction of turning).
     */
    public record Controls(double collective, double cyclicCos, double cyclicSin) {
        public static final Controls ZERO = new Controls(0, 0, 0);

        /**
         * From a pilot's sticks: collective (rad), longitudinal stick (+1 forward: the disk tilts forward), lateral stick
         * (+1 right: the disk tilts to starboard), the cyclic's travel (rad). With the flapping 90 degrees behind the
         * pitch, forward needs the least pitch on the advancing side (theta1s &lt; 0) for either direction of turning; to
         * starboard needs it over the tail when the blades turn counter-clockwise and over the nose when clockwise.
         */
        public static Controls fromSticks(double collective, double longitudinal, double lateral, double cyclicTravel, boolean clockwise) {
            double lon = Math.clamp(longitudinal, -1, 1), lat = Math.clamp(lateral, -1, 1);
            return new Controls(collective, -lat * cyclicTravel * (clockwise ? -1 : 1), -lon * cyclicTravel);
        }
    }

    /** What the rotor carries from step to step: uniform induced velocity (m/s, down through the disk) and flapping (rad). */
    public record State(double inducedMps, double beta0, double beta1c, double beta1s) {
        public static final State START = new State(0, 0, 0, 0);
    }

    /**
     * Loads in the rotor frame: force (N) and hub moment (N m, about the hub, without the shaft torque), the torque
     * the shaft must give to keep it turning (N m), thrust along the shaft (N), power (W), the coefficients
     * C_T = T / (rho A (Omega R)^2) and C_P, the state for the next call, and how much of the disk is stalled.
     */
    public record Loads(Vec3d force, Vec3d moment, double torqueNm, double thrustN, double powerW, double ct, double cp, State next,
                        double stalledShare) {}

    private final Geometry g;
    private final SectionPolar rootPolar, tipPolar;
    private final double[] x = new double[STATIONS], chord = new double[STATIONS], weight = new double[STATIONS];
    private final double dx, liftSlope;

    public HelicopterRotor(Geometry geometry) {
        this.g = Objects.requireNonNull(geometry, "geometry");
        rootPolar = SectionPolar.of(geometry.root());
        tipPolar = geometry.root().equals(geometry.tip()) ? rootPolar : SectionPolar.of(geometry.tip());
        dx = (1 - geometry.rootCutout()) / STATIONS;
        for (int j = 0; j < STATIONS; j++) {
            x[j] = geometry.rootCutout() + (j + 0.5) * dx;
            chord[j] = geometry.chordAt(x[j]);
            weight[j] = (x[j] - geometry.rootCutout()) / (1 - geometry.rootCutout());
        }
        double a = 0.5 * (rootPolar.liftSlope(3e6) + tipPolar.liftSlope(3e6));
        liftSlope = a;
    }

    public Geometry geometry() { return g; }

    /** Lock number rho a c R^4 / I_beta for an air density. */
    public double lockNumber(double density) { return density * liftSlope * g.chordAt(0.75) * Math.pow(g.radiusM(), 4) / g.flapInertia(); }

    /** Non-rotating flap frequency squared over Omega^2 at a rotor speed: 1 + (e R S_beta + K / Omega^2) / I_beta. */
    public double flapFrequencySquared(double omega) {
        double w2 = Math.max(omega * omega, 1e-6);
        return 1 + (g.hingeOffset() * g.radiusM() * g.flapMassMoment() + g.flapSpringNmPerRad() / w2) / g.flapInertia();
    }

    /**
     * One evaluation: hub velocity relative to the air and hub angular velocity (rotor frame), rotor speed (rad/s,
     * positive in its own direction), air, blade pitch, height of the hub over the ground (m; infinite for none), the
     * previous state and the time step (s) over which the inflow relaxes (0 or less: straight to its target).
     */
    public Loads loads(Vec3d hubVelocity, Vec3d hubRate, double omega, Wing.Air air, Controls controls, double heightM, State previous, double dt) {
        boolean mirror = g.clockwise();
        Vec3d u = mirror ? new Vec3d(hubVelocity.x(), hubVelocity.y(), -hubVelocity.z()) : hubVelocity;
        Vec3d w = mirror ? new Vec3d(-hubRate.x(), -hubRate.y(), hubRate.z()) : hubRate;
        Loads l = solve(u, w, Math.max(omega, 0), air, controls, heightM, previous == null ? State.START : previous, dt);
        if (!mirror) { return l; }
        State n = l.next();
        return new Loads(new Vec3d(l.force().x(), l.force().y(), -l.force().z()), new Vec3d(-l.moment().x(), -l.moment().y(), l.moment().z()),
                l.torqueNm(), l.thrustN(), l.powerW(), l.ct(), l.cp(), n, l.stalledShare());
    }

    private Loads solve(Vec3d u, Vec3d rate, double omega, Wing.Air air, Controls c, double heightM, State s, double dt) {
        double radius = g.radiusM(), area = g.diskAreaM2(), rho = air.densityKgM3();
        double om = Math.max(omega, 1e-3), tipSpeed = om * radius;
        double vh2 = u.x() * u.x() + u.z() * u.z(), vh = Math.sqrt(vh2), vc = u.y();   // in-plane speed, climb speed
        // the rear of the disk (downstream) for Drees' inflow, and the wake skew
        double cosRear = vh > 1e-6 ? u.x() / vh : 1, sinRear = vh > 1e-6 ? -u.z() / vh : 0;
        double vi = Math.max(s.inducedMps(), 0);
        double lambda = (vc + vi) / tipSpeed, mu = vh / tipSpeed;
        double skew = Math.atan2(mu, Math.max(lambda, 1e-6));
        double md = Math.min(mu, DREES_MU_MAX);
        double kx = skew > 1e-3 ? 4.0 / 3 * (1 - Math.cos(skew) - 1.8 * md * md) / Math.sin(skew) : 0, ky = -2 * md;
        double ground = groundFactor(heightM, vh, vi);
        double tipLoss = Math.max(1 - Math.sqrt(2 * Math.max(previousCt(s, rho, area, tipSpeed), 0)) / g.blades(), TIP_LOSS_MIN);
        double e = g.hingeOffset() * radius;
        double b0 = s.beta0(), b1c = s.beta1c(), b1s = s.beta1s();
        double[] force = new double[3], moment = new double[3];
        double torque = 0, m0 = 0, m1c = 0, m1s = 0;
        int stalled = 0, elements = 0;
        double perAzimuth = (double) g.blades() / AZIMUTHS;
        for (int k = 0; k < AZIMUTHS; k++) {
            double psi = 2 * Math.PI * (k + 0.5) / AZIMUTHS, cp = Math.cos(psi), sp = Math.sin(psi);
            double beta = b0 + b1c * cp + b1s * sp, dbeta = om * (-b1c * sp + b1s * cp);
            double cb = Math.cos(beta), sb = Math.sin(beta);
            double[] er = {-cp, 0, sp}, t = {sp, 0, cp};
            double[] eb = {er[0] * cb, sb, er[2] * cb}, nf = {-er[0] * sb, cb, -er[2] * sb};
            double theta0 = c.collective() + c.cyclicCos() * cp + c.cyclicSin() * sp;
            double relRear = Math.cos(psi) * cosRear + Math.sin(psi) * sinRear, relSide = Math.sin(psi) * cosRear - Math.cos(psi) * sinRear;
            double[] blade = new double[3];
            double flap = 0;
            for (int j = 0; j < STATIONS; j++) {
                double r = x[j] * radius, arm = r - e;
                double[] p = {e * er[0] + arm * eb[0], arm * eb[1], e * er[2] + arm * eb[2]};
                // element velocity: hub, hub rotation, blade rotation, flapping; the air's: the induced flow down the shaft
                double[] rot = {om * p[2], 0, -om * p[0]};   // Omega y x p
                double[] hr = cross(new double[]{rate.x(), rate.y(), rate.z()}, p);
                double induced = ground * vi * (1 + kx * x[j] * relRear + ky * x[j] * relSide);
                double wx = -(u.x() + hr[0] + rot[0] + arm * dbeta * nf[0]);
                double wy = -(u.y() + hr[1] + rot[1] + arm * dbeta * nf[1]) - induced;
                double wz = -(u.z() + hr[2] + rot[2] + arm * dbeta * nf[2]);
                double ut = -(wx * t[0] + wy * t[1] + wz * t[2]), up = wx * nf[0] + wy * nf[1] + wz * nf[2];
                double speed2 = ut * ut + up * up;
                if (speed2 < 1e-12) { continue; }
                double speed = Math.sqrt(speed2);
                double pitch = theta0 + Math.toRadians(g.twistDeg()) * (x[j] - 0.75) / (1 - g.rootCutout());
                double alpha = pitch + Math.atan2(up, ut);
                double[] co = section(j, alpha, speed, air);
                double cl = x[j] > tipLoss ? 0 : co[0];
                if (co[2] < 0.7) { stalled++; }
                elements++;
                // flow direction in the section plane, lift = flow x span
                double[] f = {(-ut * t[0] + up * nf[0]) / speed, (-ut * t[1] + up * nf[1]) / speed, (-ut * t[2] + up * nf[2]) / speed};
                double[] lift = cross(f, eb);
                double q = 0.5 * rho * speed2 * chord[j] * dx * radius;
                double[] df = {q * (cl * lift[0] + co[1] * f[0]), q * (cl * lift[1] + co[1] * f[1]), q * (cl * lift[2] + co[1] * f[2])};
                for (int a = 0; a < 3; a++) { blade[a] += df[a]; }
                double[] pm = cross(p, df);
                torque -= perAzimuth * pm[1];
                flap += arm * (df[0] * nf[0] + df[1] * nf[1] + df[2] * nf[2]);
            }
            for (int a = 0; a < 3; a++) { force[a] += perAzimuth * blade[a]; }
            // hub moment: the hinge's force at the offset, and the spring (flap axis -t)
            double[] hm = cross(new double[]{e * er[0], 0, e * er[2]}, blade);
            double spring = g.flapSpringNmPerRad() * beta;
            for (int a = 0; a < 3; a++) { moment[a] += perAzimuth * (hm[a] - spring * t[a]); }
            m0 += flap / AZIMUTHS; m1c += 2 * flap * cp / AZIMUTHS; m1s += 2 * flap * sp / AZIMUTHS;
        }
        double thrust = force[1];
        moment[1] = 0;
        // flapping: one quasi-Newton step of the harmonic balance
        State next;
        double ct = thrust / (rho * area * tipSpeed * tipSpeed);
        double viTarget = inducedTarget(thrust, rho, area, vh, vc, vi);
        double vm = Math.sqrt(vh2 + (vc + vi) * (vc + vi)), tau = 0.849 * radius / (4 * Math.max(vm, 0.5));
        double viNext = dt > 0 ? vi + dt / (tau + dt) * (viTarget - vi) : viTarget;
        if (omega > 1) {
            double ib = g.flapInertia(), w2 = omega * omega, nu2 = flapFrequencySquared(omega), gamma = lockNumber(rho);
            double r0 = nu2 * b0 - (m0 - G * g.flapMassMoment()) / (ib * w2);
            double r1c = (nu2 - 1) * b1c - m1c / (ib * w2) - 2 * rate.x() / omega;
            double r1s = (nu2 - 1) * b1s - m1s / (ib * w2) + 2 * rate.z() / omega;
            double d = nu2 - 1, k8 = gamma / 8, det = d * d + k8 * k8;
            double db0 = -r0 / nu2;
            double db1c = -(d * r1c - k8 * r1s) / det, db1s = -(k8 * r1c + d * r1s) / det;
            next = new State(viNext, clampFlap(b0 + db0), clampFlap(b1c + db1c), clampFlap(b1s + db1s));
        } else {
            next = new State(viNext, 0, 0, 0);
        }
        double power = torque * omega;
        double cpower = power / (rho * area * Math.pow(tipSpeed, 3));
        return new Loads(new Vec3d(force[0], force[1], force[2]), new Vec3d(moment[0], 0, moment[2]), torque, thrust, power, ct, cpower, next,
                elements == 0 ? 0 : (double) stalled / elements);
    }

    private static double clampFlap(double b) { return Math.clamp(b, -0.5, 0.5); }

    private double previousCt(State s, double rho, double area, double tipSpeed) {
        // the momentum thrust that goes with the carried inflow, for the tip loss
        double vi = Math.max(s.inducedMps(), 0);
        return 2 * vi * vi / (tipSpeed * tipSpeed);
    }

    /** Cheeseman and Bennett: the induced velocity in ground effect over that out of it, for a hub height (m). */
    private double groundFactor(double heightM, double vh, double vi) {
        if (!Double.isFinite(heightM) || heightM <= 0) { return heightM <= 0 ? 0.75 : 1; }
        double k = Math.min(g.radiusM() / (4 * heightM), 0.5);
        double ratio = vi > 1e-6 ? vh / vi : 0;
        return 1 - k * k / (1 + ratio * ratio);
    }

    /**
     * The uniform induced velocity for a thrust: Glauert's v = T / (2 rho A sqrt(Vh^2 + (Vc + v)^2)) by Newton's
     * method, times the induced power factor,
     * blended at low forward speed in descent (-2 vh &lt; Vc &lt; 0) into the empirical vortex-ring curve
     * v / vh = 1.15 - 1.125 x - 1.372 x^2 - 1.718 x^3 - 0.655 x^4, x = Vc / vh (Leishman eq. 2.138).
     */
    static double inducedTarget(double thrust, double rho, double area, double vh, double vc, double guess) {
        if (thrust <= 0 || rho <= 0) { return 0; }
        double hover = Math.sqrt(thrust / (2 * rho * area));
        double v = Math.max(guess / INDUCED_FACTOR, hover * 0.5);
        for (int k = 0; k < 30; k++) {
            double m = Math.sqrt(vh * vh + (vc + v) * (vc + v));
            double f = v - thrust / (2 * rho * area * Math.max(m, 1e-6));
            double df = 1 + thrust / (2 * rho * area) * (vc + v) / Math.pow(Math.max(m, 1e-6), 3);
            double step = f / Math.max(df, 0.2);
            v = Math.max(v - step, 0);
            if (Math.abs(step) < 1e-6 * hover) { break; }
        }
        v *= INDUCED_FACTOR;
        // the empirical curve already holds the factor (1.15 in hover)
        double xr = vc / hover;
        if (xr < 0 && xr > -2) {
            double ring = hover * (1.15 - 1.125 * xr - 1.372 * xr * xr - 1.718 * xr * xr * xr - 0.655 * xr * xr * xr * xr);
            double w = Math.clamp(1 - vh / hover, 0, 1);
            v = w * ring + (1 - w) * v;
        }
        return v;
    }

    /**
     * Steady flight at a hub velocity (rotor frame), rotor speed, air and pitch: the loads after the flapping and inflow
     * have converged (for benches).
     */
    public Loads trimmed(Vec3d hubVelocity, double omega, Wing.Air air, Controls controls, double heightM) {
        State s = State.START;
        Loads l = null;
        for (int k = 0; k < 400; k++) {
            l = loads(hubVelocity, Vec3d.ZERO, omega, air, controls, heightM, s, TRIM_STEP);
            State n = l.next();
            double change = Math.abs(n.inducedMps() - s.inducedMps()) / Math.max(1, Math.abs(n.inducedMps()))
                    + Math.abs(n.beta0() - s.beta0()) + Math.abs(n.beta1c() - s.beta1c()) + Math.abs(n.beta1s() - s.beta1s());
            s = n;
            if (change < 1e-7) { break; }
        }
        return l;
    }

    /** {cl, cd, separation} of station j at an angle and speed. */
    private double[] section(int j, double alpha, double speed, Wing.Air air) {
        double re = air.densityKgM3() * speed * chord[j] / air.viscosityPaS(), mach = speed / air.soundMps();
        double[] a = rootPolar.coefficients(alpha, re, mach);
        if (tipPolar == rootPolar) { return new double[]{a[0], a[1], a[3]}; }
        double[] b = tipPolar.coefficients(alpha, re, mach);
        double t = weight[j];
        return new double[]{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), Math.min(a[3], b[3])};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
}
