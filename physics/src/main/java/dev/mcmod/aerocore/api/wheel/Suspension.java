package dev.mcmod.aerocore.api.wheel;

import dev.mcmod.aerocore.api.wheel.kinematics.KinematicTable;

import java.util.Objects;

/**
 * A wheel's suspension (0.24): how it moves (travel along the body's vertical, roll centre, camber change) and what
 * holds it (a spring, a damper, bump and droop stops). The wheel itself has no mass of its own here (its mass rides
 * with the body): the tyre is a spring in series with the suspension, and the node between them has no inertia, so
 * the suspension's compression c is a state that follows k_t (P - c) = F_s(c) + c_d dc/dt, P the compression the
 * ground forces on tyre and suspension together.
 *
 * <p>Geometry (front view): a lever of length L (the front-view swing arm, to the instant centre at height h_ic)
 * gives a roll centre at h_ic (t / 2) / L on the centreline and a camber gain of 1 / L per metre of bump (top in);
 * equal parallel arms (L infinite) keep the wheel parallel to the body, a solid axle keeps it square to the ground
 * (Gillespie, Fundamentals of Vehicle Dynamics, ch. 6-7). The types give the usual layout of each:
 * <ul>
 * <li>double wishbone and multi-link: L and h_ic as designed (default 2 m and 0.08 m: a roll centre near 0.05 m);</li>
 * <li>MacPherson strut: default L 1.5 m, h_ic 0.1 m;</li>
 * <li>trailing arm: no camber change against the body, roll centre on the ground;</li>
 * <li>semi-trailing arm: the pivot axis swept by an angle (15-25 degrees, Dixon) acts as a swing arm of the arm's length
 *     over the sine of the sweep, with its instant centre at the pivot's height;</li>
 * <li>swing axle: pivots beside the differential at hub height, L the half-shaft: the roll centre stands high (wheel
 *     centre height times (t / 2) / L), the jacking of the classic rear-engined cars;</li>
 * <li>twist beam: between trailing arms and a solid axle (L the track, roll centre 0.1-0.15 m, Wu et al. 2019);</li>
 * <li>solid axles: the roll centre at the Panhard rod's or Watt linkage's pivot height, or a leaf spring's seat; the
 *     wheels stay square to the ground.</li>
 * </ul>
 * Lateral tyre forces reach the body along the line from the contact patch to the roll centre, so they jack the
 * body up or down by F_y h_rc / (t / 2) (and the tyre's load with it); longitudinal forces at the contact patch (no
 * anti-dive or anti-squat geometry).
 *
 * <p>0.33: with a kinematic table (KinematicTable, from hardpoints) the wheel moves as its links let it instead: the hub
 * along its path, the camber and toe as the table gives them (on top of the static values), the lateral force jacking by
 * the contact patch's front-view slope there and the longitudinal one by its side-view slope (anti-dive, anti-squat).
 * The geometry above still says whether the axle is solid. Without one, everything is as before.
 */
public final class Suspension {
    /** Layouts. */
    public enum Type {
        DOUBLE_WISHBONE, MULTI_LINK, MACPHERSON, TRAILING_ARM, SEMI_TRAILING_ARM, SWING_AXLE, TWIST_BEAM, SOLID_PANHARD, SOLID_WATT, SOLID_LEAF;

        public boolean solid() { return this == SOLID_PANHARD || this == SOLID_WATT || this == SOLID_LEAF; }
    }

    /** The front-view result: roll centre height above the ground (m), camber gain (rad per m of bump, negative: top in) and whether the wheel stays square to the ground. */
    public record Geometry(double rollCentreHeightM, double camberGainPerM, boolean solid) {
        /**
         * The geometry of a layout for a track and wheel centre height (m), with its lever: the front-view swing arm
         * length (m; the half-shaft for a swing axle, the arm length for a semi-trailing arm, unused for trailing arms and
         * solid axles; 0 for the type's default), its instant centre's height or a solid axle's link height (m; negative for
         * the type's default), and the semi-trailing arm's sweep (rad).
         */
        public static Geometry of(Type type, double trackM, double wheelCentreM, double swingArmM, double linkHeightM, double sweepRad) {
            Objects.requireNonNull(type, "type");
            double half = trackM / 2;
            return switch (type) {
                case DOUBLE_WISHBONE, MULTI_LINK -> lever(half, swingArmM > 0 ? swingArmM : 2.0, linkHeightM >= 0 ? linkHeightM : 0.08);
                case MACPHERSON -> lever(half, swingArmM > 0 ? swingArmM : 1.5, linkHeightM >= 0 ? linkHeightM : 0.1);
                case TRAILING_ARM -> new Geometry(0, 0, false);
                case SEMI_TRAILING_ARM -> {
                    double sweep = sweepRad > 0 ? sweepRad : Math.toRadians(20), arm = swingArmM > 0 ? swingArmM : 0.45;
                    yield lever(half, arm / Math.sin(sweep), linkHeightM >= 0 ? linkHeightM : wheelCentreM);
                }
                case SWING_AXLE -> lever(half, swingArmM > 0 ? swingArmM : 0.7 * half, wheelCentreM);
                case TWIST_BEAM -> new Geometry(linkHeightM >= 0 ? linkHeightM : 0.12, -1 / trackM, false);
                case SOLID_PANHARD, SOLID_WATT -> new Geometry(linkHeightM >= 0 ? linkHeightM : wheelCentreM, 0, true);
                case SOLID_LEAF -> new Geometry(linkHeightM >= 0 ? linkHeightM : wheelCentreM + 0.05, 0, true);
            };
        }

        private static Geometry lever(double half, double length, double icHeight) {
            return new Geometry(icHeight * half / length, -1 / length, false);
        }
    }

    /**
     * What holds the wheel up. A coil (or torsion bar, leaf, steel leg): wheel rate (N/m) and the force at full droop
     * (N, the preload). An oleo-pneumatic strut (Milwitzky and Cook, NACA TR-1154): gas over the piston area A (m^2)
     * with the extended gas volume V0 (m^3) at the extended pressure p0 (Pa, absolute), so F = p0 A (V0 / (V0 - A c))^n
     * - p_atm A, n = 1.12 in strokes (their average effective exponent). 0.33: an air spring (a bellows over an
     * effective area A, its air V0 at droop at p0) the same with n = AIR_POLYTROPIC; a levelling valve (Levelling)
     * scales the air it holds, s p0 in place of p0 (s the scale the axle keeps per wheel).
     */
    public record Spring(double rateNPerM, double preloadN, double pistonAreaM2, double gasVolumeM3, double gasPressurePa, double polytropic) {
        public Spring {
            if (!(rateNPerM >= 0) || !(preloadN >= 0) || !(pistonAreaM2 >= 0) || !(gasVolumeM3 >= 0) || !(gasPressurePa >= 0)) {
                throw new IllegalArgumentException("Spring values must not be negative");
            }
            if (!(polytropic >= 1 && polytropic <= 1.67)) { throw new IllegalArgumentException("A polytropic exponent is 1 to 1.67"); }
        }
        /** A coil or a strut's gas spring (the strut's exponent). */
        public Spring(double rateNPerM, double preloadN, double pistonAreaM2, double gasVolumeM3, double gasPressurePa) {
            this(rateNPerM, preloadN, pistonAreaM2, gasVolumeM3, gasPressurePa, POLYTROPIC);
        }
        /** A linear spring. */
        public static Spring coil(double rateNPerM, double preloadN) { return new Spring(rateNPerM, preloadN, 0, 0, 0); }
        /** An oleo-pneumatic strut's gas spring. */
        public static Spring oleo(double pistonAreaM2, double gasVolumeM3, double extendedPressurePa) { return new Spring(0, 0, pistonAreaM2, gasVolumeM3, extendedPressurePa); }
        /** 0.33: an air spring: effective area (m^2), its air's volume at full droop (m^3) and pressure there (Pa, absolute). */
        public static Spring air(double areaM2, double droopVolumeM3, double droopPressurePa) {
            return new Spring(0, 0, areaM2, droopVolumeM3, droopPressurePa, AIR_POLYTROPIC);
        }

        public boolean gas() { return pistonAreaM2 > 0 && gasVolumeM3 > 0; }

        /** Force at compression c (m; 0 at full droop), N. */
        public double force(double c) { return force(c, 1); }

        /** 0.33: the force with the gas's mass scaled (a levelling valve's air; a coil ignores it), N. */
        public double force(double c, double scale) {
            if (gas()) {
                double v = Math.max(gasVolumeM3 - pistonAreaM2 * Math.max(c, 0), 0.02 * gasVolumeM3);
                return scale * gasPressurePa * pistonAreaM2 * Math.pow(gasVolumeM3 / v, polytropic) - 101325 * pistonAreaM2;
            }
            return preloadN + rateNPerM * Math.max(c, 0);
        }

        /** Rate dF/dc at compression c, N/m. */
        public double rate(double c) { return rate(c, 1); }

        /** 0.33: the rate with the gas's mass scaled, N/m. */
        public double rate(double c, double scale) {
            if (gas()) {
                double v = Math.max(gasVolumeM3 - pistonAreaM2 * Math.max(c, 0), 0.02 * gasVolumeM3);
                return polytropic * scale * gasPressurePa * pistonAreaM2 * pistonAreaM2 / v * Math.pow(gasVolumeM3 / v, polytropic);
            }
            return rateNPerM;
        }
    }

    /** Milwitzky and Cook's average effective polytropic exponent of a strut's gas in landing strokes. */
    public static final double POLYTROPIC = 1.12;
    /**
     * 0.33: an air spring's exponent over a ride's strokes (rule: between isothermal 1 and adiabatic 1.4; makers' rate
     * sheets use about 1.3 to 1.38 for dynamic rates).
     */
    public static final double AIR_POLYTROPIC = 1.3;

    /**
     * 0.33: a levelling valve on an air spring: the ride height the driver picks ("ride_height" 0 low, 0.5 the design
     * height, 1 high) sets a target compression, lowM more than the design one at 0 and highM less at 1; the valve fills
     * the spring (its air's scale up to fillPerS a second) while the wheel is more compressed than the target by more
     * than the deadband and vents it (up to ventPerS) while less, opening in proportion to how far past the deadband the
     * wheel is (fully at four deadbands past it: a levelling valve's lever on its orifice; a valve open fully up to the
     * deadband overshoots the level and hunts round it): the body rises and falls to the level and holds it under any load.
     */
    public record Levelling(double lowM, double highM, double fillPerS, double ventPerS, double deadbandM) {
        public Levelling {
            if (!(lowM >= 0) || !(highM >= 0) || !(fillPerS > 0) || !(ventPerS > 0) || !(deadbandM > 0)) {
                throw new IllegalArgumentException("Levelling values: heights and deadband not negative, rates positive");
            }
        }

        /** The target compression for a design compression and the ride height input (0..1). */
        public double target(double designCompressionM, double input) {
            double r = Math.clamp(input, 0, 1);
            return designCompressionM + lowM * Math.max(0, 1 - 2 * r) - highM * Math.max(0, 2 * r - 1);
        }

        /** The air's scale after a step at compression c against a target (bounded 0.1 to 10). */
        public double next(double scale, double c, double target, double dt) {
            double s = scale, e = c - target;
            if (e > deadbandM) { s += fillPerS * dt * Math.min(1, (e - deadbandM) / (4 * deadbandM)); }
            else if (e < -deadbandM) { s -= ventPerS * dt * Math.min(1, (-e - deadbandM) / (4 * deadbandM)); }
            return Math.clamp(s, 0.1, 10);
        }
    }
    /** Bump stop: beyond the travel the stop adds this many times the spring's rate there (rule: a progressive rubber stop). */
    public static final double BUMP_STOP = 15;

    /**
     * Damping: linear coefficients in bump and rebound (N s/m at the wheel), and an orifice's quadratic coefficient
     * (N s^2/m^2, F = K c' |c'|, Milwitzky and Cook: K = rho A_h^3 / (2 (C_d A_n)^2), C_d 0.9), its rebound multiple.
     */
    public record Damper(double bumpNsPerM, double reboundNsPerM, double orificeNs2PerM2, double orificeRebound) {
        public Damper {
            if (!(bumpNsPerM >= 0) || !(reboundNsPerM >= 0) || !(orificeNs2PerM2 >= 0) || !(orificeRebound >= 0)) {
                throw new IllegalArgumentException("Damping must not be negative");
            }
        }
        public static Damper linear(double bump, double rebound) { return new Damper(bump, rebound, 0, 0); }
        /** An orifice of hydraulic area A_h, net orifice area A_n (m^2) in oil of density rho, discharge coefficient 0.9. */
        public static Damper orifice(double hydraulicAreaM2, double orificeAreaM2, double oilDensity, double reboundMultiple, double linearFloor) {
            double k = oilDensity * Math.pow(hydraulicAreaM2, 3) / (2 * Math.pow(0.9 * orificeAreaM2, 2));
            return new Damper(linearFloor, linearFloor, k, reboundMultiple);
        }
        /**
         * The damping coefficient (N s/m) at a compression rate (m/s; positive in bump): the force over the rate there (the
         * orifice's K |c'| is its secant), so coefficient(r) r is the force at r.
         */
        public double coefficient(double rate) {
            double linear = rate >= 0 ? bumpNsPerM : reboundNsPerM;
            double orifice = orificeNs2PerM2 * (rate >= 0 ? 1 : orificeRebound);
            return linear + orifice * Math.abs(rate);
        }
    }

    private final Geometry geometry;
    private final Spring spring;
    private final Damper damper;
    private final double travelM, staticCamberRad, designCompressionM;
    private final KinematicTable kinematics;
    private final Levelling levelling;

    /**
     * A suspension: geometry, spring, damper, travel (m, droop to bump), static camber (rad, positive top out) at the
     * design compression (m from droop, where the camber gain is counted from).
     */
    public Suspension(Geometry geometry, Spring spring, Damper damper, double travelM, double staticCamberRad, double designCompressionM) {
        this(geometry, spring, damper, travelM, staticCamberRad, designCompressionM, null);
    }

    /** 0.33: as above, moving as a kinematic table says (null: as before). */
    public Suspension(Geometry geometry, Spring spring, Damper damper, double travelM, double staticCamberRad, double designCompressionM, KinematicTable kinematics) {
        this(geometry, spring, damper, travelM, staticCamberRad, designCompressionM, kinematics, null);
    }

    /** 0.33: as above, an air spring with a levelling valve (null: none; a valve needs a gas spring). */
    public Suspension(Geometry geometry, Spring spring, Damper damper, double travelM, double staticCamberRad, double designCompressionM, KinematicTable kinematics,
                      Levelling levelling) {
        this.geometry = Objects.requireNonNull(geometry, "geometry"); this.spring = Objects.requireNonNull(spring, "spring");
        this.damper = Objects.requireNonNull(damper, "damper");
        if (!(travelM > 0) || !Double.isFinite(travelM)) { throw new IllegalArgumentException("Travel must be positive"); }
        if (levelling != null && !spring.gas()) { throw new IllegalArgumentException("A levelling valve needs an air spring"); }
        this.travelM = travelM; this.staticCamberRad = staticCamberRad; this.designCompressionM = designCompressionM;
        this.kinematics = kinematics; this.levelling = levelling;
    }

    public Geometry geometry() { return geometry; }
    public Spring spring() { return spring; }
    public Damper damper() { return damper; }
    public double travelM() { return travelM; }
    public double staticCamberRad() { return staticCamberRad; }
    public double designCompressionM() { return designCompressionM; }
    /** 0.33: the kinematic table the wheel moves by (null: straight up, the geometry's camber gain). */
    public KinematicTable kinematics() { return kinematics; }
    /** 0.33: the air spring's levelling valve, or null. */
    public Levelling levelling() { return levelling; }

    /** Spring force with the stops at compression c, N. */
    public double force(double c) { return force(c, 1); }

    /** 0.33: the force with an air spring's air scaled (Levelling), N. */
    public double force(double c, double scale) {
        double f = spring.force(Math.min(c, travelM), scale);
        return c > travelM ? f + BUMP_STOP * Math.max(spring.rate(travelM, scale), 1e3) * (c - travelM) : f;
    }

    /** Spring rate with the stops at compression c, N/m. */
    public double rate(double c) { return rate(c, 1); }

    /** 0.33: the rate with an air spring's air scaled, N/m. */
    public double rate(double c, double scale) {
        return c > travelM ? spring.rate(travelM, scale) + BUMP_STOP * Math.max(spring.rate(travelM, scale), 1e3) : spring.rate(c, scale);
    }

    /** Camber against the body at compression c, rad (0.33: the table's change on the static camber when there is one). */
    public double camberToBody(double c) {
        if (kinematics != null) { return staticCamberRad + kinematics.at(c).camberRad(); }
        return staticCamberRad + geometry.camberGainPerM() * (c - designCompressionM);
    }

    /**
     * One step of tyre and suspension in series (backward Euler): the force on the body is force + slope x dP/dt at
     * the step's end (slope in N s/m: dP/dt the end rate at which the ground closes on the hub), and the suspension's
     * compression at the end follows compression(rate).
     */
    public record Step(double forceN, double slopeNsPerM, double compression, double tyreDeflection, double tyreForceN, double suspensionRate,
                       double damping, double tyreRate, double springForce, double extra, boolean toppedOut) {
        /** The suspension's compression at the end for an end rate dP/dt (m/s), between droop (0) and as far as the stops allow. */
        public double compression(double rate, double dt) {
            if (toppedOut) { return 0; }
            double d = tyreRate + suspensionRate + damping / dt;
            double next = compression + (tyreForceN - springForce - extra + tyreRate * dt * rate) / d;
            return Math.max(next, 0);
        }
    }

    /**
     * The step for tyre penetration P (m: how far the ground pushes tyre and suspension together up from full droop),
     * its present rate (m/s), the compression at the start, an extra force on the suspension (an anti-roll bar's, N,
     * positive pushing the wheel down) and the tyre; the suspension's own rate is taken as P's to start with.
     */
    public Step step(Tyre tyre, double penetrationM, double rate, double compressionM, double extraN, double dt) {
        return step(tyre, penetrationM, rate, compressionM, rate, extraN, dt);
    }

    /**
     * The step as above, with the suspension's own compression rate at the start (m/s): the damping coefficient is
     * taken at the rate the suspension moves at over the step, found by a few fixed-point passes from it (an orifice's
     * K |c'| depends on the rate it is solving for; 0.24.0's first release took it once at P's rate, which left an oleo
     * a fifth too soft in a landing).
     */
    public Step step(Tyre tyre, double penetrationM, double rate, double compressionM, double ownRate, double extraN, double dt) {
        return step(tyre, penetrationM, rate, compressionM, ownRate, extraN, dt, 1);
    }

    /** 0.33: the step with an air spring's air scaled (Levelling). */
    public Step step(Tyre tyre, double penetrationM, double rate, double compressionM, double ownRate, double extraN, double dt, double scale) {
        double c = Math.max(compressionM, 0), delta = penetrationM - c;
        double kt = delta > 0 ? tyre.verticalStiffness(delta) : 0, ft = delta > 0 ? tyre.verticalForceN(delta) : 0;
        double ks = rate(c, scale), fs = force(c, scale);
        // the rate the suspension itself moves at, for bump or rebound damping and the orifice's coefficient
        double guess = ownRate;
        for (int i = 0; i < 4; i++) { guess = (ft - fs - extraN + kt * dt * rate) / (kt + ks + damper.coefficient(guess) / dt + 1e-9) / dt; }
        double cd = damper.coefficient(guess);
        double d = kt + ks + cd / dt;
        double next = c + (ft - fs - extraN + kt * dt * rate) / d;
        if (next < 0) {
            // topped out: the tyre carries it alone, the suspension held at droop
            return new Step(ft + kt * c, kt * dt, c, delta, ft, 0, 0, kt, fs, extraN, true);
        }
        double a = (ks + cd / dt) / d;
        return new Step(fs + extraN + a * (ft - fs - extraN), a * kt * dt, c, delta, ft, ks, cd, kt, fs, extraN, false);
    }
}
