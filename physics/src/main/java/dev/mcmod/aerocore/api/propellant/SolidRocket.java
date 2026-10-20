package dev.mcmod.aerocore.api.propellant;

import dev.mcmod.aerocore.api.gasdynamics.PerfectGas;

import java.util.Objects;

/**
 * A solid rocket motor's internal ballistics and thrust, lumped-parameter and quasi-steady (Sutton and Biblarz, Rocket
 * Propulsion Elements, 9th ed., ch. 12-14; Nakka's SRM spreadsheet method): at every burned web y the chamber pressure
 * balances what the burning surface makes against what the throat passes,
 * <pre>
 *   rho_p r(p_c) A_b(y) = p_c A_t / (eta c*)
 * </pre>
 * (solved by bisection; one root because r grows slower than p for n &lt; 1), the web burns at r(p_c), and thrust is
 * F = p_c A_t (lambda C_F,momentum + epsilon p_e / p_c) - p_a A_e, the ideal isentropic thrust coefficient of the
 * products' gamma with a divergence factor lambda = (1 + cos alpha) / 2. A nozzle overexpanded below 0.4 of ambient
 * separates there (Summerfield) and exhausts at that point instead. Not modelled: ignition and chamber-filling
 * transients, erosive burning, sliver tail-off, nozzle erosion, two-phase lag beyond the effective gamma.
 */
public final class SolidRocket {
    private SolidRocket() {}

    /** Summerfield's separation criterion: an overexpanded jet separates where its wall pressure falls to this fraction of ambient. */
    public static final double SEPARATION = 0.4;

    public enum GrainKind {
        /** Cylindrical segments, each burning in its core and on both ends; the outside bonded to the case (Ballistic Test and Evaluation System). */
        BATES,
        /** One long tube burning only in its core (ends inhibited). */
        TUBE,
        /** A solid cylinder burning only on its aft face, like a cigarette: low, long, constant thrust. */
        END_BURNER
    }

    /** Grain geometry; core diameter 0 for an end burner; segments of a given length. */
    public record Grain(GrainKind kind, double outerDiameterM, double coreDiameterM, double segmentLengthM, int segments) {
        public Grain {
            Objects.requireNonNull(kind, "kind");
            if (!(outerDiameterM > 0) || !(segmentLengthM > 0) || segments < 1) { throw new IllegalArgumentException("A grain needs size and at least one segment"); }
            if (kind != GrainKind.END_BURNER && !(coreDiameterM > 0 && coreDiameterM < outerDiameterM)) { throw new IllegalArgumentException("The core must be inside the grain"); }
            if (kind == GrainKind.END_BURNER && segments != 1) { throw new IllegalArgumentException("An end burner is one piece"); }
        }

        /** Web thickness: what burns before the grain is gone, m. */
        public double web() {
            return switch (kind) {
                case BATES -> Math.min((outerDiameterM - coreDiameterM) / 2, segmentLengthM / 2);
                case TUBE -> (outerDiameterM - coreDiameterM) / 2;
                case END_BURNER -> segmentLengthM;
            };
        }

        /** Propellant volume after burning y of web, m^3. */
        public double volume(double y) {
            double r = outerDiameterM / 2;
            return switch (kind) {
                case BATES -> {
                    double rc = coreDiameterM / 2 + y, l = segmentLengthM - 2 * y;
                    yield rc >= r || l <= 0 ? 0 : segments * Math.PI * (r * r - rc * rc) * l;
                }
                case TUBE -> {
                    double rc = coreDiameterM / 2 + y;
                    yield rc >= r ? 0 : Math.PI * (r * r - rc * rc) * segmentLengthM * segments;
                }
                case END_BURNER -> Math.max(0, Math.PI * r * r * (segmentLengthM - y));
            };
        }

        /** Burning surface after burning y of web, m^2. */
        public double burningArea(double y) {
            double r = outerDiameterM / 2;
            return switch (kind) {
                case BATES -> {
                    double rc = coreDiameterM / 2 + y, l = segmentLengthM - 2 * y;
                    yield rc >= r || l <= 0 ? 0 : segments * (2 * Math.PI * rc * l + 2 * Math.PI * (r * r - rc * rc));
                }
                case TUBE -> {
                    double rc = coreDiameterM / 2 + y;
                    yield rc >= r ? 0 : 2 * Math.PI * rc * segmentLengthM * segments;
                }
                case END_BURNER -> y >= segmentLengthM ? 0 : Math.PI * r * r;
            };
        }

        /** Free port area at the aft end (the gas's way out of the core), m^2. */
        public double portArea(double y) {
            return kind == GrainKind.END_BURNER ? Math.PI * outerDiameterM * outerDiameterM / 4 : Math.PI * Math.pow(coreDiameterM / 2 + y, 2);
        }
    }

    /**
     * A motor: propellant, grain, nozzle throat diameter and expansion ratio (exit over throat area), the nozzle's
     * divergence half-angle (radians) and the characteristic-velocity efficiency (combustion; about 0.9-0.98).
     */
    public record Design(Propellant propellant, Grain grain, double throatDiameterM, double expansionRatio, double divergenceHalfAngle,
                         double characteristicVelocityEfficiency) {
        public Design {
            Objects.requireNonNull(propellant, "propellant");
            Objects.requireNonNull(grain, "grain");
            if (!(throatDiameterM > 0) || !(expansionRatio >= 1) || !(divergenceHalfAngle >= 0 && divergenceHalfAngle < Math.PI / 4)
                    || !(characteristicVelocityEfficiency > 0 && characteristicVelocityEfficiency <= 1)) {
                throw new IllegalArgumentException("Nozzle needs a throat, an expansion ratio of at least 1, a divergence under 45 deg and an efficiency in (0, 1]");
            }
        }
        public double throatArea() { return Math.PI * throatDiameterM * throatDiameterM / 4; }
        public double exitArea() { return throatArea() * expansionRatio; }
        /** Initial burning area over throat area (the Kn amateur motor builders design by). */
        public double kn() { return grain.burningArea(0) / throatArea(); }
    }

    /**
     * A motor's burn: time, chamber pressure, vacuum thrust and mass flow at each web step (to burnout), the exit area,
     * the propellant's mass, the burn time and vacuum total impulse.
     */
    public record Performance(Design design, double[] timeS, double[] chamberPressurePa, double[] massFlowKgPerS, double propellantMassKg,
                              double burnTimeS, double maxPressurePa) {

        private double interpolate(double[] y, double t) {
            if (!(t >= 0) || t >= burnTimeS) { return 0; }
            int lo = 0, hi = timeS.length - 1;
            while (hi - lo > 1) { int mid = (lo + hi) >>> 1; if (timeS[mid] <= t) { lo = mid; } else { hi = mid; } }
            double f = (t - timeS[lo]) / Math.max(1e-12, timeS[hi] - timeS[lo]);
            return y[lo] + (y[hi] - y[lo]) * Math.clamp(f, 0, 1);
        }

        /** Chamber pressure t seconds after ignition (0 after burnout), Pa. */
        public double pressureAt(double t) { return interpolate(chamberPressurePa, t); }
        /** Propellant burned per second t seconds after ignition, kg/s. */
        public double massFlowAt(double t) { return interpolate(massFlowKgPerS, t); }
        /** Thrust t seconds after ignition into ambient pressure pa, N (never negative). */
        public double thrustAt(double t, double ambientPa) { return thrust(design, pressureAt(t), ambientPa); }

        /** Total impulse into ambient pressure pa, N s (trapezoid over the web steps). */
        public double totalImpulse(double ambientPa) {
            double sum = 0;
            for (int i = 1; i < timeS.length; i++) {
                sum += 0.5 * (thrust(design, chamberPressurePa[i - 1], ambientPa) + thrust(design, chamberPressurePa[i], ambientPa)) * (timeS[i] - timeS[i - 1]);
            }
            return sum;
        }

        /** Delivered specific impulse into ambient pressure pa, s. */
        public double specificImpulse(double ambientPa) { return totalImpulse(ambientPa) / (propellantMassKg * 9.80665); }

        /** Average thrust over the burn, N. */
        public double averageThrust(double ambientPa) { return totalImpulse(ambientPa) / burnTimeS; }
    }

    /** Number of web steps of a burn. */
    public static final int STEPS = 400;

    /** Burns the motor from ignition to web burnout. */
    public static Performance perform(Design d) {
        Grain g = d.grain();
        Propellant p = d.propellant();
        double web = g.web(), dy = web / STEPS, at = d.throatArea(), cStar = d.characteristicVelocityEfficiency() * p.characteristicVelocity();
        double[] time = new double[STEPS + 1], pressure = new double[STEPS + 1], flow = new double[STEPS + 1];
        double t = 0, max = 0;
        for (int i = 0; i <= STEPS; i++) {
            double y = Math.min(i * dy, web * (1 - 1e-9));
            double ab = g.burningArea(y);
            double pc = chamberPressure(p, ab, at, cStar);
            time[i] = t;
            pressure[i] = pc;
            flow[i] = pc * at / cStar;
            max = Math.max(max, pc);
            if (i < STEPS) {
                // the next web step burns at the mean rate of its ends (trapezoid in 1 / r)
                double yNext = Math.min((i + 1) * dy, web * (1 - 1e-9));
                double pNext = chamberPressure(p, g.burningArea(yNext), at, cStar);
                t += dy * 0.5 * (1 / p.burnRate(pc) + 1 / p.burnRate(pNext));
            }
        }
        return new Performance(d, time, pressure, flow, p.densityKgPerM3() * g.volume(0), t, max);
    }

    /** Chamber pressure at which a burning area makes what a throat passes, Pa (the grain's mass balance; bisection in ln p). */
    public static double chamberPressure(Propellant p, double burningAreaM2, double throatAreaM2, double characteristicVelocity) {
        if (burningAreaM2 <= 0) { return 0; }
        double lo = Math.log(1e3), hi = Math.log(1e10);
        for (int i = 0; i < 200 && hi - lo > 1e-13; i++) {
            double mid = 0.5 * (lo + hi), pc = Math.exp(mid);
            double made = p.densityKgPerM3() * p.burnRate(pc) * burningAreaM2, passed = pc * throatAreaM2 / characteristicVelocity;
            if (made > passed) { lo = mid; } else { hi = mid; }
        }
        return Math.exp(0.5 * (lo + hi));
    }

    /** Thrust at chamber pressure pc into ambient pa, N (separated when overexpanded below 0.4 of ambient). */
    public static double thrust(Design d, double pc, double pa) {
        if (!(pc > 0)) { return 0; }
        double g = d.propellant().gamma(), eps = d.expansionRatio(), at = d.throatArea(), lambda = (1 + Math.cos(d.divergenceHalfAngle())) / 2;
        double exitMach = eps > 1 ? PerfectGas.machAtAreaRatio(eps, g, true) : 1;
        double pe = pc / PerfectGas.totalToStaticPressure(exitMach, g);
        if (pa > 0 && pe < SEPARATION * pa && pc > pa) {
            // the jet leaves the wall where it reaches 0.4 of ambient: the nozzle ends there
            double sepMach = PerfectGas.machAtPressureRatio(pc / (SEPARATION * pa), g);
            eps = Math.max(1, PerfectGas.areaRatio(sepMach, g));
            pe = SEPARATION * pa;
        }
        double ratio = pe / pc;
        double momentum = Math.sqrt(2 * g * g / (g - 1) * Math.pow(2 / (g + 1), (g + 1) / (g - 1)) * (1 - Math.pow(ratio, (g - 1) / g)));
        double f = pc * at * (lambda * momentum + eps * ratio) - pa * at * eps;
        return Math.max(0, f);
    }

    /** Ideal vacuum thrust coefficient of a nozzle (no divergence loss): the momentum term plus the exit pressure term. */
    public static double vacuumThrustCoefficient(double gamma, double expansionRatio) {
        double exitMach = expansionRatio > 1 ? PerfectGas.machAtAreaRatio(expansionRatio, gamma, true) : 1;
        double ratio = 1 / PerfectGas.totalToStaticPressure(exitMach, gamma), g = gamma;
        return Math.sqrt(2 * g * g / (g - 1) * Math.pow(2 / (g + 1), (g + 1) / (g - 1)) * (1 - Math.pow(ratio, (g - 1) / g))) + expansionRatio * ratio;
    }
}
