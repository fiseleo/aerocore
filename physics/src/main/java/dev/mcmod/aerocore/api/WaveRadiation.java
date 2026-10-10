package dev.mcmod.aerocore.api;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Linear heave wave radiation of surface-piercing floats in deep, open water, in the time domain (Cummins 1962):
 * F_i(t) = -sum_j int_0^T K_ij(tau) w_j(t - tau) d tau, with w_j the vertical velocity of float j and
 * K_ij(t) = (2/pi) int_0^inf B_ij(omega) cos(omega t) d omega, so that B(omega) = int_0^inf K(t) cos(omega t) dt.
 * The constant infinite-frequency added mass is applied separately (FloatingBodyDynamics, 0.5 rho V_wet, equal to
 * Hulme's high-frequency limit for a hemisphere); the memory term then reproduces the frequency-dependent added
 * mass through the Kramers-Kronig relation A(omega) = A_inf - (1/omega) int K(t) sin(omega t) dt.
 *
 * Each float radiates like a floating hemisphere of its waterplane radius (Hulme 1982): exact for a hemisphere and in
 * the long-wave limit, approximate for other drafts. Floats interact by the point-absorber cross damping
 * B_ij = sqrt(B_i B_j) J0(k d_ij) (Budal 1977; Evans 1980), whose matrix is positive semi-definite, so radiation only
 * removes energy. Not modeled: surge/sway/yaw radiation of single floats, near-field (non-wave) interaction, finite
 * depth, wall reflection, submerged floats (no waterplane) and wave-current effects.
 */
public final class WaveRadiation {
    /**
     * Floating hemisphere in heave, Hulme (1982), J. Fluid Mech. 121, 443-463, as tabulated in the LHEEA reproduction:
     * ka, added mass A/(rho V), damping B/(rho V omega), V = 2/3 pi a^3, k = omega^2/g.
     */
    private static final double[][] HULME = {
            {0.0, 0.8310, 0.0000}, {0.05, 0.8764, 0.1036}, {0.1, 0.8627, 0.1816}, {0.2, 0.7938, 0.2793}, {0.3, 0.7157, 0.3254},
            {0.4, 0.6452, 0.3410}, {0.5, 0.5861, 0.3391}, {0.6, 0.5381, 0.3271}, {0.7, 0.4999, 0.3098}, {0.8, 0.4698, 0.2899},
            {0.9, 0.4464, 0.2691}, {1.0, 0.4284, 0.2484}, {1.2, 0.4047, 0.2096}, {1.4, 0.3924, 0.1756}, {1.6, 0.3871, 0.1469},
            {1.8, 0.3864, 0.1229}, {2.0, 0.3884, 0.1031}, {2.5, 0.3988, 0.0674}, {3.0, 0.4111, 0.0452}, {4.0, 0.4322, 0.0219},
            {5.0, 0.4471, 0.0116}, {6.0, 0.4574, 0.0066}, {7.0, 0.4647, 0.0040}, {8.0, 0.4700, 0.0026}, {9.0, 0.4740, 0.0017},
            {10.0, 0.4771, 0.0012}};
    /**
     * Heave exciting force of a fixed floating hemisphere in deep water, X^ = X3 / (rho g pi a^2 A), complex with
     * time factor e^{-i omega t} relative to the incident elevation at the axis, and its exact Froude-Krylov part:
     * rows {ka, Re X^, Im X^, FK}. Computed for this project by the Havelock (1955) / Hulme (1982) multipole method
     * (radiation and diffraction problems); the same computation reproduces Hulme's added mass and damping to four
     * digits, |X^| agrees with Haskind's relation B = omega^3 |X|^2 / (2 rho g^3) from Hulme's damping, and
     * multipole truncation changes |X^| by < 2e-5 and the phase by < 0.02 deg. No published phase was found to
     * compare against. The force leads the elevation (Im X^ < 0) at every ka.
     */
    private static final double[][] EXCITATION = {
            {0.0, 1.0, 0.0, 1.0}, {0.01, 0.98766, -0.00015, 0.99335}, {0.02, 0.97522, -0.00060, 0.98672},
            {0.05, 0.93797, -0.00346, 0.96698}, {0.1, 0.87787, -0.01213, 0.93458}, {0.2, 0.76895, -0.03752, 0.87166},
            {0.3, 0.67522, -0.06610, 0.81124}, {0.4, 0.59423, -0.09333, 0.75329}, {0.5, 0.52347, -0.11738, 0.69781},
            {0.6, 0.46092, -0.13764, 0.64478}, {0.7, 0.40509, -0.15407, 0.59417}, {0.8, 0.35488, -0.16688, 0.54596},
            {0.9, 0.30948, -0.17639, 0.50012}, {1.0, 0.26826, -0.18292, 0.45662}, {1.1, 0.23075, -0.18680, 0.41541},
            {1.2, 0.19658, -0.18836, 0.37646}, {1.3, 0.16543, -0.18787, 0.33973}, {1.4, 0.13707, -0.18562, 0.30516},
            {1.5, 0.11127, -0.18183, 0.27270}, {1.6, 0.08786, -0.17675, 0.24231}, {1.7, 0.06668, -0.17055, 0.21393},
            {1.8, 0.04758, -0.16344, 0.18749}, {1.9, 0.03044, -0.15558, 0.16295}, {2.0, 0.01514, -0.14711, 0.14023},
            {2.1, 0.00156, -0.13818, 0.11928}, {2.2, -0.01040, -0.12891, 0.10002}, {2.3, -0.02084, -0.11941, 0.08240},
            {2.4, -0.02985, -0.10979, 0.06634}, {2.5, -0.03753, -0.10014, 0.05178}, {2.6, -0.04397, -0.09054, 0.03865},
            {2.7, -0.04924, -0.08106, 0.02687}, {2.8, -0.05344, -0.07177, 0.01638}, {2.9, -0.05665, -0.06273, 0.00711},
            {3.0, -0.05894, -0.05398, -0.00100}};
    /** Beyond this ka the diffraction part fades linearly to zero by MAX_EXCITATION_KA + 1 (numerical blending only). */
    public static final double MAX_EXCITATION_KA = 3;
    /** Hulme's infinite-frequency heave added mass of a hemisphere, A/(rho V). */
    public static final double HEMISPHERE_ADDED_MASS_INFINITY = 0.5;
    /** Retardation memory length; kernels of float-sized bodies and their interactions decay well within it. */
    public static final double MEMORY_SECONDS = 8;
    private static final double G = Atmosphere.GRAVITY_MPS2, MAX_KA = 10;
    private static final int FREQUENCIES = 800;
    private WaveRadiation() {}

    /** Hulme heave added mass A/(rho V) at ka, linear interpolation; beyond ka = 10 it tends to 0.5 as 1/ka. */
    public static double hemisphereAddedMass(double ka) {
        Checks.nonnegative(ka, "ka");
        if (ka > MAX_KA) { return HEMISPHERE_ADDED_MASS_INFINITY - (HEMISPHERE_ADDED_MASS_INFINITY - HULME[HULME.length - 1][1]) * MAX_KA / ka; }
        return interpolate(ka, 1);
    }
    /** Hulme heave damping B/(rho V omega) at ka; beyond ka = 10 a (10/ka)^4 tail, already below 0.1 % of the peak. */
    public static double hemisphereDamping(double ka) {
        Checks.nonnegative(ka, "ka");
        if (ka > MAX_KA) { return HULME[HULME.length - 1][2] * Math.pow(MAX_KA / ka, 4); }
        return interpolate(ka, 2);
    }
    /** Hemisphere heave exciting force {Re X^, Im X^} at ka in [0, 3], linear interpolation (see EXCITATION). */
    public static double[] hemisphereExcitation(double ka) {
        Checks.nonnegative(ka, "ka");
        if (ka > MAX_EXCITATION_KA) { throw new IllegalArgumentException("Exciting-force table ends at ka = 3"); }
        return new double[]{interpolateExcitation(ka, 1), interpolateExcitation(ka, 2)};
    }
    /** Exact Froude-Krylov part of the hemisphere heave force, X_FK / (rho g pi a^2 A), real, ka in [0, 3]. */
    public static double hemisphereFroudeKrylov(double ka) {
        Checks.nonnegative(ka, "ka");
        if (ka > MAX_EXCITATION_KA) { throw new IllegalArgumentException("Exciting-force table ends at ka = 3"); }
        return interpolateExcitation(ka, 3);
    }
    /**
     * Diffraction part of the heave exciting force, X^ - FK, {Re, Im}. It is what a pressure integral over the
     * undisturbed incident wave misses. Faded linearly to zero between ka = 3 and 4 (blending, not data).
     */
    public static double[] hemisphereDiffraction(double ka) {
        Checks.nonnegative(ka, "ka");
        double fade = 1, k = ka;
        if (ka > MAX_EXCITATION_KA) { fade = Math.max(0, MAX_EXCITATION_KA + 1 - ka); k = MAX_EXCITATION_KA; }
        return new double[]{fade * (interpolateExcitation(k, 1) - interpolateExcitation(k, 3)), fade * interpolateExcitation(k, 2)};
    }
    /**
     * Upward heave diffraction force (N) on a surface-piercing float of waterplane radius a whose axis is at the
     * horizontal position (x, z): sum over components of rho g pi a^2 A Re{(X^ - FK)(k a) e^{i theta}}, with theta
     * the incident phase at the axis. The deep-water hemisphere table is used with each component's actual
     * wavenumber, so finite-depth components and other drafts are approximate. Add it to the Froude-Krylov
     * pressure integral (and drop the vertical Morison term) to obtain the hemisphere's exact linear excitation.
     */
    public static double heaveDiffraction(WaveField waves, double x, double z, double t, double waterplaneRadiusM, double densityKgPerM3) {
        Checks.nonnegative(waterplaneRadiusM, "Waterplane radius"); Checks.nonnegative(densityKgPerM3, "Density");
        if (waterplaneRadiusM == 0 || waves.isEmpty()) { return 0; }
        double sum = 0;
        for (LinearWave wave : waves.components()) {
            double[] d = hemisphereDiffraction(wave.wavenumber() * waterplaneRadiusM);
            double theta = wave.phaseAt(x, z, t);
            sum += wave.amplitudeM() * (d[0] * Math.cos(theta) - d[1] * Math.sin(theta));
        }
        return densityKgPerM3 * G * Math.PI * waterplaneRadiusM * waterplaneRadiusM * sum;
    }
    /** Deep-water heave radiation damping (kg/s) of a float with the given waterplane radius, per unit density. */
    public static double heaveDampingPerDensity(double waterplaneRadiusM, double omega) {
        Checks.nonnegative(waterplaneRadiusM, "Waterplane radius"); Checks.nonnegative(omega, "Angular frequency");
        if (waterplaneRadiusM == 0 || omega == 0) { return 0; }
        double a = waterplaneRadiusM;
        return 2.0 / 3 * Math.PI * a * a * a * omega * hemisphereDamping(omega * omega * a / G);
    }
    /** Waterplane radius of a sphere of radius R at geometric wet fraction f; zero when dry or fully submerged. */
    public static double waterplaneRadius(double sphereRadiusM, double wetFraction) {
        Checks.positive(sphereRadiusM, "Radius");
        if (!Double.isFinite(wetFraction) || wetFraction < 0 || wetFraction > 1) { throw new IllegalArgumentException("Wet fraction must be in [0,1]"); }
        if (wetFraction == 0 || wetFraction == 1) { return 0; }
        // Invert the cap fraction f = h^2 (3R - h) / (4 R^3), monotonic in the cap height h in [0, 2R].
        double low = 0, high = 2 * sphereRadiusM, r3 = 4 * Math.pow(sphereRadiusM, 3);
        for (int i = 0; i < 60; i++) {
            double h = 0.5 * (low + high);
            if (h * h * (3 * sphereRadiusM - h) / r3 < wetFraction) { low = h; } else { high = h; }
        }
        double h = 0.5 * (low + high);
        return Math.sqrt(Math.max(0, h * (2 * sphereRadiusM - h)));
    }
    /**
     * Retardation kernel per unit density, K_ij(n dt) for n = 0..count-1 (N s/m^2 per kg/m^3), between floats of
     * waterplane radii ai and aj at horizontal distance d: (2/pi) int sqrt(B_i B_j) J0(k d) cos(omega t) d omega.
     */
    public static double[] kernel(double ai, double aj, double distanceM, double dt, int count) {
        Checks.nonnegative(ai, "Radius"); Checks.nonnegative(aj, "Radius"); Checks.nonnegative(distanceM, "Distance");
        Checks.positive(dt, "Time step");
        double[] k = new double[count];
        if (ai == 0 || aj == 0) { return k; }
        double maxOmega = Math.sqrt(MAX_KA * G / Math.min(ai, aj)), step = maxOmega / FREQUENCIES;
        for (int m = 0; m <= FREQUENCIES; m++) {
            double omega = m * step;
            double b = Math.sqrt(heaveDampingPerDensity(ai, omega) * heaveDampingPerDensity(aj, omega)) * besselJ0(omega * omega / G * distanceM);
            double weight = (m == 0 || m == FREQUENCIES ? 0.5 : 1) * step * 2 / Math.PI * b;
            if (weight == 0) { continue; }
            // cos(omega n dt) by the stable Chebyshev recurrence.
            double c1 = Math.cos(omega * dt), previous = 1, current = c1;
            k[0] += weight;
            if (count > 1) { k[1] += weight * current; }
            for (int n = 2; n < count; n++) {
                double next = 2 * c1 * current - previous;
                k[n] += weight * next;
                previous = current; current = next;
            }
        }
        return k;
    }
    /** Bessel J0 by Abramowitz and Stegun 9.4.1 / 9.4.3 polynomial approximations (|error| < 1e-7). */
    static double besselJ0(double x) {
        double ax = Math.abs(x);
        if (ax <= 3) {
            double y = (x / 3) * (x / 3);
            return 1 + y * (-2.2499997 + y * (1.2656208 + y * (-0.3163866 + y * (0.0444479 + y * (-0.0039444 + y * 0.0002100)))));
        }
        double y = 3 / ax;
        double f = 0.79788456 + y * (-0.00000077 + y * (-0.00552740 + y * (-0.00009512 + y * (0.00137237 + y * (-0.00072805 + y * 0.00014476)))));
        double theta = ax - 0.78539816 + y * (-0.04166397 + y * (-0.00003954 + y * (0.00262573 + y * (-0.00054125 + y * (-0.00029333 + y * 0.00013558)))));
        return f * Math.cos(theta) / Math.sqrt(ax);
    }
    private static double interpolateExcitation(double ka, int column) {
        for (int i = 1; i < EXCITATION.length; i++) {
            if (ka <= EXCITATION[i][0]) {
                double fraction = (ka - EXCITATION[i - 1][0]) / (EXCITATION[i][0] - EXCITATION[i - 1][0]);
                return EXCITATION[i - 1][column] + fraction * (EXCITATION[i][column] - EXCITATION[i - 1][column]);
            }
        }
        return EXCITATION[EXCITATION.length - 1][column];
    }
    private static double interpolate(double ka, int column) {
        for (int i = 1; i < HULME.length; i++) {
            if (ka <= HULME[i][0]) {
                double fraction = (ka - HULME[i - 1][0]) / (HULME[i][0] - HULME[i - 1][0]);
                return HULME[i - 1][column] + fraction * (HULME[i][column] - HULME[i - 1][column]);
            }
        }
        return HULME[HULME.length - 1][column];
    }

    /** Kernel arguments, radii in order (the kernel is symmetric in them). */
    private record KernelKey(double a, double b, double distanceM, double dt, int count) {}
    /**
     * Kernels already computed, shared by every Memory (each costs FREQUENCIES x count operations, about 1.3 million at
     * a 5 ms step: recomputing them whenever a boat's waterline changed stalled frames). Read-only once made.
     */
    private static final Map<KernelKey, double[]> KERNELS = Collections.synchronizedMap(new LinkedHashMap<>(64, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<KernelKey, double[]> eldest) { return size() > 512; }
    });

    /** kernel, from the shared cache (the array must not be modified). */
    static double[] sharedKernel(double ai, double aj, double distanceM, double dt, int count) {
        KernelKey key = new KernelKey(Math.min(ai, aj), Math.max(ai, aj), distanceM, dt, count);
        double[] k = KERNELS.get(key);
        if (k == null) {
            k = kernel(key.a(), key.b(), distanceM, dt, count);
            KERNELS.put(key, k);
        }
        return k;
    }

    /** Below this radius (m) the snapping grid is linear, SNAP_STEP apart; above, SNAP_RATIO apart. */
    private static final double SNAP_LINEAR = 0.25, SNAP_STEP = 0.005, SNAP_RATIO = 1.02;

    /**
     * A waterplane radius on the grid kernels are made for when radii drift (record): within 1 % (2.5 mm below
     * 0.25 m, so a float wetted less than that radiates nothing), well inside the 3 % / 1 cm drift a kernel already
     * stands for, so drifting bodies reuse cached kernels.
     */
    static double snapRadius(double radius) {
        if (!(radius > 0)) { return 0; }
        if (radius < SNAP_LINEAR) { return Math.round(radius / SNAP_STEP) * SNAP_STEP; }
        return SNAP_LINEAR * Math.pow(SNAP_RATIO, Math.round(Math.log(radius / SNAP_LINEAR) / Math.log(SNAP_RATIO)));
    }

    /**
     * Velocity history and kernels for one body's floats. Horizontal float positions are model-local and fixed (small
     * tilt). Waterplane radii follow an exponential average (0.5 s) of the sampled immersion; kernels are remade (for
     * the radius snapped to a 2 % grid, from a shared cache) when a radius drifts by more than 1 cm or 3 %. Mutable and
     * not thread-safe; one instance per body.
     */
    public static final class Memory {
        private static final double RADIUS_TIME_CONSTANT = 0.5, RECOMPUTE_TOLERANCE = 0.01, RECOMPUTE_FRACTION = 0.03;
        private final double dt;
        private final int count, floats;
        private final double[][] distance, history;
        private final double[] radii, reference, instant;
        private double[][][] kernels;
        private int head;

        public Memory(List<Vec3d> floatPositionsLocal, double dt) {
            Checks.positive(dt, "Time step");
            this.dt = dt;
            floats = floatPositionsLocal.size();
            count = (int) Math.ceil(MEMORY_SECONDS / dt) + 1;
            distance = new double[floats][floats];
            for (int i = 0; i < floats; i++) for (int j = 0; j < floats; j++) {
                Vec3d d = floatPositionsLocal.get(i).subtract(floatPositionsLocal.get(j));
                distance[i][j] = Math.hypot(d.x(), d.z());
            }
            history = new double[floats][count];
            radii = new double[floats];
            reference = new double[floats];
            instant = new double[floats];
            kernels = new double[floats][floats][];
            rebuild(true);
        }
        public double timeStep() { return dt; }
        public double[] effectiveRadii() { return radii.clone(); }

        /**
         * Upward radiation force on each float (N) for the given water density, from the recorded vertical velocities;
         * trapezoid rule over the memory, with the most recent sample taken as the current velocity.
         */
        public double[] verticalForces(double densityKgPerM3) {
            Checks.nonnegative(densityKgPerM3, "Density");
            double[] forces = new double[floats];
            for (int i = 0; i < floats; i++) {
                double sum = 0;
                for (int j = 0; j < floats; j++) {
                    double[] k = kernels[i][j], w = history[j];
                    double partial = 0;
                    for (int n = 0; n < count; n++) {
                        double weight = n == 0 || n == count - 1 ? 0.5 : 1;
                        partial += weight * k[n] * w[(head - n + count) % count];
                    }
                    sum += partial;
                }
                forces[i] = -densityKgPerM3 * dt * sum;
            }
            return forces;
        }

        /** Records the floats' current vertical velocities (m/s, up positive) and waterplane radii (m). */
        public void record(double[] verticalVelocities, double[] waterplaneRadii) {
            if (verticalVelocities.length != floats || waterplaneRadii.length != floats) { throw new IllegalArgumentException("One value per float is required"); }
            head = (head + 1) % count;
            double blend = -Math.expm1(-dt / RADIUS_TIME_CONSTANT);
            boolean stale = false;
            for (int i = 0; i < floats; i++) {
                if (!Double.isFinite(verticalVelocities[i])) { throw new IllegalArgumentException("Velocity must be finite"); }
                Checks.nonnegative(waterplaneRadii[i], "Waterplane radius");
                history[i][head] = verticalVelocities[i];
                instant[i] = waterplaneRadii[i];
                radii[i] += (waterplaneRadii[i] - radii[i]) * blend;
                stale |= Math.abs(radii[i] - reference[i]) > Math.max(RECOMPUTE_TOLERANCE, RECOMPUTE_FRACTION * reference[i]);
            }
            if (stale) { rebuild(true); }
        }

        /** Radiation loads at model-local points (for example HullDynamics.radiationPoints), as external loads. */
        public List<AppliedForce> loadsAt(RigidBodyState s, RigidBodyProperties p, List<Vec3d> localPoints, Vec3d up, double densityKgPerM3) {
            if (localPoints.size() != floats) { throw new IllegalArgumentException("One point per float is required"); }
            double[] forces = verticalForces(densityKgPerM3);
            List<AppliedForce> loads = new java.util.ArrayList<>();
            for (int i = 0; i < floats; i++) {
                if (forces[i] != 0) { loads.add(new AppliedForce(AppliedForce.Kind.RADIATION, s.worldOffset(p, localPoints.get(i)), up.scale(forces[i]))); }
            }
            return loads;
        }
        /** Records vertical velocities at model-local points and the given waterplane radii (m). */
        public void recordAt(RigidBodyState s, RigidBodyProperties p, List<Vec3d> localPoints, Vec3d up, double[] waterplaneRadii) {
            double[] w = new double[floats];
            Vec3d omega = s.angularVelocity(p);
            for (int i = 0; i < floats; i++) { w[i] = s.velocity().add(omega.cross(s.worldOffset(p, localPoints.get(i)))).dot(up); }
            record(w, waterplaneRadii);
        }
        /** Radiation loads at the float centers, to pass as FloatingBodyDynamics external loads. up is a unit vector. */
        public List<AppliedForce> loads(RigidBodyState s, RigidBodyProperties p, List<BuoyantSphere> spheres, Vec3d up, double densityKgPerM3) {
            if (spheres.size() != floats) { throw new IllegalArgumentException("One sphere per float is required"); }
            double[] forces = verticalForces(densityKgPerM3);
            List<AppliedForce> loads = new java.util.ArrayList<>();
            for (int i = 0; i < floats; i++) {
                if (forces[i] == 0) { continue; }
                loads.add(new AppliedForce(AppliedForce.Kind.RADIATION, s.worldOffset(p, spheres.get(i).offset()), up.scale(forces[i])));
            }
            return loads;
        }
        /**
         * Exact hemisphere heave diffraction at each float with a waterplane (heaveDiffraction), as INERTIA loads at
         * the float centers, using the waterplane radii last recorded (unsmoothed). Pair it with
         * FloatingBodyDynamics.Contact.surfaceDiffraction set for floats that pierce the surface, so the vertical
         * Morison term is not counted twice. Waves use world x and z; up must be +Y.
         */
        public List<AppliedForce> diffractionLoads(RigidBodyState s, RigidBodyProperties p, List<BuoyantSphere> spheres,
                                                   double densityKgPerM3, WaveField waves, double timeSeconds) {
            if (spheres.size() != floats) { throw new IllegalArgumentException("One sphere per float is required"); }
            List<AppliedForce> loads = new java.util.ArrayList<>();
            if (waves.isEmpty()) { return loads; }
            for (int i = 0; i < floats; i++) {
                if (instant[i] == 0) { continue; }
                Vec3d r = s.worldOffset(p, spheres.get(i).offset()), center = s.position().add(r);
                double force = heaveDiffraction(waves, center.x(), center.z(), timeSeconds, instant[i], densityKgPerM3);
                loads.add(new AppliedForce(AppliedForce.Kind.INERTIA, r, Vec3d.Y.scale(force)));
            }
            return loads;
        }
        /** Records the float centers' vertical velocities and waterplane radii from a step's end state and wet fractions. */
        public void record(RigidBodyState s, RigidBodyProperties p, List<BuoyantSphere> spheres, Vec3d up, List<Double> wetFractions) {
            double[] w = new double[floats], a = new double[floats];
            Vec3d omega = s.angularVelocity(p);
            for (int i = 0; i < floats; i++) {
                BuoyantSphere sphere = spheres.get(i);
                w[i] = s.velocity().add(omega.cross(s.worldOffset(p, sphere.offset()))).dot(up);
                a[i] = waterplaneRadius(sphere.radiusM(), wetFractions.get(i));
            }
            record(w, a);
        }

        /** Sets radii immediately (for example a float already in equilibrium) and rebuilds the kernels for exactly these radii. */
        public void setRadii(double[] waterplaneRadii) {
            System.arraycopy(waterplaneRadii, 0, radii, 0, floats);
            System.arraycopy(waterplaneRadii, 0, instant, 0, floats);
            rebuild(false);
        }

        private void rebuild(boolean snap) {
            for (int i = 0; i < floats; i++) { reference[i] = snap ? snapRadius(radii[i]) : radii[i]; }
            double[][][] next = new double[floats][floats][];
            for (int i = 0; i < floats; i++) {
                for (int j = 0; j <= i; j++) {
                    double[] k = null;
                    // Equal radii and distances share one kernel (the level platform has only three distinct ones).
                    for (int p = 0; p <= i && k == null; p++) {
                        for (int q = 0; q <= p && k == null; q++) {
                            if ((p < i || q < j) && next[p][q] != null && reference[p] == reference[i] && reference[q] == reference[j]
                                    && distance[p][q] == distance[i][j]) { k = next[p][q]; }
                        }
                    }
                    if (k == null) { k = sharedKernel(reference[i], reference[j], distance[i][j], dt, count); }
                    next[i][j] = k; next[j][i] = k;
                }
            }
            kernels = next;
        }
    }
}
