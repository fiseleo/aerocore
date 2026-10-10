package dev.mcmod.aerocore.api;

/**
 * How a water surface looks: the optics the surface renderer evaluates at every vertex (colours are linear RGB in 0..1).
 * <ul>
 * <li>Reflection: the exact Fresnel reflectance of unpolarised light at an air-water surface, n = 1.333 (2 % straight
 * down, rising to 100 % at grazing angles), of the sky colour. At a distance, where the view grazes the waves, this is
 * what makes them visible: each slope reflects a different part of the sky.</li>
 * <li>Sun glitter: Cox and Munk's (1954, J. Opt. Soc. Am. 44:838) wind-dependent Gaussian distribution of the facet
 * slopes too small to draw, variance s^2 = 0.003 + 5.12e-3 U (clean surface, U the wind at 12.5 m, taken as U10), as
 * the bidirectional reflectance R p(tilt) / (4 cos^4(tilt) cos(view) cos(sun)) (their eq. for the glitter
 * radiance): a calm sea shows a sharp image of the sun, a windy one a broad glitter path. Facets hidden from the eye
 * or the sun by the waves in front of them are removed with Smith's masking-shadowing term for Gaussian slopes
 * (smithLambda), which keeps the glitter finite toward the horizon.</li>
 * <li>Water body: light that enters returns from the water and the bed, attenuated as exp(-2 K d) over the depth d down
 * and back, K = 0.2 /m (diffuse attenuation of Jerlov's coastal water type 1 near 490 nm; open ocean types are
 * clearer, 0.02-0.1 /m), so shallow water shows its bed and deep water only its own colour.</li>
 * </ul>
 * Not modelled: refraction of the bed image, caustics, subsurface scattering, polarisation and the sky's own radiance
 * distribution (one sky colour, the game's).
 */
public final class WaterOptics {
    /** Refractive index of water (visible light, 20 C). */
    public static final double REFRACTIVE_INDEX = 1.333;
    /** Diffuse attenuation coefficient, 1/m (Jerlov coastal type 1). */
    public static final double ATTENUATION = 0.2;

    private WaterOptics() {}

    /** Fresnel reflectance of unpolarised light arriving at cos(incidence) from the air side. */
    public static double fresnel(double cosIncidence) {
        double ci = Math.clamp(cosIncidence, 0, 1), si2 = 1 - ci * ci, n = REFRACTIVE_INDEX;
        double ct = Math.sqrt(Math.max(0, 1 - si2 / (n * n)));
        double rs = (ci - n * ct) / (ci + n * ct), rp = (n * ci - ct) / (n * ci + ct);
        return 0.5 * (rs * rs + rp * rp);
    }

    /**
     * Fresnel reflectance seen from under the surface at cos(incidence) in the water: total internal reflection beyond
     * the critical angle asin(1 / n) = 48.6 deg, so only Snell's window shows the sky.
     */
    public static double fresnelFromWater(double cosIncidence) {
        double ci = Math.clamp(cosIncidence, 0, 1), n = REFRACTIVE_INDEX, st2 = n * n * (1 - ci * ci);
        if (st2 >= 1) { return 1; }
        double ct = Math.sqrt(1 - st2);
        double rs = (n * ci - ct) / (n * ci + ct), rp = (ci - n * ct) / (ci + n * ct);
        return 0.5 * (rs * rs + rp * rp);
    }

    /** Cox-Munk variance of the unresolved surface slopes (both directions together) at a wind speed (m/s). */
    public static double slopeVariance(double windMps) { return 0.003 + 5.12e-3 * Math.max(0, windMps); }

    /**
     * Sun glitter reflectance factor toward the viewer: Fresnel times the Cox-Munk probability density of the facet tilt
     * that mirrors the sun into the eye, over 4 cos^4(tilt) cos(view); (nx, ny, nz) the resolved surface
     * normal, (vx, vy, vz) toward the eye and (sx, sy, sz) toward the sun, all unit. Multiply by the sun's irradiance.
     */
    public static double glitter(double nx, double ny, double nz, double vx, double vy, double vz, double sx, double sy, double sz, double slopeVariance) {
        double cosView = nx * vx + ny * vy + nz * vz, cosSun = nx * sx + ny * sy + nz * sz;
        if (cosView <= 0 || cosSun <= 0) { return 0; }
        double hx = vx + sx, hy = vy + sy, hz = vz + sz, length = Math.sqrt(hx * hx + hy * hy + hz * hz);
        hx /= length; hy /= length; hz /= length;
        // Tilt of the mirroring facet from the resolved normal.
        double cosTilt = Math.clamp(hx * nx + hy * ny + hz * nz, 1e-6, 1), tan2 = (1 - cosTilt * cosTilt) / (cosTilt * cosTilt);
        double density = Math.exp(-tan2 / slopeVariance) / (Math.PI * slopeVariance);
        double cosHalf = hx * vx + hy * vy + hz * vz, shadowing = 1 / (1 + smithLambda(cosView, slopeVariance) + smithLambda(cosSun, slopeVariance));
        return shadowing * fresnel(cosHalf) * density / (4 * Math.pow(cosTilt, 4) * cosView);
    }

    /**
     * Smith's masking (or shadowing) term Lambda for Gaussian facet slopes of total variance v (both directions), seen
     * at cos(theta) from the mean normal: the rational fit of Walter, Marschner, Li and Torrance (2007, EGSR) to the
     * exact (exp(-a^2) / (a sqrt(pi)) - erfc(a)) / 2, with a = cot(theta) / sigma and sigma^2 = v / 2; 0 beyond a = 1.6.
     * The visible share of facets is 1 / (1 + Lambda).
     */
    public static double smithLambda(double cosTheta, double slopeVariance) {
        double c = Math.clamp(cosTheta, 1e-9, 1), a = c / (Math.sqrt(0.5 * slopeVariance) * Math.sqrt(1 - c * c));
        return a < 1.6 ? (1 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a) : 0;
    }

    /** Fraction of light reaching the bed and back through depth d (m): exp(-2 K d). */
    public static double transmittance(double depthM) { return Math.exp(-2 * ATTENUATION * Math.max(depthM, 0)); }

    /** Surface tension of clean water over its density at 20 C, m^3/s^2 (0.0728 N/m / 998 kg/m^3). */
    public static final double TENSION_OVER_DENSITY = 0.0728 / 998.2;
    /** Wavenumber of the slowest capillary-gravity wave, rad/m (wavelength 1.71 cm): sqrt(g rho / sigma). */
    public static final double CAPILLARY_WAVENUMBER = Math.sqrt(Atmosphere.GRAVITY_MPS2 / TENSION_OVER_DENSITY);

    /** Deep-water capillary-gravity dispersion: omega^2 = g k + (sigma / rho) k^3, rad/s. */
    public static double rippleFrequency(double k) { return Math.sqrt(Atmosphere.GRAVITY_MPS2 * k + TENSION_OVER_DENSITY * k * k * k); }

    /**
     * Cox-Munk upwind and crosswind slope variances {up, cross} at a wind speed (clean surface; fitted separately from the
     * total, they sum to slopeVariance within 1 %).
     */
    public static double[] slopeVariances(double windMps) {
        double u = Math.max(0, windMps);
        return new double[]{3.16e-3 * u, 0.003 + 1.92e-3 * u};
    }

    /**
     * The wind ripples shorter than a sea's spectrum, as plane waves for drawing the surface only (the physics does not
     * feel them): the slope variance Cox and Munk measured at the wind, less the variance resolvedVariance that the
     * spectrum's own waves carry, shared equally per log wavenumber among count waves from kStart up to the slowest
     * capillary-gravity wave (wavelength 1.71 cm). Equal slope variance per log wavenumber is the saturation range
     * (elevation spectrum ~ k^-4, Phillips 1985, J. Fluid Mech. 156:505); it is extended to the capillary limit, where
     * Cox and Munk's glitter says the slopes still are. Each wave travels at the capillary-gravity frequency
     * (rippleFrequency), its direction drawn about the wind from |cos|^n with n = up/cross - 1 so the upwind and
     * crosswind variances keep Cox and Munk's ratio (about 1.4 at 10 m/s), its phase random; all from the seed.
     * Returns rows {kx, kz, amplitude (m), omega (rad/s), phase (rad)}; no rows when the spectrum already carries the
     * measured variance or reaches the capillary limit.
     */
    public static double[][] ripples(double windMps, double windDirectionRadians, double resolvedVariance, double kStart, int count, long seed) {
        if (!Double.isFinite(windDirectionRadians)) { throw new IllegalArgumentException("Wind direction must be finite"); }
        double tail = slopeVariance(windMps) - Math.max(0, resolvedVariance), kEnd = CAPILLARY_WAVENUMBER;
        if (!(tail > 0) || !(kStart > 0) || kStart >= kEnd || count < 1) { return new double[0][]; }
        double[] v = slopeVariances(windMps);
        double n = Math.max(0, v[0] / v[1] - 1);
        // Inverse CDF of |cos|^n on [-pi/2, pi/2], tabulated.
        int table = 512;
        double[] cdf = new double[table + 1];
        for (int i = 1; i <= table; i++) {
            double a = -Math.PI / 2 + (i - 0.5) * Math.PI / table;
            cdf[i] = cdf[i - 1] + Math.pow(Math.abs(Math.cos(a)), n);
        }
        java.util.SplittableRandom random = new java.util.SplittableRandom(seed);
        double share = tail / count;
        double[][] out = new double[count][];
        for (int j = 0; j < count; j++) {
            double k = kStart * Math.pow(kEnd / kStart, (j + 0.5) / count);
            double target = random.nextDouble() * cdf[table];
            int i = 1;
            while (i < table && cdf[i] < target) { i++; }
            double f = (target - cdf[i - 1]) / (cdf[i] - cdf[i - 1]);
            double direction = windDirectionRadians - Math.PI / 2 + (i - 1 + f) * Math.PI / table;
            double amplitude = Math.sqrt(2 * share) / k;
            out[j] = new double[]{k * Math.cos(direction), k * Math.sin(direction), amplitude, rippleFrequency(k), random.nextDouble() * 2 * Math.PI};
        }
        return out;
    }
}
