package dev.mcmod.aerocore.api.engine;

/**
 * The colour of a black body at a temperature (0.29), for glowing metal, soot and particles in a flame: Planck's law
 * integrated against the CIE 1931 2-degree colour matching functions (the multi-lobe Gaussian fits of Wyman, Sloan and
 * Shirley 2013), XYZ to linear sRGB (D65), scaled so the largest channel is 1, then sRGB-encoded. Brightness is
 * separate ({@link #radiance}): the colour alone is a chromaticity. Checked against Mitchell Charity's blackbody
 * colour table.
 */
public final class Blackbody {
    private Blackbody() {}

    private static final double H = 6.62607015e-34, C = 2.99792458e8, K = 1.380649e-23;

    /** Spectral radiance B(lambda, T), W / (m^2 sr m). */
    static double planck(double lambdaM, double kelvin) {
        double a = 2 * H * C * C / Math.pow(lambdaM, 5);
        return a / Math.expm1(H * C / (lambdaM * K * kelvin));
    }

    private static double g(double x, double mu, double s1, double s2) {
        double t = (x - mu) / (x < mu ? s1 : s2);
        return Math.exp(-0.5 * t * t);
    }

    /** CIE 1931 2-degree x-bar, y-bar, z-bar at a wavelength in nanometres (Wyman, Sloan and Shirley 2013, multi-lobe fit). */
    static double[] cmf(double nm) {
        double x = 1.056 * g(nm, 599.8, 37.9, 31.0) + 0.362 * g(nm, 442.0, 16.0, 26.7) - 0.065 * g(nm, 501.1, 20.4, 26.2);
        double y = 0.821 * g(nm, 568.8, 46.9, 40.5) + 0.286 * g(nm, 530.9, 16.3, 31.1);
        double z = 1.217 * g(nm, 437.0, 11.8, 36.0) + 0.681 * g(nm, 459.0, 26.0, 13.8);
        return new double[]{x, y, z};
    }

    /** CIE XYZ of a black body (unnormalised), from 380 to 780 nm in 1 nm steps. */
    static double[] xyz(double kelvin) {
        double[] out = new double[3];
        for (int nm = 380; nm <= 780; nm++) {
            double b = planck(nm * 1e-9, kelvin);
            double[] c = cmf(nm);
            out[0] += b * c[0]; out[1] += b * c[1]; out[2] += b * c[2];
        }
        return out;
    }

    /** sRGB (0-1 each, largest 1) of a black body at a temperature (K, at least 500). */
    public static double[] srgb(double kelvin) {
        double[] v = xyz(Math.max(500, kelvin));
        double r = 3.2406 * v[0] - 1.5372 * v[1] - 0.4986 * v[2];
        double g = -0.9689 * v[0] + 1.8758 * v[1] + 0.0415 * v[2];
        double b = 0.0557 * v[0] - 0.2040 * v[1] + 1.0570 * v[2];
        r = Math.max(0, r); g = Math.max(0, g); b = Math.max(0, b);
        double max = Math.max(r, Math.max(g, b));
        if (max <= 0) { return new double[]{0, 0, 0}; }
        return new double[]{encode(r / max), encode(g / max), encode(b / max)};
    }

    /** As {@link #srgb}, packed ARGB with a given alpha (0-255). */
    public static int argb(double kelvin, int alpha) {
        double[] c = srgb(kelvin);
        return (alpha & 255) << 24 | (int) Math.round(255 * c[0]) << 16 | (int) Math.round(255 * c[1]) << 8 | (int) Math.round(255 * c[2]);
    }

    private static double encode(double linear) {
        return linear <= 0.0031308 ? 12.92 * linear : 1.055 * Math.pow(linear, 1 / 2.4) - 0.055;
    }

    /**
     * How bright a black body at a temperature looks next to one at a reference (the luminance ratio, the
     * eye-weighted integral of Planck's law): glowing steel at 900 K is about a ten-thousandth of a flame at 2000 K.
     */
    public static double radiance(double kelvin, double referenceKelvin) {
        return xyz(Math.max(300, kelvin))[1] / xyz(referenceKelvin)[1];
    }
}
