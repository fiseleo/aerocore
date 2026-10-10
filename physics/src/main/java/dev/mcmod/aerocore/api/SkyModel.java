package dev.mcmod.aerocore.api;

/**
 * Daylight sky colour (Preetham, Shirley and Smits 1999, "A Practical Analytic Model for Daylight", SIGGRAPH; their
 * appendix A.2): the clear sky's luminance Y and CIE chromaticity x, y in any direction follow Perez et al.'s (1993)
 * all-weather distribution F(theta, gamma) = (1 + A exp(B / cos theta)) (1 + C exp(D gamma) + E cos^2 gamma), theta the
 * view's angle from the zenith, gamma its angle from the sun, scaled to the zenith values:
 * Y = Y_z F(theta, gamma) / F(0, theta_s); A..E are linear in the turbidity T (2 very clear, 3 clear, 6 hazy) and the
 * zenith Y_z (kcd/m^2), x_z, y_z are Preetham's fits in T and the sun's zenith angle theta_s. Valid for the sun above
 * the horizon; this class holds theta_s at 90 degrees below that (callers dim the twilight). Overcast sky: the CIE
 * standard overcast luminance distribution Y = Y_z (1 + 2 cos theta) / 3 (Moon and Spencer 1942) in the D65 white.
 * Colours convert from CIE xyY to linear sRGB (IEC 61966-2-1 matrix, D65).
 */
public final class SkyModel {
    /** A clear sky for one turbidity and sun: Perez coefficients {A, B, C, D, E} for Y, x and y, and the zenith values. */
    public record Clear(double turbidity, double sunZenith, double[] luminance, double[] chromaX, double[] chromaY,
                        double zenithLuminance, double zenithX, double zenithY) {
        /**
         * Mean luminance over the sky hemisphere (per unit solid angle, kcd/m^2), on a 16 x 32 grid: what an eye or
         * camera adapting to the whole sky would expose for.
         */
        public double meanLuminance() {
            double total = 0, weight = 0, sx = Math.sin(sunZenith), sy = Math.cos(sunZenith);
            for (int i = 0; i < 16; i++) {
                double theta = (i + 0.5) / 16 * Math.PI / 2, w = Math.sin(theta);
                for (int j = 0; j < 32; j++) {
                    double azimuth = (j + 0.5) / 32 * 2 * Math.PI;
                    double cosGamma = Math.sin(theta) * Math.cos(azimuth) * sx + Math.cos(theta) * sy;
                    total += w * xyY(theta, Math.acos(Math.clamp(cosGamma, -1, 1)))[0];
                    weight += w;
                }
            }
            return total / weight;
        }

        /** Luminance (kcd/m^2) and chromaticity {Y, x, y} toward a direction at zenith angle theta, gamma from the sun. */
        public double[] xyY(double theta, double gamma) {
            double t = Math.min(theta, Math.PI / 2 - 1e-3);
            return new double[]{
                    zenithLuminance * perez(luminance, t, gamma) / perez(luminance, 0, sunZenith),
                    zenithX * perez(chromaX, t, gamma) / perez(chromaX, 0, sunZenith),
                    zenithY * perez(chromaY, t, gamma) / perez(chromaY, 0, sunZenith)};
        }
    }

    /** CIE D65 white chromaticity. */
    public static final double D65_X = 0.3127, D65_Y = 0.3290;

    private SkyModel() {}

    /** Perez et al.'s distribution with coefficients {A, B, C, D, E}. */
    public static double perez(double[] c, double theta, double gamma) {
        return (1 + c[0] * Math.exp(c[1] / Math.cos(theta))) * (1 + c[2] * Math.exp(c[3] * gamma) + c[4] * Math.cos(gamma) * Math.cos(gamma));
    }

    /** Preetham's clear sky for turbidity T (1.7-10) and the sun's zenith angle (rad; held at 90 degrees). */
    public static Clear clear(double turbidity, double sunZenith) {
        double t = Math.clamp(turbidity, 1.7, 10), s = Math.clamp(sunZenith, 0, Math.PI / 2);
        double[] lum = {0.1787 * t - 1.4630, -0.3554 * t + 0.4275, -0.0227 * t + 5.3251, 0.1206 * t - 2.5771, -0.0670 * t + 0.3703};
        double[] cx = {-0.0193 * t - 0.2592, -0.0665 * t + 0.0008, -0.0004 * t + 0.2125, -0.0641 * t - 0.8989, -0.0033 * t + 0.0452};
        double[] cy = {-0.0167 * t - 0.2608, -0.0950 * t + 0.0092, -0.0079 * t + 0.2102, -0.0441 * t - 1.6537, -0.0109 * t + 0.0529};
        double chi = (4.0 / 9 - t / 120) * (Math.PI - 2 * s);
        double yz = (4.0453 * t - 4.9710) * Math.tan(chi) - 0.2155 * t + 2.4192;
        double s2 = s * s, s3 = s2 * s, t2 = t * t;
        double xz = t2 * (0.00166 * s3 - 0.00375 * s2 + 0.00209 * s)
                + t * (-0.02903 * s3 + 0.06377 * s2 - 0.03202 * s + 0.00394)
                + (0.11693 * s3 - 0.21196 * s2 + 0.06052 * s + 0.25886);
        double ychrom = t2 * (0.00275 * s3 - 0.00610 * s2 + 0.00317 * s)
                + t * (-0.04214 * s3 + 0.08970 * s2 - 0.04153 * s + 0.00516)
                + (0.15346 * s3 - 0.26756 * s2 + 0.06670 * s + 0.26688);
        return new Clear(t, s, lum, cx, cy, Math.max(yz, 1e-3), xz, ychrom);
    }

    /** Relative luminance of the CIE standard overcast sky at zenith angle theta (1 at the zenith). */
    public static double overcast(double theta) { return (1 + 2 * Math.cos(Math.min(theta, Math.PI / 2))) / 3; }

    /** Linear sRGB (D65) of CIE xyY. */
    public static double[] linearRgb(double x, double y, double luminance) {
        double yy = Math.max(y, 1e-6), bigX = x / yy * luminance, bigZ = (1 - x - y) / yy * luminance;
        return new double[]{
                3.2406 * bigX - 1.5372 * luminance - 0.4986 * bigZ,
                -0.9689 * bigX + 1.8758 * luminance + 0.0415 * bigZ,
                0.0557 * bigX - 0.2040 * luminance + 1.0570 * bigZ};
    }

    /** The sRGB transfer (IEC 61966-2-1) of a linear value in 0..1. */
    public static double encode(double linear) {
        double v = Math.clamp(linear, 0, 1);
        return v <= 0.0031308 ? 12.92 * v : 1.055 * Math.pow(v, 1 / 2.4) - 0.055;
    }
}
