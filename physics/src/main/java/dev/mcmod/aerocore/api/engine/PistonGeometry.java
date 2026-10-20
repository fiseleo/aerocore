package dev.mcmod.aerocore.api.engine;

/**
 * Geometry of a piston engine's cylinder and head: clearance volume and compression ratio from the parts that make
 * them, valve packing in the bore, the combustion chamber's surface and the flame's travel. Lengths in m, volumes in
 * m^3.
 */
public final class PistonGeometry {
    private PistonGeometry() {}

    /**
     * Distance of the piston crown at top dead centre below the deck (negative: it stands proud of the deck): deck
     * height (crank centre to deck) less half the stroke, the rod length and the piston's compression height.
     */
    public static double deckClearance(double deckHeight, double stroke, double rodLength, double compressionHeight) {
        return deckHeight - (stroke / 2 + rodLength + compressionHeight);
    }

    /** Clearance volume of one cylinder: chamber, plus the gasket and deck clearance over the bore, less the crown (a dome is positive). */
    public static double clearanceVolume(double chamberVolume, double bore, double gasketThickness, double deckClearance, double crownVolume) {
        return chamberVolume + Math.PI * bore * bore / 4 * (gasketThickness + deckClearance) - crownVolume;
    }

    /** Geometric compression ratio from the swept volume of one cylinder and its clearance volume. */
    public static double compressionRatio(double sweptVolume, double clearanceVolume) { return (sweptVolume + clearanceVolume) / clearanceVolume; }

    /**
     * Whether the valve heads fit in the bore, projected on the deck: intake valves in a row on one side, exhaust on the
     * other, rows along the cam axis with a bridge between seats and a margin to the bore wall. A valve tilted by half
     * the head's included angle projects as an ellipse D cos(angle/2) across the bore and D along it. Each row moves out
     * from the centre line as far as the bore allows; they fit when every intake and exhaust seat are a bridge apart
     * (measured along the line between their centres). The 3-D chamber and the plug boss are not checked.
     */
    public static boolean valvesFit(double bore, int intakeValves, double intakeHead, int exhaustValves, double exhaustHead,
                                    double includedAngleDeg, double seatMargin, double bridge) {
        double r = bore / 2 - seatMargin, tilt = Math.cos(Math.toRadians(includedAngleDeg / 2));
        double[] yi = rowOffsets(intakeValves, intakeHead + bridge), ye = rowOffsets(exhaustValves, exhaustHead + bridge);
        double xi = rowReach(yi, intakeHead / 2 * tilt, intakeHead / 2, r), xe = rowReach(ye, exhaustHead / 2 * tilt, exhaustHead / 2, r);
        if (Double.isNaN(xi) || Double.isNaN(xe)) { return false; }
        for (double a : yi) for (double c : ye) {
            double dx = xi + xe, dy = c - a, dist = Math.hypot(dx, dy), angle = Math.atan2(dy, dx);
            double radii = ellipseRadius(intakeHead / 2 * tilt, intakeHead / 2, angle) + ellipseRadius(exhaustHead / 2 * tilt, exhaustHead / 2, angle);
            if (dist - radii < bridge - 1e-12) { return false; }
        }
        return true;
    }
    private static double[] rowOffsets(int n, double pitch) {
        double[] y = new double[n];
        for (int k = 0; k < n; k++) { y[k] = (k - (n - 1) / 2.0) * pitch; }
        return y;
    }
    /** Distance of an ellipse's boundary from its centre along a direction (semi-axes ax across, ay along). */
    private static double ellipseRadius(double ax, double ay, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return ax * ay / Math.sqrt(ay * ay * c * c + ax * ax * s * s);
    }
    /** Farthest a row of ellipses (centres at x, y_k) may sit from the centre line inside a circle of radius r; NaN if not even at x = 0. */
    private static double rowReach(double[] y, double ax, double ay, double r) {
        java.util.function.DoublePredicate fits = x -> {
            for (double yk : y) {
                for (int i = 0; i < 72; i++) {
                    double phi = Math.PI * i / 36, px = x + ax * Math.cos(phi), py = yk + ay * Math.sin(phi);
                    if (px * px + py * py > r * r) { return false; }
                }
            }
            return true;
        };
        if (!fits.test(0)) { return Double.NaN; }
        double lo = 0, hi = r;
        for (int k = 0; k < 50; k++) {
            double mid = 0.5 * (lo + hi);
            if (fits.test(mid)) { lo = mid; } else { hi = mid; }
        }
        return lo;
    }

    /** Chamber surface over the bore area of a flat chamber (a disc of the chamber's volume over the bore, its wall included). */
    public static double flatChamberAreaFactor(double bore, double chamberVolume) {
        double a = bore / 2;
        return 1 + 2 * (chamberVolume / (Math.PI * a * a)) / a;
    }

    /** Chamber surface over the bore area of a domed chamber: a spherical cap of the chamber's volume over the bore. */
    public static double domeChamberAreaFactor(double bore, double chamberVolume) {
        double a = bore / 2;
        // spherical cap of volume V over a circle of radius a: V = pi h (3 a^2 + h^2) / 6
        double lo = 0, hi = 4 * a;
        for (int i = 0; i < 80; i++) {
            double h = 0.5 * (lo + hi);
            if (Math.PI * h * (3 * a * a + h * h) / 6 < chamberVolume) { lo = h; } else { hi = h; }
        }
        double h = 0.5 * (lo + hi);
        return (a * a + h * h) / (a * a);
    }

    /**
     * Longest distance from the nearest spark plug to the bore's edge in the plane of the deck: the flame's travel. One
     * plug sits offset towards the exhaust side; two sit offset either way along the cam axis.
     */
    public static double flameTravel(double bore, int sparkPlugs, double plugOffset) {
        double a = bore / 2, worst = 0;
        for (int i = 0; i < 360; i++) {
            double phi = Math.toRadians(i + 0.5), x = a * Math.cos(phi), y = a * Math.sin(phi), nearest = Double.MAX_VALUE;
            for (int k = 0; k < sparkPlugs; k++) {
                double px = sparkPlugs == 1 ? plugOffset : 0, py = sparkPlugs == 1 ? 0 : (k == 0 ? plugOffset : -plugOffset);
                nearest = Math.min(nearest, Math.hypot(x - px, y - py));
            }
            worst = Math.max(worst, nearest);
        }
        return worst;
    }

    /**
     * Moment of inertia of everything turning with the crank, kg m^2: the crank's own, the rods' rotating share at the
     * crank radius, and the reciprocating mass averaged over a turn (half of it at the crank radius).
     */
    public static double crankSystemInertia(double crankInertia, int cylinders, double rodMass, double rodReciprocatingFraction,
                                            double reciprocatingMass, double crankRadius) {
        double r2 = crankRadius * crankRadius;
        return crankInertia + cylinders * (1 - rodReciprocatingFraction) * rodMass * r2 + cylinders * 0.5 * reciprocatingMass * r2;
    }
}
