package dev.mcmod.aerocore.api;

/**
 * Empirical and slender-body ingredients of hull manoeuvring forces. Body axes: x forward, y up, z starboard.
 * Clarke's derivatives use his own convention (y starboard, yaw positive bow to starboard) and the L^2 prime
 * system: Y_v = 1/2 rho L^2 U Y'_v, Y_r and N_v = 1/2 rho L^3 U (.)', N_r = 1/2 rho L^4 U N'_r.
 */
public final class HullManoeuvring {
    private HullManoeuvring() {}

    /** Clarke, Gedling and Hine (1983), Trans. RINA 125:45, linear hull derivatives (as coded in Fossen's MSS clarke83.m). */
    public record Clarke(double yv, double yr, double nv, double nr, double yvDot, double yrDot, double nvDot, double nrDot) {
        public static Clarke of(double length, double beam, double draft, double blockCoefficient) {
            Checks.positive(length, "Length"); Checks.positive(beam, "Beam"); Checks.positive(draft, "Draft");
            double s = Math.PI * Math.pow(draft / length, 2), bt = beam / draft, bl = beam / length, tl = draft / length, cb = blockCoefficient;
            return new Clarke(-s * (1 + 0.40 * cb * bt), -s * (-0.5 + 2.2 * bl - 0.080 * bt), -s * (0.5 + 2.4 * tl),
                    -s * (0.25 + 0.039 * bt - 0.56 * bl), -s * (1 + 0.16 * cb * bt - 5.1 * bl * bl), -s * (0.67 * bl - 0.0033 * bt * bt),
                    -s * (1.1 * bl - 0.041 * bt), -s * (1.0 / 12 + 0.017 * cb * bt - 0.33 * bl));
        }
    }

    /**
     * Sectional sway (low frequency, rigid free surface) and heave (high frequency) added mass per metre of a
     * surface-piercing section by the Lewis form: breadth B and draft T at the waterline and area A.
     * m22 = (rho pi / 2) M^2 [(1 - a1)^2 + 3 a3^2], m33 = (rho pi / 2) M^2 [(1 + a1)^2 + 3 a3^2], M = (B/2) / (1 + a1 + a3)
     * (Lewis 1929; the sway form follows from the same mapping with a1 -> -a1). Exact for the half-immersed circle;
     * Lewis forms cannot represent every (B/T, sigma) pair, so sigma is held inside the representable range.
     */
    public static double[] lewisAddedMass(double breadth, double draft, double area, double density) {
        if (!(breadth > 0) || !(draft > 0) || !(area > 0)) { return new double[]{0, 0}; }
        double h = breadth / (2 * draft), sigma = Math.clamp(area / (breadth * draft), 0.35, 1.0), r = (h - 1) / (h + 1);
        double c1 = (3 + 4 * sigma / Math.PI) + (1 - 4 * sigma / Math.PI) * r * r;
        c1 = Math.min(c1, 4.5);
        double a3 = (-c1 + 3 + Math.sqrt(9 - 2 * c1)) / c1, a1 = r * (a3 + 1);
        double m = breadth / 2 / (1 + a1 + a3), scale = density * Math.PI / 2 * m * m;
        return new double[]{scale * ((1 - a1) * (1 - a1) + 3 * a3 * a3), scale * ((1 + a1) * (1 + a1) + 3 * a3 * a3)};
    }

    /**
     * Two-dimensional cross-flow drag coefficient of a ship section in lateral flow, against B / (2T) of its double
     * body: Hoerner's data as digitised by Leite et al. (1998), Appl. Ocean Res. 20:145 (Fossen MSS Hoerner.m).
     */
    public static double hoernerCrossFlow(double breadthOverTwiceDraft) {
        double[][] table = {{0.01, 1.97}, {0.35, 1.90}, {0.45, 1.79}, {0.47, 1.58}, {0.49, 1.28}, {0.56, 1.08}, {0.65, 1.00},
                {0.83, 0.88}, {0.99, 0.83}, {1.31, 0.76}, {1.64, 0.69}, {2.31, 0.63}, {3.0, 0.59}, {4.0, 0.56}};
        double x = Math.clamp(breadthOverTwiceDraft, table[0][0], table[table.length - 1][0]);
        for (int i = 1; i < table.length; i++) {
            if (x <= table[i][0]) {
                double t = (x - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
                return table[i - 1][1] + t * (table[i][1] - table[i - 1][1]);
            }
        }
        return table[table.length - 1][1];
    }

    /**
     * Two-dimensional drag coefficient of a rectangle with rounded corners in steady flow at Re about 1e5 (DNV-RP-C205
     * 2010, Table E-1 no. 4): L the length along the flow, D the width across it, R the corner radius. Tabulated for
     * L/D 0.5 (R/D 0, 0.083, 0.25 -> 2.5, 1.9, 1.6), 1.0 (0, 0.167 -> 2.2, 1.2) and 2.0 (0, 0.167 -> 1.6, 0.7); linear
     * between, held at the table's edges outside it.
     */
    public static double roundedRectangleDrag(double lengthOverWidth, double radiusOverWidth) {
        double[] ratios = {0.5, 1.0, 2.0};
        double[][][] rows = {{{0, 2.5}, {0.083, 1.9}, {0.25, 1.6}}, {{0, 2.2}, {0.167, 1.2}}, {{0, 1.6}, {0.167, 0.7}}};
        double[] values = new double[3];
        for (int i = 0; i < 3; i++) { values[i] = interpolate(rows[i], radiusOverWidth); }
        double x = Math.clamp(lengthOverWidth, 0.5, 2.0);
        int i = x <= 1.0 ? 0 : 1;
        double t = (x - ratios[i]) / (ratios[i + 1] - ratios[i]);
        return values[i] + t * (values[i + 1] - values[i]);
    }
    private static double interpolate(double[][] table, double x) {
        double c = Math.clamp(x, table[0][0], table[table.length - 1][0]);
        for (int i = 1; i < table.length; i++) {
            if (c <= table[i][0]) { double t = (c - table[i - 1][0]) / (table[i][0] - table[i - 1][0]); return table[i - 1][1] + t * (table[i][1] - table[i - 1][1]); }
        }
        return table[table.length - 1][1];
    }

    /**
     * Cross-flow drag coefficient of a circular cylinder against Re = v_cross D / nu (DNV-RP-C205 curve as in MSS
     * cylinderDrag.m: about 1.2 subcritical, 0.28 at the drag crisis near 6.4e5, 0.68 supercritical), log-linear between.
     */
    public static double cylinderCrossFlow(double reynolds) {
        double[][] table = {{2e5, 1.21}, {6.4e5, 0.28}, {4.9e6, 0.68}};
        if (!(reynolds > table[0][0])) { return table[0][1]; }
        if (reynolds >= table[2][0]) { return table[2][1]; }
        int i = reynolds < table[1][0] ? 1 : 2;
        double t = Math.log(reynolds / table[i - 1][0]) / Math.log(table[i][0] / table[i - 1][0]);
        return table[i - 1][1] + t * (table[i][1] - table[i - 1][1]);
    }

    /** Jorgensen (1977) cross-flow efficiency of a finite body of revolution, fit eta(L/D) (aerosandbox jorgensen_eta). */
    public static double jorgensenEta(double fineness) {
        return 1 - 23.009 / (fineness + 122.769) - Math.pow(13.006 / (fineness + 24.368), 2);
    }

    /** Söding (1982) surge added mass of a ship, m11 = 2.7 rho V^(5/3) / L^2. */
    public static double sodingSurgeAddedMass(double volume, double length, double density) {
        return 2.7 * density * Math.pow(volume, 5.0 / 3) / (length * length);
    }

    /** Lamb's added-mass coefficients {k1, k2, k'} of a prolate spheroid with fineness L/D > 1 (Imlay 1961). */
    public static double[] spheroidCoefficients(double fineness) {
        if (!(fineness > 1)) { throw new IllegalArgumentException("Prolate spheroid needs L/D > 1"); }
        double e = Math.sqrt(1 - 1 / (fineness * fineness)), log = Math.log((1 + e) / (1 - e));
        double alpha = 2 * (1 - e * e) / (e * e * e) * (0.5 * log - e), beta = 1 / (e * e) - (1 - e * e) / (2 * e * e * e) * log;
        double k1 = alpha / (2 - alpha), k2 = beta / (2 - beta);
        double kPrime = Math.pow(e, 4) * (beta - alpha) / ((2 - e * e) * (2 * e * e - (2 - e * e) * (beta - alpha)));
        return new double[]{k1, k2, kPrime};
    }
}
