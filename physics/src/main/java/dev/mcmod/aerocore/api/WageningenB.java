package dev.mcmod.aerocore.api;

/**
 * Wageningen B-series open-water regression (Oosterveld and van Oossanen 1975), first quadrant only:
 * K_T = T / (rho n^2 D^4), K_Q = Q / (rho n^2 D^5) as polynomials in J, P/D, AE/A0 and Z at Rn = 2e6, with the
 * Reynolds-number correction for Rn > 2e6. Coefficients as tabulated by Bernitsas, Ray and Kinley (1981), Univ.
 * Michigan NA&ME Publ. 237, Table 1 (and Table 2 for the correction), checked against five independent copies.
 * Valid for 2 <= Z <= 7, 0.30 <= AE/A0 <= 1.05, 0.5 <= P/D <= 1.4 and 0 <= J up to zero thrust; the report warns the
 * extremes are not fully reliable. This is a regression of measured data, used here as an independent reference.
 */
public record WageningenB(int blades, double areaRatio, double pitchRatio) {
    /** {C, s, t, u, v}: C J^s (P/D)^t (AE/A0)^u Z^v. */
    private static final double[][] KT = {
            {0.00880496, 0, 0, 0, 0}, {-0.204554, 1, 0, 0, 0}, {0.166351, 0, 1, 0, 0},
            {0.158114, 0, 2, 0, 0}, {-0.147581, 2, 0, 1, 0}, {-0.481497, 1, 1, 1, 0},
            {0.415437, 0, 2, 1, 0}, {0.0144043, 0, 0, 0, 1}, {-0.0530054, 2, 0, 0, 1},
            {0.0143481, 0, 1, 0, 1}, {0.0606826, 1, 1, 0, 1}, {-0.0125894, 0, 0, 1, 1},
            {0.0109689, 1, 0, 1, 1}, {-0.133698, 0, 3, 0, 0}, {0.00638407, 0, 6, 0, 0},
            {-0.00132718, 2, 6, 0, 0}, {0.168496, 3, 0, 1, 0}, {-0.0507214, 0, 0, 2, 0},
            {0.0854559, 2, 0, 2, 0}, {-0.0504475, 3, 0, 2, 0}, {0.010465, 1, 6, 2, 0},
            {-0.00648272, 2, 6, 2, 0}, {-0.00841728, 0, 3, 0, 1}, {0.0168424, 1, 3, 0, 1},
            {-0.00102296, 3, 3, 0, 1}, {-0.0317791, 0, 3, 1, 1}, {0.018604, 1, 0, 2, 1},
            {-0.00410798, 0, 2, 2, 1}, {-0.000606848, 0, 0, 0, 2}, {-0.0049819, 1, 0, 0, 2},
            {0.0025983, 2, 0, 0, 2}, {-0.000560528, 3, 0, 0, 2}, {-0.00163652, 1, 2, 0, 2},
            {-0.000328787, 1, 6, 0, 2}, {0.000116502, 2, 6, 0, 2}, {0.000690904, 0, 0, 1, 2},
            {0.00421749, 0, 3, 1, 2}, {0.0000565229, 3, 6, 1, 2}, {-0.00146564, 0, 3, 2, 2}};
    private static final double[][] KQ = {
            {0.00379368, 0, 0, 0, 0}, {0.00886523, 2, 0, 0, 0}, {-0.032241, 1, 1, 0, 0},
            {0.00344778, 0, 2, 0, 0}, {-0.0408811, 0, 1, 1, 0}, {-0.108009, 1, 1, 1, 0},
            {-0.0885381, 2, 1, 1, 0}, {0.188561, 0, 2, 1, 0}, {-0.00370871, 1, 0, 0, 1},
            {0.00513696, 0, 1, 0, 1}, {0.0209449, 1, 1, 0, 1}, {0.00474319, 2, 1, 0, 1},
            {-0.00723408, 2, 0, 1, 1}, {0.00438388, 1, 1, 1, 1}, {-0.0269403, 0, 2, 1, 1},
            {0.0558082, 3, 0, 1, 0}, {0.0161886, 0, 3, 1, 0}, {0.00318086, 1, 3, 1, 0},
            {0.015896, 0, 0, 2, 0}, {0.0471729, 1, 0, 2, 0}, {0.0196283, 3, 0, 2, 0},
            {-0.0502782, 0, 1, 2, 0}, {-0.030055, 3, 1, 2, 0}, {0.0417122, 2, 2, 2, 0},
            {-0.0397722, 0, 3, 2, 0}, {-0.00350024, 0, 6, 2, 0}, {-0.0106854, 3, 0, 0, 1},
            {0.00110903, 3, 3, 0, 1}, {-0.000313912, 0, 6, 0, 1}, {0.0035985, 3, 0, 1, 1},
            {-0.00142121, 0, 6, 1, 1}, {-0.00383637, 1, 0, 2, 1}, {0.0126803, 0, 2, 2, 1},
            {-0.00318278, 2, 3, 2, 1}, {0.00334268, 0, 6, 2, 1}, {-0.00183491, 1, 1, 0, 2},
            {0.000112451, 3, 2, 0, 2}, {-0.0000297228, 3, 6, 0, 2}, {0.000269551, 1, 0, 1, 2},
            {0.00083265, 2, 0, 1, 2}, {0.00155334, 0, 2, 1, 2}, {0.000302683, 0, 6, 1, 2},
            {-0.0001843, 0, 0, 2, 2}, {-0.000425399, 0, 3, 2, 2}, {0.0000869243, 3, 3, 2, 2},
            {-0.0004659, 0, 6, 2, 2}, {0.0000554194, 1, 6, 2, 2}};
    /** Base Reynolds number of the polynomials. */
    public static final double BASE_REYNOLDS = 2e6;

    public WageningenB {
        if (blades < 2 || blades > 7) { throw new IllegalArgumentException("B-series blade count must be 2..7"); }
        if (!Double.isFinite(areaRatio) || areaRatio < 0.30 || areaRatio > 1.05) { throw new IllegalArgumentException("B-series AE/A0 must be in [0.30, 1.05]"); }
        if (!Double.isFinite(pitchRatio) || pitchRatio < 0.5 || pitchRatio > 1.4) { throw new IllegalArgumentException("B-series P/D must be in [0.5, 1.4]"); }
    }

    /** Thrust coefficient at Rn = 2e6. J must be in [0, 2]; beyond zero thrust the regression has no meaning. */
    public double thrust(double advanceRatio) { return polynomial(KT, advanceRatio); }
    /** Torque coefficient at Rn = 2e6. */
    public double torque(double advanceRatio) { return polynomial(KQ, advanceRatio); }

    /**
     * Thrust coefficient at blade Reynolds number Rn (chord and resultant speed at 0.75 R), with the Bernitsas et al.
     * Table 2 correction applied only above 2e6, using L = log10(Rn) - 0.301.
     */
    public double thrust(double advanceRatio, double reynolds) {
        double base = thrust(advanceRatio);
        if (!(reynolds > BASE_REYNOLDS)) { return base; }
        double j = advanceRatio, ae = areaRatio, pd = pitchRatio, z = blades, l = Math.log10(reynolds) - 0.301, pd6 = Math.pow(pd, 6);
        return base + 0.000353485 - 0.00333758 * ae * j * j - 0.00478125 * ae * pd * j
                + 0.000257792 * l * l * ae * j * j + 0.0000643192 * l * pd6 * j * j - 0.0000110636 * l * l * pd6 * j * j
                - 0.0000276305 * l * l * z * ae * j * j + 0.0000954 * l * z * ae * pd * j + 0.0000032049 * l * z * z * ae * pd * pd * pd * j;
    }
    /** Torque coefficient at blade Reynolds number Rn, corrected above 2e6 as for thrust. */
    public double torque(double advanceRatio, double reynolds) {
        double base = torque(advanceRatio);
        if (!(reynolds > BASE_REYNOLDS)) { return base; }
        double j = advanceRatio, ae = areaRatio, pd = pitchRatio, z = blades, l = Math.log10(reynolds) - 0.301, pd6 = Math.pow(pd, 6);
        return base - 0.000591412 + 0.00696898 * pd - 0.0000666654 * z * pd6 + 0.0160818 * ae * ae
                - 0.000938091 * l * pd - 0.00059593 * l * pd * pd + 0.0000782099 * l * l * pd * pd
                + 0.0000052199 * l * z * ae * j * j - 0.00000088528 * l * l * z * ae * pd * j
                + 0.0000230171 * l * z * pd6 - 0.00000184341 * l * l * z * pd6
                - 0.00400252 * l * ae * ae + 0.000220915 * l * l * ae * ae;
    }
    /** Open-water efficiency J K_T / (2 pi K_Q). */
    public double efficiency(double advanceRatio) { return advanceRatio * thrust(advanceRatio) / (2 * Math.PI * torque(advanceRatio)); }

    /** Smallest J > 0 with K_T = 0, by bisection on [0, 2]; NaN if thrust stays positive. */
    public double zeroThrustAdvanceRatio() {
        double low = 0, high = 2;
        if (thrust(high) > 0) { return Double.NaN; }
        for (int i = 0; i < 60; i++) {
            double middle = 0.5 * (low + high);
            if (thrust(middle) > 0) { low = middle; } else { high = middle; }
        }
        return 0.5 * (low + high);
    }

    /**
     * Four-quadrant table on a uniform beta grid over [-pi, pi]: between beta = 0 (bollard) and zero thrust it is this
     * regression (C_T* = K_T / ((pi/8)(J^2 + 0.49 pi^2)), likewise C_Q*); elsewhere it is the supplied model.
     * The difference regression - model found at each end of that range is faded to zero over rampRadians outside it,
     * so the table is continuous; the ramps are numerical blending, not data. Astern and windmilling quadrants are
     * as good as the supplied model (for example MARIN's measured series, BSeriesFourQuadrant).
     */
    public PropellerTable fourQuadrant(PropellerModel elsewhere, double rampRadians, int points) {
        if (points < 3) { throw new IllegalArgumentException("Need at least three table points"); }
        Checks.positive(rampRadians, "Ramp width");
        double zero = zeroThrustAdvanceRatio(), end = Math.atan2(zero, 0.7 * Math.PI);
        PropellerCoefficients startDifference = difference(elsewhere, 0), endDifference = difference(elsewhere, end);
        java.util.List<PropellerTable.Point> table = new java.util.ArrayList<>();
        for (int i = 0; i < points; i++) {
            double beta = i == points - 1 ? Math.PI : -Math.PI + 2 * Math.PI * i / (points - 1);
            PropellerCoefficients value;
            if (beta >= 0 && beta <= end) {
                value = regression(beta);
            } else {
                PropellerCoefficients base = elsewhere.sample(beta);
                PropellerCoefficients difference = beta < 0 ? startDifference : endDifference;
                double distance = beta < 0 ? -beta : beta - end, fade = distance >= rampRadians ? 0 : Math.pow(Math.cos(0.5 * Math.PI * distance / rampRadians), 2);
                value = new PropellerCoefficients(base.thrust() + fade * difference.thrust(), base.torque() + fade * difference.torque());
            }
            table.add(new PropellerTable.Point(beta, value));
        }
        return new PropellerTable(table);
    }
    /** C_T*, C_Q* of the regression at a first-quadrant beta (J = 0.7 pi tan beta). */
    public PropellerCoefficients regression(double beta) {
        double j = 0.7 * Math.PI * Math.tan(beta), scale = Math.PI / 8 * (j * j + 0.49 * Math.PI * Math.PI);
        return new PropellerCoefficients(thrust(j) / scale, torque(j) / scale);
    }
    private PropellerCoefficients difference(PropellerModel model, double beta) {
        PropellerCoefficients data = regression(beta), base = model.sample(beta);
        return new PropellerCoefficients(data.thrust() - base.thrust(), data.torque() - base.torque());
    }

    private double polynomial(double[][] terms, double j) {
        if (!Double.isFinite(j) || j < 0 || j > 2) { throw new IllegalArgumentException("Advance ratio must be in [0, 2]"); }
        double sum = 0;
        for (double[] t : terms) {
            sum += t[0] * Math.pow(j, t[1]) * Math.pow(pitchRatio, t[2]) * Math.pow(areaRatio, t[3]) * Math.pow(blades, t[4]);
        }
        return sum;
    }
}
