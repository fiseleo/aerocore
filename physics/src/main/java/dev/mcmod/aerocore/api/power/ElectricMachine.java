package dev.mcmod.aerocore.api.power;

/**
 * An electric machine, motor or generator (0.26), by its losses: a fixed part (iron and friction, independent of load)
 * and a load part (copper, growing with the square of the load), as IEC 60034-2-1 separates them. Output P of a machine
 * rated Pr loses fixed + copper (P / Pr)^2; its efficiency peaks where the two are equal.
 * <p>
 * Size: rated efficiencies of IEC 60034-30-1 class IE3 (premium), four-pole 50 Hz, interpolated in log power between
 * the standard's table points (0.75 kW 82.5 %, 1.5 kW 85.3 %, 3 kW 87.7 %, 7.5 kW 90.4 %, 15 kW 92.1 %, 30 kW 93.6 %,
 * 55 kW 94.6 %, 110 kW 95.4 %, 200 kW and above 96.0 %); the losses at rated load split 40 % fixed, 60 % copper (typical
 * of induction machines at full load).
 */
public record ElectricMachine(double ratedW, double fixedLossW, double copperLossW) {
    public ElectricMachine {
        if (!(ratedW > 0) || !(fixedLossW >= 0) || !(copperLossW >= 0)) { throw new IllegalArgumentException("Rating positive, losses nonnegative"); }
    }

    private static final double[][] IE3 = {{750, 0.825}, {1500, 0.853}, {3000, 0.877}, {7500, 0.904}, {15000, 0.921}, {30000, 0.936},
            {55000, 0.946}, {110000, 0.954}, {200000, 0.960}};

    /** IE3 rated efficiency of a four-pole machine of a rating (W). */
    public static double ie3Efficiency(double ratedW) {
        if (ratedW <= IE3[0][0]) { return IE3[0][1]; }
        for (int i = 1; i < IE3.length; i++) {
            if (ratedW <= IE3[i][0]) {
                double t = Math.log(ratedW / IE3[i - 1][0]) / Math.log(IE3[i][0] / IE3[i - 1][0]);
                return IE3[i - 1][1] + t * (IE3[i][1] - IE3[i - 1][1]);
            }
        }
        return IE3[IE3.length - 1][1];
    }

    /** A machine of a rating with IE3 losses split 40 % fixed, 60 % copper at rated load. */
    public static ElectricMachine of(double ratedW) {
        double eta = ie3Efficiency(ratedW), loss = ratedW * (1 / eta - 1);
        return new ElectricMachine(ratedW, 0.4 * loss, 0.6 * loss);
    }

    /** Losses at an output (W), W. */
    public double lossW(double outputW) {
        double x = Math.abs(outputW) / ratedW;
        return fixedLossW + copperLossW * x * x;
    }

    /** Efficiency at an output (W): output over output plus losses (0 at no output). */
    public double efficiency(double outputW) {
        double p = Math.abs(outputW);
        return p <= 0 ? 0 : p / (p + lossW(p));
    }

    /** A motor: electrical input for a shaft output (W), W. */
    public double inputForOutput(double outputW) { return Math.abs(outputW) + lossW(outputW); }

    /** A generator: electrical output for a shaft input (W), W (0 when the input does not cover the fixed losses). */
    public double outputForInput(double inputW) {
        // input = P + fixed + copper P^2 / Pr^2  ->  (copper / Pr^2) P^2 + P + fixed - input = 0
        double a = copperLossW / (ratedW * ratedW), c = fixedLossW - inputW;
        if (c >= 0) { return 0; }
        return a <= 0 ? -c : (-1 + Math.sqrt(1 - 4 * a * c)) / (2 * a);
    }
}
