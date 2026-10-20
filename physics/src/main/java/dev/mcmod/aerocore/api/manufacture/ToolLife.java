package dev.mcmod.aerocore.api.manufacture;

/**
 * Tool life (0.35), after Groover (Fundamentals of Modern Manufacturing, ch. 23-24) and Kalpakjian and Schmid
 * (Manufacturing Engineering and Technology, ch. 21).
 *
 * <ul>
 * <li>Taylor's equation: v T^n = C, v the cutting speed and T the tool's life at it. C is the speed that wears the
 *     edge out in one minute, so here T = 60 (C / v)^(1/n) s. n depends on the tool material (high-speed steel 0.125,
 *     cemented carbide 0.25, ceramics 0.6) and C on the tool and the work: Groover table 23.2 gives high-speed steel
 *     70 m/min on steel and 120 on other metals, cemented carbide 500 and 900; (0.40) plain carbon tool steel, the
 *     tool before high-speed steel, n 0.10 and 20 and 70 m/min.</li>
 * <li>Two tests at speeds v1, v2 with lives T1, T2 give n = ln(v1 / v2) / ln(T2 / T1) and C = v1 T1^n (T in minutes).</li>
 * <li>Economic lives (Groover sec. 24.3): the life that gives the most pieces an hour is (1 / n - 1) T_t, T_t the time
 *     to change the tool; the life that gives the least cost a piece is (1 / n - 1) (T_t + C_te / C_o), C_te the cost
 *     of a cutting edge and C_o the cost of the machine and operator a second.</li>
 * </ul>
 */
public final class ToolLife {
    private ToolLife() {}

    /** Taylor exponents: high-speed steel, cemented carbide, ceramics (Groover table 23.2). */
    public static final double N_HSS = 0.125, N_CARBIDE = 0.25, N_CERAMIC = 0.6;
    /** One-minute-life speeds on steel and on other metals, m/s (Groover table 23.2). */
    public static final double C_HSS_STEEL = 70.0 / 60, C_HSS_NONSTEEL = 120.0 / 60, C_CARBIDE_STEEL = 500.0 / 60, C_CARBIDE_NONSTEEL = 900.0 / 60;
    /** Plain carbon tool steel (0.40): its exponent and one-minute-life speeds on steel and other metals, m/s (Groover table 23.2). */
    public static final double N_CARBON = 0.10, C_CARBON_STEEL = 20.0 / 60, C_CARBON_NONSTEEL = 70.0 / 60;

    /** A tool and work pair's Taylor constants: the exponent and the one-minute-life speed (m/s). */
    public record Taylor(double n, double oneMinuteMps) {
        public Taylor {
            if (!(n > 0 && n < 1) || !(oneMinuteMps > 0)) { throw new IllegalArgumentException("Taylor n in (0, 1), C positive"); }
        }
        /** The life at a speed, s. */
        public double lifeS(double speedMps) { return ToolLife.lifeS(speedMps, oneMinuteMps, n); }
        /** The speed that gives a life (s), m/s. */
        public double speedFor(double lifeS) { return ToolLife.speedForLifeMps(lifeS, oneMinuteMps, n); }
    }

    /** The life of a tool at a speed (m/s) whose one-minute-life speed is C (m/s), with Taylor exponent n, s. */
    public static double lifeS(double speedMps, double oneMinuteMps, double n) {
        check(speedMps, oneMinuteMps, n);
        return 60 * Math.pow(oneMinuteMps / speedMps, 1 / n);
    }

    /** The speed (m/s) that wears a tool out in a life (s). */
    public static double speedForLifeMps(double lifeS, double oneMinuteMps, double n) {
        check(lifeS, oneMinuteMps, n);
        return oneMinuteMps / Math.pow(lifeS / 60, n);
    }

    /** The Taylor constants from two tests: a speed (m/s) and the life it gave (s), each. */
    public static Taylor fit(double speed1Mps, double life1S, double speed2Mps, double life2S) {
        check(speed1Mps, life1S, 0.5);
        check(speed2Mps, life2S, 0.5);
        double n = Math.log(speed1Mps / speed2Mps) / Math.log(life2S / life1S);
        return new Taylor(n, speed1Mps * Math.pow(life1S / 60, n));
    }

    /** The life that gives the most pieces an hour, for a time to change the tool (s), s. */
    public static double maxProductionLifeS(double n, double changeS) {
        if (!(n > 0 && n < 1) || !(changeS >= 0)) { throw new IllegalArgumentException("Taylor n in (0, 1), change time not negative"); }
        return (1 / n - 1) * changeS;
    }

    /** The life that gives the least cost a piece: change time (s), a cutting edge's cost and the machine's cost a second (same money), s. */
    public static double minCostLifeS(double n, double changeS, double edgeCost, double costPerS) {
        if (!(costPerS > 0) || !(edgeCost >= 0)) { throw new IllegalArgumentException("Machine cost positive, edge cost not negative"); }
        return maxProductionLifeS(n, changeS + edgeCost / costPerS);
    }

    private static void check(double a, double c, double n) {
        if (!(a > 0) || !(c > 0) || !(n > 0 && n < 1)) { throw new IllegalArgumentException("Speeds and lives positive, Taylor n in (0, 1)"); }
    }
}
