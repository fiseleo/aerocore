package dev.mcmod.aerocore.api.engine;

/**
 * Friction mean effective pressure of a spark-ignition engine by the component model of Patton, Nitschke and
 * Heywood (SAE 890836, 1989) as revised by Sandoval and Heywood (SAE 2003-01-0725; Sandoval 2002, MIT thesis,
 * eq. 7b-11 and appendix A-2): crankshaft (main seals, main bearings, oil pumping through bearings), reciprocating
 * (piston skirt, rings, connecting-rod bearings, gas-pressure loading of the rings), valvetrain (camshaft bearings,
 * followers, oscillating parts) and auxiliaries (oil pump, water pump, unloaded alternator). Hydrodynamic terms scale
 * with sqrt(nu / nu_ref) (EngineOil). Pumping is not here: the cycle simulation computes it.
 * <p>
 * Units inside the correlations are the paper's: bore, stroke, bearing sizes and valve lift in mm, speed in rpm,
 * mean piston speed in m/s, pressures and results in kPa; this class converts at its boundary. Calibrated on engines
 * of 1980-1989 and 2002 (production V6, V8, V10 up to 6000 rpm); the auxiliary term's speed polynomial peaks near
 * 14600 rpm and is held there above it (its quadratic would otherwise fall).
 * <p>
 * Valvetrain constants per configuration are Sandoval's table 4.2. The OHV roller-follower constant printed there
 * (0.5) would make a roller follower several times the friction of a flat one, against every other row; this class
 * uses 0.0151, the rocker-arm value (pushrod engines drive their valves through rocker arms), an estimate.
 */
public final class Friction {
    private Friction() {}

    /** Camshaft placement and valve actuation, with Sandoval's constants (flat follower, roller follower, oscillating hydrodynamic, oscillating mixed). */
    public enum Valvetrain {
        SOHC_FINGER_FOLLOWER(600, 0.0227, 0.2, 42.8),
        SOHC_ROCKER_ARM(400, 0.0151, 0.5, 21.4),
        SOHC_DIRECT_ACTING(200, 0.0076, 0.5, 10.7),
        DOHC_FINGER_FOLLOWER(600, 0.0227, 0.2, 25.8),
        DOHC_DIRECT_ACTING(133, 0.0050, 0.5, 10.7),
        OHV(400, 0.0151, 0.5, 32.1);

        final double flat, roller, oscillatingHydrodynamic, oscillatingMixed;
        Valvetrain(double flat, double roller, double oscillatingHydrodynamic, double oscillatingMixed) {
            this.flat = flat; this.roller = roller; this.oscillatingHydrodynamic = oscillatingHydrodynamic; this.oscillatingMixed = oscillatingMixed;
        }
        /** Camshafts per cylinder bank (OHV: one for the whole engine). */
        public int camshaftsPerBank() { return this == OHV ? 0 : name().startsWith("DOHC") ? 2 : 1; }
    }

    /**
     * The engine as the friction model sees it. Lengths in metres here; ringTension is the ring tangential tension
     * relative to the model's 1980s production reference (F_t/F_to), linerRoughness its C_r (1 = reference finish).
     */
    public record Engine(double boreM, double strokeM, int cylinders, double compressionRatio,
                         int mainBearings, double mainDiameterM, double mainLengthM,
                         double rodDiameterM, double rodLengthM,
                         int camBearings, int valves, double maxLiftM, Valvetrain valvetrain, boolean rollerFollowers,
                         double ringTension, double linerRoughness) {
        public Engine {
            if (!(boreM > 0) || !(strokeM > 0) || cylinders < 1 || mainBearings < 1 || valves < 1) { throw new IllegalArgumentException("Engine geometry must be positive"); }
            java.util.Objects.requireNonNull(valvetrain, "valvetrain");
        }
    }

    /** The parts of fmep, kPa. */
    public record Breakdown(double crankshaft, double reciprocating, double gasLoading, double valvetrain, double auxiliary) {
        public double total() { return crankshaft + reciprocating + gasLoading + valvetrain + auxiliary; }
        /** Total, Pa. */
        public double totalPa() { return total() * 1000; }
    }

    /**
     * fmep at a speed (rpm), intake manifold and ambient pressures (Pa) and oil viscosity scaling sqrt(nu/nu_ref)
     * (EngineOil.frictionScaling).
     */
    public static Breakdown fmep(Engine e, double rpm, double intakePressurePa, double ambientPressurePa, double viscosityScaling) {
        double n = Math.max(rpm, 100), b = e.boreM * 1000, s = e.strokeM * 1000, nc = e.cylinders, mu = viscosityScaling;
        double sp = 2 * e.strokeM * n / 60;
        double db = e.mainDiameterM * 1000, lb = e.mainLengthM * 1000, nb = e.mainBearings;
        double dr = e.rodDiameterM * 1000, lr = e.rodLengthM * 1000;
        double crankshaft = 1.22e5 * db / (b * b * s * nc) + 3.03e-4 * mu * n * db * db * db * lb * nb / (b * b * s * nc)
                + 1.35e-10 * db * db * n * n * nb / nc;
        double reciprocating = 2.94e2 * mu * sp / b + 4.06e4 * e.ringTension * e.linerRoughness * (1 + 500 / n) / (b * b)
                + 3.03e-4 * mu * n * dr * dr * dr * lr * nc / (b * b * s * nc);
        double pr = intakePressurePa / ambientPressurePa;
        double gas = 6.89 * pr * (0.088 * mu * e.compressionRatio + 0.182 * e.ringTension * Math.pow(e.compressionRatio, 1.33 - 2 * 2.38e-2 * sp));
        Valvetrain v = e.valvetrain;
        double lv = e.maxLiftM * 1000, nv = e.valves;
        double valvetrain = 4.12 + 244 * mu * n * e.camBearings / (b * b * s * nc)
                + (e.rollerFollowers ? v.roller * n * nv / (s * nc) : v.flat * (1 + 500 / n) * nv / (s * nc))
                + v.oscillatingHydrodynamic * mu * Math.pow(lv, 1.5) * Math.sqrt(n) * nv / (b * s * nc)
                + v.oscillatingMixed * (1 + 500 / n) * lv * nv / (s * nc);
        double na = Math.min(n, 5.22e-3 / (2 * 1.79e-7));
        double auxiliary = 6.23 + 5.22e-3 * na - 1.79e-7 * na * na;
        return new Breakdown(crankshaft, reciprocating, gas, valvetrain, auxiliary);
    }
}
