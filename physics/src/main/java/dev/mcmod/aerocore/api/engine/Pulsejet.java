package dev.mcmod.aerocore.api.engine;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.IntStream;

/**
 * A pulsejet (0.28): the unsteady gas in its tube simulated in one dimension until it settles into its cycle, then
 * averaged (the way the piston engine runs its crank-angle cycle simulation rather than an efficiency map). Valved
 * (Argus As 014: reeds behind the intake) and valveless (Lockwood-Hiller: an open intake pipe, bent rearward).
 * <ul>
 * <li>Flow: quasi-one-dimensional Euler equations in a tube of varying area (intake pipe, combustion chamber, taper,
 * tailpipe), finite volumes with the HLLC Riemann solver (Toro, Riemann Solvers and Numerical Methods for Fluid Dynamics,
 * ch. 10), MUSCL reconstruction (minmod) and SSP Runge-Kutta 2; the area change as the source p dA/dx; wall friction
 * (Fanning 0.004). Fuel vapour and burned products are carried as mass fractions. Calorically perfect gas (gamma 1.35,
 * R of air: a mean of the cold charge and the hot products, as pulsejet simulations usually take it).</li>
 * <li>Fuel: fed at a set rate (the fuel valve; the V-1's nozzles were pressure fed), sprayed behind the reeds into the
 * front quarter of the chamber, or mid-chamber in a valveless engine.</li>
 * <li>Mixing: the jets coming in through the reeds or the intake stir the chamber and taper over the design's mixing time,
 * modelled as turbulent exchange of equal volumes between cells (every conserved quantity diffuses per volume, implicitly,
 * so a uniform pressure stays uniform) with D = L^2 / (pi^2 tau); elsewhere a mixing length, D = 0.02 (|u| + 10 m/s) d.</li>
 * <li>Combustion: Magnussen and Hjertager's eddy break-up (16th Symposium on Combustion, 1976), the fuel burning at
 * 4 rho min(Y_fuel, Y_air / AFR, 0.5 Y_products / (1 + AFR)) / tau: the charge burns as fast as it mixes with air and
 * with the hot gas left from the last cycle. It burns only where the gas is above the intake's total pressure: the charge
 * that comes in while the chamber is below it does not light until the compression returns (Keller, Bramlette, Westbrook
 * and Dec, Combust. Flame 79, 1990: ignition at the compression; the incoming jets' strain and the expansion keep it
 * out). Without that condition this model burns the charge as it enters and the tube does not ring. The starting spark
 * lights the first charge whatever the pressure.</li>
 * <li>Reeds: a damped spring-mass (natural frequency, damping ratio 0.1) driven by the pressure across them, from the seat
 * to the stop plate, opening an orifice (discharge 0.6, compressible, choked at the critical ratio); while they have not
 * shut, hot gas blows out through them.</li>
 * <li>Ends: an open end releases to ambient pressure when flowing out (the pressure release that makes the tube ring) and
 * draws from the surroundings' total state (the ram's for an end facing forward) less a sharp edge's entry loss (0.5 of
 * the dynamic pressure, Idelchik) when flowing in.</li>
 * <li>Walls: heat to them by Dittus-Boelter times 2.5 for pulsating flow (Dec and Keller, Combust. Flame 77, 1989), each
 * wall at the temperature where that mean heat equals what it radiates (emissivity 0.8) and convects outside (still air,
 * plus a turbulent flat plate at flight speed).</li>
 * <li>Thrust: the cycle-averaged momentum and pressure flux through the open ends (each facing rearward or forward) and
 * the reeds' blow-back, less the ram drag of all air taken in at flight speed (Foa, Elements of Flight Propulsion, ch. 13).</li>
 * </ul>
 * Started from the chamber full of stoichiometric mixture with its front fifth lit (the starting air and spark; whether
 * the part has them is its rule), run for a time and averaged over its last whole cycles; it has died if the pressure no
 * longer swings. Calibration and validation (Argus As 014 within 25 % on thrust and fuel, 15 % on frequency) in
 * docs/PULSEJET.md. Not modelled: three-dimensional jet mixing beyond its time scale, the U-bend's losses, acoustic
 * radiation, reed fatigue (the part's rule), dissociation.
 */
public final class Pulsejet {
    private Pulsejet() {}

    public enum Kind { VALVED, VALVELESS }

    static final double GAMMA = 1.35, R = 287.05287, CV = R / (GAMMA - 1), CP = GAMMA * CV;
    static final double DISCHARGE = 0.6;
    /** The reeds' damping ratio (spring steel in air: a few percent of structural damping plus the air's). */
    static final double REED_DAMPING = 0.1;
    /** Fanning friction factor of the walls (turbulent, Re 1e5-1e6, smooth to drawn steel tube). */
    static final double FRICTION = 0.004;
    /** Loss coefficient of air drawn into a sharp-edged open end (Idelchik, Handbook of Hydraulic Resistance, diagram 3-1). */
    static final double ENTRY_LOSS = 0.5;
    /** Mixing-length coefficient outside the chamber (1-3 % of velocity times diameter is the usual range). */
    static final double MIXING = 0.02;
    /** Magnussen and Hjertager's constants. */
    static final double EBU_A = 4, EBU_B = 0.5;
    /** How long the starting spark keeps the charge alight whatever the pressure, s. */
    static final double SPARK_S = 0.003;
    /** Dittus-Boelter's enhancement in a pulse combustor (Dec and Keller 1989: up to about 2.5 in the tailpipe). */
    static final double HEAT_ENHANCEMENT = 2.5;
    /** The outside of the tube: oxidized steel's emissivity, and still air's natural convection (W/m^2 K). */
    static final double EMISSIVITY = 0.8, STILL_AIR = 10, SIGMA = 5.670374e-8;
    /** Running mean of the walls' heat input, s (the walls follow only the cycle's mean). */
    static final double WALL_MEAN = 0.02;
    /** Conserved variables per cell: density, momentum, energy, fuel, burned gas (each per volume). */
    static final int NV = 5;

    /**
     * A pulsejet: its kind; the valveless intake pipe's diameter and length (m; none for a valved engine); the reed grid's
     * open area (m^2), the pressure difference that opens its reeds fully (Pa) and the reeds' natural frequency (Hz; none
     * for a valveless engine); the combustion chamber's diameter and length, the taper's length, the tailpipe's diameter and
     * length (m); whether a valveless intake points rearward (the U-tube of Lockwood and Hiller); the chamber's mixing time
     * (s: about its length over the speed of the jets coming in); the combustion efficiency; the fuel.
     */
    public record Design(Kind kind, double intakeDiameterM, double intakeLengthM, double valveAreaM2, double valveOpeningPa, double reedFrequencyHz,
                         double chamberDiameterM, double chamberLengthM, double taperLengthM, double tailDiameterM, double tailLengthM, boolean intakeFacesRear,
                         double mixingTimeS, double combustionEfficiency, Fuel fuel) {
        public Design {
            Objects.requireNonNull(kind, "kind"); Objects.requireNonNull(fuel, "fuel");
            if (!(chamberDiameterM > 0) || !(chamberLengthM > 0) || !(taperLengthM >= 0) || !(tailDiameterM > 0) || !(tailLengthM > 0)
                    || !(mixingTimeS > 0) || !(combustionEfficiency > 0 && combustionEfficiency <= 1)) {
                throw new IllegalArgumentException("A pulsejet needs a chamber, a tailpipe, a mixing time and an efficiency in (0, 1]");
            }
            if (kind == Kind.VALVED && (!(valveAreaM2 > 0) || !(valveOpeningPa > 0) || !(reedFrequencyHz > 0))) {
                throw new IllegalArgumentException("A valved pulsejet needs a valve area, an opening pressure and a reed frequency");
            }
            if (kind == Kind.VALVELESS && (!(intakeDiameterM > 0) || !(intakeLengthM > 0))) { throw new IllegalArgumentException("A valveless pulsejet needs an intake pipe"); }
        }

        public double intakeLength() { return kind == Kind.VALVELESS ? intakeLengthM : 0; }
        public double length() { return intakeLength() + chamberLengthM + taperLengthM + tailLengthM; }

        /**
         * The fuel flow the engine is sized for, kg/s: a quarter of the chamber's volume of sea-level air each cycle at the
         * quarter-wave frequency of the whole tube with its gas at 1000 K, burned stoichiometric (the Argus As 014 so comes
         * to its 0.21 kg/s). The scale of the operating map; the engine runs leaner and richer than it.
         */
        public double nominalFuelKgPerS() {
            double frequency = Math.sqrt(GAMMA * R * 1000) / (4 * length());
            double air = 0.25 * Math.PI / 4 * chamberDiameterM * chamberDiameterM * chamberLengthM * 1.225 * frequency;
            return air / fuel.stoichiometricAirFuel();
        }

        /** Flow area at a distance from the front, m^2. */
        double area(double x) {
            double a = intakeLength(), c = a + chamberLengthM, t = c + taperLengthM;
            double d;
            if (x < a) { d = intakeDiameterM; }
            else if (x < c) { d = chamberDiameterM; }
            else if (x < t) { d = chamberDiameterM + (tailDiameterM - chamberDiameterM) * (x - c) / taperLengthM; }
            else { d = tailDiameterM; }
            return Math.PI * d * d / 4;
        }
    }

    /**
     * A settled cycle: mean thrust (N), fuel fed (kg/s), air taken in at the front (kg/s), frequency (Hz), the chamber's
     * pressure swing (half peak-to-peak, Pa), the gas's peak temperature (K), the hottest wall's temperature (K), whether
     * it runs, the chamber pressure over one cycle (64 samples, Pa), the fraction of the fuel that burned in the tube.
     */
    public record Cycle(double thrustN, double fuelKgPerS, double airflowKgPerS, double frequencyHz, double pressureAmplitudePa, double peakTemperatureK,
                        double wallTemperatureK, boolean running, double[] pressureTrace, double burnedFraction) {
        public double tsfc() { return thrustN > 0 ? fuelKgPerS / thrustN : Double.POSITIVE_INFINITY; }
    }

    /** Runs the engine at a flight condition and fuel flow on a grid of cells for a simulated time and averages its last cycles. */
    public static Cycle run(Design d, Turbojet.Flight flight, double fuelKgPerS, int cells, double seconds) {
        return new Solver(d, flight, Math.max(0, fuelKgPerS), cells).run(seconds);
    }

    /** The default resolution: 120 cells, 0.5 s (results change under 1 % from 60 to 240 cells). */
    public static Cycle run(Design d, Turbojet.Flight flight, double fuelKgPerS) { return run(d, flight, fuelKgPerS, 120, 0.5); }

    /**
     * The engine's operating map at sea level: cycles for flight speeds 0 to a top speed (6 rows) and fuel flows 0 to a
     * top flow (7 columns), each run with 80 cells for 0.4 s, built in parallel (a few seconds: in the background). Other
     * altitudes by similarity: at theta = T/T_sl and delta = p/p_sl, the cycle at speed v / sqrt(theta) and fuel
     * f sqrt(theta) / delta, its thrust and amplitude times delta, airflow times delta / sqrt(theta), frequency times
     * sqrt(theta) (exact for a perfect gas without the walls' heat and the reeds' fixed frequency).
     */
    public static final class Map {
        static final int SPEEDS = 6, FUELS = 7;
        final Design design;
        final double topSpeed, topFuel;
        final Cycle[][] cycles;

        private Map(Design design, double topSpeed, double topFuel, Cycle[][] cycles) {
            this.design = design; this.topSpeed = topSpeed; this.topFuel = topFuel; this.cycles = cycles;
        }

        public static Map build(Design d, double topSpeedMps, double topFuelKgPerS) {
            if (!(topSpeedMps > 0) || !(topFuelKgPerS > 0)) { throw new IllegalArgumentException("A map needs a top speed and a top fuel flow"); }
            Cycle[][] cycles = new Cycle[SPEEDS][FUELS];
            IntStream.range(0, SPEEDS * FUELS).parallel().forEach(k -> {
                int i = k / FUELS, j = k % FUELS;
                cycles[i][j] = run(d, Turbojet.Flight.sea(topSpeedMps * i / (SPEEDS - 1)), topFuelKgPerS * j / (FUELS - 1), 80, 0.4);
            });
            return new Map(d, topSpeedMps, topFuelKgPerS, cycles);
        }

        public Design design() { return design; }
        public double topSpeedMps() { return topSpeed; }
        public double topFuelKgPerS() { return topFuel; }
        /** The cycle computed at row i (speed), column j (fuel). */
        public Cycle cycle(int i, int j) { return cycles[i][j]; }

        /**
         * The engine at a flight condition and fuel flow, interpolated in the map (bilinear; it runs where the running
         * corners outweigh the dead ones; the pressure trace from the nearest corner), outside it at its edge.
         */
        public Cycle at(Turbojet.Flight flight, double fuelKgPerS) {
            double theta = flight.temperatureK() / 288.15, delta = flight.pressurePa() / 101325, root = Math.sqrt(theta);
            double x = Math.clamp(flight.speedMps() / root / topSpeed * (SPEEDS - 1), 0, SPEEDS - 1);
            double y = Math.clamp(Math.max(0, fuelKgPerS) * root / delta / topFuel * (FUELS - 1), 0, FUELS - 1);
            int i = Math.min((int) x, SPEEDS - 2), j = Math.min((int) y, FUELS - 2);
            double fx = x - i, fy = y - j;
            double[] w = {(1 - fx) * (1 - fy), (1 - fx) * fy, fx * (1 - fy), fx * fy};
            Cycle[] c = {cycles[i][j], cycles[i][j + 1], cycles[i + 1][j], cycles[i + 1][j + 1]};
            double thrust = 0, air = 0, freq = 0, amp = 0, peak = 0, wall = 0, burned = 0, run = 0, freqW = 0;
            for (int k = 0; k < 4; k++) {
                thrust += w[k] * c[k].thrustN(); air += w[k] * c[k].airflowKgPerS(); amp += w[k] * c[k].pressureAmplitudePa();
                peak += w[k] * c[k].peakTemperatureK(); wall += w[k] * c[k].wallTemperatureK(); burned += w[k] * c[k].burnedFraction();
                if (c[k].running()) { run += w[k]; freq += w[k] * c[k].frequencyHz(); freqW += w[k]; }
            }
            boolean running = run >= 0.5;
            Cycle nearest = c[(fx < 0.5 ? 0 : 2) + (fy < 0.5 ? 0 : 1)];
            double[] trace = nearest.pressureTrace().clone();
            double p0 = flight.pressurePa();
            for (int k = 0; k < trace.length; k++) { trace[k] = p0 + (trace[k] - 101325) * delta; }
            return new Cycle(running ? thrust * delta : Math.min(0, thrust) * delta, fuelKgPerS, air * delta / root, running && freqW > 0 ? freq / freqW * root : 0,
                    amp * delta, peak * theta, wall, running, trace, burned);
        }
    }

    /** The finite-volume solver: per cell rho, rho u, E, rho Y_fuel, rho Y_products (per unit volume). */
    static final class Solver {
        final Design d;
        final int n;
        final double dx, p0, t0, pt0, tt0, v0, fuel, afr, lhv;
        /** Simulated time (s), and the pressure above which the charge burns: the total pressure at the intake (Pa). */
        double time;
        final double lightAbove;
        /** The reeds' opening (0 shut on the seat, 1 against the stop) and its rate (1/s). */
        double reedX, reedV;
        /** Mass that has come in through the ends (kg), and the last stage's rate (kg/s): the books for checking conservation. */
        double boundaryMass, edgeRate;
        final double[] areaFace, areaCell;
        /** Wall temperature per cell (K), and running means of the inside coefficient and coefficient times gas temperature. */
        final double[] wallT, hMean, htMean;
        /** Outside convection coefficient: still air plus the flight speed along the tube (W/m^2 K). */
        final double outside;
        double[][] u;   // [NV][n] conserved per volume
        final int chamberProbe, injectFrom, injectTo;

        Solver(Design d, Turbojet.Flight flight, double fuelKgPerS, int cells) {
            this.d = d;
            n = Math.max(30, cells);
            dx = d.length() / n;
            p0 = flight.pressurePa();
            t0 = flight.temperatureK();
            v0 = flight.speedMps();
            tt0 = t0 + v0 * v0 / (2 * CP);
            pt0 = p0 * Math.pow(tt0 / t0, GAMMA / (GAMMA - 1));
            lightAbove = d.kind() == Kind.VALVELESS && d.intakeFacesRear() ? p0 : pt0;
            fuel = fuelKgPerS;
            afr = d.fuel().stoichiometricAirFuel();
            lhv = d.fuel().lowerHeatingValueJPerKg() * d.combustionEfficiency();
            areaFace = new double[n + 1];
            areaCell = new double[n];
            for (int i = 0; i <= n; i++) { areaFace[i] = d.area(Math.min(d.length() * (1 - 1e-12), i * dx)); }
            for (int i = 0; i < n; i++) { areaCell[i] = d.area((i + 0.5) * dx); }
            wallT = new double[n]; hMean = new double[n]; htMean = new double[n];
            Arrays.fill(wallT, 900);
            // turbulent flat plate along the tube (0.037 Re^0.8 Pr^(1/3)), air at ambient
            double muAir = viscosity(t0), reL = p0 / (R * t0) * v0 * d.length() / muAir;
            outside = STILL_AIR + (v0 > 0 ? 0.037 * Math.pow(reL, 0.8) * Math.pow(0.7, 1.0 / 3) * muAir * CP / 0.7 / d.length() : 0);
            double cStart = d.intakeLength(), cEnd = cStart + d.chamberLengthM();
            chamberProbe = (int) ((cStart + 0.5 * d.chamberLengthM()) / dx);
            if (d.kind() == Kind.VALVED) { injectFrom = 0; injectTo = Math.max(1, (int) (0.25 * d.chamberLengthM() / dx)); }
            else { injectFrom = (int) ((cStart + 0.3 * d.chamberLengthM()) / dx); injectTo = Math.max(injectFrom + 1, (int) ((cStart + 0.7 * d.chamberLengthM()) / dx)); }
            u = new double[NV][n];
            // the starting air has filled the chamber with stoichiometric mixture and the spark has lit its front fifth
            double yStart = 1 / (1 + afr);
            for (int i = 0; i < n; i++) {
                double x = (i + 0.5) * dx;
                boolean chamber = x >= cStart && x <= cEnd, spark = chamber && x <= cStart + 0.2 * d.chamberLengthM();
                double t = spark ? 2000 : t0, rho = p0 / (R * t);
                u[0][i] = rho; u[1][i] = 0; u[2][i] = p0 / (GAMMA - 1);
                u[3][i] = chamber && !spark ? yStart * rho : 0;
                u[4][i] = spark ? rho : 0;
            }
        }

        Cycle run(double seconds) {
            double t = 0, warm = 0.4 * seconds, tMax = 0;
            List<double[]> trace = new java.util.ArrayList<>();   // {t, dt, chamber p, thrust, front air, fuel burned}
            double[] flux = new double[3];   // thrust, air in, burned fuel this step
            while (t < seconds) {
                double dt = timeStep();
                step(dt, flux);
                t += dt;
                if (t > warm) {
                    trace.add(new double[]{t, dt, p(chamberProbe), flux[0], flux[1], flux[2]});
                    for (int i = 0; i < n; i++) { tMax = Math.max(tMax, temperature(i)); }
                }
            }
            // the chamber's mean pressure and swing, then its cycles: upward crossings of the mean after it has been a
            // third of the swing below (so ripples on the wave do not count as cycles)
            double span = 0, mean = 0, pMax = -1, pMin = Double.MAX_VALUE;
            for (double[] r : trace) { span += r[1]; mean += r[2] * r[1]; pMax = Math.max(pMax, r[2]); pMin = Math.min(pMin, r[2]); }
            mean = span > 0 ? mean / span : p0;
            double amplitude = pMax > 0 ? 0.5 * (pMax - pMin) : 0;
            List<Integer> crossings = new java.util.ArrayList<>();
            boolean armed = false;
            for (int k = 1; k < trace.size(); k++) {
                double pc = trace.get(k)[2];
                if (pc < mean - amplitude / 3) { armed = true; }
                if (armed && trace.get(k - 1)[2] < mean && pc >= mean) { crossings.add(k); armed = false; }
            }
            int from = 0, to = trace.size();
            double frequency = 0;
            if (crossings.size() >= 3) {
                from = crossings.getFirst(); to = crossings.getLast();
                frequency = (crossings.size() - 1) / (trace.get(to)[0] - trace.get(from)[0]);
            }
            // averages over the whole cycles (or the whole record where there are none)
            double whole = 0, thrust = 0, air = 0, burned = 0;
            for (int k = from; k < to; k++) {
                double[] r = trace.get(k);
                whole += r[1]; thrust += r[3] * r[1]; air += r[4] * r[1]; burned += r[5] * r[1];
            }
            if (whole > 0) { thrust /= whole; air /= whole; burned /= whole; }
            double burnedFraction = fuel > 0 ? burned / fuel : 0;
            boolean running = amplitude > 0.05 * p0 && frequency > 0 && burnedFraction > 0.1;
            double[] shape = new double[64];
            Arrays.fill(shape, mean);
            if (crossings.size() >= 2) {
                int a = crossings.get(crossings.size() - 2), b = crossings.getLast(), k = a;
                double ta = trace.get(a)[0], tb = trace.get(b)[0];
                for (int q = 0; q < 64; q++) {
                    double ts = ta + (tb - ta) * q / 64;
                    while (k < b && trace.get(k + 1)[0] < ts) { k++; }
                    double[] x = trace.get(k), y = trace.get(Math.min(k + 1, b));
                    double w = y[0] > x[0] ? Math.clamp((ts - x[0]) / (y[0] - x[0]), 0, 1) : 0;
                    shape[q] = x[2] + w * (y[2] - x[2]);
                }
            }
            double wall = 0;
            for (double w : wallT) { wall = Math.max(wall, w); }
            return new Cycle(running ? thrust : Math.min(thrust, 0), fuel, air, running ? frequency : 0, amplitude, tMax, wall, running, shape, burnedFraction);
        }

        /** Gas in the tube, kg. */
        double mass() {
            double m = 0;
            for (int i = 0; i < n; i++) { m += u[0][i] * areaCell[i] * dx; }
            return m;
        }

        double rho(int i) { return u[0][i]; }
        double vel(int i) { return u[1][i] / u[0][i]; }
        double p(int i) { return Math.max(1, (GAMMA - 1) * (u[2][i] - 0.5 * u[1][i] * u[1][i] / u[0][i])); }
        double temperature(int i) { return p(i) / (R * u[0][i]); }

        double timeStep() {
            double max = 1;
            for (int i = 0; i < n; i++) { max = Math.max(max, Math.abs(vel(i)) + Math.sqrt(GAMMA * p(i) / rho(i))); }
            return 0.6 * dx / max;
        }

        /** One step; flux[] gets this step's thrust (N), air drawn in at the front (kg/s) and fuel burned (kg/s). */
        void step(double dt, double[] flux) {
            if (d.kind() == Kind.VALVED) { moveReeds(dt); }
            double[][] start = copy(u);
            double[] f1 = new double[3], f2 = new double[3];
            double[][] k1 = rates(f1);
            double edge1 = edgeRate;
            for (int q = 0; q < NV; q++) for (int i = 0; i < n; i++) { u[q][i] = start[q][i] + dt * k1[q][i]; }
            fix();
            double[][] k2 = rates(f2);
            boundaryMass += 0.5 * dt * (edge1 + edgeRate);
            for (int q = 0; q < NV; q++) for (int i = 0; i < n; i++) { u[q][i] = 0.5 * start[q][i] + 0.5 * (u[q][i] + dt * k2[q][i]); }
            fix();
            mix(dt);
            burn(dt, flux);
            wall(dt);
            for (int j = 0; j < 2; j++) { flux[j] = 0.5 * (f1[j] + f2[j]); }
            time += dt;
        }

        private static double[][] copy(double[][] a) {
            double[][] b = new double[a.length][];
            for (int q = 0; q < a.length; q++) { b[q] = a[q].clone(); }
            return b;
        }

        /** Keeps density, pressure and the fractions physical after a stage. */
        void fix() {
            for (int i = 0; i < n; i++) {
                u[0][i] = Math.max(1e-4, u[0][i]);
                double kin = 0.5 * u[1][i] * u[1][i] / u[0][i];
                u[2][i] = Math.max(kin + 10 / (GAMMA - 1), u[2][i]);
                u[3][i] = Math.clamp(u[3][i], 0, u[0][i]);
                u[4][i] = Math.clamp(u[4][i], 0, u[0][i] - u[3][i]);
            }
        }

        /** Primitive state of cell i: {rho, u, p, yFuel, yProducts}. */
        double[] prim(int i) { return new double[]{rho(i), vel(i), p(i), u[3][i] / u[0][i], u[4][i] / u[0][i]}; }

        /** d/dt of the conserved variables (per volume), and {thrust, air in at the front} from the boundaries. */
        double[][] rates(double[] out) {
            double[][] r = new double[NV][n];
            double[][] w = new double[n][];
            for (int i = 0; i < n; i++) { w[i] = prim(i); }
            double[][] faceFlux = new double[n + 1][];
            for (int f = 1; f < n; f++) { faceFlux[f] = hllc(reconstruct(w, f - 1, +1), reconstruct(w, f, -1)); }
            faceFlux[0] = leftBoundary(w[0], out);
            faceFlux[n] = open(w[n - 1], false, true, out);
            edgeRate = faceFlux[0][0] * areaFace[0] - faceFlux[n][0] * areaFace[n];
            for (int i = 0; i < n; i++) {
                double vol = areaCell[i] * dx;
                for (int q = 0; q < NV; q++) { r[q][i] = (faceFlux[i][q] * areaFace[i] - faceFlux[i + 1][q] * areaFace[i + 1]) / vol; }
                // the walls' area change pushes on the gas (p dA), and their friction holds it back (4 tau_w / D, tau_w = f rho u |u| / 2)
                r[1][i] += w[i][2] * (areaFace[i + 1] - areaFace[i]) / vol;
                r[1][i] -= 2 * FRICTION * w[i][0] * w[i][1] * Math.abs(w[i][1]) / Math.sqrt(4 * areaCell[i] / Math.PI);
            }
            // fuel: vapour at ambient temperature, less its heat of vaporization
            if (fuel > 0) {
                double length = (injectTo - injectFrom) * dx;
                for (int i = injectFrom; i < injectTo; i++) {
                    double rate = fuel / length / areaCell[i];   // kg/(s m^3)
                    r[0][i] += rate;
                    r[3][i] += rate;
                    r[2][i] += rate * (CV * t0 - d.fuel().vaporizationHeatJPerKg());
                }
            }
            return r;
        }

        private double[] reconstruct(double[][] w, int i, int side) {
            double[] c = w[i].clone();
            if (i == 0 || i == n - 1) { return c; }
            for (int q = 0; q < NV; q++) {
                double a = w[i][q] - w[i - 1][q], b = w[i + 1][q] - w[i][q];
                double slope = a * b <= 0 ? 0 : Math.abs(a) < Math.abs(b) ? a : b;
                c[q] += 0.5 * side * slope;
            }
            c[0] = Math.max(1e-4, c[0]);
            c[2] = Math.max(1, c[2]);
            return c;
        }

        /** HLLC flux (per area) between primitive states. */
        static double[] hllc(double[] l, double[] r) {
            double rl = l[0], ul = l[1], pl = l[2], rr = r[0], ur = r[1], pr = r[2];
            double cl = Math.sqrt(GAMMA * pl / rl), cr = Math.sqrt(GAMMA * pr / rr);
            double el = pl / (GAMMA - 1) + 0.5 * rl * ul * ul, er = pr / (GAMMA - 1) + 0.5 * rr * ur * ur;
            double sl = Math.min(ul - cl, ur - cr), sr = Math.max(ul + cl, ur + cr);
            double[] fl = {rl * ul, rl * ul * ul + pl, ul * (el + pl), rl * ul * l[3], rl * ul * l[4]};
            double[] fr = {rr * ur, rr * ur * ur + pr, ur * (er + pr), rr * ur * r[3], rr * ur * r[4]};
            if (sl >= 0) { return fl; }
            if (sr <= 0) { return fr; }
            double ss = (pr - pl + rl * ul * (sl - ul) - rr * ur * (sr - ur)) / (rl * (sl - ul) - rr * (sr - ur));
            if (ss >= 0) { return star(fl, rl, ul, pl, el, l, sl, ss); }
            return star(fr, rr, ur, pr, er, r, sr, ss);
        }

        private static double[] star(double[] f, double rho, double u, double p, double e, double[] w, double s, double ss) {
            double k = rho * (s - u) / (s - ss);
            double[] us = {k, k * ss, k * (e / rho + (ss - u) * (ss + p / (rho * (s - u)))), k * w[3], k * w[4]};
            double[] uk = {rho, rho * u, e, rho * w[3], rho * w[4]};
            double[] out = new double[NV];
            for (int q = 0; q < NV; q++) { out[q] = f[q] + s * (us[q] - uk[q]); }
            return out;
        }

        /**
         * The reeds over a step: a damped spring-mass driven by the pressure across them, x'' = omega^2 (dp / dp_full - x)
         * - 2 zeta omega x', stopped by the seat (shut) and the stop plate (fully open), each impact taking the reed's speed.
         */
        void moveReeds(double dt) {
            double w = 2 * Math.PI * d.reedFrequencyHz(), dp = pt0 - p(0);
            reedV += dt * (w * w * (dp / d.valveOpeningPa() - reedX) - 2 * REED_DAMPING * w * reedV);
            reedX += dt * reedV;
            if (reedX < 0) { reedX = 0; reedV = Math.max(0, reedV); }
            if (reedX > 1) { reedX = 1; reedV = Math.min(0, reedV); }
        }

        /** Compressible flow through an orifice (effective area, m^2) from an upstream total state to a downstream pressure, choked at the critical ratio; kg/s. */
        static double orifice(double area, double pt, double tt, double pDown) {
            double ratio = Math.max(pDown / pt, Math.pow(2 / (GAMMA + 1), GAMMA / (GAMMA - 1)));
            if (ratio >= 1) { return 0; }
            return area * pt * Math.sqrt(2 * GAMMA / ((GAMMA - 1) * R * tt) * (Math.pow(ratio, 2 / GAMMA) - Math.pow(ratio, (GAMMA + 1) / GAMMA)));
        }

        /** The front: a valved engine's reeds from the intake plenum, or a valveless engine's open intake. */
        private double[] leftBoundary(double[] in, double[] out) {
            if (d.kind() == Kind.VALVELESS) { return open(in, true, d.intakeFacesRear(), out); }
            double dp = pt0 - in[2], a = areaFace[0], gap = DISCHARGE * d.valveAreaM2() * reedX;
            if (gap <= 0 || dp == 0) { return new double[]{0, in[2], 0, 0, 0}; }   // shut: a wall
            if (dp > 0) {   // air from the plenum (the free stream's total state)
                double m = orifice(gap, pt0, tt0, in[2]);
                double flux = m / a, jet = flux / Math.max(1e-4, in[0]);
                out[1] += m;
                out[0] -= m * v0;   // ram drag of the air taken from the free stream
                return new double[]{flux, flux * jet + in[2], flux * CP * tt0, 0, 0};
            }
            // the reeds have not shut yet: hot gas blows out through them, its jet pushing the engine back
            double tIn = in[2] / (R * in[0]);
            double m = orifice(gap, in[2], tIn, pt0);
            double flux = -m / a, vel = flux / in[0];
            out[0] -= m * m / (in[0] * gap);
            return new double[]{flux, flux * vel + in[2], flux * (CP * tIn + 0.5 * vel * vel), flux * in[3], flux * in[4]};
        }

        /**
         * An open end: the flux through it (per area, in +x), and its thrust contribution. Out: pressure released to
         * ambient (subsonic) or the inside state (supersonic). In: drawn from the surroundings' total state (the ram's for an
         * end facing forward) to the inside velocity, less a sharp edge's entry loss.
         */
        private double[] open(double[] in, boolean left, boolean facesRear, double[] out) {
            double velOut = left ? -in[1] : in[1];   // positive leaving the tube
            double[] ghost;
            if (velOut >= 0) {
                double c = Math.sqrt(GAMMA * in[2] / in[0]);
                ghost = velOut >= c ? in.clone() : new double[]{in[0], in[1], p0, in[3], in[4]};
            } else {
                boolean ram = left && !facesRear;
                double tt = ram ? tt0 : t0, pt = ram ? pt0 : p0;
                double v = -velOut, tStatic = Math.max(0.3 * tt, tt - v * v / (2 * CP)), pStatic = pt * Math.pow(tStatic / tt, GAMMA / (GAMMA - 1));
                pStatic = Math.max(0.1 * pt, pStatic - ENTRY_LOSS * 0.5 * pStatic / (R * tStatic) * v * v);
                ghost = new double[]{pStatic / (R * tStatic), in[1], pStatic, 0, 0};
            }
            double[] f = left ? hllc(ghost, in) : hllc(in, ghost);
            double a = left ? areaFace[0] : areaFace[n];
            // force on the engine along its forward axis: (rho u^2 + p - p0) A out of the end, forward for an end facing rearward
            double push = (f[1] - p0) * a;
            out[0] += left ? (facesRear ? push : -push) : push;
            double massOut = (left ? -f[0] : f[0]) * a;
            if (massOut < 0) {   // ambient air drawn in: its ram drag; the front's is the engine's breathing air
                out[0] -= -massOut * v0;
                if (left) { out[1] += -massOut; }
            }
            return f;
        }

        /**
         * Turbulent mixing as an exchange of equal volumes between neighbouring cells: every conserved quantity diffuses per
         * volume, implicitly (one tridiagonal matrix for all). In the chamber and taper D = L^2 / (pi^2 tau) (the slowest
         * mode of the chamber's length decays over the mixing time); elsewhere D = MIXING (|u| + 10 m/s) diameter.
         */
        void mix(double dt) {
            double cStart = d.intakeLength(), tEnd = cStart + d.chamberLengthM() + d.taperLengthM();
            double stirred = d.chamberLengthM() * d.chamberLengthM() / (Math.PI * Math.PI * d.mixingTimeS());
            double[] k = new double[n + 1];   // face conductance, m^3/s (none through the ends)
            for (int f = 1; f < n; f++) {
                double x = f * dx, dia = Math.sqrt(4 * areaFace[f] / Math.PI);
                double diff = MIXING * (0.5 * (Math.abs(vel(f - 1)) + Math.abs(vel(f))) + 10) * dia;
                if (x > cStart && x < tEnd) { diff = Math.max(diff, stirred); }
                k[f] = diff * areaFace[f] / dx;
            }
            // V_i (U_i' - U_i) = dt [k_i (U_{i-1}' - U_i') + k_{i+1} (U_{i+1}' - U_i')]
            double[] a = new double[n], c = new double[n], cp = new double[n], den = new double[n];
            for (int i = 0; i < n; i++) {
                double v = areaCell[i] * dx;
                a[i] = -dt * k[i] / v;
                c[i] = -dt * k[i + 1] / v;
                double b = 1 - a[i] - c[i];
                den[i] = i == 0 ? b : b - a[i] * cp[i - 1];
                cp[i] = c[i] / den[i];
            }
            for (int q = 0; q < NV; q++) {
                double[] x = u[q];
                x[0] /= den[0];
                for (int i = 1; i < n; i++) { x[i] = (x[i] - a[i] * x[i - 1]) / den[i]; }
                for (int i = n - 2; i >= 0; i--) { x[i] -= cp[i] * x[i + 1]; }
            }
            fix();
        }

        /** Sutherland's law for air, Pa s. */
        static double viscosity(double t) { return 1.716e-5 * Math.pow(t / 273.15, 1.5) * (273.15 + 110.4) / (t + 110.4); }

        /**
         * Heat to (or from) the walls over a step, exact for the step's coefficient; then each wall's temperature where its
         * mean heat from the gas equals what it radiates and convects outside.
         */
        void wall(double dt) {
            double w = Math.min(1, dt / WALL_MEAN);
            for (int i = 0; i < n; i++) {
                double dia = Math.sqrt(4 * areaCell[i] / Math.PI), t = temperature(i), rho = rho(i), mu = viscosity(t);
                double re = rho * (Math.abs(vel(i)) + 10) * dia / mu;
                double h = HEAT_ENHANCEMENT * 0.023 * Math.pow(re, 0.8) * Math.pow(0.7, 0.3) * mu * CP / 0.7 / dia;
                double tNew = wallT[i] + (t - wallT[i]) * Math.exp(-h * 4 / dia * dt / (rho * CV));
                u[2][i] -= rho * CV * (t - tNew);
                hMean[i] += (h - hMean[i]) * w;
                htMean[i] += (h * t - htMean[i]) * w;
                double tw = wallT[i];
                for (int k = 0; k < 3; k++) {
                    double f = htMean[i] - hMean[i] * tw - EMISSIVITY * SIGMA * (Math.pow(tw, 4) - Math.pow(t0, 4)) - outside * (tw - t0);
                    double df = -hMean[i] - 4 * EMISSIVITY * SIGMA * Math.pow(tw, 3) - outside;
                    tw = Math.clamp(tw - f / df, t0, 3000);
                }
                wallT[i] = tw;
            }
        }

        /**
         * Burns fuel in each cell over a step (implicit in the rate), adding its heat; flux[2] gets the fuel burned (kg/s).
         * Eddy break-up where the gas is above the intake's total pressure (or the spark is still on).
         */
        void burn(double dt, double[] flux) {
            double burned = 0, rate = EBU_A / d.mixingTimeS();
            for (int i = 0; i < n; i++) {
                double rho = u[0][i], yf = u[3][i] / rho, yp = u[4][i] / rho, ya = Math.max(0, 1 - yf - yp);
                double can = Math.min(yf, Math.min(ya / afr, EBU_B * yp / (1 + afr)));
                if (can <= 0) { continue; }
                double gate = time < SPARK_S ? 1 : Math.clamp((p(i) - lightAbove) / (0.02 * lightAbove), 0, 1);
                if (gate <= 0) { continue; }
                double m = gate * rho * can * (1 - Math.exp(-dt * rate));   // kg/m^3 burned
                u[3][i] -= m;
                u[4][i] += m * (1 + afr);
                u[2][i] += m * lhv;
                burned += m * areaCell[i] * dx;
            }
            flux[2] = burned / dt;
        }
    }
}
