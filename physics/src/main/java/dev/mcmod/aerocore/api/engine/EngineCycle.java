package dev.mcmod.aerocore.api.engine;

import java.util.Objects;

/**
 * Crank-angle resolved, single-zone simulation of one cylinder of a four-stroke spark-ignition engine (Heywood 1988,
 * chapter 14):
 * <ul>
 * <li>Mass and energy balance of the cylinder: dU = h_in dm_in - h_out dm_out - p dV + dQ_burn - dQ_wall, ideal gas;
 *   the charge is fresh mixture (air and fuel, both with air's heat capacity) blended with burned gas (Gas).</li>
 * <li>Valves are compressible orifices (ValveFlow). On the intake side the cylinder draws from a port node whose
 *   volume is the runner's, fed from a plenum at fixed pressure and temperature through the runner's gas column
 *   (lumped inertia L/A with a quadratic loss): the Helmholtz-resonator picture of intake tuning (Heywood section
 *   7.6), which is what lets a tuned runner ram charge in after bottom centre. Exhaust goes to a plenum at a fixed
 *   back pressure. Each cylinder sees its own plenum; no interaction between cylinders, no wave dynamics beyond the
 *   lumped runner.</li>
 * <li>Combustion from spark: Wiebe burn rate (Combustion) scaled by flame travel and laminar burning velocity; heat
 *   release from the fuel's chemistry at the mixture's lambda times a combustion efficiency (crevice losses).</li>
 * <li>Woschni heat transfer to one wall temperature; Livengood-Wu knock integral on the end gas.</li>
 * <li>Port injection and carburettors evaporate the fuel before the valve (charge cooled by h_v / (AFR + 1) / cp);
 *   direct injection adds the fuel at intake valve closing and takes its heat of vaporization from the charge.</li>
 * </ul>
 * Whole cycles are run from the end of expansion until the trapped charge repeats (three by default) and the last is
 * reported. Explicit second-order (Heun) steps of 0.25 crank degrees. Crank angle 0 is firing TDC; a cycle spans
 * gas-exchange TDC (-360) to +360 degrees.
 */
public final class EngineCycle {
    private EngineCycle() {}

    /**
     * Calibration of the knock integral: the end gas autoignites when the integral of dt / (KNOCK_CALIBRATION tau)
     * reaches 1, tau from Douaud and Eyzat. With 1.45 a 79 mm, compression-ratio-11 four-valve engine on 95 RON at
     * 1500-1900 rpm and 6-7 bar IMEP starts to knock about 3.5 degrees past best-torque spark, as Bonatesta (2013,
     * section 4.3) reports for his engine ("over-advance limited to 3-4 CA degrees to avoid the inception of knock").
     * Recalibrating the pre-exponential factor this way is usual practice with this correlation; one engine's data.
     */
    public static final double KNOCK_CALIBRATION = 1.45;
    /** Polytropic exponent for Woschni's motored pressure (Heywood: about 1.3 for compression). */
    private static final double MOTORED_EXPONENT = 1.32;
    static final double STEP_DEG = 0.25;
    /** Darcy friction factor and inlet loss of a runner (smooth pipe at high Reynolds number; bell-mouth entry), a modelling choice. */
    static final double RUNNER_FRICTION = 0.02, RUNNER_ENTRY_LOSS = 0.1;

    /**
     * One cylinder: geometry, valves with their lift laws, intake runner (length 0 = the plenum is at the port),
     * mixture and walls.
     */
    public record Spec(CrankKinematics crank,
                       int intakeValves, double intakeValveM, double intakeThroatM, double intakeStemM, double intakeCd, ValveLift intakeLift,
                       int exhaustValves, double exhaustValveM, double exhaustThroatM, double exhaustStemM, double exhaustCd, ValveLift exhaustLift,
                       double intakeRunnerLengthM, double intakeRunnerDiameterM,
                       Fuel fuel, double lambda, double combustionEfficiency, boolean directInjection,
                       double flameTravelM, double wallTemperatureK, double headAreaFactor, double pistonAreaFactor) {
        public Spec {
            Objects.requireNonNull(crank, "crank"); Objects.requireNonNull(intakeLift, "intakeLift"); Objects.requireNonNull(exhaustLift, "exhaustLift");
            Objects.requireNonNull(fuel, "fuel");
            if (intakeValves < 1 || exhaustValves < 1) { throw new IllegalArgumentException("Need at least one intake and one exhaust valve"); }
            if (!(lambda > 0.3 && lambda < 3)) { throw new IllegalArgumentException("Lambda must be in (0.3, 3)"); }
            if (!(combustionEfficiency > 0 && combustionEfficiency <= 1)) { throw new IllegalArgumentException("Combustion efficiency must be in (0, 1]"); }
            if (!(flameTravelM > 0) || !(wallTemperatureK > 0)) { throw new IllegalArgumentException("Flame travel and wall temperature must be positive"); }
            if (!(intakeRunnerLengthM >= 0) || intakeRunnerLengthM > 0 && !(intakeRunnerDiameterM > 0)) { throw new IllegalArgumentException("Runner needs a positive diameter"); }
        }
        boolean runner() { return intakeRunnerLengthM > 0; }
    }

    /** Operating point: speed, plenum states and spark advance (degrees before firing TDC). */
    public record Conditions(double rpm, double intakePressurePa, double intakeTemperatureK, double exhaustPressurePa, double sparkAdvanceDeg) {
        public Conditions {
            if (!(rpm > 0) || !(intakePressurePa > 0) || !(intakeTemperatureK > 0) || !(exhaustPressurePa > 0)) { throw new IllegalArgumentException("Conditions must be positive"); }
        }
    }

    /**
     * One cycle of one cylinder. Mean effective pressures in Pa (net = whole cycle, gross = compression and
     * expansion, pumping = gross - net); masses per cycle in kg; heat in J per cycle; angles in degrees after firing
     * TDC. knockMargin is the end-gas knock integral reached when 90 % had burned (knock when >= 1).
     */
    public record Result(double netImep, double grossImep, double pumpingMep, double inductedAirKg, double trappedAirKg, double fuelKg,
                         double residualFraction, double volumetricEfficiency, double peakPressurePa, double peakPressureDeg,
                         double knockMargin, double knockOnsetDeg, double heatToWallsJ, double exhaustTemperatureK,
                         double burnAngle90, double burnExponent, double fuelEnergyJ, double meanPistonSpeed) {
        public boolean knocks() { return knockMargin >= 1; }
        /** Net indicated fuel conversion efficiency for the cylinder's displacement. */
        public double indicatedEfficiency(double displacementM3) { return fuelEnergyJ > 0 ? netImep * displacementM3 / fuelEnergyJ : 0; }
    }

    /** State carried between runs (warm start for another spark advance at the same operating point). */
    public record Warm(double[] state, double exhaustTemperatureK) {
        public Warm { state = state.clone(); }
    }
    /** A result with the state at its end. */
    public record Run(Result result, Warm warm) {}

    public static Result run(Spec spec, Conditions c) { return run(spec, c, null, 3).result(); }

    public static Run run(Spec spec, Conditions c, Warm start, int cycles) {
        if (cycles < 1) { throw new IllegalArgumentException("Need at least one cycle"); }
        return new Simulation(spec, c).run(start, cycles);
    }

    // State vector indices: cylinder mass, energy, burned mass; port mass, energy, burned mass; runner mass flow.
    static final int M = 0, U = 1, MB = 2, PM = 3, PU = 4, PB = 5, RF = 6, N = 7;

    static final class Simulation {
        final Spec s; final Conditions c; final CrankKinematics k;
        final Gas fresh = Gas.AIR, burned;
        final double omega, afr, heatRelease, rAir, chargeTemperature, chargeEnergy, sp, vd, flameScale;
        final double runnerArea, runnerVolume, runnerInertia, runnerLoss;
        double tGuess = 800, tPortGuess = 300, exhaustT = 900;
        // per-cycle combustion parameters (set at IVC and spark)
        boolean sparked; double sparkAngle, burnAngle, exponent, freshAtSpark, fuel, pIvc, vIvc, tIvc;
        /** Test hooks: valves held shut, no wall heat transfer. */
        boolean valvesShut, adiabatic;
        /** Length of the current step, s (for the flow limiter). */
        double stepSeconds;

        Simulation(Spec s, Conditions c) {
            this.s = s; this.c = c; this.k = s.crank();
            burned = s.fuel().products(s.lambda());
            omega = c.rpm() * Math.PI / 30;
            afr = s.lambda() * s.fuel().stoichiometricAirFuel();
            heatRelease = s.fuel().heatRelease(s.lambda()) * s.combustionEfficiency();
            rAir = Gas.AIR.gasConstant();
            vd = k.displacement();
            sp = 2 * k.strokeM() * c.rpm() / 60;
            double cooling = s.directInjection() ? 0 : s.fuel().vaporizationHeatJPerKg() / (afr + 1) / Gas.AIR.cp(c.intakeTemperatureK());
            chargeTemperature = c.intakeTemperatureK() - cooling;
            chargeEnergy = fresh.energy(chargeTemperature);
            flameScale = s.flameTravelM() / k.strokeM() / Combustion.REFERENCE_TRAVEL_OVER_STROKE;
            if (s.runner()) {
                runnerArea = Math.PI * s.intakeRunnerDiameterM() * s.intakeRunnerDiameterM() / 4;
                runnerVolume = runnerArea * s.intakeRunnerLengthM();
                runnerInertia = s.intakeRunnerLengthM() / runnerArea;
                runnerLoss = (RUNNER_FRICTION * s.intakeRunnerLengthM() / s.intakeRunnerDiameterM() + RUNNER_ENTRY_LOSS) / (2 * runnerArea * runnerArea);
            } else { runnerArea = 0; runnerVolume = 0; runnerInertia = 0; runnerLoss = 0; }
        }

        double gasConstant(double y) { return (1 - y) * rAir + y * burned.gasConstant(); }
        double gamma(double y, double t) {
            double cv = (1 - y) * fresh.cv(t) + y * burned.cv(t);
            return (cv + gasConstant(y)) / cv;
        }
        /**
         * Flux limiter for an orifice between two gas volumes in an explicit step: at most half the mass that would
         * bring their pressures level (adiabatic capacities V / (gamma R T); a plenum's is infinite). Near equal
         * pressures the orifice's conductance grows without bound and a plain explicit step would overshoot and
         * oscillate, the more so the thinner the gas; the limit only acts where a step would do that.
         */
        double limit(double flow, double dp, double capacityUp, double capacityDown) {
            double inverse = (Double.isInfinite(capacityUp) ? 0 : 1 / capacityUp) + (Double.isInfinite(capacityDown) ? 0 : 1 / capacityDown);
            if (inverse == 0 || stepSeconds <= 0) { return flow; }
            return Math.min(flow, 0.5 * Math.abs(dp) / inverse / stepSeconds);
        }

        double cylinderTemperature(double[] x) {
            double mass = Math.max(x[M], 1e-12), y = Math.clamp(x[MB] / mass, 0, 1);
            return tGuess = Gas.blendTemperature(fresh, burned, y, x[U] / mass, tGuess);
        }
        double portTemperature(double[] x) {
            double mass = Math.max(x[PM], 1e-12), y = Math.clamp(x[PB] / mass, 0, 1);
            return tPortGuess = Gas.blendTemperature(fresh, burned, y, x[PU] / mass, tPortGuess);
        }

        Run run(Warm start, int cycles) {
            double theta0 = Math.toRadians(180);   // end of expansion
            double[] x = new double[N];
            if (start != null) { x = start.state().clone(); exhaustT = start.exhaustTemperatureK(); }
            else {
                // Burned gas at the exhaust pressure and 1000 K in the cylinder at BDC; the port full of fresh charge at rest.
                double v = k.volume(theta0), t = 1000;
                x[M] = c.exhaustPressurePa() * v / (burned.gasConstant() * t); x[MB] = x[M]; x[U] = x[M] * burned.energy(t); tGuess = t;
                if (s.runner()) { x[PM] = c.intakePressurePa() * runnerVolume / (rAir * chargeTemperature); x[PU] = x[PM] * chargeEnergy; tPortGuess = chargeTemperature; }
            }
            Result last = null;
            for (int i = 0; i < cycles; i++) { last = cycle(x, theta0 + i * 4 * Math.PI); }
            return new Run(last, new Warm(x, exhaustT));
        }

        /** One cycle from theta0 (BDC after expansion) to theta0 + 720 degrees, advancing x in place. */
        Result cycle(double[] x, double theta0) {
            double dtheta = Math.toRadians(STEP_DEG);
            stepSeconds = dtheta / omega;
            int steps = (int) Math.round(720 / STEP_DEG);
            double workNet = 0, workGross = 0, heatWalls = 0, peakP = 0, peakDeg = 0;
            double inducted = 0, inductedBurned = 0, exhaustMass = 0, exhaustEnthalpy = 0;
            double tdc = theta0 + 3 * Math.PI;   // firing TDC of this cycle
            double ivcAngle = s.intakeLift().centreRad() + s.intakeLift().durationRad() / 2;
            ivcAngle -= 4 * Math.PI * Math.floor((ivcAngle + 2 * Math.PI) / (4 * Math.PI));   // as an angle in [-360, 360) degrees
            sparkAngle = -Math.toRadians(c.sparkAdvanceDeg());
            sparked = false; burnAngle = 0; exponent = 0; freshAtSpark = 0; fuel = 0; pIvc = 0;
            boolean closed = false;
            double trappedMass = 0, trappedBurned = 0;
            double knock = 0, knockAt90 = 0, knockOnset = Double.NaN, pSpark = 0, tSpark = 0, gammaU = 1.35;
            boolean past90 = false;
            double[] x1 = new double[N], d1 = new double[N + 7], d2 = new double[N + 7];
            for (int i = 0; i < steps; i++) {
                double theta = theta0 + i * dtheta, rel = theta - tdc;
                if (!closed && rel >= ivcAngle - 1e-9 && rel < 0) {
                    closed = true;
                    trappedMass = x[M]; trappedBurned = x[MB];
                    double air = (x[M] - x[MB]) * (s.directInjection() ? 1 : afr / (afr + 1));
                    fuel = air / afr;
                    if (s.directInjection()) { x[M] += fuel; x[U] -= fuel * s.fuel().vaporizationHeatJPerKg(); }
                    double t = cylinderTemperature(x);
                    vIvc = k.volume(theta); pIvc = x[M] * gasConstant(x[MB] / x[M]) * t / vIvc; tIvc = t;
                }
                if (closed && !sparked && rel >= sparkAngle - 1e-9) {
                    sparked = true;
                    double v = k.volume(theta), t = cylinderTemperature(x), y = x[MB] / x[M];
                    freshAtSpark = x[M] - x[MB];
                    pSpark = x[M] * gasConstant(y) * t / v; tSpark = t;
                    gammaU = gamma(0, t);
                    double laminar = s.fuel().flameSpeed().at(1 / s.lambda(), t, pSpark, 0);
                    double reference = Fuel.GASOLINE_FLAME.at(1, t, pSpark, 0);
                    burnAngle = Combustion.burnAngle90(x[M] / v, sp, y, c.sparkAdvanceDeg()) * flameScale * Math.pow(reference / laminar, 0.85);
                    exponent = Combustion.wiebeExponent(sp, y, c.sparkAdvanceDeg());
                }
                derivatives(x, theta, rel, d1);
                for (int j = 0; j < N; j++) { x1[j] = x[j] + d1[j] * dtheta; }
                x1[MB] = Math.clamp(x1[MB], 0, Math.max(x1[M], 0));
                derivatives(x1, theta + dtheta, rel + dtheta, d2);
                for (int j = 0; j < N; j++) { x[j] += 0.5 * (d1[j] + d2[j]) * dtheta; }
                x[M] = Math.max(x[M], 1e-12);
                x[MB] = Math.clamp(x[MB], 0, x[M]);
                if (s.runner()) { x[PM] = Math.max(x[PM], 1e-12); x[PB] = Math.clamp(x[PB], 0, x[PM]); }
                // bookkeeping with the averaged rates (extras after the state: pressure, wall heat, inducted, inducted burned, exhaust out, exhaust enthalpy, temperature)
                double p = 0.5 * (d1[N] + d2[N]), dV = k.volume(theta + dtheta) - k.volume(theta);
                workNet += p * dV;
                if (rel >= -Math.PI && rel < Math.PI) { workGross += p * dV; }
                heatWalls += 0.5 * (d1[N + 1] + d2[N + 1]) * dtheta;
                inducted += 0.5 * (d1[N + 2] + d2[N + 2]) * dtheta;
                inductedBurned += 0.5 * (d1[N + 3] + d2[N + 3]) * dtheta;
                exhaustMass += 0.5 * (d1[N + 4] + d2[N + 4]) * dtheta;
                exhaustEnthalpy += 0.5 * (d1[N + 5] + d2[N + 5]) * dtheta;
                if (p > peakP) { peakP = p; peakDeg = Math.toDegrees(rel); }
                if (closed && rel < Math.PI && !past90) {
                    double tEnd = sparked ? tSpark * Math.pow(p / pSpark, (gammaU - 1) / gammaU) : 0.5 * (d1[N + 6] + d2[N + 6]);
                    knock += dtheta / omega / (KNOCK_CALIBRATION * Combustion.ignitionDelay(s.fuel().researchOctane(), p, tEnd));
                    if (sparked) {
                        if (Double.isNaN(knockOnset) && knock >= 1) { knockOnset = Math.toDegrees(rel); }
                        if (Combustion.burned(Math.toDegrees(rel + dtheta - sparkAngle), burnAngle, exponent) >= 0.9) { past90 = true; knockAt90 = knock; }
                    }
                }
            }
            if (!past90) { knockAt90 = knock; }
            if (exhaustMass > 0) {
                double h = exhaustEnthalpy / exhaustMass, t = exhaustT;   // solve u_s(T) + R T = h for the burned gas
                for (int i = 0; i < 20; i++) {
                    double step = (burned.energy(t) + burned.gasConstant() * t - h) / burned.cp(t);
                    t -= step;
                    if (Math.abs(step) < 1e-6) { break; }
                }
                exhaustT = Math.clamp(t, 300, 2500);
            }
            double airFraction = s.directInjection() ? 1 : afr / (afr + 1);
            double inductedAir = (inducted - inductedBurned) * airFraction, trappedAir = (trappedMass - trappedBurned) * airFraction;
            double airDensity = c.intakePressurePa() / (rAir * c.intakeTemperatureK());
            return new Result(workNet / vd, workGross / vd, (workGross - workNet) / vd, inductedAir, trappedAir, fuel,
                    trappedMass > 0 ? trappedBurned / trappedMass : 0, inductedAir / (airDensity * vd), peakP, peakDeg, knockAt90, knockOnset,
                    heatWalls, exhaustT, burnAngle, exponent, fuel * heatRelease, sp);
        }

        /**
         * Test hook: integrates the cylinder alone between two crank angles (radians from firing TDC) with the valves
         * shut, starting from a mass at a temperature with a burned fraction, burning fuelKg from sparkRad over a
         * given Wiebe burn. Returns {work (J), final temperature, final pressure, peak pressure}.
         */
        double[] closed(double fromRad, double toRad, double mass, double temperatureK, double burnedFraction, double fuelKg,
                        double sparkRad, double burnAngleDeg, double wiebeExponent, boolean noWallHeat) {
            valvesShut = true; adiabatic = noWallHeat;
            double[] x = new double[N];
            x[M] = mass; x[MB] = mass * burnedFraction;
            tGuess = temperatureK;
            x[U] = mass * Gas.blendEnergy(fresh, burned, burnedFraction, temperatureK);
            sparked = true; sparkAngle = sparkRad; burnAngle = burnAngleDeg; exponent = wiebeExponent;
            freshAtSpark = mass * (1 - burnedFraction); fuel = fuelKg;
            vIvc = k.volume(fromRad); pIvc = mass * gasConstant(burnedFraction) * temperatureK / vIvc; tIvc = temperatureK;
            double dtheta = Math.toRadians(STEP_DEG) / 4, work = 0, peak = 0;
            int steps = (int) Math.round((toRad - fromRad) / dtheta);
            dtheta = (toRad - fromRad) / steps;
            stepSeconds = dtheta / omega;
            double[] x1 = new double[N], d1 = new double[N + 7], d2 = new double[N + 7];
            for (int i = 0; i < steps; i++) {
                double theta = fromRad + i * dtheta;
                derivatives(x, theta, theta, d1);
                for (int j = 0; j < N; j++) { x1[j] = x[j] + d1[j] * dtheta; }
                derivatives(x1, theta + dtheta, theta + dtheta, d2);
                for (int j = 0; j < N; j++) { x[j] += 0.5 * (d1[j] + d2[j]) * dtheta; }
                double p = 0.5 * (d1[N] + d2[N]);
                work += p * (k.volume(theta + dtheta) - k.volume(theta));
                peak = Math.max(peak, p);
            }
            double t = cylinderTemperature(x);
            return new double[]{work, t, x[M] * gasConstant(x[MB] / x[M]) * t / k.volume(toRad), peak};
        }

        /**
         * Rates per crank radian of the state into d[0..N-1], and extras: d[N] pressure, d[N+1] wall heat rate,
         * d[N+2] inducted mass rate, d[N+3] inducted burned, d[N+4] exhaust outflow, d[N+5] exhaust enthalpy,
         * d[N+6] temperature.
         */
        void derivatives(double[] x, double theta, double rel, double[] d) {
            java.util.Arrays.fill(d, 0);
            double mass = Math.max(x[M], 1e-12), y = Math.clamp(x[MB] / mass, 0, 1);
            double v = k.volume(theta), dv = k.volumeRate(theta);
            double t = cylinderTemperature(x), r = gasConstant(y), p = mass * r * t / v, g = gamma(y, t);
            d[N] = p; d[N + 6] = t;
            double cylinderCapacity = v / (g * r * t);
            double dm = 0, du = 0, dmb = 0;   // per second
            // the intake side: port node (runner) or plenum
            double pIn, tIn, yIn, uIn, portCapacity = Double.POSITIVE_INFINITY;
            if (s.runner()) {
                double pm = Math.max(x[PM], 1e-12);
                yIn = Math.clamp(x[PB] / pm, 0, 1); tIn = portTemperature(x); uIn = x[PU] / pm;
                pIn = pm * gasConstant(yIn) * tIn / runnerVolume;
                portCapacity = runnerVolume / (gamma(yIn, tIn) * gasConstant(yIn) * tIn);
                // runner gas column: plenum -> port, d(mdot)/dt = (A/L)(p_plenum - p_port - loss)
                double flow = x[RF], rho = flow >= 0 ? c.intakePressurePa() / (rAir * chargeTemperature) : pm / runnerVolume;
                double accel = (c.intakePressurePa() - pIn - runnerLoss * flow * Math.abs(flow) / rho) / runnerInertia;
                // A lumped column flowing back to the plenum could empty the port in a step when strong backflow at low
                // manifold pressure has set it moving (where a real runner would carry waves instead): it takes at most
                // half the port's gas per step, and its momentum is held back to that.
                double floor = stepSeconds > 0 ? -0.5 * pm / stepSeconds : Double.NEGATIVE_INFINITY;
                if (flow < floor) { accel = Math.max(accel, (floor - flow) / Math.max(stepSeconds, 1e-9)); flow = floor; }
                d[RF] = accel / omega;
                double hIn = flow >= 0 ? chargeEnergy + rAir * chargeTemperature : uIn + gasConstant(yIn) * tIn;
                d[PM] += flow / omega; d[PU] += flow * hIn / omega; d[PB] += (flow >= 0 ? 0 : flow * yIn) / omega;
            } else { pIn = c.intakePressurePa(); tIn = chargeTemperature; yIn = 0; uIn = chargeEnergy; }
            double liftI = valvesShut ? 0 : s.intakeLift().lift(theta);
            if (liftI > 0) {
                double cdA = s.intakeCd() * ValveFlow.area(s.intakeValves(), s.intakeValveM(), liftI, s.intakeThroatM(), s.intakeStemM());
                if (pIn > p) {
                    double rin = gasConstant(yIn), h = uIn + rin * tIn;
                    double flow = limit(ValveFlow.massFlow(cdA, pIn, tIn, p, rin, gamma(yIn, tIn)), pIn - p, portCapacity, cylinderCapacity);
                    dm += flow; du += flow * h; dmb += flow * yIn;
                    d[N + 2] += flow; d[N + 3] += flow * yIn;
                    if (s.runner()) { d[PM] -= flow / omega; d[PU] -= flow * h / omega; d[PB] -= flow * yIn / omega; }
                } else {
                    double flow = limit(ValveFlow.massFlow(cdA, p, t, pIn, r, g), p - pIn, cylinderCapacity, portCapacity), h = x[U] / mass + r * t;
                    dm -= flow; du -= flow * h; dmb -= flow * y;
                    d[N + 2] -= flow; d[N + 3] -= flow * y;
                    if (s.runner()) { d[PM] += flow / omega; d[PU] += flow * h / omega; d[PB] += flow * y / omega; }
                }
            }
            // exhaust valves to the exhaust plenum
            double liftE = valvesShut ? 0 : s.exhaustLift().lift(theta);
            if (liftE > 0) {
                double cdA = s.exhaustCd() * ValveFlow.area(s.exhaustValves(), s.exhaustValveM(), liftE, s.exhaustThroatM(), s.exhaustStemM());
                if (p > c.exhaustPressurePa()) {
                    double flow = limit(ValveFlow.massFlow(cdA, p, t, c.exhaustPressurePa(), r, g), p - c.exhaustPressurePa(), cylinderCapacity, Double.POSITIVE_INFINITY), h = x[U] / mass + r * t;
                    dm -= flow; du -= flow * h; dmb -= flow * y;
                    d[N + 4] += flow; d[N + 5] += flow * h;
                } else {
                    double te = exhaustT, re = burned.gasConstant();
                    double flow = limit(ValveFlow.massFlow(cdA, c.exhaustPressurePa(), te, p, re, gamma(1, te)), c.exhaustPressurePa() - p, Double.POSITIVE_INFINITY, cylinderCapacity);
                    double h = burned.energy(te) + re * te;
                    dm += flow; du += flow * h; dmb += flow;
                    d[N + 4] -= flow; d[N + 5] -= flow * h;
                }
            }
            // combustion: analytic Wiebe rate per radian
            double burnRate = 0;
            if (sparked && rel >= sparkAngle && rel < Math.PI) {
                double phi = Math.toDegrees(rel - sparkAngle) / burnAngle;
                if (phi > 0) {
                    burnRate = Combustion.WIEBE_A * exponent * Math.pow(phi, exponent - 1) / burnAngle
                            * Math.exp(-Combustion.WIEBE_A * Math.pow(phi, exponent)) * (180 / Math.PI);
                }
            }
            // walls (Woschni)
            double area = k.pistonArea() * (s.headAreaFactor() + s.pistonAreaFactor()) + k.exposedLiner(theta), w;
            if (liftI > 0 || liftE > 0) { w = 6.18 * sp; }
            else {
                w = 2.28 * sp;
                if (sparked && rel >= sparkAngle && pIvc > 0) {
                    double pMot = pIvc * Math.pow(vIvc / v, MOTORED_EXPONENT);
                    w += 3.24e-3 * vd * tIvc / (pIvc * vIvc) * Math.max(0, p - pMot);
                }
            }
            double heat = adiabatic ? 0 : Combustion.woschni(k.boreM(), p, t, w) * area * (t - s.wallTemperatureK());   // W
            d[M] = dm / omega;
            d[U] = du / omega - p * dv + burnRate * fuel * heatRelease - heat / omega;
            d[MB] = dmb / omega + burnRate * freshAtSpark;
            d[N + 1] = heat / omega;
            for (int i = N + 2; i <= N + 5; i++) { d[i] /= omega; }
        }
    }
}
