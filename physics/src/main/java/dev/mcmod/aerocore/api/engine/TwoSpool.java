package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.engine.GasPath.Mix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static dev.mcmod.aerocore.api.engine.GasPath.corrected;
import static dev.mcmod.aerocore.api.engine.GasPath.nozzle;
import static dev.mcmod.aerocore.api.engine.GasPath.retained;

/**
 * The gas path of a two-spool gas turbine, shared by Turbofan and FreeTurbine: an engine inlet, on the low-pressure
 * spool a fan (splitting the air into bypass and core) and booster stages, on the high-pressure spool a compressor,
 * the burner and a turbine, then the low-pressure turbine, which drives the fan or (a free power turbine) a load at
 * its own speed. The bypass air leaves through its own nozzle or joins the core stream in a mixer; the core leaves
 * through the core (or mixed) nozzle, or a free turbine's exhaust.
 * <p>
 * As in Turbojet: constant polytropic efficiencies and no component maps. A compressor's work per kilogram goes with
 * its spool speed squared (Euler's equation at constant velocity triangles), so a speed fixes its pressure ratio; each
 * turbine passes flow as a nozzle across its pressure ratio; duct losses go with the square of the corrected flow.
 * Turbine cooling air leaves the compressor, passes burner and high-pressure turbine and rejoins before the
 * low-pressure turbine (where it does work). A free power turbine's efficiency falls off its best blade-to-spouting
 * speed ratio (Turbomachinery.turbineSpeedEfficiency).
 * <p>
 * Matching: for given spool speeds and a turbine inlet temperature (or fuel flow), the turbines' pressure ratios,
 * the face flow (whose duct loss depends on it) and, with a mixer, the static pressure where the streams meet, are
 * solved so that the low-pressure turbine passes the high-pressure turbine's flow, the nozzle(s) pass theirs and the
 * streams meet at one static pressure (Newton's method, numerical Jacobian, damped). The equilibrium adds the spool
 * speeds as unknowns and the spools' power balances as equations. Unknowns: 0 ln(HP turbine pressure ratio), 1 ln(LP
 * turbine pressure ratio), 2 face flow (fraction of design, corrected), 3 ln(core total over mixer static pressure), 4
 * 100 x burner fuel-air ratio, 5 HP work fraction (speed fraction squared), 6 LP work fraction.
 */
final class TwoSpool {
    static final int H = 0, L = 1, Q = 2, M = 3, F = 4, WH = 5, WL = 6, N = 7;
    static final int R_LPT = 0, R_NOZZLE = 1, R_FACE = 2, R_MIXED = 3, R_FUEL = 4, R_HIGH = 5, R_LOW = 6;
    /** Mach number of the core stream entering the mixer at design (mixers are laid out for about 0.4-0.5). */
    static final double MIXER_MACH = 0.45;
    /**
     * Largest residual (relative to the flows and powers) a match is accepted at when Newton's method stalls: the gas
     * tables' interpolation and the inner iterations leave noise of about 1e-9 to 1e-8 near an unchoked nozzle.
     */
    static final double ACCEPTED = 1e-6;
    private static final double[] LOWER = {1e-7, 1e-7, 0, 1e-7, 0, 1e-4, 1e-4}, UPPER = {Math.log(60), Math.log(60), 3, Math.log(1.89), 6.5, 4, 4};

    final Turbojet.Design core;
    final boolean fan, mixed;
    final double bypassRatio, fanRatio, boosterRatio, fanEfficiency, lowTurbineEfficiency, bypassRetention, mixerRetention, exhaustRatio;
    final Turbojet.Flight condition;
    private final Gas products;
    private final double afr;
    final Mix air;
    // sized: flow areas (m^2), design works (J/kg), design corrected flows (kg/s) the losses refer to
    final double a4, a45, a8, aBypass, a6;
    final double fanWork, boosterWork, compressorWork, spoutingWork;
    final double faceFlow, burnerFlow, tailFlow, mixedFlow, pt2Design, fuelDesign;
    final double[] designX;
    final State design;

    /**
     * Sizes the engine at a flight condition. fan: the low spool drives a fan (bypassRatio, fanRatio, boosterRatio,
     * fanEfficiency; exhaustRatio unused) or else a free power turbine expands the gas to a design exhaust total
     * pressure of exhaustRatio times ambient.
     */
    TwoSpool(Turbojet.Design core, boolean fan, double bypassRatio, double fanRatio, double boosterRatio, double fanEfficiency,
             double lowTurbineEfficiency, double bypassRetention, boolean mixed, double mixerRetention, double exhaustRatio, Turbojet.Flight flight) {
        this.core = core; this.fan = fan; this.mixed = fan && mixed && bypassRatio > 0;
        this.bypassRatio = fan ? bypassRatio : 0; this.fanRatio = fan ? fanRatio : 1; this.boosterRatio = fan ? boosterRatio : 1;
        this.fanEfficiency = fanEfficiency; this.lowTurbineEfficiency = lowTurbineEfficiency;
        this.bypassRetention = bypassRetention; this.mixerRetention = mixerRetention; this.exhaustRatio = exhaustRatio;
        this.condition = flight;
        products = GasPath.stoichiometric(core.fuel());
        afr = core.fuel().stoichiometricAirFuel();
        air = mix(0);
        Gas a = Gas.AIR;
        double tt2 = flight.totalTemperature(), p0 = flight.pressurePa();
        double pt2 = flight.totalPressure() * GasPath.inletRecovery(core.inletRecovery(), core.inlet(), flight.mach(), 1);
        double mc = core.airflowKgPerS(), mb = this.bypassRatio * mc;
        double t13 = Turbomachinery.compressorExitTemperature(this.fanRatio, fanEfficiency, tt2);
        double t25 = Turbomachinery.compressorExitTemperature(this.boosterRatio, core.compressorEfficiency(), t13);
        fanWork = a.enthalpy(t13) - a.enthalpy(tt2);
        boosterWork = a.enthalpy(t25) - a.enthalpy(t13);
        double pt13 = this.fanRatio * pt2, pt25 = this.boosterRatio * pt13;
        double t3 = Turbomachinery.compressorExitTemperature(core.pressureRatio(), core.compressorEfficiency(), t25);
        compressorWork = a.enthalpy(t3) - a.enthalpy(t25);
        double pt3 = core.pressureRatio() * pt25, t4 = core.turbineInletK();
        double c = core.coolingBleed(), f = GasPath.fuelAirRatio(core.fuel(), t3, t4, core.burnerEfficiency());
        double pt4 = core.burnerPressureRatio() * pt3;
        Mix hot = mix(f);
        double m4 = mc * (1 - c) * (1 + f);
        double wtH = compressorWork / (core.mechanicalEfficiency() * (1 - c) * (1 + f));
        double t45 = hot.tAtH(hot.h(t4) - wtH);
        double lnH = (hot.phi(t4) - hot.phi(t45)) / (core.turbineEfficiency() * hot.r);
        a4 = m4 / (hot.flux(t4, Math.exp(lnH)) * pt4);
        double fo = f * (1 - c);
        Mix mid = mix(fo);
        double t45m = mid.tAtH(((1 - c) * (1 + f) * hot.h(t45) + c * a.enthalpy(t3)) / (1 + fo));
        double pt45 = pt4 / Math.exp(lnH), m45 = mc * (1 + fo);
        double lnL, t5;
        if (fan) {
            double wtL = ((mc + mb) * fanWork + mc * boosterWork) / (core.mechanicalEfficiency() * m45);
            t5 = mid.tAtH(mid.h(t45m) - wtL);
            lnL = (mid.phi(t45m) - mid.phi(t5)) / (lowTurbineEfficiency * mid.r);
        } else {
            double pt5 = exhaustRatio * p0 / core.nozzlePressureRatio();
            if (!(pt45 > 1.0001 * pt5)) {
                throw new IllegalArgumentException("The cycle does not close: the gas generator's turbine leaves no pressure for the power turbine");
            }
            lnL = Math.log(pt45 / pt5);
            t5 = mid.turbineExit(t45m, lnL, lowTurbineEfficiency);
        }
        spoutingWork = mid.h(t45m) - mid.h(mid.turbineExit(t45m, lnL, 1));
        a45 = m45 / (mid.flux(t45m, Math.exp(lnL)) * pt45);
        double pt5 = pt45 / Math.exp(lnL);
        double lnM = 0, aB = 0, aSix = 0, aEight, mixFlow = 0;
        if (this.mixed) {
            double p6 = staticAtMach(mid, t5, pt5, MIXER_MACH);
            double[] e6 = mid.expand(t5, pt5, p6);
            aSix = m45 / (p6 / (mid.r * e6[0]) * e6[1]);
            double pt16 = bypassRetention * pt13;
            if (!(pt16 > 1.0001 * p6)) {
                throw new IllegalArgumentException("The bypass air cannot enter the mixer: its pressure is below the core stream's there (raise the fan pressure ratio)");
            }
            if (air.sonic(t13)[1] * pt16 > p6) {
                throw new IllegalArgumentException("The bypass air would enter the mixer faster than sound (lower the fan pressure ratio)");
            }
            aB = mb / flowTo(air, t13, pt16, p6);
            lnM = Math.log(pt5 / p6);
            double[] mixedOut = mixOut(mid, fo, mc, m45, mb, t5, pt5, t13, pt16, p6);   // {tt, pt, flow, fuel-air ratio}
            double pt8 = mixerRetention * core.nozzlePressureRatio() * mixedOut[1];
            if (!(pt8 > 1.0001 * p0)) { throw new IllegalArgumentException("The cycle does not close: the mixed stream has no pressure left for the nozzle"); }
            aEight = GasPath.throatFor(mix(mixedOut[3]), mixedOut[2], mixedOut[0], pt8, p0);
            mixFlow = corrected(mixedOut[2], mixedOut[0], mixedOut[1]);
        } else {
            double pt8 = core.nozzlePressureRatio() * pt5;
            if (!(pt8 > 1.0001 * p0)) {
                throw new IllegalArgumentException(fan ? "The cycle does not close: the fan takes all the core's pressure and leaves none for the core jet (lower the fan pressure ratio or bypass ratio)"
                        : "The cycle does not close: the exhaust is below ambient");
            }
            aEight = GasPath.throatFor(mid, m45, t5, pt8, p0);
            if (mb > 0) {
                double pt17 = bypassRetention * pt13;
                if (!(pt17 > 1.0001 * p0)) { throw new IllegalArgumentException("The fan's pressure ratio is too low to push the bypass air out"); }
                aB = GasPath.throatFor(air, mb, t13, pt17, p0);
            }
        }
        a8 = aEight; aBypass = aB; a6 = aSix; mixedFlow = mixFlow;
        faceFlow = corrected(mc + mb, tt2, pt2);
        burnerFlow = corrected(mc, t3, pt3);
        tailFlow = corrected(m45, t5, pt5);
        pt2Design = pt2;
        fuelDesign = fo * mc;
        designX = new double[]{lnH, lnL, 1, lnM, 100 * f, 1, fan ? 1 : 0};
        design = evaluate(flight, designX, t4, Double.NaN, 1);
    }

    Mix mix(double fuelAirRatio) { return new Mix(products, afr, fuelAirRatio); }

    /** Static pressure at which flow from (tt, pt) runs at a Mach number (subsonic), Pa. */
    private static double staticAtMach(Mix m, double tt, double pt, double mach) {
        double lo = m.sonic(tt)[1] * pt, hi = pt;
        for (int i = 0; i < 80 && hi - lo > 1e-10 * pt; i++) {
            double p = 0.5 * (lo + hi);
            double[] e = m.expand(tt, pt, p);
            double cp = m.cp(e[0]), sound = Math.sqrt(cp / (cp - m.r) * m.r * e[0]);
            if (e[1] / sound > mach) { lo = p; } else { hi = p; }
        }
        return 0.5 * (lo + hi);
    }

    /** Mass flux (kg/(s m^2)) of flow from (tt, pt) expanding to static p: none if p is not below pt, sonic at most. */
    private static double flowTo(Mix m, double tt, double pt, double p) {
        if (p >= pt) { return 0; }
        double[] star = m.sonic(tt);
        if (p <= star[1] * pt) { return star[2] * pt; }
        double[] e = m.expand(tt, pt, p);
        return p / (m.r * e[0]) * e[1];
    }

    /**
     * Mixing at constant static pressure p (momentum and energy conserved, no wall force): {total temperature, total
     * pressure, flow, fuel-air ratio per kilogram of all the air} of the mixed stream.
     */
    private double[] mixOut(Mix mid, double fo, double coreAir, double m45, double mb, double t5, double pt5, double t13, double pt16, double p) {
        double v6 = p < pt5 ? mid.expand(t5, pt5, Math.max(p, mid.sonic(t5)[1] * pt5))[1] : 0;
        double v16 = p < pt16 ? air.expand(t13, pt16, Math.max(p, air.sonic(t13)[1] * pt16))[1] : 0;
        double flow = m45 + mb, fm = fo * coreAir / (coreAir + mb);
        Mix mixd = mix(fm);
        double ht = (m45 * mid.h(t5) + mb * air.h(t13)) / flow, v = (m45 * v6 + mb * v16) / flow;
        double tt = mixd.tAtH(ht), ts = mixd.tAtH(ht - v * v / 2);
        return new double[]{tt, p * Math.exp((mixd.phi(tt) - mixd.phi(ts)) / mixd.r), flow, fm};
    }

    /** One evaluation of the gas path: the stations, powers, thrust, and the residuals of every matching equation. */
    static final class State {
        final double[] r = new double[7];
        double[] x;
        double thrust, coreThrust, bypassThrust, fuel, f, core, bypass, t2, pt2, t13, pt13, t25, pt25, t3, pt3, t4, pt4, t45, pt45, t5, pt5, pc;
        double fanPower, lowTurbinePower, compressorPower, highTurbinePower, flow25, exitSpeed, exitPressure, exitArea;
        /** Free turbine: the shaft torque times the power turbine's design speed (rad/s), W; and that turbine's speed (fraction of design). */
        double shaftTorqueW, powerTurbineSpeed;
        boolean choked;
        /** The stream entering the (core or mixed) nozzle: {total temperature, total pressure after the duct's loss, flow, fuel-air ratio per kg of air} (0.28, for reheat). */
        double[] nozzleInlet;
    }

    /**
     * The gas path at the unknowns x (all seven; those not solved for are inputs: the spool work fractions, and the
     * fuel-air ratio when t4 is NaN) with a turbine inlet temperature t4 (NaN: from x[F]), a fuel flow (for the fuel
     * equation) and the power turbine's speed (fraction of design; free turbine only).
     */
    State evaluate(Turbojet.Flight fl, double[] x, double t4Given, double fuel, double powerTurbineSpeed) {
        State s = new State();
        s.x = x.clone();
        s.powerTurbineSpeed = powerTurbineSpeed;
        Gas a = Gas.AIR;
        Turbojet.Design d = core;
        double c = d.coolingBleed(), em = d.mechanicalEfficiency();
        double wl = fan ? Math.max(0, x[WL]) : 0, wh = Math.max(0, x[WH]), q = Math.max(0, x[Q]);
        double tt2 = fl.totalTemperature(), p0 = fl.pressurePa();
        double pt2 = fl.totalPressure() * GasPath.inletRecovery(d.inletRecovery(), d.inlet(), fl.mach(), q);
        double t13 = a.temperatureAtEnthalpy(a.enthalpy(tt2) + fanWork * wl);
        double pf = Math.exp(fanEfficiency * (a.entropyFunction(t13) - a.entropyFunction(tt2)) / a.gasConstant());
        double t25 = a.temperatureAtEnthalpy(a.enthalpy(t13) + boosterWork * wl);
        double pbst = Math.exp(d.compressorEfficiency() * (a.entropyFunction(t25) - a.entropyFunction(t13)) / a.gasConstant());
        double pt13 = pf * pt2, pt25 = pbst * pt13;
        double t3 = a.temperatureAtEnthalpy(a.enthalpy(t25) + compressorWork * wh);
        double pc = Math.exp(d.compressorEfficiency() * (a.entropyFunction(t3) - a.entropyFunction(t25)) / a.gasConstant());
        double pt3 = pc * pt25;
        double f, t4;
        if (Double.isNaN(t4Given)) { f = Math.max(0, x[F] / 100); t4 = f > 0 ? GasPath.burnerExit(d.fuel(), t3, f, d.burnerEfficiency()) : t3; }
        else { t4 = Math.max(t4Given, t3); f = GasPath.fuelAirRatio(d.fuel(), t3, t4, d.burnerEfficiency()); }
        s.x[F] = 100 * f;   // the solution carries the burner's fuel-air ratio either way (a warm start for a fuel-given solve)
        Mix hot = mix(f);
        double lnH = Math.max(LOWER[H], x[H]), lnL = Math.max(LOWER[L], x[L]);
        double flux4 = hot.flux(t4, Math.exp(lnH));
        double k = corrected(a4 * flux4 / ((1 - c) * (1 + f)), t3, 1) / burnerFlow;
        double pb = Math.max(0.3, 2 / (1 + Math.sqrt(1 + 4 * (1 - d.burnerPressureRatio()) * k * k)));
        double pt4 = pb * pt3, m4 = a4 * flux4 * pt4, mc = m4 / ((1 - c) * (1 + f));
        double t45 = hot.turbineExit(t4, lnH, d.turbineEfficiency()), wtH = hot.h(t4) - hot.h(t45);
        double fo = f * (1 - c);
        Mix mid = mix(fo);
        double t45m = mid.tAtH(((1 - c) * (1 + f) * hot.h(t45) + c * a.enthalpy(t3)) / (1 + fo));
        double pt45 = pt4 / Math.exp(lnH), m45 = mc * (1 + fo);
        double t5, wtL;
        if (fan) { t5 = mid.turbineExit(t45m, lnL, lowTurbineEfficiency); wtL = mid.h(t45m) - mid.h(t5); }
        else {
            // the power turbine's work and its torque times the design speed: P = k(x) w with x = n / (C0 / C0_design);
            // below the best ratio k = x (2 - x), so the torque, w (2 - x) / (C0 / C0_design), stays finite with the rotor held
            double poly = mid.h(t45m) - mid.h(mid.turbineExit(t45m, lnL, lowTurbineEfficiency));
            double c0 = Math.sqrt(Math.max(0, mid.h(t45m) - mid.h(mid.turbineExit(t45m, lnL, 1))) / spoutingWork);
            // NaN speed: the turbine at its best speed ratio (the gas generator's running line, independent of the load)
            double n = Double.isNaN(powerTurbineSpeed) ? c0 : Math.max(0, powerTurbineSpeed);
            s.powerTurbineSpeed = n;
            double ratio = Math.min(c0 > 1e-9 ? n / c0 : n > 0 ? 3 : 0, 3);   // no spouting velocity: a turning rotor is past its range
            wtL = Turbomachinery.turbineSpeedEfficiency(ratio) * poly;
            t5 = mid.tAtH(mid.h(t45m) - wtL);
            s.shaftTorqueW = ratio <= 1 ? (c0 > 1e-9 ? em * m45 * poly * (2 - ratio) / c0 : 0) : em * m45 * wtL / n;
        }
        double pt5 = pt45 / Math.exp(lnL);
        double scale = pt2 / pt2Design * Math.sqrt(condition.totalTemperature() / tt2);
        double mRef = core.airflowKgPerS() * scale * Math.max(0.03, Math.sqrt(Math.max(wh, wl)));
        s.r[R_LPT] = mismatch(a45 * mid.flux(t45m, Math.exp(lnL)) * pt45, m45, mRef);
        double mb, gross, coreGross, bypassGross = 0;
        double[] nz;
        if (mixed) {
            double p6 = pt5 / Math.exp(Math.max(LOWER[M], x[M]));
            s.r[R_NOZZLE] = mismatch(a6 * flowTo(mid, t5, pt5, p6), m45, mRef);
            double pt16 = retained(bypassRetention, q) * pt13;
            mb = aBypass * flowTo(air, t13, pt16, p6);
            double[] out = mixOut(mid, fo, mc, m45, mb, t5, pt5, t13, pt16, p6);
            Mix mixd = mix(out[3]);
            double pt8 = retained(mixerRetention * d.nozzlePressureRatio(), corrected(out[2], out[0], out[1]) / mixedFlow) * out[1];
            nz = nozzle(mixd, out[0], pt8, p0, a8, d.convergentDivergent(), d.exitAreaRatio());
            s.nozzleInlet = new double[]{out[0], pt8, out[2], out[3]};
            s.r[R_MIXED] = mismatch(nz[0], out[2], mRef);
            gross = out[2] * nz[1] + (nz[2] - p0) * nz[3];
            coreGross = gross;
        } else {
            double pt8 = retained(d.nozzlePressureRatio(), corrected(m45, t5, pt5) / tailFlow) * pt5;
            nz = nozzle(mid, t5, pt8, p0, a8, fan && d.convergentDivergent(), d.exitAreaRatio());
            s.nozzleInlet = new double[]{t5, pt8, m45, fo};
            s.r[R_NOZZLE] = mismatch(nz[0], m45, mRef);
            coreGross = m45 * nz[1] + (nz[2] - p0) * nz[3];
            mb = 0;
            if (aBypass > 0) {
                double[] bz = nozzle(air, t13, retained(bypassRetention, q) * pt13, p0, aBypass, false, 1);
                mb = bz[0];
                bypassGross = mb * bz[1] + (bz[2] - p0) * bz[3];
            }
            gross = coreGross + bypassGross;
        }
        s.r[R_FACE] = corrected(mc + mb, tt2, pt2) / faceFlow - q;
        double fuelFlow = fo * mc;
        s.r[R_FUEL] = (fuelFlow - fuel) / (fuelDesign * scale * Math.max(0.03, wh));
        s.highTurbinePower = em * m4 * wtH;
        s.compressorPower = mc * compressorWork * wh;
        s.lowTurbinePower = em * m45 * wtL;
        s.fanPower = (mc + mb) * fanWork * wl + mc * boosterWork * wl;
        double pRef = core.airflowKgPerS() * compressorWork * scale * Math.max(0.03, Math.pow(Math.max(wh, wl), 1.5));
        s.r[R_HIGH] = (s.highTurbinePower - s.compressorPower) / pRef;
        s.r[R_LOW] = (s.lowTurbinePower - s.fanPower) / pRef;
        double ram = (mc + mb) * fl.speedMps();
        s.thrust = gross - ram;
        s.coreThrust = mixed ? s.thrust : coreGross - mc * fl.speedMps();
        s.bypassThrust = mixed ? 0 : bypassGross - mb * fl.speedMps();
        s.fuel = fuelFlow; s.f = f; s.core = mc; s.bypass = mb;
        s.t2 = tt2; s.pt2 = pt2; s.t13 = t13; s.pt13 = pt13; s.t25 = t25; s.pt25 = pt25; s.t3 = t3; s.pt3 = pt3; s.t4 = t4; s.pt4 = pt4;
        s.t45 = t45m; s.pt45 = pt45; s.t5 = t5; s.pt5 = pt5; s.pc = pc;
        s.flow25 = corrected(mc, t25, pt25);
        s.exitSpeed = nz[1]; s.exitPressure = nz[2]; s.exitArea = nz[3]; s.choked = nz[4] > 0.5;
        return s;
    }

    /**
     * A flow passed against the flow delivered, (a^2 - b^2) / (2 ref^2): the same root as a - b, but a nozzle barely above
     * ambient (or a turbine across a tiny pressure ratio) passes flow as the square root of the pressure difference, whose
     * slope would be unbounded there; its square's is not.
     */
    private static double mismatch(double passed, double delivered, double ref) { return (passed * passed - delivered * delivered) / (2 * ref * ref); }

    /** What is solved for: the turbine inlet temperature given (else the fuel flow), the spools in power balance (else their speeds given). */
    private boolean[] unknowns(boolean fuelGiven, boolean equilibrium) {
        boolean[] u = new boolean[N];
        u[H] = u[L] = u[Q] = true;
        u[M] = mixed;
        u[F] = fuelGiven;
        u[WH] = equilibrium;
        u[WL] = equilibrium && fan;
        return u;
    }

    /**
     * Solves the matching from the guess x (all seven unknowns, inputs included); null if Newton's method does not
     * converge (to 1e-10, or ACCEPTED where it stalls). With t4 NaN the fuel flow is given and the fuel-air ratio solved for.
     */
    State solve(Turbojet.Flight fl, double[] guess, double t4, double fuel, double powerTurbineSpeed, boolean equilibrium) {
        boolean[] u = unknowns(Double.isNaN(t4), equilibrium);
        int n = 0;
        int[] idx = new int[N];
        for (int i = 0; i < N; i++) { if (u[i]) { idx[n++] = i; } }   // residual R_i pairs with unknown i
        double[] x = guess.clone();
        for (int i = 0; i < n; i++) { x[idx[i]] = Math.clamp(x[idx[i]], LOWER[idx[i]], UPPER[idx[i]]); }
        State s = evaluate(fl, x, t4, fuel, powerTurbineSpeed);
        double norm = norm(s, idx, n), squares = squares(s, idx, n);
        for (int it = 0; it < 40; it++) {
            if (norm < 1e-10) { return s; }
            double[][] jac = new double[n][n];
            for (int j = 0; j < n; j++) {
                int k = idx[j];
                double step = 1e-7 * Math.max(0.1, Math.abs(x[k]));
                if (x[k] + step > UPPER[k]) { step = -step; }
                double[] xs = x.clone();
                xs[k] += step;
                State t = evaluate(fl, xs, t4, fuel, powerTurbineSpeed);
                for (int i = 0; i < n; i++) { jac[i][j] = (t.r[idx[i]] - s.r[idx[i]]) / step; }
            }
            double[] rhs = new double[n];
            for (int i = 0; i < n; i++) { rhs[i] = -s.r[idx[i]]; }
            double[] dx = gauss(jac, rhs);
            if (dx == null) { return null; }
            double lambda = 1;
            State best = null;
            double[] bestX = null;
            for (int ls = 0; ls < 12; ls++) {
                double[] xt = x.clone();
                for (int i = 0; i < n; i++) { int k = idx[i]; xt[k] = Math.clamp(x[k] + lambda * dx[i], LOWER[k], UPPER[k]); }
                State t = evaluate(fl, xt, t4, fuel, powerTurbineSpeed);
                double st = squares(t, idx, n);
                // Armijo's condition on the sum of squares (Newton's step descends it at twice its value)
                if (Double.isFinite(st) && st < (1 - 1e-4 * lambda) * squares) { best = t; bestX = xt; norm = norm(t, idx, n); squares = st; break; }
                lambda *= 0.5;
            }
            if (best == null) { return norm < ACCEPTED ? s : null; }
            x = bestX; s = best;
        }
        return norm < ACCEPTED ? s : null;
    }

    private static double squares(State s, int[] idx, int n) {
        double m = 0;
        for (int i = 0; i < n; i++) { m += s.r[idx[i]] * s.r[idx[i]]; }
        return Double.isNaN(m) ? Double.POSITIVE_INFINITY : m;
    }

    private static double norm(State s, int[] idx, int n) {
        double m = 0;
        for (int i = 0; i < n; i++) { m = Math.max(m, Math.abs(s.r[idx[i]])); }
        return Double.isNaN(m) ? Double.POSITIVE_INFINITY : m;
    }

    /** Gaussian elimination with partial pivoting; null if singular. */
    private static double[] gauss(double[][] a, double[] b) {
        int n = b.length;
        for (int col = 0; col < n; col++) {
            int p = col;
            for (int i = col + 1; i < n; i++) { if (Math.abs(a[i][col]) > Math.abs(a[p][col])) { p = i; } }
            if (!(Math.abs(a[p][col]) > 1e-14)) { return null; }
            double[] tr = a[col]; a[col] = a[p]; a[p] = tr;
            double tb = b[col]; b[col] = b[p]; b[p] = tb;
            for (int i = col + 1; i < n; i++) {
                double m = a[i][col] / a[col][col];
                for (int j = col; j < n; j++) { a[i][j] -= m * a[col][j]; }
                b[i] -= m * b[col];
            }
        }
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = b[i];
            for (int j = i + 1; j < n; j++) { sum -= a[i][j] * x[j]; }
            x[i] = sum / a[i][i];
        }
        return x;
    }

    /**
     * Equilibrium at a turbine inlet temperature in flight (a free power turbine at a speed, fraction of design): from the
     * guess, or else marching the turbine temperature from the design point's corrected value at this flight.
     */
    State equilibrium(Turbojet.Flight fl, double t4, double powerTurbineSpeed, double[] guess) {
        if (guess != null) {
            State s = solve(fl, guess, t4, 0, powerTurbineSpeed, true);
            if (s != null) { return s; }
        }
        double theta = fl.totalTemperature() / condition.totalTemperature();
        double[] x = designX.clone();
        x[WH] *= theta; x[WL] *= theta;
        double start = core.turbineInletK() * theta;
        State s = null;
        int steps = 1 + (int) Math.ceil(Math.abs(t4 - start) / (0.04 * core.turbineInletK()));
        for (int i = 0; i <= steps; i++) {
            s = solve(fl, x, start + (t4 - start) * i / steps, 0, powerTurbineSpeed, true);
            if (s == null) { throw new IllegalArgumentException("No equilibrium at " + Math.round(t4) + " K in this flight condition"); }
            x = s.x;
        }
        return s;
    }

    // ---------------------------------------------------------------- running lines

    /** Columns of a running line row: corrected (to the engine face) LP and HP speed fractions, fuel flow, T4, HP compressor flow at its face, its pressure ratio, then the unknowns. */
    static final int C_LOW = 0, C_HIGH = 1, C_FUEL = 2, C_T4 = 3, C_FLOW25 = 4, C_PC = 5, C_X = 6, COLUMNS = C_X + N;
    /** Lowest corrected HP speed the running line tables hold. */
    static final double MIN_HIGH_SPEED = 0.45;

    /** One running line: rows from low to full power at one flight Mach number. */
    record Line(double mach, double[][] rows) {
        double[] column(int k) { double[] c = new double[rows.length]; for (int i = 0; i < rows.length; i++) { c[i] = rows[i][k]; } return c; }
    }

    /** Corrected row of an equilibrium state. */
    private static double[] row(State s) {
        double theta = s.t2 / Turbojet.T_REF, delta = s.pt2 / Turbojet.P_REF;
        double[] r = new double[COLUMNS];
        r[C_LOW] = Math.sqrt(Math.max(0, s.x[WL]) / theta);
        r[C_HIGH] = Math.sqrt(Math.max(0, s.x[WH]) / theta);
        r[C_FUEL] = s.fuel / (delta * Math.sqrt(theta));
        r[C_T4] = s.t4 / theta;
        r[C_FLOW25] = s.flow25;
        r[C_PC] = s.pc;
        System.arraycopy(s.x, 0, r, C_X, N);
        r[C_X + WH] = s.x[WH] / theta;
        r[C_X + WL] = s.x[WL] / theta;
        return r;
    }

    /**
     * The running line at a flight condition: equilibria from 1.08 times the design turbine temperature (as hot as in
     * the coldest standard air) down to the lowest HP speed; a free power turbine at its best speed ratio, wherever the
     * load holds it (at low power a load at its design speed would brake it).
     */
    Line line(Turbojet.Flight fl) {
        double t4max = core.turbineInletK() * 1.08 * Math.max(1, fl.temperatureK() / Turbojet.COLD_AMBIENT);
        State s = equilibrium(fl, t4max, Double.NaN, null);
        List<double[]> rows = new ArrayList<>();
        rows.add(row(s));
        double step = 0.01 * core.turbineInletK();
        for (double t4 = t4max - step; t4 > 1.15 * fl.totalTemperature(); t4 -= step) {
            State next = solve(fl, s.x, t4, 0, Double.NaN, true);
            if (next == null) { break; }
            double[] r = row(next);
            if (r[C_HIGH] >= rows.getLast()[C_HIGH] - 1e-9) { break; }   // past the lowest speed the spools hold themselves at
            s = next;
            rows.add(r);
            if (r[C_HIGH] < MIN_HIGH_SPEED) { break; }
        }
        if (rows.size() < 3) { throw new IllegalArgumentException("No running line: the engine cannot hold itself up below full power"); }
        double[][] table = rows.reversed().toArray(double[][]::new);
        return new Line(fl.mach(), table);
    }

    /** Running lines at flight Mach numbers (at the design condition's ambient state), up to the last one with a line. */
    List<Line> lines(double[] machs) {
        Turbojet.Flight c = condition;
        double sound = Math.sqrt(1.4 * Gas.AIR.gasConstant() * c.temperatureK());
        List<Line> out = new ArrayList<>();
        for (double m : machs) {
            try { out.add(line(new Turbojet.Flight(c.temperatureK(), c.pressurePa(), m * sound))); }
            catch (IllegalArgumentException e) {
                if (out.isEmpty()) { throw e; }
                break;
            }
        }
        return out;
    }

    /** A running line row at a flight Mach number and a value of a key column (C_LOW or C_HIGH): blended between the two nearest lines in ram pressure ratio. */
    static double[] at(List<Line> lines, double mach, int key, double value) {
        int i = 0;
        while (i < lines.size() - 2 && mach > lines.get(i + 1).mach()) { i++; }
        double[] a = interpolate(lines.get(i), key, value);
        if (lines.size() == 1) { return a; }
        double r = ram(mach), lo = ram(lines.get(i).mach()), hi = ram(lines.get(i + 1).mach());
        double w = Math.clamp((r - lo) / (hi - lo), 0, 1);
        double[] b = interpolate(lines.get(i + 1), key, value);
        double[] out = new double[COLUMNS];
        for (int k = 0; k < COLUMNS; k++) { out[k] = (1 - w) * a[k] + w * b[k]; }
        return out;
    }
    private static double ram(double m) { return Math.pow(1 + 0.2 * m * m, 3.5); }

    /**
     * A row of one line at a key value. Inside the table linearly; below it by the affinity laws from the first row
     * (flows and fuel with speed, pressure rises and works with speed squared, temperatures held); above it along the
     * last segment.
     */
    static double[] interpolate(Line line, int key, double value) {
        double[][] rows = line.rows();
        double first = rows[0][key];
        if (value < first) {
            double n = Math.max(0, value) / first;
            double[] r0 = rows[0], out = r0.clone();
            out[C_LOW] = r0[C_LOW] * n; out[C_HIGH] = r0[C_HIGH] * n;
            out[C_FUEL] = r0[C_FUEL] * n; out[C_FLOW25] = r0[C_FLOW25] * n;
            out[C_PC] = 1 + (r0[C_PC] - 1) * n * n;
            for (int k : new int[]{H, L, M, WH, WL}) { out[C_X + k] = r0[C_X + k] * n * n; }
            out[C_X + Q] = r0[C_X + Q] * n;
            return out;
        }
        int i = 0;
        while (i < rows.length - 2 && value > rows[i + 1][key]) { i++; }
        double t = (value - rows[i][key]) / (rows[i + 1][key] - rows[i][key]);
        t = Math.min(t, 3);
        double[] out = new double[COLUMNS];
        for (int k = 0; k < COLUMNS; k++) { out[k] = rows[i][k] + t * (rows[i + 1][k] - rows[i][k]); }
        return out;
    }

    /** The HP compressor's pressure ratio on the running line at a corrected flow at its face (the surge margin's reference). */
    static double lineCompressorRatio(List<Line> lines, double mach, double flow25) {
        // the corrected flow rises with speed along each line: find the speed with this flow, then its pressure ratio
        Line l = lines.getFirst();
        double lo = 0, hi = l.rows()[l.rows().length - 1][C_HIGH] * 1.5;
        for (int it = 0; it < 50; it++) {
            double mid = 0.5 * (lo + hi);
            if (at(lines, mach, C_HIGH, mid)[C_FLOW25] < flow25) { lo = mid; } else { hi = mid; }
        }
        return at(lines, mach, C_HIGH, 0.5 * (lo + hi))[C_PC];
    }

    /** Unknowns for a quasi-steady solve at given speeds and fuel, from the running line at the HP speed (the cold start of the solver). */
    double[] guessFromLine(List<Line> lines, Turbojet.Flight fl, double lowSpeed, double highSpeed, double fuel) {
        double theta = fl.totalTemperature() / Turbojet.T_REF;
        double[] r = at(lines, fl.mach(), C_HIGH, Math.max(0, highSpeed) / Math.sqrt(theta));
        double[] x = Arrays.copyOfRange(r, C_X, C_X + N);
        x[WH] = highSpeed * highSpeed;
        x[WL] = fan ? lowSpeed * lowSpeed : 0;
        if (fuel <= 0) { x[F] = 0; }
        return x;
    }

    // ---------------------------------------------------------------- transients

    /** Whether the core passes any flow at these work fractions: the compressors and ram give more than ambient pressure. */
    boolean flows(Turbojet.Flight fl, double wl, double wh) {
        Gas a = Gas.AIR;
        double tt2 = fl.totalTemperature(), ec = core.compressorEfficiency(), r = a.gasConstant();
        double t13 = a.temperatureAtEnthalpy(a.enthalpy(tt2) + fanWork * wl);
        double t25 = a.temperatureAtEnthalpy(a.enthalpy(t13) + boosterWork * wl);
        double t3 = a.temperatureAtEnthalpy(a.enthalpy(t25) + compressorWork * wh);
        double lnRise = (fanEfficiency * (a.entropyFunction(t13) - a.entropyFunction(tt2)) + ec * (a.entropyFunction(t3) - a.entropyFunction(t13))) / r;
        return fl.totalPressure() * Math.exp(lnRise) > fl.pressurePa() * (1 + 1e-6);
    }

    /**
     * Quasi-steady state at given spool speeds (fractions of design; the LP's unused without a fan), fuel flow (kg/s)
     * and power turbine speed: the compressors at their speeds, the flows the turbines and nozzles pass of the actual
     * gas, the spools' power left over for accelerating them. Warm-started from a previous solution (else from the
     * running line at the HP speed); null if no flow passes or no match is found.
     */
    State quasiSteady(List<Line> lines, Turbojet.Flight fl, double lowSpeed, double highSpeed, double fuel, double powerTurbineSpeed, double[] warm) {
        double wl = fan ? lowSpeed * lowSpeed : 0, wh = highSpeed * highSpeed;
        if (!flows(fl, wl, wh)) { return null; }
        if (warm != null && warm.length == N && Arrays.stream(warm).allMatch(Double::isFinite)) {
            double[] x = warm.clone();
            x[WH] = wh; x[WL] = wl;
            if (fuel <= 0) { x[F] = 0; }
            State s = solve(fl, x, Double.NaN, fuel, powerTurbineSpeed, false);
            if (s != null) { return s; }
        }
        double[] g = guessFromLine(lines, fl, lowSpeed, highSpeed, fuel);
        State s = solve(fl, g, Double.NaN, fuel, powerTurbineSpeed, false);
        if (s != null) { return s; }
        // A fuel far from the running line's (a flame-out, a cold spool): from the running line's turbine temperature at
        // these speeds, step the fuel to the one asked for.
        double theta = fl.totalTemperature() / Turbojet.T_REF;
        double[] row = at(lines, fl.mach(), C_HIGH, Math.max(0, highSpeed) / Math.sqrt(theta));
        State from = solve(fl, g, row[C_T4] * theta, 0, powerTurbineSpeed, false);
        return from == null ? null : march(fl, from, fuel, powerTurbineSpeed, 0);
    }

    /** From a solved state to a fuel flow in steps (halving a step that fails, three times at most). */
    private State march(Turbojet.Flight fl, State from, double fuel, double powerTurbineSpeed, int depth) {
        State s = from;
        double start = from.fuel;
        for (int i = 1; i <= 4; i++) {
            double target = start + (fuel - start) * i / 4;
            State next = solve(fl, s.x, Double.NaN, target, powerTurbineSpeed, false);
            if (next == null) {
                if (depth >= 3) { return null; }
                next = march(fl, s, target, powerTurbineSpeed, depth + 1);
                if (next == null) { return null; }
            }
            s = next;
        }
        return s;
    }

    /** Turbine flow parameter of gas with burner fuel-air ratio f at t4: choked flux over (1 + f), s/m. */
    double parameter(double f, double t4) { return mix(f).sonic(t4)[2] / (1 + f); }

    /**
     * Fuel flow (kg/s) that would make a turbine inlet temperature at the spool speeds of a quasi-steady state (burner
     * inlet t3, now at t4Now with fuel-air ratio fNow and coreFlow kg/s of air): the HP turbine choked, the core flow
     * goes with its flow parameter.
     */
    double fuelForTurbineInlet(double t3, double t4Now, double fNow, double coreFlow, double t4) {
        if (!(t4 > t3) || coreFlow <= 0) { return 0; }
        double f = GasPath.fuelAirRatio(core.fuel(), t3, t4, core.burnerEfficiency());
        return f * (1 - core.coolingBleed()) * coreFlow * parameter(f, t4) / parameter(fNow, t4Now);
    }

    /**
     * Fuel flow (kg/s) at which the HP compressor of a quasi-steady state (pressure ratio pc at corrected flow flow25)
     * would use a surge margin (its pressure ratio over the running line's at the same corrected flow, less one); 0
     * if even no fuel uses more.
     */
    double fuelForSurgeMargin(List<Line> lines, double mach, double t3, double t4Now, double fNow, double coreFlow, double pc, double flow25, double used) {
        if (coreFlow <= 0 || flow25 <= 0) { return 0; }
        double goal = lineFlowAtRatio(lines, mach, pc / (1 + used)) / flow25, now = parameter(fNow, t4Now);
        if (parameter(0, t3) / now <= goal) { return 0; }
        // the turbine's flow goes nearly as 1 / sqrt(T4): iterate T4 by that law
        double t4 = Math.clamp(t4Now / (goal * goal), t3 + 1e-6, 2400), f = 0;
        for (int k = 0; k < 12; k++) {
            f = GasPath.fuelAirRatio(core.fuel(), t3, t4, core.burnerEfficiency());
            double ratio = parameter(f, t4) / now / goal;
            if (Math.abs(ratio - 1) < 1e-9) { break; }
            t4 = Math.clamp(t4 * ratio * ratio, t3 + 1e-6, 2400);
        }
        return f * (1 - core.coolingBleed()) * coreFlow * goal;
    }

    /** Corrected HP compressor flow on the running line where its pressure ratio is pc. */
    static double lineFlowAtRatio(List<Line> lines, double mach, double pc) {
        Line l = lines.getFirst();
        double lo = 0, hi = l.rows()[l.rows().length - 1][C_HIGH] * 1.5;
        for (int it = 0; it < 50; it++) {
            double m = 0.5 * (lo + hi);
            if (at(lines, mach, C_HIGH, m)[C_PC] < pc) { lo = m; } else { hi = m; }
        }
        return at(lines, mach, C_HIGH, 0.5 * (lo + hi))[C_FLOW25];
    }
}
