package dev.mcmod.aerocore.api.gasdynamics;

import dev.mcmod.aerocore.api.engine.Gas;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class GasDynamicsTest {
    private static final double G = 1.4;
    private static final Thermo PERFECT = Thermo.PERFECT_AIR;

    private static void close(double expected, double actual, double relative, String what) {
        assertEquals(expected, actual, relative * Math.abs(expected), what);
    }

    @Test void isentropicRelationsMatchNacaReport1135() {
        // NACA Report 1135, Table I (gamma 1.4)
        close(1.8, PerfectGas.totalToStaticTemperature(2, G), 1e-12, "T0/T at M2");
        close(1 / 0.12780, PerfectGas.totalToStaticPressure(2, G), 2e-4, "p0/p at M2");
        close(1.6875, PerfectGas.areaRatio(2, G), 1e-4, "A/A* at M2");
        close(1 / 0.84302, PerfectGas.totalToStaticPressure(0.5, G), 2e-5, "p0/p at M0.5");
        close(1.3398, PerfectGas.areaRatio(0.5, G), 1e-4, "A/A* at M0.5");
        close(0.6847, PerfectGas.massFlowParameter(1, G), 1e-4, "mass-flow parameter at M1");
        close(0.5, PerfectGas.machAtAreaRatio(1.33984, G, false), 1e-4, "subsonic M at A/A* 1.3398");
        close(2, PerfectGas.machAtAreaRatio(1.6875, G, true), 1e-5, "supersonic M at A/A* 1.6875");
        close(2, PerfectGas.machAtPressureRatio(PerfectGas.totalToStaticPressure(2, G), G), 1e-12, "M from p0/p");
        // Prandtl-Meyer (Table II): 11.905 deg at M1.5, 26.380 at M2, 49.757 at M3
        close(11.905, Math.toDegrees(PerfectGas.prandtlMeyer(1.5, G)), 1e-4, "nu(1.5)");
        close(26.380, Math.toDegrees(PerfectGas.prandtlMeyer(2, G)), 1e-4, "nu(2)");
        close(49.757, Math.toDegrees(PerfectGas.prandtlMeyer(3, G)), 1e-4, "nu(3)");
        close(3, PerfectGas.machAtPrandtlMeyer(PerfectGas.prandtlMeyer(3, G), G), 1e-10, "M from nu");
        close(130.454, Math.toDegrees(PerfectGas.maxPrandtlMeyer(G)), 1e-4, "largest turn");
    }

    @Test void normalAndObliqueShocksMatchNacaReport1135() {
        NormalShock n2 = NormalShock.of(2, G);
        close(0.57735, n2.machBehind(), 1e-4, "M2 behind a M2 shock");
        close(4.5, n2.pressureRatio(), 1e-12, "p2/p1");
        close(1.6875, n2.temperatureRatio(), 1e-12, "T2/T1");
        close(2.6667, n2.densityRatio(), 1e-4, "rho2/rho1");
        close(0.72087, n2.totalPressureRatio(), 1e-4, "p02/p01");
        NormalShock n3 = NormalShock.of(3, G);
        close(0.47519, n3.machBehind(), 1e-4, "M behind a M3 shock");
        close(10.333, n3.pressureRatio(), 1e-4, "p2/p1 at M3");
        close(0.32834, n3.totalPressureRatio(), 1e-4, "p02/p01 at M3");
        // Chart 2 and the oblique-shock tables: M2 through 10 deg, and M3 through 20 deg
        ObliqueShock a = ObliqueShock.of(2, Math.toRadians(10), G, false);
        close(39.314, Math.toDegrees(a.waveAngle()), 2e-4, "wave angle M2 10 deg");
        close(1.7066, a.pressureRatio(), 3e-4, "p2/p1 M2 10 deg");
        close(1.6405, a.machBehind(), 3e-4, "M behind M2 10 deg");
        ObliqueShock b = ObliqueShock.of(3, Math.toRadians(20), G, false);
        close(37.76, Math.toDegrees(b.waveAngle()), 3e-4, "wave angle M3 20 deg");
        close(3.7713, b.pressureRatio(), 3e-4, "p2/p1 M3 20 deg");
        close(1.9941, b.machBehind(), 3e-4, "M behind M3 20 deg");
        double[] max = ObliqueShock.maxDeflection(2, G);
        close(22.973, Math.toDegrees(max[0]), 1e-4, "largest deflection at M2");
        assertTrue(ObliqueShock.of(2, Math.toRadians(10), G, true).waveAngle() > Math.toRadians(80), "the strong branch stands nearly normal");
        assertThrows(IllegalArgumentException.class, () -> ObliqueShock.of(2, Math.toRadians(25), G, false), "25 deg detaches at M2");
        // a zero turn is a Mach wave
        close(30, Math.toDegrees(ObliqueShock.of(2, 0, G, false).waveAngle()), 1e-6, "Mach angle at M2");
    }

    @Test void coneFlowMatchesPublishedTaylorMaccollSolutions() {
        // NASA Glenn NPARC validation archive "cone10" (Mach 2.35, 10 deg cone): surface Mach 2.1469. Its listed shock
        // angle (27.18 deg) and surface temperature (1.1063) do not conserve total temperature at gamma 1.4 and are not used.
        ConicalShock c = ConicalShock.of(2.35, Math.toRadians(10), G);
        close(2.1469, c.surfaceMach(), 2e-4, "surface Mach");
        close(PerfectGas.totalToStaticTemperature(2.35, G) / PerfectGas.totalToStaticTemperature(c.surfaceMach(), G), c.surfaceTemperatureRatio(), 1e-12,
                "total temperature kept to the surface");
        // the classic chart value: Mach 2, 10 deg cone, shock at 31.2 deg (Sims, NASA SP-3004; Anderson fig. 10.6)
        assertEquals(31.2, Math.toDegrees(ConicalShock.of(2, Math.toRadians(10), G).waveAngle()), 0.1, "shock angle M2 10 deg");
        // Mach 6, shock at 14.35 deg holds a 9.9978 deg cone (an independent RK4 solution, MATLAB Answers 1991953)
        assertEquals(9.9978, Math.toDegrees(ConicalShock.ofWaveAngle(6, Math.toRadians(14.35), G).coneHalfAngle()), 0.002, "cone at M6");
        // a cone deflects less than a wedge of the same angle: its shock is weaker
        assertTrue(c.waveAngle() < ObliqueShock.of(2.35, Math.toRadians(10), G, false).waveAngle());
        assertThrows(IllegalArgumentException.class, () -> ConicalShock.of(1.5, Math.toRadians(40), G), "detached");
    }

    @Test void rayleighAndFannoMatchNacaReport1135() {
        // Rayleigh (Table IV)
        close(0.79339, Rayleigh.totalTemperatureRatio(2, G), 1e-4, "Rayleigh T0/T0* M2");
        close(0.36364, Rayleigh.pressureRatio(2, G), 1e-4, "Rayleigh p/p* M2");
        close(0.52893, Rayleigh.temperatureRatio(2, G), 1e-4, "Rayleigh T/T* M2");
        close(1.5031, Rayleigh.totalPressureRatio(2, G), 1e-4, "Rayleigh p0/p0* M2");
        close(0.69136, Rayleigh.totalTemperatureRatio(0.5, G), 1e-4, "Rayleigh T0/T0* M0.5");
        close(1.1141, Rayleigh.totalPressureRatio(0.5, G), 1e-4, "Rayleigh p0/p0* M0.5");
        close(0.5, Rayleigh.machAtTotalTemperatureRatio(Rayleigh.totalTemperatureRatio(0.5, G), G, false), 1e-10, "subsonic inverse");
        close(2, Rayleigh.machAtTotalTemperatureRatio(Rayleigh.totalTemperatureRatio(2, G), G, true), 1e-10, "supersonic inverse");
        // Fanno (Table V)
        close(0.30499, Fanno.frictionLength(2, G), 1e-4, "Fanno 4fL*/D M2");
        close(0.40825, Fanno.pressureRatio(2, G), 1e-4, "Fanno p/p* M2");
        close(0.66667, Fanno.temperatureRatio(2, G), 1e-4, "Fanno T/T* M2");
        close(1.6875, Fanno.totalPressureRatio(2, G), 1e-4, "Fanno p0/p0* M2");
        close(1.0691, Fanno.frictionLength(0.5, G), 1e-4, "Fanno 4fL*/D M0.5");
        close(2.1381, Fanno.pressureRatio(0.5, G), 1e-4, "Fanno p/p* M0.5");
        close(0.5, Fanno.machAtFrictionLength(Fanno.frictionLength(0.5, G), G, false), 1e-10, "subsonic inverse");
        close(2, Fanno.machAtFrictionLength(Fanno.frictionLength(2, G), G, true), 1e-10, "supersonic inverse");
    }

    @Test void inletStartingLimits() {
        // the Kantrowitz limit tends to 0.600 for gamma 1.4 at high Mach (Van Wie, "Scramjet inlets", AIAA Progress 189)
        close(0.600, PerfectGas.kantrowitzContraction(1000, G), 1e-3, "Kantrowitz limit at high Mach");
        close(1 / (1.6875 * 0.72087), PerfectGas.kantrowitzContraction(2, G), 1e-4, "Kantrowitz at M2");
        for (double m : new double[]{1.5, 3, 6}) {
            assertTrue(PerfectGas.kantrowitzContraction(m, G) > PerfectGas.isentropicContraction(m, G), "a started inlet may contract more than it can start with");
        }
    }

    @Test void theThermalRoutinesWithAPerfectGasAreTheTables() {
        GasState ahead = new GasState(250, 50000, 2 * PERFECT.speedOfSound(250));
        GasState behind = NormalShock.behind(PERFECT, ahead);
        close(4.5, behind.pressurePa() / ahead.pressurePa(), 1e-9, "thermal normal shock p2/p1");
        close(1.6875, behind.temperatureK() / ahead.temperatureK(), 1e-9, "thermal normal shock T2/T1");
        ObliqueShock.Thermal o = ObliqueShock.behind(PERFECT, ahead, Math.toRadians(10));
        close(39.314, Math.toDegrees(o.waveAngle()), 2e-4, "thermal oblique wave angle");
        close(1.7066, o.behind().pressurePa() / ahead.pressurePa(), 3e-4, "thermal oblique p2/p1");
        double[] star = ThermalFlow.sonic(PERFECT, 300);
        close(250, star[0], 1e-9, "T* of a perfect gas");
        close(0.52828, star[1], 1e-4, "p*/pt");
        GasState m3 = ThermalFlow.turn(PERFECT, ahead, PerfectGas.prandtlMeyer(3, G) - PerfectGas.prandtlMeyer(2, G));
        close(3, m3.mach(PERFECT), 2e-5, "Prandtl-Meyer turn from M2 to M3");
        // the subsonic and supersonic states that carry the flux of A/A* = 1.6875
        double tt = 300, pt = 1e5, flux = star[2] * pt / 1.6875;
        close(2, ThermalFlow.atMassFlux(PERFECT, tt, pt, flux, true).mach(PERFECT), 1e-6, "supersonic branch");
        close(PerfectGas.machAtAreaRatio(1.6875, G, false), ThermalFlow.atMassFlux(PERFECT, tt, pt, flux, false).mach(PERFECT), 1e-6, "subsonic branch");
        close(2.5, ThermalFlow.atMach(PERFECT, tt, pt, 2.5).mach(PERFECT), 1e-9, "atMach");
    }

    @Test void realAirShocksConserveMassMomentumAndEnergyAndDepartFromTheTablesWhenHot() {
        Thermo air = Gas.AIR;
        for (double mach : new double[]{1.5, 3, 6, 10}) {
            GasState a = new GasState(220, 20000, mach * air.speedOfSound(220));
            GasState b = NormalShock.behind(air, a);
            double m1 = a.massFlux(air), m2 = b.massFlux(air);
            close(m1, m2, 1e-9, "mass at M" + mach);
            close(a.impulsePerArea(air), b.impulsePerArea(air), 1e-9, "momentum at M" + mach);
            close(air.enthalpy(a.temperatureK()) + 0.5 * a.speedMps() * a.speedMps(), air.enthalpy(b.temperatureK()) + 0.5 * b.speedMps() * b.speedMps(),
                    1e-9, "energy at M" + mach);
            NormalShock table = NormalShock.of(mach, air.gamma(220));
            double tRatio = b.temperatureK() / a.temperatureK();
            if (mach <= 1.5) { close(table.temperatureRatio(), tRatio, 5e-3, "a weak shock is the table's"); }
            // a hot shock layer takes energy into vibration: cooler and denser than the table says
            if (mach >= 6) {
                assertTrue(tRatio < (mach >= 10 ? 0.9 : 0.98) * table.temperatureRatio(), "T2/T1 below the perfect gas's at M" + mach);
                assertTrue(b.density(air) / a.density(air) > table.densityRatio(), "denser than the perfect gas's at M" + mach);
            }
        }
    }

    @Test void theDuctMarchReproducesIsentropicRayleighAndFannoFlow() {
        double t = 288, p = 1e5;
        // area change only, subsonic: A/A* 2.0351 (M 0.3) to 1.2
        double aIn = PerfectGas.areaRatio(0.3, G);
        QuasiOneD.Result n = QuasiOneD.march(PERFECT, QuasiOneD.Segment.nozzle(1, aIn, 1.2), state(0.3, t, p), 400);
        close(PerfectGas.machAtAreaRatio(1.2, G, false), n.exit().mach(PERFECT), 1e-6, "subsonic nozzle");
        assertFalse(n.choked());
        // supersonic expansion from M1.5 to A/A* = 2
        QuasiOneD.Result s = QuasiOneD.march(PERFECT, QuasiOneD.Segment.nozzle(1, PerfectGas.areaRatio(1.5, G), 2), state(1.5, t, p), 400);
        close(PerfectGas.machAtAreaRatio(2, G, true), s.exit().mach(PERFECT), 1e-6, "supersonic nozzle");
        // heat only: M0.3 to M0.6 by Rayleigh
        double cp = PERFECT.cp(t), tt = t * PerfectGas.totalToStaticTemperature(0.3, G);
        double q = cp * tt * (Rayleigh.totalTemperatureRatio(0.6, G) / Rayleigh.totalTemperatureRatio(0.3, G) - 1);
        QuasiOneD.Result r = QuasiOneD.march(PERFECT, QuasiOneD.Segment.heated(1, 0.1, q), state(0.3, t, p), 400);
        close(0.6, r.exit().mach(PERFECT), 1e-6, "Rayleigh heating");
        close(Rayleigh.pressureRatio(0.6, G) / Rayleigh.pressureRatio(0.3, G), r.exit().pressurePa() / p, 1e-6, "Rayleigh pressure");
        // heat beyond T0*: thermal choking inside the duct
        double qChoke = cp * tt * (1 / Rayleigh.totalTemperatureRatio(0.3, G) - 1);
        QuasiOneD.Result c = QuasiOneD.march(PERFECT, QuasiOneD.Segment.heated(1, 0.1, 1.2 * qChoke), state(0.3, t, p), 400);
        assertTrue(c.choked(), "heated past T0* the flow chokes");
        assertEquals(1 / 1.2, c.chokedAtM(), 0.01, "where the stream reached T0*");
        assertEquals(1, c.exit().mach(PERFECT), 0.02, "choked at Mach 1");
        // friction only: M0.4 to M0.7 and M2.5 to M1.8 by Fanno (f 0.005, D 0.05 m)
        double f = 0.005, d = 0.05;
        double lSub = (Fanno.frictionLength(0.4, G) - Fanno.frictionLength(0.7, G)) * d / (4 * f);
        close(0.7, QuasiOneD.march(PERFECT, QuasiOneD.Segment.rough(lSub, 0.002, f, d), state(0.4, t, p), 400).exit().mach(PERFECT), 1e-6, "subsonic Fanno");
        double lSup = (Fanno.frictionLength(2.5, G) - Fanno.frictionLength(1.8, G)) * d / (4 * f);
        close(1.8, QuasiOneD.march(PERFECT, QuasiOneD.Segment.rough(lSup, 0.002, f, d), state(2.5, t, p), 400).exit().mach(PERFECT), 1e-6, "supersonic Fanno");
    }

    @Test void injectionAndRealGasHeatingConserveMassMomentumAndEnergy() {
        // constant area, 20 % mass added with no axial momentum and the stream's own total enthalpy: rho V and p + rho V^2 by the integrals
        Thermo air = Gas.AIR;
        GasState in = state(air, 0.3, 400, 2e5);
        QuasiOneD.Result r = QuasiOneD.march(air, new QuasiOneD.Segment(1, 0.05, 0.05, 0, 0, Double.NaN, 0.2, 0), in, 800);
        close(1.2 * in.massFlux(air), r.exit().massFlux(air), 1e-6, "mass with injection");
        close(in.impulsePerArea(air), r.exit().impulsePerArea(air), 1e-6, "momentum with injection (none brought in)");
        close(air.enthalpy(in.temperatureK()) + 0.5 * in.speedMps() * in.speedMps(),
                air.enthalpy(r.exit().temperatureK()) + 0.5 * r.exit().speedMps() * r.exit().speedMps(), 1e-6, "total enthalpy with injection");
        // hot air heated through a diverging duct, supersonic (a scramjet combustor's case)
        GasState hot = state(air, 2.5, 1100, 1e5);
        double q = 6e5;
        QuasiOneD.Result h = QuasiOneD.march(air, new QuasiOneD.Segment(1, 0.05, 0.08, q, 0, Double.NaN, 0, 0), hot, 800);
        assertFalse(h.choked());
        close(air.enthalpy(hot.temperatureK()) + 0.5 * hot.speedMps() * hot.speedMps() + q,
                air.enthalpy(h.exit().temperatureK()) + 0.5 * h.exit().speedMps() * h.exit().speedMps(), 1e-6, "energy of the heated stream");
        close(hot.massFlux(air) * 0.05, h.exit().massFlux(air) * 0.08, 1e-6, "mass through the diverging duct");
        // step convergence: 100 and 800 steps agree to well within the table precision
        QuasiOneD.Result coarse = QuasiOneD.march(air, new QuasiOneD.Segment(1, 0.05, 0.08, q, 0, Double.NaN, 0, 0), hot, 100);
        close(h.exit().mach(air), coarse.exit().mach(air), 1e-6, "converged in steps");
    }

    private static GasState state(double mach, double t, double p) { return new GasState(t, p, mach * PERFECT.speedOfSound(t)); }
    private static GasState state(Thermo gas, double mach, double t, double p) { return new GasState(t, p, mach * gas.speedOfSound(t)); }
}
