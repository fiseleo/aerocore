package dev.mcmod.aerocore.api.propellant;

import dev.mcmod.aerocore.api.storage.PressureVessel;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RocketTest {
    private static final double G0 = 9.80665, SEA = 101325;

    @Test void knsbBurnsAsNakkaMeasuredAndHasItsCharacteristicVelocity() {
        // "KNDX and KNSB propellants - burn rate experimentation": 0.102 in/s at 14.7 psia, 0.443 in/s at 1000 psia
        assertEquals(0.102 * 25.4, Propellant.KNSB.burnRate(101325) * 1e3, 0.03 * 0.102 * 25.4, "at 1 atm");
        assertEquals(0.443 * 25.4, Propellant.KNSB.burnRate(6.895e6) * 1e3, 0.03 * 0.443 * 25.4, "at 1000 psia");
        // the plateau: nearly no change with pressure from 1.5 to 3.8 MPa
        assertEquals(Propellant.KNSB.burnRate(1.6e6), Propellant.KNSB.burnRate(3.7e6), 0.02 * Propellant.KNSB.burnRate(1.6e6), "mesa");
        // c* about 909 m/s (Nakka's KNSB technical sheet)
        assertEquals(909, Propellant.KNSB.characteristicVelocity(), 0.01 * 909, "KNSB c*");
    }

    @Test void apcpInTheShuttleBoostersNozzleDeliversItsImpulse() {
        // Shuttle SRB: expansion 7.72, about 625 psia (4.3 MPa) through the main burn; 242 s at sea level, 268 s in vacuum
        double c = 0.98 * Propellant.APCP.characteristicVelocity(), cf = SolidRocket.vacuumThrustCoefficient(Propellant.APCP.gamma(), 7.72);
        double lambda = 0.98;   // a contoured nozzle loses about 2 % to divergence
        double momentum = cf - 7.72 * pressureRatio(Propellant.APCP.gamma(), 7.72);
        double vacuum = c * (lambda * momentum + 7.72 * pressureRatio(Propellant.APCP.gamma(), 7.72)) / G0;
        assertEquals(268.6, vacuum, 0.05 * 268.6, "vacuum Isp");
        double sea = vacuum - SEA / 4.3e6 * 7.72 * c / G0;
        assertEquals(242, sea, 0.05 * 242, "sea-level Isp");
        // the TP-H1148 burn rate: 0.368 in/s at 625 psia
        assertEquals(0.368 * 25.4, Propellant.APCP.burnRate(4.309e6) * 1e3, 0.01 * 0.368 * 25.4, "burn rate");
    }

    private static double pressureRatio(double g, double eps) {
        double m = dev.mcmod.aerocore.api.gasdynamics.PerfectGas.machAtAreaRatio(eps, g, true);
        return 1 / dev.mcmod.aerocore.api.gasdynamics.PerfectGas.totalToStaticPressure(m, g);
    }

    @Test void anEndBurnerBurnsFlatAndAddsUpItsMass() {
        SolidRocket.Grain g = new SolidRocket.Grain(SolidRocket.GrainKind.END_BURNER, 0.1, 0, 0.3, 1);
        SolidRocket.Design d = new SolidRocket.Design(Propellant.APCP, g, 0.006, 6, Math.toRadians(12), 0.95);
        SolidRocket.Performance p = SolidRocket.perform(d);
        double pc = p.chamberPressurePa()[0];
        for (double x : p.chamberPressurePa()) { assertEquals(pc, x, 1e-9 * pc, "constant pressure"); }
        assertEquals(0.3 / Propellant.APCP.burnRate(pc), p.burnTimeS(), 1e-9 * p.burnTimeS(), "burn time = length / rate");
        assertEquals(p.propellantMassKg(), burned(p), 1e-6 * p.propellantMassKg(), "mass flow adds up to the grain");
        // the impulse into vacuum is thrust times time, and the sea-level impulse loses p_a A_e for the whole burn
        double fv = SolidRocket.thrust(d, pc, 0);
        assertEquals(fv * p.burnTimeS(), p.totalImpulse(0), 1e-6 * fv * p.burnTimeS());
        assertEquals(p.totalImpulse(0) - SEA * d.exitArea() * p.burnTimeS(), p.totalImpulse(SEA), 1e-6 * p.totalImpulse(0));
    }

    @Test void aBatesGrainOfTheClassicProportionsBurnsNearlyNeutral() {
        // segment length (3 D + d) / 2 keeps the burning area within about 12 % (Nakka, "Solid rocket motor theory: grain design")
        SolidRocket.Grain g = new SolidRocket.Grain(SolidRocket.GrainKind.BATES, 0.1, 0.04, 0.17, 3);
        SolidRocket.Design d = new SolidRocket.Design(Propellant.KNSB, g, 0.022, 8, Math.toRadians(12), 0.95);
        SolidRocket.Performance p = SolidRocket.perform(d);
        double max = 0, min = Double.MAX_VALUE, pMax = 0;
        for (int i = 0; i < SolidRocket.STEPS; i++) {
            double ab = g.burningArea(g.web() * i / SolidRocket.STEPS);
            max = Math.max(max, ab);
            min = Math.min(min, ab);
            pMax = Math.max(pMax, p.chamberPressurePa()[i]);
        }
        assertTrue(max / min < 1.15, "neutral: " + max / min);
        assertEquals(pMax, p.maxPressurePa(), 1e-9 * pMax);
        assertEquals(p.propellantMassKg(), burned(p), 1e-4 * p.propellantMassKg(), "mass flow adds up to the grain");
        // a KNSB motor of this kind delivers 115-145 s at sea level (Nakka's static tests deliver about 120-140 s)
        double isp = p.specificImpulse(SEA);
        assertTrue(isp > 115 && isp < 145, "delivered Isp " + isp + " s at " + pMax / 1e6 + " MPa");
        // a tube burns progressively (its core grows)
        SolidRocket.Performance tube = SolidRocket.perform(new SolidRocket.Design(Propellant.APCP,
                new SolidRocket.Grain(SolidRocket.GrainKind.TUBE, 0.1, 0.03, 0.5, 1), 0.02, 6, Math.toRadians(12), 0.95));
        assertTrue(tube.chamberPressurePa()[SolidRocket.STEPS - 1] > 1.5 * tube.chamberPressurePa()[0], "progressive tube");
    }

    @Test void anOverexpandedNozzleSeparatesAndStillPushes() {
        SolidRocket.Design d = new SolidRocket.Design(Propellant.KNSB, new SolidRocket.Grain(SolidRocket.GrainKind.END_BURNER, 0.1, 0, 0.2, 1),
                0.02, 40, Math.toRadians(12), 0.95);
        double pc = 2e6, attached = SolidRocket.thrust(d, pc, 0) - SEA * d.exitArea();
        assertTrue(SolidRocket.thrust(d, pc, SEA) > attached, "separation cuts the overexpansion loss");
        assertTrue(SolidRocket.thrust(d, pc, SEA) > 0);
        // a steel case for a 7 MPa burst: wall p r / sigma
        assertEquals(7e6 * 0.05 / 400e6, PressureVessel.metalCylinderWallM(7e6, 0.05, 400e6), 1e-15);
        assertEquals(7e6, PressureVessel.metalCylinderBurstPa(7e6 * 0.05 / 400e6, 0.05, 400e6), 1e-6);
    }

    private static double burned(SolidRocket.Performance p) {
        double m = 0;
        for (int i = 1; i < p.timeS().length; i++) { m += 0.5 * (p.massFlowKgPerS()[i - 1] + p.massFlowKgPerS()[i]) * (p.timeS()[i] - p.timeS()[i - 1]); }
        return m;
    }
}
