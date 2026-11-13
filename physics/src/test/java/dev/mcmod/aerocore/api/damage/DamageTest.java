package dev.mcmod.aerocore.api.damage;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** 0.42: the damage calculations against hand sums and the figures they are named after. */
class DamageTest {
    @Test void aHitLeavesTheKineticEnergyAboveTheElasticSpeed() {
        // 1400 kg stopping dead from 50 km/h: J = m v, E = m v^2 / 2 less the 4 km/h it springs back from
        double m = 1400, v = 50 / 3.6, v0 = 4 / 3.6;
        assertEquals(0.5 * m * (v * v - v0 * v0), ImpactDamage.energyJ(m * v, v), 1e-6);
        assertEquals(0.5 * m * v * v, ImpactDamage.energyJ(m * v, v, 0), 1e-6);
        // a parking nudge does nothing
        assertEquals(0, ImpactDamage.energyJ(m * 1.0, 1.0), 0);
        assertEquals(0, ImpactDamage.energyJ(0, 0), 0);
        assertThrows(IllegalArgumentException.class, () -> ImpactDamage.energyJ(-1, 3));
    }

    @Test void theEnergyGoesToWhatIsNearAndHeavy() {
        ImpactDamage.Target body = new ImpactDamage.Target(new Vec3d(-2, 0, 0), 350, ImpactDamage.SHELL_STEEL_J_PER_KG, 2);
        ImpactDamage.Target engine = new ImpactDamage.Target(new Vec3d(-0.8, 0, 0), 150, 1000, 0.4);
        ImpactDamage.Target rearAxle = new ImpactDamage.Target(new Vec3d(-3.5, 0, 0), 80, 1000, 0.5);
        List<ImpactDamage.Target> car = List.of(body, engine, rearAxle);
        double[] front = ImpactDamage.shares(Vec3d.ZERO, car);
        assertEquals(1, front[0] + front[1] + front[2], 1e-12);
        // by hand: 350 / 2 = 175, 150 / 5 = 30, 80 / 50 = 1.6
        assertEquals(175 / 206.6, front[0], 1e-9);
        assertEquals(30 / 206.6, front[1], 1e-9);
        assertEquals(1.6 / 206.6, front[2], 1e-9);
        // hit from behind, the rear axle takes far more than the engine
        double[] rear = ImpactDamage.shares(new Vec3d(-4.2, 0, 0), car);
        assertTrue(rear[2] > 5 * rear[1]);
        // damage: the share of the energy over what the part absorbs
        double e = 100e3;
        double[] damage = ImpactDamage.damage(e, Vec3d.ZERO, car);
        assertEquals(e * front[0] / (ImpactDamage.SHELL_STEEL_J_PER_KG * 350), damage[0], 1e-12);
        assertEquals(e * front[1] / (1000 * 150), damage[1], 1e-12);
        assertEquals(0, ImpactDamage.shares(Vec3d.ZERO, List.of()).length);
        // the whole-car figure: the 64 km/h offset test uses up about a quarter of a 330 kg shell
        double ncap = 0.5 * 1400 * Math.pow(64 / 3.6, 2);
        assertEquals(0.25, ncap / (ImpactDamage.SHELL_STEEL_J_PER_KG * 330), 0.02);
    }

    @Test void aBlastGivesTheEnergyOfItsImpulse() {
        // 500 Pa s on 4 m^2 of a 1000 kg car: 2000 N s, 2 m/s, 2 kJ
        assertEquals(2000, ImpactDamage.blastEnergyJ(500, 4, 1000), 1e-9);
    }

    @Test void liningsAndTreadsWearWithTheWorkDoneOnThem() {
        // one stop of a 1400 kg car from 100 km/h is 540 kJ; four pads of 50 cm^2 with 10 mm to wear hold 200 cm^3
        double stop = 0.5 * 1400 * Math.pow(100 / 3.6, 2), pads = 4 * 50e-4 * 10e-3;
        double perStop = Wear.lining(stop, Wear.BRAKE_LINING_M3_PER_J, pads);
        assertEquals(stop * 20e-15 / 200e-6, perStop, 1e-15);
        // tens of thousands of such stops in a set of pads
        assertTrue(1 / perStop > 10_000 && 1 / perStop < 40_000, "stops in a set of pads: " + 1 / perStop);
        // hot linings go faster: nothing below the knee, doubled one step above it, bounded far above
        assertEquals(1, Wear.liningHeat(400, 500, 60), 0);
        assertEquals(2, Wear.liningHeat(560, 500, 60), 1e-12);
        assertTrue(Double.isFinite(Wear.liningHeat(1e9, 500, 60)));
        // a tyre: 1.76 litres of tread, 2 kJ of slip work a km at mild severity: tens of thousands of km
        double perKm = Wear.tread(2000, Wear.TREAD_M3_PER_J, Wear.treadSeverity(0.2, 3), 1.76e-3);
        assertTrue(1 / perKm > 20_000 && 1 / perKm < 80_000, "km in a tread: " + 1 / perKm);
        // a locked wheel at 30 m/s: a hundred times the rubber per J
        assertEquals(101, Wear.treadSeverity(30, 3), 1e-9);
        assertEquals(101, Wear.treadSeverity(-30, 3), 1e-9);
    }

    @Test void machinesUseTheirLivesFasterWhenPushed() {
        // an hour of a 5000 h life at rated speed and load
        assertEquals(1.0 / 5000, Wear.running(3600, 5000 * 3600.0, 1, 2, 1, 1), 1e-15);
        // 20 % over speed (squared) at 1.5 times the load
        assertEquals(1.44 * 1.5 / 5000, Wear.running(3600, 5000 * 3600.0, 1.2, 2, 1.5, 1), 1e-15);
        // a ball bearing at twice its load lasts an eighth (ISO 281)
        assertEquals(8, Wear.running(1, 1, 1, 1, 2, 3), 1e-12);
        assertEquals(0, Wear.running(3600, 5000 * 3600.0, 0, 2, 1, 1), 0);
    }

    @Test void hotPartsCreepByLarsonMiller() {
        // the rated point gives the rated life
        assertEquals(5000, Wear.creepLifeHours(5000, 1200, 1200, Wear.LARSON_MILLER_C), 1e-6);
        // by hand: log10 t = (1200 / 1220) (20 + log10 5000) - 20 = 3.3105
        assertEquals(Math.pow(10, 1200.0 / 1220 * (20 + Math.log10(5000)) - 20), Wear.creepLifeHours(5000, 1200, 1220, 20), 1e-6);
        // about half the life for 15 to 20 K more
        double ratio = Wear.creepLifeHours(5000, 1200, 1217, 20) / 5000;
        assertEquals(0.5, ratio, 0.05);
        // an hour at the rated temperature
        assertEquals(1.0 / 5000, Wear.creep(3600, 5000, 1200, 1200, 20), 1e-12);
        // colder: far longer
        assertTrue(Wear.creepLifeHours(5000, 1200, 1100, 20) > 100 * 5000);
    }

    @Test void fatigueAddsUpByMinersRule() {
        // the reference point: one life
        assertEquals(1, Wear.fatigue(1e6, 200, 100, 200, 1e6, 4), 1e-12);
        // twice the stress with exponent 4: a sixteenth of the life
        assertEquals(16, Wear.fatigue(1e6, 400, 100, 200, 1e6, 4), 1e-9);
        // below the endurance limit: nothing
        assertEquals(0, Wear.fatigue(1e9, 90, 100, 200, 1e6, 4), 0);
        assertEquals(Wear.fatigue(10, 300, 100, 200, 1e6, 4), Wear.fatigue(10, -300, 100, 200, 1e6, 4), 0);
    }

    @Test void metalsWasteAtTheHandbookRates() {
        // ISO 9223: carbon steel inland up to 25 um/year, in sea air 80 to 200
        double air = Corrosion.rateMmPerYear(Corrosion.Metal.CARBON_STEEL, Corrosion.Exposure.AIR);
        double seaAir = Corrosion.rateMmPerYear(Corrosion.Metal.CARBON_STEEL, Corrosion.Exposure.SEA_AIR);
        assertTrue(air > 0.0013 && air <= 0.025);
        assertTrue(seaAir >= 0.08 && seaAir <= 0.2);
        // immersed in the sea 0.1 to 0.15 mm/year, more in the splash zone
        double sea = Corrosion.rateMmPerYear(Corrosion.Metal.CARBON_STEEL, Corrosion.Exposure.SEA_WATER);
        assertTrue(sea >= 0.1 && sea <= 0.15);
        assertTrue(Corrosion.rateMmPerYear(Corrosion.Metal.CARBON_STEEL, Corrosion.Exposure.SEA_SPLASH) >= 2 * sea);
        // every metal: worse in the sea than in dry air; the passive metals far below steel; nothing rusts composites
        for (Corrosion.Metal m : Corrosion.Metal.values()) {
            assertTrue(Corrosion.rateMmPerYear(m, Corrosion.Exposure.SEA_WATER) >= Corrosion.rateMmPerYear(m, Corrosion.Exposure.DRY), m.name());
            for (Corrosion.Exposure e : Corrosion.Exposure.values()) { assertTrue(Corrosion.rateMmPerYear(m, e) >= 0); }
        }
        assertTrue(Corrosion.rateMmPerYear(Corrosion.Metal.ALUMINIUM, Corrosion.Exposure.SEA_WATER) < 0.2 * sea);
        assertTrue(Corrosion.rateMmPerYear(Corrosion.Metal.MAGNESIUM, Corrosion.Exposure.SEA_WATER) > 5 * sea);
        assertEquals(0, Corrosion.rateMmPerYear(Corrosion.Metal.NONE, Corrosion.Exposure.SEA_SPLASH), 0);
        // paint: a sound system lets a few per cent through, a lost one everything
        assertEquals(0.12 * 0.03, Corrosion.coated(0.12, 1, 0.03), 1e-15);
        assertEquals(0.12, Corrosion.coated(0.12, 0, 0.03), 0);
        assertEquals(0.12 * (1 - 0.5 * 0.97), Corrosion.coated(0.12, 0.5, 0.03), 1e-15);
        // a 1 mm panel that may lose 30 %: bare in the sea it is done in two and a half years
        assertEquals(1, Corrosion.used(0.12, 2.5 * Corrosion.YEAR_S, 1, 0.3), 1e-12);
    }

    @Test void theRiskToThoseAboardFollowsTheChangeOfVelocity() {
        assertEquals(10, Casualty.deltaV(14000, 1400), 0);
        // the curve passes through its two anchors
        assertEquals(0.5, Casualty.seriousRisk(63 / 3.6), 1e-12);
        assertEquals(0.1, Casualty.seriousRisk(38 / 3.6), 1e-12);
        // a parking bump is harmless, a motorway crash into a wall all but certain; the sign does not matter
        assertTrue(Casualty.seriousRisk(10 / 3.6) < 0.02);
        assertTrue(Casualty.seriousRisk(110 / 3.6) > 0.95);
        assertEquals(Casualty.seriousRisk(15), Casualty.seriousRisk(-15), 0);
        assertThrows(IllegalArgumentException.class, () -> Casualty.seriousRisk(10, 5, 8));
    }

    @Test void aHoleBelowTheWaterlineFloods() {
        // 1 dm^2 half a metre down in fresh water: 0.6 x 0.01 x sqrt(2 x 9.81 x 0.5) x 1000 = 18.8 kg/s
        assertEquals(0.6 * 0.01 * Math.sqrt(9.81) * 1000, Casualty.inflowKgPerS(0.01, 0.5, 9.81, 1000), 1e-9);
        assertEquals(0, Casualty.inflowKgPerS(0.01, -0.2, 9.81, 1000), 0);
        assertEquals(0, Casualty.inflowKgPerS(0, 2, 9.81, 1025), 0);
    }
}
