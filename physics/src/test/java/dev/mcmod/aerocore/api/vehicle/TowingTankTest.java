package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.body.BodyModule;
import dev.mcmod.aerocore.api.body.BodyShape;
import dev.mcmod.aerocore.api.body.SectionShape;
import dev.mcmod.aerocore.api.vehicle.part.BodyPart;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class TowingTankTest {
    private static final Vec3d GRAVITY = new Vec3d(0, -9.80665, 0);
    private static final FluidState WATER = FluidState.FRESH_WATER_20C, AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();

    /** A 10 x 3 x 2 m box barge floating 0.8 m deep, its centre of mass at half its depth. */
    private static Vehicle barge() {
        SectionShape box = new SectionShape.Chined(3, 2, 0, 0, 0);
        BodyShape shape = new BodyShape(box, List.of(BodyModule.straight("box", 10, box)));
        double mass = 10 * 3 * 0.8 * WATER.densityKgPerM3();
        return new VehicleBlueprint("barge", List.of(new VehicleBlueprint.Entry("hull", TestParts.body("barge", shape, mass, BodyPart.Hydro.DISPLACEMENT),
                Mount.at(Vec3d.ZERO), List.of())), List.of()).compile();
    }

    @Test void aBoxBargesRightingArmsFollowTheWallSidedFormula() {
        // KB = T/2 = 0.4, BM = B^2 / 12T = 0.9375, KG = 1: GM 0.3375; wall-sided GZ = sin(phi) (GM + BM tan^2(phi) / 2)
        // until the bilge leaves the water (28 deg)
        double gm = 0.4 + 9 / 9.6 - 1, bm = 9 / 9.6;
        List<TowingTank.Heel> heels = TowingTank.heel(barge(), WATER, AIR, GRAVITY, 5, 10, 20);
        for (TowingTank.Heel h : heels) {
            double phi = Math.toRadians(h.heelDeg()), expected = Math.sin(phi) * (gm + 0.5 * bm * Math.pow(Math.tan(phi), 2));
            assertEquals(expected, h.gzM(), 0.03 * expected + 0.002, "GZ at " + h.heelDeg() + " deg: " + heels);
        }
    }

    /** A 12 m round-bilged hull (elliptical sections 2 m wide, 1.6 m deep, tapering to points), floating at half its volume. */
    private static Vehicle slender() {
        SectionShape full = SectionShape.Rounded.ellipse(2, 1.6, 0), end = full.scaled(0.05);
        BodyShape shape = new BodyShape(end, List.of(new BodyModule("fore", 4, full, 2, 0, Double.POSITIVE_INFINITY), BodyModule.straight("middle", 3, full),
                new BodyModule("aft", 5, end.raised(0.5), 2, 0, Double.POSITIVE_INFINITY)));
        double mass = 0.5 * shape.particulars().volumeM3() * WATER.densityKgPerM3();
        return new VehicleBlueprint("slender", List.of(new VehicleBlueprint.Entry("hull", TestParts.body("slender", shape, mass, BodyPart.Hydro.DISPLACEMENT),
                Mount.at(Vec3d.ZERO), List.of())), List.of()).compile();
    }

    @Test void aTowedHullRunsSteadilyItsResistanceRisingWithSpeed() {
        Vehicle v = slender();
        List<TowingTank.Run> runs = TowingTank.tow(v, WATER, AIR, GRAVITY, 0.5, 1, 2, 3);
        double before = 0;
        for (TowingTank.Run r : runs) {
            assertTrue(!r.lost() && r.trimSwingDeg() < 0.2, "steady: " + runs);
            assertTrue(r.resistanceN() > before, "rising: " + runs);
            assertTrue(r.airN() > 0 && r.airN() < 0.2 * r.resistanceN(), "mostly water: " + runs);
            before = r.resistanceN();
        }
        // what it measures is what the hull's own model gives (HullDynamics' resistance) plus the body's air drag
        VehicleState st = v.newState(Vec3d.ZERO, dev.mcmod.aerocore.api.Orientation.IDENTITY, TowingTank.DT);
        dev.mcmod.aerocore.api.RigidBodyState rest = TowingTank.rest(v, WATER, AIR, GRAVITY);
        st.setBody(new dev.mcmod.aerocore.api.RigidBodyState(rest.position(), new Vec3d(1, 0, 0), rest.orientation(), Vec3d.ZERO));
        VehicleDynamics.Result r = null;
        for (int i = 0; i < TowingTank.RUN_S / TowingTank.DT; i++) {
            r = VehicleDynamics.step(v, st, VehicleEnvironment.calm(0, WATER, AIR), java.util.Map.of(), GRAVITY, List.of(), null, i * TowingTank.DT, TowingTank.DT);
            var s = st.body();
            st.setBody(new dev.mcmod.aerocore.api.RigidBodyState(new Vec3d(s.position().x(), s.position().y(), 0), new Vec3d(1, s.velocity().y(), 0), s.orientation(), Vec3d.ZERO));
        }
        var hull = r.diagnostics().get("hull");
        assertEquals(hull.get("resistanceN") + hull.get("airDragN"), runs.get(1).resistanceN(), 0.01 * runs.get(1).resistanceN(), runs.get(1) + " / " + hull);
    }
}
