package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.body.BodyModule;
import dev.mcmod.aerocore.api.body.BodyShape;
import dev.mcmod.aerocore.api.body.RoadAero;
import dev.mcmod.aerocore.api.body.SectionShape;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.BodyPart;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** A body carrying an aero kit (0.34): its floor finds the ground below and its loads join the body's. */
class RoadAeroBodyTest {
    private static final double DT = 0.005;
    private static final SectionShape CELL = new SectionShape.Cabin(1.8, 1.3, 0.15, 0.6, 0.72, 0.15);
    private static final BodyShape SHAPE = new BodyShape(CELL.scaled(0.6), List.of(BodyModule.straight("cell", 4.5, CELL)));
    private static final RoadAero KIT = new RoadAero(List.of(
            new RoadAero.Underbody(4.2, 0.3, 1.6, 0.15, 0.6, 10, 0.5, 0.04, 0.06, 1.6, true),
            new RoadAero.RearWing(new Vec3d(0.3, 1.6, 0), 1.4, 0.3, 8, 0.06, 0.12, 0.15, 1.6)));

    private static PartDesign body(RoadAero kit) {
        MassProperties m = MassShapes.box(new Vec3d(4.5, 1.3, 1.8), 1).scaled(1200 / MassShapes.box(new Vec3d(4.5, 1.3, 1.8), 1).massKg()).at(new Vec3d(2.25, 0.7, 0));
        return new TestParts.Fixture("body", "body " + (kit != null), d -> new BodyPart(d, SHAPE, 0, BodyPart.Hydro.AUTO, 8, m, Map.of(), List.of(),
                List.of(), List.of(), null, kit));
    }

    /** One step at 30 m/s with the body's floor 0.12 m over flat ground, no gravity: the diagnostics and the result. */
    private static VehicleDynamics.Result step(RoadAero kit, VehicleState[] out) {
        Vehicle v = new VehicleBlueprint("car", List.of(new VehicleBlueprint.Entry("body", body(kit), Mount.at(Vec3d.ZERO))), List.of()).compile();
        VehicleState st = v.newState(new Vec3d(0, 0.12 - 0.15, 0), Orientation.IDENTITY, DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(30, 0, 0), st.body().orientation(), Vec3d.ZERO));
        out[0] = st;
        VehicleEnvironment env = VehicleEnvironment.land(Ground.flat(0, GroundContact.Surface.TARMAC), Atmosphere.atAltitude(0, Vec3d.ZERO).fluid());
        return VehicleDynamics.step(v, st, env, Map.of(), Vec3d.ZERO, List.of(), null, 0, DT);
    }

    @Test void theKitPullsTheBodyDownAtSpeed() {
        VehicleState[] plain = new VehicleState[1], kitted = new VehicleState[1];
        step(null, plain);
        step(KIT, kitted);
        Map<String, Double> p = plain[0].diagnostics().get("body"), k = kitted[0].diagnostics().get("body");
        assertFalse(p.containsKey("kitDownforceN"), "a body without a kit reports nothing new");
        assertEquals(p.get("airDragN"), k.get("airDragN"), 0, "the shape's own loads are the same");

        double rho = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid().densityKgPerM3();
        RoadAero.Loads direct = KIT.loads(new Vec3d(-30, 0, 0), rho, x -> 0.12);
        assertEquals(direct.downforceN(), k.get("kitDownforceN"), 1e-6 * direct.downforceN(), "the floor found the ground 0.12 m below");
        assertTrue(direct.downforceN() > 400, "a kit at 30 m/s: some half a kilonewton (0.35.1 calibration): " + direct.downforceN());
        assertTrue(kitted[0].body().velocity().y() < plain[0].body().velocity().y() - 1e-3, "pulled down in the step");
    }
}
