package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.earthmoving.Cut;
import dev.mcmod.aerocore.api.earthmoving.Digging;
import dev.mcmod.aerocore.api.earthmoving.Material;
import dev.mcmod.aerocore.api.ground.Terrain;
import dev.mcmod.aerocore.api.vehicle.part.LinkPart;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** 0.43: tools cutting the ground (Part.Tool, ToolCutter through LinkPart, VehicleState.takeCuts). */
class ToolTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final double RAKE = Math.toRadians(55);

    /** A 20 t sled carrying a 3 m blade whose edge is 0.3 m under its frame's origin, cutting along +x, its cut into hold (or none). */
    private static Vehicle sled(String hold) {
        Part.Tool edge = new Part.Tool("edge", new Vec3d(0, 0, -1.5), new Vec3d(0, 0, 1.5), Vec3d.X, 1.2, RAKE, Part.Tool.Kind.EDGE, hold);
        Part.Joint fixed = Part.Joint.revolute(Vec3d.ZERO, Vec3d.Z, -0.1, 0.1, 0);
        MassProperties m = MassProperties.point(10, Vec3d.ZERO);
        List<Part.Hold> holds = hold.isEmpty() ? List.of() : List.of(new Part.Hold(hold, 2, Vec3d.ZERO));
        PartDesign blade = new TestParts.Fixture("blade", "blade " + hold, d -> new LinkPart(d, m, List.of(), Map.of(), fixed, null, holds, List.of(), List.of(), List.of(),
                List.of(edge), null));
        PartDesign body = TestParts.solid("body", new Vec3d(4, 1, 3), 20000, new Vec3d(-2, 1, 0), Map.of("blade", Mount.at(new Vec3d(0, -0.3, 0))), null, 0, List.of());
        return new VehicleBlueprint("sled", List.of(new VehicleBlueprint.Entry("body", body, Mount.at(Vec3d.ZERO)),
                new VehicleBlueprint.Entry("blade", blade, Mount.on("body", "blade"), List.of(Binding.of("full", "full")))), List.of()).compile();
    }

    private static VehicleEnvironment site(Terrain t) { return VehicleEnvironment.calm(-1e4, WATER, AIR).withTerrain(t); }

    private static VehicleState moving(Vehicle v, double speed) {
        // the frame's origin 0.3 m over the loam's surface at y = 0 cuts 0.6 m deep... origin at y = 0.3: the edge at y = 0
        VehicleState st = v.newState(new Vec3d(0.5, 0.3 - 0.3, 0.5), Orientation.IDENTITY, DT);
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(speed, 0, 0), b.orientation(), Vec3d.ZERO));
        return st;
    }

    @Test
    void aBladeMeetsTheEarthmovingEquationsResistanceAndCutsItsSweep() {
        Vehicle v = sled("");
        // the edge at y = -0.3 below a surface at y = 0: 0.3 m of loam over it
        VehicleState st = v.newState(new Vec3d(0.5, 0, 0.5), Orientation.IDENTITY, DT);
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(1, 0, 0), b.orientation(), Vec3d.ZERO));
        Terrain loam = Terrain.below(0, Terrain.Cell.of("dirt", Material.LOAM));
        double mass = v.properties().massKg();
        double before = st.body().velocity().x();
        VehicleDynamics.step(v, st, site(loam), Map.of(), Vec3d.ZERO, List.of(), null, 0, DT);
        double force = (before - st.body().velocity().x()) * mass / DT;
        // twelve 0.25 m slices of 0.3 m depth at 1 m/s
        Cut one = Digging.edge(Material.LOAM, RAKE, 0.3, 0.25, 1, 0);
        double expected = 12 * one.horizontalN();
        assertEquals(expected, force, 0.03 * expected, "the blade's resistance is Digging.edge's over its width");
        double cut = 0;
        for (VehicleState.Cut c : st.takeCuts()) { cut += c.m3(); }
        // the speed falls a little within the step: the sweep 3 m x 0.3 m x v dt
        assertEquals(3 * 0.3 * 1 * DT, cut, 0.03 * 3 * 0.3 * DT, "it cuts its width x depth x speed x dt");
        assertTrue(st.takeCuts().isEmpty(), "taken once");
    }

    @Test
    void aFullBucketCutsNothingAndAWallStopsTheEdge() {
        Vehicle v = sled("bowl");
        Terrain loam = Terrain.below(0, Terrain.Cell.of("dirt", Material.LOAM));
        VehicleState st = v.newState(new Vec3d(0.5, 0, 0.5), Orientation.IDENTITY, DT);
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(1, 0, 0), b.orientation(), Vec3d.ZERO));
        VehicleDynamics.step(v, st, site(loam), Map.of("full", 1.0), Vec3d.ZERO, List.of(), null, 0, DT);
        assertTrue(st.takeCuts().isEmpty(), "a full bucket takes nothing in");
        VehicleDynamics.step(v, st, site(loam), Map.of(), Vec3d.ZERO, List.of(), null, DT, DT);
        List<VehicleState.Cut> cuts = st.takeCuts();
        assertTrue(!cuts.isEmpty() && cuts.getFirst().hold().equals("blade.bowl"), "an empty one fills its hold: " + cuts);
        // bedrock under the edge: it stops within a couple of steps
        Terrain bedrock = Terrain.below(0, new Terrain.Cell("bedrock", Material.HARD_ROCK, false, 1));
        VehicleState wall = v.newState(new Vec3d(0.5, 0, 0.5), Orientation.IDENTITY, DT);
        b = wall.body();
        wall.setBody(new RigidBodyState(b.position(), new Vec3d(1, 0, 0), b.orientation(), Vec3d.ZERO));
        double energy = wall.body().kineticEnergy(wall.properties());
        for (int i = 0; i < 4; i++) { VehicleDynamics.step(v, wall, site(bedrock), Map.of(), Vec3d.ZERO, List.of(), null, i * DT, DT); }
        // a free sled pivots over its blade; the edge itself is stopped
        double edge = edgeSpeed(wall);
        assertTrue(Math.abs(edge) < 0.05, "an undiggable cell is a wall to the edge: " + edge);
        assertTrue(wall.body().kineticEnergy(wall.properties()) < energy, "and takes energy, never gives it");
        assertTrue(wall.takeCuts().isEmpty(), "and it cuts nothing");
    }

    /** The blade edge's middle's speed along x (world): the body's velocity there. */
    private static double edgeSpeed(VehicleState st) {
        RigidBodyState b = st.body();
        Vec3d edge = b.worldPoint(st.properties(), new Vec3d(0, -0.3, 0));
        return b.velocity().add(b.angularVelocity(st.properties()).cross(edge.subtract(b.position()))).x();
    }

    @Test
    void aBladeStopsInRockAndRunsOnInSoil() {
        Vehicle v = sled("");
        double[] speed = new double[2], energy = new double[2];
        Material[] materials = {Material.LOAM, Material.SOFT_ROCK};
        for (int k = 0; k < 2; k++) {
            VehicleState st = v.newState(new Vec3d(0.5, 0, 0.5), Orientation.IDENTITY, DT);
            RigidBodyState b = st.body();
            st.setBody(new RigidBodyState(b.position(), new Vec3d(0.5, 0, 0), b.orientation(), Vec3d.ZERO));
            double start = st.body().kineticEnergy(st.properties());
            for (int i = 0; i < 10; i++) {
                VehicleDynamics.step(v, st, site(Terrain.below(0, Terrain.Cell.of("x", materials[k]))), Map.of(), Vec3d.ZERO, List.of(), null, i * DT, DT);
            }
            speed[k] = edgeSpeed(st);
            energy[k] = st.body().kineticEnergy(st.properties()) / start;
        }
        assertTrue(speed[0] > 0.4, "a 20 t sled coasting at 0.5 m/s cuts on through loam for 50 ms: " + speed[0]);
        assertTrue(Math.abs(speed[1]) < 0.05, "soft rock at 40 MPa stops a blade's edge (it needs a ripper): " + speed[1]);
        assertTrue(energy[1] < 1 && energy[0] < 1, "cutting takes energy, never gives it: " + energy[0] + ", " + energy[1]);
    }

    @Test
    void cutsGoToTheCellsTheyCameFrom() {
        // a mound one block high on loam: the edge 0.5 m into it takes from the mound's cell only
        java.util.Map<Long, Terrain.Cell> cells = new java.util.HashMap<>();
        for (int x = -2; x < 6; x++) for (int z = -3; z < 4; z++) { cells.put(Terrain.key(x, 0, z), Terrain.Cell.of("dirt", Material.LOAM)); }
        Vehicle v = sled("");
        VehicleState st = v.newState(new Vec3d(0.5, 0.8, 0.5), Orientation.IDENTITY, DT);
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(1, 0, 0), b.orientation(), Vec3d.ZERO));
        VehicleDynamics.step(v, st, site(Terrain.grid(cells)), Map.of(), Vec3d.ZERO, List.of(), null, 0, DT);
        List<VehicleState.Cut> cuts = st.takeCuts();
        assertTrue(!cuts.isEmpty());
        for (VehicleState.Cut c : cuts) { assertEquals(0, c.y(), "from the mound's layer"); assertEquals(0, c.x()); }
        double total = cuts.stream().mapToDouble(VehicleState.Cut::m3).sum();
        assertEquals(3 * 0.5 * DT, total, 0.05 * 3 * 0.5 * DT, "0.5 m of the mound over the edge");
    }
}
