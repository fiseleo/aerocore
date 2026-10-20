package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.BuoyantSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.HullModel;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.PlaningDynamics;
import dev.mcmod.aerocore.api.PlaningHull;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Rudder;
import dev.mcmod.aerocore.api.TestHull;
import dev.mcmod.aerocore.api.TestPlatform;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import dev.mcmod.aerocore.api.WaveRadiation;
import dev.mcmod.aerocore.api.Wrench;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class VehicleTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, DT = 0.005;
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-G);
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;

    // ----- mass properties -----

    @Test void massPropertiesFollowTheParallelAxisTheorem() {
        // Two 10 kg points 2 m apart along z and a 5 kg one above: centre and inertia by hand.
        MassProperties sum = MassProperties.combine(List.of(MassProperties.point(10, new Vec3d(0, 0, -1)), MassProperties.point(10, new Vec3d(0, 0, 1)),
                MassProperties.point(5, new Vec3d(0, 2, 0))));
        assertEquals(25, sum.massKg(), 1e-12);
        assertEquals(0.4, sum.centre().y(), 1e-12);
        // xx = sum m (y^2 + z^2) about the centre: 10 (0.16 + 1) * 2 + 5 * 1.6^2.
        assertEquals(2 * 10 * (0.16 + 1) + 5 * 1.6 * 1.6, sum.xx(), 1e-9);
        assertEquals(2 * 10 * 1.0, sum.yy(), 1e-9);
        assertEquals(2 * 10 * 0.16 + 5 * 1.6 * 1.6, sum.zz(), 1e-9);
        // Unit shells against closed forms: thin sphere 2/3 m r^2; solid box m (b^2 + c^2) / 12.
        MassProperties shell = MassShapes.sphere(1, 1e-5);
        // The exact shell differs from the thin limit by order t / r = 1e-5.
        assertEquals(4 * Math.PI * 1e-5, shell.massKg(), 2e-5 * shell.massKg());
        assertEquals(2.0 / 3 * shell.massKg(), shell.xx(), 2e-5 * shell.xx());
        MassProperties box = MassShapes.box(new Vec3d(2, 1, 3), 10);
        assertEquals(6, box.massKg(), 1e-12);
        assertEquals(6 * (1 + 9) / 12.0, box.xx(), 1e-12);
    }

    @Test void mountsTurnAndMovePartMass() {
        // A long box (4 x 1 x 1) yawed 90 deg lies along z: its xx and zz swap.
        PartDesign box = TestParts.box("b", new Vec3d(4, 1, 1), 1.05, true, 1, MassSpec.solid(new Material("unit", 1)));
        Vehicle straight = new VehicleBlueprint("s", List.of(new VehicleBlueprint.Entry("b", box, Mount.ORIGIN)), List.of()).compile();
        Vehicle turned = new VehicleBlueprint("t", List.of(new VehicleBlueprint.Entry("b", box, new Mount(new Vec3d(1, 2, 3), 90, 0, 0))), List.of()).compile();
        assertEquals(straight.properties().inertia().xx(), turned.properties().inertia().zz(), 1e-9);
        assertEquals(straight.properties().inertia().zz(), turned.properties().inertia().xx(), 1e-9);
        assertEquals(new Vec3d(1, 2, 3).subtract(turned.properties().centerOfMassLocal()).length(), 0, 1e-12);
    }

    // ----- the parts reproduce the integrators they wrap -----

    /** Four floats of the test platform with a given 500 kg frame between them. */
    private static VehicleBlueprint platform() {
        List<VehicleBlueprint.Entry> parts = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            BuoyantSphere s = TestPlatform.FLOATS.get(i);
            parts.add(new VehicleBlueprint.Entry("float" + i, TestParts.sphere("float", s.radiusM(), MassSpec.shell(Material.STEEL, 0.004)), Mount.at(s.offset())));
        }
        parts.add(new VehicleBlueprint.Entry("frame", TestParts.box("frame", new Vec3d(2, 0.2, 2), 0, false, 1, MassSpec.given(300)), Mount.at(new Vec3d(0, 0.35, 0))));
        parts.add(new VehicleBlueprint.Entry("ballast", TestParts.ballast("ballast", 200), Mount.at(TestPlatform.DEFAULT_BALLAST)));
        return new VehicleBlueprint("platform", parts, List.of());
    }

    @Test void floatsReproduceFloatingBodyDynamicsWithRadiation() {
        Vehicle v = platform().compile();
        RigidBodyProperties p = v.properties();
        List<BuoyantSphere> spheres = new ArrayList<>();
        for (BuoyantSphere s : TestPlatform.FLOATS) { spheres.add(new BuoyantSphere(s.offset(), s.radiusM())); }
        WaveField waves = new WaveField(List.of(new LinearWave(0.15, 6, Math.toRadians(30), Double.POSITIVE_INFINITY, 0.4)));
        Orientation q = Orientation.rotationVector(new Vec3d(0.05, 0.3, -0.04));
        VehicleState st = v.newState(new Vec3d(0, 0.1, 0), q, DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(0.3, -0.2, 0.1), q, new Vec3d(5, -3, 2)));
        RigidBodyState reference = st.body();
        WaveRadiation.Memory memory = new WaveRadiation.Memory(List.of(spheres.get(0).offset(), spheres.get(1).offset(), spheres.get(2).offset(), spheres.get(3).offset()), DT);
        for (int i = 0; i < 400; i++) {
            double t = i * DT;
            FloatingBodyDynamics.Environment fluids = TestPlatform.wavyWater(0, waves, t);
            VehicleDynamics.step(v, st, VehicleEnvironment.of(fluids, point -> HullDynamics.Water.calm(0, WATER, AIR), waves), Map.of(), GRAVITY, List.of(), null, t, DT);
            List<AppliedForce> radiation = memory.loads(reference, p, spheres, Vec3d.Y, WATER.densityKgPerM3());
            FloatingBodyDynamics.Result r = FloatingBodyDynamics.step(reference, p, spheres, fluids, GRAVITY, FloatingBodyDynamics.STANDARD_DRAG, radiation, null, DT);
            reference = r.state();
            if (i == 0) {
                double[] radii = new double[4];
                for (int k = 0; k < 4; k++) { radii[k] = WaveRadiation.waterplaneRadius(spheres.get(k).radiusM(), r.wetFractions().get(k)); }
                memory.prime(radii);
            }
            memory.record(reference, p, spheres, Vec3d.Y, r.wetFractions());
        }
        assertClose(reference, st.body(), 1e-9);
    }

    @Test void hullReproducesHullDynamicsAndDoesNotCareHowItIsMounted() {
        PartDesign design = TestParts.hull("test hull", TestHull.HULL, TestHull.MASS,
                MassSpec.given(TestHull.MASS).resolve(t -> MassShapes.hullShell(TestHull.HULL, 1e-3)));
        Vehicle straight = new VehicleBlueprint("a", List.of(new VehicleBlueprint.Entry("hull", design, Mount.ORIGIN)), List.of()).compile();
        // The same hull mounted turned and moved, the vehicle turned back: the same body in the world. (Yaw only: the heave
        // radiation memory measures point distances in the vehicle's level plane, so a tilted mount is a different layout.)
        Mount mount = new Mount(new Vec3d(0.4, -0.2, 0.7), 70, 0, 0);
        Vehicle turned = new VehicleBlueprint("b", List.of(new VehicleBlueprint.Entry("hull", design, mount)), List.of()).compile();
        HullModel model = new HullModel(TestHull.HULL, HullModel.Type.SURFACE, TestHull.MASS, WATER, 0);
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0, WATER, AIR);
        VehicleEnvironment environment = VehicleEnvironment.of((c, r) -> { throw new AssertionError("no floats"); }, calm, WaveField.NONE);
        Orientation q = Orientation.rotationVector(new Vec3d(0.04, 0.5, 0.06));
        VehicleState a = straight.newState(Vec3d.ZERO, q, DT), b = turned.newState(Vec3d.ZERO, q.multiply(mount.rotation().inverse()), DT);
        // Put b's hull exactly where a's is, moving the same.
        Vec3d velocity = new Vec3d(2, 0.1, 0.3), spin = new Vec3d(0.05, 0.2, -0.1);
        a.setBody(new RigidBodyState(new Vec3d(0, -0.05, 0), velocity, q, straight.properties().inertia().applyWorld(q, spin)));
        b.setBody(new RigidBodyState(a.body().position(), velocity, b.body().orientation(), turned.properties().inertia().applyWorld(b.body().orientation(), spin)));
        RigidBodyProperties p = straight.properties();
        RigidBodyState reference = a.body();
        WaveRadiation.Memory memory = new WaveRadiation.Memory(HullDynamics.radiationPoints(TestHull.HULL, 6), DT);
        for (int i = 0; i < 300; i++) {
            VehicleDynamics.step(straight, a, environment, Map.of(), GRAVITY, List.of(), null, i * DT, DT);
            VehicleDynamics.step(turned, b, environment, Map.of(), GRAVITY, List.of(), null, i * DT, DT);
            List<AppliedForce> radiation = memory.loadsAt(reference, p, HullDynamics.radiationPoints(TestHull.HULL, 6), Vec3d.Y, WATER.densityKgPerM3());
            HullDynamics.Result r = HullDynamics.step(reference, p, model, calm, GRAVITY, radiation, null, DT);
            reference = r.state();
            double[] radii = HullDynamics.segmentRadii(TestHull.HULL, 6, r.waterlineBreadth());
            if (i == 0) { memory.prime(radii); }
            memory.recordAt(reference, p, HullDynamics.radiationPoints(TestHull.HULL, 6), Vec3d.Y, radii);
        }
        assertClose(reference, a.body(), 1e-9);
        // The turned vehicle: same COM motion, and its frame times the mount is a's frame.
        assertEquals(0, a.body().position().subtract(b.body().position()).length(), 1e-8);
        assertEquals(0, a.body().velocity().subtract(b.body().velocity()).length(), 1e-8);
        Orientation hullB = b.body().orientation().multiply(mount.rotation());
        assertEquals(1, Math.abs(dot(a.body().orientation(), hullB)), 1e-12);
        assertEquals(0, a.body().angularMomentum().subtract(b.body().angularMomentum()).length(), 1e-6 * a.body().angularMomentum().length() + 1e-9);
    }

    @Test void planingHullReproducesPlaningDynamics() {
        PartDesign design = TestParts.planing("fridsma", 6, 1.2, 20, 0.75, 48, 1048);
        Vehicle v = new VehicleBlueprint("p", List.of(new VehicleBlueprint.Entry("hull", design, Mount.ORIGIN)), List.of()).compile();
        PlaningHull hull = PlaningHull.fridsma(6, 1.2, Math.toRadians(20), 0.75, 48);
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0.25, WATER, AIR);
        VehicleEnvironment environment = VehicleEnvironment.of((c, r) -> { throw new AssertionError("no floats"); }, calm, WaveField.NONE);
        Orientation q = Orientation.rotationVector(new Vec3d(0, 0, Math.toRadians(4)));
        VehicleState st = v.newState(Vec3d.ZERO, q, DT);
        RigidBodyProperties p = v.properties();
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(8, 0, 0), q, Vec3d.ZERO));
        RigidBodyState reference = st.body();
        List<Vec3d> points = HullDynamics.radiationPoints(hull.hull(), 6);
        WaveRadiation.Memory memory = new WaveRadiation.Memory(points, DT);
        for (int i = 0; i < 200; i++) {
            VehicleDynamics.step(v, st, environment, Map.of(), GRAVITY, List.of(), null, i * DT, DT);
            double share = 1 - PlaningDynamics.loads(reference, p, hull, calm, GRAVITY, true).diagnostics().ventilation();
            double[] forces = memory.verticalForces(WATER.densityKgPerM3());
            List<AppliedForce> radiation = new ArrayList<>();
            for (int k = 0; k < forces.length; k++) {
                if (forces[k] != 0) { radiation.add(new AppliedForce(AppliedForce.Kind.RADIATION, reference.worldOffset(p, points.get(k)), Vec3d.Y.scale(forces[k] * share))); }
            }
            PlaningDynamics.Result r = PlaningDynamics.step(reference, p, hull, calm, GRAVITY, radiation, DT);
            reference = r.state();
            double[] radii = HullDynamics.segmentRadii(hull.hull(), 6, r.diagnostics().waterlineBreadth());
            if (i == 0) { memory.prime(radii); }
            memory.recordAt(reference, p, points, Vec3d.Y, radii);
        }
        assertClose(reference, st.body(), 1e-9);
        assertTrue(st.diagnostics().get("hull").get("ventilation") > 0.5, "planing at 8 m/s: " + st.diagnostics());
    }

    // ----- shafts, motors and propellers -----

    /** A motor driving a B4-70 screw through a gearbox on a vehicle too heavy to move (bollard pull). */
    private static Vehicle bollard(double ratio, double efficiency) {
        return new VehicleBlueprint("bollard", List.of(
                new VehicleBlueprint.Entry("motor", TestParts.motor("m", 200, 20000, 3000, 0.02, 0.9, 30), Mount.ORIGIN, List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("screw", TestParts.propeller("s", 0.35, 4, 0.70, 1.4, 0.18, 0.1), Mount.at(new Vec3d(-1, -1, 0))),
                new VehicleBlueprint.Entry("weight", TestParts.ballast("w", 1e8), Mount.at(new Vec3d(0, 0, 0))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(new Connection.Shaft("motor", "screw", ratio, efficiency))).compile();
    }

    @Test void motorAndScrewSettleWhereTorquesBalanceAndTheHullFeelsTheReaction() {
        for (double[] gear : new double[][]{{1, 1}, {2, 0.95}}) {
            Vehicle v = bollard(gear[0], gear[1]);
            assertEquals(1, v.trains(), "one shaft train");
            VehicleState st = v.newState(new Vec3d(0, 0.5, 0), Orientation.IDENTITY, DT);
            VehicleEnvironment env = VehicleEnvironment.calm(0, WATER, AIR);
            VehicleDynamics.Result r = null;
            for (int i = 0; i < 4000; i++) {
                st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
                r = VehicleDynamics.step(v, st, env, Map.of("throttle", 0.6), GRAVITY, List.of(), null, i * DT, DT);
            }
            Map<String, Double> motor = r.diagnostics().get("motor"), screw = r.diagnostics().get("screw");
            double motorSpeed = st.rotorSpeed(0), screwSpeed = st.rotorSpeed(1);
            assertEquals(motorSpeed / gear[0], screwSpeed, 1e-9 * motorSpeed, "the gear divides the speed");
            // Steady: the motor's torque carries the screw's torque through the gear, divided by its efficiency.
            assertEquals(screw.get("torqueNm") / gear[0] / gear[1], motor.get("torqueNm"), 2e-3 * motor.get("torqueNm"), "torque balance " + motor + " " + screw);
            // Bollard thrust from the open-water regression at J = 0: T = K_T rho n^2 D^4.
            double n = screwSpeed / (2 * Math.PI), kt = new dev.mcmod.aerocore.api.WageningenB(4, 0.70, 1.4).thrust(0);
            assertEquals(kt * WATER.densityKgPerM3() * n * n * Math.pow(0.35, 4), screw.get("thrustN"), 0.01 * screw.get("thrustN"));
            // The reaction on the hull about the shaft axis is the screw's torque (steady rotors push nothing else).
            Wrench thrust = r.load(AppliedForce.Kind.THRUST);
            assertEquals(-screw.get("torqueNm"), thrust.torque().x() - new Vec3d(-1, -1, 0).subtract(v.properties().centerOfMassLocal()).cross(thrust.force()).x(),
                    1e-3 * screw.get("torqueNm"));
        }
    }

    @Test void anUnloadedMotorRunsUpToItsTopSpeedAndStays() {
        Vehicle v = new VehicleBlueprint("free", List.of(
                new VehicleBlueprint.Entry("motor", TestParts.motor("m", 50, 5000, 3000, 0.01, 0.9, 10), Mount.ORIGIN, List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1000)), Mount.ORIGIN)), List.of()).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleEnvironment env = VehicleEnvironment.calm(-100, WATER, AIR);
        double top = 3000 * Math.PI / 30;
        // Angular momentum of rotor plus body is conserved (no outside torque): the body spins the other way.
        for (int i = 0; i < 2000; i++) { VehicleDynamics.step(v, st, env, Map.of("throttle", 1.0), Vec3d.ZERO, List.of(), null, i * DT, DT); }
        assertEquals(top, st.rotorSpeed(0), 0.03 * top, "at most a step's worth above the top speed");
        double rotor = 0.01 * st.rotorSpeed(0), body = st.body().angularMomentum().x();
        assertEquals(0, rotor + body, 1e-6 * rotor, "rotor " + rotor + " body " + body);
    }

    /** A driver and a flywheel through a shaft that is or is not a freewheel, on a body too heavy to turn. */
    private static Vehicle clutch(boolean freewheel) {
        return new VehicleBlueprint("clutch", List.of(
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.5, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("rotor", TestParts.flywheel("r", 2.0, false), Mount.at(new Vec3d(0, 1, 0))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(new Connection.Shaft("engine", "rotor", 2, 1, freewheel))).compile();
    }

    @Test void aFreewheelDrivesButLetsTheDrivenSideOverrun() {
        for (boolean freewheel : new boolean[]{false, true}) {
            Vehicle v = clutch(freewheel);
            assertEquals(freewheel ? 2 : 1, v.trains());
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            VehicleEnvironment env = VehicleEnvironment.calm(-100, WATER, AIR);
            // driving with 10 N m for 1 s: engine 0.5 kg m^2 plus the rotor (2) through 2:1 (0.5), so 1 kg m^2
            for (int i = 0; i < 200; i++) {
                st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
                VehicleDynamics.step(v, st, env, Map.of("torque", 10.0), Vec3d.ZERO, List.of(), null, i * DT, DT);
            }
            assertEquals(10, st.rotorSpeed(0), 0.05, "engine speed after a second");
            assertEquals(5, st.rotorSpeed(1), 0.03, "the rotor turns at half");
            // braking the engine: a freewheel lets the rotor keep its speed, a plain shaft slows it too
            for (int i = 200; i < 300; i++) {
                st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
                VehicleDynamics.step(v, st, env, Map.of("torque", -10.0), Vec3d.ZERO, List.of(), null, i * DT, DT);
            }
            if (freewheel) {
                assertEquals(5, st.rotorSpeed(1), 1e-9, "the rotor overruns");
                assertEquals(0, st.rotorSpeed(0), 0.05, "the engine slows alone (0.5 kg m^2)");
            } else {
                assertEquals(st.rotorSpeed(0) / 2, st.rotorSpeed(1), 1e-9);
                assertEquals(5, st.rotorSpeed(0), 0.05, "both slow (1 kg m^2)");
            }
        }
        // driving again, the freewheel picks the rotor up when the engine catches up with it
        Vehicle v = clutch(true);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setTrainSpeed(1, 4);
        VehicleEnvironment env = VehicleEnvironment.calm(-100, WATER, AIR);
        for (int i = 0; i < 400; i++) {
            st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
            VehicleDynamics.step(v, st, env, Map.of("torque", 5.0), Vec3d.ZERO, List.of(), null, i * DT, DT);
        }
        assertEquals(st.rotorSpeed(0) / 2, st.rotorSpeed(1), 1e-9, "engaged");
        assertTrue(st.rotorSpeed(1) > 4.5);
    }

    @Test void aSpinningRotorResistsTheBodyTurningGyroscopically() {
        Vehicle v = new VehicleBlueprint("gyro", List.of(
                new VehicleBlueprint.Entry("wheel", TestParts.flywheel("w", 3.0, false), Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(2, 1, 1), 0, false, 1, MassSpec.given(500)), Mount.ORIGIN)), List.of()).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setTrainSpeed(0, 100);
        // yawing at 0.5 rad/s about +y with the wheel's momentum along +x: the body needs -(omega x H) = +z, 150 N m,
        // taken in the drift with the rotation (it shows as the inertia exchange)
        RigidBodyProperties p = v.properties();
        Vec3d angularMomentum = p.inertia().apply(new Vec3d(0, 0.5, 0));
        st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, angularMomentum));
        assertEquals(0.5, st.body().angularVelocity(p).y(), 1e-6);
        VehicleDynamics.Result r = VehicleDynamics.step(v, st, VehicleEnvironment.calm(-100, WATER, AIR), Map.of(), Vec3d.ZERO, List.of(), null, 0, DT);
        assertEquals(150, r.inertia().torque().z(), 1.0, "gyroscopic couple " + r.inertia().torque());
    }

    /**
     * A 500 kg box carrying a 3 kg m^2 wheel at 1000 rad/s (its momentum over the box's inertia about 14 rad/s), free of
     * torques and tumbling a little: the body's own rotational energy and the system's momentum stay. Taken as an
     * explicit force, the wheel's couple gained about a quarter of a percent of that energy every step (a helicopter's
     * rotor on its fuselage is the same ratio).
     */
    @Test void aFreeGyrostatKeepsItsMomentumAndEnergy() {
        Vehicle v = new VehicleBlueprint("gyrostat", List.of(
                new VehicleBlueprint.Entry("wheel", TestParts.flywheel("w", 3.0, false), Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(2, 1, 1), 0, false, 1, MassSpec.given(500)), Mount.ORIGIN)), List.of()).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setTrainSpeed(0, 1000);
        RigidBodyProperties p = v.properties();
        st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, p.inertia().apply(new Vec3d(0.2, 0.1, 0.1))));
        java.util.function.Supplier<Vec3d> total = () -> st.body().angularMomentum().add(st.body().orientation().rotate(Vec3d.X.scale(3.0 * st.rotorSpeed(0))));
        double energy = 0.5 * st.body().angularMomentum().dot(st.body().angularVelocity(p));
        Vec3d momentum = total.get();
        for (int i = 0; i < 2000; i++) {
            VehicleDynamics.step(v, st, VehicleEnvironment.calm(-100, WATER, AIR), Map.of(), Vec3d.ZERO, List.of(), null, i * DT, DT);
        }
        assertEquals(1000, st.rotorSpeed(0), 1e-9, "nothing turns the wheel");
        assertEquals(energy, 0.5 * st.body().angularMomentum().dot(st.body().angularVelocity(p)), 0.02 * energy, "the body's rotational energy");
        assertEquals(0, total.get().subtract(momentum).length(), 1e-6 * momentum.length(), "the momentum of body and wheel " + total.get() + " vs " + momentum);
    }

    /**
     * A part on another part's mount point: the host's placement, then the point, then the offset (in the point's frame);
     * chains follow; unknown parts or points are design issues; the attachment survives JSON.
     */
    @Test void partsAttachToMountPoints() {
        Map<String, Mount> points = Map.of("top", new Mount(new Vec3d(1, 2, 0), 90, 0, 0), "tip", Mount.at(new Vec3d(0, 0, 3)));
        Vehicle v = new VehicleBlueprint("attached", List.of(
                new VehicleBlueprint.Entry("host", TestParts.host("h", 100, points), new Mount(new Vec3d(10, 0, 0), 0, 0, 0)),
                new VehicleBlueprint.Entry("second", TestParts.host("s", 10, points), Mount.on("host", "top", Mount.at(new Vec3d(1, 0, 0)))),
                new VehicleBlueprint.Entry("third", TestParts.ballast("t", 1), Mount.on("second", "tip"))), List.of()).compile();
        // host at (10,0,0); its "top" at (11,2,0) turned 90 deg about +Y, so the point's +X is the vehicle's -Z
        Vehicle.Installed second = v.installed().get(1), third = v.installed().get(2);
        assertEquals(11, second.mount().position().x(), 1e-9);
        assertEquals(2, second.mount().position().y(), 1e-9);
        assertEquals(-1, second.mount().position().z(), 1e-9);
        assertEquals(90, second.mount().yawDeg(), 1e-9);
        // the second's "tip" (0,0,3) in its frame turned 90 deg about +Y: +Z goes to +X
        assertEquals(14, third.mount().position().x(), 1e-9);
        assertEquals(-1, third.mount().position().z(), 1e-9);
        // unknown point and part
        DesignException missing = assertThrows(DesignException.class, () -> new VehicleBlueprint("bad", List.of(
                new VehicleBlueprint.Entry("host", TestParts.host("h", 100, points), Mount.ORIGIN),
                new VehicleBlueprint.Entry("x", TestParts.ballast("x", 1), Mount.on("host", "nose")),
                new VehicleBlueprint.Entry("y", TestParts.ballast("y", 1), Mount.on("nobody", "top"))), List.of()).compile());
        assertEquals(2, missing.issues().stream().filter(i -> i.kind() == DesignIssue.Kind.INVALID).count(), missing.issues().toString());
        // JSON keeps the attachment
        Mount m = Mount.on("host", "top", new Mount(new Vec3d(1, 2, 3), 10, 20, 30));
        assertEquals(m, Mount.fromJson(Fields.of(Json.parse(Json.write(m.toJson())), "mount")));
        // angles survive a round trip through a rotation
        Mount turned = new Mount(new Vec3d(1, 2, 3), 35, -20, 50);
        Mount back = Mount.of(turned.position(), turned.rotation());
        assertEquals(35, back.yawDeg(), 1e-9); assertEquals(-20, back.pitchDeg(), 1e-9); assertEquals(50, back.rollDeg(), 1e-9);
    }

    /** A float body of modules: floats at its displacement, carries less with a low sill, and has its air drag in flight. */
    @Test void aBodyFloatsAndFlies() {
        var vee = new dev.mcmod.aerocore.api.body.SectionShape.Chined(1.1, 0.8, 0, 25, 5);
        java.util.function.DoubleFunction<dev.mcmod.aerocore.api.body.BodyShape> shape = sill -> new dev.mcmod.aerocore.api.body.BodyShape(vee.scaled(0.05), List.of(
                new dev.mcmod.aerocore.api.body.BodyModule("bow", 1.5, vee, 2, 0, Double.POSITIVE_INFINITY),
                new dev.mcmod.aerocore.api.body.BodyModule("middle", 4, vee, 1, 0, sill),
                new dev.mcmod.aerocore.api.body.BodyModule("stern", 1.5, vee, 1, 0, Double.POSITIVE_INFINITY)));
        double mass = 900;
        double[] wet = new double[2], height = new double[2];
        for (int k = 0; k < 2; k++) {
            Vehicle v = new VehicleBlueprint("float", List.of(new VehicleBlueprint.Entry("body", TestParts.body("b" + k, shape.apply(k == 0 ? Double.POSITIVE_INFINITY : 0.12), mass), Mount.ORIGIN)), List.of()).compile();
            assertTrue(((dev.mcmod.aerocore.api.vehicle.part.BodyPart) v.installed().get(0).part()).planing(), "a vee bottom planes");
            VehicleState st = v.newState(new Vec3d(0, 0.1, 0), Orientation.IDENTITY, DT);
            VehicleEnvironment env = VehicleEnvironment.calm(0, WATER, AIR);
            VehicleDynamics.Result r = null;
            for (int i = 0; i < (int) (25 / DT); i++) { r = VehicleDynamics.step(v, st, env, Map.of(), Vec3d.Y.scale(-G), List.of(), null, i * DT, DT); }
            wet[k] = r.diagnostics().get("body").get("wetVolumeM3");
            height[k] = v.origin(st.body()).y();
            // settled as the planing hull test's dropped boat is (PlaningDynamicsTest)
            Vec3d vel = st.body().velocity();
            assertTrue(Math.abs(vel.y()) < 0.02 && Math.hypot(vel.x(), vel.z()) < 0.08, "at rest: " + vel);
        }
        assertEquals(mass / WATER.densityKgPerM3(), wet[0], 0.03 * mass / WATER.densityKgPerM3(), "closed: floats on its displacement");
        // open amidships from 12 cm above the keel: the water there carries nothing, so it floats deeper on the bow and stern
        assertEquals(mass / WATER.densityKgPerM3(), wet[1], 0.03 * mass / WATER.densityKgPerM3(), "open: still afloat");
        assertTrue(height[1] < height[0] - 0.02, "and lower: " + height[1] + " against " + height[0]);
        // in the air, well clear of the water, at 50 m/s
        var b = shape.apply(Double.POSITIVE_INFINITY);
        Vehicle v = new VehicleBlueprint("flying", List.of(new VehicleBlueprint.Entry("body", TestParts.body("c", b, mass), Mount.ORIGIN)), List.of()).compile();
        VehicleState st = v.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(50, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
        VehicleDynamics.Result r = VehicleDynamics.step(v, st, VehicleEnvironment.calm(0, WATER, AIR), Map.of(), Vec3d.ZERO, List.of(), null, 0, DT);
        double expected = 0.5 * AIR.densityKgPerM3() * 50 * 50 * new dev.mcmod.aerocore.api.body.BodyAerodynamics(b, 0).zeroLiftDragAreaM2(50,
                new dev.mcmod.aerocore.api.aero.Wing.Air(AIR.densityKgPerM3(), AIR.dynamicViscosityPaS(), AIR.speedOfSoundMps()));
        assertEquals(expected, r.diagnostics().get("body").get("airDragN"), 0.01 * expected);
    }

    @Test void aWingLiftsAndItsAileronsRoll() {
        Vehicle v = new VehicleBlueprint("glider", List.of(
                new VehicleBlueprint.Entry("wing", TestParts.wing("w", 10, 1.2), new Mount(Vec3d.ZERO, 0, 4, 0), List.of(Binding.of("aileron", "roll"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 1, 1), 0, false, 1, MassSpec.given(300)), Mount.ORIGIN)), List.of()).compile();
        VehicleState st = v.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(40, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
        VehicleEnvironment env = VehicleEnvironment.calm(-100, WATER, AIR);
        VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, Map.of("roll", 0.0), GRAVITY, List.of(), null, 0, DT);
        double q = 0.5 * AIR.densityKgPerM3() * 40 * 40 * 12;
        double cl = r.load(AppliedForce.Kind.LIFT).force().y() / q;
        assertTrue(cl > 0.5 && cl < 0.85, "4 deg on a 2412 wing of aspect ratio 8.3: CL " + cl);
        Vec3d com = v.properties().centerOfMassLocal();
        assertEquals(0, r.load(AppliedForce.Kind.LIFT).torque().x(), 1e-3 * q, "no roll");
        st.setBody(new RigidBodyState(new Vec3d(0, 100, 0), new Vec3d(40, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
        VehicleDynamics.Result rolled = VehicleDynamics.step(v, st, env, Map.of("roll", 1.0), GRAVITY, List.of(), null, 0, DT);
        assertTrue(rolled.load(AppliedForce.Kind.LIFT).torque().x() > 0.01 * q, "roll to starboard " + rolled.load(AppliedForce.Kind.LIFT).torque() + " com " + com);
        assertTrue(rolled.diagnostics().get("wing").get("liftN") > 0);
    }

    // ----- control surfaces and boxes -----

    @Test void rudderPushesTheSternToPortAndTheRaceStrengthensIt() {
        PartDesign rudder = TestParts.surface("r", 0.22, 0.32, 35, 0.15, Rudder.lowAspectPolar(0.22, 0.32));
        Vehicle v = new VehicleBlueprint("r", List.of(
                new VehicleBlueprint.Entry("rudder", rudder, Mount.at(new Vec3d(-2, -0.3, 0)), List.of(Binding.of("deflection", "rudder"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 0.5, 1), 0, false, 1, MassSpec.given(1000)), Mount.ORIGIN)), List.of()).compile();
        VehicleEnvironment env = VehicleEnvironment.calm(1, WATER, AIR);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(3, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
        VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, Map.of("rudder", 0.5), Vec3d.ZERO, List.of(), null, 0, DT);
        Wrench lift = r.load(AppliedForce.Kind.LIFT);
        assertTrue(lift.force().z() < -50, "trailing edge to starboard pushes the stern to port: " + lift.force());
        assertTrue(lift.torque().y() < 0, "and turns the bow to starboard (negative yaw about +y): " + lift.torque());
        // A horizontal surface (rolled 90 deg) makes vertical force instead.
        Vehicle flat = new VehicleBlueprint("e", List.of(
                new VehicleBlueprint.Entry("elevator", rudder, new Mount(new Vec3d(-2, -0.3, 0), 0, 0, 90), List.of(Binding.of("deflection", "pitch"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 0.5, 1), 0, false, 1, MassSpec.given(1000)), Mount.ORIGIN)), List.of()).compile();
        VehicleState fs = flat.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        fs.setBody(new RigidBodyState(fs.body().position(), new Vec3d(3, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
        Wrench vertical = VehicleDynamics.step(flat, fs, env, Map.of("pitch", 0.5), Vec3d.ZERO, List.of(), null, 0, DT).load(AppliedForce.Kind.LIFT);
        assertEquals(lift.force().length(), vertical.force().length(), 1e-6 * lift.force().length());
        assertEquals(Math.abs(lift.force().z()), Math.abs(vertical.force().y()), 1e-6 * lift.force().length());
        assertEquals(0, vertical.force().z(), 1e-6 * lift.force().length());
    }

    @Test void aClosedBoxFloatsAtItsArchimedesDraft() {
        // 2 x 1 x 1 m box of 800 kg in fresh water: draft (800 - 2 rho_air) / ((rho_w - rho_air) 2) = 0.4004 m.
        Vehicle v = new VehicleBlueprint("box", List.of(new VehicleBlueprint.Entry("box",
                TestParts.box("b", new Vec3d(2, 1, 1), 1.05, true, 4, MassSpec.given(800)), Mount.ORIGIN)), List.of()).compile();
        VehicleEnvironment env = VehicleEnvironment.calm(0, WATER, AIR);
        double rhoAir = AIR.densityKgPerM3(), draft = (800 - rhoAir * 2) / ((WATER.densityKgPerM3() - rhoAir) * 2);
        for (double offset : new double[]{0, 0.01, -0.02}) {
            // Placed at rest with its bottom at the draft plus offset: the net force is rho g A times the offset, upward when deeper.
            VehicleState st = v.newState(new Vec3d(0, 0.5 - draft - offset, 0), Orientation.IDENTITY, DT);
            VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, Map.of(), GRAVITY, List.of(), null, 0, DT);
            double net = r.load(AppliedForce.Kind.WEIGHT).force().y() + r.load(AppliedForce.Kind.BUOYANCY).force().y();
            assertEquals((WATER.densityKgPerM3() - rhoAir) * G * 2 * offset, net, 1e-6 * 800 * G, "offset " + offset);
            assertEquals(0, r.load(AppliedForce.Kind.BUOYANCY).torque().length(), 1e-6, "a level box is not turned");
        }
    }

    @Test void stateRoundTripsThroughJson() {
        Vehicle v = bollard(2, 0.95);
        VehicleState st = v.newState(new Vec3d(1, 2, 3), Orientation.rotationVector(new Vec3d(0.1, 0.2, 0.3)), DT);
        st.setTrainSpeed(0, 123.4);
        VehicleState back = VehicleState.fromJson(v, Json.parse(Json.write(st.toJson())), DT);
        assertClose(st.body(), back.body(), 1e-15);
        assertEquals(123.4, back.trainSpeed(0));
    }

    private static double dot(Orientation a, Orientation b) { return a.w() * b.w() + a.x() * b.x() + a.y() * b.y() + a.z() * b.z(); }

    private static void assertClose(RigidBodyState expected, RigidBodyState actual, double tolerance) {
        assertEquals(0, expected.position().subtract(actual.position()).length(), tolerance * (1 + expected.position().length()), "position");
        assertEquals(0, expected.velocity().subtract(actual.velocity()).length(), tolerance * (1 + expected.velocity().length()), "velocity");
        assertEquals(1, Math.abs(dot(expected.orientation(), actual.orientation())), tolerance, "orientation");
        assertEquals(0, expected.angularMomentum().subtract(actual.angularMomentum()).length(), tolerance * (1 + expected.angularMomentum().length()), "angular momentum");
    }
}
