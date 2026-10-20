package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.vehicle.Binding;
import dev.mcmod.aerocore.api.vehicle.Coupling;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.Fields;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.MassShapes;
import dev.mcmod.aerocore.api.vehicle.MassSpec;
import dev.mcmod.aerocore.api.vehicle.Mount;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.vehicle.PartTypes;
import dev.mcmod.aerocore.api.vehicle.VehicleBlueprint;
import dev.mcmod.aerocore.api.vehicle.part.SpherePart;
import dev.mcmod.aerocore.neoforge.probe.BodyWorld;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;
import java.util.Map;

/** The game side of part-built vehicles that AeroCore itself owns (vehicle GameTests with real parts are in hdfv). */
@PrefixGameTestTemplate(false)
public final class VehicleGameTests {
    /** A test-only part type, registered when the GameTests load. */
    static final String BALL = "aerocore:gametest_ball";
    static { PartTypes.register(BALL, f -> new Ball(f.string("name"), f.number("radiusM"), f.number("massKg"))); }

    private VehicleGameTests() {}

    /**
     * A ball (SpherePart) whose loads throw while its "fail" input is set: a part with a physics bug. It stands for
     * the helicopter that crashed the GameTest server during hdfv 0.4 (RigidBodyDynamics failing to converge).
     */
    record Ball(String name, double radiusM, double massKg) implements PartDesign {
        @Override public String type() { return BALL; }
        @Override public List<DesignIssue> check() { return List.of(); }
        @Override public Map<String, Object> parameters() { return Fields.object("radiusM", radiusM, "massKg", massKg); }
        @Override public Part compile() {
            SpherePart sphere = new SpherePart(this, radiusM, MassSpec.given(massKg).resolve(t -> MassShapes.sphere(radiusM, t)));
            return new Part() {
                @Override public PartDesign design() { return Ball.this; }
                @Override public MassProperties mass() { return sphere.mass(); }
                @Override public List<CollisionSphere> collision() { return sphere.collision(); }
                @Override public List<DesignIssue> issues() { return List.of(); }
                @Override public List<Vec3d> radiationPoints() { return sphere.radiationPoints(); }
                @Override public List<String> inputs() { return List.of("fail", "drop"); }
                @Override public void loads(PartStep step) {
                    if (step.input("fail", 0) > 0.5) { throw new IllegalStateException("Test ball failed on request"); }
                    if (step.input("drop", 0) > 0.5) { step.jettison(); }
                    sphere.loads(step);
                }
            };
        }
    }

    /**
     * Two balls fall; one's physics throws in mid-air. The server ticks on, that ball stops where it was (at rest,
     * saved with its fault, reported) and the other lands; resumed with the fault gone, the stopped one falls too.
     */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120)
    public static void aPhysicsErrorStopsOnlyThatVehicle(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        PartVehicle broken = spawn(h, 6), sound = spawn(h, 16);
        // This failure is the point of the test: it must not fail the test as an unexpected one would.
        broken.addTag(BodyWorld.EXPECTED_FAULT_TAG);
        final Vec3d[] stopped = {null};
        final double[] lowest = {Double.MAX_VALUE};
        for (int tick = 1; tick <= 40; tick++) {
            h.runAfterDelay(tick, () -> lowest[0] = Math.min(lowest[0], sound.vehicleState().body().position().y()));
        }
        h.runAfterDelay(10, () -> {
            h.assertTrue(broken.vehicleState().body().velocity().y() < -1, "Both balls must be falling first: " + broken.vehicleState().body().velocity());
            broken.setControl("fail", 1);
        });
        h.runAfterDelay(12, () -> {
            h.assertTrue(broken.fault() != null && broken.fault().contains("Test ball failed on request"), "The error must be kept: " + broken.fault());
            h.assertTrue(broken.vehicleState().body().velocity().length() == 0 && broken.vehicleState().body().angularMomentum().length() == 0,
                    "The broken ball must be at rest");
            h.assertTrue(sound.fault() == null && sound.vehicleState().body().velocity().y() < -2, "The other ball must fall on: " + sound.vehicleState().body().velocity());
            stopped[0] = broken.vehicleState().body().position();
        });
        h.runAfterDelay(40, () -> {
            h.assertTrue(broken.vehicleState().body().position().equals(stopped[0]), "The broken ball must stay where it stopped: "
                    + broken.vehicleState().body().position() + " vs " + stopped[0]);
            double floor = h.absolutePos(BlockPos.ZERO).getY() + 1;
            h.assertTrue(sound.fault() == null && Math.abs(lowest[0] - floor - 0.4) < 0.1, "The other ball must land on the floor: lowest " + lowest[0]
                    + " (floor " + floor + ")");
            // Without the tag the failure would fail this test: it is the test whose structure holds the ball.
            h.assertTrue(BodyWorld.owningTests(h.getLevel(), broken.position()).equals(List.of(h.testInfo)), "The ball's test must be found");
            // The fault is saved with the vehicle.
            CompoundTag tag = new CompoundTag();
            broken.saveWithoutId(tag);
            PartVehicle copy = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
            if (copy == null) { throw new IllegalStateException("Vehicle factory failed"); }
            copy.load(tag);
            h.assertTrue(broken.fault().equals(copy.fault()), "The fault must come back from the save: " + copy.fault());
            h.assertTrue(copy.vehicleState().body().velocity().length() == 0, "The saved state must be at rest");
            broken.setControl("fail", 0);
            broken.resume();
        });
        h.runAfterDelay(80, () -> {
            h.assertTrue(broken.fault() == null && broken.vehicleState().body().position().y() < stopped[0].y() - 3,
                    "Resumed without its cause, the ball must fall: " + broken.vehicleState().body().position() + " from " + stopped[0]);
            h.succeed();
        });
    }

    /**
     * Wheel rays (0.24, WorldGround) meet blocks by their collision shapes: a full block, a slab, four snow layers,
     * a fence post (1.5 m), a block's side; and the surfaces by block (ice, sand, dirt, stone).
     */
    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 20)
    public static void wheelRaysFollowBlockShapesAndSurfaces(GameTestHelper h) {
        h.setBlock(new BlockPos(1, 1, 1), Blocks.STONE);
        h.setBlock(new BlockPos(3, 1, 1), Blocks.STONE_SLAB);
        h.setBlock(new BlockPos(5, 1, 1), Blocks.SNOW.defaultBlockState().setValue(net.minecraft.world.level.block.SnowLayerBlock.LAYERS, 4));
        h.setBlock(new BlockPos(7, 1, 1), Blocks.OAK_FENCE);
        h.setBlock(new BlockPos(1, 1, 3), Blocks.ICE);
        h.setBlock(new BlockPos(3, 1, 3), Blocks.SAND);
        h.setBlock(new BlockPos(5, 1, 3), Blocks.DIRT);
        WorldGround ground = new WorldGround(h.getLevel());
        double base = h.absolutePos(new BlockPos(0, 1, 0)).getY();
        double[][] tops = {{1, 1, 1}, {3, 1, 0.5}, {5, 1, 0.375}, {7, 1, 1.5}};
        for (double[] t : tops) {
            dev.mcmod.aerocore.api.ground.Ground.Hit hit = ground.cast(above(h, (int) t[0], (int) t[1]), new Vec3d(0, -1, 0), 4);
            h.assertTrue(hit != null && Math.abs(hit.point().y() - (base + t[2])) < 1e-6 && hit.normal().y() > 0.999,
                    "ray onto the block at x " + t[0] + ": " + hit + ", expected y " + (base + t[2]));
        }
        String[][] surfaces = {{"1", "ice"}, {"3", "sand"}, {"5", "soil"}};
        for (String[] s : surfaces) {
            dev.mcmod.aerocore.api.ground.Ground.Hit hit = ground.cast(above(h, Integer.parseInt(s[0]), 3), new Vec3d(0, -1, 0), 4);
            h.assertTrue(hit != null && hit.surface().name().equals(s[1]), "surface at x " + s[0] + ": " + (hit == null ? null : hit.surface().name()));
        }
        h.assertTrue(ground.cast(above(h, 1, 1), new Vec3d(0, -1, 0), 4).surface().name().equals("tarmac"), "stone is hard ground");
        // along x into the stone block's west face
        BlockPos stone = h.absolutePos(new BlockPos(1, 1, 1));
        dev.mcmod.aerocore.api.ground.Ground.Hit side = ground.cast(new Vec3d(stone.getX() - 0.9, stone.getY() + 0.5, stone.getZ() + 0.5), Vec3d.X, 4);
        h.assertTrue(side != null && Math.abs(side.distanceM() - 0.9) < 1e-6 && side.normal().x() < -0.999, "the block's side: " + side);
        h.succeed();
    }

    /**
     * Two balls side by side make one vehicle, thrown sideways; one asks to be cast off (0.28). The vehicle goes on as
     * the other ball alone: half the mass, its centre where that ball was, moving on as before; the event names the ball
     * that went, where it was and how fast it moved.
     */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 60)
    public static void aPartAskingToGoIsCastOff(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        VehicleBlueprint blueprint = new VehicleBlueprint("pair", List.of(
                new VehicleBlueprint.Entry("left", new Ball("ball", 0.4, 200), Mount.at(new Vec3d(0, 0, -1))),
                new VehicleBlueprint.Entry("right", new Ball("ball", 0.4, 200), Mount.at(new Vec3d(0, 0, 1)), List.of(Binding.of("drop", "drop")))), List.of());
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        PartVehicle v = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
        if (v == null) { throw new IllegalStateException("Vehicle factory failed"); }
        v.setBlueprint(blueprint, new Vec3d(o.getX() + 6, o.getY() + 10, o.getZ() + 8), Orientation.IDENTITY);
        h.getLevel().addFreshEntity(v);
        List<PartVehicle.Jettisoned> heard = new java.util.ArrayList<>();
        java.util.function.Consumer<PartVehicle.Jettisoned> listener = e -> { if (e.vehicle() == v) { heard.add(e); } };
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.addListener(PartVehicle.Jettisoned.class, listener);
        final dev.mcmod.aerocore.api.RigidBodyState[] before = {null};
        h.runAfterDelay(2, () -> {
            var s = v.vehicleState().body();
            v.vehicleState().setBody(new dev.mcmod.aerocore.api.RigidBodyState(s.position(), new Vec3d(3, 0, 0), s.orientation(), Vec3d.ZERO));
        });
        h.runAfterDelay(5, () -> { before[0] = v.vehicleState().body(); v.setControl("drop", 1); });
        h.runAfterDelay(7, () -> {
            net.neoforged.neoforge.common.NeoForge.EVENT_BUS.unregister(listener);
            h.assertTrue(v.blueprint().parts().size() == 1 && v.blueprint().parts().getFirst().id().equals("left"), "Only the left ball must be left: " + v.blueprint().parts());
            h.assertTrue(Math.abs(v.vehicle().properties().massKg() - 200) < 1e-9, "Half the mass must be left: " + v.vehicle().properties().massKg());
            h.assertTrue(heard.size() == 1 && heard.getFirst().parts().size() == 1 && heard.getFirst().parts().getFirst().entry().id().equals("right"),
                    "The event must name the ball that went: " + heard);
            var dropped = heard.getFirst().parts().getFirst();
            h.assertTrue(Math.abs(dropped.velocity().x() - 3) < 0.05, "The ball went at the vehicle's speed: " + dropped.velocity());
            var now = v.vehicleState().body();
            h.assertTrue(Math.abs(now.velocity().x() - 3) < 0.05 && now.velocity().y() < before[0].velocity().y(), "The rest must fly on: " + now.velocity());
            h.assertTrue(Math.abs(now.position().z() - (before[0].position().z() - 1)) < 1e-6, "The rest's centre is where the left ball was: " + now.position());
            h.succeed();
        });
    }

    /**
     * A heavy ball shot level at Mach 2 and at Mach 5, high over the ground (0.28): it reads its obstacles along its
     * path, keeps the chunks ahead loaded and flies on; where they are not ready yet it waits with its speed kept, so it
     * is never stopped (how far it gets in 2 s depends on how fast the test world generates terrain ahead of it).
     */
    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 200, batch = "aerocore_fast")
    public static void fastVehiclesFlyOnThroughTheirChunks(GameTestHelper h) {
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        double[] machs = {2, 5};
        PartVehicle[] balls = new PartVehicle[2];
        Vec3d[] start = new Vec3d[2];
        for (int k = 0; k < 2; k++) {
            VehicleBlueprint blueprint = new VehicleBlueprint("shot", List.of(new VehicleBlueprint.Entry("ball", new Ball("ball", 0.4, 1e7), Mount.ORIGIN)), List.of());
            PartVehicle v = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
            if (v == null) { throw new IllegalStateException("Vehicle factory failed"); }
            // high over any terrain (the test world is a normal one, its tests near the bottom): a straight flight in open air
            v.setBlueprint(blueprint, new Vec3d(o.getX() + 2, 290 + 10 * k, o.getZ() + 2 + 4 * k), Orientation.IDENTITY);
            v.vehicle().warmUp();
            h.getLevel().addFreshEntity(v);
            var s = v.vehicleState().body();
            double speed = machs[k] * 340;
            v.vehicleState().setBody(new dev.mcmod.aerocore.api.RigidBodyState(s.position(), new Vec3d(speed, 0, 0), s.orientation(), Vec3d.ZERO));
            balls[k] = v;
            start[k] = s.position();
        }
        h.runAfterDelay(40, () -> {
            // in the test world's unexplored terrain the chunks ahead are generated as it goes: it waits most of the
            // time (it flies as far as the world is generated), keeping its speed
            StringBuilder report = new StringBuilder();
            for (int k = 0; k < 2; k++) {
                var s = balls[k].vehicleState().body();
                report.append(String.format(" Mach %.0f: %.0f m, %.0f m/s;", machs[k], s.position().x() - start[k].x(), s.velocity().x()));
            }
            for (int k = 0; k < 2; k++) {
                var s = balls[k].vehicleState().body();
                double speed = machs[k] * 340, flown = s.position().x() - start[k].x();
                h.assertTrue(s.velocity().x() > 0.95 * speed, "Mach " + machs[k] + ": its speed must be kept, not stopped:" + report);
                h.assertTrue(flown >= 0.05 * speed * 0.99, "Mach " + machs[k] + ": it must have flown on, a tick's travel at least:" + report);
                h.assertTrue(balls[k].fault() == null, "no physics error: " + balls[k].fault());
            }
            com.mojang.logging.LogUtils.getLogger().info("Fast flight GameTest:{}", report);
            for (PartVehicle v : balls) { v.discard(); }
            h.succeed();
        });
    }

    /**
     * A ball at 2000 m/s (100 blocks a tick) at a wall one block thick 18 m ahead (0.28): within the first tick it
     * reaches the wall and bounces; it must not pass through. It is taken away after that tick (bouncing back at
     * hundreds of metres a second it would fly into the neighbouring tests).
     */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 40, batch = "aerocore_fast_wall")
    public static void aFastBallDoesNotTunnelThroughAWall(GameTestHelper h) {
        for (int y = 1; y < 12; y++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(20, y, z), Blocks.STONE); }
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        VehicleBlueprint blueprint = new VehicleBlueprint("shot", List.of(new VehicleBlueprint.Entry("ball", new Ball("ball", 0.4, 200), Mount.ORIGIN)), List.of());
        PartVehicle v = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
        if (v == null) { throw new IllegalStateException("Vehicle factory failed"); }
        v.setBlueprint(blueprint, new Vec3d(o.getX() + 2, o.getY() + 6, o.getZ() + 8), Orientation.IDENTITY);
        v.vehicle().warmUp();
        h.getLevel().addFreshEntity(v);
        var s0 = v.vehicleState().body();
        double start = s0.position().x(), wall = o.getX() + 20;
        v.vehicleState().setBody(new dev.mcmod.aerocore.api.RigidBodyState(s0.position(), new Vec3d(2000, 0, 0), s0.orientation(), Vec3d.ZERO));
        h.succeedWhen(() -> {
            var s = v.vehicleState().body();
            h.assertTrue(s.position().x() < wall, "The ball must not pass the wall (x " + wall + "): " + s.position().x());
            if (s.position().x() == start) { throw new net.minecraft.gametest.framework.GameTestAssertException("The ball has not moved yet"); }
            h.assertTrue(s.velocity().x() < 0, "It must have hit the wall and bounced: " + s.velocity());
            v.discard();
        });
    }

    /** Three metres above the middle of a test cell (x, 1, z). */
    private static Vec3d above(GameTestHelper h, int x, int z) {
        BlockPos p = h.absolutePos(new BlockPos(x, 1, z));
        return new Vec3d(p.getX() + 0.5, p.getY() + 3, p.getZ() + 0.5);
    }

    /**
     * 0.36: two balls on a stone floor, the second hung on the first by a ball joint between them. Thrown along x, the
     * leader drags the follower with it, the points held together; the link saves and loads; a third coupling refuses
     * (too far, already coupled, a cycle); the follower's physics fault ends the link, and so does removing the leader.
     */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 80)
    public static void aCoupledVehicleIsDraggedAlong(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        PartVehicle leader = resting(h, 6), follower = resting(h, 7.2), other = resting(h, 12);
        follower.addTag(BodyWorld.EXPECTED_FAULT_TAG);
        Coupling ball = Coupling.ball(new Vec3d(0.6, 0, 0), new Vec3d(-0.6, 0, 0));
        final double[] start = {0, 0};
        h.runAfterDelay(5, () -> {
            h.assertTrue(leader.couple(follower, ball) == Couplings.Result.OK, "coupled");
            h.assertTrue(follower.coupling() == ball && follower.leader() == leader && leader.followers().equals(List.of(follower)), "the link both ways");
            h.assertTrue(leader.couple(other, ball) == Couplings.Result.TOO_FAR, "too far");
            h.assertTrue(other.couple(follower, ball.withCapture(10)) == Couplings.Result.ALREADY, "already coupled");
            h.assertTrue(follower.couple(leader, ball.withCapture(10)) == Couplings.Result.CYCLE, "a cycle");
            RigidBodyState s = leader.vehicleState().body();
            leader.vehicleState().setBody(new RigidBodyState(s.position(), new Vec3d(3, 0, 0), s.orientation(), s.angularMomentum()));
            start[0] = follower.vehicleState().body().position().x();
            start[1] = leader.vehicleState().body().position().x();
        });
        h.runAfterDelay(20, () -> {
            double gap = ball.geometry(leader.vehicleState().body(), leader.vehicle().properties(), follower.vehicleState().body(), follower.vehicle().properties()).gap().length();
            double moved = follower.vehicleState().body().position().x() - start[0], led = leader.vehicleState().body().position().x() - start[1];
            h.assertTrue(moved > 0.2 && Math.abs(moved - led) < 0.1, "the follower is dragged along about as far as the leader went (a ball lets it swing): " + moved + " and " + led);
            h.assertTrue(gap < 0.01, "the points together: " + gap);
            h.assertTrue(follower.couplingForceN().length() > 0, "the joint carries a force");
            CompoundTag tag = new CompoundTag();
            follower.saveWithoutId(tag);
            PartVehicle copy = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
            if (copy == null) { throw new IllegalStateException("Vehicle factory failed"); }
            copy.load(tag);
            h.assertTrue(ball.equals(copy.leaderCoupling()) && leader.getUUID().equals(copy.leaderId()), "the link comes back from the save");
            CompoundTag towing = new CompoundTag();
            leader.saveWithoutId(towing);
            h.assertTrue(towing.getList("Towing", net.minecraft.nbt.Tag.TAG_COMPOUND).size() == 1, "the leader saves its follower");
            follower.setControl("fail", 1);
        });
        h.runAfterDelay(23, () -> {
            h.assertTrue(follower.fault() != null && follower.coupling() == null && leader.followers().isEmpty(), "a fault ends the link");
            follower.setControl("fail", 0);
            follower.resume();
            h.assertTrue(leader.couple(other, ball.withCapture(100)) == Couplings.Result.OK, "coupled again (another)");
            leader.discard();
            h.assertTrue(other.coupling() == null && other.leaderId() == null, "removing the leader ends the link");
            h.succeed();
        });
    }

    /** 0.37: Contacted events by vehicle entity id, collected while the GameTests run. */
    private static final Map<Integer, List<VehicleContacts.Contacted>> CONTACTS = new java.util.concurrent.ConcurrentHashMap<>();
    static {
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.addListener((VehicleContacts.Contacted e) ->
                CONTACTS.computeIfAbsent(e.vehicle().getId(), k -> new java.util.concurrent.CopyOnWriteArrayList<>()).add(e));
    }

    /**
     * 0.37: a ball falling 5.6 m onto stone is one impact: the closing speed it met at, and an impulse equal to its mass
     * times its change of velocity over that tick (within the tick's gravity); resting there afterwards it makes no
     * more. A second ball thrown along the floor at 6 m/s slides (scrapes) until it rolls.
     */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 100)
    public static void aLandingIsAnImpactAndASlideAScrape(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        PartVehicle drop = spawn(h, 6), slide = resting(h, 14);
        double mass = drop.vehicle().properties().massKg();
        final Vec3d[] last = {null};
        final double[] jump = {0, 0};   // the largest upward change of velocity in a tick, and the speed it fell at before
        for (int tick = 1; tick <= 60; tick++) {
            h.runAfterDelay(tick, () -> {
                Vec3d v = drop.vehicleState().body().velocity();
                if (last[0] != null && v.y() - last[0].y() > jump[0]) { jump[0] = v.y() - last[0].y(); jump[1] = -last[0].y(); }
                last[0] = v;
            });
        }
        h.runAfterDelay(3, () -> {
            RigidBodyState s = slide.vehicleState().body();
            slide.vehicleState().setBody(new RigidBodyState(s.position(), new Vec3d(6, 0, 0), s.orientation(), s.angularMomentum()));
        });
        h.runAfterDelay(60, () -> {
            List<VehicleContacts.Contacted> events = CONTACTS.getOrDefault(drop.getId(), List.of());
            List<VehicleContacts.Impact> impacts = events.stream().map(VehicleContacts.Contacted::impact).filter(java.util.Objects::nonNull).toList();
            h.assertTrue(!impacts.isEmpty(), "the landing must be an impact");
            VehicleContacts.Impact first = impacts.getFirst();
            h.assertTrue(Math.abs(first.approachMps() - jump[1]) < 0.15 * jump[1], "met at its falling speed: " + first.approachMps() + " vs " + jump[1]);
            // the floor's impulse gives the tick's change of velocity plus what gravity took meanwhile
            double expected = mass * (jump[0] + 9.81 * 0.05);
            h.assertTrue(Math.abs(first.impulseNs() - expected) < 0.2 * expected, "impulse m dv: " + first.impulseNs() + " vs " + expected);
            h.assertTrue(first.material() == VehicleContacts.materialIndex(dev.mcmod.aerocore.api.ContactMaterial.STEEL_ON_ROCK) && !first.body(), "against stone");
            h.assertTrue(Math.abs(first.point().y() - (h.absolutePos(BlockPos.ZERO).getY() + 1)) < 0.1, "at the floor: " + first.point());
            double settled = impacts.stream().filter(i -> i.approachMps() < 1).count();
            h.assertTrue(impacts.size() <= 6 && settled <= 4, "it bounces a few times at most and then makes no more: " + impacts);
            List<VehicleContacts.Scrape> scrapes = CONTACTS.getOrDefault(slide.getId(), List.of()).stream().map(VehicleContacts.Contacted::scrape)
                    .filter(java.util.Objects::nonNull).toList();
            h.assertTrue(!scrapes.isEmpty() && scrapes.getFirst().slideMps() > 3 && scrapes.getFirst().powerW() > 1000, "the thrown ball scrapes first: " + scrapes);
            h.assertTrue(scrapes.size() < 30, "and rolls afterwards: " + scrapes.size() + " scraping ticks");
            h.succeed();
        });
    }

    /** A ball resting on the floor (its centre 0.4 m over it) at x. */
    /**
     * 0.38: two balls resting 8 m apart on stone. VehicleTracks.near finds the one within 3 m of the first and both
     * within 10 m, nearest first; a track has the vehicle's centre and velocity, the ground under it and whether it
     * touches; a removed vehicle is left out.
     */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 60, batch = "aerocore_tracks")
    public static void tracksFindTheVehiclesNearAPoint(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        PartVehicle a = resting(h, 6), b = resting(h, 14);
        h.runAfterDelay(20, () -> {
            net.minecraft.server.level.ServerLevel level = h.getLevel();
            Vec3d centre = new Vec3d(a.getX(), a.getY(), a.getZ());
            List<java.util.UUID> close = VehicleTracks.near(level, centre, 3).stream().map(VehicleTracks.Track::id).toList();
            h.assertTrue(close.contains(a.getUUID()) && !close.contains(b.getUUID()), "only the near one within 3 m: " + close);
            List<java.util.UUID> wide = VehicleTracks.near(level, centre, 10).stream().map(VehicleTracks.Track::id).toList();
            h.assertTrue(wide.contains(b.getUUID()) && wide.indexOf(a.getUUID()) < wide.indexOf(b.getUUID()), "both within 10 m, nearest first: " + wide);
            VehicleTracks.Track t = VehicleTracks.of(a);
            h.assertTrue(t.position().subtract(a.vehicleState().body().position()).length() < 1e-9, "the centre of mass");
            h.assertTrue(t.velocity().length() < 0.05 && t.groundSpeedMps() < 0.05, "at rest: " + t.velocity());
            h.assertTrue(Math.abs(t.heightAboveGroundM() - 0.4) < 0.05, "a 0.4 m ball on the floor: " + t.heightAboveGroundM());
            h.assertTrue(t.touching() && t.ready() && t.massKg() == 200, "touching the floor, ready, 200 kg");
            h.assertTrue(Math.abs(t.headingDeg() - 90) < 1e-6, "an unturned vehicle's nose (+x) points east");
            b.discard();
        });
        h.runAfterDelay(22, () -> {
            h.assertTrue(VehicleTracks.near(h.getLevel(), new Vec3d(a.getX(), a.getY(), a.getZ()), 10).stream().noneMatch(t -> t.id().equals(b.getUUID())),
                    "a removed vehicle is left out");
            h.succeed();
        });
    }

    private static PartVehicle resting(GameTestHelper h, double x) {
        VehicleBlueprint blueprint = new VehicleBlueprint("test ball", List.of(
                new VehicleBlueprint.Entry("ball", new Ball("ball", 0.4, 200), Mount.ORIGIN, List.of(Binding.of("fail", "fail")))), List.of());
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        PartVehicle v = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
        if (v == null) { throw new IllegalStateException("Vehicle factory failed"); }
        v.setBlueprint(blueprint, new Vec3d(o.getX() + x, o.getY() + 1.4, o.getZ() + 8), Orientation.IDENTITY);
        h.getLevel().addFreshEntity(v);
        return v;
    }

    private static PartVehicle spawn(GameTestHelper h, double x) {
        VehicleBlueprint blueprint = new VehicleBlueprint("test ball", List.of(
                new VehicleBlueprint.Entry("ball", new Ball("ball", 0.4, 200), Mount.ORIGIN, List.of(Binding.of("fail", "fail")))), List.of());
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        PartVehicle v = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
        if (v == null) { throw new IllegalStateException("Vehicle factory failed"); }
        v.setBlueprint(blueprint, new Vec3d(o.getX() + x, o.getY() + 7, o.getZ() + 8), Orientation.IDENTITY);
        h.getLevel().addFreshEntity(v);
        return v;
    }
}
