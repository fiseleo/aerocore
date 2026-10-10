package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Vec3d;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

/** Loaded only by the explicit AeroCore GameTest run; never modifies a player's save. */
@PrefixGameTestTemplate(false)
public final class ProbeGameTests {
    private ProbeGameTests() {}

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 160)
    public static void dryProbeFallsAndStopsOnGround(GameTestHelper helper) {
        floor(helper);
        PhysicsProbe probe = spawn(helper, 250, 4, 5, 4);
        double groundY = helper.absolutePos(BlockPos.ZERO).getY() + 1;
        helper.runAfterDelay(120, () -> {
            helper.assertTrue(Math.abs(probe.getY() - groundY) < 0.01, "Probe must stop on the floor: y=" + probe.getY() + ", ticks=" + probe.tickCount + ", removed=" + probe.isRemoved() + ", v=" + probe.velocityMps() + ", free=" + helper.getLevel().noCollision(probe, probe.getBoundingBox()) + ", chunks=" + helper.getLevel().hasChunksAt(probe.blockPosition().offset(-1, 0, -1), probe.blockPosition().offset(1, 0, 1)) + ", info=" + probe.diagnostics().getString());
            helper.assertTrue(probe.velocityMps().length() < 0.02, "Resting probe must not keep falling");
            helper.assertTrue(probe.wetFraction() == 0, "Dry blocks must not generate buoyancy even below sea level");
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 90)
    public static void launchedProbeDoesNotPassThroughWall(GameTestHelper helper) {
        floor(helper);
        for (int y = 1; y < 8; y++) {
            for (int z = 0; z < 9; z++) { helper.setBlock(new BlockPos(6, y, z), Blocks.STONE); }
        }
        PhysicsProbe probe = spawn(helper, 250, 3, 5, 4);
        probe.setVelocityMps(Vec3d.X.scale(30));
        double wallX = helper.absolutePos(new BlockPos(6, 0, 0)).getX();
        final double[] reach = {Double.NEGATIVE_INFINITY};
        for (int tick = 1; tick <= 60; tick++) {
            helper.runAfterDelay(tick, () -> {
                reach[0] = Math.max(reach[0], probe.getBoundingBox().maxX);
                helper.assertTrue(probe.getBoundingBox().maxX <= wallX + 0.01, "Probe must not penetrate wall: " + (probe.getBoundingBox().maxX - wallX));
            });
        }
        helper.runAfterDelay(61, () -> {
            // Speculative contact may rebound up to v*dt (0.15 m at 30 m/s) before the face.
            helper.assertTrue(reach[0] > wallX - 0.2, "Probe must reach the wall: " + (reach[0] - wallX));
            helper.assertTrue(probe.velocityMps().x() < 0.5, "Wall reaction must stop or reverse the approach: " + probe.velocityMps());
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 80)
    public static void probeBouncesOffStoneButNotOffSand(GameTestHelper helper) {
        // Block materials (BlockMaterials): granite-like stone returns a dropped steel ball, loose sand swallows the impact.
        floor(helper);
        for (int z = 0; z < 9; z++) for (int x = 0; x < 4; x++) { helper.setBlock(new BlockPos(x, 1, z), Blocks.SAND); }
        for (int z = 0; z < 9; z++) for (int x = 5; x < 9; x++) { helper.setBlock(new BlockPos(x, 1, z), Blocks.STONE); }
        PhysicsProbe onSand = spawn(helper, 250, 1.5, 5, 4), onStone = spawn(helper, 250, 6.5, 5, 4);
        final double[] rebound = {0, 0};
        for (int tick = 1; tick <= 60; tick++) {
            helper.runAfterDelay(tick, () -> {
                rebound[0] = Math.max(rebound[0], onSand.velocityMps().y());
                rebound[1] = Math.max(rebound[1], onStone.velocityMps().y());
            });
        }
        helper.runAfterDelay(61, () -> {
            // About 7.7 m/s at impact: Thornton's e for steel on granite is about 0.3 there, sand gives none.
            helper.assertTrue(rebound[1] > 1.0, "Stone must return the ball: " + rebound[1]);
            helper.assertTrue(rebound[0] < 0.2, "Sand must not: " + rebound[0]);
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 60)
    public static void launchedProbeRollsWithoutSlipping(GameTestHelper helper) {
        // Solid sphere sliding on level ground: friction spins it up until v = 5/7 v0, then it rolls (v = omega R).
        floor(helper);
        PhysicsProbe probe = spawn(helper, 250, 1, 1, 4);
        probe.setVelocityMps(Vec3d.X.scale(5));
        helper.runAfterDelay(30, () -> {
            double v = probe.velocityMps().x(), slip = v + probe.angularVelocity().z() * PhysicsProbe.RADIUS_M;
            helper.assertTrue(Math.abs(v - 25.0 / 7) < 0.15, "Rolling speed must approach 5/7 of 5 m/s: " + v);
            helper.assertTrue(Math.abs(slip) < 0.05, "Contact point must stop slipping: " + slip);
            helper.assertTrue(probe.contactForceN().y() > 0, "Floor must carry the probe");
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 700)
    public static void lightProbeFloatsInRealWater(GameTestHelper helper) {
        pool(helper);
        PhysicsProbe probe = spawn(helper, 250, 4, 5, 4);
        double surface = waterSurface(helper);
        helper.runAfterDelay(600, () -> {
            helper.assertTrue(Math.abs(probe.centerM().y() - surface) < 0.15, "Light sphere should settle around the surface: " + probe.centerM().y());
            helper.assertTrue(probe.wetFraction() > 0.32 && probe.wetFraction() < 0.62, "Expected roughly 48% displacement, got " + probe.wetFraction());
            helper.assertTrue(Math.abs(probe.velocityMps().y()) < 0.4, "Floating oscillation should decay");
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 250)
    public static void heavyProbeSinksAndStopsOnPoolBottom(GameTestHelper helper) {
        pool(helper);
        PhysicsProbe probe = spawn(helper, 1000, 4, 5, 4);
        double bottom = helper.absolutePos(BlockPos.ZERO).getY() + 1;
        helper.runAfterDelay(200, () -> {
            helper.assertTrue(Math.abs(probe.getY() - bottom) < 0.01, "Heavy sphere must sink to the floor");
            helper.assertTrue(probe.wetFraction() > 0.99, "Sphere at pool bottom must be fully immersed");
            helper.assertTrue(Math.abs(probe.velocityMps().y()) < 0.02, "Bottom collision must stop sinking");
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 100)
    public static void currentAcceleratesSubmergedProbe(GameTestHelper helper) {
        pool(helper);
        double neutralMass = FluidState.FRESH_WATER_20C.densityKgPerM3() * 4.0 / 3 * Math.PI * 0.125;
        PhysicsProbe probe = spawn(helper, neutralMass, 4, 2, 4);
        probe.setAddedCurrentMps(Vec3d.X);
        double startX = probe.getX();
        // At 1 m/s relative speed the 1 m sphere is at Re 1e6, past the drag crisis (C_D about 0.11): it takes a few
        // seconds to pick up the current.
        helper.runAfterDelay(80, () -> {
            helper.assertTrue(probe.getX() - startX > 0.15, "Water current must advect the sphere: " + (probe.getX() - startX));
            helper.assertTrue(probe.velocityMps().x() > 0 && probe.velocityMps().x() < 1, "Drag must approach current speed without overshooting");
            helper.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "probe_empty", timeoutTicks = 5)
    public static void samplerDistinguishesDryHalfAndFullImmersion(GameTestHelper helper) {
        pool(helper);
        double surface = waterSurface(helper);
        BlockPos center = helper.absolutePos(new BlockPos(4, 0, 4));
        WorldWaterSampler sampler = new WorldWaterSampler(helper.getLevel());
        helper.assertTrue(sampler.sample(new Vec3d(center.getX() + 0.5, surface + 1, center.getZ() + 0.5), 0.5).fraction() == 0, "Above water is dry");
        helper.assertTrue(Math.abs(sampler.sample(new Vec3d(center.getX() + 0.5, surface, center.getZ() + 0.5), 0.5).fraction() - 0.5) < 1e-8, "Surface bisects the sphere");
        helper.assertTrue(sampler.sample(new Vec3d(center.getX() + 0.5, surface - 1, center.getZ() + 0.5), 0.5).fraction() > 0.99, "Underwater sphere is full");
        helper.succeed();
    }

    private static PhysicsProbe spawn(GameTestHelper helper, double mass, double x, double y, double z) {
        BlockPos origin = helper.absolutePos(BlockPos.ZERO);
        PhysicsProbe probe = ProbeEntities.PHYSICS_PROBE.get().create(helper.getLevel());
        if (probe == null) { throw new IllegalStateException("Probe type could not create entity"); }
        probe.setPos(origin.getX() + x + 0.5, origin.getY() + y, origin.getZ() + z + 0.5);
        probe.configure(mass, null, Vec3d.ZERO);
        helper.getLevel().addFreshEntity(probe);
        return probe;
    }

    private static double waterSurface(GameTestHelper helper) {
        BlockPos position = helper.absolutePos(new BlockPos(4, 3, 4));
        return position.getY() + helper.getLevel().getFluidState(position).getHeight(helper.getLevel(), position);
    }

    private static void floor(GameTestHelper helper) {
        for (int x = 0; x < 9; x++) {
            for (int z = 0; z < 9; z++) { helper.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        }
    }

    private static void pool(GameTestHelper helper) {
        floor(helper);
        for (int y = 1; y <= 4; y++) {
            for (int x = 0; x < 9; x++) {
                for (int z = 0; z < 9; z++) {
                    if (x == 0 || z == 0 || x == 8 || z == 8) { helper.setBlock(new BlockPos(x, y, z), Blocks.STONE); }
                    else if (y <= 3) { helper.setBlock(new BlockPos(x, y, z), Blocks.WATER); }
                }
            }
        }
    }
}
