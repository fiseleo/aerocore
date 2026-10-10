package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.*;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;

/** The slender-hull demonstrator in a 22 x 14 m, 4 m deep pool (hull_pool template, 24 x 10 x 16). */
@PrefixGameTestTemplate(false)
public final class HullGameTests {
    private HullGameTests() {}

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 420)
    public static void hullFloatsLevelAtItsDesignWaterline(GameTestHelper h) {
        double surface = pool(h);
        double waterline = TestHull.model().design().waterlineY();
        FloatingHull hull = spawn(h, 11, surface - waterline + 0.05, 0);
        final double[] sum = {0, 0, 0};
        for (int tick = 300; tick < 400; tick++) {
            h.runAfterDelay(tick, () -> {
                Vec3d origin = hull.modelOrigin(), ahead = hull.physicsState().orientation().rotate(Vec3d.X);
                sum[0] += origin.y() + waterline - surface;
                sum[1] += Math.abs(ahead.y());
                sum[2]++;
            });
        }
        h.runAfterDelay(401, () -> {
            double sinkage = sum[0] / sum[2], trim = sum[1] / sum[2];
            h.assertTrue(Math.abs(sinkage) < 0.02, "Hull must float at its design waterline: " + sinkage + " m off");
            h.assertTrue(trim < 0.01, "Hull must float level: " + trim);
            h.assertTrue(hull.wetVolumeM3() > 0.97 * TestHull.model().designVolume() && hull.wetVolumeM3() < 1.03 * TestHull.model().designVolume(),
                    "Displacement must equal the design volume: " + hull.wetVolumeM3());
            h.assertTrue(h.getBlockState(new BlockPos(11, 5, 8)).isAir(), "The hull must not place water");
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 200)
    public static void propellerDrivesAndRudderTurnsToStarboard(GameTestHelper h) {
        double surface = pool(h);
        FloatingHull hull = spawn(h, 6, surface - TestHull.model().design().waterlineY(), 0);
        hull.setPropellerRps(8);
        hull.setRudderDegrees(25);
        Vec3d start = hull.modelOrigin();
        final double[] turn = {0};
        for (int tick = 1; tick <= 140; tick++) { h.runAfterDelay(tick, () -> turn[0] += hull.physicsState().angularVelocity(hull.properties()).y()); }
        h.runAfterDelay(141, () -> {
            Vec3d moved = hull.modelOrigin().subtract(start);
            h.assertTrue(moved.x() > 1.5, "Thrust must drive the boat ahead: " + moved);
            // Bow to starboard is a negative rotation about +y; with the bow along +x it also drifts to +z.
            h.assertTrue(turn[0] < 0, "Positive rudder must turn the bow to starboard: " + turn[0]);
            h.assertTrue(hull.thrustN() > 0 && hull.resistanceN() > 0, "Thrust and resistance must be reported");
            h.assertTrue(hull.displayForces().stream().anyMatch(f -> f.kind() == AppliedForce.Kind.LIFT), "Rudder and hull lift must be displayed");
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 200)
    public static void floatingBodiesPushEachOtherWithEqualAndOppositeImpulses(GameTestHelper h) {
        double surface = pool(h);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        FloatingPlatform moving = ProbeEntities.FLOATING_PLATFORM.get().create(h.getLevel()), resting = ProbeEntities.FLOATING_PLATFORM.get().create(h.getLevel());
        if (moving == null || resting == null) { throw new IllegalStateException("Platform factory failed"); }
        // Model origins at the floating height (0.106 m above the still level), 6 m apart.
        moving.configure(null, new Vec3d(o.getX() + 7, surface + 0.106, o.getZ() + 8));
        resting.configure(null, new Vec3d(o.getX() + 13, surface + 0.106, o.getZ() + 8));
        h.getLevel().addFreshEntity(moving); h.getLevel().addFreshEntity(resting);
        moving.setVelocityMps(Vec3d.X.scale(2));
        double mass = moving.properties().massKg();
        final double[] closest = {Double.MAX_VALUE}, before = {0}, contactTick = {-1};
        for (int tick = 1; tick <= 150; tick++) {
            int index = tick;
            h.runAfterDelay(tick, () -> {
                double gap = resting.modelOrigin().x() - moving.modelOrigin().x();
                closest[0] = Math.min(closest[0], gap);
                if (contactTick[0] < 0 && resting.physicsState().velocity().x() > 0.05) { contactTick[0] = index; }
                if (contactTick[0] < 0) { before[0] = mass * (moving.physicsState().velocity().x() + resting.physicsState().velocity().x()); }
            });
        }
        h.runAfterDelay(151, () -> {
            double after = mass * (moving.physicsState().velocity().x() + resting.physicsState().velocity().x());
            h.assertTrue(contactTick[0] > 0, "The moving platform must reach and push the resting one");
            // Float spheres (r 0.45 m at +-0.85 m) touch at a centre distance of 2.6 m; allow the contact slop.
            h.assertTrue(closest[0] > 2.5, "Bodies must not pass through each other: closest " + closest[0]);
            h.assertTrue(resting.physicsState().velocity().x() > 0.2, "Momentum must pass to the resting platform: " + resting.physicsState().velocity());
            h.assertTrue(moving.physicsState().velocity().x() < resting.physicsState().velocity().x() + 0.05, "The pusher must not overtake");
            // Contact impulses cancel; only water drag removes momentum over the remaining ticks.
            h.assertTrue(after > 0.3 * before[0] && after <= before[0] * 1.001, "Pair momentum " + after + " vs " + before[0] + " before contact");
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 260)
    public static void standingEntityTiltsThePlatformAndIsCarried(GameTestHelper h) {
        double surface = pool(h);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        FloatingPlatform platform = ProbeEntities.FLOATING_PLATFORM.get().create(h.getLevel());
        if (platform == null) { throw new IllegalStateException("Platform factory failed"); }
        platform.configure(null, new Vec3d(o.getX() + 6, surface + 0.106, o.getZ() + 8));
        h.getLevel().addFreshEntity(platform);
        // An armor stand (about 54 kg by its box) dropped near the +X edge of the deck (deck top at local y 0.25).
        net.minecraft.world.entity.decoration.ArmorStand stand = net.minecraft.world.entity.EntityType.ARMOR_STAND.create(h.getLevel());
        if (stand == null) { throw new IllegalStateException("Armor stand factory failed"); }
        Vec3d drop = platform.toWorld(new Vec3d(0.9, 0.6, 0));
        stand.moveTo(drop.x(), drop.y(), drop.z(), 0, 0);
        h.getLevel().addFreshEntity(stand);
        final double[] standStart = {0}, platformStart = {0};
        // From tick 10 the stand must stay on the deck top while the platform settles and tilts under it.
        final double[] worst = {0};
        for (int tick = 10; tick < 100; tick++) {
            h.runAfterDelay(tick, () -> worst[0] = Math.max(worst[0], Math.abs(platform.toLocal(new Vec3d(stand.getX(), stand.getY(), stand.getZ())).y() - 0.25)));
        }
        h.runAfterDelay(100, () -> {
            Vec3d feet = platform.toLocal(new Vec3d(stand.getX(), stand.getY(), stand.getZ()));
            h.assertTrue(worst[0] < 0.05, "The stand must stay on the deck top: worst offset " + worst[0] + " m, now local y " + feet.y());
            Vec3d up = platform.physicsState().orientation().rotate(Vec3d.Y);
            h.assertTrue(up.x() > 0.002, "Its weight must lower the +X edge: " + up);
            h.assertTrue(platform.wetFractions().get(1) > platform.wetFractions().get(0), "+X floats must sink deeper: " + platform.wetFractions());
            standStart[0] = stand.getX(); platformStart[0] = platform.modelOrigin().x();
            platform.setPropellerRps(15);
        });
        h.runAfterDelay(200, () -> {
            double platformMoved = platform.modelOrigin().x() - platformStart[0], standMoved = stand.getX() - standStart[0];
            h.assertTrue(platformMoved > 0.8, "The propeller must move the platform: " + platformMoved);
            h.assertTrue(Math.abs(standMoved - platformMoved) < 0.15, "The deck must carry the stand: " + standMoved + " vs " + platformMoved);
            Vec3d feet = platform.toLocal(new Vec3d(stand.getX(), stand.getY(), stand.getZ()));
            h.assertTrue(Math.abs(feet.y() - 0.25) < 0.08 && !stand.isInWater(), "The stand must still stand on the deck: " + feet);
            h.succeed();
        });
    }

    /** Stone floor and walls with 4 m of water inside; returns the absolute still-water level. */
    private static double pool(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) {
            h.setBlock(new BlockPos(x, 0, z), Blocks.STONE);
            for (int y = 1; y <= 5; y++) {
                boolean wall = x == 0 || z == 0 || x == 23 || z == 15;
                if (wall) { h.setBlock(new BlockPos(x, y, z), Blocks.STONE); }
                else if (y <= 4) { h.setBlock(new BlockPos(x, y, z), Blocks.WATER); }
            }
        }
        BlockPos column = h.absolutePos(new BlockPos(11, 4, 8));
        return new WorldWaterSampler(h.getLevel()).stillSurface(column);
    }
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 260)
    public static void planingBoatFloatsAtItsStaticTrim(GameTestHelper h) {
        double surface = pool(h);
        PlaningBoat boat = spawnPlaning(h, 4, surface - 0.12);
        final double[] trim = {0, 0};
        for (int tick = 200; tick < 240; tick++) {
            h.runAfterDelay(tick, () -> {
                Vec3d ahead = boat.physicsState().orientation().rotate(Vec3d.X);
                trim[0] += Math.toDegrees(Math.asin(ahead.y())); trim[1]++;
            });
        }
        h.runAfterDelay(241, () -> {
            double mean = trim[0] / trim[1];
            // Same hydrostatics as PlaningDynamicsTest's model-scale configuration A (Fridsma tau_0 0.86 deg).
            h.assertTrue(Math.abs(mean - 0.86) < 0.4, "Planing boat must float at its static trim: " + mean);
            h.assertTrue(boat.physicsState().velocity().length() < 0.1, "It must come to rest: " + boat.physicsState().velocity());
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120)
    public static void planingBoatAcceleratesUnderItsPropeller(GameTestHelper h) {
        double surface = pool(h);
        PlaningBoat boat = spawnPlaning(h, 2.5, surface - 0.12);
        h.runAfterDelay(40, () -> boat.setPropellerRps(30));
        final Vec3d[] start = new Vec3d[1];
        h.runAfterDelay(41, () -> start[0] = boat.modelOrigin());
        h.runAfterDelay(71, () -> {
            Vec3d moved = boat.modelOrigin().subtract(start[0]);
            h.assertTrue(moved.x() > 2, "Thrust must drive the planing boat ahead: " + moved);
            h.assertTrue(boat.thrustN() > 0, "Thrust must be reported");
            Vec3d ahead = boat.physicsState().orientation().rotate(Vec3d.X);
            h.assertTrue(ahead.y() > 0, "Accelerating, the bow must rise: " + ahead);
            boat.setPropellerRps(0);
            h.succeed();
        });
    }

    private static PlaningBoat spawnPlaning(GameTestHelper h, double x, double originY) {
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        PlaningBoat boat = ProbeEntities.PLANING_BOAT.get().create(h.getLevel());
        if (boat == null) { throw new IllegalStateException("Planing boat factory failed"); }
        boat.configure(null, new Vec3d(o.getX() + x, originY, o.getZ() + 8), 0);
        h.getLevel().addFreshEntity(boat);
        return boat;
    }

    private static FloatingHull spawn(GameTestHelper h, double x, double originY, double heading) {
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        FloatingHull hull = ProbeEntities.FLOATING_HULL.get().create(h.getLevel());
        if (hull == null) { throw new IllegalStateException("Hull factory failed"); }
        hull.configure(null, new Vec3d(o.getX() + x, originY, o.getZ() + 8), heading);
        h.getLevel().addFreshEntity(hull);
        return hull;
    }
}
