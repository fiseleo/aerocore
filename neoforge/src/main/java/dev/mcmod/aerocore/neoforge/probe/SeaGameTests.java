package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CompiledWaves;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.decoration.ArmorStand;
import net.minecraft.world.entity.npc.Villager;
import net.minecraft.world.entity.vehicle.Boat;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.common.NeoForgeMod;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;

/**
 * Sea regions shared by AeroCore bodies and vanilla entities, in the 22 x 14 m hull pool (3.89 m of water). Each test
 * has its own batch: regions of neighbouring tests running at the same time would otherwise replace each other.
 */
@PrefixGameTestTemplate(false)
public final class SeaGameTests {
    private SeaGameTests() {}

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 20, batch = "aerocore_sea_command")
    public static void seaCommandRegionIsReadIdenticallyByBodiesAndVanilla(GameTestHelper h) throws Exception {
        double still = pool(h);
        BlockPos centre = h.absolutePos(new BlockPos(11, 4, 8));
        WaveRegion region = WaveCommands.createSea(h.getLevel(), centre.getX() + 0.5, still, centre.getZ() + 0.5, 0.3, 3, 30, 12,
                WaveCommands.DEFAULT_SPREADING, 7);
        // The command surveys the pool; wait for its coastal field so every reader below sees the final surface.
        region.awaitCoast();
        try {
            h.assertTrue(region.sea() != null && region.waves().components().size() == WaveCommands.SEA_COMPONENTS, "Sea must carry its spectrum");
            double floor = h.absolutePos(new BlockPos(0, 1, 0)).getY();
            h.assertTrue(Math.abs(region.waves().components().getFirst().depthM() - (still - floor)) < 1e-9, "Depth must be measured in the column");
            WaveRegions regions = WaveRegions.of(h.getLevel());
            WaveRegions copy = WaveRegions.load(regions.save(new net.minecraft.nbt.CompoundTag(), h.getLevel().registryAccess()), h.getLevel().registryAccess());
            h.assertTrue(copy.regions().contains(region), "Sea region must survive save and load");
            WorldWaterSampler sampler = new WorldWaterSampler(h.getLevel());
            double t = SeaSurface.time(h.getLevel());
            for (int i = 0; i < 9; i++) {
                double x = centre.getX() + 0.5 + (i % 3 - 1) * 3.3, z = centre.getZ() + 0.5 + (i / 3 - 1) * 2.7, time = t + 0.37 * i;
                WaveRegions.Local local = regions.at(x, z);
                double body = sampler.column(new Vec3d(x, still - 0.3, z), local.compiled(), local.stillWaterY(), time).surfaceY();
                double vanilla = SeaSurface.surfaceNear(h.getLevel(), x, z, still - 1, still, time);
                h.assertTrue(Math.abs(body - vanilla) < 1e-9, "Bodies and vanilla entities must see one surface: " + body + " vs " + vanilla);
            }
            // Cost of one 7 x 7 sphere sample in this 32-component sea (reported, not asserted).
            Vec3d sphere = new Vec3d(centre.getX() + 0.5, still, centre.getZ() + 0.5);
            CompiledWaves field = regions.at(sphere.x(), sphere.z()).compiled();
            long start = System.nanoTime();
            for (int i = 0; i < 2000; i++) { sampler.sample(sphere, 0.5, field, still, t + i * 0.005); }
            System.out.printf("[AeroCore] irregular-sea sphere sample: %.1f us%n", (System.nanoTime() - start) / 2000.0 / 1000);
        } finally {
            WaveRegions.of(h.getLevel()).removeAt(h.getLevel(), region.centerX(), region.centerZ());
        }
        h.succeed();
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 140, batch = "aerocore_sea_entities")
    public static void vanillaFluidHeightAndEyesFollowTheMovingSurface(GameTestHelper h) {
        double still = pool(h);
        WaveRegion region = regularWave(h, still, 0.3);
        // Feet 0.2 m below still water: wet under crests, dry under troughs. A second stand has its eyes at still level.
        ArmorStand feet = stand(h, 9, still - 0.2), eyes = stand(h, 13, still);
        eyes.setPos(eyes.getX(), still - eyes.getEyeHeight(), eyes.getZ());
        final int[] wet = {0, 0}, underEye = {0, 0};
        for (int tick = 20; tick < 120; tick++) {
            h.runAfterDelay(tick, () -> {
                double t = SeaSurface.time(h.getLevel());
                double surface = SeaSurface.surfaceY(h.getLevel(), region, feet.getX(), feet.getZ(), t);
                double expected = surface - feet.getBoundingBox().minY, actual = feet.getFluidTypeHeight(NeoForgeMod.WATER_TYPE.value());
                if (expected > 0.02) {
                    h.assertTrue(Math.abs(actual - expected) < 0.02, "Water height must follow the surface: " + actual + " vs " + expected);
                    h.assertTrue(feet.isInWater(), "Stand under a crest must be in water");
                    wet[0]++;
                } else if (expected < -0.02) {
                    h.assertTrue(!feet.isInWater() && actual == 0, "Stand above a trough must be dry: " + actual);
                    wet[1]++;
                }
                double eyeSurface = SeaSurface.surfaceY(h.getLevel(), region, eyes.getX(), eyes.getZ(), t) - eyes.getEyeY();
                boolean under = eyes.isEyeInFluidType(NeoForgeMod.WATER_TYPE.value());
                if (eyeSurface > 0.02) { h.assertTrue(under, "Eyes under a crest must be in water"); underEye[0]++; }
                else if (eyeSurface < -0.02) { h.assertTrue(!under, "Eyes above a trough must be dry"); underEye[1]++; }
            });
        }
        h.runAfterDelay(121, () -> {
            remove(h, region);
            h.assertTrue(wet[0] > 10 && wet[1] > 10, "Both crests and troughs must pass the stand: " + wet[0] + "/" + wet[1]);
            h.assertTrue(underEye[0] > 10 && underEye[1] > 10, "Both must pass the eyes: " + underEye[0] + "/" + underEye[1]);
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 260, batch = "aerocore_sea_boat")
    public static void vanillaBoatRidesTheWaves(GameTestHelper h) {
        double still = pool(h);
        WaveRegion region = regularWave(h, still, 0.25);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        Boat boat = EntityType.BOAT.create(h.getLevel());
        if (boat == null) { throw new IllegalStateException("Boat factory failed"); }
        boat.setPos(o.getX() + 11.5, still - 0.02, o.getZ() + 8.5);
        h.getLevel().addFreshEntity(boat);
        final double[] range = {Double.MAX_VALUE, -Double.MAX_VALUE}, offset = {0, 0, 0};
        for (int tick = 100; tick < 240; tick++) {
            h.runAfterDelay(tick, () -> {
                range[0] = Math.min(range[0], boat.getY()); range[1] = Math.max(range[1], boat.getY());
                double surface = SeaSurface.surfaceY(h.getLevel(), region, boat.getX(), boat.getZ(), SeaSurface.time(h.getLevel()));
                offset[0] += surface - boat.getY(); offset[1]++;
                offset[2] = Math.max(offset[2], Math.abs(RowingBoats.attitude(boat, 1)[1]));
                h.assertTrue(!boat.isUnderWater(), "Boat must stay afloat: y " + boat.getY() + ", surface " + surface);
            });
        }
        h.runAfterDelay(241, () -> {
            remove(h, region);
            double heave = (range[1] - range[0]) / 2, mean = offset[0] / offset[1];
            h.assertTrue(heave > 0.15 && heave < 0.4, "The boat must heave with 0.25 m waves: " + heave);
            // An empty 35 kg pram draws 35 / (1000 x 1.75) = 0.02 m (vanilla floated it 0.37 m deep).
            h.assertTrue(Math.abs(mean - 0.02) < 0.03, "Boat must float at its hydrostatic draft below the moving surface: " + mean);
            // Beam-on to a 12 m wave of slope ka = 0.13 rad, it rolls with the surface.
            h.assertTrue(offset[2] > Math.toRadians(3), "The boat must roll with the wave slope: " + Math.toDegrees(offset[2]));
            boat.discard();
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120, batch = "aerocore_sea_wake")
    public static void wakeRaisesTheSurfaceBehindItsSourceForEveryReader(GameTestHelper h) {
        double still = pool(h);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        int id = -7001;
        double speed = 3.0;
        // A test hull source running along +x down the pool for 2 s; its wake trails behind it.
        final Vec3d[] origin = {new Vec3d(o.getX() + 3.5, still - 0.2, o.getZ() + 7.5)};
        for (int tick = 1; tick <= 40; tick++) {
            h.runAfterDelay(tick, () -> {
                origin[0] = origin[0].add(new Vec3d(speed * 0.05, 0, 0));
                Wakes.update(h.getLevel(), id, Wakes.TEST_HULL, origin[0], new Vec3d(1, 0, 0), new Vec3d(speed, 0, 0));
            });
        }
        h.runAfterDelay(43, () -> {
            double t = SeaSurface.time(h.getLevel()), x = origin[0].x() - 4, z = origin[0].z() + 1;
            double wake = Wakes.elevation(h.getLevel(), still, x, z, -1);
            h.assertTrue(Math.abs(wake) > 1e-3, "The test hull at 3 m/s must make a visible wake: " + wake);
            h.assertTrue(Wakes.zones(h.getLevel()).stream().anyMatch(zone -> zone.contains(origin[0].x(), origin[0].z())), "A calm zone must cover the wake");
            double vanilla = SeaSurface.surfaceNear(h.getLevel(), x, z, still - 1, still, t);
            h.assertTrue(Math.abs(vanilla - (still + wake)) < 1e-9, "Vanilla entities must see the wake: " + vanilla + " vs " + (still + wake));
            WorldWaterSampler sampler = new WorldWaterSampler(h.getLevel());
            double body = sampler.column(new Vec3d(x, still - 0.3, z), WaveField.NONE, Double.NaN, t).surfaceY();
            h.assertTrue(Math.abs(body - (still + wake)) < 1e-9, "Other bodies must feel the wake: " + body);
            double own = new WorldWaterSampler(h.getLevel()).excludingWakeOf(id).column(new Vec3d(x, still - 0.3, z), WaveField.NONE, Double.NaN, t).surfaceY();
            h.assertTrue(Math.abs(own - still) < 1e-12, "A body must not feel its own wake: " + own);
            double ahead = Math.abs(Wakes.elevation(h.getLevel(), still, origin[0].x() + 6, z, -1));
            h.assertTrue(ahead < 0.2 * Math.abs(wake), "Little wave ahead of the bow: " + ahead + " vs " + wake);
        });
        // The source is gone: its waves are no longer carried along but keep running where they were made.
        h.runAfterDelay(60, () -> {
            h.assertTrue(!Wakes.zones(h.getLevel()).isEmpty(), "Running waves keep their zone");
            h.assertTrue(Wakes.maxElevation(h.getLevel()) > 1e-4, "The wake must outlive its source: " + Wakes.maxElevation(h.getLevel()));
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120, batch = "aerocore_sea_boat_wake")
    public static void pushedVanillaBoatCoastsAndLeavesAWake(GameTestHelper h) {
        double still = pool(h);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        Boat boat = EntityType.BOAT.create(h.getLevel());
        if (boat == null) { throw new IllegalStateException("Boat factory failed"); }
        boat.setPos(o.getX() + 3.5, still - 0.02, o.getZ() + 8.5);
        boat.setYRot(-90);
        h.getLevel().addFreshEntity(boat);
        // A shove of 1.6 m/s along its bow (+x), taken by the rowing physics as an impulse.
        h.runAfterDelay(2, () -> boat.setDeltaMovement(1.6 * 0.05, 0, 0));
        final double[] start = new double[1];
        h.runAfterDelay(5, () -> start[0] = boat.getX());
        h.runAfterDelay(45, () -> {
            double travelled = boat.getX() - start[0], speed = (boat.getX() - boat.xo) / 0.05;
            h.assertTrue(travelled > 1.5 && speed > 0.1 && speed < 1.6, "A shoved boat must coast and slow: " + travelled + " m, " + speed + " m/s");
            h.assertTrue(Wakes.maxElevation(h.getLevel()) > 1e-4, "A moving boat must leave a wake: " + Wakes.maxElevation(h.getLevel()));
            h.assertTrue(!Wakes.zones(h.getLevel()).isEmpty(), "Its calm zone must exist");
            boat.discard();
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120, batch = "aerocore_sea_boat_load")
    public static void vanillaBoatSinksToTheDraftOfItsLoad(GameTestHelper h) {
        double still = pool(h);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        Boat boat = EntityType.BOAT.create(h.getLevel());
        Villager villager = EntityType.VILLAGER.create(h.getLevel());
        if (boat == null || villager == null) { throw new IllegalStateException("Entity factory failed"); }
        boat.setPos(o.getX() + 11.5, still - 0.02, o.getZ() + 8.5);
        h.getLevel().addFreshEntity(boat);
        villager.setPos(boat.getX(), boat.getY() + 0.2, boat.getZ());
        h.getLevel().addFreshEntity(villager);
        villager.startRiding(boat, true);
        h.runAfterDelay(100, () -> {
            // 35 kg hull + 62 kg person on a 1.75 x 1.0 m bottom.
            double draft = still - boat.getY(), expected = (35 + 62) / (998.2 * 1.75);
            h.assertTrue(Math.abs(draft - expected) < 0.01, "Loaded draft must follow Archimedes: " + draft + " vs " + expected);
            boat.discard(); villager.discard();
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 40, batch = "aerocore_sea_coast")
    public static void islandShadesItsLeeForEveryReader(GameTestHelper h) throws Exception {
        double still = pool(h);
        // A 3 x 4 m stone island standing out of the water upwind of the pool's centre, built before the survey.
        for (int x = 7; x <= 9; x++) for (int z = 6; z <= 9; z++) for (int y = 1; y <= 5; y++) { h.setBlock(new BlockPos(x, y, z), Blocks.STONE); }
        BlockPos centre = h.absolutePos(new BlockPos(11, 4, 8));
        WaveRegion region = WaveCommands.create(h.getLevel(), centre.getX() + 0.5, still, centre.getZ() + 0.5, 0.12, 8, 0, 12);
        try {
            h.assertTrue(region.survey() != null && region.survey().landNodes() > 0, "The command must survey the pool's walls and island");
            h.assertTrue(region.awaitCoast() != null, "The coastal field must be built");
            BlockPos o = h.absolutePos(BlockPos.ZERO);
            double period = region.waves().components().getFirst().periodS(), t0 = SeaSurface.time(h.getLevel());
            double front = amplitude(h, region, o.getX() + 4.5, o.getZ() + 8, t0, period);
            double lee = amplitude(h, region, o.getX() + 13.5, o.getZ() + 8, t0, period);
            System.out.printf("[AeroCore] island in the pool: wave amplitude %.3f m upwind, %.3f m in its lee%n", front, lee);
            h.assertTrue(front > 0.06, "Waves must reach the island's weather side: " + front);
            h.assertTrue(lee < 0.75 * front, "The island must shade its lee: " + lee + " vs " + front);
            // One surface for every reader, behind the island too.
            WorldWaterSampler sampler = new WorldWaterSampler(h.getLevel());
            for (double dx : new double[]{11.5, 13.5, 18.5}) {
                double x = o.getX() + dx, z = o.getZ() + 7.3, time = t0 + dx;
                WaveRegions.Local local = WaveRegions.of(h.getLevel()).at(x, z);
                double body = sampler.column(new Vec3d(x, still - 0.3, z), local.compiled(), local.stillWaterY(), time).surfaceY();
                double vanilla = SeaSurface.surfaceNear(h.getLevel(), x, z, still - 1, still, time);
                h.assertTrue(Math.abs(body - vanilla) < 1e-9, "Bodies and vanilla entities must see one coastal surface: " + body + " vs " + vanilla);
            }
        } finally {
            remove(h, region);
        }
        h.succeed();
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 220, batch = "aerocore_sea_wind")
    public static void windDriftsAVanillaBoatDownwind(GameTestHelper h) {
        double still = pool(h);
        BlockPos centre = h.absolutePos(new BlockPos(11, 4, 8)), o = h.absolutePos(BlockPos.ZERO);
        // A calm region with a 10 m/s wind blowing toward +x.
        WaveRegion region = new WaveRegion(centre.getX() + 0.5, centre.getZ() + 0.5, 12, still, WaveField.NONE, null, null, new WaveRegion.Wind(10, 0));
        WaveRegions.of(h.getLevel()).add(h.getLevel(), region);
        // A villager aboard, as the loaded pram of RowboatTest.driftsDownwindLikeASmallOpenBoat.
        Boat boat = EntityType.BOAT.create(h.getLevel());
        Villager villager = EntityType.VILLAGER.create(h.getLevel());
        if (boat == null || villager == null) { throw new IllegalStateException("Entity factory failed"); }
        boat.setPos(o.getX() + 4.5, still - 0.02, o.getZ() + 8.5);
        h.getLevel().addFreshEntity(boat);
        villager.setPos(boat.getX(), boat.getY() + 0.2, boat.getZ());
        h.getLevel().addFreshEntity(villager);
        villager.startRiding(boat, true);
        final double[] start = new double[2];
        h.runAfterDelay(20, () -> { start[0] = boat.getX(); start[1] = boat.getZ(); });
        h.runAfterDelay(200, () -> {
            double dx = boat.getX() - start[0], dz = boat.getZ() - start[1];
            System.out.printf("[AeroCore] loaded boat in a 10 m/s wind: %.2f m downwind, %.2f m across in 9 s%n", dx, dz);
            remove(h, region);
            // Leeway of an occupied pram about 6 % of U10 plus the wind's surface drift (about 2 %): 0.6-0.8 m/s once steady.
            h.assertTrue(dx > 3 && dx < 9, "The boat must drift downwind at a few percent of the wind: " + dx);
            h.assertTrue(Math.abs(dz) < 0.3 * dx, "Mostly downwind: " + dz);
            boat.discard(); villager.discard();
            h.succeed();
        });
    }

    /** Half the range of the surface at (x, z) over one period. */
    private static double amplitude(GameTestHelper h, WaveRegion region, double x, double z, double t0, double period) {
        double low = Double.MAX_VALUE, high = -Double.MAX_VALUE;
        for (int i = 0; i < 48; i++) {
            double y = SeaSurface.surfaceY(h.getLevel(), region, x, z, t0 + period * i / 48);
            low = Math.min(low, y); high = Math.max(high, y);
        }
        return (high - low) / 2;
    }

    /** A 12 m regular wave across the pool in a 12 m region (3 m taper) centred on the pool. */
    private static WaveRegion regularWave(GameTestHelper h, double still, double amplitude) {
        BlockPos centre = h.absolutePos(new BlockPos(11, 4, 8));
        WaveRegion region = new WaveRegion(centre.getX() + 0.5, centre.getZ() + 0.5, 12, still,
                new WaveField(List.of(new LinearWave(amplitude, 12, 0, 4, 0))));
        WaveRegions.of(h.getLevel()).add(h.getLevel(), region);
        return region;
    }

    private static void remove(GameTestHelper h, WaveRegion region) {
        WaveRegions.of(h.getLevel()).removeAt(h.getLevel(), region.centerX(), region.centerZ());
    }

    private static ArmorStand stand(GameTestHelper h, double x, double feetY) {
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        ArmorStand stand = EntityType.ARMOR_STAND.create(h.getLevel());
        if (stand == null) { throw new IllegalStateException("Armor stand factory failed"); }
        stand.setNoGravity(true);
        stand.setPos(o.getX() + x + 0.5, feetY, o.getZ() + 8.5);
        h.getLevel().addFreshEntity(stand);
        return stand;
    }

    /** Stone floor and walls with 4 m of water inside; returns the absolute still-water level. */
    private static double pool(GameTestHelper h) {
        // The GameTest server ticks faster than real time; step wakes on its thread so they keep pace.
        Wakes.synchronous = true;
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) {
            h.setBlock(new BlockPos(x, 0, z), Blocks.STONE);
            for (int y = 1; y <= 5; y++) {
                boolean wall = x == 0 || z == 0 || x == 23 || z == 15;
                if (wall) { h.setBlock(new BlockPos(x, y, z), Blocks.STONE); }
                else if (y <= 4) { h.setBlock(new BlockPos(x, y, z), Blocks.WATER); }
            }
        }
        return new WorldWaterSampler(h.getLevel()).stillSurface(h.absolutePos(new BlockPos(11, 4, 8)));
    }
}
