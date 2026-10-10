package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.decoration.ArmorStand;
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
                double body = sampler.column(new Vec3d(x, still - 0.3, z), local.waves(), local.stillWaterY(), time).surfaceY();
                double vanilla = SeaSurface.surfaceNear(h.getLevel(), x, z, still - 1, still, time);
                h.assertTrue(Math.abs(body - vanilla) < 1e-9, "Bodies and vanilla entities must see one surface: " + body + " vs " + vanilla);
            }
            // Cost of one 7 x 7 sphere sample in this 32-component sea (reported, not asserted).
            Vec3d sphere = new Vec3d(centre.getX() + 0.5, still, centre.getZ() + 0.5);
            WaveField field = regions.at(sphere.x(), sphere.z()).waves();
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
    public static void vanillaBoatHeavesWithTheWaves(GameTestHelper h) {
        double still = pool(h);
        WaveRegion region = regularWave(h, still, 0.25);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        Boat boat = EntityType.BOAT.create(h.getLevel());
        if (boat == null) { throw new IllegalStateException("Boat factory failed"); }
        boat.setPos(o.getX() + 11.5, still - 0.37, o.getZ() + 8.5);
        h.getLevel().addFreshEntity(boat);
        final double[] range = {Double.MAX_VALUE, -Double.MAX_VALUE}, offset = {0, 0};
        for (int tick = 100; tick < 240; tick++) {
            h.runAfterDelay(tick, () -> {
                range[0] = Math.min(range[0], boat.getY()); range[1] = Math.max(range[1], boat.getY());
                double surface = SeaSurface.surfaceY(h.getLevel(), region, boat.getX(), boat.getZ(), SeaSurface.time(h.getLevel()));
                offset[0] += surface - boat.getY(); offset[1]++;
                h.assertTrue(!boat.isUnderWater(), "Boat must stay afloat: y " + boat.getY() + ", surface " + surface);
            });
        }
        h.runAfterDelay(241, () -> {
            remove(h, region);
            double heave = (range[1] - range[0]) / 2, mean = offset[0] / offset[1];
            h.assertTrue(heave > 0.15 && heave < 0.4, "Vanilla boat must heave with 0.25 m waves: " + heave);
            // Vanilla buoyancy balances gravity with (waterLevel - y) / 0.5625 = 0.65, about 0.37 m.
            h.assertTrue(Math.abs(mean - 0.37) < 0.1, "Boat must float at vanilla's depth below the moving surface: " + mean);
            boat.discard();
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 80, batch = "aerocore_sea_wake")
    public static void wakeRaisesTheSurfaceBehindItsSourceForEveryReader(GameTestHelper h) {
        double still = pool(h);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        int bin = Wakes.bin(3.0), id = -7001;
        double speed = Wakes.binSpeed(bin);
        Wakes.prefetch(Wakes.TEST_HULL, bin);
        dev.mcmod.aerocore.api.KelvinWake.Pattern pattern = Wakes.computeNow(Wakes.TEST_HULL, bin);
        // A source moving along +x near the east wall; its wake trails toward -x.
        Vec3d origin = new Vec3d(o.getX() + 18.5, still - 0.2, o.getZ() + 7.5);
        for (int tick = 1; tick <= 30; tick++) { h.runAfterDelay(tick, () -> Wakes.update(h.getLevel(), id, Wakes.TEST_HULL, origin, new Vec3d(speed, 0, 0))); }
        h.runAfterDelay(30, () -> {
            double t = SeaSurface.time(h.getLevel()), x = origin.x() - 8, z = origin.z() + 1;
            double expected = pattern.elevation(-8, 1);
            h.assertTrue(Math.abs(expected) > 1e-3, "The test hull at 3 m/s must make a visible wake: " + expected);
            h.assertTrue(Wakes.zones(h.getLevel()).stream().anyMatch(zone -> zone.contains(origin.x(), origin.z())), "A calm zone must follow the wake");
            double vanilla = SeaSurface.surfaceNear(h.getLevel(), x, z, still - 1, still, t);
            h.assertTrue(Math.abs(vanilla - (still + expected)) < 1e-9, "Vanilla entities must see the wake: " + vanilla + " vs " + (still + expected));
            WorldWaterSampler sampler = new WorldWaterSampler(h.getLevel());
            double body = sampler.column(new Vec3d(x, still - 0.3, z), WaveField.NONE, Double.NaN, t).surfaceY();
            h.assertTrue(Math.abs(body - (still + expected)) < 1e-9, "Other bodies must feel the wake: " + body);
            double own = new WorldWaterSampler(h.getLevel()).excludingWakeOf(id).column(new Vec3d(x, still - 0.3, z), WaveField.NONE, Double.NaN, t).surfaceY();
            h.assertTrue(Math.abs(own - still) < 1e-12, "A body must not feel its own wake: " + own);
            double ahead = SeaSurface.surfaceNear(h.getLevel(), origin.x() + 4, z, still - 1, still, t);
            h.assertTrue(Math.abs(ahead - still) < 1e-12, "No wake ahead of the bow: " + ahead);
        });
        // Unreported sources fade out and their zone goes away.
        h.runAfterDelay(60, () -> {
            h.assertTrue(Wakes.zones(h.getLevel()).isEmpty(), "Faded wakes must leave no zone");
            h.assertTrue(Wakes.elevation(h.getLevel(), still, origin.x() - 8, origin.z() + 1, -1) == 0, "Faded wakes must leave a still surface");
            h.succeed();
        });
    }

    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120, batch = "aerocore_sea_boat_wake")
    public static void movingVanillaBoatLeavesAWake(GameTestHelper h) {
        double still = pool(h);
        Wakes.Design design = Wakes.vanillaBoat(EntityType.BOAT.getWidth(), EntityType.BOAT.getHeight());
        for (int bin = Wakes.bin(3.5); bin <= Wakes.bin(5.5); bin++) { Wakes.prefetch(design, bin); }
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        Boat boat = EntityType.BOAT.create(h.getLevel());
        if (boat == null) { throw new IllegalStateException("Boat factory failed"); }
        boat.setPos(o.getX() + 3.5, still - 0.37, o.getZ() + 8.5);
        h.getLevel().addFreshEntity(boat);
        for (int tick = 1; tick <= 60; tick++) {
            h.runAfterDelay(tick, () -> boat.setDeltaMovement(0.25, boat.getDeltaMovement().y, 0));
        }
        h.runAfterDelay(60, () -> {
            double behind = Wakes.elevation(h.getLevel(), still, boat.getX() - 3, boat.getZ() + 0.5, -1);
            h.assertTrue(Math.abs(behind) > 1e-4, "A boat at about 4.5 m/s must leave a wake: " + behind);
            h.assertTrue(!Wakes.zones(h.getLevel()).isEmpty(), "Its calm zone must exist");
            boat.discard();
            h.succeed();
        });
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
