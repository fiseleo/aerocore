package dev.mcmod.aerocore.neoforge;

import dev.mcmod.aerocore.api.Vec3d;
import net.minecraft.world.phys.Vec3;

/** Adapter assuming one block = one metre and one simulation tick = 0.05 s. */
public final class MinecraftUnits {
    public static final double TICK_SECONDS = 0.05;
    private MinecraftUnits() {}

    public static Vec3d fromMinecraft(Vec3 vector) {
        return new Vec3d(vector.x, vector.y, vector.z);
    }

    public static Vec3 toMinecraft(Vec3d vector) {
        return new Vec3(vector.x(), vector.y(), vector.z());
    }

    public static Vec3d toMetersPerSecond(Vec3 blocksPerTick) {
        return fromMinecraft(blocksPerTick).scale(1 / TICK_SECONDS);
    }

    public static Vec3 toBlocksPerTick(Vec3d metersPerSecond) {
        return toMinecraft(metersPerSecond.scale(TICK_SECONDS));
    }
}

