package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.mixin.LivingEntityAccessor;
import it.unimi.dsi.fastutil.objects.Object2DoubleMap;
import net.minecraft.world.effect.MobEffects;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.ai.attributes.Attributes;
import net.minecraft.world.entity.animal.Turtle;
import net.minecraft.world.entity.animal.WaterAnimal;
import net.minecraft.world.entity.animal.axolotl.Axolotl;
import net.minecraft.world.entity.item.ItemEntity;
import net.minecraft.world.entity.monster.Guardian;
import net.minecraft.world.entity.vehicle.Boat;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import net.neoforged.neoforge.common.NeoForgeMod;
import net.neoforged.neoforge.fluids.FluidType;

/**
 * Vanilla entities in sea regions: after vanilla scans the water blocks, the sea's contribution to the entity's water
 * height is replaced by the moving surface (so items, mobs, players and boats bob, swim and drown against the same
 * surface that is drawn and that AeroCore bodies feel), and the wave orbital velocity drifts the entity horizontally.
 * The drift uses each vanilla movement model's own per-tick water damping f: adding u (1 - f) / f per tick makes the
 * damping act on the velocity relative to the moving water, whose steady state is the water velocity. No new
 * coefficient is introduced; vanilla's movement models themselves are game models, not hydrodynamics.
 */
public final class VanillaWater {
    private VanillaWater() {}

    public static void afterFluidScan(Entity entity, Object2DoubleMap<FluidType> heights) {
        if (entity instanceof CoupledBody) { return; }
        Level level = entity.level();
        AABB box = entity.getBoundingBox().deflate(0.001);
        double t = SeaSurface.time(level), x = entity.getX(), z = entity.getZ();
        WaveRegion region = SeaSurface.governing(level, x, z, box.minY, box.maxY);
        if (region == null) { return; }
        double surface = SeaSurface.surfaceY(level, region, x, z, t, entity instanceof Boat ? entity.getId() : -1);
        FluidType water = NeoForgeMod.WATER_TYPE.value();
        // Water standing above the sea level inside the box (a waterfall, a raised pool) keeps vanilla's height.
        if (heights.containsKey(water) && box.minY + heights.getDouble(water) > region.stillWaterY() + WaveRegion.LEVEL_TOLERANCE) { return; }
        double height = surface - box.minY;
        if (height <= 0) { heights.removeDouble(water); return; }
        heights.put(water, height);
        if (!entity.isPushedByFluid(water)) { return; }
        double f = waterDamping(entity, height);
        if (!(f > 0 && f < 1)) { return; }
        double wetMiddle = box.minY + Math.min(height, box.getYsize()) / 2;
        Vec3d u = SeaSurface.orbitalVelocity(level, region, x, wetMiddle, z, t);
        double gain = (1 - f) / f * MinecraftUnits.TICK_SECONDS;
        entity.setDeltaMovement(entity.getDeltaMovement().add(new Vec3(u.x() * gain, 0, u.z() * gain)));
    }

    /**
     * Sea surface under a vanilla boat, or NaN where vanilla rules apply. vanillaLevel is the water level vanilla
     * found at the boat's bottom (NaN if none): water standing above the sea level there keeps vanilla's result.
     */
    public static double boatSurface(Boat boat, double vanillaLevel) {
        AABB box = boat.getBoundingBox();
        Level level = boat.level();
        WaveRegion region = SeaSurface.governing(level, boat.getX(), boat.getZ(), box.minY, box.maxY);
        if (region == null || vanillaLevel > region.stillWaterY() + WaveRegion.LEVEL_TOLERANCE) { return Double.NaN; }
        return SeaSurface.surfaceY(level, region, boat.getX(), boat.getZ(), SeaSurface.time(level), boat.getId());
    }

    /** Eye fluid against the moving surface; returns the vanilla value where the sea does not govern the point. */
    public static FluidType eyeFluid(Entity entity, FluidType vanilla) {
        if (entity instanceof CoupledBody) { return vanilla; }
        double eye = entity.getEyeY();
        if (entity.getVehicle() instanceof Boat boat && !boat.isUnderWater() && boat.getBoundingBox().maxY >= eye && boat.getBoundingBox().minY <= eye) {
            return vanilla;
        }
        Boolean under = SeaSurface.submerged(entity.level(), entity.getX(), eye, entity.getZ(), SeaSurface.time(entity.level()));
        if (under == null) { return vanilla; }
        FluidType water = NeoForgeMod.WATER_TYPE.value();
        if (under) { return water; }
        return vanilla == water ? NeoForgeMod.EMPTY_TYPE.value() : vanilla;
    }

    /**
     * Horizontal velocity factor per tick that the entity's vanilla movement applies in water, or NaN when it has none
     * that this coupling knows. LivingEntity.travel: sprinting 0.9 else getWaterSlowDown(), blended toward 0.546 by
     * water movement efficiency (halved off ground), 0.96 with Dolphin's Grace. Fish, guardians, turtles and axolotls
     * swim with their own travel() scaling by 0.9. ItemEntity (water height above 0.1): 0.99 x 0.98. Boats get
     * none: RowingBoats moves them against the water's velocity itself.
     */
    static double waterDamping(Entity entity, double height) {
        if (entity instanceof Boat) { return Double.NaN; }
        if (entity instanceof ItemEntity) { return height > 0.1 ? 0.99 * 0.98 : Double.NaN; }
        if (entity instanceof WaterAnimal || entity instanceof Guardian || entity instanceof Turtle || entity instanceof Axolotl) { return 0.9; }
        if (entity instanceof LivingEntity living) {
            double f = living.isSprinting() ? 0.9 : ((LivingEntityAccessor) living).aerocore$waterSlowDown();
            double efficiency = living.getAttributeValue(Attributes.WATER_MOVEMENT_EFFICIENCY);
            if (!living.onGround()) { efficiency *= 0.5; }
            if (efficiency > 0) { f += (0.54600006 - f) * efficiency; }
            if (living.hasEffect(MobEffects.DOLPHINS_GRACE)) { f = 0.96; }
            return f;
        }
        return Double.NaN;
    }
}
