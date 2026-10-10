package dev.mcmod.aerocore.neoforge.mixin;

import net.minecraft.world.entity.LivingEntity;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Invoker;

/** Exposes LivingEntity's per-tick water damping, which VanillaWater uses to drift entities with the waves. */
@Mixin(LivingEntity.class)
public interface LivingEntityAccessor {
    @Invoker("getWaterSlowDown") float aerocore$waterSlowDown();
}
