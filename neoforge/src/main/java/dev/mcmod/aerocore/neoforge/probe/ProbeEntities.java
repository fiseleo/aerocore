package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.core.registries.Registries;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.MobCategory;
import net.neoforged.neoforge.registries.DeferredHolder;
import net.neoforged.neoforge.registries.DeferredRegister;

public final class ProbeEntities {
    public static final DeferredRegister<EntityType<?>> TYPES = DeferredRegister.create(Registries.ENTITY_TYPE, AeroCore.MOD_ID);
    public static final DeferredHolder<EntityType<?>, EntityType<PhysicsProbe>> PHYSICS_PROBE = TYPES.register("physics_probe",
            () -> EntityType.Builder.<PhysicsProbe>of(PhysicsProbe::new, MobCategory.MISC)
                    .sized(1, 1).clientTrackingRange(8).updateInterval(1).noSave().fireImmune()
                    .build(AeroCore.MOD_ID + ":physics_probe"));
    public static final DeferredHolder<EntityType<?>, EntityType<FloatingPlatform>> FLOATING_PLATFORM = TYPES.register("floating_platform",
            () -> EntityType.Builder.<FloatingPlatform>of(FloatingPlatform::new, MobCategory.MISC)
                    .sized(2.6f, 1.2f).clientTrackingRange(8).updateInterval(1).noSave().fireImmune()
                    .build(AeroCore.MOD_ID + ":floating_platform"));
    public static final DeferredHolder<EntityType<?>, EntityType<FloatingHull>> FLOATING_HULL = TYPES.register("floating_hull",
            () -> EntityType.Builder.<FloatingHull>of(FloatingHull::new, MobCategory.MISC)
                    .sized(6.8f, 1.2f).clientTrackingRange(10).updateInterval(1).noSave().fireImmune()
                    .build(AeroCore.MOD_ID + ":floating_hull"));
    public static final DeferredHolder<EntityType<?>, EntityType<PlaningBoat>> PLANING_BOAT = TYPES.register("planing_boat",
            () -> EntityType.Builder.<PlaningBoat>of(PlaningBoat::new, MobCategory.MISC)
                    .sized(6.8f, 1.2f).clientTrackingRange(10).updateInterval(1).noSave().fireImmune()
                    .build(AeroCore.MOD_ID + ":planing_boat"));
    private ProbeEntities() {}
}
