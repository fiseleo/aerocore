package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.core.registries.Registries;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.syncher.EntityDataSerializer;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.MobCategory;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.neoforge.registries.DeferredHolder;
import net.neoforged.neoforge.registries.DeferredRegister;
import net.neoforged.neoforge.registries.NeoForgeRegistries;

import java.nio.charset.StandardCharsets;

/**
 * AeroCore's part-built vehicle entity type and the entity-data serializer it syncs blueprints with. A vehicle mod
 * registers its own entity types of PartVehicle (or a subclass) the same way, to give them its own renderers.
 */
public final class VehicleRegistry {
    /** Blueprints with drawn hulls can pass the 32767-character limit of the vanilla string serializer. */
    public static final int MAX_TEXT_BYTES = 1 << 22;

    public static final DeferredRegister<EntityDataSerializer<?>> SERIALIZERS = DeferredRegister.create(NeoForgeRegistries.Keys.ENTITY_DATA_SERIALIZERS, AeroCore.MOD_ID);
    /** A UTF-8 string of up to MAX_TEXT_BYTES. */
    public static final DeferredHolder<EntityDataSerializer<?>, EntityDataSerializer<String>> LONG_TEXT = SERIALIZERS.register("long_text",
            () -> EntityDataSerializer.forValueType(StreamCodec.<RegistryFriendlyByteBuf, String>of(
                    (buffer, text) -> buffer.writeByteArray(text.getBytes(StandardCharsets.UTF_8)),
                    buffer -> new String(buffer.readByteArray(MAX_TEXT_BYTES), StandardCharsets.UTF_8))));

    public static final DeferredRegister<EntityType<?>> TYPES = DeferredRegister.create(Registries.ENTITY_TYPE, AeroCore.MOD_ID);
    /** A vehicle built from any blueprint, for commands, tests and as the example of registering one. */
    public static final DeferredHolder<EntityType<?>, EntityType<PartVehicle>> PART_VEHICLE = TYPES.register("part_vehicle",
            () -> EntityType.Builder.<PartVehicle>of(PartVehicle::new, MobCategory.MISC)
                    .sized(1, 1).clientTrackingRange(10).updateInterval(1).fireImmune()
                    .build(AeroCore.MOD_ID + ":part_vehicle"));

    private VehicleRegistry() {}

    public static void register(IEventBus modBus) {
        SERIALIZERS.register(modBus);
        TYPES.register(modBus);
        modBus.addListener(VehicleNetwork::register);
    }
}
