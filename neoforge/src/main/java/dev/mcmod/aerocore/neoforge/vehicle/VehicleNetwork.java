package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.entity.Entity;
import net.neoforged.neoforge.network.PacketDistributor;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;

import java.util.LinkedHashMap;
import java.util.Map;

/** Control input from a client to the vehicle it drives (the server decides whether it may: PartVehicle.canControl). */
public final class VehicleNetwork {
    /** Channels per message and characters per channel name: a client cannot make the server keep more. */
    public static final int MAX_CHANNELS = 64, MAX_NAME = 64;
    /** Control values are bounded; parts clamp them further to their own ranges. */
    public static final double MAX_VALUE = 1000;

    private VehicleNetwork() {}

    public record ControlsPayload(int entityId, Map<String, Double> controls) implements CustomPacketPayload {
        public static final Type<ControlsPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "vehicle_controls"));
        public static final StreamCodec<FriendlyByteBuf, ControlsPayload> STREAM_CODEC = StreamCodec.of(
                (buffer, p) -> {
                    buffer.writeVarInt(p.entityId());
                    buffer.writeVarInt(p.controls().size());
                    p.controls().forEach((k, v) -> { buffer.writeUtf(k, MAX_NAME); buffer.writeDouble(v); });
                },
                buffer -> {
                    int id = buffer.readVarInt(), n = buffer.readVarInt();
                    if (n < 0 || n > MAX_CHANNELS) { throw new IllegalArgumentException("Too many control channels"); }
                    Map<String, Double> map = new LinkedHashMap<>();
                    for (int i = 0; i < n; i++) {
                        String key = buffer.readUtf(MAX_NAME);
                        double value = buffer.readDouble();
                        if (!Double.isFinite(value) || Math.abs(value) > MAX_VALUE) { throw new IllegalArgumentException("Control value out of range"); }
                        map.put(key, value);
                    }
                    return new ControlsPayload(id, map);
                });
        @Override public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }

    static void register(RegisterPayloadHandlersEvent event) {
        event.registrar("1").playToServer(ControlsPayload.TYPE, ControlsPayload.STREAM_CODEC, (payload, context) -> {
            if (!(context.player() instanceof ServerPlayer player)) { return; }
            Entity entity = player.level().getEntity(payload.entityId());
            if (entity instanceof PartVehicle vehicle && vehicle.canControl(player)) { vehicle.setControls(payload.controls()); }
        });
    }

    /** Client side: asks the server to set these channels on the vehicle (all other channels keep their values). */
    public static void send(PartVehicle vehicle, Map<String, Double> controls) {
        PacketDistributor.sendToServer(new ControlsPayload(vehicle.getId(), Map.copyOf(controls)));
    }
}
