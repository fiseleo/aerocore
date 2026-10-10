package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.network.codec.ByteBufCodecs;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.entity.player.PlayerEvent;
import net.neoforged.neoforge.network.PacketDistributor;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;

import java.util.List;
import java.util.function.Consumer;

/** Mirrors each dimension's wave regions to its players, who need them to draw the moving surface. */
public final class WaveNetwork {
    /** Full replacement list for one dimension. */
    public record RegionsPayload(ResourceLocation dimension, List<WaveRegion> regions) implements CustomPacketPayload {
        public static final Type<RegionsPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "wave_regions"));
        public static final StreamCodec<FriendlyByteBuf, RegionsPayload> STREAM_CODEC = StreamCodec.composite(
                ResourceLocation.STREAM_CODEC, RegionsPayload::dimension,
                WaveRegion.STREAM_CODEC.apply(ByteBufCodecs.list(WaveRegions.MAX_REGIONS)), RegionsPayload::regions,
                RegionsPayload::new);
        public RegionsPayload { regions = List.copyOf(regions); }
        @Override public Type<RegionsPayload> type() { return TYPE; }
    }

    /** Present heights of the regions' seas (WaveRegion.scale), in the order of the dimension's region list. */
    public record ScalesPayload(ResourceLocation dimension, List<Double> scales) implements CustomPacketPayload {
        public static final Type<ScalesPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "wave_scales"));
        public static final StreamCodec<FriendlyByteBuf, ScalesPayload> STREAM_CODEC = StreamCodec.composite(
                ResourceLocation.STREAM_CODEC, ScalesPayload::dimension,
                ByteBufCodecs.DOUBLE.apply(ByteBufCodecs.list(WaveRegions.MAX_REGIONS)), ScalesPayload::scales,
                ScalesPayload::new);
        public ScalesPayload { scales = List.copyOf(scales); }
        @Override public Type<ScalesPayload> type() { return TYPE; }
    }

    private static volatile Consumer<ScalesPayload> clientScales = payload -> {};
    public static void setClientScales(Consumer<ScalesPayload> handler) { clientScales = handler; }

    /** Sends the regions' present sea heights to the dimension's players. */
    public static void syncScales(ServerLevel level) {
        List<Double> scales = WaveRegions.of(level).regions().stream().map(WaveRegion::scale).toList();
        PacketDistributor.sendToPlayersInDimension(level, new ScalesPayload(level.dimension().location(), scales));
    }

    /** Set by the client bootstrap; the dedicated server never receives this payload. */
    private static volatile Consumer<RegionsPayload> clientHandler = payload -> {};

    private WaveNetwork() {}

    public static void setClientHandler(Consumer<RegionsPayload> handler) { clientHandler = handler; }

    public static void register(RegisterPayloadHandlersEvent event) {
        event.registrar("1").playToClient(RegionsPayload.TYPE, RegionsPayload.STREAM_CODEC,
                (payload, context) -> clientHandler.accept(payload))
                .playToClient(ScalesPayload.TYPE, ScalesPayload.STREAM_CODEC, (payload, context) -> clientScales.accept(payload))
                .playToClient(WorldWind.WindPayload.TYPE, WorldWind.WindPayload.STREAM_CODEC, (payload, context) -> WorldWind.accept(payload));
    }

    public static void sync(ServerLevel level) {
        PacketDistributor.sendToPlayersInDimension(level, payload(level));
    }

    private static void send(ServerPlayer player) {
        if (player.level() instanceof ServerLevel level) { PacketDistributor.sendToPlayer(player, payload(level)); }
    }

    private static RegionsPayload payload(ServerLevel level) {
        return new RegionsPayload(level.dimension().location(), WaveRegions.of(level).regions());
    }

    @EventBusSubscriber(modid = AeroCore.MOD_ID)
    public static final class Events {
        private Events() {}
        @SubscribeEvent public static void login(PlayerEvent.PlayerLoggedInEvent event) {
            if (event.getEntity() instanceof ServerPlayer player) { send(player); }
        }
        @SubscribeEvent public static void changeDimension(PlayerEvent.PlayerChangedDimensionEvent event) {
            if (event.getEntity() instanceof ServerPlayer player) { send(player); }
        }
        @SubscribeEvent public static void respawn(PlayerEvent.PlayerRespawnEvent event) {
            if (event.getEntity() instanceof ServerPlayer player) { send(player); }
        }
    }
}
