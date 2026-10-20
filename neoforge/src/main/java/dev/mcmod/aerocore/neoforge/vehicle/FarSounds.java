package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.level.entity.EntityTypeTest;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.tick.ServerTickEvent;
import net.neoforged.neoforge.network.PacketDistributor;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.function.Function;

/**
 * Loud vehicles beyond the entity tracking range (0.37): a jet two kilometres off is heard long before Minecraft
 * sends it to the client. A vehicle mod gives each vehicle a {@link Signature} (its sound power, a kind of sound it
 * defines, a rate such as its engine's speed; {@link #provide}); every {@link #EVERY_TICKS} ticks the server tells each
 * player of the vehicles it does not track whose level would be at least {@link #HEARD_DB} dB at the player
 * (spherical spreading: r = 10^((Lw - 11 - 30) / 20), at most {@link #MAX_M}), the {@link #MAX_PER_PLAYER} loudest,
 * with where they are and how they move. Clients keep what they were told for {@link #KEEP_TICKS} ticks.
 * Only vehicles the server has loaded are heard (far from every player they are not ticking at all).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class FarSounds {
    private FarSounds() {}

    private static final Logger LOGGER = com.mojang.logging.LogUtils.getLogger();
    public static final int EVERY_TICKS = 5, MAX_PER_PLAYER = 12, KEEP_TICKS = 15;
    public static final double MAX_M = 1500, HEARD_DB = 30, LOUD_DB = 90;

    /** What a vehicle sounds like from afar: its sound power (dB), a kind (the mod's own numbering) and a rate (Hz). */
    public record Signature(double levelDb, int kind, double rateHz) {}

    /** A vehicle heard from afar: a key (its UUID's hash), where it is (m), how it moves (m/s) and its signature. */
    public record Far(int key, Vec3d position, Vec3d velocity, Signature signature) {}

    public record FarPayload(List<Far> far) implements CustomPacketPayload {
        public static final Type<FarPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "far_sounds"));
        public static final StreamCodec<RegistryFriendlyByteBuf, FarPayload> CODEC = StreamCodec.of((buf, p) -> {
            buf.writeVarInt(p.far().size());
            for (Far f : p.far()) {
                buf.writeInt(f.key());
                buf.writeDouble(f.position().x()); buf.writeDouble(f.position().y()); buf.writeDouble(f.position().z());
                buf.writeFloat((float) f.velocity().x()); buf.writeFloat((float) f.velocity().y()); buf.writeFloat((float) f.velocity().z());
                buf.writeFloat((float) f.signature().levelDb());
                buf.writeVarInt(f.signature().kind());
                buf.writeFloat((float) f.signature().rateHz());
            }
        }, buf -> {
            int n = Math.min(buf.readVarInt(), MAX_PER_PLAYER);
            List<Far> list = new ArrayList<>(n);
            for (int i = 0; i < n; i++) {
                int key = buf.readInt();
                Vec3d at = new Vec3d(finite(buf.readDouble()), finite(buf.readDouble()), finite(buf.readDouble()));
                Vec3d v = new Vec3d(finite(buf.readFloat()), finite(buf.readFloat()), finite(buf.readFloat()));
                list.add(new Far(key, at, v, new Signature(finite(buf.readFloat()), buf.readVarInt(), finite(buf.readFloat()))));
            }
            return new FarPayload(list);
        });
        @Override public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }

    private static double finite(double v) { return Double.isFinite(v) ? Math.clamp(v, -3e7, 3e7) : 0; }

    private static volatile Function<PartVehicle, Signature> provider;

    /** The vehicle mod's signatures (server; null: the vehicle is not heard from afar). */
    public static void provide(Function<PartVehicle, Signature> signatures) { provider = signatures; }

    /** How far a sound power is heard, m. */
    public static double rangeM(double levelDb) { return Math.min(MAX_M, Math.pow(10, (levelDb - 11 - HEARD_DB) / 20)); }

    // ---- the server

    private static final Set<UUID> TOLD = new HashSet<>();
    private static long ticks;
    /** How many lists were sent (GameTests). */
    public static volatile int sends;

    @SubscribeEvent static void tick(ServerTickEvent.Post event) {
        Function<PartVehicle, Signature> p = provider;
        if (p == null || ticks++ % EVERY_TICKS != 0) { return; }
        Set<UUID> toldNow = new HashSet<>();
        for (ServerLevel level : event.getServer().getAllLevels()) {
            List<ServerPlayer> players = level.players();
            if (players.isEmpty()) { continue; }
            Map<ServerPlayer, List<Far>> lists = new HashMap<>();
            for (PartVehicle v : level.getEntities(EntityTypeTest.forClass(PartVehicle.class), x -> !x.isRemoved())) {
                Signature s;
                try { s = p.apply(v); } catch (RuntimeException e) { LOGGER.debug("AeroCore: no far signature for {}", v, e); continue; }
                if (s == null || !(s.levelDb() >= LOUD_DB)) { continue; }
                double range = rangeM(s.levelDb());
                for (ServerPlayer player : players) {
                    if (v.seenBy(player) || v.hasPassenger(player)) { continue; }
                    if (player.distanceToSqr(v) > range * range) { continue; }
                    lists.computeIfAbsent(player, k -> new ArrayList<>()).add(new Far(v.getUUID().hashCode(), new Vec3d(v.getX(), v.getY(), v.getZ()), v.velocity(), s));
                }
            }
            for (ServerPlayer player : players) {
                List<Far> list = lists.getOrDefault(player, List.of());
                if (list.isEmpty() && !TOLD.contains(player.getUUID())) { continue; }
                List<Far> sorted = new ArrayList<>(list);
                sorted.sort(Comparator.comparingDouble((Far f) -> -(f.signature().levelDb() - 20 * Math.log10(Math.max(1, player.position().distanceTo(new net.minecraft.world.phys.Vec3(f.position().x(), f.position().y(), f.position().z())))))));
                if (sorted.size() > MAX_PER_PLAYER) { sorted = sorted.subList(0, MAX_PER_PLAYER); }
                PacketDistributor.sendToPlayer(player, new FarPayload(List.copyOf(sorted)));
                sends++;
                if (!sorted.isEmpty()) { toldNow.add(player.getUUID()); }
            }
        }
        TOLD.clear();
        TOLD.addAll(toldNow);
    }

    // ---- the client (plain maps: safe on a server)

    private record Heard(Far far, long tick) {}
    private static final Map<Integer, Heard> HEARD = new HashMap<>();
    private static long clientTicks;

    /** Client, payload handler: the latest list (anything not in it is dropped after KEEP_TICKS). */
    static void received(FarPayload p) {
        for (Far f : p.far()) { HEARD.put(f.key(), new Heard(f, clientTicks)); }
    }

    /** Client, each tick: forgets what was not heard of lately. */
    public static void clientTick() {
        clientTicks++;
        HEARD.values().removeIf(h -> clientTicks - h.tick() > KEEP_TICKS);
    }

    /** Client: the vehicles heard from afar now, with how many ticks ago each was told. */
    public static Collection<Far> heard() {
        List<Far> out = new ArrayList<>(HEARD.size());
        for (Heard h : HEARD.values()) { out.add(h.far()); }
        return out;
    }

    /** Client: ticks since a far vehicle was last told of. */
    public static long age(int key) {
        Heard h = HEARD.get(key);
        return h == null ? Long.MAX_VALUE : clientTicks - h.tick();
    }

    /** Client: forgets everything (a new level). */
    public static void forget() { HEARD.clear(); }

    /** The client's clock for what it was told (a class of its own: it names client-only events). */
    @EventBusSubscriber(modid = AeroCore.MOD_ID, value = net.neoforged.api.distmarker.Dist.CLIENT)
    public static final class Client {
        private Client() {}
        @SubscribeEvent static void tick(net.neoforged.neoforge.client.event.ClientTickEvent.Post event) {
            if (net.minecraft.client.Minecraft.getInstance().level == null) { forget(); return; }
            clientTick();
        }
    }
}
