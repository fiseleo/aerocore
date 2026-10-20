package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * What a vehicle's body touched in a tick (0.37), for sounds and effects; read from the contact solve, it changes
 * nothing in it. On the server, PartVehicle sums its substeps' {@link ContactSolver.Touch touches} into at most one
 * {@link Impact} (contacts that met it closing faster than {@link #IMPACT_MPS}) and one {@link Scrape} (contacts sliding
 * faster than {@link #SCRAPE_MPS}) per tick and posts {@link Contacted}; an impact is also sent to the players tracking
 * the vehicle (one small packet), a scrape rides on the display sync. Wheels are not in it (tyres touch the ground
 * through their own model).
 */
public final class VehicleContacts {
    private VehicleContacts() {}

    /** Closing speed (m/s) from which a contact is an impact. */
    public static final double IMPACT_MPS = 0.6;
    /** Sliding speed (m/s) from which a contact scrapes. */
    public static final double SCRAPE_MPS = 0.3;

    /** The block materials a contact can be against, in a fixed order (their index goes over the network); -1 for others. */
    public static final List<ContactMaterial> MATERIALS = List.of(ContactMaterial.STEEL_ON_ROCK, ContactMaterial.STEEL_ON_STEEL,
            ContactMaterial.STEEL_ON_WOOD, ContactMaterial.STEEL_ON_ICE, ContactMaterial.STEEL_ON_SAND, ContactMaterial.STEEL_ON_GRAVEL,
            ContactMaterial.STEEL_ON_SOIL, ContactMaterial.STEEL_ON_MUD, ContactMaterial.STEEL_ON_SNOW, ContactMaterial.STEEL_ON_WOOL,
            ContactMaterial.STEEL_ON_GLASS);

    public static int materialIndex(ContactMaterial m) { return m == null ? -1 : MATERIALS.indexOf(m); }
    public static ContactMaterial material(int index) { return index >= 0 && index < MATERIALS.size() ? MATERIALS.get(index) : null; }

    /**
     * A tick's impact: where (world point), the summed normal impulse (N s), the fastest closing speed (m/s), the
     * strongest contact's material index (-1 when not a block material) and whether it was another body.
     */
    public record Impact(Vec3d point, double impulseNs, double approachMps, int material, boolean body) {}

    /** A tick's scraping: where (world point), the mean frictional power (W), the fastest sliding speed and the material index. */
    public record Scrape(Vec3d point, double powerW, double slideMps, int material, boolean body) {}

    /** Server: a vehicle's body hit or scraped something this tick (either may be null). */
    public static final class Contacted extends net.neoforged.bus.api.Event {
        private final PartVehicle vehicle;
        private final Impact impact;
        private final Scrape scrape;
        public Contacted(PartVehicle vehicle, Impact impact, Scrape scrape) { this.vehicle = vehicle; this.impact = impact; this.scrape = scrape; }
        public PartVehicle vehicle() { return vehicle; }
        public Impact impact() { return impact; }
        public Scrape scrape() { return scrape; }
    }

    /** Sums one tick's touches (server). */
    static final class Tally {
        double impactNs, impactMps, wx, wy, wz, strongest = -1;
        int impactMaterial = -1;
        boolean impactBody;
        double scrapeJ, scrapeMps, sx, sy, sz, scrapeWeight, scrapeStrongest = -1;
        int scrapeMaterial = -1;
        boolean scrapeBody;

        void clear() {
            impactNs = impactMps = wx = wy = wz = 0; strongest = -1; impactMaterial = -1; impactBody = false;
            scrapeJ = scrapeMps = sx = sy = sz = scrapeWeight = 0; scrapeStrongest = -1; scrapeMaterial = -1; scrapeBody = false;
        }

        void add(Vec3d com, List<ContactSolver.Touch> touches, double dt) {
            for (ContactSolver.Touch t : touches) {
                Vec3d p = com.add(t.offset());
                if (-t.approachMps() >= IMPACT_MPS) {
                    impactNs += t.normalImpulseNs();
                    impactMps = Math.max(impactMps, -t.approachMps());
                    wx += p.x() * t.normalImpulseNs(); wy += p.y() * t.normalImpulseNs(); wz += p.z() * t.normalImpulseNs();
                    if (t.normalImpulseNs() > strongest) { strongest = t.normalImpulseNs(); impactMaterial = materialIndex(t.material()); impactBody = t.other() >= 0; }
                }
                if (t.slideMps() >= SCRAPE_MPS) {
                    // frictional work this substep: mu N |slip| dt, with N dt the normal impulse
                    double work = t.material().friction() * t.normalImpulseNs() * t.slideMps();
                    scrapeJ += work;
                    scrapeMps = Math.max(scrapeMps, t.slideMps());
                    sx += p.x() * work; sy += p.y() * work; sz += p.z() * work; scrapeWeight += work;
                    if (work > scrapeStrongest) { scrapeStrongest = work; scrapeMaterial = materialIndex(t.material()); scrapeBody = t.other() >= 0; }
                }
            }
        }

        Impact impact() {
            return impactNs <= 0 ? null : new Impact(new Vec3d(wx / impactNs, wy / impactNs, wz / impactNs), impactNs, impactMps, impactMaterial, impactBody);
        }

        Scrape scrape(double tickSeconds) {
            return scrapeWeight <= 0 ? null : new Scrape(new Vec3d(sx / scrapeWeight, sy / scrapeWeight, sz / scrapeWeight), scrapeJ / tickSeconds, scrapeMps, scrapeMaterial, scrapeBody);
        }
    }

    /** An impact sent to the clients tracking the vehicle. */
    public record ImpactPayload(int entityId, float x, float y, float z, float impulseNs, float approachMps, byte material, boolean body) implements CustomPacketPayload {
        public static final Type<ImpactPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "vehicle_impact"));
        public static final StreamCodec<FriendlyByteBuf, ImpactPayload> STREAM_CODEC = StreamCodec.of(
                (b, p) -> {
                    b.writeVarInt(p.entityId()); b.writeFloat(p.x()); b.writeFloat(p.y()); b.writeFloat(p.z());
                    b.writeFloat(p.impulseNs()); b.writeFloat(p.approachMps()); b.writeByte(p.material()); b.writeBoolean(p.body());
                },
                b -> new ImpactPayload(b.readVarInt(), b.readFloat(), b.readFloat(), b.readFloat(), b.readFloat(), b.readFloat(), b.readByte(), b.readBoolean()));
        @Override public Type<? extends CustomPacketPayload> type() { return TYPE; }
        public Impact impact() { return new Impact(new Vec3d(x, y, z), impulseNs, approachMps, material, body); }
    }

    private static final List<ClientListener> CLIENT = new CopyOnWriteArrayList<>();

    /** Client: told of every impact the server sends (on the game thread). */
    public interface ClientListener { void impact(PartVehicle vehicle, Impact impact); }

    public static void listen(ClientListener listener) { CLIENT.add(listener); }

    static void received(PartVehicle vehicle, Impact impact) {
        for (ClientListener l : CLIENT) { l.impact(vehicle, impact); }
    }
}
