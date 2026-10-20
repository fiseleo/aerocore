package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.vehicle.Coupling;

import java.util.UUID;

/**
 * 0.36: one vehicle hanging on another in the world (a trailer on a tractor), kept on the two entities: the follower
 * saves its leader's UUID and the coupling ("Coupling"), the leader its followers' ("Towing"), each with where the other
 * was last seen. BodyWorld steps a joined group together and solves the joints with the contacts.
 * <ul>
 *   <li>A group steps only when every member is loaded and ready and at least one stands in an entity-ticking chunk; a
 *   joined vehicle steps where its chunk is merely loaded and keeps its chunk loaded for 2 s each tick it steps, so the
 *   group never parts at a chunk edge.</li>
 *   <li>A partner missing (unloaded) keeps the group waiting where it is; one missing PATIENCE_TICKS while its last
 *   place is loaded is gone, and the link with it ends (REMOVED). Points found farther apart than BREAK_M (a vehicle
 *   moved by a command, a save edited) end the link (BROKEN); nearer, separateAll closes the gap.</li>
 *   <li>A follower copies its leader's controls on the coupling's forwarded channels each tick (brake lights, trailer brakes).</li>
 *   <li>Links end with the follower's uncouple (COMMAND), a physics fault of either (FAULT), either removed (killed,
 *   discarded, another dimension: REMOVED), placed or refitted (MOVED). replaceBlueprint and castOff keep them.</li>
 * </ul>
 * Coupled and Uncoupled are posted on NeoForge's bus (server).
 */
public final class Couplings {
    private Couplings() {}

    /** Ticks a missing partner is waited for once its last place is loaded. */
    public static final int PATIENCE_TICKS = 200;
    /** Points this far apart (m) end the link. */
    public static final double BREAK_M = 1.0;
    /** Couplings followed up a chain before it counts as a cycle. */
    static final int MOST_LINKS = 64;

    /** What PartVehicle.couple did. */
    public enum Result { OK, TOO_FAR, ALREADY, CYCLE, NOT_READY, OTHER_LEVEL }
    /** Why a link ended. */
    public enum Reason { COMMAND, FAULT, REMOVED, MOVED, BROKEN }

    /** A follower now hangs on a leader. */
    public static final class Coupled extends net.neoforged.bus.api.Event {
        private final PartVehicle leader, follower;
        private final Coupling coupling;
        public Coupled(PartVehicle leader, PartVehicle follower, Coupling coupling) { this.leader = leader; this.follower = follower; this.coupling = coupling; }
        public PartVehicle leader() { return leader; }
        public PartVehicle follower() { return follower; }
        public Coupling coupling() { return coupling; }
    }

    /** A link ended; either entity may be null (not loaded, or gone), the UUIDs are always there. */
    public static final class Uncoupled extends net.neoforged.bus.api.Event {
        private final PartVehicle leader, follower;
        private final UUID leaderId, followerId;
        private final Reason reason;
        public Uncoupled(PartVehicle leader, PartVehicle follower, UUID leaderId, UUID followerId, Reason reason) {
            this.leader = leader; this.follower = follower; this.leaderId = leaderId; this.followerId = followerId; this.reason = reason;
        }
        public PartVehicle leader() { return leader; }
        public PartVehicle follower() { return follower; }
        public UUID leaderId() { return leaderId; }
        public UUID followerId() { return followerId; }
        public Reason reason() { return reason; }
    }
}
