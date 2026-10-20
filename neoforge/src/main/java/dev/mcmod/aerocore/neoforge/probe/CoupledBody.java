package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.WorldBox;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.phys.AABB;

import java.util.List;

/**
 * A server-side test body stepped by BodyWorld together with every other body in its level, so that contacts between
 * bodies are solved jointly (equal and opposite impulses). Call order per tick: beginTick, then for each substep
 * prepareSubstep on all bodies, one joint contact solve, finishSubstep on all, then endTick. A body whose calls throw
 * is taken out alone (physicsFailed).
 */
public interface CoupledBody {
    int SUBSTEPS = 10;
    Entity entity();
    /** Collects obstacles, waves and controls for this tick; false when the body must not move (unloaded, removed). */
    boolean beginTick(ServerLevel level);
    /** False once the body has stopped for the rest of this tick (it left its collected region). */
    boolean active();
    RigidBodyState physicsState();
    RigidBodyProperties properties();
    List<CollisionSphere> collisionShape();
    ContactMaterial contactMaterial();
    /** Blocks, vanilla collidable entities and the world border collected in beginTick. */
    List<WorldBox> staticObstacles();
    /** Loads and implicit velocity of the substep; returns the pre-contact state. */
    RigidBodyState prepareSubstep(int substep);
    /** Completes the substep from the joint contact result (drift, static projection, bookkeeping). */
    void finishSubstep(int substep, ContactSolver.BodyResult contact);
    /** Pose correction from body-body penetration projection; velocity is unchanged. */
    void correctPose(RigidBodyState corrected);
    void endTick();
    /** Current box around the contact spheres, for pair tests. */
    AABB physicsEnvelope();
    /**
     * This body's physics threw in phase (one of the calls above, or the contact solve on its state): BodyWorld steps it
     * no further this tick and goes on with the other bodies. A body that can stop safely stops for good (beginTick
     * false until something restarts it) from a sound state and logs the error; the default rethrows, which stops the
     * server as before.
     */
    default void physicsFailed(String phase, RuntimeException error) { throw error; }

    /**
     * Whether BodyWorld steps this body where its chunk is loaded but does not tick entities (0.28: a fast vehicle,
     * whose path is kept loaded but not ticking ahead of it); otherwise only in entity-ticking chunks.
     */
    default boolean stepsWhereLoaded() { return false; }

    // 0.36: bodies joined by couplings (a trailer on a tractor) step together, their joints solved with the contacts

    /** The entity UUID of the body this one hangs on, or null. */
    default java.util.UUID leaderId() { return null; }
    /** How this body hangs on its leader (null when it hangs on none). */
    default dev.mcmod.aerocore.api.vehicle.Coupling leaderCoupling() { return null; }
    /** The entity UUIDs of the bodies hanging on this one. */
    default java.util.Collection<java.util.UUID> followerIds() { return List.of(); }
    /** What this body's coupling to its leader carried in the substep just solved. */
    default void carried(ContactSolver.JointResult joint) {}
    /**
     * Once a tick when its group does not step (beginTick has run): missing are the partners not found in the level
     * (an unloaded or removed one), the others being there but not stepping (not ready, faulted, none in a ticking chunk).
     */
    default void groupWaits(ServerLevel level, java.util.Set<java.util.UUID> missing) {}
}
