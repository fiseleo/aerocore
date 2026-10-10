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
 * prepareSubstep on all bodies, one joint contact solve, finishSubstep on all, then endTick.
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
}
