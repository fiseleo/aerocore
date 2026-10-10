package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import net.minecraft.world.entity.Entity;

import java.util.List;

/**
 * A test body that vanilla entities collide with, stand on and are carried by (BodyCollisions). Its solid shape for
 * entities is a set of boxes in model coordinates; the pose is the physics state on the server and the interpolated
 * presentation pose on the client, so each side collides with what it simulates or shows.
 */
public interface SolidBody {
    Entity entity();
    /** World COM of the pose used on this side. */
    Vec3d poseCenter();
    Orientation poseOrientation();
    Vec3d centerOfMassLocal();
    /** Solid boxes (min, max) in model coordinates. */
    List<WorldBox> solidBoxesLocal();
    /** World velocity of the body at a world point (server physics; client: zero is acceptable). */
    Vec3d velocityAt(Vec3d worldPoint);
    /** Server: a force from an entity at a model-local point, applied during the next physics tick, then cleared. */
    void addEntityLoad(Vec3d localPoint, Vec3d worldForce);

    default Vec3d toLocal(Vec3d world) { return poseOrientation().inverse().rotate(world.subtract(poseCenter())).add(centerOfMassLocal()); }
    default Vec3d toWorld(Vec3d local) { return poseCenter().add(poseOrientation().rotate(local.subtract(centerOfMassLocal()))); }
}
