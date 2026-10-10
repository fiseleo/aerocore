package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.probe.BodyCollisions;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.phys.Vec3;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Clips the movement vanilla allows against AeroCore bodies' solid boxes, so entities stand on and walk into rotated
 * decks and hulls. Entity.move derives its collision flags (onGround, horizontalCollision) from the clipped result.
 */
@Mixin(Entity.class)
abstract class EntityCollideMixin {
    @Inject(method = "collide", at = @At("RETURN"), cancellable = true)
    private void aerocore$collideWithBodies(Vec3 movement, CallbackInfoReturnable<Vec3> result) {
        Vec3 allowed = result.getReturnValue(), clipped = BodyCollisions.clip((Entity) (Object) this, allowed);
        if (clipped != allowed) { result.setReturnValue(clipped); }
    }
}
