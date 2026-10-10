package dev.mcmod.aerocore.neoforge.mixin;

import net.minecraft.world.entity.vehicle.Boat;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/** Vanilla boat fields the rowing physics reads (player input, status) and writes (the turn passed to riders). */
@Mixin(Boat.class)
public interface BoatAccessor {
    @Accessor("inputLeft") boolean aerocore$inputLeft();
    @Accessor("inputRight") boolean aerocore$inputRight();
    @Accessor("inputUp") boolean aerocore$inputUp();
    @Accessor("inputDown") boolean aerocore$inputDown();
    @Accessor("status") Boat.Status aerocore$status();
    @Accessor("deltaRotation") void aerocore$setDeltaRotation(float degreesPerTick);
}
