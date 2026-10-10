package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import net.minecraft.world.entity.Entity;

import java.util.List;
import java.util.UUID;

/** What PlatformDebug needs from a server-side test body to draw its forces and sampled flow. */
public interface DebugBody {
    Entity entity();
    int debugMode();
    UUID owner();
    /** Forces as integrated in the last substep, with offsets from forceOrigin(). */
    List<AppliedForce> displayForces();
    Vec3d forceOrigin();
    WaveRegions.Local waves();
    Vec3d modelOrigin();
    RigidBodyState physicsState();
    RigidBodyProperties properties();
    PlatformEnvironment environment();
    /** Lumped body temperature, or NaN for a body without a thermal model. */
    double bodyTemperatureCelsius();
}
