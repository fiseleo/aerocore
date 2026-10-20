package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.api.aero.Downwash;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.core.particles.BlockParticleOption;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.util.RandomSource;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.Vec3;
import net.minecraft.world.phys.shapes.CollisionContext;

/**
 * A rotor's (or a downward jet's) wash on what is below it (0.30), a tick at a time: dust off the block under the
 * ring where the outwash is strong (that block's own particles, blown outward), and on water a ring of spray
 * (SprayRenderer's droplets). How strong comes from Downwash (a display approximation); how many particles is a
 * display budget (at most 12 a tick a rotor). Client only.
 */
public final class RotorWash {
    private RotorWash() {}

    private static final int MOST = 12;

    /**
     * The wash this tick of a rotor at a hub (world) blowing along a direction (unit, down for a helicopter), of a
     * radius (m) and induced velocity (m/s).
     */
    public static void tick(ClientLevel level, Vec3 hub, Vec3 down, double radiusM, double inducedMps, RandomSource random) {
        if (!(radiusM > 0) || !(inducedMps > 0.5)) { return; }
        double reach = 4.5 * 2 * radiusM;
        BlockHitResult hit = level.clip(new ClipContext(hub, hub.add(down.scale(reach)), ClipContext.Block.COLLIDER, ClipContext.Fluid.ANY, CollisionContext.empty()));
        if (hit.getType() != HitResult.Type.BLOCK) { return; }
        double height = hit.getLocation().distanceTo(hub);
        double strongest = Downwash.outwash(inducedMps, radiusM, height, radiusM);
        if (strongest < 1.5) { return; }
        int n = Math.min(MOST, (int) Math.round(strongest * 0.8));
        BlockPos pos = hit.getBlockPos();
        BlockState state = level.getBlockState(pos);
        boolean water = !level.getFluidState(pos).isEmpty();
        Vec3 ground = hit.getLocation();
        for (int i = 0; i < n; i++) {
            double a = random.nextDouble() * 2 * Math.PI, r = radiusM * (0.8 + 1.2 * random.nextDouble());
            double u = Downwash.outwash(inducedMps, radiusM, height, r);
            double cx = Math.cos(a), cz = Math.sin(a);
            double x = ground.x + cx * r, z = ground.z + cz * r;
            if (water) {
                SprayRenderer.emit(x, ground.y + 0.05, z, cx * u * 0.6, u * (0.15 + 0.2 * random.nextDouble()), cz * u * 0.6, ground.y);
            } else if (!state.isAir()) {
                level.addParticle(new BlockParticleOption(ParticleTypes.BLOCK, state), x, ground.y + 0.1, z, cx * u * 0.4, 0.1 + 0.2 * random.nextDouble(), cz * u * 0.4);
            }
        }
    }
}
