package dev.mcmod.aerocore.neoforge.vehicle;

import net.minecraft.world.level.block.state.BlockState;

/**
 * 0.42: a block whose top is a plane wheels roll on (a road ramp). A block's collision shape can only stand for a slope
 * as steps, which wheels' rays take for treads and risers; WorldGround asks a block that implements this for the plane
 * instead (api.ground.SlopedCell) and keeps the collision shape for the block's sides and for everything else that
 * collides with it (feet, bodies).
 */
public interface SlopedGround {
    /**
     * The top of a state as {h0, gx, gz}: its height above the block's floor (blocks) at the corner with the lowest x
     * and z, and its rise per block along +x and +z; null for a state that is not sloped (its shape is the ground).
     * The top should stay within the block's height.
     */
    double[] groundSlope(BlockState state);
}
