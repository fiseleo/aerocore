package dev.mcmod.aerocore.neoforge.mixin;

import dev.mcmod.aerocore.neoforge.client.ClientWaveRegions;
import net.minecraft.client.renderer.block.LiquidBlockRenderer;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.block.state.BlockState;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Reports the top face of a wave-region water surface as occluded, so chunk meshes leave it out (both its upper
 * and its backward face) and WaveSurfaceRenderer draws the moving surface instead. Side and bottom faces are kept.
 */
@Mixin(LiquidBlockRenderer.class)
abstract class LiquidBlockRendererMixin {
    @Inject(method = "isFaceOccludedByNeighbor", at = @At("HEAD"), cancellable = true)
    private static void aerocore$hideWaveSurface(BlockGetter level, BlockPos pos, Direction side, float height, BlockState neighbor,
                                                 CallbackInfoReturnable<Boolean> result) {
        if (side == Direction.UP && ClientWaveRegions.surfaceRegion(level, pos) != null) { result.setReturnValue(true); }
    }
}
