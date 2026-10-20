package dev.mcmod.aerocore.neoforge.client;

import net.minecraft.client.renderer.RenderType;

/**
 * Which render type draws a finish in a layer (0.29): the part shader (PartShader: highlights, reflections; gloss
 * per pixel from the texture's alpha, metalness per finish) where it is active, else Minecraft's entity shaders with
 * the same mipmapped tiling texture. Glow is always Minecraft's additive eyes type.
 */
public final class PartShading {
    private PartShading() {}

    public static RenderType type(PartFinishes.Look look, PartRenderTypes.Layer layer) {
        var texture = PartFinishes.bindable(look);
        if (layer != PartRenderTypes.Layer.GLOW && PartShader.active()) {
            return PartRenderTypes.of(texture, layer, PartShader.STATE, PartShader.surface(look));
        }
        return PartRenderTypes.of(texture, layer);
    }
}
