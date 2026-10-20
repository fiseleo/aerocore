package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.blaze3d.vertex.VertexFormat;
import net.minecraft.client.renderer.RenderStateShard;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.resources.ResourceLocation;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Render types for finished parts (0.29), one per texture and layer, made once. Each binds its texture with blur and
 * mipmaps (trilinear; a {@link FinishTexture} has the mips) and uses Minecraft's entity shaders, so they draw like
 * other entities (and shader packs treat them as such); the part shader (0.29 R4) swaps in where it is on.
 */
public final class PartRenderTypes {
    private PartRenderTypes() {}

    /** Opaque (back faces culled), opaque two-sided, on another surface (depth offset), see-through, and added light (glow). */
    public enum Layer { SOLID, SOLID_NO_CULL, DECAL, TRANSLUCENT, GLOW }

    private record Key(ResourceLocation texture, Layer layer, Object shading) {}

    private static final Map<Key, RenderType> TYPES = new ConcurrentHashMap<>();

    /** The render type for a bindable texture (see PartFinishes.bindable) and a layer, with Minecraft's shaders. */
    public static RenderType of(ResourceLocation texture, Layer layer) { return of(texture, layer, null, null); }

    /**
     * The render type with a shader swapped in for the lit layers (null: Minecraft's) and a state set up before each
     * batch (the part shader's per-finish uniforms; null: none). The shading object keys the cache with them.
     */
    public static RenderType of(ResourceLocation texture, Layer layer, RenderStateShard.ShaderStateShard shader, RenderStateShard.TexturingStateShard uniforms) {
        Object shading = shader == null ? null : java.util.List.of(shader, uniforms == null ? "none" : uniforms);
        return TYPES.computeIfAbsent(new Key(texture, layer, shading), k -> make(texture, layer, shader, uniforms));
    }

    private static RenderType make(ResourceLocation texture, Layer layer, RenderStateShard.ShaderStateShard shader, RenderStateShard.TexturingStateShard uniforms) {
        RenderStateShard.TextureStateShard tex = new RenderStateShard.TextureStateShard(texture, true, true);
        RenderType.CompositeState.CompositeStateBuilder b = RenderType.CompositeState.builder().setTextureState(tex);
        if (uniforms != null) { b.setTexturingState(uniforms); }
        String name = "aerocore_part_" + layer.name().toLowerCase(java.util.Locale.ROOT);
        switch (layer) {
            case SOLID -> {
                b.setShaderState(shader != null ? shader : RenderStateShard.RENDERTYPE_ENTITY_SOLID_SHADER)
                        .setTransparencyState(RenderStateShard.NO_TRANSPARENCY).setLightmapState(RenderStateShard.LIGHTMAP).setOverlayState(RenderStateShard.OVERLAY);
                return RenderType.create(name, DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 1536, true, false, b.createCompositeState(true));
            }
            case SOLID_NO_CULL -> {
                b.setShaderState(shader != null ? shader : RenderStateShard.RENDERTYPE_ENTITY_CUTOUT_NO_CULL_SHADER)
                        .setTransparencyState(RenderStateShard.NO_TRANSPARENCY).setCullState(RenderStateShard.NO_CULL)
                        .setLightmapState(RenderStateShard.LIGHTMAP).setOverlayState(RenderStateShard.OVERLAY);
                return RenderType.create(name, DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 1536, true, false, b.createCompositeState(true));
            }
            case DECAL -> {
                b.setShaderState(shader != null ? shader : RenderStateShard.RENDERTYPE_ENTITY_CUTOUT_NO_CULL_SHADER)
                        .setTransparencyState(RenderStateShard.NO_TRANSPARENCY).setCullState(RenderStateShard.NO_CULL)
                        .setLightmapState(RenderStateShard.LIGHTMAP).setOverlayState(RenderStateShard.OVERLAY)
                        .setLayeringState(RenderStateShard.POLYGON_OFFSET_LAYERING);
                return RenderType.create(name, DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 1536, false, false, b.createCompositeState(true));
            }
            case TRANSLUCENT -> {
                b.setShaderState(shader != null ? shader : RenderStateShard.RENDERTYPE_ENTITY_TRANSLUCENT_SHADER)
                        .setTransparencyState(RenderStateShard.TRANSLUCENT_TRANSPARENCY).setCullState(RenderStateShard.NO_CULL)
                        .setLightmapState(RenderStateShard.LIGHTMAP).setOverlayState(RenderStateShard.OVERLAY);
                return RenderType.create(name, DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 1536, true, true, b.createCompositeState(true));
            }
            default -> {
                // glow: added light, no depth written, unlit (flames, hot metal); the eyes shader, as glowing eyes are
                b.setShaderState(RenderStateShard.RENDERTYPE_EYES_SHADER).setTransparencyState(RenderStateShard.ADDITIVE_TRANSPARENCY)
                        .setCullState(RenderStateShard.NO_CULL).setWriteMaskState(RenderStateShard.COLOR_WRITE);
                return RenderType.create(name, DefaultVertexFormat.NEW_ENTITY, VertexFormat.Mode.QUADS, 1536, false, true, b.createCompositeState(false));
            }
        }
    }

    /** Forgets the types made (a shader or its setting changed). */
    public static void clear() { TYPES.clear(); }

    /** 0.31: forgets the render types made for a texture (a finish defined at run time and taken back). */
    static void forget(net.minecraft.resources.ResourceLocation texture) { TYPES.keySet().removeIf(k -> k.texture().equals(texture)); }
}
