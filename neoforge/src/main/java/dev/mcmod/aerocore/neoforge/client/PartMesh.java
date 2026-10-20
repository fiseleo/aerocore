package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.mesh.MeshBuilder;
import net.minecraft.client.renderer.texture.OverlayTexture;
import net.minecraft.resources.ResourceLocation;

/**
 * Vertex-coloured quads for drawing parts (PartVehicleRenderer, PartRenderer, a mod's design previews). The shapes are
 * built by {@link MeshBuilder} (0.29: moved to the physics module, with texture coordinates, finishes and tags); this
 * class adds emitting them through a pose. The plain emit here draws every quad with {@link #TEXTURE} and no texture
 * coordinates, as before 0.29; textured, multi-pass drawing goes through PartLayers.
 * Colours are ARGB; an alpha below 0xff marks a quad for a translucent pass (a design screen's cutaway), while a
 * vehicle in the world draws every quad opaque.
 */
public final class PartMesh extends MeshBuilder<PartMesh> {
    /** The plain white texture parts are drawn with (the colour is per vertex). */
    public static final ResourceLocation TEXTURE = ResourceLocation.withDefaultNamespace("textures/block/white_concrete.png");

    @Override protected PartMesh create() { return new PartMesh(); }

    // ---------------------------------------------------------------- drawing

    /** Emits the quads through a pose. */
    public void emit(VertexConsumer out, PoseStack stack, int light) {
        for (Quad q : quads()) { emit(out, stack, q, light); }
    }

    /** Emits only the opaque quads, or only the translucent ones (for a translucent render type drawn after the opaque pass). */
    public void emit(VertexConsumer out, PoseStack stack, int light, boolean translucent) {
        for (Quad q : quads()) { if (q.translucent() == translucent) { emit(out, stack, q, light); } }
    }

    /** Emits one quad, with its corner normals and colours where it has them. */
    public static void emit(VertexConsumer out, PoseStack stack, Quad q, int light) {
        Vec3d[] corners = {q.a(), q.b(), q.c(), q.d()};
        for (int i = 0; i < 4; i++) {
            vertex(out, stack, corners[i], q.colors() == null ? q.color() : q.colors()[i], q.normals() == null ? q.normal() : q.normals()[i], light);
        }
    }

    static void vertex(VertexConsumer out, PoseStack stack, Vec3d p, int color, Vec3d n, int light) {
        out.addVertex(stack.last(), (float) p.x(), (float) p.y(), (float) p.z()).setColor(color).setUv(0, 0)
                .setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(stack.last(), (float) n.x(), (float) n.y(), (float) n.z());
    }
}
