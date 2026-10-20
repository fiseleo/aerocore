package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.BufferBuilder;
import com.mojang.blaze3d.vertex.ByteBufferBuilder;
import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.blaze3d.vertex.MeshData;
import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexBuffer;
import com.mojang.blaze3d.vertex.VertexFormat;
import dev.mcmod.aerocore.api.mesh.PackedMesh;
import net.minecraft.client.renderer.LightTexture;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.ShaderInstance;
import net.minecraft.client.renderer.texture.OverlayTexture;
import org.joml.Matrix4f;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Baked meshes' opaque groups kept on the GPU (0.29): each uploaded once, when first drawn (positions and normals in
 * the mesh's own frame, the occlusion shade in the colour), then drawn every frame with the part's pose, normal matrix
 * and light set as the part shader's uniforms (PartPose, PartNormalMat, PartLight), instead of every vertex being
 * transformed and written again (about 35 ns a vertex on the render thread). Only with the part shader: Minecraft's
 * entity shaders take pose and light from each vertex. A buffer is freed when its mesh is forgotten (MeshBakery's
 * cache dropped it) or when all are cleared. Render thread only.
 */
public final class PartGpu {
    private PartGpu() {}

    /** A group of a baked mesh to draw where a part is: the mesh, the group's index, the pose and the packed light. */
    record Job(PackedMesh mesh, int group, PoseStack.Pose pose, int light) {}

    private record Held(WeakReference<PackedMesh> mesh, VertexBuffer[] buffers) {}

    private static final Map<PackedMesh, Held> HELD = new WeakHashMap<>();
    private static final List<Held> ALL = new ArrayList<>();
    private static volatile boolean clearing;
    /** Whether to draw from buffers at all (a development switch: /aerocoredev parts gpu on|off; 0.41: public for the development tools mod). */
    public static volatile boolean enabled = true;
    private static final Matrix4f IDENTITY = new Matrix4f();

    /** Frees every buffer at the next draw (resources reloaded, bakes cleared). */
    static void clear() { clearing = true; }

    /** Whether the shader of a render type takes the part's pose and light as uniforms (the part shader does). */
    static boolean takes(ShaderInstance shader) {
        return shader != null && shader.getUniform("PartPose") != null && shader.getUniform("PartNormalMat") != null && shader.getUniform("PartLight") != null;
    }

    /** Draws a render type's jobs from their buffers (uploading what is new); false (nothing drawn) when its shader cannot. */
    static boolean draw(RenderType type, List<Job> jobs) {
        sweep();
        type.setupRenderState();
        ShaderInstance shader = RenderSystem.getShader();
        if (!takes(shader)) { type.clearRenderState(); return false; }
        var pose = shader.getUniform("PartPose");
        var normal = shader.getUniform("PartNormalMat");
        var light = shader.getUniform("PartLight");
        Matrix4f view = RenderSystem.getModelViewMatrix(), projection = RenderSystem.getProjectionMatrix();
        Matrix4f normalMatrix = new Matrix4f();
        int calls = 0;
        // 0.39: the shader set up once for the render type, then per job only its pose, normal matrix and light uploaded
        // before the draw. VertexBuffer.drawWithShader applies the whole shader every call (each sampler's location
        // looked up, its texture bound, every uniform checked): with a vehicle's wheels drawn as pieces (hdfv 0.22) a
        // line-up of five vehicles spent 5 ms a frame there
        shader.setDefaultUniforms(VertexFormat.Mode.QUADS, view, projection, net.minecraft.client.Minecraft.getInstance().getWindow());
        shader.apply();
        for (Job j : jobs) {
            VertexBuffer vb = buffer(j.mesh(), j.group());
            if (vb == null) { continue; }
            calls++;
            pose.set(j.pose().pose());
            normal.set(normalMatrix.set(j.pose().normal()));
            light.set(j.light() & 0xffff, (j.light() >> 16) & 0xffff);
            pose.upload();
            normal.upload();
            light.upload();
            vb.bind();
            vb.draw();
        }
        shader.clear();
        VertexBuffer.unbind();
        PartStats.calls(calls);
        // back to vertices that carry their own pose and light, for everything written vertex by vertex
        pose.set(IDENTITY);
        normal.set(IDENTITY);
        light.set(-1, -1);
        type.clearRenderState();
        return true;
    }

    private static VertexBuffer buffer(PackedMesh mesh, int group) {
        Held h = HELD.get(mesh);
        if (h == null) {
            h = new Held(new WeakReference<>(mesh), new VertexBuffer[mesh.groups().size()]);
            HELD.put(mesh, h);
            ALL.add(h);
        }
        if (h.buffers()[group] == null) { h.buffers()[group] = upload(mesh, mesh.groups().get(group)); }
        return h.buffers()[group];
    }

    private static VertexBuffer upload(PackedMesh m, PackedMesh.Group g) {
        if (g.count() == 0) { return null; }
        try (ByteBufferBuilder bytes = new ByteBufferBuilder(4 * g.count() * DefaultVertexFormat.NEW_ENTITY.getVertexSize())) {
            BufferBuilder b = new BufferBuilder(bytes, VertexFormat.Mode.QUADS, DefaultVertexFormat.NEW_ENTITY);
            float[] p = m.positions(), n = m.normals(), uv = m.uvs();
            int[] c = m.colors();
            for (int v = 4 * g.first(), end = 4 * (g.first() + g.count()); v < end; v++) {
                int col = c[v];
                float shade = m.light(v) / 255f;
                b.addVertex(p[3 * v], p[3 * v + 1], p[3 * v + 2])
                        .setColor((int) (((col >> 16) & 255) * shade + 0.5f), (int) (((col >> 8) & 255) * shade + 0.5f), (int) ((col & 255) * shade + 0.5f), (col >>> 24) & 255)
                        .setUv(uv[2 * v], uv[2 * v + 1]).setOverlay(OverlayTexture.NO_OVERLAY).setLight(LightTexture.FULL_BRIGHT)
                        .setNormal(n[3 * v], n[3 * v + 1], n[3 * v + 2]);
            }
            MeshData data = b.build();
            if (data == null) { return null; }
            VertexBuffer vb = new VertexBuffer(VertexBuffer.Usage.STATIC);
            vb.bind();
            vb.upload(data);
            VertexBuffer.unbind();
            return vb;
        }
    }

    /** Frees the buffers of meshes no longer held anywhere, or all after a clear. */
    private static void sweep() {
        boolean all = clearing;
        clearing = false;
        for (Iterator<Held> it = ALL.iterator(); it.hasNext(); ) {
            Held h = it.next();
            if (!all && h.mesh().get() != null) { continue; }
            for (VertexBuffer vb : h.buffers()) { if (vb != null) { vb.close(); } }
            it.remove();
        }
        if (all) { HELD.clear(); }
    }

    /** Buffers held now (PartStats). */
    static int held() { return ALL.size(); }
}
