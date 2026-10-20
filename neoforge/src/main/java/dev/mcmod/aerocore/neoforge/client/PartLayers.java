package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.mesh.MeshBaker;
import dev.mcmod.aerocore.api.mesh.MeshBuilder;
import dev.mcmod.aerocore.api.mesh.PackedMesh;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.texture.OverlayTexture;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Where a renderer puts finished parts during a frame (0.29), drawn all at once by {@link #flush}: each finish's
 * texture and each layer (solid, two-sided, decal, translucent, glow) is asked of the buffers once, with every
 * mesh that uses it. A buffer source ends a batch whenever a different render type is asked for, so drawing part by
 * part with a type per finish would end dozens of batches per vehicle; recording first keeps it to one per type.
 * <p>
 * Meshes are baked ({@link PackedMesh}); a small mesh built on the spot can be baked here without occlusion. Old
 * code that writes to a VertexConsumer can write to {@link #legacy()}, which records the vertices and draws them
 * with Minecraft's plain entity type ({@link PartMesh#TEXTURE}).
 */
public final class PartLayers {
    /** Which passes of a mesh to draw. */
    public static final int SOLID = 1, TRANSLUCENT = 2, DECAL = 4, ALL = SOLID | TRANSLUCENT | DECAL;

    private record Draw(PackedMesh mesh, PoseStack.Pose pose, int light, int tint, boolean noCull, int passes, boolean glow, boolean hotOnly, boolean once, int lampOnly) {
        Draw(PackedMesh mesh, PoseStack.Pose pose, int light, int tint, boolean noCull, int passes, boolean glow, boolean hotOnly, boolean once) {
            this(mesh, pose, light, tint, noCull, passes, glow, hotOnly, once, -1);
        }
    }

    /** 0.30: a lamp's halo, beam and pool to draw where it is now, at a brightness (LampRenderer). */
    record LampDraw(dev.mcmod.aerocore.api.mesh.Lamp lamp, PoseStack.Pose pose, float level, boolean glint) {
        LampDraw(dev.mcmod.aerocore.api.mesh.Lamp lamp, PoseStack.Pose pose, float level) { this(lamp, pose, level, false); }
    }

    private final List<LampDraw> lamps = new ArrayList<>();
    private static final List<LampDraw> LATE_LAMPS = new ArrayList<>();

    private record Job(Draw draw, PackedMesh.Group group) {}

    private final List<Draw> draws = new ArrayList<>();
    private Recorder legacy;
    private boolean gpu, late;
    /**
     * 0.30: groups smaller than this are written vertex by vertex even with gpu: each group on the GPU is a draw call,
     * and many tiny ones (a lamp's lens, a small piece) cost more in calls than writing their few vertices. 0.39: 32
     * (96 before): a call no longer applies the whole shader (PartGpu), so a group of a few dozen quads (a wheel's nuts,
     * a hub, a caliper) is cheaper drawn from its buffer than written every frame.
     */
    static final int GPU_MIN_QUADS = 32;
    /** 0.30: translucent groups of world renderers asked to draw late (glass), by render type, until the late flush. */
    private static final Map<RenderType, List<Job>> LATE = new LinkedHashMap<>();

    /**
     * Draws opaque groups from buffers kept on the GPU where it can (PartGpu: with the part shader, untinted): the
     * world's renderers ask for it; a GUI view writes its vertices as before.
     */
    public PartLayers gpu(boolean on) { gpu = on; return this; }

    /**
     * 0.30: translucent groups (glass) drawn after the world's translucent blocks (PartLate) instead of at the flush:
     * drawn with the entities, glass writes depth before the water behind it is drawn, which then does not show
     * through. World renderers ask for it; a GUI view draws at once.
     */
    public PartLayers late(boolean on) { late = on; return this; }

    /** Draws a mesh's opaque, decal and translucent groups through a pose, with a packed light. */
    public PartLayers draw(PackedMesh mesh, PoseStack stack, int light) { return draw(mesh, stack, light, 0xffffffff, false, ALL); }

    /**
     * Draws some of a mesh's passes through a pose with a light, its colours multiplied by a tint (ARGB), back
     * faces drawn too where noCull (an open or cut model). With gpu, a mesh drawn this way is taken to be kept (a
     * bake from MeshBakery): its opaque groups are uploaded once and drawn from the GPU after.
     */
    public PartLayers draw(PackedMesh mesh, PoseStack stack, int light, int tint, boolean noCull, int passes) {
        return record(mesh, stack, light, tint, noCull, passes, false);
    }

    private PartLayers record(PackedMesh mesh, PoseStack stack, int light, int tint, boolean noCull, int passes, boolean once) {
        if (mesh != null && !mesh.isEmpty()) { draws.add(new Draw(mesh, stack.last().copy(), light, tint, noCull, passes, false, false, once)); }
        return this;
    }

    /** Draws a mesh built on the spot (baked here, without occlusion): small moving pieces. */
    public PartLayers draw(MeshBuilder<?> mesh, PoseStack stack, int light) { return record(MeshBaker.bake(mesh, null), stack, light, 0xffffffff, false, ALL, true); }

    /**
     * Adds light: a mesh drawn as glow (additive, unlit, no depth written) in a colour whose alpha is its strength,
     * either whole (a flame) or only its hot groups (glowing metal over the solid drawing).
     */
    public PartLayers glow(PackedMesh mesh, PoseStack stack, int argb, boolean hotOnly) {
        if (mesh != null && !mesh.isEmpty() && (argb >>> 24) > 0) { draws.add(new Draw(mesh, stack.last().copy(), 0xf000f0, argb, true, ALL, true, hotOnly, true)); }
        return this;
    }

    /**
     * 0.30: a piece's lamps where it is drawn (Drawn: the bake's lamps and how bright the renderer said each is now):
     * the lens glows, and a point or beam lamp also gets its halo, cone and pool (in the world only: a layers drawn
     * late; a GUI view shows lit lenses).
     */
    public PartLayers lamps(PartRenderer.Drawn drawn, int piece, PoseStack stack) {
        if (drawn == null) { return this; }
        List<dev.mcmod.aerocore.api.mesh.Lamp> all = drawn.baked().lamps();
        PackedMesh mesh = drawn.piece(piece);
        for (int i = 0; i < all.size() && i < drawn.lampLevels().length; i++) {
            dev.mcmod.aerocore.api.mesh.Lamp lamp = all.get(i);
            float level = drawn.lampLevels()[i];
            if (lamp.piece() != piece || !(level > 0.01f)) { continue; }
            int strength = (int) (255 * Math.min(1f, level));
            if (mesh != null && !mesh.isEmpty()) {
                draws.add(new Draw(mesh, stack.last().copy(), 0xf000f0, (strength << 24) | (lamp.argb() & 0xffffff), true, ALL, true, false, true, lamp.id()));
            }
            if (lamp.kind() != dev.mcmod.aerocore.api.mesh.Lamp.Kind.LENS) { lamps.add(new LampDraw(lamp, stack.last().copy(), level)); }
            else if (late && Beautify.on() && mesh != null && !mesh.isEmpty()) {
                // 0.39 beautified: a soft halo over each lit patch of the lens (a tower's windows, a tail lamp)
                PoseStack.Pose pose = stack.last().copy();
                for (dev.mcmod.aerocore.api.mesh.LensPatches.Patch p : patches(mesh, lamp.id())) {
                    dev.mcmod.aerocore.api.mesh.Lamp glint = dev.mcmod.aerocore.api.mesh.Lamp.point(lamp.id(), piece, new dev.mcmod.aerocore.api.Vec3d(p.x(), p.y(), p.z()),
                            new dev.mcmod.aerocore.api.Vec3d(p.nx(), p.ny(), p.nz()), p.radiusM(), lamp.argb(), 360, lamp.role());
                    lamps.add(new LampDraw(glint, pose, level, true));
                }
            }
        }
        return this;
    }

    /** 0.39: a baked mesh's lens patches by lamp number, kept while the mesh is (render thread). */
    private static final Map<PackedMesh, Map<Integer, List<dev.mcmod.aerocore.api.mesh.LensPatches.Patch>>> PATCHES = new java.util.WeakHashMap<>();

    private static List<dev.mcmod.aerocore.api.mesh.LensPatches.Patch> patches(PackedMesh mesh, int lamp) {
        return PATCHES.computeIfAbsent(mesh, k -> new java.util.HashMap<>()).computeIfAbsent(lamp, l -> dev.mcmod.aerocore.api.mesh.LensPatches.of(mesh, l));
    }

    /** A VertexConsumer for code written before 0.29: what it writes is drawn with the plain entity type at the flush. */
    public VertexConsumer legacy() {
        if (legacy == null) { legacy = new Recorder(); }
        return legacy;
    }

    public boolean isEmpty() { return draws.isEmpty() && (legacy == null || legacy.count == 0); }

    /** Draws everything recorded into the buffers, a render type at a time, and forgets it. */
    public void flush(MultiBufferSource buffers) {
        long t0 = System.nanoTime();
        if (legacy != null && legacy.count > 0) { legacy.replay(buffers.getBuffer(RenderType.entitySolid(PartMesh.TEXTURE))); legacy.count = 0; }
        Map<RenderType, List<Job>> byType = new LinkedHashMap<>();
        Map<RenderType, List<PartGpu.Job>> onGpu = new LinkedHashMap<>();
        boolean gpuNow = gpu && PartGpu.enabled && PartShader.active();
        List<RenderType>[] order = orderLists();
        for (Draw d : draws) {
            List<PackedMesh.Group> groups = d.mesh().groups();
            for (int gi = 0; gi < groups.size(); gi++) {
                PackedMesh.Group g = groups.get(gi);
                PartRenderTypes.Layer layer;
                if (d.glow()) {
                    if (d.hotOnly() && !g.hot()) { continue; }
                    if (d.lampOnly() >= 0 && g.lamp() != d.lampOnly()) { continue; }
                    layer = PartRenderTypes.Layer.GLOW;
                } else {
                    layer = switch (g.pass()) {
                        case SOLID -> (d.passes() & SOLID) == 0 ? null : d.noCull() ? PartRenderTypes.Layer.SOLID_NO_CULL : PartRenderTypes.Layer.SOLID;
                        case DECAL -> (d.passes() & DECAL) == 0 ? null : PartRenderTypes.Layer.DECAL;
                        case TRANSLUCENT -> (d.passes() & TRANSLUCENT) == 0 ? null : PartRenderTypes.Layer.TRANSLUCENT;
                    };
                    if (layer == null) { continue; }
                }
                RenderType type = PartShading.type(PartFinishes.look(g.finish()), layer);
                if (late && layer == PartRenderTypes.Layer.TRANSLUCENT) {
                    LATE.computeIfAbsent(type, k -> new ArrayList<>()).add(new Job(d, g));
                    continue;
                }
                if (gpuNow && !d.once() && !d.glow() && d.tint() == 0xffffffff && layer != PartRenderTypes.Layer.TRANSLUCENT && g.count() >= GPU_MIN_QUADS) {
                    onGpu.computeIfAbsent(type, k -> new ArrayList<>()).add(new PartGpu.Job(d.mesh(), gi, d.pose(), d.light()));
                    continue;
                }
                List<Job> jobs = byType.get(type);
                if (jobs == null) { jobs = new ArrayList<>(); byType.put(type, jobs); order[layer.ordinal()].add(type); }
                jobs.add(new Job(d, g));
            }
        }
        int quads = 0, types = 0;
        for (Map.Entry<RenderType, List<PartGpu.Job>> e : onGpu.entrySet()) {
            types++;
            if (PartGpu.draw(e.getKey(), e.getValue())) {
                for (PartGpu.Job j : e.getValue()) { quads += j.mesh().groups().get(j.group()).count(); }
            } else {   // the shader cannot take a pose (a pack replaced it): written as before
                VertexConsumer out = buffers.getBuffer(e.getKey());
                for (PartGpu.Job j : e.getValue()) {
                    quads += emit(out, new Draw(j.mesh(), j.pose(), j.light(), 0xffffffff, false, ALL, false, false, false), j.mesh().groups().get(j.group()));
                }
            }
        }
        for (List<RenderType> layerTypes : order) {
            for (RenderType type : layerTypes) {
                VertexConsumer out = buffers.getBuffer(type);
                types++;
                for (Job j : byType.get(type)) { quads += emit(out, j.draw(), j.group()); }
            }
        }
        draws.clear();
        if (late) { LATE_LAMPS.addAll(lamps); }   // halos and beams after the world's translucent blocks
        lamps.clear();
        PartStats.drawn(quads, types, System.nanoTime() - t0);
    }

    /** 0.30: draws the late translucent groups (PartLate, after the world's translucent blocks) and forgets them. */
    static void flushLate(MultiBufferSource.BufferSource buffers) {
        if (!LATE_LAMPS.isEmpty()) { LampRenderer.draw(buffers, LATE_LAMPS); LATE_LAMPS.clear(); }
        if (LATE.isEmpty()) { return; }
        long t0 = System.nanoTime();
        int quads = 0, types = 0;
        for (Map.Entry<RenderType, List<Job>> e : LATE.entrySet()) {
            VertexConsumer out = buffers.getBuffer(e.getKey());
            types++;
            for (Job j : e.getValue()) { quads += emit(out, j.draw(), j.group()); }
            buffers.endBatch(e.getKey());
        }
        LATE.clear();
        PartStats.drawn(quads, types, System.nanoTime() - t0);
    }

    /** 0.30: forgets late groups not drawn (a frame whose late stage did not come: a screen over the world). */
    static void dropLate() { LATE.clear(); LATE_LAMPS.clear(); }

    @SuppressWarnings("unchecked")
    private static List<RenderType>[] orderLists() {
        List<RenderType>[] out = new List[PartRenderTypes.Layer.values().length];
        for (int i = 0; i < out.length; i++) { out[i] = new ArrayList<>(); }
        return out;
    }

    private static int emit(VertexConsumer out, Draw d, PackedMesh.Group g) {
        PackedMesh m = d.mesh();
        float[] p = m.positions(), n = m.normals(), uv = m.uvs();
        int[] c = m.colors();
        int tint = d.tint();
        float tr = ((tint >> 16) & 255) / 255f, tg = ((tint >> 8) & 255) / 255f, tb = (tint & 255) / 255f, ta = ((tint >>> 24) & 255) / 255f;
        // the pose applied here (the consumer's pose overloads make two vectors a vertex)
        org.joml.Matrix4f mp = d.pose().pose();
        org.joml.Matrix3f mn = d.pose().normal();
        float a00 = mp.m00(), a01 = mp.m01(), a02 = mp.m02(), a10 = mp.m10(), a11 = mp.m11(), a12 = mp.m12(), a20 = mp.m20(), a21 = mp.m21(), a22 = mp.m22();
        float a30 = mp.m30(), a31 = mp.m31(), a32 = mp.m32();
        float b00 = mn.m00(), b01 = mn.m01(), b02 = mn.m02(), b10 = mn.m10(), b11 = mn.m11(), b12 = mn.m12(), b20 = mn.m20(), b21 = mn.m21(), b22 = mn.m22();
        int light = d.light();
        boolean glow = d.glow();
        for (int v = 4 * g.first(), end = 4 * (g.first() + g.count()); v < end; v++) {
            int col = c[v];
            float shade = glow ? 1 : m.light(v) / 255f;
            int r = (int) (((col >> 16) & 255) * tr * shade + 0.5f), gg = (int) (((col >> 8) & 255) * tg * shade + 0.5f), b = (int) ((col & 255) * tb * shade + 0.5f);
            int a = (int) (((col >>> 24) & 255) * ta + 0.5f);
            if (glow) { r = r * a / 255; gg = gg * a / 255; b = b * a / 255; }   // added light: its alpha is its strength
            float x = p[3 * v], y = p[3 * v + 1], z = p[3 * v + 2];
            float nx = n[3 * v], ny = n[3 * v + 1], nz = n[3 * v + 2];
            float tx = b00 * nx + b10 * ny + b20 * nz, ty = b01 * nx + b11 * ny + b21 * nz, tz = b02 * nx + b12 * ny + b22 * nz;
            float len = (float) Math.sqrt(tx * tx + ty * ty + tz * tz);
            if (len > 1e-12f) { tx /= len; ty /= len; tz /= len; }
            out.addVertex(a00 * x + a10 * y + a20 * z + a30, a01 * x + a11 * y + a21 * z + a31, a02 * x + a12 * y + a22 * z + a32)
                    .setColor(r, gg, b, a).setUv(uv[2 * v], uv[2 * v + 1]).setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(tx, ty, tz);
        }
        return g.count();
    }

    /** Vertices written by old code, in the frame they were written in (the consumer's defaults transform by the pose first). */
    private static final class Recorder implements VertexConsumer {
        private static final int FLOATS = 8, INTS = 3;
        float[] f = new float[FLOATS * 256];   // x y z u v nx ny nz
        int[] i = new int[INTS * 256];         // colour, overlay, light
        int count;

        @Override public VertexConsumer addVertex(float x, float y, float z) {
            if (FLOATS * (count + 1) > f.length) { f = Arrays.copyOf(f, f.length * 2); i = Arrays.copyOf(i, i.length * 2); }
            int a = FLOATS * count, b = INTS * count;
            count++;
            f[a] = x; f[a + 1] = y; f[a + 2] = z; f[a + 3] = 0; f[a + 4] = 0; f[a + 5] = 0; f[a + 6] = 1; f[a + 7] = 0;
            i[b] = 0xffffffff; i[b + 1] = OverlayTexture.NO_OVERLAY; i[b + 2] = 0xf000f0;
            return this;
        }

        @Override public VertexConsumer setColor(int r, int g, int b, int a) {
            i[INTS * (count - 1)] = (a & 255) << 24 | (r & 255) << 16 | (g & 255) << 8 | (b & 255);
            return this;
        }

        @Override public VertexConsumer setUv(float u, float v) { int a = FLOATS * (count - 1); f[a + 3] = u; f[a + 4] = v; return this; }

        @Override public VertexConsumer setUv1(int u, int v) { i[INTS * (count - 1) + 1] = (v & 0xffff) << 16 | (u & 0xffff); return this; }

        @Override public VertexConsumer setUv2(int u, int v) { i[INTS * (count - 1) + 2] = (v & 0xffff) << 16 | (u & 0xffff); return this; }

        @Override public VertexConsumer setNormal(float x, float y, float z) { int a = FLOATS * (count - 1); f[a + 5] = x; f[a + 6] = y; f[a + 7] = z; return this; }

        void replay(VertexConsumer out) {
            for (int k = 0; k < count; k++) {
                int a = FLOATS * k, b = INTS * k;
                int overlay = i[b + 1], light = i[b + 2];
                out.addVertex(f[a], f[a + 1], f[a + 2]).setColor(i[b]).setUv(f[a + 3], f[a + 4])
                        .setUv1(overlay & 0xffff, overlay >>> 16).setUv2(light & 0xffff, light >>> 16).setNormal(f[a + 5], f[a + 6], f[a + 7]);
            }
        }
    }
}
