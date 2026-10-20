package dev.mcmod.aerocore.neoforge.client;

import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.mesh.AmbientOcclusion;
import dev.mcmod.aerocore.api.mesh.Finish;
import dev.mcmod.aerocore.api.mesh.MeshBaker;
import dev.mcmod.aerocore.api.mesh.MeshBuilder;
import dev.mcmod.aerocore.api.mesh.PackedMesh;
import net.minecraft.Util;
import org.slf4j.Logger;

import java.util.Map;
import java.util.WeakHashMap;
import java.util.function.Supplier;

/**
 * Bakes part meshes on a background thread and keeps them (0.29): a renderer asks by a key it holds (a compiled
 * part, a design screen's model), gets null until the bake is done (a frame or a few), then the levels of detail.
 * Keys are held weakly, so a mesh goes when what it was baked for does. Building the mesh is part of the job, so model
 * code runs off the render thread too; it must not touch the world or GL. Cleared on resource reload.
 */
public final class MeshBakery {
    private MeshBakery() {}

    private static final Logger LOGGER = LogUtils.getLogger();

    /**
     * How to bake: the finish for quads without one, ambient occlusion or not, levels 0 to last, and whether enclosed
     * and interior quads are left out (a part drawn whole) or kept (a cutaway); 0.30: the largest occlusion voxel
     * (metres, 0: by the part's size), finer for a large part whose small openings matter (a fuselage's windows).
     */
    public record Spec(Finish defaultFinish, boolean occlusion, int lastLevel, boolean cull, double maxVoxelM) {
        public static final Spec WORLD = new Spec(null, true, 2, true), PREVIEW = new Spec(null, true, 0, false);

        public Spec(Finish defaultFinish, boolean occlusion, int lastLevel, boolean cull) { this(defaultFinish, occlusion, lastLevel, cull, 0); }

        /** This spec with a largest occlusion voxel. */
        public Spec voxel(double metres) { return new Spec(defaultFinish, occlusion, lastLevel, cull, metres); }
    }

    /**
     * A bake's result: levels (0 nearest) and the radius of the sphere about the box centre holding the mesh; 0.30: by
     * piece too ([piece][level], piece 0 the fixed part, which is what levels holds), for a mesh with moving pieces.
     */
    public record Baked(PackedMesh[] levels, Vec3d centre, double radius, PackedMesh[][] pieces, java.util.List<dev.mcmod.aerocore.api.mesh.Lamp> lamps) {
        public Baked(PackedMesh[] levels, Vec3d centre, double radius) { this(levels, centre, radius, new PackedMesh[][]{levels}, java.util.List.of()); }

        public Baked(PackedMesh[] levels, Vec3d centre, double radius, PackedMesh[][] pieces) { this(levels, centre, radius, pieces, java.util.List.of()); }

        public PackedMesh level(int i) { return levels[Math.min(i, levels.length - 1)]; }

        /** How many pieces (1: only the fixed part). */
        public int pieceCount() { return pieces.length; }

        /** A piece at a level (an empty mesh past the pieces). */
        public PackedMesh piece(int piece, int level) {
            if (piece < 0 || piece >= pieces.length) { return PackedMesh.EMPTY; }
            PackedMesh[] l = pieces[piece];
            return l[Math.min(level, l.length - 1)];
        }
    }

    private static final class Entry {
        volatile Baked baked;
        volatile boolean failed;
        final int generation;
        /** 0.39: the bake of the generation before, drawn until this one is done (refresh: nothing vanishes meanwhile). */
        volatile Baked previous;
        Entry(int generation) { this.generation = generation; }

        Baked shown() { Baked b = baked; return b != null ? b : previous; }
    }

    private static final Map<Object, Entry> CACHE = new WeakHashMap<>();
    /** Development switches (/aerocoredev parts): bake without occlusion, or without leaving out what cannot be seen. */
    static volatile boolean noOcclusion, noCulling;
    private static volatile int generation;

    /**
     * The baked mesh for a key, or null while it is being baked (the first call starts it). The builder runs on a
     * background thread; a builder that throws gives an empty mesh (and one log line).
     */
    public static Baked get(Object key, Supplier<? extends MeshBuilder<?>> build, Spec spec) {
        Entry e;
        synchronized (CACHE) {
            e = CACHE.get(key);
            if (e == null || e.generation != generation) {
                Entry fresh = new Entry(generation);
                if (e != null) { fresh.previous = e.shown(); }
                CACHE.put(key, fresh);
                PartStats.queued(1);
                Util.backgroundExecutor().execute(() -> bake(fresh, build, spec));
                e = fresh;
            }
        }
        return e.shown();
    }

    /** The baked mesh for a key if it is done, without starting anything. */
    public static Baked peek(Object key) {
        synchronized (CACHE) { Entry e = CACHE.get(key); return e == null ? null : e.shown(); }
    }

    /** Bakes now on this thread (a small mesh, or a test): no cache. */
    public static Baked bakeNow(MeshBuilder<?> mesh, Spec spec) { return bakeMesh(mesh, spec); }

    private static void bake(Entry e, Supplier<? extends MeshBuilder<?>> build, Spec spec) {
        long t0 = System.nanoTime();
        try {
            MeshBuilder<?> mesh = build.get();
            e.baked = bakeMesh(mesh, spec);
            e.previous = null;
            if (!net.neoforged.fml.loading.FMLEnvironment.production && !mesh.quads().isEmpty()) {
                // development: what the bake kept and how dark it came out (a check on occlusion and culling)
                PackedMesh l0 = e.baked.levels()[0];
                long sum = 0;
                for (int v = 0; v < 4 * l0.quads(); v++) { sum += l0.light(v); }
                LOGGER.info("Part mesh baked: {} quads, level 0 keeps {}, last level {}, mean brightness {}, {} ms", mesh.quads().size(), l0.quads(),
                        e.baked.levels()[e.baked.levels().length - 1].quads(), String.format("%.2f", l0.quads() == 0 ? 0 : sum / (255.0 * 4 * l0.quads())),
                        (System.nanoTime() - t0) / 1_000_000);
            }
        } catch (RuntimeException ex) {
            LOGGER.warn("A part mesh could not be built; drawn empty", ex);
            e.failed = true;
            e.baked = new Baked(new PackedMesh[]{PackedMesh.EMPTY}, Vec3d.ZERO, 0);
        } finally {
            PartStats.queued(-1);
            PartStats.baked(System.nanoTime() - t0);
        }
    }

    private static Baked bakeMesh(MeshBuilder<?> mesh, Spec spec) {
        Vec3d[] b = mesh.bounds();
        if (b == null) { return new Baked(new PackedMesh[]{PackedMesh.EMPTY}, Vec3d.ZERO, 0); }
        Vec3d centre = b[0].add(b[1]).scale(0.5);
        double radius = b[1].subtract(b[0]).length() / 2;
        double extent = Math.max(b[1].x() - b[0].x(), Math.max(b[1].y() - b[0].y(), b[1].z() - b[0].z()));
        AmbientOcclusion.Settings ao = spec.occlusion() && !noOcclusion ? AmbientOcclusion.Settings.forSize(extent) : null;
        if (ao != null && spec.maxVoxelM() > 0 && ao.voxelM() > spec.maxVoxelM()) {
            // finer than its size would give (0.30): a cabin seen through windows a few voxels wide; the grid may then
            // hold more cells (bits; up to 24 million, 6 MB)
            Vec3d e = b[1].subtract(b[0]);
            double v = spec.maxVoxelM(), cells = (e.x() / v + 5) * (e.y() / v + 5) * (e.z() / v + 5);
            ao = new AmbientOcclusion.Settings(v, (int) Math.min(24_000_000, Math.max(ao.maxCells(), cells)), ao.rays(), ao.radiusM(), ao.strength(), ao.floor());
        }
        // 0.39: beautified, a face is also split where its middle is lighter or darker than its corners
        if (ao != null && Beautify.on()) { ao = ao.withRefine(AmbientOcclusion.Refine.CENTRE); }
        boolean cull = spec.cull() && !noCulling;
        java.util.List<dev.mcmod.aerocore.api.mesh.Lamp> lamps = java.util.List.copyOf(mesh.lamps());
        if (MeshBaker.hasPieces(mesh)) {
            PackedMesh[][] pieces = MeshBaker.pieces(mesh, spec.defaultFinish(), ao, spec.lastLevel(), cull);
            return new Baked(pieces[0], centre, radius, pieces, lamps);
        }
        PackedMesh[] levels = MeshBaker.levels(mesh, spec.defaultFinish(), ao, spec.lastLevel(), cull);
        return new Baked(levels, centre, radius, new PackedMesh[][]{levels}, lamps);
    }

    /**
     * 0.39: bakes every key again when next asked, drawing its old bake until the new one is done (beautified shading
     * switched: the occlusion and the meshes content mods build differ). Unlike clear, nothing disappears meanwhile.
     */
    public static void refresh() {
        synchronized (CACHE) { generation++; }
    }

    /** Forgets every bake (resources reloaded: finishes may have changed); keys bake again when next asked. */
    public static void clear() {
        synchronized (CACHE) { generation++; CACHE.clear(); }
        PartGpu.clear();   // their buffers too
    }
}
