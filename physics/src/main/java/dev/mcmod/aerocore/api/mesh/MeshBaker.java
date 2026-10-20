package dev.mcmod.aerocore.api.mesh;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Bakes meshes for drawing (0.29): quads without a finish get a default one, quads without texture coordinates are
 * projected on their most facing plane, coordinates are divided by the finish's tile, ambient occlusion is applied,
 * and the quads are sorted into draw groups (pass, finish, hot). Levels of detail are baked from one occlusion pass
 * over the whole mesh (interiors included, so they still shade what is outside), each level leaving out what its
 * tags (and, where asked, enclosure) say it should.
 */
public final class MeshBaker {
    private MeshBaker() {}

    /** Bakes all of a mesh's quads, without occlusion. */
    public static PackedMesh bake(MeshBuilder<?> mesh, Finish defaultFinish) {
        return bake(mesh.quads(), null, null, defaultFinish);
    }

    /**
     * Bakes the quads a keep-list allows (null: all), with their corners' brightness from an occlusion result over the
     * same list (null: open).
     */
    public static PackedMesh bake(List<MeshBuilder.Quad> quads, boolean[] keep, AmbientOcclusion.Result ao, Finish defaultFinish) {
        List<Integer> order = new ArrayList<>();
        for (int i = 0; i < quads.size(); i++) { if (keep == null || keep[i]) { order.add(i); } }
        Comparator<Integer> byGroup = Comparator.comparingInt((Integer i) -> pass(quads.get(i)).ordinal())
                .thenComparing(i -> finishOf(quads.get(i), defaultFinish) == null ? "" : finishOf(quads.get(i), defaultFinish).id())
                .thenComparingInt(i -> quads.get(i).has(MeshBuilder.HOT) ? 1 : 0)
                .thenComparingInt(i -> quads.get(i).lamp());
        order.sort(byGroup);   // stable: the quads keep their order within a group
        int n = order.size();
        float[] position = new float[12 * n], normal = new float[12 * n], uv = new float[8 * n];
        int[] color = new int[4 * n];
        byte[] light = new byte[4 * n];
        Arrays.fill(light, (byte) 255);
        List<PackedMesh.Group> groups = new ArrayList<>();
        int start = 0;
        for (int at = 0; at < n; at++) {
            int qi = order.get(at);
            MeshBuilder.Quad q = quads.get(qi);
            Finish f = finishOf(q, defaultFinish);
            double tile = f == null ? Finish.DEFAULT_TILE_M : f.tileM();
            float[] t = q.uv() == null ? MeshBuilder.planar(q.normal(), q.a(), q.b(), q.c(), q.d()) : q.uv();
            for (int c = 0; c < 4; c++) {
                int v = 4 * at + c;
                position[3 * v] = (float) q.corner(c).x(); position[3 * v + 1] = (float) q.corner(c).y(); position[3 * v + 2] = (float) q.corner(c).z();
                normal[3 * v] = (float) q.normalAt(c).x(); normal[3 * v + 1] = (float) q.normalAt(c).y(); normal[3 * v + 2] = (float) q.normalAt(c).z();
                uv[2 * v] = (float) (t[2 * c] / tile); uv[2 * v + 1] = (float) (t[2 * c + 1] / tile);
                color[v] = q.colorAt(c);
                if (ao != null) { light[v] = (byte) Math.round(255 * Math.max(0, Math.min(1, ao.at(qi, c)))); }
            }
            boolean last = at == n - 1 || byGroup.compare(qi, order.get(at + 1)) != 0;
            if (last) {
                groups.add(new PackedMesh.Group(f, pass(q), q.has(MeshBuilder.HOT), start, at + 1 - start, q.lamp()));
                start = at + 1;
            }
        }
        return new PackedMesh(position, normal, uv, color, light, groups);
    }

    /** An occlusion result with its shade scaled toward open (0: none, 1: as it was). */
    static AmbientOcclusion.Result softened(AmbientOcclusion.Result r, double keep) {
        float[] c = r.corner().clone();
        for (int i = 0; i < c.length; i++) { c[i] = (float) (1 - keep * (1 - c[i])); }
        return new AmbientOcclusion.Result(r.quads(), c, r.enclosed());
    }

    private static Finish finishOf(MeshBuilder.Quad q, Finish fallback) { return q.finish() == null ? fallback : q.finish(); }

    private static PackedMesh.Pass pass(MeshBuilder.Quad q) {
        return q.translucent() ? PackedMesh.Pass.TRANSLUCENT : q.has(MeshBuilder.DECAL) ? PackedMesh.Pass.DECAL : PackedMesh.Pass.SOLID;
    }

    /**
     * Levels of detail 0 (nearest) to a last level, from one occlusion pass (null settings: no occlusion). Each level
     * keeps quads whose detail plus the level is at most 3; with cull, quads tagged interior or found enclosed are
     * left out of every level (a part seen whole in the world), else they are kept (a design screen's cutaway). A mesh
     * with stand-ins (MeshBuilder.STAND_IN) and two or more levels draws them alone at its last level, and never
     * elsewhere. 0.30: a quad is drawn from its nearest level (MeshBuilder.NEAR) on; a mesh with such far variants is
     * shaded twice (its nearest level, its farther ones), so the variants do not hide each other; cabin quads are
     * never left out as enclosed at level 0 and are left out farther, where glazing is drawn opaque and darker.
     */
    public static PackedMesh[] levels(MeshBuilder<?> mesh, Finish defaultFinish, AmbientOcclusion.Settings occlusion, int lastLevel, boolean cull) {
        return baked(mesh, defaultFinish, occlusion, lastLevel, cull, false)[0];
    }

    /**
     * 0.30: as {@link #levels}, by piece (MeshBuilder.piece: 0 the fixed part, then the moving ones), [piece][level]:
     * one occlusion over all of them as built (their rest pose), so a door shades its jamb and a wheel its arch as when
     * at rest. A renderer draws each piece under its own pose.
     */
    public static PackedMesh[][] pieces(MeshBuilder<?> mesh, Finish defaultFinish, AmbientOcclusion.Settings occlusion, int lastLevel, boolean cull) {
        return baked(mesh, defaultFinish, occlusion, lastLevel, cull, true);
    }

    /** Whether any quad of a mesh belongs to a moving piece (MeshBuilder.piece above 0). */
    public static boolean hasPieces(MeshBuilder<?> mesh) {
        for (MeshBuilder.Quad q : mesh.quads()) { if (q.piece() > 0) { return true; } }
        return false;
    }

    private static PackedMesh[][] baked(MeshBuilder<?> mesh, Finish defaultFinish, AmbientOcclusion.Settings occlusion, int lastLevel, boolean cull, boolean byPiece) {
        // level 0 takes the refined occlusion (quads split where shade changes much, so it stays where it is cast);
        // farther levels the mesh's own quads, their shade halved (a long quad's corner shade spreads along it, which
        // shows less far away and costs no extra quads)
        // stand-ins apart: they wrap the whole, so with it the outside air would reach nothing and all would be enclosed
        List<MeshBuilder.Quad> main = new ArrayList<>(), standIns = new ArrayList<>();
        int pieces = 1;
        boolean far = false, cabin = false;
        for (MeshBuilder.Quad q : mesh.quads()) {
            (q.has(MeshBuilder.STAND_IN) ? standIns : main).add(q);
            if (byPiece) { pieces = Math.max(pieces, q.piece() + 1); }
            far |= q.near() > 0;
            cabin |= q.has(MeshBuilder.CABIN);
        }
        // 0.30: a mesh with far variants (quads drawn only from level 1, such as a hull drawn uncut) shades its nearest
        // level and its farther ones apart: drawn together, the two would cover each other and both be left out
        List<MeshBuilder.Quad> nearSet = main, farSet = main;
        if (far) {
            nearSet = new ArrayList<>(); farSet = new ArrayList<>();
            for (MeshBuilder.Quad q : main) {
                if (q.near() == 0) { nearSet.add(q); }
                if (q.near() > 0 || q.detail() + 1 <= MeshBuilder.DETAIL) { farSet.add(q); }
            }
        }
        AmbientOcclusion.Both near = occlusion == null || nearSet.isEmpty() ? null : AmbientOcclusion.both(nearSet, occlusion);
        AmbientOcclusion.Result farAo = occlusion == null || farSet.isEmpty() ? null : !far ? (near == null ? null : near.coarse()) : AmbientOcclusion.compute(farSet, occlusion);
        AmbientOcclusion.Result soft = farAo == null ? null : softened(farAo, 0.5);
        PackedMesh[][] out = new PackedMesh[pieces][lastLevel + 1];
        for (int level = 0; level <= lastLevel; level++) {
            if (!standIns.isEmpty() && lastLevel >= 2 && level == lastLevel) {
                boolean[] all = new boolean[standIns.size()];
                Arrays.fill(all, true);
                out[0][level] = bake(standIns, all, null, defaultFinish);
                for (int p = 1; p < pieces; p++) { out[p][level] = PackedMesh.EMPTY; }
                continue;
            }
            AmbientOcclusion.Result ao = level == 0 ? (near == null ? null : near.fine()) : soft;
            List<MeshBuilder.Quad> quads = ao != null ? ao.quads() : level == 0 ? nearSet : farSet;
            if (level > 0 && cabin) { quads = shut(quads); }
            boolean[] keep = new boolean[quads.size()];
            for (int i = 0; i < keep.length; i++) {
                MeshBuilder.Quad q = quads.get(i);
                boolean inLevel = q.near() <= level && q.detail() + level <= MeshBuilder.DETAIL && !(level > 0 && q.has(MeshBuilder.CABIN));
                boolean hidden = cull && (q.has(MeshBuilder.INTERIOR) || (ao != null && ao.enclosed()[i] && !(level == 0 && q.has(MeshBuilder.CABIN))));
                keep[i] = inLevel && !hidden;
            }
            for (int p = 0; p < pieces; p++) {
                boolean[] mine = keep;
                if (pieces > 1) {
                    mine = new boolean[keep.length];
                    for (int i = 0; i < keep.length; i++) { mine[i] = keep[i] && quads.get(i).piece() == p; }
                }
                out[p][level] = bake(quads, mine, ao, defaultFinish);
            }
        }
        return out;
    }

    /**
     * Glazing drawn opaque and darker (0.30), for the levels where a cabin is left out: the window then reads as glass
     * over a dark inside, and draws with the solid quads (cached on the graphics card) instead of the translucent pass.
     */
    private static List<MeshBuilder.Quad> shut(List<MeshBuilder.Quad> quads) {
        List<MeshBuilder.Quad> out = new ArrayList<>(quads.size());
        for (MeshBuilder.Quad q : quads) {
            if (!q.has(MeshBuilder.CLEAR) || !q.translucent()) { out.add(q); continue; }
            int[] colors = null;
            if (q.colors() != null) { colors = new int[4]; for (int c = 0; c < 4; c++) { colors[c] = shutColour(q.colors()[c]); } }
            out.add(new MeshBuilder.Quad(q.a(), q.b(), q.c(), q.d(), shutColour(q.color()), q.normal(), q.normals(), colors, q.uv(), q.finish(), q.tags()));
        }
        return out;
    }

    /** A glazing colour made opaque, at about half its brightness (what is behind dark glass, seen from afar). */
    private static int shutColour(int argb) {
        int r = ((argb >>> 16) & 255) * 11 / 20, g = ((argb >>> 8) & 255) * 11 / 20, b = (argb & 255) * 11 / 20;
        return 0xff000000 | (r << 16) | (g << 8) | b;
    }
}
