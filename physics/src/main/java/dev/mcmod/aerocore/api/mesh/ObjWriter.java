package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Writes meshes as Wavefront OBJ with a material library (0.29), for looking at a game model in Blender (reference
 * renders) or any other tool. One object per named mesh; one material per finish and colour, named
 * {@code <finish>_<rrggbb>} ({@code plain} where there is no finish), with the colour as its diffuse colour and the
 * alpha as its dissolve. Texture coordinates are written in repeats of the finish's tile, so a tiling texture on the
 * material lands at the game's scale. Triangles (degenerate quads) are written as triangles.
 */
public final class ObjWriter {
    private ObjWriter() {}

    /** The OBJ text and its material library's text. */
    public record Files(String obj, String mtl) {}

    public static Files write(Map<String, ? extends MeshBuilder<?>> meshes, String mtlName) {
        StringBuilder obj = new StringBuilder(), mtl = new StringBuilder();
        obj.append("# AeroCore mesh export\nmtllib ").append(mtlName).append('\n');
        Map<Vec3d, Integer> positions = new HashMap<>(), normals = new HashMap<>();
        Map<Long, Integer> uvs = new HashMap<>();
        Map<String, int[]> materials = new LinkedHashMap<>();
        int[] counts = {0, 0, 0};
        for (Map.Entry<String, ? extends MeshBuilder<?>> e : meshes.entrySet()) {
            obj.append("o ").append(e.getKey().isBlank() ? "mesh" : e.getKey().replace(' ', '_')).append('\n');
            String current = null;
            for (MeshBuilder.Quad q : e.getValue().quads()) {
                String material = materialName(q);
                materials.putIfAbsent(material, new int[]{q.color()});
                if (!material.equals(current)) { obj.append("usemtl ").append(material).append('\n'); current = material; }
                double tile = q.finish() == null ? Finish.DEFAULT_TILE_M : q.finish().tileM();
                int corners = q.d().equals(q.c()) ? 3 : 4;
                StringBuilder face = new StringBuilder("f");
                float[] uv = q.uv() == null ? MeshBuilder.planar(q.normal(), q.a(), q.b(), q.c(), q.d()) : q.uv();
                for (int i = 0; i < corners; i++) {
                    int p = index(positions, q.corner(i), obj, "v", counts, 0);
                    float u = (float) (uv[2 * i] / tile), v = (float) (uv[2 * i + 1] / tile);
                    long key = ((long) Float.floatToIntBits(u) << 32) | (Float.floatToIntBits(v) & 0xffffffffL);
                    Integer t = uvs.get(key);
                    if (t == null) { t = ++counts[1]; uvs.put(key, t); obj.append(String.format(Locale.ROOT, "vt %.6f %.6f%n", u, v)); }
                    int n = index(normals, q.normalAt(i), obj, "vn", counts, 2);
                    face.append(' ').append(p).append('/').append(t).append('/').append(n);
                }
                obj.append(face).append('\n');
            }
        }
        for (Map.Entry<String, int[]> m : materials.entrySet()) {
            int c = m.getValue()[0];
            mtl.append("newmtl ").append(m.getKey()).append('\n');
            mtl.append(String.format(Locale.ROOT, "Kd %.4f %.4f %.4f%n", ((c >> 16) & 255) / 255.0, ((c >> 8) & 255) / 255.0, (c & 255) / 255.0));
            mtl.append("Ka 0 0 0\n");
            mtl.append(String.format(Locale.ROOT, "d %.4f%n", ((c >>> 24) & 255) / 255.0));
        }
        return new Files(obj.toString(), mtl.toString());
    }

    /** The material a quad is written with. */
    public static String materialName(MeshBuilder.Quad q) {
        return (q.finish() == null ? "plain" : q.finish().id().replace(':', '_').replace('/', '_')) + String.format(Locale.ROOT, "_%06x", q.color() & 0xffffff);
    }

    private static int index(Map<Vec3d, Integer> seen, Vec3d v, StringBuilder obj, String tag, int[] counts, int slot) {
        Integer i = seen.get(v);
        if (i != null) { return i; }
        i = ++counts[slot];
        seen.put(v, i);
        obj.append(String.format(Locale.ROOT, "%s %.6f %.6f %.6f%n", tag, v.x(), v.y(), v.z()));
        return i;
    }
}
