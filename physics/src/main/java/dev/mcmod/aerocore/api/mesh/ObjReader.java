package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Vec3d;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StringReader;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;

/**
 * Reads Wavefront OBJ text (0.29) into quads: detail parts and models made in Blender. Positions {@code v}, texture
 * coordinates {@code vt}, normals {@code vn}, faces {@code f} with 1-based or negative (relative) indices, materials
 * {@code usemtl} and groups {@code o}/{@code g}. Triangles become degenerate quads; faces with more corners are
 * fanned into quads (and counted, since some loaders take only four). Material names map to a finish and a colour.
 * Texture coordinates are kept as written (u, v) times a scale (an asset's metres per unit of its UV map).
 */
public final class ObjReader {
    private ObjReader() {}

    /** What a material name means: the finish its faces get, and their vertex colour. */
    public record Material(Finish finish, int color) {}

    /** The faces read, all together and by group, with what was found in the file. */
    public record Result(Mesh mesh, Map<String, Mesh> groups, Set<String> materials, int polygons, boolean vertexColours, boolean missingMaterial) {
        /** One group's mesh (empty if there is none of that name). */
        public Mesh group(String name) { Mesh m = groups.get(name); return m == null ? new Mesh() : m; }
    }

    /** Material names as finishes of a tile, white. */
    public static Function<String, Material> plain(double tileM) { return name -> new Material(new Finish(name, tileM), 0xffffffff); }

    public static Result read(String text, Function<String, Material> materials, double uvScale) {
        return read(new StringReader(text), materials, uvScale);
    }

    public static Result read(Reader source, Function<String, Material> materials, double uvScale) {
        List<Vec3d> positions = new ArrayList<>(), normals = new ArrayList<>();
        List<float[]> uvs = new ArrayList<>();
        Mesh all = new Mesh();
        Map<String, Mesh> groups = new LinkedHashMap<>();
        Set<String> used = new LinkedHashSet<>();
        Material material = null;
        String group = "";
        int polygons = 0;
        boolean colours = false, missing = false;
        Map<String, Material> cache = new LinkedHashMap<>();
        try (BufferedReader in = new BufferedReader(source)) {
            String line;
            int number = 0;
            while ((line = in.readLine()) != null) {
                number++;
                line = line.strip();
                if (line.isEmpty() || line.startsWith("#")) { continue; }
                String[] t = line.split("\\s+");
                switch (t[0]) {
                    case "v" -> {
                        positions.add(new Vec3d(num(t, 1, number), num(t, 2, number), num(t, 3, number)));
                        if (t.length >= 7) { colours = true; }
                    }
                    case "vt" -> uvs.add(new float[]{(float) (num(t, 1, number) * uvScale), (float) ((t.length > 2 ? num(t, 2, number) : 0) * uvScale)});
                    case "vn" -> normals.add(MeshBuilder.unit(new Vec3d(num(t, 1, number), num(t, 2, number), num(t, 3, number))));
                    case "usemtl" -> {
                        String name = t.length > 1 ? line.substring(line.indexOf(' ') + 1).strip() : "";
                        used.add(name);
                        material = cache.computeIfAbsent(name, materials);
                    }
                    case "o", "g" -> group = t.length > 1 ? line.substring(line.indexOf(' ') + 1).strip() : "";
                    case "f" -> {
                        int n = t.length - 1;
                        if (n < 3) { throw new IllegalArgumentException("OBJ line " + number + ": a face needs three corners"); }
                        if (n > 4) { polygons++; }
                        if (material == null) { missing = true; }
                        Vec3d[] p = new Vec3d[n], nn = new Vec3d[n];
                        float[][] tex = new float[n][];
                        boolean haveNormals = true, haveUv = true;
                        for (int i = 0; i < n; i++) {
                            String[] idx = t[i + 1].split("/", -1);
                            p[i] = positions.get(index(idx[0], positions.size(), number));
                            if (idx.length > 1 && !idx[1].isEmpty()) { tex[i] = uvs.get(index(idx[1], uvs.size(), number)); } else { haveUv = false; }
                            if (idx.length > 2 && !idx[2].isEmpty()) { nn[i] = normals.get(index(idx[2], normals.size(), number)); } else { haveNormals = false; }
                        }
                        Mesh target = groups.computeIfAbsent(group, k -> new Mesh());
                        // a fan of quads from the first corner (the last of an odd count is a triangle)
                        for (int i = 1; i + 1 < n; i += 2) {
                            int k = Math.min(i + 2, n - 1);
                            int[] ix = {0, i, i + 1, k};
                            Vec3d[] cp = new Vec3d[4], cn = haveNormals ? new Vec3d[4] : null;
                            float[] uv = haveUv ? new float[8] : null;
                            for (int c = 0; c < 4; c++) {
                                cp[c] = p[ix[c]];
                                if (cn != null) { cn[c] = nn[ix[c]]; }
                                if (uv != null) { uv[2 * c] = tex[ix[c]][0]; uv[2 * c + 1] = tex[ix[c]][1]; }
                            }
                            Vec3d face = cp[2].subtract(cp[0]).cross(cp[3].subtract(cp[1]));
                            if (face.lengthSquared() < 1e-30) { face = cp[1].subtract(cp[0]).cross(cp[2].subtract(cp[0])); }
                            // the face normal follows the winding (counter-clockwise seen from outside); a file whose
                            // normals disagree is an asset to fix, which the asset tests look for
                            Vec3d unit = MeshBuilder.unit(face);
                            int color = material == null ? 0xffffffff : material.color();
                            Finish finish = material == null ? null : material.finish();
                            MeshBuilder.Quad q = new MeshBuilder.Quad(cp[0], cp[1], cp[2], cp[3], color, unit, cn, null, uv, finish, 0);
                            target.quads().add(q);
                            all.quads().add(q);
                        }
                    }
                    default -> { }   // mtllib, s, l and the rest: not needed
                }
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return new Result(all, Collections.unmodifiableMap(groups), Collections.unmodifiableSet(used), polygons, colours, missing);
    }

    private static double num(String[] t, int i, int line) {
        if (i >= t.length) { throw new IllegalArgumentException("OBJ line " + line + ": missing a number"); }
        try { return Double.parseDouble(t[i]); }
        catch (NumberFormatException e) { throw new IllegalArgumentException("OBJ line " + line + ": not a number: " + t[i]); }
    }

    private static int index(String s, int count, int line) {
        int i;
        try { i = Integer.parseInt(s); }
        catch (NumberFormatException e) { throw new IllegalArgumentException("OBJ line " + line + ": not an index: " + s); }
        int at = i > 0 ? i - 1 : count + i;
        if (i == 0 || at < 0 || at >= count) { throw new IllegalArgumentException("OBJ line " + line + ": index out of range: " + s); }
        return at;
    }
}
