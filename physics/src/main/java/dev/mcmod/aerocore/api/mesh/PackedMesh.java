package dev.mcmod.aerocore.api.mesh;

import java.util.List;

/**
 * A baked mesh ready to draw (0.29): flat arrays, four vertices per quad, quads sorted into groups that draw with one
 * texture in one pass, so a renderer asks for each buffer once. Positions (metres, the part's frame), normals,
 * texture coordinates in repeats of the finish's tile, ARGB colours, and a brightness byte per vertex (ambient
 * occlusion, 255 = open). Immutable by convention: the arrays are shared, not copied; do not write to them.
 * About 37 bytes a vertex.
 */
public final class PackedMesh {
    /** How a group is drawn: opaque, see-through (a cutaway's ghost), or lying on another surface (depth offset). */
    public enum Pass { SOLID, TRANSLUCENT, DECAL }

    /**
     * Quads [first, first + count) with one finish (null: the drawing's default), drawn in one pass; hot ones glow too;
     * 0.30: a lamp's lens (its number, -1 none) is a group of its own, lit by the part's lamp rules.
     */
    public record Group(Finish finish, Pass pass, boolean hot, int first, int count, int lamp) {
        public Group(Finish finish, Pass pass, boolean hot, int first, int count) { this(finish, pass, hot, first, count, -1); }
    }

    public static final PackedMesh EMPTY = new PackedMesh(new float[0], new float[0], new float[0], new int[0], new byte[0], List.of());

    private final float[] position, normal, uv;
    private final int[] color;
    private final byte[] light;
    private final List<Group> groups;

    PackedMesh(float[] position, float[] normal, float[] uv, int[] color, byte[] light, List<Group> groups) {
        this.position = position; this.normal = normal; this.uv = uv; this.color = color; this.light = light;
        this.groups = List.copyOf(groups);
    }

    public int quads() { return color.length / 4; }

    public List<Group> groups() { return groups; }

    /** Vertex v's coordinates: x at 3v, y at 3v + 1, z at 3v + 2 (v = 4 quad + corner). */
    public float[] positions() { return position; }

    public float[] normals() { return normal; }

    /** Vertex v's u at 2v, v at 2v + 1, in tile repeats. */
    public float[] uvs() { return uv; }

    public int[] colors() { return color; }

    /** Ambient brightness of vertex v, 0-255. */
    public int light(int v) { return light[v] & 255; }

    public boolean isEmpty() { return color.length == 0; }

    /** The box around every vertex (min x, y, z, max x, y, z); zeros when empty. */
    public float[] bounds() {
        if (isEmpty()) { return new float[6]; }
        float[] b = {Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE};
        for (int i = 0; i < position.length; i += 3) {
            for (int a = 0; a < 3; a++) { b[a] = Math.min(b[a], position[i + a]); b[3 + a] = Math.max(b[3 + a], position[i + a]); }
        }
        return b;
    }
}
