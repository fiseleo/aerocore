package dev.mcmod.aerocore.api.mesh;

/** A plain mesh (0.29): built, read from OBJ, worked on with {@link MeshOps} and baked, without any drawing. */
public final class Mesh extends MeshBuilder<Mesh> {
    @Override protected Mesh create() { return new Mesh(); }

    /** A mesh holding copies of another's quads. */
    public static Mesh of(MeshBuilder<?> other) {
        Mesh m = new Mesh();
        m.quads().addAll(other.quads());
        m.lamps().addAll(other.lamps());
        return m;
    }
}
