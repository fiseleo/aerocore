package dev.mcmod.aerocore.api.ground;

import dev.mcmod.aerocore.api.earthmoving.Material;

/**
 * 0.43: what the ground is made of, cell by cell (the game's 1 m blocks, by the integer coordinates of their low
 * corner), for tools that dig it (a bucket's lip, a blade, a ripper, a cutter drum). Wheels and tracks stand on Ground;
 * a tool asks Terrain what it is cutting and how hard that is. The game builds it from its blocks (which block is
 * which material is its rule); offline tests use below() or grid().
 */
public interface Terrain {
    /** The material in the cell (x, y, z), or null where there is none (air, water, a plant). */
    Cell cell(int x, int y, int z);

    /**
     * One cell: its id (the game's block), its material (api.earthmoving: density, strength, swell), whether a tool may
     * dig it (false: bedrock, a machine, a chest: a tool meeting it is stopped as by a wall) and how much of it is left
     * (0..1 of the cell: partly dug).
     */
    record Cell(String id, Material material, boolean diggable, double remaining) {
        public Cell {
            java.util.Objects.requireNonNull(id, "id"); java.util.Objects.requireNonNull(material, "material");
            if (!(remaining > 0 && remaining <= 1)) { throw new IllegalArgumentException("A cell has some of itself left, at most all"); }
        }
        /** A whole cell of a material a tool may dig. */
        public static Cell of(String id, Material material) { return new Cell(id, material, true, 1); }
        /** The same cell with less of it left. */
        public Cell remaining(double share) { return new Cell(id, material, diggable, Math.clamp(share, 1e-9, 1)); }
    }

    /** No ground to dig. */
    Terrain NONE = (x, y, z) -> null;

    /** Everything whose cells lie at or below a level y (their tops at or under it) is one cell kind: a flat offline site. */
    static Terrain below(double y, Cell cell) {
        return (x, cy, z) -> cy + 1 <= y + 1e-9 ? cell : null;
    }

    /** A grid of cells (an offline site with trenches, mounds and walls): the cells given by key, nothing elsewhere. */
    static Terrain grid(java.util.Map<Long, Cell> cells) {
        return (x, y, z) -> cells.get(key(x, y, z));
    }

    /** A cell's key for grid (21 bits a coordinate, offset so negatives fit). */
    static long key(int x, int y, int z) {
        return ((long) (x + (1 << 20)) << 42) | ((long) (y + (1 << 20)) << 21) | (long) (z + (1 << 20));
    }
}
