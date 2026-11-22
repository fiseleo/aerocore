package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.ground.Terrain;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * 0.43: the world's blocks as Terrain for vehicles' tools (Part.tools). Which block is what material is a vehicle
 * mod's rule: it registers a resolver (BlockState and position to a Terrain.Cell, or null for none); with none
 * registered nothing is diggable ground and tools find nothing. Blocks in chunks that are not loaded are nothing (it
 * never loads a chunk). Cells are kept until clear() (PartVehicle clears them every tick, as WorldGround's).
 */
public final class WorldTerrain implements Terrain {
    /** What a block is to a tool: its material, or null (air, water, a plant). */
    @FunctionalInterface
    public interface Resolver { Terrain.Cell resolve(Level level, BlockPos pos, BlockState state); }

    private static volatile Resolver resolver;

    /** A vehicle mod's rule for what blocks are (the last registered wins). */
    public static void register(Resolver r) { resolver = Objects.requireNonNull(r, "resolver"); }
    /** Whether a mod has said what blocks are. */
    public static boolean active() { return resolver != null; }

    private final Level level;
    private final Map<Long, Terrain.Cell> cells = new HashMap<>();
    private static final Terrain.Cell NONE = new Terrain.Cell("", dev.mcmod.aerocore.api.earthmoving.Material.LOAM, false, 1);

    public WorldTerrain(Level level) { this.level = level; }

    /** Forgets the cells seen so far (the blocks may have changed). */
    public void clear() { cells.clear(); }

    @Override public Terrain.Cell cell(int x, int y, int z) {
        Resolver r = resolver;
        if (r == null) { return null; }
        long key = BlockPos.asLong(x, y, z);
        Terrain.Cell c = cells.get(key);
        if (c == null) {
            BlockPos pos = new BlockPos(x, y, z);
            c = level.isLoaded(pos) ? r.resolve(level, pos, level.getBlockState(pos)) : null;
            cells.put(key, c == null ? NONE : c);
            return c;
        }
        return c == NONE ? null : c;
    }
}
