package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.neoforge.probe.GroundSurfaces;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.Vec3;
import net.minecraft.world.phys.shapes.CollisionContext;
import net.minecraft.world.phys.shapes.VoxelShape;

/**
 * The world's blocks as ground for wheels (0.24): rays walk the block cells they cross (Amanatides and Woo) and clip
 * each block's collision shape, so slabs, stairs, snow layers, carpets and multi-box shapes count as they are; a cell
 * also tries the block below it (fences and walls stand 1.5 m tall). The surface is GroundSurfaces' (wet in rain).
 * Blocks in chunks that are not loaded are no ground (never loads a chunk). Shapes and surfaces are kept until clear(),
 * which the owner calls once a tick (blocks may change between ticks, not within one).
 */
public final class WorldGround implements Ground {
    private record Cell(VoxelShape shape, GroundContact.Surface surface) {}
    private static final Cell EMPTY = new Cell(null, null);

    private final Level level;
    private final Long2ObjectOpenHashMap<Cell> cells = new Long2ObjectOpenHashMap<>();

    public WorldGround(Level level) { this.level = level; }

    /** Forgets the blocks read so far (call once a tick). */
    public void clear() { cells.clear(); }

    @Override public Hit cast(Vec3d from, Vec3d direction, double maxM) {
        if (!(maxM > 0) || !Double.isFinite(maxM)) { return null; }
        Vec3 start = new Vec3(from.x(), from.y(), from.z()), end = new Vec3(from.x() + direction.x() * maxM, from.y() + direction.y() * maxM,
                from.z() + direction.z() * maxM);
        int x = (int) Math.floor(from.x()), y = (int) Math.floor(from.y()), z = (int) Math.floor(from.z());
        double[] o = {from.x(), from.y(), from.z()}, d = {direction.x(), direction.y(), direction.z()};
        int[] cell = {x, y, z}, step = new int[3];
        double[] next = new double[3], delta = new double[3];
        for (int k = 0; k < 3; k++) {
            if (d[k] > 0) { step[k] = 1; next[k] = (cell[k] + 1 - o[k]) / d[k]; delta[k] = 1 / d[k]; }
            else if (d[k] < 0) { step[k] = -1; next[k] = (o[k] - cell[k]) / -d[k]; delta[k] = -1 / d[k]; }
            else { next[k] = Double.POSITIVE_INFINITY; delta[k] = Double.POSITIVE_INFINITY; }
        }
        Hit best = null;
        // at most the cells a segment of length maxM can cross, plus slack
        int limit = 3 * ((int) Math.ceil(maxM) + 2);
        for (int visited = 0; visited < limit; visited++) {
            for (int below = 0; below <= 1; below++) {
                Hit hit = clip(cell[0], cell[1] - below, cell[2], start, end, from);
                if (hit != null && (best == null || hit.distanceM() < best.distanceM())) { best = hit; }
            }
            int axis = next[0] <= next[1] ? (next[0] <= next[2] ? 0 : 2) : (next[1] <= next[2] ? 1 : 2);
            double entry = next[axis];
            if (entry > maxM || (best != null && best.distanceM() <= entry)) { break; }
            cell[axis] += step[axis];
            next[axis] += delta[axis];
        }
        return best;
    }

    private Hit clip(int x, int y, int z, Vec3 start, Vec3 end, Vec3d from) {
        Cell c = cell(x, y, z);
        if (c.shape() == null) { return null; }
        BlockHitResult r = c.shape().clip(start, end, new BlockPos(x, y, z));
        if (r == null) { return null; }
        Vec3d normal = new Vec3d(r.getDirection().getStepX(), r.getDirection().getStepY(), r.getDirection().getStepZ());
        if (r.isInside()) { return new Hit(from, normal, 0, c.surface()); }
        Vec3 at = r.getLocation();
        Vec3d point = new Vec3d(at.x, at.y, at.z);
        return new Hit(point, normal, point.subtract(from).length(), c.surface());
    }

    private Cell cell(int x, int y, int z) {
        long key = BlockPos.asLong(x, y, z);
        Cell c = cells.get(key);
        if (c != null) { return c; }
        BlockPos pos = new BlockPos(x, y, z);
        if (!level.hasChunkAt(pos)) {
            c = EMPTY;
        } else {
            BlockState state = level.getBlockState(pos);
            VoxelShape shape = state.isAir() ? null : state.getCollisionShape(level, pos, CollisionContext.empty());
            c = shape == null || shape.isEmpty() ? EMPTY : new Cell(shape, GroundSurfaces.of(level, pos));
        }
        cells.put(key, c);
        return c;
    }
}
