package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import net.minecraft.core.BlockPos;
import net.minecraft.util.Mth;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.border.WorldBorder;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.CollisionContext;
import net.minecraft.world.phys.shapes.VoxelShape;

import java.util.ArrayList;
import java.util.List;

/** Static contact boxes around an entity: block and entity collision shapes plus the world border. */
public final class WorldObstacles {
    private WorldObstacles() {}

    /**
     * Obstacles are one-way: the solver pushes the moving body, never these blocks or entities. Block boxes carry
     * their block's material (BlockMaterials); entity boxes use the body's default.
     */
    public static List<WorldBox> collect(Entity self, AABB region) {
        List<WorldBox> boxes = new ArrayList<>();
        Level level = self.level();
        CollisionContext context = CollisionContext.of(self);
        for (BlockPos pos : BlockPos.betweenClosed(Mth.floor(region.minX) - 1, Mth.floor(region.minY) - 1, Mth.floor(region.minZ) - 1,
                Mth.floor(region.maxX) + 1, Mth.floor(region.maxY) + 1, Mth.floor(region.maxZ) + 1)) {
            BlockState state = level.getBlockState(pos);
            if (state.isAir()) { continue; }
            VoxelShape shape = state.getCollisionShape(level, pos, context);
            if (shape.isEmpty()) { continue; }
            ContactMaterial material = BlockMaterials.of(state);
            for (AABB box : shape.move(pos.getX(), pos.getY(), pos.getZ()).toAabbs()) {
                if (box.intersects(region)) { boxes.add(new WorldBox(new Vec3d(box.minX, box.minY, box.minZ), new Vec3d(box.maxX, box.maxY, box.maxZ), material)); }
            }
        }
        for (VoxelShape shape : self.level().getEntityCollisions(self, region)) { for (AABB box : shape.toAabbs()) { boxes.add(box(box)); } }
        WorldBorder border = self.level().getWorldBorder();
        double x0 = region.minX - 1, y0 = region.minY - 1, z0 = region.minZ - 1, x1 = region.maxX + 1, y1 = region.maxY + 1, z1 = region.maxZ + 1;
        // Finite boxes stand in for the outside of the world border near this region.
        if (x0 < border.getMinX()) { boxes.add(new WorldBox(new Vec3d(x0, y0, z0), new Vec3d(border.getMinX(), y1, z1))); }
        if (x1 > border.getMaxX()) { boxes.add(new WorldBox(new Vec3d(border.getMaxX(), y0, z0), new Vec3d(x1, y1, z1))); }
        if (z0 < border.getMinZ()) { boxes.add(new WorldBox(new Vec3d(x0, y0, z0), new Vec3d(x1, y1, border.getMinZ()))); }
        if (z1 > border.getMaxZ()) { boxes.add(new WorldBox(new Vec3d(x0, y0, border.getMaxZ()), new Vec3d(x1, y1, z1))); }
        return boxes;
    }

    /**
     * Obstacles along a fast body's path (0.28): the blocks in each box of the corridor (each block once), skipping the
     * part of a box above the terrain (the highest motion-blocking block of its columns: nothing to hit there, so a
     * vehicle high over the ground reads no blocks however fast it goes), then entities and the world border as collect.
     */
    public static List<WorldBox> collect(Entity self, List<AABB> corridor) {
        List<WorldBox> boxes = new ArrayList<>();
        Level level = self.level();
        CollisionContext context = CollisionContext.of(self);
        java.util.Set<Long> seen = new java.util.HashSet<>();
        for (AABB region : corridor) {
            int x0 = Mth.floor(region.minX) - 1, z0 = Mth.floor(region.minZ) - 1, x1 = Mth.floor(region.maxX) + 1, z1 = Mth.floor(region.maxZ) + 1;
            // (columns of chunks that are not loaded are skipped: reading them would generate them here)
            int top = level.getMinBuildHeight();
            for (int x = x0; x <= x1; x++) for (int z = z0; z <= z1; z++) {
                if (!level.hasChunk(x >> 4, z >> 4)) { continue; }
                top = Math.max(top, level.getHeight(net.minecraft.world.level.levelgen.Heightmap.Types.MOTION_BLOCKING, x, z));
            }
            int y0 = Mth.floor(region.minY) - 1, y1 = Math.min(Mth.floor(region.maxY) + 1, top);
            // wholly above the terrain: no blocks (and betweenClosed with y1 < y0 would count round the int range)
            for (BlockPos pos : y1 < y0 ? List.<BlockPos>of() : BlockPos.betweenClosed(x0, y0, z0, x1, y1, z1)) {
                if (!level.hasChunk(pos.getX() >> 4, pos.getZ() >> 4) || !seen.add(pos.asLong())) { continue; }
                BlockState state = level.getBlockState(pos);
                if (state.isAir()) { continue; }
                VoxelShape shape = state.getCollisionShape(level, pos, context);
                if (shape.isEmpty()) { continue; }
                ContactMaterial material = BlockMaterials.of(state);
                for (AABB box : shape.move(pos.getX(), pos.getY(), pos.getZ()).toAabbs()) {
                    if (box.intersects(region)) { boxes.add(new WorldBox(new Vec3d(box.minX, box.minY, box.minZ), new Vec3d(box.maxX, box.maxY, box.maxZ), material)); }
                }
            }
            for (VoxelShape shape : level.getEntityCollisions(self, region)) { for (AABB box : shape.toAabbs()) { boxes.add(box(box)); } }
        }
        AABB all = corridor.getFirst();
        for (AABB r : corridor) { all = all.minmax(r); }
        WorldBorder border = level.getWorldBorder();
        double bx0 = all.minX - 1, by0 = all.minY - 1, bz0 = all.minZ - 1, bx1 = all.maxX + 1, by1 = all.maxY + 1, bz1 = all.maxZ + 1;
        if (bx0 < border.getMinX()) { boxes.add(new WorldBox(new Vec3d(bx0, by0, bz0), new Vec3d(border.getMinX(), by1, bz1))); }
        if (bx1 > border.getMaxX()) { boxes.add(new WorldBox(new Vec3d(border.getMaxX(), by0, bz0), new Vec3d(bx1, by1, bz1))); }
        if (bz0 < border.getMinZ()) { boxes.add(new WorldBox(new Vec3d(bx0, by0, bz0), new Vec3d(bx1, by1, border.getMinZ()))); }
        if (bz1 > border.getMaxZ()) { boxes.add(new WorldBox(new Vec3d(bx0, by0, border.getMaxZ()), new Vec3d(bx1, by1, bz1))); }
        return boxes;
    }

    /** Whether inner lies wholly inside one of the boxes. */
    public static boolean contains(List<AABB> outer, AABB inner) {
        for (AABB o : outer) { if (contains(o, inner)) { return true; } }
        return false;
    }

    public static boolean contains(AABB outer, AABB inner) {
        return inner.minX >= outer.minX && inner.minY >= outer.minY && inner.minZ >= outer.minZ
                && inner.maxX <= outer.maxX && inner.maxY <= outer.maxY && inner.maxZ <= outer.maxZ;
    }

    private static WorldBox box(AABB b) { return new WorldBox(new Vec3d(b.minX, b.minY, b.minZ), new Vec3d(b.maxX, b.maxY, b.maxZ)); }
}
