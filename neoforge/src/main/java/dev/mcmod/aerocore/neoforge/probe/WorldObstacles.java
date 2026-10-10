package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.border.WorldBorder;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;

import java.util.ArrayList;
import java.util.List;

/** Static contact boxes around an entity: block and entity collision shapes plus the world border. */
final class WorldObstacles {
    private WorldObstacles() {}

    /** Obstacles are one-way: the solver pushes the moving body, never these blocks or entities. */
    static List<WorldBox> collect(Entity self, AABB region) {
        List<WorldBox> boxes = new ArrayList<>();
        for (VoxelShape shape : self.level().getBlockCollisions(self, region)) { for (AABB box : shape.toAabbs()) { boxes.add(box(box)); } }
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

    static boolean contains(AABB outer, AABB inner) {
        return inner.minX >= outer.minX && inner.minY >= outer.minY && inner.minZ >= outer.minZ
                && inner.maxX <= outer.maxX && inner.maxY <= outer.maxY && inner.maxZ <= outer.maxZ;
    }

    private static WorldBox box(AABB b) { return new WorldBox(new Vec3d(b.minX, b.minY, b.minZ), new Vec3d(b.maxX, b.maxY, b.maxZ)); }
}
