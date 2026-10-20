package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.AirData;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.VehicleBlueprint;
import dev.mcmod.aerocore.neoforge.probe.BodyWorld;
import dev.mcmod.aerocore.neoforge.probe.CoupledBody;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.levelgen.Heightmap;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * The vehicles near a point and where they are going (0.38): what a radar, a traffic display or a control tower in
 * a vehicle mod reads. Only vehicles in loaded chunks exist on the server; a client knows those its player tracks
 * (about 160 m; FarSounds sends louder ones from farther).
 */
public final class VehicleTracks {
    private VehicleTracks() {}

    /**
     * One vehicle: its entity's id and number, centre of mass and velocity (world, m and m/s), attitude (body to world;
     * the vehicle's nose is +X), the ground's height under it (top of the motion-blocking heightmap without leaves;
     * NaN where its chunk is not loaded), whether its contact spheres touched anything in the last tick (server; never
     * on a client), whether its tables are built, its owner and pilot (null without), its blueprint and mass (NaN on a
     * client that has not compiled it).
     */
    public record Track(UUID id, int entityId, Vec3d position, Vec3d velocity, Orientation orientation, double groundY,
                        boolean touching, boolean ready, UUID owner, UUID pilot, VehicleBlueprint blueprint, double massKg) {
        /** Where the nose points (unit). */
        public Vec3d nose() { return orientation.rotate(Vec3d.X); }

        /** The nose's compass heading (degrees, north = -Z). */
        public double headingDeg() { Vec3d n = nose(); return AirData.compassDeg(n.x(), n.z()); }

        /** The compass track over the ground (degrees), or the heading when it barely moves. */
        public double trackDeg() {
            return groundSpeedMps() < 0.5 ? headingDeg() : AirData.compassDeg(velocity.x(), velocity.z());
        }

        public double groundSpeedMps() { return Math.hypot(velocity.x(), velocity.z()); }

        public double verticalSpeedMps() { return velocity.y(); }

        /** Height of the centre of mass above the ground under it (m; NaN where unknown). */
        public double heightAboveGroundM() { return position.y() - groundY; }

        /** The air's velocity relative to the vehicle given the wind there (m/s): its true airspeed is the length. */
        public Vec3d airVelocity(Vec3d wind) { return velocity.subtract(wind); }
    }

    /** A vehicle's track now (either side). */
    public static Track of(PartVehicle v) {
        Level level = v.level();
        double ground = groundBelow(level, v.getX(), v.getY(), v.getZ());
        Entity pilot = v.getFirstPassenger();
        double mass = v.vehicle() == null ? Double.NaN : v.vehicle().properties().massKg();
        return new Track(v.getUUID(), v.getId(), new Vec3d(v.getX(), v.getY(), v.getZ()), v.velocity(), v.poseOrientation(), ground,
                !level.isClientSide && v.touching(), v.ready(), v.owner(), pilot == null ? null : pilot.getUUID(), v.blueprint(), mass);
    }

    /**
     * The top of the ground under a point (m): the motion-blocking heightmap (without leaves) where nothing is over the
     * point, else the first block below it that blocks motion or holds a fluid (inside a hangar, under a bridge or a
     * GameTest's barrier roof), at the top of its collision shape. NaN where the chunk is not loaded or nothing is below
     * within 512 blocks. Never loads a chunk.
     */
    public static double groundBelow(Level level, double x, double y, double z) {
        int bx = net.minecraft.util.Mth.floor(x), bz = net.minecraft.util.Mth.floor(z), by = net.minecraft.util.Mth.floor(y);
        if (!level.hasChunk(bx >> 4, bz >> 4)) { return Double.NaN; }
        int top = level.getHeight(Heightmap.Types.MOTION_BLOCKING_NO_LEAVES, bx, bz);
        if (top <= by) { return top; }
        net.minecraft.core.BlockPos.MutableBlockPos p = new net.minecraft.core.BlockPos.MutableBlockPos(bx, by, bz);
        for (int i = 0; i < 512 && p.getY() >= level.getMinBuildHeight(); i++, p.move(0, -1, 0)) {
            net.minecraft.world.level.block.state.BlockState s = level.getBlockState(p);
            if (s.blocksMotion() || !s.getFluidState().isEmpty()) {
                net.minecraft.world.phys.shapes.VoxelShape shape = s.getCollisionShape(level, p);
                double height = shape.isEmpty() ? s.getFluidState().getHeight(level, p) : shape.max(net.minecraft.core.Direction.Axis.Y);
                if (p.getY() + height <= y) { return p.getY() + height; }
            }
        }
        return Double.NaN;
    }

    /**
     * The vehicles within a horizontal radius (m) of a point on the server, nearest first: from the level's bodies
     * (BodyWorld), not a scan of all entities. Removed vehicles and ones without a blueprint are left out.
     */
    public static List<Track> near(ServerLevel level, Vec3d centre, double radiusM) {
        List<PartVehicle> found = new ArrayList<>();
        double r2 = radiusM * radiusM;
        for (CoupledBody b : BodyWorld.bodies(level)) {
            if (!(b instanceof PartVehicle v) || v.isRemoved() || v.blueprint() == null) { continue; }
            double dx = v.getX() - centre.x(), dz = v.getZ() - centre.z();
            if (dx * dx + dz * dz <= r2) { found.add(v); }
        }
        found.sort(java.util.Comparator.comparingDouble(v -> Math.hypot(v.getX() - centre.x(), v.getZ() - centre.z())));
        List<Track> out = new ArrayList<>(found.size());
        for (PartVehicle v : found) { out.add(of(v)); }
        return out;
    }
}
