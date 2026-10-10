package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.*;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.core.BlockPos;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.network.chat.Component;
import net.minecraft.network.syncher.EntityDataAccessor;
import net.minecraft.network.syncher.EntityDataSerializers;
import net.minecraft.network.syncher.SynchedEntityData;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.InteractionHand;
import net.minecraft.world.InteractionResult;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import net.neoforged.neoforge.fluids.FluidType;
import org.joml.Quaternionf;

import java.util.List;
import java.util.Locale;
import java.util.UUID;

/**
 * Server-authoritative rigid test sphere: weight, buoyancy, drag and added mass from FloatingBodyDynamics, and
 * sphere contact from ContactSolver, so it bounces, slides and rolls. A temporary experiment, not a vehicle.
 */
public final class PhysicsProbe extends Entity implements CoupledBody, SolidBody {
    /** Solid box for vanilla entities: a cube of half-size 0.35 m inside the sphere. */
    private static final List<WorldBox> CUBE = List.of(new WorldBox(new Vec3d(-0.35, -0.35, -0.35), new Vec3d(0.35, 0.35, 0.35)));
    private final EntityLoads entityLoads = new EntityLoads();
    public static final double RADIUS_M = 0.5;
    public static final double MIN_MASS_KG = 50, MAX_MASS_KG = 2000;
    public static final int LIFETIME_TICKS = 20 * 60 * 10;
    /** A steel shell on blocks taken as rock (ContactMaterial.STEEL_ON_ROCK): friction 0.4, Thornton restitution. */
    public static final ContactMaterial MATERIAL = ContactMaterial.STEEL_ON_ROCK;
    private static final List<BuoyantSphere> HULL = List.of(new BuoyantSphere(Vec3d.ZERO, RADIUS_M));
    private static final List<CollisionSphere> SHAPE = List.of(new CollisionSphere(Vec3d.ZERO, RADIUS_M));
    private static final int SUBSTEPS = 10;
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private static final EntityDataAccessor<Float> MASS = SynchedEntityData.defineId(PhysicsProbe.class, EntityDataSerializers.FLOAT);
    private static final EntityDataAccessor<Float> WET = SynchedEntityData.defineId(PhysicsProbe.class, EntityDataSerializers.FLOAT);
    private static final EntityDataAccessor<Quaternionf> ROTATION = SynchedEntityData.defineId(PhysicsProbe.class, EntityDataSerializers.QUATERNION);
    private RigidBodyState state = new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
    /** Heave wave-radiation memory of the floating sphere. */
    private final WaveRadiation.Memory radiation = new WaveRadiation.Memory(List.of(Vec3d.ZERO), MinecraftUnits.TICK_SECONDS / SUBSTEPS);
    private Vec3d windMps = Vec3d.ZERO, addedCurrentMps = Vec3d.ZERO;
    private Vec3d buoyancyN = Vec3d.ZERO, dragN = Vec3d.ZERO, contactN = Vec3d.ZERO;
    private UUID owner;
    private int lerpSteps;
    private Vec3 lerpTarget = Vec3.ZERO;
    private Orientation clientRotation = Orientation.IDENTITY, previousClientRotation = Orientation.IDENTITY;

    public PhysicsProbe(EntityType<? extends PhysicsProbe> type, Level level) {
        super(type, level);
        setNoGravity(true); // Weight is applied exactly once by FloatingBodyDynamics.
    }

    @Override protected void defineSynchedData(SynchedEntityData.Builder builder) {
        builder.define(MASS, 250f);
        builder.define(WET, 0f);
        builder.define(ROTATION, new Quaternionf());
    }

    public double massKg() { return entityData.get(MASS); }
    public double wetFraction() { return entityData.get(WET); }
    public Vec3d velocityMps() { return state.velocity(); }
    public Vec3d angularVelocity() { return state.angularVelocity(properties()); }
    public Vec3d contactForceN() { return contactN; }
    public Vec3d windMps() { return windMps; }
    public Vec3d addedCurrentMps() { return addedCurrentMps; }
    public UUID owner() { return owner; }
    public Vec3d centerM() { return new Vec3d(getX(), getY() + RADIUS_M, getZ()); }
    /** Uniform solid sphere: I = 2/5 m R^2 about the center. */
    public RigidBodyProperties properties() {
        double mass = massKg(), inertia = 0.4 * mass * RADIUS_M * RADIUS_M;
        return new RigidBodyProperties(mass, Vec3d.ZERO, new InertiaTensor(inertia, inertia, inertia, 0, 0, 0));
    }
    public Orientation visualOrientation(float partialTick) {
        return level().isClientSide ? previousClientRotation.interpolate(clientRotation, Math.clamp(partialTick, 0, 1)) : state.orientation();
    }

    public void configure(double massKg, UUID owner, Vec3d velocity) {
        if (!Double.isFinite(massKg) || massKg < MIN_MASS_KG || massKg > MAX_MASS_KG) {
            throw new IllegalArgumentException("Probe mass must be in [50, 2000] kg");
        }
        entityData.set(MASS, (float) massKg);
        this.owner = owner;
        setVelocityMps(velocity);
        setCustomName(Component.translatable("entity.aerocore.physics_probe.label", format(massKg)));
        setCustomNameVisible(true);
    }

    /** External test adjustment: sets translation only and keeps the spin. */
    public void setVelocityMps(Vec3d velocity) {
        state = new RigidBodyState(centerM(), velocity, state.orientation(), state.angularMomentum());
        setDeltaMovement(MinecraftUnits.toBlocksPerTick(velocity));
        hasImpulse = true;
    }

    public void setWindMps(Vec3d wind) { windMps = wind; }
    public void setAddedCurrentMps(Vec3d current) { addedCurrentMps = current; }

    @Override public void tick() {
        super.tick();
        if (level().isClientSide) {
            previousClientRotation = clientRotation;
            Quaternionf q = entityData.get(ROTATION);
            clientRotation = clientRotation.interpolate(new Orientation(q.w, q.x, q.y, q.z), 0.5);
            if (lerpSteps > 0) {
                setPos(position().lerp(lerpTarget, 1.0 / lerpSteps));
                lerpSteps--;
            }
            BodyCollisions.trackClient(this);
            BodyCollisions.pushOut(this);
            return;
        }
        // Physics runs in BodyWorld at the start of the level tick, jointly with the other bodies.
        if (!isRemoved() && (tickCount >= LIFETIME_TICKS || getY() < level().getMinBuildHeight() - 16)) { discard(); }
    }

    // ---- CoupledBody: one tick in SUBSTEPS phases, driven by BodyWorld ----
    private WorldWaterSampler sampler;
    private int seaLevel;
    private double startTime;
    private AABB region;
    private FloatingBodyDynamics.Collision collision;
    private WaveRegions.Local waves = WaveRegions.Local.CALM;
    private boolean moving;
    private FloatingBodyDynamics.Prepared pending;
    private List<AppliedForce> pendingRadiation = List.of();
    private Vec3d dragSum = Vec3d.ZERO, buoyancySum = Vec3d.ZERO, contactSum = Vec3d.ZERO;
    private int steps;

    @Override public Entity entity() { return this; }
    @Override public boolean beginTick(ServerLevel server) {
        if (isRemoved()) { return false; }
        RigidBodyProperties properties = properties();
        // The entity position is authoritative for teleports between ticks.
        state = new RigidBodyState(centerM(), state.velocity(), state.orientation(), state.angularMomentum());
        sampler = new WorldWaterSampler(server);
        seaLevel = server.getChunkSource().getGenerator().getSeaLevel();
        double travel = (state.velocity().length() + state.angularVelocity(properties).length() * RADIUS_M) * MinecraftUnits.TICK_SECONDS;
        region = getBoundingBox().inflate(2 * travel + 1);
        if (!server.hasChunksAt(BlockPos.containing(region.minX, region.minY, region.minZ), BlockPos.containing(region.maxX, region.maxY, region.maxZ))) {
            stop();
            publish();
            return false;
        }
        collision = new FloatingBodyDynamics.Collision(SHAPE, WorldObstacles.collect(this, region), MATERIAL);
        // The same world wave regions as the platform; the taper factor is taken at the sphere centre.
        waves = WaveRegions.of(server).at(state.position().x(), state.position().z());
        startTime = server.getGameTime() * MinecraftUnits.TICK_SECONDS;
        dragSum = Vec3d.ZERO; buoyancySum = Vec3d.ZERO; contactSum = Vec3d.ZERO; steps = 0; moving = true;
        entityLoads.beginTick();
        return true;
    }
    // ---- SolidBody ----
    @Override public Vec3d poseCenter() { return level().isClientSide ? centerM() : state.position(); }
    @Override public Orientation poseOrientation() { return level().isClientSide ? clientRotation : state.orientation(); }
    @Override public Vec3d centerOfMassLocal() { return Vec3d.ZERO; }
    @Override public List<WorldBox> solidBoxesLocal() { return CUBE; }
    @Override public Vec3d velocityAt(Vec3d point) {
        return level().isClientSide ? Vec3d.ZERO : state.velocity().add(state.angularVelocity(properties()).cross(point.subtract(state.position())));
    }
    @Override public void addEntityLoad(Vec3d localPoint, Vec3d worldForce) { if (!level().isClientSide) { entityLoads.add(localPoint, worldForce); } }
    @Override public boolean active() { return moving; }
    @Override public RigidBodyState physicsState() { return state; }
    @Override public List<CollisionSphere> collisionShape() { return SHAPE; }
    @Override public ContactMaterial contactMaterial() { return MATERIAL; }
    @Override public List<WorldBox> staticObstacles() { return collision.obstacles(); }
    @Override public AABB physicsEnvelope() {
        Vec3d c = state.position();
        return new AABB(c.x() - RADIUS_M, c.y() - RADIUS_M, c.z() - RADIUS_M, c.x() + RADIUS_M, c.y() + RADIUS_M, c.z() + RADIUS_M);
    }
    @Override public RigidBodyState prepareSubstep(int step) {
        double dt = MinecraftUnits.TICK_SECONDS / SUBSTEPS, time = startTime + step * dt;
        FloatingBodyDynamics.Environment fluids = (center, radius) -> {
            WorldWaterSampler.Contact c = sampler.sample(center, radius, waves.waves(), waves.stillWaterY(), time);
            FluidState air = Atmosphere.atAltitude(Math.clamp(center.y() - seaLevel, -500, 20000), windMps).fluid();
            FluidState water = FluidState.FRESH_WATER_20C.withVelocity(c.currentMps().add(addedCurrentMps));
            return new FloatingBodyDynamics.Contact(c.fraction(), c.wetCentroidOffset(), air, water,
                    c.pressureFraction(), c.pressureCentroidOffset(), c.accelerationMps2(), c.fraction() > 0 && c.fraction() < 1);
        };
        RigidBodyProperties properties = properties();
        double rho = FluidState.FRESH_WATER_20C.densityKgPerM3();
        pendingRadiation = radiation.loads(state, properties, HULL, Vec3d.Y, rho);
        List<AppliedForce> loads = new java.util.ArrayList<>(pendingRadiation);
        loads.addAll(radiation.diffractionLoads(state, properties, HULL, rho, waves.waves(), time));
        loads.addAll(entityLoads.forces(state, properties));
        pending = FloatingBodyDynamics.prepare(state, properties, HULL, fluids, GRAVITY, FloatingBodyDynamics.STANDARD_DRAG, loads, dt);
        return pending.damped();
    }
    @Override public void finishSubstep(int step, ContactSolver.BodyResult contact) {
        FloatingBodyDynamics.Result result = FloatingBodyDynamics.finish(pending, contact.state(), contact.wrench(), contact.forces(),
                contact.staticFrictionW() + contact.pairFrictionW(), collision);
        Vec3d center = result.state().position();
        AABB next = new AABB(center.x() - RADIUS_M, center.y() - RADIUS_M, center.z() - RADIUS_M,
                center.x() + RADIUS_M, center.y() + RADIUS_M, center.z() + RADIUS_M);
        if (!WorldObstacles.contains(region, next)) { stop(); moving = false; return; }
        state = result.state();
        radiation.record(state, properties(), HULL, Vec3d.Y, result.wetFractions());
        for (AppliedForce load : pendingRadiation) { dragSum = dragSum.add(load.force()); }
        for (AppliedForce force : result.forces()) {
            if (force.kind() == AppliedForce.Kind.BUOYANCY) { buoyancySum = buoyancySum.add(force.force()); }
        }
        dragSum = dragSum.add(result.averageDrag().force());
        contactSum = contactSum.add(result.contact().force());
        steps++;
    }
    @Override public void correctPose(RigidBodyState corrected) { state = corrected; }
    @Override public void endTick() {
        double n = Math.max(steps, 1), dt = MinecraftUnits.TICK_SECONDS / SUBSTEPS;
        dragN = dragSum.scale(1 / n);
        buoyancyN = buoyancySum.scale(1 / n);
        contactN = contactSum.scale(1 / n);
        setPos(state.position().x(), state.position().y() - RADIUS_M, state.position().z());
        entityData.set(WET, (float) sampler.sample(centerM(), RADIUS_M, waves.waves(), waves.stillWaterY(), startTime + steps * dt).fraction());
        publish();
        BodyCollisions.pushOut(this);
    }

    private void stop() { state = new RigidBodyState(state.position(), Vec3d.ZERO, state.orientation(), Vec3d.ZERO); }

    private void publish() {
        Orientation q = state.orientation();
        entityData.set(ROTATION, new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
        setDeltaMovement(MinecraftUnits.toBlocksPerTick(state.velocity()));
        hasImpulse = true;
    }

    @Override public void lerpTo(double x, double y, double z, float yaw, float pitch, int steps) {
        lerpTarget = new Vec3(x, y, z);
        lerpSteps = Math.max(1, steps);
        setRot(yaw, pitch);
    }

    @Override public boolean isPushedByFluid() { return false; }
    @Override public boolean isPushedByFluid(FluidType type) { return false; }
    @Override public boolean isPickable() { return !isRemoved(); }
    @Override protected MovementEmission getMovementEmission() { return MovementEmission.NONE; }

    @Override public InteractionResult interact(Player player, InteractionHand hand) {
        if (!level().isClientSide && hand == InteractionHand.MAIN_HAND) { player.sendSystemMessage(diagnostics()); }
        return InteractionResult.sidedSuccess(level().isClientSide);
    }

    public Component diagnostics() {
        return Component.translatable("command.aerocore.probe.info", getId(), format(massKg()),
                format(velocityMps().length()), format(velocityMps().y()), format(angularVelocity().length()), format(100 * wetFraction()),
                format(buoyancyN.y()), format(massKg() * Atmosphere.GRAVITY_MPS2), format(dragN.length()), format(contactN.length()));
    }

    @Override protected void addAdditionalSaveData(CompoundTag tag) {
        tag.putDouble("MassKg", massKg());
        if (owner != null) { tag.putUUID("Owner", owner); }
        writeVector(tag, "VelocityMps", velocityMps());
        writeVector(tag, "WindMps", windMps);
        writeVector(tag, "CurrentMps", addedCurrentMps);
    }

    @Override protected void readAdditionalSaveData(CompoundTag tag) {
        double mass = tag.contains("MassKg") ? tag.getDouble("MassKg") : 250;
        configure(Double.isFinite(mass) ? Math.clamp(mass, MIN_MASS_KG, MAX_MASS_KG) : 250,
                tag.hasUUID("Owner") ? tag.getUUID("Owner") : null, readVector(tag, "VelocityMps"));
        windMps = readVector(tag, "WindMps");
        addedCurrentMps = readVector(tag, "CurrentMps");
    }

    private static void writeVector(CompoundTag tag, String key, Vec3d vector) {
        CompoundTag value = new CompoundTag();
        value.putDouble("x", vector.x()); value.putDouble("y", vector.y()); value.putDouble("z", vector.z());
        tag.put(key, value);
    }

    private static Vec3d readVector(CompoundTag tag, String key) {
        CompoundTag value = tag.getCompound(key);
        double x = value.getDouble("x"), y = value.getDouble("y"), z = value.getDouble("z");
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z)
                ? new Vec3d(Math.clamp(x, -100, 100), Math.clamp(y, -100, 100), Math.clamp(z, -100, 100)) : Vec3d.ZERO;
    }

    private static String format(double value) { return String.format(Locale.ROOT, "%.2f", value); }
}
