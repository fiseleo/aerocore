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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.UUID;

/**
 * Planing demonstrator (TestPlaningBoat): Zarnick 2D+t loads (PlaningDynamics) with zero-speed heave radiation faded out as the transom ventilates,
 * B4-70 P/D 1.4 propeller and spade rudder, set by commands. Not drivable and not a vehicle. Stepped by BodyWorld like
 * the other bodies; it leaves a pressure-patch Kelvin wake and planing whisker spray.
 */
public final class PlaningBoat extends Entity implements DebugBody, CoupledBody, SolidBody, Wakes.Source {
    private static final List<WorldBox> STRIPS = strips();
    private static final EntityDataAccessor<Quaternionf> ROTATION = SynchedEntityData.defineId(PlaningBoat.class, EntityDataSerializers.QUATERNION);
    private static final EntityDataAccessor<Float> PROPELLER = SynchedEntityData.defineId(PlaningBoat.class, EntityDataSerializers.FLOAT);
    private static final EntityDataAccessor<Float> RUDDER = SynchedEntityData.defineId(PlaningBoat.class, EntityDataSerializers.FLOAT);
    private static final EntityDataAccessor<Float> WETTED = SynchedEntityData.defineId(PlaningBoat.class, EntityDataSerializers.FLOAT);
    private static final int SUBSTEPS = 10, RADIATION_SEGMENTS = 6;
    private static final List<Vec3d> RADIATION_POINTS = HullDynamics.radiationPoints(TestPlaningBoat.HULL.hull(), RADIATION_SEGMENTS);
    private final WaveRadiation.Memory radiation = new WaveRadiation.Memory(RADIATION_POINTS, MinecraftUnits.TICK_SECONDS / SUBSTEPS);
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private final EntityLoads entityLoads = new EntityLoads();
    private final RigidBodyProperties properties = TestPlaningBoat.properties();
    private RigidBodyState state = new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
    private final PlatformEnvironment environment = new PlatformEnvironment();
    private WaveRegions.Local waves = WaveRegions.Local.CALM;
    private double propellerRps, rudderRadians, thrustN, torqueNm, frictionN, surgeMps, wettedLength, ventilation, rudderLiftN, dragPowerW;
    private Wrench contactLoad = Wrench.ZERO;
    private int debugMode;
    private List<AppliedForce> displayForces = List.of();
    private Vec3d forceOrigin = Vec3d.ZERO;
    private UUID owner;
    private int lerpSteps;
    private Vec3 targetPosition = Vec3.ZERO;
    private Orientation clientRotation = Orientation.IDENTITY, previousClientRotation = Orientation.IDENTITY;

    public PlaningBoat(EntityType<? extends PlaningBoat> type, Level level) {
        super(type, level);
        setNoGravity(true);
    }
    @Override protected void defineSynchedData(SynchedEntityData.Builder builder) {
        builder.define(ROTATION, new Quaternionf());
        builder.define(PROPELLER, 0f);
        builder.define(RUDDER, 0f);
        builder.define(WETTED, 0f);
    }
    @Override public Entity entity() { return this; }
    @Override public UUID owner() { return owner; }
    @Override public int debugMode() { return debugMode; }
    public void setDebugMode(int mode) {
        if (mode < 0 || mode > 7) { throw new IllegalArgumentException("Invalid debug mode"); }
        debugMode = mode;
    }
    @Override public List<AppliedForce> displayForces() { return displayForces; }
    @Override public Vec3d forceOrigin() { return forceOrigin; }
    @Override public WaveRegions.Local waves() { return waves; }
    @Override public Vec3d modelOrigin() { return state.worldPoint(properties, Vec3d.ZERO); }
    @Override public RigidBodyState physicsState() { return state; }
    @Override public RigidBodyProperties properties() { return properties; }
    @Override public PlatformEnvironment environment() { return environment; }
    @Override public double bodyTemperatureCelsius() { return Double.NaN; }
    public double thrustN() { return thrustN; }
    public double surgeSpeedMps() { return surgeMps; }
    public double wettedLengthM() { return level().isClientSide ? entityData.get(WETTED) : wettedLength; }
    public double propellerRps() { return level().isClientSide ? entityData.get(PROPELLER) : propellerRps; }
    public double rudderRadians() { return level().isClientSide ? entityData.get(RUDDER) : rudderRadians; }
    public void setPropellerRps(double value) {
        if (!Double.isFinite(value) || Math.abs(value) > TestPlaningBoat.MAX_PROPELLER_RPS) { throw new IllegalArgumentException("Propeller speed outside [-45, 45] rev/s"); }
        propellerRps = value; publish();
    }
    public void setRudderDegrees(double degrees) {
        if (!Double.isFinite(degrees) || Math.abs(degrees) > 35) { throw new IllegalArgumentException("Rudder angle outside [-35, 35] deg"); }
        rudderRadians = Math.toRadians(degrees); publish();
    }
    public Orientation visualOrientation(float partialTick) {
        return level().isClientSide ? previousClientRotation.interpolate(clientRotation, Math.clamp(partialTick, 0, 1)) : state.orientation();
    }
    /** Places the transom-keel origin at the given point, upright and level with the given heading (radians about +y). */
    public void configure(UUID owner, Vec3d origin, double headingRadians) {
        this.owner = owner;
        state = TestPlaningBoat.atModelOrigin(origin, Orientation.rotationVector(Vec3d.Y.scale(headingRadians)));
        setCustomName(Component.translatable("entity.aerocore.planing_boat"));
        setCustomNameVisible(true);
        publish();
    }

    // ---- Wakes.Source: a pressure patch over the wetted bottom; whisker spray at the chines ----
    @Override public Wakes.Design wakeDesign() {
        Vec3d ahead = poseOrientation().rotate(Vec3d.X);
        double trim = Math.asin(Math.clamp(ahead.y(), -1, 1));
        return Wakes.planingBoat(TestPlaningBoat.BEAM, TestPlaningBoat.DEADRISE, wettedLengthM(), TestPlaningBoat.MASS * Atmosphere.GRAVITY_MPS2, trim);
    }
    @Override public Vec3d wakeOrigin() { return toWorld(Vec3d.ZERO); }
    @Override public Vec3d wakeForward() { return poseOrientation().rotate(Vec3d.X); }
    @Override public Vec3d wakeVelocity() {
        return level().isClientSide ? new Vec3d(getX() - xo, getY() - yo, getZ() - zo).scale(1 / MinecraftUnits.TICK_SECONDS) : state.velocity();
    }

    @Override public void tick() {
        super.tick();
        if (level().isClientSide) {
            previousClientRotation = clientRotation;
            Quaternionf q = entityData.get(ROTATION);
            clientRotation = clientRotation.interpolate(new Orientation(q.w, q.x, q.y, q.z), 0.5);
            if (lerpSteps > 0) { setPos(position().lerp(targetPosition, 1.0 / lerpSteps--)); }
            setBoundingBox(makeBoundingBox());
            BodyCollisions.trackClient(this);
            BodyCollisions.pushOut(this);
            return;
        }
        if (!isRemoved() && (tickCount >= PhysicsProbe.LIFETIME_TICKS || getY() < level().getMinBuildHeight() - 20)) { discard(); }
    }

    // ---- SolidBody ----
    @Override public Vec3d poseCenter() { return level().isClientSide ? new Vec3d(getX(), getY(), getZ()) : state.position(); }
    @Override public Orientation poseOrientation() { return level().isClientSide ? clientRotation : state.orientation(); }
    @Override public Vec3d centerOfMassLocal() { return TestPlaningBoat.CENTER_OF_MASS; }
    @Override public List<WorldBox> solidBoxesLocal() { return STRIPS; }
    @Override public Vec3d velocityAt(Vec3d point) {
        return level().isClientSide ? Vec3d.ZERO : state.velocity().add(state.angularVelocity(properties).cross(point.subtract(state.position())));
    }
    @Override public void addEntityLoad(Vec3d localPoint, Vec3d worldForce) { if (!level().isClientSide) { entityLoads.add(localPoint, worldForce); } }
    private static List<WorldBox> strips() {
        List<WorldBox> boxes = new ArrayList<>();
        SlenderHull hull = TestPlaningBoat.HULL.hull();
        for (int i = 0; i < hull.stations(); i += 4) {
            HullSection s = hull.sections().get(i);
            double back = TestPlaningBoat.HULL.boundary(i), front = TestPlaningBoat.HULL.boundary(Math.min(hull.stations(), i + 4));
            boxes.add(new WorldBox(new Vec3d(back, s.minY(), -s.maxHalfBreadth()), new Vec3d(front, TestPlaningBoat.DEPTH, s.maxHalfBreadth())));
        }
        return List.copyOf(boxes);
    }

    // ---- CoupledBody: one tick in SUBSTEPS phases, driven by BodyWorld ----
    private WorldWaterSampler sampler;
    private int seaLevel;
    private double startTime;
    private AABB region;
    private FloatingBodyDynamics.Collision collision;
    private boolean moving;
    private PlaningDynamics.Prepared pending;
    private double pendingThrust, pendingTorque, pendingRudder;
    private double thrustSum, torqueSum, frictionSum, surgeSum, wetSum, ventSum, rudderSum, powerSum;
    private Wrench contactSum = Wrench.ZERO;
    private int steps;

    @Override public boolean beginTick(ServerLevel server) {
        if (isRemoved()) { return false; }
        sampler = new WorldWaterSampler(server).excludingWakeOf(getId());
        seaLevel = server.getChunkSource().getGenerator().getSeaLevel();
        List<CollisionSphere> shape = TestPlaningBoat.collisionShape();
        double reach = 0;
        for (CollisionSphere s : shape) { reach = Math.max(reach, s.centerLocal().subtract(properties.centerOfMassLocal()).length() + s.radiusM()); }
        double travel = (state.velocity().length() + state.angularVelocity(properties).length() * reach) * MinecraftUnits.TICK_SECONDS;
        region = envelope(state.position(), state.orientation()).inflate(2 * travel + 1);
        if (!loaded(region)) { stop(); publish(); return false; }
        collision = new FloatingBodyDynamics.Collision(shape, WorldObstacles.collect(this, region), TestHull.CONTACT_MATERIAL);
        startTime = server.getGameTime() * MinecraftUnits.TICK_SECONDS;
        Vec3d origin = modelOrigin();
        waves = WaveRegions.of(server).at(origin.x(), origin.z());
        thrustSum = torqueSum = frictionSum = surgeSum = wetSum = ventSum = rudderSum = powerSum = 0;
        contactSum = Wrench.ZERO; steps = 0; moving = true;
        entityLoads.beginTick();
        return true;
    }
    @Override public boolean active() { return moving; }
    @Override public List<CollisionSphere> collisionShape() { return collision.shape(); }
    @Override public ContactMaterial contactMaterial() { return TestHull.CONTACT_MATERIAL; }
    @Override public List<WorldBox> staticObstacles() { return collision.obstacles(); }
    @Override public AABB physicsEnvelope() { return envelope(state.position(), state.orientation()); }
    @Override public RigidBodyState prepareSubstep(int i) {
        double dt = MinecraftUnits.TICK_SECONDS / SUBSTEPS, time = startTime + i * dt;
        HullDynamics.Environment water = point -> {
            WorldWaterSampler.Column c = sampler.column(point, waves.waves(), waves.stillWaterY(), time);
            FluidState air = environment.airAt(point, seaLevel).fluid();
            FluidState liquid = environment.waterAt(c.currentMps());
            return new HullDynamics.Water(c.surfaceY(), c.slope(), c.stillY(), c.decayWavenumber(), liquid.velocity(), c.accelerationMps2(), liquid, air);
        };
        double rho = environment.waterThermal().fluid().densityKgPerM3();
        List<AppliedForce> loads = new ArrayList<>();
        // Zero-speed heave radiation (Cummins memory) while the transom is wet; the 2D+t loads take over when planing.
        for (AppliedForce f : radiation.loadsAt(state, properties, RADIATION_POINTS, Vec3d.Y, rho)) {
            loads.add(new AppliedForce(f.kind(), f.offset(), f.force().scale(1 - ventilation), f.couple().scale(1 - ventilation)));
        }
        Vec3d hub = state.worldPoint(properties, TestPlaningBoat.PROPELLER.positionLocal());
        WorldWaterSampler.Contact disk = sampler.sample(hub, TestPlaningBoat.PROPELLER.diameterM() / 2, waves.waves(), waves.stillWaterY(), time);
        PropellerSolver.Load screw = PropellerSolver.load(state, properties, TestPlaningBoat.PROPELLER, propellerRps, environment.waterAt(disk.currentMps()));
        double immersion = disk.fraction();
        if (immersion > 0 && (screw.force().force().lengthSquared() > 0 || screw.force().couple().lengthSquared() > 0)) {
            loads.add(new AppliedForce(AppliedForce.Kind.THRUST, screw.force().offset(), screw.force().force().scale(immersion), screw.force().couple().scale(immersion)));
        }
        Vec3d rudderPoint = state.worldPoint(properties, TestPlaningBoat.RUDDER.positionLocal());
        WorldWaterSampler.Contact blade = sampler.sample(rudderPoint, TestPlaningBoat.RUDDER.spanM() / 2, waves.waves(), waves.stillWaterY(), time);
        SurfaceForces rudder = TestPlaningBoat.RUDDER.load(state, properties, rudderRadians, environment.waterAt(blade.currentMps()),
                screw.thrustN() * immersion, TestPlaningBoat.PROPELLER.diameterM());
        Vec3d rudderForce = rudder.wrench().force().scale(blade.fraction());
        if (rudderForce.lengthSquared() > 0) {
            loads.add(new AppliedForce(AppliedForce.Kind.LIFT, state.worldOffset(properties, TestPlaningBoat.RUDDER.positionLocal()), rudderForce));
        }
        loads.addAll(entityLoads.forces(state, properties));
        pendingThrust = screw.thrustN() * immersion; pendingTorque = screw.torqueNm() * immersion;
        pendingRudder = rudderForce.dot(state.orientation().rotate(Vec3d.Z));
        pending = PlaningDynamics.prepare(state, properties, TestPlaningBoat.HULL, water, GRAVITY, loads, dt);
        return pending.damped();
    }
    @Override public void finishSubstep(int i, ContactSolver.BodyResult contact) {
        PlaningDynamics.Result result = PlaningDynamics.finish(pending, contact.state(), contact.wrench(), contact.forces(),
                contact.staticFrictionW() + contact.pairFrictionW(), collision);
        if (!WorldObstacles.contains(region, envelope(result.state().position(), result.state().orientation()))) { stop(); moving = false; return; }
        forceOrigin = state.position();
        displayForces = result.forces();
        PlaningDynamics.Diagnostics d = result.diagnostics();
        radiation.recordAt(result.state(), properties, RADIATION_POINTS, Vec3d.Y,
                HullDynamics.segmentRadii(TestPlaningBoat.HULL.hull(), RADIATION_SEGMENTS, d.waterlineBreadth()));
        state = result.state();
        thrustSum += pendingThrust; torqueSum += pendingTorque; rudderSum += pendingRudder;
        frictionSum += d.frictionN(); surgeSum += d.surgeMps(); wetSum += d.wettedLengthM(); ventSum += d.ventilation(); powerSum += d.dragPowerW();
        contactSum = contactSum.add(result.contact());
        steps++;
    }
    @Override public void correctPose(RigidBodyState corrected) { state = corrected; }
    @Override public void endTick() {
        double n = Math.max(steps, 1);
        thrustN = thrustSum / n; torqueNm = torqueSum / n; frictionN = frictionSum / n; surgeMps = surgeSum / n;
        wettedLength = wetSum / n; ventilation = ventSum / n; rudderLiftN = rudderSum / n; dragPowerW = powerSum / n;
        contactLoad = new Wrench(contactSum.force().scale(1 / n), contactSum.torque().scale(1 / n));
        publish();
        BodyCollisions.pushOut(this);
        PlatformDebug.display(this, sampler, startTime);
    }

    private void stop() { state = new RigidBodyState(state.position(), Vec3d.ZERO, state.orientation(), Vec3d.ZERO); }
    private void publish() {
        Orientation q = state.orientation();
        entityData.set(ROTATION, new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
        entityData.set(PROPELLER, (float) propellerRps);
        entityData.set(RUDDER, (float) rudderRadians);
        entityData.set(WETTED, (float) wettedLength);
        setPos(state.position().x(), state.position().y(), state.position().z());
        setBoundingBox(envelope(state.position(), q));
        setDeltaMovement(MinecraftUnits.toBlocksPerTick(state.velocity()));
        hasImpulse = true;
    }
    @Override protected AABB makeBoundingBox() {
        if (state == null) { return new AABB(getX() - 3.5, getY() - 1, getZ() - 3.5, getX() + 3.5, getY() + 1, getZ() + 3.5); }
        return envelope(new Vec3d(getX(), getY(), getZ()), level().isClientSide ? clientRotation : state.orientation());
    }
    /** Box around the hull, propeller and rudder, rotated into the world. */
    public static AABB envelope(Vec3d position, Orientation q) {
        Vec3d origin = position.subtract(q.rotate(TestPlaningBoat.CENTER_OF_MASS));
        double x0 = -0.7, x1 = TestPlaningBoat.LENGTH, y0 = -0.45, y1 = TestPlaningBoat.DEPTH, z = TestPlaningBoat.BEAM / 2;
        AABB result = null;
        for (double x : new double[]{x0, x1}) for (double y : new double[]{y0, y1}) for (double s : new double[]{-z, z}) {
            Vec3d c = origin.add(q.rotate(new Vec3d(x, y, s)));
            AABB a = new AABB(c.x(), c.y(), c.z(), c.x(), c.y(), c.z());
            result = result == null ? a : result.minmax(a);
        }
        return result;
    }
    private boolean loaded(AABB box) {
        return !(level() instanceof ServerLevel server) || server.hasChunksAt(BlockPos.containing(box.minX, box.minY, box.minZ), BlockPos.containing(box.maxX, box.maxY, box.maxZ));
    }
    @Override public void lerpTo(double x, double y, double z, float yaw, float pitch, int steps) { targetPosition = new Vec3(x, y, z); lerpSteps = Math.max(1, steps); }
    @Override public boolean isPushedByFluid() { return false; }
    @Override public boolean isPushedByFluid(FluidType type) { return false; }
    @Override public boolean isPickable() { return !isRemoved(); }
    @Override protected MovementEmission getMovementEmission() { return MovementEmission.NONE; }
    @Override public InteractionResult interact(Player player, InteractionHand hand) {
        if (!level().isClientSide && hand == InteractionHand.MAIN_HAND) { for (Component line : report()) { player.sendSystemMessage(line); } }
        return InteractionResult.sidedSuccess(level().isClientSide);
    }
    public List<Component> report() { return List.of(diagnostics(), controlDiagnostics()); }
    public Component diagnostics() {
        Vec3d up = state.orientation().rotate(Vec3d.Y), ahead = state.orientation().rotate(Vec3d.X);
        Vec3d level = new Vec3d(ahead.x(), 0, ahead.z());
        double heel = level.lengthSquared() < 1e-9 ? 0 : Math.toDegrees(Math.asin(Math.clamp(up.dot(level.normalized().cross(Vec3d.Y)), -1, 1)));
        double trim = Math.toDegrees(Math.asin(Math.clamp(ahead.y(), -1, 1)));
        double heading = Math.toDegrees(Math.atan2(-ahead.x(), ahead.z()));
        double cv = Math.abs(surgeMps) / Math.sqrt(Atmosphere.GRAVITY_MPS2 * TestPlaningBoat.BEAM);
        double knotsPerRootFoot = Math.abs(surgeMps) / 0.514444 / Math.sqrt(TestPlaningBoat.LENGTH / 0.3048);
        return Component.translatable("command.aerocore.planing.info", getId(), f(state.velocity().length()), f(surgeMps), f(cv), f(knotsPerRootFoot),
                f(heading), f(Math.toDegrees(-state.angularVelocity(properties).y())), f(heel), f(trim),
                f(wettedLength / TestPlaningBoat.BEAM), f(ventilation * 100), f(frictionN), f(dragPowerW));
    }
    public Component controlDiagnostics() {
        double d = TestPlaningBoat.PROPELLER.diameterM();
        String ratio = propellerRps == 0 ? "-" : f(surgeMps / (propellerRps * d));
        return Component.translatable("command.aerocore.hull.controls", f(propellerRps), ratio, f(thrustN), f(torqueNm),
                f(2 * Math.PI * propellerRps * torqueNm), f(Math.toDegrees(rudderRadians)), f(rudderLiftN), vector(contactLoad.force()));
    }
    private static String vector(Vec3d v) { return "(" + f(v.x()) + ", " + f(v.y()) + ", " + f(v.z()) + ")"; }
    private static String f(double value) { return String.format(Locale.ROOT, "%.2f", value); }
    @Override protected void addAdditionalSaveData(CompoundTag tag) {
        if (owner != null) { tag.putUUID("Owner", owner); }
        Orientation q = state.orientation();
        tag.putDouble("Qw", q.w()); tag.putDouble("Qx", q.x()); tag.putDouble("Qy", q.y()); tag.putDouble("Qz", q.z());
    }
    @Override protected void readAdditionalSaveData(CompoundTag tag) {
        owner = tag.hasUUID("Owner") ? tag.getUUID("Owner") : null;
        try {
            Orientation q = new Orientation(tag.getDouble("Qw"), tag.getDouble("Qx"), tag.getDouble("Qy"), tag.getDouble("Qz"));
            state = new RigidBodyState(new Vec3d(getX(), getY(), getZ()), Vec3d.ZERO, q, Vec3d.ZERO);
        } catch (IllegalArgumentException invalid) {
            state = new RigidBodyState(new Vec3d(getX(), getY(), getZ()), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        }
        publish();
    }
}
