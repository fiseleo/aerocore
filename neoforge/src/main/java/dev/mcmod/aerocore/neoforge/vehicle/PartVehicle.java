package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.vehicle.HullDesign;
import dev.mcmod.aerocore.api.vehicle.Json;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PlaningHullDesign;
import dev.mcmod.aerocore.api.vehicle.Vehicle;
import dev.mcmod.aerocore.api.vehicle.VehicleBlueprint;
import dev.mcmod.aerocore.api.vehicle.VehicleDynamics;
import dev.mcmod.aerocore.api.vehicle.VehicleEnvironment;
import dev.mcmod.aerocore.api.vehicle.VehicleState;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.probe.BodyCollisions;
import dev.mcmod.aerocore.neoforge.probe.CoupledBody;
import dev.mcmod.aerocore.neoforge.probe.DebugBody;
import dev.mcmod.aerocore.neoforge.probe.EntityLoads;
import dev.mcmod.aerocore.neoforge.probe.EntityMasses;
import dev.mcmod.aerocore.neoforge.probe.PlatformDebug;
import dev.mcmod.aerocore.neoforge.probe.PlatformEnvironment;
import dev.mcmod.aerocore.neoforge.probe.SolidBody;
import dev.mcmod.aerocore.neoforge.probe.Wakes;
import dev.mcmod.aerocore.neoforge.probe.WaveRegions;
import dev.mcmod.aerocore.neoforge.probe.WorldObstacles;
import dev.mcmod.aerocore.neoforge.probe.WorldWaterSampler;
import net.minecraft.core.BlockPos;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.network.syncher.EntityDataAccessor;
import net.minecraft.network.syncher.EntityDataSerializers;
import net.minecraft.network.syncher.SynchedEntityData;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
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
import org.slf4j.Logger;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

/**
 * A vehicle built from a blueprint (VehicleBlueprint): the game side of VehicleDynamics, for vehicle mods to use as
 * it is or extend. The server owns the physics: each tick in SUBSTEPS substeps, stepped with the other bodies by
 * BodyWorld (contacts with blocks, AeroCore bodies and vanilla entities), in the shared sea (SeaSurface waves, coasts,
 * wakes), the world's wind and weather. Clients receive the blueprint, the pose and the controls and rotor speeds
 * for animation; they never simulate. People stand on the parts' decks (SolidBody) and sit in seats (riders' weight
 * loads their seat); the hull parts make wakes and spray.
 *
 * <p>Control: setControls (server) or VehicleNetwork.send (client, accepted when canControl allows: by default the
 * player in the first seat). A vehicle mod overrides canControl, usesDefaultControls and the interaction to suit it.
 */
public class PartVehicle extends Entity implements CoupledBody, SolidBody, DebugBody, Wakes.Source {
    public static final int SUBSTEPS = 10;
    public static final double SUBSTEP = MinecraftUnits.TICK_SECONDS / SUBSTEPS;
    private static final Logger LOGGER = com.mojang.logging.LogUtils.getLogger();
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private static final EntityDataAccessor<String> BLUEPRINT = SynchedEntityData.defineId(PartVehicle.class, VehicleRegistry.LONG_TEXT.get());
    private static final EntityDataAccessor<Quaternionf> ROTATION = SynchedEntityData.defineId(PartVehicle.class, EntityDataSerializers.QUATERNION);
    /** Controls and rotor speeds for animation, as JSON {"c": {channel: value}, "r": [rad/s...]}. */
    private static final EntityDataAccessor<String> DISPLAY = SynchedEntityData.defineId(PartVehicle.class, VehicleRegistry.LONG_TEXT.get());
    /**
     * Thin-ship wake designs by hull design JSON, built on the warm-up thread (a hull's wake forcing takes a fifth of a
     * second); the vehicle makes no wake until its design is ready. Both sides use them (clients draw the wakes).
     */
    private static final Map<String, java.util.concurrent.CompletableFuture<Wakes.Design>> WAKES = Collections.synchronizedMap(new LinkedHashMap<>(16, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<String, java.util.concurrent.CompletableFuture<Wakes.Design>> eldest) { return size() > 32; }
    });

    /**
     * One background thread builds new designs' tables (a hull's Michell table takes about a second; on the server
     * thread that froze the game when such a vehicle was spawned or loaded). The vehicle waits until they are ready.
     */
    private static final java.util.concurrent.ExecutorService WARM_UP = java.util.concurrent.Executors.newSingleThreadExecutor(task -> {
        Thread t = new Thread(task, "AeroCore vehicle warm-up");
        t.setDaemon(true);
        return t;
    });

    private VehicleBlueprint blueprint;
    private Vehicle vehicle;
    /** Vehicle-frame bounds of the contact spheres and decks, for boxFor (see envelope). */
    private Vehicle boxFor;
    private double[] boxLow, boxHigh;
    private java.util.concurrent.CompletableFuture<Void> warm;
    private VehicleState state;
    private String failure;
    private final Map<String, Double> controls = new LinkedHashMap<>();
    private UUID owner;
    private int debugMode;
    private final PlatformEnvironment environment = new PlatformEnvironment();
    private WaveRegions.Local waves = WaveRegions.Local.CALM;
    private final EntityLoads entityLoads = new EntityLoads();
    private List<AppliedForce> displayForces = List.of();
    private Vec3d forceOrigin = Vec3d.ZERO;
    private String published = "";
    // Client pose and animation.
    private Orientation clientRotation = Orientation.IDENTITY, previousClientRotation = Orientation.IDENTITY;
    private int lerpSteps;
    private Vec3 targetPosition = Vec3.ZERO;
    private final Map<String, Double> clientControls = new LinkedHashMap<>();
    private double[] clientRotorSpeeds = new double[0], rotorAngles = new double[0], previousRotorAngles = new double[0];

    public PartVehicle(EntityType<? extends PartVehicle> type, Level level) {
        super(type, level);
        setNoGravity(true);
    }

    @Override protected void defineSynchedData(SynchedEntityData.Builder builder) {
        builder.define(BLUEPRINT, "");
        builder.define(ROTATION, new Quaternionf());
        builder.define(DISPLAY, "{}");
    }

    // ---------------------------------------------------------------- setup and access

    /**
     * Server: builds this vehicle from a blueprint with its frame origin at origin (world) turned by q, at rest. Throws
     * DesignException (with every issue) when the blueprint does not compile; warnings are in vehicle().issues().
     */
    public void setBlueprint(VehicleBlueprint blueprint, Vec3d origin, Orientation q) {
        Vehicle compiled = blueprint.compile();
        this.blueprint = blueprint;
        this.vehicle = compiled;
        this.failure = null;
        this.state = compiled.newState(origin, q, SUBSTEP);
        entityData.set(BLUEPRINT, Json.write(blueprint.toJson()));
        warmUp();
        publish();
    }

    private void warmUp() {
        if (level().isClientSide || vehicle == null) { return; }
        Vehicle v = vehicle;
        warm = java.util.concurrent.CompletableFuture.runAsync(v::warmUp, WARM_UP);
        wakeDesign();
    }

    /** Server: whether the parts' tables are built and the vehicle steps (it waits in place until then). */
    public boolean ready() { return warm == null || warm.isDone() && !warm.isCompletedExceptionally(); }

    public VehicleBlueprint blueprint() { return blueprint; }
    /** The compiled vehicle, or null before a blueprint arrives or when it failed to compile (see failure()). */
    public Vehicle vehicle() { return vehicle; }
    /** Server only: the physics state (null on clients). */
    public VehicleState vehicleState() { return state; }
    /** Why the stored blueprint does not compile (for example a part type of a mod that is missing), or null. */
    public String failure() { return failure; }
    public void setOwner(UUID owner) { this.owner = owner; }

    /** Server: sets these control channels (others keep their values). */
    public void setControls(Map<String, Double> values) {
        for (Map.Entry<String, Double> e : values.entrySet()) {
            if (e.getValue() == null || !Double.isFinite(e.getValue())) { continue; }
            if (controls.size() >= VehicleNetwork.MAX_CHANNELS && !controls.containsKey(e.getKey())) { continue; }
            controls.put(e.getKey(), e.getValue());
        }
    }
    public void setControl(String channel, double value) { setControls(Map.of(channel, value)); }
    /** The control channels: the server's own, or on a client the last synced ones. */
    public Map<String, Double> controls() { return Collections.unmodifiableMap(level().isClientSide ? clientControls : controls); }

    /** Whether this player may drive: by default whoever sits in the first seat. */
    public boolean canControl(Player player) { return getFirstPassenger() == player; }
    /** Whether AeroCore's client maps the movement keys to "throttle" and "rudder" while the driver rides (AeroCore's QA mapping). */
    public boolean usesDefaultControls() { return true; }

    /** Speed of a rotor (index in vehicle().rotors()), rad/s: server physics, client the last synced value. */
    public double rotorSpeed(int rotor) {
        if (!level().isClientSide) { return state == null ? 0 : state.rotorSpeed(rotor); }
        return rotor < clientRotorSpeeds.length ? clientRotorSpeeds[rotor] : 0;
    }
    /** Client: rotor angle for drawing (rad, integrated from the synced speed), interpolated within the tick. */
    public double rotorAngle(int rotor, float partialTick) {
        if (rotor >= rotorAngles.length) { return 0; }
        return previousRotorAngles[rotor] + (rotorAngles[rotor] - previousRotorAngles[rotor]) * partialTick;
    }
    /** The input a part reads for a role (offset + scale * channel, summed over its bindings), from the controls known here. */
    public double input(String partId, String role) {
        Vehicle.Installed installed = vehicle == null ? null : vehicle.installed(partId);
        if (installed == null) { return 0; }
        double sum = 0;
        for (var b : installed.bindings()) {
            if (b.role().equals(role)) { sum += b.offset() + b.scale() * controls().getOrDefault(b.channel(), 0.0); }
        }
        return sum;
    }

    public Orientation visualOrientation(float partialTick) {
        return level().isClientSide ? previousClientRotation.interpolate(clientRotation, Math.clamp(partialTick, 0, 1)) : poseOrientation();
    }

    @Override public void onSyncedDataUpdated(EntityDataAccessor<?> key) {
        super.onSyncedDataUpdated(key);
        if (!level().isClientSide) { return; }
        if (BLUEPRINT.equals(key)) { compileFromJson(entityData.get(BLUEPRINT)); }
        if (DISPLAY.equals(key)) { readDisplay(entityData.get(DISPLAY)); }
    }

    private void compileFromJson(String json) {
        if (json.isEmpty()) { return; }
        try {
            blueprint = VehicleBlueprint.fromJsonText(json);
            vehicle = blueprint.compile();
            failure = null;
            int rotors = vehicle.rotors().size();
            clientRotorSpeeds = new double[rotors]; rotorAngles = new double[rotors]; previousRotorAngles = new double[rotors];
            // Start building the hull's wake (drawn here too) before the first tick asks for it.
            wakeDesign();
        } catch (RuntimeException e) {
            vehicle = null;
            failure = e.getMessage();
            LOGGER.warn("Vehicle {} has a blueprint that does not compile: {}", getId(), e.getMessage());
        }
    }

    private void readDisplay(String json) {
        try {
            Map<?, ?> m = (Map<?, ?>) Json.parse(json);
            clientControls.clear();
            if (m.get("c") instanceof Map<?, ?> c) { c.forEach((k, v) -> { if (v instanceof Number n) { clientControls.put(String.valueOf(k), n.doubleValue()); } }); }
            if (m.get("r") instanceof List<?> r) {
                for (int i = 0; i < Math.min(r.size(), clientRotorSpeeds.length); i++) { if (r.get(i) instanceof Number n) { clientRotorSpeeds[i] = n.doubleValue(); } }
            }
        } catch (RuntimeException ignored) {
            // A malformed display update only affects animation.
        }
    }

    // ---------------------------------------------------------------- ticking

    @Override public void tick() {
        super.tick();
        if (level().isClientSide) {
            previousClientRotation = clientRotation;
            Quaternionf q = entityData.get(ROTATION);
            if (q.lengthSquared() > 0) { clientRotation = clientRotation.interpolate(new Orientation(q.w, q.x, q.y, q.z), 0.5); }
            if (lerpSteps > 0) { setPos(position().lerp(targetPosition, 1.0 / lerpSteps--)); }
            for (int i = 0; i < rotorAngles.length; i++) {
                previousRotorAngles[i] = rotorAngles[i];
                rotorAngles[i] += clientRotorSpeeds[i] * MinecraftUnits.TICK_SECONDS;
                if (Math.abs(rotorAngles[i]) > 1e4) { double wrap = 2 * Math.PI * Math.floor(rotorAngles[i] / (2 * Math.PI)); rotorAngles[i] -= wrap; previousRotorAngles[i] -= wrap; }
            }
            setBoundingBox(makeBoundingBox());
            if (vehicle != null) {
                BodyCollisions.trackClient(this);
                BodyCollisions.pushOut(this);
            }
            return;
        }
        if (getY() < level().getMinBuildHeight() - 64) { discard(); }
    }

    // ---------------------------------------------------------------- CoupledBody: one tick in SUBSTEPS substeps

    private WorldWaterSampler sampler;
    private int seaLevel;
    private double startTime;
    private AABB region;
    private FloatingBodyDynamics.Collision collision;
    private boolean moving;
    private VehicleDynamics.Prepared pending;
    /**
     * Air (with the world's wind) by block for this tick: the wind varies over metres and seconds, and asking the
     * world wind at every hull station in every substep was a fifth of a vehicle's step.
     */
    private final Map<Long, FluidState> air = new java.util.HashMap<>();

    @Override public Entity entity() { return this; }

    @Override public boolean beginTick(ServerLevel server) {
        if (isRemoved() || vehicle == null || state == null) { return false; }
        if (warm != null && !warm.isDone()) { publish(); return false; }
        if (warm != null && warm.isCompletedExceptionally()) {
            failure = "Part tables could not be built: " + warm.handle((x, e) -> String.valueOf(e)).join();
            LOGGER.error("Vehicle {} stopped: {}", getId(), failure);
            vehicle = null;
            return false;
        }
        sampler = new WorldWaterSampler(server).excludingWakeOf(getId());
        seaLevel = server.getChunkSource().getGenerator().getSeaLevel();
        RigidBodyState s = state.body();
        double reach = 0;
        for (CollisionSphere c : vehicle.collision()) { reach = Math.max(reach, c.centerLocal().subtract(vehicle.properties().centerOfMassLocal()).length() + c.radiusM()); }
        double travel = (s.velocity().length() + s.angularVelocity(vehicle.properties()).length() * reach) * MinecraftUnits.TICK_SECONDS;
        region = envelope(s.position(), s.orientation()).inflate(2 * travel + 1);
        if (!loaded(region)) { stop(); publish(); return false; }
        collision = new FloatingBodyDynamics.Collision(vehicle.collision(), WorldObstacles.collect(this, region), vehicle.contactMaterial());
        startTime = server.getGameTime() * MinecraftUnits.TICK_SECONDS;
        Vec3d origin = vehicle.origin(s);
        waves = WaveRegions.of(server).at(server, origin.x(), origin.z());
        environment.setSea(waves);
        sampler.drifting(waves.region());
        entityLoads.beginTick();
        air.clear();
        moving = true;
        return true;
    }

    @Override public boolean active() { return moving; }
    @Override public RigidBodyState physicsState() { return state.body(); }
    @Override public RigidBodyProperties properties() { return vehicle.properties(); }
    @Override public List<CollisionSphere> collisionShape() { return collision.shape(); }
    @Override public ContactMaterial contactMaterial() { return vehicle.contactMaterial(); }
    @Override public List<WorldBox> staticObstacles() { return collision.obstacles(); }
    @Override public AABB physicsEnvelope() { return envelope(state.body().position(), state.body().orientation()); }

    /** The sea, wind and terrain the parts sample during one substep. */
    private VehicleEnvironment environment(double time) {
        WaveField field = waves.waves();
        return new VehicleEnvironment() {
            @Override public FloatingBodyDynamics.Contact sphere(Vec3d centre, double radius) {
                WorldWaterSampler.Contact c = sampler.sample(centre, radius, waves.compiled(), waves.stillWaterY(), time);
                FluidState air = airAt(centre);
                // Surface-piercing floats take their heave diffraction from the hemisphere table (FloatDesign).
                return new FloatingBodyDynamics.Contact(c.fraction(), c.wetCentroidOffset(), air, environment.waterAt(c.currentMps()),
                        c.pressureFraction(), c.pressureCentroidOffset(), c.accelerationMps2(), c.fraction() > 0 && c.fraction() < 1);
            }
            @Override public HullDynamics.Water point(Vec3d p) {
                WorldWaterSampler.Column c = sampler.column(p, waves.compiled(), waves.stillWaterY(), time);
                FluidState air = airAt(p), liquid = environment.waterAt(c.currentMps());
                return new HullDynamics.Water(c.surfaceY(), c.slope(), c.stillY(), c.decayWavenumber(), liquid.velocity(), c.accelerationMps2(), liquid, air);
            }
            @Override public WaveField waves() { return field; }
        };
    }

    private FluidState airAt(Vec3d p) {
        long key = BlockPos.asLong((int) Math.floor(p.x()), (int) Math.floor(p.y()), (int) Math.floor(p.z()));
        return air.computeIfAbsent(key, k -> environment.airAt(p, seaLevel).fluid());
    }

    @Override public RigidBodyState prepareSubstep(int i) {
        double time = startTime + i * SUBSTEP;
        RigidBodyState s = state.body();
        RigidBodyProperties p = vehicle.properties();
        List<AppliedForce> external = new ArrayList<>(entityLoads.forces(s, p));
        // Riders' weight on their seats.
        List<Entity> riders = getPassengers();
        for (int k = 0; k < Math.min(riders.size(), vehicle.seats().size()); k++) {
            double mass = EntityMasses.massKg(riders.get(k));
            external.add(new AppliedForce(AppliedForce.Kind.CONTACT, s.worldOffset(p, vehicle.seats().get(k)), GRAVITY.scale(mass)));
        }
        pending = VehicleDynamics.prepare(vehicle, state, environment(time), controls, GRAVITY, external, time, SUBSTEP);
        return pending.damped();
    }

    @Override public void finishSubstep(int i, ContactSolver.BodyResult contact) {
        RigidBodyState before = state.body();
        VehicleDynamics.Result result = VehicleDynamics.finish(pending, contact.state(), contact.wrench(), contact.forces(),
                contact.staticFrictionW() + contact.pairFrictionW(), collision);
        if (!WorldObstacles.contains(region, envelope(result.state().position(), result.state().orientation()))) {
            // Left the region whose obstacles were collected: never move into unchecked space.
            state.setBody(new RigidBodyState(before.position(), Vec3d.ZERO, before.orientation(), Vec3d.ZERO));
            moving = false;
            return;
        }
        forceOrigin = before.position();
        displayForces = result.forces();
    }

    @Override public void correctPose(RigidBodyState corrected) { state.setBody(corrected); }

    @Override public void endTick() {
        publish();
        BodyCollisions.pushOut(this);
        PlatformDebug.display(this, sampler, startTime);
    }

    private void stop() {
        RigidBodyState s = state.body();
        state.setBody(new RigidBodyState(s.position(), Vec3d.ZERO, s.orientation(), Vec3d.ZERO));
    }

    private boolean loaded(AABB box) {
        return !(level() instanceof ServerLevel server) || server.hasChunksAt(BlockPos.containing(box.minX, box.minY, box.minZ), BlockPos.containing(box.maxX, box.maxY, box.maxZ));
    }

    private void publish() {
        if (state == null) { return; }
        RigidBodyState s = state.body();
        Orientation q = s.orientation();
        entityData.set(ROTATION, new Quaternionf((float) q.x(), (float) q.y(), (float) q.z(), (float) q.w()));
        setPos(s.position().x(), s.position().y(), s.position().z());
        setBoundingBox(envelope(s.position(), q));
        setDeltaMovement(MinecraftUnits.toBlocksPerTick(s.velocity()));
        hasImpulse = true;
        List<Object> speeds = new ArrayList<>();
        for (int r = 0; r < vehicle.rotors().size(); r++) { speeds.add((double) (float) state.rotorSpeed(r)); }
        Map<String, Object> display = new LinkedHashMap<>();
        display.put("c", new LinkedHashMap<String, Object>(controls));
        display.put("r", speeds);
        String text = Json.write(display);
        if (!text.equals(published)) { published = text; entityData.set(DISPLAY, text); }
    }

    // ---------------------------------------------------------------- pose, bounds, SolidBody

    @Override public Vec3d poseCenter() { return level().isClientSide || state == null ? new Vec3d(getX(), getY(), getZ()) : state.body().position(); }
    @Override public Orientation poseOrientation() { return level().isClientSide || state == null ? clientRotation : state.body().orientation(); }
    @Override public Vec3d centerOfMassLocal() { return vehicle == null ? Vec3d.ZERO : vehicle.properties().centerOfMassLocal(); }
    @Override public List<WorldBox> solidBoxesLocal() { return vehicle == null ? List.of() : vehicle.deck(); }
    @Override public Vec3d velocityAt(Vec3d point) {
        if (level().isClientSide || state == null) { return Vec3d.ZERO; }
        RigidBodyState s = state.body();
        return s.velocity().add(s.angularVelocity(vehicle.properties()).cross(point.subtract(s.position())));
    }
    @Override public void addEntityLoad(Vec3d localPoint, Vec3d worldForce) { if (!level().isClientSide) { entityLoads.add(localPoint, worldForce); } }

    @Override protected AABB makeBoundingBox() {
        if (vehicle == null) { return new AABB(getX() - 0.5, getY() - 0.5, getZ() - 0.5, getX() + 0.5, getY() + 0.5, getZ() + 0.5); }
        return envelope(new Vec3d(getX(), getY(), getZ()), poseOrientation());
    }

    /** The box around the contact spheres and decks, turned into the world, for a COM at position. */
    public AABB envelope(Vec3d position, Orientation q) {
        if (boxFor != vehicle) {
            // Vehicle-frame box, once per compiled vehicle (BodyWorld asks for the envelope of every pair every substep).
            double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
            for (CollisionSphere c : vehicle.collision()) {
                Vec3d v = c.centerLocal();
                double[] xyz = {v.x(), v.y(), v.z()};
                for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], xyz[k] - c.radiusM()); hi[k] = Math.max(hi[k], xyz[k] + c.radiusM()); }
            }
            for (WorldBox b : vehicle.deck()) {
                double[] mn = {b.min().x(), b.min().y(), b.min().z()}, mx = {b.max().x(), b.max().y(), b.max().z()};
                for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], mn[k]); hi[k] = Math.max(hi[k], mx[k]); }
            }
            boxLow = lo; boxHigh = hi; boxFor = vehicle;
        }
        double[] lo = boxLow, hi = boxHigh;
        if (lo[0] > hi[0]) { return new AABB(position.x() - 0.5, position.y() - 0.5, position.z() - 0.5, position.x() + 0.5, position.y() + 0.5, position.z() + 0.5); }
        Vec3d com = vehicle.properties().centerOfMassLocal();
        AABB result = null;
        for (int corner = 0; corner < 8; corner++) {
            Vec3d local = new Vec3d((corner & 1) == 0 ? lo[0] : hi[0], (corner & 2) == 0 ? lo[1] : hi[1], (corner & 4) == 0 ? lo[2] : hi[2]);
            Vec3d w = position.add(q.rotate(local.subtract(com)));
            AABB a = new AABB(w.x(), w.y(), w.z(), w.x(), w.y(), w.z());
            result = result == null ? a : result.minmax(a);
        }
        return result;
    }

    @Override public void lerpTo(double x, double y, double z, float yaw, float pitch, int steps) { targetPosition = new Vec3(x, y, z); lerpSteps = Math.max(1, steps); }
    @Override public boolean isPushedByFluid() { return false; }
    @Override public boolean isPushedByFluid(FluidType type) { return false; }
    @Override public boolean isPickable() { return !isRemoved(); }
    @Override protected MovementEmission getMovementEmission() { return MovementEmission.NONE; }

    // ---------------------------------------------------------------- riders

    @Override protected boolean canAddPassenger(Entity passenger) { return vehicle != null && getPassengers().size() < vehicle.seats().size(); }

    @Override protected void positionRider(Entity passenger, MoveFunction move) {
        int index = getPassengers().indexOf(passenger);
        if (vehicle == null || index < 0 || index >= vehicle.seats().size()) { super.positionRider(passenger, move); return; }
        Vec3d seat = toWorld(vehicle.seats().get(index));
        move.accept(passenger, seat.x(), seat.y(), seat.z());
    }

    /** Right-click: sit in a free seat; with sneak, the vehicle's report in chat. */
    @Override public InteractionResult interact(Player player, InteractionHand hand) {
        if (hand != InteractionHand.MAIN_HAND) { return InteractionResult.PASS; }
        if (player.isSecondaryUseActive()) {
            if (!level().isClientSide && player instanceof ServerPlayer sp) { VehicleCommands.report(sp.createCommandSourceStack(), this); }
            return InteractionResult.sidedSuccess(level().isClientSide);
        }
        if (!canAddPassenger(player)) { return InteractionResult.PASS; }
        if (!level().isClientSide) { player.startRiding(this); }
        return InteractionResult.sidedSuccess(level().isClientSide);
    }

    // ---------------------------------------------------------------- wakes

    /** The first hull part's wake: Michell thin-ship sources for a displacement hull, a pressure patch for a planing one. */
    @Override public Wakes.Design wakeDesign() {
        if (vehicle == null) { return null; }
        for (Vehicle.Installed installed : vehicle.installed()) {
            PartDesign design = installed.part().design();
            if (design instanceof HullDesign hull && hull.hullType() == dev.mcmod.aerocore.api.HullModel.Type.SURFACE) {
                return WAKES.computeIfAbsent(Json.write(hull.toJson()), key -> java.util.concurrent.CompletableFuture.supplyAsync(() -> {
                    var geometry = hull.geometry();
                    return Wakes.thinShip(geometry, hull.designParticulars(geometry));
                }, WARM_UP).exceptionally(failed -> {
                    LOGGER.warn("No wake for hull design {}: {}", hull.name(), failed.toString());
                    return null;
                })).getNow(null);
            }
            if (design instanceof PlaningHullDesign planing) {
                Vec3d ahead = poseOrientation().rotate(installed.rotation().rotate(Vec3d.X));
                double trim = Math.asin(Math.clamp(ahead.y(), -1, 1)), wetted = 0;
                if (state != null) {
                    Map<String, Double> d = state.diagnostics().get(installed.id());
                    if (d != null) { wetted = d.getOrDefault("wettedLengthM", 0.0); }
                }
                return Wakes.planingBoat(planing.chineBeamM(), Math.toRadians(planing.deadriseDeg()), wetted, vehicle.properties().massKg() * Atmosphere.GRAVITY_MPS2, trim);
            }
        }
        return null;
    }
    private Vehicle.Installed wakePart() {
        for (Vehicle.Installed installed : vehicle.installed()) {
            PartDesign d = installed.part().design();
            if (d instanceof HullDesign || d instanceof PlaningHullDesign) { return installed; }
        }
        return null;
    }
    @Override public Vec3d wakeOrigin() {
        Vehicle.Installed part = vehicle == null ? null : wakePart();
        return toWorld(part == null ? Vec3d.ZERO : part.mount().position());
    }
    @Override public Vec3d wakeForward() {
        Vehicle.Installed part = vehicle == null ? null : wakePart();
        return poseOrientation().rotate(part == null ? Vec3d.X : part.rotation().rotate(Vec3d.X));
    }
    @Override public Vec3d wakeVelocity() {
        return level().isClientSide || state == null ? new Vec3d(getX() - xo, getY() - yo, getZ() - zo).scale(1 / MinecraftUnits.TICK_SECONDS) : state.body().velocity();
    }

    // ---------------------------------------------------------------- DebugBody (force arrows for its owner)

    @Override public UUID owner() { return owner; }
    @Override public int debugMode() { return debugMode; }
    public void setDebugMode(int mode) {
        if (mode < 0 || mode > 7) { throw new IllegalArgumentException("Invalid debug mode"); }
        debugMode = mode;
    }
    @Override public List<AppliedForce> displayForces() { return displayForces; }
    @Override public Vec3d forceOrigin() { return forceOrigin; }
    @Override public WaveRegions.Local waves() { return waves; }
    @Override public Vec3d modelOrigin() { return state == null ? new Vec3d(getX(), getY(), getZ()) : vehicle.origin(state.body()); }
    @Override public PlatformEnvironment environment() { return environment; }
    @Override public double bodyTemperatureCelsius() { return Double.NaN; }

    // ---------------------------------------------------------------- saving

    @Override protected void addAdditionalSaveData(CompoundTag tag) {
        if (owner != null) { tag.putUUID("Owner", owner); }
        String json = blueprint != null ? Json.write(blueprint.toJson()) : entityData.get(BLUEPRINT);
        tag.putByteArray("Blueprint", json.getBytes(StandardCharsets.UTF_8));
        if (state != null) { tag.putString("State", Json.write(state.toJson())); }
        CompoundTag c = new CompoundTag();
        controls.forEach(c::putDouble);
        tag.put("Controls", c);
    }

    @Override protected void readAdditionalSaveData(CompoundTag tag) {
        owner = tag.hasUUID("Owner") ? tag.getUUID("Owner") : null;
        String json = new String(tag.getByteArray("Blueprint"), StandardCharsets.UTF_8);
        controls.clear();
        CompoundTag c = tag.getCompound("Controls");
        for (String key : c.getAllKeys()) { controls.put(key, c.getDouble(key)); }
        if (json.isEmpty()) { return; }
        // Keep the blueprint even when it no longer compiles (a part mod removed): it is saved back unchanged.
        entityData.set(BLUEPRINT, json);
        try {
            blueprint = VehicleBlueprint.fromJsonText(json);
            vehicle = blueprint.compile();
            failure = null;
            Vec3d origin = new Vec3d(getX(), getY(), getZ());
            state = tag.contains("State") ? VehicleState.fromJson(vehicle, Json.parse(tag.getString("State")), SUBSTEP)
                    : vehicle.newState(origin, Orientation.IDENTITY, SUBSTEP);
            warmUp();
            publish();
        } catch (RuntimeException e) {
            blueprint = null; vehicle = null; state = null;
            failure = e.getMessage();
            LOGGER.warn("Vehicle at {} keeps a blueprint that does not compile: {}", blockPosition(), e.getMessage());
        }
    }
}
