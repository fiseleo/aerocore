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
import dev.mcmod.aerocore.api.vehicle.Coupling;
import dev.mcmod.aerocore.api.vehicle.Json;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.WakeSource;
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
 * loads their seat); the hull parts make wakes and spray. When its physics throws, the vehicle stops alone (at rest,
 * stepped no more, the error logged and kept in fault()) and the server and the other bodies go on.
 *
 * <p>Control: setControls (server) or VehicleNetwork.send (client, accepted when canControl allows: by default the
 * player in the first seat). A vehicle mod overrides canControl, usesDefaultControls and the interaction to suit it.
 */
public class PartVehicle extends Entity implements CoupledBody, SolidBody, DebugBody, Wakes.Source {
    public static final int SUBSTEPS = 10;
    /**
     * Above this speed (m/s: 4 m a tick) a vehicle collects its obstacles along its predicted path instead of in a box
     * around it, keeps the chunks ahead of it loaded, and waits (its motion kept) where they are not ready (0.28).
     */
    public static final double FAST_MPS = 80;
    /** Above this speed (m/s; orbital speed is 7.8 km/s) the physics has run away: the vehicle stops as for any physics error. */
    public static final double MAX_MPS = 30000;
    /** The most a fast vehicle's corridor widens for its turning in a tick (m), and the most blocks its boxes may hold before its motion counts as run away. */
    static final double MAX_TURN_MARGIN = 10, MAX_CORRIDOR_BLOCKS = 2e6;
    /** How far ahead a fast vehicle keeps its path's chunks loaded: seconds, and at most this many chunks. */
    static final double LOOK_AHEAD_S = 1.0;
    static final int MAX_AHEAD_CHUNKS = 48;
    /**
     * Short tickets (2 s, renewed each tick) on the chunks under a fast vehicle's path: loaded, not ticking, which is all
     * a fast vehicle needs to be stepped there (stepsWhereLoaded).
     */
    private static final net.minecraft.server.level.TicketType<net.minecraft.world.level.ChunkPos> PATH =
            net.minecraft.server.level.TicketType.create("aerocore_vehicle_path", java.util.Comparator.comparingLong(net.minecraft.world.level.ChunkPos::toLong), 40);
    public static final double SUBSTEP = MinecraftUnits.TICK_SECONDS / SUBSTEPS;
    private static final Logger LOGGER = com.mojang.logging.LogUtils.getLogger();
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private static final EntityDataAccessor<String> BLUEPRINT = SynchedEntityData.defineId(PartVehicle.class, VehicleRegistry.LONG_TEXT.get());
    private static final EntityDataAccessor<Quaternionf> ROTATION = SynchedEntityData.defineId(PartVehicle.class, EntityDataSerializers.QUATERNION);
    /**
     * Controls, rotor speeds and shown values for animation, and the tanks' contents (0.26), as JSON {"c": {channel:
     * value}, "r": [rad/s...], "s": [shown...], "f": [[kg, fuel]...]}. 0.37 adds the velocity "v": [m/s x3] (vanilla's
     * motion packet clips it at 78 m/s a component) and, while the body scrapes something, "k": [W, m/s, x, y, z,
     * material] (VehicleContacts.Scrape, the point as an offset from the vehicle's position).
     */
    private static final EntityDataAccessor<String> DISPLAY = SynchedEntityData.defineId(PartVehicle.class, VehicleRegistry.LONG_TEXT.get());
    /** 0.36: the leader a follower hangs on, "entity id, a space, the coupling's JSON", or "" (none, or its leader not loaded). */
    private static final EntityDataAccessor<String> LINK = SynchedEntityData.defineId(PartVehicle.class, VehicleRegistry.LONG_TEXT.get());
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
    /** Why the physics stopped (physicsFailed), or null while the vehicle steps; saved. */
    private String fault;
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
    /** The tanks as the client last heard (0.26). */
    private double[] clientTankKg = new double[0];
    private String[] clientTankFuel = new String[0];
    private double[] clientRotorSpeeds = new double[0], rotorAngles = new double[0], previousRotorAngles = new double[0];
    /** Client: the parts' shown values (Part.shownValues) as last synced, and as drawn at this tick and the one before. */
    private double[] syncedShown = new double[0], shown = new double[0], previousShown = new double[0];
    // 0.36: couplings (Couplings). Server: the leader this hangs on and how, where it was last seen; the followers and where.
    private UUID leaderId;
    private Coupling coupling;
    private Vec3d leaderSeen;
    private final Map<UUID, Vec3d> followers = new LinkedHashMap<>();
    /** Ticks each missing partner has been waited for with its last place loaded. */
    private final Map<UUID, Integer> waited = new java.util.HashMap<>();
    private ContactSolver.JointResult lastJoint;
    private String publishedLink = "";
    /** Client: the leader's entity id (-1: none) and the coupling, as synced. */
    private int clientLeader = -1;
    private Coupling clientCoupling;
    /** 0.37: this tick's touches (server), and what the client last heard of the velocity and scraping. */
    private final VehicleContacts.Tally tally = new VehicleContacts.Tally();
    /** 0.37: whether the last tick had an impact, and the impulse of the last one told (a jam is told once). */
    private boolean impacting;
    /** 0.38: whether a contact sphere touched anything in this tick's substeps so far, and in the last whole tick. */
    private boolean touched, touching;
    private double toldImpactNs;
    private Vec3d clientVelocity;
    private VehicleContacts.Scrape clientScrape;

    public PartVehicle(EntityType<? extends PartVehicle> type, Level level) {
        super(type, level);
        setNoGravity(true);
    }

    @Override protected void defineSynchedData(SynchedEntityData.Builder builder) {
        builder.define(BLUEPRINT, "");
        builder.define(ROTATION, new Quaternionf());
        builder.define(DISPLAY, "{}");
        builder.define(LINK, "");
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
        this.fault = null;
        this.state = compiled.newState(origin, q, SUBSTEP);
        entityData.set(BLUEPRINT, Json.write(blueprint.toJson()));
        warmUp();
        publish();
    }

    /**
     * Server, 0.31: gives the vehicle a blueprint that differs only in looks (a repainted part, stickers), keeping
     * everything about its motion: place, velocity, shaft speeds, internal states, tanks, wave memory
     * (VehicleState.restyled). Returns false, changing nothing, when the blueprint does not compile or is not the same
     * vehicle to the physics (other mass, parts, shafts, tanks or floats). Its parts' tables are built again in the
     * background (a changed design is a new part) while it waits where it is. Not to be called inside a substep.
     */
    public boolean replaceBlueprint(VehicleBlueprint next) {
        if (level().isClientSide || blueprint == null || vehicle == null || state == null) { return false; }
        Vehicle compiled;
        VehicleState kept;
        try {
            compiled = next.compile();
            kept = state.restyled(compiled);
        } catch (RuntimeException e) {
            LOGGER.warn("Vehicle {} cannot take the new blueprint: {}", getId(), e.getMessage());
            return false;
        }
        blueprint = next;
        vehicle = compiled;
        state = kept;
        pending = null;
        entityData.set(BLUEPRINT, Json.write(next.toJson()));
        warmUp();
        publish();
        return true;
    }

    /**
     * Server, 0.34: refits the vehicle in place with a blueprint whose physics may differ in anything (a modification
     * shop's reshaped, resized or retuned parts): it is set down at rest where it stands with what still fits aboard
     * (VehicleState.refitted: parts' states and shaft speeds by id, fuel up to the new tanks at densityKgPerM3 of each
     * fuel). Riders whose seats are gone get off. Its parts' tables are built again in the background while it waits.
     * Returns the fuel spilled by tank id (kg), or null, changing nothing, when the blueprint does not compile. Not to
     * be called inside a substep.
     */
    public Map<String, Double> refit(VehicleBlueprint next, java.util.function.ToDoubleFunction<String> densityKgPerM3) {
        if (level().isClientSide || blueprint == null || vehicle == null || state == null) { return null; }
        Vehicle compiled;
        VehicleState.Refitted r;
        try {
            compiled = next.compile();
            r = state.refitted(compiled, densityKgPerM3);
        } catch (RuntimeException e) {
            LOGGER.warn("Vehicle {} cannot be refitted: {}", getId(), e.getMessage());
            return null;
        }
        List<Entity> riders = getPassengers();
        for (int k = riders.size() - 1; k >= compiled.seats().size(); k--) { riders.get(k).stopRiding(); }
        uncoupleAll(Couplings.Reason.MOVED);
        blueprint = next;
        vehicle = compiled;
        state = r.state();
        failure = null;
        fault = null;
        pending = null;
        lastGood = state.body();
        displayForces = List.of();
        entityData.set(BLUEPRINT, Json.write(next.toJson()));
        warmUp();
        publish();
        return r.spilledKg();
    }

    /** refit with AeroCore's fuel densities (as refuel takes them: Fuel.byName's, water's for a fuel it does not know). */
    public Map<String, Double> refit(VehicleBlueprint next) {
        return refit(next, fuel -> {
            dev.mcmod.aerocore.api.engine.Fuel f = dev.mcmod.aerocore.api.engine.Fuel.byName(fuel);
            return f == null ? 1000 : f.liquidDensityKgPerM3();
        });
    }

    /**
     * Server, 0.32: sets the vehicle down at rest with its frame origin at origin (world) turned by q, keeping what is
     * aboard: tanks, shaft speeds, internal states (VehicleState.placed); setBlueprint would fill the tanks to the
     * design's fill and start every part afresh. Where an assembly room rolls a finished vehicle out. Returns false when
     * there is no vehicle to place. Not to be called inside a substep.
     */
    public boolean place(Vec3d origin, Orientation q) {
        if (level().isClientSide || vehicle == null || state == null) { return false; }
        uncoupleAll(Couplings.Reason.MOVED);
        state = state.placed(origin, q);
        pending = null;
        lastGood = state.body();
        displayForces = List.of();
        publish();
        return true;
    }

    /** Whether the parts' tables are built now (0.25: a vehicle mod's unfinished vehicle may wait, then call warmUp). */
    protected boolean warmsUp() { return true; }

    /** Server: builds the parts' tables in the background (setBlueprint and loading do; a subclass that waited calls it). */
    protected void warmUp() {
        if (level().isClientSide || vehicle == null || !warmsUp()) { return; }
        Vehicle v = vehicle;
        warm = java.util.concurrent.CompletableFuture.runAsync(v::warmUp, WARM_UP);
        wakeDesign();
    }

    /** Number of fuel tanks (0.26). */
    public int tankCount() { return vehicle == null ? 0 : vehicle.tanks().size(); }

    /** Fuel in a tank, kg: the state's on the server, the last heard on a client. */
    public double tankKg(int tank) {
        if (!level().isClientSide && state != null) { return state.tankKg(tank); }
        return tank < clientTankKg.length ? clientTankKg[tank] : 0;
    }

    /** The fuel in a tank ("" when empty). */
    public String tankFuel(int tank) {
        if (!level().isClientSide && state != null) { return state.tankFuel(tank); }
        return tank < clientTankFuel.length ? clientTankFuel[tank] : "";
    }

    /** A tank's capacity, m^3. */
    public double tankCapacityM3(int tank) { return vehicle == null ? 0 : vehicle.tanks().get(tank).tank().capacityM3(); }

    /**
     * Server: puts fuel in a tank (0.26), at most to the brim at the fuel's density (AeroCore's preset, else water's);
     * a different fuel only into an empty tank. Returns the kg it took.
     */
    public double refuel(int tank, String fuel, double kg) {
        if (state == null || tank < 0 || tank >= tankCount() || !(kg > 0)) { return 0; }
        String now = state.tankFuel(tank);
        if (!now.isEmpty() && !now.equals(fuel)) { return 0; }
        dev.mcmod.aerocore.api.engine.Fuel f = dev.mcmod.aerocore.api.engine.Fuel.byName(fuel);
        double before = state.tankKg(tank);
        double after = state.setTank(tank, fuel, before + kg, f == null ? 1000 : f.liquidDensityKgPerM3());
        publish();
        return after - before;
    }

    /**
     * Server: sets a tank's contents (0.26): a fuel ("" empties it) and its mass, at most the tank's volume at the
     * density given (the caller's own fuels). Returns the kg now in it.
     */
    public double setTank(int tank, String fuel, double kg, double densityKgPerM3) {
        if (state == null || tank < 0 || tank >= tankCount()) { return 0; }
        double now = state.setTank(tank, fuel, Math.max(0, kg), densityKgPerM3);
        publish();
        return now;
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
    /**
     * Server: why this vehicle no longer steps (its physics threw, see physicsFailed: the phase and the error), or
     * null while it steps. Saved with the vehicle; resume() clears it.
     */
    public String fault() { return fault; }
    /**
     * Server: steps a vehicle stopped by a physics error again, from rest where it stands. The error comes back if its
     * cause has not gone (a part mod fixed, other controls).
     */
    public void resume() { fault = null; }
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

    /** 0.37 server: the players this vehicle is sent to (Minecraft's tracking), for FarSounds. */
    private final java.util.Set<UUID> seenBy = new java.util.HashSet<>();

    @Override public void startSeenByPlayer(ServerPlayer player) {
        super.startSeenByPlayer(player);
        seenBy.add(player.getUUID());
    }

    @Override public void stopSeenByPlayer(ServerPlayer player) {
        super.stopSeenByPlayer(player);
        seenBy.remove(player.getUUID());
    }

    /** Server: whether a player is sent this vehicle now (it is within their tracking range). */
    public boolean seenBy(ServerPlayer player) { return seenBy.contains(player.getUUID()); }

    /**
     * 0.38 (server): whether any of the vehicle's contact spheres touched a block, a body or an entity in the last tick.
     * Wheels and legs meet the ground by their own rays, not spheres: a car standing on its wheels is not touching.
     */
    public boolean touching() { return touching; }
    /** Whether AeroCore's client maps the movement keys to "throttle" and "rudder" while the driver rides (AeroCore's QA mapping). */
    public boolean usesDefaultControls() { return true; }

    /**
     * 0.37: the vehicle's velocity (m/s): the server's physics, on a client the last synced one (from the entity's
     * motion until the first sync; vanilla's motion packet clips each component at 78 m/s).
     */
    public Vec3d velocity() {
        if (!level().isClientSide) { return state == null ? Vec3d.ZERO : state.body().velocity(); }
        if (clientVelocity != null) { return clientVelocity; }
        Vec3 m = getDeltaMovement();
        return new Vec3d(m.x * 20, m.y * 20, m.z * 20);
    }

    /**
     * 0.37 client: what the body scrapes now (the point as an offset from the vehicle's position), or null; the server
     * hears it from VehicleContacts.Contacted.
     */
    public VehicleContacts.Scrape scrape() { return level().isClientSide ? clientScrape : null; }

    /** Speed of a rotor (index in vehicle().rotors()), rad/s: server physics, client the last synced value. */
    public double rotorSpeed(int rotor) {
        if (!level().isClientSide) { return state == null ? 0 : state.rotorSpeed(rotor); }
        return rotor < clientRotorSpeeds.length ? clientRotorSpeeds[rotor] : 0;
    }
    /**
     * A part's i-th shown value (Part.shownValues: a wheel's steer angle, its suspension's compression): the server's
     * own, or on a client the synced one interpolated within the tick (0 when the part shows nothing there).
     */
    public double shown(String partId, int i, float partialTick) {
        Vehicle.Installed installed = vehicle == null ? null : vehicle.installed(partId);
        if (installed == null || i < 0 || i >= installed.part().shownValues()) { return 0; }
        if (!level().isClientSide) { return state == null ? 0 : state.shown(partId, i); }
        int k = installed.firstShown() + i;
        if (k >= shown.length) { return 0; }
        return previousShown[k] + (shown[k] - previousShown[k]) * Math.clamp(partialTick, 0, 1);
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

    /**
     * 0.29: a point in a part's own frame (at its mount, turned by its rotation) where it is drawn in the world at a
     * partial tick: where an effect starts on a part (smoke at a nozzle). The renderer's frame: the entity's position
     * and visual orientation about the centre of mass.
     */
    public Vec3d partPoint(Vehicle.Installed installed, Vec3d local, float partialTick) {
        Vec3d inVehicle = installed.mount().position().add(installed.rotation().rotate(local)).subtract(centerOfMassLocal());
        net.minecraft.world.phys.Vec3 at = getPosition(partialTick);
        return new Vec3d(at.x, at.y, at.z).add(drawnShift(partialTick)).add(visualOrientation(partialTick).rotate(inVehicle));
    }

    /** 0.29: a direction in a part's own frame in the world, as drawn at a partial tick. */
    public Vec3d partDirection(Vehicle.Installed installed, Vec3d local, float partialTick) {
        return visualOrientation(partialTick).rotate(installed.rotation().rotate(local));
    }

    public Orientation visualOrientation(float partialTick) {
        return level().isClientSide ? previousClientRotation.interpolate(clientRotation, Math.clamp(partialTick, 0, 1)) : poseOrientation();
    }

    @Override public void onSyncedDataUpdated(EntityDataAccessor<?> key) {
        super.onSyncedDataUpdated(key);
        if (!level().isClientSide) { return; }
        if (BLUEPRINT.equals(key)) { compileFromJson(entityData.get(BLUEPRINT)); }
        if (DISPLAY.equals(key)) { readDisplay(entityData.get(DISPLAY)); }
        if (LINK.equals(key)) { readLink(entityData.get(LINK)); }
    }

    private void readLink(String text) {
        clientLeader = -1;
        clientCoupling = null;
        int space = text.indexOf(' ');
        if (space <= 0) { return; }
        try {
            clientLeader = Integer.parseInt(text.substring(0, space));
            clientCoupling = Coupling.parse(text.substring(space + 1));
        } catch (RuntimeException e) {
            clientLeader = -1;
            clientCoupling = null;
        }
    }

    private void compileFromJson(String json) {
        if (json.isEmpty()) { return; }
        try {
            blueprint = VehicleBlueprint.fromJsonText(json);
            vehicle = blueprint.compile();
            failure = null;
            int rotors = vehicle.rotors().size();
            clientRotorSpeeds = new double[rotors]; rotorAngles = new double[rotors]; previousRotorAngles = new double[rotors];
            int values = vehicle.shownValues();
            syncedShown = new double[values]; shown = new double[values]; previousShown = new double[values];
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
            if (m.get("s") instanceof List<?> v) {
                for (int i = 0; i < Math.min(v.size(), syncedShown.length); i++) { if (v.get(i) instanceof Number n) { syncedShown[i] = n.doubleValue(); } }
            }
            if (m.get("v") instanceof List<?> v && v.size() == 3 && v.get(0) instanceof Number x && v.get(1) instanceof Number y && v.get(2) instanceof Number z) {
                clientVelocity = new Vec3d(x.doubleValue(), y.doubleValue(), z.doubleValue());
            }
            clientScrape = null;
            if (m.get("k") instanceof List<?> k && k.size() >= 6) {
                double[] n = new double[k.size()];
                for (int i = 0; i < n.length; i++) { n[i] = k.get(i) instanceof Number x ? x.doubleValue() : 0; }
                clientScrape = new VehicleContacts.Scrape(new Vec3d(n[2], n[3], n[4]), n[0], n[1], (int) n[5], n.length > 6 && n[6] != 0);
            }
            if (m.get("f") instanceof List<?> f) {
                double[] kg = new double[f.size()];
                String[] fuel = new String[f.size()];
                for (int i = 0; i < f.size(); i++) {
                    if (f.get(i) instanceof List<?> t && t.size() == 2 && t.get(0) instanceof Number n) { kg[i] = n.doubleValue(); fuel[i] = String.valueOf(t.get(1)); }
                    else { fuel[i] = ""; }
                }
                clientTankKg = kg; clientTankFuel = fuel;
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
            // 0.37: carried forward by the synced velocity: vanilla's motion packet clips each component at 3.9 blocks a
            // tick (78 m/s), so one flying along an axis faster than that was never taken for fast and glided two ticks behind
            Vec3d v = velocity();
            Vec3 motion = new Vec3(v.x() * MinecraftUnits.TICK_SECONDS, v.y() * MinecraftUnits.TICK_SECONDS, v.z() * MinecraftUnits.TICK_SECONDS);
            // as it came in, until the server moves it: a position of the last server tick, like lerpTo's (taken before the
            // level counted this tick; otherwise a fast one's first step here is two ticks of travel)
            if (!targetKnown) { targetPosition = position(); targetTick = tickCount - 1; targetKnown = true; }
            if (motion.length() * 20 >= FAST_MPS) { setPos(targetPosition.add(motion.scale(tickCount - targetTick))); lerpSteps = 0; }
            else if (lerpSteps > 0) { setPos(position().lerp(targetPosition, 1.0 / lerpSteps--)); }
            System.arraycopy(shown, 0, previousShown, 0, shown.length);
            System.arraycopy(syncedShown, 0, shown, 0, shown.length);
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
    /** The boxes this tick's motion may take the vehicle through: the region, or a fast vehicle's corridor. */
    private List<AABB> corridor = List.of();
    private boolean fast;
    private FloatingBodyDynamics.Collision collision;
    private boolean moving;
    private VehicleDynamics.Prepared pending;
    /** The committed state at the start of the current substep: where physicsFailed stops the vehicle. */
    private RigidBodyState lastGood;
    /**
     * Air (with the world's wind) by block for this tick: the wind varies over metres and seconds, and asking the
     * world wind at every hull station in every substep was a fifth of a vehicle's step.
     */
    private final Map<Long, FluidState> air = new java.util.HashMap<>();
    /** The world's blocks as ground for wheels (0.24), read once a tick. */
    private WorldGround ground;

    @Override public Entity entity() { return this; }

    @Override public boolean beginTick(ServerLevel server) {
        if (isRemoved() || vehicle == null || state == null || fault != null) { return false; }
        lastGood = state.body();
        if (leaderId != null || !followers.isEmpty()) { links(server); }
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
        double turning = s.angularVelocity(vehicle.properties()).length() * reach;
        double travel = (s.velocity().length() + turning) * MinecraftUnits.TICK_SECONDS;
        fast = s.velocity().length() >= FAST_MPS;
        if (!(s.velocity().length() < MAX_MPS)) {
            // no vehicle goes this fast: its physics has run away (and its corridor would be kilometres of blocks)
            throw new IllegalStateException("Speed " + s.velocity().length() + " m/s is beyond any vehicle's");
        }
        if (fast) {
            corridor = path(s, turning);
            double blocks = 0;
            for (AABB box : corridor) { blocks += box.getXsize() * box.getYsize() * box.getZsize(); }
            if (blocks > MAX_CORRIDOR_BLOCKS) { throw new IllegalStateException("A corridor of " + (long) blocks + " blocks: the motion has run away"); }
            region = corridor.getFirst();
            for (AABB box : corridor) { region = region.minmax(box); }
            keepAhead(server, s);
            // the chunks are not ready: wait here, moving as before, until they are
            for (AABB box : corridor) { if (!loaded(box)) { publish(); return false; } }
            collision = new FloatingBodyDynamics.Collision(vehicle.collision(), WorldObstacles.collect(this, corridor), vehicle.contactMaterial());
        } else {
            region = envelope(s.position(), s.orientation()).inflate(2 * travel + 1);
            corridor = List.of(region);
            if (!loaded(region)) { publish(); return false; }   // wait for the chunks (0.28: the motion is kept, no longer zeroed)
            collision = new FloatingBodyDynamics.Collision(vehicle.collision(), WorldObstacles.collect(this, region), vehicle.contactMaterial());
        }
        startTime = server.getGameTime() * MinecraftUnits.TICK_SECONDS;
        Vec3d origin = vehicle.origin(s);
        waves = WaveRegions.of(server).at(server, origin.x(), origin.z());
        environment.setSea(waves);
        sampler.drifting(waves.region());
        entityLoads.beginTick();
        tally.clear();
        air.clear();
        if (ground == null) { ground = new WorldGround(server); }
        ground.clear();
        moving = true;
        return true;
    }

    @Override public boolean active() { return moving; }
    /** 0.28: a fast vehicle; 0.36: a joined one (its group steps when one of it stands in a ticking chunk). */
    @Override public boolean stepsWhereLoaded() { return state != null && (state.body().velocity().length() >= FAST_MPS || leaderId != null || !followers.isEmpty()); }
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
                // Surface-piercing floats take their heave diffraction from the hemisphere table (SpherePart).
                return new FloatingBodyDynamics.Contact(c.fraction(), c.wetCentroidOffset(), air, environment.waterAt(c.currentMps()),
                        c.pressureFraction(), c.pressureCentroidOffset(), c.accelerationMps2(), c.fraction() > 0 && c.fraction() < 1);
            }
            @Override public HullDynamics.Water point(Vec3d p) {
                WorldWaterSampler.Column c = sampler.column(p, waves.compiled(), waves.stillWaterY(), time);
                FluidState air = airAt(p), liquid = environment.waterAt(c.currentMps());
                return new HullDynamics.Water(c.surfaceY(), c.slope(), c.stillY(), c.decayWavenumber(), liquid.velocity(), c.accelerationMps2(), liquid, air);
            }
            @Override public WaveField waves() { return field; }
            @Override public dev.mcmod.aerocore.api.ground.Ground ground() { return ground == null ? dev.mcmod.aerocore.api.ground.Ground.NONE : ground; }
        };
    }

    private FluidState airAt(Vec3d p) {
        long key = BlockPos.asLong((int) Math.floor(p.x()), (int) Math.floor(p.y()), (int) Math.floor(p.z()));
        return air.computeIfAbsent(key, k -> environment.airAt(p, seaLevel).fluid());
    }

    @Override public RigidBodyState prepareSubstep(int i) {
        double time = startTime + i * SUBSTEP;
        RigidBodyState s = state.body();
        lastGood = s;
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
        if (!contact.touches().isEmpty()) { tally.add(before.position(), contact.touches(), SUBSTEP); touched = true; }
        VehicleDynamics.Result result = VehicleDynamics.finish(pending, contact.state(), contact.wrench(), contact.forces(),
                contact.staticFrictionW() + contact.pairFrictionW(), collision);
        if (!WorldObstacles.contains(corridor, envelope(result.state().position(), result.state().orientation()))) {
            // Left the region whose obstacles were collected: never move into unchecked space. A slow vehicle stops there;
            // a fast one (its path turned more than predicted) waits for the next tick's corridor, keeping its motion.
            state.setBody(fast ? before : new RigidBodyState(before.position(), Vec3d.ZERO, before.orientation(), Vec3d.ZERO));
            moving = false;
            return;
        }
        forceOrigin = before.position();
        displayForces = result.forces();
    }

    @Override public void correctPose(RigidBodyState corrected) { state.setBody(corrected); }

    @Override public void endTick() {
        if (leaderId != null || !followers.isEmpty()) {
            // a joined vehicle keeps its chunk loaded while its group steps, so the group never parts at a chunk edge
            net.minecraft.world.level.ChunkPos chunk = chunkPosition();
            ((ServerLevel) level()).getChunkSource().addRegionTicket(PATH, chunk, 0, chunk);
        }
        VehicleContacts.Impact impact = tally.impact();
        VehicleContacts.Scrape scrape = tally.scrape(MinecraftUnits.TICK_SECONDS);
        publish(scrape);
        if (impact != null || scrape != null) {
            net.neoforged.neoforge.common.NeoForge.EVENT_BUS.post(new VehicleContacts.Contacted(this, impact, scrape));
            // a vehicle pressed into something (two jammed together) closes on it every tick: the players hear the first
            // impact of a run, and again only when it is struck at least twice as hard
            if (impact != null && (!impacting || impact.impulseNs() >= 2 * toldImpactNs)) {
                toldImpactNs = impact.impulseNs();
                net.neoforged.neoforge.network.PacketDistributor.sendToPlayersTrackingEntity(this, new VehicleContacts.ImpactPayload(getId(),
                        (float) impact.point().x(), (float) impact.point().y(), (float) impact.point().z(), (float) impact.impulseNs(),
                        (float) impact.approachMps(), (byte) impact.material(), impact.body()));
            }
        }
        impacting = impact != null;
        touching = touched;
        touched = false;
        List<String> asked = state.takeJettisons();
        if (!asked.isEmpty()) { castOff(asked); }
        BodyCollisions.pushOut(this);
        PlatformDebug.display(this, sampler, startTime);
    }

    /**
     * The physics threw (a part's loads, the drift, the contact solve on this vehicle's state...). The vehicle stops
     * where the last sound state left it and steps no more: at rest, shafts stopped, the parts' internal states back
     * at their start (a new state, since the failed step may have left them half written). The error is logged once
     * with the stack; fault() and /aerocore vehicle info tell why, and it is saved, so a reload does not restart a
     * vehicle that would fail again (resume() does). Stepping on from the broken state could only spread the error.
     */
    @Override public void physicsFailed(String phase, RuntimeException error) {
        RigidBodyState pose = lastGood != null ? lastGood : state.body();
        fault = phase + ": " + error;
        LOGGER.error("Vehicle {} ({}) at {} stopped: its physics failed in {}", getId(), blueprint.name(), pose.position(), phase, error);
        state = vehicle.newState(vehicle.origin(pose), pose.orientation(), SUBSTEP);
        moving = false;
        pending = null;
        displayForces = List.of();
        uncoupleAll(Couplings.Reason.FAULT);
        publish();
    }

    // ---------------------------------------------------------------- couplings (0.36)

    /**
     * Server: hangs follower on this vehicle by a coupling, when both are ready (built, no fault, tables warm), in the
     * same level, the follower hangs on nothing yet, this vehicle is not (through any chain) hanging on the follower,
     * and the two points are within the coupling's captureM. Posts Couplings.Coupled.
     */
    public Couplings.Result couple(PartVehicle follower, Coupling how) {
        if (level().isClientSide) { throw new IllegalStateException("Couple on the server"); }
        java.util.Objects.requireNonNull(how, "coupling");
        if (follower == this) { return Couplings.Result.CYCLE; }
        if (follower.level() != level()) { return Couplings.Result.OTHER_LEVEL; }
        if (!steppable() || !follower.steppable()) { return Couplings.Result.NOT_READY; }
        if (follower.leaderId != null) { return Couplings.Result.ALREADY; }
        PartVehicle up = this;
        for (int k = 0; up != null && k < Couplings.MOST_LINKS; k++) {
            if (up == follower) { return Couplings.Result.CYCLE; }
            up = up.leader();
        }
        double gap = how.geometry(state.body(), vehicle.properties(), follower.state.body(), follower.vehicle.properties()).gap().length();
        if (gap > how.captureM()) { return Couplings.Result.TOO_FAR; }
        follower.leaderId = getUUID();
        follower.coupling = how;
        follower.leaderSeen = state.body().position();
        follower.lastJoint = null;
        followers.put(follower.getUUID(), follower.state.body().position());
        follower.publish();
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.post(new Couplings.Coupled(this, follower, how));
        return Couplings.Result.OK;
    }

    /** Server: unhooks this vehicle from its leader (COMMAND); false when it hangs on none. */
    public boolean uncouple() { return uncouple(Couplings.Reason.COMMAND); }

    /** Server: unhooks this vehicle from its leader for a reason; posts Couplings.Uncoupled. */
    public boolean uncouple(Couplings.Reason reason) {
        if (level().isClientSide || leaderId == null) { return false; }
        PartVehicle leader = leader();
        UUID was = leaderId;
        if (leader != null) { leader.followers.remove(getUUID()); }
        leaderId = null;
        coupling = null;
        leaderSeen = null;
        lastJoint = null;
        waited.remove(was);
        publish();
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.post(new Couplings.Uncoupled(leader, this, was, getUUID(), reason));
        return true;
    }

    /** Server: unhooks this vehicle from its leader and its followers from it. */
    public void uncoupleAll(Couplings.Reason reason) {
        if (level().isClientSide) { return; }
        uncouple(reason);
        for (UUID id : List.copyOf(followers.keySet())) {
            PartVehicle f = byId(id);
            if (f != null && getUUID().equals(f.leaderId)) { f.uncouple(reason); continue; }
            followers.remove(id);
            waited.remove(id);
            net.neoforged.neoforge.common.NeoForge.EVENT_BUS.post(new Couplings.Uncoupled(this, f, getUUID(), id, reason));
        }
    }

    /** The vehicle this one hangs on, if it is loaded (server; the client's from the synced link). */
    public PartVehicle leader() {
        if (level().isClientSide) { return clientLeader >= 0 && level().getEntity(clientLeader) instanceof PartVehicle p ? p : null; }
        return leaderId == null ? null : byId(leaderId);
    }
    /** Server: the loaded vehicles hanging on this one. */
    public List<PartVehicle> followers() {
        List<PartVehicle> out = new ArrayList<>();
        for (UUID id : followers.keySet()) { PartVehicle f = byId(id); if (f != null) { out.add(f); } }
        return out;
    }
    /** How this vehicle hangs on its leader, or null (the client's as synced). */
    public Coupling coupling() { return level().isClientSide ? clientCoupling : coupling; }
    /** Server: the force its leader put on this vehicle at the coupling in the last substep (world, N; zero when none). */
    public Vec3d couplingForceN() { return lastJoint == null ? Vec3d.ZERO : lastJoint.forceN(); }
    /** Server: the articulation (the coupling's yaw angle, rad) at the last substep, 0 when none. */
    public double articulationRad() { return lastJoint == null ? 0 : lastJoint.yawRad(); }

    private boolean steppable() { return vehicle != null && state != null && fault == null && ready() && !isRemoved(); }

    private PartVehicle byId(UUID id) {
        return level() instanceof ServerLevel server && server.getEntity(id) instanceof PartVehicle p ? p : null;
    }

    /**
     * Once a tick before stepping: the links made good with the partners that are loaded (a follower that hangs on
     * another now is dropped; a leader that lost this follower takes it back), a gap beyond BREAK_M ends the link, the
     * leader's forwarded controls are copied, where each partner stands is remembered.
     */
    private void links(ServerLevel server) {
        for (UUID id : List.copyOf(followers.keySet())) {
            PartVehicle f = byId(id);
            if (f == null) { continue; }
            if (!getUUID().equals(f.leaderId)) { followers.remove(id); waited.remove(id); continue; }
            followers.put(id, f.position3());
            waited.remove(id);
        }
        if (leaderId == null) { return; }
        PartVehicle leader = byId(leaderId);
        if (leader == null) { return; }
        waited.remove(leaderId);
        leaderSeen = leader.position3();
        leader.followers.putIfAbsent(getUUID(), position3());
        if (coupling == null || leader.state == null || leader.vehicle == null) { return; }
        double gap = coupling.geometry(leader.state.body(), leader.vehicle.properties(), state.body(), vehicle.properties()).gap().length();
        if (gap > Couplings.BREAK_M) {
            LOGGER.warn("Vehicle {} came {} m off its leader {}: uncoupled", getId(), gap, leader.getId());
            uncouple(Couplings.Reason.BROKEN);
            return;
        }
        for (String channel : coupling.forwarded()) {
            Double value = leader.controls.get(channel);
            if (value != null) { controls.put(channel, value); } else { controls.remove(channel); }
        }
    }

    private Vec3d position3() { return state != null ? state.body().position() : new Vec3d(getX(), getY(), getZ()); }

    @Override public UUID leaderId() { return leaderId; }
    @Override public Coupling leaderCoupling() { return coupling; }
    @Override public java.util.Collection<UUID> followerIds() { return followers.keySet(); }
    @Override public void carried(ContactSolver.JointResult joint) { lastJoint = joint; }

    /** A missing partner whose last place is loaded is waited for PATIENCE_TICKS, then the link with it ends (REMOVED). */
    @Override public void groupWaits(ServerLevel level, java.util.Set<UUID> missing) {
        for (UUID id : missing) {
            Vec3d seen = id.equals(leaderId) ? leaderSeen : followers.get(id);
            if (seen != null && !level.hasChunkAt(BlockPos.containing(seen.x(), seen.y(), seen.z()))) { waited.remove(id); continue; }
            int ticks = waited.merge(id, 1, Integer::sum);
            if (ticks < Couplings.PATIENCE_TICKS) { continue; }
            waited.remove(id);
            if (id.equals(leaderId)) { uncouple(Couplings.Reason.REMOVED); }
            else if (followers.remove(id) != null) { net.neoforged.neoforge.common.NeoForge.EVENT_BUS.post(new Couplings.Uncoupled(this, null, getUUID(), id, Couplings.Reason.REMOVED)); }
        }
    }

    /** Removed for good (killed, discarded, another dimension): its links end; unloaded, they wait for it. */
    @Override public void remove(RemovalReason reason) {
        if (!level().isClientSide && reason != RemovalReason.UNLOADED_TO_CHUNK && reason != RemovalReason.UNLOADED_WITH_PLAYER) { uncoupleAll(Couplings.Reason.REMOVED); }
        super.remove(reason);
    }

    /**
     * Client: how far from its own interpolated place a follower is drawn so that its coupling point sits on its
     * leader's as the leader is drawn (each interpolated on its own, the kingpin would wander off the fifth wheel);
     * zero for a vehicle hanging on none, a leader not here, or a shift over a metre (a link about to break).
     */
    public Vec3d drawnShift(float partialTick) { return drawnShift(partialTick, 0); }

    private Vec3d drawnShift(float partialTick, int depth) {
        if (!level().isClientSide || clientCoupling == null || vehicle == null || depth > Couplings.MOST_LINKS) { return Vec3d.ZERO; }
        if (!(level().getEntity(clientLeader) instanceof PartVehicle leader) || leader.vehicle == null) { return Vec3d.ZERO; }
        net.minecraft.world.phys.Vec3 l = leader.getPosition(partialTick), at = getPosition(partialTick);
        Vec3d hitch = new Vec3d(l.x, l.y, l.z).add(leader.drawnShift(partialTick, depth + 1))
                .add(leader.visualOrientation(partialTick).rotate(clientCoupling.leaderPoint().subtract(leader.centerOfMassLocal())));
        Vec3d centre = hitch.subtract(visualOrientation(partialTick).rotate(clientCoupling.followerPoint().subtract(centerOfMassLocal())));
        Vec3d shift = centre.subtract(new Vec3d(at.x, at.y, at.z));
        return shift.length() < 1 ? shift : Vec3d.ZERO;
    }

    // ---------------------------------------------------------------- casting parts off (0.28)

    /**
     * Posted on NeoForge's bus (server) after a vehicle has cast parts off, asked by a part (PartStep.jettison) or by
     * castOff: each part that went (with those attached to it), where its mount was and how it was turned and moving,
     * and what its tanks held, so the vehicle mod can drop or spawn what fell. The vehicle has its new blueprint by then.
     */
    public static final class Jettisoned extends net.neoforged.bus.api.Event {
        /** A part that went: its blueprint entry, its mount's world position and orientation, its velocity there, its tanks (by tank name: kg and fuel). */
        public record Dropped(VehicleBlueprint.Entry entry, Vec3d position, Orientation orientation, Vec3d velocity, Map<String, Double> tankKg,
                              Map<String, String> tankFuel) {}

        private final PartVehicle vehicle;
        private final List<Dropped> parts;

        public Jettisoned(PartVehicle vehicle, List<Dropped> parts) { this.vehicle = vehicle; this.parts = List.copyOf(parts); }
        public PartVehicle vehicle() { return vehicle; }
        public List<Dropped> parts() { return parts; }
    }

    /**
     * Server: casts parts off (0.28) and what is attached to them: the vehicle goes on as its blueprint without them
     * (VehicleBlueprint.without), keeping its place and motion (VehicleState.separate); riders whose seats went get off;
     * Jettisoned is posted. Refused (false) when nothing would be left or the rest does not compile. The parts are the
     * compiled ones already in use (PartTypes caches them), so the rest steps on at once.
     */
    public boolean castOff(java.util.Collection<String> ids) {
        if (level().isClientSide || blueprint == null || vehicle == null || state == null || ids.isEmpty()) { return false; }
        VehicleBlueprint.Split split = blueprint.without(ids);
        if (split.removed().isEmpty() || split.rest().parts().isEmpty()) { return false; }
        Vehicle rest;
        try {
            rest = split.rest().compile();
        } catch (RuntimeException e) {
            LOGGER.warn("Vehicle {} cannot cast off {}: {}", getId(), split.removed(), e.getMessage());
            return false;
        }
        RigidBodyState s = state.body();
        RigidBodyProperties p = vehicle.properties();
        Vec3d omega = s.angularVelocity(p);
        List<Jettisoned.Dropped> dropped = new ArrayList<>();
        java.util.Set<Integer> goneSeats = new java.util.HashSet<>();
        int seat = 0;
        for (Vehicle.Installed i : vehicle.installed()) {
            boolean gone = split.removed().contains(i.id());
            for (int k = 0; k < i.part().seats().size(); k++, seat++) { if (gone) { goneSeats.add(seat); } }
            if (!gone) { continue; }
            Vec3d at = s.worldPoint(p, i.mount().position());
            Vec3d velocity = s.velocity().add(omega.cross(at.subtract(s.position())));
            Map<String, Double> kg = new LinkedHashMap<>();
            Map<String, String> fuel = new LinkedHashMap<>();
            for (int k = 0; k < vehicle.tanks().size(); k++) {
                Vehicle.Tank t = vehicle.tanks().get(k);
                if (t.part() == i.index()) { kg.put(t.tank().name(), state.tankKg(k)); fuel.put(t.tank().name(), state.tankFuel(k)); }
            }
            dropped.add(new Jettisoned.Dropped(blueprint.part(i.id()), at, s.orientation().multiply(i.rotation()), velocity, kg, fuel));
        }
        List<Entity> riders = getPassengers();
        for (int k = riders.size() - 1; k >= 0; k--) { if (goneSeats.contains(k)) { riders.get(k).stopRiding(); } }
        VehicleState next = state.separate(rest);
        rest.warmUp();
        blueprint = split.rest();
        vehicle = rest;
        state = next;
        warm = null;
        pending = null;
        entityData.set(BLUEPRINT, Json.write(blueprint.toJson()));
        publish();
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.post(new Jettisoned(this, dropped));
        return true;
    }

    /**
     * A fast vehicle's corridor for this tick: its envelope swept from substep to substep along the path its velocity
     * and gravity predict, each box widened by a margin for what the forces and the turning may change (1 m, its tips'
     * travel by rotation in the tick, and 2 g of unpredicted acceleration over the tick).
     */
    private List<AABB> path(RigidBodyState s, double turning) {
        AABB here = envelope(s.position(), s.orientation());
        double dt = SUBSTEP, tick = MinecraftUnits.TICK_SECONDS;
        double margin = 1 + Math.min(turning * tick, MAX_TURN_MARGIN) + 0.5 * 2 * Atmosphere.GRAVITY_MPS2 * tick * tick;
        List<AABB> boxes = new ArrayList<>();
        Vec3d from = Vec3d.ZERO;
        for (int k = 1; k <= SUBSTEPS; k++) {
            double t = k * dt;
            Vec3d to = s.velocity().scale(t).add(GRAVITY.scale(0.5 * t * t));
            boxes.add(here.move(from.x(), from.y(), from.z()).minmax(here.move(to.x(), to.y(), to.z())).inflate(margin));
            from = to;
        }
        return boxes;
    }

    /** Server: renews short tickets on the chunks along a fast vehicle's path for the next LOOK_AHEAD_S (at most MAX_AHEAD_CHUNKS). */
    private void keepAhead(ServerLevel server, RigidBodyState s) {
        double speed = s.velocity().length(), step = 8;
        AABB box = envelope(s.position(), s.orientation());
        double half = Math.max(box.getXsize(), box.getZsize()) / 2 + 2;   // the corridor's half width, and its margin
        int samples = (int) Math.min(speed * LOOK_AHEAD_S / step, 4 * MAX_AHEAD_CHUNKS);
        java.util.Set<Long> done = new java.util.HashSet<>();
        for (int k = 0; k <= samples && done.size() < MAX_AHEAD_CHUNKS; k++) {
            double t = k * step / speed;
            Vec3d at = s.position().add(s.velocity().scale(t)).add(GRAVITY.scale(0.5 * t * t));
            // the chunks under the corridor's width there (one, or two or four at their edges)
            for (int c = 0; c < 4; c++) {
                double x = at.x() + ((c & 1) == 0 ? -half : half), z = at.z() + ((c & 2) == 0 ? -half : half);
                net.minecraft.world.level.ChunkPos chunk = new net.minecraft.world.level.ChunkPos(BlockPos.containing(x, at.y(), z));
                if (done.add(chunk.toLong())) { server.getChunkSource().addRegionTicket(PATH, chunk, 0, chunk); }
            }
        }
    }

    private boolean loaded(AABB box) {
        return !(level() instanceof ServerLevel server) || server.hasChunksAt(BlockPos.containing(box.minX, box.minY, box.minZ), BlockPos.containing(box.maxX, box.maxY, box.maxZ));
    }

    private void publish() { publish(null); }

    private void publish(VehicleContacts.Scrape scrape) {
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
        double[] values = state.shownValues();
        if (values.length > 0) {
            // a tenth of a millimetre or milliradian: changes below that are not worth a packet
            List<Object> list = new ArrayList<>();
            for (double value : values) { list.add(Math.rint(value * 1e4) / 1e4); }
            display.put("s", list);
        }
        if (!vehicle.tanks().isEmpty()) {
            List<Object> tanks = new ArrayList<>();
            for (int k = 0; k < vehicle.tanks().size(); k++) { tanks.add(List.of(Math.rint(state.tankKg(k) * 10) / 10, state.tankFuel(k))); }
            display.put("f", tanks);
        }
        // 0.37: the true velocity (2 cm/s steps: one at rest sends nothing new) and scraping
        Vec3d v = s.velocity();
        display.put("v", List.of(Math.rint(v.x() * 50) / 50, Math.rint(v.y() * 50) / 50, Math.rint(v.z() * 50) / 50));
        if (scrape != null) {
            Vec3d at = scrape.point().subtract(s.position());
            display.put("k", List.of(Math.rint(scrape.powerW()), Math.rint(scrape.slideMps() * 10) / 10,
                    Math.rint(at.x() * 10) / 10, Math.rint(at.y() * 10) / 10, Math.rint(at.z() * 10) / 10, scrape.material(), scrape.body() ? 1 : 0));
        }
        String text = Json.write(display);
        if (!text.equals(published)) { published = text; entityData.set(DISPLAY, text); }
        PartVehicle leader = coupling == null ? null : leader();
        String link = leader == null ? "" : leader.getId() + " " + coupling.write();
        if (!link.equals(publishedLink)) { publishedLink = link; entityData.set(LINK, link); }
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

    /**
     * Client: the server's position. A slow vehicle glides to it over the steps; a fast one (0.28) is drawn where it is
     * by now, the position carried forward by its synced velocity each tick (it would otherwise lag two ticks: 170 m at
     * Mach 5), until the next one comes.
     */
    @Override public void lerpTo(double x, double y, double z, float yaw, float pitch, int steps) {
        targetPosition = new Vec3(x, y, z);
        lerpSteps = Math.max(1, steps);
        targetTick = tickCount;
        targetKnown = true;
    }
    private int targetTick;
    /** 0.37: whether targetPosition is a position yet (it starts at the world's origin). */
    private boolean targetKnown;
    @Override public boolean isPushedByFluid() { return false; }
    @Override public boolean isPushedByFluid(FluidType type) { return false; }
    @Override public boolean isPickable() { return !isRemoved(); }
    @Override protected MovementEmission getMovementEmission() { return MovementEmission.NONE; }

    // ---------------------------------------------------------------- riders

    @Override protected boolean canAddPassenger(Entity passenger) { return vehicle != null && getPassengers().size() < vehicle.seats().size(); }

    /**
     * 0.30: how far below a seat's point (its cushion) a rider's own position is, along the vehicle's up: a sitting
     * rider's hips are on the cushion (a player's legs are 0.75 m long), so the eyes are about 0.9 m over it, under a
     * car's roof and behind a cockpit's windscreen. Before 0.30 the feet were on the cushion and the head through the roof.
     */
    public static final double SEATED_HIP_M = 0.72;

    @Override protected void positionRider(Entity passenger, MoveFunction move) {
        int index = getPassengers().indexOf(passenger);
        if (vehicle == null || index < 0 || index >= vehicle.seats().size()) { super.positionRider(passenger, move); return; }
        Vec3d seat = toWorld(vehicle.seats().get(index).subtract(new Vec3d(0, SEATED_HIP_M, 0)));
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

    /** 0.30: each wake-making part's design JSON (the WAKES key), by part. */
    private static final Map<dev.mcmod.aerocore.api.vehicle.Part, String> WAKE_KEYS = new java.util.WeakHashMap<>();

    /** The first part with a wake (Part.wake): Michell thin-ship sources for a displacement hull, a pressure patch for a planing one. */
    @Override public Wakes.Design wakeDesign() {
        if (vehicle == null) { return null; }
        for (Vehicle.Installed installed : vehicle.installed()) {
            WakeSource source = installed.part().wake();
            if (source instanceof WakeSource.ThinShip ship) {
                PartDesign design = installed.part().design();
                // 0.30: the design's JSON (the wake's key) written once a part, not on every call (it was every tick)
                String json;
                synchronized (WAKE_KEYS) { json = WAKE_KEYS.computeIfAbsent(installed.part(), part -> Json.write(design.toJson())); }
                return WAKES.computeIfAbsent(json, key -> java.util.concurrent.CompletableFuture.supplyAsync(() -> {
                    var hull = ship.hull();
                    return Wakes.thinShip(hull, hull.particulars(hull.waterlineFor(ship.designVolumeM3())));
                }, WARM_UP).exceptionally(failed -> {
                    LOGGER.warn("No wake for hull design {}: {}", design.name(), failed.toString());
                    return null;
                })).getNow(null);
            }
            if (source instanceof WakeSource.Planing planing) {
                Vec3d ahead = poseOrientation().rotate(installed.rotation().rotate(Vec3d.X));
                double trim = Math.asin(Math.clamp(ahead.y(), -1, 1)), wetted = 0;
                if (state != null) {
                    Map<String, Double> d = state.diagnostics().get(installed.id());
                    if (d != null) { wetted = d.getOrDefault(planing.wettedLengthDiagnostic(), 0.0); }
                }
                return Wakes.planingBoat(planing.chineBeamM(), planing.deadriseRad(), wetted, vehicle.properties().massKg() * Atmosphere.GRAVITY_MPS2, trim);
            }
        }
        return null;
    }
    private Vehicle.Installed wakePart() {
        for (Vehicle.Installed installed : vehicle.installed()) {
            if (installed.part().wake() != null) { return installed; }
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
        if (fault != null) { tag.putString("Fault", fault); }
        CompoundTag c = new CompoundTag();
        controls.forEach(c::putDouble);
        tag.put("Controls", c);
        if (leaderId != null && coupling != null) {
            CompoundTag link = place(leaderSeen);
            link.putUUID("Leader", leaderId);
            link.putString("Spec", coupling.write());
            tag.put("Coupling", link);
        }
        if (!followers.isEmpty()) {
            net.minecraft.nbt.ListTag towing = new net.minecraft.nbt.ListTag();
            followers.forEach((id, seen) -> { CompoundTag f = place(seen); f.putUUID("Id", id); towing.add(f); });
            tag.put("Towing", towing);
        }
    }

    private static CompoundTag place(Vec3d at) {
        CompoundTag t = new CompoundTag();
        if (at != null) { t.putDouble("X", at.x()); t.putDouble("Y", at.y()); t.putDouble("Z", at.z()); }
        return t;
    }
    private static Vec3d place(CompoundTag t) { return t.contains("X") ? new Vec3d(t.getDouble("X"), t.getDouble("Y"), t.getDouble("Z")) : null; }

    @Override protected void readAdditionalSaveData(CompoundTag tag) {
        owner = tag.hasUUID("Owner") ? tag.getUUID("Owner") : null;
        fault = tag.contains("Fault") ? tag.getString("Fault") : null;
        String json = new String(tag.getByteArray("Blueprint"), StandardCharsets.UTF_8);
        controls.clear();
        CompoundTag c = tag.getCompound("Controls");
        for (String key : c.getAllKeys()) { controls.put(key, c.getDouble(key)); }
        leaderId = null;
        coupling = null;
        leaderSeen = null;
        followers.clear();
        if (tag.contains("Coupling")) {
            CompoundTag link = tag.getCompound("Coupling");
            try {
                coupling = Coupling.parse(link.getString("Spec"));
                leaderId = link.getUUID("Leader");
                leaderSeen = place(link);
            } catch (RuntimeException e) {
                coupling = null;
                LOGGER.warn("Vehicle at {} has a coupling that cannot be read: {}", blockPosition(), e.getMessage());
            }
        }
        for (net.minecraft.nbt.Tag t : tag.getList("Towing", net.minecraft.nbt.Tag.TAG_COMPOUND)) {
            CompoundTag f = (CompoundTag) t;
            if (f.hasUUID("Id")) { followers.put(f.getUUID("Id"), place(f)); }
        }
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
