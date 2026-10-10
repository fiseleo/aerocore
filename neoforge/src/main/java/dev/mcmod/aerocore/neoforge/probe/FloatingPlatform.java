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
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.UUID;

/**
 * Temporary six-DOF demonstrator, stepped by BodyWorld. Contact uses sphere-based impulses against blocks, vanilla
 * collidable entities and the world border (which are never pushed) and, two-way, against other AeroCore bodies.
 * The stern propeller is a demo thruster.
 */
public final class FloatingPlatform extends Entity implements DebugBody, CoupledBody, SolidBody {
    /** Solid boxes for vanilla entities: the deck and cubes of half-size 0.3 m around the floats (ballast added per pose). */
    private static final List<WorldBox> DECK_AND_FLOATS = deckAndFloats();
    private final EntityLoads entityLoads=new EntityLoads();
    private static final EntityDataAccessor<Quaternionf> ROTATION = SynchedEntityData.defineId(FloatingPlatform.class,EntityDataSerializers.QUATERNION);
    private static final EntityDataAccessor<Vector3f> BALLAST = SynchedEntityData.defineId(FloatingPlatform.class,EntityDataSerializers.VECTOR3);
    private static final EntityDataAccessor<Float> PROPELLER = SynchedEntityData.defineId(FloatingPlatform.class,EntityDataSerializers.FLOAT);
    private static final int SUBSTEPS=10;
    public static final double MAX_PROPELLER_RPS=30;
    private static final Vec3d GRAVITY=Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private Vec3d ballast=TestPlatform.DEFAULT_BALLAST;
    private RigidBodyProperties properties=TestPlatform.properties(ballast);
    private RigidBodyState state=new RigidBodyState(Vec3d.ZERO,Vec3d.ZERO,Orientation.IDENTITY,Vec3d.ZERO);
    private List<Double> wet=List.of(0.0,0.0,0.0,0.0);
    private Wrench fluidLoad=Wrench.ZERO,contactLoad=Wrench.ZERO;
    private int contactPoints;
    private final PlatformEnvironment environment=new PlatformEnvironment();
    /** World wave region at the model origin, refreshed each tick; its taper factor is held across the platform. */
    private WaveRegions.Local waves=WaveRegions.Local.CALM;
    /** Heave wave-radiation memory of the four floats (fixed horizontal layout), one substep resolution. */
    private final WaveRadiation.Memory radiation=new WaveRadiation.Memory(
            TestPlatform.FLOATS.stream().map(BuoyantSphere::offset).toList(),MinecraftUnits.TICK_SECONDS/SUBSTEPS);
    private HeatTransfer.Body thermalBody=new HeatTransfer.Body(293.15,TestPlatform.HEAT_CAPACITY);
    private double heatRateW,dragDissipationW,frictionHeatW,contactFrictionW,contactHeatW;
    private double propellerRps;
    private Thrust thrust=Thrust.IDLE;
    private int debugMode;
    private List<AppliedForce> displayForces=List.of();
    private Vec3d forceOrigin=Vec3d.ZERO;
    private UUID owner;
    private int lerpSteps;
    private Vec3 targetPosition=Vec3.ZERO;
    private Orientation clientRotation=Orientation.IDENTITY,previousClientRotation=Orientation.IDENTITY;

    /** Tick-averaged propeller report; the force is the last substep's, as integrated. */
    private record Thrust(AppliedForce force,double thrustN,double torqueNm,double advanceMps,double shaftPowerW,double immersion) {
        static final Thrust IDLE=new Thrust(null,0,0,0,0,0);
    }

    public FloatingPlatform(EntityType<? extends FloatingPlatform> type,Level level) {
        super(type,level);
        setNoGravity(true);
    }
    @Override protected void defineSynchedData(SynchedEntityData.Builder builder) {
        builder.define(ROTATION,new Quaternionf());
        builder.define(BALLAST,new Vector3f(0,-0.35f,0));
        builder.define(PROPELLER,0f);
    }
    @Override public Entity entity() { return this; }
    public UUID owner() { return owner; }
    public RigidBodyState physicsState() { return state; }
    public RigidBodyProperties properties() { return properties; }
    public List<Double> wetFractions() { return wet; }
    public PlatformEnvironment environment() {return environment;}
    public WaveRegions.Local waves() {return waves;}
    public int debugMode() {return debugMode;}
    public void setDebugMode(int mode) {
        if(mode<0 || mode>7) {throw new IllegalArgumentException("Invalid debug mode");}
        debugMode=mode;
    }
    public List<AppliedForce> displayForces() {return displayForces;}
    public Vec3d forceOrigin() {return forceOrigin;}
    public Wrench contactLoad() {return contactLoad;}
    public double dragDissipationW() {return dragDissipationW;}
    public double frictionHeatW() {return frictionHeatW;}
    public double contactFrictionW() {return contactFrictionW;}
    public double contactHeatW() {return contactHeatW;}
    public double propellerThrustN() {return thrust.thrustN();}
    public double propellerRps() {return level().isClientSide?entityData.get(PROPELLER):propellerRps;}
    public void setPropellerRps(double value) {
        if(!Double.isFinite(value)||Math.abs(value)>MAX_PROPELLER_RPS) {throw new IllegalArgumentException("Propeller speed outside [-30,30] rev/s");}
        propellerRps=value;
        publish();
    }
    public double bodyTemperatureCelsius() {return thermalBody.temperatureKelvin()-273.15;}
    public void setBodyTemperatureCelsius(double value) {
        if(!Double.isFinite(value)||value < -50||value>250) {throw new IllegalArgumentException("Body temperature outside [-50,250] C");}
        thermalBody=new HeatTransfer.Body(value+273.15,TestPlatform.HEAT_CAPACITY);
    }
    public Vec3d ballastLocal() {
        if (!level().isClientSide) { return ballast; }
        Vector3f b=entityData.get(BALLAST);
        return new Vec3d(b.x,b.y,b.z);
    }
    public Orientation visualOrientation(float partialTick) {
        return level().isClientSide ? previousClientRotation.interpolate(clientRotation,Math.clamp(partialTick,0,1)) : state.orientation();
    }
    public Vec3d modelOrigin() { return state.worldPoint(properties,Vec3d.ZERO); }
    public void configure(UUID owner,Vec3d origin) {
        this.owner=owner;
        state=TestPlatform.atModelOrigin(origin,Orientation.IDENTITY,properties);
        setCustomName(Component.translatable("entity.aerocore.floating_platform"));
        setCustomNameVisible(true);
        publish();
    }
    /** Experimental reconfiguration: keep visible frame pose, reset motion; external work is not simulated. */
    public boolean reconfigure(Vec3d newBallast,Orientation orientation) {
        if (Math.abs(newBallast.x())>3 || newBallast.y() < -2 || newBallast.y()>10 || Math.abs(newBallast.z())>3) {
            throw new IllegalArgumentException("Ballast outside demo range");
        }
        RigidBodyProperties nextProperties=TestPlatform.properties(newBallast);
        RigidBodyState next=TestPlatform.atModelOrigin(modelOrigin(),orientation,nextProperties);
        AABB envelope=envelope(next.position(),next.orientation(),nextProperties,newBallast);
        if (!level().getWorldBorder().isWithinBounds(envelope) || !level().noCollision(this,envelope) || !loaded(envelope)) { return false; }
        ballast=newBallast; properties=nextProperties; state=next;
        publish();
        return true;
    }
    public void setAngularVelocity(Vec3d worldRadiansPerSecond) {
        state=new RigidBodyState(state.position(),state.velocity(),state.orientation(),properties.inertia().applyWorld(state.orientation(),worldRadiansPerSecond));
        publish();
    }
    public void setVelocityMps(Vec3d velocity) {
        state=new RigidBodyState(state.position(),velocity,state.orientation(),state.angularMomentum());
        publish();
    }
    @Override public void tick() {
        super.tick();
        if (level().isClientSide) {
            previousClientRotation=clientRotation;
            Quaternionf q=entityData.get(ROTATION);
            clientRotation=clientRotation.interpolate(new Orientation(q.w,q.x,q.y,q.z),0.5);
            if (lerpSteps>0) { setPos(position().lerp(targetPosition,1.0/lerpSteps--)); }
            setBoundingBox(makeBoundingBox());
            BodyCollisions.trackClient(this);
            BodyCollisions.pushOut(this);
            return;
        }
        // Physics runs in BodyWorld at the start of the level tick, jointly with the other bodies.
        if (!isRemoved() && (tickCount>=PhysicsProbe.LIFETIME_TICKS || getY()<level().getMinBuildHeight()-20)) { discard(); }
    }

    // ---- CoupledBody: one tick in SUBSTEPS phases, driven by BodyWorld ----
    private WorldWaterSampler sampler;
    private int seaLevel;
    private double startTime;
    private AABB region;
    private FloatingBodyDynamics.Collision collision;
    private boolean moving;
    private FloatingBodyDynamics.Prepared pending;
    private Thrust pendingThrust;
    private List<AppliedForce> pendingRadiation=List.of();
    private Wrench fluidSum=Wrench.ZERO,contactSum=Wrench.ZERO;
    private double airPower,waterPower,airSquare,waterSquare,staticFriction,pairFriction;
    private double thrustSum,torqueSum,advanceSum,powerSum,immersionSum;
    private AppliedForce lastThrust;
    private int steps;

    @Override public boolean beginTick(ServerLevel server) {
        if (isRemoved()) { return false; }
        sampler=new WorldWaterSampler(server);
        seaLevel=server.getChunkSource().getGenerator().getSeaLevel();
        List<CollisionSphere> shape=TestPlatform.collisionShape(ballast);
        // Obstacles are collected once per tick around everything the body can reach in two ticks.
        double reach=furthestReach(properties,shape);
        double travel=(state.velocity().length()+state.angularVelocity(properties).length()*reach)*MinecraftUnits.TICK_SECONDS;
        region=envelope(state.position(),state.orientation(),properties,ballast).inflate(2*travel+1);
        if (!loaded(region)) { stop(); publish(); return false; }
        collision=new FloatingBodyDynamics.Collision(shape,WorldObstacles.collect(this,region),TestPlatform.CONTACT_MATERIAL);
        startTime=server.getGameTime()*MinecraftUnits.TICK_SECONDS;
        Vec3d origin=modelOrigin();
        waves=WaveRegions.of(server).at(origin.x(),origin.z());
        environment.setSea(waves);
        sampler.drifting(waves.region());
        fluidSum=Wrench.ZERO; contactSum=Wrench.ZERO;
        airPower=waterPower=airSquare=waterSquare=staticFriction=pairFriction=0;
        thrustSum=torqueSum=advanceSum=powerSum=immersionSum=0;
        lastThrust=null; steps=0; moving=true;
        entityLoads.beginTick();
        return true;
    }
    // ---- SolidBody: vanilla entities stand on and push the deck ----
    @Override public Vec3d poseCenter() { return level().isClientSide?new Vec3d(getX(),getY(),getZ()):state.position(); }
    @Override public Orientation poseOrientation() { return level().isClientSide?clientRotation:state.orientation(); }
    @Override public Vec3d centerOfMassLocal() { return level().isClientSide?TestPlatform.properties(ballastLocal()).centerOfMassLocal():properties.centerOfMassLocal(); }
    @Override public List<WorldBox> solidBoxesLocal() {
        List<WorldBox> boxes=new ArrayList<>(DECK_AND_FLOATS);
        Vec3d b=ballastLocal(),h=new Vec3d(0.2,0.2,0.2);
        boxes.add(new WorldBox(b.subtract(h),b.add(h)));
        return boxes;
    }
    @Override public Vec3d velocityAt(Vec3d point) {
        return level().isClientSide?Vec3d.ZERO:state.velocity().add(state.angularVelocity(properties).cross(point.subtract(state.position())));
    }
    @Override public void addEntityLoad(Vec3d localPoint,Vec3d worldForce) { if(!level().isClientSide) {entityLoads.add(localPoint,worldForce);} }
    private static List<WorldBox> deckAndFloats() {
        List<WorldBox> boxes=new ArrayList<>();
        boxes.add(new WorldBox(new Vec3d(-1.2,0.05,-1.2),new Vec3d(1.2,0.25,1.2)));
        for(BuoyantSphere f:TestPlatform.FLOATS) {boxes.add(new WorldBox(f.offset().subtract(new Vec3d(0.3,0.3,0.3)),f.offset().add(new Vec3d(0.3,0.3,0.3))));}
        return List.copyOf(boxes);
    }
    @Override public boolean active() { return moving; }
    @Override public List<CollisionSphere> collisionShape() { return collision.shape(); }
    @Override public ContactMaterial contactMaterial() { return TestPlatform.CONTACT_MATERIAL; }
    @Override public List<WorldBox> staticObstacles() { return collision.obstacles(); }
    @Override public AABB physicsEnvelope() { return envelope(state.position(),state.orientation(),properties,ballast); }
    @Override public RigidBodyState prepareSubstep(int i) {
        double dt=MinecraftUnits.TICK_SECONDS/SUBSTEPS, time=startTime+i*dt;
        FloatingBodyDynamics.Environment fluids=(center,radius)-> {
            WorldWaterSampler.Contact contact=sampler.sample(center,radius,waves.compiled(),waves.stillWaterY(),time);
            FluidState air=environment.airAt(center,seaLevel).fluid();
            // Surface-piercing floats take their heave diffraction from the radiation memory's hemisphere table.
            return new FloatingBodyDynamics.Contact(contact.fraction(),contact.wetCentroidOffset(),air,
                    environment.waterAt(contact.currentMps()),contact.pressureFraction(),contact.pressureCentroidOffset(),
                    contact.accelerationMps2(),contact.fraction()>0 && contact.fraction()<1);
        };
        pendingThrust=propeller(sampler,seaLevel,time);
        // Wave radiation from the floats' past heave (memory force) and heave diffraction, then the propeller.
        double rho=environment.waterThermal().fluid().densityKgPerM3();
        List<AppliedForce> radiationLoads=new ArrayList<>(radiation.loads(state,properties,TestPlatform.FLOATS,Vec3d.Y,rho));
        radiationLoads.addAll(radiation.diffractionLoads(state,properties,TestPlatform.FLOATS,rho,waves.waves(),time));
        pendingRadiation=radiationLoads;
        List<AppliedForce> loads=new ArrayList<>(radiationLoads);
        if(pendingThrust.force()!=null) {loads.add(pendingThrust.force());}
        loads.addAll(entityLoads.forces(state,properties));
        pending=FloatingBodyDynamics.prepare(state,properties,TestPlatform.FLOATS,fluids,GRAVITY,FloatingBodyDynamics.STANDARD_DRAG,loads,dt);
        return pending.damped();
    }
    @Override public void finishSubstep(int i, ContactSolver.BodyResult contact) {
        FloatingBodyDynamics.Result result=FloatingBodyDynamics.finish(pending,contact.state(),contact.wrench(),contact.forces(),
                contact.staticFrictionW()+contact.pairFrictionW(),collision);
        if (!WorldObstacles.contains(region,envelope(result.state().position(),result.state().orientation(),properties,ballast))) {
            // Left the region whose obstacles were collected; never move into unchecked space.
            stop(); moving=false; return;
        }
        forceOrigin=state.position();
        displayForces=result.forces();
        state=result.state();
        wet=result.wetFractions();
        radiation.record(state,properties,TestPlatform.FLOATS,Vec3d.Y,wet);
        fluidSum=fluidSum.add(result.hydrostatics()).add(result.averageDrag()).add(result.inertia());
        for(AppliedForce load:pendingRadiation) {fluidSum=fluidSum.add(load.wrench());}
        contactSum=contactSum.add(result.contact());
        staticFriction+=contact.staticFrictionW(); pairFriction+=contact.pairFrictionW();
        contactPoints=(int)result.forces().stream().filter(f->f.kind()==AppliedForce.Kind.CONTACT).count();
        FloatingBodyDynamics.DragDissipation d=result.dissipation();
        airPower+=d.airW(); waterPower+=d.waterW(); airSquare+=d.airMeanSquareSpeed(); waterSquare+=d.waterMeanSquareSpeed();
        Thrust propeller=pendingThrust;
        thrustSum+=propeller.thrustN(); torqueSum+=propeller.torqueNm(); advanceSum+=propeller.advanceMps();
        powerSum+=propeller.shaftPowerW(); immersionSum+=propeller.immersion(); lastThrust=propeller.force();
        steps++;
    }
    @Override public void correctPose(RigidBodyState corrected) { state=corrected; }
    @Override public void endTick() {
        setPos(state.position().x(),state.position().y(),state.position().z());
        setBoundingBox(envelope(state.position(),state.orientation(),properties,ballast));
        double n=Math.max(steps,1);
        fluidLoad=new Wrench(fluidSum.force().scale(1/n).subtract(GRAVITY.scale(properties.massKg())),fluidSum.torque().scale(1/n));
        contactLoad=new Wrench(contactSum.force().scale(1/n),contactSum.torque().scale(1/n));
        thrust=new Thrust(lastThrust,thrustSum/n,torqueSum/n,advanceSum/n,powerSum/n,immersionSum/n);
        double wetFraction=wet.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        // Convection from the floats (spheres): Whitaker forced and Churchill free convection at the tick's rms relative
        // speed in each medium, wet area approximated by the volume fraction; reservoirs at prescribed temperatures.
        ThermalFluidState air=environment.airAt(state.position(),seaLevel),water=environment.waterThermal();
        double diameter=2*TestPlatform.FLOAT_RADIUS,surface=thermalBody.temperatureKelvin(),g=Atmosphere.GRAVITY_MPS2;
        double airExpansion=2/(surface+air.temperatureKelvin());
        double airConductance=TestPlatform.FLOATS.size()*(1-wetFraction)
                *HeatTransfer.sphereConductance(air,airExpansion,diameter,Math.sqrt(airSquare/n),surface,g);
        double waterConductance=TestPlatform.FLOATS.size()*wetFraction
                *HeatTransfer.sphereConductance(water,waterExpansion(water),diameter,Math.sqrt(waterSquare/n),surface,g);
        dragDissipationW=(airPower+waterPower)/n;
        // Viscous heating: recovery-temperature excess per medium, never more than that medium's drag dissipation.
        frictionHeatW=HeatTransfer.frictionHeatingW(airConductance,air,airSquare/n,HeatTransfer.turbulentRecoveryFactor(air),airPower/n)
                +HeatTransfer.frictionHeatingW(waterConductance,water,waterSquare/n,HeatTransfer.turbulentRecoveryFactor(water),waterPower/n);
        // Contact friction: against blocks the steel-like frame takes its effusivity share and the ground keeps the rest;
        // against another body each takes half (like materials).
        contactFrictionW=(staticFriction+pairFriction)/n;
        contactHeatW=staticFriction/n*HeatTransfer.contactHeatPartition(TestPlatform.FRAME_EFFUSIVITY,TestPlatform.GROUND_EFFUSIVITY)+pairFriction/n;
        HeatTransfer.Result heat=HeatTransfer.reservoirs(thermalBody,List.of(
                new HeatTransfer.Reservoir(air.temperatureKelvin(),airConductance),
                new HeatTransfer.Reservoir(environment.waterKelvin(),waterConductance)),frictionHeatW+contactHeatW,MinecraftUnits.TICK_SECONDS);
        thermalBody=heat.body();heatRateW=heat.energyChangeJ()/MinecraftUnits.TICK_SECONDS;
        publish();
        BodyCollisions.pushOut(this);
        PlatformDebug.display(this,sampler,startTime);
    }
    /** Water and air loads are blended by the disk's sampled immersion; ventilation of single blades is not modeled. */
    private Thrust propeller(WorldWaterSampler sampler,int seaLevel,double time) {
        Propeller p=TestPlatform.PROPELLER;
        Vec3d hub=state.worldPoint(properties,p.positionLocal());
        WorldWaterSampler.Contact disk=sampler.sample(hub,p.diameterM()/2,waves.compiled(),waves.stillWaterY(),time);
        PropellerSolver.Load water=PropellerSolver.load(state,properties,p,propellerRps,environment.waterAt(disk.currentMps()));
        PropellerSolver.Load air=PropellerSolver.load(state,properties,p,propellerRps,environment.airAt(hub,seaLevel).fluid());
        double f=disk.fraction(),g=1-f;
        Vec3d force=water.force().force().scale(f).add(air.force().force().scale(g));
        Vec3d couple=water.force().couple().scale(f).add(air.force().couple().scale(g));
        AppliedForce load=force.lengthSquared()==0 && couple.lengthSquared()==0?null:
                new AppliedForce(AppliedForce.Kind.THRUST,water.force().offset(),force,couple);
        return new Thrust(load,f*water.thrustN()+g*air.thrustN(),f*water.torqueNm()+g*air.torqueNm(),
                f*water.advanceSpeedMps()+g*air.advanceSpeedMps(),f*water.shaftPowerW()+g*air.shaftPowerW(),f);
    }
    private void stop() { state=new RigidBodyState(state.position(),Vec3d.ZERO,state.orientation(),Vec3d.ZERO); }
    private void publish() {
        Orientation q=state.orientation();
        entityData.set(ROTATION,new Quaternionf((float)q.x(),(float)q.y(),(float)q.z(),(float)q.w()));
        entityData.set(BALLAST,new Vector3f((float)ballast.x(),(float)ballast.y(),(float)ballast.z()));
        entityData.set(PROPELLER,(float)propellerRps);
        setPos(state.position().x(),state.position().y(),state.position().z());
        setBoundingBox(envelope(state.position(),q,properties,ballast));
        setDeltaMovement(MinecraftUnits.toBlocksPerTick(state.velocity()));
        hasImpulse=true;
    }
    @Override protected AABB makeBoundingBox() {
        // Entity's constructor can invoke this before this class's fields have initialized.
        if (state==null) { return new AABB(getX()-1.3,getY()-0.6,getZ()-1.3,getX()+1.3,getY()+0.6,getZ()+1.3); }
        Vec3d b=ballastLocal();
        RigidBodyProperties p=level().isClientSide?TestPlatform.properties(b):properties;
        Orientation q=level().isClientSide?clientRotation:state.orientation();
        return envelope(new Vec3d(getX(),getY(),getZ()),q,p,b);
    }
    private boolean loaded(AABB box) {
        return !(level() instanceof ServerLevel server) || server.hasChunksAt(BlockPos.containing(box.minX,box.minY,box.minZ),BlockPos.containing(box.maxX,box.maxY,box.maxZ));
    }
    /** Union of the contact spheres' bounding boxes: floats, deck and ballast corners, propeller disk. */
    public static AABB envelope(Vec3d position,Orientation q,RigidBodyProperties p,Vec3d ballast) {
        Vec3d origin=position.subtract(q.rotate(p.centerOfMassLocal()));
        AABB result=null;
        for(CollisionSphere sphere:TestPlatform.collisionShape(ballast)) {
            Vec3d c=origin.add(q.rotate(sphere.centerLocal())); double r=sphere.radiusM();
            AABB a=new AABB(c.x()-r,c.y()-r,c.z()-r,c.x()+r,c.y()+r,c.z()+r);
            result=result==null?a:result.minmax(a);
        }
        return result;
    }
    private static double furthestReach(RigidBodyProperties p,List<CollisionSphere> shape) {
        double reach=0;
        for(CollisionSphere sphere:shape) {reach=Math.max(reach,sphere.centerLocal().subtract(p.centerOfMassLocal()).length()+sphere.radiusM());}
        return reach;
    }
    @Override public void lerpTo(double x,double y,double z,float yaw,float pitch,int steps) {
        targetPosition=new Vec3(x,y,z); lerpSteps=Math.max(1,steps);
    }
    @Override public boolean isPushedByFluid() { return false; }
    @Override public boolean isPushedByFluid(FluidType type) { return false; }
    @Override public boolean isPickable() { return !isRemoved(); }
    @Override protected MovementEmission getMovementEmission() { return MovementEmission.NONE; }
    @Override public InteractionResult interact(Player player,InteractionHand hand) {
        if(!level().isClientSide && hand==InteractionHand.MAIN_HAND) {
            for(Component line:report()) {player.sendSystemMessage(line);}
        }
        return InteractionResult.sidedSuccess(level().isClientSide);
    }
    public List<Component> report() {
        return List.of(diagnostics(),contactDiagnostics(),propellerDiagnostics(),waveDiagnostics(),thermalDiagnostics());
    }
    public Component diagnostics() {
        Vec3d omega=state.angularVelocity(properties);
        double tilt=Math.toDegrees(Math.acos(Math.clamp(state.orientation().rotate(Vec3d.Y).y(),-1,1)));
        return Component.translatable("command.aerocore.platform.info",getId(),f(tilt),f(state.velocity().length()),
                vector(omega),vector(properties.centerOfMassLocal()),vector(ballast),
                f(wet.get(0)*100),f(wet.get(1)*100),f(wet.get(2)*100),f(wet.get(3)*100),
                f(fluidLoad.force().y()),vector(fluidLoad.torque()));
    }
    public Component contactDiagnostics() {
        ContactMaterial m=TestPlatform.CONTACT_MATERIAL;
        return Component.translatable("command.aerocore.platform.contact.info",contactPoints,vector(contactLoad.force()),
                vector(contactLoad.torque()),f(m.friction()),f(m.restitutionAt(1)),f(m.restitutionAt(5)));
    }
    public Component propellerDiagnostics() {
        double d=TestPlatform.PROPELLER.diameterM();
        String ratio=propellerRps==0?"-":f(thrust.advanceMps()/(propellerRps*d));
        return Component.translatable("command.aerocore.platform.propeller.info",f(propellerRps),f(thrust.advanceMps()),ratio,
                f(thrust.thrustN()),f(thrust.torqueNm()),f(thrust.shaftPowerW()),f(thrust.immersion()*100));
    }
    public Component waveDiagnostics() {
        WaveRegion region=waves.region();
        if(region==null || region.waves().isEmpty()) {return Component.translatable("command.aerocore.platform.waves.calm");}
        LinearWave w=region.waves().components().getFirst();
        Vec3d origin=modelOrigin();
        WaveRegion.Sea sea=region.sea();
        if(sea!=null) {
            return Component.translatable("command.aerocore.platform.waves.sea",f(sea.significantHeightM()),f(sea.peakPeriodS()),f(sea.gamma()),
                    f(Math.toDegrees(sea.directionRadians())),f(sea.spreading()),region.waves().components().size(),f(w.depthM()),
                    f(region.stillWaterY()),f(region.radiusM()),f(region.weight(origin.x(),origin.z())));
        }
        return Component.translatable("command.aerocore.platform.waves.info",f(w.amplitudeM()),f(w.wavelengthM()),f(w.periodS()),
                f(w.phaseSpeedMps()),f(Math.toDegrees(w.directionRadians())),f(w.depthM()),f(region.stillWaterY()),
                f(region.radiusM()),f(region.weight(origin.x(),origin.z())));
    }
    /** Re-reads the wave region at the current position, for commands issued between ticks. */
    public void refreshWaves() {
        if(level() instanceof ServerLevel server) {Vec3d o=modelOrigin();waves=WaveRegions.of(server).at(o.x(),o.z());}
    }
    public Component thermalDiagnostics() {
        int seaLevel=level() instanceof ServerLevel server?server.getChunkSource().getGenerator().getSeaLevel():63;
        ThermalFluidState air=environment.airAt(state.position(),seaLevel);
        FluidState water=environment.waterAt(Vec3d.ZERO);
        return Component.translatable("command.aerocore.platform.thermal.info",f(bodyTemperatureCelsius()),
                f(air.temperatureKelvin()-273.15),f(environment.waterKelvin()-273.15),f(air.pressurePa()/1000),
                f(air.fluid().densityKgPerM3()),f(air.fluid().dynamicViscosityPaS()*1e6),
                f(water.densityKgPerM3()),f(water.dynamicViscosityPaS()*1000),f(heatRateW),f(dragDissipationW),f(frictionHeatW),
                f(contactFrictionW),f(contactHeatW));
    }
    private static String vector(Vec3d v) {return "("+f(v.x())+", "+f(v.y())+", "+f(v.z())+")";}
    private static String f(double value) {return String.format(Locale.ROOT,"%.3f",value);}
    @Override protected void addAdditionalSaveData(CompoundTag tag) {
        if(owner!=null) {tag.putUUID("Owner",owner);}
        tag.putDouble("BallastX",ballast.x()); tag.putDouble("BallastY",ballast.y()); tag.putDouble("BallastZ",ballast.z());
        Orientation q=state.orientation();
        tag.putDouble("Qw",q.w()); tag.putDouble("Qx",q.x()); tag.putDouble("Qy",q.y()); tag.putDouble("Qz",q.z());
    }
    @Override protected void readAdditionalSaveData(CompoundTag tag) {
        owner=tag.hasUUID("Owner")?tag.getUUID("Owner"):null;
        try {
            Vec3d b=new Vec3d(tag.getDouble("BallastX"),tag.getDouble("BallastY"),tag.getDouble("BallastZ"));
            ballast=new Vec3d(Math.clamp(b.x(),-3,3),Math.clamp(b.y(),-2,10),Math.clamp(b.z(),-3,3));
            properties=TestPlatform.properties(ballast);
            Orientation q=new Orientation(tag.getDouble("Qw"),tag.getDouble("Qx"),tag.getDouble("Qy"),tag.getDouble("Qz"));
            state=new RigidBodyState(new Vec3d(getX(),getY(),getZ()),Vec3d.ZERO,q,Vec3d.ZERO);
        } catch(IllegalArgumentException invalid) {
            ballast=TestPlatform.DEFAULT_BALLAST;properties=TestPlatform.properties(ballast);
            state=new RigidBodyState(new Vec3d(getX(),getY(),getZ()),Vec3d.ZERO,Orientation.IDENTITY,Vec3d.ZERO);
        }
        publish();
    }
    /** Volumetric expansion coefficient of liquid water, -(1/rho) d rho/dT, from the water property model. */
    private static double waterExpansion(ThermalFluidState water) {
        double t=Math.clamp(water.temperatureKelvin(),273.65,352.65),p=water.pressurePa();
        double up=Thermodynamics.liquidWater(t+0.5,p,Vec3d.ZERO).fluid().densityKgPerM3(),down=Thermodynamics.liquidWater(t-0.5,p,Vec3d.ZERO).fluid().densityKgPerM3();
        return -(up-down)/(0.5*(up+down));
    }
}
