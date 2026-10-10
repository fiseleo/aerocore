package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.*;
import net.minecraft.core.BlockPos;
import net.minecraft.core.particles.DustParticleOptions;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.entity.Entity;
import org.joml.Vector3f;

/**
 * Owner-only sampled-field/force arrows of a test body (platform or hull), built from short-lived vanilla dust
 * particles. No fluid solver.
 */
public final class PlatformDebug {
    public static final int FORCES=1,FLOW=2;
    private static final DustParticleOptions WEIGHT=color(1,0.2f,0.2f),BUOYANCY=color(0.2f,1,1),
            DRAG=color(1,0.8f,0.1f),CONTACT=color(1,0.25f,0.85f),THRUST=color(1,0.45f,0),INERTIA=color(0.6f,0.5f,1),RADIATION=color(0.1f,0.5f,0.4f),
            LIFT=color(0.15f,0.3f,0.85f),NET=color(0.35f,1,0.25f),AIR=color(0.95f,0.95f,1),WATER=color(0.25f,0.45f,1),SURFACE=color(0.55f,0.85f,1);
    private PlatformDebug() {}
    public static void display(DebugBody platform,WorldWaterSampler sampler,double time) {
        Entity entity=platform.entity();
        if(platform.debugMode()==0 || entity.tickCount%10!=0 || platform.owner()==null || !(entity.level() instanceof ServerLevel level)) {return;}
        ServerPlayer player=level.getServer().getPlayerList().getPlayer(platform.owner());
        if(player==null || player.level()!=level || player.distanceToSqr(entity)>32*32) {return;}
        CompiledWaves waves=platform.waves().compiled();
        double waveLevel=platform.waves().stillWaterY();
        if((platform.debugMode()&FORCES)!=0) {
            Painter painter=new Painter(level,player,140);
            Vec3d total=Vec3d.ZERO;
            for(AppliedForce force:platform.displayForces()) {
                DustParticleOptions color=switch(force.kind()) {
                    case WEIGHT->WEIGHT;case BUOYANCY->BUOYANCY;case DRAG->DRAG;case CONTACT->CONTACT;case THRUST->THRUST;case INERTIA->INERTIA;case RADIATION->RADIATION;case LIFT->LIFT;
                };
                painter.arrow(platform.forceOrigin().add(force.offset()),force.force(),1.0/1000,3,color);
                total=total.add(force.force());
            }
            painter.arrow(platform.forceOrigin(),total,1.0/1000,3,NET);
        }
        if((platform.debugMode()&FLOW)!=0) {
            Painter painter=new Painter(level,player,140);
            Vec3d origin=platform.modelOrigin();
            for(int y=-1;y<=1;y+=2) for(int x=-3;x<=3;x+=3) for(int z=-3;z<=3;z+=3) {
                Vec3d point=origin.add(new Vec3d(x,y,z));
                BlockPos block=BlockPos.containing(point.x(),point.y(),point.z());
                if(!level.hasChunkAt(block) || !level.getBlockState(block).getCollisionShape(level,block).isEmpty()) {continue;}
                WorldWaterSampler.Contact sample=sampler.sample(point,0.025,waves,waveLevel,time);
                boolean water=sample.fraction()>=0.5;
                Vec3d velocity=water?platform.environment().waterAt(sample.currentMps()).velocity():platform.environment().windAt(point);
                if((platform.debugMode()&4)!=0) {
                    Vec3d r=point.subtract(platform.physicsState().position());
                    velocity=velocity.subtract(platform.physicsState().velocity())
                            .subtract(platform.physicsState().angularVelocity(platform.properties()).cross(r));
                }
                painter.arrow(point,velocity,0.25,1,water?WATER:AIR);
            }
            // Sampled wave surface around the platform (taper held at the platform); water blocks do not move.
            double still=waveLevel;
            if(!waves.isEmpty() && Double.isFinite(still)) {
                Painter surface=new Painter(level,player,49);
                for(int x=-3;x<=3;x++) for(int z=-3;z<=3;z++) {
                    double wx=origin.x()+x*0.75,wz=origin.z()+z*0.75;
                    surface.dot(new Vec3d(wx,still+waves.elevation(wx,wz,time),wz),SURFACE);
                }
            }
        }
        if(entity.tickCount%20==0) {
            double celsius=platform.bodyTemperatureCelsius();
            player.displayClientMessage(Component.translatable("command.aerocore.platform.debug.legend",
                    Component.translatable((platform.debugMode()&4)!=0?"command.aerocore.platform.debug.relative":"command.aerocore.platform.debug.world"),
                    Double.isNaN(celsius)?"-":String.format(java.util.Locale.ROOT,"%.1f",celsius)),true);
        }
    }
    private static DustParticleOptions color(float r,float g,float b) {return new DustParticleOptions(new Vector3f(r,g,b),0.9f);}
    private static final class Painter {
        private final ServerLevel level;
        private final ServerPlayer player;
        private int budget;
        private Painter(ServerLevel level,ServerPlayer player,int budget) {this.level=level;this.player=player;this.budget=budget;}
        private void dot(Vec3d p,DustParticleOptions color) {
            if(budget--<=0) {return;}
            level.sendParticles(player,color,false,p.x(),p.y(),p.z(),1,0,0,0,0);
        }
        private void arrow(Vec3d start,Vec3d value,double scale,double cap,DustParticleOptions color) {
            double magnitude=value.length();
            if(magnitude<1e-6) {dot(start,color);return;}
            double length=Math.min(cap,magnitude*scale);
            Vec3d direction=value.scale(1/magnitude);
            int segments=Math.max(2,Math.min(9,(int)Math.ceil(length/0.2)));
            for(int i=0;i<=segments;i++) {dot(start.add(direction.scale(length*i/segments)),color);}
            Vec3d tip=start.add(direction.scale(length));
            Vec3d side=direction.cross(Math.abs(direction.y())<0.8?Vec3d.Y:Vec3d.X).normalized();
            double head=Math.min(0.22,length*0.35);
            dot(tip.subtract(direction.scale(head)).add(side.scale(head*0.65)),color);
            dot(tip.subtract(direction.scale(head)).subtract(side.scale(head*0.65)),color);
        }
    }
}
