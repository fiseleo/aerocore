package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import com.mojang.math.Axis;
import dev.mcmod.aerocore.api.*;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.FloatingPlatform;
import dev.mcmod.aerocore.neoforge.probe.ProbeEntities;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.entity.EntityRenderer;
import net.minecraft.client.renderer.entity.EntityRendererProvider;
import net.minecraft.client.renderer.texture.OverlayTexture;
import net.minecraft.resources.ResourceLocation;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.EntityRenderersEvent;
import org.joml.Quaternionf;
import org.joml.Vector3f;

/** Rotating deck, four distinct floats, ballast and direction marks, all generated geometry. */
@EventBusSubscriber(modid=AeroCore.MOD_ID,bus=EventBusSubscriber.Bus.MOD,value=Dist.CLIENT)
public final class PlatformRenderer extends EntityRenderer<FloatingPlatform> {
    private static final ResourceLocation TEXTURE=ResourceLocation.withDefaultNamespace("textures/block/white_concrete.png");
    private static final int[] COLORS={0xffda6262,0xffffad4d,0xff518ee0,0xff48c7ca};
    public PlatformRenderer(EntityRendererProvider.Context context) {super(context);shadowRadius=1.3f;}
    @SubscribeEvent public static void register(EntityRenderersEvent.RegisterRenderers event) {
        event.registerEntityRenderer(ProbeEntities.FLOATING_PLATFORM.get(),PlatformRenderer::new);
    }
    @Override public void render(FloatingPlatform platform,float yaw,float partialTick,PoseStack stack,MultiBufferSource buffers,int light) {
        stack.pushPose();
        Orientation q=platform.visualOrientation(partialTick);
        stack.mulPose(new Quaternionf((float)q.x(),(float)q.y(),(float)q.z(),(float)q.w()));
        Vec3d ballast=platform.ballastLocal(),com=TestPlatform.properties(ballast).centerOfMassLocal();
        stack.translate(-com.x(),-com.y(),-com.z());
        VertexConsumer out=buffers.getBuffer(RenderType.entitySolid(TEXTURE));
        box(out,stack,-1.2,0.05,-1.2,1.2,0.25,1.2,0xff647582,light);
        // Red +X and blue +Z strips make model-local ballast coordinates visible.
        box(out,stack,0,0.251,-0.035,1.05,0.267,0.035,0xffec6666,light);
        box(out,stack,-0.035,0.251,0,0.035,0.267,1.05,0xff72b4ff,light);
        for(int i=0;i<TestPlatform.FLOATS.size();i++) {
            BuoyantSphere f=TestPlatform.FLOATS.get(i);
            stack.pushPose();stack.translate(f.offset().x(),f.offset().y(),f.offset().z());
            sphere(out,stack,f.radiusM(),COLORS[i],light);stack.popPose();
        }
        Vec3d start=new Vec3d(0,0.15,0),direction=ballast.subtract(start);
        if(direction.length()>1e-6) {
            stack.pushPose();stack.translate(start.x(),start.y(),start.z());
            Vec3d unit=direction.normalized();
            stack.mulPose(new Quaternionf().rotationTo(new Vector3f(0,1,0),new Vector3f((float)unit.x(),(float)unit.y(),(float)unit.z())));
            box(out,stack,-0.025,0,-0.025,0.025,direction.length(),0.025,0xffdddddd,light);stack.popPose();
        }
        box(out,stack,ballast.x()-0.2,ballast.y()-0.2,ballast.z()-0.2,ballast.x()+0.2,ballast.y()+0.2,ballast.z()+0.2,0xffffdb55,light);
        propeller(out,stack,platform,partialTick,light);
        // COM marker is decorative and contributes no extra displacement or mass.
        box(out,stack,com.x()-0.045,com.y()-0.045,com.z()-0.045,com.x()+0.045,com.y()+0.045,com.z()+0.045,0xffff38d1,light);
        stack.popPose();super.render(platform,yaw,partialTick,stack,buffers,light);
    }
    /** Strut, hub and three pitched blades turning about local +X. Visual rotation follows the synced rev/s. */
    private static void propeller(VertexConsumer out,PoseStack stack,FloatingPlatform platform,float partialTick,int light) {
        Propeller p=TestPlatform.PROPELLER;
        Vec3d hub=p.positionLocal();
        box(out,stack,hub.x()-0.02,hub.y(),-0.02,hub.x()+0.02,0.07,0.02,0xff8a8f96,light);
        box(out,stack,hub.x()-0.02,0.03,-0.02,-1.18,0.07,0.02,0xff8a8f96,light);
        box(out,stack,hub.x()-0.04,hub.y()-0.035,-0.035,hub.x()+0.04,hub.y()+0.035,0.035,0xff8a8f96,light);
        double turns=(platform.tickCount+partialTick)*0.05*platform.propellerRps();
        float angle=(float)(2*Math.PI*(turns-Math.floor(turns)));
        double radius=p.diameterM()/2;
        for(int blade=0;blade<3;blade++) {
            stack.pushPose();
            stack.translate(hub.x(),hub.y(),hub.z());
            stack.mulPose(Axis.XP.rotation(angle+blade*(float)(2*Math.PI/3)));
            stack.mulPose(Axis.YP.rotationDegrees(35));
            box(out,stack,-0.006,0.03,-0.03,0.006,radius,0.03,0xffc8913a,light);
            stack.popPose();
        }
    }
    private static void sphere(VertexConsumer out,PoseStack stack,double radius,int color,int light) {
        for(int lat=0;lat<12;lat++) for(int lon=0;lon<20;lon++) {
            double a=Math.PI*lat/12,b=Math.PI*(lat+1)/12,c=2*Math.PI*lon/20,d=2*Math.PI*(lon+1)/20;
            int band=lat==5 || lat==6?0xffeeeeee:color;
            sphereVertex(out,stack,a,c,radius,band,light);sphereVertex(out,stack,a,d,radius,band,light);
            sphereVertex(out,stack,b,d,radius,band,light);sphereVertex(out,stack,b,c,radius,band,light);
        }
    }
    private static void sphereVertex(VertexConsumer out,PoseStack stack,double theta,double phi,double radius,int color,int light) {
        float x=(float)(Math.sin(theta)*Math.cos(phi)),y=(float)Math.cos(theta),z=(float)(Math.sin(theta)*Math.sin(phi));
        vertex(out,stack,x*radius,y*radius,z*radius,(float)(phi/(2*Math.PI)),(float)(theta/Math.PI),x,y,z,color,light);
    }
    private static void box(VertexConsumer out,PoseStack s,double x0,double y0,double z0,double x1,double y1,double z1,int color,int light) {
        double[][] v={{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},{x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}};
        int[][] faces={{0,3,2,1},{4,5,6,7},{0,4,7,3},{1,2,6,5},{0,1,5,4},{3,7,6,2}};
        float[][] normals={{0,0,-1},{0,0,1},{-1,0,0},{1,0,0},{0,-1,0},{0,1,0}};
        for(int face=0;face<6;face++) for(int i=0;i<4;i++) {
            double[] p=v[faces[face][i]];float[] n=normals[face];
            vertex(out,s,p[0],p[1],p[2],i==1||i==2?1:0,i>=2?1:0,n[0],n[1],n[2],color,light);
        }
    }
    private static void vertex(VertexConsumer out,PoseStack stack,double x,double y,double z,float u,float v,float nx,float ny,float nz,int color,int light) {
        out.addVertex(stack.last(),(float)x,(float)y,(float)z).setColor(color).setUv(u,v)
                .setOverlay(OverlayTexture.NO_OVERLAY).setLight(light).setNormal(stack.last(),nx,ny,nz);
    }
    @Override public ResourceLocation getTextureLocation(FloatingPlatform platform) {return TEXTURE;}
}
