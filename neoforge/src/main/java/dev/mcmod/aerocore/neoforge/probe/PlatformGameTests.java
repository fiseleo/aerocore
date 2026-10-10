package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.*;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;

@PrefixGameTestTemplate(false)
public final class PlatformGameTests {
    private PlatformGameTests() {}
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=1000)
    public static void symmetricPlatformFloatsWithoutSpinning(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,5);
        h.runAfterDelay(900,()->{
            h.assertTrue(p.physicsState().angularVelocity(p.properties()).length()<1e-5,"Symmetry must not generate spin: "+p.diagnostics().getString());
            double wet=p.wetFractions().stream().mapToDouble(Double::doubleValue).average().orElseThrow();
            h.assertTrue(wet>0.25 && wet<0.42,"Expected about 33% displacement: "+wet);
            h.assertTrue(p.physicsState().velocity().length()<0.2,"Heave should decay");
            for(double corner:p.wetFractions()) {h.assertTrue(Math.abs(corner-wet)<1e-5,"Symmetric corners must match");}
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=1300)
    public static void tiltedPlatformRestoresInStillWater(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        h.assertTrue(p.reconfigure(TestPlatform.DEFAULT_BALLAST,Orientation.rotationVector(Vec3d.Z.scale(0.25))),"Tilt setup must fit");
        final double[] early={0},late={0};
        for(int tick=1;tick<=1200;tick++) {
            int index=tick;
            h.runAfterDelay(tick,()->{
                double angle=Math.acos(Math.clamp(p.physicsState().orientation().rotate(Vec3d.Y).y(),-1,1));
                if(index<200) {early[0]=Math.max(early[0],angle);}
                if(index>1000) {late[0]=Math.max(late[0],angle);}
            });
        }
        h.runAfterDelay(1201,()->{
            h.assertTrue(late[0]<early[0]*0.5,"Roll envelope must decay: early="+early[0]+", late="+late[0]);
            h.assertTrue(p.physicsState().orientation().rotate(Vec3d.Y).y()>0.99,"Platform must recover upright");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=1000)
    public static void shiftedBallastTiltsAndLoadsTheCorrectCorners(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        h.assertTrue(p.reconfigure(new Vec3d(0.8,-0.35,0),Orientation.IDENTITY),"Ballast setup must fit");
        h.runAfterDelay(900,()->{
            Vec3d up=p.physicsState().orientation().rotate(Vec3d.Y);
            h.assertTrue(up.x()>0.02 && up.y()>0.9,"+X ballast must lower the +X edge: "+up);
            h.assertTrue(p.wetFractions().get(1)>p.wetFractions().get(0),"+X float must displace more water");
            h.assertTrue(Math.abs(p.properties().centerOfMassLocal().x()-0.32)<1e-12,"COM must use both masses");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=200)
    public static void platformLandsAndRestsOnDryFloor(GameTestHelper h) {
        floor(h);
        FloatingPlatform p=spawn(h,5);
        double floor=h.absolutePos(BlockPos.ZERO).getY()+1;
        h.runAfterDelay(160,()->{
            double minY=p.getBoundingBox().minY;
            h.assertTrue(minY>floor-0.01 && minY<floor+0.01,"Lowest contact sphere must rest on the floor: "+(minY-floor));
            h.assertTrue(p.physicsState().velocity().length()<0.01 && p.physicsState().angularVelocity(p.properties()).length()<0.01,
                    "Platform must come to rest: "+p.diagnostics().getString());
            // Air buoyancy (about 18 N) is the only other vertical load, so the floor carries ~ the weight.
            double weight=p.properties().massKg()*Atmosphere.GRAVITY_MPS2;
            h.assertTrue(Math.abs(p.contactLoad().force().y()-weight)<0.02*weight,"Contact reaction must carry the weight: "+p.contactLoad().force());
            h.assertTrue(p.displayForces().stream().anyMatch(f->f.kind()==AppliedForce.Kind.CONTACT),"Contact forces must be displayed");
            h.assertTrue(p.wetFractions().stream().allMatch(f->f==0),"Dry floor must not produce buoyancy");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=100)
    public static void movingPlatformDoesNotTunnelThroughWall(GameTestHelper h) {
        floor(h);
        for(int y=1;y<8;y++) for(int z=0;z<9;z++) {h.setBlock(new BlockPos(7,y,z),Blocks.STONE);}
        FloatingPlatform p=spawn(h,5);
        p.setVelocityMps(Vec3d.X.scale(30));
        double wall=h.absolutePos(new BlockPos(7,0,0)).getX();
        final double[] reach={Double.NEGATIVE_INFINITY};
        for(int tick=1;tick<=70;tick++) {
            h.runAfterDelay(tick,()->{
                reach[0]=Math.max(reach[0],p.getBoundingBox().maxX);
                h.assertTrue(p.getBoundingBox().maxX<=wall+0.01,"Envelope must not pass the wall beyond slop: "+(p.getBoundingBox().maxX-wall));
            });
        }
        h.runAfterDelay(71,()->{
            // Speculative contacts bounce in the step the impact is found, up to v*dt (0.15 m) before the face.
            h.assertTrue(reach[0]>wall-0.2,"Platform must actually reach wall: "+(reach[0]-wall));
            h.assertTrue(p.physicsState().velocity().x()<0.5,"Wall reaction must stop or reverse the approach: "+p.physicsState().velocity());
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=10)
    public static void ballastEditPreservesDeckAndRejectsObstructions(GameTestHelper h) {
        floor(h);
        FloatingPlatform p=spawn(h,3);
        Vec3d origin=p.modelOrigin();
        p.setAngularVelocity(Vec3d.Z);
        h.assertTrue(p.reconfigure(new Vec3d(0.8,-0.35,0),Orientation.IDENTITY),"Ballast move must fit");
        h.assertTrue(p.modelOrigin().subtract(origin).length()<1e-10,"Changing COM must not teleport deck");
        h.assertTrue(p.physicsState().angularMomentum().length()==0,"Configuration action resets motion");
        h.setBlock(new BlockPos(6,3,4),Blocks.STONE);
        h.assertTrue(!p.reconfigure(new Vec3d(3,0,0),Orientation.IDENTITY),"Reject ballast/envelope inside obstruction");
        h.assertTrue(Math.abs(p.ballastLocal().x()-0.8)<1e-12,"Rejected edit must leave old properties intact");
        h.succeed();
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=5)
    public static void sampledWetCentroidStaysSymmetric(GameTestHelper h) {
        pool(h);
        BlockPos center=h.absolutePos(new BlockPos(4,3,4));
        double surface=center.getY()+h.getLevel().getFluidState(center).getHeight(h.getLevel(),center);
        WorldWaterSampler.Contact c=new WorldWaterSampler(h.getLevel()).sample(new Vec3d(center.getX()+0.5,surface,center.getZ()+0.5),0.45);
        h.assertTrue(Math.abs(c.wetCentroidOffset().x())<1e-12 && Math.abs(c.wetCentroidOffset().z())<1e-12,"Horizontal centroid must stay symmetric");
        h.assertTrue(Math.abs(c.wetCentroidOffset().y()+3*0.45/8)<0.01,"Half-sphere centroid must approach analytic -3r/8");
        h.succeed();
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=100)
    public static void rotatingBallastNeverCrossesWall(GameTestHelper h) {
        floor(h);
        for(int y=1;y<9;y++) for(int z=0;z<9;z++) {h.setBlock(new BlockPos(7,y,z),Blocks.STONE);}
        FloatingPlatform p=spawn(h,4);
        h.assertTrue(p.reconfigure(new Vec3d(2,2,0),Orientation.IDENTITY),"Initial ballast must clear wall");
        p.setAngularVelocity(Vec3d.Z.scale(-2));
        double wall=h.absolutePos(new BlockPos(7,0,0)).getX();
        final boolean[] touched={false};
        for(int tick=1;tick<=60;tick++) {
            h.runAfterDelay(tick,()->{
                h.assertTrue(p.getBoundingBox().maxX<=wall+0.01,"Rotating ballast must never cross wall: "+(p.getBoundingBox().maxX-wall));
                touched[0]|=p.contactLoad().force().length()>0;
            });
        }
        h.runAfterDelay(61,()->{
            h.assertTrue(p.physicsState().orientation().rotate(Vec3d.Y).x()>0.01,"Test must exercise actual rotation before contact");
            h.assertTrue(touched[0],"Contact reactions must act on the swinging ballast");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=160)
    public static void warmPlatformCoolsWithoutCrossingReservoirTemperature(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        p.environment().setAirCelsius(20);p.environment().setWaterCelsius(20);p.setBodyTemperatureCelsius(80);
        h.runAfterDelay(120,()->{
            h.assertTrue(p.bodyTemperatureCelsius()<79.8 && p.bodyTemperatureCelsius()>20,"Hot platform should cool toward prescribed reservoirs: "+p.bodyTemperatureCelsius());
            h.assertTrue(p.wetFractions().stream().anyMatch(f->f>0),"Thermal case must exercise water contact");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=70)
    public static void sampledCurrentAlsoPushesThePlatform(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        p.environment().setCurrent(Vec3d.X);
        double start=p.getX();
        h.runAfterDelay(40,()->{
            h.assertTrue(p.getX()>start+0.02,"Configured flow must influence physics, not just particles");
            h.assertTrue(p.displayForces().stream().filter(f->f.kind()==AppliedForce.Kind.DRAG).mapToDouble(f->f.force().x()).sum()>0,"Displayed drag must point with current");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=5)
    public static void localThermalControlsChangeFluidPropertiesWithoutEditingWorld(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        var cold=p.environment().waterAt(Vec3d.ZERO);
        p.environment().setWaterCelsius(80);
        var hot=p.environment().waterAt(Vec3d.ZERO);
        h.assertTrue(hot.densityKgPerM3()<cold.densityKgPerM3() && hot.dynamicViscosityPaS()<cold.dynamicViscosityPaS(),"Temperature must change actual water properties");
        p.environment().setAirCelsius(50);
        h.assertTrue(Math.abs(p.environment().airAt(p.modelOrigin(),63).temperatureKelvin()-323.15)<1e-10,"Air temperature control must reach solver");
        h.assertTrue(h.getBlockState(new BlockPos(4,3,4)).is(Blocks.WATER),"Experiment must preserve Minecraft water blocks");
        h.succeed();
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=80)
    public static void propellerPushesPlatformAndRollsItByShaftReaction(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        p.setPropellerRps(15);
        double start=p.getX();
        final double[] roll={0};
        for(int tick=1;tick<=30;tick++) {
            h.runAfterDelay(tick,()->roll[0]+=p.physicsState().orientation().rotate(Vec3d.Y).z());
        }
        h.runAfterDelay(31,()->{
            h.assertTrue(p.getX()>start+0.3,"15 rev/s ahead must push the platform along local +X: "+(p.getX()-start));
            h.assertTrue(p.propellerThrustN()>0,"Reported thrust must be positive");
            h.assertTrue(p.displayForces().stream().filter(f->f.kind()==AppliedForce.Kind.THRUST).mapToDouble(f->f.force().x()).sum()>0,
                    "Displayed thrust must be the integrated load");
            // Shaft reaction -Q about +X heels the deck so that its up vector gains a negative z component.
            h.assertTrue(roll[0]/30<0,"Shaft reaction must roll the platform: "+roll[0]/30);
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=80)
    public static void windDragHeatsPlatformAboveEqualReservoirs(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        p.environment().setAirCelsius(20);p.environment().setWaterCelsius(20);p.setBodyTemperatureCelsius(20);
        p.environment().setWind(Vec3d.X.scale(10));
        h.runAfterDelay(60,()->{
            h.assertTrue(p.dragDissipationW()>0,"Wind must dissipate power in drag");
            h.assertTrue(p.frictionHeatW()>0 && p.frictionHeatW()<=p.dragDissipationW(),
                    "Frictional heating must be positive and bounded by dissipation: "+p.frictionHeatW()+" / "+p.dragDissipationW());
            h.assertTrue(p.bodyTemperatureCelsius()>20,"Only friction can warm the body when all reservoirs are 20 C: "+p.bodyTemperatureCelsius());
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=80)
    public static void slidingPlatformIsHeatedByContactFriction(GameTestHelper h) {
        floor(h);
        // Lowest contact spheres reach 0.55 m below the model origin; start 1 cm above the floor.
        FloatingPlatform p=spawn(h,1.56);
        p.environment().setAirCelsius(20);p.environment().setWaterCelsius(20);p.setBodyTemperatureCelsius(20);
        p.setVelocityMps(Vec3d.Z.scale(4));
        double kinetic=0.5*p.properties().massKg()*16;
        double share=HeatTransfer.contactHeatPartition(TestPlatform.FRAME_EFFUSIVITY,TestPlatform.GROUND_EFFUSIVITY);
        final double[] friction={0};
        for(int tick=1;tick<=60;tick++) {h.runAfterDelay(tick,()->friction[0]=Math.max(friction[0],p.contactFrictionW()));}
        h.runAfterDelay(61,()->{
            double rise=p.bodyTemperatureCelsius()-20,bound=share*kinetic/TestPlatform.HEAT_CAPACITY;
            h.assertTrue(friction[0]>0,"Sliding must dissipate power in contact friction");
            h.assertTrue(p.physicsState().velocity().length()<0.05,"Friction must stop the slide: "+p.physicsState().velocity());
            // First law: the body cannot gain more than its share of the initial kinetic energy. Normal impacts of the
            // chattering contact spheres (restitution) take part of it without heating, so the lower limit is loose.
            h.assertTrue(rise>0.4*bound && rise<1.02*bound,"Frictional warming "+rise+" K against bound "+bound+" K");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=140)
    public static void waveRadiationOnlyDrainsHeaveEnergy(GameTestHelper h) {
        pool(h);
        // Released 0.25 m above its floating height, the platform heaves; radiation forces must oppose the floats' heave.
        FloatingPlatform p=spawn(h,4.27);
        final double[] work={0};
        final boolean[] seen={false};
        for(int tick=1;tick<=120;tick++) {
            h.runAfterDelay(tick,()->{
                Vec3d omega=p.physicsState().angularVelocity(p.properties());
                for(AppliedForce f:p.displayForces()) {
                    if(f.kind()!=AppliedForce.Kind.RADIATION) {continue;}
                    seen[0]=true;
                    work[0]+=f.force().dot(p.physicsState().velocity().add(omega.cross(f.offset())))*0.05;
                }
            });
        }
        h.runAfterDelay(121,()->{
            h.assertTrue(seen[0],"Floats in water must carry radiation forces");
            h.assertTrue(work[0]<0,"Radiation must remove energy: "+work[0]+" J");
            h.assertTrue(p.physicsState().velocity().length()<0.05,"Heave should have decayed: "+p.physicsState().velocity());
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=5)
    public static void wavesRaiseAndLowerTheSampledSurfaceOnly(GameTestHelper h) {
        pool(h);
        BlockPos column=h.absolutePos(new BlockPos(4,3,4));
        WorldWaterSampler sampler=new WorldWaterSampler(h.getLevel());
        double still=sampler.stillSurface(column);
        LinearWave wave=new LinearWave(0.3,16,0,sampler.depthBelow(column,still),0);
        WaveField waves=new WaveField(List.of(wave));
        Vec3d center=new Vec3d(column.getX()+0.5,still,column.getZ()+0.5);
        double crestTime=wave.wavenumber()*center.x()/wave.angularFrequency();
        WorldWaterSampler.Contact calm=sampler.sample(center,0.3);
        WorldWaterSampler.Contact crest=sampler.sample(center,0.3,waves,crestTime);
        WorldWaterSampler.Contact trough=sampler.sample(center,0.3,waves,crestTime+wave.periodS()/2);
        h.assertTrue(Math.abs(calm.fraction()-0.5)<0.01,"Still surface must half-immerse the sphere: "+calm.fraction());
        h.assertTrue(crest.fraction()>0.9 && trough.fraction()<0.1,"Crest/trough must move the local surface: "+crest.fraction()+" / "+trough.fraction());
        // A fully submerged sample sees orbital velocity along the wave under crests and against it under troughs.
        Vec3d deep=center.subtract(Vec3d.Y.scale(0.8));
        WorldWaterSampler.Contact belowCrest=sampler.sample(deep,0.2,waves,crestTime);
        WorldWaterSampler.Contact belowTrough=sampler.sample(deep,0.2,waves,crestTime+wave.periodS()/2);
        h.assertTrue(belowCrest.currentMps().x()>0 && belowTrough.currentMps().x()<0,
                "Orbital velocity sign: "+belowCrest.currentMps()+" / "+belowTrough.currentMps());
        // Under a crest the water accelerates downward, so the pressure lift on a submerged body drops: 1 + a_z/g.
        double expectedCrest=1+belowCrest.accelerationMps2().y()/Atmosphere.GRAVITY_MPS2;
        h.assertTrue(belowCrest.pressureFraction()<1 && belowTrough.pressureFraction()>1,
                "Wave pressure must reduce lift under crests: "+belowCrest.pressureFraction()+" / "+belowTrough.pressureFraction());
        h.assertTrue(Math.abs(belowCrest.pressureFraction()-expectedCrest)<0.01,"Froude-Krylov lift: "+belowCrest.pressureFraction()+" vs "+expectedCrest);
        h.assertTrue(Math.abs(calm.pressureFraction()-calm.fraction())<1e-12,"Still water pressure must follow geometry");
        h.assertTrue(h.getBlockState(new BlockPos(4,4,4)).isAir(),"Waves must not place water blocks");
        h.succeed();
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=320)
    public static void platformHeavesInPrescribedPoolWaves(GameTestHelper h) {
        pool(h);
        FloatingPlatform p=spawn(h,4.02);
        BlockPos column=h.absolutePos(new BlockPos(4,3,4));
        WorldWaterSampler sampler=new WorldWaterSampler(h.getLevel());
        double still=sampler.stillSurface(column);
        // A world region (radius 5 m, so its 2 m taper ends outside the pool walls) carries the waves.
        WaveRegion region=new WaveRegion(column.getX()+0.5,column.getZ()+0.5,5,still,
                new WaveField(List.of(new LinearWave(0.2,16,0,sampler.depthBelow(column,still),0))));
        WaveRegions.of(h.getLevel()).add(h.getLevel(),region);
        final double[] range={Double.MAX_VALUE,-Double.MAX_VALUE};
        for(int tick=100;tick<=300;tick++) {
            h.runAfterDelay(tick,()->{range[0]=Math.min(range[0],p.getY());range[1]=Math.max(range[1],p.getY());});
        }
        h.runAfterDelay(301,()->{
            WaveRegions.of(h.getLevel()).removeAt(h.getLevel(),region.centerX(),region.centerZ());
            p.refreshWaves();
            double heave=(range[1]-range[0])/2;
            h.assertTrue(heave>0.1 && heave<0.3,"Platform must heave with 0.2 m waves: "+heave);
            h.assertTrue(p.waves().region()==null,"Removing the region must calm the platform's water");
            h.succeed();
        });
    }
    @GameTest(templateNamespace="aerocore",template="probe_empty",timeoutTicks=5)
    public static void waveRegionMovesOnlyItsOwnLevelAndTapersToCalm(GameTestHelper h) {
        pool(h);
        BlockPos column=h.absolutePos(new BlockPos(4,3,4));
        WorldWaterSampler sampler=new WorldWaterSampler(h.getLevel());
        double still=sampler.stillSurface(column);
        LinearWave wave=new LinearWave(0.3,16,0,sampler.depthBelow(column,still),0);
        WaveRegion region=new WaveRegion(column.getX()+0.5,column.getZ()+0.5,8,still,new WaveField(List.of(wave)));
        WaveRegions regions=WaveRegions.of(h.getLevel());
        regions.add(h.getLevel(),region);
        try {
            // Core: the plain Airy field. Taper: smoothstep amplitude. Outside: calm.
            h.assertTrue(Math.abs(region.weight(region.centerX()+5.9,region.centerZ())-1)<1e-12,"Core must carry the full wave");
            // 0.5 m inside the rim of the 2 m ramp: s = 1/4, smoothstep = 5/32.
            double rim=region.weight(region.centerX()+7.5,region.centerZ());
            h.assertTrue(Math.abs(rim-0.15625)<1e-9,"Taper must follow smoothstep: "+rim);
            h.assertTrue(regions.at(region.centerX()+8.5,region.centerZ()).region()==null,"Outside the circle the water is calm");
            WaveRegions.Local local=regions.at(region.centerX(),region.centerZ());
            h.assertTrue(local.region()==region && Math.abs(local.stillWaterY()-still)<1e-12,"Centre must resolve to the region");
            Vec3d center=new Vec3d(column.getX()+0.5,still,column.getZ()+0.5);
            double crest=wave.wavenumber()*center.x()/wave.angularFrequency();
            h.assertTrue(sampler.sample(center,0.3,local.waves(),local.stillWaterY(),crest).fraction()>0.9,"The region's own surface must rise under a crest");
            // The same field at another still level (another water body inside the circle) must not move it.
            h.assertTrue(Math.abs(sampler.sample(center,0.3,local.waves(),still+1,crest).fraction()-0.5)<0.01,"Other levels stay calm");
            // Saved data round trip keeps the region exactly.
            WaveRegions copy=WaveRegions.load(regions.save(new net.minecraft.nbt.CompoundTag(),h.getLevel().registryAccess()),h.getLevel().registryAccess());
            h.assertTrue(copy.regions().contains(region),"Region must survive save and load: "+copy.regions());
            h.assertTrue(h.getBlockState(new BlockPos(4,4,4)).isAir(),"Regions must not place water blocks");
        } finally {
            regions.removeAt(h.getLevel(),region.centerX(),region.centerZ());
        }
        h.succeed();
    }
    private static FloatingPlatform spawn(GameTestHelper h,double y) {
        BlockPos o=h.absolutePos(BlockPos.ZERO);
        FloatingPlatform p=ProbeEntities.FLOATING_PLATFORM.get().create(h.getLevel());
        if(p==null) {throw new IllegalStateException("Platform factory failed");}
        p.configure(null,new Vec3d(o.getX()+4.5,o.getY()+y,o.getZ()+4.5));
        h.getLevel().addFreshEntity(p);return p;
    }
    private static void floor(GameTestHelper h) {
        for(int x=0;x<9;x++) for(int z=0;z<9;z++) {h.setBlock(new BlockPos(x,0,z),Blocks.STONE);}
    }
    private static void pool(GameTestHelper h) {
        floor(h);
        for(int y=1;y<=4;y++) for(int x=0;x<9;x++) for(int z=0;z<9;z++) {
            if(x==0||z==0||x==8||z==8) {h.setBlock(new BlockPos(x,y,z),Blocks.STONE);}
            else if(y<=3) {h.setBlock(new BlockPos(x,y,z),Blocks.WATER);}
        }
    }
}
