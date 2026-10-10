package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import static org.junit.jupiter.api.Assertions.*;

class RigidBodyTest {
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private static final RigidBodyProperties DEFAULT = TestPlatform.properties(TestPlatform.DEFAULT_BALLAST);

    @Test void quaternionRotatesAndInvertsWithoutLosingNorm() {
        Orientation q = Orientation.rotationVector(Vec3d.Z.scale(Math.PI/2));
        assertEquals(0,q.rotate(Vec3d.X).subtract(Vec3d.Y).length(),1e-12);
        Vec3d v = new Vec3d(2,-5,7);
        assertEquals(0,q.inverse().rotate(q.rotate(v)).subtract(v).length(),1e-12);
        for (int i=0;i<10000;i++) { q=q.advanceWorld(new Vec3d(1,2,3),0.005); }
        assertEquals(1,q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z(),1e-14);
    }
    @Test void inertiaIncludesParallelAxisAndCrossTerms() {
        Vec3d ballast = new Vec3d(1,2,3);
        RigidBodyProperties p = TestPlatform.properties(ballast);
        assertEquals(0,new Vec3d(0.4,0.8,1.2).subtract(p.centerOfMassLocal()).length(),1e-12);
        assertEquals(-120*1*2,p.inertia().xy(),1e-12);
        assertEquals(-120*1*3,p.inertia().xz(),1e-12);
        Vec3d v=new Vec3d(3,-2,5);
        assertEquals(0,p.inertia().solve(p.inertia().apply(v)).subtract(v).length(),1e-12);
    }
    @Test void offCenterImpulseChangesBothMomenta() {
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.ZERO,Orientation.IDENTITY,DEFAULT);
        Wrench load=Wrench.atOffset(Vec3d.Y.scale(100),Vec3d.X);
        RigidBodyState after=RigidBodyDynamics.kick(s,DEFAULT,load,0.1);
        assertEquals(0.02,after.velocity().y(),1e-12);
        assertEquals(10,after.angularMomentum().z(),1e-12);
        assertEquals(10/DEFAULT.inertia().zz(),after.angularVelocity(DEFAULT).z(),1e-12);
    }
    @Test void principalAxisSpinMatchesAnalyticAngle() {
        RigidBodyState s=new RigidBodyState(Vec3d.ZERO,Vec3d.ZERO,Orientation.IDENTITY,Vec3d.Y.scale(DEFAULT.inertia().yy()*2));
        for(int i=0;i<200;i++) {s=RigidBodyDynamics.drift(s,DEFAULT,0.005);}
        assertEquals(0,s.orientation().rotate(Vec3d.X).subtract(Orientation.rotationVector(Vec3d.Y.scale(2)).rotate(Vec3d.X)).length(),1e-10);
    }
    @Test void asymmetricFreeRotationConservesWorldMomentumAndBoundsEnergy() {
        RigidBodyProperties p=new RigidBodyProperties(1,Vec3d.ZERO,new InertiaTensor(2,3,4,0,0,0));
        Vec3d momentum=new Vec3d(1,2,3);
        RigidBodyState s=new RigidBodyState(Vec3d.ZERO,Vec3d.ZERO,Orientation.IDENTITY,momentum);
        double initial=s.kineticEnergy(p),maxError=0;
        for(int i=0;i<20000;i++) {
            s=RigidBodyDynamics.drift(s,p,0.005);
            maxError=Math.max(maxError,Math.abs(s.kineticEnergy(p)-initial)/initial);
        }
        assertEquals(momentum,s.angularMomentum());
        assertTrue(maxError<1e-5,"Relative energy error="+maxError);
        assertTrue(s.angularVelocity(p).subtract(p.inertia().solve(momentum)).length()>0.01,"Gyroscopic motion must change omega");
    }
    @Test void coupledDragIsDissipativeEvenAtLargeTimeStep() {
        RigidBodyState s=new RigidBodyState(Vec3d.ZERO,new Vec3d(3,-2,5),Orientation.rotationVector(new Vec3d(0.3,0.7,0.2)),new Vec3d(400,200,-300));
        List<RigidBodyDynamics.PointDrag> points=List.of(new RigidBodyDynamics.PointDrag(new Vec3d(2,1,3),Vec3d.ZERO,100,10000),
                new RigidBodyDynamics.PointDrag(new Vec3d(-1,2,-2),Vec3d.ZERO,10,20000));
        double initial=s.kineticEnergy(DEFAULT);
        s=RigidBodyDynamics.drag(s,DEFAULT,points,10);
        assertTrue(s.kineticEnergy(DEFAULT)<initial);
    }
    @Test void pointOrderDoesNotCreateDirectionalBias() {
        RigidBodyState s=new RigidBodyState(Vec3d.ZERO,new Vec3d(0,-3,0),Orientation.IDENTITY,Vec3d.ZERO);
        List<RigidBodyDynamics.PointDrag> points=new ArrayList<>();
        for(BuoyantSphere f:TestPlatform.FLOATS) {points.add(new RigidBodyDynamics.PointDrag(s.worldOffset(DEFAULT,f.offset()),Vec3d.ZERO,0,100));}
        RigidBodyState a=RigidBodyDynamics.drag(s,DEFAULT,points,0.05);
        Collections.reverse(points);
        RigidBodyState b=RigidBodyDynamics.drag(s,DEFAULT,points,0.05);
        assertEquals(0,a.angularMomentum().length(),1e-12);
        assertEquals(0,a.velocity().subtract(b.velocity()).length(),1e-12);
    }
    @Test void dragIsInvariantUnderCommonVelocityShift() {
        Vec3d shift=new Vec3d(7,-3,4),v=new Vec3d(1,2,-1),r=new Vec3d(2,1,3);
        RigidBodyState a=new RigidBodyState(Vec3d.ZERO,v,Orientation.IDENTITY,new Vec3d(20,30,40));
        RigidBodyState b=new RigidBodyState(Vec3d.ZERO,v.add(shift),Orientation.IDENTITY,a.angularMomentum());
        a=RigidBodyDynamics.drag(a,DEFAULT,List.of(new RigidBodyDynamics.PointDrag(r,Vec3d.ZERO,5,20)),0.1);
        b=RigidBodyDynamics.drag(b,DEFAULT,List.of(new RigidBodyDynamics.PointDrag(r,shift,5,20)),0.1);
        assertEquals(0,b.velocity().subtract(a.velocity()).subtract(shift).length(),1e-12);
        assertEquals(0,b.angularMomentum().subtract(a.angularMomentum()).length(),1e-10);
    }
    @Test void symmetricDropNeverDevelopsSpin() {
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.Y,Orientation.IDENTITY,DEFAULT);
        for(int i=0;i<12000;i++) {s=step(s,DEFAULT,0.005);}
        assertEquals(0,s.angularVelocity(DEFAULT).length(),1e-10);
        assertEquals(1,s.orientation().rotate(Vec3d.Y).y(),1e-12);
        assertTrue(Math.abs(s.velocity().y())<0.15);
    }
    @Test void tiltedLowBallastRestoresAndOscillationEnvelopeDecays() {
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.Y.scale(0.12),Orientation.rotationVector(Vec3d.Z.scale(0.3)),DEFAULT);
        double early=0,late=0,maxAngle=0;
        for(int i=0;i<24000;i++) {
            s=step(s,DEFAULT,0.005);
            double angle=Math.acos(Math.clamp(s.orientation().rotate(Vec3d.Y).y(),-1,1));
            maxAngle=Math.max(maxAngle,angle);
            if(i<4000) {early=Math.max(early,angle);}
            if(i>=20000) {late=Math.max(late,angle);}
        }
        assertTrue(maxAngle<0.35,"Unbounded initial oscillation="+maxAngle);
        assertTrue(late<early*0.25,"Envelope early="+early+", late="+late);
        assertTrue(s.angularVelocity(DEFAULT).length()<0.08);
    }
    @Test void shiftedBallastHeelsTowardItsSideAndMirrorReverses() {
        Vec3d right=settledUp(new Vec3d(0.8,-0.35,0)),left=settledUp(new Vec3d(-0.8,-0.35,0));
        assertTrue(right.x()>0.02 && right.y()>0.9,"Right load must lower right side: "+right);
        assertEquals(-right.x(),left.x(),1e-7);
        assertEquals(right.y(),left.y(),1e-7);
    }
    @Test void sufficientlyHighBallastCanCapsize() {
        RigidBodyProperties p=TestPlatform.properties(new Vec3d(0,10,0));
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.Y.scale(0.12),Orientation.rotationVector(Vec3d.Z.scale(0.05)),p);
        boolean capsized=false;
        for(int i=0;i<20000;i++) {
            s=step(s,p,0.005);
            if(s.orientation().rotate(Vec3d.Y).y()<0) {capsized=true;break;}
        }
        assertTrue(capsized,"High COM must permit overturning, not artificially self-right");
    }
    @Test void halvingStepConvergesForBuoyantRotation() {
        RigidBodyState a=trajectory(0.01),b=trajectory(0.005),reference=trajectory(0.00125);
        double coarse=error(a,reference),fine=error(b,reference);
        assertTrue(fine<coarse*0.75,"coarse="+coarse+", fine="+fine);
    }
    @Test void reportedWrenchMatchesMomentumChange() {
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.ZERO,Orientation.rotationVector(Vec3d.Z.scale(0.2)),DEFAULT);
        FloatingBodyDynamics.Result result=FloatingBodyDynamics.step(s,DEFAULT,TestPlatform.FLOATS,TestPlatform.flatWater(0),GRAVITY,0.47,0.005);
        // Added mass and the entrained-water exchange are part of the body's momentum balance.
        Wrench sum=result.hydrostatics().add(result.averageDrag()).add(result.inertia());
        assertEquals(0,result.state().velocity().subtract(s.velocity()).scale(DEFAULT.massKg()/0.005).subtract(sum.force()).length(),1e-9);
        assertEquals(0,result.state().angularMomentum().subtract(s.angularMomentum()).scale(1/0.005).subtract(sum.torque()).length(),1e-9);
    }
    @Test void rejectsInvalidInertiaQuaternionAndTimeStep() {
        assertThrows(IllegalArgumentException.class,()->new InertiaTensor(1,1,1,2,0,0));
        assertThrows(IllegalArgumentException.class,()->new Orientation(0,0,0,0));
        assertThrows(IllegalArgumentException.class,()->RigidBodyDynamics.drift(TestPlatform.atModelOrigin(Vec3d.ZERO,Orientation.IDENTITY,DEFAULT),DEFAULT,0));
    }
    private static RigidBodyState step(RigidBodyState s,RigidBodyProperties p,double dt) {
        return FloatingBodyDynamics.step(s,p,TestPlatform.FLOATS,TestPlatform.flatWater(0),GRAVITY,0.47,dt).state();
    }
    private static Vec3d settledUp(Vec3d ballast) {
        RigidBodyProperties p=TestPlatform.properties(ballast);
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.Y.scale(0.12),Orientation.IDENTITY,p);
        for(int i=0;i<16000;i++) {s=step(s,p,0.005);}
        return s.orientation().rotate(Vec3d.Y);
    }
    private static RigidBodyState trajectory(double dt) {
        RigidBodyState s=TestPlatform.atModelOrigin(Vec3d.Y.scale(0.4),Orientation.rotationVector(new Vec3d(0.15,0,0.2)),DEFAULT);
        for(int i=0;i<(int)Math.round(3/dt);i++) {s=step(s,DEFAULT,dt);}
        return s;
    }
    private static double error(RigidBodyState a,RigidBodyState b) {
        return a.position().subtract(b.position()).length()+a.orientation().rotate(Vec3d.Y).subtract(b.orientation().rotate(Vec3d.Y)).length();
    }
}
