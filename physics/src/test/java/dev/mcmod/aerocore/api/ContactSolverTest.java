package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import static org.junit.jupiter.api.Assertions.*;

class ContactSolverTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, DT = 0.005;
    private static final Vec3d DOWN = Vec3d.Y.scale(-G);
    private static final List<WorldBox> FLOOR = List.of(new WorldBox(new Vec3d(-100,-1,-100),new Vec3d(100,0,100)));
    /** Solid sphere, m = 1 kg, r = 0.5 m, I = 2/5 m r^2. */
    private static final RigidBodyProperties BALL = new RigidBodyProperties(1,Vec3d.ZERO,new InertiaTensor(0.1,0.1,0.1,0,0,0));
    private static final List<CollisionSphere> BALL_SHAPE = List.of(new CollisionSphere(Vec3d.ZERO,0.5));
    /** 10 kg, 1 x 0.5 x 1 m box represented by inset corner spheres. */
    private static final RigidBodyProperties BOX = new RigidBodyProperties(10,Vec3d.ZERO,InertiaTensor.box(10,new Vec3d(1,0.5,1)));
    private static final List<CollisionSphere> BOX_SHAPE = corners(0.5,0.25,0.5,0.02);

    @Test void headOnSpheresFollowTheOneDimensionalImpactLaw() {
        // m1 = 1, m2 = 3 (scaled ball), v1 = 2, v2 = 0: v1' = (m1 - e m2) v1/(m1+m2), v2' = (1+e) m1 v1/(m1+m2).
        RigidBodyProperties heavy = new RigidBodyProperties(3, Vec3d.ZERO, new InertiaTensor(0.3, 0.3, 0.3, 0, 0, 0));
        for (double e : new double[]{0, 0.5, 1}) {
            ContactMaterial material = new ContactMaterial(0, e);
            List<ContactSolver.Body> bodies = List.of(
                    new ContactSolver.Body(new RigidBodyState(new Vec3d(-0.5, 0, 0), new Vec3d(2, 0, 0), Orientation.IDENTITY, Vec3d.ZERO), BALL),
                    new ContactSolver.Body(new RigidBodyState(new Vec3d(0.5, 0, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO), heavy));
            // Touching spheres approaching at 2 m/s.
            List<ContactSolver.PairContact> contacts = ContactSolver.generatePair(1, bodies.get(1), BALL_SHAPE, 0, bodies.get(0), BALL_SHAPE, material, 0.02);
            assertEquals(1, contacts.size());
            List<ContactSolver.BodyResult> r = ContactSolver.solveAll(bodies, contacts, 0.02);
            double v1 = r.get(0).state().velocity().x(), v2 = r.get(1).state().velocity().x();
            assertEquals((1 - 3 * e) * 2 / 4, v1, 1e-6, "e=" + e);
            assertEquals((1 + e) * 2 / 4, v2, 1e-6, "e=" + e);
            assertEquals(2, v1 + 3 * v2, 1e-9, "Momentum must be conserved");
            // Equal and opposite reported forces.
            assertEquals(0, r.get(0).wrench().force().add(r.get(1).wrench().force()).length(), 1e-9);
        }
    }
    @Test void bodyPairContactsConserveLinearAndAngularMomentum() {
        // Random off-centre frictional contacts between two free boxes: total momentum and angular momentum about
        // the origin are unchanged by the joint solve, and friction never adds energy.
        Random random = new Random(5);
        for (int trial = 0; trial < 50; trial++) {
            RigidBodyState a = new RigidBodyState(new Vec3d(0, 0.5, 0), randomVector(random, 2), Orientation.rotationVector(randomVector(random, 1)), randomVector(random, 3));
            RigidBodyState b = new RigidBodyState(new Vec3d(0.3, 0, 0.2), randomVector(random, 2), Orientation.rotationVector(randomVector(random, 1)), randomVector(random, 3));
            List<ContactSolver.Body> bodies = List.of(new ContactSolver.Body(a, BOX), new ContactSolver.Body(b, BOX));
            List<ContactSolver.PairContact> contacts = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                Vec3d point = new Vec3d(random.nextDouble() * 0.6 - 0.3, 0.25, random.nextDouble() * 0.6 - 0.3);
                contacts.add(new ContactSolver.PairContact(0, 1, point.subtract(a.position()), point.subtract(b.position()), Vec3d.Y, -0.001,
                        new ContactMaterial(0.6, random.nextDouble())));
            }
            List<ContactSolver.BodyResult> r = ContactSolver.solveAll(bodies, contacts, DT);
            Vec3d p0 = a.velocity().scale(10).add(b.velocity().scale(10));
            Vec3d p1 = r.get(0).state().velocity().scale(10).add(r.get(1).state().velocity().scale(10));
            Vec3d l0 = a.angularMomentum().add(b.angularMomentum()).add(a.position().cross(a.velocity().scale(10))).add(b.position().cross(b.velocity().scale(10)));
            Vec3d l1 = r.get(0).state().angularMomentum().add(r.get(1).state().angularMomentum())
                    .add(a.position().cross(r.get(0).state().velocity().scale(10))).add(b.position().cross(r.get(1).state().velocity().scale(10)));
            assertEquals(0, p1.subtract(p0).length(), 1e-9, "Linear momentum, trial " + trial);
            assertEquals(0, l1.subtract(l0).length(), 1e-9, "Angular momentum, trial " + trial);
            // Shared equally; non-negative up to the PGS residual (24 iterations), as for static contacts.
            double scale = (a.kineticEnergy(BOX) + b.kineticEnergy(BOX)) / DT;
            assertTrue(r.get(0).pairFrictionW() >= -1e-4 * scale && Math.abs(r.get(0).pairFrictionW() - r.get(1).pairFrictionW()) < 1e-12,
                    "Friction power is shared and non-negative: " + r.get(0).pairFrictionW());
        }
    }
    @Test void stackedBodiesRestWithTheWholeWeightOnTheFloor() {
        // A box resting on another box on the floor: after settling the floor carries both weights, the upper
        // contact carries the top box's weight, and neither body sinks.
        ContactMaterial material = new ContactMaterial(0.5, 0);
        RigidBodyState lower = new RigidBodyState(new Vec3d(0, 0.25, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        // Corner spheres line up, so the upper box stands on four sphere-sphere contacts.
        RigidBodyState upper = new RigidBodyState(new Vec3d(0, 0.75, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        List<ContactSolver.BodyResult> result = null;
        for (int i = 0; i < 400; i++) {
            lower = RigidBodyDynamics.kick(lower, BOX, new Wrench(DOWN.scale(10), Vec3d.ZERO), DT);
            upper = RigidBodyDynamics.kick(upper, BOX, new Wrench(DOWN.scale(10), Vec3d.ZERO), DT);
            List<ContactSolver.Body> bodies = List.of(new ContactSolver.Body(lower, BOX), new ContactSolver.Body(upper, BOX));
            List<ContactSolver.PairContact> contacts = new ArrayList<>(ContactSolver.tag(0, ContactSolver.generate(lower, BOX, BOX_SHAPE, FLOOR, DT), material));
            contacts.addAll(ContactSolver.generatePair(1, bodies.get(1), BOX_SHAPE, 0, bodies.get(0), BOX_SHAPE, material, DT));
            result = ContactSolver.solveAll(bodies, contacts, DT);
            lower = RigidBodyDynamics.drift(result.get(0).state(), BOX, DT);
            upper = RigidBodyDynamics.drift(result.get(1).state(), BOX, DT);
        }
        // The lower box receives floor support up and the upper box's weight down: net = its own weight.
        assertEquals(10 * G, result.get(0).wrench().force().y(), 0.02 * 10 * G);
        assertEquals(10 * G, result.get(1).wrench().force().y(), 0.02 * 10 * G);
        assertEquals(0.25, lower.position().y(), 0.01);
        assertTrue(upper.position().y() > 0.69 && upper.position().y() < 0.77, "Upper box rests on the lower one: " + upper.position().y());
        assertTrue(lower.velocity().length() < 1e-3 && upper.velocity().length() < 1e-3);
    }
    @Test void pairProjectionSeparatesByInverseMassWithoutChangingMomentum() {
        RigidBodyProperties heavy = new RigidBodyProperties(3, Vec3d.ZERO, new InertiaTensor(0.3, 0.3, 0.3, 0, 0, 0));
        List<ContactSolver.Body> bodies = List.of(
                new ContactSolver.Body(new RigidBodyState(new Vec3d(-0.4, 0, 0), new Vec3d(1, 0, 0), Orientation.IDENTITY, Vec3d.ZERO), BALL),
                new ContactSolver.Body(new RigidBodyState(new Vec3d(0.4, 0, 0), new Vec3d(-1, 0, 0), Orientation.IDENTITY, Vec3d.ZERO), heavy));
        ContactMaterial material = new ContactMaterial(0, 0);
        List<RigidBodyState> separated = ContactSolver.separateAll(bodies,
                current -> ContactSolver.generatePair(0, current.get(0), BALL_SHAPE, 1, current.get(1), BALL_SHAPE, material, 0));
        double moveA = separated.get(0).position().x() + 0.4, moveB = separated.get(1).position().x() - 0.4;
        assertTrue(moveA < 0 && moveB > 0, "Penetrating spheres must be pushed apart");
        assertEquals(3, -moveA / moveB, 1e-9, "Displacements follow inverse masses");
        assertEquals(new Vec3d(1, 0, 0), separated.get(0).velocity());
        assertEquals(new Vec3d(-1, 0, 0), separated.get(1).velocity());
    }
    @Test void slidingSphereStartsRollingAtFiveSeventhsOfItsSpeed() {
        // Classic result: friction mu m g slows a sliding solid sphere and spins it up until v = 5/7 v0,
        // reached at t = 2 v0 / (7 mu g); afterwards it rolls without slipping.
        ContactMaterial material=new ContactMaterial(0.3,0);
        RigidBodyState s=new RigidBodyState(new Vec3d(0,0.5,0),new Vec3d(5,0,0),Orientation.IDENTITY,Vec3d.ZERO);
        double rolling=-1;
        for(int i=0;i<600;i++) {
            s=step(s,BALL,BALL_SHAPE,FLOOR,material,DOWN,null);
            double slip=s.velocity().x()+0.5*s.angularVelocity(BALL).z();
            if(rolling<0 && Math.abs(slip)<1e-6) {rolling=(i+1)*DT;}
        }
        assertEquals(25.0/7,s.velocity().x(),0.01);
        assertEquals(-2*s.velocity().x(),s.angularVelocity(BALL).z(),1e-5);
        assertEquals(10/(7*0.3*G),rolling,0.011);
        assertEquals(0.5,s.position().y(),ContactSolver.SLOP);
    }
    @Test void slidingFrictionHeatEqualsTheKineticEnergyItRemoves() {
        // A box sliding to rest on level ground turns all its kinetic energy into friction heat.
        RigidBodyState s=new RigidBodyState(new Vec3d(0,0.25,0),new Vec3d(4,0,0),Orientation.IDENTITY,Vec3d.ZERO);
        double initial=s.kineticEnergy(BOX),heat=0,last=Double.NaN;
        for(int i=0;i<600;i++) {
            ContactSolver.Result result=solveStep(s,BOX,BOX_SHAPE,FLOOR,new ContactMaterial(0.4,0),DOWN);
            heat+=result.frictionDissipationW()*DT; last=result.frictionDissipationW();
            s=finish(result.state(),BOX,BOX_SHAPE,FLOOR);
        }
        assertTrue(s.velocity().length()<1e-6,"Box must stop");
        assertEquals(initial,heat,initial*0.01);
        assertEquals(0,last,1e-9,"Sticking contacts must not generate heat");
    }
    @Test void rollingWithoutSlipStopsFrictionHeating() {
        // Sliding-to-rolling sphere: friction removes 2/7 of the initial kinetic energy, then none.
        RigidBodyState s=new RigidBodyState(new Vec3d(0,0.5,0),new Vec3d(5,0,0),Orientation.IDENTITY,Vec3d.ZERO);
        double heat=0,late=0;
        for(int i=0;i<600;i++) {
            ContactSolver.Result result=solveStep(s,BALL,BALL_SHAPE,FLOOR,new ContactMaterial(0.3,0),DOWN);
            heat+=result.frictionDissipationW()*DT;
            if(i>=400) {late=Math.max(late,Math.abs(result.frictionDissipationW()));}
            s=finish(result.state(),BALL,BALL_SHAPE,FLOOR);
        }
        // Power is evaluated at the end-of-step slip, which drops by 3.5 mu g dt each step: the sum undershoots by
        // 1/2 mu m g (3.5 mu g dt) t* = 0.037 J here (about 1 %), an O(dt) term that vanishes as dt -> 0.
        double expected=0.5*25*2/7,undershoot=0.5*0.3*G*(3.5*0.3*G*DT)*(10/(7*0.3*G));
        assertEquals(expected-undershoot,heat,expected*0.002);
        assertEquals(0,late,1e-9);
    }
    @Test void contactHeatPartitionEqualizesInterfaceTemperatures() {
        // Semi-infinite solid under constant surface flux q: dT = 2 q sqrt(t/pi) / e (Carslaw and Jaeger).
        double steel=HeatTransfer.effusivity(45,7850,470),granite=HeatTransfer.effusivity(2.9,2700,790);
        double share=HeatTransfer.contactHeatPartition(steel,granite),q=1000,t=3;
        assertEquals(2*share*q*Math.sqrt(t/Math.PI)/steel,2*(1-share)*q*Math.sqrt(t/Math.PI)/granite,1e-9);
        assertEquals(1,share+HeatTransfer.contactHeatPartition(granite,steel),1e-12);
        assertEquals(0.5,HeatTransfer.contactHeatPartition(steel,steel),1e-12);
        assertTrue(share>0.8 && share<0.9,"Steel takes most of the heat against granite: "+share);
    }
    @Test void boxHoldsBelowTheFrictionAngleAndSlidesAboveIt() {
        // Tilted gravity is equivalent to an incline. Static if tan(theta) <= mu, else a = g (sin - mu cos).
        ContactMaterial material=new ContactMaterial(0.5,0);
        assertEquals(0,slideAcceleration(20,material),2e-3);
        double theta=Math.toRadians(35);
        assertEquals(G*(Math.sin(theta)-0.5*Math.cos(theta)),slideAcceleration(35,material),0.02);
    }
    @Test void restingBoxCarriesItsWeightWithoutDrift() {
        RigidBodyState s=new RigidBodyState(new Vec3d(0,0.25,0),Vec3d.ZERO,Orientation.IDENTITY,Vec3d.ZERO);
        List<Wrench> contact=new ArrayList<>();
        for(int i=0;i<1000;i++) {s=step(s,BOX,BOX_SHAPE,FLOOR,new ContactMaterial(0.5,0.2),DOWN,contact);}
        Wrench mean=mean(contact.subList(800,1000));
        assertEquals(10*G,mean.force().y(),10*G*1e-4);
        assertEquals(0,mean.torque().length(),1e-3);
        assertTrue(s.velocity().length()<1e-5,"Resting box must not creep: "+s.velocity());
        assertEquals(0.25,s.position().y(),ContactSolver.SLOP);
        assertEquals(0,s.position().x(),1e-9);
    }
    @Test void bounceHeightScalesWithRestitutionSquared() {
        RigidBodyState s=new RigidBodyState(new Vec3d(0,1.5,0),Vec3d.ZERO,Orientation.IDENTITY,Vec3d.ZERO);
        boolean rising=false; double apex=0;
        for(int i=0;i<800;i++) {
            s=step(s,BALL,BALL_SHAPE,FLOOR,new ContactMaterial(0,0.5),DOWN,null);
            if(s.velocity().y()>0) {rising=true;}
            if(rising) {apex=Math.max(apex,s.position().y()-0.5);}
            if(rising && s.velocity().y()<0) {break;}
        }
        assertTrue(rising,"Ball must bounce");
        assertEquals(0.25,apex,0.03);
    }
    @Test void velocitySolveNeverAddsKineticEnergyWithoutRestitution() {
        // With e = 0 the converged impulses project velocity onto a convex set containing zero, which cannot
        // raise kinetic energy; speculative gaps only relax that set. The tolerance covers finite PGS iterations.
        Random random=new Random(42);
        List<WorldBox> corner=List.of(FLOOR.getFirst(),new WorldBox(new Vec3d(0.3,-1,-100),new Vec3d(2,10,100)));
        for(int trial=0;trial<200;trial++) {
            RigidBodyState s=new RigidBodyState(new Vec3d(-random.nextDouble()*0.3,0.1+random.nextDouble()*0.3,0),
                    randomVector(random,6),Orientation.rotationVector(randomVector(random,3)),randomVector(random,8));
            List<ContactSolver.Contact> contacts=ContactSolver.generate(s,BOX,BOX_SHAPE,corner,DT);
            RigidBodyState after=ContactSolver.solve(s,BOX,contacts,new ContactMaterial(random.nextDouble(),0),DT).state();
            assertTrue(after.kineticEnergy(BOX)<=s.kineticEnergy(BOX)*(1+1e-3)+1e-9,
                    "Trial "+trial+": "+s.kineticEnergy(BOX)+" -> "+after.kineticEnergy(BOX));
        }
    }
    @Test void frictionNeverGeneratesWorkInResolvedContacts() {
        // Converged Coulomb friction opposes slip at every contact. States are placed as the simulation leaves them
        // (penetration within slop, often touching floor and wall at once); the tolerance is the PGS residual.
        Random random=new Random(5);
        List<WorldBox> corner=List.of(FLOOR.getFirst(),new WorldBox(new Vec3d(2,0,-100),new Vec3d(3,10,100)));
        int loaded=0;
        for(int trial=0;trial<500;trial++) {
            Orientation q=Orientation.rotationVector(randomVector(random,3));
            double minY=Double.MAX_VALUE,maxX=-Double.MAX_VALUE;
            for(CollisionSphere c:BOX_SHAPE) {
                Vec3d p=q.rotate(c.centerLocal());
                minY=Math.min(minY,p.y()-c.radiusM()); maxX=Math.max(maxX,p.x()+c.radiusM());
            }
            double gapY=-0.004+random.nextDouble()*0.02,gapX=random.nextBoolean()?-0.004+random.nextDouble()*0.02:1;
            RigidBodyState s=new RigidBodyState(new Vec3d(2-gapX-maxX,gapY-minY,0),randomVector(random,6),q,randomVector(random,8));
            ContactSolver.Result result=ContactSolver.solve(s,BOX,ContactSolver.generate(s,BOX,BOX_SHAPE,corner,DT),
                    new ContactMaterial(random.nextDouble(),0),DT);
            if(!result.forces().isEmpty()) {loaded++;}
            assertTrue(result.frictionDissipationW()>=-1e-4*Math.max(1,s.kineticEnergy(BOX)/DT),
                    "Trial "+trial+": friction power "+result.frictionDissipationW());
        }
        assertTrue(loaded>100,"Trials must exercise loaded contacts: "+loaded);
    }
    @Test void inelasticTumblingIntoACornerLosesMechanicalEnergy() {
        Random random=new Random(9);
        List<WorldBox> corner=List.of(FLOOR.getFirst(),new WorldBox(new Vec3d(2,-1,-100),new Vec3d(3,10,100)));
        for(int trial=0;trial<10;trial++) {
            RigidBodyState s=new RigidBodyState(new Vec3d(0,1.5,0),new Vec3d(3+random.nextDouble()*3,-1,random.nextDouble()-0.5),
                    Orientation.rotationVector(randomVector(random,2)),randomVector(random,10));
            double initial=mechanical(s),peak=initial;
            for(int i=0;i<600;i++) {
                s=step(s,BOX,BOX_SHAPE,corner,new ContactMaterial(0.6,0),DOWN,null);
                peak=Math.max(peak,mechanical(s));
                assertTrue(s.position().x()<2,"Box must stay on its side of the wall");
            }
            // Position projection may lift the body by at most the resolved penetration.
            assertTrue(peak<=initial+BOX.massKg()*G*0.01,"Energy rose from "+initial+" to "+peak);
            assertTrue(mechanical(s)<initial*0.8,"Inelastic impacts and friction must dissipate energy");
        }
    }
    @Test void speculativeContactsStopAFastSphereAtAThinWall() {
        List<WorldBox> wall=List.of(new WorldBox(new Vec3d(1,-10,-10),new Vec3d(1.05,10,10)));
        RigidBodyProperties pellet=new RigidBodyProperties(1,Vec3d.ZERO,new InertiaTensor(1e-3,1e-3,1e-3,0,0,0));
        List<CollisionSphere> shape=List.of(new CollisionSphere(Vec3d.ZERO,0.05));
        RigidBodyState s=new RigidBodyState(Vec3d.ZERO,new Vec3d(60,0,0),Orientation.IDENTITY,Vec3d.ZERO);
        for(int i=0;i<100;i++) {
            s=step(s,pellet,shape,wall,new ContactMaterial(0,0),Vec3d.ZERO,null);
            assertTrue(s.position().x()<=0.95+ContactSolver.SLOP,"Tunnelled to "+s.position().x());
        }
        assertEquals(0,s.velocity().x(),1e-9);
    }
    @Test void adjacentBlocksDoNotPushThroughSharedFaces() {
        List<WorldBox> floor=List.of(new WorldBox(new Vec3d(0,-1,0),new Vec3d(1,0,1)),new WorldBox(new Vec3d(1,-1,0),new Vec3d(2,0,1)));
        RigidBodyState s=new RigidBodyState(new Vec3d(0.999,-0.05,0.5),Vec3d.ZERO,Orientation.IDENTITY,Vec3d.ZERO);
        List<ContactSolver.Contact> contacts=ContactSolver.generate(s,BALL,List.of(new CollisionSphere(Vec3d.ZERO,0.1)),floor,DT);
        assertFalse(contacts.isEmpty());
        for(ContactSolver.Contact c:contacts) {assertEquals(Vec3d.Y,c.normal(),"Seam must not create sideways normals");}
    }
    @Test void platformOnDryGroundIsInStaticEquilibrium() {
        RigidBodyProperties p=TestPlatform.properties(new Vec3d(0.4,-0.35,0.2));
        FloatingBodyDynamics.Collision collision=new FloatingBodyDynamics.Collision(TestPlatform.collisionShape(new Vec3d(0.4,-0.35,0.2)),FLOOR,TestPlatform.CONTACT_MATERIAL);
        RigidBodyState s=TestPlatform.atModelOrigin(new Vec3d(0,0.7,0),Orientation.IDENTITY,p);
        FloatingBodyDynamics.Result result=null;
        List<Wrench> contact=new ArrayList<>();
        for(int i=0;i<1600;i++) {
            result=FloatingBodyDynamics.step(s,p,TestPlatform.FLOATS,TestPlatform.flatWater(-10),DOWN,0.47,List.of(),collision,DT);
            Wrench sum=result.hydrostatics().add(result.averageDrag()).add(result.inertia()).add(result.contact());
            assertEquals(0,result.state().velocity().subtract(s.velocity()).scale(p.massKg()/DT).subtract(sum.force()).length(),1e-6);
            assertEquals(0,result.state().angularMomentum().subtract(s.angularMomentum()).scale(1/DT).subtract(sum.torque()).length(),1e-6);
            contact.add(result.contact());
            s=result.state();
        }
        assertTrue(s.velocity().length()<1e-3 && s.angularVelocity(p).length()<1e-3,"Platform must come to rest");
        assertEquals(-result.hydrostatics().force().y(),mean(contact.subList(1400,1600)).force().y(),1e-3*p.massKg()*G);
        double minX=Double.MAX_VALUE,maxX=-Double.MAX_VALUE,minZ=Double.MAX_VALUE,maxZ=-Double.MAX_VALUE;
        for(AppliedForce f:result.forces()) {
            if(f.kind()!=AppliedForce.Kind.CONTACT) {continue;}
            minX=Math.min(minX,f.offset().x());maxX=Math.max(maxX,f.offset().x());
            minZ=Math.min(minZ,f.offset().z());maxZ=Math.max(maxZ,f.offset().z());
        }
        // The COM (offset origin) must project inside the support polygon's bounding rectangle.
        assertTrue(minX<=0 && maxX>=0 && minZ<=0 && maxZ>=0,"COM outside support: x["+minX+","+maxX+"] z["+minZ+","+maxZ+"]");
    }

    private static double slideAcceleration(double degrees,ContactMaterial material) {
        double t=Math.toRadians(degrees);
        Vec3d gravity=new Vec3d(G*Math.sin(t),-G*Math.cos(t),0);
        RigidBodyState s=new RigidBodyState(new Vec3d(0,0.25,0),Vec3d.ZERO,Orientation.IDENTITY,Vec3d.ZERO);
        for(int i=0;i<100;i++) {s=step(s,BOX,BOX_SHAPE,FLOOR,material,gravity,null);}
        double start=s.velocity().x();
        for(int i=0;i<200;i++) {s=step(s,BOX,BOX_SHAPE,FLOOR,material,gravity,null);}
        return s.velocity().x()-start;
    }
    private static RigidBodyState step(RigidBodyState s,RigidBodyProperties p,List<CollisionSphere> shape,List<WorldBox> obstacles,
                                       ContactMaterial material,Vec3d gravity,List<Wrench> contact) {
        ContactSolver.Result result=solveStep(s,p,shape,obstacles,material,gravity);
        if(contact!=null) {contact.add(result.wrench());}
        return finish(result.state(),p,shape,obstacles);
    }
    private static ContactSolver.Result solveStep(RigidBodyState s,RigidBodyProperties p,List<CollisionSphere> shape,List<WorldBox> obstacles,
                                                  ContactMaterial material,Vec3d gravity) {
        s=RigidBodyDynamics.kick(s,p,new Wrench(gravity.scale(p.massKg()),Vec3d.ZERO),DT);
        return ContactSolver.solve(s,p,ContactSolver.generate(s,p,shape,obstacles,DT),material,DT);
    }
    private static RigidBodyState finish(RigidBodyState s,RigidBodyProperties p,List<CollisionSphere> shape,List<WorldBox> obstacles) {
        return ContactSolver.separate(RigidBodyDynamics.drift(s,p,DT),p,shape,obstacles);
    }
    private static Wrench mean(List<Wrench> wrenches) {
        Wrench sum=wrenches.stream().reduce(Wrench.ZERO,Wrench::add);
        return new Wrench(sum.force().scale(1.0/wrenches.size()),sum.torque().scale(1.0/wrenches.size()));
    }
    private static double mechanical(RigidBodyState s) {return s.kineticEnergy(BOX)+BOX.massKg()*G*s.position().y();}
    private static Vec3d randomVector(Random random,double scale) {
        return new Vec3d(random.nextDouble()*2-1,random.nextDouble()*2-1,random.nextDouble()*2-1).scale(scale);
    }
    private static List<CollisionSphere> corners(double x,double y,double z,double radius) {
        List<CollisionSphere> shape=new ArrayList<>();
        for(int i=-1;i<=1;i+=2) for(int j=-1;j<=1;j+=2) for(int k=-1;k<=1;k+=2) {
            shape.add(new CollisionSphere(new Vec3d(i*(x-radius),j*(y-radius),k*(z-radius)),radius));
        }
        return shape;
    }
}
