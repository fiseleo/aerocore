package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Non-overlapping spherical displacement elements attached to a rigid body. Each wet element carries
 * potential-flow added mass and, in waves, Froude-Krylov forces with a Morison diffraction term. Wave radiation
 * memory and the exact heave diffraction of surface-piercing floats come from WaveRadiation.Memory as external loads.
 */
public final class FloatingBodyDynamics {
    /** Pass as the drag coefficient to use the smooth-sphere standard drag curve (SphereDrag) at each sphere's Reynolds number. */
    public static final double STANDARD_DRAG = -1;
    /** Added-mass coefficient of a sphere in unbounded ideal fluid (Newman, Marine Hydrodynamics, 1977). */
    public static final double ADDED_MASS_COEFFICIENT = 0.5;
    private FloatingBodyDynamics() {}
    /**
     * Wet centroid and pressure centroid are world-direction offsets from the sphere center, not from the body COM.
     * pressureFraction is the vertical pressure force over rho g V (equal to fraction in steady water); the
     * buoyancy uses it at pressureCentroidOffset. waterAcceleration is the local wave acceleration at the wet
     * centroid; it drives added mass and the horizontal Froude-Krylov force. The Morison diffraction term
     * C_a rho V_wet a_f acts in all directions unless surfaceDiffraction is set: then its vertical part is left out,
     * because the caller supplies the exact heave diffraction of a surface-piercing float
     * (WaveRadiation.Memory.diffractionLoads) as an external load.
     */
    public record Contact(double fraction, Vec3d wetCentroidOffset, FluidState air, FluidState water,
                          double pressureFraction, Vec3d pressureCentroidOffset, Vec3d waterAcceleration,
                          boolean surfaceDiffraction) {
        /** Steady water: pressure follows geometry and the water does not accelerate. */
        public Contact(double fraction, Vec3d wetCentroidOffset, FluidState air, FluidState water) {
            this(fraction, wetCentroidOffset, air, water, fraction, wetCentroidOffset, Vec3d.ZERO, false);
        }
        /** Waves with the Morison diffraction approximation in every direction. */
        public Contact(double fraction, Vec3d wetCentroidOffset, FluidState air, FluidState water,
                       double pressureFraction, Vec3d pressureCentroidOffset, Vec3d waterAcceleration) {
            this(fraction, wetCentroidOffset, air, water, pressureFraction, pressureCentroidOffset, waterAcceleration, false);
        }
        public Contact {
            if (!Double.isFinite(fraction) || fraction < 0 || fraction > 1) { throw new IllegalArgumentException("Wet fraction must be in [0,1]"); }
            if (!Double.isFinite(pressureFraction) || pressureFraction < -1 || pressureFraction > 3) {
                throw new IllegalArgumentException("Pressure fraction must be in [-1,3]");
            }
            Objects.requireNonNull(wetCentroidOffset); Objects.requireNonNull(air); Objects.requireNonNull(water);
            Objects.requireNonNull(pressureCentroidOffset); Objects.requireNonNull(waterAcceleration);
        }
    }
    @FunctionalInterface public interface Environment { Contact sample(Vec3d sphereCenter, double radius); }
    /** Static obstacles for ContactSolver; shape is model-local, obstacles are world boxes. */
    public record Collision(List<CollisionSphere> shape, List<WorldBox> obstacles, ContactMaterial material) {
        public Collision {
            shape = List.copyOf(shape); obstacles = List.copyOf(obstacles); Objects.requireNonNull(material);
        }
    }
    /**
     * Mechanical power removed by drag at the end of the step, W, per medium. It is the sum of c|u|^2 over
     * drag elements (u relative to the local fluid), so it is non-negative and frame independent. Mean square
     * speeds are weighted by dry (air) or wet (water) area fraction, m^2/s^2, for recovery-temperature heating.
     */
    public record DragDissipation(double airW, double waterW, double airMeanSquareSpeed, double waterMeanSquareSpeed) {
        public static final DragDissipation NONE = new DragDissipation(0, 0, 0, 0);
        public double totalW() { return airW + waterW; }
    }
    /**
     * hydrostatics includes weight; inertia is added-mass reaction plus horizontal Froude-Krylov force, including
     * the momentum exchanged with entrained water while rotating; external echoes caller loads. All wrenches use
     * the step's starting COM and sum to the body's momentum change. contactFrictionW is frictional heat power.
     */
    public record Result(RigidBodyState state, Wrench hydrostatics, Wrench averageDrag, Wrench inertia, Wrench contact,
                         Wrench external, List<Double> wetFractions, List<AppliedForce> forces, DragDissipation dissipation,
                         double contactFrictionW) {}

    public static Result step(RigidBodyState state, RigidBodyProperties properties, List<BuoyantSphere> floats,
                              Environment environment, Vec3d gravity, double cd, double dt) {
        return step(state, properties, floats, environment, gravity, cd, List.of(), null, dt);
    }

    /**
     * Implicit velocity update with weight, buoyancy, caller loads, wave excitation, drag and added mass; contact
     * impulses (when collision is not null); drift of body plus entrained water; then position projection.
     * Do not add weight, buoyancy, float drag or added mass to externalLoads.
     */
    public static Result step(RigidBodyState state, RigidBodyProperties properties, List<BuoyantSphere> floats,
                              Environment environment, Vec3d gravity, double cd, List<AppliedForce> externalLoads,
                              Collision collision, double dt) {
        Prepared prepared = prepare(state, properties, floats, environment, gravity, cd, externalLoads, dt);
        if (collision == null) { return finish(prepared, prepared.damped(), Wrench.ZERO, List.of(), 0, null); }
        List<ContactSolver.Contact> contacts=ContactSolver.generate(prepared.damped(),properties,collision.shape(),collision.obstacles(),dt);
        ContactSolver.Result solved=ContactSolver.solve(prepared.damped(),properties,contacts,collision.material(),dt);
        return finish(prepared, solved.state(), solved.wrench(), solved.forces(), solved.frictionDissipationW(), collision);
    }

    /**
     * First half of step(): loads and the implicit velocity update, stopping before contact. damped is the
     * pre-contact state at the step's start pose. Several bodies can then share one ContactSolver.solveAll before
     * each calls finish().
     */
    public record Prepared(RigidBodyState start, RigidBodyProperties properties, RigidBodyState damped, List<RigidBodyDynamics.PointMass> added,
                           Wrench hydrostatics, Wrench averageDrag, Wrench inertia, Wrench external, List<Double> wetFractions,
                           List<AppliedForce> forces, DragDissipation dissipation, double dt) {}

    /**
     * Second half of step(): drift of body plus entrained water from the post-contact state, then position
     * projection against collision's static obstacles when it is not null.
     */
    public static Result finish(Prepared p, RigidBodyState afterContact, Wrench contact, List<AppliedForce> contactForces, double contactFrictionW,
                                Collision collision) {
        List<AppliedForce> forces=new ArrayList<>(p.forces());
        forces.addAll(contactForces);
        Wrench inertia=p.inertia();
        RigidBodyState next=driftWithAddedMass(afterContact,p.properties(),p.added(),p.dt());
        Vec3d exchangeForce=next.velocity().subtract(afterContact.velocity()).scale(p.properties().massKg()/p.dt());
        Vec3d exchangeCouple=next.angularMomentum().subtract(afterContact.angularMomentum()).scale(1/p.dt());
        if(exchangeForce.lengthSquared()>0 || exchangeCouple.lengthSquared()>0) {
            AppliedForce exchange=new AppliedForce(AppliedForce.Kind.INERTIA,Vec3d.ZERO,exchangeForce,exchangeCouple);
            forces.add(exchange);
            inertia=inertia.add(exchange.wrench());
        }
        if(collision!=null) {next=ContactSolver.separate(next,p.properties(),collision.shape(),collision.obstacles());}
        return new Result(next,p.hydrostatics(),p.averageDrag(),inertia,contact,p.external(),p.wetFractions(),List.copyOf(forces),p.dissipation(),contactFrictionW);
    }

    public static Prepared prepare(RigidBodyState state, RigidBodyProperties properties, List<BuoyantSphere> floats,
                                   Environment environment, Vec3d gravity, double cd, List<AppliedForce> externalLoads, double dt) {
        Checks.positive(dt,"Time step");
        if (cd != STANDARD_DRAG) { Checks.nonnegative(cd,"Drag coefficient"); }
        Vec3d omega0=state.angularVelocity(properties);
        Wrench hydro = new Wrench(gravity.scale(properties.massKg()), Vec3d.ZERO);
        Vec3d down = gravity.lengthSquared()==0?Vec3d.ZERO:gravity.normalized();
        List<RigidBodyDynamics.PointDrag> drag = new ArrayList<>();
        List<RigidBodyDynamics.PointMass> added = new ArrayList<>();
        List<Vec3d> excitations = new ArrayList<>();
        List<Double> wet = new ArrayList<>();
        List<AppliedForce> forces = new ArrayList<>();
        forces.add(new AppliedForce(AppliedForce.Kind.WEIGHT,Vec3d.ZERO,gravity.scale(properties.massKg())));
        for (BuoyantSphere sphere : floats) {
            Vec3d r = state.worldOffset(properties,sphere.offset());
            Contact contact = environment.sample(state.position().add(r),sphere.radiusM());
            wet.add(contact.fraction());
            double volume=sphere.volumeM3(), air=contact.air().densityKgPerM3(), water=contact.water().densityKgPerM3();
            // Full-volume air buoyancy, then water pressure on the wet part replacing the air it displaces.
            hydro = hydro.add(Wrench.atOffset(gravity.scale(-air*volume),r));
            double displaced = (water*contact.pressureFraction()-air*contact.fraction())*volume;
            hydro = hydro.add(Wrench.atOffset(gravity.scale(-displaced),r.add(contact.pressureCentroidOffset())));
            double totalDisplaced=air*volume+displaced;
            Vec3d buoyancyPoint=totalDisplaced==0?r:r.add(contact.pressureCentroidOffset().scale(displaced/totalDisplaced));
            forces.add(new AppliedForce(AppliedForce.Kind.BUOYANCY,buoyancyPoint,gravity.scale(-totalDisplaced)));
            Vec3d pointVelocity=state.velocity().add(omega0.cross(r));
            addDrag(drag,r,pointVelocity,contact.air(),1-contact.fraction(),sphere.radiusM(),cd);
            addDrag(drag,r,pointVelocity,contact.water(),contact.fraction(),sphere.radiusM(),cd);
            double wetMass=water*contact.fraction()*volume;
            if (wetMass>0) {
                // Morison inertia: rho V a_f (horizontal part; the vertical part is in the pressure integral)
                // plus C_a rho V (a_f - a_body), whose body-acceleration term is implicit via the added mass.
                // With surfaceDiffraction the vertical C_a rho V a_f is replaced by the caller's exact heave diffraction.
                Vec3d a=contact.waterAcceleration(), horizontal=a.subtract(down.scale(a.dot(down)));
                Vec3d diffracted=contact.surfaceDiffraction()?horizontal:a;
                added.add(new RigidBodyDynamics.PointMass(r.add(contact.wetCentroidOffset()),ADDED_MASS_COEFFICIENT*wetMass));
                excitations.add(horizontal.scale(wetMass).add(diffracted.scale(ADDED_MASS_COEFFICIENT*wetMass)));
            }
        }
        Wrench external = Wrench.ZERO;
        for (AppliedForce load : externalLoads) { external = external.add(load.wrench()); forces.add(load); }
        Wrench excitation = Wrench.ZERO;
        for (int i=0;i<added.size();i++) { excitation = excitation.add(Wrench.atOffset(excitations.get(i),added.get(i).offset())); }
        Wrench load = hydro.add(external).add(excitation);
        RigidBodyState kicked = RigidBodyDynamics.kick(state,properties,load,dt);
        RigidBodyState damped = RigidBodyDynamics.implicitVelocity(state,properties,load,drag,added,dt);
        Vec3d beforeOmega=kicked.angularVelocity(properties),afterOmega=damped.angularVelocity(properties);
        double airPower=0,waterPower=0,airSquare=0,waterSquare=0,dryArea=0,wetArea=0;
        Wrench averageDrag=Wrench.ZERO;
        for(int i=0;i<drag.size();i+=2) {
            Vec3d force=Vec3d.ZERO;
            for(int j=i;j<i+2;j++) {
                RigidBodyDynamics.PointDrag point=drag.get(j);
                double speed=kicked.velocity().add(beforeOmega.cross(point.offset())).subtract(point.fluidVelocity()).length();
                double coefficient=point.linearResistance()+point.quadraticResistance()*speed;
                Vec3d relative=damped.velocity().add(afterOmega.cross(point.offset())).subtract(point.fluidVelocity());
                force=force.add(relative.scale(-coefficient));
                // -F.u = c|u|^2 >= 0: power this element's drag removes from the relative motion.
                double power=coefficient*relative.lengthSquared();
                double area=j==i?1-wet.get(i/2):wet.get(i/2);
                if(j==i) {airPower+=power;airSquare+=area*relative.lengthSquared();dryArea+=area;}
                else {waterPower+=power;waterSquare+=area*relative.lengthSquared();wetArea+=area;}
            }
            AppliedForce dragForce=new AppliedForce(AppliedForce.Kind.DRAG,drag.get(i).offset(),force);
            forces.add(dragForce);
            averageDrag=averageDrag.add(dragForce.wrench());
        }
        DragDissipation dissipation=new DragDissipation(airPower,waterPower,
                dryArea>0?airSquare/dryArea:0,wetArea>0?waterSquare/wetArea:0);
        Wrench inertia=Wrench.ZERO;
        Vec3d startOmega=state.angularVelocity(properties);
        for(int i=0;i<added.size();i++) {
            Vec3d r=added.get(i).offset();
            Vec3d acceleration=damped.velocity().add(afterOmega.cross(r)).subtract(state.velocity().add(startOmega.cross(r))).scale(1/dt);
            AppliedForce force=new AppliedForce(AppliedForce.Kind.INERTIA,r,excitations.get(i).subtract(acceleration.scale(added.get(i).massKg())));
            forces.add(force);
            inertia=inertia.add(force.wrench());
        }
        return new Prepared(state,properties,damped,List.copyOf(added),hydro,averageDrag,inertia,external,List.copyOf(wet),List.copyOf(forces),dissipation,dt);
    }

    /**
     * Torque-free drift of the body and its added point masses as one rigid system (body mass plus entrained
     * water), so free rotation keeps the system's momentum and kinetic energy rather than the body's alone.
     * The added masses are held at their start-of-step body positions for this step.
     */
    private static RigidBodyState driftWithAddedMass(RigidBodyState s, RigidBodyProperties p,
                                                     List<RigidBodyDynamics.PointMass> added, double dt) {
        double extra=0;
        Vec3d moment=Vec3d.ZERO;
        for(RigidBodyDynamics.PointMass m:added) {extra+=m.massKg();moment=moment.add(m.offset().scale(m.massKg()));}
        if(extra==0) {return RigidBodyDynamics.drift(s,p,dt);}
        double total=p.massKg()+extra;
        Orientation q=s.orientation(),inverse=q.inverse();
        Vec3d c=moment.scale(1/total),cBody=inverse.rotate(c);
        InertiaTensor inertia=p.inertia().shifted(p.massKg(),cBody);
        for(RigidBodyDynamics.PointMass m:added) {inertia=inertia.shifted(m.massKg(),inverse.rotate(m.offset()).subtract(cBody));}
        RigidBodyProperties system=new RigidBodyProperties(total,p.centerOfMassLocal().add(cBody),inertia);
        Vec3d omega=s.angularVelocity(p);
        RigidBodyState virtual=new RigidBodyState(s.position().add(c),s.velocity().add(omega.cross(c)),q,inertia.applyWorld(q,omega));
        RigidBodyState drifted=RigidBodyDynamics.drift(virtual,system,dt);
        Orientation q1=drifted.orientation();
        Vec3d omega1=drifted.angularVelocity(system),c1=q1.rotate(cBody);
        return new RigidBodyState(drifted.position().subtract(c1),drifted.velocity().subtract(omega1.cross(c1)),q1,p.inertia().applyWorld(q1,omega1));
    }
    /**
     * Drag of one sphere in one fluid, weighted by the fraction of the sphere in it. With STANDARD_DRAG the coefficient
     * follows the smooth-sphere standard curve (SphereDrag, Clift, Grace and Weber 1978) at the start-of-step relative
     * speed, applied implicitly as a linear resistance; otherwise Stokes drag plus a constant C_D (for analytic tests).
     */
    private static void addDrag(List<RigidBodyDynamics.PointDrag> points, Vec3d r, Vec3d pointVelocity, FluidState fluid, double fraction, double radius, double cd) {
        if (cd == STANDARD_DRAG) {
            double speed = pointVelocity.subtract(fluid.velocity()).length();
            points.add(new RigidBodyDynamics.PointDrag(r,fluid.velocity(),SphereDrag.resistance(fluid,radius,speed)*fraction,0));
            return;
        }
        points.add(new RigidBodyDynamics.PointDrag(r,fluid.velocity(),6*Math.PI*fluid.dynamicViscosityPaS()*radius*fraction,
                0.5*fluid.densityKgPerM3()*Math.PI*radius*radius*cd*fraction));
    }
}
