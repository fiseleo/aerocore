package dev.mcmod.aerocore.api;

import java.util.List;
import java.util.Objects;

/** Fixed-step momentum integration. Forces/torques and offsets are world-frame quantities. */
public final class RigidBodyDynamics {
    private RigidBodyDynamics() {}
    public record PointDrag(Vec3d offset, Vec3d fluidVelocity, double linearResistance, double quadraticResistance) {
        public PointDrag {
            Objects.requireNonNull(offset); Objects.requireNonNull(fluidVelocity);
            Checks.nonnegative(linearResistance, "Linear drag"); Checks.nonnegative(quadraticResistance, "Quadratic drag");
        }
    }
    /** Isotropic point added mass in kg at a world offset from the COM, for example C_a rho V of a wet sphere. */
    public record PointMass(Vec3d offset, double massKg) {
        public PointMass { Objects.requireNonNull(offset); Checks.nonnegative(massKg, "Added mass"); }
    }
    /**
     * One implicit velocity update for load W, point drag C and added mass Ma:
     * (M + Ma + C dt) nu1 = (M + Ma) nu0 + dt W + C dt u_f, with nu = (v, omega) and Ma = sum m_i J_i^T J_i,
     * J_i nu = v + omega x r_i. The added mass therefore resists changes of point velocity implicitly, which
     * stays stable when it exceeds the body mass. Quadratic drag coefficients are frozen at the speeds after
     * an explicit kick by W, so with no added mass this is exactly kick() followed by drag().
     */
    public static RigidBodyState implicitVelocity(RigidBodyState s, RigidBodyProperties p, Wrench load,
                                                  List<PointDrag> points, List<PointMass> added, double dt) {
        Checks.positive(dt, "Time step");
        RigidBodyState kicked = kick(s, p, load, dt);
        if (added.isEmpty()) { return drag(kicked, p, points, dt); }
        double[][] a = massMatrix(s, p);
        Vec3d momentum = kicked.velocity().scale(p.massKg());
        double[] b = {momentum.x(), momentum.y(), momentum.z(), kicked.angularMomentum().x(), kicked.angularMomentum().y(), kicked.angularMomentum().z()};
        Vec3d omega = s.angularVelocity(p);
        for (PointMass point : added) {
            double[][] j = jacobian(point.offset());
            double[] velocity = values(s.velocity().add(omega.cross(point.offset())));
            accumulate(a, b, j, point.massKg(), velocity);
        }
        Vec3d kickedOmega = kicked.angularVelocity(p);
        for (PointDrag point : points) {
            Vec3d r = point.offset();
            double speed = kicked.velocity().add(kickedOmega.cross(r)).subtract(point.fluidVelocity()).length();
            accumulate(a, b, jacobian(r), (point.linearResistance()+point.quadraticResistance()*speed)*dt, values(point.fluidVelocity()));
        }
        double[] velocity = solvePositiveDefinite(a,b);
        return new RigidBodyState(s.position(), new Vec3d(velocity[0],velocity[1],velocity[2]), s.orientation(),
                p.inertia().applyWorld(s.orientation(),new Vec3d(velocity[3],velocity[4],velocity[5])));
    }
    /**
     * Implicit velocity update (M + A) nu1 = (M + A) nu0 + dt W for a general symmetric positive semi-definite added-mass
     * matrix A (6 x 6, world axes, about the COM, ordered v then omega), for example a hull's anisotropic strip added
     * mass. Stable for any A; velocity-dependent fluid forces must be supplied in W.
     */
    public static RigidBodyState implicitVelocity(RigidBodyState s, RigidBodyProperties p, Wrench load, double[][] addedMass, double dt) {
        Checks.positive(dt, "Time step");
        if (addedMass.length != 6) { throw new IllegalArgumentException("Added mass must be 6 x 6"); }
        double[][] a = massMatrix(s, p);
        Vec3d omega = s.angularVelocity(p);
        double[] nu = {s.velocity().x(), s.velocity().y(), s.velocity().z(), omega.x(), omega.y(), omega.z()};
        Vec3d momentum = s.velocity().scale(p.massKg()).add(load.force().scale(dt));
        Vec3d angular = s.angularMomentum().add(load.torque().scale(dt));
        double[] b = {momentum.x(), momentum.y(), momentum.z(), angular.x(), angular.y(), angular.z()};
        for (int i = 0; i < 6; i++) {
            if (addedMass[i].length != 6) { throw new IllegalArgumentException("Added mass must be 6 x 6"); }
            for (int j = 0; j < 6; j++) {
                if (!Double.isFinite(addedMass[i][j]) || Math.abs(addedMass[i][j] - addedMass[j][i]) > 1e-9 * (1 + Math.abs(addedMass[i][j]))) {
                    throw new IllegalArgumentException("Added mass must be finite and symmetric");
                }
                a[i][j] += addedMass[i][j];
                b[i] += addedMass[i][j] * nu[j];
            }
        }
        double[] velocity = solvePositiveDefinite(a, b);
        return new RigidBodyState(s.position(), new Vec3d(velocity[0], velocity[1], velocity[2]), s.orientation(),
                p.inertia().applyWorld(s.orientation(), new Vec3d(velocity[3], velocity[4], velocity[5])));
    }
    /**
     * The general implicit velocity update of a composite body (VehicleDynamics): a 6 x 6 added mass A (world axes, about
     * the COM, as in the overload taking a matrix), point added masses and point drags (as in the overload taking lists),
     * all at once:
     * (M + A + sum m_i J_i^T J_i + sum c_i dt J_i^T J_i) nu1 = M nu0 + dt W + A nu0 + sum m_i J_i^T u_i0 + sum c_i dt J_i^T u_f.
     * Quadratic drag coefficients are frozen at the speeds after an explicit kick by W, as there. With A null and the
     * lists empty or not, it equals the other two overloads up to rounding.
     */
    public static RigidBodyState implicitVelocity(RigidBodyState s, RigidBodyProperties p, Wrench load, double[][] addedMass,
                                                  List<PointDrag> points, List<PointMass> added, double dt) {
        Checks.positive(dt, "Time step");
        RigidBodyState kicked = kick(s, p, load, dt);
        double[][] a = massMatrix(s, p);
        Vec3d momentum = kicked.velocity().scale(p.massKg());
        double[] b = {momentum.x(), momentum.y(), momentum.z(), kicked.angularMomentum().x(), kicked.angularMomentum().y(), kicked.angularMomentum().z()};
        Vec3d omega = s.angularVelocity(p);
        if (addedMass != null) {
            if (addedMass.length != 6) { throw new IllegalArgumentException("Added mass must be 6 x 6"); }
            double[] nu = {s.velocity().x(), s.velocity().y(), s.velocity().z(), omega.x(), omega.y(), omega.z()};
            for (int i = 0; i < 6; i++) {
                if (addedMass[i].length != 6) { throw new IllegalArgumentException("Added mass must be 6 x 6"); }
                for (int j = 0; j < 6; j++) {
                    if (!Double.isFinite(addedMass[i][j]) || Math.abs(addedMass[i][j] - addedMass[j][i]) > 1e-9 * (1 + Math.abs(addedMass[i][j]))) {
                        throw new IllegalArgumentException("Added mass must be finite and symmetric");
                    }
                    a[i][j] += addedMass[i][j];
                    b[i] += addedMass[i][j] * nu[j];
                }
            }
        }
        for (PointMass point : added) {
            accumulate(a, b, jacobian(point.offset()), point.massKg(), values(s.velocity().add(omega.cross(point.offset()))));
        }
        Vec3d kickedOmega = kicked.angularVelocity(p);
        for (PointDrag point : points) {
            Vec3d r = point.offset();
            double speed = kicked.velocity().add(kickedOmega.cross(r)).subtract(point.fluidVelocity()).length();
            accumulate(a, b, jacobian(r), (point.linearResistance() + point.quadraticResistance() * speed) * dt, values(point.fluidVelocity()));
        }
        double[] velocity = solvePositiveDefinite(a, b);
        return new RigidBodyState(s.position(), new Vec3d(velocity[0], velocity[1], velocity[2]), s.orientation(),
                p.inertia().applyWorld(s.orientation(), new Vec3d(velocity[3], velocity[4], velocity[5])));
    }
    /**
     * Torque-free drift of the body and its added point masses as one rigid system (body mass plus entrained
     * water), so free rotation keeps the system's momentum and kinetic energy rather than the body's alone.
     * The added masses are held at their start-of-step body positions for this step; with none it is drift().
     */
    public static RigidBodyState driftWithPointMasses(RigidBodyState s, RigidBodyProperties p, List<PointMass> added, double dt) {
        double extra=0;
        Vec3d moment=Vec3d.ZERO;
        for(PointMass m:added) {extra+=m.massKg();moment=moment.add(m.offset().scale(m.massKg()));}
        if(extra==0) {return drift(s,p,dt);}
        double total=p.massKg()+extra;
        Orientation q=s.orientation(),inverse=q.inverse();
        Vec3d c=moment.scale(1/total),cBody=inverse.rotate(c);
        InertiaTensor inertia=p.inertia().shifted(p.massKg(),cBody);
        for(PointMass m:added) {inertia=inertia.shifted(m.massKg(),inverse.rotate(m.offset()).subtract(cBody));}
        RigidBodyProperties system=new RigidBodyProperties(total,p.centerOfMassLocal().add(cBody),inertia);
        Vec3d omega=s.angularVelocity(p);
        RigidBodyState virtual=new RigidBodyState(s.position().add(c),s.velocity().add(omega.cross(c)),q,inertia.applyWorld(q,omega));
        RigidBodyState drifted=drift(virtual,system,dt);
        Orientation q1=drifted.orientation();
        Vec3d omega1=drifted.angularVelocity(system),c1=q1.rotate(cBody);
        return new RigidBodyState(drifted.position().subtract(c1),drifted.velocity().subtract(omega1.cross(c1)),q1,p.inertia().applyWorld(q1,omega1));
    }
    private static double[][] massMatrix(RigidBodyState s, RigidBodyProperties p) {
        double[][] a = new double[6][6];
        for (int i=0; i<3; i++) { a[i][i] = p.massKg(); }
        Vec3d[] axes = {Vec3d.X, Vec3d.Y, Vec3d.Z};
        for (int j=0; j<3; j++) {
            double[] column = values(p.inertia().applyWorld(s.orientation(), axes[j]));
            for (int i=0; i<3; i++) { a[i+3][j+3] = column[i]; }
        }
        return a;
    }
    /** Rows of J with J nu = v + omega x r. */
    private static double[][] jacobian(Vec3d r) {
        return new double[][]{{1,0,0,0,r.z(),-r.y()}, {0,1,0,-r.z(),0,r.x()}, {0,0,1,r.y(),-r.x(),0}};
    }
    /** a += w J^T J, b += w J^T u. */
    private static void accumulate(double[][] a, double[] b, double[][] j, double weight, double[] u) {
        for (int row=0; row<6; row++) {
            for (int k=0; k<3; k++) { b[row] += weight*j[k][row]*u[k]; }
            for (int col=0; col<6; col++) {
                for (int k=0; k<3; k++) { a[row][col] += weight*j[k][row]*j[k][col]; }
            }
        }
    }
    public static RigidBodyState kick(RigidBodyState s, RigidBodyProperties p, Wrench wrench, double dt) {
        Checks.positive(dt, "Time step");
        return new RigidBodyState(s.position(), s.velocity().add(wrench.force().scale(dt/p.massKg())),
                s.orientation(), s.angularMomentum().add(wrench.torque().scale(dt)));
    }
    /**
     * Simultaneous backward-Euler point drag, with quadratic coefficients frozen at input speed.
     * The symmetric positive-definite 6x6 solve dissipates kinetic energy in stationary fluid,
     * including coupling between translation and rotation. No order-dependent per-corner impulses.
     */
    public static RigidBodyState drag(RigidBodyState s, RigidBodyProperties p, List<PointDrag> points, double dt) {
        Checks.positive(dt, "Time step");
        if (points.isEmpty()) { return s; }
        double[][] a = new double[6][6];
        for (int i=0; i<3; i++) { a[i][i] = p.massKg(); }
        Vec3d[] axes = {Vec3d.X, Vec3d.Y, Vec3d.Z};
        for (int j=0; j<3; j++) {
            double[] column = values(p.inertia().applyWorld(s.orientation(), axes[j]));
            for (int i=0; i<3; i++) { a[i+3][j+3] = column[i]; }
        }
        Vec3d momentum = s.velocity().scale(p.massKg());
        double[] b = {momentum.x(), momentum.y(), momentum.z(), s.angularMomentum().x(), s.angularMomentum().y(), s.angularMomentum().z()};
        Vec3d omega = s.angularVelocity(p);
        for (PointDrag point : points) {
            Vec3d r = point.offset();
            double speed = s.velocity().add(omega.cross(r)).subtract(point.fluidVelocity()).length();
            double cdt = (point.linearResistance()+point.quadraticResistance()*speed)*dt;
            double[][] j = {{1,0,0,0,r.z(),-r.y()}, {0,1,0,-r.z(),0,r.x()}, {0,0,1,r.y(),-r.x(),0}};
            double[] flow = values(point.fluidVelocity());
            for (int row=0; row<6; row++) {
                for (int k=0; k<3; k++) { b[row] += cdt*j[k][row]*flow[k]; }
                for (int col=0; col<6; col++) {
                    for (int k=0; k<3; k++) { a[row][col] += cdt*j[k][row]*j[k][col]; }
                }
            }
        }
        double[] velocity = solvePositiveDefinite(a,b);
        Vec3d linear = new Vec3d(velocity[0],velocity[1],velocity[2]);
        Vec3d angular = new Vec3d(velocity[3],velocity[4],velocity[5]);
        return new RigidBodyState(s.position(), linear, s.orientation(), p.inertia().applyWorld(s.orientation(),angular));
    }
    /** Torque-free drift keeps world angular momentum fixed; midpoint orientation includes gyroscopic motion. */
    public static RigidBodyState drift(RigidBodyState s, RigidBodyProperties p, double dt) {
        Checks.positive(dt, "Time step");
        Orientation q = driftRotation(s.orientation(), s.angularMomentum(), p.inertia(), dt, 0);
        return new RigidBodyState(s.position().add(s.velocity().scale(dt)), s.velocity(), q, s.angularMomentum());
    }
    private static Orientation driftRotation(Orientation initial, Vec3d momentum, InertiaTensor inertia, double dt, int depth) {
        Vec3d omega = inertia.solveWorld(initial, momentum);
        for (int i=0; i<24; i++) {
            Orientation midpoint = initial.advanceWorld(omega, dt/2);
            Vec3d next = inertia.solveWorld(midpoint, momentum);
            if (next.subtract(omega).length()*dt < 1e-12) { return initial.advanceWorld(next, dt); }
            omega = next;
        }
        if (depth >= 10) { throw new IllegalArgumentException("Rotation step failed to converge; reduce time step or angular momentum"); }
        Orientation half = driftRotation(initial,momentum,inertia,dt/2,depth+1);
        return driftRotation(half,momentum,inertia,dt/2,depth+1);
    }
    private static double[] values(Vec3d v) { return new double[]{v.x(),v.y(),v.z()}; }
    private static double[] solvePositiveDefinite(double[][] a, double[] b) {
        double[][] l = new double[6][6];
        for (int i=0; i<6; i++) {
            for (int j=0; j<=i; j++) {
                double value = a[i][j];
                for (int k=0; k<j; k++) { value -= l[i][k]*l[j][k]; }
                if (i==j) {
                    if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException("Singular drag system"); }
                    l[i][j] = Math.sqrt(value);
                } else { l[i][j] = value/l[j][j]; }
            }
        }
        double[] x = b.clone();
        for (int i=0; i<6; i++) {
            for (int j=0; j<i; j++) { x[i] -= l[i][j]*x[j]; }
            x[i] /= l[i][i];
        }
        for (int i=5; i>=0; i--) {
            for (int j=i+1; j<6; j++) { x[i] -= l[j][i]*x[j]; }
            x[i] /= l[i][i];
        }
        return x;
    }
}
