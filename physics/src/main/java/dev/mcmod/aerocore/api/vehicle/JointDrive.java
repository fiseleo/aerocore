package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;

import java.util.List;

/**
 * 0.43: how a vehicle's joints (Part.joint) move in a step, and what their motion does to the rigid body.
 *
 * <p>Each joint carries a subtree (its part and everything attached below it). Its actuator (PartStep.driveJoint) asks
 * for a rate and can push either way up to a generalized force (N m about the axis, or N along it). The force the
 * actuator needs to reach the rate within the step is I (rate - rate0)/dt - Q, with I the subtree's inertia about the
 * joint (its mass for a slide) and Q the generalized force on the subtree from outside it: its weight, the loads its
 * parts wrote this step, and what the contact solve and the tyres' dampers did to it in the last step (taken one
 * substep late). Within the actuator's reach the joint moves at the rate; beyond it the actuator gives its most and the
 * joint accelerates by what is left (it stalls, or an overloaded cylinder gives way). What the contacts and dampers did
 * (the lagged part of Q) only resists: it can slow a joint that gives way to a stop, never drive it on (the contact
 * solve already puts that force on the body; letting it also fling the subtree counted it twice, and a stiff ground's
 * lagged push threw a 20 t excavator into the air when its bucket lay on the ground). Undriven joints hold. The base's
 * coupling through the subtree's inertia is not part of I (the reaction on the base is, through the momentum below).
 *
 * <p>Momentum: the state's velocity is that of the vehicle-frame point at the centre of mass, its angular momentum the
 * vehicle frame's (I omega); the moving parts' momentum against the frame is apart: P (linear) and H (angular, about
 * the centre). A change of P or H between steps is the joints' reaction on the body (-dP/dt, -dH/dt), H rides with the
 * rotors' spin in the drift, and after the drift the state is re-referenced to the centre of mass of the new pose, so
 * internal motion moves the centre of mass only as momentum says and the frame turns back as a skater's arms do.
 */
final class JointDrive {
    private JointDrive() {}

    /** The smallest inertia (kg m^2 or kg) a joint's subtree is taken to have. */
    static final double MIN_INERTIA = 1e-6;

    /** What a part asked of its joint this step: the rate and how hard its actuator can push up and down. */
    record Command(double rate, double up, double down) {}

    /** The momentum of the moving parts against the vehicle frame at a pose and joint rates: P and H (about the centre of mass), vehicle frame. */
    record Relative(Vec3d linear, Vec3d angular) {
        static final Relative NONE = new Relative(Vec3d.ZERO, Vec3d.ZERO);
    }

    /** P and H of the moving parts and the cargo and fuel they carry, vehicle frame. */
    static Relative relative(Vehicle v, VehiclePose pose, VehicleState st, double[] rates) {
        boolean any = false;
        for (double r : rates) { any |= r != 0; }
        if (!any) { return Relative.NONE; }
        Vec3d c = pose.properties().centerOfMassLocal();
        Vec3d p = Vec3d.ZERO, h = Vec3d.ZERO;
        for (Vehicle.Installed i : v.installed()) {
            int part = i.index();
            if (v.chain(part).length == 0) { continue; }
            MassProperties m = pose.partMass(part);
            if (m.massKg() > 0) {
                Vec3d u = pose.relativeVelocity(part, m.centre(), rates);
                p = p.add(u.scale(m.massKg()));
                h = h.add(m.centre().subtract(c).cross(u).scale(m.massKg()));
            }
            Vec3d w = pose.relativeSpin(part, rates);
            if (w.lengthSquared() > 0) { h = h.add(apply(m, w)); }
        }
        for (int k = 0; k < v.tanks().size(); k++) {
            double kg = st.tankKg[k];
            int part = v.tanks().get(k).part();
            if (!(kg > 0) || v.chain(part).length == 0) { continue; }
            Vec3d at = pose.tankCentre(k), u = pose.relativeVelocity(part, at, rates);
            p = p.add(u.scale(kg)); h = h.add(at.subtract(c).cross(u).scale(kg));
        }
        for (int k = 0; k < v.holds().size(); k++) {
            double kg = st.holdKg[k];
            int part = v.holds().get(k).part();
            if (!(kg > 0) || v.chain(part).length == 0) { continue; }
            Vec3d at = pose.holdCentre(k), u = pose.relativeVelocity(part, at, rates);
            p = p.add(u.scale(kg)); h = h.add(at.subtract(c).cross(u).scale(kg));
        }
        return new Relative(p, h);
    }

    /** Mass of what joint j moves: its parts and the fuel and cargo in them, with its centre (vehicle frame). */
    static MassProperties subtree(Vehicle v, VehiclePose pose, VehicleState st, int joint) {
        List<MassProperties> masses = new java.util.ArrayList<>();
        for (Vehicle.Installed i : v.installed()) { if (v.moves(joint, i.index())) { masses.add(pose.partMass(i.index())); } }
        for (int k = 0; k < v.tanks().size(); k++) {
            if (st.tankKg[k] > 0 && v.moves(joint, v.tanks().get(k).part())) { masses.add(MassProperties.point(st.tankKg[k], pose.tankCentre(k))); }
        }
        for (int k = 0; k < v.holds().size(); k++) {
            if (st.holdKg[k] > 0 && v.moves(joint, v.holds().get(k).part())) { masses.add(MassProperties.point(st.holdKg[k], pose.holdCentre(k))); }
        }
        return MassProperties.combine(masses);
    }

    /** I v for a mass's inertia about its centre (which may be singular: a point mass has none). */
    static Vec3d apply(MassProperties m, Vec3d v) {
        return new Vec3d(m.xx() * v.x() + m.xy() * v.y() + m.xz() * v.z(), m.xy() * v.x() + m.yy() * v.y() + m.yz() * v.z(),
                m.xz() * v.x() + m.yz() * v.y() + m.zz() * v.z());
    }

    /** The subtree's inertia about joint j (kg m^2 about the axis through the origin, or its mass for a slide). */
    static double inertia(Vehicle v, VehiclePose pose, MassProperties sub, int joint) {
        if (v.joints().get(joint).joint().kind() == Part.Joint.Kind.PRISMATIC) { return Math.max(sub.massKg(), MIN_INERTIA); }
        Vec3d a = pose.jointAxis(joint), r = sub.centre().subtract(pose.jointOrigin(joint));
        Vec3d perpendicular = r.subtract(a.scale(r.dot(a)));
        double about = a.dot(apply(sub, a)) + sub.massKg() * perpendicular.lengthSquared();
        return Math.max(about, MIN_INERTIA);
    }

    /** A world wrench (force, and torque about the centre of mass) as a generalized force on joint j: about its axis, or along it. */
    static double generalized(Vehicle v, VehiclePose pose, int joint, Wrench w, dev.mcmod.aerocore.api.Orientation q) {
        Vec3d axis = q.rotate(pose.jointAxis(joint));
        if (v.joints().get(joint).joint().kind() == Part.Joint.Kind.PRISMATIC) { return axis.dot(w.force()); }
        Vec3d origin = q.rotate(pose.jointOrigin(joint).subtract(pose.properties().centerOfMassLocal()));
        return axis.dot(w.torque().subtract(origin.cross(w.force())));
    }

    /** A force at a world offset from the centre of mass as a generalized force on joint j. */
    static double generalized(Vehicle v, VehiclePose pose, int joint, AppliedForce f, dev.mcmod.aerocore.api.Orientation q) {
        return generalized(v, pose, joint, f.wrench(), q);
    }

    /**
     * The joints' end values and rates for the step and the actuators' efforts, given the start, the commands (null:
     * hold), each joint's generalized load from outside (weight included) and the subtree inertias.
     */
    static void resolve(Vehicle v, double[] q0, double[] rate0, Command[] commands, double[] load, double[] inertia, double dt,
                        double[] q1, double[] rate1, double[] effort) {
        resolve(v, q0, rate0, commands, load, new double[load.length], inertia, dt, q1, rate1, effort);
    }

    /**
     * As above with each joint's load split: load drives (weight, the parts' own loads this step), resist only resists
     * (the contacts' and dampers' lagged load): the actuator must overcome both to keep its rate, but a joint that gives
     * way is accelerated by load and the actuator alone, resist slowing it at most to a stop.
     */
    static void resolve(Vehicle v, double[] q0, double[] rate0, Command[] commands, double[] load, double[] resist, double[] inertia, double dt,
                        double[] q1, double[] rate1, double[] effort) {
        for (Vehicle.Joint j : v.joints()) {
            int k = j.index();
            Command c = commands[k];
            double target = c == null ? 0 : c.rate();
            double up = c == null ? Double.POSITIVE_INFINITY : c.up(), down = c == null ? Double.POSITIVE_INFINITY : c.down();
            double needed = inertia[k] * (target - rate0[k]) / dt - load[k] - resist[k];
            double rate;
            if (needed <= up && needed >= -down) {
                rate = target;
                effort[k] = needed;
            } else {
                double given = needed > up ? up : -down;
                effort[k] = given;
                rate = rate0[k] + dt * (load[k] + given) / inertia[k];
                // the contacts only oppose the motion: to a stop at most
                double opposed = rate + dt * resist[k] / inertia[k];
                if (Math.signum(resist[k]) != Math.signum(rate) && rate != 0) { rate = Math.signum(opposed) == Math.signum(rate) ? opposed : 0; }
            }
            double end = q0[k] + rate * dt;
            Part.Joint joint = j.joint();
            if (end > joint.maximum()) { end = joint.maximum(); rate = (end - q0[k]) / dt; }
            else if (end < joint.minimum()) { end = joint.minimum(); rate = (end - q0[k]) / dt; }
            q1[k] = end;
            rate1[k] = rate;
        }
    }
}
