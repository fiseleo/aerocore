package dev.mcmod.aerocore.api.wheel;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.wheel.kinematics.KinematicTable;

import java.util.Objects;

/**
 * One wheel of a part (0.24): it finds the ground, carries its share of the vehicle on tyre and suspension in series,
 * and passes the tyre's forces to the body and its rotor. Axles (two) and landing-gear legs (one) are built of these.
 *
 * <p>The wheel moves along the part's vertical between full droop and full bump. A fan of RAYS rays across the lower
 * half of the wheel's circle (+-FAN about straight down, in the wheel's plane) cast straight down the body's vertical
 * from above full bump; the ray whose circle point the ground lifts highest sets the penetration P (how far the ground
 * pushes tyre and suspension together up from droop) and gives the contact point, normal and surface. A step the
 * forward rays find lifts the wheel before the centre does, so kerbs up to about the radius are climbed; ground that
 * lifts the wheel past full bump meets the bump stop. When the deciding ray is off-centre the ground pushes along the
 * line from that point to the hub, so the force's part along the heading (load times the tangent of the ray's angle)
 * resists the climb.
 *
 * <p>Per step: the suspension's series step (Suspension.Step) gives the force along its travel and its slope against
 * the ground's closing rate; the ground's normal force is that over the cosine between travel and normal (the links
 * take the rest), and it acts along the normal at the contact patch with a line damper for the slope (applied along the
 * body's vertical instead, a rolled body would get a sideways push from its own suspension). The tyre's transient
 * contact (TyreContact) gives the
 * in-plane forces, which act at the contact patch (the longitudinal one as a rolling force on the wheel's rotor, both
 * with line dampers for their slopes against the body), the lateral one jacking the body by F_y h_rc / (t / 2) on the
 * way (and loading the tyre with it); the brake and the rolling resistance hold the rotor as one friction row; the
 * aligning moment turns the body. after() moves on the suspension's compression, the tyre's deflections and the
 * brake's temperature, and shows the wheel's spin, steer and travel.
 *
 * <p>0.33: a suspension with a kinematic table moves the hub along the table's path (the rays follow it), adds its toe
 * change to the steer (bump steer) and its camber change, and jacks by its front-view slope for the lateral force and its
 * side-view slope for the longitudinal one (the contact patch's when braking or on a solid axle, the hub's when driving
 * an independent wheel). Without one, nothing changes.
 *
 * <p>0.33: on a twisting frame (FrameFlex) the owning part probes each wheel first (probe: the ground found and its
 * force and stiffness at the start twist), and loads() then moves the wheel's attachment by the twist (its lift at the
 * start, its rate over the step) and reuses the ground. Without a frame nothing changes.
 *
 * <p>Internal states (STATES, from the owning part's index): compression (m from droop), its rate (m/s), the tyre's
 * deflections u and v (m), the brake's temperature (K). Shown values (SHOWN): spin angle (rad), steer angle (rad, left
 * positive), compression (m). The wheel's own mass rides with the body (no unsprung mass in this version).
 */
public final class WheelCorner {
    /** Internal states per wheel. */
    public static final int STATES = 5;
    /** Shown values per wheel. */
    public static final int SHOWN = 3;
    /**
     * 0.30: shown values a part may add for a wheel (at an index of its own, after its others): the tyre's slip speed
     * over the ground (m/s, in 0.1 m/s steps: smoke, marks) and its load (kN, 0.1 kN steps: dust thrown), 0 off the ground.
     */
    public static final int EXTRA = 2;
    /**
     * 0.30: how finely the steering and compression are shown (rad, m; with the 3/4-step hysteresis of
     * PartStep.stepped), and the spin under which a wheel's shown angle stands still: a parked wheel, creeping by
     * microns as its tyre and strut settle, sends nothing.
     */
    public static final double SHOWN_STEER_RAD = 1e-3, SHOWN_COMPRESSION_M = 1e-3, SHOWN_CREEP_RAD_S = 0.05;
    /** Rays across the wheel's lower half, and their spread either side of straight down (rad). */
    public static final int RAYS = 9;
    /** A step in the ground under the fan taller than this is an edge to find, m. */
    public static final double EDGE = 0.005;
    public static final double FAN = Math.toRadians(60);
    /** How far above full bump the rays start, m. */
    public static final double MARGIN = 0.15;
    /** Air temperature for brake cooling, K (rule; the world's air is not passed in). */
    public static final double AIR_K = 293.15;

    /**
     * A wheel corner's design: the hub at full droop (part frame), the side (+1 starboard, -1 port, 0 on the
     * centreline), the axle's track (m; 0 for a single wheel: no jacking), tyre, the inertia of rim, hub and brake rotor
     * about the axle (kg m^2), suspension, brake (null: none) and toe (rad, positive toe-in).
     */
    public record Spec(Vec3d droopHub, double side, double trackM, Tyre tyre, double rimInertiaKgM2, Suspension suspension, Brake brake, double toeRad) {
        public Spec {
            Objects.requireNonNull(droopHub, "droopHub"); Objects.requireNonNull(tyre, "tyre"); Objects.requireNonNull(suspension, "suspension");
            if (side != 1 && side != -1 && side != 0) { throw new IllegalArgumentException("Side is +1, -1 or 0"); }
            if (!(trackM >= 0) || !(rimInertiaKgM2 >= 0)) { throw new IllegalArgumentException("Track and inertia must not be negative"); }
        }
    }

    /** What loads() found, for after() (0.33: and how fast a twisting frame lifted the wheel's attachment, m/s). */
    public record Carry(boolean contact, Vec3d hubPart, Vec3d up, Suspension.Step suspension, Vec3d contactPart, Vec3d heading, Vec3d lateral,
                        Vec3d groundVelocity, double loadN, double rollingRadiusM, int row, double brakeShare, double steerRad, double speedMps,
                        double flexRateMps) {}

    /** The ground under the wheel: the deciding ray's hit (null: none), how far it reaches into the travel (m) and its angle off straight down. */
    public record Found(Ground.Hit hit, double best, double angle) {}

    /** 0.33: a probe (on a twisting frame): the ground found, the force the wheel gives at the start twist (N) and its stiffness within the step (N/m). */
    public record Probe(Found found, double forceN, double stiffnessNPerM) {}

    private final Spec spec;

    public WheelCorner(Spec spec) { this.spec = Objects.requireNonNull(spec, "spec"); }

    public Spec spec() { return spec; }
    /** The wheel's rotor inertia: tyre plus rim, hub and brake rotor, kg m^2. */
    public double inertiaKgM2() { return spec.tyre().inertiaKgM2() + spec.rimInertiaKgM2(); }
    /** The hub at a compression, part frame (0.33: along the kinematic table's path, mirrored for a left wheel). */
    public Vec3d hub(double compressionM) {
        Vec3d h = spec.droopHub().add(Vec3d.Y.scale(compressionM));
        KinematicTable k = spec.suspension().kinematics();
        if (k == null) { return h; }
        Vec3d o = k.hubOffset(compressionM);
        return h.add(new Vec3d(o.x(), 0, spec.side() == 0 ? 0 : spec.side() * o.z()));
    }

    /** The toe change from the kinematic table at a compression, rad toe-in (0 without one). */
    private double bumpToe(double compressionM) {
        KinematicTable k = spec.suspension().kinematics();
        return k == null ? 0 : k.at(compressionM).toeRad();
    }

    /** Initial internal states: at the design compression, tyre relaxed, brake at air temperature. */
    public void initial(double[] states, int at) {
        states[at] = spec.suspension().designCompressionM();
        states[at + 4] = AIR_K;
    }

    /**
     * The wheel's loads for a step: its rotor and first internal state and shown value in the part, the steer angle
     * (rad, left positive), the driver's brake demand (0..1) and the rule between it and the brake (null: none) with the
     * first of its memory states, an extra holding torque on the rotor (N m, a handbrake), an extra force on the
     * suspension (N, an anti-roll bar's, positive pushing the wheel down) and the prefix of its diagnostics.
     */
    public Carry loads(PartStep step, int rotor, int state, int shown, double steerRad, double brakeDemand, BrakeControl control, int memory, double holdNm,
                       double extraN, String key) {
        return loads(step, rotor, state, shown, steerRad, brakeDemand, control, memory, holdNm, extraN, key, null);
    }

    /**
     * 0.33: a probe on a twisting frame: finds the ground and reports (PartStep.corner, at the hub) the force the wheel
     * would give over the step at the start twist and its stiffness against the attachment's movement (the series
     * step's slope over the step); 0 off the ground or topped out.
     */
    public Probe probe(PartStep step, int state, double steerRad, double extraN) { return probe(step, state, steerRad, extraN, 1); }

    /** 0.33: the probe with an air spring's air scaled (Suspension.Levelling). */
    public Probe probe(PartStep step, int state, double steerRad, double extraN, double airScale) {
        Suspension sus = spec.suspension();
        double dt = step.dt(), c = step.internal(state);
        Found f = find(step, c, steerRad);
        Vec3d hubPart = hub(c);
        double best = f.best() - step.flexLift(hubPart);
        if (f.hit() == null || best <= 0) {
            step.corner(hubPart, 0, 0);
            return new Probe(f, 0, 0);
        }
        Vec3d hubVelocity = step.state().velocity().add(step.angularVelocity().cross(step.offset(hubPart))).subtract(f.hit().velocity());
        double closing = -hubVelocity.dot(step.axis(Vec3d.Y));
        Suspension.Step ss = sus.step(spec.tyre(), best, closing, c, step.internal(state + 1), extraN, dt, airScale);
        double force = Math.max(ss.forceN() + ss.slopeNsPerM() * closing, 0), stiffness = ss.toppedOut() || force == 0 ? 0 : ss.slopeNsPerM() / dt;
        step.corner(hubPart, force, stiffness);
        return new Probe(f, force, stiffness);
    }

    /** The ground under the wheel at a compression and steer (the fan of rays and the edge between them). */
    private Found find(PartStep step, double c, double steerRad) {
        Suspension sus = spec.suspension();
        Tyre tyre = spec.tyre();
        KinematicTable kin = sus.kinematics();
        double yaw = kin == null ? steerRad + spec.side() * spec.toeRad() : steerRad + spec.side() * (spec.toeRad() + bumpToe(c));
        // the rays start over the hub's path at the step's compression (0.33; straight up without a table)
        Vec3d base = kin == null ? spec.droopHub() : hub(c).subtract(Vec3d.Y.scale(c));
        Vec3d headingPart = new Vec3d(Math.cos(yaw), 0, -Math.sin(yaw));
        Vec3d up = step.axis(Vec3d.Y), down = up.scale(-1);
        double r0 = tyre.radiusM(), top = sus.travelM() + MARGIN;
        Ground ground = step.environment().ground();
        double best = Double.NEGATIVE_INFINITY, angle = 0;
        Ground.Hit hit = null;
        Ground.Hit[] hits = new Ground.Hit[RAYS];
        int at = -1;
        for (int i = 0; i < RAYS; i++) {
            double theta = -FAN + 2 * FAN * i / (RAYS - 1);
            hits[i] = cast(step, ground, base, headingPart, r0 * Math.sin(theta), top, down, r0);
            if (hits[i] == null) { continue; }
            double lift = top - hits[i].distanceM() + r0 * Math.cos(theta);
            if (lift > best) { best = lift; hit = hits[i]; angle = theta; at = i; }
        }
        // an edge between the deciding ray and its neighbour toward the centre: find it, so the lift grows smoothly as the
        // wheel rolls up to a step instead of jumping each time a ray passes it
        int inner = at < 0 || at == (RAYS - 1) / 2 ? -1 : at < (RAYS - 1) / 2 ? at + 1 : at - 1;
        if (inner >= 0 && (hits[inner] == null || hits[inner].distanceM() > hit.distanceM() + EDGE)) {
            double high = r0 * Math.sin(angle), low = r0 * Math.sin(-FAN + 2 * FAN * inner / (RAYS - 1));
            Ground.Hit top0 = hit;
            for (int k = 0; k < 6; k++) {
                double mid = 0.5 * (high + low);
                Ground.Hit h = cast(step, ground, base, headingPart, mid, top, down, r0);
                if (h != null && h.distanceM() <= top0.distanceM() + EDGE) { high = mid; top0 = h; } else { low = mid; }
            }
            double lift = top - top0.distanceM() + Math.sqrt(Math.max(r0 * r0 - high * high, 0));
            if (lift > best) { best = lift; hit = top0; angle = Math.asin(Math.clamp(high / r0, -1, 1)); }
        }
        return new Found(hit, best, angle);
    }

    /** As loads above, reusing a probe's ground and following a twisting frame (0.33; probe null: finding the ground itself). */
    public Carry loads(PartStep step, int rotor, int state, int shown, double steerRad, double brakeDemand, BrakeControl control, int memory, double holdNm,
                       double extraN, String key, Probe probe) {
        return loads(step, rotor, state, shown, steerRad, brakeDemand, control, memory, holdNm, extraN, key, probe, 1);
    }

    /** As loads above with an air spring's air scaled (0.33, Suspension.Levelling; 1: as designed). */
    public Carry loads(PartStep step, int rotor, int state, int shown, double steerRad, double brakeDemand, BrakeControl control, int memory, double holdNm,
                       double extraN, String key, Probe probe, double airScale) {
        return loads(step, rotor, state, shown, steerRad, brakeDemand, control, memory, holdNm, extraN, key, probe, airScale, 0);
    }

    /**
     * As loads above with the wheel's attachment lifted by mountLiftM (0.36: a telescoping leg drawn up; negative lowers
     * it), the way a twisting frame lifts it: so much less of the ground reaches into the travel. 0: as above, bit for bit.
     */
    public Carry loads(PartStep step, int rotor, int state, int shown, double steerRad, double brakeDemand, BrakeControl control, int memory, double holdNm,
                       double extraN, String key, Probe probe, double airScale, double mountLiftM) {
        Suspension sus = spec.suspension();
        Tyre tyre = spec.tyre();
        double dt = step.dt(), c = step.internal(state), u = step.internal(state + 2), v = step.internal(state + 3), temperature = step.internal(state + 4);
        KinematicTable kin = sus.kinematics();
        double yaw = kin == null ? steerRad + spec.side() * spec.toeRad() : steerRad + spec.side() * (spec.toeRad() + bumpToe(c));
        Vec3d headingPart = new Vec3d(Math.cos(yaw), 0, -Math.sin(yaw));
        Vec3d up = step.axis(Vec3d.Y), heading = step.axis(headingPart);
        double r0 = tyre.radiusM();
        Found found = probe != null ? probe.found() : find(step, c, steerRad);
        Ground.Hit hit = found.hit();
        double best = found.best(), angle = found.angle();
        // 0.30: shown to a milliradian and a millimetre (with hysteresis): a wheel at rest sends nothing
        step.show(shown + 1, PartStep.stepped(step.shown(shown + 1), steerRad, SHOWN_STEER_RAD));
        step.show(shown + 2, PartStep.stepped(step.shown(shown + 2), c, SHOWN_COMPRESSION_M));
        Vec3d hubPart = hub(c), hubOffset = step.offset(hubPart);
        RigidBodyState s = step.state();
        Vec3d omega = step.angularVelocity();
        // 0.33: a twisting frame lifts the wheel's attachment (less of the travel taken) and moves it over the step
        double flexRate = 0;
        if (step.flexing()) { best -= step.flexLift(hubPart); flexRate = step.flexRate(hubPart); }
        if (mountLiftM != 0) { best -= mountLiftM; }
        if (hit == null || best <= 0) { return free(step, rotor, state, steerRad, brakeDemand, control, memory, holdNm, extraN, key, Math.min(best, 0), airScale); }
        Vec3d n = hit.normal(), ground0 = hit.velocity();
        Vec3d hubVelocity = s.velocity().add(omega.cross(hubOffset)).subtract(ground0);
        double closing = -hubVelocity.dot(up);
        if (flexRate != 0) { closing -= flexRate; }
        Suspension.Step ss = airScale == 1 ? sus.step(tyre, best, closing, c, step.internal(state + 1), extraN, dt)
                : sus.step(tyre, best, closing, c, step.internal(state + 1), extraN, dt, airScale);
        // the suspension balances the ground's push along its travel; the links take the rest, so the ground's normal
        // force is the suspension's over the cosine between travel and normal, and it is that the body gets
        double cosine = Math.max(up.dot(n), 0.5);
        double vertical = Math.max(ss.forceN() + ss.slopeNsPerM() * closing, 0);
        double load = vertical / cosine;
        // the tyre's frame on the ground
        Vec3d flat = heading.subtract(n.scale(n.dot(heading)));
        if (flat.lengthSquared() < 1e-12) { flat = up.cross(n).cross(n); }
        Vec3d hg = flat.normalized(), lateral = n.cross(hg);
        Vec3d contactOffset = hit.point().subtract(s.position());
        Vec3d pointVelocity = s.velocity().add(omega.cross(contactOffset)), relative = pointVelocity.subtract(ground0);
        double vx = relative.dot(hg), vy = relative.dot(lateral);
        double re = r0 - Math.max(ss.tyreDeflection(), 0) / 3, spin = step.rotorSpeed(rotor);
        // camber to the ground: against the body, plus the body's lean at this wheel (a solid axle stays square)
        double camber = 0;
        if (spec.side() != 0) {
            Vec3d out = step.axis(Vec3d.Z.scale(spec.side()));
            camber = sus.geometry().solid() ? sus.staticCamberRad() : sus.camberToBody(c) - Math.asin(Math.clamp(out.dot(n), -1, 1));
        }
        GroundContact.Surface surface = hit.surface();
        TyreContact.Step t = TyreContact.step(tyre, surface, load, vx, vy, spin * re, -spec.side() * camber, u, v, dt);
        double jack = 0;
        if (spec.trackM() > 0 && spec.side() != 0) {
            double outward = t.lateralN() * lateral.dot(step.axis(Vec3d.Z.scale(spec.side())));
            if (kin == null) {
                jack = -outward * sus.geometry().rollCentreHeightM() / (spec.trackM() / 2);
            } else {
                // 0.33: the forces that do no work along the wheel's path go through the links: the lateral one by the
                // contact patch's front-view slope, the longitudinal one by its side-view slope (the hub's for drive on an
                // independent wheel, whose torque the body reacts)
                KinematicTable.Sample k = kin.at(c);
                boolean reacted = sus.geometry().solid() || t.longitudinalN() * vx < 0;
                jack = -outward * k.frontJackTan() - t.longitudinalN() * (reacted ? k.sideTanContact() : k.sideTanHub());
            }
            double jacked = Math.max(load + jack, 0);
            jack = jacked - load;
            load = jacked;
            t = TyreContact.step(tyre, surface, load, vx, vy, spin * re, -spec.side() * camber, u, v, dt);
        }
        // normal: the suspension's force through the contact patch along the ground's normal, implicit in the closing rate
        step.force(new AppliedForce(AppliedForce.Kind.CONTACT, contactOffset, n.scale(ss.forceN() / cosine + jack)));
        step.damper(contactOffset, n, ss.slopeNsPerM() / cosine, n.dot(ground0));
        // off-centre contact: the ground pushes along the line to the hub
        if (angle != 0) { step.force(new AppliedForce(AppliedForce.Kind.CONTACT, contactOffset, hg.scale(-vertical * Math.tan(angle)))); }
        // in-plane tyre forces: longitudinal on the rotor (rolling force), lateral on the body, each with its slope; the
        // dampers' reference is the point's velocity carried round with the body for the step (turning steadily, a velocity
        // fixed to the body is no slip, though its world direction swings)
        Vec3d carried = pointVelocity.add(omega.cross(pointVelocity).scale(dt));
        step.rollingForce(rotor, contactOffset, hg, t.longitudinalN(), t.longitudinalPerMps() * re, -re);
        step.damper(contactOffset, hg, t.longitudinalPerMps(), carried.dot(hg));
        if (t.lateralN() != 0) { step.force(new AppliedForce(AppliedForce.Kind.CONTACT, contactOffset, lateral.scale(t.lateralN()))); }
        step.damper(contactOffset, lateral, t.lateralPerMps(), carried.dot(lateral));
        if (t.aligningNm() != 0) { step.load(new Wrench(Vec3d.ZERO, n.scale(t.aligningNm())), AppliedForce.Kind.CONTACT); }
        // brake, handbrake and rolling resistance hold the rotor as one friction
        double brake = spec.brake() == null ? 0 : spec.brake().torqueNm(controlled(step, control, memory, brakeDemand, t.slipX(), Math.abs(vx)), temperature);
        double hold = brake + Math.max(holdNm, 0);
        double resist = t.rollingResistanceN() * Math.max(r0 - Math.max(ss.tyreDeflection(), 0), 0.5 * r0);
        int row = step.hold(rotor, hold + resist);
        step.diagnostic(key + "LoadN", load);
        step.diagnostic(key + "SlipRatio", t.slipX());
        step.diagnostic(key + "SlipAngleDeg", Math.toDegrees(Math.atan(-t.slipY())));
        step.diagnostic(key + "Friction", t.peakFriction());
        step.diagnostic(key + "CamberDeg", Math.toDegrees(camber));
        step.diagnostic(key + "ForceN", t.longitudinalN());
        step.diagnostic(key + "SideForceN", t.lateralN());
        return new Carry(true, hubPart, up, ss, step.partPoint(hit.point()), hg, lateral, ground0, load, re, row, hold + resist > 0 ? hold / (hold + resist) : 0,
                steerRad, Math.abs(vx), flexRate);
    }

    /**
     * 0.30: the wheel off the ground for a step (also a retracted leg's, 0.30): the suspension extends, the brake and an
     * extra holding torque hold the rotor (a wheel spins down), no contact. Lift is how far the ground reaches into
     * the wheel's travel (negative or minus infinity when it does not).
     */
    public Carry free(PartStep step, int rotor, int state, int shown, double steerRad, double brakeDemand, BrakeControl control, int memory, double holdNm,
                      double extraN, String key) {
        step.show(shown + 1, PartStep.stepped(step.shown(shown + 1), steerRad, SHOWN_STEER_RAD));
        step.show(shown + 2, PartStep.stepped(step.shown(shown + 2), step.internal(state), SHOWN_COMPRESSION_M));
        return free(step, rotor, state, steerRad, brakeDemand, control, memory, holdNm, extraN, key, Double.NEGATIVE_INFINITY, 1);
    }

    private Carry free(PartStep step, int rotor, int state, double steerRad, double brakeDemand, BrakeControl control, int memory, double holdNm,
                       double extraN, String key, double lift, double airScale) {
        Suspension sus = spec.suspension();
        double dt = step.dt(), c = step.internal(state), temperature = step.internal(state + 4);
        double yaw = sus.kinematics() == null ? steerRad + spec.side() * spec.toeRad() : steerRad + spec.side() * (spec.toeRad() + bumpToe(c));
        Vec3d up = step.axis(Vec3d.Y), heading = step.axis(new Vec3d(Math.cos(yaw), 0, -Math.sin(yaw)));
        RigidBodyState s = step.state();
        double brake = spec.brake() == null ? 0 : spec.brake().torqueNm(controlled(step, control, memory, brakeDemand, 0, s.velocity().length()), temperature);
        double hold = brake + Math.max(holdNm, 0);
        Suspension.Step ss = airScale == 1 ? sus.step(spec.tyre(), lift, 0, c, extraN, dt) : sus.step(spec.tyre(), lift, 0, c, 0, extraN, dt, airScale);
        int row = step.hold(rotor, hold);
        step.diagnostic(key + "LoadN", 0);
        return new Carry(false, hub(c), up, ss, Vec3d.ZERO, heading, Vec3d.ZERO, Vec3d.ZERO, 0, spec.tyre().radiusM(), row, 1, steerRad, s.velocity().length(), 0);
    }

    /** The brake demand after the rule (its memory read and written as internal states). */
    private static double controlled(PartStep step, BrakeControl control, int memory, double demand, double slip, double speed) {
        if (control == null) { return demand; }
        double[] m = new double[control.memory()];
        for (int i = 0; i < m.length; i++) { m[i] = step.internal(memory + i); }
        double d = control.demand(demand, slip, speed, m, step.dt());
        for (int i = 0; i < m.length; i++) { step.setInternal(memory + i, m[i]); }
        return Double.isFinite(d) ? Math.clamp(d, 0, 1) : demand;
    }

    /** A ray straight down the body from above full bump at a distance ahead of the hub (m, in the wheel's plane), over a droop hub. */
    private Ground.Hit cast(PartStep step, Ground ground, Vec3d droopHub, Vec3d headingPart, double ahead, double top, Vec3d down, double r0) {
        return ground.cast(step.point(droopHub.add(Vec3d.Y.scale(top)).add(headingPart.scale(ahead))), down, top + r0);
    }

    /** After the step: compression, deflections, brake temperature and the wheel's spin angle. */
    public void after(PartAfter a, Carry k, int rotor, int state, int shown) { after(a, k, rotor, state, shown, -1); }

    /** As after, also setting the wheel's EXTRA shown values from an index (0.30; negative: none). */
    public void after(PartAfter a, Carry k, int rotor, int state, int shown, int extra) {
        double dt = a.dt(), spin = a.rotorSpeed(rotor), c = a.internal(state);
        double closing = k.contact() ? -a.velocity(k.hubPart()).subtract(k.groundVelocity()).dot(k.up()) : 0;
        if (k.flexRateMps() != 0) { closing -= k.flexRateMps(); }
        double next = Math.min(k.suspension().compression(closing, dt), spec.suspension().travelM() * 1.5);
        a.setInternal(state, next);
        a.setInternal(state + 1, (next - c) / dt);
        if (k.contact()) {
            Vec3d relative = a.velocity(k.contactPart()).subtract(k.groundVelocity());
            double[] d = TyreContact.deflections(spec.tyre(), k.loadN(), relative.dot(k.heading()), relative.dot(k.lateral()), spin * k.rollingRadiusM(),
                    a.internal(state + 2), a.internal(state + 3), dt);
            a.setInternal(state + 2, d[0]);
            a.setInternal(state + 3, d[1]);
        } else {
            a.setInternal(state + 2, 0);
            a.setInternal(state + 3, 0);
        }
        if (spec.brake() != null) {
            double heat = k.brakeShare() * Math.abs(a.rowTorque(k.row()) * spin);
            a.setInternal(state + 4, spec.brake().temperatureK(a.internal(state + 4), heat, a.state().velocity().length(), AIR_K, dt));
        }
        // 0.30: the angle stands still below a creep (a parked wheel sends nothing); the compression to a millimetre
        if (Math.abs(spin) >= SHOWN_CREEP_RAD_S) { a.show(shown, Math.IEEEremainder(a.shown(shown) + spin * dt, 2 * Math.PI)); }
        a.show(shown + 2, PartStep.stepped(a.shown(shown + 2), next, SHOWN_COMPRESSION_M));
        if (extra >= 0) {
            double slip = 0, load = 0;
            if (k.contact()) {
                Vec3d relative = a.velocity(k.contactPart()).subtract(k.groundVelocity());
                slip = Math.hypot(relative.dot(k.heading()) - spin * k.rollingRadiusM(), relative.dot(k.lateral()));
                load = k.loadN() / 1000;
            }
            a.show(extra, PartStep.stepped(a.shown(extra), slip, 0.1));
            a.show(extra + 1, PartStep.stepped(a.shown(extra + 1), load, 0.1));
        }
    }
}
