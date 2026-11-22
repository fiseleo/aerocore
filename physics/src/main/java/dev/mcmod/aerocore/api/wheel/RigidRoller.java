package dev.mcmod.aerocore.api.wheel;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
import dev.mcmod.aerocore.api.vehicle.PartStep;

/**
 * 0.43: a rigid roller pressed on the ground: a crawler's road wheel, idler or sprocket with its track shoe under it, or
 * a road roller's steel drum (one slice of its width). Shared by TrackPart and DrumPart.
 *
 * <p>Contact: a fan of rays from the hub in the roller's plane (its own angles: straight down for a road wheel, round to
 * the front for an idler that meets a kerb first); the ray that reaches deepest into the circle sets the penetration p
 * (radius less the distance to the ground) and the line from that contact point to the hub the direction the ground
 * pushes (so a step's edge pushes back as well as up, as WheelCorner's edge does). The push is a spring k p and a
 * damper c on the closing speed (taken implicitly, PartStep.damper): a steel undercarriage has no suspension, so k is
 * the shoe's and the ground's own stiffness, chosen for a few centimetres under the static load.
 *
 * <p>Friction along the ground (lateral here; a crawler's drive is TrackPart's, a drum's DrumPart's) uses a shear
 * deformation state w (m) like the tyre's transient slip (Pacejka 2012, ch. 7): dw/dt = -v_slip - |v_roll| w / sigma,
 * force mu N (1 - exp(-|w| / K)) sign(w) with K the shear deformation modulus (Janosi & Hanamoto 1961: 0.025 m for
 * soils; 5 mm on hard ground, a rule), slope taken implicitly. Standing still the state holds the force like a spring,
 * so a crawler parks on a slope its friction can hold; the state is bounded at 6 K.
 */
public final class RigidRoller {
    private RigidRoller() {}

    /** Shear deformation modulus on hard ground (m; a rule: steel shoes and drums on rock, asphalt, concrete). */
    public static final double HARD_SHEAR_M = 0.005;
    /** The state's bound, in shear moduli. */
    public static final double MAX_SHEAR = 6;
    /** Below this rolling speed (m/s) the lateral relaxation length's term fades (it would divide by nothing). */
    public static final double CREEP_MPS = 0.05;

    /** A roller: its hub in the part frame, its radius to the ground side (wheel plus shoe), its rays' angles (rad, from straight down toward +x). */
    public record Station(Vec3d hubPart, double radiusM, double[] raysRad) {}

    /** What a roller found: the hit, how deep the ground reaches (m), the contact point (world) and the unit direction the ground pushes (world). */
    public record Contact(Ground.Hit hit, double penetrationM, Vec3d point, Vec3d push) {}

    /** The deepest ray's contact, or null when no ray reaches into the circle. */
    public static Contact find(PartStep step, Station station) {
        Ground ground = step.environment().ground();
        Vec3d hub = step.point(station.hubPart());
        Contact best = null;
        for (double angle : station.raysRad()) {
            Vec3d direction = step.axis(new Vec3d(Math.sin(angle), -Math.cos(angle), 0));
            Ground.Hit hit = ground.cast(hub, direction, station.radiusM() + 0.05);
            if (hit == null) { continue; }
            double p = station.radiusM() - hit.distanceM();
            if (!(p > 0) || (best != null && p <= best.penetrationM())) { continue; }
            Vec3d toHub = hub.subtract(hit.point());
            Vec3d push = toHub.lengthSquared() > 1e-12 ? toHub.normalized() : hit.normal();
            if (push.dot(hit.normal()) < 0.2) { push = hit.normal(); }   // a ray started inside the ground: its normal
            best = new Contact(hit, p, hit.point(), push);
        }
        return best;
    }

    /**
     * Presses the roller: k p along the push and a damper c along it (implicit, on the closing speed against the
     * ground's own velocity). Returns the normal load to use for friction (N, not negative): the spring and the
     * damper at the step's start speed.
     */
    public static double press(PartStep step, Contact c, double stiffnessNPerM, double dampingNsPerM) {
        RigidBodyState s = step.state();
        Vec3d offset = c.point().subtract(s.position());
        Vec3d velocity = s.velocity().add(step.angularVelocity().cross(offset)).subtract(c.hit().velocity());
        double closing = -velocity.dot(c.push());
        double spring = stiffnessNPerM * c.penetrationM();
        step.force(new AppliedForce(AppliedForce.Kind.CONTACT, offset, c.push().scale(spring)));
        step.damper(offset, c.push(), dampingNsPerM, c.push().dot(c.hit().velocity()));
        return Math.max(spring + dampingNsPerM * closing, 0);
    }

    /** The shear deformation modulus K (m) of a surface: its soil's, or HARD_SHEAR_M on hard ground. */
    public static double shearModulus(GroundContact.Surface surface) {
        return surface.soil() == null ? HARD_SHEAR_M : surface.soil().shearModulusM();
    }

    /** One friction direction's step: the force (N, along the direction), its slope against the slip speed (N s/m) and the state predicted at the step's end. */
    public record Shear(double forceN, double slopeNsPerM, double predicted) {}

    /**
     * Friction along a direction with its shear state w (m): capacity (N, the most it can carry), slip speed (m/s,
     * the ground-relative velocity of the contact along the direction: the force opposes it), rolling speed (m/s, along
     * the roller's heading: relaxes the state over sigma), relaxation length sigma (m), K and the step.
     */
    public static Shear shear(double capacityN, double slipMps, double rollingMps, double sigmaM, double shearM, double w, double dt) {
        double relax = 1 + dt * Math.max(Math.abs(rollingMps), CREEP_MPS) / Math.max(sigmaM, 1e-3);
        double next = (w - dt * slipMps) / relax;
        double bound = MAX_SHEAR * shearM;
        next = Math.clamp(next, -bound, bound);
        if (!(capacityN > 0)) { return new Shear(0, 0, next); }
        double e = Math.exp(-Math.abs(next) / shearM);
        double force = capacityN * (1 - e) * Math.signum(next);
        double slope = Math.abs(next) >= bound ? 0 : capacityN / shearM * e * dt / relax;
        return new Shear(force, slope, next);
    }

    /** The state after the step from the end slip and rolling speeds (as shear predicted it). */
    public static double relaxed(double slipMps, double rollingMps, double sigmaM, double shearM, double w, double dt) {
        double relax = 1 + dt * Math.max(Math.abs(rollingMps), CREEP_MPS) / Math.max(sigmaM, 1e-3);
        return Math.clamp((w - dt * slipMps) / relax, -MAX_SHEAR * shearM, MAX_SHEAR * shearM);
    }

    /**
     * Applies friction at a world contact point along a unit world direction: the force and an implicit damper with the
     * shear's slope, referenced (as WheelCorner's) to the point's velocity carried round with the body for the step.
     */
    public static void rub(PartStep step, Vec3d point, Vec3d direction, Shear shear) {
        RigidBodyState s = step.state();
        Vec3d offset = point.subtract(s.position()), omega = step.angularVelocity();
        Vec3d velocity = s.velocity().add(omega.cross(offset));
        Vec3d carried = velocity.add(omega.cross(velocity).scale(step.dt()));
        if (shear.forceN() != 0) { step.force(new AppliedForce(AppliedForce.Kind.CONTACT, offset, direction.scale(shear.forceN()))); }
        if (shear.slopeNsPerM() > 0) { step.damper(offset, direction, shear.slopeNsPerM(), carried.dot(direction)); }
    }

    /** The ground-relative velocity of a world point at the step's end (PartAfter), less the ground's velocity. */
    public static Vec3d endVelocity(PartAfter a, Vec3d partPoint, Vec3d groundVelocity) {
        return a.velocity(partPoint).subtract(groundVelocity);
    }
}
