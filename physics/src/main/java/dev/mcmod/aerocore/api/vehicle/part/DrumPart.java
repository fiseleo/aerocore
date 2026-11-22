package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.wheel.RigidRoller;
import dev.mcmod.aerocore.api.wheel.WheelCorner;

import java.util.List;
import java.util.Objects;

/**
 * 0.43: a road roller's steel drum across the part's z (centre at the part origin, axis along z): smooth or padfoot,
 * driven or not, with a vibrator.
 *
 * <ul>
 * <li>Rotor "drum" (about -z: rolling forward is positive) that a shaft or a hydraulic motor may drive.</li>
 * <li>Ground: the drum's width in slices, each a rigid roller (RigidRoller: spring and damper of steel on the ground,
 *     rays round the front and back so it rides up a kerb).</li>
 * <li>Drive and braking along the ground: one shear state for the drum (RigidRoller.shear) with the capacity on hard
 *     ground the surface's peak friction x 0.5 (steel on asphalt, a rule), on soft ground the rigid wheel's tractive
 *     limit (GroundContact.rigidWheel: A c + W tan phi over its contact); padfoot shells grip soil 1.3 times better and
 *     roll 1.5 times harder (rules).</li>
 * <li>Sideways: each slice's shear state with the same friction.</li>
 * <li>Rolling resistance: on soft ground the rigid wheel's compaction resistance (Bekker, exact integral), on hard ground
 *     0.02 of the load (steel on asphalt; Caterpillar, a rule); held at the rotor with the brake.</li>
 * <li>Vibration on "vibration" (0..1): the eccentric's centrifugal force m e (2 pi f)^2 is reported ("dynamicForceN", N)
 *     for the game's compaction; it is not stepped (at 25 to 50 Hz the 200 Hz substeps could not carry it), so the
 *     drum's motion is the static one.</li>
 * <li>Inputs: "brake" and "park" (0..1), "vibration" (0..1).</li>
 * </ul>
 * Shown values: the drum's angle, its mean compression and the vibration (0..1).
 */
public final class DrumPart implements Part {
    /** The drum's shell. */
    public enum Shell { SMOOTH, PADFOOT }

    /**
     * Width (m), radius to the shell's outside (m), slices across the width (at least 1), shell, each slice's stiffness
     * (N/m) and damping (N s/m), the drum's inertia (kg m^2), the eccentric's moment m e (kg m) and frequency (Hz), brake
     * and parking torques (N m).
     */
    public record Spec(double widthM, double radiusM, int slices, Shell shell, double sliceStiffnessNPerM, double sliceDampingNsPerM, double inertiaKgM2,
                       double eccentricKgM, double frequencyHz, double brakeNm, double parkNm) {
        public Spec {
            Objects.requireNonNull(shell, "shell");
            if (!(widthM > 0) || !(radiusM > 0) || slices < 1 || !(sliceStiffnessNPerM > 0) || !(sliceDampingNsPerM >= 0) || !(inertiaKgM2 > 0)
                    || !(eccentricKgM >= 0) || !(frequencyHz >= 0) || !(brakeNm >= 0) || !(parkNm >= 0)) {
                throw new IllegalArgumentException("Drum values out of range");
            }
        }
        /** The eccentric's centrifugal force at full vibration, N. */
        public double dynamicForceN() { double w = 2 * Math.PI * frequencyHz; return eccentricKgM * w * w; }
    }

    /** Hard-ground friction share of a steel drum, hard-ground rolling resistance, padfoot factors (rules). */
    public static final double STEEL_GRIP = 0.5, HARD_ROLLING = 0.02, PADFOOT_GRIP = 1.3, PADFOOT_ROLLING = 1.5;

    private final PartDesign design;
    private final Spec spec;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final RigidRoller.Station[] slices;

    public DrumPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design"); this.spec = Objects.requireNonNull(spec, "spec");
        this.mass = Objects.requireNonNull(mass, "mass"); this.issues = List.copyOf(issues);
        slices = new RigidRoller.Station[spec.slices()];
        double[] rays = {-0.9, -0.45, 0, 0.45, 0.9};
        for (int i = 0; i < slices.length; i++) {
            double z = -spec.widthM() / 2 + (i + 0.5) * spec.widthM() / slices.length;
            slices[i] = new RigidRoller.Station(new Vec3d(0, 0, z), spec.radiusM(), rays);
        }
    }

    public Spec spec() { return spec; }
    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor("drum", Vec3d.Z.scale(-1), spec.inertiaKgM2(), false)); }
    @Override public List<String> inputs() { return List.of("brake", "park", "vibration"); }
    /** The drive's shear state, then each slice's lateral one. */
    @Override public int internalStates() { return 1 + slices.length; }
    /** The drum's angle, its mean compression and the vibration. */
    @Override public int shownValues() { return 3; }

    private record Carry(boolean[] touching, Vec3d[] pointPart, Vec3d[] lateral, Vec3d[] ground, Vec3d copPart, Vec3d heading, Vec3d copGround, double shear) {}

    @Override public void loads(PartStep step) {
        int n = slices.length;
        RigidBodyState s = step.state();
        Vec3d heading = step.axis(Vec3d.X), omega = step.angularVelocity();
        boolean[] touching = new boolean[n];
        Vec3d[] pointPart = new Vec3d[n], lateral = new Vec3d[n], ground = new Vec3d[n];
        RigidRoller.Contact[] contacts = new RigidRoller.Contact[n];
        double[] load = new double[n];
        double total = 0, compression = 0, heaviest = 0;
        Vec3d cop = Vec3d.ZERO, normal = Vec3d.ZERO, groundVelocity = Vec3d.ZERO;
        GroundContact.Surface surface = null;
        for (int i = 0; i < n; i++) {
            RigidRoller.Contact c = RigidRoller.find(step, slices[i]);
            if (c == null) { continue; }
            contacts[i] = c;
            double w = RigidRoller.press(step, c, spec.sliceStiffnessNPerM(), spec.sliceDampingNsPerM());
            load[i] = w; touching[i] = true;
            pointPart[i] = step.partPoint(c.point()); ground[i] = c.hit().velocity();
            compression += c.penetrationM();
            if (w <= 0) { continue; }
            total += w;
            cop = cop.add(c.point().scale(w)); normal = normal.add(c.hit().normal().scale(w)); groundVelocity = groundVelocity.add(c.hit().velocity().scale(w));
            if (w > heaviest) { heaviest = w; surface = c.hit().surface(); }
        }
        double vibration = Math.clamp(step.input("vibration", 0), 0, 1);
        step.show(1, PartStep.stepped(step.shown(1), compression / n, 1e-3));
        step.show(2, vibration);
        step.diagnostic("dynamicForceN", vibration * spec.dynamicForceN());
        step.diagnostic("frequencyHz", vibration > 0 ? spec.frequencyHz() : 0);
        double brake = Math.clamp(step.input("brake", 0), 0, 1) * spec.brakeNm() + Math.clamp(step.input("park", 0), 0, 1) * spec.parkNm();
        if (!(total > 0) || surface == null) {
            step.hold(0, brake);
            step.diagnostic("loadN", 0);
            step.carry(new Carry(touching, pointPart, lateral, ground, null, heading, Vec3d.ZERO, RigidRoller.HARD_SHEAR_M));
            return;
        }
        cop = cop.scale(1 / total); normal = normal.normalized(); groundVelocity = groundVelocity.scale(1 / total);
        double shearM = RigidRoller.shearModulus(surface), sigma = Math.max(spec.radiusM(), 0.2);
        Vec3d flat = heading.subtract(normal.scale(normal.dot(heading)));
        Vec3d hg = flat.lengthSquared() > 1e-12 ? flat.normalized() : heading;
        Vec3d offset = cop.subtract(s.position());
        Vec3d relative = s.velocity().add(omega.cross(offset)).subtract(groundVelocity);
        double vx = relative.dot(hg), rim = step.rotorSpeed(0) * spec.radiusM();
        GroundContact.Soil soil = surface.soil();
        boolean pad = spec.shell() == Shell.PADFOOT;
        double capacity, resist;
        if (soil != null) {
            GroundContact.Sinkage sink = GroundContact.rigidWheel(soil, total, 2 * spec.radiusM(), spec.widthM());
            capacity = sink.maxTractionN() * (pad ? PADFOOT_GRIP : 1);
            resist = sink.compactionN() * (pad ? PADFOOT_ROLLING : 1);
            step.diagnostic("sinkageM", sink.sinkageM());
        } else {
            capacity = total * surface.grip().d() * STEEL_GRIP;
            resist = total * HARD_ROLLING * (pad ? PADFOOT_ROLLING : 1);
        }
        RigidRoller.Shear drive = RigidRoller.shear(capacity, vx - rim, vx, sigma, shearM, step.internal(0), step.dt());
        Vec3d carried = s.velocity().add(omega.cross(offset));
        carried = carried.add(omega.cross(carried).scale(step.dt()));
        step.rollingForce(0, offset, hg, drive.forceN(), drive.slopeNsPerM() * spec.radiusM(), -spec.radiusM());
        if (drive.slopeNsPerM() > 0) { step.damper(offset, hg, drive.slopeNsPerM(), carried.dot(hg)); }
        double mu = capacity / total;
        for (int i = 0; i < n; i++) {
            if (!touching[i] || !(load[i] > 0)) { continue; }
            RigidRoller.Contact c = contacts[i];
            Vec3d ni = c.hit().normal(), fi = heading.subtract(ni.scale(ni.dot(heading)));
            Vec3d hi = fi.lengthSquared() > 1e-12 ? fi.normalized() : hg, li = ni.cross(hi);
            lateral[i] = li;
            Vec3d oi = c.point().subtract(s.position());
            Vec3d vi = s.velocity().add(omega.cross(oi)).subtract(c.hit().velocity());
            RigidRoller.Shear side = RigidRoller.shear(mu * load[i], vi.dot(li), vi.dot(hi), sigma, RigidRoller.shearModulus(c.hit().surface()),
                    step.internal(1 + i), step.dt());
            RigidRoller.rub(step, c.point(), li, side);
        }
        step.hold(0, brake + resist * spec.radiusM());
        step.diagnostic("loadN", total);
        step.diagnostic("linearLoadNPerM", total / spec.widthM());
        step.diagnostic("tractionN", drive.forceN());
        step.diagnostic("resistanceN", resist);
        step.carry(new Carry(touching, pointPart, lateral, ground, step.partPoint(cop), hg, groundVelocity, shearM));
    }

    @Override public void after(PartAfter a) {
        if (!(a.carried() instanceof Carry c)) { return; }
        double dt = a.dt(), spin = a.rotorSpeed(0), sigma = Math.max(spec.radiusM(), 0.2);
        if (c.copPart() == null) {
            for (int i = 0; i < internalStates(); i++) { a.setInternal(i, 0); }
        } else {
            double vx = a.velocity(c.copPart()).subtract(c.copGround()).dot(c.heading());
            a.setInternal(0, RigidRoller.relaxed(vx - spin * spec.radiusM(), vx, sigma, c.shear(), a.internal(0), dt));
            for (int i = 0; i < c.touching().length; i++) {
                if (!c.touching()[i] || c.lateral()[i] == null) { a.setInternal(1 + i, 0); continue; }
                Vec3d v = a.velocity(c.pointPart()[i]).subtract(c.ground()[i]);
                a.setInternal(1 + i, RigidRoller.relaxed(v.dot(c.lateral()[i]), v.dot(c.heading()), sigma, c.shear(), a.internal(1 + i), dt));
            }
        }
        if (Math.abs(spin) >= WheelCorner.SHOWN_CREEP_RAD_S) { a.show(0, Math.IEEEremainder(a.shown(0) + spin * dt, 2 * Math.PI)); }
    }
}
