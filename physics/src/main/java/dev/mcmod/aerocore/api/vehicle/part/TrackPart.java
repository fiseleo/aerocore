package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.damage.Wear;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.wheel.RigidRoller;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * 0.43: a crawler undercarriage's two tracks (left at -z, right at +z of the part frame, x forward, y up, origin between
 * the tracks at the road wheels' hubs), each driven by its sprocket: an excavator's, a bulldozer's, a crawler crane's.
 *
 * <ul>
 * <li>Rotors: "left" and "right", the sprockets (about -z: forward is positive); a shaft or a hydraulic motor drives each.
 *     The belt moves at sprocket speed x sprocket radius.</li>
 * <li>Ground: each side's road wheels, front idler and rear sprocket are rigid rollers under the shoes (RigidRoller): a
 *     spring and damper of the shoe and ground, no suspension (a steel undercarriage's), rays round the idler's front so
 *     a kerb pushes back and up. The track's weight on the ground is the sum of what they carry.</li>
 * <li>Drive (along the track): Wong, Theory of Ground Vehicles (4th ed., 2008) ch. 2: the shear displacement under a
 *     track grows from front to rear as slip x distance, so the tractive effort is (A c + W tan phi) times a factor of slip
 *     that reaches 1 when the shear displacement passes K; here as a shear state u that relaxes over half the contact
 *     length (RigidRoller.shear), which gives the same slip curve rolling steadily and holds standing still. On hard
 *     ground the capacity is the surface's peak friction (its Magic Formula D) x 0.6 for steel shoes (grousers scratch
 *     rock and asphalt; a rule) or 0.9 for rubber ones.</li>
 * <li>Sideways: every roller's contact has its own shear state with the lateral resistance coefficient mu_t (Wong ch. 7:
 *     0.5 to 0.8 on soils), so a skid-steered crawler turning about its centre meets the resisting moment of the shoes
 *     sliding sideways front and rear: mu_t W L / 4 with the weight spread evenly (Wong eq. 7.7).</li>
 * <li>Resistance: internal (the track's links, rollers and sprockets: a coefficient of the weight, Wong: 0.03 to 0.06 for
 *     steel tracks at low speed) and on soft ground the compaction of a Bekker plate as wide as the shoe and as long as
 *     the contact: z0 = (p / (kc/b + kphi))^(1/n), R_c = b (kc/b + kphi) z0^(n+1) / (n+1) (Bekker 1956); both hold the
 *     sprocket as one friction with the brakes.</li>
 * <li>Inputs: "brake" (0..1, both), "brake_left", "brake_right" (a steering brake), "park" (0..1).</li>
 * </ul>
 * The tracks' mass rides with the body. Shown values: each side's sprocket angle and the road wheels' mean compression
 * (left first). 0.43 wear: with a Wearing, one condition TRACKS (links, pins and rollers by running distance): the
 * capacity falls to half as it wears out, to a tenth when it has (a thrown track).
 */
public final class TrackPart implements Part {
    /** What the shoes are. */
    public enum Shoe { STEEL, RUBBER }

    /**
     * One track's geometry and its contact (both sides alike): gauge (m between the tracks' centrelines), the contact
     * length between the first and last road wheel (m), shoe width (m), road wheels (at least 2), their radius and the
     * shoe's thickness under them (m), the idler's and sprocket's radii (m), how far their runs clear the ground (m), the
     * shoe kind, each roller's stiffness (N/m) and damping (N s/m), the lateral resistance coefficient, the internal
     * resistance coefficient, each side's inertia at the sprocket (kg m^2: sprocket and belt), the brakes' and the parking
     * brake's torque at each sprocket (N m).
     */
    public record Spec(double gaugeM, double contactLengthM, double shoeWidthM, int roadWheels, double roadWheelRadiusM, double shoeThicknessM,
                       double idlerRadiusM, double sprocketRadiusM, double runLiftM, Shoe shoe, double stationStiffnessNPerM, double stationDampingNsPerM,
                       double lateralResistance, double internalResistance, double sideInertiaKgM2, double brakeNm, double parkNm) {
        public Spec {
            Objects.requireNonNull(shoe, "shoe");
            if (!(gaugeM > 0) || !(contactLengthM > 0) || !(shoeWidthM > 0) || roadWheels < 2 || !(roadWheelRadiusM > 0) || !(shoeThicknessM >= 0)
                    || !(idlerRadiusM > 0) || !(sprocketRadiusM > 0) || !(runLiftM >= 0) || !(stationStiffnessNPerM > 0) || !(stationDampingNsPerM >= 0)
                    || !(lateralResistance >= 0) || !(internalResistance >= 0) || !(sideInertiaKgM2 > 0) || !(brakeNm >= 0) || !(parkNm >= 0)) {
                throw new IllegalArgumentException("Track values out of range");
            }
        }
    }

    /** 0.43: how the tracks wear: their life in running time (s) at the rated belt speed (m/s), Wear.running with the speed to the first power. */
    public record Wearing(double lifeS, double ratedSpeedMps) {
        public Wearing { if (!(lifeS > 0) || !(ratedSpeedMps > 0)) { throw new IllegalArgumentException("Track life and speed must be positive"); } }
    }

    /** The condition's name (0.43). */
    public static final String TRACKS = "tracks";
    /** Hard-ground friction share of steel and rubber shoes (rules). */
    public static final double STEEL_GRIP = 0.6, RUBBER_GRIP = 0.9;

    private final PartDesign design;
    private final Spec spec;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final Wearing wearing;
    private final RigidRoller.Station[][] stations;

    public TrackPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues) { this(design, spec, mass, issues, null); }

    private TrackPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues, Wearing wearing) {
        this.design = Objects.requireNonNull(design, "design"); this.spec = Objects.requireNonNull(spec, "spec");
        this.mass = Objects.requireNonNull(mass, "mass"); this.issues = List.copyOf(issues); this.wearing = wearing;
        stations = new RigidRoller.Station[2][];
        for (int k = 0; k < 2; k++) { stations[k] = stations(spec, k == 0 ? -1 : 1); }
    }

    /** A copy of these tracks that wears (null: does not). */
    public TrackPart wearing(Wearing wearing) { return new TrackPart(design, spec, mass, issues, wearing); }
    public Spec spec() { return spec; }

    /** One side's rollers: the road wheels front to rear, then the idler, then the sprocket. */
    private static RigidRoller.Station[] stations(Spec s, int side) {
        int n = s.roadWheels();
        RigidRoller.Station[] out = new RigidRoller.Station[n + 2];
        double z = side * s.gaugeM() / 2, half = s.contactLengthM() / 2;
        double[] down = {-0.5, 0, 0.5};
        for (int i = 0; i < n; i++) {
            double x = half - i * s.contactLengthM() / (n - 1);
            out[i] = new RigidRoller.Station(new Vec3d(x, 0, z), s.roadWheelRadiusM() + s.shoeThicknessM(), down);
        }
        out[n] = new RigidRoller.Station(new Vec3d(half + s.idlerRadiusM(), s.idlerRadiusM() - s.roadWheelRadiusM() + s.runLiftM(), z),
                s.idlerRadiusM() + s.shoeThicknessM(), new double[]{0, 0.45, 0.9, 1.35});
        out[n + 1] = new RigidRoller.Station(new Vec3d(-half - s.sprocketRadiusM(), s.sprocketRadiusM() - s.roadWheelRadiusM() + s.runLiftM(), z),
                s.sprocketRadiusM() + s.shoeThicknessM(), new double[]{0, -0.45, -0.9, -1.35});
        return out;
    }

    private int perSide() { return 1 + spec.roadWheels() + 2; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<String> conditions() { return wearing == null ? List.of() : List.of(TRACKS); }
    @Override public List<Rotor> rotors() {
        return List.of(new Rotor("left", Vec3d.Z.scale(-1), spec.sideInertiaKgM2(), false), new Rotor("right", Vec3d.Z.scale(-1), spec.sideInertiaKgM2(), false));
    }
    @Override public List<String> inputs() { return List.of("brake", "brake_left", "brake_right", "park"); }
    /** Each side: the drive's shear state, then each roller's lateral one. */
    @Override public int internalStates() { return 2 * perSide(); }
    /** Each side's sprocket angle and the road wheels' mean compression. */
    @Override public int shownValues() { return 4; }

    /** What one side carried from loads to after: its rollers in contact (part-frame points, the lateral directions), the drive's point and direction. */
    private record Side(boolean[] touching, Vec3d[] pointPart, Vec3d[] lateral, Vec3d[] ground, Vec3d copPart, Vec3d heading, Vec3d copGround, double sigma, double shear) {}

    @Override public void loads(PartStep step) {
        double brake = Math.clamp(step.input("brake", 0), 0, 1), park = Math.clamp(step.input("park", 0), 0, 1);
        double worn = wearing == null ? 0 : step.used(0);
        Side[] sides = new Side[2];
        for (int k = 0; k < 2; k++) {
            double own = Math.clamp(step.input(k == 0 ? "brake_left" : "brake_right", 0), 0, 1);
            sides[k] = side(step, k, Math.max(brake, own), park, worn);
        }
        if (wearing != null) { step.diagnostic("tracksUsed", worn); }
        step.carry(sides);
    }

    private Side side(PartStep step, int k, double brake, double park, double worn) {
        RigidRoller.Station[] rollers = stations[k];
        int n = rollers.length, base = k * perSide();
        String key = k == 0 ? "left" : "right";
        RigidBodyState s = step.state();
        Vec3d heading = step.axis(Vec3d.X), omega = step.angularVelocity();
        boolean[] touching = new boolean[n];
        Vec3d[] pointPart = new Vec3d[n], lateral = new Vec3d[n], ground = new Vec3d[n];
        double[] load = new double[n];
        RigidRoller.Contact[] contacts = new RigidRoller.Contact[n];
        double total = 0, compression = 0;
        Vec3d cop = Vec3d.ZERO, normal = Vec3d.ZERO, groundVelocity = Vec3d.ZERO;
        GroundContact.Surface surface = null;
        double heaviest = 0;
        for (int i = 0; i < n; i++) {
            RigidRoller.Contact c = RigidRoller.find(step, rollers[i]);
            if (c == null) { continue; }
            contacts[i] = c;
            double w = RigidRoller.press(step, c, spec.stationStiffnessNPerM(), spec.stationDampingNsPerM());
            load[i] = w;
            touching[i] = true;
            pointPart[i] = step.partPoint(c.point());
            ground[i] = c.hit().velocity();
            if (i < spec.roadWheels()) { compression += c.penetrationM(); }
            if (w <= 0) { continue; }
            total += w;
            cop = cop.add(c.point().scale(w));
            normal = normal.add(c.hit().normal().scale(w));
            groundVelocity = groundVelocity.add(c.hit().velocity().scale(w));
            if (w > heaviest) { heaviest = w; surface = c.hit().surface(); }
        }
        step.show(2 * k + 1, PartStep.stepped(step.shown(2 * k + 1), compression / spec.roadWheels(), 1e-3));
        double sigma = spec.contactLengthM() / 2;
        int rotor = k;
        if (!(total > 0) || surface == null) {
            step.hold(rotor, brake * spec.brakeNm() + park * spec.parkNm());
            step.diagnostic(key + "LoadN", 0);
            return new Side(touching, pointPart, lateral, ground, null, heading, Vec3d.ZERO, sigma, RigidRoller.HARD_SHEAR_M);
        }
        cop = cop.scale(1 / total);
        normal = normal.normalized();
        groundVelocity = groundVelocity.scale(1 / total);
        double shearM = RigidRoller.shearModulus(surface);
        Vec3d flat = heading.subtract(normal.scale(normal.dot(heading)));
        Vec3d hg = flat.lengthSquared() > 1e-12 ? flat.normalized() : heading;
        Vec3d offset = cop.subtract(s.position());
        Vec3d relative = s.velocity().add(omega.cross(offset)).subtract(groundVelocity);
        double vx = relative.dot(hg), belt = step.rotorSpeed(rotor) * spec.sprocketRadiusM();
        // the drive: shear capacity of the contact
        GroundContact.Soil soil = surface.soil();
        double area = spec.shoeWidthM() * spec.contactLengthM();
        double capacity = soil != null ? area * soil.cohesionKpa() * 1000 + total * Math.tan(soil.frictionAngle())
                : total * surface.grip().d() * (spec.shoe() == Shoe.STEEL ? STEEL_GRIP : RUBBER_GRIP);
        if (worn > 0) { capacity *= worn >= 1 ? 0.1 : 1 - 0.5 * worn; }
        RigidRoller.Shear drive = RigidRoller.shear(capacity, vx - belt, vx, sigma, shearM, step.internal(base), step.dt());
        Vec3d carried = s.velocity().add(omega.cross(offset));
        carried = carried.add(omega.cross(carried).scale(step.dt()));
        step.rollingForce(rotor, offset, hg, drive.forceN(), drive.slopeNsPerM() * spec.sprocketRadiusM(), -spec.sprocketRadiusM());
        if (drive.slopeNsPerM() > 0) { step.damper(offset, hg, drive.slopeNsPerM(), carried.dot(hg)); }
        // sideways: each roller's own shear (the skid-steer resisting moment comes from their spread along the track)
        for (int i = 0; i < n; i++) {
            if (!touching[i] || !(load[i] > 0)) { continue; }
            RigidRoller.Contact c = contacts[i];
            Vec3d ni = c.hit().normal(), fi = heading.subtract(ni.scale(ni.dot(heading)));
            Vec3d hi = fi.lengthSquared() > 1e-12 ? fi.normalized() : hg;
            Vec3d li = ni.cross(hi);
            lateral[i] = li;
            Vec3d oi = c.point().subtract(s.position());
            Vec3d vi = s.velocity().add(omega.cross(oi)).subtract(c.hit().velocity());
            GroundContact.Surface si = c.hit().surface();
            double mu = spec.lateralResistance() * (si.soil() != null ? 1 : si.grip().d());
            if (worn >= 1) { mu *= 0.5; }
            RigidRoller.Shear side = RigidRoller.shear(mu * load[i], vi.dot(li), vi.dot(hi), sigma, RigidRoller.shearModulus(si), step.internal(base + 1 + i), step.dt());
            RigidRoller.rub(step, c.point(), li, side);
        }
        // resistance: internal and soft ground's compaction, held at the sprocket with the brakes
        double resist = spec.internalResistance() * total, sinkage = 0;
        if (soil != null) {
            double b = spec.shoeWidthM(), kz = 1000 * (soil.kc() / b + soil.kphi()), p = total / area;
            sinkage = Math.pow(p / kz, 1 / soil.n());
            resist += b * kz * Math.pow(sinkage, soil.n() + 1) / (soil.n() + 1);
        }
        step.hold(rotor, brake * spec.brakeNm() + park * spec.parkNm() + resist * spec.sprocketRadiusM());
        step.diagnostic(key + "LoadN", total);
        step.diagnostic(key + "TractionN", drive.forceN());
        step.diagnostic(key + "CapacityN", capacity);
        step.diagnostic(key + "Slip", (belt - vx) / Math.max(Math.max(Math.abs(belt), Math.abs(vx)), 0.1));
        step.diagnostic(key + "SinkageM", sinkage);
        step.diagnostic(key + "ResistanceN", resist);
        return new Side(touching, pointPart, lateral, ground, step.partPoint(cop), hg, groundVelocity, sigma, shearM);
    }

    @Override public void after(PartAfter a) {
        if (!(a.carried() instanceof Side[] sides)) { return; }
        double dt = a.dt();
        for (int k = 0; k < 2; k++) {
            Side c = sides[k];
            int base = k * perSide();
            double spin = a.rotorSpeed(k), belt = spin * spec.sprocketRadiusM();
            if (c.copPart() == null) {
                for (int i = 0; i < perSide(); i++) { a.setInternal(base + i, 0); }
            } else {
                double vx = a.velocity(c.copPart()).subtract(c.copGround()).dot(c.heading());
                a.setInternal(base, RigidRoller.relaxed(vx - belt, vx, c.sigma(), c.shear(), a.internal(base), dt));
                for (int i = 0; i < c.touching().length; i++) {
                    if (!c.touching()[i] || c.lateral()[i] == null) { a.setInternal(base + 1 + i, 0); continue; }
                    Vec3d v = a.velocity(c.pointPart()[i]).subtract(c.ground()[i]);
                    a.setInternal(base + 1 + i, RigidRoller.relaxed(v.dot(c.lateral()[i]), v.dot(c.heading()), c.sigma(), c.shear(), a.internal(base + 1 + i), dt));
                }
            }
            if (Math.abs(spin) >= dev.mcmod.aerocore.api.wheel.WheelCorner.SHOWN_CREEP_RAD_S) {
                a.show(2 * k, Math.IEEEremainder(a.shown(2 * k) + spin * dt, 2 * Math.PI));
            }
            if (wearing != null && belt != 0) { a.wear(0, Wear.running(dt, wearing.lifeS(), Math.abs(belt) / wearing.ratedSpeedMps(), 1, 1, 0)); }
        }
    }

    /** The rollers of side k (0 left, 1 right), road wheels front to rear then idler then sprocket (for drawing and tests). */
    public List<RigidRoller.Station> rollers(int side) { return new ArrayList<>(List.of(stations[side])); }
}
