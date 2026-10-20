package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * How a follower vehicle hangs on a leader (0.36): a trailer's kingpin in a tractor's fifth wheel, or a drawbar's eye
 * on a ball. The two points (each in its own vehicle's model frame: the fifth wheel's hinge centre, the point on the
 * kingpin's axis at that height) are held together; ContactSolver solves the joint with the bodies' contacts.
 * <ul>
 *   <li>pitchAxis (leader frame) is the fifth wheel's hinge, yawAxis (follower frame) the kingpin's axis. The pitch
 *   angle is the kingpin's axis turned about the hinge from the leader's up (its +y square to the hinge), the yaw angle
 *   the follower's forward (+x) turned about the kingpin from the leader's, both square to the kingpin; the roll angle
 *   is how far the hinge leans out of square with the kingpin (asin of their dot product).</li>
 *   <li>pitchLimitRad and yawLimitRad (the jackknife angle) are one-sided stops at plus and minus the angle; PI is none.</li>
 *   <li>rollLimitRad 0 holds the hinge square to the kingpin (a fifth wheel carries the trailer's roll), through a spring
 *   of rollStiffnessNmPerRad and a damper of rollDampingNms (an infinite stiffness is rigid). Above 0 the roll is free
 *   within the limit (still through the spring and damper, which may be 0) with a stop there (an oscillating fifth
 *   wheel); from PI/2 it is free (a ball).</li>
 *   <li>captureM is how far apart the points may be for coupling to take; forwarded names the control channels the
 *   follower copies from the leader each tick (brakes, lights); collide lets the two bodies' spheres touch each other.</li>
 * </ul>
 * A coupling is not part of either vehicle's state: who tows whom is kept by the caller (the game keeps it on the entities).
 */
public record Coupling(Kind kind, Vec3d leaderPoint, Vec3d followerPoint, Vec3d pitchAxis, Vec3d yawAxis,
                       double pitchLimitRad, double yawLimitRad, double rollLimitRad, double rollStiffnessNmPerRad, double rollDampingNms,
                       double captureM, List<String> forwarded, boolean collide) {
    public enum Kind { BALL, FIFTH_WHEEL }

    /** A fifth wheel's default capture distance, m. */
    public static final double CAPTURE_M = 0.15;

    public Coupling {
        Objects.requireNonNull(kind, "kind");
        Objects.requireNonNull(leaderPoint, "leaderPoint"); Objects.requireNonNull(followerPoint, "followerPoint");
        finite(leaderPoint, "Leader point"); finite(followerPoint, "Follower point");
        Objects.requireNonNull(pitchAxis, "pitchAxis"); Objects.requireNonNull(yawAxis, "yawAxis");
        pitchAxis = pitchAxis.normalized();
        yawAxis = yawAxis.normalized();
        if (Math.abs(pitchAxis.y()) > 0.9) { throw new IllegalArgumentException("The pitch axis must lie across the leader, not up"); }
        if (Math.abs(yawAxis.x()) > 0.9) { throw new IllegalArgumentException("The yaw axis must stand in the follower, not lie along it"); }
        if (!(pitchLimitRad > 0 && pitchLimitRad <= Math.PI)) { throw new IllegalArgumentException("Pitch limit must be in (0, PI]"); }
        if (!(yawLimitRad > 0 && yawLimitRad <= Math.PI)) { throw new IllegalArgumentException("Yaw limit must be in (0, PI]"); }
        if (!(rollLimitRad >= 0 && rollLimitRad <= Math.PI)) { throw new IllegalArgumentException("Roll limit must be in [0, PI]"); }
        if (!(rollStiffnessNmPerRad >= 0) || Double.isNaN(rollStiffnessNmPerRad)) { throw new IllegalArgumentException("Roll stiffness must be nonnegative"); }
        if (!(rollDampingNms >= 0) || !Double.isFinite(rollDampingNms)) { throw new IllegalArgumentException("Roll damping must be finite and nonnegative"); }
        if (rollLimitRad > 0 && rollStiffnessNmPerRad == Double.POSITIVE_INFINITY) { throw new IllegalArgumentException("A roll with play cannot be rigid"); }
        if (!(captureM >= 0) || !Double.isFinite(captureM)) { throw new IllegalArgumentException("Capture distance must be finite and nonnegative"); }
        forwarded = List.copyOf(forwarded);
    }

    /** A fifth wheel: the hinge across the leader (+z), the kingpin up the follower (+y), roll locked rigidly, the default capture. */
    public static Coupling fifthWheel(Vec3d leaderPoint, Vec3d followerPoint, double pitchLimitRad, double yawLimitRad) {
        return new Coupling(Kind.FIFTH_WHEEL, leaderPoint, followerPoint, Vec3d.Z, Vec3d.Y, pitchLimitRad, yawLimitRad, 0,
                Double.POSITIVE_INFINITY, 0, CAPTURE_M, List.of(), false);
    }

    /** A ball: the points held together, no stops, roll free. */
    public static Coupling ball(Vec3d leaderPoint, Vec3d followerPoint) {
        return new Coupling(Kind.BALL, leaderPoint, followerPoint, Vec3d.Z, Vec3d.Y, Math.PI, Math.PI, Math.PI, 0, 0, CAPTURE_M, List.of(), false);
    }

    public Coupling withForwarded(List<String> channels) {
        return new Coupling(kind, leaderPoint, followerPoint, pitchAxis, yawAxis, pitchLimitRad, yawLimitRad, rollLimitRad,
                rollStiffnessNmPerRad, rollDampingNms, captureM, channels, collide);
    }
    public Coupling withRoll(double limitRad, double stiffnessNmPerRad, double dampingNms) {
        return new Coupling(kind, leaderPoint, followerPoint, pitchAxis, yawAxis, pitchLimitRad, yawLimitRad, limitRad,
                stiffnessNmPerRad, dampingNms, captureM, forwarded, collide);
    }
    public Coupling withLimits(double pitchRad, double yawRad) {
        return new Coupling(kind, leaderPoint, followerPoint, pitchAxis, yawAxis, pitchRad, yawRad, rollLimitRad,
                rollStiffnessNmPerRad, rollDampingNms, captureM, forwarded, collide);
    }
    public Coupling withCapture(double metres) {
        return new Coupling(kind, leaderPoint, followerPoint, pitchAxis, yawAxis, pitchLimitRad, yawLimitRad, rollLimitRad,
                rollStiffnessNmPerRad, rollDampingNms, metres, forwarded, collide);
    }
    public Coupling withCollide(boolean on) {
        return new Coupling(kind, leaderPoint, followerPoint, pitchAxis, yawAxis, pitchLimitRad, yawLimitRad, rollLimitRad,
                rollStiffnessNmPerRad, rollDampingNms, captureM, forwarded, on);
    }

    /** Whether the roll is held (a spring or a rigid lock about square), rather than free or free within play. */
    public boolean rollHeld() { return rollLimitRad == 0; }
    /** Whether the roll has a stop: held, or free within play. */
    public boolean rollStopped() { return rollLimitRad < Math.PI / 2; }

    /**
     * Where the joint stands between two poses: each point's world offset from its body's COM, the gap from the leader's
     * point to the follower's (world), the hinge and kingpin axes (world) and the three angles (see the class).
     */
    public record Geometry(Vec3d leaderOffset, Vec3d followerOffset, Vec3d gap, Vec3d pitchAxis, Vec3d yawAxis,
                           double pitchRad, double yawRad, double rollRad) {}

    public Geometry geometry(RigidBodyState leader, RigidBodyProperties leaderProperties, RigidBodyState follower, RigidBodyProperties followerProperties) {
        Vec3d rl = leader.worldOffset(leaderProperties, leaderPoint), rf = follower.worldOffset(followerProperties, followerPoint);
        Vec3d gap = follower.position().add(rf).subtract(leader.position().add(rl));
        Vec3d p = leader.orientation().rotate(pitchAxis), k = follower.orientation().rotate(yawAxis);
        // pitch: the kingpin turned about the hinge from the leader's up
        Vec3d up = leader.orientation().rotate(square(Vec3d.Y, pitchAxis)), kp = k.subtract(p.scale(k.dot(p)));
        double pitch = Math.atan2(up.cross(kp).dot(p), up.dot(kp));
        // yaw: the follower's forward turned about the kingpin from the leader's
        Vec3d fl = leader.orientation().rotate(Vec3d.X), ff = follower.orientation().rotate(Vec3d.X);
        fl = fl.subtract(k.scale(fl.dot(k)));
        ff = ff.subtract(k.scale(ff.dot(k)));
        double yaw = Math.atan2(fl.cross(ff).dot(k), fl.dot(ff));
        double roll = Math.asin(Math.clamp(p.dot(k), -1, 1));
        return new Geometry(rl, rf, gap, p, k, pitch, yaw, roll);
    }

    private static Vec3d square(Vec3d v, Vec3d axis) { return v.subtract(axis.scale(v.dot(axis))).normalized(); }
    private static void finite(Vec3d v, String name) {
        if (!Double.isFinite(v.x()) || !Double.isFinite(v.y()) || !Double.isFinite(v.z())) { throw new IllegalArgumentException(name + " must be finite"); }
    }

    /** JSON: a rigid roll leaves out its stiffness; the defaults (no collide, no channels) are left out. */
    public Map<String, Object> toJson() {
        Map<String, Object> m = Fields.object("kind", kind.name().toLowerCase(java.util.Locale.ROOT),
                "leaderPoint", Fields.vector(leaderPoint), "followerPoint", Fields.vector(followerPoint),
                "pitchAxis", Fields.vector(pitchAxis), "yawAxis", Fields.vector(yawAxis),
                "pitchLimitRad", pitchLimitRad, "yawLimitRad", yawLimitRad, "rollLimitRad", rollLimitRad);
        if (rollStiffnessNmPerRad != Double.POSITIVE_INFINITY) { m.put("rollStiffnessNmPerRad", rollStiffnessNmPerRad); }
        if (rollDampingNms != 0) { m.put("rollDampingNms", rollDampingNms); }
        m.put("captureM", captureM);
        if (!forwarded.isEmpty()) { m.put("forwarded", new ArrayList<Object>(forwarded)); }
        if (collide) { m.put("collide", true); }
        return m;
    }

    public static Coupling fromJson(Object value, String path) {
        Fields f = Fields.of(value, path);
        Kind kind = Kind.valueOf(f.string("kind").toUpperCase(java.util.Locale.ROOT));
        List<String> forwarded = new ArrayList<>();
        for (Object o : f.list("forwarded", List.of())) { forwarded.add(String.valueOf(o)); }
        return new Coupling(kind, f.vec("leaderPoint"), f.vec("followerPoint"), f.vec("pitchAxis", Vec3d.Z), f.vec("yawAxis", Vec3d.Y),
                f.number("pitchLimitRad", Math.PI), f.number("yawLimitRad", Math.PI), f.number("rollLimitRad", 0),
                f.number("rollStiffnessNmPerRad", Double.POSITIVE_INFINITY), f.number("rollDampingNms", 0),
                f.number("captureM", CAPTURE_M), forwarded, f.bool("collide", false));
    }

    public static Coupling parse(String json) { return fromJson(Json.parse(json), "coupling"); }
    public String write() { return Json.write(toJson()); }
}
