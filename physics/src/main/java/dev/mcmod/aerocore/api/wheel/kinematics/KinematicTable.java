package dev.mcmod.aerocore.api.wheel.kinematics;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;
import java.util.Objects;

/**
 * How a wheel moves on its hardpoints (0.33): the upright (the knuckle, carrying the hub) as a rigid body held by its
 * links (each a fixed length between a point on the body and one on the upright; a strut a line through its top mount),
 * driven by the hub's height, solved by Newton's method continued from the design position, at SAMPLES compressions from
 * full droop to past the bump stop (1.5 times the travel). Compression is WheelCorner's: the hub's rise from full
 * droop, the design compression at the design position.
 *
 * <p>Each sample (for the right wheel; the left is the mirror image): the hub (it moves along a curve, not straight
 * up), the upright's rotation from the design, the camber change (positive top out) and toe change (positive toe-in:
 * bump steer) from the design, and the slopes of the paths that set how forces reach the body (the force that does no
 * work along the wheel's travel goes through the links): the contact patch's (a point fixed to the upright one tyre
 * radius under the hub) sideways rise per rise, dz/dy, which jacks the body by the lateral force times it (the front
 * view's line to the instant centre: the roll centre is it times half the track over the ground), and its and the
 * hub's forward motion per rise, dx/dy (side view: anti-dive and anti-squat; the contact patch's when the brakes or a
 * solid axle react the torque, the hub's for an independent driven wheel), and the motion ratio (spring compression
 * per wheel rise). A solid axle's wheels stay square to the ground: only its side view moves (the links' arc), its
 * roll centre the locator's height.
 *
 * <p>Not modelled: compliance in bushes (roll steer), camber change with steering, the steering's own geometry (the
 * steer angle is applied about the vertical as before); caster, kingpin, scrub and trail are given at the design
 * position for the bench.
 */
public final class KinematicTable {
    /** Samples over the compression. */
    public static final int SAMPLES = 81;
    /** Past the bump stop, as a share of the travel. */
    public static final double RANGE = 1.5;
    /** The step for the slopes, m. */
    static final double H = 1e-4;

    /** One position: compression (m), the hub, the upright's rotation from the design, camber, toe, the slopes and the motion ratio. */
    public record Sample(double compressionM, Vec3d hub, Orientation upright, double camberRad, double toeRad, double frontJackTan, double sideTanContact,
                         double sideTanHub, double motionRatio) {}

    /** The steering axis at the design position: caster (top rearward positive), kingpin inclination (top inward), scrub radius and trail at the ground (m). */
    public record SteeringAxis(double casterRad, double kingpinRad, double scrubM, double trailM) {}

    private final Hardpoints hardpoints;
    private final double tyreRadiusM, designCompressionM, lo, hi;
    private final Sample[] samples;
    private final SteeringAxis axis;

    private KinematicTable(Hardpoints hardpoints, double tyreRadiusM, double designCompressionM, double lo, double hi, Sample[] samples, SteeringAxis axis) {
        this.hardpoints = hardpoints; this.tyreRadiusM = tyreRadiusM; this.designCompressionM = designCompressionM; this.lo = lo; this.hi = hi;
        this.samples = samples; this.axis = axis;
    }

    public Hardpoints hardpoints() { return hardpoints; }
    public double tyreRadiusM() { return tyreRadiusM; }
    public double designCompressionM() { return designCompressionM; }
    /** The compressions the table covers (it holds its ends beyond them). */
    public double minCompressionM() { return lo; }
    public double maxCompressionM() { return hi; }
    public SteeringAxis steeringAxis() { return axis; }
    public List<Sample> samples() { return List.of(samples); }

    /** The roll centre's height over the ground at the design position, m (the front-view jacking slope times half the track). */
    public double rollCentreHeightM() { return at(designCompressionM).frontJackTan() * hardpoints.hub().z(); }

    /**
     * The table for hardpoints (right wheel, part frame, design position), the tyre's loaded radius, the design
     * compression and the travel (m). Throws LinkageLockException when the links bind within the travel; past it the
     * table ends where they bind.
     */
    public static KinematicTable of(Hardpoints hardpoints, double tyreRadiusM, double designCompressionM, double travelM) {
        Objects.requireNonNull(hardpoints, "hardpoints");
        if (!(tyreRadiusM > 0) || !(travelM > 0) || !(designCompressionM >= 0)) { throw new IllegalArgumentException("Radius and travel must be positive"); }
        Mechanism m = Mechanism.of(hardpoints, tyreRadiusM);
        double top = RANGE * travelM;
        double[] cs = new double[SAMPLES];
        for (int i = 0; i < SAMPLES; i++) { cs[i] = top * i / (SAMPLES - 1); }
        Sample[] out = new Sample[SAMPLES];
        int first = 0, last = SAMPLES - 1;
        // from the design position up, then down
        double[] q = new double[m.unknowns()];
        double[] start = q.clone();
        for (int i = 0; i < SAMPLES; i++) {
            if (cs[i] < designCompressionM) { continue; }
            try {
                q = m.solve(q, cs[i] - designCompressionM);
                out[i] = m.sample(q, cs[i], cs[i] - designCompressionM);
            } catch (LinkageLockException e) {
                if (cs[i] <= travelM + 1e-9) { throw new LinkageLockException(cs[i], e.getMessage()); }
                last = i - 1;
                break;
            }
        }
        q = start;
        for (int i = SAMPLES - 1; i >= 0; i--) {
            if (cs[i] >= designCompressionM) { continue; }
            try {
                q = m.solve(q, cs[i] - designCompressionM);
                out[i] = m.sample(q, cs[i], cs[i] - designCompressionM);
            } catch (LinkageLockException e) {
                throw new LinkageLockException(cs[i], e.getMessage());
            }
        }
        if (last < first + 1) { throw new LinkageLockException(designCompressionM, "the links bind at the design position"); }
        Sample[] kept = new Sample[last - first + 1];
        System.arraycopy(out, first, kept, 0, kept.length);
        return new KinematicTable(hardpoints, tyreRadiusM, designCompressionM, cs[first], cs[last], kept, m.axis());
    }

    /** The toe change (rad, toe-in) with the hub dy above its design position, solved in a few steps from there. */
    static double toe(Hardpoints hardpoints, double tyreRadiusM, double dy) {
        Mechanism m = Mechanism.of(hardpoints, tyreRadiusM);
        double[] q = new double[m.unknowns()];
        for (int i = 1; i <= 5; i++) { q = m.solve(q, dy * i / 5); }
        Vec3d axle = m.rotation(q).rotate(Vec3d.Z);
        return Math.atan2(axle.x(), axle.z());
    }

    /** The sample at a compression (linear between samples, the ends held beyond them; the rotation interpolated). */
    public Sample at(double c) {
        if (samples.length == 1 || c <= lo) { return samples[0]; }
        if (c >= hi) { return samples[samples.length - 1]; }
        double f = (c - lo) / (hi - lo) * (samples.length - 1);
        int i = Math.min((int) f, samples.length - 2);
        double t = f - i;
        Sample a = samples[i], b = samples[i + 1];
        return new Sample(c, lerp(a.hub(), b.hub(), t), a.upright().interpolate(b.upright(), t), a.camberRad() + t * (b.camberRad() - a.camberRad()),
                a.toeRad() + t * (b.toeRad() - a.toeRad()), a.frontJackTan() + t * (b.frontJackTan() - a.frontJackTan()),
                a.sideTanContact() + t * (b.sideTanContact() - a.sideTanContact()), a.sideTanHub() + t * (b.sideTanHub() - a.sideTanHub()),
                a.motionRatio() + t * (b.motionRatio() - a.motionRatio()));
    }

    /** The hub's offset from straight up through the design position at a compression (x, 0, z; right wheel), m. */
    public Vec3d hubOffset(double c) {
        Vec3d h = at(c).hub(), d = hardpoints.hub();
        return new Vec3d(h.x() - d.x(), 0, h.z() - d.z());
    }

    /** Where a point fixed to the upright (given at the design position) is at a sample. */
    public Vec3d onUpright(Sample s, Vec3d designPoint) { return s.hub().add(s.upright().rotate(designPoint.subtract(hardpoints.hub()))); }

    private static Vec3d lerp(Vec3d a, Vec3d b, double t) { return a.add(b.subtract(a).scale(t)); }

    /** A linkage: its unknowns, residuals and what a solved position shows. */
    private abstract static class Mechanism {
        final Hardpoints hp;
        final Vec3d hub0, contact0;

        Mechanism(Hardpoints hp, double radius) { this.hp = hp; this.hub0 = hp.hub(); this.contact0 = hub0.subtract(new Vec3d(0, radius, 0)); }

        static Mechanism of(Hardpoints hp, double radius) {
            return switch (hp) {
                case Hardpoints.DoubleWishbone d -> new Links(d, radius, Hardpoints.asMultiLink(d).links(), null) {
                    @Override double springLength(double[] q) {
                        Vec3d ball = point(q, d.lowerBall()), arm = aboutAxis(d.lowerFront(), d.lowerRear(), d.lowerBall(), ball, d.springArm());
                        return arm.subtract(d.springBody()).length();
                    }
                    @Override SteeringAxis axis() { return steering(d.lowerBall(), d.upperBall(), hub0, contact0); }
                };
                case Hardpoints.MultiLink ml -> new Links(ml, radius, ml.links(), null) {
                    @Override double springLength(double[] q) { return point(q, ml.springUpright()).subtract(ml.springBody()).length(); }
                };
                case Hardpoints.MacPherson mp -> new Links(mp, radius, List.of(new Hardpoints.Link(mp.lowerFront(), mp.lowerBall()), new Hardpoints.Link(mp.lowerRear(), mp.lowerBall()),
                        new Hardpoints.Link(mp.tieInner(), mp.tieOuter())), mp) {
                    @Override double springLength(double[] q) { return point(q, mp.strutLower()).subtract(mp.strutTop()).length(); }
                    @Override SteeringAxis axis() { return steering(mp.lowerBall(), mp.strutTop(), hub0, contact0); }
                };
                case Hardpoints.TrailingArm t -> new Arm(t, radius);
                case Hardpoints.SolidAxle s -> new Solid(s, radius);
            };
        }

        abstract int unknowns();
        abstract double[] residuals(double[] q, double dy);
        abstract Orientation rotation(double[] q);
        abstract Vec3d translation(double[] q);
        abstract double springLength(double[] q);
        SteeringAxis axis() { return new SteeringAxis(Double.NaN, Double.NaN, Double.NaN, Double.NaN); }
        double frontJack(Vec3d dContact) { return dContact.z() / dContact.y(); }
        boolean square() { return false; }

        /** A point fixed to the upright at a position. */
        Vec3d point(double[] q, Vec3d design) { return hub0.add(translation(q)).add(rotation(q).rotate(design.subtract(hub0))); }

        /** Newton's method from q for the hub dy above the design position. */
        double[] solve(double[] q0, double dy) {
            int n = unknowns();
            double[] q = q0.clone();
            for (int it = 0; it < 40; it++) {
                double[] f = residuals(q, dy);
                double norm = 0;
                for (double v : f) { norm = Math.max(norm, Math.abs(v)); }
                if (norm < 1e-13) { return q; }
                double[][] j = new double[n][n];
                for (int k = 0; k < n; k++) {
                    double[] a = q.clone(), b = q.clone();
                    a[k] += 1e-7; b[k] -= 1e-7;
                    double[] fa = residuals(a, dy), fb = residuals(b, dy);
                    for (int r = 0; r < n; r++) { j[r][k] = (fa[r] - fb[r]) / 2e-7; }
                }
                double[] dq = gauss(j, f);
                if (dq == null) { throw new LinkageLockException(dy, "the links are square to their motion"); }
                double scale = 1;
                double step = 0;
                for (double v : dq) { step = Math.max(step, Math.abs(v)); }
                if (step > 0.2) { scale = 0.2 / step; }
                for (int k = 0; k < n; k++) { q[k] -= scale * dq[k]; }
            }
            double[] f = residuals(q, dy);
            double norm = 0;
            for (double v : f) { norm = Math.max(norm, Math.abs(v)); }
            if (norm < 1e-9) { return q; }
            throw new LinkageLockException(dy, "no position holds all the links");
        }

        Sample sample(double[] q, double c, double dy) {
            double[] up = solve(q, dy + H), down = solve(q, dy - H);
            Vec3d hub = hub0.add(translation(q));
            Orientation r = rotation(q);
            Vec3d dContact = point(up, contact0).subtract(point(down, contact0)), dHub = translation(up).subtract(translation(down));
            double camber = 0, toe = 0;
            if (!square()) {
                Vec3d axle = r.rotate(Vec3d.Z);
                camber = Math.asin(Math.clamp(-axle.y(), -1, 1));
                toe = Math.atan2(axle.x(), axle.z());
            }
            double ratio = -(springLength(up) - springLength(down)) / (2 * H);
            return new Sample(c, hub, r, camber, toe, frontJack(dContact), dContact.x() / dContact.y(), dHub.x() / dHub.y(), ratio);
        }
    }

    /** An upright held by links (and a strut): position and rotation vector, six unknowns. */
    private abstract static class Links extends Mechanism {
        final List<Hardpoints.Link> links;
        final double[] lengths;
        final Hardpoints.MacPherson strut;
        final Vec3d strutAxis, across1, across2;

        Links(Hardpoints hp, double radius, List<Hardpoints.Link> links, Hardpoints.MacPherson strut) {
            super(hp, radius);
            this.links = links; this.strut = strut;
            lengths = new double[links.size()];
            for (int i = 0; i < links.size(); i++) { lengths[i] = links.get(i).length(); }
            if (strut != null) {
                strutAxis = strut.strutTop().subtract(strut.strutLower()).normalized();
                Vec3d any = Math.abs(strutAxis.x()) < 0.9 ? Vec3d.X : Vec3d.Y;
                across1 = strutAxis.cross(any).normalized();
                across2 = strutAxis.cross(across1).normalized();
            } else {
                strutAxis = across1 = across2 = null;
            }
        }

        @Override int unknowns() { return 6; }
        @Override Orientation rotation(double[] q) { return Orientation.rotationVector(new Vec3d(q[3], q[4], q[5])); }
        @Override Vec3d translation(double[] q) { return new Vec3d(q[0], q[1], q[2]); }

        @Override double[] residuals(double[] q, double dy) {
            double[] f = new double[6];
            int i = 0;
            for (int k = 0; k < links.size(); k++) {
                Hardpoints.Link l = links.get(k);
                f[i++] = point(q, l.upright()).subtract(l.body()).length() - lengths[k];
            }
            if (strut != null) {
                Orientation r = rotation(q);
                Vec3d d = strut.strutTop().subtract(point(q, strut.strutLower()));
                f[i++] = d.dot(r.rotate(across1));
                f[i++] = d.dot(r.rotate(across2));
            }
            f[i] = q[1] - dy;
            return f;
        }
    }

    /** An upright fixed to an arm turning about its pivots' axis: one unknown, the angle. */
    private static final class Arm extends Mechanism {
        final Hardpoints.TrailingArm t;
        final Vec3d axis;

        Arm(Hardpoints.TrailingArm t, double radius) { super(t, radius); this.t = t; axis = t.pivotB().subtract(t.pivotA()).normalized(); }

        @Override int unknowns() { return 1; }
        @Override Orientation rotation(double[] q) { return Orientation.rotationVector(axis.scale(q[0])); }
        @Override Vec3d translation(double[] q) {
            Vec3d from = hub0.subtract(t.pivotA());
            return t.pivotA().add(rotation(q).rotate(from)).subtract(hub0);
        }
        @Override double[] residuals(double[] q, double dy) { return new double[]{translation(q).y() - dy}; }
        @Override double springLength(double[] q) { return point(q, t.springArm()).subtract(t.springBody()).length(); }
    }

    /** A solid axle in side view: fore-aft, height and pitch, held by its two trailing links; square to the ground. */
    private static final class Solid extends Mechanism {
        final Hardpoints.SolidAxle s;
        final double upper, lower;

        Solid(Hardpoints.SolidAxle s, double radius) {
            super(s, radius);
            this.s = s;
            upper = s.upperAxle().subtract(s.upperBody()).length(); lower = s.lowerAxle().subtract(s.lowerBody()).length();
        }

        @Override int unknowns() { return 3; }
        @Override Orientation rotation(double[] q) { return Orientation.rotationVector(new Vec3d(0, 0, q[2])); }
        @Override Vec3d translation(double[] q) { return new Vec3d(q[0], q[1], 0); }
        @Override double[] residuals(double[] q, double dy) {
            return new double[]{point(q, s.upperAxle()).subtract(s.upperBody()).length() - upper, point(q, s.lowerAxle()).subtract(s.lowerBody()).length() - lower, q[1] - dy};
        }
        @Override double springLength(double[] q) { return point(q, s.springAxle()).subtract(s.springBody()).length(); }
        @Override double frontJack(Vec3d dContact) { return (s.locatorY() - contact0.y()) / hub0.z(); }
        @Override boolean square() { return true; }
    }

    /** The lower arm turned about its pivot axis to carry its ball joint from its design point to where it is, and a point on it with it. */
    static Vec3d aboutAxis(Vec3d front, Vec3d rear, Vec3d ball0, Vec3d ball, Vec3d onArm) {
        Vec3d u = front.subtract(rear).normalized();
        Vec3d a = ball0.subtract(rear), b = ball.subtract(rear);
        Vec3d a1 = a.subtract(u.scale(a.dot(u))), b1 = b.subtract(u.scale(b.dot(u)));
        double angle = Math.atan2(a1.cross(b1).dot(u), a1.dot(b1));
        return rear.add(Orientation.rotationVector(u.scale(angle)).rotate(onArm.subtract(rear)));
    }

    /** The steering axis through two points (lower first) against the ground under the hub. */
    static SteeringAxis steering(Vec3d lower, Vec3d upper, Vec3d hub, Vec3d contact) {
        Vec3d d = upper.subtract(lower);
        double t = (contact.y() - lower.y()) / d.y();
        Vec3d g = lower.add(d.scale(t));
        return new SteeringAxis(Math.atan2(-d.x(), d.y()), Math.atan2(-d.z(), d.y()), contact.z() - g.z(), g.x() - contact.x());
    }

    /** Gaussian elimination with partial pivoting; null when singular. */
    static double[] gauss(double[][] a0, double[] b0) {
        int n = b0.length;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++) { a[i] = a0[i].clone(); }
        double[] b = b0.clone();
        double scale = 0;
        for (double[] row : a) { for (double v : row) { scale = Math.max(scale, Math.abs(v)); } }
        for (int c = 0; c < n; c++) {
            int p = c;
            for (int r = c + 1; r < n; r++) { if (Math.abs(a[r][c]) > Math.abs(a[p][c])) { p = r; } }
            if (Math.abs(a[p][c]) <= 1e-10 * Math.max(scale, 1e-30)) { return null; }
            double[] t = a[c]; a[c] = a[p]; a[p] = t;
            double tb = b[c]; b[c] = b[p]; b[p] = tb;
            for (int r = c + 1; r < n; r++) {
                double f = a[r][c] / a[c][c];
                if (f == 0) { continue; }
                for (int k = c; k < n; k++) { a[r][k] -= f * a[c][k]; }
                b[r] -= f * b[c];
            }
        }
        double[] x = new double[n];
        for (int r = n - 1; r >= 0; r--) {
            double s = b[r];
            for (int k = r + 1; k < n; k++) { s -= a[r][k] * x[k]; }
            x[r] = s / a[r][r];
        }
        return x;
    }
}
