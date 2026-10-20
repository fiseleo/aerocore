package dev.mcmod.aerocore.api.wheel.kinematics;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.wheel.Suspension;

import java.util.List;
import java.util.Objects;

/**
 * A suspension's hardpoints (0.33): where its arms, links and spring are, for the right-hand (+z) wheel in the part's
 * frame (x forward, y up, z starboard) at the design compression, with the wheel centre there (hub). The left wheel is
 * the mirror image. KinematicTable solves how the wheel moves over its travel.
 *
 * <ul>
 * <li>Double wishbone: each arm's front and rear pivots on the body and its ball joint on the upright; the tie rod's
 *     inner (rack) and outer (steering arm) ends; the spring from a point on the body to a point on the lower arm.</li>
 * <li>MacPherson strut: the strut's top mount on the body and a point on its axis fixed to the upright (the strut slides
 *     along the line between them), the lower arm, the tie rod; the spring is the strut.</li>
 * <li>Multi-link: five links, each from a point on the body to a point on the upright (one of them the toe link); the
 *     spring from the body to the upright. A double wishbone is a multi-link whose links meet in pairs.</li>
 * <li>Trailing arm (also semi-trailing, swing axle and, approximately, twist beam): the upright fixed to an arm turning
 *     about the axis through two pivots; the spring from the body to a point on the arm.</li>
 * <li>Solid axle: the lateral locator's height at the centreline (a Panhard rod's or Watt linkage's pivot, a leaf
 *     spring's seat: the roll centre), and in side view an upper and a lower trailing link from the body to the axle
 *     (parallel and level: no anti-dive or anti-squat); the spring from the body to the axle. The wheels stay square to
 *     the ground (no camber or toe change).</li>
 * </ul>
 */
public sealed interface Hardpoints {
    /** The wheel centre at the design compression. */
    Vec3d hub();

    record DoubleWishbone(Vec3d hub, Vec3d lowerFront, Vec3d lowerRear, Vec3d lowerBall, Vec3d upperFront, Vec3d upperRear, Vec3d upperBall,
                          Vec3d tieInner, Vec3d tieOuter, Vec3d springBody, Vec3d springArm) implements Hardpoints {
        public DoubleWishbone {
            for (Vec3d v : new Vec3d[]{hub, lowerFront, lowerRear, lowerBall, upperFront, upperRear, upperBall, tieInner, tieOuter, springBody, springArm}) { Objects.requireNonNull(v); }
        }
    }

    record MacPherson(Vec3d hub, Vec3d strutTop, Vec3d strutLower, Vec3d lowerFront, Vec3d lowerRear, Vec3d lowerBall, Vec3d tieInner,
                      Vec3d tieOuter) implements Hardpoints {
        public MacPherson {
            for (Vec3d v : new Vec3d[]{hub, strutTop, strutLower, lowerFront, lowerRear, lowerBall, tieInner, tieOuter}) { Objects.requireNonNull(v); }
            if (strutTop.subtract(strutLower).length() < 0.05) { throw new IllegalArgumentException("A strut needs its top and a lower point apart"); }
        }
    }

    /** A link from a point on the body to a point on the upright. */
    record Link(Vec3d body, Vec3d upright) {
        public Link { Objects.requireNonNull(body); Objects.requireNonNull(upright); }
        public double length() { return body.subtract(upright).length(); }
    }

    record MultiLink(Vec3d hub, List<Link> links, Vec3d springBody, Vec3d springUpright) implements Hardpoints {
        public MultiLink {
            Objects.requireNonNull(hub); Objects.requireNonNull(springBody); Objects.requireNonNull(springUpright);
            links = List.copyOf(links);
            if (links.size() != 5) { throw new IllegalArgumentException("A multi-link has five links"); }
        }
    }

    record TrailingArm(Vec3d hub, Vec3d pivotA, Vec3d pivotB, Vec3d springBody, Vec3d springArm) implements Hardpoints {
        public TrailingArm {
            for (Vec3d v : new Vec3d[]{hub, pivotA, pivotB, springBody, springArm}) { Objects.requireNonNull(v); }
            if (pivotA.subtract(pivotB).length() < 0.01) { throw new IllegalArgumentException("A trailing arm needs two pivots apart"); }
        }
    }

    record SolidAxle(Vec3d hub, double locatorY, Vec3d upperBody, Vec3d upperAxle, Vec3d lowerBody, Vec3d lowerAxle, Vec3d springBody,
                     Vec3d springAxle) implements Hardpoints {
        public SolidAxle {
            for (Vec3d v : new Vec3d[]{hub, upperBody, upperAxle, lowerBody, lowerAxle, springBody, springAxle}) { Objects.requireNonNull(v); }
            if (!Double.isFinite(locatorY)) { throw new IllegalArgumentException("The locator's height must be finite"); }
        }
    }

    /** The double wishbone as five links (lower front, lower rear, upper front, upper rear, tie rod). */
    static MultiLink asMultiLink(DoubleWishbone d) {
        return new MultiLink(d.hub(), List.of(new Link(d.lowerFront(), d.lowerBall()), new Link(d.lowerRear(), d.lowerBall()), new Link(d.upperFront(), d.upperBall()),
                new Link(d.upperRear(), d.upperBall()), new Link(d.tieInner(), d.tieOuter())), d.springBody(), d.springArm());
    }

    /**
     * Hardpoints that give a layout's simple geometry (Suspension.Geometry.of: its front-view swing arm and instant
     * centre, so the same roll centre and camber gain at the design position) for a track, the hub at the design
     * compression (part frame) and the tyre's radius (the ground that far under the hub): the arms' inner pivots on the
     * lines from their ball joints to the instant centre, pivot axes along x and level (no anti-dive or anti-squat), the
     * tie rod on the line to the instant centre (no bump steer to first order) with its inner end where the toe changes
     * alike 50 mm up and down (none to second order: found from the classic start, between the arms' inner ends). Arms scale
     * with the tyre (a 0.31 m tyre: a 0.38 m lower arm). The starting point of a hardpoint design.
     */
    static Hardpoints fromSimple(Suspension.Type type, double trackM, Vec3d hub, double tyreRadiusM, double swingArmM, double linkHeightM, double sweepRad) {
        Objects.requireNonNull(type, "type"); Objects.requireNonNull(hub, "hub");
        if (!(trackM > 0) || !(tyreRadiusM > 0)) { throw new IllegalArgumentException("Track and tyre radius must be positive"); }
        Suspension.Geometry g = Suspension.Geometry.of(type, trackM, tyreRadiusM, swingArmM, linkHeightM, sweepRad);
        double k = tyreRadiusM / 0.31, ground = hub.y() - tyreRadiusM, half = trackM / 2;
        double length = g.camberGainPerM() != 0 ? -1 / g.camberGainPerM() : Double.POSITIVE_INFINITY;
        double icHeight = Double.isFinite(length) ? g.rollCentreHeightM() * length / half : 0;
        // the instant centre in front view (z, y): inboard of the contact patch by the swing arm, at its height
        double icZ = hub.z() - length, icY = ground + icHeight;
        return switch (type) {
            case DOUBLE_WISHBONE, MULTI_LINK -> {
                Vec3d lowerBall = hub.add(new Vec3d(0, -0.13 * k, -0.02 * k)), upperBall = hub.add(new Vec3d(0, 0.17 * k, -0.06 * k));
                Vec3d lowerInner = toward(lowerBall, icZ, icY, 0.38 * k), upperInner = toward(upperBall, icZ, icY, 0.28 * k);
                Vec3d tieOuter = hub.add(new Vec3d(-0.13 * k, 0.02 * k, -0.07 * k));
                Vec3d tieInner = tieInner(tieOuter, icZ, icY, lowerInner, upperInner);
                Vec3d springArm = lowerInner.add(lowerBall.subtract(lowerInner).scale(0.6)), springBody = springArm.add(new Vec3d(0, 0.32 * k, -0.04 * k));
                Vec3d lf = lowerInner.add(new Vec3d(0.14 * k, 0, 0)), lr = lowerInner.add(new Vec3d(-0.14 * k, 0, 0));
                Vec3d uf = upperInner.add(new Vec3d(0.11 * k, 0, 0)), ur = upperInner.add(new Vec3d(-0.11 * k, 0, 0));
                tieInner = levelled(tieOuter, tieInner, tyreRadiusM, 0.05 * k,
                        q -> new DoubleWishbone(hub, lf, lr, lowerBall, uf, ur, upperBall, q, tieOuter, springBody, springArm));
                Vec3d tie = tieInner;
                DoubleWishbone d = new DoubleWishbone(hub, lf, lr, lowerBall, uf, ur, upperBall, tie, tieOuter, springBody, springArm);
                yield type == Suspension.Type.DOUBLE_WISHBONE ? d : asMultiLink(d);
            }
            case MACPHERSON -> {
                Vec3d lowerBall = hub.add(new Vec3d(0, -0.12 * k, -0.04 * k)), lowerInner = toward(lowerBall, icZ, icY, 0.36 * k);
                Vec3d strutLower = hub.add(new Vec3d(0, 0.12 * k, -0.10 * k));
                double yT = hub.y() + 0.58 * k, c = (icY - yT) * (yT - strutLower.y()), sum = icZ + strutLower.z(), disc = Math.pow(icZ - strutLower.z(), 2) + 4 * c;
                // the strut top where the line from it square to the strut's axis meets the lower arm's line at the instant centre
                double zT = disc >= 0 ? 0.5 * (sum + Math.sqrt(disc)) : strutLower.z() - 0.1 * k;
                if (disc >= 0 && Math.abs(0.5 * (sum - Math.sqrt(disc)) - strutLower.z()) < Math.abs(zT - strutLower.z())) { zT = 0.5 * (sum - Math.sqrt(disc)); }
                Vec3d strutTop = new Vec3d(hub.x(), yT, zT);
                Vec3d tieOuter = hub.add(new Vec3d(-0.13 * k, 0.02 * k, -0.08 * k));
                Vec3d lf = lowerInner.add(new Vec3d(0.14 * k, 0, 0)), lr = lowerInner.add(new Vec3d(-0.14 * k, 0, 0));
                Vec3d tieInner = levelled(tieOuter, tieInner(tieOuter, icZ, icY, lowerInner, strutTop), tyreRadiusM, 0.05 * k,
                        q -> new MacPherson(hub, strutTop, strutLower, lf, lr, lowerBall, q, tieOuter));
                yield new MacPherson(hub, strutTop, strutLower, lf, lr, lowerBall, tieInner, tieOuter);
            }
            case TRAILING_ARM -> new TrailingArm(hub, hub.add(new Vec3d(0.45 * k, 0, -0.32 * k)), hub.add(new Vec3d(0.45 * k, 0, -0.05 * k)),
                    hub.add(new Vec3d(-0.05 * k, 0.4 * k, -0.15 * k)), hub.add(new Vec3d(0.05 * k, 0, -0.15 * k)));
            case SEMI_TRAILING_ARM, SWING_AXLE, TWIST_BEAM -> {
                // the pivot axis pierces the plane across the hub at the instant centre; swept from across the car by the sweep
                double sweep = type == Suspension.Type.SWING_AXLE ? Math.PI / 2 : type == Suspension.Type.TWIST_BEAM ? Math.atan2(0.45, length)
                        : sweepRad > 0 ? sweepRad : Math.toRadians(20);
                Vec3d a, b;
                if (sweep > Math.toRadians(89)) {
                    a = new Vec3d(hub.x() + 0.12 * k, icY, icZ); b = new Vec3d(hub.x() - 0.12 * k, icY, icZ);
                } else {
                    double t = Math.tan(sweep);
                    a = new Vec3d(hub.x() + (length - 0.4 * k) * t, icY, hub.z() - 0.4 * k);
                    b = new Vec3d(hub.x() + (length - 0.1 * k) * t, icY, hub.z() - 0.1 * k);
                }
                yield new TrailingArm(hub, a, b, hub.add(new Vec3d(-0.05 * k, 0.4 * k, -0.15 * k)), hub.add(new Vec3d(0.05 * k, 0, -0.15 * k)));
            }
            case SOLID_PANHARD, SOLID_WATT, SOLID_LEAF -> new SolidAxle(hub, ground + g.rollCentreHeightM(),
                    hub.add(new Vec3d(0.5 * k, 0.15 * k, -0.25 * k)), hub.add(new Vec3d(0, 0.15 * k, -0.25 * k)),
                    hub.add(new Vec3d(0.55 * k, -0.1 * k, -0.3 * k)), hub.add(new Vec3d(0, -0.1 * k, -0.3 * k)),
                    hub.add(new Vec3d(0, 0.35 * k, -0.2 * k)), hub.add(new Vec3d(0, 0.05 * k, -0.2 * k)));
        };
    }

    /**
     * The tie rod's inner end moved along the rod's line (keeping it through the instant centre) until the toe changes
     * alike a distance up and down (secant method from the start; the start where that fails).
     */
    private static Vec3d levelled(Vec3d outer, Vec3d start, double radius, double dy, java.util.function.Function<Vec3d, Hardpoints> with) {
        Vec3d along = start.subtract(outer);
        java.util.function.DoubleUnaryOperator f = s -> {
            Hardpoints h = with.apply(outer.add(along.scale(s)));
            return KinematicTable.toe(h, radius, dy) + KinematicTable.toe(h, radius, -dy);
        };
        try {
            double s0 = 1, s1 = 1.05, f0 = f.applyAsDouble(s0), f1 = f.applyAsDouble(s1);
            for (int i = 0; i < 12 && Math.abs(f1) > 1e-9 && f1 != f0; i++) {
                double s2 = s1 - f1 * (s1 - s0) / (f1 - f0);
                if (!(s2 > 0.3 && s2 < 3)) { return start; }
                s0 = s1; f0 = f1; s1 = s2; f1 = f.applyAsDouble(s1);
            }
            return Math.abs(f1) < Math.abs(f.applyAsDouble(1)) ? outer.add(along.scale(s1)) : start;
        } catch (LinkageLockException e) {
            return start;
        }
    }

    /** A point a distance from a ball joint toward the instant centre, in front view (same x). */
    private static Vec3d toward(Vec3d ball, double icZ, double icY, double distance) {
        double dz = icZ - ball.z(), dy = icY - ball.y(), l = Math.hypot(dz, dy);
        if (!Double.isFinite(l) || l == 0) { return ball.add(new Vec3d(0, 0, -distance)); }
        return new Vec3d(ball.x(), ball.y() + dy / l * distance, ball.z() + dz / l * distance);
    }

    /**
     * The tie rod's inner end: on the line from its outer end to the instant centre, where that crosses the line
     * through the two inner points (front view): the classic construction for little bump steer.
     */
    private static Vec3d tieInner(Vec3d outer, double icZ, double icY, Vec3d innerA, Vec3d innerB) {
        double z1 = outer.z(), y1 = outer.y(), z2 = Double.isFinite(icZ) ? icZ : outer.z() - 100, y2 = Double.isFinite(icZ) ? icY : outer.y();
        double z3 = innerA.z(), y3 = innerA.y(), z4 = innerB.z(), y4 = innerB.y();
        double den = (z1 - z2) * (y3 - y4) - (y1 - y2) * (z3 - z4);
        if (Math.abs(den) < 1e-12) { return new Vec3d(outer.x(), y1, 0.5 * (z3 + z4)); }
        double t = ((z1 - z3) * (y3 - y4) - (y1 - y3) * (z3 - z4)) / den;
        return new Vec3d(outer.x(), y1 + t * (y2 - y1), z1 + t * (z2 - z1));
    }
}
