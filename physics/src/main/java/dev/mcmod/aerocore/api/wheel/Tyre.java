package dev.mcmod.aerocore.api.wheel;

import dev.mcmod.aerocore.api.GroundContact;

import java.util.Objects;

/**
 * A pneumatic tyre from its size, pressure, construction and tread (0.24): stiffnesses, contact patch, grip on each
 * surface, rolling resistance, hydroplaning speed and its steady force curves. Every property comes from the design
 * through a published relation or a stated rule; sources and checks are in docs/WHEELS.md.
 *
 * <ul>
 * <li>Geometry: section height H = width x aspect ratio, outside diameter = rim + 2H. Effective rolling radius
 *     r_e = r - delta / 3 (Whitbread, in Smiley and Horne, NACA TN 4110, eq. 66), loaded radius r - delta.</li>
 * <li>Vertical stiffness: road tyres by Rhyne's relation K_z = 0.00028 P sqrt(W OD) + 3.45 kg/mm (P kPa, footprint
 *     width W = (1.03 - 0.004 AR%) section width, OD mm; Padula, The Pneumatic Tire, NHTSA 2006, ch. 5 eqs. 4-9),
 *     linear up to bottoming at 0.7 H; aircraft tyres by Smiley and Horne eq. 24 (Type III, C_z = 0.03).</li>
 * <li>Nominal load (the load the curves are scaled at): road tyres the load that deflects them 0.16 H (a rule that puts
 *     205/55R16 at 230 kPa at 4.6 kN, 245/40R18 at 4.8 kN, where published Magic Formula sets put their F_z0);
 *     aircraft tyres the rated load 0.57 p_r w sqrt(w d) (Smiley and Horne eq. 27).</li>
 * <li>Cornering stiffness: road tyres Pacejka's K_ya = p_Ky1 F_z0 sin(p_Ky4 atan(F_z / (p_Ky2 F_z0))) with p_Ky4 1.8,
 *     p_Ky2 1.5 and p_Ky1 = 19 (0.55 / AR)^0.4 times the tread's stiffness (published sets: 20 and 23.7 for 205/55R16,
 *     21.9 for 245/40R18, 12.5 for 185/80R14); bias road tyres 0.7 of it (a rule); aircraft tyres Smiley and Horne
 *     eq. 82 (cornering power, Type III, +-20 %).</li>
 * <li>Longitudinal slip stiffness: road tyres C_k = F_z (21 + 4 dF_z) (published sets 18.5-25 F_z, dF_z the load's
 *     excess over nominal as a share of it) times the tread's stiffness; aircraft tyres 1.5 times the cornering
 *     stiffness (Pacejka: measured C_k is about 50 % above C_a).</li>
 * <li>Relaxation lengths: lateral C_a / K_y with the lateral carcass stiffness K_y = 0.6 K_z (road; gives 0.6 m for
 *     205/55R16 at 4.5 kN, Pacejka measured 0.56-0.69 m for 195/60R14 at 2.9-5.9 kN) or Smiley and Horne's lateral
 *     spring (aircraft, eq. 33); longitudinal half the lateral (TPT: braking relaxation is appreciably shorter).</li>
 * <li>Friction: the surface's peak (GroundContact.Surface grip D: dry 1.0, wet 0.82, gravel 0.6, sand 0.6, soil 0.55,
 *     mud 0.35, snow 0.3, ice 0.1) times the tread's factor for that kind of ground and the compound, falling 14 % per
 *     doubling of load (published sets -3.4 %/kN at 4 kN; a Goodyear tyre -14 % from nominal to double). The curve is
 *     Pacejka's sin(C atan(B s)) with C set by the surface's sliding-to-peak ratio at lock (tarmac 0.72: TPT 0.70,
 *     Balkwill 0.75; wet 0.80; gravel 0.92 and earth 0.95 from Wong's table; snow 0.75; ice 0.70) and B by the
 *     stiffness, so the peak slip is about 0.12 and the peak slip angle 6-8 degrees on tarmac.</li>
 * <li>Combined slip: normalised slips n_x = s_x / s_px, n_y = s_y / s_py share the curve along their direction (the
 *     friction ellipse of the peaks; exact for pure slip, linear stiffnesses unchanged).</li>
 * <li>Camber: an equivalent lateral slip 0.12 gamma (rule; radial camber stiffness about a tenth of the cornering
 *     stiffness). Aligning moment: the lateral force acts the brush model's pneumatic trail L / 6 behind the contact's
 *     centre (falling to zero at 1.5 times the peak slip), so M_z = -t F_y about the normal turns the wheel toward
 *     where it is going.</li>
 * <li>Rolling resistance: SAE J2452's P^a F_z^b (a + b v + c v^2) with Michelin's P205/65R15 coefficients (alpha
 *     -0.3947, beta 0.9468, 0.113, 1.89e-4, 2.24e-6 with v in m/s; NREL ADVISOR data), 8.8 N/kN at 4 kN, 240 kPa,
 *     80 km/h; times the tread's factor and the surface's firm resistance over tarmac's; on soft ground Bekker's
 *     compaction for a rigid wheel of the tyre's size is added (GroundContact).</li>
 * <li>Hydroplaning on wet ground (Horne and Dreher, NASA TN D-2056): V_p = 9 sqrt(p psi) knots = 0.0558 sqrt(p Pa) m/s
 *     for a smooth or rib tyre, raised by the tread's drainage (rule); grip fades from 0.7 V_p (where Horne saw spin-down
 *     begin) to a twentieth at V_p (rule).</li>
 * </ul>
 */
public final class Tyre {
    /** How the carcass is built. */
    public enum Construction { RADIAL, BIAS, AIRCRAFT }

    /** Kinds of ground for the tread's grip factors. */
    public enum Ground { DRY, WET, LOOSE, SNOW, ICE }

    /**
     * Tread patterns: grip factor on dry, wet, loose (gravel, sand, earth, mud), snow and ice ground; stiffness factor
     * (siped or tall blocks are softer); rolling resistance factor; drainage (how much the grooves raise the
     * hydroplaning speed). All rules.
     */
    public enum Tread {
        ROAD(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.2),
        PERFORMANCE(1.1, 1.05, 0.9, 0.6, 0.8, 1.15, 1.1, 1.1),
        WINTER(0.9, 1.0, 1.05, 1.4, 1.5, 0.8, 1.1, 1.3),
        ALL_TERRAIN(0.95, 0.9, 1.2, 1.15, 1.0, 0.85, 1.25, 1.3),
        MUD_TERRAIN(0.85, 0.75, 1.4, 1.1, 0.9, 0.7, 1.5, 1.4),
        SLICK(1.2, 0.5, 0.7, 0.3, 0.7, 1.25, 0.9, 1.0),
        AIRCRAFT_RIB(0.85, 1.0, 0.9, 1.0, 1.0, 1.0, 1.0, 1.0);

        final double dry, wet, loose, snow, ice, stiffness, rolling, drainage;

        Tread(double dry, double wet, double loose, double snow, double ice, double stiffness, double rolling, double drainage) {
            this.dry = dry; this.wet = wet; this.loose = loose; this.snow = snow; this.ice = ice; this.stiffness = stiffness; this.rolling = rolling;
            this.drainage = drainage;
        }

        /** The grip factor on a kind of ground. */
        public double grip(Ground g) {
            return switch (g) { case DRY -> dry; case WET -> wet; case LOOSE -> loose; case SNOW -> snow; case ICE -> ice; };
        }
        public double stiffness() { return stiffness; }
        public double rolling() { return rolling; }
        public double drainage() { return drainage; }
    }

    /**
     * A tyre's design: construction, section width (m), aspect ratio (section height over width, 0.55 for a "/55"),
     * rim diameter (m), inflation pressure (Pa), rated pressure (Pa; an aircraft tyre's, or a road tyre's maximum),
     * tread and compound grip (1 a normal compound; softer compounds grip more).
     */
    public record Spec(Construction construction, double sectionWidthM, double aspectRatio, double rimDiameterM, double pressurePa,
                       double ratedPressurePa, Tread tread, double compoundGrip) {
        public Spec {
            Objects.requireNonNull(construction, "construction"); Objects.requireNonNull(tread, "tread");
            if (!(sectionWidthM > 0.02 && sectionWidthM < 2)) { throw new IllegalArgumentException("Section width must be 0.02-2 m"); }
            if (!(aspectRatio > 0.15 && aspectRatio <= 1.5)) { throw new IllegalArgumentException("Aspect ratio must be 0.15-1.5"); }
            if (!(rimDiameterM > 0.05 && rimDiameterM < 3)) { throw new IllegalArgumentException("Rim diameter must be 0.05-3 m"); }
            if (!(pressurePa > 10e3 && pressurePa < 3e6)) { throw new IllegalArgumentException("Pressure must be 10 kPa-3 MPa"); }
            if (!(ratedPressurePa > 10e3 && ratedPressurePa < 3e6)) { throw new IllegalArgumentException("Rated pressure must be 10 kPa-3 MPa"); }
            if (!(compoundGrip > 0.3 && compoundGrip < 2)) { throw new IllegalArgumentException("Compound grip must be 0.3-2"); }
        }
    }

    /** Steady forces of the contact (N, along the heading and to the left) and the aligning moment (N m, about the normal). */
    public record Forces(double longitudinalN, double lateralN, double aligningNm) {}

    private static final double KG_PER_MM = 9.80665e3;
    /** Bottoming: deflection beyond this share of the section height meets the rim (stiffness times BOTTOMING_STIFFENING). */
    private static final double BOTTOMING = 0.7, BOTTOMING_STIFFENING = 20;
    /** Smiley and Horne's C_z for Type III tyres. */
    private static final double CZ = 0.03;

    private final Spec spec;
    private final double height, radius, rhyne, nominal;

    public Tyre(Spec spec) {
        this.spec = Objects.requireNonNull(spec, "spec");
        height = spec.sectionWidthM() * spec.aspectRatio();
        radius = spec.rimDiameterM() / 2 + height;
        double footprint = (1.03 - 0.4 * spec.aspectRatio()) * spec.sectionWidthM();
        rhyne = KG_PER_MM * (0.00028 * spec.pressurePa() / 1000 * Math.sqrt(footprint * 1000 * 2 * radius * 1000) + 3.45);
        if (spec.construction() == Construction.AIRCRAFT) {
            double w = spec.sectionWidthM(), d = 2 * radius;
            nominal = 0.57 * spec.ratedPressurePa() * w * Math.sqrt(w * d);
        } else {
            nominal = rhyne * 0.16 * height;
        }
    }

    public Spec spec() { return spec; }
    /** Unloaded outside radius, m. */
    public double radiusM() { return radius; }
    public double sectionHeightM() { return height; }
    /** The load the tyre's curves are scaled at (road: its nominal working load; aircraft: rated load), N. */
    public double nominalLoadN() { return nominal; }
    /** The load it is made to carry at most (road: 1.35 times nominal, a rule; aircraft: rated), N. */
    public double maxLoadN() { return spec.construction() == Construction.AIRCRAFT ? nominal : 1.35 * nominal; }

    // ----- vertical -----

    /** Vertical force (N) at a deflection (m), stiffening as it bottoms on the rim. */
    public double verticalForceN(double deflectionM) {
        if (!(deflectionM > 0)) { return 0; }
        double limit = BOTTOMING * height;
        if (spec.construction() == Construction.AIRCRAFT) {
            double w = spec.sectionWidthM(), d = 2 * radius, scale = (spec.pressurePa() + 0.08 * spec.ratedPressurePa()) * w * Math.sqrt(w * d);
            double x = Math.min(deflectionM, limit) / w, f = scale * 2.4 * (x - CZ * (1 - Math.exp(-0.6 * x / CZ)));
            return deflectionM <= limit ? f : f + BOTTOMING_STIFFENING * scale * 2.4 / w * (deflectionM - limit);
        }
        return deflectionM <= limit ? rhyne * deflectionM : rhyne * limit + BOTTOMING_STIFFENING * rhyne * (deflectionM - limit);
    }

    /** Tangent vertical stiffness (N/m) at a deflection. */
    public double verticalStiffness(double deflectionM) {
        double limit = BOTTOMING * height;
        if (spec.construction() == Construction.AIRCRAFT) {
            double w = spec.sectionWidthM(), d = 2 * radius, scale = (spec.pressurePa() + 0.08 * spec.ratedPressurePa()) * w * Math.sqrt(w * d);
            double x = Math.max(0, Math.min(deflectionM, limit)) / w, k = scale * 2.4 / w * (1 - 0.6 * Math.exp(-0.6 * x / CZ));
            return deflectionM <= limit ? k : BOTTOMING_STIFFENING * scale * 2.4 / w;
        }
        return deflectionM <= limit ? rhyne : BOTTOMING_STIFFENING * rhyne;
    }

    /** The deflection (m) that carries a load (N). */
    public double deflectionM(double loadN) {
        if (!(loadN > 0)) { return 0; }
        double lo = 0, hi = height;
        while (verticalForceN(hi) < loadN) { hi *= 2; }
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            if (verticalForceN(mid) < loadN) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }

    /** Effective rolling radius under a load: r - delta / 3, m. */
    public double effectiveRadiusM(double loadN) { return radius - deflectionM(loadN) / 3; }
    /** Contact patch length under a load (road: load over pressure and footprint width; aircraft: Smiley and Horne eq. 5), m. */
    public double contactLengthM(double loadN) {
        if (!(loadN > 0)) { return 0; }
        if (spec.construction() == Construction.AIRCRAFT) {
            double d = 2 * radius, r = Math.min(deflectionM(loadN) / d, 0.5);
            return 2 * 0.85 * d * Math.sqrt(r - r * r);
        }
        double footprint = (1.03 - 0.4 * spec.aspectRatio()) * spec.sectionWidthM();
        return Math.min(loadN / (spec.pressurePa() * footprint), 2 * radius);
    }

    // ----- in-plane and lateral stiffness -----

    /** Cornering stiffness at a load, N/rad. */
    public double corneringStiffness(double loadN) {
        double fz = Math.max(loadN, 0);
        if (spec.construction() == Construction.AIRCRAFT) {
            // Smiley and Horne eq. 82 (consistent units): N = C_c (p + 0.44 p_r) w^2 f(delta / d), C_c = 69 per rad for Type III
            double w = spec.sectionWidthM(), d = 2 * radius, r = deflectionM(fz) / d;
            double f = r <= 0.0875 ? 1.2 * r - 8.8 * r * r : Math.max(0.0674 - 0.34 * r, 0.005);
            return 69 * (spec.pressurePa() + 0.44 * spec.ratedPressurePa()) * w * w * Math.max(f, 0);
        }
        double p1 = 19 * Math.pow(0.55 / spec.aspectRatio(), 0.4) * spec.tread().stiffness() * (spec.construction() == Construction.BIAS ? 0.7 : 1);
        return p1 * nominal * Math.sin(1.8 * Math.atan(fz / (1.5 * nominal)));
    }

    /** Longitudinal slip stiffness at a load, N per unit slip ratio. */
    public double slipStiffness(double loadN) {
        double fz = Math.max(loadN, 0);
        if (spec.construction() == Construction.AIRCRAFT) { return 1.5 * corneringStiffness(fz); }
        double dfz = fz / nominal - 1;
        return fz * Math.max(21 + 4 * dfz, 5) * spec.tread().stiffness() * (spec.construction() == Construction.BIAS ? 0.8 : 1);
    }

    /** Lateral carcass stiffness at the contact under a load, N/m. */
    public double lateralStiffness(double loadN) {
        if (spec.construction() == Construction.AIRCRAFT) {
            // Smiley and Horne eq. 33: K = tau w (p + 0.24 p_r) (1 - 0.7 delta / w), tau = 2 for Type III
            double w = spec.sectionWidthM();
            return 2 * w * (spec.pressurePa() + 0.24 * spec.ratedPressurePa()) * Math.max(1 - 0.7 * deflectionM(loadN) / w, 0.2);
        }
        return 0.6 * rhyne;
    }

    /** Lateral relaxation length at a load, m (at least a tenth of the radius). */
    public double lateralRelaxationM(double loadN) { return Math.max(corneringStiffness(loadN) / lateralStiffness(loadN), 0.1 * radius); }
    /** Longitudinal relaxation length at a load, m. */
    public double longitudinalRelaxationM(double loadN) { return 0.5 * lateralRelaxationM(loadN); }

    // ----- friction -----

    /** The kind of ground a surface is for the tread. */
    public static Ground ground(GroundContact.Surface surface) {
        String name = surface.name();
        if (surface == GroundContact.Surface.ICE || name.contains("ice")) { return Ground.ICE; }
        if (surface == GroundContact.Surface.SNOW || name.contains("snow")) { return Ground.SNOW; }
        if (name.startsWith("wet")) { return Ground.WET; }
        if (surface.soil() != null || surface == GroundContact.Surface.GRAVEL || name.contains("gravel") || name.contains("mud") || name.contains("sand")
                || name.contains("soil") || name.contains("dirt")) { return Ground.LOOSE; }
        return Ground.DRY;
    }

    /** Lock-to-peak friction ratio of a surface (rule from Wong's table, TPT and Balkwill). */
    public static double slidingRatio(GroundContact.Surface surface) {
        return switch (surface.name()) {
            case "tarmac" -> 0.72;
            case "wet tarmac" -> 0.80;
            case "gravel" -> 0.92;
            case "sand", "soil" -> 0.95;
            case "mud" -> 0.85;
            case "snow" -> 0.75;
            case "ice" -> 0.70;
            default -> 0.80;
        };
    }

    /** Horne's hydroplaning speed of the tyre at its pressure with its tread's drainage, m/s. */
    public double hydroplaningSpeedMps() { return 0.0558 * Math.sqrt(spec.pressurePa()) * spec.tread().drainage(); }

    /** Peak friction coefficient on a surface under a load at a speed (m/s). */
    public double peakFriction(GroundContact.Surface surface, double loadN, double speedMps) {
        Ground g = ground(surface);
        double mu = surface.grip().d() * spec.tread().grip(g) * spec.compoundGrip();
        // load sensitivity: -14 % per doubling
        mu *= Math.clamp(1 - 0.14 * Math.log(Math.max(loadN, 1e-3) / nominal) / Math.log(2), 0.6, 1.3);
        if (g == Ground.WET) {
            double vp = hydroplaningSpeedMps(), x = (Math.abs(speedMps) / vp - 0.7) / 0.3;
            if (x > 0) { mu *= Math.max(1 - 0.95 * Math.min(x, 1), 0.05); }
        }
        return mu;
    }

    /**
     * Steady forces at transient slips s_x (longitudinal: the slip ratio, driving positive) and s_y (lateral: minus the
     * tangent of the slip angle, so a positive slip pushes to the left), camber gamma (rad, positive pushes left),
     * on a surface under a load at a speed.
     */
    public Forces forces(GroundContact.Surface surface, double loadN, double sx, double sy, double camberRad, double speedMps) {
        Curve c = curve(surface, loadN, speedMps);
        double[] f = c.forces(sx, sy + 0.12 * camberRad);
        double nx = sx / c.peakX, ny = (sy + 0.12 * camberRad) / c.peakY, n = Math.hypot(nx, ny);
        double trail = contactLengthM(loadN) / 6 * Math.max(0, 1 - n / 1.5);
        return new Forces(f[0], f[1], -trail * f[1]);
    }

    /** The force curve at a load and speed: friction, shape, stiffnesses and peak slips. */
    Curve curve(GroundContact.Surface surface, double loadN, double speedMps) {
        double fz = Math.max(loadN, 0), mu = peakFriction(surface, fz, speedMps);
        double kx = Math.max(slipStiffness(fz), 1e-6), ky = Math.max(corneringStiffness(fz), 1e-6);
        return new Curve(fz, mu, slidingRatio(surface), kx, ky);
    }

    /**
     * Pacejka's curve sin(C atan(B s)) on normalised slips: peak at n = 1 where C atan(y_p) = pi / 2, so the peak
     * slips are y_p / B with B = K / (C mu F_z). C is solved so that the locked wheel (slip ratio 1) keeps the
     * surface's sliding ratio of the peak: low-friction surfaces peak at small slips, so lock lies further out.
     */
    static final class Curve {
        final double load, mu, c, yPeak, peakX, peakY, kx, ky;

        Curve(double load, double mu, double sliding, double kx, double ky) {
            this.load = load; this.mu = mu; this.kx = kx; this.ky = ky;
            double muFz = Math.max(mu * load, 1e-9), r = Math.clamp(sliding, 0.3, 0.999), shape = 2 - 2 / Math.PI * Math.asin(r);
            for (int i = 0; i < 8; i++) {
                double y = Math.tan(Math.PI / (2 * shape)), lock = kx / (y * shape * muFz);
                shape = Math.clamp((Math.PI - Math.asin(r)) / Math.atan(y * lock), 1.01, 1.99);
            }
            c = shape;
            yPeak = Math.tan(Math.PI / (2 * c));
            peakX = yPeak * c * muFz / kx;
            peakY = yPeak * c * muFz / ky;
        }

        /** g(n) = sin(C atan(y_p n)) / n (the curve over the normalised slip), and its derivative. */
        private double over(double n) { return n < 1e-9 ? c * yPeak : Math.sin(c * Math.atan(yPeak * n)) / n; }
        private double overSlope(double n) {
            if (n < 1e-6) { return 0; }
            double g = Math.sin(c * Math.atan(yPeak * n)), dg = Math.cos(c * Math.atan(yPeak * n)) * c * yPeak / (1 + yPeak * yPeak * n * n);
            return (dg * n - g) / (n * n);
        }

        /** {F_x, F_y} at slips (sx, sy). */
        double[] forces(double sx, double sy) {
            double nx = sx / peakX, ny = sy / peakY, n = Math.hypot(nx, ny), g = over(n) * mu * load;
            return new double[]{g * nx, g * ny};
        }

        /** dF_x/ds_x and dF_y/ds_y at slips (sx, sy), the other held. */
        double[] slopes(double sx, double sy) {
            double nx = sx / peakX, ny = sy / peakY, n = Math.hypot(nx, ny), g = over(n), dg = overSlope(n), scale = mu * load;
            double x = n < 1e-9 ? g : g + dg * nx * nx / n, y = n < 1e-9 ? g : g + dg * ny * ny / n;
            return new double[]{scale * x / peakX, scale * y / peakY};
        }
    }

    // ----- rolling resistance and mass -----

    /**
     * Rolling resistance (N) on a surface under a load at a speed: SAE J2452 with Michelin's P205/65R15 coefficients,
     * the tread's factor and the surface's firm resistance over tarmac's; Bekker compaction added on soft soil.
     */
    public double rollingResistanceN(GroundContact.Surface surface, double loadN, double speedMps) {
        if (!(loadN > 0)) { return 0; }
        double v = Math.abs(speedMps), kpa = spec.pressurePa() / 1000;
        double firm = Math.pow(kpa, -0.3947) * Math.pow(loadN, 0.9468) * (0.113 + 1.89e-4 * v + 2.24e-6 * v * v);
        double resistance = firm * spec.tread().rolling() * surface.rollingResistance() / GroundContact.Surface.TARMAC.rollingResistance()
                * (spec.construction() == Construction.BIAS ? 1.2 : 1);
        if (surface.soil() != null) {
            resistance += GroundContact.rigidWheel(surface.soil(), loadN, 2 * radius, spec.sectionWidthM()).compactionN();
        }
        return resistance;
    }

    /** Mass of the tyre (rubber, cords, beads): 240 kg per cubic metre of its annulus (rule; 9.1 kg for 205/55R16, 24 kg for LT265/75R16, 5.3 kg for a 6.00-6). */
    public double massKg() {
        double d = 2 * radius, rim = spec.rimDiameterM();
        return 240 * Math.PI / 4 * (d * d - rim * rim) * spec.sectionWidthM();
    }

    /** Moment of inertia about the axle: its mass at 0.85 of the outside radius (the tread and belts are outermost), kg m^2. */
    public double inertiaKgM2() { return massKg() * Math.pow(0.85 * radius, 2); }
}
