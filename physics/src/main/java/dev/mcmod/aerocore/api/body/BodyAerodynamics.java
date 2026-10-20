package dev.mcmod.aerocore.api.body;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.aero.Wing;

import java.util.List;
import java.util.Objects;

/**
 * Air loads on a body from its shape (body axes and origin as BodyShape).
 * <ul>
 * <li>Along the body (Allen and Perkins: the axial force goes with the axial speed squared):
 *   <ul>
 *   <li>skin friction on the wetted area (turbulent flat plate, Schlichting's fit with Raymer's compressibility factor)
 *     times Hoerner's form factor for streamlined bodies, 1 + 1.5 / f^1.5 + 7 / f^3 (Fluid-Dynamic Drag eq. 6-28);</li>
 *   <li>a blunt base: Hoerner's C_Db = k (d_b/d)^3 / sqrt(C_Df) on the frontal area (eq. 3-35, C_Df the forebody's
 *     friction and face drag), with k = 0.042 rather than his 0.029 from projectiles: fitted with the slant drag below to
 *     the Ahmed body's totals from a square back to 35 degrees (NASA TP-1999-206574 also finds 0.029 too low for bluff
 *     vehicles and fits 0.055);</li>
 *   <li>a blunt front: the face's share of the frontal area times a flat face's pressure drag, 0.8 with square edges
 *     falling to 0.02 as the edges round off (radius over diameter 0.3): the radius is as much as the first module grows
 *     from the face, within its length, halved for a straight chamfer;</li>
 *   <li>the tail: an upswept round tail Raymer's D/q = 3.83 u^2.5 A_max (eq. 12.36); a car's sloping rear (a cabin
 *     section last) Ahmed's slant: drag and lift against the slant angle, rising to 30 degrees and collapsing past it
 *     where the flow leaves the slant (Ahmed, Ramm and Faltin 1984, SAE 840300);</li>
 *   </ul></li>
 * <li>Along the air: what the shape does not show (door seams, open cockpits, a rotor hub, an undercarriage), given as a
 *   drag area on the full dynamic pressure (since 0.23 not along the body: tilted with it, a draggy fuselage's axial
 *   drag gave lift downward at a positive angle).</li>
 * <li>Across it, station by station: slender-body potential flow (the force on a strip is U d(m_a w)/dx, m_a the
 *   section's added mass per length for that direction, rho pi (b/2)^2 with b the breadth across the cross-flow), which
 *   gives a blunt base's lift and Munk's moment; and the viscous cross-flow, 1/2 rho eta C_dc b |w| w per length (Allen and
 *   Perkins NACA TR 1048), eta against fineness and C_dc against the cross-flow Mach and Reynolds numbers for round
 *   sections from Jorgensen (NASA TR R-474 figs. 1, 2, 4), 2.0 for square ones; times an onset factor of the local
 *   cross-flow angle, ((a - 4 deg) / 26 deg)^0.8 up to 1 at 30 degrees: the cross-flow separates only over the after body
 *   at small angles, and with the full Allen-Perkins term an ogive-cylinder's normal force came out 60 % above
 *   Jorgensen's at 10 degrees (fig. 19, N2C1). The cross-flow at each station includes the body's rotation, so it also
 *   damps turns.</li>
 * </ul>
 * <li>Supersonic pressure drag (0.28), from the stations' areas (each its equivalent circle, the area rule's view):
 *   rings turned to the flow at the tangent cone's pressure (Newtonian past the cone's detachment and on a flat nose
 *   face), rings turned away by Prandtl-Meyer expansion of the free stream, the base at its dead-air pressure
 *   (SurfacePressure); it replaces the subsonic face, tail and base drag from Mach 1.2. Between the critical Mach number
 *   (a rule of the fineness: 1 - 0.6 / f^0.7, 0.88 at fineness 10, 0.72 at 3) and Mach 1.2 the Mach 1.2 value comes in
 *   along a smooth step (the transonic drag rise), the subsonic terms going out.</li>
 * </ul>
 * Not modelled: vortex lift of the forebody at high angles beyond the cross-flow, a car's underbody and ground effect (0.34: an aero kit's, RoadAero),
 * interference with wings and engines (each part gives its own loads), shock-expansion on the afterbody (the
 * expansions start from the free stream, not the forebody's compressed flow), real-gas effects above Mach 6.
 */
public final class BodyAerodynamics {
    /** Loads about the body origin in body axes; drag, lift and side force against the air at the volume centroid. */
    public record Loads(Vec3d force, Vec3d moment, double dragN, double liftN, double sideN, double airspeedMps) {
        public static final Loads ZERO = new Loads(Vec3d.ZERO, Vec3d.ZERO, 0, 0, 0, 0);
    }

    /** Hoerner's base drag numerator, fitted to the Ahmed body (see the class description). */
    public static final double BASE_DRAG = 0.042;

    /** Jorgensen's eta against cylinder fineness (TR R-474 fig. 4). */
    private static final double[][] ETA = {{1, 0.53}, {2, 0.57}, {3, 0.62}, {5, 0.62}, {10, 0.68}, {20, 0.76}, {40, 0.84}, {1e6, 1}};
    /** A round section's subcritical C_dc against the cross-flow Mach number (TR R-474 fig. 1). */
    private static final double[][] CDC_MACH = {{0, 1.2}, {0.2, 1.2}, {0.4, 1.27}, {0.6, 1.47}, {0.8, 1.5}, {1.0, 2.0}, {1.2, 1.67}, {1.5, 1.5},
            {2, 1.42}, {3, 1.33}, {4, 1.29}, {4.8, 1.27}};
    /** The supercritical fall of a round section's C_dc against the cross-flow Reynolds number, as a factor (TR R-474 fig. 2). */
    private static final double[][] CDC_REYNOLDS = {{2.5e5, 1}, {4.5e5, 0.25}, {1e6, 0.5}, {1e7, 0.5}};
    /** A flat face's pressure drag against its edges' radius over the diameter (Hoerner ch. 3, the NASA van's corners). */
    private static final double[][] FACE = {{0, 0.8}, {0.02, 0.55}, {0.05, 0.35}, {0.1, 0.17}, {0.15, 0.08}, {0.2, 0.04}, {0.3, 0.02}, {10, 0.02}};
    /** The Ahmed body's slant: drag and lift increments on the frontal area against the slant angle (high-drag branch to 30 deg). */
    private static final double[][] SLANT_DRAG = {{0, 0}, {5, 0.012}, {10, 0.028}, {12.5, 0.03}, {15, 0.04}, {20, 0.07}, {25, 0.125}, {30, 0.228},
            {31, 0.12}, {35, 0.12}, {40, 0.125}, {90, 0.125}};
    private static final double[][] SLANT_LIFT = {{0, 0}, {10, 0.12}, {20, 0.26}, {25, 0.345}, {30, 0.36}, {31, 0.08}, {35, 0}, {90, 0}};

    private final BodyShape shape;
    private final BodyShape.Particulars p;
    private final double extraDragArea, eta, faceDrag, slantDrag, slantLift, slantX;
    private final boolean carRear;
    private final double[] x, y, width, height, squareness, area;
    /** Supersonic pressure drag area (m^2) at SUPERSONIC_MACHS, built on first need (warmUp). */
    private volatile double[] supersonic;
    static final double[] SUPERSONIC_MACHS = {1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2, 2.2, 2.5, 2.8, 3.2, 3.6, 4, 4.5, 5, 6, 7, 8, 10, 12, 15, 20};

    /** A body with extra drag area (m^2, at the free-stream dynamic pressure) for what its smooth shape leaves out. */
    public BodyAerodynamics(BodyShape shape, double extraDragAreaM2) {
        this.shape = Objects.requireNonNull(shape, "shape");
        Require.nonnegative(extraDragAreaM2, "Extra drag area");
        this.p = shape.particulars();
        this.extraDragArea = extraDragAreaM2;
        this.eta = crossflowEta(p.finenessRatio());
        List<BodyShape.Station> st = shape.stations();
        int n = st.size();
        x = new double[n]; y = new double[n]; width = new double[n]; height = new double[n]; squareness = new double[n]; area = new double[n];
        for (int i = 0; i < n; i++) {
            BodyShape.Station s = st.get(i);
            x[i] = s.xM(); y[i] = s.centroidYM(); width[i] = s.widthM(); height[i] = s.topM() - s.bottomM(); area[i] = s.areaM2();
            // how square the section is: pi/4 for an ellipse (0), 1 for a rectangle (1)
            double fill = width[i] * height[i] > 0 ? s.areaM2() / (width[i] * height[i]) : Math.PI / 4;
            squareness[i] = Math.clamp((fill - Math.PI / 4) / (1 - Math.PI / 4), 0, 1);
        }
        // the front: the nose face's share of the frontal area; its edges are rounded by as much as the first module grows
        // from the face (half that for a straight chamfer), no more than its length
        double a = Math.max(p.frontalAreaM2(), 1e-9), d = Math.sqrt(4 * a / Math.PI);
        BodyModule first = shape.modules().get(0);
        double grown = 1 - Math.sqrt(Math.min(p.noseAreaM2() / Math.max(BodyShape.area(first.rear().outline()), 1e-12), 1));
        double radius = Math.min(first.lengthM(), grown * 0.5 * d) * (first.fullness() >= 1.5 ? 1 : 0.5);
        faceDrag = Math.min(p.noseAreaM2() / a, 1) * table(FACE, radius / d);
        // the rear: a car's slant (a cabin section last), else a round tail's upsweep
        int last = shape.modules().size() - 1;
        carRear = shape.modules().get(last).rear() instanceof SectionShape.Cabin;
        double slant = Math.max(p.rearSlantDeg(), 0);
        slantDrag = carRear ? table(SLANT_DRAG, slant) : 0;
        slantLift = carRear ? table(SLANT_LIFT, slant) : 0;
        slantX = 0.5 * (shape.moduleFrontX(last) + shape.moduleRearX(last));
    }

    public BodyShape shape() { return shape; }

    /** Turbulent flat-plate skin friction at a Reynolds number (Schlichting) and Mach number (Raymer's factor). */
    public static double skinFriction(double reynolds, double mach) {
        double re = Math.max(reynolds, 1e4);
        return 0.455 / Math.pow(Math.log10(re), 2.58) / Math.pow(1 + 0.144 * mach * mach, 0.65);
    }

    /** Hoerner's form factor of a streamlined body of revolution against its fineness (length over diameter). */
    public static double formFactor(double fineness) {
        double f = Math.max(fineness, 1);
        return 1 + 1.5 / Math.pow(f, 1.5) + 7 / Math.pow(f, 3);
    }

    /** Base drag on the frontal area: BASE_DRAG (d_b/d)^3 / sqrt(forebody drag coefficient on the frontal area). */
    public static double baseDrag(double baseAreaRatio, double forebodyDragCoefficient) {
        double ratio = Math.clamp(baseAreaRatio, 0, 1);
        return BASE_DRAG * Math.pow(ratio, 1.5) / Math.sqrt(Math.max(forebodyDragCoefficient, 1e-4));
    }

    /** Raymer's (after Kroo) drag area of an upswept aft body: 3.83 u^2.5 A_max (u the upsweep, rad). */
    public static double upsweepDragArea(double upsweepRad, double frontalAreaM2) {
        return 3.83 * Math.pow(Math.max(upsweepRad, 0), 2.5) * frontalAreaM2;
    }

    /** Jorgensen's cross-flow factor of a finite cylinder against its fineness (to 1 for an infinite one). */
    public static double crossflowEta(double fineness) { return table(ETA, Math.max(fineness, 1)); }

    /**
     * Cross-flow drag coefficient of a section: round ones from Jorgensen against the cross-flow Mach number, falling
     * through the drag crisis with the cross-flow Reynolds number (below Mach 0.4 of cross-flow); square ones 2.0;
     * between by squareness 0..1.
     */
    public static double crossflowDrag(double machN, double reynoldsN, double squareness) {
        double round = table(CDC_MACH, Math.abs(machN));
        if (Math.abs(machN) < 0.4) {
            double crisis = table(CDC_REYNOLDS, Math.max(reynoldsN, 2e5), true);
            round *= crisis + (1 - crisis) * Math.clamp((Math.abs(machN) - 0.2) / 0.2, 0, 1);
        }
        return round + (2.0 - round) * Math.clamp(squareness, 0, 1);
    }

    /** The drag area (m^2) at zero angle, at a speed through the air. */
    public double zeroLiftDragAreaM2(double speedMps, Wing.Air air) {
        double v = Math.max(Math.abs(speedMps), 1e-3);
        double re = air.densityKgM3() * v * p.lengthM() / air.viscosityPaS(), mach = v / air.soundMps();
        double friction = skinFriction(re, mach) * formFactor(p.finenessRatio()) * p.wettedAreaM2();
        double a = Math.max(p.frontalAreaM2(), 1e-9);
        double base = baseDrag(p.baseAreaM2() / a, friction / a + faceDrag) * a;
        double tail = carRear ? slantDrag * a : upsweepDragArea(Math.toRadians(p.upsweepDeg()), a);
        double subsonic = base + faceDrag * a + tail, mcr = criticalMach();
        if (mach <= mcr) { return friction + subsonic + extraDragArea; }
        double t = Math.min(1, (mach - mcr) / (1.2 - mcr)), step = t * t * (3 - 2 * t);
        return friction + (1 - step) * subsonic + step * supersonicPressureDragAreaM2(Math.max(mach, 1.2)) + extraDragArea;
    }

    /** Where the transonic drag rise begins: a rule of the fineness, 1 - 0.6 / f^0.7 (0.6 to 0.95). */
    public double criticalMach() { return Math.clamp(1 - 0.6 / Math.pow(Math.max(p.finenessRatio(), 1), 0.7), 0.6, 0.95); }

    /**
     * The supersonic pressure drag area (m^2, on the free stream's dynamic pressure; friction apart) at a Mach number
     * (1.2 and up; the table's ends held): tangent cone, Newtonian, Prandtl-Meyer and base pressure on the stations' rings.
     */
    public double supersonicPressureDragAreaM2(double mach) {
        double[] table = supersonic;
        if (table == null) { warmUp(); table = supersonic; }
        double[] m = SUPERSONIC_MACHS;
        double v = Math.clamp(mach, m[0], m[m.length - 1]);
        int i = 0;
        while (i < m.length - 2 && v > m[i + 1]) { i++; }
        return table[i] + (v - m[i]) / (m[i + 1] - m[i]) * (table[i + 1] - table[i]);
    }

    /** The pressure drag area at one Mach number, from the rings (what the table holds). */
    double ringPressureDragAreaM2(double mach) {
        int n = x.length;
        // from the nose (largest x: x is forward) to the tail
        int first = x[0] >= x[n - 1] ? 0 : n - 1, last = n - 1 - first, dir = first == 0 ? 1 : -1;
        double sum = area[first] * dev.mcmod.aerocore.api.gasdynamics.SurfacePressure.stagnation(mach);   // a flat nose face
        for (int i = first + dir; i != last + dir; i += dir) {
            double ds = area[i] - area[i - dir], dx = x[i] - x[i - dir];
            if (ds == 0) { continue; }
            double dr = Math.sqrt(Math.max(area[i], 0) / Math.PI) - Math.sqrt(Math.max(area[i - dir], 0) / Math.PI);
            double theta = Math.atan2(Math.abs(dr), Math.abs(dx));
            sum += ds * (ds > 0 ? dev.mcmod.aerocore.api.gasdynamics.SurfacePressure.tangentCone(mach, theta)
                    : dev.mcmod.aerocore.api.gasdynamics.SurfacePressure.boattail(mach, theta));
        }
        return sum - dev.mcmod.aerocore.api.gasdynamics.SurfacePressure.base(mach) * area[last];
    }

    /** Builds the supersonic table (the first time a body needs it the cone table is built too, about 0.1 s): off the game thread. */
    public void warmUp() {
        if (supersonic != null) { return; }
        double[] t = new double[SUPERSONIC_MACHS.length];
        for (int i = 0; i < t.length; i++) { t[i] = ringPressureDragAreaM2(SUPERSONIC_MACHS[i]); }
        supersonic = t;
    }

    /** The lift area (m^2) of a car's slant (on the axial dynamic pressure, acting at the last module). */
    public double slantLiftAreaM2() { return slantLift * p.frontalAreaM2(); }

    /**
     * Loads from the body's velocity through the air (body axes, at the origin) and its rate of turn (body axes),
     * about the origin.
     */
    public Loads loads(Vec3d velocity, Vec3d omega, Wing.Air air) {
        double rho = air.densityKgM3();
        Vec3d c = new Vec3d(p.centroidXM(), p.centroidYM(), 0);
        Vec3d vc = velocity.add(omega.cross(c));
        double speed = vc.length();
        if (!(speed > 1e-6) || !(rho > 0)) { return Loads.ZERO; }
        // along the body on the axial dynamic pressure; the extra drag area (what the smooth shape leaves out) along the air
        double u = vc.x();
        Vec3d axial = new Vec3d(-0.5 * rho * (zeroLiftDragAreaM2(speed, air) - extraDragArea) * u * Math.abs(u), 0, 0).add(vc.scale(-0.5 * rho * extraDragArea * speed));
        Vec3d force = axial, moment = c.cross(axial);
        if (slantLift != 0 && u > 0) {
            Vec3d lift = new Vec3d(0, 0.5 * rho * u * u * slantLiftAreaM2(), 0), at = new Vec3d(slantX, shape.topAt(slantX), 0);
            force = force.add(lift); moment = moment.add(at.cross(lift));
        }
        // across it, station by station
        int n = x.length;
        double[] wy = new double[n], wz = new double[n], uu = new double[n];
        for (int i = 0; i < n; i++) {
            Vec3d r = new Vec3d(x[i], y[i], 0), v = velocity.add(omega.cross(r));
            uu[i] = v.x(); wy[i] = v.y(); wz[i] = v.z();
        }
        double nu = air.viscosityPaS() / rho;
        for (int i = 1; i < n; i++) {
            double dx = x[i] - x[i - 1];
            if (!(dx > 0)) { continue; }
            double um = 0.5 * (uu[i] + uu[i - 1]);
            // potential: U [m_a w] over the strip, m_a = rho pi (breadth across the flow / 2)^2
            double my0 = rho * Math.PI * 0.25 * width[i - 1] * width[i - 1], my1 = rho * Math.PI * 0.25 * width[i] * width[i];
            double mz0 = rho * Math.PI * 0.25 * height[i - 1] * height[i - 1], mz1 = rho * Math.PI * 0.25 * height[i] * height[i];
            double fy = um * (my1 * wy[i] - my0 * wy[i - 1]), fz = um * (mz1 * wz[i] - mz0 * wz[i - 1]);
            // viscous cross-flow
            double wym = 0.5 * (wy[i] + wy[i - 1]), wzm = 0.5 * (wz[i] + wz[i - 1]), w = Math.hypot(wym, wzm);
            if (w > 1e-9) {
                double bw = 0.5 * (width[i] + width[i - 1]), bh = 0.5 * (height[i] + height[i - 1]), d = 0.5 * (bw + bh);
                double cdc = crossflowDrag(w / air.soundMps(), w * d / nu, 0.5 * (squareness[i] + squareness[i - 1]));
            double onset = Math.pow(Math.clamp((Math.toDegrees(Math.atan2(w, Math.abs(um))) - 4) / 26, 0, 1), 0.8);
            cdc *= onset;
                fy -= 0.5 * rho * eta * cdc * bw * dx * wym * w;
                fz -= 0.5 * rho * eta * cdc * bh * dx * wzm * w;
            }
            Vec3d f = new Vec3d(0, fy, fz), at = new Vec3d(0.5 * (x[i] + x[i - 1]), 0.5 * (y[i] + y[i - 1]), 0);
            force = force.add(f);
            moment = moment.add(at.cross(f));
        }
        Vec3d dir = vc.scale(-1 / speed);   // the air's direction past the body
        double drag = force.dot(dir);
        Vec3d across = force.subtract(dir.scale(drag));
        // lift across the air in the plane of the body's y axis, side force likewise toward its z axis
        Vec3d liftDir = Vec3d.Y.subtract(dir.scale(dir.y())), sideDir = Vec3d.Z.subtract(dir.scale(dir.z()));
        double lift = liftDir.lengthSquared() > 1e-12 ? across.dot(liftDir.normalized()) : 0;
        double side = sideDir.lengthSquared() > 1e-12 ? across.dot(sideDir.normalized()) : 0;
        return new Loads(force, moment, drag, lift, side, speed);
    }

    private static double table(double[][] t, double v) { return table(t, v, false); }

    /** Linear interpolation in a table (in log10 of the argument when log), held at its ends. */
    private static double table(double[][] t, double v, boolean log) {
        if (v <= t[0][0]) { return t[0][1]; }
        for (int i = 1; i < t.length; i++) {
            if (v <= t[i][0]) {
                double a = log ? Math.log10(t[i - 1][0]) : t[i - 1][0], b = log ? Math.log10(t[i][0]) : t[i][0], x = log ? Math.log10(v) : v;
                return t[i - 1][1] + (x - a) / (b - a) * (t[i][1] - t[i - 1][1]);
            }
        }
        return t[t.length - 1][1];
    }
}
