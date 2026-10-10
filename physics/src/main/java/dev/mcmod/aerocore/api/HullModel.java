package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Hydrodynamic coefficients of one slender hull at its design condition (upright, level, at the waterline that floats
 * the given mass, or fully submerged), built once and used by HullDynamics:
 * <ul>
 * <li>calm-water resistance (1 + k) C_F + C_W: ITTC-1957 friction, Holtrop-Mennen (surface) or Hoerner (submerged)
 *     form factor, and a Michell wave-resistance table over Froude number (surface hulls only);</li>
 * <li>surge added mass: Söding (surface) or Lamb's prolate spheroid (submerged);</li>
 * <li>a lift element: the lateral (and, submerged, vertical) force -|u| M_L w at the trailing station x_T, which with
 *     the ideal-fluid (Kirchhoff, Munk-moment) terms of the strip added mass reproduces Y_v and N_v of Clarke et al.
 *     (1983) for surface hulls, or of the DARPA SUBOFF bare hull scaled with (D/L)^2 for submerged bodies.</li>
 * </ul>
 * Displacement-hull scope: planing (volumetric Froude number above about 1.2), sinkage, trim-dependent resistance,
 * wave breaking, shallow water and free-surface effects on submerged bodies are not modelled.
 */
public final class HullModel {
    public enum Type { SURFACE, SUBMERGED }
    /** SUBOFF bare hull (Roddy 1990 via Cao et al. 2016): Y'_v / (D/L)^2 and lift station aft of the centre of buoyancy, in L. */
    public static final double SUBMERGED_LIFT_SLOPE = 5.948e-3 / Math.pow(0.508 / 4.356, 2), SUBMERGED_LIFT_STATION = -0.51;
    /** Michell table: Fn 0.10..0.60 every 0.02 (humps and hollows), then 0.65..2.0 every 0.05. */
    private static final double[] FROUDE = froudeGrid();
    private static final double FROUDE_MIN = FROUDE[0], FROUDE_MAX = FROUDE[FROUDE.length - 1];
    private static final int FROUDE_POINTS = FROUDE.length;

    private final SlenderHull hull;
    private final Type type;
    private final SlenderHull.Particulars design;
    private final double density, kinematicViscosity, formFactor, surgeAddedMass, liftMass, liftStationAhead, liftStationAstern;
    private final double[] waveResistance;
    private final double designVolume, diameter, heaveCrossFlow;

    /**
     * @param massKg displacement used for the design waterline (surface) or ignored (submerged, fully wet)
     * @param water design water (density and viscosity)
     * @param sternShape Holtrop's C_stern: -10 V-shaped, 0 normal, +10 U-shaped sections
     */
    public HullModel(SlenderHull hull, Type type, double massKg, FluidState water, double sternShape) {
        this.hull = Objects.requireNonNull(hull, "hull");
        this.type = Objects.requireNonNull(type, "type");
        density = water.densityKgPerM3();
        kinematicViscosity = water.dynamicViscosityPaS() / density;
        double top = Double.NEGATIVE_INFINITY;
        for (HullSection s : hull.sections()) { top = Math.max(top, s.maxY()); }
        double waterline = type == Type.SURFACE ? hull.waterlineFor(massKg / density) : top + 1;
        design = hull.particulars(waterline);
        designVolume = design.volumeM3();
        double maxArea = 0;
        for (HullSection s : hull.sections()) { maxArea = Math.max(maxArea, s.area()); }
        diameter = 2 * Math.sqrt(maxArea / Math.PI);
        double length = design.lengthM();
        heaveCrossFlow = type == Type.SURFACE ? midshipHeaveDrag(hull, waterline) : 0;
        // Strip-theory sway added mass at the design condition, as HullDynamics computes it.
        double sway = 0;
        for (int i = 0; i < hull.stations(); i++) {
            HullSection s = hull.sections().get(i);
            sway += sectionAddedMass(s, s.clip(1, 0, waterline), density)[0] * hull.stripLength(i);
        }
        if (type == Type.SURFACE) {
            formFactor = HullResistance.holtropFormFactor(length, design.beamM(), design.draftM(), design.prismaticCoefficient(), design.lcbPercent(), sternShape);
            surgeAddedMass = HullManoeuvring.sodingSurgeAddedMass(designVolume, length, density);
            HullManoeuvring.Clarke clarke = HullManoeuvring.Clarke.of(length, design.beamM(), design.draftM(), design.blockCoefficient());
            liftMass = -0.5 * density * length * length * clarke.yv();
            // Clarke's derivatives are about amidships, and our yaw (about +y) has the opposite sign:
            // N_v(ours) = -N_v(Clarke) = (m22 - m11) U [Munk couple] + U M_L (x_T - x_mid) [lift element].
            double target = -0.5 * density * Math.pow(length, 3) * clarke.nv(), midship = 0.5 * (design.xForwardM() + design.xAftM());
            liftStationAhead = midship + (target - (sway - surgeAddedMass)) / liftMass;
        } else {
            formFactor = HullResistance.hoernerFormFactor(diameter / length);
            surgeAddedMass = HullManoeuvring.spheroidCoefficients(Math.max(1.01, length / diameter))[0] * density * designVolume;
            liftMass = 0.5 * density * length * length * SUBMERGED_LIFT_SLOPE * Math.pow(diameter / length, 2);
            liftStationAhead = design.centreOfBuoyancyX() + SUBMERGED_LIFT_STATION * length;
        }
        // Astern, the trailing station is the mirror image about the waterline's mid-length.
        liftStationAstern = design.xForwardM() + design.xAftM() - liftStationAhead;
        waveResistance = new double[FROUDE_POINTS];
        if (type == Type.SURFACE) {
            for (int i = 0; i < FROUDE_POINTS; i++) {
                double froude = froude(i), speed = froude * Math.sqrt(Atmosphere.GRAVITY_MPS2 * length);
                double r = HullResistance.michell((x, depth) -> hull.halfBreadth(x, depth, waterline), design.xAftM(), design.xForwardM(),
                        design.draftM(), 60, 12, speed, density, Atmosphere.GRAVITY_MPS2, 30);
                waveResistance[i] = r / (0.5 * density * speed * speed * design.wettedAreaM2());
            }
        }
    }

    public SlenderHull hull() { return hull; }
    public Type type() { return type; }
    public SlenderHull.Particulars design() { return design; }
    public double formFactor() { return formFactor; }
    public double surgeAddedMass() { return surgeAddedMass; }
    /** Lift-element mass per unit speed M_L (kg/m): lateral force -|u| M_L w at the trailing station. */
    public double liftMass() { return liftMass; }
    /** Trailing station x_T (model-local x) for forward (ahead) or astern motion. */
    public double liftStation(boolean ahead) { return ahead ? liftStationAhead : liftStationAstern; }
    public double kinematicViscosity() { return kinematicViscosity; }
    public double designVolume() { return designVolume; }
    /**
     * Heave cross-flow drag coefficient of the surface hull: the largest section at the design waterline mirrored in
     * the surface (a 2T x B double body) as a rounded rectangle of DNV-RP-C205 Table E-1 no. 4, its corner radius from
     * the area it lacks against B T: B T - A = 2 (1 - pi/4) R^2.
     */
    public double heaveCrossFlow() { return heaveCrossFlow; }
    private static double midshipHeaveDrag(SlenderHull hull, double waterline) {
        HullSection.Wet best = HullSection.Wet.DRY;
        for (HullSection s : hull.sections()) { HullSection.Wet w = s.clip(1, 0, waterline); if (w.areaM2() > best.areaM2()) { best = w; } }
        double b = best.waterlineBreadthM(), t = best.draftM();
        if (!(b > 0) || !(t > 0)) { return HullManoeuvring.roundedRectangleDrag(1, 0); }
        double radius = Math.min(Math.min(b / 2, t), Math.sqrt(Math.max(0, b * t - best.areaM2()) / (2 * (1 - Math.PI / 4))));
        return HullManoeuvring.roundedRectangleDrag(2 * t / b, radius / b);
    }
    /** Equivalent diameter of the largest section, for submerged cross-flow Reynolds numbers. */
    public double diameter() { return diameter; }

    /** Michell wave-resistance coefficient C_W = R_W / (1/2 rho U^2 S) at Froude number U / sqrt(g L); zero when submerged. */
    public double waveResistanceCoefficient(double froude) {
        if (type != Type.SURFACE || !(froude > 0)) { return 0; }
        // Below the first table point (Fn 0.1, where C_W is tiny) it fades linearly to zero; above Fn 2 it is held.
        if (froude < FROUDE_MIN) { return waveResistance[0] * froude / FROUDE_MIN; }
        double f = Math.min(froude, FROUDE_MAX);
        int i = 1;
        while (i < FROUDE_POINTS - 1 && FROUDE[i] < f) { i++; }
        double t = (f - FROUDE[i - 1]) / (FROUDE[i] - FROUDE[i - 1]);
        return (1 - t) * waveResistance[i - 1] + t * waveResistance[i];
    }
    /** Total calm-water resistance coefficient on wetted area at relative speed U and waterline length L. */
    public double resistanceCoefficient(double speed) {
        double length = design.lengthM(), reynolds = Math.abs(speed) * length / kinematicViscosity;
        return formFactor * HullResistance.ittc1957(reynolds) + waveResistanceCoefficient(Math.abs(speed) / Math.sqrt(Atmosphere.GRAVITY_MPS2 * length));
    }

    /**
     * Sectional {sway m22, heave m33} added mass per metre for a wet section: Lewis form while it pierces the
     * surface, the circumscribed ellipse (rho pi c^2, rho pi b^2) once fully submerged, zero when dry.
     */
    static double[] sectionAddedMass(HullSection section, HullSection.Wet wet, double density) {
        if (wet.areaM2() <= 0) { return new double[]{0, 0}; }
        if (wet.piercing()) { return HullManoeuvring.lewisAddedMass(wet.waterlineBreadthM(), wet.draftM(), wet.areaM2(), density); }
        double b = section.maxHalfBreadth(), c = 0.5 * (section.maxY() - section.minY());
        return new double[]{density * Math.PI * c * c, density * Math.PI * b * b};
    }

    /**
     * Contact spheres along the hull: at every other station, keel and both bilges at mid-depth plus both deck edges,
     * each sized to stay inside the section (radius a third of the smaller half-dimension, at least 3 cm).
     */
    public List<CollisionSphere> collisionShape() {
        List<CollisionSphere> shape = new ArrayList<>();
        for (int i = 0; i < hull.stations(); i += 2) {
            HullSection s = hull.sections().get(i);
            double b = s.maxHalfBreadth(), low = s.minY(), high = s.maxY(), r = Math.max(0.03, Math.min(b, 0.5 * (high - low)) / 3);
            double mid = 0.5 * (low + high);
            shape.add(new CollisionSphere(new Vec3d(s.xM(), low + r, 0), r));
            for (int side = -1; side <= 1; side += 2) {
                double zMid = Math.max(0, s.halfBreadthAt(mid) - r), zTop = Math.max(0, s.halfBreadthAt(high - r) - r);
                shape.add(new CollisionSphere(new Vec3d(s.xM(), mid, side * zMid), r));
                shape.add(new CollisionSphere(new Vec3d(s.xM(), high - r, side * zTop), r));
            }
        }
        return List.copyOf(shape);
    }

    private static double froude(int i) { return FROUDE[i]; }
    private static double[] froudeGrid() {
        double[] grid = new double[26 + 28];
        for (int i = 0; i < 26; i++) { grid[i] = 0.10 + 0.02 * i; }
        for (int i = 0; i < 28; i++) { grid[26 + i] = 0.65 + 0.05 * i; }
        return grid;
    }
}
