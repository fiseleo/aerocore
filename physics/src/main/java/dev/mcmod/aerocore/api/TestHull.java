package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;

/**
 * Demonstrator boat for the slender-hull model: a 6 m round-bilge launch (beam 1.4 m, deck at local y 0.45 m, keel at
 * -0.45 m amidships rising towards the bow, a nearly closed stern), 1200 kg with the COM over the centre of buoyancy and 0.2 m below the local
 * origin, a 0.4 m B3-65 propeller and a 0.30 x 0.50 m spade rudder. Example values for a game test, not a real design.
 * Its proportions stay inside the Holtrop-Mennen regression (L/B about 4.3) and the stern is nearly closed, because
 * Michell's integral treats a wide immersed transom as a strong wave source (transom resistance is not modelled).
 */
public final class TestHull {
    public static final double LENGTH = 6.0, HALF_BEAM = 0.7, DECK_Y = 0.45, KEEL_Y = -0.45, MASS = 1200;
    public static final SlenderHull HULL = geometry();
    /** Over the design centre of buoyancy, so the boat floats level, and 0.2 m below the local origin. */
    public static final Vec3d CENTER_OF_MASS = new Vec3d(
            HULL.particulars(HULL.waterlineFor(MASS / FluidState.FRESH_WATER_20C.densityKgPerM3())).centreOfBuoyancyX(), -0.2, 0);
    /** Same non-dimensional four-quadrant B3-65 data as the platform screw, at 0.4 m diameter. */
    public static final Propeller PROPELLER = new Propeller(new Vec3d(-3.15, -0.38, 0), Vec3d.X, 0.4, TestPlatform.PROPELLER.model());
    public static final Rudder RUDDER = new Rudder(new Vec3d(-3.45, -0.42, 0), 0.30, 0.50, Rudder.lowAspectPolar(0.30, 0.50));
    /** A steel hull: blocks act as rock (ContactMaterial.STEEL_ON_ROCK). */
    public static final ContactMaterial CONTACT_MATERIAL = ContactMaterial.STEEL_ON_ROCK;
    private static HullModel model;
    private TestHull() {}

    /** Hull coefficients for fresh water at 20 C; built once on first use (the Michell table takes about a second). */
    public static synchronized HullModel model() {
        if (model == null) { model = new HullModel(HULL, HullModel.Type.SURFACE, MASS, FluidState.FRESH_WATER_20C, 0); }
        return model;
    }

    public static RigidBodyProperties properties() {
        InertiaTensor inertia = InertiaTensor.box(MASS, new Vec3d(0.8 * LENGTH, 0.6 * (DECK_Y - KEEL_Y), 0.8 * 2 * HALF_BEAM));
        return new RigidBodyProperties(MASS, CENTER_OF_MASS, inertia);
    }

    /** Contact spheres: the hull's keel, bilge and gunwale spheres plus spheres around the propeller and rudder. */
    public static List<CollisionSphere> collisionShape() {
        List<CollisionSphere> shape = new ArrayList<>(model().collisionShape());
        shape.add(new CollisionSphere(PROPELLER.positionLocal(), PROPELLER.diameterM() / 2));
        shape.add(new CollisionSphere(RUDDER.positionLocal(), RUDDER.spanM() / 2));
        return List.copyOf(shape);
    }

    public static RigidBodyState atModelOrigin(Vec3d origin, Orientation q) {
        return new RigidBodyState(origin.add(q.rotate(CENTER_OF_MASS)), Vec3d.ZERO, q, Vec3d.ZERO);
    }

    /**
     * 25 stations: half-beam (1 - 0.97 |s|^2.5) aft (a 3 % transom, nearly a canoe stern) and (1 - s^2) forward of
     * amidships; the keel rises 0.35 s^2 towards the bow; bilge radius equal to the half-beam (round bilge).
     */
    private static SlenderHull geometry() {
        List<HullSection> sections = new ArrayList<>();
        int stations = 25;
        for (int i = 0; i < stations; i++) {
            double s = -1 + 2.0 * i / (stations - 1);
            double w = s < 0 ? 1 - 0.97 * Math.pow(-s, 2.5) : Math.max(0.03, 1 - s * s);
            double keel = KEEL_Y + (s > 0 ? 0.35 * s * s : 0);
            sections.add(HullSection.rounded(s * LENGTH / 2, HALF_BEAM * w, keel, DECK_Y, HALF_BEAM * w, 6));
        }
        return new SlenderHull(sections);
    }
}
