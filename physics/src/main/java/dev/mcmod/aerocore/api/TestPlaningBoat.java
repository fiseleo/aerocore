package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;

/**
 * Demonstrator planing boat: Fridsma's (1969) 20 deg deadrise, L/b 5 model scaled to a 1.2 m chine beam (6 m long,
 * 0.75 m deep) and loaded like his configuration A (C_Delta = Delta / (rho g b^3) = 0.608, so 1048 kg; LCG 59 % of the
 * length aft of the bow, VCG 0.294 b, pitch gyradius 25.1 % L), so PlaningDynamicsTest's tank comparison applies to
 * it by Froude scaling (friction scales with Reynolds number). Driven by a 0.35 m B4-70 propeller of pitch ratio 1.4
 * below the transom and a spade rudder behind it. A test design, not a production boat; the roll and yaw inertias take
 * the hull's bounding box as uniformly filled, an estimate.
 */
public final class TestPlaningBoat {
    public static final double BEAM = 1.2, LENGTH = 5 * BEAM, DEPTH = 0.625 * BEAM, DEADRISE = Math.toRadians(20);
    /** Fridsma's light loading C_Delta 0.608 in fresh water. */
    public static final double MASS = 0.608 * FluidState.FRESH_WATER_20C.densityKgPerM3() * BEAM * BEAM * BEAM;
    public static final PlaningHull HULL = PlaningHull.fridsma(LENGTH, BEAM, DEADRISE, DEPTH, 48);
    public static final Vec3d CENTER_OF_MASS = new Vec3d(LENGTH * (1 - 0.59), 0.294 * BEAM, 0);
    /** B4-70, P/D 1.4: the regression ahead, MARIN's measured four-quadrant series (BSeriesFourQuadrant) elsewhere. */
    public static final WageningenB PROPELLER_SERIES = new WageningenB(4, 0.70, 1.4);
    public static final Propeller PROPELLER = new Propeller(new Vec3d(-0.25, -0.2, 0), Vec3d.X, 0.35,
            PROPELLER_SERIES.fourQuadrant(BSeriesFourQuadrant.B4_70_PITCH_1_4, Math.toRadians(15), 361));
    public static final Rudder RUDDER = new Rudder(new Vec3d(-0.5, -0.22, 0), 0.22, 0.32, Rudder.lowAspectPolar(0.22, 0.32));
    public static final double MAX_PROPELLER_RPS = 45;
    private TestPlaningBoat() {}

    public static RigidBodyProperties properties() {
        double pitch = MASS * Math.pow(0.251 * LENGTH, 2), roll = MASS * (BEAM * BEAM + DEPTH * DEPTH) / 12;
        return new RigidBodyProperties(MASS, CENTER_OF_MASS, new InertiaTensor(roll, pitch, pitch, 0, 0, 0));
    }

    public static RigidBodyState atModelOrigin(Vec3d origin, Orientation q) {
        return new RigidBodyState(origin.add(q.rotate(CENTER_OF_MASS)), Vec3d.ZERO, q, Vec3d.ZERO);
    }

    /** Spheres along the keel, both chines and both deck edges at every fourth station, plus propeller and rudder. */
    public static List<CollisionSphere> collisionShape() {
        List<CollisionSphere> shape = new ArrayList<>();
        SlenderHull hull = HULL.hull();
        for (int i = 0; i < hull.stations(); i += 4) {
            double x = hull.sections().get(i).xM(), b = HULL.chineHalfBeam(i), keel = HULL.keelY(i);
            double r = Math.max(0.04, Math.min(0.12, b / 3)), chineY = Math.min(keel + b * Math.tan(DEADRISE), DEPTH - r);
            shape.add(new CollisionSphere(new Vec3d(x, keel + r, 0), r));
            for (int side = -1; side <= 1; side += 2) {
                shape.add(new CollisionSphere(new Vec3d(x, chineY + r * 0.5, side * Math.max(0, b - r)), r));
                shape.add(new CollisionSphere(new Vec3d(x, DEPTH - r, side * Math.max(0, b - r)), r));
            }
        }
        shape.add(new CollisionSphere(PROPELLER.positionLocal(), PROPELLER.diameterM() / 2));
        shape.add(new CollisionSphere(RUDDER.positionLocal(), RUDDER.spanM() / 2));
        return List.copyOf(shape);
    }
}
