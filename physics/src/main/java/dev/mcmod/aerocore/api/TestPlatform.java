package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;

/** Shared demonstrator geometry. The frame is 300 kg, the movable cubical ballast is 200 kg. */
public final class TestPlatform {
    public static final double FRAME_MASS = 300, BALLAST_MASS = 200, FLOAT_RADIUS = 0.45;
    public static final Vec3d DEFAULT_BALLAST = new Vec3d(0,-0.35,0);
    public static final List<BuoyantSphere> FLOATS = List.of(
            new BuoyantSphere(new Vec3d(-0.85,0,-0.85),FLOAT_RADIUS),
            new BuoyantSphere(new Vec3d(0.85,0,-0.85),FLOAT_RADIUS),
            new BuoyantSphere(new Vec3d(-0.85,0,0.85),FLOAT_RADIUS),
            new BuoyantSphere(new Vec3d(0.85,0,0.85),FLOAT_RADIUS));
    /** The steel frame on blocks taken as rock (ContactMaterial.STEEL_ON_ROCK). */
    public static final ContactMaterial CONTACT_MATERIAL = ContactMaterial.STEEL_ON_ROCK;
    /**
     * Frame and ballast as plain carbon steel at 300 K (k 60.5 W/m/K, rho 7854 kg/m^3, c_p 434 J/kg/K; Incropera and
     * DeWitt Table A.1, via Cengel and Ghajar Table A-3) on ground taken as Barre granite (k 2.79, rho 2630, c_p 775;
     * Incropera Table A.3). Used for the frictional heat partition and the lumped heat capacity.
     */
    public static final double STEEL_SPECIFIC_HEAT = 434;
    public static final double FRAME_EFFUSIVITY = HeatTransfer.effusivity(60.5,7854,STEEL_SPECIFIC_HEAT),
            GROUND_EFFUSIVITY = HeatTransfer.effusivity(2.79,2630,775);
    /** Lumped heat capacity of frame and ballast, J/K. */
    public static final double HEAT_CAPACITY = (FRAME_MASS+BALLAST_MASS)*STEEL_SPECIFIC_HEAT;
    /** The demonstrator screw's B-series designation: three blades, AE/A0 = 0.65, P/D = 1.0 (B3-65), a measured four-quadrant set. */
    public static final WageningenB PROPELLER_SERIES = new WageningenB(3,0.65,1.0);
    /**
     * Section for the BEMT outside the regression's quadrant: lift slope 2 pi, stall 12 deg, CD0 0.02. Chosen as the
     * best constant-chord fit to B3-50 over J = 0..0.9 (rms 0.021 in K_T and 10 K_Q); not measured section data.
     */
    public static final AirfoilPolar PROPELLER_SECTION = new SymmetricAirfoil(2*Math.PI,Math.toRadians(12),Math.toRadians(10),0.02,1e12,1);
    /**
     * 0.3 m B3-65 stern thruster pushing along local +X. Ahead quadrant (J from 0 to zero thrust): the Wageningen
     * regression; other quadrants: MARIN's measured four-quadrant series (BSeriesFourQuadrant, Roddy et al. 2006),
     * joined by 15 degree blending ramps. Tabulated once at 1 degree spacing; its mass is part of the frame.
     */
    public static final Propeller PROPELLER = new Propeller(new Vec3d(-1.4,-0.4,0),Vec3d.X,0.3,
            PROPELLER_SERIES.fourQuadrant(BSeriesFourQuadrant.B3_65_PITCH_1_0,Math.toRadians(15),361));
    private static final double CORNER_RADIUS = 0.05;
    private TestPlatform() {}
    public static RigidBodyProperties properties(Vec3d ballast) {
        double mass = FRAME_MASS+BALLAST_MASS;
        Vec3d center = ballast.scale(BALLAST_MASS/mass);
        InertiaTensor frame = InertiaTensor.box(FRAME_MASS,new Vec3d(2.4,0.3,2.4)).shifted(FRAME_MASS,center);
        InertiaTensor weight = InertiaTensor.box(BALLAST_MASS,new Vec3d(0.4,0.4,0.4)).shifted(BALLAST_MASS,ballast.subtract(center));
        return new RigidBodyProperties(mass,center,frame.add(weight));
    }
    /**
     * Contact spheres: the four floats, inset spheres at the deck (y 0.05..0.25) and ballast-cube corners,
     * the ballast cube's inscribed sphere and the propeller disk's enclosing sphere. The strut is ignored.
     */
    public static List<CollisionSphere> collisionShape(Vec3d ballast) {
        List<CollisionSphere> shape = new ArrayList<>();
        for (BuoyantSphere f : FLOATS) { shape.add(new CollisionSphere(f.offset(),f.radiusM())); }
        double deck = 1.2-CORNER_RADIUS, edge = 0.2-CORNER_RADIUS;
        for (int x=-1;x<=1;x+=2) for (int z=-1;z<=1;z+=2) {
            for (double y : new double[]{0.05+CORNER_RADIUS,0.25-CORNER_RADIUS}) {
                shape.add(new CollisionSphere(new Vec3d(x*deck,y,z*deck),CORNER_RADIUS));
            }
            for (int y=-1;y<=1;y+=2) {
                shape.add(new CollisionSphere(ballast.add(new Vec3d(x*edge,y*edge,z*edge)),CORNER_RADIUS));
            }
        }
        shape.add(new CollisionSphere(ballast,0.2));
        shape.add(new CollisionSphere(PROPELLER.positionLocal(),PROPELLER.diameterM()/2));
        return List.copyOf(shape);
    }
    public static RigidBodyState atModelOrigin(Vec3d origin, Orientation q, RigidBodyProperties properties) {
        return new RigidBodyState(origin.add(q.rotate(properties.centerOfMassLocal())),Vec3d.ZERO,q,Vec3d.ZERO);
    }
    /** Still water at waterY: exact spherical-cap immersion. */
    public static FloatingBodyDynamics.Environment flatWater(double waterY) {
        FluidState air = Atmosphere.atAltitude(0,Vec3d.ZERO).fluid();
        return (center,radius) -> {
            Immersion immersion = HydroSolver.sphereImmersion(new BuoyantSphere(Vec3d.ZERO,radius),waterY-center.y(),Vec3d.Y);
            return new FloatingBodyDynamics.Contact(immersion.fraction(),immersion.centerOfBuoyancy(),air,FluidState.FRESH_WATER_20C);
        };
    }
    /**
     * Still level waterY plus a prescribed wave field at time t, in open water. Uses 15 x 15 column quadrature
     * with the depth-attenuated linear wave pressure, plus orbital velocity and acceleration at the wet centroid.
     */
    public static FloatingBodyDynamics.Environment wavyWater(double waterY, WaveField waves, double time) {
        return wavyWater(waterY, waves, time, false);
    }
    /**
     * As above; with surfaceDiffraction, floats that pierce the surface leave their vertical diffraction to
     * WaveRadiation.Memory.diffractionLoads (the exact hemisphere excitation) instead of the Morison term.
     */
    public static FloatingBodyDynamics.Environment wavyWater(double waterY, WaveField waves, double time, boolean surfaceDiffraction) {
        if (waves.isEmpty()) { return flatWater(waterY); }
        FluidState air = Atmosphere.atAltitude(0,Vec3d.ZERO).fluid();
        return (center,radius) -> {
            WaveImmersion w = HydroSolver.sphereInWaves(center,radius,waves,waterY,time,15);
            return new FloatingBodyDynamics.Contact(w.fraction(),w.wetCentroidOffset(),air,FluidState.FRESH_WATER_20C.withVelocity(w.velocity()),
                    w.pressureFraction(),w.pressureCentroidOffset(),w.acceleration(),surfaceDiffraction && w.fraction()>0 && w.fraction()<1);
        };
    }
}
