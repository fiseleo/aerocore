package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AirfoilPolar;
import dev.mcmod.aerocore.api.ElectricMotor;
import dev.mcmod.aerocore.api.HullModel;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.PlaningHull;
import dev.mcmod.aerocore.api.PropellerModels;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.part.BoxPart;
import dev.mcmod.aerocore.api.vehicle.part.ControlSurfacePart;
import dev.mcmod.aerocore.api.vehicle.part.HullPart;
import dev.mcmod.aerocore.api.vehicle.part.MassPart;
import dev.mcmod.aerocore.api.vehicle.part.MotorPart;
import dev.mcmod.aerocore.api.vehicle.part.PlaningHullPart;
import dev.mcmod.aerocore.api.vehicle.part.PropellerPart;
import dev.mcmod.aerocore.api.vehicle.part.SpherePart;

import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * AeroCore's runtime parts behind minimal designs, for testing the vehicle framework (a vehicle mod has the real
 * designs). A fixture's key must name everything its part depends on: compiled parts are cached by design JSON.
 */
final class TestParts {
    private TestParts() {}

    /** A design whose part is built by a function. */
    record Fixture(String name, String key, Function<PartDesign, Part> build) implements PartDesign {
        @Override public String type() { return "test:fixture"; }
        @Override public List<DesignIssue> check() { return List.of(); }
        @Override public Part compile() { return build.apply(this); }
        @Override public Map<String, Object> parameters() { return Fields.object("key", key); }
    }

    static PartDesign sphere(String name, double radiusM, MassSpec mass) {
        MassProperties m = mass.resolve(t -> MassShapes.sphere(radiusM, t));
        return new Fixture(name, "sphere " + radiusM + " " + m, d -> new SpherePart(d, radiusM, m));
    }

    static PartDesign box(String name, Vec3d sizeM, double dragCoefficient, boolean closed, int cells, MassSpec mass) {
        MassProperties m = mass.resolve(t -> MassShapes.box(sizeM, t));
        return new Fixture(name, "box " + sizeM + " " + dragCoefficient + " " + closed + " " + cells + " " + m, d -> new BoxPart(d, sizeM, dragCoefficient, closed, cells, m));
    }

    /** A point mass with named mount points (for attachments). */
    static PartDesign host(String name, double massKg, Map<String, Mount> points) {
        MassProperties m = MassProperties.point(massKg, Vec3d.ZERO);
        return new Fixture(name, "host " + massKg + " " + points, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public Map<String, Mount> mountPoints() { return points; }
            @Override public void loads(PartStep step) {}
        });
    }

    /** A body of modules with a given mass spread as a thin shell's would be (a box's inertia of the body's size). */
    static PartDesign body(String name, dev.mcmod.aerocore.api.body.BodyShape shape, double massKg) {
        return body(name, shape, massKg, dev.mcmod.aerocore.api.vehicle.part.BodyPart.Hydro.AUTO);
    }

    /** The same with the water model chosen. */
    static PartDesign body(String name, dev.mcmod.aerocore.api.body.BodyShape shape, double massKg, dev.mcmod.aerocore.api.vehicle.part.BodyPart.Hydro hydro) {
        var p = shape.particulars();
        MassProperties solid = MassShapes.box(new Vec3d(p.lengthM(), p.maxHeightM(), p.maxWidthM()), 1e9);
        MassProperties m = solid.scaled(massKg / solid.massKg()).transformed(new Vec3d(p.centroidXM(), p.centroidYM(), 0), dev.mcmod.aerocore.api.Orientation.IDENTITY);
        return new Fixture(name, "body " + p + " " + massKg + " " + hydro, d -> new dev.mcmod.aerocore.api.vehicle.part.BodyPart(d, shape, 0,
                hydro, 6, m, Map.of(), List.of(), List.of()));
    }

    static PartDesign ballast(String name, double massKg) {
        return new Fixture(name, "ballast " + massKg, d -> new MassPart(d, MassProperties.point(massKg, Vec3d.ZERO), List.of()));
    }

    static PartDesign hull(String name, SlenderHull hull, double displacementKg, MassProperties mass) {
        return new Fixture(name, "hull " + hull.totalVolume() + " " + hull.stations() + " " + displacementKg + " " + mass,
                d -> new HullPart(d, hull, HullModel.Type.SURFACE, displacementKg, 0, 6, mass, List.of()));
    }

    static PartDesign planing(String name, double lengthM, double chineBeamM, double deadriseDeg, double depthM, int stations, double massKg) {
        PlaningHull planing = PlaningHull.fridsma(lengthM, chineBeamM, Math.toRadians(deadriseDeg), depthM, stations);
        MassProperties m = MassSpec.given(massKg).resolve(t -> MassShapes.hullShell(planing.hull(), 1e-3));
        return new Fixture(name, "planing " + lengthM + " " + chineBeamM + " " + deadriseDeg + " " + depthM + " " + stations + " " + massKg,
                d -> new PlaningHullPart(d, planing, chineBeamM, deadriseDeg, depthM, 6, m, List.of()));
    }

    static PartDesign motor(String name, double maxTorqueNm, double maxPowerW, double maxSpeedRpm, double rotorInertia, double efficiency, double massKg) {
        double side = Math.cbrt(massKg / 4000);
        MassProperties m = MassProperties.of(massKg, Vec3d.ZERO, InertiaTensor.box(massKg, new Vec3d(side, side, side)));
        return new Fixture(name, "motor " + maxTorqueNm + " " + maxPowerW + " " + maxSpeedRpm + " " + rotorInertia + " " + efficiency + " " + massKg,
                d -> new MotorPart(d, new ElectricMotor(maxTorqueNm, maxPowerW, maxSpeedRpm), rotorInertia, efficiency, m, List.of()));
    }

    /** A bronze propeller: blades as plates, a hub cylinder 0.3 D long (unit masses of a vehicle mod's propeller design). */
    static PartDesign propeller(String name, double diameterM, int blades, double areaRatio, double pitchRatio, double hubRatio, double thicknessRatio) {
        double chordRatio = PropellerModels.chordRatio(blades, areaRatio, hubRatio);
        double r = diameterM / 2, rh = hubRatio * r, chord = chordRatio * diameterM, hubLength = 0.3 * diameterM;
        double blade = blades * chord * (r - rh) * thicknessRatio * chord, hub = Math.PI * rh * rh * hubLength;
        double spin = blade * (r * r * r - rh * rh * rh) / (3 * (r - rh)) + 0.5 * hub * rh * rh;
        double across = 0.5 * blade * (r * r * r - rh * rh * rh) / (3 * (r - rh)) + hub * (3 * rh * rh + hubLength * hubLength) / 12;
        MassProperties unit = new MassProperties(blade + hub, Vec3d.ZERO, spin, across, across, 0, 0, 0);
        MassProperties m = MassSpec.solid(Material.BRONZE).resolve(t -> unit);
        return new Fixture(name, "propeller " + diameterM + " " + blades + " " + areaRatio + " " + pitchRatio + " " + hubRatio + " " + thicknessRatio,
                d -> new PropellerPart(d, diameterM, blades, pitchRatio, hubRatio, chordRatio,
                        () -> PropellerModels.fourQuadrant(blades, areaRatio, pitchRatio, hubRatio), m, List.of()));
    }

    /** A wheel on a shaft (no loads of its own), or one that drives with the torque input "torque" (N m). */
    static PartDesign flywheel(String name, double inertiaKgM2, boolean driver) {
        MassProperties m = new MassProperties(10, Vec3d.ZERO, inertiaKgM2, inertiaKgM2 / 2, inertiaKgM2 / 2, 0, 0, 0);
        return new Fixture(name, "flywheel " + inertiaKgM2 + " " + driver, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, inertiaKgM2, driver)); }
            @Override public List<String> inputs() { return driver ? List.of("torque") : List.of(); }
            @Override public void loads(PartStep step) { if (driver) { step.rotorTorque(0, step.input("torque", 0), 0, false); } }
        });
    }

    /**
     * A part with rotors (each 10 kg about its axis' centre) whose loads and after are given (a gearbox, an axle, a
     * brake for drivetrain tests); key must name its behaviour (compiled parts are cached by it).
     */
    static PartDesign gears(String name, String key, List<Part.Rotor> rotors, List<Part.Link> links, List<String> inputs, int internals, int shown,
                            java.util.function.Consumer<PartStep> loads, java.util.function.Consumer<PartAfter> after) {
        MassProperties m = new MassProperties(10, Vec3d.ZERO, 1, 1, 1, 0, 0, 0);
        return new Fixture(name, "gears " + key, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public List<Rotor> rotors() { return rotors; }
            @Override public List<Link> links() { return links; }
            @Override public List<String> inputs() { return inputs; }
            @Override public int internalStates() { return internals; }
            @Override public int shownValues() { return shown; }
            @Override public void loads(PartStep step) { loads.accept(step); }
            @Override public void after(PartAfter step) { after.accept(step); }
        });
    }

    /** A 205/55R16 road tyre at 230 kPa. */
    static dev.mcmod.aerocore.api.wheel.Tyre carTyre() {
        return new dev.mcmod.aerocore.api.wheel.Tyre(new dev.mcmod.aerocore.api.wheel.Tyre.Spec(dev.mcmod.aerocore.api.wheel.Tyre.Construction.RADIAL,
                0.205, 0.55, 16 * 0.0254, 230e3, 300e3, dev.mcmod.aerocore.api.wheel.Tyre.Tread.ROAD, 1));
    }

    /** A car brake: vented 288 mm disc, 1500 N m at full demand. */
    static dev.mcmod.aerocore.api.wheel.Brake carBrake() {
        return new dev.mcmod.aerocore.api.wheel.Brake(dev.mcmod.aerocore.api.wheel.Brake.Type.VENTED_DISC, 1500, 7, 0.288, 0.09);
    }

    /**
     * A car axle of 205/55R16 tyres, its hubs at y = 0 of the part at the design compression (0.12 of 0.2 m travel), with
     * a wheel rate, the preload that puts a corner load there, linear damping (bump, rebound), a layout, brake, drive,
     * steering and anti-roll bar; 100 kg at the hubs' height.
     */
    static PartDesign axle(String name, double trackM, double wheelRate, double cornerLoadN, double bump, double rebound,
                           dev.mcmod.aerocore.api.wheel.Suspension.Type type, boolean brakes, dev.mcmod.aerocore.api.vehicle.part.AxlePart.Drive drive,
                           dev.mcmod.aerocore.api.vehicle.part.AxlePart.Steering steering, double antiRoll, double handbrakeNm) {
        return axle(name, trackM, wheelRate, cornerLoadN, bump, rebound, type, brakes, drive, steering, antiRoll, handbrakeNm, null);
    }

    /** 0.33: the same moving on hardpoints (the right wheel's, in the part's frame: its hub at (0, 0, track / 2)); null: straight up. */
    static PartDesign axle(String name, double trackM, double wheelRate, double cornerLoadN, double bump, double rebound,
                           dev.mcmod.aerocore.api.wheel.Suspension.Type type, boolean brakes, dev.mcmod.aerocore.api.vehicle.part.AxlePart.Drive drive,
                           dev.mcmod.aerocore.api.vehicle.part.AxlePart.Steering steering, double antiRoll, double handbrakeNm,
                           dev.mcmod.aerocore.api.wheel.kinematics.Hardpoints hardpoints) {
        String key = "axle " + trackM + " " + wheelRate + " " + cornerLoadN + " " + bump + " " + rebound + " " + type + " " + brakes + " " + drive + " " + steering + " "
                + antiRoll + " " + handbrakeNm + (hardpoints == null ? "" : " " + hardpoints);
        MassProperties m = new MassProperties(100, Vec3d.ZERO, 100 * trackM * trackM / 12, 100 * trackM * trackM / 12, 5, 0, 0, 0);
        return new Fixture(name, key, d -> {
            dev.mcmod.aerocore.api.wheel.Tyre tyre = carTyre();
            double design = 0.12, travel = 0.2;
            dev.mcmod.aerocore.api.wheel.Suspension.Geometry g = dev.mcmod.aerocore.api.wheel.Suspension.Geometry.of(type, trackM, tyre.radiusM(), 0, -1, 0);
            dev.mcmod.aerocore.api.wheel.Suspension s = new dev.mcmod.aerocore.api.wheel.Suspension(g,
                    dev.mcmod.aerocore.api.wheel.Suspension.Spring.coil(wheelRate, Math.max(cornerLoadN - wheelRate * design, 0)),
                    dev.mcmod.aerocore.api.wheel.Suspension.Damper.linear(bump, rebound), travel, 0, design,
                    hardpoints == null ? null : dev.mcmod.aerocore.api.wheel.kinematics.KinematicTable.of(hardpoints, tyre.radiusM(), design, travel));
            dev.mcmod.aerocore.api.wheel.WheelCorner left = new dev.mcmod.aerocore.api.wheel.WheelCorner(new dev.mcmod.aerocore.api.wheel.WheelCorner.Spec(
                    new Vec3d(0, -design, -trackM / 2), -1, trackM, tyre, 0.8, s, brakes ? carBrake() : null, 0));
            dev.mcmod.aerocore.api.wheel.WheelCorner right = new dev.mcmod.aerocore.api.wheel.WheelCorner(new dev.mcmod.aerocore.api.wheel.WheelCorner.Spec(
                    new Vec3d(0, -design, trackM / 2), 1, trackM, tyre, 0.8, s, brakes ? carBrake() : null, 0));
            return new dev.mcmod.aerocore.api.vehicle.part.AxlePart(d, left, right, drive, steering, antiRoll, handbrakeNm, null, m, List.of());
        });
    }

    /** A rectangular wing of a NACA 2412 section (100 kg) with ailerons on the "aileron" role. */
    static PartDesign wing(String name, double spanM, double chordM) {
        dev.mcmod.aerocore.api.aero.Airfoil a = dev.mcmod.aerocore.api.aero.Airfoil.naca4(0.02, 0.4, 0.12);
        MassProperties m = new MassProperties(100, Vec3d.ZERO, 100 * spanM * spanM / 12, 100 * spanM * spanM / 12, 100 * chordM * chordM / 12, 0, 0, 0);
        return new Fixture(name, "wing " + spanM + " " + chordM, d -> new dev.mcmod.aerocore.api.vehicle.part.WingPart(d,
                new dev.mcmod.aerocore.api.aero.Wing.Planform(spanM, chordM, chordM, 0, 0, 0, 0, true), a, a,
                List.of(new dev.mcmod.aerocore.api.aero.Wing.ControlSurface("aileron", dev.mcmod.aerocore.api.aero.SectionPolar.FlapType.PLAIN, 0.25, 0.6, 0.95, 20, 20, true)),
                8, m, List.of()));
    }

    /** A steel all-moving surface of a NACA section of the thickness ratio (area 0.685 t c). */
    static PartDesign surface(String name, double chordM, double spanM, double maxDeflectionDeg, double thicknessRatio, AirfoilPolar polar) {
        double c = chordM, s = spanM, t = thicknessRatio * c, volume = 0.685 * t * c * s;
        MassProperties unit = new MassProperties(volume, Vec3d.ZERO, volume * (s * s + t * t) / 12, volume * (c * c + t * t) / 12,
                volume * (c * c + s * s) / 12, 0, 0, 0);
        MassProperties m = MassSpec.solid(Material.STEEL).resolve(th -> unit);
        return new Fixture(name, "surface " + chordM + " " + spanM + " " + maxDeflectionDeg + " " + thicknessRatio + " " + polar,
                d -> new ControlSurfacePart(d, chordM, spanM, maxDeflectionDeg, thicknessRatio, polar, m, List.of()));
    }
}
