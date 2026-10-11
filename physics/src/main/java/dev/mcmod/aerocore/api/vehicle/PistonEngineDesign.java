package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.engine.CrankKinematics;
import dev.mcmod.aerocore.api.engine.Dyno;
import dev.mcmod.aerocore.api.engine.EngineCycle;
import dev.mcmod.aerocore.api.engine.EngineOil;
import dev.mcmod.aerocore.api.engine.Friction;
import dev.mcmod.aerocore.api.engine.Fuel;
import dev.mcmod.aerocore.api.engine.ValveFlow;
import dev.mcmod.aerocore.api.engine.ValveLift;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.IntStream;

/**
 * A four-stroke spark-ignition piston engine assembled from designed components (block, head, pistons, rods,
 * crankshaft, valves, camshaft, spark plugs, injectors, valve train, fuel, ignition, lubrication, cooling, intake,
 * exhaust). Its crank turns about the part's +X axis and drives a shaft (Connection.Shaft) to whatever it powers.
 * <p>
 * What the components decide, rather than a parameter: the compression ratio (chamber, gasket, deck clearance and
 * piston crown), the flame's travel (plug count and position over the bore), breathing (valves, ports, cam, runners,
 * throttle), knock (octane, compression, chamber, spark control), friction (bearings, rings, valve train, oil),
 * inertia (crank, flywheel, rods, pistons), mass, and the limits (valve float, piston speed, rod fatigue, crown
 * strength, injector flow, cooling).
 * <p>
 * Performance comes from the crank-angle cycle simulation (EngineCycle, calibrated per point by Dyno) on a map of
 * speed and manifold pressure, built off the game thread (warmUp); friction (Friction) is applied at run time with
 * the oil's viscosity at the engine's temperature, so a cold engine is weaker. Inputs: "throttle" 0..1 and "ignition"
 * (1 runs, 0 stops; unbound = on). Internal states: manifold pressure over ambient, engine temperature (K), running.
 */
public record PistonEngineDesign(String name, EngineBlockDesign block, CylinderHeadDesign head, PistonDesign piston, ConnectingRodDesign rod,
                                 CrankshaftDesign crank, ValveDesign valves, CamshaftDesign camshaft, SparkPlugDesign sparkPlug,
                                 FuelInjectorDesign injector, ValvetrainDesign valvetrain, FuelSystemDesign fuelSystem, IgnitionDesign ignition,
                                 LubricationDesign lubrication, CoolingDesign cooling, IntakeDesign intake, ExhaustDesign exhaust) implements PartDesign {
    public static final String TYPE = "aerocore:piston_engine";
    /** Wall margin around a valve seat and the bridge between seats, m (typical seat-insert walls; rules of the packing check). */
    public static final double SEAT_MARGIN = 0.0015, VALVE_BRIDGE = 0.003;
    /** Highest mean piston speed in production and racing engines, about 25 m/s (Formula One); beyond it is extrapolation. */
    public static final double PISTON_SPEED_LIMIT = 25;

    public PistonEngineDesign {
        Objects.requireNonNull(name, "name");
        for (Object o : new Object[]{block, head, piston, rod, crank, valves, camshaft, sparkPlug, injector, valvetrain, fuelSystem, ignition,
                lubrication, cooling, intake, exhaust}) { Objects.requireNonNull(o, "component"); }
    }

    @Override public String type() { return TYPE; }

    /** The components with their JSON keys, in order. */
    public Map<String, EngineComponent> components() {
        Map<String, EngineComponent> m = new LinkedHashMap<>();
        m.put("block", block); m.put("head", head); m.put("piston", piston); m.put("rod", rod); m.put("crank", crank); m.put("valves", valves);
        m.put("camshaft", camshaft); m.put("sparkPlug", sparkPlug); m.put("injector", injector); m.put("valvetrain", valvetrain);
        m.put("fuelSystem", fuelSystem); m.put("ignition", ignition); m.put("lubrication", lubrication); m.put("cooling", cooling);
        m.put("intake", intake); m.put("exhaust", exhaust);
        return m;
    }

    // ---------------------------------------------------------------- derived geometry

    public double bore() { return block.boreM(); }
    public double stroke() { return crank.strokeM(); }
    public double pistonArea() { return Math.PI * bore() * bore() / 4; }
    /** Swept volume of one cylinder, m^3. */
    public double cylinderDisplacement() { return pistonArea() * stroke(); }
    /** Swept volume of the engine, m^3. */
    public double displacement() { return cylinderDisplacement() * block.cylinders(); }
    /** Distance from the piston crown at TDC down from the deck, m (negative: it stands proud of the deck). */
    public double deckClearance() { return block.deckHeightM() - (stroke() / 2 + rod.lengthM() + piston.compressionHeightM()); }
    /** Clearance volume of one cylinder: chamber + gasket + deck clearance - crown, m^3. */
    public double clearanceVolume() {
        return head.chamberVolumeM3() + pistonArea() * (head.gasketThicknessM() + deckClearance()) - piston.crownVolumeM3();
    }
    public double compressionRatio() { return (cylinderDisplacement() + clearanceVolume()) / clearanceVolume(); }

    // ---------------------------------------------------------------- checks

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        for (Map.Entry<String, EngineComponent> e : components().entrySet()) {
            for (DesignIssue i : e.getValue().check()) { issues.add(i.under(e.getKey())); }
        }
        if (issues.stream().anyMatch(i -> i.kind() == DesignIssue.Kind.INVALID)) { return issues; }
        double bore = bore();
        double clearance = bore - piston.diameterM();
        if (!(clearance > 0)) { issues.add(DesignIssue.invalid("piston.diameterM", "The piston is not smaller than the bore")); }
        else if (clearance > 0.0005) { issues.add(DesignIssue.invalid("piston.diameterM", "The piston is more than 0.5 mm smaller than the bore: it would rock and blow by")); }
        else { DesignIssue.range(issues, "pistonClearanceM", clearance, 0.00002, 0.00015, "running clearances of aluminium pistons"); }
        double bigEnd = rod.bigEndBoreM() - crank.pinDiameterM();
        if (bigEnd < 0 || bigEnd > 0.0002) { issues.add(DesignIssue.invalid("rod.bigEndBoreM", "The rod's big end does not fit the crankpin (0-0.2 mm bearing clearance)")); }
        double smallEnd = rod.smallEndBoreM() - piston.pinDiameterM();
        if (smallEnd < 0 || smallEnd > 0.0001) { issues.add(DesignIssue.invalid("rod.smallEndBoreM", "The rod's small end does not fit the piston pin")); }
        int rodsPerPin = block.layout() == EngineBlockDesign.Layout.VEE ? 2 : 1;
        if (rodsPerPin * rod.bigEndWidthM() > crank.pinWidthM() + 1e-9) { issues.add(DesignIssue.invalid("rod.bigEndWidthM", "The rods are wider than the crankpin")); }
        if (deckClearance() < -head.gasketThicknessM()) { issues.add(DesignIssue.invalid("block.deckHeightM", "The piston rises past the gasket into the head")); }
        if (!(clearanceVolume() > 0)) { issues.add(DesignIssue.invalid("piston.crownVolumeM3", "No clearance volume left: the piston hits the head")); }
        else { DesignIssue.range(issues, "compressionRatio", compressionRatio(), 6, 14, "spark-ignition engines; knock and burn correlations fitted near 9-12"); }
        if (!valvesFit()) { issues.add(DesignIssue.invalid("valves", "The valve heads do not fit in the bore")); }
        if (sparkPlug.threadDiameterM() > bore / 4) { issues.add(DesignIssue.invalid("sparkPlug.threadDiameterM", "The plug is too large for the bore")); }
        DesignIssue.range(issues, "intakeLiftOverDiameter", camshaft.intakeLiftM() / valves.intakeHeadM(), 0.15, 0.35, "lift beyond a third of the valve diameter flows no more (port limited)");
        DesignIssue.range(issues, "exhaustLiftOverDiameter", camshaft.exhaustLiftM() / valves.exhaustHeadM(), 0.15, 0.35, "lift beyond a third of the valve diameter flows no more (port limited)");
        if (fuelSystem.delivery() == FuelSystemDesign.Delivery.DIRECT_INJECTION && injector.kind() != FuelInjectorDesign.Kind.DIRECT) {
            issues.add(DesignIssue.invalid("injector.kind", "Direct injection needs direct injectors"));
        }
        if (fuelSystem.delivery() == FuelSystemDesign.Delivery.PORT_INJECTION && injector.kind() != FuelInjectorDesign.Kind.PORT) {
            issues.add(DesignIssue.invalid("injector.kind", "Port injection needs port injectors"));
        }
        if (intake.throttleBodies() > block.cylinders()) { issues.add(DesignIssue.invalid("intake.throttleBodies", "More throttle bodies than cylinders")); }
        DesignIssue.range(issues, "meanPistonSpeedAtRevLimit", 2 * stroke() * ignition.revLimitRpm() / 60, 0, PISTON_SPEED_LIMIT, "production and racing engines");
        DesignIssue.range(issues, "rodRatio", rod.lengthM() / stroke(), 1.4, 2.2, "production engines");
        return issues;
    }

    /**
     * Valve packing in the bore, projected on the deck: intake valves in a row on one side (x < 0), exhaust on the other,
     * rows along the cam axis (y) with VALVE_BRIDGE between seats and SEAT_MARGIN to the bore wall. A valve tilted by
     * half the head's included angle projects as an ellipse D cos(angle/2) across the bore and D along it. Each row moves
     * out from the centre line as far as the bore allows; they fit when every intake and exhaust seat are a bridge apart
     * (measured along the line between their centres). The 3-D chamber and the plug boss are not checked.
     */
    public boolean valvesFit() {
        double r = bore() / 2 - SEAT_MARGIN, b = VALVE_BRIDGE, tilt = Math.cos(Math.toRadians(head.valveAngleDeg() / 2));
        double di = valves.intakeHeadM(), de = valves.exhaustHeadM();
        double[] yi = rowOffsets(head.intakeValves(), di + b), ye = rowOffsets(head.exhaustValves(), de + b);
        double xi = rowReach(yi, di / 2 * tilt, di / 2, r), xe = rowReach(ye, de / 2 * tilt, de / 2, r);
        if (Double.isNaN(xi) || Double.isNaN(xe)) { return false; }
        for (double a : yi) for (double c : ye) {
            double dx = xi + xe, dy = c - a, dist = Math.hypot(dx, dy), angle = Math.atan2(dy, dx);
            double radii = ellipseRadius(di / 2 * tilt, di / 2, angle) + ellipseRadius(de / 2 * tilt, de / 2, angle);
            if (dist - radii < b - 1e-12) { return false; }
        }
        return true;
    }
    private static double[] rowOffsets(int n, double pitch) {
        double[] y = new double[n];
        for (int k = 0; k < n; k++) { y[k] = (k - (n - 1) / 2.0) * pitch; }
        return y;
    }
    /** Distance of an ellipse's boundary from its centre along a direction (semi-axes ax across, ay along). */
    private static double ellipseRadius(double ax, double ay, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return ax * ay / Math.sqrt(ay * ay * c * c + ax * ax * s * s);
    }
    /** Farthest a row of ellipses (centres at x, y_k) may sit from the centre line inside a circle of radius r; NaN if not even at x = 0. */
    private static double rowReach(double[] y, double ax, double ay, double r) {
        java.util.function.DoublePredicate fits = x -> {
            for (double yk : y) {
                for (int i = 0; i < 72; i++) {
                    double phi = Math.PI * i / 36, px = x + ax * Math.cos(phi), py = yk + ay * Math.sin(phi);
                    if (px * px + py * py > r * r) { return false; }
                }
            }
            return true;
        };
        if (!fits.test(0)) { return Double.NaN; }
        double lo = 0, hi = r;
        for (int k = 0; k < 50; k++) {
            double mid = 0.5 * (lo + hi);
            if (fits.test(mid)) { lo = mid; } else { hi = mid; }
        }
        return lo;
    }

    // ---------------------------------------------------------------- mass

    /** Mass of the whole engine with its systems, kg. */
    public double totalMassKg() {
        int n = block.cylinders();
        int camshafts = head.layout() == CylinderHeadDesign.Layout.OHV ? 1 : block.banks() * (head.layout() == CylinderHeadDesign.Layout.DOHC ? 2 : 1);
        int valvesPerCylinder = head.intakeValves() + head.exhaustValves();
        double m = block.massKg() + head.massFor(block) + n * (piston.massKg() + rod.massKg()) + crank.massFor(block)
                + n * (head.intakeValves() * valves.intakeValveMassKg() + head.exhaustValves() * valves.exhaustValveMassKg())
                + camshafts * camshaft.massFor(block, head.layout() == CylinderHeadDesign.Layout.DOHC ? Math.max(head.intakeValves(), head.exhaustValves()) : valvesPerCylinder)
                + n * head.sparkPlugs() * sparkPlug.massKg()
                + (fuelSystem.delivery() == FuelSystemDesign.Delivery.CARBURETTOR ? 0 : n * fuelSystem.injectorsPerCylinder() * injector.massKg())
                + valvetrain.massFor(n * valvesPerCylinder) + fuelSystem.massKg() + ignition.massKg() + lubrication.massKg() + cooling.massKg()
                + intake.massFor(n) + exhaust.massKg();
        return m;
    }

    /** Reciprocating mass per cylinder (piston with pin and rings plus the rod's small-end share), kg. */
    public double reciprocatingMass() { return piston.massKg() + rod.reciprocatingFraction() * rod.massKg(); }

    /** Moment of inertia of everything turning with the crank, kg m^2: crank, flywheel, rods' big ends and, averaged over a turn, half the reciprocating mass at the crank radius. */
    public double crankInertia() {
        double r = stroke() / 2, n = block.cylinders();
        return crank.rotatingInertia(block) + n * (1 - rod.reciprocatingFraction()) * rod.massKg() * r * r + n * 0.5 * reciprocatingMass() * r * r;
    }

    // ---------------------------------------------------------------- the cycle model

    /** The fuel. */
    public Fuel fuel() { return fuelSystem.fuelModel(); }

    /** EngineCycle spec of one cylinder at a lambda and cam phases (crank degrees later). */
    public EngineCycle.Spec cylinderSpec(double lambda, double intakePhaseDeg, double exhaustPhaseDeg) {
        CrankKinematics k = new CrankKinematics(bore(), stroke(), rod.lengthM(), clearanceVolume());
        ValveLift in = camshaft.intakeLift(intakePhaseDeg), ex = camshaft.exhaustLift(exhaustPhaseDeg);
        double headFactor = head.chamberAreaFactor(bore());
        return new EngineCycle.Spec(k, head.intakeValves(), valves.intakeHeadM(), head.intakeThroatRatio() * valves.intakeHeadM(), valves.intakeStemM(),
                head.portFlowCoefficient(), in, head.exhaustValves(), valves.exhaustHeadM(), head.exhaustThroatRatio() * valves.exhaustHeadM(),
                valves.exhaustStemM(), head.portFlowCoefficient(), ex, intake.runnerLengthM(), intake.runnerDiameterM(), fuel(), lambda,
                COMBUSTION_EFFICIENCY, fuelSystem.delivery() == FuelSystemDesign.Delivery.DIRECT_INJECTION, head.flameTravel(bore()),
                cooling.wallTemperatureK(), headFactor, 1);
    }
    /** Fraction of the fuel that burns in a warm engine at lambda >= 1 (Heywood: 95-98 % for spark-ignition engines; crevices and quench), a typical value. */
    public static final double COMBUSTION_EFFICIENCY = 0.97;

    /** The friction model's view of this engine. */
    public Friction.Engine frictionModel() {
        int camshafts = head.layout() == CylinderHeadDesign.Layout.OHV ? 1 : block.banks() * (head.layout() == CylinderHeadDesign.Layout.DOHC ? 2 : 1);
        return new Friction.Engine(bore(), stroke(), block.cylinders(), compressionRatio(), block.mainBearings(),
                crank.mainDiameterM(), crank.mainWidthM(), crank.pinDiameterM(), rod.bigEndWidthM(), camshafts * camshaft.bearingsPerShaft(),
                block.cylinders() * (head.intakeValves() + head.exhaustValves()), Math.max(camshaft.intakeLiftM(), camshaft.exhaustLiftM()),
                valvetrain.friction(head.layout()), valvetrain.rollerFollowers(), piston.ringTensionRatio(), block.linerRoughness());
    }

    /** Relative air-fuel ratio the fuel system targets at a manifold pressure ratio: part-load value up to 0.7, full-load value at 1. */
    public double targetLambda(double pressureRatio) {
        double f = Math.clamp((pressureRatio - 0.7) / 0.3, 0, 1);
        return fuelSystem.partLoadLambda() + f * (fuelSystem.fullLoadLambda() - fuelSystem.partLoadLambda());
    }

    /** Most fuel the injectors deliver per cylinder per cycle at a speed (rpm), kg; infinite for a carburettor. */
    public double injectorLimit(double rpm, double manifoldPa) {
        if (fuelSystem.delivery() == FuelSystemDesign.Delivery.CARBURETTOR) { return Double.POSITIVE_INFINITY; }
        double dp = fuelSystem.delivery() == FuelSystemDesign.Delivery.DIRECT_INJECTION ? fuelSystem.fuelPressurePa() - 2e6 : fuelSystem.fuelPressurePa();
        double cycle = 120 / rpm;
        return fuelSystem.injectorsPerCylinder() * injector.flowM3PerS(dp) * fuel().liquidDensityKgPerM3() * FuelInjectorDesign.MAX_DUTY * cycle;
    }

    @Override public Map<String, Object> parameters() {
        Map<String, Object> m = new LinkedHashMap<>();
        for (Map.Entry<String, EngineComponent> e : components().entrySet()) { m.put(e.getKey(), e.getValue().toJson()); }
        return m;
    }

    static PartDesign read(Fields f) {
        return new PistonEngineDesign(f.string("name", "engine"),
                component(f, "block", EngineBlockDesign::read), component(f, "head", CylinderHeadDesign::read), component(f, "piston", PistonDesign::read),
                component(f, "rod", ConnectingRodDesign::read), component(f, "crank", CrankshaftDesign::read), component(f, "valves", ValveDesign::read),
                component(f, "camshaft", CamshaftDesign::read), component(f, "sparkPlug", SparkPlugDesign::read), component(f, "injector", FuelInjectorDesign::read),
                component(f, "valvetrain", ValvetrainDesign::read), component(f, "fuelSystem", FuelSystemDesign::read), component(f, "ignition", IgnitionDesign::read),
                component(f, "lubrication", LubricationDesign::read), component(f, "cooling", CoolingDesign::read), component(f, "intake", IntakeDesign::read),
                component(f, "exhaust", ExhaustDesign::read));
    }
    private static <T> T component(Fields f, String key, java.util.function.BiFunction<String, Fields, T> reader) {
        Fields c = f.child(key);
        return reader.apply(c.string("name", key), c.has("parameters") ? c.child("parameters") : c);
    }

    @Override public Part compile() {
        List<DesignIssue> issues = check();
        DesignException.throwIfInvalid(issues);
        return new Compiled(this, issues.stream().filter(i -> i.kind() == DesignIssue.Kind.EXTRAPOLATION).toList());
    }

    // ---------------------------------------------------------------- the map

    /** Speeds (rpm) and manifold pressure ratios of the map. */
    public double[] mapSpeeds() {
        double top = ignition.revLimitRpm() * 1.08, low = 500;
        int n = 40;   // a tuned runner's ram peaks are a few hundred rpm wide
        double[] rpms = new double[n];
        for (int i = 0; i < n; i++) { rpms[i] = low + (top - low) * i / (n - 1); }
        return rpms;
    }
    public static final double[] MAP_PRESSURES = {0.12, 0.2, 0.3, 0.42, 0.56, 0.7, 0.85, 1.0};
    /** Standard ambient the map is computed at (pressure Pa, temperature K); run time scales flows by the actual air density. */
    public static final double AMBIENT_PA = 101325, AMBIENT_K = 298.15, INTAKE_HEATING_K = 10;

    /** One map point: per cylinder per cycle. */
    public record MapPoint(double rpm, double pressureRatio, double netImepPa, double pumpingMepPa, double airKg, double fuelKg, double wallHeatJ,
                           double exhaustTemperatureK, double sparkAdvanceDeg, boolean knockLimited, boolean knocking, double peakPressurePa,
                           double lambda, double intakePhaseDeg, double exhaustPhaseDeg) {}

    /** The engine's performance map: rows of speeds, columns of pressure ratios. Immutable. */
    public record EngineMap(double[] rpms, double[] pressures, MapPoint[][] points) {
        public MapPoint at(int i, int j) { return points[i][j]; }

        /** Bilinear interpolation of one quantity at a speed and pressure ratio (clamped to the map). */
        public double value(double rpm, double ratio, java.util.function.ToDoubleFunction<MapPoint> q) {
            int i = index(rpms, rpm), j = index(pressures, ratio);
            double fi = Math.clamp((rpm - rpms[i]) / (rpms[i + 1] - rpms[i]), 0, 1), fj = Math.clamp((ratio - pressures[j]) / (pressures[j + 1] - pressures[j]), 0, 1);
            double a = q.applyAsDouble(points[i][j]), b = q.applyAsDouble(points[i + 1][j]), c = q.applyAsDouble(points[i][j + 1]), d = q.applyAsDouble(points[i + 1][j + 1]);
            return (1 - fi) * ((1 - fj) * a + fj * c) + fi * ((1 - fj) * b + fj * d);
        }
        private static int index(double[] grid, double x) {
            int i = 0;
            while (i < grid.length - 2 && x > grid[i + 1]) { i++; }
            return i;
        }
    }

    /**
     * Builds the map (seconds of work; the speeds in parallel): every point calibrated by Dyno at the fuel system's
     * target lambda, leaner where the injectors run out, cam phases chosen for best full-load torque at each speed
     * (and kept at part load), exhaust back pressure from the exhaust system.
     */
    public EngineMap buildMap() {
        double[] rpms = mapSpeeds();
        MapPoint[][] points = new MapPoint[rpms.length][MAP_PRESSURES.length];
        Dyno.SparkControl spark = new Dyno.SparkControl(ignition.knockControl(), ignition.maxAdvanceDeg(), 1);
        double intakeT = AMBIENT_K + INTAKE_HEATING_K;
        IntStream.range(0, rpms.length).parallel().forEach(i -> {
            double rpm = rpms[i];
            double[] phases = bestPhases(rpm, spark, intakeT);
            for (int j = 0; j < MAP_PRESSURES.length; j++) { points[i][j] = point(rpm, MAP_PRESSURES[j], phases[0], phases[1], spark, intakeT); }
        });
        return new EngineMap(rpms, MAP_PRESSURES.clone(), points);
    }

    private double[] bestPhases(double rpm, Dyno.SparkControl spark, double intakeT) {
        double ri = valvetrain.intakePhaseRangeDeg(), re = valvetrain.exhaustPhaseRangeDeg();
        double bestI = 0, bestE = 0;
        if (ri > 0) {
            double best = Double.NEGATIVE_INFINITY;
            for (double p : new double[]{-ri, -ri / 2, 0, ri / 2, ri}) {
                double imep = point(rpm, 1, p, 0, spark, intakeT).netImepPa();
                if (imep > best) { best = imep; bestI = p; }
            }
        }
        if (re > 0) {
            double best = Double.NEGATIVE_INFINITY;
            for (double p : new double[]{-re, -re / 2, 0, re / 2, re}) {
                double imep = point(rpm, 1, bestI, p, spark, intakeT).netImepPa();
                if (imep > best) { best = imep; bestE = p; }
            }
        }
        return new double[]{bestI, bestE};
    }

    private MapPoint point(double rpm, double ratio, double phaseI, double phaseE, Dyno.SparkControl spark, double intakeT) {
        double pIm = ratio * AMBIENT_PA, sp = 2 * stroke() * rpm / 60;
        double pEx = exhaust.backPressure(AMBIENT_PA, pIm, sp);
        double lambda = targetLambda(ratio);
        Dyno.Point p = Dyno.operate(cylinderSpec(lambda, phaseI, phaseE), rpm, pIm, intakeT, pEx, spark);
        double limit = injectorLimit(rpm, pIm);
        if (p.cycle().fuelKg() > limit) {
            // The injectors run out: the same air with the fuel they can give, a leaner mixture.
            double leaner = Math.min(2.5, lambda * p.cycle().fuelKg() / limit);
            p = Dyno.operate(cylinderSpec(leaner, phaseI, phaseE), rpm, pIm, intakeT, pEx, spark);
            lambda = leaner;
        }
        EngineCycle.Result r = p.cycle();
        return new MapPoint(rpm, ratio, r.netImep(), r.pumpingMep(), r.inductedAirKg(), r.fuelKg(), r.heatToWallsJ(), r.exhaustTemperatureK(),
                p.sparkAdvanceDeg(), p.knockLimited(), p.knocking(), r.peakPressurePa(), lambda, phaseI, phaseE);
    }

    // ---------------------------------------------------------------- compiled part

    /** Indices of the internal states. */
    public static final int STATE_MANIFOLD = 0, STATE_TEMPERATURE = 1, STATE_RUNNING = 2;

    private static final class Compiled implements Part {
        private final PistonEngineDesign design;
        private final List<DesignIssue> issues;
        private final MassProperties mass;
        private final double inertia, displacement, plenumVolume, closedAngle = Math.toRadians(5);
        private final Friction.Engine friction;
        private final EngineOil oil;
        private final double heatCapacity;
        private volatile EngineMap map;

        Compiled(PistonEngineDesign design, List<DesignIssue> issues) {
            this.design = design; this.issues = issues;
            double m = design.totalMassKg();
            EngineBlockDesign b = design.block();
            double length = b.length() + 0.2;
            double width = b.layout() == EngineBlockDesign.Layout.INLINE ? b.boreM() + 0.25
                    : 2 * (b.deckHeightM() + design.head().castingHeight()) * Math.sin(Math.toRadians(b.effectiveBankAngle()) / 2) + b.boreM() + 0.1;
            double height = b.deckHeightM() + design.head().castingHeight() + 0.15;
            mass = MassProperties.of(m, Vec3d.ZERO, InertiaTensor.box(m, new Vec3d(length, height, width)));
            inertia = design.crankInertia();
            displacement = design.displacement();
            plenumVolume = design.intake().plenumVolumeM3() + b.cylinders() * Math.PI * Math.pow(design.intake().runnerDiameterM(), 2) / 4 * design.intake().runnerLengthM();
            friction = design.frictionModel();
            oil = design.lubrication().oil();
            double metal = 0;
            metal += b.massKg() * b.material().specificHeatJPerKgK() + design.head().massFor(b) * design.head().material().specificHeatJPerKgK();
            metal += (m - b.massKg() - design.head().massFor(b)) * 500;   // the rest, mostly steel and aluminium, at a middling 500 J/(kg K)
            heatCapacity = metal + design.cooling().coolantVolumeM3() * CoolingDesign.COOLANT_DENSITY * CoolingDesign.COOLANT_SPECIFIC_HEAT
                    + design.lubrication().oilVolumeM3() * LubricationDesign.OIL_DENSITY * LubricationDesign.OIL_SPECIFIC_HEAT;
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<CollisionSphere> collision() { return List.of(); }
        @Override public List<DesignIssue> issues() { return issues; }
        @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, inertia, true)); }
        @Override public List<String> inputs() { return List.of("throttle", "ignition"); }
        @Override public int internalStates() { return 3; }
        @Override public double[] initialInternals() { return new double[]{1, 0, 0}; }

        @Override public void warmUp() { map(); }
        EngineMap map() {
            EngineMap m = map;
            if (m == null) {
                synchronized (this) {
                    if (map == null) {
                        String key = Json.write(design.toJson());
                        EngineMap built = MAPS.get(key);
                        if (built == null) { built = design.buildMap(); MAPS.put(key, built); }
                        map = built;
                    }
                    m = map;
                }
            }
            return m;
        }

        @Override public void loads(PartStep step) {
            EngineMap m = map();
            double dt = step.dt();
            FluidState air = step.environment().point(step.point(Vec3d.ZERO)).air();
            double airT = air.speedOfSoundMps() * air.speedOfSoundMps() / (1.4 * 287.05), ambient = air.densityKgPerM3() * 287.05 * airT;
            double densityScale = ambient / AMBIENT_PA * AMBIENT_K / airT;
            double omega = step.rotorSpeed(0), rpm = Math.max(0, omega * 30 / Math.PI);
            double throttle = Math.clamp(step.input("throttle", 0), 0, 1), ignitionOn = step.input("ignition", 1);
            double ratio = Math.clamp(step.internal(STATE_MANIFOLD), 0.02, 1.2);
            double temperature = step.internal(STATE_TEMPERATURE);
            if (!(temperature > 0)) { temperature = airT; }
            boolean running = step.internal(STATE_RUNNING) > 0.5;
            IgnitionDesign ign = design.ignition();
            // run state: the starter cranks until the engine fires; below 300 rpm it stalls
            if (ignitionOn < 0.5) { running = false; }
            else if (!running && rpm >= 400) { running = true; }
            else if (running && rpm < 300) { running = false; }
            boolean cranking = ignitionOn >= 0.5 && !running;
            // idle control: below the idle speed the ECU opens an air bypass (proportional, a control rule)
            double bypass = running ? Math.clamp(0.04 + 0.4 * (ign.idleRpm() - rpm) / ign.idleRpm(), 0, 0.12) : 0.04;
            double opening = Math.max(throttle, bypass);
            double angle = closedAngle + opening * (Math.PI / 2 - closedAngle);
            double throttleArea = 0.8 * design.intake().throttleBodies() * ValveFlow.throttleArea(design.intake().throttleDiameterM(), closedAngle, angle);
            double fullArea = design.intake().throttleArea();
            // manifold pressure: implicit balance of throttle inflow and engine draw over the step
            int n = design.block().cylinders();
            double cycles = n * rpm / 120;
            double rho = air.densityKgPerM3(), rAir = 287.05;
            double previousFlow = cycles * m.value(rpm, ratio, MapPoint::airKg) * densityScale;
            double filterDrop = design.intake().filterLoss() * previousFlow * previousFlow / (2 * rho * fullArea * fullArea);
            double upstream = Math.max(0.5 * ambient, ambient - filterDrop);
            double capacity = plenumVolume / (rAir * airT);
            double lo = 0.02, hi = Math.min(1.2, upstream / ambient);
            final double startRatio = ratio, finalRpm = rpm;
            java.util.function.DoubleUnaryOperator balance = x -> {
                double p = x * ambient;
                double in = ValveFlow.massFlow(throttleArea, upstream, airT, p, rAir, 1.4);
                double draw = cycles * m.value(finalRpm, x, MapPoint::airKg) * densityScale;
                return capacity * (x - startRatio) * ambient / dt - (in - draw);
            };
            for (int k = 0; k < 40; k++) {
                double mid = 0.5 * (lo + hi);
                if (balance.applyAsDouble(mid) > 0) { hi = mid; } else { lo = mid; }
            }
            ratio = 0.5 * (lo + hi);
            final double manifold = ratio;
            // torque
            boolean fuelCut = !running || rpm > ign.revLimitRpm() || (throttle < 0.02 && rpm > ign.idleRpm() + 600);
            double scaleTorque = displacement / (4 * Math.PI);
            double viscosity = oil.frictionScaling(temperature - 273.15);
            java.util.function.DoubleUnaryOperator torqueAt = w -> {
                double r = Math.max(0, w * 30 / Math.PI);
                double imep = fuelCut ? -m.value(r, manifold, MapPoint::pumpingMepPa) : m.value(r, manifold, MapPoint::netImepPa) * densityScale;
                double fmep = Friction.fmep(friction, Math.max(r, 50), manifold * ambient, ambient, viscosity).totalPa();
                double spin = Math.tanh(w / 2);   // friction and pumping oppose the rotation; smooth through standstill
                double t = fuelCut ? (imep - fmep) * scaleTorque * spin : (imep - fmep * spin) * scaleTorque;
                if (cranking) { t += ign.starterTorqueNm() * Math.clamp((500 - r) / 200, 0, 1); }
                return t;
            };
            double torque = torqueAt.applyAsDouble(omega);
            double dw = Math.max(1, 1e-3 * Math.abs(omega));
            double slope = (torqueAt.applyAsDouble(omega + dw) - torqueAt.applyAsDouble(omega - dw)) / (2 * dw);
            step.rotorTorque(0, torque, slope, false);
            // fuel and heat
            double fuelFlow = fuelCut ? 0 : cycles * m.value(rpm, ratio, MapPoint::fuelKg) * densityScale;
            double wallHeat = fuelCut ? 0 : cycles * m.value(rpm, ratio, MapPoint::wallHeatJ) * densityScale;
            double frictionPower = Math.abs(Friction.fmep(friction, Math.max(rpm, 50), ratio * ambient, ambient, viscosity).totalPa() * scaleTorque * omega);
            double airspeed = step.velocity(Vec3d.ZERO).subtract(air.velocity()).length();
            double conductivity = 0.0263 * Math.pow(airT / 300, 0.8);   // air, W/(m K), approximate temperature dependence
            double rejected = design.cooling().rejection(temperature, airT, air.densityKgPerM3(), air.dynamicViscosityPaS(), conductivity, airspeed);
            double nextT = temperature + dt * (wallHeat + frictionPower - rejected) / heatCapacity;
            step.setInternal(STATE_MANIFOLD, ratio);
            step.setInternal(STATE_TEMPERATURE, nextT);
            step.setInternal(STATE_RUNNING, running ? 1 : 0);
            step.diagnostic("rpm", rpm);
            step.diagnostic("throttle", throttle);
            step.diagnostic("manifoldKpa", ratio * ambient / 1000);
            step.diagnostic("torqueNm", torque);
            step.diagnostic("powerW", torque * omega);
            step.diagnostic("fuelKgPerS", fuelFlow);
            step.diagnostic("engineC", nextT - 273.15);
            step.diagnostic("running", running ? 1 : 0);
            step.diagnostic("knocking", !fuelCut && m.value(rpm, ratio, p -> p.knocking() ? 1 : 0) > 0.5 ? 1 : 0);
        }
    }

    /** Built maps by design JSON (a design's map is the same for every vehicle that carries it). */
    private static final Map<String, EngineMap> MAPS = java.util.Collections.synchronizedMap(new LinkedHashMap<>(8, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<String, EngineMap> eldest) { return size() > 32; }
    });
}
