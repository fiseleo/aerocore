package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.engine.Gas;
import dev.mcmod.aerocore.api.engine.Turbojet;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A single-spool turbojet assembled from designed components: inlet, compressor, combustor, turbine, nozzle and fuel
 * control. It points along the part's +X axis (inlet forward, jet aft) and pushes forward; its spool is a rotor about
 * +X, so spinning it up kicks the vehicle the other way.
 * <p>
 * The cycle (Turbojet) is sized at sea-level static from the compressor's pressure ratio and airflow and the control's
 * turbine inlet temperature; the spool's design speed is the compressor's. In flight the spool follows its fuel
 * (Turbojet.quasiSteady): thrust including ram drag, turbine and compressor power, surge margin used. Inputs:
 * "throttle" 0..1 and "ignition" (unbound = on). Internal states: fuel flow (kg/s), burner (0 out, 1 lit while the
 * starter still helps, 2 running on its own), surge time left (s).
 */
public record JetEngineDesign(String name, JetInletDesign inlet, CompressorDesign compressor, CombustorDesign combustor, TurbineDesign turbine,
                              JetNozzleDesign nozzle, JetControlDesign control) implements PartDesign {
    public static final String TYPE = "aerocore:jet_engine";
    /** Turbine stage loading (work over tip speed squared) of practical turbines (Smith's chart) and where none works. */
    public static final double TURBINE_LOADING = 2.5, TURBINE_LOADING_LIMIT = 4;

    public JetEngineDesign {
        Objects.requireNonNull(name, "name");
        for (Object o : new Object[]{inlet, compressor, combustor, turbine, nozzle, control}) { Objects.requireNonNull(o, "component"); }
    }

    @Override public String type() { return TYPE; }

    public Map<String, EngineComponent> components() {
        Map<String, EngineComponent> m = new LinkedHashMap<>();
        m.put("inlet", inlet); m.put("compressor", compressor); m.put("combustor", combustor); m.put("turbine", turbine);
        m.put("nozzle", nozzle); m.put("control", control);
        return m;
    }

    /** The cycle these components make. */
    public Turbojet.Design cycle() {
        return new Turbojet.Design(compressor.pressureRatio(), compressor.airflowKgPerS(), compressor.polytropicEfficiency(), control.turbineInletK(),
                1 - combustor.pressureLoss(), combustor.efficiency(), turbine.polytropicEfficiency(), turbine.mechanicalEfficiency(), turbine.coolingBleed(),
                nozzle.pressureRetention(), nozzle.convergentDivergent(), nozzle.convergentDivergent() ? nozzle.exitAreaRatio() : 1, inlet.recovery(),
                inlet.kind(), control.fuelModel());
    }
    /** The cycle sized at sea-level static. */
    public Turbojet.Sizing sizing() { return Turbojet.size(cycle(), Turbojet.Flight.sea(0)); }

    /** Design spool speed, rad/s. */
    public double designSpeed() { return compressor.designRpm() * Math.PI / 30; }
    public double turbineTipSpeed() { return Math.PI * turbine.tipDiameterM() * compressor.designRpm() / 60; }
    /** Spool moment of inertia (compressor and turbine rotors), kg m^2. */
    public double spoolInertia() { return compressor.rotorInertia() + turbine.rotorInertia(); }
    /** Overall length, m: inlet, compressor (0.06 diameters per axial stage, 0.4 per centrifugal), combustor, turbine (0.1 diameters per stage), tailpipe. */
    public double length() {
        double c = compressor.tipDiameterM() * (compressor.kind() == CompressorDesign.Kind.AXIAL ? 0.06 * compressor.stages() : 0.4 * compressor.stages());
        return inlet.lengthM() + c + combustor.lengthM() + 0.1 * turbine.tipDiameterM() * turbine.stages() + nozzle.tailpipeLengthM();
    }
    public double diameter() { return Math.max(Math.max(compressor.tipDiameterM(), turbine.tipDiameterM()), combustor.casingDiameterM()) * 1.1; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        for (Map.Entry<String, EngineComponent> e : components().entrySet()) {
            for (DesignIssue i : e.getValue().check()) { issues.add(i.under(e.getKey())); }
        }
        if (issues.stream().anyMatch(i -> i.kind() == DesignIssue.Kind.INVALID)) { return issues; }
        Turbojet.Sizing s;
        try { s = sizing(); }
        catch (IllegalArgumentException e) { issues.add(DesignIssue.invalid("cycle", e.getMessage())); return issues; }
        Turbojet.Point p = s.point();
        double t3 = p.tt3();
        if (turbine.metalTemperature(control.turbineInletK(), t3) > turbine.material().serviceTemperatureK()) {
            issues.add(DesignIssue.invalid("control.turbineInletK", "The turbine blades would run above their material's service temperature (cooled, they take up to "
                    + Math.round(turbine.maxInletTemperature(t3)) + " K)"));
        }
        if (p.tt5() > nozzle.material().serviceTemperatureK()) {
            issues.add(DesignIssue.invalid("nozzle.material", "The tailpipe material cannot take the " + Math.round(p.tt5()) + " K exhaust"));
        }
        double flux = compressor.airflowKgPerS() / inlet.captureAreaM2();
        if (flux > CompressorDesign.CHOKED_FLUX) { issues.add(DesignIssue.invalid("inlet.captureAreaM2", "The inlet is too small: it chokes below the design airflow")); }
        else { DesignIssue.range(issues, "inletFlux", flux, 0, CompressorDesign.FACE_MACH_07_FLUX, "inlets sized for Mach 0.5-0.7 at the face"); }
        double ut = turbineTipSpeed(), wt = p.turbinePowerW() / (turbine.mechanicalEfficiency() * p.airflowKgPerS() * (1 - turbine.coolingBleed()));
        double loading = wt / (turbine.stages() * ut * ut);
        if (loading > TURBINE_LOADING_LIMIT) { issues.add(DesignIssue.invalid("turbine.stages", "Too few turbine stages or too slow a spool: stage loading " + loading)); }
        else { DesignIssue.range(issues, "turbineStageLoading", loading, 0, TURBINE_LOADING, "turbine stage loadings, Smith's chart"); }
        double stress = turbine.discStress(ut);
        if (stress > turbine.material().yieldPa()) { issues.add(DesignIssue.invalid("turbine.tipDiameterM", "The turbine rotor would yield at this tip speed")); }
        else { DesignIssue.range(issues, "turbineDiscStressOverYield", stress / turbine.material().yieldPa(), 0, 2.0 / 3, "rotors kept under two thirds of yield (cold strength)"); }
        return issues;
    }

    /** Mass of the engine, kg. */
    public double totalMassKg() {
        double a8 = sizing().nozzleThroatM2();
        return inlet.massKg() + compressor.massKg() + combustor.massKg() + turbine.massKg() + nozzle.massFor(a8) + control.massFor(compressor.airflowKgPerS());
    }

    @Override public Map<String, Object> parameters() {
        Map<String, Object> m = new LinkedHashMap<>();
        for (Map.Entry<String, EngineComponent> e : components().entrySet()) { m.put(e.getKey(), e.getValue().toJson()); }
        return m;
    }

    static PartDesign read(Fields f) {
        return new JetEngineDesign(f.string("name", "jet"), component(f, "inlet", JetInletDesign::read), component(f, "compressor", CompressorDesign::read),
                component(f, "combustor", CombustorDesign::read), component(f, "turbine", TurbineDesign::read), component(f, "nozzle", JetNozzleDesign::read),
                component(f, "control", JetControlDesign::read));
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

    public static final int STATE_FUEL = 0, STATE_LIT = 1, STATE_SURGE = 2;
    /** Fuel valve lag, s; how long a surge lasts, s; starter cut-out speed and flame-out speed (fractions); fuel gain per unit speed error (rules of the control). */
    static final double FUEL_LAG = 0.1, SURGE_TIME = 0.6, STARTER_CUTOUT = 0.45, FLAMEOUT = 0.1, SPEED_GAIN = 1.5;

    /** The sized cycle and its running lines, built once per design. */
    private record Model(Turbojet.Sizing sizing, Turbojet.RunningLines lines) {}
    private static final Map<String, Model> MODELS = java.util.Collections.synchronizedMap(new LinkedHashMap<>(8, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<String, Model> eldest) { return size() > 32; }
    });

    private static final class Compiled implements Part {
        private final JetEngineDesign design;
        private final List<DesignIssue> issues;
        private final MassProperties mass;
        private final double inertia, omegaDesign, halfLength;
        private volatile Model model;

        Compiled(JetEngineDesign design, List<DesignIssue> issues) {
            this.design = design; this.issues = issues;
            double m = design.totalMassKg(), length = design.length(), d = design.diameter();
            mass = MassProperties.of(m, Vec3d.ZERO, InertiaTensor.box(m, new Vec3d(length, d, d)));
            inertia = design.spoolInertia();
            omegaDesign = design.designSpeed();
            halfLength = length / 2;
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<CollisionSphere> collision() { return List.of(); }
        @Override public List<DesignIssue> issues() { return issues; }
        @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, inertia, true)); }
        @Override public List<String> inputs() { return List.of("throttle", "ignition"); }
        @Override public int internalStates() { return 3; }
        @Override public void warmUp() { model(); }

        Model model() {
            Model m = model;
            if (m == null) {
                synchronized (this) {
                    if (model == null) {
                        String key = Json.write(design.toJson());
                        Model built = MODELS.get(key);
                        if (built == null) {
                            Turbojet.Sizing s = design.sizing();
                            built = new Model(s, Turbojet.runningLines(s));
                            MODELS.put(key, built);
                        }
                        model = built;
                    }
                    m = model;
                }
            }
            return m;
        }

        @Override public void loads(PartStep step) {
            Model m = model();
            Turbojet.Sizing s = m.sizing();
            JetControlDesign c = design.control();
            double dt = step.dt();
            Vec3d inletPoint = new Vec3d(halfLength, 0, 0), axis = step.axis(Vec3d.X);
            HullDynamics.Water sample = step.environment().point(step.point(inletPoint));
            FluidState air = sample.air();
            boolean drowned = sample.hasWater() && step.point(inletPoint).y() < sample.surfaceY();
            double t0 = air.speedOfSoundMps() * air.speedOfSoundMps() / (1.4 * Gas.AIR.gasConstant());
            double p0 = air.densityKgPerM3() * Gas.AIR.gasConstant() * t0;
            double v0 = Math.max(0, step.velocity(inletPoint).subtract(air.velocity()).dot(axis));
            Turbojet.Flight flight = new Turbojet.Flight(t0, p0, v0);
            double omega = step.rotorSpeed(0), n = Math.max(0, omega / omegaDesign);
            double throttle = Math.clamp(step.input("throttle", 0), 0, 1), ignition = step.input("ignition", 1);
            double fuel = Math.max(0, step.internal(STATE_FUEL));
            double state = step.internal(STATE_LIT);
            boolean lit = state > 0.5, running = state > 1.5;
            double surge = Math.max(0, step.internal(STATE_SURGE) - dt);
            if (ignition < 0.5 || drowned || (lit && n < FLAMEOUT)) { lit = false; running = false; }
            else if (!lit && n >= c.lightOffFraction()) { lit = true; }
            if (lit && n >= STARTER_CUTOUT) { running = true; }   // the starter has cut out for good
            // the fuel control
            Turbojet.RunningLines lines = m.lines();
            double command = 0;
            if (lit) {
                double target = c.idleFraction() + throttle * (c.maxSpeedFraction() - c.idleFraction());
                double here = Turbojet.lineFuel(s, lines, flight, n);
                command = Turbojet.lineFuel(s, lines, flight, target) + SPEED_GAIN * (target - n) * s.point().fuelKgPerS();
                double maxFuel = Turbojet.fuelForTurbineInlet(s, lines, flight, n, c.turbineInletK());
                if (c.accelerationSchedule()) {
                    double allowed = design.compressor().surgeMargin() * (1 - c.surgeReserve());
                    maxFuel = Math.min(maxFuel, Turbojet.fuelForSurgeMargin(s, lines, flight, n, allowed));
                }
                command = Math.clamp(command, 0.5 * here, Math.max(0.5 * here, maxFuel));
                if (n > c.maxSpeedFraction() * 1.01) { command = 0.5 * here; }
            }
            fuel += (command - fuel) * Math.min(1, dt / FUEL_LAG);
            if (!lit) { fuel = 0; }
            Turbojet.Point p = Turbojet.quasiSteady(s, lines, flight, n, drowned ? 0 : fuel);
            if (lit && p.surgeMarginUsed() > design.compressor().surgeMargin()) { surge = SURGE_TIME; }
            double share = surge > 0 ? 0.3 : 1;   // a surging compressor loses most of its flow (a rule for the stall's effect)
            double turbine = p.turbinePowerW() * share, compressor = p.compressorPowerW() * share;
            double w = Math.max(omega, 0.05 * omegaDesign);
            double torque = (turbine - compressor) / w;
            boolean starter = ignition >= 0.5 && !running && n < STARTER_CUTOUT;   // turns the spool to light-off and helps it up to cut-out
            if (starter) { torque += c.starterTorqueNm(); }
            double slope = -(turbine + compressor) / (w * w);
            step.rotorTorque(0, torque, slope, false);
            double thrust = drowned ? 0 : p.thrustN() * share;
            step.force(new AppliedForce(AppliedForce.Kind.THRUST, step.offset(new Vec3d(-halfLength, 0, 0)), axis.scale(thrust)));
            step.setInternal(STATE_FUEL, fuel);
            step.setInternal(STATE_LIT, running ? 2 : lit ? 1 : 0);
            step.setInternal(STATE_SURGE, surge);
            step.diagnostic("speedFraction", n);
            step.diagnostic("rpm", omega * 30 / Math.PI);
            step.diagnostic("thrustN", thrust);
            step.diagnostic("fuelKgPerS", fuel);
            step.diagnostic("turbineInletK", p.tt4());
            step.diagnostic("exhaustK", p.tt5());
            step.diagnostic("pressureRatio", p.pressureRatio());
            step.diagnostic("airflowKgPerS", p.airflowKgPerS());
            step.diagnostic("surgeMarginUsed", p.surgeMarginUsed());
            step.diagnostic("lit", lit ? 1 : 0);
            step.diagnostic("starter", starter ? 1 : 0);
            step.diagnostic("surging", surge > 0 ? 1 : 0);
        }
    }
}
