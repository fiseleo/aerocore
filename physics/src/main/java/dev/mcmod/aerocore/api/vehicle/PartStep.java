package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;

import java.util.Map;

/**
 * One part's view of a step (VehicleDynamics.prepare): what it reads (the vehicle's start state, the environment,
 * its inputs, rotor speeds) and where it writes its loads. All world quantities: forces in N with offsets from the
 * vehicle's COM, torques about the COM. The part frame view (partState, partProperties) is the same rigid body seen
 * with the part's axes, so models written for a whole body in its own frame (a hull) can be evaluated for a mounted
 * part unchanged.
 */
public final class PartStep {
    private final VehicleDynamics.Collector out;
    private final Vehicle.Installed installed;
    private final RigidBodyState state, partState;
    private final RigidBodyProperties properties;
    private final Vec3d omega, gravity;
    private final VehicleEnvironment environment;
    private final Map<String, Double> controls;
    private final double time, dt;
    /** Where this part's rows and rolling forces start in the collector's lists (they are contiguous: one loads() call). */
    private final int firstRow, firstRolling;
    /** Inputs set by the part itself for the rest of its loads() (override). */
    private Map<String, Double> overrides;

    PartStep(VehicleDynamics.Collector out, Vehicle.Installed installed, RigidBodyState state, RigidBodyProperties properties, Vec3d omega,
             VehicleEnvironment environment, Map<String, Double> controls, Vec3d gravity, double time, double dt) {
        this.out = out; this.installed = installed; this.state = state; this.properties = properties; this.omega = omega;
        this.environment = environment; this.controls = controls; this.gravity = gravity; this.time = time; this.dt = dt;
        Orientation q = state.orientation().multiply(installed.rotation());
        // Same COM point and velocity; the angular momentum is a world vector, unchanged by the choice of body axes.
        this.partState = new RigidBodyState(state.position(), state.velocity(), q, state.angularMomentum());
        this.firstRow = out.rows.size();
        this.firstRolling = out.rolling.size();
    }

    // ----- reading -----
    public String id() { return installed.id(); }
    public Mount mount() { return installed.mount(); }
    public RigidBodyState state() { return state; }
    public RigidBodyProperties properties() { return properties; }
    public Vec3d angularVelocity() { return omega; }
    public VehicleEnvironment environment() { return environment; }
    public Vec3d gravity() { return gravity; }
    public double time() { return time; }
    public double dt() { return dt; }

    /** The vehicle's state with the part's axes (same COM point, velocity and angular momentum). */
    public RigidBodyState partState() { return partState; }
    /** The vehicle's mass, COM and inertia expressed in the part frame, to go with partState. */
    public RigidBodyProperties partProperties() { return installed.partProperties(); }

    /** A part-frame point in the vehicle frame. */
    public Vec3d vehiclePoint(Vec3d partPoint) { return installed.mount().position().add(installed.rotation().rotate(partPoint)); }
    /** World offset from the COM of a part-frame point. */
    public Vec3d offset(Vec3d partPoint) { return state.worldOffset(properties, vehiclePoint(partPoint)); }
    /** World position of a part-frame point. */
    public Vec3d point(Vec3d partPoint) { return state.position().add(offset(partPoint)); }
    /** The part-frame point at a world position (the inverse of point()). */
    public Vec3d partPoint(Vec3d worldPoint) {
        Vec3d local = state.orientation().inverse().rotate(worldPoint.subtract(state.position())).add(properties.centerOfMassLocal());
        return installed.rotation().inverse().rotate(local.subtract(installed.mount().position()));
    }
    /** World direction of a part-frame direction. */
    public Vec3d axis(Vec3d partDirection) { return state.orientation().rotate(installed.rotation().rotate(partDirection)); }
    /** A world direction (or vector) in the part frame. */
    public Vec3d partDirection(Vec3d worldDirection) { return installed.rotation().inverse().rotate(state.orientation().inverse().rotate(worldDirection)); }
    /** World velocity of a part-frame point. */
    public Vec3d velocity(Vec3d partPoint) { return state.velocity().add(omega.cross(offset(partPoint))); }

    /**
     * The input bound to role (offset + scale * channel, summed over bindings), or fallback when nothing is bound; a
     * value the part set with override() comes first.
     */
    public double input(String role, double fallback) {
        if (overrides != null) {
            Double o = overrides.get(role);
            if (o != null) { return o; }
        }
        double sum = 0;
        boolean bound = false;
        for (Binding b : installed.bindings()) {
            if (!b.role().equals(role)) { continue; }
            bound = true;
            Double value = controls.get(b.channel());
            sum += b.offset() + b.scale() * (value == null || !Double.isFinite(value) ? 0 : value);
        }
        return bound ? sum : fallback;
    }

    /**
     * Sets an input for the rest of this part's loads() (0.24): a part that wraps another (a vehicle mod's gearbox around
     * AeroCore's TransmissionPart) turns the driver's inputs into the hardware's commands by its own rules, then lets the
     * inner part read them. Only this step of this part sees it.
     */
    public void override(String role, double value) {
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Input must be finite"); }
        if (overrides == null) { overrides = new java.util.HashMap<>(); }
        overrides.put(role, value);
    }

    /** This part's i-th internal state at the step start (Part.internalStates). */
    public double internal(int i) {
        if (i < 0 || i >= installed.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        return out.internals[installed.firstInternal() + i];
    }
    /** Sets this part's i-th internal state for the end of the step (committed by VehicleDynamics.finish). */
    public void setInternal(int i, double value) {
        if (i < 0 || i >= installed.part().internalStates()) { throw new IndexOutOfBoundsException("Internal state " + i); }
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Internal state must be finite"); }
        out.nextInternals[installed.firstInternal() + i] = value;
    }

    /** Speed of this part's k-th rotor at the step start, rad/s. */
    public double rotorSpeed(int k) { return out.rotorSpeed(installed.firstRotor() + k); }
    /** Waterplane radius last recorded for this part's k-th radiation point (unsmoothed), m. */
    public double lastRadiationRadius(int k) { return out.lastRadius(installed.firstRadiation() + k); }
    /** Thrust in water and in air (N, by the disk's immersion) and diameter (m) of the propeller whose race this part sits in, or null. */
    public double[] slipstream() { return installed.slipstreamFrom() == null ? null : out.thrust(installed.slipstreamFrom()); }

    // ----- writing -----
    /** A load that is also drawn (its kind sorts it in the result's sums). */
    public void force(AppliedForce force) { out.force(force, true); }
    /** A load whose wrench is given in full (for example with a couple or a moment not at one point), drawn as display. */
    public void load(Wrench wrench, AppliedForce.Kind kind) { out.wrench(wrench, kind); }
    /** An arrow for display only; the load must be given by load() as well. */
    public void display(AppliedForce force) { out.force(force, false); }
    /** Wave excitation or ideal-fluid terms, counted with the added-mass reaction as inertia. */
    public void inertia(Wrench wrench) { out.inertia(wrench); }
    /** A 6 x 6 added mass (world axes, about the COM, v then omega), taken implicitly. */
    public void addedMass(double[][] world) { out.addedMass(world); }
    /** An isotropic point added mass (taken implicitly; it also drifts with the body as entrained water) and the excitation force it carries. */
    public void pointMass(RigidBodyDynamics.PointMass mass, Vec3d excitation) { out.pointMass(mass, excitation); }
    /** A point drag, taken implicitly. */
    public void pointDrag(RigidBodyDynamics.PointDrag drag) { out.pointDrag(drag); }
    /**
     * A torque on this part's k-th rotor about its axis (N m) and its derivative with the rotor's speed (N m s/rad).
     * external: from outside the vehicle (a fluid on a propeller); otherwise from the vehicle itself (a motor's stator),
     * whose reaction the vehicle then feels.
     */
    public void rotorTorque(int k, double torqueNm, double perRadPerSecond, boolean external) {
        out.rotorTorque(installed.firstRotor() + k, torqueNm, perRadPerSecond, external);
    }
    /** Waterplane radius (m) of this part's k-th radiation point at the step start, and the share (0..1) of its memory force to apply. */
    public void radiationRadius(int k, double radiusM, double share) { out.radiation(installed.firstRadiation() + k, radiusM, share); }
    /** Publishes this part's thrust in water and in air (N) and its diameter (m) for surfaces in its race. */
    public void thrust(double waterThrustN, double airThrustN, double diameterM) { out.thrust(installed.id(), waterThrustN, airThrustN, diameterM); }
    /** A number shown to players or used by benches (key in camelCase with units, for example "thrustN"). */
    public void diagnostic(String key, double value) { out.diagnostic(installed.id(), key, value); }

    /**
     * Asks the vehicle's tanks for fuel (0.26): kg/s of any of the named fuels through this step, drawn after it from
     * the tanks holding one of them, in proportion to what each holds. fuelSupply() tells next step how much came.
     */
    public void burn(java.util.Collection<String> fuels, double kgPerS) {
        if (!(kgPerS >= 0) || !Double.isFinite(kgPerS)) { throw new IllegalArgumentException("Fuel flow must be finite and nonnegative"); }
        if (kgPerS > 0) { out.burns.add(new VehicleDynamics.Collector.Burn(installed.index(), java.util.Set.copyOf(fuels), kgPerS * dt)); }
    }

    /**
     * Asks to cast this part off the vehicle (0.28: a burnt-out booster). The request waits in the state
     * (VehicleState.takeJettisons) for the game, which rebuilds the vehicle without the part (VehicleState.separate).
     */
    public void jettison() { out.jettisons.add(installed.id()); }

    /** The share of the fuel this part asked for in the last step that it got (1 when it asked none or the vehicle has no tanks). */
    public double fuelSupply() { return out.supply[installed.index()]; }

    // ----- drivetrain (0.24): rows between this part's rotors, solved with the trains (Drivetrain) -----

    /** The vehicle's index of this part's k-th rotor. */
    private int rotor(int k) {
        if (k < 0 || k >= installed.part().rotors().size()) { throw new IndexOutOfBoundsException("Rotor " + k + " of part " + installed.id()); }
        return installed.firstRotor() + k;
    }

    private int add(int[] rotors, double[] coefficients, double target, double low, double high, double efficiency, int boundRow, double boundBase,
                    double boundPer) {
        if (rotors.length == 0 || rotors.length != coefficients.length) { throw new IllegalArgumentException("A row needs one coefficient per rotor"); }
        int[] global = new int[rotors.length];
        for (int i = 0; i < rotors.length; i++) {
            global[i] = rotor(rotors[i]);
            if (!Double.isFinite(coefficients[i])) { throw new IllegalArgumentException("Row coefficients must be finite"); }
        }
        if (!Double.isFinite(target)) { throw new IllegalArgumentException("Row target must be finite"); }
        if (Double.isNaN(low) || Double.isNaN(high) || low > high || low == Double.POSITIVE_INFINITY || high == Double.NEGATIVE_INFINITY) {
            throw new IllegalArgumentException("Row torque bounds must be ordered: " + low + " .. " + high);
        }
        if (!(efficiency > 0 && efficiency <= 1)) { throw new IllegalArgumentException("Row efficiency must be in (0, 1]"); }
        out.rows.add(new VehicleDynamics.Collector.Row(installed.index(), global, coefficients.clone(), target, low, high, efficiency, boundRow, boundBase, boundPer));
        return out.rows.size() - 1 - firstRow;
    }

    /**
     * A row of the drivetrain solve: sum c_i omega_i = target (rad/s; omega_i the end speed of this part's rotor k_i
     * relative to the vehicle) while the row's torque stays within [minimum, maximum] N m; at a bound it slips at that
     * torque. The torque acts on rotor k_i times c_i, the vehicle's body taking the reaction (a gearbox's case, a brake's
     * caliper); efficiency (0..1] is lost on the rotors that take power from the row. Returns this part's handle for
     * the row (PartAfter.rowTorque): 0, 1, ... in the order the part makes them.
     */
    public int row(int[] rotors, double[] coefficients, double targetRadPerSecond, double minimumNm, double maximumNm, double efficiency) {
        return add(rotors, coefficients, targetRadPerSecond, minimumNm, maximumNm, efficiency, -1, 0, 0);
    }

    /** Gears in mesh: omega_a = ratio x omega_b (2 halves b's speed, negative reverses it), with mechanical efficiency. */
    public int gear(int a, int b, double ratio, double efficiency) {
        checkRatio(ratio);
        return row(new int[]{a, b}, new double[]{1, -ratio}, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, efficiency);
    }

    /** A friction clutch: holds omega_a = ratio x omega_b up to capacityNm (on a) and slips at it beyond (0 when open). */
    public int clutch(int a, int b, double ratio, double capacityNm) {
        checkRatio(ratio);
        checkCapacity(capacityNm);
        return row(new int[]{a, b}, new double[]{1, -ratio}, 0, -capacityNm, capacityNm, 1);
    }

    /** A brake (or a parking pawl with infinite capacity): holds rotor k still against the vehicle up to capacityNm. */
    public int hold(int k, double capacityNm) {
        checkCapacity(capacityNm);
        return row(new int[]{k}, new double[]{1}, 0, -capacityNm, capacityNm, 1);
    }

    /**
     * A one-way clutch: rotor from drives rotor to at to = from / ratio in the sense direction (+1 or -1 about from's
     * axis); to may overrun (turn faster that way), never fall behind.
     */
    public int freewheel(int from, int to, double ratio, double efficiency, int direction) {
        checkRatio(ratio);
        if (direction != 1 && direction != -1) { throw new IllegalArgumentException("Freewheel direction is +1 or -1"); }
        return row(new int[]{from, to}, new double[]{-direction / ratio, direction}, 0, 0, Double.POSITIVE_INFINITY, efficiency);
    }

    /**
     * A differential: input = ratio x (shareA x omega_a + (1 - shareA) x omega_b), so the input's torque times ratio
     * divides shareA : 1 - shareA between a and b whatever their speeds (bevel gears 0.5; a planetary centre
     * differential its sun's share). The ratio is the final drive.
     */
    public int differential(int input, int a, int b, double ratio, double shareA, double efficiency) {
        checkRatio(ratio);
        if (!(shareA > 0 && shareA < 1)) { throw new IllegalArgumentException("Differential share must be in (0, 1)"); }
        return row(new int[]{input, a, b}, new double[]{1, -ratio * shareA, -ratio * (1 - shareA)}, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
                efficiency);
    }

    /**
     * Friction between rotors a and b that holds them together up to preloadNm + perNm x |torque of this part's row|
     * and slips beyond: a limited-slip differential (a clutch pack's preload and ramp, a Torsen's bias on its input
     * torque) or a coupling's clutch. Returns its handle.
     */
    public int biasFriction(int a, int b, double preloadNm, double perNm, int row) {
        checkCapacity(preloadNm);
        if (!(perNm >= 0) || !Double.isFinite(perNm)) { throw new IllegalArgumentException("Bias factor must be finite and not negative"); }
        if (row < 0 || firstRow + row >= out.rows.size()) { throw new IndexOutOfBoundsException("Row " + row + " of part " + installed.id()); }
        return add(new int[]{a, b}, new double[]{1, -1}, 0, -preloadNm, preloadNm, 1, firstRow + row, preloadNm, perNm);
    }

    /** A viscous coupling: on a -c s and on b ratio c s for the slip s = omega_a - ratio x omega_b (c in N m s/rad), taken implicitly. */
    public void viscous(int a, int b, double ratio, double coefficient) {
        checkRatio(ratio);
        if (!(coefficient >= 0) || !Double.isFinite(coefficient)) { throw new IllegalArgumentException("Viscous coefficient must be finite and not negative"); }
        double slip = out.rotorSpeed(rotor(a)) - ratio * out.rotorSpeed(rotor(b));
        torquePair(a, b, -coefficient * slip, ratio * coefficient * slip, -coefficient, coefficient * ratio, coefficient * ratio, -coefficient * ratio * ratio);
    }

    /**
     * Torques on two of this part's rotors from inside the vehicle (a torque converter's pump and turbine: the body
     * takes the rest, its stator's reaction) with their slopes dT_a/domega_a, dT_a/domega_b, dT_b/domega_a,
     * dT_b/domega_b (N m s/rad), taken implicitly together.
     */
    public void torquePair(int a, int b, double torqueA, double torqueB, double aByA, double aByB, double bByA, double bByB) {
        int ra = rotor(a), rb = rotor(b);
        if (ra == rb) { throw new IllegalArgumentException("A torque pair acts on two rotors"); }
        for (double v : new double[]{aByB, bByA}) { if (!Double.isFinite(v)) { throw new IllegalArgumentException("Torque slopes must be finite"); } }
        out.rotorTorque(ra, torqueA, aByA, false);
        out.rotorTorque(rb, torqueB, bByB, false);
        if (aByB != 0) { out.cross.add(new VehicleDynamics.Collector.Cross(ra, rb, aByB)); }
        if (bByA != 0) { out.cross.add(new VehicleDynamics.Collector.Cross(rb, ra, bByA)); }
    }

    /**
     * A force from the ground on rotor k's wheel: forceN + perRadPerSecond x (end speed - start speed of the rotor)
     * along the unit world direction at a world offset from the COM (the contact patch), so the drivetrain solve
     * sees its slope. The rotor takes armM times the force about its axis (the signed rolling radius, part's choice of
     * sign) and the body the force at the offset. Returns its handle (PartAfter.rollingForce).
     */
    public int rollingForce(int k, Vec3d offset, Vec3d direction, double forceN, double perRadPerSecond, double armM) {
        int r = rotor(k);
        if (!Double.isFinite(forceN) || !Double.isFinite(perRadPerSecond) || !Double.isFinite(armM)) {
            throw new IllegalArgumentException("Rolling force must be finite");
        }
        out.rotorTorque(r, armM * forceN, armM * perRadPerSecond, false);
        out.rolling.add(new VehicleDynamics.Collector.Rolling(installed.index(), r, offset, direction.normalized(), forceN, perRadPerSecond));
        return out.rolling.size() - 1 - firstRolling;
    }

    /**
     * A damper on the body along a unit world direction at a world offset from the COM (a tyre's or a suspension's
     * damping against the ground): -c (d . u - referenceSpeed) d, taken implicitly (RigidBodyDynamics.LineDamper).
     */
    public void damper(Vec3d offset, Vec3d direction, double coefficient, double referenceSpeed) {
        out.dampers.add(new RigidBodyDynamics.LineDamper(offset, direction, coefficient, referenceSpeed));
    }

    /** 0.30: how many shown values the installed part has (a part wrapping another may show fewer than it). */
    public int shownCount() { return installed.part().shownValues(); }

    /** 0.30: this part's i-th shown value as it stands this step (the last step's until set). */
    public double shown(int i) {
        if (i < 0 || i >= installed.part().shownValues()) { throw new IndexOutOfBoundsException("Shown value " + i); }
        return out.shown[installed.firstShown() + i];
    }

    /**
     * 0.30: a shown value moved in steps of a size, and only once it is three quarters of a step from where it stands:
     * a part at rest then sends clients nothing as its value jitters round a step's edge.
     */
    public static double stepped(double was, double now, double step) {
        return Math.abs(now - was) < 0.75 * step ? was : Math.round(now / step) * step;
    }

    /** Sets this part's i-th shown value (Part.shownValues) for clients. */
    public void show(int i, double value) {
        if (i < 0 || i >= installed.part().shownValues()) { throw new IndexOutOfBoundsException("Shown value " + i); }
        if (!Double.isFinite(value)) { throw new IllegalArgumentException("Shown value must be finite"); }
        out.shown[installed.firstShown() + i] = value;
    }

    /** Keeps something this step worked out (a tyre's contact frames) for this part's after() (PartAfter.carried). */
    public void carry(Object value) { out.carried[installed.index()] = value; }

    // ----- a twisting frame (0.33; FrameFlex) -----

    /** Whether the vehicle has a twisting frame this step (its wheels are probed and follow its twist). */
    public boolean flexing() { return out.flexStart != null; }

    /**
     * In the probe (Part.probe): a wheel's report at a part-frame point (its hub): its force on the body along its
     * travel (N, up) and its stiffness within the step (N/m), at the step's start twist.
     */
    public void corner(Vec3d partPoint, double forceN, double stiffnessNPerM) {
        if (!out.probing) { throw new IllegalStateException("Corners are reported in the probe"); }
        if (!Double.isFinite(forceN) || !(stiffnessNPerM >= 0) || !Double.isFinite(stiffnessNPerM)) { throw new IllegalArgumentException("Corner values must be finite"); }
        Vec3d at = vehiclePoint(partPoint);
        out.corners.add(new FrameFlex.Corner(at.x(), at.z(), forceN, stiffnessNPerM));
    }

    /** In the probe: something for this part's loads() this step (the ground its wheels found). */
    public void keep(Object value) { out.kept[installed.index()] = value; }
    /** What the probe kept (null without a frame or a probe). */
    public Object kept() { return out.kept[installed.index()]; }

    /** How far the frame's twist at the step's start lifts a part-frame point against the rigid body (m along the vehicle's up; 0 without a frame). */
    public double flexLift(Vec3d partPoint) {
        if (out.flexStart == null) { return 0; }
        Vec3d at = vehiclePoint(partPoint);
        return -at.z() * FrameFlex.at(out.flexX, out.flexStart, at.x());
    }

    /** How fast the frame's twist over the step lifts a part-frame point (m/s; 0 without a frame and in the probe). */
    public double flexRate(Vec3d partPoint) {
        if (out.flexStart == null || out.flexEnd == out.flexStart) { return 0; }
        Vec3d at = vehiclePoint(partPoint);
        return -at.z() * (FrameFlex.at(out.flexX, out.flexEnd, at.x()) - FrameFlex.at(out.flexX, out.flexStart, at.x())) / dt;
    }

    private static void checkRatio(double ratio) {
        if (!Double.isFinite(ratio) || ratio == 0) { throw new IllegalArgumentException("Ratio must be finite and not zero"); }
    }

    private static void checkCapacity(double capacityNm) {
        if (!(capacityNm >= 0)) { throw new IllegalArgumentException("Capacity must not be negative"); }
    }
}
