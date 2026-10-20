package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * One substep of a part-built vehicle, split like the other integrators so several bodies can share a contact solve:
 * prepare() asks every part for its loads at the step's start (propellers first, so control surfaces see their
 * slipstream), adds the vehicle's weight, the heave wave-radiation memory and the shaft trains, and makes one
 * implicit velocity update with every part's added mass and drag together; finish() drifts body plus entrained water
 * from the post-contact state, separates it from obstacles and commits the new state.
 *
 * <p>Shaft trains: each train is one degree of freedom; its root speed advances by backward Euler on the reflected
 * torques, I_eff = sum I_r k_r^2 and tau_eff = sum k_r e_r tau_r (k_r the rotor's speed over the train's, e_r its gear
 * efficiency: load torques reflected toward the root are divided by it, driving ones multiplied). The vehicle feels the
 * reaction -sum (I_r alpha_r - tau_external,r) along each rotor axis, so a motor turning a propeller passes the
 * propeller's torque to the hull through its mounting, and a spinning-up rotor kicks the hull the other way.
 * When the vehicle turns at omega each rotor's spin momentum H swings, so the body takes -omega x H (0.22). Freewheel shafts
 * (one-way clutches) split trains that lock together while the driving side keeps up (0.22).
 *
 * <p>Drivetrain (0.24): parts may join trains with rows (gears, clutches, brakes, differentials; PartStep.row) and
 * couple them smoothly (a viscous coupling, a torque converter); then the trains are solved together with the rows
 * (Drivetrain), freewheel shafts becoming one-way rows. A step without rows or couplings takes the trains alone as
 * before, to the last bit. Rolling forces (a tyre on the ground) act on a rotor with their slope in that solve and
 * then on the body at the end speed; line dampers (a tyre's or a suspension's damping) join the implicit velocity
 * update. After the contact solve every part's after() sees the end state (tyre deflection, brake heat).
 */
public final class VehicleDynamics {
    private VehicleDynamics() {}

    /** Accumulates the parts' contributions during prepare(). */
    static final class Collector {
        final List<AppliedForce> display = new ArrayList<>();
        final EnumMap<AppliedForce.Kind, Wrench> sums = new EnumMap<>(AppliedForce.Kind.class);
        Wrench inertia = Wrench.ZERO;
        double[][] added;
        final List<RigidBodyDynamics.PointMass> masses = new ArrayList<>();
        final List<Vec3d> excitations = new ArrayList<>();
        final List<RigidBodyDynamics.PointDrag> drags = new ArrayList<>();
        final double[] internal, external, derivative, speeds, lastRadii, radii, shares, internals, nextInternals;
        final Map<String, double[]> thrusts = new HashMap<>();
        final Map<String, Map<String, Double>> diagnostics = new LinkedHashMap<>();
        /** A drivetrain row in rotor space (PartStep.row): owner is the part's installed index; boundRow a global row index or -1. */
        record Row(int owner, int[] rotors, double[] coefficients, double target, double low, double high, double efficiency, int boundRow,
                   double boundBase, double boundPer) {}
        /** An off-diagonal torque slope: dT(rotor)/domega(of). */
        record Cross(int rotor, int of, double slope) {}
        /** A rolling force (PartStep.rollingForce): its torque and slope are already on the rotor. */
        record Rolling(int owner, int rotor, Vec3d offset, Vec3d direction, double force, double slope) {}
        final List<Row> rows = new ArrayList<>();
        final List<Cross> cross = new ArrayList<>();
        final List<Rolling> rolling = new ArrayList<>();
        final List<RigidBodyDynamics.LineDamper> dampers = new ArrayList<>();
        final double[] shown;
        final Object[] carried;
        /** Fuel asked for this step (0.26): a part, the fuels it takes, kg. */
        record Burn(int part, java.util.Set<String> fuels, double kg) {}
        final List<Burn> burns = new ArrayList<>();
        /** Parts that asked to be cast off this step (0.28), by id. */
        final java.util.Set<String> jettisons = new java.util.LinkedHashSet<>();
        final double[] supply;
        /** 0.33: the wheels' reports in the probe pass, what parts kept for their loads, the frame's start and end twists (null: no frame). */
        final List<FrameFlex.Corner> corners = new ArrayList<>();
        final Object[] kept;
        double[] flexX, flexStart, flexEnd;
        boolean probing;

        Collector(Vehicle v, VehicleState st) {
            int n = v.rotors().size();
            internal = new double[n]; external = new double[n]; derivative = new double[n]; speeds = new double[n];
            for (int k = 0; k < n; k++) { speeds[k] = st.rotorSpeed(k); }
            lastRadii = st.radii.clone();
            radii = new double[v.radiationPoints().size()];
            shares = new double[radii.length];
            java.util.Arrays.fill(shares, 1);
            internals = st.internals.clone(); nextInternals = st.internals.clone();
            shown = st.shown.clone();
            carried = new Object[v.installed().size()];
            kept = new Object[v.installed().size()];
            supply = st.supply.clone();
        }

        double rotorSpeed(int k) { return speeds[k]; }
        double lastRadius(int k) { return lastRadii[k]; }
        double[] thrust(String id) { return thrusts.get(id); }

        void force(AppliedForce f, boolean load) {
            display.add(f);
            if (load) { sums.merge(f.kind(), f.wrench(), Wrench::add); }
        }
        void wrench(Wrench w, AppliedForce.Kind kind) { sums.merge(kind, w, Wrench::add); }
        void inertia(Wrench w) { inertia = inertia.add(w); }
        void addedMass(double[][] world) {
            if (added == null) { added = new double[6][6]; }
            for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { added[i][j] += world[i][j]; }
        }
        void pointMass(RigidBodyDynamics.PointMass m, Vec3d excitation) { masses.add(m); excitations.add(excitation == null ? Vec3d.ZERO : excitation); }
        void pointDrag(RigidBodyDynamics.PointDrag d) { drags.add(d); }
        void rotorTorque(int k, double torque, double perRadPerSecond, boolean outside) {
            if (!Double.isFinite(torque) || !Double.isFinite(perRadPerSecond)) { throw new IllegalArgumentException("Rotor torque must be finite"); }
            if (outside) { external[k] += torque; } else { internal[k] += torque; }
            derivative[k] += perRadPerSecond;
        }
        void radiation(int k, double radius, double share) { radii[k] = Math.max(0, radius); shares[k] = Math.clamp(share, 0, 1); }
        void thrust(String id, double water, double air, double diameter) { thrusts.put(id, new double[]{water, air, diameter}); }
        void diagnostic(String id, String key, double value) { diagnostics.computeIfAbsent(id, k -> new LinkedHashMap<>()).put(key, value); }
    }

    /**
     * The first half of a step. Loads come from state and environment at the step's start (time, s); controls map
     * channel names to values; external are further loads (people standing on deck, a tow line), world offsets from the COM.
     */
    public record Prepared(Vehicle vehicle, VehicleState state, RigidBodyState start, RigidBodyState damped, List<AppliedForce> forces,
                           Map<AppliedForce.Kind, Wrench> loads, Wrench inertia, List<RigidBodyDynamics.PointMass> masses,
                           double[] trainSpeeds, double[] radii, Map<String, Map<String, Double>> diagnostics, double dt, double[] internals,
                           Drive drive, List<Collector.Burn> burns, java.util.Set<String> jettisons) {}

    /**
     * What the drivetrain left for Part.after (0.24): every row's torque (N m) and every rolling force's end value (N)
     * in the order the parts made them, with each part's (by installed index) first one and count; what each part
     * carried; the shown values after loads(); the rotors' start speeds; the step's time; the pivoting steps of the
     * solve and whether it fell back to Gauss-Seidel (with no rows: 0, false).
     */
    public record Drive(double[] rowTorques, int[] firstRow, int[] rows, double[] rollingForces, int[] firstRolling, int[] rolling,
                        Object[] carried, double[] shown, double[] startRotorSpeeds, double time, int iterations, boolean fallback) {}

    /**
     * A finished step: new state, every load drawn (world offsets from the step's starting COM), the loads summed by
     * kind (weight, buoyancy, drag, lift, thrust, radiation; inertia holds added-mass reactions, wave excitation and
     * entrained-water exchange), contact, part diagnostics and frictional contact heat (W).
     */
    public record Result(RigidBodyState state, List<AppliedForce> forces, Map<AppliedForce.Kind, Wrench> loads, Wrench inertia,
                         Wrench contact, Map<String, Map<String, Double>> diagnostics, double contactFrictionW) {
        public Wrench load(AppliedForce.Kind kind) { return loads.getOrDefault(kind, Wrench.ZERO); }
    }

    /**
     * One-way clutches: trains start apart; a freewheel engages when, stepped apart, its driving side would turn at
     * least as fast as its driven side (factor x speed), and the two are then stepped as one (the driven train's
     * inertia and torques reflected by the factor, its loads needing 1/efficiency more). Repeated until no more engage,
     * so chains of freewheels work.
     */
    private static void freewheels(Vehicle v, double[] start, double[] speeds, double[] inertia, double[] torque, double[] stiffness, double dt) {
        int n = v.trains();
        int[] root = new int[n];
        double[] factor = new double[n], eff = new double[n];
        for (int t = 0; t < n; t++) { root[t] = t; factor[t] = 1; eff[t] = 1; }
        boolean changed = true;
        for (int pass = 0; changed && pass <= v.freewheels().size(); pass++) {
            changed = false;
            for (Vehicle.Freewheel f : v.freewheels()) {
                int a = root[f.from()], b = root[f.to()];
                if (a == b) { continue; }
                double driving = f.factor() * speeds[f.from()], driven = speeds[f.to()];
                // engaged when the driving side, stepped alone, would not fall behind (for either sense of turning)
                if (Math.signum(driving) == 0 || (driving - driven) * Math.signum(driving) < -1e-9) { continue; }
                // trains in b's group move to a's group: speed_t = factor_t x speed_root
                double k = f.factor() * factor[f.from()] / factor[f.to()], e = f.efficiency() * eff[f.from()] / eff[f.to()];
                for (int t = 0; t < n; t++) { if (root[t] == b) { root[t] = a; factor[t] *= k; eff[t] *= e; } }
                double sumI = 0, sumT = 0, sumK = 0;
                for (int t = 0; t < n; t++) {
                    if (root[t] != a) { continue; }
                    double load = factor[t] * torque[t];
                    double w = load >= 0 ? eff[t] : 1 / eff[t];
                    sumI += factor[t] * factor[t] * inertia[t];
                    sumT += w * load;
                    sumK += factor[t] * factor[t] * w * stiffness[t];
                }
                // engaging is a rigid lock: the group starts from the momentum the trains bring (reflected to the root)
                double momentum = 0;
                for (int t = 0; t < n; t++) { if (root[t] == a) { momentum += factor[t] * inertia[t] * start[t]; } }
                double speed = momentum / sumI + dt * sumT / (sumI - dt * Math.min(sumK, 0));
                for (int t = 0; t < n; t++) { if (root[t] == a) { speeds[t] = factor[t] * speed; } }
                changed = true;
            }
        }
    }

    public static Prepared prepare(Vehicle v, VehicleState st, VehicleEnvironment environment, Map<String, Double> controls, Vec3d gravity,
                                   List<AppliedForce> external, double time, double dt) {
        if (st.vehicle() != v) { throw new IllegalArgumentException("State belongs to another vehicle"); }
        if (Math.abs(dt - st.dt()) > 1e-12) { throw new IllegalArgumentException("Step " + dt + " s differs from the state's substep " + st.dt() + " s"); }
        RigidBodyState s = st.body();
        RigidBodyProperties p = v.properties();
        Vec3d omega = s.angularVelocity(p);
        Collector c = new Collector(v, st);
        c.force(new AppliedForce(AppliedForce.Kind.WEIGHT, Vec3d.ZERO, gravity.scale(p.massKg())), true);
        // fuel in the tanks: point masses with their weight (0.26)
        for (int k = 0; k < v.tanks().size(); k++) {
            double kg = st.tankKg[k];
            if (!(kg > 0)) { continue; }
            Vec3d at = s.worldOffset(p, v.tanks().get(k).centre());
            c.pointMass(new RigidBodyDynamics.PointMass(at, kg), null);
            c.force(new AppliedForce(AppliedForce.Kind.WEIGHT, at, gravity.scale(kg)), true);
        }
        // 0.33: a twisting frame: probe the wheels at the start twist, solve the stations' twists, then the loads follow them
        Vehicle.Flex flex = v.flex();
        if (flex != null) {
            Vehicle.Installed frame = v.installed().get(flex.part());
            int n = flex.stationX().length;
            double[] start = new double[n];
            System.arraycopy(st.internals, frame.firstInternal(), start, 0, n);
            c.flexX = flex.stationX(); c.flexStart = start; c.flexEnd = start;
            c.probing = true;
            for (Vehicle.Installed installed : v.installed()) {
                if (installed.part().probes()) { installed.part().probe(new PartStep(c, installed, s, p, omega, environment, controls, gravity, time, dt)); }
            }
            c.probing = false;
            double[] end = FrameFlex.solve(flex, start, c.corners);
            for (double t : end) { if (!Double.isFinite(t)) { throw new IllegalStateException("The frame's twist is not finite"); } }
            c.flexEnd = end;
            System.arraycopy(end, 0, c.nextInternals, frame.firstInternal(), n);
        }
        for (int pass = 0; pass < 2; pass++) {
            for (Vehicle.Installed installed : v.installed()) {
                if (installed.part().early() != (pass == 0)) { continue; }
                installed.part().loads(new PartStep(c, installed, s, p, omega, environment, controls, gravity, time, dt));
            }
        }
        // Heave radiation memory (point absorbers of all parts together), each point's force scaled by its part's share.
        // 0.30: a vehicle with no part in the water (every share 0) gets no radiation force: not computed
        boolean wet = false;
        for (double share : c.shares) { wet |= share > 0; }
        if (st.radiation != null && wet) {
            double g = gravity.length();
            Vec3d up = g > 0 ? gravity.scale(-1 / g) : Vec3d.Y;
            double rho = environment.point(s.position()).water().densityKgPerM3();
            double[] forces = st.radiation.verticalForces(rho);
            for (int k = 0; k < forces.length; k++) {
                double f = forces[k] * c.shares[k];
                if (f != 0) { c.force(new AppliedForce(AppliedForce.Kind.RADIATION, s.worldOffset(p, v.radiationPoints().get(k)), up.scale(f)), true); }
            }
        }
        for (AppliedForce f : external) { c.force(f, true); }
        // Shaft trains, and the reaction of the vehicle's own torques on the rotors.
        double[] trainSpeeds = st.trainSpeeds.clone();
        List<Vehicle.Rotor> rotors = v.rotors();
        double[] rowTorques = new double[c.rows.size()];
        int iterations = 0;
        boolean fallback = false;
        if (!rotors.isEmpty()) {
            double[] inertiaSum = new double[v.trains()], torque = new double[v.trains()], stiffness = new double[v.trains()], reflect = new double[rotors.size()];
            for (int k = 0; k < rotors.size(); k++) {
                Vehicle.Rotor r = rotors.get(k);
                double tau = c.internal[k] + c.external[k], toward = tau * r.ratio() * Math.signum(trainSpeeds[r.train()] == 0 ? 1 : trainSpeeds[r.train()]);
                // Driving torques pass through the gears with efficiency e, loads need 1/e more at the root.
                double e = r.driver() ? 1 : toward >= 0 ? r.efficiency() : 1 / r.efficiency();
                reflect[k] = e;
                inertiaSum[r.train()] += r.inertiaKgM2() * r.ratio() * r.ratio();
                torque[r.train()] += r.ratio() * e * tau;
                stiffness[r.train()] += r.ratio() * r.ratio() * e * c.derivative[k];
            }
            if (c.rows.isEmpty() && c.cross.isEmpty()) {
                for (int t = 0; t < v.trains(); t++) {
                    // Backward Euler with only the damping part of the torque slope (a rising slope is taken explicitly).
                    trainSpeeds[t] += dt * torque[t] / (inertiaSum[t] - dt * Math.min(stiffness[t], 0));
                }
                if (!v.freewheels().isEmpty()) { freewheels(v, st.trainSpeeds, trainSpeeds, inertiaSum, torque, stiffness, dt); }
            } else {
                Drivetrain.Solution solved = drivetrain(v, c, st.trainSpeeds, inertiaSum, torque, stiffness, reflect, dt);
                trainSpeeds = solved.speeds();
                System.arraycopy(solved.torques(), 0, rowTorques, 0, rowTorques.length);
                iterations = solved.iterations();
                fallback = solved.fallback();
            }
            for (int k = 0; k < rotors.size(); k++) {
                Vehicle.Rotor r = rotors.get(k);
                double alpha = r.ratio() * (trainSpeeds[r.train()] - st.trainSpeeds[r.train()]) / dt;
                double onRotor = r.momentumKgM2() * alpha - c.external[k];
                if (onRotor == 0) { continue; }
                Vec3d axis = s.orientation().rotate(r.axis());
                c.force(new AppliedForce(AppliedForce.Kind.THRUST, s.worldOffset(p, r.position()), Vec3d.ZERO, axis.scale(-onRotor)), true);
            }
            // The gyroscopic reaction (the body turning at omega swings each rotor's spin momentum H, which needs omega x H)
            // is taken in the drift in finish, with the rotation (RigidBodyDynamics.drift of a gyrostat).
        }
        // Rolling forces at the rotors' end speeds, on the body at their contact points.
        double[] rollingForces = new double[c.rolling.size()];
        for (int i = 0; i < rollingForces.length; i++) {
            Collector.Rolling r = c.rolling.get(i);
            Vehicle.Rotor rotor = rotors.get(r.rotor());
            double force = r.force() + r.slope() * (rotor.ratio() * trainSpeeds[rotor.train()] - c.speeds[r.rotor()]);
            rollingForces[i] = force;
            if (force != 0) { c.force(new AppliedForce(AppliedForce.Kind.CONTACT, r.offset(), r.direction().scale(force)), true); }
        }
        Wrench load = c.inertia;
        for (Wrench w : c.sums.values()) { load = load.add(w); }
        Wrench excitation = Wrench.ZERO;
        for (int i = 0; i < c.masses.size(); i++) { excitation = excitation.add(Wrench.atOffset(c.excitations.get(i), c.masses.get(i).offset())); }
        load = load.add(excitation);
        RigidBodyState damped = c.dampers.isEmpty() ? RigidBodyDynamics.implicitVelocity(s, p, load, c.added, c.drags, c.masses, dt)
                : RigidBodyDynamics.implicitVelocity(s, p, load, c.added, c.drags, c.masses, c.dampers, dt);
        // Implicit drag forces (frozen coefficients, as the solve used them) for arrows and the drag sum.
        RigidBodyState kicked = RigidBodyDynamics.kick(s, p, load, dt);
        Vec3d kickedOmega = kicked.angularVelocity(p), afterOmega = damped.angularVelocity(p);
        for (RigidBodyDynamics.PointDrag d : c.drags) {
            Vec3d r = d.offset();
            double speed = kicked.velocity().add(kickedOmega.cross(r)).subtract(d.fluidVelocity()).length();
            double coefficient = d.linearResistance() + d.quadraticResistance() * speed;
            Vec3d relative = damped.velocity().add(afterOmega.cross(r)).subtract(d.fluidVelocity());
            if (coefficient > 0) { c.force(new AppliedForce(AppliedForce.Kind.DRAG, r, relative.scale(-coefficient)), true); }
        }
        // Line dampers at the velocity the solve gave them (ground contact: a tyre's slip, a suspension's damping).
        for (RigidBodyDynamics.LineDamper d : c.dampers) {
            Vec3d force = d.force(damped.velocity().add(afterOmega.cross(d.offset())));
            if (force.lengthSquared() > 0) { c.force(new AppliedForce(AppliedForce.Kind.CONTACT, d.offset(), force), true); }
        }
        // Inertia: excitation and ideal-fluid terms, minus the added masses' reaction to this step's acceleration.
        Vec3d omega0 = omega, dv = damped.velocity().subtract(s.velocity()), dw = afterOmega.subtract(omega0);
        Wrench inertia = c.inertia.add(excitation);
        if (c.added != null) {
            double[] change = {dv.x(), dv.y(), dv.z(), dw.x(), dw.y(), dw.z()}, reaction = new double[6];
            for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { reaction[i] -= c.added[i][j] * change[j] / dt; }
            inertia = inertia.add(new Wrench(new Vec3d(reaction[0], reaction[1], reaction[2]), new Vec3d(reaction[3], reaction[4], reaction[5])));
        }
        for (RigidBodyDynamics.PointMass m : c.masses) {
            Vec3d r = m.offset(), a = damped.velocity().add(afterOmega.cross(r)).subtract(s.velocity().add(omega0.cross(r))).scale(1 / dt);
            inertia = inertia.add(Wrench.atOffset(a.scale(-m.massKg()), r));
        }
        if (inertia.force().lengthSquared() > 0 || inertia.torque().lengthSquared() > 0) {
            c.display.add(new AppliedForce(AppliedForce.Kind.INERTIA, Vec3d.ZERO, inertia.force(), inertia.torque()));
        }
        int n = v.installed().size();
        int[] firstRow = new int[n], rowCount = new int[n], firstRolling = new int[n], rollingCount = new int[n];
        java.util.Arrays.fill(firstRow, -1);
        java.util.Arrays.fill(firstRolling, -1);
        for (int i = 0; i < c.rows.size(); i++) {
            int owner = c.rows.get(i).owner();
            if (firstRow[owner] < 0) { firstRow[owner] = i; }
            rowCount[owner]++;
        }
        for (int i = 0; i < c.rolling.size(); i++) {
            int owner = c.rolling.get(i).owner();
            if (firstRolling[owner] < 0) { firstRolling[owner] = i; }
            rollingCount[owner]++;
        }
        Drive drive = new Drive(rowTorques, firstRow, rowCount, rollingForces, firstRolling, rollingCount, c.carried, c.shown, c.speeds, time,
                iterations, fallback);
        return new Prepared(v, st, s, damped, List.copyOf(c.display), new EnumMap<>(c.sums), inertia, List.copyOf(c.masses),
                trainSpeeds, c.radii, c.diagnostics, dt, c.nextInternals, drive, List.copyOf(c.burns), java.util.Set.copyOf(c.jettisons));
    }

    /**
     * The trains with the parts' rows, couplings and the freewheel shafts (one-way rows) solved together (Drivetrain).
     * Rotor-space rows become train-space ones: a rotor's coefficient times its ratio, its path efficiency for the
     * losses (1 for a driver, as its torques); couplings' cross slopes are reflected like the trains' own.
     */
    private static Drivetrain.Solution drivetrain(Vehicle v, Collector c, double[] start, double[] inertia, double[] torque, double[] stiffness,
                                                  double[] reflect, double dt) {
        int trains = v.trains();
        List<Vehicle.Rotor> rotors = v.rotors();
        double[][] slopes = new double[trains][trains];
        for (int t = 0; t < trains; t++) { slopes[t][t] = stiffness[t]; }
        for (Collector.Cross x : c.cross) {
            Vehicle.Rotor a = rotors.get(x.rotor()), b = rotors.get(x.of());
            slopes[a.train()][b.train()] += a.ratio() * reflect[x.rotor()] * x.slope() * b.ratio();
        }
        List<Drivetrain.Row> rows = new ArrayList<>();
        for (Collector.Row r : c.rows) {
            int m = r.rotors().length;
            int[] at = new int[m];
            double[] coefficients = new double[m], path = new double[m];
            for (int e = 0; e < m; e++) {
                Vehicle.Rotor rotor = rotors.get(r.rotors()[e]);
                at[e] = rotor.train();
                coefficients[e] = r.coefficients()[e] * rotor.ratio();
                path[e] = rotor.driver() ? 1 : rotor.efficiency();
            }
            rows.add(new Drivetrain.Row(at, coefficients, path, r.target(), r.low(), r.high(), r.efficiency(), r.boundRow(), r.boundBase(), r.boundPer()));
        }
        // a freewheel shaft: the driven train may not fall behind factor x the driving one, in the sense the driving one
        // turns when stepped alone (as the trains alone engage it)
        for (Vehicle.Freewheel f : v.freewheels()) {
            int t = f.from();
            double alone = start[t] + dt * torque[t] / (inertia[t] - dt * Math.min(stiffness[t], 0));
            double sense = Math.signum(f.factor() * alone);
            if (sense == 0) { continue; }
            rows.add(new Drivetrain.Row(new int[]{f.to(), f.from()}, new double[]{sense, -sense * f.factor()}, new double[]{1, 1}, 0, 0,
                    Double.POSITIVE_INFINITY, f.efficiency(), -1, 0, 0));
        }
        return Drivetrain.solve(inertia, torque, slopes, start, rows, dt);
    }

    /**
     * The second half: drift of body plus entrained water from the post-contact state, separation from collision's
     * static obstacles (when not null), then the state is committed (body, train speeds, wave memory).
     */
    public static Result finish(Prepared prepared, RigidBodyState afterContact, Wrench contact, List<AppliedForce> contactForces,
                                double contactFrictionW, FloatingBodyDynamics.Collision collision) {
        Vehicle v = prepared.vehicle();
        VehicleState st = prepared.state();
        RigidBodyProperties p = v.properties();
        List<AppliedForce> forces = new ArrayList<>(prepared.forces());
        forces.addAll(contactForces);
        Wrench inertia = prepared.inertia();
        // the rotors' spin momentum at this step's train speeds rides with the body through the drift (gyroscopic couples)
        Vec3d spin = Vec3d.ZERO;
        for (Vehicle.Rotor r : v.rotors()) {
            double speed = r.ratio() * prepared.trainSpeeds()[r.train()];
            if (speed != 0) { spin = spin.add(r.axis().scale(r.momentumKgM2() * speed)); }
        }
        RigidBodyState next = RigidBodyDynamics.driftWithPointMasses(afterContact, p, prepared.masses(), spin, prepared.dt());
        Vec3d exchangeForce = next.velocity().subtract(afterContact.velocity()).scale(p.massKg() / prepared.dt());
        Vec3d exchangeCouple = next.angularMomentum().subtract(afterContact.angularMomentum()).scale(1 / prepared.dt());
        if (exchangeForce.lengthSquared() > 0 || exchangeCouple.lengthSquared() > 0) {
            AppliedForce exchange = new AppliedForce(AppliedForce.Kind.INERTIA, Vec3d.ZERO, exchangeForce, exchangeCouple);
            forces.add(exchange);
            inertia = inertia.add(exchange.wrench());
        }
        if (collision != null) { next = ContactSolver.separate(next, p, collision.shape(), collision.obstacles()); }
        st.setBody(next);
        System.arraycopy(prepared.trainSpeeds(), 0, st.trainSpeeds, 0, st.trainSpeeds.length);
        System.arraycopy(prepared.internals(), 0, st.internals, 0, st.internals.length);
        System.arraycopy(prepared.drive().shown(), 0, st.shown, 0, st.shown.length);
        drawFuel(v, st, prepared.burns());
        st.jettisons.addAll(prepared.jettisons());
        for (Vehicle.Installed i : v.installed()) { i.part().after(new PartAfter(prepared, i, st, next)); }
        if (st.radiation != null) {
            // The first step starts the waterplane radii where the vehicle floats (Memory.prime), not at zero.
            if (!st.primed) { st.radiation.prime(prepared.radii()); st.primed = true; }
            st.radiation.recordAt(next, p, v.radiationPoints(), Vec3d.Y, prepared.radii());
            System.arraycopy(prepared.radii(), 0, st.radii, 0, st.radii.length);
        }
        st.forces = List.copyOf(forces);
        st.diagnostics = prepared.diagnostics();
        return new Result(next, st.forces, prepared.loads(), inertia, contact, prepared.diagnostics(), contactFrictionW);
    }

    /**
     * The fuel the parts asked for this step (0.26), from the tanks holding a fuel each takes, in proportion to their
     * contents; each part's supply is what it got over what it asked. A vehicle with no tanks supplies everything.
     * A part with own tanks (0.28) draws only on them, and nobody else does.
     */
    static void drawFuel(Vehicle v, VehicleState st, List<Collector.Burn> burns) {
        java.util.Arrays.fill(st.supply, 1);
        if (v.tanks().isEmpty() || burns.isEmpty()) { return; }
        double[] asked = new double[st.supply.length], got = new double[st.supply.length];
        boolean[] owns = new boolean[st.supply.length];
        for (Vehicle.Tank t : v.tanks()) { if (t.tank().own()) { owns[t.part()] = true; } }
        for (Collector.Burn b : burns) {
            double have = 0;
            for (int k = 0; k < st.tankKg.length; k++) { if (feeds(v.tanks().get(k), owns, b, st, k)) { have += st.tankKg[k]; } }
            double take = Math.min(b.kg(), have);
            if (take > 0) {
                for (int k = 0; k < st.tankKg.length; k++) {
                    if (feeds(v.tanks().get(k), owns, b, st, k)) {
                        st.tankKg[k] -= take * st.tankKg[k] / have;
                        if (st.tankKg[k] < 1e-9) { st.tankKg[k] = 0; st.tankFuel[k] = ""; }
                    }
                }
            }
            asked[b.part()] += b.kg();
            got[b.part()] += take;
        }
        for (int i = 0; i < asked.length; i++) { if (asked[i] > 0) { st.supply[i] = got[i] / asked[i]; } }
    }

    /** Whether tank k feeds a burn: it holds one of its fuels, and is the part's own tank exactly when the part has own tanks. */
    private static boolean feeds(Vehicle.Tank tank, boolean[] owns, Collector.Burn b, VehicleState st, int k) {
        if (!(st.tankKg[k] > 0) || !b.fuels().contains(st.tankFuel[k])) { return false; }
        return owns[b.part()] ? tank.tank().own() && tank.part() == b.part() : !tank.tank().own();
    }

    /** prepare() and finish() with this body's own contact solve against collision's obstacles (none when null). */
    public static Result step(Vehicle v, VehicleState st, VehicleEnvironment environment, Map<String, Double> controls, Vec3d gravity,
                              List<AppliedForce> external, FloatingBodyDynamics.Collision collision, double time, double dt) {
        Prepared prepared = prepare(v, st, environment, controls, gravity, external, time, dt);
        if (collision == null) { return finish(prepared, prepared.damped(), Wrench.ZERO, List.of(), 0, null); }
        List<ContactSolver.Contact> contacts = ContactSolver.generate(prepared.damped(), v.properties(), collision.shape(), collision.obstacles(), dt);
        ContactSolver.Result solved = ContactSolver.solve(prepared.damped(), v.properties(), contacts, collision.material(), dt);
        return finish(prepared, solved.state(), solved.wrench(), solved.forces(), solved.frictionDissipationW(), collision);
    }
}
