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
 * Rotor gyroscopic moments when the vehicle turns are not modelled.
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

        Collector(Vehicle v, VehicleState st) {
            int n = v.rotors().size();
            internal = new double[n]; external = new double[n]; derivative = new double[n]; speeds = new double[n];
            for (int k = 0; k < n; k++) { speeds[k] = st.rotorSpeed(k); }
            lastRadii = st.radii.clone();
            radii = new double[v.radiationPoints().size()];
            shares = new double[radii.length];
            java.util.Arrays.fill(shares, 1);
            internals = st.internals.clone(); nextInternals = st.internals.clone();
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
                           double[] trainSpeeds, double[] radii, Map<String, Map<String, Double>> diagnostics, double dt, double[] internals) {}

    /**
     * A finished step: new state, every load drawn (world offsets from the step's starting COM), the loads summed by
     * kind (weight, buoyancy, drag, lift, thrust, radiation; inertia holds added-mass reactions, wave excitation and
     * entrained-water exchange), contact, part diagnostics and frictional contact heat (W).
     */
    public record Result(RigidBodyState state, List<AppliedForce> forces, Map<AppliedForce.Kind, Wrench> loads, Wrench inertia,
                         Wrench contact, Map<String, Map<String, Double>> diagnostics, double contactFrictionW) {
        public Wrench load(AppliedForce.Kind kind) { return loads.getOrDefault(kind, Wrench.ZERO); }
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
        for (int pass = 0; pass < 2; pass++) {
            for (Vehicle.Installed installed : v.installed()) {
                if (installed.part().early() != (pass == 0)) { continue; }
                installed.part().loads(new PartStep(c, installed, s, p, omega, environment, controls, gravity, time, dt));
            }
        }
        // Heave radiation memory (point absorbers of all parts together), each point's force scaled by its part's share.
        if (st.radiation != null) {
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
        if (!rotors.isEmpty()) {
            double[] inertiaSum = new double[v.trains()], torque = new double[v.trains()], stiffness = new double[v.trains()];
            for (int k = 0; k < rotors.size(); k++) {
                Vehicle.Rotor r = rotors.get(k);
                double tau = c.internal[k] + c.external[k], toward = tau * r.ratio() * Math.signum(trainSpeeds[r.train()] == 0 ? 1 : trainSpeeds[r.train()]);
                // Driving torques pass through the gears with efficiency e, loads need 1/e more at the root.
                double e = r.driver() ? 1 : toward >= 0 ? r.efficiency() : 1 / r.efficiency();
                inertiaSum[r.train()] += r.inertiaKgM2() * r.ratio() * r.ratio();
                torque[r.train()] += r.ratio() * e * tau;
                stiffness[r.train()] += r.ratio() * r.ratio() * e * c.derivative[k];
            }
            for (int t = 0; t < v.trains(); t++) {
                // Backward Euler with only the damping part of the torque slope (a rising slope is taken explicitly).
                trainSpeeds[t] += dt * torque[t] / (inertiaSum[t] - dt * Math.min(stiffness[t], 0));
            }
            for (int k = 0; k < rotors.size(); k++) {
                Vehicle.Rotor r = rotors.get(k);
                double alpha = r.ratio() * (trainSpeeds[r.train()] - st.trainSpeeds[r.train()]) / dt;
                double onRotor = r.inertiaKgM2() * alpha - c.external[k];
                if (onRotor == 0) { continue; }
                Vec3d axis = s.orientation().rotate(r.axis());
                c.force(new AppliedForce(AppliedForce.Kind.THRUST, s.worldOffset(p, r.position()), Vec3d.ZERO, axis.scale(-onRotor)), true);
            }
        }
        Wrench load = c.inertia;
        for (Wrench w : c.sums.values()) { load = load.add(w); }
        Wrench excitation = Wrench.ZERO;
        for (int i = 0; i < c.masses.size(); i++) { excitation = excitation.add(Wrench.atOffset(c.excitations.get(i), c.masses.get(i).offset())); }
        load = load.add(excitation);
        RigidBodyState damped = RigidBodyDynamics.implicitVelocity(s, p, load, c.added, c.drags, c.masses, dt);
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
        return new Prepared(v, st, s, damped, List.copyOf(c.display), new EnumMap<>(c.sums), inertia, List.copyOf(c.masses),
                trainSpeeds, c.radii, c.diagnostics, dt, c.nextInternals);
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
        RigidBodyState next = RigidBodyDynamics.driftWithPointMasses(afterContact, p, prepared.masses(), prepared.dt());
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
