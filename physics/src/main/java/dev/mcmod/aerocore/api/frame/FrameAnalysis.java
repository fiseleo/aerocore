package dev.mcmod.aerocore.api.frame;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Design-time analyses of a Frame (0.33), linear and static: the twist stiffness between stations (for the physics'
 * frame flex), the bending stiffness, each member's stress under a load case and the standard chassis cases, and mass.
 * <p>Model: 12-freedom Euler–Bernoulli space beams with St Venant torsion, rigidly joined at the nodes, loads at nodes.
 * Left out: shear deformation; an open section's shear-centre offset (every member twists about its centroid); warping,
 * except as each open member's restrained-end torsional stiffness (Frame.torsionNm2); large displacements and buckling.
 * Stress: the axial stress and both bending stresses at the farthest corner, plus an open section's warping stress where
 * its ends hold its warping, with the torsional shear stress (closed: T / (2 A_m t), Bredt; open: the St Venant part of
 * the torque, T t / J free to warp; Section.torsionStress), as von Mises sqrt(σ² + 3 τ²) against the yield strength
 * (the moments, bimoments and torsional shear are each taken at their greatest along the member, which is
 * conservative where they peak apart). Left out of the stress: transverse shear and warping shear, stress
 * concentrations at the joints.
 * <p>Rigid-body motions the supports leave free are held at the node nearest the frame's middle; the loads must leave
 * those motions balanced (a load case checks it), so that node carries nothing.
 */
public final class FrameAnalysis {
    /** Load factors of the standard cases (g). */
    public static final double BUMP_G = 2.5, BRAKING_G = 0.8, CORNERING_G = 0.8;
    /**
     * How far the ground under the port front wheel drops in the ditch case (m). A rule: twist-ditch tests drive one wheel
     * into a pit or onto a ramp of 150 to 200 mm.
     */
    public static final double DITCH_DROP_M = 0.15;
    /** Share of the loads' size a free rigid-body motion may be pushed by before a case counts as unbalanced. */
    private static final double BALANCE = 1e-9;

    /**
     * Where a twist is measured and a torque put on. A pair of nodes across the frame (a ladder frame's rails): the twist
     * is their vertical displacements' difference over their spacing, (v_left - v_right) / (z_right - z_left), positive
     * about +x, and the torque a pair of opposite vertical forces. Or one node on the frame's axis (a backbone): its
     * rotation about x and a moment about x. A pair's nodes stand at the same x (to 1 mm per metre of their spacing),
     * the left (port, smaller z) first.
     */
    public record Station(double xM, int leftNode, int rightNode) {
        public Station {
            Require.finite(xM, "Station x");
            if (leftNode < 0 || rightNode < 0) { throw new IllegalArgumentException("A station's nodes are node indices"); }
        }
        public static Station pair(double xM, int leftNode, int rightNode) {
            if (leftNode == rightNode) { throw new IllegalArgumentException("A pair station has two nodes"); }
            return new Station(xM, leftNode, rightNode);
        }
        public static Station node(double xM, int node) { return new Station(xM, node, node); }
        /** One node (a backbone station). */
        public boolean single() { return leftNode == rightNode; }
    }

    /**
     * Holds a node to the ground with springs along x, y, z (N/m) and about them (N m/rad), in the frame's axes:
     * 0 free, FIXED (positive infinity) rigid.
     */
    public record Support(int node, double xNpm, double yNpm, double zNpm, double rxNmRad, double ryNmRad, double rzNmRad) {
        public static final double FIXED = Double.POSITIVE_INFINITY;

        public Support {
            if (node < 0) { throw new IllegalArgumentException("A support's node is a node index"); }
            for (double r : new double[]{xNpm, yNpm, zNpm, rxNmRad, ryNmRad, rzNmRad}) {
                if (!(r >= 0)) { throw new IllegalArgumentException("A support's rates are 0 (free) or more (FIXED)"); }
            }
        }
        /** Fixed in all six. */
        public static Support fixed(int node) { return new Support(node, FIXED, FIXED, FIXED, FIXED, FIXED, FIXED); }
        /** Fixed in position, free to turn. */
        public static Support pinned(int node) { return new Support(node, FIXED, FIXED, FIXED, 0, 0, 0); }
        /** A vertical spring (N/m; FIXED: held). */
        public static Support vertical(int node, double rateNpm) { return new Support(node, 0, rateNpm, 0, 0, 0, 0); }

        double rate(int freedom) {
            return switch (freedom) {
                case 0 -> xNpm; case 1 -> yNpm; case 2 -> zNpm; case 3 -> rxNmRad; case 4 -> ryNmRad; default -> rzNmRad;
            };
        }
    }

    /** A force (N) and a moment (N m) on a node, in the frame's axes. */
    public record NodeLoad(int node, Vec3d forceN, Vec3d momentNm) {
        public NodeLoad {
            if (node < 0) { throw new IllegalArgumentException("A load's node is a node index"); }
            Objects.requireNonNull(forceN, "force"); Objects.requireNonNull(momentNm, "moment");
        }
        public static NodeLoad force(int node, Vec3d forceN) { return new NodeLoad(node, forceN, Vec3d.ZERO); }
    }

    /** What a joint puts on a member's end, in the member's axes (Frame.axes): force (N) and moment (N m). */
    public record EndForces(Vec3d forceN, Vec3d momentNm) {}

    /** A member's end forces, its greatest von Mises stress (Pa) and that over its yield strength (usage; 1 at yield). */
    public record MemberResult(int member, EndForces atA, EndForces atB, double vonMisesPa, double usage) {
        /** Axial force (N), tension positive. */
        public double axialN() { return atB.forceN().x(); }
        /** Torque about the member's axis (N m). */
        public double torqueNm() { return atB.momentNm().x(); }
    }

    /** What the supports put on a node, in the frame's axes: force (N) and moment (N m). */
    public record Reaction(int node, Vec3d forceN, Vec3d momentNm) {}

    /**
     * A load case's answer: displacements (6 per node by node index: m along x, y, z, then rad about them), the loads put
     * on (the same layout, N and N m), the reactions at the supported nodes and each member's forces and usage.
     */
    public record LoadResult(double[] displacements, double[] loads, List<Reaction> reactions, List<MemberResult> members) {
        public LoadResult {
            displacements = displacements.clone(); loads = loads.clone();
            reactions = List.copyOf(reactions); members = List.copyOf(members);
        }
        @Override public double[] displacements() { return displacements.clone(); }
        @Override public double[] loads() { return loads.clone(); }
        public Vec3d displacement(int node) { return new Vec3d(displacements[node * 6], displacements[node * 6 + 1], displacements[node * 6 + 2]); }
        public Vec3d rotation(int node) { return new Vec3d(displacements[node * 6 + 3], displacements[node * 6 + 4], displacements[node * 6 + 5]); }
        public double maxUsage() { double u = 0; for (MemberResult m : members) { u = Math.max(u, m.usage()); } return u; }
        /** The member with the greatest usage (-1 with none). */
        public int worstMember() {
            int worst = -1;
            for (MemberResult m : members) { if (worst < 0 || m.usage() > members.get(worst).usage()) { worst = m.member(); } }
            return worst;
        }
    }

    /** A carried mass (kg) at a node: engine, body, payload. */
    public record Mass(int node, double kg) {
        public Mass {
            if (node < 0) { throw new IllegalArgumentException("A mass's node is a node index"); }
            Require.nonnegative(kg, "Mass");
        }
    }

    /** A wheel's vertical spring at its mount node (N/m: suspension and tyre in series; FIXED: rigid). */
    public record Wheel(int node, double rateNpm) {
        public Wheel {
            if (node < 0) { throw new IllegalArgumentException("A wheel's node is a node index"); }
            if (!(rateNpm > 0)) { throw new IllegalArgumentException("A wheel's rate must be positive"); }
        }
    }

    /**
     * The standard cases: a 2.5 g bump, a ditch (1 g with the ground under the port front wheel DITCH_DROP_M lower; null
     * with fewer than four wheel nodes), 0.8 g braking and 0.8 g cornering (to port: the load toward starboard), each with
     * 1 g down.
     */
    public record StandardCases(LoadResult bump, LoadResult ditch, LoadResult braking, LoadResult cornering) {
        public double maxUsage() {
            double u = Math.max(bump.maxUsage(), Math.max(braking.maxUsage(), cornering.maxUsage()));
            return ditch != null ? Math.max(u, ditch.maxUsage()) : u;
        }
    }

    private FrameAnalysis() {}

    /** Mass (kg): the members' sections along their lengths (joints, welds and fittings not counted). */
    public static double massKg(Frame frame) {
        double m = 0;
        for (int i = 0; i < frame.memberCount(); i++) { m += frame.member(i).section().kgPerM(frame.member(i).material()) * frame.lengthM(i); }
        return m;
    }

    /** Each node's share of the members' mass (kg, by node index): half of each member at each end. */
    public static double[] nodeMassesKg(Frame frame) {
        double[] out = new double[frame.nodeCount()];
        for (int i = 0; i < frame.memberCount(); i++) {
            Frame.Member m = frame.member(i);
            double half = 0.5 * m.section().kgPerM(m.material()) * frame.lengthM(i);
            out[m.a()] += half; out[m.b()] += half;
        }
        return out;
    }

    /**
     * The stations' twist stiffness K (N m/rad), N x N with the stations in order of x (ties in the list's order): the
     * torques that hold the stations at twists θ are T_s = Σ_t K[s][t] θ_t. Symmetric, positive semi-definite, each row
     * summing to 0 (twisting the whole frame alike takes no torque: the null space is all ones). Found from the
     * flexibility: a unit torque at each station s, balanced at the first, the twists relative to the first; that matrix
     * inverted and bordered with the first station's row and column. One station: [[0]].
     */
    public static double[][] twistStiffness(Frame frame, List<Station> stations) {
        List<Station> st = new ArrayList<>(stations);
        st.sort(Comparator.comparingDouble(Station::xM));
        int n = st.size();
        if (n == 0) { throw new IllegalArgumentException("A twist stiffness needs stations"); }
        for (Station s : st) { check(frame, s); }
        if (n == 1) { FrameSolver.requireConnected(frame); return new double[][]{{0}}; }
        int nodes = frame.nodeCount();
        boolean[] fixed = new boolean[6 * nodes];
        int held = st.get(0).leftNode();
        Arrays.fill(fixed, held * 6, held * 6 + 6, true);
        FrameSolver solver = FrameSolver.factor(frame, null, fixed);
        double[][] c = new double[n - 1][n - 1];
        for (int s = 1; s < n; s++) {
            double[] f = new double[6 * nodes];
            torque(frame, st.get(s), 1, f);
            torque(frame, st.get(0), -1, f);
            double[] u = solver.solve(f);
            double t0 = twist(frame, st.get(0), u);
            for (int t = 1; t < n; t++) { c[t - 1][s - 1] = twist(frame, st.get(t), u) - t0; }
        }
        for (int i = 0; i < n - 1; i++) {
            for (int j = 0; j < i; j++) { double m = 0.5 * (c[i][j] + c[j][i]); c[i][j] = m; c[j][i] = m; }
        }
        double[][] kr = Dense.inverse(c);
        if (kr == null) { throw new IllegalArgumentException("Two stations twist as one: give each station its own nodes"); }
        double[][] k = new double[n][n];
        for (int s = 1; s < n; s++) { for (int t = 1; t < n; t++) { k[s][t] = 0.5 * (kr[s - 1][t - 1] + kr[t - 1][s - 1]); } }
        double corner = 0;
        for (int t = 1; t < n; t++) {
            double sum = 0;
            for (int s = 1; s < n; s++) { sum += k[s][t]; }
            k[0][t] = -sum; k[t][0] = -sum; corner += sum;
        }
        k[0][0] = corner;
        return k;
    }

    /**
     * The vertical bending stiffness (N/m): the frame resting on the rear and front stations (each node held vertically,
     * free to turn) and loaded downward at the loaded station (split between a pair's nodes), the load over the loaded
     * nodes' mean sag.
     */
    public static double bendingStiffness(Frame frame, Station rear, Station front, Station loaded) {
        check(frame, rear); check(frame, front); check(frame, loaded);
        List<Support> supports = new ArrayList<>();
        LinkedHashSet<Integer> held = new LinkedHashSet<>();
        for (Station s : List.of(rear, front)) { for (int node : nodes(s)) { held.add(node); } }
        for (int node : held) { supports.add(Support.vertical(node, Support.FIXED)); }
        int[] at = nodes(loaded);
        for (int node : at) { if (held.contains(node)) { throw new IllegalArgumentException("The loaded station rests on a support"); } }
        double load = 1000;
        double[] f = new double[6 * frame.nodeCount()];
        for (int node : at) { f[node * 6 + 1] -= load / at.length; }
        Held h = new Held(frame, supports);
        double[] u = h.solve(f);
        double sag = 0;
        for (int node : at) { sag -= u[node * 6 + 1] / at.length; }
        if (!(sag > 0)) { throw new IllegalStateException("The frame does not sag under its load"); }
        return load / sag;
    }

    /**
     * Displacements, reactions and member stresses under loads at nodes, the frame held by supports (any number:
     * springs or fixed, e.g. vertical wheel springs on three or more axles). Throws when the supports leave a rigid-body
     * motion free that the loads would push.
     */
    public static LoadResult loadCase(Frame frame, List<NodeLoad> nodeLoads, List<Support> supports) {
        double[] f = new double[6 * frame.nodeCount()];
        for (NodeLoad l : nodeLoads) {
            frame.position(l.node());
            int b = l.node() * 6;
            f[b] += l.forceN().x(); f[b + 1] += l.forceN().y(); f[b + 2] += l.forceN().z();
            f[b + 3] += l.momentNm().x(); f[b + 4] += l.momentNm().y(); f[b + 5] += l.momentNm().z();
        }
        Held h = new Held(frame, supports);
        return h.result(f, h.solve(f));
    }

    /**
     * The standard chassis cases on the wheels' vertical springs, with the frame's own mass (nodeMassesKg) and the carried
     * masses at their nodes, gravity g (m/s^2) down:
     * <ul>
     * <li>bump: 2.5 g down;</li>
     * <li>ditch: 1 g with the ground under the port front wheel (largest x, then smallest z) DITCH_DROP_M lower, so the
     * frame twists over the diagonal as far as its springs and its own stiffness share the drop (a stiff frame lifts the
     * wheel off; a soft one keeps it down and carries less torque). The springs only push: a wheel its spring would have
     * to pull down is let go, and one let go whose mount would sink below its ground put back, until neither is left.
     * Needs four or more wheel nodes, else null;</li>
     * <li>braking: 1 g down and 0.8 g forward on every mass, held by rearward tyre forces of 0.8 times each wheel's load
     * in the stop (an ideal brake balance; found by a few re-solves);</li>
     * <li>cornering: 1 g down and 0.8 g toward starboard, held the same way by the tyres' side forces.</li>
     * </ul>
     * The tyres' forces act at the contact patch, groundYM (m) below each mount, and reach the mount with their moment
     * (the suspension links taken as rigid in that, without anti-dive or roll centre geometry). Outside the ditch the
     * springs hold whether pushed or pulled (a wheel that would lift is not let go).
     */
    public static StandardCases standardCases(Frame frame, List<Mass> carried, List<Wheel> wheels, double groundYM, double gravity) {
        return standardCases(frame, carried, wheels, List.of(), groundYM, gravity);
    }

    /**
     * The standard cases with some nodes held by another vehicle (0.36: a trailer's kingpin in a tractor's fifth wheel):
     * each pin is a vertical spring at its node (rate as a wheel's; FIXED: rigid) that takes its share of the weight like
     * a wheel and, braking and cornering, the same share of the horizontal force at its own height (the tractor pulls at
     * the kingpin, no patch below it). A pin is never the wheel in the ditch. Wheel and pin nodes together must be three
     * or more; the ditch needs four wheel nodes. With no pins this is the method above, bit for bit.
     */
    public static StandardCases standardCases(Frame frame, List<Mass> carried, List<Wheel> wheels, List<Wheel> pins, double groundYM, double gravity) {
        Require.finite(groundYM, "Ground height"); Require.positive(gravity, "Gravity");
        double[] weight = nodeMassesKg(frame);
        for (Mass m : carried) { frame.position(m.node()); weight[m.node()] += m.kg(); }
        for (int i = 0; i < weight.length; i++) { weight[i] *= gravity; }
        Map<Integer, Double> rates = new LinkedHashMap<>();
        for (Wheel w : wheels) { frame.position(w.node()); rates.merge(w.node(), w.rateNpm(), Double::sum); }
        Map<Integer, Double> pinRates = new LinkedHashMap<>();
        for (Wheel p : pins) {
            frame.position(p.node());
            if (rates.containsKey(p.node())) { throw new IllegalArgumentException("A pin's node carries a wheel"); }
            pinRates.merge(p.node(), p.rateNpm(), Double::sum);
        }
        if (rates.size() + pinRates.size() < 3) {
            throw new IllegalArgumentException(pinRates.isEmpty() ? "The standard cases need three or more wheel nodes" : "The standard cases need three or more wheel and pin nodes");
        }
        int[] at = rates.keySet().stream().mapToInt(Integer::intValue).toArray();
        List<Support> supports = new ArrayList<>();
        for (int node : at) { supports.add(Support.vertical(node, rates.get(node))); }
        List<Support> pinned = new ArrayList<>();
        for (Map.Entry<Integer, Double> p : pinRates.entrySet()) { pinned.add(Support.vertical(p.getKey(), p.getValue())); }
        supports.addAll(pinned);
        // the nodes carrying the frame: the wheels, then the pins; the patch's arm below each (a pin's force is at its node)
        int[] carriers = new int[at.length + pinned.size()];
        double[] arms = new double[carriers.length];
        for (int i = 0; i < at.length; i++) { carriers[i] = at[i]; arms[i] = frame.position(at[i]).y() - groundYM; }
        for (int i = 0; i < pinned.size(); i++) { carriers[at.length + i] = pinned.get(i).node(); }
        Held all = new Held(frame, supports);
        double[] bumpLoads = vertical(weight, BUMP_G);
        LoadResult bump = all.result(bumpLoads, all.solve(bumpLoads));
        double[] still = vertical(weight, 1);
        double[] stillLoads = wheelLoads(all, still, all.solve(still), carriers);
        LoadResult braking = tyreCase(all, weight, carriers, stillLoads, arms, 0, BRAKING_G);
        LoadResult cornering = tyreCase(all, weight, carriers, stillLoads, arms, 2, CORNERING_G);
        LoadResult ditch = null;
        if (at.length >= 4) {
            int dropped = 0;
            for (int i = 1; i < at.length; i++) {
                Vec3d p = frame.position(at[i]), q = frame.position(at[dropped]);
                if (p.x() > q.x() || (p.x() == q.x() && p.z() < q.z())) { dropped = i; }
            }
            double[] k = new double[at.length];
            for (int i = 0; i < at.length; i++) { k[i] = rates.get(at[i]); }
            ditch = ditch(all, at, k, dropped, still, pinned);
        }
        return new StandardCases(bump, ditch, braking, cornering);
    }

    /**
     * The ditch: the ground under one wheel DITCH_DROP_M lower. A spring's ground end lowered by d is a load -k d at its
     * node (a rigid wheel's node is moved d instead); the reactions are the springs' own forces. Which wheels touch is
     * found one wheel at a time, the worst first: one its spring pulls down is let go, one let go whose node would sink
     * below its ground is put back (both measured as the spring's force); a change that leaves wheels unable to hold the
     * frame is passed over for the next worst.
     */
    private static LoadResult ditch(Held all, int[] at, double[] rates, int dropped, double[] still, List<Support> pinned) {
        double total = 0;
        for (int i = 1; i < still.length; i += 6) { total += Math.abs(still[i]); }
        double tolerance = 1e-9 * total;
        boolean[] off = new boolean[at.length];
        Standing s = stand(all, at, rates, dropped, still, off, pinned);
        if (s == null) { throw new IllegalArgumentException("The wheels do not hold the frame"); }
        for (int round = 0; round <= 4 * at.length; round++) {
            double[] wrong = new double[at.length];
            List<Integer> order = new ArrayList<>();
            for (int i = 0; i < at.length; i++) {
                int b = at[i] * 6 + 1;
                if (!off[i]) {
                    wrong[i] = -(s.internal()[b] - still[b]);
                } else {
                    double sink = (i == dropped ? -DITCH_DROP_M : 0) - s.u()[b];
                    wrong[i] = sink <= 0 ? 0 : rates[i] == Support.FIXED ? Double.MAX_VALUE : rates[i] * sink;
                }
                if (wrong[i] > tolerance) { order.add(i); }
            }
            if (order.isEmpty()) { return s.held().result(still, s.u()); }
            order.sort((p, q) -> Double.compare(wrong[q], wrong[p]));
            Standing next = null;
            for (int i : order) {
                off[i] = !off[i];
                next = stand(all, at, rates, dropped, still, off, pinned);
                if (next != null) { break; }
                off[i] = !off[i];
            }
            if (next == null) { break; }
            s = next;
        }
        throw new IllegalStateException("The ditch finds no set of wheels for the frame to stand on");
    }

    /** The frame on the wheels not let go, solved; null when they cannot hold it. */
    private record Standing(Held held, double[] u, double[] internal) {}

    private static Standing stand(Held all, int[] at, double[] rates, int dropped, double[] still, boolean[] off, List<Support> pinned) {
        Held h = all;
        boolean any = false;
        for (boolean o : off) { any |= o; }
        if (any) {
            List<Support> touching = new ArrayList<>();
            for (int i = 0; i < at.length; i++) { if (!off[i]) { touching.add(Support.vertical(at[i], rates[i])); } }
            touching.addAll(pinned);
            h = new Held(all.frame, touching);
        }
        double[] f = still.clone(), shift = new double[still.length];
        if (!off[dropped]) {
            int b = at[dropped] * 6 + 1;
            if (rates[dropped] == Support.FIXED) {
                shift[b] = -DITCH_DROP_M;
                double[] pushed = h.internal(shift, null);
                for (int i = 0; i < f.length; i++) { f[i] -= pushed[i]; }
            } else {
                f[b] -= rates[dropped] * DITCH_DROP_M;
            }
        }
        double[] u;
        try {
            u = h.solve(f);
        } catch (IllegalArgumentException unheld) {
            return null;
        }
        for (int i = 0; i < u.length; i++) { u[i] += shift[i]; }
        return new Standing(h, u, h.internal(u, null));
    }

    /**
     * Braking (axis 0) or cornering (axis 2): inertia on every mass, tyre forces in proportion to each wheel's load, each
     * arm (m) above its patch (0.36: a pin's 0).
     */
    private static LoadResult tyreCase(Held h, double[] weight, int[] wheels, double[] start, double[] arms, int axis, double g) {
        double total = 0;
        for (double w : weight) { total += w; }
        double[] loads = start.clone(), f = null, u = null;
        for (int iteration = 0; iteration < 20; iteration++) {
            f = vertical(weight, 1);
            for (int k = 0; k < weight.length; k++) { f[k * 6 + axis] += g * weight[k]; }
            for (int i = 0; i < wheels.length; i++) {
                int b = wheels[i] * 6;
                double tyre = -g * loads[i], arm = arms[i];
                f[b + axis] += tyre;
                // the contact patch's force carried up to the mount: (patch - mount) × force
                if (axis == 0) { f[b + 5] += arm * tyre; } else { f[b + 3] -= arm * tyre; }
            }
            u = h.solve(f);
            double[] next = wheelLoads(h, f, u, wheels);
            double change = 0;
            for (int i = 0; i < wheels.length; i++) { change = Math.max(change, Math.abs(next[i] - loads[i])); }
            loads = next;
            if (change <= 1e-12 * total) { break; }
        }
        return h.result(f, u);
    }

    private static double[] vertical(double[] weight, double g) {
        double[] f = new double[weight.length * 6];
        for (int k = 0; k < weight.length; k++) { f[k * 6 + 1] = -g * weight[k]; }
        return f;
    }

    /** The upward reactions at the wheels' nodes. */
    private static double[] wheelLoads(Held h, double[] loads, double[] u, int[] wheels) {
        double[] internal = h.internal(u, null);
        double[] out = new double[wheels.length];
        for (int i = 0; i < wheels.length; i++) { out[i] = internal[wheels[i] * 6 + 1] - loads[wheels[i] * 6 + 1]; }
        return out;
    }

    private static void check(Frame frame, Station s) {
        Vec3d l = frame.position(s.leftNode()), r = frame.position(s.rightNode());
        if (s.single()) { return; }
        double dz = r.z() - l.z();
        if (!(dz > 1e-6)) { throw new IllegalArgumentException("A station's right node must lie to starboard of its left"); }
        if (Math.abs(r.x() - l.x()) > 1e-3 * dz) { throw new IllegalArgumentException("A station's two nodes must stand at the same x"); }
    }

    private static int[] nodes(Station s) { return s.single() ? new int[]{s.leftNode()} : new int[]{s.leftNode(), s.rightNode()}; }

    private static void torque(Frame frame, Station s, double torqueNm, double[] f) {
        if (s.single()) { f[s.leftNode() * 6 + 3] += torqueNm; return; }
        double dz = frame.position(s.rightNode()).z() - frame.position(s.leftNode()).z();
        f[s.leftNode() * 6 + 1] += torqueNm / dz;
        f[s.rightNode() * 6 + 1] -= torqueNm / dz;
    }

    private static double twist(Frame frame, Station s, double[] u) {
        if (s.single()) { return u[s.leftNode() * 6 + 3]; }
        double dz = frame.position(s.rightNode()).z() - frame.position(s.leftNode()).z();
        return (u[s.leftNode() * 6 + 1] - u[s.rightNode() * 6 + 1]) / dz;
    }

    /** A frame on its supports, factored, with the rigid-body motions they leave free held at one node. */
    private static final class Held {
        final Frame frame;
        final FrameSolver solver;
        /** The supported nodes, each once, in the supports' order. */
        final int[] supported;
        /** The free rigid-body motions as unit 6-vectors (translation, rotation times lc) about the origin. */
        final double[][] free;
        final Vec3d origin;
        final double lc;

        Held(Frame frame, List<Support> supports) {
            this.frame = frame;
            int n = frame.nodeCount();
            if (n == 0) { throw new IllegalArgumentException("A frame needs nodes"); }
            double[] springs = new double[6 * n];
            boolean[] fixed = new boolean[6 * n];
            LinkedHashSet<Integer> nodes = new LinkedHashSet<>();
            for (Support s : supports) {
                frame.position(s.node());
                nodes.add(s.node());
                for (int d = 0; d < 6; d++) {
                    double r = s.rate(d);
                    if (r == Support.FIXED) { fixed[s.node() * 6 + d] = true; } else { springs[s.node() * 6 + d] += r; }
                }
            }
            supported = nodes.stream().mapToInt(Integer::intValue).toArray();
            double x = 0, y = 0, z = 0;
            for (Vec3d p : frame.nodes()) { x += p.x(); y += p.y(); z += p.z(); }
            origin = new Vec3d(x / n, y / n, z / n);
            double reach = 0;
            int reference = 0;
            for (int i = 0; i < n; i++) {
                double d = frame.position(i).subtract(origin).length();
                reach = Math.max(reach, d);
                if (d < frame.position(reference).subtract(origin).length()) { reference = i; }
            }
            lc = reach > 1e-9 ? reach : 1;
            // the rigid motions the supports hold: the span of their freedoms' rows
            double[][] held = new double[6][6];
            double[] row = new double[6];
            for (int i = 0; i < 6 * n; i++) {
                if (!fixed[i] && !(springs[i] > 0)) { continue; }
                modeRow(i / 6, i % 6, row);
                for (int a = 0; a < 6; a++) { for (int b = 0; b < 6; b++) { held[a][b] += row[a] * row[b]; } }
            }
            double[][] vectors = new double[6][6];
            double[] values = Dense.eigen(held, vectors);
            double top = 0;
            for (double v : values) { top = Math.max(top, v); }
            List<double[]> loose = new ArrayList<>();
            for (int k = 0; k < 6; k++) {
                if (values[k] > 1e-10 * top) { continue; }
                double[] q = new double[6];
                for (int a = 0; a < 6; a++) { q[a] = vectors[a][k]; }
                loose.add(q);
            }
            free = loose.toArray(new double[0][]);
            boolean[] all = fixed.clone();
            for (int d : holding(reference)) { all[reference * 6 + d] = true; }
            solver = FrameSolver.factor(frame, springs, all);
        }

        /** How far freedom d of a node moves in each unit rigid motion (translation x y z, rotation about x y z times lc). */
        private void modeRow(int node, int d, double[] row) {
            Arrays.fill(row, 0);
            Vec3d r = frame.position(node).subtract(origin).scale(1 / lc);
            switch (d) {
                case 0 -> { row[0] = 1; row[4] = r.z(); row[5] = -r.y(); }
                case 1 -> { row[1] = 1; row[3] = -r.z(); row[5] = r.x(); }
                case 2 -> { row[2] = 1; row[3] = r.y(); row[4] = -r.x(); }
                default -> row[d] = 1;
            }
        }

        /** Freedoms of the reference node that pin down the free motions (complete pivoting on their rows). */
        private int[] holding(int reference) {
            int m = free.length;
            double[][] g = new double[6][m];
            double[] row = new double[6];
            for (int d = 0; d < 6; d++) {
                modeRow(reference, d, row);
                for (int k = 0; k < m; k++) { double s = 0; for (int a = 0; a < 6; a++) { s += row[a] * free[k][a]; } g[d][k] = s; }
            }
            boolean[] rowUsed = new boolean[6], colUsed = new boolean[m];
            int[] out = new int[m];
            for (int step = 0; step < m; step++) {
                int pr = -1, pc = -1;
                double best = -1;
                for (int r = 0; r < 6; r++) {
                    if (rowUsed[r]) { continue; }
                    for (int c = 0; c < m; c++) { if (!colUsed[c] && Math.abs(g[r][c]) > best) { best = Math.abs(g[r][c]); pr = r; pc = c; } }
                }
                out[step] = pr; rowUsed[pr] = true; colUsed[pc] = true;
                for (int r = 0; r < 6; r++) {
                    if (rowUsed[r]) { continue; }
                    double factor = g[r][pc] / g[pr][pc];
                    for (int c = 0; c < m; c++) { g[r][c] -= factor * g[pr][c]; }
                }
            }
            return out;
        }

        /** Solves after checking the loads leave the free rigid motions balanced. */
        double[] solve(double[] loads) {
            double size = 0;
            for (int i = 0; i < loads.length; i++) { size += i % 6 < 3 ? Math.abs(loads[i]) : Math.abs(loads[i]) / lc; }
            double[] row = new double[6];
            for (double[] q : free) {
                double work = 0;
                for (int i = 0; i < loads.length; i++) {
                    if (loads[i] == 0) { continue; }
                    modeRow(i / 6, i % 6, row);
                    double move = 0;
                    for (int a = 0; a < 6; a++) { move += row[a] * q[a]; }
                    work += (i % 6 < 3 ? loads[i] : loads[i] / lc) * move;
                }
                if (Math.abs(work) > BALANCE * size) {
                    throw new IllegalArgumentException("The supports do not hold these loads: they would move the frame as a rigid body");
                }
            }
            return solver.solve(loads);
        }

        /** The forces the members need at the nodes (6 per node); members (or null) receives each member's result. */
        double[] internal(double[] u, List<MemberResult> members) {
            double[] out = new double[6 * frame.nodeCount()];
            for (int m = 0; m < frame.memberCount(); m++) {
                Frame.Member mem = frame.member(m);
                double[] r = Beam.rotation(frame, m), f = Beam.endForces(frame, m, r, Beam.local(frame, m), u);
                for (int blk = 0; blk < 4; blk++) {
                    Vec3d v = Beam.toFrame(f, blk, r);
                    int b = (blk < 2 ? mem.a() : mem.b()) * 6 + (blk % 2) * 3;
                    out[b] += v.x(); out[b + 1] += v.y(); out[b + 2] += v.z();
                }
                if (members == null) { continue; }
                Section s = mem.section();
                Section.TorsionStress torsion = s.torsionStress(mem.material(), frame.lengthM(m), mem.jointFactor(), f[9]);
                double axial = Math.abs(f[6]) / s.areaM2();
                double sigma = axial + Math.max(s.bendingPa(f[4], f[5]), s.bendingPa(f[10], f[11])) + torsion.warpingPa();
                double tau = torsion.shearPa();
                double vonMises = Math.sqrt(sigma * sigma + 3 * tau * tau);
                members.add(new MemberResult(m, new EndForces(new Vec3d(f[0], f[1], f[2]), new Vec3d(f[3], f[4], f[5])),
                        new EndForces(new Vec3d(f[6], f[7], f[8]), new Vec3d(f[9], f[10], f[11])), vonMises, vonMises / mem.material().yieldPa()));
            }
            return out;
        }

        LoadResult result(double[] loads, double[] u) {
            List<MemberResult> members = new ArrayList<>(frame.memberCount());
            double[] internal = internal(u, members);
            List<Reaction> reactions = new ArrayList<>(supported.length);
            for (int node : supported) {
                int b = node * 6;
                reactions.add(new Reaction(node, new Vec3d(internal[b] - loads[b], internal[b + 1] - loads[b + 1], internal[b + 2] - loads[b + 2]),
                        new Vec3d(internal[b + 3] - loads[b + 3], internal[b + 4] - loads[b + 4], internal[b + 5] - loads[b + 5])));
            }
            return new LoadResult(u, loads, reactions, members);
        }
    }
}
