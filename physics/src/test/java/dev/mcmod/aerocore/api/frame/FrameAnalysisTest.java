package dev.mcmod.aerocore.api.frame;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.LoadResult;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.Mass;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.MemberResult;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.NodeLoad;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.Reaction;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.StandardCases;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.Station;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.Support;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.Wheel;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The frame FEM: exact beam answers (cantilever, torsion, a simply held beam, a portal frame), reactions that balance,
 * the condensed twist matrix, a frame that falls apart, and the time a 200-node tube frame takes.
 */
class FrameAnalysisTest {
    static final FrameMaterial STEEL = new FrameMaterial(7850, 210e9, 81e9, 355e6);

    static void assertVec(Vec3d expected, Vec3d actual, double tolerance, String what) {
        assertEquals(0, actual.subtract(expected).length(), tolerance, what + ": " + actual + " against " + expected);
    }

    /** A straight run of members from start along dir, length l, in pieces. */
    static int[] run(Frame f, Vec3d start, Vec3d dir, double l, int pieces, Section s, Vec3d up) {
        int[] node = new int[pieces + 1];
        for (int i = 0; i <= pieces; i++) { node[i] = f.node(start.add(dir.scale(l * i / pieces))); }
        for (int i = 0; i < pieces; i++) { f.member(node[i], node[i + 1], s, STEEL, up, Frame.WELDED); }
        return node;
    }

    @Test void aCantileverBendsAsPLCubedOverThreeEI() {
        Section s = Section.rectTube(0.06, 0.10, 0.004);
        Vec3d dir = new Vec3d(1, 0.4, -0.7).normalized(), up = new Vec3d(0.2, 1, 0.3);
        double l = 2.3, p = 1500, e = STEEL.modulusPa();
        for (int pieces : new int[]{1, 4}) {
            Frame f = new Frame();
            int[] node = run(f, new Vec3d(0.3, 0.5, -0.2), dir, l, pieces, s, up);
            int tip = node[pieces];
            Vec3d[] axes = f.axes(0);
            // along the member's y: bending on Iz
            Vec3d force = axes[1].scale(p);
            LoadResult r = FrameAnalysis.loadCase(f, List.of(NodeLoad.force(tip, force)), List.of(Support.fixed(node[0])));
            double sag = p * l * l * l / (3 * e * s.izM4()), turn = p * l * l / (2 * e * s.izM4());
            assertEquals(sag, r.displacement(tip).dot(axes[1]), 1e-9 * sag, pieces + " pieces: PL³/3EIz");
            assertEquals(0, r.displacement(tip).dot(axes[2]), 1e-9 * sag);
            assertEquals(0, r.displacement(tip).dot(axes[0]), 1e-9 * sag);
            assertEquals(turn, r.rotation(tip).dot(axes[2]), 1e-9 * turn, "PL²/2EIz");
            // the root: M c / I over yield, and the reaction holds the force and its moment
            double usage = p * l * s.outerYM() / s.izM4() / STEEL.yieldPa();
            assertEquals(usage, r.members().get(0).usage(), 1e-9 * usage);
            Reaction root = r.reactions().get(0);
            assertVec(force.scale(-1), root.forceN(), 1e-9 * p, "root force");
            assertVec(dir.scale(l).cross(force).scale(-1), root.momentNm(), 1e-9 * p * l, "root moment");
            // along the member's z: bending on Iy
            force = axes[2].scale(p);
            r = FrameAnalysis.loadCase(f, List.of(NodeLoad.force(tip, force)), List.of(Support.fixed(node[0])));
            sag = p * l * l * l / (3 * e * s.iyM4()); turn = p * l * l / (2 * e * s.iyM4());
            assertEquals(sag, r.displacement(tip).dot(axes[2]), 1e-9 * sag, pieces + " pieces: PL³/3EIy");
            assertEquals(0, r.displacement(tip).dot(axes[1]), 1e-9 * sag);
            assertEquals(-turn, r.rotation(tip).dot(axes[1]), 1e-9 * turn, "PL²/2EIy");
        }
    }

    @Test void aRoundTubeTwistsAsTLOverGJ() {
        double d = 0.05, t = 0.003;
        Section s = Section.roundTube(d, t);
        double i = Math.PI * (Math.pow(d, 4) - Math.pow(d - 2 * t, 4)) / 64;
        assertEquals(i, s.izM4(), 1e-12 * i);
        assertEquals(2 * i, s.torsionM4(), 1e-12 * i);
        Vec3d dir = new Vec3d(-0.3, 0.8, 0.5).normalized();
        double l = 1.7, torque = 450;
        Frame f = new Frame();
        int[] node = run(f, new Vec3d(1, 0, 0), dir, l, 3, s, Vec3d.X);
        int tip = node[3];
        LoadResult r = FrameAnalysis.loadCase(f, List.of(new NodeLoad(tip, Vec3d.ZERO, dir.scale(torque))), List.of(Support.fixed(node[0])));
        double twist = torque * l / (STEEL.shearModulusPa() * s.torsionM4());
        assertEquals(twist, r.rotation(tip).dot(dir), 1e-9 * twist, "TL/GJ");
        assertEquals(twist, r.rotation(tip).length(), 1e-9 * twist, "only about its axis");
        assertEquals(0, r.displacement(tip).length(), 1e-9 * twist * l);
        assertEquals(torque, r.members().get(2).torqueNm(), 1e-9 * torque);
        double tau = torque / (2 * s.enclosedAreaM2() * t);
        assertEquals(Math.sqrt(3) * tau / STEEL.yieldPa(), r.members().get(0).usage(), 1e-9);
    }

    @Test void aSimplyHeldBeamIsAsStiffAs48EIOverLCubed() {
        Section s = Section.rectTube(0.05, 0.08, 0.003);
        double l = 2.4;
        Frame f = new Frame();
        int[] node = run(f, Vec3d.ZERO, Vec3d.X, l, 2, s, Vec3d.Y);
        double k = FrameAnalysis.bendingStiffness(f, Station.node(0, node[0]), Station.node(l, node[2]), Station.node(l / 2, node[1]));
        double exact = 48 * STEEL.modulusPa() * s.izM4() / (l * l * l);
        assertEquals(exact, k, 1e-9 * exact);
    }

    @Test void aPortalFrameSwaysAsSlopeDeflectionSays() {
        // fixed bases; stiff in stretch (1000 m² of area: the columns' stretch, which the closed form leaves out, moves the sway 5e-9)
        double h = 2.0, span = 3.0, ic = 2e-6, ib = 5e-6, push = 2000;
        Section column = new Section(Section.Kind.CUSTOM, 1000, ic, ic, 2 * ic, 0, 0.05, 0.05, true, 0.005, 0.01, new double[0][]);
        Section beam = new Section(Section.Kind.CUSTOM, 1000, ib, ib, 2 * ib, 0, 0.06, 0.06, true, 0.005, 0.01, new double[0][]);
        Frame f = new Frame();
        int a = f.node(new Vec3d(0, 0, 0)), b = f.node(new Vec3d(0, h, 0)), c = f.node(new Vec3d(span, h, 0)), d = f.node(new Vec3d(span, 0, 0));
        f.member(a, b, column, STEEL, Vec3d.X, Frame.WELDED);
        f.member(b, c, beam, STEEL, Vec3d.Y, Frame.WELDED);
        f.member(d, c, column, STEEL, Vec3d.X, Frame.WELDED);
        LoadResult r = FrameAnalysis.loadCase(f, List.of(NodeLoad.force(b, new Vec3d(push, 0, 0))), List.of(Support.fixed(a), Support.fixed(d)));
        // slope-deflection (clockwise positive): θ_B = θ_C = 3 k_c ψ / (2 k_c + 3 k_b), shear H h = 12 k_c (2 ψ - θ)
        double e = STEEL.modulusPa(), kc = e * ic / h, kb = e * ib / span;
        double sway = push * h * h * (2 * kc + 3 * kb) / (12 * kc * (kc + 6 * kb));
        double psi = sway / h, theta = 3 * kc * psi / (2 * kc + 3 * kb), base = 2 * kc * (theta - 3 * psi);
        assertEquals(sway, r.displacement(b).x(), 1e-6 * sway, "sway");
        assertEquals(sway, r.displacement(c).x(), 1e-6 * sway, "sway at the far corner");
        assertEquals(-theta, r.rotation(b).z(), 1e-6 * theta, "corner rotation");
        assertEquals(-theta, r.rotation(c).z(), 1e-6 * theta, "far corner rotation");
        for (Reaction base0 : r.reactions()) {
            assertEquals(Math.abs(base), Math.abs(base0.momentNm().z()), 1e-6 * Math.abs(base), "base moment");
            assertEquals(-push / 2, base0.forceN().x(), 1e-6 * push, "base shear");
        }
        assertEquals(0, r.displacement(b).z(), 1e-12);
    }

    /** A ladder frame: two channel rails at z = ±0.45 m, split at each cross member (rectangular tube, welded) at xs. */
    record Ladder(Frame frame, int[] left, int[] right, double[] xs) {
        Station station(int i) { return Station.pair(xs[i], left[i], right[i]); }
    }

    static Ladder ladder(double[] xs, double railJoint) {
        return ladder(xs, Section.channel(0.15, 0.06, 0.005), Section.rectTube(0.08, 0.06, 0.003), railJoint, Frame.WELDED, 0.45);
    }

    static Ladder ladder(double[] xs, Section rail, Section cross, double railJoint, double crossJoint, double halfTrack) {
        Frame f = new Frame();
        int n = xs.length;
        int[] l = new int[n], r = new int[n];
        for (int i = 0; i < n; i++) { l[i] = f.node(new Vec3d(xs[i], 0, -halfTrack)); r[i] = f.node(new Vec3d(xs[i], 0, halfTrack)); }
        for (int i = 0; i + 1 < n; i++) {
            f.member(l[i], l[i + 1], rail, STEEL, Vec3d.Y, railJoint);
            f.member(r[i], r[i + 1], rail, STEEL, Vec3d.Y, railJoint);
        }
        for (int i = 0; i < n; i++) { f.member(l[i], r[i], cross, STEEL, Vec3d.Y, crossJoint); }
        return new Ladder(f, l, r, xs);
    }

    /** The loads a case puts on: each node's own share of the members and the carried masses, g down. */
    static List<NodeLoad> weights(Frame f, List<Mass> masses, double g) {
        double[] kg = FrameAnalysis.nodeMassesKg(f);
        for (Mass m : masses) { kg[m.node()] += m.kg(); }
        List<NodeLoad> out = new ArrayList<>();
        for (int i = 0; i < kg.length; i++) { out.add(NodeLoad.force(i, new Vec3d(0, -kg[i] * g, 0))); }
        return out;
    }

    @Test void aDitchTwistsTheFrameAsSpringsInSeries() {
        Ladder lad = ladder(new double[]{0, 1.0, 2.2, 3.4, 4.5}, Frame.WELDED);
        Frame f = lad.frame();
        double kf = 1.2e5, kr = 1.6e5, t = 0.9, d = FrameAnalysis.DITCH_DROP_M, g = 9.81;
        List<Wheel> wheels = List.of(new Wheel(lad.left()[0], kr), new Wheel(lad.right()[0], kr), new Wheel(lad.left()[4], kf), new Wheel(lad.right()[4], kf));
        // heavy and even across: the wheel stays down in the ditch, and the weight itself puts no torque on the frame
        List<Mass> heavy = new ArrayList<>();
        for (int i = 0; i < 5; i++) { heavy.add(new Mass(lad.left()[i], 300)); heavy.add(new Mass(lad.right()[i], 300)); }
        LoadResult ditch = FrameAnalysis.standardCases(f, heavy, wheels, -0.3, g).ditch();
        assertEquals(4, ditch.reactions().size());
        for (Reaction re : ditch.reactions()) { assertTrue(re.forceN().y() > 0, "every wheel stands"); }
        // the drop d over the track t is a twist shared by the front axle's roll spring k t² / 2, the frame and the rear's
        double k = FrameAnalysis.twistStiffness(f, List.of(lad.station(0), lad.station(4)))[1][1];
        double cf = kf * t * t / 2, cr = kr * t * t / 2;
        double torque = (d / t) / (1 / cf + 1 / cr + 1 / k);
        double front = t / 2 * (load(ditch, lad.right()[4]) - load(ditch, lad.left()[4]));
        double rear = t / 2 * (load(ditch, lad.left()[0]) - load(ditch, lad.right()[0]));
        assertEquals(torque, front, 1e-9 * torque, "torque through the front axle");
        assertEquals(torque, rear, 1e-9 * torque, "torque through the rear axle");
        // light, the weight behind the middle: the frame holds the corner up, the wheel hangs clear of the ditch's floor,
        // and the case is the frame standing on the other three
        List<Mass> light = List.of(new Mass(lad.left()[1], 40), new Mass(lad.right()[1], 40));
        LoadResult hanging = FrameAnalysis.standardCases(f, light, wheels, -0.3, g).ditch();
        assertEquals(3, hanging.reactions().size());
        for (Reaction re : hanging.reactions()) { assertNotEquals(lad.left()[4], re.node(), "the port front wheel hangs"); }
        assertTrue(hanging.displacement(lad.left()[4]).y() > -d, "clear of the ditch's floor");
        LoadResult three = FrameAnalysis.loadCase(f, weights(f, light, g),
                List.of(Support.vertical(lad.left()[0], kr), Support.vertical(lad.right()[0], kr), Support.vertical(lad.right()[4], kf)));
        assertEquals(three.maxUsage(), hanging.maxUsage(), 1e-9 * three.maxUsage());
        for (Reaction re : three.reactions()) { assertEquals(re.forceN().y(), load(hanging, re.node()), 1e-9 * Math.abs(re.forceN().y()) + 1e-9); }
        // a rigid wheel's ground drops the same way: the node goes down with it
        List<Wheel> rigid = List.of(new Wheel(lad.left()[0], kr), new Wheel(lad.right()[0], kr), new Wheel(lad.left()[4], Support.FIXED), new Wheel(lad.right()[4], kf));
        LoadResult pressed = FrameAnalysis.standardCases(f, heavy, rigid, -0.3, g).ditch();
        assertEquals(-d, pressed.displacement(lad.left()[4]).y(), 1e-12);
        assertBalanced(f, pressed, "a rigid wheel in the ditch");
    }

    @Test void aTruckLadderTakesTheDitchOnItsSprings() {
        // three axles: 7.5 m, riveted C270x80x8 rails 0.86 m apart, riveted C200x70x6 cross members, 12 t, 1.4 Hz wheels
        double[] xs = {0, 0.9, 1.6, 2.3, 3.4, 4.6, 5.8, 6.6, 7.5};
        Ladder lad = ladder(xs, Section.channel(0.27, 0.08, 0.008), Section.channel(0.2, 0.07, 0.006), Frame.RIVETED, Frame.RIVETED, 0.43);
        Frame f = lad.frame();
        double g = 9.81, frameKg = FrameAnalysis.massKg(f), cab = 4200, cargo = 12000 - frameKg - cab;
        List<Mass> masses = new ArrayList<>();
        for (int i = 6; i < 9; i++) { masses.add(new Mass(lad.left()[i], cab / 6)); masses.add(new Mass(lad.right()[i], cab / 6)); }
        for (int i = 0; i < 6; i++) { masses.add(new Mass(lad.left()[i], cargo / 12)); masses.add(new Mass(lad.right()[i], cargo / 12)); }
        int[] axle = {1, 3, 7};
        List<Support> nominal = new ArrayList<>();
        for (int a : axle) { nominal.add(Support.vertical(lad.left()[a], 1.5e5)); nominal.add(Support.vertical(lad.right()[a], 1.5e5)); }
        LoadResult still = FrameAnalysis.loadCase(f, weights(f, masses, g), nominal);
        // each wheel sprung at 1.4 Hz on its own load
        double omega = 2 * Math.PI * 1.4;
        List<Wheel> wheels = new ArrayList<>();
        for (Reaction re : still.reactions()) { wheels.add(new Wheel(re.node(), re.forceN().y() / g * omega * omega)); }
        StandardCases cases = FrameAnalysis.standardCases(f, masses, wheels, -0.55, g);
        // the old reading: the port front wheel taken away, all of a rail's torque as St Venant shear
        List<Support> without = new ArrayList<>();
        for (Wheel w : wheels) { if (w.node() != lad.left()[7]) { without.add(Support.vertical(w.node(), w.rateNpm())); } }
        LoadResult removed = FrameAnalysis.loadCase(f, weights(f, masses, g), without);
        Section rail = f.member(0).section();
        double ditchTorque = railTorque(f, cases.ditch()), removedTorque = railTorque(f, removed);
        double k = FrameAnalysis.twistStiffness(f, List.of(lad.station(1), lad.station(7)))[1][1];
        MemberResult worst = cases.ditch().members().get(cases.ditch().worstMember());
        Section.TorsionStress ts = f.member(worst.member()).section().torsionStress(STEEL, f.lengthM(worst.member()),
                f.member(worst.member()).jointFactor(), worst.torqueNm());
        System.out.printf("truck ladder %.0f kg frame, 12 t: twist %.0f N m/deg between the outer axles; front wheel load %.1f kN still, %.1f kN in the ditch%n",
                frameKg, k * Math.PI / 180, load(cases.bump(), lad.left()[7]) / FrameAnalysis.BUMP_G / 1000, load(cases.ditch(), lad.left()[7]) / 1000);
        System.out.printf("truck ladder ditch: greatest rail torque %.2f kN m (wheel taken away: %.2f kN m, St Venant %.0f MPa); worst member %d usage %.2f"
                        + " (St Venant shear %.0f MPa, warping %.0f MPa, von Mises %.0f MPa)%n",
                ditchTorque / 1000, removedTorque / 1000, rail.torsionShearPa(removedTorque) / 1e6, worst.member(), worst.usage(),
                ts.shearPa() / 1e6, ts.warpingPa() / 1e6, worst.vonMisesPa() / 1e6);
        System.out.printf("truck ladder usage: bump %.2f, ditch %.2f, braking %.2f, cornering %.2f (wheel taken away, new stresses: %.2f)%n",
                cases.bump().maxUsage(), cases.ditch().maxUsage(), cases.braking().maxUsage(), cases.cornering().maxUsage(), removed.maxUsage());
        for (Reaction re : cases.ditch().reactions()) { assertTrue(re.forceN().y() > 0); }
        assertTrue(ditchTorque < removedTorque, "a soft frame takes the ditch on its springs");
        assertTrue(cases.ditch().maxUsage() < removed.maxUsage());
    }

    /** The greatest torque in a ladder's rails (the members before the cross members). */
    static double railTorque(Frame f, LoadResult r) {
        double t = 0;
        for (MemberResult m : r.members()) {
            Frame.Member mem = f.member(m.member());
            if (Math.abs(f.position(mem.a()).z() - f.position(mem.b()).z()) < 1e-9) { t = Math.max(t, Math.abs(m.torqueNm())); }
        }
        return t;
    }

    @Test void theTwistMatrixIsSymmetricSemidefiniteAndBlindToAWholeTwist() {
        Ladder lad = ladder(new double[]{0, 1.0, 2.2, 3.4, 4.5}, Frame.WELDED);
        List<Station> shuffled = List.of(lad.station(3), lad.station(0), lad.station(4), lad.station(1));
        double[][] k = FrameAnalysis.twistStiffness(lad.frame(), shuffled);
        int n = k.length;
        assertEquals(4, n);
        double top = 0;
        for (double[] row : k) { for (double v : row) { top = Math.max(top, Math.abs(v)); } }
        for (int i = 0; i < n; i++) {
            double sum = 0;
            for (int j = 0; j < n; j++) { assertEquals(k[i][j], k[j][i], 1e-9 * top, "symmetric"); sum += k[i][j]; }
            assertEquals(0, sum, 1e-9 * top, "K 1 = 0");
            assertTrue(k[i][i] > 0);
        }
        double[] values = Dense.eigen(k, null);
        Arrays.sort(values);
        assertEquals(0, values[0], 1e-9 * top, "the null space is the whole frame twisting alike");
        for (int i = 1; i < n; i++) { assertTrue(values[i] > 1e-6 * top, "positive semi-definite, one null direction: " + Arrays.toString(values)); }
        // in order of x: the same as from the sorted list
        double[][] sorted = FrameAnalysis.twistStiffness(lad.frame(), List.of(lad.station(0), lad.station(1), lad.station(3), lad.station(4)));
        for (int i = 0; i < n; i++) { assertArrayEquals(sorted[i], k[i], 1e-12 * top); }
        // two stations against a load case held otherwise: the rear pair on vertical supports, a couple at the front
        double[][] two = FrameAnalysis.twistStiffness(lad.frame(), List.of(lad.station(0), lad.station(4)));
        double torque = 1000, dz = 0.9;
        LoadResult r = FrameAnalysis.loadCase(lad.frame(),
                List.of(NodeLoad.force(lad.left()[4], new Vec3d(0, torque / dz, 0)), NodeLoad.force(lad.right()[4], new Vec3d(0, -torque / dz, 0))),
                List.of(Support.vertical(lad.left()[0], Support.FIXED), Support.vertical(lad.right()[0], Support.FIXED)));
        double twist = (r.displacement(lad.left()[4]).y() - r.displacement(lad.right()[4]).y()) / dz;
        assertEquals(torque / twist, two[1][1], 1e-9 * two[1][1], "end-to-end twist stiffness");
        assertEquals(-two[1][1], two[0][1], 1e-9 * two[1][1]);
        // bolted open rails hold less of their warping: softer
        double[][] bolted = FrameAnalysis.twistStiffness(ladder(lad.xs(), Frame.BOLTED).frame(), List.of(lad.station(0), lad.station(4)));
        assertTrue(bolted[1][1] < two[1][1]);
        System.out.printf("ladder 4.5 m, C150x60x5 rails: %.0f N m/deg welded, %.0f bolted; bending %.0f kN/m%n",
                two[1][1] * Math.PI / 180, bolted[1][1] * Math.PI / 180,
                FrameAnalysis.bendingStiffness(lad.frame(), lad.station(0), lad.station(4), lad.station(2)) / 1000);
        assertArrayEquals(new double[]{0}, FrameAnalysis.twistStiffness(lad.frame(), List.of(lad.station(2)))[0]);
    }

    @Test void aBackboneTwistsAsItsTubesInSeries() {
        double[] xs = {0, 1.1, 2.5, 4.0};
        for (Section s : List.of(Section.roundTube(0.12, 0.004), Section.channel(0.15, 0.06, 0.005))) {
            Frame f = new Frame();
            int[] node = new int[xs.length];
            for (int i = 0; i < xs.length; i++) { node[i] = f.node(new Vec3d(xs[i], 0.3, 0)); }
            for (int i = 0; i + 1 < xs.length; i++) { f.member(node[i], node[i + 1], s, STEEL, Vec3d.Y, Frame.RIVETED); }
            List<Station> stations = new ArrayList<>();
            for (int i = 0; i < xs.length; i++) { stations.add(Station.node(xs[i], node[i])); }
            double[][] k = FrameAnalysis.twistStiffness(f, stations);
            double[] seg = new double[xs.length - 1];
            for (int i = 0; i < seg.length; i++) { seg[i] = f.torsionNm2(i) / (xs[i + 1] - xs[i]); }
            double[][] chain = new double[xs.length][xs.length];
            for (int i = 0; i < seg.length; i++) {
                chain[i][i] += seg[i]; chain[i + 1][i + 1] += seg[i]; chain[i][i + 1] -= seg[i]; chain[i + 1][i] -= seg[i];
            }
            for (int i = 0; i < xs.length; i++) {
                for (int j = 0; j < xs.length; j++) { assertEquals(chain[i][j], k[i][j], 1e-9 * seg[0], s.kind() + " K[" + i + "][" + j + "]"); }
            }
        }
    }

    /** A ladder on three axles (rear two at xs[0], xs[1], front at xs[4]); the wheels' nodes, rear left first. */
    static int[] axles(Ladder lad) {
        return new int[]{lad.left()[0], lad.right()[0], lad.left()[1], lad.right()[1], lad.left()[4], lad.right()[4]};
    }

    static void assertBalanced(Frame f, LoadResult r, String what) {
        double[] loads = r.loads();
        Vec3d force = Vec3d.ZERO, moment = Vec3d.ZERO;
        double size = 0;
        for (int node = 0; node < f.nodeCount(); node++) {
            Vec3d fo = new Vec3d(loads[node * 6], loads[node * 6 + 1], loads[node * 6 + 2]);
            Vec3d mo = new Vec3d(loads[node * 6 + 3], loads[node * 6 + 4], loads[node * 6 + 5]);
            force = force.add(fo);
            moment = moment.add(f.position(node).cross(fo)).add(mo);
            size += fo.length() + mo.length();
        }
        for (Reaction re : r.reactions()) {
            force = force.add(re.forceN());
            moment = moment.add(f.position(re.node()).cross(re.forceN())).add(re.momentNm());
            size += re.forceN().length();
        }
        assertEquals(0, force.length(), 1e-9 * size, what + ": forces " + force);
        assertEquals(0, moment.length(), 1e-9 * size * 6, what + ": moments " + moment);
    }

    @Test void reactionsBalanceTheLoads() {
        Ladder lad = ladder(new double[]{0, 1.0, 2.0, 3.2, 4.4, 5.6}, Frame.WELDED);
        Frame f = lad.frame();
        int[] wheels = axles(lad);
        List<Support> springs = new ArrayList<>();
        for (int w : wheels) { springs.add(new Support(w, 5e5, 2e5, 5e5, 0, 0, 0)); }
        List<NodeLoad> loads = List.of(
                new NodeLoad(lad.left()[2], new Vec3d(300, -4000, 120), new Vec3d(50, -20, 10)),
                new NodeLoad(lad.right()[3], new Vec3d(-150, -2500, -400), Vec3d.ZERO),
                new NodeLoad(lad.right()[5], new Vec3d(0, -6000, 0), new Vec3d(0, 0, 300)));
        LoadResult r = FrameAnalysis.loadCase(f, loads, springs);
        assertEquals(6, r.reactions().size());
        assertBalanced(f, r, "three axles on springs");
        for (Reaction re : r.reactions()) {
            // a spring pushes back k u
            Vec3d u = r.displacement(re.node());
            assertEquals(-2e5 * u.y(), re.forceN().y(), 1e-6 * Math.abs(re.forceN().y()) + 1e-9);
        }
        // vertical springs alone leave the frame free to slide and yaw: a sideways push is not held
        List<Support> vertical = new ArrayList<>();
        for (int w : wheels) { vertical.add(Support.vertical(w, 2e5)); }
        assertThrows(IllegalArgumentException.class, () -> FrameAnalysis.loadCase(f, List.of(NodeLoad.force(lad.left()[2], new Vec3d(0, 0, 100))), vertical));
        // the standard cases: each balances its loads with its reactions alone (the node holding the free motions carries nothing)
        double carried = 400 + 300;
        List<Mass> masses = List.of(new Mass(lad.left()[4], 200), new Mass(lad.right()[4], 200), new Mass(lad.left()[2], 150), new Mass(lad.right()[2], 150));
        List<Wheel> ws = new ArrayList<>();
        for (int w : wheels) { ws.add(new Wheel(w, 2e5)); }
        double g = 9.81;
        StandardCases cases = FrameAnalysis.standardCases(f, masses, ws, -0.35, g);
        double weight = (FrameAnalysis.massKg(f) + carried) * g;
        double nodeMass = 0;
        for (double m : FrameAnalysis.nodeMassesKg(f)) { nodeMass += m; }
        assertEquals(FrameAnalysis.massKg(f), nodeMass, 1e-12 * nodeMass);
        for (var c : List.of(cases.bump(), cases.ditch(), cases.braking(), cases.cornering())) { assertBalanced(f, c, "standard case"); }
        assertEquals(FrameAnalysis.BUMP_G * weight, upward(cases.bump()), 1e-9 * weight);
        assertEquals(weight, upward(cases.ditch()), 1e-9 * weight);
        // in the ditch the springs only push, and the port front one stands on ground DITCH_DROP_M lower
        for (Reaction re : cases.ditch().reactions()) {
            assertTrue(re.forceN().y() > -1e-9 * weight, "a spring pulls at node " + re.node());
            double ground = re.node() == lad.left()[4] ? -FrameAnalysis.DITCH_DROP_M : 0;
            assertEquals(-2e5 * (cases.ditch().displacement(re.node()).y() - ground), re.forceN().y(), 1e-9 * weight, "node " + re.node());
        }
        // braking moves load forward, cornering to starboard
        assertTrue(load(cases.braking(), lad.left()[4]) > load(cases.bump(), lad.left()[4]) / FrameAnalysis.BUMP_G);
        assertTrue(load(cases.cornering(), lad.right()[4]) > load(cases.cornering(), lad.left()[4]));
        // the tyres take 0.8 g in all, each wheel in proportion to its load in the stop or the turn
        double[] nodeWeight = FrameAnalysis.nodeMassesKg(f);
        for (Mass m : masses) { nodeWeight[m.node()] += m.kg(); }
        for (int axis : new int[]{0, 2}) {
            LoadResult c = axis == 0 ? cases.braking() : cases.cornering();
            double ratio = axis == 0 ? FrameAnalysis.BRAKING_G : FrameAnalysis.CORNERING_G, tyres = 0;
            for (int w : wheels) {
                double tyre = c.loads()[w * 6 + axis] - ratio * nodeWeight[w] * g;
                assertEquals(-ratio * load(c, w), tyre, 1e-9 * weight, "tyre force in proportion to the wheel's load");
                tyres += tyre;
            }
            assertEquals(-ratio * weight, tyres, 1e-9 * weight);
        }
        System.out.printf("three-axle ladder %.0f kg: usage bump %.3f, ditch %.3f, braking %.3f, cornering %.3f%n", FrameAnalysis.massKg(f),
                cases.bump().maxUsage(), cases.ditch().maxUsage(), cases.braking().maxUsage(), cases.cornering().maxUsage());
    }

    static double upward(LoadResult r) {
        double s = 0;
        for (Reaction re : r.reactions()) { s += re.forceN().y(); }
        return s;
    }

    static double load(LoadResult r, int node) {
        for (Reaction re : r.reactions()) { if (re.node() == node) { return re.forceN().y(); } }
        return 0;
    }

    @Test void aTrailerStandsOnItsAxlesAndItsKingpin() {
        // 0.36: a 12 m semi-trailer ladder, 20 t, on a tandem at the rear and its kingpin (both rails at the front) in a fifth wheel
        double[] xs = {0, 1.0, 2.3, 3.6, 5.0, 6.5, 8.0, 9.5, 11.0, 12.0};
        Ladder lad = ladder(xs, Section.channel(0.3, 0.09, 0.009), Section.channel(0.2, 0.07, 0.006), Frame.WELDED, Frame.WELDED, 0.45);
        Frame f = lad.frame();
        double g = 9.81, cargo = 20000 - FrameAnalysis.massKg(f);
        List<Mass> masses = new ArrayList<>();
        for (int i = 0; i < xs.length; i++) { masses.add(new Mass(lad.left()[i], cargo / 20)); masses.add(new Mass(lad.right()[i], cargo / 20)); }
        List<Wheel> wheels = new ArrayList<>();
        for (int a : new int[]{1, 2}) { wheels.add(new Wheel(lad.left()[a], 8e5)); wheels.add(new Wheel(lad.right()[a], 8e5)); }
        List<Wheel> pins = List.of(new Wheel(lad.left()[9], Support.FIXED), new Wheel(lad.right()[9], Support.FIXED));
        StandardCases cases = FrameAnalysis.standardCases(f, masses, wheels, pins, -0.6, g);
        double weight = 20000 * g, pinned = load(cases.bump(), lad.left()[9]) + load(cases.bump(), lad.right()[9]);
        double total = 0;
        for (Reaction re : cases.bump().reactions()) { total += re.forceN().y(); }
        assertEquals(FrameAnalysis.BUMP_G * weight, total, 1e-6 * weight, "the axles and the kingpin hold the bump");
        // the kingpin 12 m ahead of the COM at 6 m, the tandem about 1.65 m behind it: a little under half the weight on the pin
        assertTrue(pinned > 0.35 * FrameAnalysis.BUMP_G * weight && pinned < 0.5 * FrameAnalysis.BUMP_G * weight, "on the kingpin: " + pinned / FrameAnalysis.BUMP_G / weight);
        // braking: the tyres and the kingpin together take 0.8 g, the kingpin its share at its own height
        double[] applied = cases.braking().loads();
        double forward = 0;
        for (int i = 0; i < applied.length; i += 6) { forward += applied[i]; }
        assertEquals(0, forward, 1e-6 * weight, "the inertia held, nothing left over");
        double pinForward = applied[lad.left()[9] * 6] + applied[lad.right()[9] * 6];
        assertTrue(pinForward < -0.25 * FrameAnalysis.BRAKING_G * weight, "the tractor holds part of it: " + pinForward);
        // the ditch drops the trailer's own front port wheel, not the kingpin
        assertNotNull(cases.ditch());
        assertTrue(load(cases.ditch(), lad.left()[9]) > 0 && load(cases.ditch(), lad.right()[9]) > 0, "the kingpin still carries");
        assertTrue(load(cases.ditch(), lad.left()[2]) < load(cases.ditch(), lad.right()[2]), "the port front wheel is in the ditch");
        // no pins: the same as before
        List<Wheel> four = new ArrayList<>(wheels);
        four.add(new Wheel(lad.left()[8], 8e5)); four.add(new Wheel(lad.right()[8], 8e5));
        StandardCases old = FrameAnalysis.standardCases(f, masses, four, -0.6, g), none = FrameAnalysis.standardCases(f, masses, four, List.of(), -0.6, g);
        assertEquals(old.maxUsage(), none.maxUsage());
        assertArrayEquals(old.braking().displacements(), none.braking().displacements());
        assertArrayEquals(old.ditch().displacements(), none.ditch().displacements());
        assertThrows(IllegalArgumentException.class, () -> FrameAnalysis.standardCases(f, masses, wheels, List.of(new Wheel(lad.left()[1], 1e6)), -0.6, g));
    }

    @Test void aFrameInTwoPiecesNamesTheNodeLeftOut() {
        Ladder lad = ladder(new double[]{0, 1.5, 3.0}, Frame.WELDED);
        Frame f = lad.frame();
        int loose = f.node(new Vec3d(1.0, 0.5, 0)), other = f.node(new Vec3d(2.0, 0.5, 0));
        f.member(loose, other, Section.roundTube(0.03, 0.002), STEEL, Vec3d.Y, Frame.WELDED);
        FrameDisconnectedException e = assertThrows(FrameDisconnectedException.class,
                () -> FrameAnalysis.twistStiffness(f, List.of(lad.station(0), lad.station(2))));
        assertEquals(loose, e.node());
        assertTrue(e.getMessage().contains("節點 " + loose + " 沒接上"), e.getMessage());
        // a node joined to nothing
        Ladder lone = ladder(new double[]{0, 1.5, 3.0}, Frame.WELDED);
        int alone = lone.frame().node(new Vec3d(1, 1, 1));
        e = assertThrows(FrameDisconnectedException.class, () -> FrameAnalysis.bendingStiffness(lone.frame(), lone.station(0), lone.station(2), lone.station(1)));
        assertEquals(alone, e.node());
        // the solver's own pivot check, with no supports at all: the frame floats
        e = assertThrows(FrameDisconnectedException.class, () -> FrameSolver.factor(ladder(new double[]{0, 1}, Frame.WELDED).frame(), null, null));
        assertTrue(e.getMessage().contains("沒接上"), e.getMessage());
    }

    /** A tube space frame: rings of 8 nodes (a 1.2 x 0.6 m box section) every 0.18 m, longerons and diagonals. */
    record TubeFrame(Frame frame, int rings) {
        int node(int ring, int k) { return ring * 8 + k; }
    }

    static TubeFrame tubeFrame(int rings) {
        Frame f = new Frame();
        double[][] ring = {{-0.6, 0}, {0, 0}, {0.6, 0}, {0.6, 0.3}, {0.6, 0.6}, {0, 0.6}, {-0.6, 0.6}, {-0.6, 0.3}};
        for (int i = 0; i < rings; i++) { for (double[] p : ring) { f.node(new Vec3d(0.18 * i, p[1], p[0])); } }
        Section tube = Section.roundTube(0.0381, 0.0016), main = Section.roundTube(0.0445, 0.0024);
        for (int i = 0; i < rings; i++) {
            for (int k = 0; k < 8; k++) { f.member(i * 8 + k, i * 8 + (k + 1) % 8, tube, STEEL, Vec3d.X, Frame.WELDED); }
            if (i + 1 == rings) { continue; }
            for (int k = 0; k < 8; k++) {
                f.member(i * 8 + k, (i + 1) * 8 + k, k % 2 == 0 ? main : tube, STEEL, Vec3d.Y, Frame.WELDED);
                int a = i % 2 == 0 ? i * 8 + k : i * 8 + (k + 1) % 8, b = i % 2 == 0 ? (i + 1) * 8 + (k + 1) % 8 : (i + 1) * 8 + k;
                f.member(a, b, tube, STEEL, Vec3d.Y, Frame.WELDED);
            }
        }
        return new TubeFrame(f, rings);
    }

    @Test void aTwoHundredNodeTubeFrameIsAnalysedInUnderFiftyMilliseconds() {
        TubeFrame tf = tubeFrame(25);
        Frame f = tf.frame();
        assertEquals(200, f.nodeCount());
        int[] at = {2, 8, 16, 22};
        List<Station> stations = new ArrayList<>();
        for (int ring : at) { stations.add(Station.pair(0.18 * ring, tf.node(ring, 0), tf.node(ring, 2))); }
        List<Wheel> wheels = List.of(new Wheel(tf.node(2, 0), 3e4), new Wheel(tf.node(2, 2), 3e4), new Wheel(tf.node(22, 0), 3e4), new Wheel(tf.node(22, 2), 3e4));
        List<Mass> masses = List.of(new Mass(tf.node(18, 0), 60), new Mass(tf.node(18, 1), 80), new Mass(tf.node(18, 2), 60), new Mass(tf.node(12, 1), 80));
        double[] result = new double[4];
        Runnable all = () -> {
            double[][] k = FrameAnalysis.twistStiffness(f, stations);
            double bend = FrameAnalysis.bendingStiffness(f, stations.get(0), stations.get(3), Station.pair(0.18 * 12, tf.node(12, 0), tf.node(12, 2)));
            StandardCases cases = FrameAnalysis.standardCases(f, masses, wheels, -0.25, 9.81);
            result[0] = -k[0][3]; result[1] = bend; result[2] = cases.maxUsage(); result[3] = FrameAnalysis.massKg(f);
        };
        long t0 = System.nanoTime();
        all.run();
        double cold = (System.nanoTime() - t0) / 1e6;
        for (int i = 0; i < 5; i++) { all.run(); }
        double[] ms = new double[11];
        for (int i = 0; i < ms.length; i++) { long s = System.nanoTime(); all.run(); ms[i] = (System.nanoTime() - s) / 1e6; }
        Arrays.sort(ms);
        double median = ms[ms.length / 2];
        int band = FrameSolver.factor(f, null, fixedNode(f, 0)).bandwidth();
        System.out.printf("tube frame: %d nodes, %d members, %d freedoms, half-bandwidth %d; cold %.1f ms, warm median %.1f ms (best %.1f)%n",
                f.nodeCount(), f.memberCount(), 6 * f.nodeCount(), band, cold, median, ms[0]);
        System.out.printf("tube frame: %.1f kg, twist between the outer stations %.0f N m/deg, bending %.0f kN/m, worst usage %.3f%n",
                result[3], result[0] * Math.PI / 180, result[1] / 1000, result[2]);
        assertTrue(median < 50, "the whole set takes " + median + " ms");
        assertTrue(result[0] > 0 && result[1] > 0 && result[2] > 0);
    }

    static boolean[] fixedNode(Frame f, int node) {
        boolean[] fixed = new boolean[6 * f.nodeCount()];
        Arrays.fill(fixed, node * 6, node * 6 + 6, true);
        return fixed;
    }

    @Test void aMembersStressCountsItsLoads() {
        // axial plus bending at the corner, and torsion, as von Mises
        Section s = Section.rectTube(0.05, 0.08, 0.003);
        Frame f = new Frame();
        int a = f.node(Vec3d.ZERO), b = f.node(new Vec3d(1.2, 0, 0));
        f.member(a, b, s, STEEL, Vec3d.Y, Frame.WELDED);
        double pull = 20000, side = 300, up = 500, torque = 200;
        LoadResult r = FrameAnalysis.loadCase(f, List.of(new NodeLoad(b, new Vec3d(pull, up, side), new Vec3d(torque, 0, 0))), List.of(Support.fixed(a)));
        MemberResult m = r.members().get(0);
        assertEquals(pull, m.axialN(), 1e-9 * pull);
        assertEquals(torque, m.torqueNm(), 1e-9 * torque);
        // at the root: Mz = up L, My = -side L (about the member's y)
        double sigma = pull / s.areaM2() + side * 1.2 * s.outerZM() / s.iyM4() + up * 1.2 * s.outerYM() / s.izM4();
        double tau = torque / (2 * s.enclosedAreaM2() * s.wallM());
        double vm = Math.sqrt(sigma * sigma + 3 * tau * tau);
        assertEquals(vm, m.vonMisesPa(), 1e-9 * vm);
        assertEquals(vm / STEEL.yieldPa(), m.usage(), 1e-9);
        assertEquals(0, r.worstMember());
    }
}
