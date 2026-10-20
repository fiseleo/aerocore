package dev.mcmod.aerocore.api.frame;

import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.LoadResult;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.NodeLoad;
import dev.mcmod.aerocore.api.frame.FrameAnalysis.Support;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Sections: their outlines against their properties, Bredt's tubes, and open sections with their warping held. */
class SectionTest {
    static final FrameMaterial STEEL = new FrameMaterial(7850, 210e9, 81e9, 355e6);

    /** Area, first moments about z and y, Iy = ∫ z² dA and Iz = ∫ y² dA of the outline's loops (holes clockwise). */
    static double[] polygon(double[][] loops) {
        double[] p = new double[5];
        for (double[] loop : loops) {
            int n = loop.length / 2;
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                double zi = loop[2 * i], yi = loop[2 * i + 1], zj = loop[2 * j], yj = loop[2 * j + 1];
                double c = zi * yj - zj * yi;
                p[0] += c / 2;
                p[1] += (zi + zj) * c / 6;
                p[2] += (yi + yj) * c / 6;
                p[3] += (zi * zi + zi * zj + zj * zj) * c / 12;
                p[4] += (yi * yi + yi * yj + yj * yj) * c / 12;
            }
        }
        return p;
    }

    @Test void outlinesHoldTheSectionsAreasAndMoments() {
        for (Section s : List.of(Section.rectTube(0.08, 0.12, 0.004), Section.box(0.1, 0.16, 0.003, 0.006),
                Section.channel(0.15, 0.06, 0.005), Section.iBeam(0.2, 0.1, 0.0056, 0.0085))) {
            double[] p = polygon(s.outline());
            String what = s.kind().toString();
            assertEquals(s.areaM2(), p[0], 1e-12 * s.areaM2(), what + " area");
            assertEquals(0, p[1] / p[0], 1e-12, what + " centroid across");
            assertEquals(0, p[2] / p[0], 1e-12, what + " centroid up");
            assertEquals(s.iyM4(), p[3], 1e-12 * s.iyM4(), what + " Iy");
            assertEquals(s.izM4(), p[4], 1e-12 * s.izM4(), what + " Iz");
            double far = 0, wide = 0;
            for (double[] loop : s.outline()) {
                for (int i = 0; i < loop.length; i += 2) { wide = Math.max(wide, Math.abs(loop[i])); far = Math.max(far, Math.abs(loop[i + 1])); }
            }
            assertEquals(s.outerYM(), far, 1e-15, what + " farthest fibre up");
            assertEquals(s.outerZM(), wide, 1e-15, what + " farthest fibre across");
        }
        // a round tube's outline is a 32-gon: within a couple of per cent
        Section round = Section.roundTube(0.05, 0.003);
        double[] p = polygon(round.outline());
        assertEquals(round.areaM2(), p[0], 0.01 * round.areaM2());
        assertEquals(round.izM4(), p[4], 0.02 * round.izM4());
        assertEquals(round.iyM4(), p[3], 0.02 * round.iyM4());
    }

    @Test void aRectangularTubeTwistsAsBredtSays() {
        double b = 0.08, h = 0.12, t = 0.004;
        Section tube = Section.rectTube(b, h, t);
        // Bredt: J = 4 A_m² / ∮ ds / t on the wall's midline
        double am = (b - t) * (h - t), loop = 2 * (b - t) / t + 2 * (h - t) / t;
        assertEquals(4 * am * am / loop, tube.torsionM4(), 1e-14 * tube.torsionM4());
        assertEquals(2 * t * (b - t) * (b - t) * (h - t) * (h - t) / (b + h - 2 * t), tube.torsionM4(), 1e-14 * tube.torsionM4());
        assertTrue(tube.closed());
        assertEquals(am, tube.enclosedAreaM2(), 1e-18);
        Section same = Section.box(b, h, t, t);
        assertEquals(tube.areaM2(), same.areaM2(), 1e-18); assertEquals(tube.iyM4(), same.iyM4(), 1e-20);
        assertEquals(tube.izM4(), same.izM4(), 1e-20); assertEquals(tube.torsionM4(), same.torsionM4(), 1e-20);
        // walls of two thicknesses: ∮ ds / t = 2 (b - tw) / tf + 2 (h - tf) / tw, the shear stress set by the thinner
        double tw = 0.003, tf = 0.006;
        Section mixed = Section.box(b, h, tw, tf);
        double amm = (b - tw) * (h - tf);
        assertEquals(4 * amm * amm / (2 * (b - tw) / tf + 2 * (h - tf) / tw), mixed.torsionM4(), 1e-14 * mixed.torsionM4());
        assertEquals(tw, mixed.wallM());
        // Bredt on a thin round tube is the thin-wall limit of its exact J = 2 I
        double d = 0.1, tt = 0.001, rm = (d - tt) / 2;
        Section thin = Section.roundTube(d, tt);
        assertEquals(2 * Math.PI * rm * rm * rm * tt, thin.torsionM4(), 2e-4 * thin.torsionM4());
        // a member of the rectangular tube twists T L / (G J), welded or bolted alike: a closed section has no warping to hold
        double length = 1.5, torque = 800;
        for (double joint : new double[]{Frame.WELDED, Frame.BOLTED}) {
            Frame f = new Frame();
            int a = f.node(new Vec3d(0, 0, 0)), e = f.node(new Vec3d(length, 0, 0));
            f.member(a, e, tube, STEEL, Vec3d.Y, joint);
            LoadResult r = FrameAnalysis.loadCase(f, List.of(new NodeLoad(e, Vec3d.ZERO, new Vec3d(torque, 0, 0))), List.of(Support.fixed(a)));
            double twist = torque * length / (STEEL.shearModulusPa() * tube.torsionM4());
            assertEquals(twist, r.rotation(e).x(), 1e-9 * twist, "joint " + joint);
            // Bredt's shear stress T / (2 A_m t), as von Mises sqrt(3) τ
            double tau = torque / (2 * am * t);
            assertEquals(Math.sqrt(3) * tau / STEEL.yieldPa(), r.members().get(0).usage(), 1e-9 * tau / STEEL.yieldPa());
        }
    }

    /**
     * The twist rate ψ = φ' at steps + 1 points along a member under torque T with both ends' warping held, from the
     * restrained-torsion equation T = G J ψ - E Cw ψ'' (ψ = 0 at both ends), by finite differences.
     */
    static double[] twistRate(double gj, double ecw, double length, int steps, double torque) {
        double h = length / steps, off = ecw / (h * h), diag = -2 * ecw / (h * h) - gj;
        int n = steps - 1;
        double[] c = new double[n], d = new double[n];
        // Thomas algorithm on the tridiagonal system
        c[0] = off / diag; d[0] = -torque / diag;
        for (int i = 1; i < n; i++) {
            double m = diag - off * c[i - 1];
            c[i] = off / m;
            d[i] = (-torque - off * d[i - 1]) / m;
        }
        double[] psi = new double[steps + 1];
        psi[n] = d[n - 1];
        for (int i = n - 2; i >= 0; i--) { psi[i + 1] = d[i] - c[i] * psi[i + 2]; }
        return psi;
    }

    /** G J_eff = T L / φ(L) from twistRate. */
    static double restrainedByDifferences(double gj, double ecw, double length, int steps) {
        double[] psi = twistRate(gj, ecw, length, steps, 1);
        double twist = 0;
        for (double v : psi) { twist += length / steps * v; }
        return length / twist;
    }

    @Test void aHeldOpenSectionCarriesItsTorqueByWarpingAsVlasovSays() {
        double length = 1.2, torque = 500;
        int steps = 4000;
        for (Section open : List.of(Section.channel(0.15, 0.06, 0.005), Section.iBeam(0.2, 0.1, 0.0056, 0.0085))) {
            String what = open.kind().toString();
            double gj = STEEL.shearModulusPa() * open.torsionM4(), ecw = STEEL.modulusPa() * open.warpingM6();
            double kappa = Math.sqrt(gj / ecw), u = kappa * length / 2;
            // without the closed forms: the St Venant torque G J ψ, greatest at mid-length, and the bimoment -E Cw ψ', greatest at the ends
            double[] psi = twistRate(gj, ecw, length, steps, torque);
            double h = length / steps, stVenant = 0;
            for (double v : psi) { stVenant = Math.max(stVenant, gj * v); }
            double bimoment = ecw * (-3 * psi[0] + 4 * psi[1] - psi[2]) / (2 * h);
            assertEquals(torque * (1 - 1 / Math.cosh(u)), stVenant, 1e-6 * torque, what + " St Venant torque at mid-length");
            assertEquals(torque * Math.tanh(u) / kappa, bimoment, 1e-5 * bimoment, what + " bimoment at the ends");
            // the section's stresses, held (welded), half held and free
            Section.TorsionStress held = open.torsionStress(STEEL, length, Frame.WELDED, torque);
            Section.TorsionStress free = open.torsionStress(STEEL, length, Frame.UNRESTRAINED, torque);
            Section.TorsionStress half = open.torsionStress(STEEL, length, 0.5, torque);
            assertEquals(stVenant * open.wallM() / open.torsionM4(), held.shearPa(), 1e-6 * held.shearPa(), what);
            assertEquals(bimoment * open.sectorialM2() / open.warpingM6(), held.warpingPa(), 1e-5 * held.warpingPa(), what);
            assertEquals(open.torsionShearPa(torque), free.shearPa(), 0);
            assertEquals(0, free.warpingPa(), 0);
            assertEquals(0.5 * (held.shearPa() + free.shearPa()), half.shearPa(), 1e-9 * half.shearPa());
            assertEquals(0.5 * held.warpingPa(), half.warpingPa(), 1e-9 * half.warpingPa());
            System.out.printf("%s %.0f mm held: St Venant shear %.1f MPa (free %.1f), warping stress %.1f MPa under %.0f N m%n",
                    what, length * 1000, held.shearPa() / 1e6, free.shearPa() / 1e6, held.warpingPa() / 1e6, torque);
            // in a frame: a member held at one end, the torque at the other
            Frame f = new Frame();
            int a = f.node(Vec3d.ZERO), e = f.node(new Vec3d(length, 0, 0));
            f.member(a, e, open, STEEL, Vec3d.Y, Frame.WELDED);
            LoadResult r = FrameAnalysis.loadCase(f, List.of(new NodeLoad(e, Vec3d.ZERO, new Vec3d(torque, 0, 0))), List.of(Support.fixed(a)));
            double vm = Math.sqrt(held.warpingPa() * held.warpingPa() + 3 * held.shearPa() * held.shearPa());
            assertEquals(vm, r.members().get(0).vonMisesPa(), 1e-9 * vm, what + " von Mises");
        }
        // ω_max from the shear centre: Cw = ∫ ω² t ds along the midline
        double hc = 0.15, bc = 0.06, tc = 0.005, hm = hc - tc, bm = bc - tc / 2, e = 3 * bm * bm / (6 * bm + hm);
        Section c = Section.channel(hc, bc, tc);
        double cw = tc * e * e * hm * hm * hm / 12 + tc * hm * hm / 6 * (Math.pow(bm - e, 3) + Math.pow(e, 3));
        assertEquals(c.warpingM6(), cw, 1e-12 * cw, "the channel's ω about its shear centre");
        assertEquals(hm * (bm - e) / 2, c.sectorialM2(), 1e-15);
        Section i = Section.iBeam(0.2, 0.1, 0.0056, 0.0085);
        assertEquals(i.warpingM6(), i.sectorialM2() * i.sectorialM2() * 2 * 0.0085 * 0.1 / 3, 1e-12 * i.warpingM6(), "the I-beam's ω");
        assertEquals(0, Section.rectTube(0.08, 0.12, 0.004).sectorialM2());
    }

    @Test void heldWarpingStiffensAnOpenSectionAsKollbrunnerAndBaslerSay() {
        double length = 1.2;
        for (Section open : List.of(Section.channel(0.15, 0.06, 0.005), Section.iBeam(0.2, 0.1, 0.0056, 0.0085))) {
            String what = open.kind().toString();
            double gj = STEEL.shearModulusPa() * open.torsionM4(), ecw = STEEL.modulusPa() * open.warpingM6();
            double held = open.restrainedTorsionNm2(STEEL, length);
            // the closed form against the differential equation, solved without it
            double numeric = restrainedByDifferences(gj, ecw, length, 4000);
            assertEquals(numeric, held, 1e-6 * numeric, what + " against the restrained-torsion equation");
            double u = 0.5 * length * Math.sqrt(gj / ecw);
            assertEquals(gj / (1 - Math.tanh(u) / u), held, 1e-12 * held, what);
            System.out.printf("%s %.0f mm long: κL/2 = %.3f, G J_eff / G J = %.3f%n", what, length * 1000, u, held / gj);
            assertTrue(held > 1.5 * gj, what + " held warping stiffens it");
            // long: toward G J; short: toward the warping stiffness 12 E Cw / L²
            assertEquals(gj, open.restrainedTorsionNm2(STEEL, 2000), 2e-3 * gj, what + " long");
            double tiny = 1e-4;
            assertEquals(12 * ecw / (tiny * tiny), open.restrainedTorsionNm2(STEEL, tiny), 1e-6 * 12 * ecw / (tiny * tiny), what + " short");
            // in a frame, the joint factor blends G J toward it: a member held at one end twists T L / (G J + f (G J_eff - G J))
            for (double joint : new double[]{Frame.WELDED, Frame.RIVETED, Frame.BOLTED, Frame.UNRESTRAINED}) {
                Frame f = new Frame();
                int a = f.node(new Vec3d(0.2, 0.1, 0.4)), e = f.node(new Vec3d(0.2 + length, 0.1, 0.4));
                f.member(a, e, open, STEEL, Vec3d.Y, joint);
                double torque = 300;
                LoadResult r = FrameAnalysis.loadCase(f, List.of(new NodeLoad(e, Vec3d.ZERO, new Vec3d(torque, 0, 0))), List.of(Support.fixed(a)));
                double twist = torque * length / (gj + joint * (held - gj));
                assertEquals(twist, r.rotation(e).x(), 1e-9 * twist, what + " joint " + joint);
                assertEquals(gj + joint * (held - gj), f.torsionNm2(0), 1e-12 * held);
            }
        }
        Section closed = Section.rectTube(0.08, 0.12, 0.004);
        assertEquals(STEEL.shearModulusPa() * closed.torsionM4(), closed.restrainedTorsionNm2(STEEL, 0.3), 0);
    }
}
