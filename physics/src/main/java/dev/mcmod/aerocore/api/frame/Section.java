package dev.mcmod.aerocore.api.frame;

import java.util.Objects;

/**
 * A member's cross-section (0.33) about its centroid, in the member's axes (Frame.axes): y up (toward the member's up
 * vector), z across. Area (m^2); second moments Iy = ∫ z² dA (bending about y: sideways) and Iz = ∫ y² dA (bending about z:
 * in the up plane), m^4; St Venant torsion constant J (m^4); warping constant Cw (m^6; 0 for a closed section, whose
 * warping is left out); the farthest fibres from the centroid along y and z (m); closed (a tube) or open; the wall that
 * sets the torsional shear stress (m: a closed section's thinnest, an open one's thickest); the area a closed wall's
 * midline encloses (m^2, 0 when open); the outline, for drawing; and the greatest normalised sectorial coordinate ω_max
 * (m^2, about the shear centre: where an open section's warping stress peaks, at the flange tips; 0 when closed or not
 * known, and then no warping stress is counted).
 * <p>Outline: one row per closed loop of (z, y) pairs about the centroid, {z0, y0, z1, y1, ...} in m; the first loop the
 * outside, counter-clockwise with z to the right and y up, the others holes, clockwise.
 * <p>Thin-walled sections with sharp corners (no corner radii); the torsion and warping constants use the walls'
 * midlines. A channel's shear centre lies off its web, but the frame model twists every member about its centroid.
 */
public record Section(Kind kind, double areaM2, double iyM4, double izM4, double torsionM4, double warpingM6,
                      double outerYM, double outerZM, boolean closed, double wallM, double enclosedAreaM2, double[][] outline,
                      double sectorialM2) {
    /** The section's form; CUSTOM for one made from its properties. */
    public enum Kind { RECT_TUBE, BOX, ROUND_TUBE, CHANNEL, I_BEAM, CUSTOM }

    /**
     * The torsional stresses in a member (Pa): the greatest St Venant shear stress and the greatest warping normal stress
     * (at an open section's flange tips, at the member's ends).
     */
    public record TorsionStress(double shearPa, double warpingPa) {}

    /** Points round a round tube's outline loops. */
    private static final int ROUND_POINTS = 32;

    /** A section without its sectorial coordinate (ω_max 0: no warping stress is counted). */
    public Section(Kind kind, double areaM2, double iyM4, double izM4, double torsionM4, double warpingM6,
                   double outerYM, double outerZM, boolean closed, double wallM, double enclosedAreaM2, double[][] outline) {
        this(kind, areaM2, iyM4, izM4, torsionM4, warpingM6, outerYM, outerZM, closed, wallM, enclosedAreaM2, outline, 0);
    }

    public Section {
        Require.nonnegative(sectorialM2, "Sectorial coordinate");
        Objects.requireNonNull(kind, "kind");
        Require.positive(areaM2, "Area"); Require.positive(iyM4, "Iy"); Require.positive(izM4, "Iz");
        Require.positive(torsionM4, "Torsion constant"); Require.nonnegative(warpingM6, "Warping constant");
        Require.positive(outerYM, "Farthest fibre along y"); Require.positive(outerZM, "Farthest fibre along z");
        Require.positive(wallM, "Wall"); Require.nonnegative(enclosedAreaM2, "Enclosed area");
        if (closed && !(enclosedAreaM2 > 0)) { throw new IllegalArgumentException("A closed section encloses an area"); }
        Objects.requireNonNull(outline, "outline");
        outline = copy(outline);
        for (double[] loop : outline) {
            if (loop.length % 2 != 0) { throw new IllegalArgumentException("An outline loop is (z, y) pairs"); }
            for (double v : loop) { Require.finite(v, "Outline point"); }
        }
    }

    /** The outline loops (a copy). */
    @Override public double[][] outline() { return copy(outline); }

    /** Mass per length (kg/m) in a material. */
    public double kgPerM(FrameMaterial material) { return areaM2 * material.densityKgM3(); }

    /**
     * Torsional stiffness G J_eff (N m^2) of a member of this section and length with both ends' warping held. Under an end
     * torque T such a member twists T L / (G J) (1 - tanh(u) / u), u = κ L / 2, κ = sqrt(G J / (E Cw)) (Kollbrunner and
     * Basler, Torsion in Structures), so G J_eff = G J / (1 - tanh(u) / u): stiffer the shorter it is, toward 12 E Cw / L²,
     * and G J when long. G J for a closed section or an open one without a warping constant.
     */
    public double restrainedTorsionNm2(FrameMaterial material, double lengthM) {
        Require.positive(lengthM, "Length");
        double gj = material.shearModulusPa() * torsionM4;
        if (closed || warpingM6 == 0) { return gj; }
        double u = 0.5 * lengthM * Math.sqrt(gj / (material.modulusPa() * warpingM6));
        return gj / unrestrainedShare(u);
    }

    /** 1 - tanh(u) / u, by its series where the difference would cancel. */
    static double unrestrainedShare(double u) {
        if (u < 1e-2) { double u2 = u * u; return u2 / 3 - 2 * u2 * u2 / 15 + 17 * u2 * u2 * u2 / 315; }
        return 1 - Math.tanh(u) / u;
    }

    /**
     * The greatest torsional shear stress (Pa) when the whole torque (N m) is St Venant's: T / (2 A_m t) closed (Bredt),
     * T t / J open (free to warp).
     */
    public double torsionShearPa(double torqueNm) {
        double t = Math.abs(torqueNm);
        return closed ? t / (2 * enclosedAreaM2 * wallM) : t * wallM / torsionM4;
    }

    /**
     * The torsional stresses in a member of this section, length L and joint factor f under a torque T (N m). Closed, or
     * open with no warping constant or sectorial coordinate: all of T as St Venant shear (torsionShearPa), no warping
     * stress. Open with both ends' warping held, Vlasov's T = G J φ' - E Cw φ''' with φ' = 0 at both ends gives
     * (Kollbrunner and Basler, Torsion in Structures; Trahair, Flexural-Torsional Buckling of Structures):
     * St Venant torque T (1 - cosh(κ (x - L / 2)) / cosh(u)), greatest at mid-length, T (1 - 1 / cosh u), u = κ L / 2;
     * bimoment B = (T / κ) sinh(κ (x - L / 2)) / cosh(u), greatest at the ends, (T / κ) tanh u; warping stress
     * B ω_max / Cw. The joint factor blends the two: shear (1 - f) T t / J + f T (1 - 1 / cosh u) t / J, bimoment
     * f (T / κ) tanh u. Left out: the warping shear stress at the ends (the flanges' shear from the warping torque).
     */
    public TorsionStress torsionStress(FrameMaterial material, double lengthM, double jointFactor, double torqueNm) {
        Require.positive(lengthM, "Length");
        if (!(jointFactor >= 0 && jointFactor <= 1)) { throw new IllegalArgumentException("The joint factor is 0..1"); }
        double t = Math.abs(torqueNm);
        if (closed || warpingM6 == 0 || sectorialM2 == 0 || jointFactor == 0) { return new TorsionStress(torsionShearPa(t), 0); }
        double gj = material.shearModulusPa() * torsionM4, kappa = Math.sqrt(gj / (material.modulusPa() * warpingM6));
        double u = 0.5 * kappa * lengthM;
        double stVenant = t * ((1 - jointFactor) + jointFactor * (1 - 1 / Math.cosh(u)));
        double bimoment = jointFactor * t * Math.tanh(u) / kappa;
        return new TorsionStress(stVenant * wallM / torsionM4, bimoment * sectorialM2 / warpingM6);
    }

    /**
     * The greatest bending stress (Pa) under moments about the section's y and z (N m): a round tube's at the resultant
     * moment, any other's at the farthest corner (|My| z_max / Iy + |Mz| y_max / Iz).
     */
    public double bendingPa(double momentYNm, double momentZNm) {
        if (kind == Kind.ROUND_TUBE) { return Math.hypot(momentYNm, momentZNm) * outerYM / izM4; }
        return Math.abs(momentYNm) * outerZM / iyM4 + Math.abs(momentZNm) * outerYM / izM4;
    }

    /** A rectangular tube: width b (along z), height h (along y), wall t; J by Bredt, 2 t (b - t)² (h - t)² / (b + h - 2 t). */
    public static Section rectTube(double widthM, double heightM, double wallM) {
        return box(Kind.RECT_TUBE, widthM, heightM, wallM, wallM);
    }

    /**
     * A box section: width b (along z), height h (along y), side walls (webs) tw and top and bottom (flanges) tf. J by
     * Bredt, 4 A_m² / ∮ ds / t with A_m = (b - tw)(h - tf) and ∮ ds / t = 2 (b - tw) / tf + 2 (h - tf) / tw.
     */
    public static Section box(double widthM, double heightM, double webM, double flangeM) {
        return box(Kind.BOX, widthM, heightM, webM, flangeM);
    }

    private static Section box(Kind kind, double b, double h, double tw, double tf) {
        Require.positive(b, "Width"); Require.positive(h, "Height"); Require.positive(tw, "Web"); Require.positive(tf, "Flange");
        if (!(2 * tw < b) || !(2 * tf < h)) { throw new IllegalArgumentException("A tube's walls must leave it hollow"); }
        double bi = b - 2 * tw, hi = h - 2 * tf;
        double area = b * h - bi * hi;
        double iz = (b * h * h * h - bi * hi * hi * hi) / 12, iy = (h * b * b * b - hi * bi * bi * bi) / 12;
        double bm = b - tw, hm = h - tf, am = bm * hm;
        double j = 4 * am * am / (2 * bm / tf + 2 * hm / tw);
        return new Section(kind, area, iy, iz, j, 0, h / 2, b / 2, true, Math.min(tw, tf), am,
                new double[][]{rectangle(b / 2, h / 2, true), rectangle(bi / 2, hi / 2, false)});
    }

    /** A round tube: outside diameter d, wall t (t = d / 2: a solid bar); J = 2 I, exact for a tube. */
    public static Section roundTube(double diameterM, double wallM) {
        Require.positive(diameterM, "Diameter"); Require.positive(wallM, "Wall");
        if (!(wallM <= diameterM / 2)) { throw new IllegalArgumentException("A tube's wall is at most half its diameter"); }
        double di = diameterM - 2 * wallM;
        double area = Math.PI * (diameterM * diameterM - di * di) / 4;
        double i = Math.PI * (Math.pow(diameterM, 4) - Math.pow(di, 4)) / 64;
        double am = Math.PI * (diameterM - wallM) * (diameterM - wallM) / 4;
        double[][] loops = di > 0 ? new double[][]{circle(diameterM / 2, true), circle(di / 2, false)} : new double[][]{circle(diameterM / 2, true)};
        return new Section(Kind.ROUND_TUBE, area, i, i, 2 * i, 0, diameterM / 2, diameterM / 2, true, wallM, am, loops);
    }

    /**
     * A channel (C) of one thickness: height h (the web, along y), flange width b (along z, the flanges toward +z), wall t.
     * With the midline lengths h_m = h - t and b_m = b - t / 2: J = (h_m + 2 b_m) t³ / 3,
     * Cw = t b_m³ h_m² (3 b_m + 2 h_m) / (12 (6 b_m + h_m)); the shear centre e = 3 b_m² / (6 b_m + h_m) behind the web's
     * midline, and ω_max = h_m (b_m - e) / 2 at the flange tips (the web corners have h_m e / 2, less).
     */
    public static Section channel(double heightM, double flangeM, double wallM) {
        double h = heightM, b = flangeM, t = wallM;
        Require.positive(h, "Height"); Require.positive(b, "Flange width"); Require.positive(t, "Wall");
        if (!(2 * t < h) || !(t < b)) { throw new IllegalArgumentException("A channel's wall must be thinner than its web and flanges"); }
        double fl = b - t;
        double area = h * t + 2 * fl * t;
        // z from the web's back face toward the flanges' tips
        double zc = (h * t * t / 2 + 2 * fl * t * (t + fl / 2)) / area;
        double arm = (h - t) / 2;
        double iz = t * h * h * h / 12 + 2 * (fl * t * t * t / 12 + fl * t * arm * arm);
        double iy = h * t * t * t / 12 + h * t * sq(t / 2 - zc) + 2 * (t * fl * fl * fl / 12 + t * fl * sq(t + fl / 2 - zc));
        double hm = h - t, bm = b - t / 2;
        double j = (hm + 2 * bm) * t * t * t / 3;
        double cw = t * bm * bm * bm * hm * hm * (3 * bm + 2 * hm) / (12 * (6 * bm + hm));
        double y0 = h / 2, y1 = h / 2 - t;
        double shearCentre = 3 * bm * bm / (6 * bm + hm);
        double[] loop = shift(new double[]{0, -y0, b, -y0, b, -y1, t, -y1, t, y1, b, y1, b, y0, 0, y0}, -zc);
        return new Section(Kind.CHANNEL, area, iy, iz, j, cw, h / 2, Math.max(zc, b - zc), false, t, 0, new double[][]{loop},
                hm * (bm - shearCentre) / 2);
    }

    /**
     * An I-beam: height h (along y), flange width b (along z), web tw, flanges tf. With h_o = h - tf between the flanges'
     * midlines: J = (2 b tf³ + h_o tw³) / 3, Cw = tf b³ h_o² / 24, ω_max = h_o b / 4 at the flange tips.
     */
    public static Section iBeam(double heightM, double flangeWidthM, double webM, double flangeM) {
        double h = heightM, b = flangeWidthM, tw = webM, tf = flangeM;
        Require.positive(h, "Height"); Require.positive(b, "Flange width"); Require.positive(tw, "Web"); Require.positive(tf, "Flange");
        if (!(2 * tf < h) || !(tw < b)) { throw new IllegalArgumentException("An I-beam's web and flanges must leave it an I"); }
        double hw = h - 2 * tf;
        double area = 2 * b * tf + hw * tw;
        double iz = (b * h * h * h - (b - tw) * hw * hw * hw) / 12;
        double iy = 2 * tf * b * b * b / 12 + hw * tw * tw * tw / 12;
        double ho = h - tf;
        double j = (2 * b * tf * tf * tf + ho * tw * tw * tw) / 3;
        double cw = tf * b * b * b * ho * ho / 24;
        double z0 = b / 2, z1 = tw / 2, y0 = h / 2, y1 = h / 2 - tf;
        double[] loop = {-z0, -y0, z0, -y0, z0, -y1, z1, -y1, z1, y1, z0, y1, z0, y0, -z0, y0, -z0, y1, -z1, y1, -z1, -y1, -z0, -y1};
        return new Section(Kind.I_BEAM, area, iy, iz, j, cw, h / 2, b / 2, false, Math.max(tw, tf), 0, new double[][]{loop}, ho * b / 4);
    }

    private static double sq(double v) { return v * v; }

    private static double[] rectangle(double halfZ, double halfY, boolean outside) {
        return outside ? new double[]{-halfZ, -halfY, halfZ, -halfY, halfZ, halfY, -halfZ, halfY}
                : new double[]{-halfZ, -halfY, -halfZ, halfY, halfZ, halfY, halfZ, -halfY};
    }

    private static double[] circle(double r, boolean outside) {
        double[] loop = new double[2 * ROUND_POINTS];
        for (int i = 0; i < ROUND_POINTS; i++) {
            double a = 2 * Math.PI * (outside ? i : -i) / ROUND_POINTS;
            loop[2 * i] = r * Math.cos(a);
            loop[2 * i + 1] = r * Math.sin(a);
        }
        return loop;
    }

    private static double[] shift(double[] loop, double dz) {
        for (int i = 0; i < loop.length; i += 2) { loop[i] += dz; }
        return loop;
    }

    private static double[][] copy(double[][] loops) {
        double[][] out = new double[loops.length][];
        for (int i = 0; i < loops.length; i++) { out[i] = Objects.requireNonNull(loops[i], "outline loop").clone(); }
        return out;
    }
}
