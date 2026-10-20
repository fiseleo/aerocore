package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.AeroCoefficients;
import dev.mcmod.aerocore.api.AirfoilPolar;
import dev.mcmod.aerocore.api.FlowConditions;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * A section's polar at every angle (-180 to 180 degrees), Reynolds and Mach number, from its shape: AirfoilAnalysis
 * tabulated each degree to +-RANGE_DEG at the Reynolds numbers REYNOLDS (log-interpolated between them, held outside),
 * continued by a flat plate beyond. Never throws for finite input. Built once per shape (of() caches them).
 * <ul>
 * <li>Compressibility, attached flow: Prandtl and Glauert, 1/sqrt(1 - M^2) on the attached lift and moment (held
 *   above Mach 0.9).</li>
 * <li>Compressibility, stall: a stall angle falling with Mach number. The separation at angle a is taken from the
 *   incompressible table at a0 + (a - a0) k, k = (incompressible stall angle) / (angle at which the suction peak
 *   turns supersonic enough), with the peak reached when M times the incompressible peak velocity ratio passes
 *   SONIC_PEAK = 1.35: Beddoes' stall angles of the NACA 0012 at Mach 0.3 to 0.8 (Leishman, Principles of Helicopter
 *   Aerodynamics, table 9.1) all fall where this product is 1.1 to 1.4.</li>
 * <li>Wave drag: critical Mach number from the incompressible pressure peak by the Karman-Tsien rule and the sonic
 *   pressure coefficient; above it Lock's 20 (M - Mcr)^4. Above Mach 0.95 blended into Ackeret's linear supersonic
 *   theory (lift 4 a / sqrt(M^2 - 1), a biconvex section's thickness drag 16/3 (t/c)^2 / sqrt(M^2 - 1)), reached at
 *   1.2: transonic and supersonic sections are rough.</li>
 * <li>Beyond +-RANGE_DEG (blended from BLEND_DEG): a flat plate, normal force AirfoilAnalysis.flatPlateNormal, the
 *   centre of pressure from 0.4 of the chord at 30 degrees to mid-chord at 90 and 0.75 at 180 (a rule after
 *   flat-plate measurements); near 180 degrees the section flies backwards with a sharp leading edge: lift slope
 *   0.8 of the forward one, stalled by 10 degrees (a rule).</li>
 * <li>Flaps and ailerons (flapped): Glauert's thin-airfoil flap effectiveness tau(cf) times a deflection efficiency
 *   (rules after DATCOM's trends: plain and split flaps lose effectiveness above 10 degrees, slotted ones above 20),
 *   the zero-lift angle shifted by tau eta delta, the maximum lift raised by a share of the lift increment, thin-airfoil
 *   moment, a drag increment K cf^1.38 sin^2(delta), and a Fowler flap's chord extension.</li>
 * </ul>
 */
public final class SectionPolar implements AirfoilPolar {
    /** Reynolds numbers (on chord) analysed. */
    public static final double[] REYNOLDS = {5e4, 1e5, 2e5, 5e5, 1e6, 2e6, 5e6, 1e7, 3e7};
    /** The Reynolds number the angle-only sample uses. */
    public static final double DEFAULT_REYNOLDS = 3e6;
    /** Angles analysed (degrees either side of zero) and where the flat plate starts blending in. */
    public static final int RANGE_DEG = 32, BLEND_DEG = 26;
    /** Free-stream Mach number times the incompressible peak velocity ratio at which a section stalls from its shock. */
    public static final double SONIC_PEAK = 1.35;
    /** Lock's wave drag coefficient. */
    public static final double LOCK = 20;
    private static final double GAMMA = 1.4, STEP = Math.toRadians(1), MACH_STEP = 0.05;
    private static final int MACHS = 21, CACHE = 128;
    private static final Map<Airfoil, SectionPolar> POLARS = new LinkedHashMap<>(16, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<Airfoil, SectionPolar> e) { return size() > CACHE; }
    };

    private final Airfoil airfoil;
    private final double thickness;
    private final int na = 2 * RANGE_DEG + 1;
    // per angle (inviscid): lift, quarter-chord moment, lowest pressure coefficient, critical Mach number
    private final double[] clInv = new double[na], cmInv = new double[na], cpMin = new double[na], mcr = new double[na];
    // per Reynolds number and angle: separation f, friction drag
    private final double[][] sep = new double[REYNOLDS.length][na], friction = new double[REYNOLDS.length][na];
    // per Reynolds number: viscous lift factor, stall angles above and below the zero-lift angle (from it, rad), maximum and minimum lift
    private final double[] kv = new double[REYNOLDS.length], stallUp = new double[REYNOLDS.length], stallDown = new double[REYNOLDS.length];
    private final double[] clMax = new double[REYNOLDS.length], clMin = new double[REYNOLDS.length], cd0 = new double[REYNOLDS.length];
    private final double alpha0;
    // 1 / (angle from the zero-lift angle at which the peak becomes SONIC_PEAK), above and below, per Mach number (0: never)
    private final double[][] sonic = new double[2][MACHS];

    /** The polar of a shape, built on first use and cached (by value) for later ones. */
    public static SectionPolar of(Airfoil airfoil) {
        Objects.requireNonNull(airfoil, "airfoil");
        synchronized (POLARS) {
            SectionPolar p = POLARS.get(airfoil);
            if (p != null) { return p; }
        }
        SectionPolar built = new SectionPolar(airfoil);
        synchronized (POLARS) {
            SectionPolar p = POLARS.putIfAbsent(airfoil, built);
            return p == null ? built : p;
        }
    }

    private SectionPolar(Airfoil airfoil) {
        this.airfoil = airfoil;
        this.thickness = airfoil.thickness();
        AirfoilAnalysis analysis = new AirfoilAnalysis(airfoil);
        double[][] cl = new double[REYNOLDS.length][na], cd = new double[REYNOLDS.length][na];
        for (int i = 0; i < na; i++) {
            AirfoilAnalysis.Inviscid inv = analysis.inviscid(angle(i));
            clInv[i] = inv.clInviscid;
            cmInv[i] = inv.moment;
            cpMin[i] = inv.cpMin();
            mcr[i] = criticalMach(cpMin[i]);
            for (int r = 0; r < REYNOLDS.length; r++) {
                AirfoilAnalysis.Point p = inv.at(REYNOLDS[r]);
                sep[r][i] = p.separation();
                cl[r][i] = p.cl(); cd[r][i] = p.cd();
                double ca = Math.cos(p.alpha()), sa = Math.sin(p.alpha());
                // the analysis' drag less the pressure drag of its Kirchhoff forces is the friction
                friction[r][i] = Math.max(1e-4, p.cd() - pressureDrag(p.attachedCl(), p.separation(), p.alpha(), ca, sa));
            }
        }
        // zero-lift angle of the inviscid lift (the viscous factor only scales it)
        double z = 0;
        for (int i = 0; i < na - 1; i++) {
            if (clInv[i] <= 0 && clInv[i + 1] > 0) { z = angle(i) + STEP * (-clInv[i]) / (clInv[i + 1] - clInv[i]); break; }
        }
        alpha0 = z;
        for (int r = 0; r < REYNOLDS.length; r++) {
            kv[r] = AirfoilAnalysis.viscousFactor(thickness, REYNOLDS[r]);
            int up = 0, down = 0;
            double best = -1e9, worst = 1e9, least = 1e9;
            // the peaks before the flow separates (not the flat plate's at large angles)
            for (int i = 0; i < na; i++) {
                if (sep[r][i] < 0.3) { continue; }
                if (angle(i) >= alpha0 && cl[r][i] > best) { best = cl[r][i]; up = i; }
                if (angle(i) <= alpha0 && cl[r][i] < worst) { worst = cl[r][i]; down = i; }
                least = Math.min(least, cd[r][i]);
            }
            clMax[r] = best; clMin[r] = worst; cd0[r] = least;
            stallUp[r] = Math.max(STEP, angle(up) - alpha0);
            stallDown[r] = Math.max(STEP, alpha0 - angle(down));
        }
        for (int side = 0; side < 2; side++) {
            for (int m = 1; m < MACHS; m++) {
                double mach = m * MACH_STEP, need = SONIC_PEAK / mach, found = 0;
                // scan from the zero-lift angle toward this side for the peak velocity ratio sqrt(1 - Cp) reaching need
                double start = (alpha0 - angle(0)) / STEP;
                int i0 = (int) Math.floor(start);
                for (int k = 0; k < na; k++) {
                    int i = side == 0 ? i0 + 1 + k : i0 - k;
                    if (i < 0 || i >= na) { break; }
                    if (Math.sqrt(1 - cpMin[i]) >= need) {
                        int j = side == 0 ? i - 1 : i + 1;
                        double a = angle(i);
                        if (j >= 0 && j < na) {
                            double vi = Math.sqrt(1 - cpMin[i]), vj = Math.sqrt(1 - cpMin[j]);
                            a = vi == vj ? a : angle(j) + (angle(i) - angle(j)) * (need - vj) / (vi - vj);
                        }
                        found = 1 / Math.max(Math.abs(a - alpha0), 0.25 * STEP);
                        break;
                    }
                }
                sonic[side][m] = found;
            }
        }
    }

    private static double angle(int i) { return (i - RANGE_DEG) * STEP; }

    private static double pressureDrag(double attachedCl, double f, double alpha, double ca, double sa) {
        double factor = Math.pow((1 + Math.sqrt(f)) / 2, 2);
        double cn = attachedCl * ca * factor, cc = attachedCl * sa * Math.sqrt(f);
        double plate = (1 - f) * AirfoilAnalysis.flatPlateNormal(alpha);
        if (plate > Math.abs(cn)) { cn = Math.copySign(plate, cn == 0 ? alpha : cn); }
        return Math.max(0, cn * sa - cc * ca);
    }

    /**
     * Critical Mach number for an incompressible lowest pressure coefficient: the Karman-Tsien pressure equals the sonic
     * one, Cp0 / (b + M^2 / (1 + b) Cp0 / 2) = 2 / (g M^2) (((2 + (g - 1) M^2) / (g + 1))^(g / (g - 1)) - 1), b = sqrt(1 - M^2).
     */
    public static double criticalMach(double cpMin) {
        if (cpMin >= -1e-6) { return 1; }
        double lo = 0.01, hi = 0.999;
        for (int k = 0; k < 60; k++) {
            double m = 0.5 * (lo + hi), b = Math.sqrt(1 - m * m);
            double cp = cpMin / (b + m * m / (1 + b) * cpMin / 2);
            double star = 2 / (GAMMA * m * m) * (Math.pow((2 + (GAMMA - 1) * m * m) / (GAMMA + 1), GAMMA / (GAMMA - 1)) - 1);
            // Karman-Tsien's denominator vanishes before the sonic point for strong peaks: past that the peak is supersonic
            if (b + m * m / (1 + b) * cpMin / 2 <= 0 || cp <= star) { hi = m; } else { lo = m; }
        }
        return 0.5 * (lo + hi);
    }

    public Airfoil airfoil() { return airfoil; }
    /** Zero-lift angle (rad). */
    public double zeroLiftAngle() { return alpha0; }
    /** Lift slope per radian at a Reynolds number (incompressible, at the zero-lift angle). */
    public double liftSlope(double reynolds) {
        double k = AirfoilAnalysis.viscousFactor(thickness, Double.isFinite(reynolds) ? reynolds : DEFAULT_REYNOLDS);
        double a = alpha0 - 2 * STEP, b = alpha0 + 2 * STEP;
        return k * (inviscid(clInv, b) - inviscid(clInv, a)) / (b - a);
    }
    /** Maximum lift coefficient at a Reynolds number, incompressible. */
    public double maxLift(double reynolds) { return re(reynolds).of(clMax); }
    /** Minimum (most negative) lift coefficient at a Reynolds number, incompressible. */
    public double minLift(double reynolds) { return re(reynolds).of(clMin); }
    /** Angle of maximum lift (rad) at a Reynolds number, incompressible. */
    public double stallAngle(double reynolds) { return alpha0 + re(reynolds).of(stallUp); }
    /** Least drag coefficient at a Reynolds number, incompressible. */
    public double minimumDrag(double reynolds) { return re(reynolds).of(cd0); }
    /** Critical Mach number at an angle of attack (rad). */
    public double criticalMachAt(double alpha) {
        double a = wrap(alpha);
        return Math.abs(a) > RANGE_DEG * STEP ? 0.3 : inviscid(mcr, a);
    }

    @Override public AeroCoefficients sample(double alpha) { return sample(alpha, DEFAULT_REYNOLDS, 0); }

    @Override public AeroCoefficients sample(double alpha, FlowConditions flow) {
        return sample(alpha, flow.reynoldsNumber(), flow.machNumber());
    }

    /** Coefficients at an angle of attack (rad), Reynolds number (on chord) and Mach number. */
    public AeroCoefficients sample(double alpha, double reynolds, double mach) {
        double[] c = coefficients(alpha, reynolds, mach);
        return new AeroCoefficients(c[0], c[1], c[2], c[3] < 0.7);
    }

    /** {lift, drag, moment, separation f} at an angle (rad), Reynolds and Mach number. */
    double[] coefficients(double alpha, double reynolds, double mach) {
        double a = wrap(alpha);
        double m = Double.isFinite(mach) ? Math.max(0, mach) : 0;
        Interp re = re(Double.isFinite(reynolds) ? reynolds : DEFAULT_REYNOLDS);
        double cdFric = re.of(cd0);
        double[] out;
        double abs = Math.abs(a);
        if (abs >= RANGE_DEG * STEP) {
            out = flatPlate(a, re, cdFric);
        } else {
            out = analysed(a, re, Math.min(m, 0.95));
            if (abs > BLEND_DEG * STEP) {
                double w = (abs - BLEND_DEG * STEP) / ((RANGE_DEG - BLEND_DEG) * STEP);
                double[] plate = flatPlate(a, re, cdFric);
                for (int k = 0; k < 4; k++) { out[k] += w * (plate[k] - out[k]); }
            }
            // wave drag above the critical Mach number, at most a biconvex section's at Mach 1.2
            double over = Math.max(0, Math.min(m, 1) - inviscid(mcr, a));
            out[1] += Math.min(LOCK * over * over * over * over, 16.0 / 3 * thickness * thickness / Math.sqrt(0.44));
        }
        if (m > 0.95) {
            double w = Math.min(1, (m - 0.95) / 0.25);
            double[] sup = supersonic(a, Math.max(m, 1.2), cdFric);
            for (int k = 0; k < 3; k++) { out[k] += w * (sup[k] - out[k]); }
        }
        return out;
    }

    /**
     * The attached (unseparated) lift at an angle (rad), Reynolds and Mach number: the inviscid lift times the viscous
     * factor and Prandtl-Glauert, held at its value at +-RANGE_DEG beyond (for rotational stall delay corrections).
     */
    public double attachedLift(double alpha, double reynolds, double mach) {
        double a = Math.clamp(wrap(alpha), -RANGE_DEG * STEP, RANGE_DEG * STEP);
        double m = Double.isFinite(mach) ? Math.clamp(mach, 0, 0.9) : 0;
        double k = AirfoilAnalysis.viscousFactor(thickness, Double.isFinite(reynolds) ? reynolds : DEFAULT_REYNOLDS);
        return k * inviscid(clInv, a) / Math.sqrt(1 - m * m);
    }

    /** The analysed range: Kirchhoff on the attached forces with Prandtl-Glauert and the Mach number's earlier separation. */
    private double[] analysed(double a, Interp re, double mach) {
        double beta = Math.sqrt(1 - Math.min(mach, 0.9) * Math.min(mach, 0.9));
        double k = AirfoilAnalysis.viscousFactor(thickness, re.value);
        double clAtt = k * inviscid(clInv, a) / beta, cmAtt = k * inviscid(cmInv, a) / beta;
        // the separation: the incompressible one at the angle scaled by how much earlier the peak turns supersonic
        boolean up = a >= alpha0;
        double kappa = 1;
        if (mach > 0) {
            double s = mach / MACH_STEP;
            int i = Math.min((int) s, MACHS - 2);
            double inv = sonic[up ? 0 : 1][i] + (s - i) * (sonic[up ? 0 : 1][i + 1] - sonic[up ? 0 : 1][i]);
            kappa = Math.max(1, re.of(up ? stallUp : stallDown) * inv);
        }
        double at = alpha0 + (a - alpha0) * kappa;
        double f = Math.abs(at) >= RANGE_DEG * STEP ? 0 : re.of(sep, at);
        double fric = re.of(friction, a);
        double ca = Math.cos(a), sa = Math.sin(a);
        double factor = Math.pow((1 + Math.sqrt(f)) / 2, 2);
        double cn = clAtt * ca * factor, cc = clAtt * sa * Math.sqrt(f);
        double plate = (1 - f) * AirfoilAnalysis.flatPlateNormal(a);
        if (plate > Math.abs(cn)) { cn = Math.copySign(plate, cn == 0 ? a : cn); }
        double cl = cn * ca + cc * sa, cd = fric + Math.max(0, cn * sa - cc * ca);
        double cm = cmAtt * factor - cn * 0.25 * (1 - f) * (1 - f);
        return new double[]{cl, cd, cm, f};
    }

    /** Beyond the analysed range: a flat plate, and near 180 degrees a reversed section with a sharp leading edge. */
    private double[] flatPlate(double a, Interp re, double cdFric) {
        double abs = Math.abs(a), s = Math.sin(a), c = Math.cos(a);
        double cn = Math.copySign(AirfoilAnalysis.flatPlateNormal(a), s);
        double reversed = Math.PI - abs;   // angle from flying backwards
        double f = 0;
        if (reversed < Math.toRadians(30)) {
            f = Math.clamp(1 - (reversed - Math.toRadians(4)) / Math.toRadians(6), 0, 1);
            double slope = 0.8 * liftSlope(re.value);
            double attached = slope * s, factor = Math.pow((1 + Math.sqrt(f)) / 2, 2);
            double sepCn = attached * factor, plate = (1 - f) * Math.abs(cn);
            cn = Math.abs(sepCn) >= plate ? sepCn : Math.copySign(plate, s);
        }
        double ct = cdFric * c;   // friction along the chord, toward the downstream edge
        double xcp = abs <= Math.PI / 2 ? 0.4 + 0.1 * Math.clamp((abs - Math.toRadians(30)) / Math.toRadians(60), 0, 1)
                : 0.5 + 0.25 * (abs - Math.PI / 2) / (Math.PI / 2);
        double cl = cn * c - ct * s, cd = Math.max(cdFric * 0.5, cn * s + ct * c);
        return new double[]{cl, cd, -cn * (xcp - 0.25), f};
    }

    /** Ackeret's linear supersonic section (attached), capped at a flat plate's normal force. */
    private double[] supersonic(double a, double mach, double cdFric) {
        double b = Math.sqrt(mach * mach - 1), s = Math.sin(a), c = Math.cos(a);
        double cn = 4 * s * Math.abs(c) / b;
        double plate = AirfoilAnalysis.flatPlateNormal(a);
        if (Math.abs(cn) > plate) { cn = Math.copySign(plate, s); }
        double thick = 16.0 / 3 * thickness * thickness / b;
        return new double[]{cn * c, cdFric + thick + Math.abs(cn * s), -0.25 * cn};
    }

    // ----- flaps -----

    /** Kinds of trailing-edge flap: plain (also ailerons), split, single slotted, Fowler (slotted and extending aft). */
    public enum FlapType {
        PLAIN(10, 0.7, 0.9, 0), SPLIT(10, 0.7, 1.7, 0), SLOTTED(20, 0.85, 0.7, 0), FOWLER(20, 0.9, 0.5, 0.8);

        /** Deflection (deg) up to which the flap keeps its thin-airfoil effectiveness. */
        final double linearDeg;
        /** Share of the lift increment that raises the maximum lift. */
        final double maxLiftShare;
        /** Drag increment K in K cf^1.38 sin^2(delta). */
        final double drag;
        /** Chord extension at full travel (30 degrees) per flap chord. */
        final double extension;

        FlapType(double linearDeg, double maxLiftShare, double drag, double extension) {
            this.linearDeg = linearDeg; this.maxLiftShare = maxLiftShare; this.drag = drag; this.extension = extension;
        }
    }

    /** Glauert's flap effectiveness tau = 1 - (theta - sin theta) / pi, cos theta = 2 cf - 1 (cf: flap chord / chord). */
    public static double flapEffectiveness(double chordFraction) {
        double th = Math.acos(Math.clamp(2 * chordFraction - 1, -1, 1));
        return 1 - (th - Math.sin(th)) / Math.PI;
    }

    /** Deflection efficiency of a flap kind at a deflection (rad): 1 within its linear range, then (linear / delta)^0.6. */
    public static double flapEfficiency(FlapType type, double deflection) {
        double d = Math.toDegrees(Math.abs(deflection));
        return d <= type.linearDeg ? 1 : Math.pow(type.linearDeg / d, 0.6);
    }

    /**
     * Coefficients with a trailing-edge flap of a kind and chord fraction deflected by delta (rad, positive trailing
     * edge down), on the section's original chord.
     */
    public AeroCoefficients flapped(double alpha, double reynolds, double mach, FlapType type, double chordFraction, double delta) {
        double[] c = flappedCoefficients(alpha, reynolds, mach, type, chordFraction, delta);
        return new AeroCoefficients(c[0], c[1], c[2], c[3] < 0.7);
    }

    /** {lift, drag, moment, separation f} with a flap (as flapped); no flap when type is null or delta is 0. */
    double[] flappedCoefficients(double alpha, double reynolds, double mach, FlapType type, double chordFraction, double delta) {
        if (type == null || delta == 0 || !(chordFraction > 0) || !Double.isFinite(delta)) { return coefficients(alpha, reynolds, mach); }
        double cf = Math.min(chordFraction, 0.6);
        double tau = flapEffectiveness(cf), eta = flapEfficiency(type, delta);
        double shift = tau * eta * delta;                     // zero-lift angle change (rad)
        double slope = liftSlope(Double.isFinite(reynolds) ? reynolds : DEFAULT_REYNOLDS);
        double increment = slope * shift;                     // lift increment at fixed angle
        double keep = type.maxLiftShare;
        // the curve moves by (1 - keep) of the shift and keeps the rest as an increment: the maximum lift rises by keep x increment
        double[] base = coefficients(alpha + (1 - keep) * shift, reynolds, mach);
        double a = Math.abs(wrap(alpha));
        double fade = Math.clamp(1 - (a - Math.toRadians(25)) / Math.toRadians(35), 0, 1);
        double ext = 1 + type.extension * cf * Math.min(1, Math.abs(delta) / Math.toRadians(30));
        double th = Math.acos(Math.clamp(2 * cf - 1, -1, 1));
        double dcm = -0.5 * Math.sin(th) * (1 - Math.cos(th)) * eta * delta;
        double dcd = type.drag * Math.pow(cf, 1.38) * Math.pow(Math.sin(delta), 2);
        return new double[]{(base[0] + keep * increment * fade) * ext, (base[1] + dcd) * ext, (base[2] + dcm * fade) * ext * ext, base[3]};
    }

    // ----- interpolation -----

    private static double wrap(double a) {
        if (!Double.isFinite(a)) { return 0; }
        double w = Math.IEEEremainder(a, 2 * Math.PI);
        return w <= -Math.PI ? w + 2 * Math.PI : w;
    }

    private double inviscid(double[] table, double a) {
        double s = Math.clamp((a - angle(0)) / STEP, 0, na - 1.000001);
        int i = (int) s;
        return table[i] + (s - i) * (table[i + 1] - table[i]);
    }

    private Interp re(double reynolds) { return new Interp(reynolds); }

    /** Log-Reynolds interpolation weights (held at the ends). */
    private final class Interp {
        final double value, w;
        final int i;

        Interp(double reynolds) {
            value = Math.clamp(reynolds, REYNOLDS[0], REYNOLDS[REYNOLDS.length - 1]);
            int k = 0;
            while (k < REYNOLDS.length - 2 && value > REYNOLDS[k + 1]) { k++; }
            i = k;
            w = Math.clamp(Math.log(value / REYNOLDS[k]) / Math.log(REYNOLDS[k + 1] / REYNOLDS[k]), 0, 1);
        }

        double of(double[] perRe) { return perRe[i] + w * (perRe[i + 1] - perRe[i]); }
        double of(double[][] table, double a) {
            double lo = inviscid(table[i], a), hi = inviscid(table[i + 1], a);
            return lo + w * (hi - lo);
        }
    }
}
