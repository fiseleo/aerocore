package dev.mcmod.aerocore.api.body;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A body's structure as a thin-walled beam, module by module: each station's section is a closed shell along its
 * outline (skin plus the stringers smeared into it), its bending stiffness about the horizontal axis through the shell's
 * centroid I = sum t L (y - yc)^2 (plus each edge's own L^3 t sin^2 / 12). Doors and windows remove their share of the
 * side skin (half the girth) over their height and add a frame of reinforcement round each opening.
 * <p>Loads: the body's mass along its length (structure, floor and what it carries) at a load factor, held up either at
 * support points (a wing's root, axles; the reactions balance the weight and its moment, a lone support with one at the
 * far end, as an aircraft's tail balances it about its wing) or by buoyancy (in proportion to the section area): the
 * shear and bending moment along the length and the bending stress M c / I at each station (c the farthest fibre).
 * A pressurised module adds the hoop stress p r / t (r the radius of a circle of the section's area).
 * <p>Torsion (0.33, torsion()): each section one closed thin-walled cell by Bredt, J = 4 A^2 / (closed integral of ds / t),
 * t the skin's own (the stringers carry no shear flow), where doors, windows, a glazed module's windscreen and an open
 * module's top make parts of the wall framed bands (Vierendeel girders of the frames round the openings, semi-rigid
 * joints) of an equivalent shear thickness instead of skin (sectionTorsion() and framedThickness() have the method);
 * between stations the length's stiffness in series,
 * k = 1 / (integral of dx / G J), G = E / (2 (1 + nu)) with nu = 0.3 for every material. The sections keep their shape
 * (Bredt's assumption: frames and bulkheads hold it): the result is a body in white twisted between rigid bulkheads at
 * the stations.
 * <p>Not modelled: torsion's stresses (the stiffness only); the sections' distortion between frames; warping restraint;
 * the glass's and closures' own stiffness (a body in white: glazing and doors carry nothing); a hood's or trunk lid's
 * opening (a module without doors, windows or an open top is closed all round); a space frame's tubes (its skin alone);
 * shear buckling of the skin, frames' own bending in the bending check, local loads at fittings.
 */
public final class BodyStructure {
    /** How a module is built. */
    public enum Build { SEMI_MONOCOQUE, MONOCOQUE, SPACE_FRAME }

    /**
     * One module's structure: build, material (density kg/m^3, yield and Young's modulus Pa), skin thickness, stringers
     * as a share of the skin's area (smeared in), frame spacing and each frame's cross-section area (m^2), floor mass
     * per area (kg/m^2, over the section's width), doors (count, width, height) and window share of the side,
     * cabin pressure over ambient (Pa).
     */
    public record Module(Build build, double densityKgM3, double yieldPa, double modulusPa, double skinM, double stringerShare,
                         double frameSpacingM, double frameAreaM2, double floorKgM2, int doors, double doorWidthM, double doorHeightM,
                         double windowShare, double pressurePa) {
        public Module {
            Objects.requireNonNull(build, "build");
            Require.positive(densityKgM3, "Density"); Require.positive(yieldPa, "Yield strength"); Require.positive(modulusPa, "Young's modulus");
            Require.positive(skinM, "Skin thickness"); Require.nonnegative(stringerShare, "Stringer share");
            Require.positive(frameSpacingM, "Frame spacing"); Require.nonnegative(frameAreaM2, "Frame area"); Require.nonnegative(floorKgM2, "Floor mass");
            if (doors < 0) { throw new IllegalArgumentException("Doors must not be negative"); }
            Require.nonnegative(doorWidthM, "Door width"); Require.nonnegative(doorHeightM, "Door height");
            if (!(windowShare >= 0 && windowShare < 1)) { throw new IllegalArgumentException("Window share is 0..1"); }
            Require.nonnegative(pressurePa, "Cabin pressure");
        }
        /** The skin with its stringers smeared in. */
        public double equivalentSkinM() { return skinM * (1 + stringerShare); }
    }

    /** A mass the body carries at a length position (x from the rear end), e.g. payload, an engine on a mount. */
    public record PointMass(double xM, double massKg) {}

    /** Mass of each module's skin, frames, floor and opening frames (kg). */
    public record ModuleMass(double skinKg, double framesKg, double floorKg, double openingsKg) {
        public double totalKg() { return skinKg + framesKg + floorKg + openingsKg; }
    }

    /**
     * Along the body: the station x, shear (N) and bending moment (N m, positive sagging: the middle down), the bending
     * stress at the farthest fibre (Pa), the hoop stress (Pa); the greatest bending and hoop stress and where, and each
     * module's mass.
     */
    public record Result(double[] xM, double[] shearN, double[] momentNm, double[] bendingPa, double[] hoopPa, double maxBendingPa,
                         double maxBendingAtM, int maxBendingModule, double maxHoopPa, int maxHoopModule, List<ModuleMass> masses) {
        public double massKg() { double m = 0; for (ModuleMass mm : masses) { m += mm.totalKg(); } return m; }
    }

    private BodyStructure() {}

    /** Each module's mass from its shell (the stations' girth over the module's length), frames, floor and openings. */
    public static List<ModuleMass> masses(BodyShape shape, List<Module> modules) {
        check(shape, modules);
        List<ModuleMass> out = new ArrayList<>();
        for (int m = 0; m < modules.size(); m++) {
            Module s = modules.get(m);
            double front = shape.moduleFrontX(m), rear = shape.moduleRearX(m), girth = 0, width = 0, count = 0;
            for (BodyShape.Station st : shape.stations()) {
                if (st.xM() >= rear - 1e-9 && st.xM() <= front + 1e-9) { girth += st.perimeterM(); width += st.widthM(); count++; }
            }
            girth = count > 0 ? girth / count : 0; width = count > 0 ? width / count : 0;
            double length = front - rear;
            double skin = s.build() == Build.SPACE_FRAME ? s.skinM() * girth * length * s.densityKgM3()
                    : s.equivalentSkinM() * girth * length * s.densityKgM3();
            double frames = Math.floor(length / s.frameSpacingM() + 1) * girth * s.frameAreaM2() * s.densityKgM3();
            double floor = s.floorKgM2() * width * length;
            // a frame round each opening, of the frame section, and the window frames as a share of the side
            double openings = s.doors() * 2 * (s.doorWidthM() + s.doorHeightM()) * s.frameAreaM2() * s.densityKgM3()
                    + s.windowShare() * 0.5 * girth * length * s.equivalentSkinM() * s.densityKgM3() * 0.5;
            out.add(new ModuleMass(skin, frames, floor, openings));
        }
        return out;
    }

    /**
     * Shear, moment and stresses at a load factor, the body's own structure (masses()) and the point masses held up at
     * the support points (x, share of the load; null or empty: buoyancy in proportion to the section area).
     */
    public static Result analyze(BodyShape shape, List<Module> modules, List<PointMass> carried, double loadFactor, double gravity,
                                 List<double[]> supports) {
        check(shape, modules);
        List<ModuleMass> masses = masses(shape, modules);
        List<BodyShape.Station> st = shape.stations();
        int n = st.size();
        double[] x = new double[n];
        for (int i = 0; i < n; i++) { x[i] = st.get(i).xM(); }
        // weight per station: each module's mass spread over its stations' strips; point masses at the nearest station
        double[] down = new double[n];
        for (int m = 0; m < modules.size(); m++) {
            double front = shape.moduleFrontX(m), rear = shape.moduleRearX(m), weight = masses.get(m).totalKg() * gravity * loadFactor;
            double span = front - rear;
            for (int i = 0; i < n; i++) {
                double lo = Math.max(i > 0 ? 0.5 * (x[i - 1] + x[i]) : x[i], rear), hi = Math.min(i < n - 1 ? 0.5 * (x[i] + x[i + 1]) : x[i], front);
                if (hi > lo && span > 0) { down[i] += weight * (hi - lo) / span; }
            }
        }
        for (PointMass pm : carried) { down[nearest(x, pm.xM())] += pm.massKg() * gravity * loadFactor; }
        double total = 0;
        for (double d : down) { total += d; }
        // what holds it up
        double[] up = new double[n];
        // reactions holding both the weight and its moment: R_i = w_i (a + b x_i), the least sum of R^2 / w (a lone
        // support is balanced at the far end, as a tail balances an aircraft about its wing). Buoyancy: every station
        // a support weighted by its strip's volume (the hull trims until its buoyancy stands under its weight).
        List<double[]> held = new ArrayList<>();
        if (supports == null || supports.isEmpty()) {
            for (int i = 0; i < n; i++) {
                double lo = i > 0 ? 0.5 * (x[i - 1] + x[i]) : x[i], hi = i < n - 1 ? 0.5 * (x[i] + x[i + 1]) : x[i];
                double volume = st.get(i).areaM2() * (hi - lo);
                if (volume > 0) { held.add(new double[]{x[i], volume}); }
            }
        } else {
            held.addAll(supports);
        }
        {
            double moment0 = 0;
            for (int i = 0; i < n; i++) { moment0 += down[i] * x[i]; }
            double spread = 0;
            for (double[] a : held) for (double[] b : held) { spread = Math.max(spread, Math.abs(a[0] - b[0])); }
            if (spread < 1e-6 * Math.max(x[n - 1] - x[0], 1e-9)) {
                double at = held.get(0)[0], far = Math.abs(at - x[0]) > Math.abs(at - x[n - 1]) ? x[0] : x[n - 1];
                held.add(new double[]{far, held.get(0)[1]});
            }
            double sw = 0, swx = 0, swxx = 0;
            for (double[] a : held) { sw += a[1]; swx += a[1] * a[0]; swxx += a[1] * a[0] * a[0]; }
            double det = sw * swxx - swx * swx;
            double ca = (total * swxx - moment0 * swx) / det, cb = (sw * moment0 - swx * total) / det;
            for (double[] a : held) { up[nearest(x, a[0])] += a[1] * (ca + cb * a[0]); }
        }
        // integrate from the rear end: shear V and moment M
        double[] shear = new double[n], moment = new double[n], bending = new double[n], hoop = new double[n];
        double v = 0, mo = 0;
        for (int i = 0; i < n; i++) {
            if (i > 0) { mo += v * (x[i] - x[i - 1]); }
            v += up[i] - down[i];
            shear[i] = v; moment[i] = mo;
        }
        double maxB = 0, maxAt = 0, maxH = 0;
        int maxModule = 0, hoopModule = 0;
        for (int i = 0; i < n; i++) {
            BodyShape.Station s = st.get(i);
            Module mod = modules.get(s.module());
            double[] section = bendingSection(s, mod);
            bending[i] = section[0] > 0 ? Math.abs(moment[i]) * section[1] / section[0] : 0;
            double r = Math.sqrt(s.areaM2() / Math.PI);
            hoop[i] = mod.pressurePa() > 0 ? mod.pressurePa() * r / mod.skinM() : 0;
            if (bending[i] / mod.yieldPa() > maxB / modules.get(maxModule).yieldPa()) { maxB = bending[i]; maxAt = x[i]; maxModule = s.module(); }
            if (hoop[i] > maxH) { maxH = hoop[i]; hoopModule = s.module(); }
        }
        return new Result(x, shear, moment, bending, hoop, maxB, maxAt, maxModule, maxH, hoopModule, masses);
    }

    /**
     * The section's second moment of area about its horizontal centroidal axis (m^4) and the distance to its farthest
     * fibre (m): the outline as a shell of the module's skin, the side skin removed over the doors' and windows' share.
     */
    static double[] bendingSection(BodyShape.Station s, Module m) {
        List<double[]> o = s.outline();
        int n = o.size();
        double t = m.build() == Build.SPACE_FRAME ? m.skinM() : m.equivalentSkinM();
        // the share of the sides cut away: windows, and doors over the module's length (both sides counted together)
        double cut = Math.min(0.8, m.windowShare() + m.doors() * m.doorWidthM() / Math.max(2 * s.perimeterM(), 1e-9));
        double area = 0, first = 0;
        double[] len = new double[n], mid = new double[n], weight = new double[n];
        for (int i = 0; i < n; i++) {
            double[] p = o.get(i), q = o.get((i + 1) % n);
            len[i] = Math.hypot(q[0] - p[0], q[1] - p[1]);
            mid[i] = 0.5 * (p[0] + q[0]);
            // sides (steeper than 45 degrees) lose the cut share of their skin
            weight[i] = Math.abs(q[0] - p[0]) > Math.abs(q[1] - p[1]) ? 1 - cut : 1;
            area += t * len[i] * weight[i]; first += t * len[i] * weight[i] * mid[i];
        }
        if (!(area > 0)) { return new double[]{0, 0}; }
        double yc = first / area, inertia = 0, far = 0;
        for (int i = 0; i < n; i++) {
            double[] p = o.get(i), q = o.get((i + 1) % n);
            double dy = q[0] - p[0];
            inertia += weight[i] * t * (len[i] * Math.pow(mid[i] - yc, 2) + len[i] * dy * dy / 12);
            far = Math.max(far, Math.abs(p[0] - yc));
        }
        return new double[]{inertia, far};
    }

    // ---------------------------------------------------------------- torsion (0.33)

    /** Poisson's ratio taken for every material, for the shear modulus G = E / (2 (1 + nu)) (metals are 0.27..0.34). */
    public static final double POISSON = 0.3;

    /**
     * A glazed module's top is a windscreen where it rises or falls along the length more steeply than this, dy/dx (17
     * degrees: a roof's camber is gentler, a raked windscreen or rear window steeper).
     */
    public static final double WINDSCREEN_SLOPE = 0.3;

    /**
     * A body's torsion: the sampled sections (x from the rear end, ascending; a joint between modules twice, once with each
     * module's structure, so G J may step there) and their G J (N m^2); the stations (ascending x), the torsional spring
     * between each two neighbours (N m/rad, one fewer than the stations) and the stations' stiffness matrix (N m/rad):
     * T_s = sum over t of K[s][t] theta_t, symmetric, positive semi-definite, each row summing to 0 (the body free, its
     * ends beyond the outer stations unloaded).
     */
    public record Torsion(double[] xM, double[] gjNm2, double[] stationXM, double[] springNmPerRad, double[][] stiffnessNmPerRad) {
        /** The stiffness between two stations (indices into stationXM) alone: the springs between them in series, N m/rad. */
        public double betweenNmPerRad(int a, int b) {
            int lo = Math.min(a, b), hi = Math.max(a, b);
            if (lo == hi) { return Double.POSITIVE_INFINITY; }
            double c = 0;
            for (int i = lo; i < hi; i++) { c += 1 / springNmPerRad[i]; }
            return 1 / c;
        }
    }

    /** The shear modulus of a material of Young's modulus E (Pa), Poisson's ratio POISSON. */
    public static double shearModulusPa(double modulusPa) { return modulusPa / (2 * (1 + POISSON)); }

    /**
     * The body's torsional stiffness between stations at the given x (from the rear end, within the body, at least 1 um
     * apart, in any order; the result's ascending), as springs in series along the length: between neighbours the
     * compliance is the integral of dx / G J(x), 1 / G J linear between the sampled sections (the shape's stations, each by
     * sectionTorsion() with its module's structure). The ends beyond the outer stations are free.
     */
    public static Torsion torsion(BodyShape shape, List<Module> modules, double[] stationX) {
        check(shape, modules);
        Objects.requireNonNull(stationX, "stationX");
        if (stationX.length == 0) { throw new IllegalArgumentException("At least one station"); }
        double length = shape.lengthM();
        double[] at = stationX.clone();
        for (int i = 0; i < at.length; i++) {
            if (!(at[i] >= -1e-9 && at[i] <= length + 1e-9)) { throw new IllegalArgumentException("Stations must be within the body, 0.." + length + " m"); }
            at[i] = Math.clamp(at[i], 0, length);
        }
        java.util.Arrays.sort(at);
        for (int i = 1; i < at.length; i++) {
            if (!(at[i] - at[i - 1] >= 1e-6)) { throw new IllegalArgumentException("Stations must be apart"); }
        }
        List<BodyShape.Station> st = shape.stations();
        int n = st.size();
        // the top's slope along the length at each station (a windscreen where a glazed module's is steep)
        double[] slope = new double[n];
        for (int i = 0; i < n; i++) {
            int a = Math.max(i - 1, 0), b = Math.min(i + 1, n - 1);
            double dx = st.get(b).xM() - st.get(a).xM();
            slope[i] = dx > 0 ? (st.get(b).topM() - st.get(a).topM()) / dx : 0;
        }
        // each module's sections, its joints at both ends included, rear module first
        List<double[]> samples = new ArrayList<>();
        for (int m = modules.size() - 1; m >= 0; m--) {
            Module mod = modules.get(m);
            double front = shape.moduleFrontX(m), rear = shape.moduleRearX(m), sill = shape.modules().get(m).sillM();
            List<Integer> in = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                double x = st.get(i).xM();
                if (x >= rear - 1e-9 && x <= front + 1e-9) { in.add(i); }
            }
            boolean[] screened = new boolean[in.size()];
            double screen = 0;
            if (mod.windowShare() > 0) {
                for (int k = 0; k < in.size(); k++) {
                    int i = in.get(k);
                    if (!(Math.abs(slope[i]) > WINDSCREEN_SLOPE)) { continue; }
                    screened[k] = true;
                    double lo = k > 0 ? 0.5 * (st.get(in.get(k - 1)).xM() + st.get(i).xM()) : rear;
                    double hi = k < in.size() - 1 ? 0.5 * (st.get(i).xM() + st.get(in.get(k + 1)).xM()) : front;
                    screen += Math.max(Math.min(hi, front) - Math.max(lo, rear), 0);
                }
            }
            double g = shearModulusPa(mod.modulusPa());
            for (int k = 0; k < in.size(); k++) {
                BodyShape.Station s = st.get(in.get(k));
                samples.add(new double[]{s.xM(), g * sectionTorsion(s.outline(), mod, front - rear, screened[k] ? screen : 0, sill)});
            }
        }
        int ns = samples.size();
        double[] xs = new double[ns], gj = new double[ns], flex = new double[ns];
        for (int k = 0; k < ns; k++) {
            xs[k] = samples.get(k)[0]; gj[k] = samples.get(k)[1];
            flex[k] = gj[k] > 0 ? 1 / gj[k] : Double.POSITIVE_INFINITY;
        }
        int count = at.length;
        double[] spring = new double[count - 1];
        double[][] stiffness = new double[count][count];
        for (int i = 0; i + 1 < count; i++) {
            double k = 1 / integrate(xs, flex, at[i], at[i + 1]);
            spring[i] = k;
            stiffness[i][i] += k; stiffness[i + 1][i + 1] += k;
            stiffness[i][i + 1] -= k; stiffness[i + 1][i] -= k;
        }
        return new Torsion(xs, gj, at, spring, stiffness);
    }

    /** The integral of f over [x0, x1], f linear between the samples (x ascending, a repeated x a step). */
    private static double integrate(double[] x, double[] f, double x0, double x1) {
        double sum = 0;
        for (int k = 0; k + 1 < x.length; k++) {
            double a = x[k], b = x[k + 1], lo = Math.max(a, x0), hi = Math.min(b, x1);
            if (!(b > a) || !(hi > lo)) { continue; }
            if (!Double.isFinite(f[k]) || !Double.isFinite(f[k + 1])) { return Double.POSITIVE_INFINITY; }
            double fl = f[k] + (f[k + 1] - f[k]) * (lo - a) / (b - a), fh = f[k] + (f[k + 1] - f[k]) * (hi - a) / (b - a);
            sum += 0.5 * (fl + fh) * (hi - lo);
        }
        return sum;
    }

    /**
     * A section's torsion constant J (m^4) as one closed thin-walled cell (Bredt): J = 4 A^2 / (the sum round the outline
     * of each edge's length over its thickness). The thickness is the skin's own: the shear flow runs in the skin, the
     * stringers carry direct stress only (as a stiffened shell is idealised, Megson, Aircraft Structures for Engineering
     * Students), so they count where the frames round the openings bend (framedThickness()) but not here. Where
     * openings make the wall a framed band, the band's equivalent shear thickness stands in for the skin, as the simple
     * structural surfaces method takes a car's sides (Brown, Robertson and Serpento, Motor Vehicle Structures, 2002):
     * <ul>
     * <li>doors (starboard (doors + 1) / 2 of them, port doors / 2, a body's doors alternating from starboard): on each
     * side (its edges steeper than 45 degrees) a band of the door's height centred on the side, the sill below and the roof
     * rail above sharing the rest of the side's height, the doors in bays of the module's length over their count;</li>
     * <li>windows: a band over the upper half of each side, in bays as long as the band is high, each open over
     * 2 windowShare of its length (so the windows are windowShare of the side);</li>
     * <li>a windscreen (windscreenM long, torsion(): where a glazed module's top slopes along the length more steeply than
     * WINDSCREEN_SLOPE): the top (the edges flatter than 45 degrees above the section's mid-height) one opening of that
     * length, the frames along its sides the chords, those at its ends (a cowl, a header) the posts;</li>
     * <li>an open module (sillM below the section's top): the section cut at the sill and closed there by its rim, one
     * opening the module's length the same way.</li>
     * </ul>
     * Where bands overlap the thinner holds. A section without openings is exactly Bredt's closed cell; one whose framed
     * bands carry nothing (no frames round a door the module's length) is no less than its walls as an open section,
     * the sum of L t^3 / 3.
     */
    static double sectionTorsion(List<double[]> outline, Module m, double lengthM, double windscreenM, double sillM) {
        double t = m.skinM(), direct = m.build() == Build.SPACE_FRAME ? m.skinM() : m.equivalentSkinM();
        double lowest = Double.POSITIVE_INFINITY, highest = Double.NEGATIVE_INFINITY;
        for (double[] p : outline) { lowest = Math.min(lowest, p[0]); highest = Math.max(highest, p[0]); }
        boolean cut = sillM < highest - 1e-9;
        List<double[]> o = outline;
        if (cut) {
            if (!(sillM > lowest + 1e-9)) { return 0; }
            o = BodyShape.below(outline, sillM);
        }
        int n = o.size();
        if (n < 3) { return 0; }
        double area = Math.abs(BodyShape.area(o)), bottom = Double.POSITIVE_INFINITY, top = Double.NEGATIVE_INFINITY;
        for (double[] p : o) { bottom = Math.min(bottom, p[0]); top = Math.max(top, p[0]); }
        double mid = 0.5 * (bottom + top);
        // each edge: a side's (steeper than 45 degrees; [0] port, [1] starboard), the rim of an open top, or the top's
        double[] len = new double[n], sideLow = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY}, sideHigh = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        int[] kind = new int[n], which = new int[n];   // kind: 0 skin, 1 side, 2 rim, 3 top
        double rim = 0, roof = 0, open = 0;
        for (int i = 0; i < n; i++) {
            double[] p = o.get(i), q = o.get((i + 1) % n);
            len[i] = Math.hypot(q[0] - p[0], q[1] - p[1]);
            which[i] = p[1] + q[1] >= 0 ? 1 : 0;
            if (Math.abs(q[0] - p[0]) > Math.abs(q[1] - p[1])) {
                kind[i] = 1;
                sideLow[which[i]] = Math.min(sideLow[which[i]], Math.min(p[0], q[0]));
                sideHigh[which[i]] = Math.max(sideHigh[which[i]], Math.max(p[0], q[0]));
            } else if (cut && p[0] >= sillM - 1e-6 && q[0] >= sillM - 1e-6) {
                kind[i] = 2; rim += len[i];
            } else if (0.5 * (p[0] + q[0]) > mid) {
                kind[i] = 3; roof += len[i];
            }
            if (kind[i] != 2) { open += len[i] * t * t * t / 3; }
        }
        // the sides' bands of openings: {from, to, equivalent thickness}
        List<List<double[]>> bands = List.of(new ArrayList<>(), new ArrayList<>());
        for (int s = 0; s < 2; s++) {
            double h0 = sideLow[s], h1 = sideHigh[s], height = h1 - h0;
            if (!(height > 0)) { continue; }
            int doors = s == 1 ? (m.doors() + 1) / 2 : m.doors() / 2;
            if (doors > 0 && m.doorWidthM() > 0 && m.doorHeightM() > 0) {
                double h = Math.min(m.doorHeightM(), height), y0 = 0.5 * (h0 + h1 - h), y1 = y0 + h;
                bands.get(s).add(new double[]{y0, y1, framedThickness(t, direct, m, lengthM, doors, m.doorWidthM(), h, y0 - h0, h1 - y1)});
            }
            if (m.windowShare() > 0) {
                double h = 0.5 * height, y0 = h1 - h;
                int bays = Math.max(1, (int) Math.round(lengthM / h));
                double c = Math.min(1, 2 * m.windowShare()) * lengthM / bays;
                bands.get(s).add(new double[]{y0, h1, framedThickness(t, direct, m, lengthM, bays, c, h, y0 - h0, 0)});
            }
        }
        double rimT = cut ? framedThickness(t, direct, m, lengthM, 1, lengthM, rim, 0, 0) : t;
        double roofT = !cut && windscreenM > 0 ? framedThickness(t, direct, m, windscreenM, 1, windscreenM, roof, 0, 0) : t;
        double round = 0;
        for (int i = 0; i < n; i++) {
            List<double[]> b = kind[i] == 1 ? bands.get(which[i]) : List.of();
            if (kind[i] == 2) { round += len[i] / rimT; continue; }
            if (kind[i] == 3) { round += len[i] / roofT; continue; }
            if (b.isEmpty()) { round += len[i] / t; continue; }
            // a side's edge split where the bands start and end, each piece the thinnest band over it
            double[] p = o.get(i), q = o.get((i + 1) % n);
            double ya = Math.min(p[0], q[0]), yb = Math.max(p[0], q[0]);
            List<Double> cuts = new ArrayList<>(List.of(ya, yb));
            for (double[] band : b) {
                for (int e = 0; e < 2; e++) { if (band[e] > ya && band[e] < yb) { cuts.add(band[e]); } }
            }
            cuts.sort(null);
            for (int c = 0; c + 1 < cuts.size(); c++) {
                double y0 = cuts.get(c), y1 = cuts.get(c + 1), ym = 0.5 * (y0 + y1), thick = t;
                for (double[] band : b) { if (ym >= band[0] && ym <= band[1]) { thick = Math.min(thick, band[2]); } }
                round += len[i] * (y1 - y0) / (yb - ya) / thick;
            }
        }
        return Math.max(4 * area * area / round, open);
    }

    /**
     * The equivalent shear thickness (m) of a band of openings in a wall, framed as a Vierendeel girder standing in for
     * the skin: count openings each c wide (along the length) in a band h across, in bays a = length / count long. The
     * chords along the band's edges (a sill, a roof rail) are each a frame member and the wall beyond the edge (eLo, eHi
     * across); a post (pillar) is the wall left between two openings (w = a - c wide) with a frame member down each
     * opening's edge beside it. A frame member is a thin-walled square tube of the skin's sheet holding the frame area A_f
     * (side d = A_f / (4 t_skin), I = A_f d^2 / 6, shear area A_f / 2): the frames round the openings that masses() counts.
     * The wall bends with its stringers (direct, the equivalent skin; a space frame's skin alone) and shears in its skin
     * (shear, t_skin). The shear flow q round the section is carried by each bay's members bending in double curvature,
     * the chords over the opening's width and the posts over its height, so by virtual work over a bay
     * <pre>1 / (G t_eq) = h c^3 / (12 E (I_lo + I_hi) a) + 2 a h^2 / (12 E I_post) + a / (G A_post)</pre>
     * (V = q h through the chords, shared in proportion to their stiffness; q a through each post). The 2 is the joints:
     * spot-welded sheet joints are semi-rigid, their flexibility of the order of the members' (as automotive body joint
     * studies found, e.g. Chang, Effects of flexible connections on body structural response, SAE 740041, 1974), taken as
     * each post end's rotational stiffness equal to the post's own, 6 E I / h; with rigid joints the sedan of
     * BodyTorsionTest twists about 50 kN m per degree between its axles, above every real saloon's. No thicker than the
     * skin it replaces; 0 when nothing frames the openings.
     */
    static double framedThickness(double shear, double direct, Module m, double lengthM, int count, double c, double h, double eLo, double eHi) {
        if (!(count > 0 && c > 0 && h > 0 && lengthM > 0)) { return shear; }
        double a = lengthM / count, open = Math.min(c, a), w = a - open;
        double af = m.frameAreaM2(), d = af / (4 * m.skinM()), own = af * d * d / 6;
        double chords = chord(direct, eLo, af, own) + chord(direct, eHi, af, own);
        double post = direct * w * w * w / 12 + 2 * (own + af * 0.25 * (w + d) * (w + d)), web = shear * w + af;
        double e = m.modulusPa(), g = shearModulusPa(e);
        double compliance = h * open * open * open / (12 * e * chords * a) + 2 * a * h * h / (12 * e * post) + a / (g * web);
        return Math.min(shear, 1 / (g * compliance));
    }

    /** A chord's second moment in its wall's plane (m^4): the wall e across beyond the band's edge (t thick) and a frame member on the edge. */
    private static double chord(double t, double e, double af, double own) {
        double strip = t * e, total = strip + af;
        if (!(total > 0)) { return 0; }
        double yc = -0.5 * e * strip / total;   // the strip's centre at -e/2 from the edge, the member's on it
        return t * e * e * e / 12 + strip * (0.5 * e + yc) * (0.5 * e + yc) + own + af * yc * yc;
    }

    /**
     * ISO 12215-5 (2019, as tabulated by Tivari 2022): a planing craft's bottom design pressure (Pa) from its loaded mass
     * (kg), waterline length and chine beam (m), deadrise at 0.4 L (deg), speed (knots) and design category factor k_DC
     * (A 1.0, B 0.8, C 0.6, D 0.4): the dynamic load factor k_DYN = min(0.32 (L/(10 B) + 0.084)(50 - beta) V^2 B^2 / m,
     * 0.5 V / m^0.17), the base pressure 0.1 m / (L B) (1 + sqrt(k_DC) k_DYN) kN/m^2, no less than the minimum
     * max(0.45 m^0.33 + 0.9 L k_DC, 7) kN/m^2. The panel's aspect and the longitudinal distribution factors (k_AR, k_L) are
     * left at 1: the base pressure, the most demanding part of the bottom's.
     */
    public static double bottomPressurePa(double massKg, double waterlineM, double chineBeamM, double deadriseDeg, double speedKnots, double designCategory) {
        double kdyn = dynamicLoadFactor(massKg, waterlineM, chineBeamM, deadriseDeg, speedKnots);
        double base = 0.1 * massKg / (waterlineM * chineBeamM) * (1 + Math.sqrt(designCategory) * kdyn);
        double minimum = Math.max(0.45 * Math.pow(massKg, 0.33) + 0.9 * waterlineM * designCategory, 7);
        return 1000 * Math.max(base, minimum);
    }

    /** ISO 12215-5's dynamic load factor k_DYN (the lesser of its two forms; speed in knots, mass in kg). */
    public static double dynamicLoadFactor(double massKg, double waterlineM, double chineBeamM, double deadriseDeg, double speedKnots) {
        double v = speedKnots, b = chineBeamM;
        double k1 = 0.32 * (waterlineM / (10 * b) + 0.084) * (50 - deadriseDeg) * v * v * b * b / massKg;
        double k2 = 0.5 * v / Math.pow(massKg, 0.17);
        return Math.max(Math.min(k1, k2), 0);
    }

    /**
     * Plating under a uniform pressure: the thickness (m) a long panel of the given short side (frame spacing, m) needs to
     * hold it at a design stress (Pa), t = s sqrt(k2 p / sigma_d) with ISO 12215-5's k2 = 0.5 for a panel over twice as
     * long as it is wide.
     */
    public static double plateThicknessM(double pressurePa, double shortSideM, double designStressPa) {
        return shortSideM * Math.sqrt(0.5 * pressurePa / designStressPa);
    }

    private static int nearest(double[] x, double at) {
        int best = 0;
        for (int i = 1; i < x.length; i++) { if (Math.abs(x[i] - at) < Math.abs(x[best] - at)) { best = i; } }
        return best;
    }

    private static void check(BodyShape shape, List<Module> modules) {
        if (modules.size() != shape.modules().size()) { throw new IllegalArgumentException("One structure per module"); }
    }
}
