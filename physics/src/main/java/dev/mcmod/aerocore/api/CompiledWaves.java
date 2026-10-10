package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Fast evaluation of a WaveField: each component's wave vector, frequency, phase, amplitude and depth terms are
 * computed once and held in arrays, so a query costs one sin/cos (and one or two exp below the still level) per
 * component instead of re-deriving the dispersion relation. grid() fills a regular grid by phasor recurrence (one
 * complex multiply per component per point) for drawing the surface every frame. Every linear result equals the
 * corresponding WaveField / LinearWave method to rounding, including the depth clamping of the linear kinematics.
 * <p>
 * Second-order surface (secondOrder, surfaceTerms, gridSecondOrder): the Choppy Wave Model (Nouguier, Guerin and
 * Chapron 2009, JGR 114 C09012) moves each surface particle horizontally by D = -sum a k^ coth(kh) sin(theta), the
 * linear particle excursion; expanded to second order at a fixed point the surface is eta - D . grad(eta), plus the
 * mean level M = sum a^2 k coth(kh) / 2 that keeps the still level (the model's Eulerian mean is -M, Pierson 1961).
 * For one deep-water wave this is exactly Stokes' second order, a cos(theta) + (k a^2 / 2) cos(2 theta): sharper crests,
 * flatter troughs. On top of it SecondOrderCorrection adds the difference to the full second-order theory of Sharma and
 * Dean (1981), which Forristall's (2000) crest distributions are built on: the set-down under wave groups, oblique
 * pairs and finite depth (the bound harmonic then equals Stokes'). Velocities, accelerations and pressure heads stay
 * linear. gridSecondOrder holds only the cheap part; gridCorrection adds the rest on a grid.
 * <p>
 * Coasts (CoastalWaves): each component becomes Re(M a e^{i theta}) with its envelope phasor M at the point. The
 * linear slope includes the envelope's gradient (so refracted, shortened waves have their own slopes); the second-order
 * part is evaluated from the enveloped phasors with the open sea's kernels, because Stokes-type theory is not valid in
 * the shallow water where the local kernels would differ (Ursell number above about 26) and cnoidal shoaling crests
 * are not modelled. A body samples a local view (at()): plane waves with the envelope's amplitude and phase at its
 * position, travelling along the local phase gradient with the wavenumber and depth there, which agrees with the
 * coastal surface at that point and approximates it nearby (WKB).
 */
public final class CompiledWaves {
    public static final CompiledWaves NONE = new CompiledWaves(WaveField.NONE);
    private static final double MIN_DEPTH = 0.05;
    private final double[] kx, kz, k, omega, phase, amplitude, depth, cosineDenominator, sineDenominator, imageFactor;
    /** Wave vectors, wavenumbers and coth(kh) of the second-order part: the field's own, or the open sea's in a local view. */
    private final double[] sx, sz, sk, scoth;
    private final WaveField field;
    private final double maxAmplitude, meanLevel;
    private final SecondOrderCorrection correction;
    /** Amplitudes over the ones the correction was decomposed with (SecondOrderCorrection.fold). */
    private final double[] fold;
    private static final ThreadLocal<double[][]> PHASES = ThreadLocal.withInitial(() -> new double[8][0]);

    public CompiledWaves(WaveField field) {
        this.field = field;
        int n = field.components().size();
        kx = new double[n]; kz = new double[n]; k = new double[n]; omega = new double[n]; phase = new double[n];
        amplitude = new double[n]; depth = new double[n]; cosineDenominator = new double[n]; sineDenominator = new double[n];
        imageFactor = new double[n];
        double[] coth = new double[n];
        for (int i = 0; i < n; i++) {
            LinearWave w = field.components().get(i);
            k[i] = w.wavenumber();
            kx[i] = k[i] * Math.cos(w.directionRadians()); kz[i] = k[i] * Math.sin(w.directionRadians());
            omega[i] = w.angularFrequency(); phase[i] = w.phaseRadians(); amplitude[i] = w.amplitudeM(); depth[i] = w.depthM();
            coth[i] = depthTerms(i);
        }
        sx = kx; sz = kz; sk = k; scoth = coth;
        double largest = 0;
        for (int i = 0; i < n; i++) { largest = Math.max(largest, amplitude[i]); }
        double[] relative = new double[n];
        for (int i = 0; i < n; i++) { relative[i] = largest > 0 ? amplitude[i] / largest : 0; }
        correction = largest > 0 ? SecondOrderCorrection.of(kx, kz, relative, depth, coth) : SecondOrderCorrection.NONE;
        fold = correction.fold(amplitude);
        meanLevel = mean();
        maxAmplitude = bound();
    }

    /** A view of open with other linear components (amplitude, wave vector, depth, phase), sharing its second-order kernels. */
    private CompiledWaves(CompiledWaves open, double[] amplitude, double[] kx, double[] kz, double[] depth, double[] phase, WaveField field) {
        this.field = field; this.amplitude = amplitude; this.kx = kx; this.kz = kz; this.depth = depth; this.phase = phase;
        omega = open.omega; sx = open.sx; sz = open.sz; sk = open.sk; scoth = open.scoth; correction = open.correction;
        int n = amplitude.length;
        k = new double[n]; cosineDenominator = new double[n]; sineDenominator = new double[n]; imageFactor = new double[n];
        double[] f = new double[n];
        for (int i = 0; i < n; i++) {
            k[i] = Math.hypot(kx[i], kz[i]);
            depthTerms(i);
            f[i] = open.amplitude[i] > 0 ? open.fold[i] * amplitude[i] / open.amplitude[i] : 0;
        }
        fold = f;
        meanLevel = mean();
        maxAmplitude = bound();
    }

    /** Fills the depth denominators of component i and returns its coth(kh). */
    private double depthTerms(int i) {
        boolean finite = Double.isFinite(depth[i]);
        cosineDenominator[i] = finite ? 1 + Math.exp(-2 * k[i] * depth[i]) : 1;
        sineDenominator[i] = finite ? -Math.expm1(-2 * k[i] * depth[i]) : 1;
        // exp(-k (z + 2h)) = exp(-2kh) / exp(kz): the bottom image term costs a division, not a second exp.
        imageFactor[i] = finite ? Math.exp(-2 * k[i] * depth[i]) : 0;
        return finite ? cosineDenominator[i] / sineDenominator[i] : 1;
    }

    private double mean() {
        double sum = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += 0.5 * amplitude[i] * amplitude[i] * sk[i] * scoth[i]; }
        return sum;
    }

    /** |eta2| <= M + |D| |grad eta| <= M + (sum a coth)(sum a k), plus the correction's sum |weight| (sum |mode fold|)^2. */
    private double bound() {
        double sum = 0, excursion = 0, slope = 0, extra = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += amplitude[i]; excursion += amplitude[i] * scoth[i]; slope += amplitude[i] * sk[i]; }
        for (int r = 0; r < correction.cosModes.length; r++) { extra += Math.abs(correction.cosWeights[r]) * Math.pow(l1(correction.cosModes[r], fold), 2); }
        for (int r = 0; r < correction.sinModes.length; r++) { extra += Math.abs(correction.sinWeights[r]) * Math.pow(l1(correction.sinModes[r], fold), 2); }
        return sum + meanLevel + excursion * slope + extra;
    }

    private static double l1(double[] v, double[] fold) { double s = 0; for (int i = 0; i < v.length; i++) { s += Math.abs(v[i] * fold[i]); } return s; }

    /** This sea with every amplitude times factor (0 to 1; a tapered copy), sharing the second-order decomposition. */
    public CompiledWaves scaled(double factor) {
        if (!(factor >= 0 && factor <= 1)) { throw new IllegalArgumentException("Amplitude factor must be in [0, 1]"); }
        if (factor == 1) { return this; }
        double[] a = new double[amplitude.length];
        for (int i = 0; i < a.length; i++) { a[i] = amplitude[i] * factor; }
        return new CompiledWaves(this, a, kx, kz, depth, phase, field.scaled(factor));
    }

    /**
     * This sea at (x, z) on a coast, its amplitudes times taper (0 to 1): each component is the plane wave with the
     * envelope's gain and phase at the point, travelling along the local phase gradient with the wavenumber of the
     * local smoothed depth (so its frequency is unchanged) over that depth. The second-order part keeps the open sea's
     * kernels as the coastal surface does. field() lists the same waves as LinearWaves for frequency-domain loads,
     * their amplitudes held within LinearWave's steepness and height limits. Without a coast (or outside its grid)
     * this is scaled(taper).
     */
    public CompiledWaves at(CoastalWaves coast, double x, double z, double taper) {
        int n = amplitude.length;
        double[][] s = scratch(n);
        double[] re = s[2], im = s[3], reX = s[4], imX = s[5], reZ = s[6], imZ = s[7];
        if (coast == null || n == 0 || coast.components() != n || !coast.envelopes(x, z, re, im, reX, imX, reZ, imZ)) { return scaled(taper); }
        if (!(taper >= 0 && taper <= 1)) { throw new IllegalArgumentException("Amplitude factor must be in [0, 1]"); }
        double local = coast.depth(x, z);
        double[] a = new double[n], qx = new double[n], qz = new double[n], h = new double[n], ph = new double[n];
        List<LinearWave> waves = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            double g2 = re[i] * re[i] + im[i] * im[i];
            a[i] = taper * amplitude[i] * Math.sqrt(g2);
            // Local phase gradient k + grad(delta), grad(delta) = Im(conj(M) grad M) / |M|^2.
            double gx = kx[i], gz = kz[i];
            if (g2 > 1e-12) { gx += (re[i] * imX[i] - im[i] * reX[i]) / g2; gz += (re[i] * imZ[i] - im[i] * reZ[i]) / g2; }
            double length = Math.hypot(gx, gz);
            if (!(length > 1e-9)) { gx = kx[i]; gz = kz[i]; length = k[i]; }
            h[i] = Double.isFinite(local) && local > 0 && local < 1e3 ? Math.max(local, MIN_DEPTH) : depth[i];
            double kl = CoastalWaves.wavenumber(omega[i], h[i]);
            qx[i] = kl * gx / length; qz[i] = kl * gz / length;
            ph[i] = phase[i] + Math.atan2(im[i], re[i]) + (kx[i] - qx[i]) * x + (kz[i] - qz[i]) * z;
            double wavelength = 2 * Math.PI / kl, limit = 0.5 * LinearWave.MAX_STEEPNESS * wavelength;
            if (Double.isFinite(h[i])) { limit = Math.min(limit, 0.5 * LinearWave.MAX_HEIGHT_TO_DEPTH * h[i]); }
            waves.add(new LinearWave(Math.min(a[i], 0.999 * limit), wavelength, Math.atan2(qz[i], qx[i]), h[i], ph[i]));
        }
        return new CompiledWaves(this, a, qx, qz, h, ph, new WaveField(waves));
    }

    private static double[][] scratch(int n) {
        double[][] s = PHASES.get();
        if (s[0].length < n) { for (int i = 0; i < s.length; i++) { s[i] = new double[n]; } }
        return s;
    }

    /** Number of rank-one terms in the second-order correction (for diagnostics). */
    public int correctionRank() { return correction.rank(); }

    /** Mean-level correction M = sum a^2 k coth(kh) / 2 of the second-order surface, m. */
    public double meanLevel() { return meanLevel; }

    /** Amplitude-weighted mean wavenumber of the linear components, rad/m (0 for a calm sea). */
    public double meanWavenumber() {
        double sum = 0, weighted = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += amplitude[i]; weighted += amplitude[i] * k[i]; }
        return sum > 0 ? weighted / sum : 0;
    }

    /**
     * Second-order surface correction eta2 = M - D . grad(eta) and its slopes from phases at offset, into
     * out[0..2] = {eta2, d eta2/dx, d eta2/dz}.
     */
    public void secondOrder(double[] cos, double[] sin, int offset, double[] out) {
        double dx = 0, dz = 0, gx = 0, gz = 0, axx = 0, axz = 0, azz = 0, hxx = 0, hxz = 0, hzz = 0;
        for (int i = 0; i < amplitude.length; i++) {
            double c = cos[offset + i], s = sin[offset + i], a = amplitude[i], ex = sx[i] / sk[i], ez = sz[i] / sk[i], ac = a * scoth[i];
            dx -= ac * ex * s; dz -= ac * ez * s;
            gx -= a * sx[i] * s; gz -= a * sz[i] * s;
            axx += ac * ex * sx[i] * c; axz += ac * ex * sz[i] * c; azz += ac * ez * sz[i] * c;
            hxx += a * sx[i] * sx[i] * c; hxz += a * sx[i] * sz[i] * c; hzz += a * sz[i] * sz[i] * c;
        }
        out[0] = meanLevel - (dx * gx + dz * gz);
        // d/dx of -(D . G): dDx/dx = -axx, dDz/dx = -axz, dGx/dx = -hxx, dGz/dx = -hxz (and likewise in z).
        out[1] = axx * gx + dx * hxx + axz * gz + dz * hxz;
        out[2] = axz * gx + dx * hxz + azz * gz + dz * hzz;
        correction.add(cos, sin, offset, fold, sx, sz, out);
    }

    /** The second-order height alone from phases at offset (secondOrder without slopes), m. */
    public double secondOrderHeight(double[] cos, double[] sin, int offset) {
        return cheapSecondOrderHeight(sin, offset) + correctionHeight(cos, sin, offset);
    }

    /** The Choppy-Wave-Model part of secondOrderHeight, m. */
    public double cheapSecondOrderHeight(double[] sin, int offset) {
        double dx = 0, dz = 0, gx = 0, gz = 0;
        for (int i = 0; i < amplitude.length; i++) {
            double s = sin[offset + i], a = amplitude[i], ac = a * scoth[i];
            dx -= ac * sx[i] / sk[i] * s; dz -= ac * sz[i] / sk[i] * s;
            gx -= a * sx[i] * s; gz -= a * sz[i] * s;
        }
        return meanLevel - (dx * gx + dz * gz);
    }

    /** The SecondOrderCorrection part of secondOrderHeight, m: it varies over half a dominant wavelength. */
    public double correctionHeight(double[] cos, double[] sin, int offset) { return correction.height(cos, sin, offset, fold); }

    /**
     * Linear and second-order surface terms at a point: out[0..5] = {eta1, d eta1/dx, d eta1/dz, eta2, d eta2/dx,
     * d eta2/dz}. A taper w on the amplitudes scales the first three by w and the last three by w^2.
     */
    public void surfaceTerms(double x, double z, double t, double[] out) { surfaceTerms(x, z, t, null, out); }

    /** surfaceTerms on a coast (null for open sea): every component enveloped by its phasor at (x, z). */
    public void surfaceTerms(double x, double z, double t, CoastalWaves coast, double[] out) {
        int n = amplitude.length;
        double[][] scratch = scratch(n);
        double[] cs = scratch[0], sn = scratch[1], re = scratch[2], im = scratch[3], reX = scratch[4], imX = scratch[5], reZ = scratch[6], imZ = scratch[7];
        boolean modulated = coast != null && coast.components() == n && coast.envelopes(x, z, re, im, reX, imX, reZ, imZ);
        double eta = 0, lx = 0, lz = 0, mean = 0, dx = 0, dz = 0, gx = 0, gz = 0, axx = 0, axz = 0, azz = 0, hxx = 0, hxz = 0, hzz = 0;
        for (int i = 0; i < n; i++) {
            double theta = kx[i] * x + kz[i] * z - omega[i] * t + phase[i], c0 = Math.cos(theta), s0 = Math.sin(theta);
            double c, s, dcx, dcz;
            if (modulated) {
                c = re[i] * c0 - im[i] * s0; s = re[i] * s0 + im[i] * c0;
                // d/dx Re(M e^{i theta}) = Re(M_x e^{i theta}) - kx Im(M e^{i theta}).
                dcx = reX[i] * c0 - imX[i] * s0 - kx[i] * s; dcz = reZ[i] * c0 - imZ[i] * s0 - kz[i] * s;
            } else { c = c0; s = s0; dcx = -kx[i] * s; dcz = -kz[i] * s; }
            cs[i] = c; sn[i] = s;
            double a = amplitude[i], ex = sx[i] / sk[i], ez = sz[i] / sk[i], ac = a * scoth[i];
            eta += a * c; lx += a * dcx; lz += a * dcz;
            mean += 0.5 * a * a * sk[i] * scoth[i] * (c * c + s * s);
            dx -= ac * ex * s; dz -= ac * ez * s;
            gx -= a * sx[i] * s; gz -= a * sz[i] * s;
            axx += ac * ex * sx[i] * c; axz += ac * ex * sz[i] * c; azz += ac * ez * sz[i] * c;
            hxx += a * sx[i] * sx[i] * c; hxz += a * sx[i] * sz[i] * c; hzz += a * sz[i] * sz[i] * c;
        }
        out[0] = eta; out[1] = lx; out[2] = lz;
        double[] second = {mean - (dx * gx + dz * gz), axx * gx + dx * hxx + axz * gz + dz * hxz, axz * gx + dx * hxz + azz * gz + dz * hzz};
        correction.add(cs, sn, 0, fold, sx, sz, second);
        out[3] = second[0]; out[4] = second[1]; out[5] = second[2];
    }

    /** Surface elevation to second order (linear plus secondOrder) at a point, m above the still level. */
    public double surface(double x, double z, double t) {
        double[] out = new double[6];
        surfaceTerms(x, z, t, out);
        return out[0] + out[3];
    }

    /**
     * Multiplies the phases at offset by the coast's envelope phasors at (x, z), in place: afterwards the column
     * queries (elevation, verticalAcceleration, variance) give the coastal surface there. False when nothing changed.
     */
    public boolean modulate(CoastalWaves coast, double x, double z, double[] cos, double[] sin, int offset) {
        int n = amplitude.length;
        double[][] s = scratch(n);
        double[] re = s[2], im = s[3];
        if (coast == null || coast.components() != n || !coast.envelopes(x, z, re, im, null, null, null, null)) { return false; }
        for (int i = 0; i < n; i++) {
            double c = cos[offset + i], sn = sin[offset + i];
            cos[offset + i] = re[i] * c - im[i] * sn; sin[offset + i] = re[i] * sn + im[i] * c;
        }
        return true;
    }

    /** Variance of the linear surface from phases at offset, sum a^2 (cos^2 + sin^2) / 2 (enveloped phases carry g^2), m^2. */
    public double variance(double[] cos, double[] sin, int offset) {
        double sum = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += 0.5 * amplitude[i] * amplitude[i] * (cos[offset + i] * cos[offset + i] + sin[offset + i] * sin[offset + i]); }
        return sum;
    }

    /**
     * Linear vertical acceleration of the surface, m/s^2: -sum a omega^2 cos(theta), the acceleration of a surface particle
     * at its label to first order (used for the breaking criterion).
     */
    public double verticalAcceleration(double[] cos, int offset) {
        double sum = 0;
        for (int i = 0; i < amplitude.length; i++) { sum -= amplitude[i] * omega[i] * omega[i] * cos[offset + i]; }
        return sum;
    }

    /** Standard deviation of the surface's vertical acceleration, sqrt(m4) = sqrt(sum a^2 omega^4 / 2), m/s^2. */
    public double accelerationDeviation() {
        double sum = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += 0.5 * Math.pow(amplitude[i] * omega[i] * omega[i], 2); }
        return Math.sqrt(sum);
    }

    public WaveField field() { return field; }
    public boolean isEmpty() { return amplitude.length == 0; }
    public int size() { return amplitude.length; }
    /** Upper bound of the second-order surface elevation above or below still water, m (the linear bound is WaveField.maxAmplitude). */
    public double maxAmplitude() { return maxAmplitude; }

    public double elevation(double x, double z, double t) {
        double sum = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += amplitude[i] * Math.cos(kx[i] * x + kz[i] * z - omega[i] * t + phase[i]); }
        return sum;
    }
    /** {eta, d eta/dx, d eta/dz}. */
    public double[] elevationAndSlope(double x, double z, double t) {
        double eta = 0, sx = 0, sz = 0;
        for (int i = 0; i < amplitude.length; i++) {
            double theta = kx[i] * x + kz[i] * z - omega[i] * t + phase[i], c = Math.cos(theta), s = Math.sin(theta);
            eta += amplitude[i] * c; sx -= amplitude[i] * kx[i] * s; sz -= amplitude[i] * kz[i] * s;
        }
        return new double[]{eta, sx, sz};
    }

    /** cos and sin of every component's phase at (x, z, t), for the column queries below (offset 0). */
    public void phases(double x, double z, double t, double[] cos, double[] sin) {
        for (int i = 0; i < amplitude.length; i++) {
            double theta = kx[i] * x + kz[i] * z - omega[i] * t + phase[i];
            cos[i] = Math.cos(theta); sin[i] = Math.sin(theta);
        }
    }
    /**
     * Phases on the grid x0 + i dx, z0 + j dz (i < nx, j < nz) by phasor recurrence, into cos/sin[(i nz + j) size() + c];
     * use offset (i nz + j) size() with the column queries.
     */
    public void gridPhases(double x0, double z0, double dx, double dz, int nx, int nz, double t, double[] cos, double[] sin) {
        int n = amplitude.length;
        for (int c = 0; c < n; c++) {
            double stepXr = Math.cos(kx[c] * dx), stepXi = Math.sin(kx[c] * dx), stepZr = Math.cos(kz[c] * dz), stepZi = Math.sin(kz[c] * dz);
            double theta = kx[c] * x0 + kz[c] * z0 - omega[c] * t + phase[c], rowRe = Math.cos(theta), rowIm = Math.sin(theta);
            for (int i = 0; i < nx; i++) {
                double re = rowRe, im = rowIm;
                for (int j = 0; j < nz; j++) {
                    int p = (i * nz + j) * n + c;
                    cos[p] = re; sin[p] = im;
                    double next = re * stepZr - im * stepZi;
                    im = re * stepZi + im * stepZr; re = next;
                }
                double next = rowRe * stepXr - rowIm * stepXi;
                rowIm = rowRe * stepXi + rowIm * stepXr; rowRe = next;
            }
        }
    }
    /** Elevation from phases (offset into the phase arrays). */
    public double elevation(double[] cos, int offset) {
        double sum = 0;
        for (int i = 0; i < amplitude.length; i++) { sum += amplitude[i] * cos[offset + i]; }
        return sum;
    }
    public double elevation(double[] cos) { return elevation(cos, 0); }
    /** Surface slopes {d eta/dx, d eta/dz} from phases (offset 0). */
    public double[] slope(double[] sin) {
        double sx = 0, sz = 0;
        for (int i = 0; i < amplitude.length; i++) { sx -= amplitude[i] * kx[i] * sin[i]; sz -= amplitude[i] * kz[i] * sin[i]; }
        return new double[]{sx, sz};
    }
    /** Dynamic pressure head (m) at a height above still water, from phases(); see LinearWave.dynamicPressureHead. */
    public double pressureHead(double[] cos, double heightAboveStill) {
        double sum = 0, z = Math.min(heightAboveStill, 0);
        for (int i = 0; i < amplitude.length; i++) {
            double zc = Double.isFinite(depth[i]) ? Math.max(z, -depth[i]) : z, decay = Math.exp(k[i] * zc);
            sum += amplitude[i] * (decay + imageFactor[i] / decay) / cosineDenominator[i] * cos[i];
        }
        return sum;
    }
    /**
     * Orbital velocity (m/s) into out[0..2] and local acceleration (m/s^2) into out[3..5] at a height above still
     * water, from phases(); see LinearWave.velocity / acceleration.
     */
    public void kinematics(double[] cos, double[] sin, double heightAboveStill, double[] out) {
        double ux = 0, uy = 0, uz = 0, ax = 0, ay = 0, az = 0, z = Math.min(heightAboveStill, 0);
        for (int i = 0; i < amplitude.length; i++) {
            double zc = Double.isFinite(depth[i]) ? Math.max(z, -depth[i]) : z;
            double decay = Math.exp(k[i] * zc), image = imageFactor[i] / decay;
            double horizontal = (decay + image) / sineDenominator[i], vertical = (decay - image) / sineDenominator[i];
            double speed = amplitude[i] * omega[i], accel = speed * omega[i], dirX = kx[i] / k[i], dirZ = kz[i] / k[i];
            double uh = speed * horizontal * cos[i], ah = accel * horizontal * sin[i];
            ux += uh * dirX; uz += uh * dirZ; uy += speed * vertical * sin[i];
            ax += ah * dirX; az += ah * dirZ; ay -= accel * vertical * cos[i];
        }
        out[0] = ux; out[1] = uy; out[2] = uz; out[3] = ax; out[4] = ay; out[5] = az;
    }
    /**
     * One wet segment of a column, from phases at offset: out[0], out[1] = dynamic pressure heads at the segment's
     * low and high ends, out[2..7] = velocity and acceleration at its middle (heights above still water). The middle
     * depth factor is the geometric mean of the end factors when no clamp intervenes, which is exact for exp.
     */
    public void segment(double[] cos, double[] sin, int offset, double low, double high, double[] out) {
        double hl = 0, hh = 0, ux = 0, uy = 0, uz = 0, ax = 0, ay = 0, az = 0;
        double zl = Math.min(low, 0), zh = Math.min(high, 0), zm = Math.min((low + high) / 2, 0);
        boolean geometric = high <= 0;
        for (int i = 0; i < amplitude.length; i++) {
            double c = cos[offset + i], s = sin[offset + i];
            boolean finite = Double.isFinite(depth[i]);
            double cl = finite ? Math.max(zl, -depth[i]) : zl, ch = finite ? Math.max(zh, -depth[i]) : zh;
            double dl = Math.exp(k[i] * cl), dh = ch == 0 ? 1 : Math.exp(k[i] * ch);
            double dm = geometric && (!finite || low >= -depth[i]) ? Math.sqrt(dl * dh)
                    : Math.exp(k[i] * (finite ? Math.max(zm, -depth[i]) : zm));
            double f = imageFactor[i], a = amplitude[i];
            hl += a * (dl + f / dl) / cosineDenominator[i] * c;
            hh += a * (dh + f / dh) / cosineDenominator[i] * c;
            double im = f / dm, horizontal = (dm + im) / sineDenominator[i], vertical = (dm - im) / sineDenominator[i];
            double speed = a * omega[i], accel = speed * omega[i], dirX = kx[i] / k[i], dirZ = kz[i] / k[i];
            double uh = speed * horizontal * c, ah = accel * horizontal * s;
            ux += uh * dirX; uz += uh * dirZ; uy += speed * vertical * s;
            ax += ah * dirX; az += ah * dirZ; ay -= accel * vertical * c;
        }
        out[0] = hl; out[1] = hh; out[2] = ux; out[3] = uy; out[4] = uz; out[5] = ax; out[6] = ay; out[7] = az;
    }
    /** Orbital velocity at a point, m/s; equals WaveField.velocity. */
    public Vec3d velocity(Vec3d point, double stillWaterY, double t) {
        double[] cos = new double[amplitude.length], sin = new double[amplitude.length], out = new double[6];
        phases(point.x(), point.z(), t, cos, sin);
        kinematics(cos, sin, point.y() - stillWaterY, out);
        return new Vec3d(out[0], out[1], out[2]);
    }

    private static final ThreadLocal<double[][]> SECOND_ORDER_SCRATCH = ThreadLocal.withInitial(() -> new double[14][0]);
    private static final ThreadLocal<int[][]> STENCIL = ThreadLocal.withInitial(() -> new int[1][0]);

    /**
     * The cheap (Choppy Wave Model) part of secondOrder on the same grid as grid(): eta2 and its slopes into height2,
     * slope2X, slope2Z (overwritten), by the same phasor recurrence. gridCorrection adds the rest.
     */
    public void gridSecondOrder(double x0, double z0, double dx, double dz, int nx, int nz, double t, double[] height2, double[] slope2X, double[] slope2Z) {
        int points = nx * nz;
        double[][] s = SECOND_ORDER_SCRATCH.get();
        if (s[0].length < points) { for (int i = 0; i < s.length; i++) { s[i] = new double[points]; } }
        for (int r = 0; r < 10; r++) { Arrays.fill(s[r], 0, points, 0); }
        double[] ddx = s[0], ddz = s[1], gx = s[2], gz = s[3], axx = s[4], axz = s[5], azz = s[6], hxx = s[7], hxz = s[8], hzz = s[9];
        for (int c = 0; c < amplitude.length; c++) {
            double stepRe = Math.cos(kz[c] * dz), stepIm = Math.sin(kz[c] * dz), a = amplitude[c], ex = sx[c] / sk[c], ez = sz[c] / sk[c], ac = a * scoth[c];
            double cdx = ac * ex, cdz = ac * ez, cgx = a * sx[c], cgz = a * sz[c];
            double cxx = ac * ex * sx[c], cxz = ac * ex * sz[c], czz = ac * ez * sz[c], qxx = a * sx[c] * sx[c], qxz = a * sx[c] * sz[c], qzz = a * sz[c] * sz[c];
            for (int i = 0; i < nx; i++) {
                double theta = kx[c] * (x0 + i * dx) + kz[c] * z0 - omega[c] * t + phase[c], re = Math.cos(theta), im = Math.sin(theta);
                int row = i * nz;
                for (int j = 0; j < nz; j++) {
                    int p = row + j;
                    ddx[p] -= cdx * im; ddz[p] -= cdz * im; gx[p] -= cgx * im; gz[p] -= cgz * im;
                    axx[p] += cxx * re; axz[p] += cxz * re; azz[p] += czz * re;
                    hxx[p] += qxx * re; hxz[p] += qxz * re; hzz[p] += qzz * re;
                    double next = re * stepRe - im * stepIm;
                    im = re * stepIm + im * stepRe;
                    re = next;
                }
            }
        }
        for (int p = 0; p < points; p++) {
            height2[p] = meanLevel - (ddx[p] * gx[p] + ddz[p] * gz[p]);
            slope2X[p] = axx[p] * gx[p] + ddx[p] * hxx[p] + axz[p] * gz[p] + ddz[p] * hxz[p];
            slope2Z[p] = axz[p] * gx[p] + ddx[p] * hxz[p] + azz[p] * gz[p] + ddz[p] * hzz[p];
        }
    }

    /** Bilinear stencils of the grid points on the coast grid: base node index (-1 outside) and fractions. */
    private static int[] stencils(CoastalWaves coast, double x0, double z0, double dx, double dz, int nx, int nz, double[] fx, double[] fz) {
        int[][] holder = STENCIL.get();
        if (holder[0].length < nx * nz) { holder[0] = new int[nx * nz]; }
        int[] base = holder[0];
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int p = i * nz + j;
            double gx = (x0 + i * dx - coast.x0()) / coast.spacing(), gz = (z0 + j * dz - coast.z0()) / coast.spacing();
            if (coast.uniform() || !(gx >= 0) || gx > coast.nx() - 1 || !(gz >= 0) || gz > coast.nz() - 1) { base[p] = -1; continue; }
            int a = Math.min((int) gx, coast.nx() - 2), b = Math.min((int) gz, coast.nz() - 2);
            base[p] = a * coast.nz() + b; fx[p] = gx - a; fz[p] = gz - b;
        }
        return base;
    }

    /**
     * grid and gridSecondOrder over a coast: each component's phasor times its envelope (bilinear on the coast grid),
     * the linear slopes from d/dx Re(M e^{i theta}), the second-order part from the enveloped phasors with the open
     * sea's kernels. Equals surfaceTerms(x, z, t, coast) at the grid points, less the correction, which
     * gridCorrection(..., coast) adds.
     */
    public void gridCoastal(CoastalWaves coast, double x0, double z0, double dx, double dz, int nx, int nz, double t,
                            double[] height, double[] slopeX, double[] slopeZ, double[] height2, double[] slope2X, double[] slope2Z) {
        int points = nx * nz;
        double[][] s = SECOND_ORDER_SCRATCH.get();
        if (s[0].length < points) { for (int i = 0; i < s.length; i++) { s[i] = new double[points]; } }
        for (int r = 0; r < 11; r++) { Arrays.fill(s[r], 0, points, 0); }
        Arrays.fill(height, 0, points, 0); Arrays.fill(slopeX, 0, points, 0); Arrays.fill(slopeZ, 0, points, 0);
        double[] ddx = s[0], ddz = s[1], gx = s[2], gz = s[3], axx = s[4], axz = s[5], azz = s[6], hxx = s[7], hxz = s[8], hzz = s[9], mean = s[10];
        double[] fxs = s[11], fzs = s[12];
        int[] base = stencils(coast, x0, z0, dx, dz, nx, nz, fxs, fzs);
        int cnz = coast.nz();
        double inv = 1 / coast.spacing();
        for (int c = 0; c < amplitude.length; c++) {
            float[] cr = coast.re[c], ci = coast.im[c];
            double stepRe = Math.cos(kz[c] * dz), stepIm = Math.sin(kz[c] * dz), a = amplitude[c], ex = sx[c] / sk[c], ez = sz[c] / sk[c], ac = a * scoth[c];
            double half = 0.5 * a * a * sk[c] * scoth[c];
            for (int i = 0; i < nx; i++) {
                double theta = kx[c] * (x0 + i * dx) + kz[c] * z0 - omega[c] * t + phase[c], re = Math.cos(theta), im = Math.sin(theta);
                int row = i * nz;
                for (int j = 0; j < nz; j++) {
                    int p = row + j, q = base[p];
                    double mr = 1, mi = 0, mrx = 0, mix = 0, mrz = 0, miz = 0;
                    if (q >= 0) {
                        double fx = fxs[p], fz = fzs[p];
                        double r00 = cr[q], r01 = cr[q + 1], r10 = cr[q + cnz], r11 = cr[q + cnz + 1];
                        double i00 = ci[q], i01 = ci[q + 1], i10 = ci[q + cnz], i11 = ci[q + cnz + 1];
                        mr = (r00 * (1 - fz) + r01 * fz) * (1 - fx) + (r10 * (1 - fz) + r11 * fz) * fx;
                        mi = (i00 * (1 - fz) + i01 * fz) * (1 - fx) + (i10 * (1 - fz) + i11 * fz) * fx;
                        mrx = ((r10 - r00) * (1 - fz) + (r11 - r01) * fz) * inv; mix = ((i10 - i00) * (1 - fz) + (i11 - i01) * fz) * inv;
                        mrz = ((r01 - r00) * (1 - fx) + (r11 - r10) * fx) * inv; miz = ((i01 - i00) * (1 - fx) + (i11 - i10) * fx) * inv;
                    }
                    double cc = mr * re - mi * im, ss = mr * im + mi * re;
                    height[p] += a * cc;
                    slopeX[p] += a * (mrx * re - mix * im - kx[c] * ss);
                    slopeZ[p] += a * (mrz * re - miz * im - kz[c] * ss);
                    mean[p] += half * (cc * cc + ss * ss);
                    ddx[p] -= ac * ex * ss; ddz[p] -= ac * ez * ss; gx[p] -= a * sx[c] * ss; gz[p] -= a * sz[c] * ss;
                    axx[p] += ac * ex * sx[c] * cc; axz[p] += ac * ex * sz[c] * cc; azz[p] += ac * ez * sz[c] * cc;
                    hxx[p] += a * sx[c] * sx[c] * cc; hxz[p] += a * sx[c] * sz[c] * cc; hzz[p] += a * sz[c] * sz[c] * cc;
                    double next = re * stepRe - im * stepIm;
                    im = re * stepIm + im * stepRe;
                    re = next;
                }
            }
        }
        for (int p = 0; p < points; p++) {
            height2[p] = mean[p] - (ddx[p] * gx[p] + ddz[p] * gz[p]);
            slope2X[p] = axx[p] * gx[p] + ddx[p] * hxx[p] + axz[p] * gz[p] + ddz[p] * hxz[p];
            slope2Z[p] = axz[p] * gx[p] + ddx[p] * hxz[p] + azz[p] * gz[p] + ddz[p] * hzz[p];
        }
    }

    /**
     * Adds the second-order correction (SecondOrderCorrection) and its slopes on the grid x0 + i dx, z0 + k dz into
     * height2, slope2X, slope2Z, by phasor recurrence; with gridSecondOrder this is secondOrder on the grid.
     */
    public void gridCorrection(double x0, double z0, double dx, double dz, int nx, int nz, double t, double[] height2, double[] slope2X, double[] slope2Z) {
        gridCorrection(null, x0, z0, dx, dz, nx, nz, t, height2, slope2X, slope2Z);
    }

    /** gridCorrection with every phasor enveloped by the coast (null for open sea), as surfaceTerms with a coast. */
    public void gridCorrection(CoastalWaves coast, double x0, double z0, double dx, double dz, int nx, int nz, double t,
                               double[] height2, double[] slope2X, double[] slope2Z) {
        int points = nx * nz, rc = correction.cosModes.length, rs = correction.sinModes.length;
        if (rc + rs == 0) { return; }
        double[][] acc = new double[3 * (rc + rs)][points];
        double[] fxs = new double[points], fzs = new double[points];
        int[] base = coast == null ? null : stencils(coast, x0, z0, dx, dz, nx, nz, fxs, fzs).clone();
        int cnz = coast == null ? 0 : coast.nz();
        for (int c = 0; c < amplitude.length; c++) {
            double stepRe = Math.cos(kz[c] * dz), stepIm = Math.sin(kz[c] * dz);
            float[] cr = coast == null ? null : coast.re[c], ci = coast == null ? null : coast.im[c];
            for (int i = 0; i < nx; i++) {
                double theta = kx[c] * (x0 + i * dx) + kz[c] * z0 - omega[c] * t + phase[c], re = Math.cos(theta), im = Math.sin(theta);
                int row = i * nz;
                for (int j = 0; j < nz; j++) {
                    int p = row + j;
                    double cc = re, ss = im;
                    if (base != null && base[p] >= 0) {
                        int q = base[p];
                        double fx = fxs[p], fz = fzs[p];
                        double mr = (cr[q] * (1 - fz) + cr[q + 1] * fz) * (1 - fx) + (cr[q + cnz] * (1 - fz) + cr[q + cnz + 1] * fz) * fx;
                        double mi = (ci[q] * (1 - fz) + ci[q + 1] * fz) * (1 - fx) + (ci[q + cnz] * (1 - fz) + ci[q + cnz + 1] * fz) * fx;
                        cc = mr * re - mi * im; ss = mr * im + mi * re;
                    }
                    for (int r = 0; r < rc; r++) {
                        double u = correction.cosModes[r][c] * fold[c];
                        acc[3 * r][p] += u * cc; acc[3 * r + 1][p] -= u * sx[c] * ss; acc[3 * r + 2][p] -= u * sz[c] * ss;
                    }
                    for (int r = 0; r < rs; r++) {
                        double u = correction.sinModes[r][c] * fold[c];
                        int b = 3 * (rc + r);
                        acc[b][p] += u * ss; acc[b + 1][p] += u * sx[c] * cc; acc[b + 2][p] += u * sz[c] * cc;
                    }
                    double next = re * stepRe - im * stepIm;
                    im = re * stepIm + im * stepRe;
                    re = next;
                }
            }
        }
        for (int p = 0; p < points; p++) {
            for (int r = 0; r < rc + rs; r++) {
                double w = r < rc ? correction.cosWeights[r] : correction.sinWeights[r - rc];
                double v = acc[3 * r][p];
                height2[p] += w * v * v; slope2X[p] += 2 * w * v * acc[3 * r + 1][p]; slope2Z[p] += 2 * w * v * acc[3 * r + 2][p];
            }
        }
    }

    /**
     * Elevation and slopes on the grid x0 + i dx, z0 + k dz (i < nx, k < nz) at time t into height[i * nz + k],
     * slopeX and slopeZ. Phasors are advanced by complex multiplication along z and re-seeded with exact trig at
     * every row, so rounding stays at the 1e-12 level.
     */
    public void grid(double x0, double z0, double dx, double dz, int nx, int nz, double t, double[] height, double[] slopeX, double[] slopeZ) {
        Arrays.fill(height, 0, nx * nz, 0); Arrays.fill(slopeX, 0, nx * nz, 0); Arrays.fill(slopeZ, 0, nx * nz, 0);
        for (int c = 0; c < amplitude.length; c++) {
            double stepRe = Math.cos(kz[c] * dz), stepIm = Math.sin(kz[c] * dz), a = amplitude[c];
            for (int i = 0; i < nx; i++) {
                double theta = kx[c] * (x0 + i * dx) + kz[c] * z0 - omega[c] * t + phase[c], re = Math.cos(theta), im = Math.sin(theta);
                int row = i * nz;
                for (int j = 0; j < nz; j++) {
                    height[row + j] += a * re;
                    slopeX[row + j] -= a * kx[c] * im;
                    slopeZ[row + j] -= a * kz[c] * im;
                    double next = re * stepRe - im * stepIm;
                    im = re * stepIm + im * stepRe;
                    re = next;
                }
            }
        }
    }
}
