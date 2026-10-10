package dev.mcmod.aerocore.api;

import java.util.Arrays;

/**
 * How a sea's components change over a depth map: refraction, shoaling, sheltering behind land and depth-limited
 * breaking, by geometric optics on a grid (Dean and Dalrymple 1991, ch. 4; Coastal Engineering Manual 2002, II-3).
 * <ul>
 * <li>Phase: each component's phase psi solves the eikonal |grad psi| = k(h), k from the linear dispersion relation at
 * the local depth (3 x 3 averaged, so blocky beds do not kink the rays), by first-arrival fast marching (Sethian 1996)
 * from the grid edges the component enters by, where psi is the offshore plane wave (from every entry edge cell when
 * none is near the offshore depth). Land cells are obstacles. A body of water the open edges do not reach (a lake or
 * pool inside the grid) takes the sea at its upwind shore instead, with the free field's phase there: its first water
 * cell along each line in the wave's direction is a source, so an island inside it still shades its lee. Whether such
 * a basin would have these waves at all (a wind sea grows with fetch) is the region's choice.</li>
 * <li>Amplitude: wave energy flux E c_g is conserved along the ray tubes of a second march, the free field, in which
 * land is water as deep as the nearest water (so it neither blocks rays nor offers a shortcut past a shallow shore)
 * (refraction and shoaling, CEM II-3-10 to 13), d ln(E c_g)/ds = -div(k^), integrated upwind in arrival order;
 * focusing is limited at fold caustics (CAUSTIC_GAIN).</li>
 * <li>Shelter: behind islands and headlands the phase around the obstacle exceeds the free field's phase through it
 * by delta = k x (path excess), and the amplitude is Sommerfeld's half-plane factor at that path excess (Penney and
 * Price 1952; 0.5 on the shadow line), as for an edge-diffracted wave; the phase there is the first arrival around the
 * obstacle, the edge-diffracted phase (Keller 1962). Path excesses below SHADOW_NOISE rad are rounding, not shadow.</li>
 * <li>Breaking: a component may not exceed the linear steepness limit H/L <= 0.1 used elsewhere, nor its share of a
 * depth-saturated sea, Hs <= SATURATED_HS_PER_DEPTH h, all components scaled alike as SWAN distributes depth-induced
 * dissipation; the cap is applied while marching, so energy lost on a bar stays lost behind it. The fraction of energy
 * removed at each cell is kept for surf foam.</li>
 * </ul>
 * A wind sea (WindSea: waves raised by a wind blowing over the region) is not a sea arriving from offshore: wind
 * keeps generating it over every stretch of open water, so behind land it regrows with distance instead of staying in
 * shadow. Its envelope is the growth of each point's own fetch, the straight-line distance over water upwind to land
 * (Coastal Engineering Manual II-2-3: straight-line fetch, fetch width not used, after Resio and Vincent 1979), plus the
 * open water beyond the grid: Hs(U10, fetch) / Hs of the sea as given (SeaWind's JONSWAP growth, deep-water growth in
 * all depths as the CEM recommends), averaged over 3 x 3 nodes; all components alike, so the spectral shape is the
 * given one everywhere. The phase follows the free field (refraction over the bed, no shadows), and the breaking caps
 * apply as for any sea.
 * <p>
 * The result is an envelope per component, the phasor M = g e^{i delta} of its gain g and phase shift
 * delta = psi - k_ref . x, so the coastal component is Re(M a e^{i(k_ref . x - omega t + phi)}): offshore M = 1 and
 * the plane wave is unchanged. M is stored per grid node and interpolated bilinearly; on land and in water no wave
 * reaches it is 0, so the amplitude falls to zero across the last cell before the shore while the phase stays that of
 * the water side. Not modelled: reflection (from
 * cliffs and walls up to 90 %, natural beaches 5-20 %), crossing wave trains behind islands (one arrival is kept, so a
 * small island's far lee stays at half amplitude instead of healing), currents, set-up and run-up.
 */
public final class CoastalWaves {
    /**
     * Significant height of a depth-saturated (inner surf zone) sea per unit depth: Thornton and Guza (1983)
     * H_rms = 0.42 h, with Hs = sqrt(2) H_rms for Rayleigh-distributed heights (0.59; CEM II-4-10 gives H_m0 = 0.6 h).
     */
    public static final double SATURATED_HS_PER_DEPTH = 0.42 * Math.sqrt(2);
    /** The linear steepness limit H/L of LinearWave. */
    public static final double MAX_STEEPNESS = LinearWave.MAX_STEEPNESS;
    /** Largest focusing by refraction over the shoaling gain (see transport). */
    public static final double CAUSTIC_GAIN = 2.5;
    /** Path excess (phase, rad) below which a cell counts as lit: marching rounding, about 1/6000 of a wavelength. */
    public static final double SHADOW_NOISE = 1e-3;
    private static final double G = Atmosphere.GRAVITY_MPS2, MIN_DEPTH = 0.05;

    private final double x0, z0, spacing;
    private final int nx, nz;
    /** Smoothed depth (land 0 or less) and the fraction of wave energy removed by breaking, per node. */
    final float[] depth, breaking;
    /** Envelope phasors per component and node: re[c][i nz + j], im[c][i nz + j]. */
    final float[][] re, im;
    /** Local wavenumber (rad/m) and coth(k h) per component and node, for the second-order kernels near the shore. */
    final float[][] kLocal, cothLocal;
    /** Nodes where some component's envelope differs from 1 by more than CHANGED. */
    private final boolean[] changed;
    private final boolean uniform;
    private final double maxGain;
    /** Envelope change below which a node counts as open sea (1 % in amplitude or 0.01 rad in phase). */
    public static final double CHANGED = 0.01;

    private CoastalWaves(double x0, double z0, double spacing, int nx, int nz, float[] depth, float[][] re, float[][] im, float[] breaking,
                         float[][] kLocal, float[][] cothLocal) {
        this.x0 = x0; this.z0 = z0; this.spacing = spacing; this.nx = nx; this.nz = nz;
        this.depth = depth; this.re = re; this.im = im; this.breaking = breaking; this.kLocal = kLocal; this.cothLocal = cothLocal;
        changed = new boolean[nx * nz];
        boolean same = true;
        double largest = 0;
        for (int c = 0; c < re.length; c++) for (int cell = 0; cell < changed.length; cell++) {
            double r = re[c][cell], i = im[c][cell];
            largest = Math.max(largest, Math.hypot(r, i));
            if (Math.abs(r - 1) > CHANGED || Math.abs(i) > CHANGED) { changed[cell] = true; same = false; }
        }
        uniform = same;
        maxGain = re.length == 0 ? 1 : largest;
    }

    /** True when no component changes anywhere by more than 1 % (open water of uniform depth). */
    public boolean uniform() { return uniform; }
    /** Largest envelope gain anywhere (shoaling and focusing), at least 0. */
    public double maxGain() { return maxGain; }
    public int components() { return re.length; }
    public double x0() { return x0; }
    public double z0() { return z0; }
    public double spacing() { return spacing; }
    public int nx() { return nx; }
    public int nz() { return nz; }

    /**
     * A sea raised by a wind over the region: 10 m wind speed (m/s), the direction it blows toward (rad), the open
     * water upwind of the grid (m; infinite for an open sea) and the peak period the sea was synthesised with (s; NaN:
     * the period of its largest component, which is coarse with few components).
     */
    public record WindSea(double windMps, double directionRadians, double fetchBeyondM, double peakPeriodS) {
        public WindSea {
            if (!(windMps > 0 && windMps <= SeaWind.MAX_WIND) || !Double.isFinite(directionRadians) || !(fetchBeyondM >= 0)
                    || !(Double.isNaN(peakPeriodS) || peakPeriodS > 0)) {
                throw new IllegalArgumentException("Invalid wind sea");
            }
        }
        public WindSea(double windMps, double directionRadians, double fetchBeyondM) { this(windMps, directionRadians, fetchBeyondM, Double.NaN); }
    }

    /**
     * Builds the envelopes of a sea over a depth grid: cell (i, j) at (x0 + i spacing, z0 + j spacing), index
     * i * nz + j, depth in m (0 or less is land).
     */
    public static CoastalWaves build(double x0, double z0, double spacing, int nx, int nz, float[] depth, WaveField waves) {
        return build(x0, z0, spacing, nx, nz, depth, waves, null);
    }

    /**
     * Straight-line fetch of every water node (NaN on land), m: the distance upwind (against directionRadians) over
     * water to the first land node, or to the grid's edge plus fetchBeyond.
     */
    public static double[] fetches(double spacing, int nx, int nz, float[] depth, double directionRadians, double fetchBeyond) {
        double[] out = new double[nx * nz];
        double ex = Math.cos(directionRadians), ez = Math.sin(directionRadians), step = 0.5;
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int cell = i * nz + j;
            if (depth[cell] <= 0) { out[cell] = Double.NaN; continue; }
            double t = 0;
            while (true) {
                double next = t + step;
                int a = (int) Math.round(i - next * ex), b = (int) Math.round(j - next * ez);
                if (a < 0 || a >= nx || b < 0 || b >= nz) { out[cell] = next * spacing + fetchBeyond; break; }
                if (depth[a * nz + b] <= 0) { out[cell] = next * spacing; break; }
                t = next;
            }
        }
        return out;
    }

    /** Envelopes over a depth grid; with wind, the sea is a wind sea that grows with each node's fetch (see the class notes). */
    public static CoastalWaves build(double x0, double z0, double spacing, int nx, int nz, float[] depth, WaveField waves, WindSea wind) {
        Checks.positive(spacing, "Spacing");
        if (nx < 2 || nz < 2 || depth.length != nx * nz) { throw new IllegalArgumentException("Invalid depth grid"); }
        int n = waves.components().size(), cells = nx * nz;
        double m0 = 0;
        for (LinearWave w : waves.components()) { m0 += 0.5 * w.amplitudeM() * w.amplitudeM(); }
        double hs0 = 4 * Math.sqrt(m0);
        float[][] re = new float[n][cells], im = new float[n][cells];
        double[] removed = new double[cells], before = new double[cells];
        float[] smooth = smoothed(depth, nx, nz), extended = extended(smooth, nx, nz);
        float[][] kLocal = new float[n][cells], cothLocal = new float[n][cells];
        for (int c = 0; c < n; c++) {
            LinearWave w = waves.components().get(c);
            double omega = w.angularFrequency(), reference = Double.isFinite(w.depthM()) ? w.depthM() : Double.POSITIVE_INFINITY;
            for (int cell = 0; cell < cells; cell++) {
                double h = smooth[cell] > 0 ? Math.max(smooth[cell], MIN_DEPTH) : reference;
                if (h > 1e3) { h = reference; }
                double k = wavenumber(omega, h);
                kLocal[c][cell] = (float) k;
                cothLocal[c][cell] = (float) (Double.isFinite(h) ? 1 / Math.tanh(k * h) : 1);
            }
        }
        if (extended == null) { return new CoastalWaves(x0, z0, spacing, nx, nz, smooth, new float[n][cells], new float[n][cells], new float[cells], kLocal, cothLocal); }
        double[] growth = wind == null || hs0 <= 0 ? null
                : growth(spacing, nx, nz, depth, wind, hs0, Double.isNaN(wind.peakPeriodS()) ? peakPeriod(waves) : wind.peakPeriodS());
        for (int c = 0; c < n; c++) {
            LinearWave w = waves.components().get(c);
            double k0 = w.wavenumber(), ex = Math.cos(w.directionRadians()), ez = Math.sin(w.directionRadians());
            double omega = w.angularFrequency(), reference = Double.isFinite(w.depthM()) ? w.depthM() : 1e4;
            // Without obstacles (refraction and shoaling without shelter), then with land as obstacles (the phase);
            // their difference is the path excess around headlands and islands.
            Field free = march(x0, z0, spacing, nx, nz, extended, omega, k0, ex, ez, null);
            if (growth != null) {
                windComponent(free, growth, smooth, depth, x0, z0, spacing, nz, k0, ex, ez, w.amplitudeM(), hs0, re[c], im[c], removed, before);
                continue;
            }
            Field blocked = march(x0, z0, spacing, nx, nz, smooth, omega, k0, ex, ez, free.psi);
            double[] g = new double[cells];
            transport(free, spacing, nx, nz, smooth, extended, omega, w.amplitudeM(), hs0, g, groupVelocity(omega, k0, reference), ex, ez, removed, before);
            double[] shelter = shelter(blocked.psi, free.psi, depth, nx, nz);
            for (int cell = 0; cell < cells; cell++) {
                if (depth[cell] <= 0 || !Double.isFinite(blocked.psi[cell])) { continue; }
                double x = x0 + (cell / nz) * spacing, z = z0 + (cell % nz) * spacing;
                double open = g[cell] * shelter[cell], hc = Math.max(smooth[cell], MIN_DEPTH), k = blocked.k[cell];
                // Breaking caps on the final amplitude: the linear steepness limit, and this component's share of a
                // depth-saturated sea (all components scaled alike, as SWAN distributes depth-induced dissipation).
                double cap = Math.min(hs0 > 0 ? SATURATED_HS_PER_DEPTH * hc / hs0 : Double.POSITIVE_INFINITY,
                        MAX_STEEPNESS * Math.PI / (k * Math.max(w.amplitudeM(), 1e-12)));
                open = Math.min(open, cap);
                double delta = blocked.psi[cell] - k0 * (ex * x + ez * z);
                re[c][cell] = (float) (open * Math.cos(delta)); im[c][cell] = (float) (open * Math.sin(delta));
            }
        }
        float[] breaking = new float[cells];
        for (int cell = 0; cell < cells; cell++) { breaking[cell] = before[cell] > 0 ? (float) (removed[cell] / before[cell]) : 0; }
        return new CoastalWaves(x0, z0, spacing, nx, nz, smooth, re, im, breaking, kLocal, cothLocal);
    }

    /** Period of the component with the largest amplitude, s: the spectral peak of a single-summation sea. */
    private static double peakPeriod(WaveField waves) {
        LinearWave peak = waves.components().getFirst();
        for (LinearWave w : waves.components()) { if (w.amplitudeM() > peak.amplitudeM()) { peak = w; } }
        return 2 * Math.PI / peak.angularFrequency();
    }

    /**
     * Growth factor of a wind sea per water node: Hs(U10, fetch) over the sea's Hs, averaged over the water of 3 x 3
     * nodes. The components keep their frequencies, so a longer fetch can only raise their amplitudes: the growth stops
     * where the sea would pass DNV's steepness limit 2 pi Hs / (g Tp^2) = 1/15 at the spectrum's own peak period
     * (SeaWind.MAX_STEEPNESS; the real sea there would also have lengthened its period, which this field cannot).
     */
    private static double[] growth(double spacing, int nx, int nz, float[] depth, WindSea wind, double hs0, double peakPeriodS) {
        double[] fetch = fetches(spacing, nx, nz, depth, wind.directionRadians(), wind.fetchBeyondM()), raw = new double[nx * nz], out = new double[nx * nz];
        double steepest = Math.max(hs0, (1 - 1e-9) * SeaWind.MAX_STEEPNESS * Atmosphere.GRAVITY_MPS2 * peakPeriodS * peakPeriodS / (2 * Math.PI));
        for (int cell = 0; cell < raw.length; cell++) {
            if (depth[cell] <= 0) { continue; }
            raw[cell] = Math.min(SeaWind.sea(wind.windMps(), Math.max(fetch[cell], 0.5 * spacing), Double.POSITIVE_INFINITY)[0], steepest) / hs0;
        }
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int cell = i * nz + j;
            if (depth[cell] <= 0) { continue; }
            double sum = 0;
            int count = 0;
            for (int a = Math.max(0, i - 1); a <= Math.min(nx - 1, i + 1); a++) for (int b = Math.max(0, j - 1); b <= Math.min(nz - 1, j + 1); b++) {
                int m = a * nz + b;
                if (depth[m] > 0) { sum += raw[m]; count++; }
            }
            out[cell] = sum / count;
        }
        return out;
    }

    /** One component of a wind sea: amplitude by the fetch growth with the breaking caps, phase from the free field. */
    private static void windComponent(Field free, double[] growth, float[] smooth, float[] depth, double x0, double z0, double spacing, int nz,
                                      double k0, double ex, double ez, double amplitude, double hs0, float[] re, float[] im, double[] removed, double[] before) {
        for (int cell = 0; cell < depth.length; cell++) {
            if (depth[cell] <= 0 || !Double.isFinite(free.psi[cell])) { continue; }
            double x = x0 + (cell / nz) * spacing, z = z0 + (cell % nz) * spacing, hc = Math.max(smooth[cell], MIN_DEPTH), k = free.k[cell];
            double g = growth[cell], cap = Math.min(SATURATED_HS_PER_DEPTH * hc / hs0, MAX_STEEPNESS * Math.PI / (k * Math.max(amplitude, 1e-12)));
            before[cell] += amplitude * amplitude * g * g;
            if (g > cap) { removed[cell] += amplitude * amplitude * (g * g - cap * cap); g = cap; }
            double delta = free.psi[cell] - k0 * (ex * x + ez * z);
            re[cell] = (float) (g * Math.cos(delta)); im[cell] = (float) (g * Math.sin(delta));
        }
    }

    /**
     * Wavenumber at depth h for angular frequency omega, rad/m: Fenton and McKee's (1990) explicit approximation
     * k0 / tanh((k0 h)^(3/4))^(2/3) (within 1.7 %), refined by Newton on omega^2 = g k tanh(k h) to 1e-12.
     */
    public static double wavenumber(double omega, double h) {
        double deep = omega * omega / G;
        if (!Double.isFinite(h) || deep * h > 20) { return deep; }
        double k = deep / Math.pow(Math.tanh(Math.pow(deep * h, 0.75)), 2.0 / 3);
        for (int i = 0; i < 50; i++) {
            double t = Math.tanh(k * h), f = G * k * t - omega * omega, d = G * (t + k * h * (1 - t * t));
            double next = k - f / d;
            if (Math.abs(next - k) < 1e-12 * k) { return next; }
            k = next;
        }
        return k;
    }

    /** Group velocity at wavenumber k and depth h, m/s. */
    public static double groupVelocity(double omega, double k, double h) {
        double kh = k * h, ratio = kh > 20 ? 0 : 2 * kh / Math.sinh(2 * kh);
        return 0.5 * omega / k * (1 + ratio);
    }

    private record Field(double[] psi, double[] k, int[] order, int accepted, boolean[] source) {}

    /**
     * First-order fast marching of |grad psi| = k(h) from the edges the wave enters by; land (depth <= 0) is an
     * obstacle. With enclosedPhase (the free field's phase), water bodies no edge reaches get sources at their
     * upwind shores with that phase.
     */
    private static Field march(double x0, double z0, double h, int nx, int nz, float[] depth, double omega, double k0, double ex, double ez,
                               double[] enclosedPhase) {
        int cells = nx * nz;
        double[] psi = new double[cells], k = new double[cells];
        Arrays.fill(psi, Double.POSITIVE_INFINITY);
        byte[] state = new byte[cells];
        boolean[] source = new boolean[cells];
        int[] order = new int[cells];
        Heap heap = new Heap(cells, psi);
        for (int cell = 0; cell < cells; cell++) {
            if (depth[cell] > 0) { k[cell] = wavenumber(omega, Math.max(depth[cell], MIN_DEPTH)); }
        }
        // Only edges at about the offshore wavenumber inject the plane wave (shallower edge cells are reached from
        // inside), unless no entry edge is that deep.
        boolean anyDeep = false;
        for (int pass = 0; pass < 2 && !anyDeep; pass++) {
            for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
                int cell = i * nz + j;
                if (depth[cell] <= 0) { continue; }
                boolean entry = (i == 0 && ex > 1e-9) || (i == nx - 1 && ex < -1e-9) || (j == 0 && ez > 1e-9) || (j == nz - 1 && ez < -1e-9);
                if (!entry || (pass == 0 && k[cell] > 1.1 * k0)) { continue; }
                anyDeep = true;
                psi[cell] = k0 * (ex * (x0 + i * h) + ez * (z0 + j * h));
                source[cell] = true;
                state[cell] = 1;
                heap.push(cell);
            }
        }
        if (enclosedPhase != null) { enclosedSources(x0, z0, h, nx, nz, depth, k0, ex, ez, enclosedPhase, psi, state, source, heap); }
        int accepted = 0;
        while (!heap.isEmpty()) {
            int cell = heap.pop();
            if (state[cell] == 2) { continue; }
            state[cell] = 2;
            order[accepted++] = cell;
            int i = cell / nz, j = cell % nz;
            int[][] neighbours = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
            for (int[] nb : neighbours) {
                if (nb[0] < 0 || nb[0] >= nx || nb[1] < 0 || nb[1] >= nz) { continue; }
                int m = nb[0] * nz + nb[1];
                if (depth[m] <= 0 || state[m] == 2 || source[m]) { continue; }
                double value = update(psi, state, nb[0], nb[1], nx, nz, h, k[m]);
                if (value < psi[m]) {
                    psi[m] = value;
                    if (state[m] == 1) { heap.decrease(m); } else { state[m] = 1; heap.push(m); }
                }
            }
        }
        return new Field(psi, k, order, accepted, source);
    }

    /**
     * Sources for water bodies no edge source reaches: label the water cells by 4-connected body, and in each body
     * without a source take every cell from which a line drawn upwind (against e) leaves the grid without crossing
     * water of the same body.
     */
    private static void enclosedSources(double x0, double z0, double h, int nx, int nz, float[] depth, double k0, double ex, double ez,
                                        double[] phase, double[] psi, byte[] state, boolean[] source, Heap heap) {
        int cells = nx * nz;
        int[] body = new int[cells];
        Arrays.fill(body, -1);
        int[] queue = new int[cells];
        java.util.List<Boolean> fed = new java.util.ArrayList<>();
        for (int start = 0; start < cells; start++) {
            if (depth[start] <= 0 || body[start] >= 0) { continue; }
            int label = fed.size(), head = 0, tail = 0;
            boolean hasSource = false;
            body[start] = label; queue[tail++] = start;
            while (head < tail) {
                int cell = queue[head++], i = cell / nz, j = cell % nz;
                hasSource |= source[cell];
                int[][] neighbours = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
                for (int[] nb : neighbours) {
                    if (nb[0] < 0 || nb[0] >= nx || nb[1] < 0 || nb[1] >= nz) { continue; }
                    int m = nb[0] * nz + nb[1];
                    if (depth[m] > 0 && body[m] < 0) { body[m] = label; queue[tail++] = m; }
                }
            }
            fed.add(hasSource);
        }
        for (int cell = 0; cell < cells; cell++) {
            if (body[cell] < 0 || fed.get(body[cell])) { continue; }
            int i = cell / nz, j = cell % nz;
            boolean first = true;
            for (double t = 0.5; first; t += 0.5) {
                int a = (int) Math.round(i - t * ex), b = (int) Math.round(j - t * ez);
                if (a < 0 || a >= nx || b < 0 || b >= nz) { break; }
                int m = a * nz + b;
                if (m != cell && body[m] == body[cell]) { first = false; }
            }
            if (!first) { continue; }
            psi[cell] = Double.isFinite(phase[cell]) ? phase[cell] : k0 * (ex * (x0 + i * h) + ez * (z0 + j * h));
            source[cell] = true;
            state[cell] = 1;
            heap.push(cell);
        }
    }

    /** Upwind eikonal update from accepted neighbours: ((psi - a)/h)^2 + ((psi - b)/h)^2 = k^2, or one-sided. */
    private static double update(double[] psi, byte[] state, int i, int j, int nx, int nz, double h, double k) {
        double a = Math.min(known(psi, state, i - 1, j, nx, nz), known(psi, state, i + 1, j, nx, nz));
        double b = Math.min(known(psi, state, i, j - 1, nx, nz), known(psi, state, i, j + 1, nx, nz));
        double kh = k * h;
        if (!Double.isFinite(a)) { return b + kh; }
        if (!Double.isFinite(b)) { return a + kh; }
        if (Math.abs(a - b) >= kh) { return Math.min(a, b) + kh; }
        return 0.5 * (a + b + Math.sqrt(2 * kh * kh - (a - b) * (a - b)));
    }

    private static double known(double[] psi, byte[] state, int i, int j, int nx, int nz) {
        if (i < 0 || i >= nx || j < 0 || j >= nz) { return Double.POSITIVE_INFINITY; }
        int cell = i * nz + j;
        return state[cell] == 2 ? psi[cell] : Double.POSITIVE_INFINITY;
    }

    /**
     * Energy flux along the rays of an unobstructed field in arrival order, with the breaking caps; gain = amplitude /
     * offshore amplitude. Refraction focusing is limited to CAUSTIC_GAIN over shoaling (fold-caustic peaks are
     * 1.69 (k rho)^(1/6) of the single-branch amplitude, 2.5 at k rho = 10, and directional spreading smears them).
     * Adds the energy each water cell's cap removes, and what it held before, to removed / before (weighted by a^2).
     * water is the bed with land at 0 or less, depth the same with land filled in (extended).
     */
    private static void transport(Field f, double h, int nx, int nz, float[] water, float[] depth, double omega, double amplitude,
                                  double hs0, double[] gain, double cg0, double ex, double ez, double[] removed, double[] before) {
        int cells = nx * nz;
        double[] dx = new double[cells], dz = new double[cells], lnFlux = new double[cells];
        boolean[] done = new boolean[cells];
        // Ray directions from the upwind phase gradient, the one the arrival came by (so a cell where two fronts meet
        // takes one front's direction instead of an average across them).
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int cell = i * nz + j;
            if (!Double.isFinite(f.psi[cell])) { continue; }
            double gx = upwind(f.psi, i, j, nx, nz, true, h), gz = upwind(f.psi, i, j, nx, nz, false, h), length = Math.hypot(gx, gz);
            if (f.source[cell] || length == 0) { gx = ex; gz = ez; length = 1; }
            dx[cell] = gx / length; dz[cell] = gz / length;
        }
        // Ray divergence by differences taken upstream along the ray: where two first-arrival fronts meet (behind an
        // island) the other front's directions are downstream and do not enter, so the meeting line is not a caustic.
        double[] divergence = new double[cells];
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int cell = i * nz + j;
            if (!Double.isFinite(f.psi[cell])) { continue; }
            divergence[cell] = upstream(dx, f.psi, i, j, nx, nz, true, dx[cell] >= 0, h) + upstream(dz, f.psi, i, j, nx, nz, false, dz[cell] >= 0, h);
        }
        for (int q = 0; q < f.accepted; q++) {
            int cell = f.order[q], i = cell / nz, j = cell % nz;
            double hc = Math.max(depth[cell], MIN_DEPTH), k = f.k[cell], cg = groupVelocity(omega, k, hc);
            double ln;
            // The offshore energy flux enters at the sources (shoaled there when the edge is shallower).
            if (f.source[cell]) { ln = Math.log(cg0); }
            else {
                // Upwind point one spacing back along the ray, from the cells already done around it.
                double ux = i - dx[cell], uz = j - dz[cell];
                int a = (int) Math.floor(ux), b = (int) Math.floor(uz);
                double fx = ux - a, fz = uz - b, sum = 0, weight = 0;
                for (int di = 0; di <= 1; di++) for (int dj = 0; dj <= 1; dj++) {
                    int ci = a + di, cj = b + dj;
                    if (ci < 0 || ci >= nx || cj < 0 || cj >= nz) { continue; }
                    int m = ci * nz + cj;
                    if (!done[m]) { continue; }
                    double w = (di == 0 ? 1 - fx : fx) * (dj == 0 ? 1 - fz : fz);
                    sum += w * lnFlux[m]; weight += w;
                }
                ln = weight > 1e-9 ? sum / weight - divergence[cell] * h : bestNeighbour(lnFlux, done, i, j, nx, nz, Math.log(cg));
            }
            double g = Math.exp(0.5 * (ln - Math.log(cg)));
            double cap = Math.min(Math.min(hs0 > 0 ? SATURATED_HS_PER_DEPTH * hc / hs0 : Double.POSITIVE_INFINITY,
                    MAX_STEEPNESS * Math.PI / (k * Math.max(amplitude, 1e-12))), CAUSTIC_GAIN * Math.sqrt(cg0 / cg));
            if (water[cell] > 0) { before[cell] += amplitude * amplitude * g * g; }
            if (g > cap) {
                if (water[cell] > 0) { removed[cell] += amplitude * amplitude * (g * g - cap * cap); }
                g = cap;
                ln = Math.log(cg) + 2 * Math.log(g);
            }
            lnFlux[cell] = ln;
            gain[cell] = g;
            done[cell] = true;
        }
    }

    /**
     * The bed with every land cell given the mean depth of its neighbours one step nearer the water (breadth-first
     * from the shore), for the free field; null when there is no water.
     */
    private static float[] extended(float[] depth, int nx, int nz) {
        int cells = nx * nz;
        float[] out = depth.clone();
        int[] layer = new int[cells];
        int[] queue = new int[cells];
        int head = 0, tail = 0;
        for (int cell = 0; cell < cells; cell++) { if (depth[cell] > 0) { queue[tail++] = cell; layer[cell] = 0; } else { layer[cell] = -1; } }
        if (tail == 0) { return null; }
        while (head < tail) {
            int cell = queue[head++], i = cell / nz, j = cell % nz;
            int[][] neighbours = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
            for (int[] nb : neighbours) {
                if (nb[0] < 0 || nb[0] >= nx || nb[1] < 0 || nb[1] >= nz) { continue; }
                int m = nb[0] * nz + nb[1];
                if (layer[m] >= 0) { continue; }
                layer[m] = layer[cell] + 1;
                // Mean of the neighbours already filled (one layer nearer the water).
                double sum = 0;
                int count = 0;
                int a = m / nz, b = m % nz;
                int[][] around = {{a - 1, b}, {a + 1, b}, {a, b - 1}, {a, b + 1}};
                for (int[] q : around) {
                    if (q[0] < 0 || q[0] >= nx || q[1] < 0 || q[1] >= nz) { continue; }
                    int r = q[0] * nz + q[1];
                    if (layer[r] >= 0 && layer[r] < layer[m]) { sum += out[r]; count++; }
                }
                out[m] = (float) (sum / count);
                queue[tail++] = m;
            }
        }
        return out;
    }

    /** Depth averaged over the water cells of each 3 x 3 block (land stays land): wave speeds from blocky beds. */
    private static float[] smoothed(float[] depth, int nx, int nz) {
        float[] out = new float[depth.length];
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int cell = i * nz + j;
            if (depth[cell] <= 0) { out[cell] = depth[cell]; continue; }
            double sum = 0;
            int count = 0;
            for (int a = Math.max(0, i - 1); a <= Math.min(nx - 1, i + 1); a++) for (int b = Math.max(0, j - 1); b <= Math.min(nz - 1, j + 1); b++) {
                float d = depth[a * nz + b];
                if (d > 0) { sum += d; count++; }
            }
            out[cell] = (float) (sum / count);
        }
        return out;
    }

    /**
     * Shelter factor from the extra phase delta = psi(around obstacles) - psi(through them): Sommerfeld's half-plane
     * solution (Penney and Price 1952) at sigma = sqrt(2 delta / pi), |F| = sqrt((f^2 + g^2) / 2) with Abramowitz and
     * Stegun's 7.3.32-33 auxiliary functions: 0.5 on the shadow line, 0.2 at sigma = 1. Lit cells (delta = 0) keep 1;
     * the step at the shadow line is softened by one 3 x 3 average of the factor.
     */
    private static double[] shelter(double[] blocked, double[] free, float[] depth, int nx, int nz) {
        int cells = nx * nz;
        double[] raw = new double[cells];
        for (int cell = 0; cell < cells; cell++) {
            double delta = blocked[cell] - free[cell];
            if (!Double.isFinite(delta) || depth[cell] <= 0) { raw[cell] = Double.isFinite(blocked[cell]) ? 1 : 0; continue; }
            if (delta <= SHADOW_NOISE) { raw[cell] = 1; continue; }
            double z = Math.sqrt(2 * delta / Math.PI);
            double f = (1 + 0.926 * z) / (2 + 1.792 * z + 3.104 * z * z), g = 1 / (2 + 4.142 * z + 3.492 * z * z + 6.670 * z * z * z);
            raw[cell] = Math.sqrt((f * f + g * g) / 2);
        }
        double[] out = new double[cells];
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            int cell = i * nz + j;
            if (depth[cell] <= 0) { continue; }
            double sum = 0;
            int count = 0;
            for (int a = Math.max(0, i - 1); a <= Math.min(nx - 1, i + 1); a++) for (int b = Math.max(0, j - 1); b <= Math.min(nz - 1, j + 1); b++) {
                int m = a * nz + b;
                if (depth[m] > 0) { sum += raw[m]; count++; }
            }
            out[cell] = sum / count;
        }
        return out;
    }

    private static double bestNeighbour(double[] values, boolean[] done, int i, int j, int nx, int nz, double fallback) {
        int[][] neighbours = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
        for (int[] nb : neighbours) {
            if (nb[0] < 0 || nb[0] >= nx || nb[1] < 0 || nb[1] >= nz) { continue; }
            int m = nb[0] * nz + nb[1];
            if (done[m]) { return values[m]; }
        }
        return fallback;
    }

    /** Gradient component of psi from the lower of the two neighbours along the axis, as in the eikonal update; 0 if neither is lower. */
    private static double upwind(double[] psi, int i, int j, int nx, int nz, boolean alongX, double h) {
        int di = alongX ? 1 : 0, dj = alongX ? 0 : 1, cell = i * nz + j;
        double back = valid(psi, i - di, j - dj, nx, nz) ? psi[(i - di) * nz + j - dj] : Double.POSITIVE_INFINITY;
        double ahead = valid(psi, i + di, j + dj, nx, nz) ? psi[(i + di) * nz + j + dj] : Double.POSITIVE_INFINITY;
        if (back <= ahead && back < psi[cell]) { return (psi[cell] - back) / h; }
        if (ahead < back && ahead < psi[cell]) { return -(psi[cell] - ahead) / h; }
        return 0;
    }

    /** One-sided difference of v from the upstream neighbour (backward when the ray runs toward +), else 0 when it is unreached. */
    private static double upstream(double[] v, double[] reach, int i, int j, int nx, int nz, boolean alongX, boolean backward, double h) {
        int di = alongX ? 1 : 0, dj = alongX ? 0 : 1, cell = i * nz + j;
        if (backward) { return valid(reach, i - di, j - dj, nx, nz) ? (v[cell] - v[(i - di) * nz + j - dj]) / h : 0; }
        return valid(reach, i + di, j + dj, nx, nz) ? (v[(i + di) * nz + j + dj] - v[cell]) / h : 0;
    }

    private static boolean valid(double[] reach, int i, int j, int nx, int nz) {
        return i >= 0 && i < nx && j >= 0 && j < nz && Double.isFinite(reach[i * nz + j]);
    }

    /**
     * Envelope phasors of every component at (x, z), bilinear on the grid, into re[c], im[c], and their x and z
     * derivatives (per m) into reX, imX, reZ, imZ when those are not null. Returns false, with M = 1 and no
     * derivatives, outside the grid or when the field is uniform.
     */
    public boolean envelopes(double x, double z, double[] reOut, double[] imOut, double[] reX, double[] imX, double[] reZ, double[] imZ) {
        int n = re.length;
        double gx = (x - x0) / spacing, gz = (z - z0) / spacing;
        if (uniform || !(gx >= 0) || gx > nx - 1 || !(gz >= 0) || gz > nz - 1) {
            Arrays.fill(reOut, 0, n, 1); Arrays.fill(imOut, 0, n, 0);
            if (reX != null) { Arrays.fill(reX, 0, n, 0); Arrays.fill(imX, 0, n, 0); Arrays.fill(reZ, 0, n, 0); Arrays.fill(imZ, 0, n, 0); }
            return false;
        }
        int i = Math.min((int) gx, nx - 2), j = Math.min((int) gz, nz - 2);
        double fx = gx - i, fz = gz - j;
        int p00 = i * nz + j, p01 = p00 + 1, p10 = p00 + nz, p11 = p10 + 1;
        double w00 = (1 - fx) * (1 - fz), w01 = (1 - fx) * fz, w10 = fx * (1 - fz), w11 = fx * fz;
        for (int c = 0; c < n; c++) {
            float[] r = re[c], m = im[c];
            reOut[c] = w00 * r[p00] + w01 * r[p01] + w10 * r[p10] + w11 * r[p11];
            imOut[c] = w00 * m[p00] + w01 * m[p01] + w10 * m[p10] + w11 * m[p11];
            if (reX != null) {
                reX[c] = ((r[p10] - r[p00]) * (1 - fz) + (r[p11] - r[p01]) * fz) / spacing;
                imX[c] = ((m[p10] - m[p00]) * (1 - fz) + (m[p11] - m[p01]) * fz) / spacing;
                reZ[c] = ((r[p01] - r[p00]) * (1 - fx) + (r[p11] - r[p10]) * fx) / spacing;
                imZ[c] = ((m[p01] - m[p00]) * (1 - fx) + (m[p11] - m[p10]) * fx) / spacing;
            }
        }
        return true;
    }

    /**
     * Local wavenumber (rad/m) and coth(k h) of every component at (x, z), bilinear on the grid, into k[c], coth[c]:
     * the second-order kernels near the shore. False outside the grid or for a uniform field (then nothing is written).
     */
    public boolean kernels(double x, double z, double[] k, double[] coth) {
        double gx = (x - x0) / spacing, gz = (z - z0) / spacing;
        if (uniform || !(gx >= 0) || gx > nx - 1 || !(gz >= 0) || gz > nz - 1) { return false; }
        int i = Math.min((int) gx, nx - 2), j = Math.min((int) gz, nz - 2);
        double fx = gx - i, fz = gz - j;
        int p00 = i * nz + j, p01 = p00 + 1, p10 = p00 + nz, p11 = p10 + 1;
        double w00 = (1 - fx) * (1 - fz), w01 = (1 - fx) * fz, w10 = fx * (1 - fz), w11 = fx * fz;
        for (int c = 0; c < re.length; c++) {
            float[] kc = kLocal[c], cc = cothLocal[c];
            k[c] = w00 * kc[p00] + w01 * kc[p01] + w10 * kc[p10] + w11 * kc[p11];
            coth[c] = w00 * cc[p00] + w01 * cc[p01] + w10 * cc[p10] + w11 * cc[p11];
        }
        return true;
    }

    /** True when some node of the rectangle (or next to it) has an envelope differing from open sea. */
    public boolean changedIn(double minX, double minZ, double maxX, double maxZ) {
        if (uniform) { return false; }
        int i0 = Math.max(0, (int) Math.floor((minX - x0) / spacing)), i1 = Math.min(nx - 1, (int) Math.ceil((maxX - x0) / spacing));
        int j0 = Math.max(0, (int) Math.floor((minZ - z0) / spacing)), j1 = Math.min(nz - 1, (int) Math.ceil((maxZ - z0) / spacing));
        for (int i = i0; i <= i1; i++) for (int j = j0; j <= j1; j++) { if (changed[i * nz + j]) { return true; } }
        return false;
    }

    /** Fraction of the local wave energy depth-limited breaking removes at (x, z), 0 to 1 (bilinear). */
    public double breaking(double x, double z) { return bilinear(breaking, x, z, 0); }

    /** The smoothed depth the waves were refracted over at (x, z), bilinear, m (land 0 or less); NaN outside the grid. */
    public double depth(double x, double z) { return bilinear(depth, x, z, Double.NaN); }

    private double bilinear(float[] v, double x, double z, double outside) {
        double gx = (x - x0) / spacing, gz = (z - z0) / spacing;
        if (!(gx >= 0) || gx > nx - 1 || !(gz >= 0) || gz > nz - 1) { return outside; }
        int i = Math.min((int) gx, nx - 2), j = Math.min((int) gz, nz - 2);
        double fx = gx - i, fz = gz - j;
        int p = i * nz + j;
        return (v[p] * (1 - fz) + v[p + 1] * fz) * (1 - fx) + (v[p + nz] * (1 - fz) + v[p + nz + 1] * fz) * fx;
    }

    /** Binary min-heap of cell indices keyed by psi, with decrease-key. */
    private static final class Heap {
        private final int[] items, position;
        private final double[] key;
        private int size;
        Heap(int capacity, double[] key) { items = new int[capacity]; position = new int[capacity]; Arrays.fill(position, -1); this.key = key; }
        boolean isEmpty() { return size == 0; }
        void push(int cell) { items[size] = cell; position[cell] = size; up(size++); }
        void decrease(int cell) { up(position[cell]); }
        int pop() {
            int top = items[0];
            position[top] = -1;
            if (--size > 0) { items[0] = items[size]; position[items[0]] = 0; down(0); }
            return top;
        }
        private void up(int i) {
            while (i > 0) {
                int parent = (i - 1) / 2;
                if (key[items[parent]] <= key[items[i]]) { return; }
                swap(i, parent); i = parent;
            }
        }
        private void down(int i) {
            while (true) {
                int l = 2 * i + 1, r = l + 1, m = i;
                if (l < size && key[items[l]] < key[items[m]]) { m = l; }
                if (r < size && key[items[r]] < key[items[m]]) { m = r; }
                if (m == i) { return; }
                swap(i, m); i = m;
            }
        }
        private void swap(int a, int b) {
            int t = items[a]; items[a] = items[b]; items[b] = t;
            position[items[a]] = a; position[items[b]] = b;
        }
    }
}
