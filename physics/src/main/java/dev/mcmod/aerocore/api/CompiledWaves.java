package dev.mcmod.aerocore.api;

/**
 * Fast evaluation of a WaveField: each component's wave vector, frequency, phase, amplitude and depth terms are
 * computed once and held in arrays, so a query costs one sin/cos (and one or two exp below the still level) per
 * component instead of re-deriving the dispersion relation. grid() fills a regular grid by phasor recurrence (one
 * complex multiply per component per point) for drawing the surface every frame. Every result equals the
 * corresponding WaveField / LinearWave method to rounding, including the depth clamping of the linear kinematics.
 */
public final class CompiledWaves {
    public static final CompiledWaves NONE = new CompiledWaves(WaveField.NONE);
    private final double[] kx, kz, k, omega, phase, amplitude, depth, cosineDenominator, sineDenominator, imageFactor;
    private final WaveField field;
    private final double maxAmplitude;

    public CompiledWaves(WaveField field) {
        this.field = field;
        int n = field.components().size();
        kx = new double[n]; kz = new double[n]; k = new double[n]; omega = new double[n]; phase = new double[n];
        amplitude = new double[n]; depth = new double[n]; cosineDenominator = new double[n]; sineDenominator = new double[n];
        imageFactor = new double[n];
        double sum = 0;
        for (int i = 0; i < n; i++) {
            LinearWave w = field.components().get(i);
            k[i] = w.wavenumber();
            kx[i] = k[i] * Math.cos(w.directionRadians()); kz[i] = k[i] * Math.sin(w.directionRadians());
            omega[i] = w.angularFrequency(); phase[i] = w.phaseRadians(); amplitude[i] = w.amplitudeM(); depth[i] = w.depthM();
            boolean finite = Double.isFinite(depth[i]);
            cosineDenominator[i] = finite ? 1 + Math.exp(-2 * k[i] * depth[i]) : 1;
            sineDenominator[i] = finite ? -Math.expm1(-2 * k[i] * depth[i]) : 1;
            // exp(-k (z + 2h)) = exp(-2kh) / exp(kz): the bottom image term costs a division, not a second exp.
            imageFactor[i] = finite ? Math.exp(-2 * k[i] * depth[i]) : 0;
            sum += amplitude[i];
        }
        maxAmplitude = sum;
    }

    public WaveField field() { return field; }
    public boolean isEmpty() { return amplitude.length == 0; }
    public int size() { return amplitude.length; }
    /** Upper bound of |eta| (sum of amplitudes). */
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

    /**
     * Elevation and slopes on the grid x0 + i dx, z0 + k dz (i < nx, k < nz) at time t into height[i * nz + k],
     * slopeX and slopeZ. Phasors are advanced by complex multiplication along z and re-seeded with exact trig at
     * every row, so rounding stays at the 1e-12 level.
     */
    public void grid(double x0, double z0, double dx, double dz, int nx, int nz, double t, double[] height, double[] slopeX, double[] slopeZ) {
        java.util.Arrays.fill(height, 0, nx * nz, 0); java.util.Arrays.fill(slopeX, 0, nx * nz, 0); java.util.Arrays.fill(slopeZ, 0, nx * nz, 0);
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
