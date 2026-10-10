package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CoastalWaves;
import dev.mcmod.aerocore.api.CompiledWaves;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import com.mojang.logging.LogUtils;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * A circular sea area: free surfaces at stillWaterY (the level measured at the centre) inside it move with a prescribed
 * linear wave field (none for a calm sea), shared by every body, vanilla entity and the client renderer. Inside
 * radius - taper the field is the plain superposition; across the outer taper its amplitude falls smoothly to zero so
 * the surface meets the static vanilla water. The taper is a blending choice, not wave physics. A sea-state record
 * (JONSWAP parameters) is kept for display when the field was synthesised from a spectrum.
 * <p>
 * With a depth survey of the terrain (Survey) the waves follow the coast: CoastalWaves refracts, shoals, shelters and
 * breaks them over the surveyed bed, built in the background on each side when the region is made or arrives (the
 * open-sea surface is used until it is ready, about a second). A wind (Wind) blows over the region: it sets the
 * whitecap coverage, drifts the surface water and pushes boats and bodies.
 */
public final class WaveRegion {
    public static final double MIN_RADIUS = 4, MAX_RADIUS = 128;
    public static final int MAX_COMPONENTS = 64;
    /** Free surfaces within this distance of stillWaterY belong to the region (vanilla source water is exact). */
    public static final double LEVEL_TOLERANCE = 0.01;
    /**
     * Spectrum parameters of a synthesised sea: Hs (m), Tp (s), gamma, mean direction (rad), cos^n exponent; for a wind
     * sea also the 10 m wind that raised it (m/s), the open water upwind of the survey (m, may be infinite) and the
     * representative fetch it was synthesised for (m), NaN otherwise. A wind sea grows with each spot's own fetch over the
     * coast (CoastalWaves.WindSea), and in time with the wind (SeaState: scale).
     */
    public record Sea(double significantHeightM, double peakPeriodS, double gamma, double directionRadians, double spreading,
                      double windMps, double fetchBeyondM, double fetchM) {
        public Sea(double significantHeightM, double peakPeriodS, double gamma, double directionRadians, double spreading) {
            this(significantHeightM, peakPeriodS, gamma, directionRadians, spreading, Double.NaN, Double.NaN, Double.NaN);
        }
        public boolean windSea() { return windMps > 0 && fetchBeyondM >= 0; }
        CoastalWaves.WindSea coastal() { return windSea() ? new CoastalWaves.WindSea(windMps, directionRadians, fetchBeyondM, peakPeriodS) : null; }
    }

    /**
     * Wind over the region: the 10 m mean speed above the still level (m/s) and the direction it blows toward (rad, like
     * wave headings). With rain and thunder speeds (not NaN) it follows the dimension's weather: the clear-weather speed,
     * moved toward the rain speed by the rain level and on toward the thunder speed by the thunder level (both 0 to 1,
     * Minecraft's own smooth transitions). Minecraft's weather has no wind, so these speeds are a rule (defaults from the
     * Beaufort scale, WEATHER_DEFAULTS). Gusts (WindGusts) ride on the mean wind.
     */
    public record Wind(double speedMps, double directionRadians, double rainMps, double thunderMps) {
        public static final Wind CALM = new Wind(0, 0);
        /**
         * Default weather speeds, m/s: clear 5 (Beaufort 3, gentle breeze, 3.4-5.4), rain 10 (Beaufort 5, fresh breeze,
         * 8.0-10.7), thunder 15 (Beaufort 7, near gale, 13.9-17.1). A rule: Minecraft's weather says nothing of wind.
         */
        public static final double[] WEATHER_DEFAULTS = {5, 10, 15};
        public Wind(double speedMps, double directionRadians) { this(speedMps, directionRadians, Double.NaN, Double.NaN); }
        public Wind {
            if (!speed(speedMps) || !Double.isFinite(directionRadians) || Double.isNaN(rainMps) != Double.isNaN(thunderMps)
                    || (!Double.isNaN(rainMps) && !(speed(rainMps) && speed(thunderMps)))) {
                throw new IllegalArgumentException("Wind speed must be in [0, 30] m/s");
            }
        }
        private static boolean speed(double v) { return v >= 0 && v <= SeaWind.MAX_WIND; }
        public boolean weather() { return !Double.isNaN(rainMps); }
        public boolean calm() { return speedMps == 0 && !weather(); }
        /** The 10 m mean wind speed now: fixed, or following the level's rain and thunder, m/s. */
        public double mean(net.minecraft.world.level.Level level) {
            if (!weather()) { return speedMps; }
            double rain = level.getRainLevel(1), thunder = level.getThunderLevel(1);
            return Math.clamp(speedMps + (rainMps - speedMps) * rain + (thunderMps - rainMps) * thunder, 0, SeaWind.MAX_WIND);
        }
        public double directionX() { return Math.cos(directionRadians); }
        public double directionZ() { return Math.sin(directionRadians); }
    }

    /**
     * Depth of the bed below the still level on a square grid over the region, nodes (x0 + i spacing, z0 + j spacing)
     * at index i nz + j, in decimetres; LAND where the column at the still level is not water, DEEP where no bed was
     * found within the survey depth or the chunk was not loaded (open water of the sea's own depth). Optionally the
     * height above the still level of the solid ground and of the top including leaves (decimetres, 0 or more), the
     * barriers that shelter the wind (SeaWind.shelter); null in surveys from before 0.14.
     */
    public static final class Survey {
        public static final short LAND = -1, DEEP = Short.MAX_VALUE;
        public static final double SPACING = 2;
        /** Deepest bed searched for, m. */
        public static final int MAX_DEPTH = 64;
        private final double x0, z0, spacing;
        private final int nx, nz, hash;
        private final short[] depthDm, groundDm, canopyDm;

        public Survey(double x0, double z0, double spacing, int nx, int nz, short[] depthDm) { this(x0, z0, spacing, nx, nz, depthDm, null, null); }
        public Survey(double x0, double z0, double spacing, int nx, int nz, short[] depthDm, short[] groundDm, short[] canopyDm) {
            if (!(spacing > 0) || nx < 2 || nz < 2 || nx > 512 || nz > 512 || depthDm.length != nx * nz || !Double.isFinite(x0) || !Double.isFinite(z0)
                    || (groundDm == null) != (canopyDm == null) || (groundDm != null && (groundDm.length != nx * nz || canopyDm.length != nx * nz))) {
                throw new IllegalArgumentException("Invalid depth survey");
            }
            this.x0 = x0; this.z0 = z0; this.spacing = spacing; this.nx = nx; this.nz = nz; this.depthDm = depthDm.clone();
            this.groundDm = groundDm == null ? null : groundDm.clone(); this.canopyDm = canopyDm == null ? null : canopyDm.clone();
            hash = Objects.hash(x0, z0, spacing, nx, nz, Arrays.hashCode(depthDm), Arrays.hashCode(groundDm), Arrays.hashCode(canopyDm));
        }

        /** Whether the survey has the barrier heights. */
        public boolean barriers() { return groundDm != null; }
        public short groundDm(int index) { return groundDm[index]; }
        public short canopyDm(int index) { return canopyDm[index]; }

        public double x0() { return x0; }
        public double z0() { return z0; }
        public double spacing() { return spacing; }
        public int nx() { return nx; }
        public int nz() { return nz; }
        public short depthDm(int index) { return depthDm[index]; }
        public int landNodes() { int n = 0; for (short d : depthDm) { if (d == LAND) { n++; } } return n; }

        /** Depths in m for CoastalWaves: land 0, DEEP the given open-sea depth. */
        float[] depths(double deep) {
            float[] out = new float[depthDm.length];
            for (int i = 0; i < out.length; i++) { out[i] = depthDm[i] == LAND ? 0 : depthDm[i] == DEEP ? (float) deep : Math.max(depthDm[i], 1) / 10f; }
            return out;
        }

        @Override public boolean equals(Object o) {
            return o == this || o instanceof Survey s && s.hash == hash && s.x0 == x0 && s.z0 == z0 && s.spacing == spacing && s.nx == nx && s.nz == nz
                    && Arrays.equals(s.depthDm, depthDm) && Arrays.equals(s.groundDm, groundDm) && Arrays.equals(s.canopyDm, canopyDm);
        }
        @Override public int hashCode() { return hash; }
    }

    public static final StreamCodec<FriendlyByteBuf, WaveRegion> STREAM_CODEC = StreamCodec.of(WaveRegion::write, WaveRegion::read);

    private static final org.slf4j.Logger LOGGER = LogUtils.getLogger();
    private static final ExecutorService COAST_BUILDER = Executors.newSingleThreadExecutor(task -> {
        Thread thread = new Thread(task, "AeroCore coastal waves");
        thread.setDaemon(true);
        thread.setPriority(Thread.MIN_PRIORITY);
        return thread;
    });
    private record CoastKey(Survey survey, WaveField waves, CoastalWaves.WindSea wind) {}
    /** Coastal fields by survey and sea, so region copies (a new wind, a resync to the client) share one build. */
    private static final Map<CoastKey, CompletableFuture<CoastalWaves>> COASTS = Collections.synchronizedMap(new LinkedHashMap<>(16, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<CoastKey, CompletableFuture<CoastalWaves>> eldest) { return size() > 32; }
    });

    private final double centerX, centerZ, radiusM, stillWaterY;
    private final WaveField waves;
    private final CompiledWaves compiled;
    private final Sea sea;
    private final Survey survey;
    private final Wind wind;
    private final CompletableFuture<CoastalWaves> coast;
    private final dev.mcmod.aerocore.api.WindGusts gusts;
    private final int hash;
    /** Height of a wind sea now relative to as synthesised (0 to 1, SeaState); 1 for other seas. Not part of equality. */
    private volatile double scale = 1;
    /** Made by NaturalSeas around a player (not saved, replaced as the player moves); server side only. */
    private volatile boolean natural;
    /** The waves at the present scale. */
    private volatile CompiledWaves current;

    public WaveRegion(double centerX, double centerZ, double radiusM, double stillWaterY, WaveField waves) {
        this(centerX, centerZ, radiusM, stillWaterY, waves, null);
    }
    public WaveRegion(double centerX, double centerZ, double radiusM, double stillWaterY, WaveField waves, Sea sea) {
        this(centerX, centerZ, radiusM, stillWaterY, waves, sea, null, Wind.CALM);
    }
    public WaveRegion(double centerX, double centerZ, double radiusM, double stillWaterY, WaveField waves, Sea sea, Survey survey, Wind wind) {
        if (!Double.isFinite(centerX) || !Double.isFinite(centerZ) || !Double.isFinite(stillWaterY)) {
            throw new IllegalArgumentException("Wave region position must be finite");
        }
        if (!Double.isFinite(radiusM) || radiusM < MIN_RADIUS || radiusM > MAX_RADIUS) {
            throw new IllegalArgumentException("Wave region radius must be in [4, 128] m");
        }
        Objects.requireNonNull(waves, "waves");
        if (waves.components().size() > MAX_COMPONENTS) { throw new IllegalArgumentException("At most 64 wave components"); }
        this.centerX = centerX; this.centerZ = centerZ; this.radiusM = radiusM; this.stillWaterY = stillWaterY;
        this.waves = waves; this.compiled = new CompiledWaves(waves); this.current = compiled; this.sea = sea; this.survey = survey;
        this.wind = Objects.requireNonNull(wind, "wind");
        CoastalWaves.WindSea windSea = sea == null ? null : sea.coastal();
        this.coast = survey == null || waves.isEmpty() ? null : COASTS.computeIfAbsent(new CoastKey(survey, waves, windSea), key -> CompletableFuture.supplyAsync(() -> {
            double deep = waves.components().getFirst().depthM();
            try {
                return CoastalWaves.build(survey.x0(), survey.z0(), survey.spacing(), survey.nx(), survey.nz(), survey.depths(Double.isFinite(deep) ? deep : 1e4), waves, windSea);
            } catch (RuntimeException failed) {
                LOGGER.warn("Coastal wave field could not be built; the region keeps open-sea waves", failed);
                throw failed;
            }
        }, COAST_BUILDER));
        this.hash = Objects.hash(centerX, centerZ, radiusM, stillWaterY, waves, sea, survey, wind);
        // The same gust seed on every side and for every copy of this region.
        this.gusts = new dev.mcmod.aerocore.api.WindGusts(Objects.hash(centerX, centerZ, stillWaterY));
    }

    public double centerX() { return centerX; }
    public double centerZ() { return centerZ; }
    public double radiusM() { return radiusM; }
    public double stillWaterY() { return stillWaterY; }
    public WaveField waves() { return waves; }
    /** The waves now (a wind sea at its present scale). */
    public CompiledWaves compiled() { return current; }
    /** Height of the sea now relative to as synthesised (SeaState). */
    public double scale() { return scale; }
    /** Sets the sea's present height factor (server: SeaState; client: the synced value). */
    public void setScale(double next) {
        double s = Math.clamp(next, 0, 1);
        if (s == scale) { return; }
        scale = s;
        current = s == 1 ? compiled : compiled.scaled(s);
        drawn.clear();
    }
    /** The spectrum this sea was synthesised from, or null for a single wave or a calm sea. */
    public Sea sea() { return sea; }
    /** The depth survey, or null when the region has none (open-sea waves everywhere). */
    public Survey survey() { return survey; }
    public Wind wind() { return wind; }
    /** The coastal wave field once built, else null (open sea). */
    public CoastalWaves coast() {
        CompletableFuture<CoastalWaves> c = coast;
        return c != null && c.isDone() && !c.isCompletedExceptionally() ? c.getNow(null) : null;
    }
    /** Waits for the coastal field (tests; it takes milliseconds to about a second), null without a survey. */
    CoastalWaves awaitCoast() { return coast == null ? null : coast.join(); }
    /** True while a coastal field is still being built. */
    public boolean coastPending() { return coast != null && !coast.isDone(); }

    public WaveRegion withWind(Wind next) { return carry(new WaveRegion(centerX, centerZ, radiusM, stillWaterY, waves, sea, survey, next)); }
    public WaveRegion withSurvey(Survey next) { return carry(new WaveRegion(centerX, centerZ, radiusM, stillWaterY, waves, sea, next, wind)); }
    private WaveRegion carry(WaveRegion next) { next.setScale(scale); next.natural = natural; return next; }

    /** Whether NaturalSeas made this region (server side). */
    public boolean natural() { return natural; }
    /** Marks this region as made by NaturalSeas; returns it. */
    public WaveRegion markNatural() { natural = true; return this; }

    private final java.util.Map<Double, CompiledWaves> drawn = new java.util.concurrent.ConcurrentHashMap<>();

    /** The waves a surface mesh of this vertex spacing (m) can draw: none shorter than four spacings (no aliasing). */
    public CompiledWaves drawnWaves(double spacing) { return drawn.computeIfAbsent(spacing, s -> current.withoutShorterThan(4 * s)); }

    /** Width of the outer amplitude ramp: a quarter of the radius, kept between 2 and 8 m. */
    public double taperM() { return Math.clamp(radiusM / 4, 2, 8); }

    /** Amplitude factor at a horizontal position: 1 in the core, smoothstep to 0 at the rim, 0 outside. */
    public double weight(double x, double z) {
        double r = Math.hypot(x - centerX, z - centerZ);
        if (r >= radiusM) { return 0; }
        double inner = radiusM - taperM();
        if (r <= inner) { return 1; }
        double s = (radiusM - r) / taperM();
        return s * s * (3 - 2 * s);
    }

    public boolean contains(double x, double z) { return Math.hypot(x - centerX, z - centerZ) < radiusM; }
    public boolean overlaps(WaveRegion other) { return Math.hypot(centerX - other.centerX, centerZ - other.centerZ) < radiusM + other.radiusM; }
    /** True for a free surface at the region's still level; other water bodies inside the circle stay calm. */
    public boolean ownsSurface(double surfaceY) { return Math.abs(surfaceY - stillWaterY) <= LEVEL_TOLERANCE; }

    /**
     * The sea as a body at (x, z) samples it: the taper and the coast taken at that point and held over a small body
     * (CompiledWaves.at), with the same waves as LinearWaves for frequency-domain loads.
     */
    public CompiledWaves localWaves(double x, double z) { return current.at(coast(), x, z, weight(x, z)); }
    /** The local waves as a WaveField (see localWaves). */
    public WaveField fieldAt(double x, double z) { return localWaves(x, z).field(); }

    /** How far the surface can move from the still level, m: the open-sea bound, scaled by the coast's largest gain. */
    public double reach() {
        CoastalWaves c = coast();
        double gain = c == null ? 1 : Math.max(1, c.maxGain());
        return current.maxAmplitude() * gain * gain;
    }

    private static final ThreadLocal<double[]> TERMS = ThreadLocal.withInitial(() -> new double[6]);

    /**
     * Instantaneous tapered elevation above the still level, to second order (CompiledWaves, over the coast when there
     * is one): the taper w scales the linear surface by w and the second-order part by w^2, as it does the amplitudes.
     */
    public double elevation(double x, double z, double t) {
        double w = weight(x, z);
        if (w == 0 || current.isEmpty()) { return 0; }
        double[] terms = TERMS.get();
        current.surfaceTerms(x, z, t, coast(), terms);
        return w * terms[0] + w * w * terms[3];
    }

    /** Tapered second-order elevation and its slopes (the taper's own gradient neglected) into out[0..2]. */
    public void surface(double x, double z, double t, double[] out) {
        double w = weight(x, z);
        out[0] = 0; out[1] = 0; out[2] = 0;
        if (w == 0 || current.isEmpty()) { return; }
        double[] terms = TERMS.get();
        current.surfaceTerms(x, z, t, coast(), terms);
        out[0] = w * terms[0] + w * w * terms[3]; out[1] = w * terms[1] + w * w * terms[4]; out[2] = w * terms[2] + w * w * terms[5];
    }

    /** Orbital velocity at a point (local waves), m/s. */
    public Vec3d orbitalVelocity(double x, double y, double z, double t) {
        if (current.isEmpty() || weight(x, z) == 0) { return Vec3d.ZERO; }
        return localWaves(x, z).velocity(new Vec3d(x, y, z), stillWaterY, t);
    }

    /** Wind-driven current at a depth below the surface (SeaWind.drift of the mean wind), tapered like the waves, m/s. */
    public Vec3d windDrift(net.minecraft.world.level.Level level, double x, double z, double depthM) {
        if (wind.calm()) { return Vec3d.ZERO; }
        double w = weight(x, z), u = wind.mean(level) * shelter(x, z);
        if (w == 0 || u == 0) { return Vec3d.ZERO; }
        double speed = w * SeaWind.drift(u, depthM);
        return new Vec3d(speed * wind.directionX(), 0, speed * wind.directionZ());
    }

    /**
     * The 10 m mean wind now (weather included), tapered like the waves and sheltered by terrain, as a horizontal vector,
     * m/s; a region without its own wind has the level's (WorldWind) 10 m above its still level.
     */
    public Vec3d meanWind(net.minecraft.world.level.Level level, double x, double z) {
        if (wind.calm()) { return WorldWind.mean(level, x, stillWaterY + 10, z); }
        double speed = wind.mean(level) * weight(x, z) * shelter(x, z);
        return new Vec3d(speed * wind.directionX(), 0, speed * wind.directionZ());
    }

    private volatile float[] shelterGrid;

    /**
     * The wind's share at (x, z) behind the surveyed ground and trees (SeaWind.shelter): along the line upwind, the
     * lowest share any barrier gives at its distance in its own heights, the solid ground with the fence curve and a
     * canopy rising above it with the tree-belt curve; 1 without a survey with barrier heights. Bilinear on the survey.
     */
    public double shelter(double x, double z) {
        if (survey == null || !survey.barriers() || wind.calm()) { return 1; }
        float[] grid = shelterGrid;
        if (grid == null) { grid = shelterGrid = shelterGrid(); }
        double gx = (x - survey.x0()) / survey.spacing(), gz = (z - survey.z0()) / survey.spacing();
        int nx = survey.nx(), nz = survey.nz();
        if (!(gx >= 0) || gx > nx - 1 || !(gz >= 0) || gz > nz - 1) { return 1; }
        int i = Math.min((int) gx, nx - 2), j = Math.min((int) gz, nz - 2);
        double fx = gx - i, fz = gz - j;
        int p = i * nz + j;
        return (grid[p] * (1 - fz) + grid[p + 1] * fz) * (1 - fx) + (grid[p + nz] * (1 - fz) + grid[p + nz + 1] * fz) * fx;
    }

    private float[] shelterGrid() {
        int nx = survey.nx(), nz = survey.nz();
        double ex = wind.directionX(), ez = wind.directionZ(), spacing = survey.spacing();
        float[] out = new float[nx * nz];
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            double share = 1;
            for (double d = spacing; ; d += 0.5 * spacing) {
                int a = (int) Math.round(i - d / spacing * ex), b = (int) Math.round(j - d / spacing * ez);
                if (a < 0 || a >= nx || b < 0 || b >= nz) { break; }
                int q = a * nz + b;
                double ground = survey.groundDm(q) / 10.0, canopy = survey.canopyDm(q) / 10.0;
                if (ground > 0.5) { share = Math.min(share, dev.mcmod.aerocore.api.SeaWind.shelter(d / ground, false)); }
                if (canopy > ground + 0.5) { share = Math.min(share, dev.mcmod.aerocore.api.SeaWind.shelter(d / canopy, true)); }
            }
            out[i * nz + j] = (float) share;
        }
        return out;
    }

    /** The gust at (x, z) at time t (WindGusts, frozen turbulence drifting downwind), tapered like the waves, m/s. */
    public Vec3d gust(net.minecraft.world.level.Level level, double x, double z, double t) {
        if (wind.calm()) { return WorldWind.gust(level, x, stillWaterY + 10, z, t); }
        double w = weight(x, z) * shelter(x, z), u = wind.mean(level);
        if (w == 0 || u == 0) { return Vec3d.ZERO; }
        double ex = wind.directionX(), ez = wind.directionZ();
        double[] g = new double[2];
        gusts.sample(u, t, (x - centerX) * ex + (z - centerZ) * ez, g);
        return new Vec3d(w * (g[0] * ex - g[1] * ez), 0, w * (g[0] * ez + g[1] * ex));
    }

    /** Horizontal wind at a point: the mean wind's log profile at its height above the still level plus the gust, m/s. */
    public Vec3d windAt(net.minecraft.world.level.Level level, double x, double y, double z, double t) {
        if (wind.calm()) { return WorldWind.at(level, x, y, z, t); }
        Vec3d mean = meanWind(level, x, z);
        double speed = mean.length();
        if (speed == 0) { return Vec3d.ZERO; }
        return mean.scale(SeaWind.speedAt(speed, y - stillWaterY) / speed).add(gust(level, x, z, t));
    }

    /** Fraction of wave energy breaking on the coast at (x, z), 0 to 1 (0 without a coast). */
    public double breaking(double x, double z) {
        CoastalWaves c = coast();
        return c == null ? 0 : c.breaking(x, z);
    }

    @Override public boolean equals(Object o) {
        return o == this || o instanceof WaveRegion r && r.hash == hash && r.centerX == centerX && r.centerZ == centerZ && r.radiusM == radiusM
                && r.stillWaterY == stillWaterY && r.waves.equals(waves) && Objects.equals(r.sea, sea) && Objects.equals(r.survey, survey) && r.wind.equals(wind);
    }
    @Override public int hashCode() { return hash; }
    @Override public String toString() { return "WaveRegion[" + centerX + ", " + centerZ + ", r " + radiusM + ", y " + stillWaterY + ", " + waves.components().size() + " components]"; }

    private static void write(FriendlyByteBuf buffer, WaveRegion region) {
        buffer.writeDouble(region.centerX).writeDouble(region.centerZ).writeDouble(region.radiusM).writeDouble(region.stillWaterY);
        buffer.writeVarInt(region.waves.components().size());
        for (LinearWave wave : region.waves.components()) {
            buffer.writeDouble(wave.amplitudeM()).writeDouble(wave.wavelengthM()).writeDouble(wave.directionRadians())
                    .writeDouble(wave.depthM()).writeDouble(wave.phaseRadians());
        }
        buffer.writeBoolean(region.sea != null);
        if (region.sea != null) {
            buffer.writeDouble(region.sea.significantHeightM()).writeDouble(region.sea.peakPeriodS()).writeDouble(region.sea.gamma())
                    .writeDouble(region.sea.directionRadians()).writeDouble(region.sea.spreading()).writeDouble(region.sea.windMps()).writeDouble(region.sea.fetchBeyondM())
                    .writeDouble(region.sea.fetchM());
        }
        buffer.writeDouble(region.wind.speedMps()).writeDouble(region.wind.directionRadians()).writeDouble(region.wind.rainMps()).writeDouble(region.wind.thunderMps());
        buffer.writeDouble(region.scale);
        Survey s = region.survey;
        buffer.writeBoolean(s != null);
        if (s != null) {
            buffer.writeDouble(s.x0()).writeDouble(s.z0()).writeDouble(s.spacing()).writeVarInt(s.nx()).writeVarInt(s.nz());
            for (int i = 0; i < s.nx() * s.nz(); i++) { buffer.writeShort(s.depthDm(i)); }
            buffer.writeBoolean(s.barriers());
            if (s.barriers()) { for (int i = 0; i < s.nx() * s.nz(); i++) { buffer.writeShort(s.groundDm(i)).writeShort(s.canopyDm(i)); } }
        }
    }

    private static WaveRegion read(FriendlyByteBuf buffer) {
        double x = buffer.readDouble(), z = buffer.readDouble(), radius = buffer.readDouble(), still = buffer.readDouble();
        int count = buffer.readVarInt();
        if (count < 0 || count > MAX_COMPONENTS) { throw new IllegalArgumentException("Too many wave components"); }
        List<LinearWave> waves = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            waves.add(new LinearWave(buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble()));
        }
        Sea sea = buffer.readBoolean() ? new Sea(buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble(),
                buffer.readDouble(), buffer.readDouble(), buffer.readDouble()) : null;
        Wind wind = new Wind(buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble());
        double scale = buffer.readDouble();
        Survey survey = null;
        if (buffer.readBoolean()) {
            double x0 = buffer.readDouble(), z0 = buffer.readDouble(), spacing = buffer.readDouble();
            int nx = buffer.readVarInt(), nz = buffer.readVarInt();
            if (nx < 2 || nz < 2 || nx > 512 || nz > 512) { throw new IllegalArgumentException("Invalid depth survey"); }
            short[] depth = new short[nx * nz];
            for (int i = 0; i < depth.length; i++) { depth[i] = buffer.readShort(); }
            short[] ground = null, canopy = null;
            if (buffer.readBoolean()) {
                ground = new short[depth.length]; canopy = new short[depth.length];
                for (int i = 0; i < depth.length; i++) { ground[i] = buffer.readShort(); canopy[i] = buffer.readShort(); }
            }
            survey = new Survey(x0, z0, spacing, nx, nz, depth, ground, canopy);
        }
        WaveRegion region = new WaveRegion(x, z, radius, still, new WaveField(waves), sea, survey, wind);
        region.setScale(scale);
        return region;
    }
}
