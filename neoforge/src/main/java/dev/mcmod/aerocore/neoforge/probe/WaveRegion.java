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
    /** Spectrum parameters of a synthesised sea: Hs (m), Tp (s), gamma, mean direction (rad), cos^n exponent. */
    public record Sea(double significantHeightM, double peakPeriodS, double gamma, double directionRadians, double spreading) {}

    /** Wind over the region: speed at 10 m above the still level (m/s) and the direction it blows toward (rad, like wave headings). */
    public record Wind(double speedMps, double directionRadians) {
        public static final Wind CALM = new Wind(0, 0);
        public Wind {
            if (!(speedMps >= 0 && speedMps <= SeaWind.MAX_WIND) || !Double.isFinite(directionRadians)) {
                throw new IllegalArgumentException("Wind speed must be in [0, 30] m/s");
            }
        }
        public boolean calm() { return speedMps == 0; }
        /** The 10 m wind as a horizontal vector, m/s. */
        public Vec3d vector() { return new Vec3d(speedMps * Math.cos(directionRadians), 0, speedMps * Math.sin(directionRadians)); }
        /** Horizontal wind velocity at a height above the still level (log profile), m/s. */
        public Vec3d at(double heightM) {
            return calm() ? Vec3d.ZERO : new Vec3d(Math.cos(directionRadians), 0, Math.sin(directionRadians)).scale(SeaWind.speedAt(speedMps, heightM));
        }
    }

    /**
     * Depth of the bed below the still level on a square grid over the region, nodes (x0 + i spacing, z0 + j spacing)
     * at index i nz + j, in decimetres; LAND where the column at the still level is not water, DEEP where no bed was
     * found within the survey depth or the chunk was not loaded (open water of the sea's own depth).
     */
    public static final class Survey {
        public static final short LAND = -1, DEEP = Short.MAX_VALUE;
        public static final double SPACING = 2;
        /** Deepest bed searched for, m. */
        public static final int MAX_DEPTH = 64;
        private final double x0, z0, spacing;
        private final int nx, nz, hash;
        private final short[] depthDm;

        public Survey(double x0, double z0, double spacing, int nx, int nz, short[] depthDm) {
            if (!(spacing > 0) || nx < 2 || nz < 2 || nx > 512 || nz > 512 || depthDm.length != nx * nz || !Double.isFinite(x0) || !Double.isFinite(z0)) {
                throw new IllegalArgumentException("Invalid depth survey");
            }
            this.x0 = x0; this.z0 = z0; this.spacing = spacing; this.nx = nx; this.nz = nz; this.depthDm = depthDm.clone();
            hash = Objects.hash(x0, z0, spacing, nx, nz, Arrays.hashCode(depthDm));
        }

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
                    && Arrays.equals(s.depthDm, depthDm);
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
    private record CoastKey(Survey survey, WaveField waves) {}
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
    private final int hash;

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
        this.waves = waves; this.compiled = new CompiledWaves(waves); this.sea = sea; this.survey = survey;
        this.wind = Objects.requireNonNull(wind, "wind");
        this.coast = survey == null || waves.isEmpty() ? null : COASTS.computeIfAbsent(new CoastKey(survey, waves), key -> CompletableFuture.supplyAsync(() -> {
            double deep = waves.components().getFirst().depthM();
            try {
                return CoastalWaves.build(survey.x0(), survey.z0(), survey.spacing(), survey.nx(), survey.nz(), survey.depths(Double.isFinite(deep) ? deep : 1e4), waves);
            } catch (RuntimeException failed) {
                LOGGER.warn("Coastal wave field could not be built; the region keeps open-sea waves", failed);
                throw failed;
            }
        }, COAST_BUILDER));
        this.hash = Objects.hash(centerX, centerZ, radiusM, stillWaterY, waves, sea, survey, wind);
    }

    public double centerX() { return centerX; }
    public double centerZ() { return centerZ; }
    public double radiusM() { return radiusM; }
    public double stillWaterY() { return stillWaterY; }
    public WaveField waves() { return waves; }
    public CompiledWaves compiled() { return compiled; }
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

    public WaveRegion withWind(Wind next) { return new WaveRegion(centerX, centerZ, radiusM, stillWaterY, waves, sea, survey, next); }
    public WaveRegion withSurvey(Survey next) { return new WaveRegion(centerX, centerZ, radiusM, stillWaterY, waves, sea, next, wind); }

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
    public CompiledWaves localWaves(double x, double z) { return compiled.at(coast(), x, z, weight(x, z)); }
    /** The local waves as a WaveField (see localWaves). */
    public WaveField fieldAt(double x, double z) { return localWaves(x, z).field(); }

    /** How far the surface can move from the still level, m: the open-sea bound, scaled by the coast's largest gain. */
    public double reach() {
        CoastalWaves c = coast();
        double gain = c == null ? 1 : Math.max(1, c.maxGain());
        return compiled.maxAmplitude() * gain * gain;
    }

    private static final ThreadLocal<double[]> TERMS = ThreadLocal.withInitial(() -> new double[6]);

    /**
     * Instantaneous tapered elevation above the still level, to second order (CompiledWaves, over the coast when there
     * is one): the taper w scales the linear surface by w and the second-order part by w^2, as it does the amplitudes.
     */
    public double elevation(double x, double z, double t) {
        double w = weight(x, z);
        if (w == 0 || compiled.isEmpty()) { return 0; }
        double[] terms = TERMS.get();
        compiled.surfaceTerms(x, z, t, coast(), terms);
        return w * terms[0] + w * w * terms[3];
    }

    /** Tapered second-order elevation and its slopes (the taper's own gradient neglected) into out[0..2]. */
    public void surface(double x, double z, double t, double[] out) {
        double w = weight(x, z);
        out[0] = 0; out[1] = 0; out[2] = 0;
        if (w == 0 || compiled.isEmpty()) { return; }
        double[] terms = TERMS.get();
        compiled.surfaceTerms(x, z, t, coast(), terms);
        out[0] = w * terms[0] + w * w * terms[3]; out[1] = w * terms[1] + w * w * terms[4]; out[2] = w * terms[2] + w * w * terms[5];
    }

    /** Orbital velocity at a point (local waves), m/s. */
    public Vec3d orbitalVelocity(double x, double y, double z, double t) {
        if (compiled.isEmpty() || weight(x, z) == 0) { return Vec3d.ZERO; }
        return localWaves(x, z).velocity(new Vec3d(x, y, z), stillWaterY, t);
    }

    /** Wind-driven current at a depth below the surface (SeaWind.drift), tapered like the waves, m/s. */
    public Vec3d windDrift(double x, double z, double depthM) {
        if (wind.calm()) { return Vec3d.ZERO; }
        double w = weight(x, z);
        if (w == 0) { return Vec3d.ZERO; }
        double speed = w * SeaWind.drift(wind.speedMps(), depthM);
        return new Vec3d(speed * Math.cos(wind.directionRadians()), 0, speed * Math.sin(wind.directionRadians()));
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
                    .writeDouble(region.sea.directionRadians()).writeDouble(region.sea.spreading());
        }
        buffer.writeDouble(region.wind.speedMps()).writeDouble(region.wind.directionRadians());
        Survey s = region.survey;
        buffer.writeBoolean(s != null);
        if (s != null) {
            buffer.writeDouble(s.x0()).writeDouble(s.z0()).writeDouble(s.spacing()).writeVarInt(s.nx()).writeVarInt(s.nz());
            for (int i = 0; i < s.nx() * s.nz(); i++) { buffer.writeShort(s.depthDm(i)); }
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
        Sea sea = buffer.readBoolean() ? new Sea(buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble()) : null;
        Wind wind = new Wind(buffer.readDouble(), buffer.readDouble());
        Survey survey = null;
        if (buffer.readBoolean()) {
            double x0 = buffer.readDouble(), z0 = buffer.readDouble(), spacing = buffer.readDouble();
            int nx = buffer.readVarInt(), nz = buffer.readVarInt();
            if (nx < 2 || nz < 2 || nx > 512 || nz > 512) { throw new IllegalArgumentException("Invalid depth survey"); }
            short[] depth = new short[nx * nz];
            for (int i = 0; i < depth.length; i++) { depth[i] = buffer.readShort(); }
            survey = new Survey(x0, z0, spacing, nx, nz, depth);
        }
        return new WaveRegion(x, z, radius, still, new WaveField(waves), sea, survey, wind);
    }
}
