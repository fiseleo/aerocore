package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.api.BowSpray;
import dev.mcmod.aerocore.api.CoastalWaves;
import dev.mcmod.aerocore.api.CompiledWaves;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.Whitecaps;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.probe.SeaSurface;
import dev.mcmod.aerocore.neoforge.probe.Wakes;
import dev.mcmod.aerocore.neoforge.probe.WaveRegion;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.util.Mth;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * Foam on the sea surface, a client-side visual field on a 1 m grid in 16 m tiles within FOAM_RANGE of the camera,
 * computed only on the columns WaveSurfaceRenderer draws as sea surface and on half of the tiles each tick (a
 * rendering budget; foam lasts seconds, a breaking crest a fraction of one): the ambient sea foams where it breaks (Whitecaps: downward surface
 * acceleration past the threshold for its whitecap coverage, set by the region's wind where it blows, sheltered by the
 * surveyed terrain, or else the wind that raises its Hs), the surf
 * zone foams on its crests (below), boats add foam where their bow wave breaks (BowSpray,
 * crests of at least SprayRenderer's 3 cm; the spilling band, about a crest height wide, swept along the track, so
 * a grid disc of radius r gets the share crest / r, not all of it) and in the aerated water behind a dry transom
 * (Wakes.Design.whiteWaterWidth, fully white),
 * and all foam decays with Whitecaps.FOAM_DECAY_S. Foam stays where it formed (its drift is not modelled).
 * <p>
 * Surf: where the coast's depth limit removes a fraction b of the wave energy (CoastalWaves.breaking), a crest foams
 * when the local surface stands above the level a Gaussian sea of the local variance exceeds with probability b, so
 * about that share of the surface whitens at each moment (at most half). This is a presentation rule tied to the
 * modelled dissipation, not a model of breaker types or bores.
 * <p>
 * Beyond FOAM_RANGE the renderer draws the foam's mean instead (meanFoam): the whitecap coverage and the surf share,
 * which is what a pixel spanning many crests shows.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class SeaFoam {
    public static final int TILE = 16;
    public static final double SPACING = 1.0;
    public static final int POINTS = (int) Math.round(TILE / SPACING) + 1;
    private static final double FOAM_RANGE = 40, MIN_CREST_M = 0.03;
    /** Breaking fractions below MIN_SURF draw no surf foam; above MAX_SURF the foamed share stays at one half (presentation limits). */
    private static final double MIN_SURF = 0.01, MAX_SURF = 0.5;
    private static final int FORGET_TICKS = 40;

    private record Key(WaveRegion region, int x, int z) {}
    private static final class Patch {
        final float[] foam = new float[POINTS * POINTS];
        long tick = Long.MIN_VALUE, used, built = Long.MIN_VALUE;
        /**
         * Per point, rebuilt when the coast or the surface mask changes and every REBUILD_TICKS: taper weight (0 off the
         * sea surface), breaking threshold over the acceleration deviation (infinite where nothing breaks), surf level,
         * envelope phasors.
         */
        float[] weight, threshold, surf, envRe, envIm;
        CoastalWaves coast;
        boolean[] mask;
        /** Wave components the envelopes were built for. */
        int components;
    }
    /** How often a patch's per-point data is rebuilt, ticks (the renderer refreshes its masks every 10). */
    private static final int REBUILD_TICKS = 20;
    private static final Map<Key, Patch> PATCHES = new HashMap<>();
    /**
     * Patch rebuilds per tick: a region that moves with its player hands every patch to a new region at once, and
     * rebuilding them all in one tick stalled a frame (patches waiting keep their previous data).
     */
    private static final int BUILDS_PER_TICK = 4;
    private static int builds;
    private record Migration(List<WaveRegion> before, List<WaveRegion> after) {}
    private static final java.util.Queue<Migration> MIGRATIONS = new java.util.concurrent.ConcurrentLinkedQueue<>();

    /**
     * The region list changed (ClientWaveRegions): patches of a region that went pass, with their foam, to a new region
     * of the same still level covering their tile, so a natural sea moving with its player keeps its foam.
     */
    static void migrate(List<WaveRegion> before, List<WaveRegion> after) { MIGRATIONS.add(new Migration(List.copyOf(before), List.copyOf(after))); }

    private static void applyMigration(Migration m) {
        Map<Key, Patch> next = new HashMap<>();
        for (Map.Entry<Key, Patch> e : PATCHES.entrySet()) {
            Key key = e.getKey();
            int kept = m.after().indexOf(key.region());
            if (kept >= 0) { next.put(new Key(m.after().get(kept), key.x(), key.z()), e.getValue()); continue; }
            for (WaveRegion n : m.after()) {
                if (m.before().contains(n) || Math.abs(n.stillWaterY() - key.region().stillWaterY()) > WaveRegion.LEVEL_TOLERANCE
                        || Math.hypot(Math.max(0, Math.max(key.x() - n.centerX(), n.centerX() - key.x() - TILE)),
                                Math.max(0, Math.max(key.z() - n.centerZ(), n.centerZ() - key.z() - TILE))) >= n.radiusM()) { continue; }
                next.putIfAbsent(new Key(n, key.x(), key.z()), e.getValue());
                break;
            }
        }
        PATCHES.clear();
        PATCHES.putAll(next);
    }
    /** Per region: the surface acceleration deviation and Hs of the sea as synthesised. */
    private static final Map<WaveRegion, double[]> THRESHOLDS = new java.util.concurrent.ConcurrentHashMap<>();
    private static double[] cos = new double[0], sin = new double[0];

    private SeaFoam() {}

    /** The foam grid of a tile (index i * POINTS + k for x = tx + i SPACING, z = tz + k SPACING), or null. */
    public static float[] patch(WaveRegion region, int tx, int tz) {
        Patch p = PATCHES.get(new Key(region, tx, tz));
        return p == null ? null : p.foam;
    }

    /** The acceleration deviation (m/s^2) and Hs (m) of a region's sea as synthesised; the present scale multiplies both. */
    private static double[] statistics(WaveRegion region) {
        return THRESHOLDS.computeIfAbsent(region, r -> {
            if (r.waves().isEmpty()) { return new double[]{0, 0}; }
            double m0 = 0;
            for (LinearWave w : r.waves().components()) { m0 += 0.5 * w.amplitudeM() * w.amplitudeM(); }
            return new double[]{new CompiledWaves(r.waves()).accelerationDeviation(), r.sea() != null ? r.sea().significantHeightM() : 4 * Math.sqrt(m0)};
        });
    }

    /**
     * The 10 m wind whose whitecaps a point of the region gets, m/s: the region's mean wind (weather included) tapered
     * and sheltered by terrain like the waves; with no wind of its own, the level's wind there (WorldWind).
     */
    static double whitecapWind(WaveRegion region, ClientLevel level, double x, double z) {
        if (!region.wind().calm()) { return region.wind().mean(level) * region.weight(x, z) * region.shelter(x, z); }
        return region.meanWind(level, x, z).length();
    }

    /**
     * Mean foam (0 to 1) at a point under a 10 m wind there (as whitecapWind gives), for drawing beyond FOAM_RANGE: the
     * whitecap coverage, or the surf share where larger.
     */
    static double meanFoam(WaveRegion region, double x, double z, double whitecapWind) {
        if (region.waves().isEmpty() || region.weight(x, z) <= 0) { return 0; }
        double surf = 0;
        CoastalWaves coast = region.coast();
        if (coast != null) {
            double b = Math.min(coast.breaking(x, z), MAX_SURF);
            if (b > MIN_SURF) { surf = b; }
        }
        return Math.max(Whitecaps.coverage(Math.max(whitecapWind, 0)), surf);
    }

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        if (level == null) { PATCHES.clear(); THRESHOLDS.clear(); MIGRATIONS.clear(); return; }
        for (Migration m; (m = MIGRATIONS.poll()) != null; ) { applyMigration(m); }
        if (minecraft.isPaused()) { return; }
        builds = BUILDS_PER_TICK;
        List<WaveRegion> regions = ClientWaveRegions.current();
        if (regions.isEmpty()) { PATCHES.clear(); THRESHOLDS.clear(); return; }
        THRESHOLDS.keySet().retainAll(regions);
        Vec3 eye = minecraft.gameRenderer.getMainCamera().getPosition();
        long tick = level.getGameTime();
        double t = SeaSurface.time(level);
        for (WaveRegion region : regions) {
            double r = region.radiusM();
            int x0 = Math.floorDiv(Mth.floor(Math.max(region.centerX() - r, eye.x - FOAM_RANGE)), TILE) * TILE;
            int z0 = Math.floorDiv(Mth.floor(Math.max(region.centerZ() - r, eye.z - FOAM_RANGE)), TILE) * TILE;
            int x1 = Mth.floor(Math.min(region.centerX() + r, eye.x + FOAM_RANGE)), z1 = Mth.floor(Math.min(region.centerZ() + r, eye.z + FOAM_RANGE));
            for (int tx = x0; tx <= x1; tx += TILE) for (int tz = z0; tz <= z1; tz += TILE) {
                Patch p = PATCHES.computeIfAbsent(new Key(region, tx, tz), k -> new Patch());
                update(p, region, level, tx, tz, t, tick);
            }
        }
        for (Wakes.Moving m : Wakes.moving(level)) { boat(regions, m, tick); }
        for (Iterator<Patch> it = PATCHES.values().iterator(); it.hasNext(); ) { if (tick - it.next().used > FORGET_TICKS) { it.remove(); } }
    }

    /** Decay since the last update, then the ambient breaking (on half of the tiles each tick). */
    private static void update(Patch p, WaveRegion region, ClientLevel level, int tx, int tz, double t, long tick) {
        p.used = tick;
        if (p.tick == tick || (p.tick != Long.MIN_VALUE && ((Math.floorDiv(tx, TILE) + Math.floorDiv(tz, TILE) + tick) & 1) != 0)) { return; }
        double dt = p.tick == Long.MIN_VALUE ? 0 : (tick - p.tick) * MinecraftUnits.TICK_SECONDS;
        p.tick = tick;
        float decay = (float) Math.exp(-dt / Whitecaps.FOAM_DECAY_S);
        for (int i = 0; i < p.foam.length; i++) { p.foam[i] *= decay; }
        double deviation = statistics(region)[0] * region.scale();
        CompiledWaves waves = region.compiled();
        boolean[] mask = WaveSurfaceRenderer.surfaceMask(region, tx, tz);
        // Not drawn (out of view or no sea surface in it yet): nothing to foam.
        if (waves.isEmpty() || mask == null) { return; }
        CoastalWaves coast = region.coast();
        if (p.weight == null || p.coast != coast || p.mask != mask || p.components != waves.size() || tick - p.built >= REBUILD_TICKS) {
            if (builds > 0) { builds--; build(p, region, level, waves, coast, mask, tx, tz, tick); }
            // Waiting for a later tick: without data that fits these waves, no breaking yet.
            else if (p.weight == null || p.components != waves.size()) { return; }
        }
        if (!(deviation > 0) && p.surf == null) { return; }
        int n = waves.size(), count = POINTS * POINTS * n;
        if (cos.length < count) { cos = new double[count]; sin = new double[count]; }
        waves.gridPhases(tx, tz, SPACING, SPACING, POINTS, POINTS, t, cos, sin);
        for (int q = 0; q < POINTS * POINTS; q++) {
            double w = p.weight[q];
            if (w == 0) { continue; }
            int offset = q * n;
            if (p.envRe != null) {
                for (int c = 0; c < n; c++) {
                    double mr = p.envRe[offset + c], mi = p.envIm[offset + c], re = cos[offset + c], im = sin[offset + c];
                    cos[offset + c] = mr * re - mi * im; sin[offset + c] = mr * im + mi * re;
                }
            }
            double threshold = deviation * p.threshold[q];
            if (deviation > 0 && Double.isFinite(threshold) && w * waves.verticalAcceleration(cos, offset) < -threshold) { p.foam[q] = 1; continue; }
            if (p.surf != null && !Float.isNaN(p.surf[q])) {
                double sigma = w * Math.sqrt(waves.variance(cos, sin, offset));
                if (sigma > 0 && w * waves.elevation(cos, offset) > sigma * p.surf[q]) { p.foam[q] = 1; }
            }
        }
    }

    /** The per-point data of a patch: taper on the drawn sea surface, breaking thresholds, the coast's envelopes and surf levels. */
    private static void build(Patch p, WaveRegion region, ClientLevel level, CompiledWaves waves, CoastalWaves coast, boolean[] mask, int tx, int tz, long tick) {
        int n = waves.size(), points = POINTS * POINTS;
        p.components = n;
        boolean coastal = coast != null && coast.components() == n && coast.changedIn(tx, tz, tx + TILE, tz + TILE);
        p.weight = new float[points];
        p.threshold = new float[points];
        p.envRe = coastal ? new float[points * n] : null; p.envIm = coastal ? new float[points * n] : null;
        p.surf = null;
        double[] re = new double[n], im = new double[n];
        for (int i = 0; i < POINTS; i++) for (int k = 0; k < POINTS; k++) {
            int q = i * POINTS + k;
            double x = tx + i * SPACING, z = tz + k * SPACING;
            if (!mask[Math.min(i, TILE - 1) * TILE + Math.min(k, TILE - 1)]) { continue; }
            p.weight[q] = (float) region.weight(x, z);
            // The threshold for a unit deviation: Whitecaps.thresholdAtWind is proportional to the deviation.
            p.threshold[q] = (float) Whitecaps.thresholdAtWind(1, whitecapWind(region, level, x, z));
            if (!coastal) { continue; }
            coast.envelopes(x, z, re, im, null, null, null, null);
            for (int c = 0; c < n; c++) { p.envRe[q * n + c] = (float) re[c]; p.envIm[q * n + c] = (float) im[c]; }
            double b = Math.min(coast.breaking(x, z), MAX_SURF);
            if (b > MIN_SURF) {
                if (p.surf == null) { p.surf = new float[points]; java.util.Arrays.fill(p.surf, Float.NaN); }
                p.surf[q] = (float) Whitecaps.inverseNormal(1 - b);
            }
        }
        p.coast = coast; p.mask = mask; p.built = tick;
    }

    /** Foam from one boat: at the shoulders of a breaking bow wave, and across a dry transom's wake. */
    private static void boat(List<WaveRegion> regions, Wakes.Moving m, long tick) {
        Wakes.Bow bow = m.design().bow(m.speed());
        BowSpray spray = bow.spray();
        double fx = m.dirX(), fz = m.dirZ(), lx = -fz, lz = fx;
        if (spray.breaking() && spray.crestHeightM() >= MIN_CREST_M) {
            // The crest stands about its own height out from the hull; foam a patch that size at each shoulder.
            double radius = Math.max(SPACING, 2 * spray.crestHeightM()), share = Math.min(1, spray.crestHeightM() / radius);
            for (int side = -1; side <= 1; side += 2) {
                double off = bow.halfWidthM() + spray.crestHeightM();
                stamp(regions, m, tick, m.originX() + fx * (bow.forwardM() - radius) + lx * side * off,
                        m.originZ() + fz * (bow.forwardM() - radius) + lz * side * off, radius, share);
            }
        }
        double width = m.design().whiteWaterWidth(m.speed());
        if (width > 0) {
            // This tick's stretch of track behind the origin, at the hull's width.
            double travel = m.speed() * MinecraftUnits.TICK_SECONDS;
            for (double d = 0; d <= travel; d += SPACING) {
                stamp(regions, m, tick, m.originX() - fx * d, m.originZ() - fz * d, width / 2, 1);
            }
        }
    }

    /** Foam covering share (0 to 1) of the grid points within radius of (x, z); never less than they have. */
    private static void stamp(List<WaveRegion> regions, Wakes.Moving m, long tick, double x, double z, double radius, double share) {
        for (double dx = -radius; dx <= radius; dx += SPACING) for (double dz = -radius; dz <= radius; dz += SPACING) {
            if (dx * dx + dz * dz > radius * radius) { continue; }
            double px = x + dx, pz = z + dz;
            WaveRegion region = SeaSurface.regionAt(regions, px, pz);
            if (region == null || !region.ownsSurface(m.stillY())) { continue; }
            int tx = Math.floorDiv(Mth.floor(px), TILE) * TILE, tz = Math.floorDiv(Mth.floor(pz), TILE) * TILE;
            Patch p = PATCHES.get(new Key(region, tx, tz));
            if (p == null) { continue; }
            int i = (int) Math.round((px - tx) / SPACING), k = (int) Math.round((pz - tz) / SPACING);
            if (i >= 0 && i < POINTS && k >= 0 && k < POINTS) { p.foam[i * POINTS + k] = Math.max(p.foam[i * POINTS + k], (float) share); }
        }
    }
}
