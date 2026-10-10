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
 * Foam on the sea surface, a client-side visual field on a 0.5 m grid in 16 m tiles within FOAM_RANGE of the camera
 * (a rendering budget), updated every tick: the ambient sea foams where it breaks (Whitecaps: downward surface
 * acceleration past the threshold for its whitecap coverage, set by the region's wind when it has one), the surf
 * zone foams on its crests (below), boats add foam where their bow wave breaks (BowSpray,
 * crests of at least SprayRenderer's 3 cm) and in the aerated water behind a dry transom (Wakes.Design.whiteWaterWidth),
 * and all foam decays with Whitecaps.FOAM_DECAY_S. Foam stays where it formed (its drift is not modelled).
 * <p>
 * Surf: where the coast's depth limit removes a fraction b of the wave energy (CoastalWaves.breaking), a crest foams
 * when the local surface stands above the level a Gaussian sea of the local variance exceeds with probability b, so
 * about that share of the surface whitens at each moment (at most half). This is a presentation rule tied to the
 * modelled dissipation, not a model of breaker types or bores.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class SeaFoam {
    public static final int TILE = 16;
    public static final double SPACING = 0.5;
    public static final int POINTS = (int) Math.round(TILE / SPACING) + 1;
    private static final double FOAM_RANGE = 40, MIN_CREST_M = 0.03;
    /** Breaking fractions below MIN_SURF draw no surf foam; above MAX_SURF the foamed share stays at one half (presentation limits). */
    private static final double MIN_SURF = 0.01, MAX_SURF = 0.5;
    private static final int FORGET_TICKS = 40;

    private record Key(WaveRegion region, int x, int z) {}
    private static final class Patch {
        final float[] foam = new float[POINTS * POINTS];
        long tick = Long.MIN_VALUE, used;
    }
    private static final Map<Key, Patch> PATCHES = new HashMap<>();
    private static final Map<WaveRegion, Double> THRESHOLDS = new HashMap<>();
    private static double[] cos = new double[0], sin = new double[0];

    private SeaFoam() {}

    /** The foam grid of a tile (index i * POINTS + k for x = tx + i SPACING, z = tz + k SPACING), or null. */
    public static float[] patch(WaveRegion region, int tx, int tz) {
        Patch p = PATCHES.get(new Key(region, tx, tz));
        return p == null ? null : p.foam;
    }

    /** Breaking acceleration threshold of a region's sea (Hs from its spectrum, 4 sqrt(m0)), m/s^2. */
    private static double threshold(WaveRegion region) {
        return THRESHOLDS.computeIfAbsent(region, r -> {
            CompiledWaves waves = r.compiled();
            if (waves.isEmpty()) { return Double.POSITIVE_INFINITY; }
            double m0 = 0;
            for (LinearWave w : r.waves().components()) { m0 += 0.5 * w.amplitudeM() * w.amplitudeM(); }
            if (!r.wind().calm()) { return Whitecaps.thresholdAtWind(waves.accelerationDeviation(), r.wind().speedMps()); }
            return Whitecaps.threshold(waves.accelerationDeviation(), r.sea() != null ? r.sea().significantHeightM() : 4 * Math.sqrt(m0));
        });
    }

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        if (level == null) { PATCHES.clear(); THRESHOLDS.clear(); return; }
        if (minecraft.isPaused()) { return; }
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
                update(p, region, tx, tz, t, tick);
            }
        }
        for (Wakes.Moving m : Wakes.moving(level)) { boat(regions, m, tick); }
        for (Iterator<Patch> it = PATCHES.values().iterator(); it.hasNext(); ) { if (tick - it.next().used > FORGET_TICKS) { it.remove(); } }
    }

    /** Decay since the last update, then the ambient breaking. */
    private static void update(Patch p, WaveRegion region, int tx, int tz, double t, long tick) {
        p.used = tick;
        if (p.tick == tick) { return; }
        double dt = p.tick == Long.MIN_VALUE ? 0 : (tick - p.tick) * MinecraftUnits.TICK_SECONDS;
        p.tick = tick;
        float decay = (float) Math.exp(-dt / Whitecaps.FOAM_DECAY_S);
        for (int i = 0; i < p.foam.length; i++) { p.foam[i] *= decay; }
        double threshold = threshold(region);
        CompiledWaves waves = region.compiled();
        CoastalWaves coast = region.coast();
        boolean coastal = coast != null && coast.changedIn(tx, tz, tx + TILE, tz + TILE);
        if (waves.isEmpty() || (!Double.isFinite(threshold) && !coastal)) { return; }
        int n = waves.size(), count = POINTS * POINTS * n;
        if (cos.length < count) { cos = new double[count]; sin = new double[count]; }
        waves.gridPhases(tx, tz, SPACING, SPACING, POINTS, POINTS, t, cos, sin);
        for (int i = 0; i < POINTS; i++) for (int k = 0; k < POINTS; k++) {
            int q = i * POINTS + k, offset = q * n;
            double x = tx + i * SPACING, z = tz + k * SPACING, w = region.weight(x, z);
            if (w == 0) { continue; }
            if (coastal) { waves.modulate(coast, x, z, cos, sin, offset); }
            if (Double.isFinite(threshold) && w * waves.verticalAcceleration(cos, offset) < -threshold) { p.foam[q] = 1; continue; }
            double b = coastal ? Math.min(coast.breaking(x, z), MAX_SURF) : 0;
            if (b > MIN_SURF) {
                double sigma = w * Math.sqrt(waves.variance(cos, sin, offset));
                if (sigma > 0 && w * waves.elevation(cos, offset) > sigma * Whitecaps.inverseNormal(1 - b)) { p.foam[q] = 1; }
            }
        }
    }

    /** Foam from one boat: at the shoulders of a breaking bow wave, and across a dry transom's wake. */
    private static void boat(List<WaveRegion> regions, Wakes.Moving m, long tick) {
        Wakes.Bow bow = m.design().bow(m.speed());
        BowSpray spray = bow.spray();
        double fx = m.dirX(), fz = m.dirZ(), lx = -fz, lz = fx;
        if (spray.breaking() && spray.crestHeightM() >= MIN_CREST_M) {
            // The crest stands about its own height out from the hull; foam a patch that size at each shoulder.
            double radius = Math.max(SPACING, 2 * spray.crestHeightM());
            for (int side = -1; side <= 1; side += 2) {
                double off = bow.halfWidthM() + spray.crestHeightM();
                stamp(regions, m, tick, m.originX() + fx * (bow.forwardM() - radius) + lx * side * off,
                        m.originZ() + fz * (bow.forwardM() - radius) + lz * side * off, radius);
            }
        }
        double width = m.design().whiteWaterWidth(m.speed());
        if (width > 0) {
            // This tick's stretch of track behind the origin, at the hull's width.
            double travel = m.speed() * MinecraftUnits.TICK_SECONDS;
            for (double d = 0; d <= travel; d += SPACING) {
                stamp(regions, m, tick, m.originX() - fx * d, m.originZ() - fz * d, width / 2);
            }
        }
    }

    private static void stamp(List<WaveRegion> regions, Wakes.Moving m, long tick, double x, double z, double radius) {
        for (double dx = -radius; dx <= radius; dx += SPACING) for (double dz = -radius; dz <= radius; dz += SPACING) {
            if (dx * dx + dz * dz > radius * radius) { continue; }
            double px = x + dx, pz = z + dz;
            WaveRegion region = SeaSurface.regionAt(regions, px, pz);
            if (region == null || !region.ownsSurface(m.stillY())) { continue; }
            int tx = Math.floorDiv(Mth.floor(px), TILE) * TILE, tz = Math.floorDiv(Mth.floor(pz), TILE) * TILE;
            Patch p = PATCHES.get(new Key(region, tx, tz));
            if (p == null) { continue; }
            int i = (int) Math.round((px - tx) / SPACING), k = (int) Math.round((pz - tz) / SPACING);
            if (i >= 0 && i < POINTS && k >= 0 && k < POINTS) { p.foam[i * POINTS + k] = 1; }
        }
    }
}
