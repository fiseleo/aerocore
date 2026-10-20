package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.api.SurfaceWind;
import dev.mcmod.aerocore.api.Thermodynamics;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WindGusts;
import net.minecraft.resources.ResourceKey;
import net.minecraft.world.level.Level;

/**
 * The air at a point of the world (0.38), on either side: what an instrument or a weather report there would read,
 * the same air a vehicle at that point feels (PartVehicle: a sea region's own wind inside it, else the level's
 * {@link WorldWind}; the standard atmosphere at the height above sea level).
 * <ul>
 *   <li>Wind: mean, gust and vertical parts. A region with its own wind has its log profile above the still level
 *   and its gusts, and no vertical part.</li>
 *   <li>Air: ISA temperature and pressure at y - sea level (clamped to -500 to 20 000 m as the vehicles' air is), density
 *   of dry air.</li>
 *   <li>Visibility: a rule of the weather (the world has no aerosol or fog model): clear 20 km, rain 5 km, a
 *   thunderstorm 1.5 km, between them by how far the rain and the thunder have set in. Only lights and reports read
 *   it; nothing in the physics does.</li>
 * </ul>
 * Reads blocks (through the wind's terrain analysis): server thread or render thread.
 */
public final class WorldAir {
    private WorldAir() {}

    /** Visibility (meteorological optical range, m) in clear air, rain and a thunderstorm: a rule. */
    public static final double CLEAR_M = 20_000, RAIN_M = 5_000, THUNDER_M = 1_500;

    /** The air at a point: mean and gust wind (horizontal), vertical wind (m/s), altitude above sea level (m), temperature (K), pressure (Pa), density (kg/m^3), visibility (m). */
    public record Air(Vec3d meanWind, Vec3d gust, double verticalMps, double altitudeM, double temperatureK, double pressurePa,
                      double densityKgM3, double visibilityM) {
        /** The whole wind (mean, gust and vertical), m/s. */
        public Vec3d wind() { return meanWind.add(gust).add(new Vec3d(0, verticalMps, 0)); }
    }

    /** The air at a point now. */
    public static Air at(Level level, double x, double y, double z) {
        double t = SeaSurface.time(level);
        WaveRegion region = SeaSurface.regionAt(SeaSurface.persistentRegions(level), x, z);
        Vec3d mean, gust;
        double vertical;
        if (region != null && !region.wind().calm()) {
            Vec3d m10 = region.meanWind(level, x, z);
            double speed = m10.length();
            mean = speed == 0 ? Vec3d.ZERO : m10.scale(SeaWind.speedAt(speed, y - region.stillWaterY()) / speed);
            gust = speed == 0 ? Vec3d.ZERO : region.gust(level, x, z, t);
            vertical = 0;
        } else {
            mean = WorldWind.mean(level, x, y, z);
            gust = WorldWind.gust(level, x, y, z, t);
            vertical = WorldWind.vertical(level, x, y, z, t);
        }
        double altitude = y - level.getSeaLevel();
        Atmosphere.Sample isa = Atmosphere.atAltitude(Math.clamp(altitude, -500, 20_000), Vec3d.ZERO);
        double density = Thermodynamics.dryAir(isa.temperatureKelvin(), isa.pressurePa(), Vec3d.ZERO).fluid().densityKgPerM3();
        return new Air(mean, gust, vertical, altitude, isa.temperatureKelvin(), isa.pressurePa(), density, visibilityM(level));
    }

    /** The step a peak gust is sampled at (s). */
    private static final double GUST_STEP_S = 0.5;

    /**
     * The highest horizontal wind speed at a point (m/s), averaged over averageS, in the last windowS of game time: a
     * report's gust (the highest 3 s mean in 10 minutes, WMO-No. 8). The gusts are a function of time, so the past is
     * sampled exactly as the vehicles there felt them; the mean wind is taken as now's over the window. Costs one gust
     * evaluation per half second of the window (a report every few seconds, not every tick).
     */
    public static double peakGustMps(Level level, double x, double y, double z, double windowS, double averageS) {
        double now = SeaSurface.time(level);
        WaveRegion region = SeaSurface.regionAt(SeaSurface.persistentRegions(level), x, z);
        boolean own = region != null && !region.wind().calm();
        Vec3d mean = at(level, x, y, z).meanWind();
        if (mean.lengthSquared() == 0) { return 0; }
        int n = Math.max(1, (int) Math.ceil(windowS / GUST_STEP_S));
        double[] speed = new double[n + 1];
        for (int i = 0; i <= n; i++) {
            double t = now - windowS + i * GUST_STEP_S;
            Vec3d g = own ? region.gust(level, x, z, t) : WorldWind.gust(level, x, y, z, t);
            speed[i] = Math.hypot(mean.x() + g.x(), mean.z() + g.z());
        }
        return WindGusts.runningExtremes(speed, (int) Math.round(averageS / GUST_STEP_S))[0];
    }

    /** The prevailing direction of a level's wind (radians, blowing toward), from the world seed: WorldWind's synoptic wander is about it. */
    public static double prevailingTowardRad(long seed, ResourceKey<Level> dimension) {
        return new SurfaceWind.Synoptic(seed ^ dimension.location().hashCode()).prevailing();
    }

    /** Visibility now (m). */
    public static double visibilityM(Level level) { return visibilityM(level, 1f); }

    /** Visibility at a partial tick (m): interpolated in the logarithm between clear, rain and thunder. */
    public static double visibilityM(Level level, float partialTick) {
        double rain = Math.clamp(level.getRainLevel(partialTick), 0, 1), thunder = Math.clamp(level.getThunderLevel(partialTick), 0, 1);
        double log = Math.log(CLEAR_M) + rain * (Math.log(RAIN_M) - Math.log(CLEAR_M)) + rain * thunder * (Math.log(THUNDER_M) - Math.log(RAIN_M));
        return Math.exp(log);
    }
}
