package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.context.CommandContext;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.DynamicCommandExceptionType;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.SeaSpectrum;
import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.api.WaveField;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.core.BlockPos;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;

import java.util.List;
import java.util.Locale;

/**
 * /aerocore waves: world sea regions centred on the command position. A region holds one regular wave, an irregular
 * JONSWAP sea (given by Hs and Tp, or raised by a wind over the region's own water), or calm water (no ambient waves, but the same
 * surface handling for ship wakes and vanilla entities). Regions with waves survey the bed when they are made (and on
 * survey) so their waves follow the coast; wind sets a region's wind.
 */
public final class WaveCommands {
    public static final double DEFAULT_RADIUS = 24, DEFAULT_SEA_RADIUS = 64;
    /** Components per synthesised sea: enough for a short-crested, non-repeating surface at a bounded cost per sample. */
    public static final int SEA_COMPONENTS = 32;
    /** cos^n spreading for wind sea; DNV-RP-C205 3.5.8.6 gives n = 2 to 4 for wind sea and n >= 6 for swell. */
    public static final double DEFAULT_SPREADING = 4;
    private static final SimpleCommandExceptionType NO_WATER = new SimpleCommandExceptionType(Component.translatable("command.aerocore.waves.no_water"));
    private static final SimpleCommandExceptionType LIMIT = new SimpleCommandExceptionType(Component.translatable("command.aerocore.waves.limit"));
    private static final SimpleCommandExceptionType NO_REGION = new SimpleCommandExceptionType(Component.translatable("command.aerocore.waves.no_region"));
    private static final DynamicCommandExceptionType TOO_HIGH = new DynamicCommandExceptionType(
            height -> Component.translatable("command.aerocore.waves.wind_sea_too_high", height));
    /** Highest significant wave height the sea commands accept, m. */
    public static final double MAX_SIGNIFICANT_HEIGHT = 4;
    private static final DynamicCommandExceptionType INVALID = new DynamicCommandExceptionType(
            reason -> Component.translatable("command.aerocore.platform.waves.invalid", reason));
    private WaveCommands() {}

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("waves").requires(source -> source.hasPermission(2))
                .then(Commands.literal("off").executes(context -> {
                    CommandSourceStack s = context.getSource();
                    int removed = WaveRegions.of(s.getLevel()).removeAt(s.getLevel(), s.getPosition().x, s.getPosition().z);
                    s.sendSuccess(() -> Component.translatable("command.aerocore.waves.removed", removed), false);
                    return removed;
                }))
                .then(Commands.literal("clear").executes(context -> {
                    CommandSourceStack s = context.getSource();
                    int removed = WaveRegions.of(s.getLevel()).clear(s.getLevel());
                    s.sendSuccess(() -> Component.translatable("command.aerocore.waves.removed", removed), false);
                    return removed;
                }))
                .then(Commands.literal("info").executes(context -> {
                    CommandSourceStack s = context.getSource();
                    List<WaveRegion> regions = WaveRegions.of(s.getLevel()).regions();
                    if (regions.isEmpty()) { s.sendSuccess(() -> Component.translatable("command.aerocore.waves.none"), false); }
                    for (int i = 0; i < regions.size(); i++) { Component line = describe(i + 1, regions.get(i), s.getLevel()); s.sendSuccess(() -> line, false); }
                    return regions.size();
                }))
                .then(Commands.literal("natural")
                        .then(Commands.literal("on").executes(context -> natural(context.getSource(), true)))
                        .then(Commands.literal("off").executes(context -> natural(context.getSource(), false))))
                .then(Commands.literal("survey").executes(WaveCommands::survey)
                        .then(Commands.literal("clear").executes(WaveCommands::clearSurvey)))
                .then(Commands.literal("wind")
                        .then(Commands.literal("weather")
                                .then(Commands.argument("direction_degrees", DoubleArgumentType.doubleArg(-180, 180))
                                        .executes(context -> windWeather(context, WaveRegion.Wind.WEATHER_DEFAULTS))
                                        .then(Commands.argument("clear_mps", DoubleArgumentType.doubleArg(0, SeaWind.MAX_WIND))
                                                .then(Commands.argument("rain_mps", DoubleArgumentType.doubleArg(0, SeaWind.MAX_WIND))
                                                        .then(Commands.argument("thunder_mps", DoubleArgumentType.doubleArg(0, SeaWind.MAX_WIND))
                                                                .executes(context -> windWeather(context, new double[]{DoubleArgumentType.getDouble(context, "clear_mps"),
                                                                        DoubleArgumentType.getDouble(context, "rain_mps"), DoubleArgumentType.getDouble(context, "thunder_mps")})))))))
                        .then(Commands.argument("speed_mps", DoubleArgumentType.doubleArg(0, SeaWind.MAX_WIND))
                                .then(Commands.argument("direction_degrees", DoubleArgumentType.doubleArg(-180, 180)).executes(WaveCommands::wind))))
                .then(Commands.literal("windsea")
                        .then(Commands.argument("wind_mps", DoubleArgumentType.doubleArg(0.5, SeaWind.MAX_WIND))
                                .then(Commands.argument("direction_degrees", DoubleArgumentType.doubleArg(-180, 180))
                                        .executes(context -> windSea(context, Double.NaN, DEFAULT_SEA_RADIUS))
                                        .then(Commands.argument("fetch_km", DoubleArgumentType.doubleArg(0, 1000))
                                                .executes(context -> windSea(context, 1000 * DoubleArgumentType.getDouble(context, "fetch_km"), DEFAULT_SEA_RADIUS))
                                                .then(Commands.argument("radius_m", DoubleArgumentType.doubleArg(WaveRegion.MIN_RADIUS, WaveRegion.MAX_RADIUS))
                                                        .executes(context -> windSea(context, 1000 * DoubleArgumentType.getDouble(context, "fetch_km"),
                                                                DoubleArgumentType.getDouble(context, "radius_m"))))))))
                .then(Commands.literal("calm")
                        .executes(context -> calm(context, DEFAULT_SEA_RADIUS))
                        .then(Commands.argument("radius_m", DoubleArgumentType.doubleArg(WaveRegion.MIN_RADIUS, WaveRegion.MAX_RADIUS))
                                .executes(context -> calm(context, DoubleArgumentType.getDouble(context, "radius_m")))))
                .then(Commands.literal("sea")
                        .then(Commands.argument("significant_height_m", DoubleArgumentType.doubleArg(0.05, 4))
                                .then(Commands.argument("peak_period_s", DoubleArgumentType.doubleArg(1, 20))
                                        .then(Commands.argument("direction_degrees", DoubleArgumentType.doubleArg(-180, 180))
                                                .executes(context -> sea(context, DEFAULT_SEA_RADIUS, DEFAULT_SPREADING))
                                                .then(Commands.argument("radius_m", DoubleArgumentType.doubleArg(WaveRegion.MIN_RADIUS, WaveRegion.MAX_RADIUS))
                                                        .executes(context -> sea(context, DoubleArgumentType.getDouble(context, "radius_m"), DEFAULT_SPREADING))
                                                        .then(Commands.argument("spreading_exponent", DoubleArgumentType.doubleArg(1, 30))
                                                                .executes(context -> sea(context, DoubleArgumentType.getDouble(context, "radius_m"),
                                                                        DoubleArgumentType.getDouble(context, "spreading_exponent")))))))))
                .then(Commands.argument("amplitude_m", DoubleArgumentType.doubleArg(0, 2))
                        .then(Commands.argument("wavelength_m", DoubleArgumentType.doubleArg(2, 64))
                                .then(Commands.argument("direction_degrees", DoubleArgumentType.doubleArg(-180, 180))
                                        .executes(context -> create(context, DEFAULT_RADIUS))
                                        .then(Commands.argument("radius_m", DoubleArgumentType.doubleArg(WaveRegion.MIN_RADIUS, WaveRegion.MAX_RADIUS))
                                                .executes(context -> create(context, DoubleArgumentType.getDouble(context, "radius_m")))))));
    }

    private static int create(CommandContext<CommandSourceStack> context, double radius) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        WaveRegion region = create(s.getLevel(), s.getPosition().x, s.getPosition().y, s.getPosition().z,
                DoubleArgumentType.getDouble(context, "amplitude_m"), DoubleArgumentType.getDouble(context, "wavelength_m"),
                DoubleArgumentType.getDouble(context, "direction_degrees"), radius);
        s.sendSuccess(() -> describe(1, region, s.getLevel()), false);
        return 1;
    }

    private static int sea(CommandContext<CommandSourceStack> context, double radius, double spreading) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        WaveRegion region = createSea(s.getLevel(), s.getPosition().x, s.getPosition().y, s.getPosition().z,
                DoubleArgumentType.getDouble(context, "significant_height_m"), DoubleArgumentType.getDouble(context, "peak_period_s"),
                DoubleArgumentType.getDouble(context, "direction_degrees"), radius, spreading, s.getLevel().getRandom().nextLong());
        s.sendSuccess(() -> describe(1, region, s.getLevel()), false);
        return 1;
    }

    private static int calm(CommandContext<CommandSourceStack> context, double radius) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        WaveRegion region = add(s.getLevel(), s.getPosition().x, s.getPosition().y, s.getPosition().z, radius, depth -> WaveField.NONE, null);
        s.sendSuccess(() -> describe(1, region, s.getLevel()), false);
        return 1;
    }

    /** The region containing the command position, or an error. */
    private static WaveRegion here(CommandSourceStack s) throws CommandSyntaxException {
        WaveRegion region = WaveRegions.of(s.getLevel()).regionAt(s.getPosition().x, s.getPosition().z);
        if (region == null) { throw NO_REGION.create(); }
        return region;
    }

    private static int survey(CommandContext<CommandSourceStack> context) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        ServerLevel level = s.getLevel();
        WaveRegion old = here(s);
        WaveRegion.Survey survey = WaveRegions.survey(level, old.centerX(), old.centerZ(), old.radiusM(), old.stillWaterY());
        WaveRegions.of(level).replace(level, old, old.withSurvey(survey));
        s.sendSuccess(() -> Component.translatable("command.aerocore.waves.surveyed", survey.nx(), survey.nz(), f(survey.spacing()), survey.landNodes()), false);
        return survey.landNodes();
    }

    /** Drops the region's survey: open-sea waves everywhere again (no refraction, shelter, surf or fetch growth). */
    private static int clearSurvey(CommandContext<CommandSourceStack> context) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        ServerLevel level = s.getLevel();
        WaveRegion old = here(s);
        WaveRegions.of(level).replace(level, old, old.withSurvey(null));
        s.sendSuccess(() -> Component.translatable("command.aerocore.waves.survey_cleared"), false);
        return 1;
    }

    /** The region's wind follows the weather: clear, rain and thunder speeds (a rule) toward the given direction. */
    private static int windWeather(CommandContext<CommandSourceStack> context, double[] speeds) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        ServerLevel level = s.getLevel();
        WaveRegion old = here(s);
        double degrees = DoubleArgumentType.getDouble(context, "direction_degrees");
        WaveRegion.Wind wind = new WaveRegion.Wind(speeds[0], Math.toRadians(degrees), speeds[1], speeds[2]);
        WaveRegions.of(level).replace(level, old, old.withWind(wind));
        s.sendSuccess(() -> Component.translatable("command.aerocore.waves.wind_weather_set", f(speeds[0]), f(speeds[1]), f(speeds[2]), f(degrees), f(wind.mean(level))), false);
        return 1;
    }

    /** Switches natural seas (NaturalSeas) for the caller's level. */
    private static int natural(CommandSourceStack s, boolean on) {
        WaveRegions.of(s.getLevel()).setNaturalSeas(s.getLevel(), on);
        s.sendSuccess(() -> Component.translatable(on ? "command.aerocore.waves.natural_on" : "command.aerocore.waves.natural_off"), true);
        return 1;
    }

    private static int wind(CommandContext<CommandSourceStack> context) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        ServerLevel level = s.getLevel();
        WaveRegion old = here(s);
        double speed = DoubleArgumentType.getDouble(context, "speed_mps"), degrees = DoubleArgumentType.getDouble(context, "direction_degrees");
        WaveRegions.of(level).replace(level, old, old.withWind(new WaveRegion.Wind(speed, Math.toRadians(degrees))));
        s.sendSuccess(() -> Component.translatable("command.aerocore.waves.wind_set", f(speed), f(degrees)), false);
        return 1;
    }

    /**
     * A sea raised by a wind over the water the region covers: the bed is surveyed, every water point's straight-line
     * fetch measured upwind to land (plus the open water beyond the survey), and the sea synthesised at the median fetch
     * over the region's water (SeaWind.sea at the measured depth); each spot then grows with its own fetch. The open
     * water beyond is fetch_km when given, else measured along the wind line through the command position over loaded
     * chunks (unlimited when it runs past them over water). The wind is kept on the region.
     */
    private static int windSea(CommandContext<CommandSourceStack> context, double fetchBeyond, double radius) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        ServerLevel level = s.getLevel();
        double u10 = DoubleArgumentType.getDouble(context, "wind_mps"), degrees = DoubleArgumentType.getDouble(context, "direction_degrees");
        double heading = Math.toRadians(degrees), x = s.getPosition().x, z = s.getPosition().z;
        WorldWaterSampler sampler = new WorldWaterSampler(level);
        BlockPos column = BlockPos.containing(x, s.getPosition().y, z);
        double surface = sampler.stillSurface(column);
        if (Double.isNaN(surface)) { throw NO_WATER.create(); }
        double depth = sampler.depthBelow(column, surface);
        WaveRegion.Survey survey = WaveRegions.survey(level, x, z, radius, surface);
        double beyond = Double.isNaN(fetchBeyond) ? openWaterBeyond(level, survey, x, z, surface, heading) : fetchBeyond;
        float[] bed = survey.depths(Double.isFinite(depth) ? depth : 1e4);
        double[] fetch = dev.mcmod.aerocore.api.CoastalWaves.fetches(survey.spacing(), survey.nx(), survey.nz(), bed, heading, beyond);
        double[] wet = java.util.Arrays.stream(fetch).filter(f -> !Double.isNaN(f)).sorted().toArray();
        if (wet.length == 0) { throw NO_WATER.create(); }
        double median = wet[wet.length / 2];
        double[] sea = SeaWind.sea(u10, Math.max(median, survey.spacing()), depth);
        if (sea[0] > MAX_SIGNIFICANT_HEIGHT) { throw TOO_HIGH.create(f(sea[0])); }
        WaveRegion region;
        try {
            double gamma = SeaSpectrum.recommendedGamma(sea[0], sea[1]);
            WaveField waves = SeaSpectrum.synthesize(sea[0], sea[1], gamma, heading, DEFAULT_SPREADING, depth, SEA_COMPONENTS, level.getRandom().nextLong());
            region = new WaveRegion(x, z, radius, surface, waves, new WaveRegion.Sea(sea[0], sea[1], gamma, heading, DEFAULT_SPREADING, u10, beyond, median),
                    survey, new WaveRegion.Wind(u10, heading));
        } catch (IllegalArgumentException invalid) { throw INVALID.create(invalid.getMessage()); }
        try { WaveRegions.of(level).add(level, region); } catch (IllegalStateException full) { throw LIMIT.create(); }
        Component open = Double.isInfinite(beyond) ? Component.translatable("command.aerocore.waves.fetch_unlimited")
                : Component.translatable("command.aerocore.waves.fetch_m", f(beyond));
        double[] shortest = SeaWind.sea(u10, Math.max(wet[wet.length / 10], survey.spacing()), depth), longest = SeaWind.sea(u10, Math.max(wet[wet.length * 9 / 10], survey.spacing()), depth);
        WaveRegion made = region;
        s.sendSuccess(() -> Component.translatable("command.aerocore.waves.wind_sea", f(u10), f(median), open, f(sea[0]), f(sea[1]), f(shortest[0]), f(longest[0])), false);
        s.sendSuccess(() -> describe(1, made, level), false);
        return 1;
    }

    /**
     * Open water upwind of the survey along the wind line through (x, z), m: from where the line leaves the surveyed
     * square, over water at the still level, to land; unlimited when it runs into unloaded chunks (open sea assumed)
     * or past 10 km.
     */
    private static double openWaterBeyond(ServerLevel level, WaveRegion.Survey survey, double x, double z, double stillY, double heading) {
        double ex = -Math.cos(heading), ez = -Math.sin(heading);
        double maxX = survey.x0() + (survey.nx() - 1) * survey.spacing(), maxZ = survey.z0() + (survey.nz() - 1) * survey.spacing();
        double t = 0;
        while (x + t * ex >= survey.x0() && x + t * ex <= maxX && z + t * ez >= survey.z0() && z + t * ez <= maxZ) { t += 0.5; }
        int y = net.minecraft.util.Mth.floor(stillY - 1e-6);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        for (double d = 0; d < 10000; d += 1) {
            pos.set(net.minecraft.util.Mth.floor(x + (t + d) * ex), y, net.minecraft.util.Mth.floor(z + (t + d) * ez));
            if (!level.hasChunkAt(pos)) { return Double.POSITIVE_INFINITY; }
            if (!level.getFluidState(pos).is(net.minecraft.tags.FluidTags.WATER)) { return d; }
        }
        return Double.POSITIVE_INFINITY;
    }


    /**
     * Measures the still surface and depth in the column through (x, y, z), then adds a region of the given radius
     * centred there, replacing overlapping regions. Shared with /aerocore platform waves.
     */
    static WaveRegion create(ServerLevel level, double x, double y, double z, double amplitude, double wavelength,
                             double directionDegrees, double radius) throws CommandSyntaxException {
        return add(level, x, y, z, radius,
                depth -> new WaveField(List.of(new LinearWave(amplitude, wavelength, Math.toRadians(directionDegrees), depth, 0))), null);
    }

    /** An irregular sea from the DNV JONSWAP spectrum with the recommended peak enhancement, seeded for synthesis. */
    static WaveRegion createSea(ServerLevel level, double x, double y, double z, double significantHeight, double peakPeriod,
                                double directionDegrees, double radius, double spreading, long seed) throws CommandSyntaxException {
        double gamma = SeaSpectrum.recommendedGamma(significantHeight, peakPeriod), heading = Math.toRadians(directionDegrees);
        return add(level, x, y, z, radius,
                depth -> SeaSpectrum.synthesize(significantHeight, peakPeriod, gamma, heading, spreading, depth, SEA_COMPONENTS, seed),
                new WaveRegion.Sea(significantHeight, peakPeriod, gamma, heading, spreading));
    }

    private interface Field { WaveField at(double depthM); }

    private static WaveRegion add(ServerLevel level, double x, double y, double z, double radius, Field field, WaveRegion.Sea sea)
            throws CommandSyntaxException {
        return add(level, x, y, z, radius, field, sea, WaveRegion.Wind.CALM);
    }

    /** Measures the still surface and depth at the position, builds the field, surveys the bed when there are waves, and adds the region. */
    private static WaveRegion add(ServerLevel level, double x, double y, double z, double radius, Field field, WaveRegion.Sea sea,
                                  WaveRegion.Wind wind) throws CommandSyntaxException {
        WorldWaterSampler sampler = new WorldWaterSampler(level);
        BlockPos column = BlockPos.containing(x, y, z);
        double surface = sampler.stillSurface(column);
        if (Double.isNaN(surface)) { throw NO_WATER.create(); }
        WaveRegion region;
        try {
            WaveField waves = field.at(sampler.depthBelow(column, surface));
            WaveRegion.Survey survey = waves.isEmpty() ? null : WaveRegions.survey(level, x, z, radius, surface);
            region = new WaveRegion(x, z, radius, surface, waves, sea, survey, wind);
        } catch (IllegalArgumentException invalid) { throw INVALID.create(invalid.getMessage()); }
        try { WaveRegions.of(level).add(level, region); } catch (IllegalStateException full) { throw LIMIT.create(); }
        return region;
    }

    static Component describe(int index, WaveRegion region, net.minecraft.world.level.Level level) {
        Component coast;
        if (region.survey() == null) { coast = Component.translatable("command.aerocore.waves.coast.none"); }
        else if (region.coastPending()) { coast = Component.translatable("command.aerocore.waves.coast.pending"); }
        else if (region.coast() == null) { coast = Component.translatable("command.aerocore.waves.coast.failed"); }
        else { coast = Component.translatable("command.aerocore.waves.coast.ready", region.survey().landNodes(), region.survey().nx() * region.survey().nz()); }
        WaveRegion.Wind wind = region.wind();
        Component air = wind.weather()
                ? Component.translatable("command.aerocore.waves.environment_weather", f(wind.speedMps()), f(wind.rainMps()), f(wind.thunderMps()),
                f(wind.mean(level)), f(Math.toDegrees(wind.directionRadians())), coast)
                : Component.translatable("command.aerocore.waves.environment", f(wind.speedMps()), f(Math.toDegrees(wind.directionRadians())), coast);
        Component out = summary(index, region).copy().append("\n").append(air);
        if (region.sea() != null && region.sea().windSea()) {
            out = out.copy().append("\n").append(Component.translatable("command.aerocore.waves.sea_state", f(100 * region.scale()), f(region.scale() * region.sea().significantHeightM())));
        }
        return out;
    }

    private static Component summary(int index, WaveRegion region) {
        List<LinearWave> waves = region.waves().components();
        if (waves.isEmpty()) {
            return Component.translatable("command.aerocore.waves.calm_info", index, f(region.centerX()), f(region.centerZ()), f(region.radiusM()),
                    f(region.stillWaterY()));
        }
        WaveRegion.Sea sea = region.sea();
        if (sea != null) {
            double depth = waves.getFirst().depthM();
            return Component.translatable("command.aerocore.waves.sea_info", index, f(region.centerX()), f(region.centerZ()), f(region.radiusM()),
                    f(region.taperM()), f(sea.significantHeightM()), f(sea.peakPeriodS()), f(sea.gamma()), f(Math.toDegrees(sea.directionRadians())),
                    f(sea.spreading()), waves.size(), f(SeaSpectrum.wavelength(2 * Math.PI / sea.peakPeriodS(), depth)), f(depth), f(region.stillWaterY()));
        }
        LinearWave w = waves.getFirst();
        return Component.translatable("command.aerocore.waves.info", index, f(region.centerX()), f(region.centerZ()), f(region.radiusM()),
                f(region.taperM()), f(w.amplitudeM()), f(w.wavelengthM()), f(w.periodS()), f(Math.toDegrees(w.directionRadians())),
                f(w.depthM()), f(region.stillWaterY()));
    }

    private static String f(double value) { return String.format(Locale.ROOT, "%.2f", value); }
}
