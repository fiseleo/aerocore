package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.context.CommandContext;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.DynamicCommandExceptionType;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.SeaSpectrum;
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
 * JONSWAP sea, or calm water (no ambient waves, but the same surface handling for ship wakes and vanilla entities).
 */
public final class WaveCommands {
    public static final double DEFAULT_RADIUS = 24, DEFAULT_SEA_RADIUS = 64;
    /** Components per synthesised sea: enough for a short-crested, non-repeating surface at a bounded cost per sample. */
    public static final int SEA_COMPONENTS = 32;
    /** cos^n spreading for wind sea; DNV-RP-C205 3.5.8.6 gives n = 2 to 4 for wind sea and n >= 6 for swell. */
    public static final double DEFAULT_SPREADING = 4;
    private static final SimpleCommandExceptionType NO_WATER = new SimpleCommandExceptionType(Component.translatable("command.aerocore.waves.no_water"));
    private static final SimpleCommandExceptionType LIMIT = new SimpleCommandExceptionType(Component.translatable("command.aerocore.waves.limit"));
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
                    for (int i = 0; i < regions.size(); i++) { Component line = describe(i + 1, regions.get(i)); s.sendSuccess(() -> line, false); }
                    return regions.size();
                }))
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
        s.sendSuccess(() -> describe(1, region), false);
        return 1;
    }

    private static int sea(CommandContext<CommandSourceStack> context, double radius, double spreading) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        WaveRegion region = createSea(s.getLevel(), s.getPosition().x, s.getPosition().y, s.getPosition().z,
                DoubleArgumentType.getDouble(context, "significant_height_m"), DoubleArgumentType.getDouble(context, "peak_period_s"),
                DoubleArgumentType.getDouble(context, "direction_degrees"), radius, spreading, s.getLevel().getRandom().nextLong());
        s.sendSuccess(() -> describe(1, region), false);
        return 1;
    }

    private static int calm(CommandContext<CommandSourceStack> context, double radius) throws CommandSyntaxException {
        CommandSourceStack s = context.getSource();
        WaveRegion region = add(s.getLevel(), s.getPosition().x, s.getPosition().y, s.getPosition().z, radius, depth -> WaveField.NONE, null);
        s.sendSuccess(() -> describe(1, region), false);
        return 1;
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
        WorldWaterSampler sampler = new WorldWaterSampler(level);
        BlockPos column = BlockPos.containing(x, y, z);
        double surface = sampler.stillSurface(column);
        if (Double.isNaN(surface)) { throw NO_WATER.create(); }
        WaveRegion region;
        try {
            region = new WaveRegion(x, z, radius, surface, field.at(sampler.depthBelow(column, surface)), sea);
        } catch (IllegalArgumentException invalid) { throw INVALID.create(invalid.getMessage()); }
        try { WaveRegions.of(level).add(level, region); } catch (IllegalStateException full) { throw LIMIT.create(); }
        return region;
    }

    static Component describe(int index, WaveRegion region) {
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
