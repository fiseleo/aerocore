package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.api.Vec3d;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;

import java.util.Locale;

/** /aerocore wind: the level's wind (WorldWind) at the caller, and fixing or freeing it. */
public final class WindCommands {
    private WindCommands() {}

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("wind")
                .executes(context -> info(context.getSource()))
                .then(Commands.literal("set").requires(source -> source.hasPermission(2))
                        .then(Commands.argument("speed_mps", DoubleArgumentType.doubleArg(0, SeaWind.MAX_WIND))
                                .then(Commands.argument("direction_degrees", DoubleArgumentType.doubleArg(-180, 180)).executes(context -> {
                                    double speed = DoubleArgumentType.getDouble(context, "speed_mps"), degrees = DoubleArgumentType.getDouble(context, "direction_degrees");
                                    WorldWind.setOverride(context.getSource().getLevel(), new WorldWind.Reference(speed, Math.toRadians(degrees)));
                                    context.getSource().sendSuccess(() -> Component.translatable("command.aerocore.wind.set", f(speed), f(degrees)), true);
                                    return 1;
                                }))))
                .then(Commands.literal("natural").requires(source -> source.hasPermission(2)).executes(context -> {
                    ServerLevel level = context.getSource().getLevel();
                    WorldWind.setOverride(level, null);
                    WorldWind.Reference now = WorldWind.reference(level);
                    context.getSource().sendSuccess(() -> Component.translatable("command.aerocore.wind.natural", f(now.speedMps()),
                            f(Math.toDegrees(now.directionRadians()))), true);
                    return 1;
                }));
    }

    private static int info(CommandSourceStack source) {
        ServerLevel level = source.getLevel();
        WorldWind.Reference ref = WorldWind.reference(level);
        double x = source.getPosition().x, y = source.getPosition().y + (source.getEntity() == null ? 0 : source.getEntity().getEyeHeight()), z = source.getPosition().z;
        WorldWind.Details d = WorldWind.describe(level, x, y, z);
        Vec3d gust = WorldWind.gust(level, x, y, z, SeaSurface.time(level));
        String mode = WorldWind.override(level) != null ? "command.aerocore.wind.fixed" : "command.aerocore.wind.weather";
        source.sendSuccess(() -> Component.translatable("command.aerocore.wind.info", Component.translatable(mode), f(ref.speedMps()),
                f(Math.toDegrees(ref.directionRadians())), f(d.heightM()), String.format(Locale.ROOT, "%.4f", d.roughnessM()), f(d.canopyM()),
                f(d.meanMps()), f(100 * d.hillSpeedUp()), f(100 * d.shelter()), f(gust.length())), false);
        return 1;
    }

    private static String f(double value) { return String.format(Locale.ROOT, "%.2f", value); }
}
