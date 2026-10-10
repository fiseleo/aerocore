package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.phys.Vec3;

import java.util.Comparator;
import java.util.List;

/** /aerocore hull: the slender-hull demonstrator, set by commands only. */
public final class HullCommands {
    private static final SimpleCommandExceptionType NONE = new SimpleCommandExceptionType(Component.translatable("command.aerocore.hull.none"));
    private static final SimpleCommandExceptionType BLOCKED = new SimpleCommandExceptionType(Component.translatable("command.aerocore.hull.blocked"));
    private static final SimpleCommandExceptionType LIMIT = new SimpleCommandExceptionType(Component.translatable("command.aerocore.hull.limit"));
    private HullCommands() {}

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("hull").requires(source -> source.hasPermission(2))
                .then(Commands.literal("spawn").executes(context -> spawn(context.getSource())))
                .then(Commands.literal("inspect").executes(context -> {
                    FloatingHull hull = nearest(context.getSource());
                    for (Component line : hull.report()) { context.getSource().sendSuccess(() -> line, false); }
                    return 1;
                }))
                .then(Commands.literal("propeller").then(Commands.argument("rev_per_s", DoubleArgumentType.doubleArg(-FloatingHull.MAX_PROPELLER_RPS, FloatingHull.MAX_PROPELLER_RPS))
                        .executes(context -> {
                            FloatingHull hull = nearest(context.getSource());
                            hull.setPropellerRps(DoubleArgumentType.getDouble(context, "rev_per_s"));
                            context.getSource().sendSuccess(hull::controlDiagnostics, false);
                            return 1;
                        })))
                .then(Commands.literal("rudder").then(Commands.argument("degrees", DoubleArgumentType.doubleArg(-35, 35)).executes(context -> {
                    FloatingHull hull = nearest(context.getSource());
                    hull.setRudderDegrees(DoubleArgumentType.getDouble(context, "degrees"));
                    context.getSource().sendSuccess(hull::controlDiagnostics, false);
                    return 1;
                })))
                .then(debugCommand())
                .then(flowCommand("wind"))
                .then(flowCommand("current"))
                .then(Commands.literal("clear").executes(context -> {
                    List<FloatingHull> hulls = owned(context.getSource(), 64);
                    hulls.forEach(FloatingHull::discard);
                    context.getSource().sendSuccess(() -> Component.translatable("command.aerocore.hull.cleared", hulls.size()), false);
                    return hulls.size();
                }));
    }

    private static LiteralArgumentBuilder<CommandSourceStack> debugCommand() {
        var command = Commands.literal("debug");
        String[] names = {"off", "forces", "flow", "all", "relative", "all_relative"};
        int[] modes = {0, 1, 2, 3, 6, 7};
        for (int i = 0; i < names.length; i++) {
            final int mode = i;
            command.then(Commands.literal(names[i]).executes(context -> {
                nearest(context.getSource()).setDebugMode(modes[mode]);
                context.getSource().sendSuccess(() -> Component.translatable("command.aerocore.platform.debug.changed", names[mode]), false);
                return 1;
            }));
        }
        return command;
    }

    private static LiteralArgumentBuilder<CommandSourceStack> flowCommand(String name) {
        return Commands.literal(name).then(Commands.argument("x_mps", DoubleArgumentType.doubleArg(-10, 10))
                .then(Commands.argument("y_mps", DoubleArgumentType.doubleArg(-10, 10))
                        .then(Commands.argument("z_mps", DoubleArgumentType.doubleArg(-10, 10)).executes(context -> {
                            FloatingHull hull = nearest(context.getSource());
                            Vec3d velocity = new Vec3d(DoubleArgumentType.getDouble(context, "x_mps"), DoubleArgumentType.getDouble(context, "y_mps"),
                                    DoubleArgumentType.getDouble(context, "z_mps"));
                            if (name.equals("wind")) { hull.environment().setWind(velocity); } else { hull.environment().setCurrent(velocity); }
                            context.getSource().sendSuccess(() -> Component.translatable("command.aerocore.platform.flow",
                                    Component.translatable("command.aerocore.probe." + name), velocity.x(), velocity.y(), velocity.z()), false);
                            return 1;
                        }))));
    }

    /** 8 m ahead of the player, bow along the player's horizontal look direction, deck 0.3 m above the eye height. */
    private static int spawn(CommandSourceStack source) throws CommandSyntaxException {
        ServerPlayer player = source.getPlayerOrException();
        int count = 0;
        for (var entity : source.getLevel().getAllEntities()) { if (entity instanceof FloatingHull && ++count >= 8) { throw LIMIT.create(); } }
        FloatingHull hull = ProbeEntities.FLOATING_HULL.get().create(source.getLevel());
        if (hull == null) { throw BLOCKED.create(); }
        Vec3 look = player.getLookAngle();
        double lx = look.x, lz = look.z, length = Math.hypot(lx, lz);
        if (length < 1e-6) { lx = 1; lz = 0; length = 1; }
        lx /= length; lz /= length;
        Vec3 origin = player.getEyePosition().add(lx * 8, 0, lz * 8);
        hull.configure(player.getUUID(), MinecraftUnits.fromMinecraft(origin), Math.atan2(-lz, lx));
        if (!source.getLevel().getWorldBorder().isWithinBounds(hull.getBoundingBox()) || !source.getLevel().noCollision(hull, hull.getBoundingBox())) { throw BLOCKED.create(); }
        if (!source.getLevel().addFreshEntity(hull)) { throw BLOCKED.create(); }
        source.sendSuccess(() -> Component.translatable("command.aerocore.hull.spawned", hull.getId()), false);
        return 1;
    }

    private static List<FloatingHull> owned(CommandSourceStack source, double radius) throws CommandSyntaxException {
        ServerPlayer player = source.getPlayerOrException();
        return source.getLevel().getEntitiesOfClass(FloatingHull.class, player.getBoundingBox().inflate(radius),
                hull -> player.getUUID().equals(hull.owner()));
    }
    private static FloatingHull nearest(CommandSourceStack source) throws CommandSyntaxException {
        return owned(source, 32).stream().min(Comparator.comparingDouble(h -> h.position().distanceToSqr(source.getPosition()))).orElseThrow(NONE::create);
    }
}
