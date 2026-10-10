package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.context.CommandContext;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.phys.Vec3;

import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public final class ProbeCommands {
    private static final SimpleCommandExceptionType NONE = error("command.aerocore.probe.none");
    private static final SimpleCommandExceptionType BLOCKED = error("command.aerocore.probe.blocked");
    private static final SimpleCommandExceptionType LIMIT = error("command.aerocore.probe.limit");
    private ProbeCommands() {}

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("probe").requires(source -> source.hasPermission(2))
                .then(Commands.literal("spawn").executes(context -> spawn(context.getSource(), 250, 0))
                        .then(Commands.argument("mass_kg", DoubleArgumentType.doubleArg(PhysicsProbe.MIN_MASS_KG, PhysicsProbe.MAX_MASS_KG))
                                .executes(context -> spawn(context.getSource(), number(context, "mass_kg"), 0))
                                .then(Commands.argument("speed_mps", DoubleArgumentType.doubleArg(0, 30))
                                        .executes(context -> spawn(context.getSource(), number(context, "mass_kg"), number(context, "speed_mps"))))))
                .then(Commands.literal("inspect").executes(context -> {
                    PhysicsProbe probe = nearest(context.getSource());
                    context.getSource().sendSuccess(probe::diagnostics, false);
                    return 1;
                }))
                .then(Commands.literal("clear").executes(context -> clear(context.getSource())))
                .then(flowCommand("wind"))
                .then(flowCommand("current"));
    }

    private static LiteralArgumentBuilder<CommandSourceStack> flowCommand(String kind) {
        return Commands.literal(kind)
                .then(Commands.argument("x_mps", DoubleArgumentType.doubleArg(-10, 10))
                        .then(Commands.argument("y_mps", DoubleArgumentType.doubleArg(-10, 10))
                                .then(Commands.argument("z_mps", DoubleArgumentType.doubleArg(-10, 10))
                                        .executes(context -> {
                                            PhysicsProbe probe = nearest(context.getSource());
                                            Vec3d velocity = new Vec3d(number(context, "x_mps"), number(context, "y_mps"), number(context, "z_mps"));
                                            if (kind.equals("wind")) { probe.setWindMps(velocity); }
                                            else { probe.setAddedCurrentMps(velocity); }
                                            context.getSource().sendSuccess(() -> Component.translatable("command.aerocore.probe.flow",
                                                    probe.getId(), Component.translatable("command.aerocore.probe." + kind),
                                                    format(velocity.x()), format(velocity.y()), format(velocity.z())), false);
                                            return 1;
                                        }))));
    }

    private static int spawn(CommandSourceStack source, double mass, double speed) throws CommandSyntaxException {
        ServerPlayer player = source.getPlayerOrException();
        ServerLevel level = source.getLevel();
        int count = 0;
        for (var entity : level.getAllEntities()) {
            if (entity instanceof PhysicsProbe && ++count >= 32) { throw LIMIT.create(); }
        }
        PhysicsProbe probe = ProbeEntities.PHYSICS_PROBE.get().create(level);
        if (probe == null) { throw BLOCKED.create(); }
        Vec3 direction = player.getLookAngle();
        Vec3 center = player.getEyePosition().add(direction.scale(3));
        probe.setPos(center.x, center.y - PhysicsProbe.RADIUS_M, center.z);
        probe.configure(mass, player.getUUID(), MinecraftUnits.fromMinecraft(direction.scale(speed)));
        if (!level.getWorldBorder().isWithinBounds(probe.getBoundingBox()) || !level.noCollision(probe, probe.getBoundingBox())) {
            throw BLOCKED.create();
        }
        if (!level.addFreshEntity(probe)) { throw BLOCKED.create(); }
        source.sendSuccess(() -> Component.translatable("command.aerocore.probe.spawned", probe.getId(), format(mass), format(speed)), false);
        return 1;
    }

    private static List<PhysicsProbe> owned(CommandSourceStack source, double radius) throws CommandSyntaxException {
        ServerPlayer player = source.getPlayerOrException();
        return source.getLevel().getEntitiesOfClass(PhysicsProbe.class, player.getBoundingBox().inflate(radius),
                probe -> !probe.isRemoved() && player.getUUID().equals(probe.owner()) && probe.distanceToSqr(player) <= radius * radius);
    }

    private static PhysicsProbe nearest(CommandSourceStack source) throws CommandSyntaxException {
        return owned(source, 32).stream().min(Comparator.comparingDouble(probe -> probe.position().distanceToSqr(source.getPosition())))
                .orElseThrow(NONE::create);
    }

    private static int clear(CommandSourceStack source) throws CommandSyntaxException {
        List<PhysicsProbe> probes = owned(source, 64);
        probes.forEach(PhysicsProbe::discard);
        source.sendSuccess(() -> Component.translatable("command.aerocore.probe.cleared", probes.size()), false);
        return probes.size();
    }

    private static double number(CommandContext<CommandSourceStack> context, String key) { return DoubleArgumentType.getDouble(context, key); }
    private static String format(double value) { return String.format(Locale.ROOT, "%.2f", value); }
    private static SimpleCommandExceptionType error(String key) { return new SimpleCommandExceptionType(Component.translatable(key)); }
}
