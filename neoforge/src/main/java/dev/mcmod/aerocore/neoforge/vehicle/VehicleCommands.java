package dev.mcmod.aerocore.neoforge.vehicle;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.arguments.IntegerArgumentType;
import com.mojang.brigadier.arguments.StringArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignException;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.SampleBlueprints;
import dev.mcmod.aerocore.api.vehicle.Vehicle;
import dev.mcmod.aerocore.api.vehicle.VehicleBlueprint;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.server.MinecraftServer;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * /aerocore vehicle: write the sample blueprints, spawn a blueprint file, set controls, report, show force arrows,
 * remove. Blueprint files are JSON in the server directory's aerocore/blueprints (the game directory in single player).
 */
public final class VehicleCommands {
    private static final SimpleCommandExceptionType NONE = new SimpleCommandExceptionType(Component.translatable("command.aerocore.vehicle.none"));
    private static final double SEARCH = 64;

    private VehicleCommands() {}

    public static Path folder(MinecraftServer server) { return server.getServerDirectory().resolve("aerocore").resolve("blueprints"); }

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("vehicle").requires(source -> source.hasPermission(2))
                .then(Commands.literal("samples").executes(c -> samples(c.getSource())))
                .then(Commands.literal("spawn").then(Commands.argument("name", StringArgumentType.word()).executes(c -> spawn(c.getSource(), StringArgumentType.getString(c, "name")))))
                .then(Commands.literal("control").then(Commands.argument("channel", StringArgumentType.word())
                        .then(Commands.argument("value", DoubleArgumentType.doubleArg(-VehicleNetwork.MAX_VALUE, VehicleNetwork.MAX_VALUE)).executes(c -> {
                            PartVehicle v = nearest(c.getSource());
                            String channel = StringArgumentType.getString(c, "channel");
                            double value = DoubleArgumentType.getDouble(c, "value");
                            v.setControl(channel, value);
                            c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.control", channel, f(value)), false);
                            return 1;
                        }))))
                .then(Commands.literal("info").executes(c -> { report(c.getSource(), nearest(c.getSource())); return 1; }))
                .then(Commands.literal("debug").then(Commands.argument("mode", IntegerArgumentType.integer(0, 7)).executes(c -> {
                    PartVehicle v = nearest(c.getSource());
                    if (c.getSource().getEntity() != null) { v.setOwner(c.getSource().getEntity().getUUID()); }
                    v.setDebugMode(IntegerArgumentType.getInteger(c, "mode"));
                    c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.debug", IntegerArgumentType.getInteger(c, "mode")), false);
                    return 1;
                })))
                .then(Commands.literal("remove").executes(c -> {
                    nearest(c.getSource()).discard();
                    c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.removed"), false);
                    return 1;
                }));
    }

    /** Writes the sample blueprints that are not there yet. */
    private static int samples(CommandSourceStack source) {
        Path dir = folder(source.getServer());
        int written = 0;
        try {
            Files.createDirectories(dir);
            for (Map.Entry<String, VehicleBlueprint> e : SampleBlueprints.all().entrySet()) {
                Path file = dir.resolve(e.getKey() + ".json");
                if (Files.exists(file)) { continue; }
                Files.writeString(file, e.getValue().toJsonText(), StandardCharsets.UTF_8);
                written++;
            }
        } catch (IOException e) {
            source.sendFailure(Component.translatable("command.aerocore.vehicle.io", e.getMessage()));
            return 0;
        }
        int count = written;
        source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.samples", count, dir.toAbsolutePath().toString()), false);
        return count;
    }

    /** Spawns the blueprint file name.json four metres ahead of the source, facing where it looks. */
    private static int spawn(CommandSourceStack source, String name) {
        if (!name.matches("[A-Za-z0-9_.-]{1,64}")) { source.sendFailure(Component.translatable("command.aerocore.vehicle.name")); return 0; }
        Path file = folder(source.getServer()).resolve(name + ".json");
        VehicleBlueprint blueprint;
        try {
            blueprint = VehicleBlueprint.fromJsonText(Files.readString(file, StandardCharsets.UTF_8));
        } catch (IOException e) {
            source.sendFailure(Component.translatable("command.aerocore.vehicle.missing", file.toAbsolutePath().toString()));
            return 0;
        } catch (IllegalArgumentException e) {
            source.sendFailure(Component.translatable("command.aerocore.vehicle.malformed", e.getMessage()));
            return 0;
        }
        PartVehicle vehicle = VehicleRegistry.PART_VEHICLE.get().create(source.getLevel());
        if (vehicle == null) { return 0; }
        double yaw = Math.toRadians(source.getRotation().y);
        Vec3d ahead = new Vec3d(-Math.sin(yaw), 0, Math.cos(yaw));
        Vec3 at = source.getPosition();
        Vec3d origin = new Vec3d(at.x, at.y, at.z).add(ahead.scale(4));
        // The vehicle's +x along the look direction: a yaw of -(look yaw + 90 deg) about +y.
        Orientation q = Orientation.rotationVector(Vec3d.Y.scale(-(yaw + Math.PI / 2)));
        try {
            vehicle.setBlueprint(blueprint, origin, q);
        } catch (DesignException e) {
            source.sendFailure(Component.translatable("command.aerocore.vehicle.invalid", e.issues().stream().filter(i -> i.kind() == DesignIssue.Kind.INVALID).count()));
            issues(source, e.issues().stream().filter(i -> i.kind() == DesignIssue.Kind.INVALID).toList());
            return 0;
        }
        if (source.getEntity() != null) { vehicle.setOwner(source.getEntity().getUUID()); }
        source.getLevel().addFreshEntity(vehicle);
        Vehicle v = vehicle.vehicle();
        source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.spawned", blueprint.name(), v.installed().size(), f(v.properties().massKg()),
                v.issues().size()), false);
        issues(source, v.issues());
        return 1;
    }

    private static void issues(CommandSourceStack source, List<DesignIssue> issues) {
        for (DesignIssue i : issues.subList(0, Math.min(issues.size(), 6))) {
            source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.issue", i.parameter(), i.message()), false);
        }
    }

    private static PartVehicle nearest(CommandSourceStack source) throws CommandSyntaxException {
        Vec3 at = source.getPosition();
        return source.getLevel().getEntitiesOfClass(PartVehicle.class, new AABB(at, at).inflate(SEARCH)).stream()
                .min(Comparator.comparingDouble(v -> v.distanceToSqr(at))).orElseThrow(NONE::create);
    }

    /** The vehicle's state, controls and each part's diagnostics, in chat. */
    public static void report(CommandSourceStack source, PartVehicle v) {
        if (v.vehicle() == null) {
            source.sendFailure(Component.translatable("command.aerocore.vehicle.broken", String.valueOf(v.failure())));
            return;
        }
        Vehicle vehicle = v.vehicle();
        double speed = v.vehicleState() == null ? 0 : v.vehicleState().body().velocity().length();
        source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.info", v.blueprint().name(), v.getId(), f(vehicle.properties().massKg()),
                f(speed), f(speed * 3.6), v.controls().toString()), false);
        if (v.vehicleState() == null) { return; }
        v.vehicleState().diagnostics().forEach((part, values) -> {
            StringBuilder line = new StringBuilder();
            values.forEach((k, value) -> line.append(k).append(' ').append(f(value)).append("  "));
            source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.part", part, line.toString().trim()), false);
        });
    }

    private static String f(double value) { return String.format(Locale.ROOT, "%.2f", value); }
}
