package dev.mcmod.aerocore.neoforge.vehicle;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.arguments.IntegerArgumentType;
import com.mojang.brigadier.arguments.StringArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.Coupling;
import dev.mcmod.aerocore.api.vehicle.DesignException;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
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
 * /aerocore vehicle: spawn a blueprint file, set controls, report, show force arrows, restart a vehicle stopped by a
 * physics error, remove; 0.36: couple the nearest vehicle to the next nearest by a ball where they come closest, uncouple. Blueprint files are JSON in the server directory's aerocore/blueprints (the game directory in single player).
 */
public final class VehicleCommands {
    private static final SimpleCommandExceptionType NONE = new SimpleCommandExceptionType(Component.translatable("command.aerocore.vehicle.none"));
    private static final double SEARCH = 64;

    private VehicleCommands() {}

    public static Path folder(MinecraftServer server) { return server.getServerDirectory().resolve("aerocore").resolve("blueprints"); }

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("vehicle").requires(source -> source.hasPermission(2))
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
                .then(Commands.literal("resume").executes(c -> {
                    PartVehicle v = nearest(c.getSource());
                    if (v.vehicle() == null) { report(c.getSource(), v); return 0; }
                    if (v.fault() == null) { c.getSource().sendFailure(Component.translatable("command.aerocore.vehicle.running")); return 0; }
                    v.resume();
                    c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.resumed", v.blueprint().name()), false);
                    return 1;
                }))
                .then(Commands.literal("debug").then(Commands.argument("mode", IntegerArgumentType.integer(0, 7)).executes(c -> {
                    PartVehicle v = nearest(c.getSource());
                    if (c.getSource().getEntity() != null) { v.setOwner(c.getSource().getEntity().getUUID()); }
                    v.setDebugMode(IntegerArgumentType.getInteger(c, "mode"));
                    c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.debug", IntegerArgumentType.getInteger(c, "mode")), false);
                    return 1;
                })))
                .then(Commands.literal("couple").executes(c -> couple(c.getSource())))
                .then(Commands.literal("uncouple").executes(c -> {
                    PartVehicle v = nearest(c.getSource());
                    if (v.coupling() == null && v.followers().isEmpty()) { c.getSource().sendFailure(Component.translatable("command.aerocore.vehicle.not_coupled")); return 0; }
                    v.uncoupleAll(Couplings.Reason.COMMAND);
                    c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.uncoupled"), false);
                    return 1;
                }))
                .then(Commands.literal("remove").executes(c -> {
                    nearest(c.getSource()).discard();
                    c.getSource().sendSuccess(() -> Component.translatable("command.aerocore.vehicle.removed"), false);
                    return 1;
                }));
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

    /**
     * 0.36: the nearest vehicle hung on the next nearest (within the search) by a ball joint where their contact spheres
     * come closest (between their centres when either has none): a debugging coupling for any two vehicles.
     */
    private static int couple(CommandSourceStack source) throws CommandSyntaxException {
        PartVehicle follower = nearest(source);
        Vec3 at = source.getPosition();
        PartVehicle leader = source.getLevel().getEntitiesOfClass(PartVehicle.class, new AABB(at, at).inflate(SEARCH)).stream().filter(v -> v != follower)
                .min(Comparator.comparingDouble(v -> v.distanceToSqr(follower))).orElseThrow(NONE::create);
        if (leader.vehicle() == null || follower.vehicle() == null || leader.vehicleState() == null || follower.vehicleState() == null) {
            source.sendFailure(Component.translatable("command.aerocore.vehicle.couple_failed", Couplings.Result.NOT_READY.name()));
            return 0;
        }
        RigidBodyState l = leader.vehicleState().body(), f = follower.vehicleState().body();
        Vec3d a = l.position(), b = f.position();
        double best = Double.MAX_VALUE;
        for (CollisionSphere sa : leader.vehicle().collision()) {
            Vec3d pa = l.worldPoint(leader.vehicle().properties(), sa.centerLocal());
            for (CollisionSphere sb : follower.vehicle().collision()) {
                Vec3d pb = f.worldPoint(follower.vehicle().properties(), sb.centerLocal());
                double d = pa.subtract(pb).length() - sa.radiusM() - sb.radiusM();
                if (d < best) { best = d; a = pa; b = pb; }
            }
        }
        Vec3d mid = a.add(b).scale(0.5);
        Coupling ball = Coupling.ball(local(l, leader.vehicle().properties(), mid), local(f, follower.vehicle().properties(), mid));
        Couplings.Result r = leader.couple(follower, ball);
        if (r != Couplings.Result.OK) { source.sendFailure(Component.translatable("command.aerocore.vehicle.couple_failed", r.name())); return 0; }
        source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.coupled", follower.blueprint().name(), leader.blueprint().name()), false);
        return 1;
    }

    /** A world point in a body's model frame. */
    private static Vec3d local(RigidBodyState s, dev.mcmod.aerocore.api.RigidBodyProperties p, Vec3d world) {
        return p.centerOfMassLocal().add(s.orientation().inverse().rotate(world.subtract(s.position())));
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
        if (v.fault() != null) { source.sendFailure(Component.translatable("command.aerocore.vehicle.halted", v.fault())); }
        if (v.coupling() != null) {
            PartVehicle leader = v.leader();
            source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.coupling", leader == null ? "?" : leader.blueprint() == null ? "?" : leader.blueprint().name(),
                    f(v.couplingForceN().length()), f(Math.toDegrees(v.articulationRad()))), false);
        }
        if (v.vehicleState() == null) { return; }
        v.vehicleState().diagnostics().forEach((part, values) -> {
            StringBuilder line = new StringBuilder();
            values.forEach((k, value) -> line.append(k).append(' ').append(f(value)).append("  "));
            source.sendSuccess(() -> Component.translatable("command.aerocore.vehicle.part", part, line.toString().trim()), false);
        });
    }

    private static String f(double value) { return String.format(Locale.ROOT, "%.2f", value); }
}
