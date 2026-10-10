package dev.mcmod.aerocore.neoforge;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import dev.mcmod.aerocore.api.*;
import dev.mcmod.aerocore.neoforge.probe.ProbeCommands;
import dev.mcmod.aerocore.neoforge.probe.ProbeEntities;
import dev.mcmod.aerocore.neoforge.probe.ProbeGameTests;
import dev.mcmod.aerocore.neoforge.probe.PlatformCommands;
import dev.mcmod.aerocore.neoforge.probe.PlatformGameTests;
import dev.mcmod.aerocore.neoforge.probe.WaveCommands;
import dev.mcmod.aerocore.neoforge.probe.WaveNetwork;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.fml.common.Mod;
import net.neoforged.neoforge.common.NeoForge;
import net.neoforged.neoforge.event.RegisterCommandsEvent;
import net.neoforged.neoforge.event.RegisterGameTestsEvent;

import java.util.Locale;

@Mod(AeroCore.MOD_ID)
public final class AeroCore {
    public static final String MOD_ID = "aerocore";

    public AeroCore(IEventBus modBus) {
        ProbeEntities.TYPES.register(modBus);
        modBus.addListener(WaveNetwork::register);
        if (Boolean.getBoolean("aerocore.gametests")) {
            modBus.addListener((RegisterGameTestsEvent event) -> event.register(ProbeGameTests.class));
            modBus.addListener((RegisterGameTestsEvent event) -> event.register(PlatformGameTests.class));
            modBus.addListener((RegisterGameTestsEvent event) -> event.register(dev.mcmod.aerocore.neoforge.probe.HullGameTests.class));
            modBus.addListener((RegisterGameTestsEvent event) -> event.register(dev.mcmod.aerocore.neoforge.probe.SeaGameTests.class));
        }
        NeoForge.EVENT_BUS.addListener(AeroCore::registerCommands);
    }

    private static void registerCommands(RegisterCommandsEvent event) {
        event.getDispatcher().register(Commands.literal("aerocore")
                .then(ProbeCommands.tree())
                .then(PlatformCommands.tree())
                .then(WaveCommands.tree())
                .then(dev.mcmod.aerocore.neoforge.probe.WindCommands.tree())
                .then(dev.mcmod.aerocore.neoforge.probe.GroundSurfaces.tree())
                .then(dev.mcmod.aerocore.neoforge.probe.HullCommands.tree())
                .then(dev.mcmod.aerocore.neoforge.probe.PlaningCommands.tree())
                .then(Commands.literal("sample")
                        .executes(context -> sample(context.getSource(), 30, 5))
                        .then(Commands.argument("speed_mps", DoubleArgumentType.doubleArg(0, 100))
                                .executes(context -> sample(context.getSource(),
                                        DoubleArgumentType.getDouble(context, "speed_mps"), 5))
                                .then(Commands.argument("angle_degrees", DoubleArgumentType.doubleArg(-180, 180))
                                        .executes(context -> sample(context.getSource(),
                                                DoubleArgumentType.getDouble(context, "speed_mps"),
                                                DoubleArgumentType.getDouble(context, "angle_degrees"))))))
                .then(Commands.literal("air")
                        .executes(context -> air(context.getSource(), 0, 30, 5))
                        .then(Commands.argument("altitude_m", DoubleArgumentType.doubleArg(-500, 20000))
                                .executes(context -> air(context.getSource(), DoubleArgumentType.getDouble(context, "altitude_m"), 30, 5))
                                .then(Commands.argument("speed_mps", DoubleArgumentType.doubleArg(0, 100))
                                        .executes(context -> air(context.getSource(), DoubleArgumentType.getDouble(context, "altitude_m"),
                                                DoubleArgumentType.getDouble(context, "speed_mps"), 5))
                                        .then(Commands.argument("angle_degrees", DoubleArgumentType.doubleArg(-180, 180))
                                                .executes(context -> air(context.getSource(), DoubleArgumentType.getDouble(context, "altitude_m"),
                                                        DoubleArgumentType.getDouble(context, "speed_mps"),
                                                        DoubleArgumentType.getDouble(context, "angle_degrees")))))))
                .then(Commands.literal("water")
                        .executes(context -> water(context.getSource(), 0.5, 0))
                        .then(Commands.argument("radius_m", DoubleArgumentType.doubleArg(0.001, 10))
                                .executes(context -> water(context.getSource(), DoubleArgumentType.getDouble(context, "radius_m"), 0))
                                .then(Commands.argument("center_depth_m", DoubleArgumentType.doubleArg(-100, 100))
                                        .executes(context -> water(context.getSource(), DoubleArgumentType.getDouble(context, "radius_m"),
                                                DoubleArgumentType.getDouble(context, "center_depth_m")))))));
    }

    private static int sample(CommandSourceStack source, double speed, double angleDegrees) {
        double angle = Math.toRadians(angleDegrees);
        BodyMotion motion = new BodyMotion(new Vec3d(speed * Math.cos(angle), -speed * Math.sin(angle), 0), Vec3d.ZERO);
        LiftingSurface wing = new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 10, SymmetricAirfoil.DEFAULT);
        SurfaceForces result = AeroSolver.surface(motion, AirState.STILL_SEA_LEVEL, wing);
        double lift = result.dynamicPressurePa() * wing.areaM2() * result.coefficients().lift();
        double drag = result.dynamicPressurePa() * wing.areaM2() * result.coefficients().drag();
        source.sendSuccess(() -> Component.translatable("command.aerocore.sample.input", format(speed),
                format(angleDegrees)), false);
        source.sendSuccess(() -> Component.translatable("command.aerocore.sample.output", format(lift), format(drag),
                format(result.dynamicPressurePa()), Component.translatable(result.coefficients().stalled()
                        ? "command.aerocore.stalled" : "command.aerocore.attached")), false);
        return 1;
    }

    private static String format(double value) { return String.format(Locale.ROOT, "%.2f", value); }

    private static int air(CommandSourceStack source, double altitude, double speed, double angleDegrees) {
        Atmosphere.Sample environment = Atmosphere.atAltitude(altitude, Vec3d.ZERO);
        double angle = Math.toRadians(angleDegrees);
        BodyMotion motion = new BodyMotion(new Vec3d(speed * Math.cos(angle), -speed * Math.sin(angle), 0), Vec3d.ZERO);
        LiftingSurface wing = new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 10, 1, SymmetricAirfoil.DEFAULT);
        SurfaceForces result = AeroSolver.surface(motion, environment.fluid(), wing);
        source.sendSuccess(() -> Component.translatable("command.aerocore.air.environment", format(altitude),
                format(environment.temperatureKelvin()), format(environment.pressurePa()),
                String.format(Locale.ROOT, "%.4f", environment.fluid().densityKgPerM3())), false);
        source.sendSuccess(() -> Component.translatable("command.aerocore.air.flow", format(speed), format(angleDegrees),
                String.format(Locale.ROOT, "%.0f", result.flow().reynoldsNumber()),
                String.format(Locale.ROOT, "%.4f", result.flow().machNumber())), false);
        source.sendSuccess(() -> Component.translatable("command.aerocore.sample.output",
                format(result.dynamicPressurePa() * wing.areaM2() * result.coefficients().lift()),
                format(result.dynamicPressurePa() * wing.areaM2() * result.coefficients().drag()),
                format(result.dynamicPressurePa()), Component.translatable(result.coefficients().stalled()
                        ? "command.aerocore.stalled" : "command.aerocore.attached")), false);
        return 1;
    }

    private static int water(CommandSourceStack source, double radius, double depth) {
        BuoyantSphere sphere = new BuoyantSphere(Vec3d.ZERO, radius);
        Immersion immersion = HydroSolver.sphereImmersion(sphere, depth, Vec3d.Y);
        Wrench result = HydroSolver.buoyancy(FluidState.FRESH_WATER_20C, immersion, Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2));
        source.sendSuccess(() -> Component.translatable("command.aerocore.water.input", format(radius), format(depth)), false);
        source.sendSuccess(() -> Component.translatable("command.aerocore.water.output", format(100 * immersion.fraction()),
                String.format(Locale.ROOT, "%.5f", immersion.displacedVolumeM3()), format(result.force().y()),
                String.format(Locale.ROOT, "%.4f", immersion.centerOfBuoyancy().y())), false);
        return 1;
    }
}
