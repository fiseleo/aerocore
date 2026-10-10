package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.context.CommandContext;
import com.mojang.brigadier.exceptions.CommandSyntaxException;
import com.mojang.brigadier.exceptions.SimpleCommandExceptionType;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.TestPlatform;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.phys.Vec3;
import java.util.Comparator;
import java.util.List;

public final class PlatformCommands {
    private static final SimpleCommandExceptionType NONE=error("command.aerocore.platform.none");
    private static final SimpleCommandExceptionType BLOCKED=error("command.aerocore.platform.blocked");
    private static final SimpleCommandExceptionType LIMIT=error("command.aerocore.platform.limit");
    private PlatformCommands() {}
    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("platform").requires(source->source.hasPermission(2))
                .then(Commands.literal("spawn").executes(context->spawn(context.getSource())))
                .then(Commands.literal("inspect").executes(context->{
                    FloatingPlatform p=nearest(context.getSource());
                    for(Component line:p.report()) {context.getSource().sendSuccess(()->line,false);}
                    return 1;
                }))
                .then(debugCommand())
                .then(flowCommand("wind"))
                .then(flowCommand("current"))
                .then(waveCommand())
                .then(Commands.literal("propeller").then(Commands.argument("rev_per_s",
                        DoubleArgumentType.doubleArg(-FloatingPlatform.MAX_PROPELLER_RPS,FloatingPlatform.MAX_PROPELLER_RPS)).executes(context->{
                    FloatingPlatform p=nearest(context.getSource());p.setPropellerRps(number(context,"rev_per_s"));
                    context.getSource().sendSuccess(p::propellerDiagnostics,false);return 1;
                })))
                .then(thermalCommand())
                .then(Commands.literal("ballast")
                        .then(Commands.argument("x_m",DoubleArgumentType.doubleArg(-3,3))
                                .then(Commands.argument("y_m",DoubleArgumentType.doubleArg(-2,10))
                                        .then(Commands.argument("z_m",DoubleArgumentType.doubleArg(-3,3)).executes(context->{
                                            FloatingPlatform p=nearest(context.getSource());
                                            return change(context.getSource(),p,new Vec3d(number(context,"x_m"),number(context,"y_m"),number(context,"z_m")),p.physicsState().orientation());
                                        })))))
                .then(Commands.literal("tilt").then(Commands.argument("x_degrees",DoubleArgumentType.doubleArg(-180,180))
                        .then(Commands.argument("z_degrees",DoubleArgumentType.doubleArg(-180,180)).executes(context->{
                            FloatingPlatform p=nearest(context.getSource());
                            Orientation q=Orientation.rotationVector(Vec3d.Z.scale(Math.toRadians(number(context,"z_degrees"))))
                                    .multiply(Orientation.rotationVector(Vec3d.X.scale(Math.toRadians(number(context,"x_degrees")))));
                            return change(context.getSource(),p,p.ballastLocal(),q);
                        }))))
                .then(Commands.literal("spin").then(Commands.argument("x_rad_s",DoubleArgumentType.doubleArg(-2,2))
                        .then(Commands.argument("y_rad_s",DoubleArgumentType.doubleArg(-2,2))
                                .then(Commands.argument("z_rad_s",DoubleArgumentType.doubleArg(-2,2)).executes(context->{
                                    FloatingPlatform p=nearest(context.getSource());
                                    p.setAngularVelocity(new Vec3d(number(context,"x_rad_s"),number(context,"y_rad_s"),number(context,"z_rad_s")));
                                    context.getSource().sendSuccess(p::diagnostics,false); return 1;
                                })))))
                .then(Commands.literal("reset").executes(context->change(context.getSource(),nearest(context.getSource()),TestPlatform.DEFAULT_BALLAST,Orientation.IDENTITY)))
                .then(Commands.literal("clear").executes(context->{
                    List<FloatingPlatform> platforms=owned(context.getSource(),64);platforms.forEach(FloatingPlatform::discard);
                    context.getSource().sendSuccess(()->Component.translatable("command.aerocore.platform.cleared",platforms.size()),false);return platforms.size();
                }));
    }
    private static LiteralArgumentBuilder<CommandSourceStack> debugCommand() {
        var command=Commands.literal("debug");
        String[] names={"off","forces","flow","all","relative","all_relative"};
        int[] modes={0,1,2,3,6,7};
        for(int i=0;i<names.length;i++) {
            final int mode=i;
            command.then(Commands.literal(names[i]).executes(context->{
                nearest(context.getSource()).setDebugMode(modes[mode]);
                context.getSource().sendSuccess(()->Component.translatable("command.aerocore.platform.debug.changed",names[mode]),false);return 1;
            }));
        }
        return command;
    }
    private static LiteralArgumentBuilder<CommandSourceStack> flowCommand(String name) {
        return Commands.literal(name).then(Commands.argument("x_mps",DoubleArgumentType.doubleArg(-10,10))
                .then(Commands.argument("y_mps",DoubleArgumentType.doubleArg(-10,10))
                        .then(Commands.argument("z_mps",DoubleArgumentType.doubleArg(-10,10)).executes(context->{
                            FloatingPlatform platform=nearest(context.getSource());
                            Vec3d velocity=new Vec3d(number(context,"x_mps"),number(context,"y_mps"),number(context,"z_mps"));
                            if(name.equals("wind")) {platform.environment().setWind(velocity);} else {platform.environment().setCurrent(velocity);}
                            context.getSource().sendSuccess(()->Component.translatable("command.aerocore.platform.flow",
                                    Component.translatable("command.aerocore.probe."+name),velocity.x(),velocity.y(),velocity.z()),false);return 1;
                        }))));
    }
    /**
     * Shortcut for a world wave region (WaveCommands) of radius 24 m centred on the platform; depth and still level
     * are measured in the column under its model origin. "off" removes the region the platform is in.
     */
    private static LiteralArgumentBuilder<CommandSourceStack> waveCommand() {
        return Commands.literal("waves")
                .then(Commands.literal("off").executes(context->{
                    FloatingPlatform p=nearest(context.getSource());Vec3d origin=p.modelOrigin();
                    WaveRegions.of(context.getSource().getLevel()).removeAt(context.getSource().getLevel(),origin.x(),origin.z());
                    p.refreshWaves();
                    context.getSource().sendSuccess(p::waveDiagnostics,false);return 1;
                }))
                .then(Commands.literal("sea").then(Commands.argument("significant_height_m",DoubleArgumentType.doubleArg(0.05,4))
                        .then(Commands.argument("peak_period_s",DoubleArgumentType.doubleArg(1,20))
                                .then(Commands.argument("direction_degrees",DoubleArgumentType.doubleArg(-180,180)).executes(context->{
                                    FloatingPlatform p=nearest(context.getSource());
                                    Vec3d origin=p.modelOrigin();
                                    WaveCommands.createSea(context.getSource().getLevel(),origin.x(),origin.y(),origin.z(),number(context,"significant_height_m"),
                                            number(context,"peak_period_s"),number(context,"direction_degrees"),WaveCommands.DEFAULT_SEA_RADIUS,
                                            WaveCommands.DEFAULT_SPREADING,context.getSource().getLevel().getRandom().nextLong());
                                    p.refreshWaves();
                                    context.getSource().sendSuccess(p::waveDiagnostics,false);return 1;
                                })))))
                .then(Commands.argument("amplitude_m",DoubleArgumentType.doubleArg(0,2))
                        .then(Commands.argument("wavelength_m",DoubleArgumentType.doubleArg(2,64))
                                .then(Commands.argument("direction_degrees",DoubleArgumentType.doubleArg(-180,180)).executes(context->{
                                    FloatingPlatform p=nearest(context.getSource());
                                    Vec3d origin=p.modelOrigin();
                                    WaveCommands.create(context.getSource().getLevel(),origin.x(),origin.y(),origin.z(),number(context,"amplitude_m"),
                                            number(context,"wavelength_m"),number(context,"direction_degrees"),WaveCommands.DEFAULT_RADIUS);
                                    p.refreshWaves();
                                    context.getSource().sendSuccess(p::waveDiagnostics,false);return 1;
                                }))));
    }
    private static LiteralArgumentBuilder<CommandSourceStack> thermalCommand() {
        return Commands.literal("thermal").executes(context->{
            FloatingPlatform p=nearest(context.getSource());context.getSource().sendSuccess(p::thermalDiagnostics,false);return 1;
        }).then(temperatureCommand("air",-60,80)).then(temperatureCommand("water",0,80)).then(temperatureCommand("body",-50,250))
                .then(Commands.literal("standard_air").executes(context->{
                    FloatingPlatform p=nearest(context.getSource());p.environment().standardAir();
                    context.getSource().sendSuccess(p::thermalDiagnostics,false);return 1;
                }));
    }
    private static LiteralArgumentBuilder<CommandSourceStack> temperatureCommand(String material,double min,double max) {
        return Commands.literal(material).then(Commands.argument("celsius",DoubleArgumentType.doubleArg(min,max)).executes(context->{
            FloatingPlatform p=nearest(context.getSource());double value=number(context,"celsius");
            switch(material) {
                case "air" -> p.environment().setAirCelsius(value);
                case "water" -> p.environment().setWaterCelsius(value);
                case "body" -> p.setBodyTemperatureCelsius(value);
                default -> throw new IllegalArgumentException(material);
            }
            context.getSource().sendSuccess(p::thermalDiagnostics,false);return 1;
        }));
    }
    private static int spawn(CommandSourceStack source) throws CommandSyntaxException {
        ServerPlayer player=source.getPlayerOrException();
        int count=0;
        for(var entity:source.getLevel().getAllEntities()) {if(entity instanceof FloatingPlatform && ++count>=16) {throw LIMIT.create();}}
        FloatingPlatform platform=ProbeEntities.FLOATING_PLATFORM.get().create(source.getLevel());
        if(platform==null) {throw BLOCKED.create();}
        Vec3 origin=player.getEyePosition().add(player.getLookAngle().scale(5));
        platform.configure(player.getUUID(),MinecraftUnits.fromMinecraft(origin));
        if(!source.getLevel().getWorldBorder().isWithinBounds(platform.getBoundingBox()) || !source.getLevel().noCollision(platform,platform.getBoundingBox())) {throw BLOCKED.create();}
        if(!source.getLevel().addFreshEntity(platform)) {throw BLOCKED.create();}
        source.sendSuccess(()->Component.translatable("command.aerocore.platform.spawned",platform.getId()),false);return 1;
    }
    private static int change(CommandSourceStack source,FloatingPlatform p,Vec3d ballast,Orientation q) throws CommandSyntaxException {
        if(!p.reconfigure(ballast,q)) {throw BLOCKED.create();}
        source.sendSuccess(()->Component.translatable("command.aerocore.platform.changed"),false);
        source.sendSuccess(p::diagnostics,false);return 1;
    }
    private static List<FloatingPlatform> owned(CommandSourceStack source,double radius) throws CommandSyntaxException {
        ServerPlayer player=source.getPlayerOrException();
        return source.getLevel().getEntitiesOfClass(FloatingPlatform.class,player.getBoundingBox().inflate(radius),
                p->!p.isRemoved() && player.getUUID().equals(p.owner()) && p.distanceToSqr(player)<=radius*radius);
    }
    private static FloatingPlatform nearest(CommandSourceStack source) throws CommandSyntaxException {
        return owned(source,32).stream().min(Comparator.comparingDouble(p->p.position().distanceToSqr(source.getPosition()))).orElseThrow(NONE::create);
    }
    private static double number(CommandContext<CommandSourceStack> context,String key) {return DoubleArgumentType.getDouble(context,key);}
    private static SimpleCommandExceptionType error(String key) {return new SimpleCommandExceptionType(Component.translatable(key));}
}
