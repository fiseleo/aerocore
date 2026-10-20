package dev.mcmod.aerocore.neoforge.client;

import com.mojang.brigadier.arguments.StringArgumentType;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.commands.Commands;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.fml.loading.FMLEnvironment;
import net.neoforged.neoforge.client.event.RegisterClientCommandsEvent;

/**
 * Development client command (0.29, not registered in production): /aerocoredev parts occlusion|culling on|off
 * switches how part meshes are baked (and bakes them again), to compare the drawing with and without each step;
 * /aerocoredev parts shading auto|shader|vanilla picks the shading for the session; /aerocoredev parts gpu on|off
 * draws baked parts from GPU buffers or writes their vertices every frame (PartGpu); 0.39: /aerocoredev parts beautify
 * on|off|config overrides the beautified shading for the session (Beautify).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class DevPartCommands {
    private DevPartCommands() {}

    @SubscribeEvent public static void register(RegisterClientCommandsEvent event) {
        if (FMLEnvironment.production) { return; }
        event.getDispatcher().register(Commands.literal("aerocoredev").then(Commands.literal("parts")
                .then(Commands.argument("what", StringArgumentType.word()).then(Commands.argument("on", StringArgumentType.word()).executes(c -> {
                    String value = StringArgumentType.getString(c, "on");
                    boolean on = value.equals("on");
                    switch (StringArgumentType.getString(c, "what")) {
                        case "occlusion" -> MeshBakery.noOcclusion = !on;
                        case "culling" -> MeshBakery.noCulling = !on;
                        case "gpu" -> { PartGpu.enabled = on; return 1; }   // draw bakes from GPU buffers, or write them each frame
                        case "beautify" -> {   // 0.39: on, off, or config (the setting again); the next tick rebakes (Beautify.tick)
                            Beautify.override = value.equals("config") ? null : on;
                            return 1;
                        }
                        case "shading" -> {   // auto, shader or vanilla, for this session (not saved)
                            dev.mcmod.aerocore.neoforge.AeroCoreClientConfig.PART_SHADING.set(
                                    dev.mcmod.aerocore.neoforge.AeroCoreClientConfig.PartShading.valueOf(value.toUpperCase(java.util.Locale.ROOT)));
                            return 1;
                        }
                        default -> { return 0; }
                    }
                    MeshBakery.clear();
                    return 1;
                }))))
                .then(Commands.literal("lamps")
                        .then(Commands.literal("clear").executes(c -> { FieldLamps.remove(GRID); return 1; }))
                        .then(Commands.literal("grid").then(Commands.argument("n", com.mojang.brigadier.arguments.IntegerArgumentType.integer(1, 60))
                                .then(Commands.argument("spacing", com.mojang.brigadier.arguments.DoubleArgumentType.doubleArg(0.5, 500)).executes(c -> {
                                    grid(com.mojang.brigadier.arguments.IntegerArgumentType.getInteger(c, "n"),
                                            com.mojang.brigadier.arguments.DoubleArgumentType.getDouble(c, "spacing"));
                                    return 1;
                                }))))));
    }

    private static final Object GRID = new Object();

    /**
     * 0.38: a test field of lights ahead of the player along their heading: n rows of n all-round lights (white,
     * blue, green, red by row; 200 cd) at the spacing, then two rows of edge lights (10 000 cd) running on as far
     * again, 20 sequenced flashers in front of them and a PAPI (four units) beside their start, all aimed back at the
     * player.
     */
    private static void grid(int n, double spacing) {
        var player = net.minecraft.client.Minecraft.getInstance().player;
        if (player == null) { return; }
        double yaw = Math.toRadians(player.getYRot());
        dev.mcmod.aerocore.api.Vec3d ahead = new dev.mcmod.aerocore.api.Vec3d(-Math.sin(yaw), 0, Math.cos(yaw)), side = new dev.mcmod.aerocore.api.Vec3d(-ahead.z(), 0, ahead.x());
        dev.mcmod.aerocore.api.Vec3d base = new dev.mcmod.aerocore.api.Vec3d(player.getX(), player.getY() + 0.3, player.getZ()), back = ahead.scale(-1);
        java.util.List<dev.mcmod.aerocore.api.light.FieldLamp> lamps = new java.util.ArrayList<>();
        int[] colours = {0xffffffff, 0xff3050ff, 0xff20ff40, 0xffff2010};
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dev.mcmod.aerocore.api.Vec3d at = base.add(ahead.scale(10 + i * spacing)).add(side.scale((j - (n - 1) / 2.0) * spacing));
                lamps.add(dev.mcmod.aerocore.api.light.FieldLamp.omni(at, 200, colours[i % 4], 0, 0.1));
            }
        }
        double start = 10 + n * spacing, runway = Math.max(400, n * spacing);
        for (double s = 0; s <= runway; s += 60) {
            for (int k = -1; k <= 1; k += 2) {
                lamps.add(dev.mcmod.aerocore.api.light.FieldLamp.omni(base.add(ahead.scale(start + s)).add(side.scale(k * 15)), 10_000, 0xfffff4e0, 1, 0.15));
            }
        }
        for (int f = 0; f < 20; f++) {
            dev.mcmod.aerocore.api.Vec3d at = base.add(ahead.scale(start - 30 * (20 - f)));
            lamps.add(dev.mcmod.aerocore.api.light.FieldLamp.beam(at, back, 30, 20, 6, 20_000, 0xffffffff, 2, 0.15)
                    .with(dev.mcmod.aerocore.api.light.LightSignal.sequence(0.5, f, 20)));
        }
        double[] minutes = {210, 190, 170, 150};
        for (int u = 0; u < 4; u++) {
            dev.mcmod.aerocore.api.Vec3d at = base.add(ahead.scale(start + 300)).add(side.scale(-25 - 9 * u));
            lamps.add(dev.mcmod.aerocore.api.light.FieldLamp.beam(at, back, 20, 16, 3, 15_000, 0xffffffff, 3, 0.2)
                    .with(dev.mcmod.aerocore.api.light.SectorColours.papi(Math.toRadians(minutes[u] / 60), Math.toRadians(3.0 / 60), 0xffff2010, 0xffffffff)));
        }
        FieldLamps.put(player.level().dimension(), GRID, lamps);
        player.displayClientMessage(net.minecraft.network.chat.Component.literal(lamps.size() + " field lights"), false);
    }
}
