package dev.mcmod.aerocore.neoforge.client;

import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.minecraft.client.Screenshot;
import net.minecraft.client.gui.screens.GenericMessageScreen;
import net.minecraft.client.gui.screens.TitleScreen;
import net.minecraft.client.server.IntegratedServer;
import net.minecraft.network.chat.Component;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.fml.loading.FMLEnvironment;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import org.slf4j.Logger;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Development-only screenshot runner, so the sea's appearance can be checked in a QA world without a person at the
 * keyboard. Inert unless the game runs from the development environment (not FMLEnvironment.production) and its game
 * directory holds aerocore-autoshot.txt, which is read and deleted at the first title screen. It opens only worlds
 * whose folder name starts with "AeroCore" (never a player's own world), then runs one step per client tick:
 * <pre>
 * world &lt;folder&gt;     the QA world (required, first)
 * size &lt;w&gt; &lt;h&gt;       window size in pixels
 * cmd &lt;command&gt;     a command on the integrated server, without the slash
 * wait &lt;ticks&gt;
 * shot &lt;name&gt;        screenshots/aerocore-&lt;name&gt;.png with the GUI hidden
 * debug &lt;view&gt;      the sea shader's development view (WaveSurfaceRenderer.debugView; 0 the picture)
 * fps                logs the frame rate of the last second, the slowest frame and server tick since the last fps
 * clouds &lt;on|off&gt;     draws clouds for this run (the player's setting comes back at the end)
 * key &lt;k&gt; &lt;on|off&gt;   holds or releases a movement key (forward, back, left, right, jump, sneak), e.g. to row a boat
 * camera &lt;c&gt;         first, back or front person view
 * look &lt;yaw&gt; &lt;pitch&gt; turns the player's view (degrees, as /tp)
 * quit               leave the world (the server saves it as usual) and close the game
 * </pre>
 * Lines starting with # are comments. The game is kept from pausing when its window loses focus while the plan runs.
 * Frames slower than 50 ms are logged on their own with where the time went (collector, JIT, the sea, the client tick).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class AutoShot {
    private static final Logger LOGGER = LogUtils.getLogger();
    private static final String PLAN = "aerocore-autoshot.txt", WORLD_PREFIX = "AeroCore";
    /** Null until the first title screen; empty when there is nothing (more) to do. */
    private static List<String> steps;
    private static int next, waiting;
    private static boolean entered, leaving, focusChanged, pauseOnLostFocus;
    private static final java.util.Set<net.minecraft.client.KeyMapping> HELD = new java.util.HashSet<>();
    private static net.minecraft.client.CloudStatus savedClouds;
    private static long lastFrame, worstFrame, slowFrames, tickStarted, tickNanos, lastGcMillis, lastGcCount, lastJitMillis, lastAccept;

    private AutoShot() {}

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        if (FMLEnvironment.production) { return; }
        Minecraft minecraft = Minecraft.getInstance();
        try {
            if (steps == null) { start(minecraft); return; }
            if (leaving) {
                if (minecraft.level == null) { finish(minecraft); }
                return;
            }
            if (steps.isEmpty() || next >= steps.size()) { return; }
            if (!entered) {
                if (minecraft.level == null || minecraft.player == null || minecraft.screen != null) { return; }
                entered = true;
            }
            // Held keys are pressed again every tick (losing focus releases them).
            for (net.minecraft.client.KeyMapping key : HELD) { key.setDown(true); }
            if (waiting > 0) { waiting--; return; }
            run(minecraft, steps.get(next++));
        } catch (RuntimeException | java.io.IOException e) {
            LOGGER.error("AeroCore autoshot stopped", e);
            steps = List.of();
            if (minecraft.level != null && !leaving) { leave(minecraft); } else { restoreFocus(minecraft); }
        }
    }

    /** Frame times while the plan runs, for the fps step: stutter shows in the slowest frame, not the average. */
    @SubscribeEvent public static void frame(net.neoforged.neoforge.client.event.RenderFrameEvent.Post event) {
        if (FMLEnvironment.production || !entered || leaving) { return; }
        long now = System.nanoTime(), gcMillis = 0, gcCount = 0;
        for (java.lang.management.GarbageCollectorMXBean gc : java.lang.management.ManagementFactory.getGarbageCollectorMXBeans()) {
            gcMillis += Math.max(0, gc.getCollectionTime()); gcCount += Math.max(0, gc.getCollectionCount());
        }
        long jitMillis = java.lang.management.ManagementFactory.getCompilationMXBean().getTotalCompilationTime();
        if (lastFrame != 0) {
            long dt = now - lastFrame;
            worstFrame = Math.max(worstFrame, dt);
            if (dt > 50_000_000L) {
                slowFrames++;
                // Where a slow frame went: collector pauses, JIT time (other threads, a hint of new code running), the
                // sea's drawing, the client tick (all mods and vanilla) and accepting region updates.
                LOGGER.info("AeroCore autoshot: slow frame {} ms: GC {} ms in {}, JIT {} ms, sea {} ms (refresh {} ms for {} tiles, look {} ms, compute {} ms), client tick {} ms, region updates {} ms",
                        String.format("%.1f", dt / 1e6), gcMillis - lastGcMillis, gcCount - lastGcCount, jitMillis - lastJitMillis,
                        String.format("%.1f", WaveSurfaceRenderer.lastDrawNanos / 1e6), String.format("%.1f", WaveSurfaceRenderer.phase[0] / 1e6),
                        WaveSurfaceRenderer.refreshes, String.format("%.1f", WaveSurfaceRenderer.phase[1] / 1e6), String.format("%.1f", WaveSurfaceRenderer.phase[2] / 1e6),
                        String.format("%.1f", tickNanos / 1e6),
                        String.format("%.1f", (ClientWaveRegions.acceptNanos - lastAccept) / 1e6));
            }
        }
        lastFrame = now; lastGcMillis = gcMillis; lastGcCount = gcCount; lastJitMillis = jitMillis; lastAccept = ClientWaveRegions.acceptNanos; tickNanos = 0;
    }

    @SubscribeEvent(priority = net.neoforged.bus.api.EventPriority.HIGHEST)
    public static void tickStart(ClientTickEvent.Pre event) { tickStarted = System.nanoTime(); }

    @SubscribeEvent(priority = net.neoforged.bus.api.EventPriority.LOWEST)
    public static void tickEnd(ClientTickEvent.Post event) { if (tickStarted != 0) { tickNanos += System.nanoTime() - tickStarted; } }

    private static void start(Minecraft minecraft) throws java.io.IOException {
        if (!(minecraft.screen instanceof TitleScreen)) { return; }
        Path file = minecraft.gameDirectory.toPath().resolve(PLAN);
        steps = List.of();
        if (!Files.isRegularFile(file)) { return; }
        List<String> lines = new ArrayList<>();
        for (String line : Files.readAllLines(file)) {
            String s = line.strip();
            if (!s.isEmpty() && !s.startsWith("#")) { lines.add(s); }
        }
        Files.delete(file);
        if (lines.isEmpty() || !lines.getFirst().startsWith("world ")) { throw new IllegalArgumentException("The plan must start with: world <folder>"); }
        String world = lines.getFirst().substring(6).strip();
        if (!world.startsWith(WORLD_PREFIX) || world.contains("/") || world.contains("\\") || !minecraft.getLevelSource().levelExists(world)) {
            throw new IllegalArgumentException("Autoshot opens only an existing QA world whose folder starts with " + WORLD_PREFIX + ": " + world);
        }
        LOGGER.info("AeroCore autoshot: {} steps in {}", lines.size() - 1, world);
        steps = lines;
        next = 1;
        pauseOnLostFocus = minecraft.options.pauseOnLostFocus;
        focusChanged = true;
        minecraft.options.pauseOnLostFocus = false;
        minecraft.createWorldOpenFlows().openWorld(world, () -> {
            LOGGER.error("AeroCore autoshot: could not open {}", world);
            steps = List.of();
            restoreFocus(minecraft);
        });
    }

    private static void run(Minecraft minecraft, String step) {
        String[] parts = step.split("\\s+", 2);
        String argument = parts.length > 1 ? parts[1] : "";
        switch (parts[0]) {
            case "size" -> {
                String[] size = argument.split("\\s+");
                minecraft.getWindow().setWindowed(Integer.parseInt(size[0]), Integer.parseInt(size[1]));
            }
            case "cmd" -> {
                IntegratedServer server = minecraft.getSingleplayerServer();
                if (server == null) { throw new IllegalStateException("No integrated server"); }
                server.execute(() -> server.getCommands().performPrefixedCommand(server.createCommandSourceStack(), argument));
            }
            case "wait" -> waiting = Integer.parseInt(argument);
            case "shot" -> {
                minecraft.options.hideGui = true;
                Screenshot.grab(minecraft.gameDirectory, "aerocore-" + argument + ".png", minecraft.getMainRenderTarget(),
                        message -> LOGGER.info("AeroCore autoshot: {}", message.getString()));
            }
            case "debug" -> WaveSurfaceRenderer.debugView = Integer.parseInt(argument);
            case "key" -> {
                String[] parts2 = argument.split("\\s+");
                net.minecraft.client.KeyMapping key = switch (parts2[0]) {
                    case "forward" -> minecraft.options.keyUp;
                    case "back" -> minecraft.options.keyDown;
                    case "left" -> minecraft.options.keyLeft;
                    case "right" -> minecraft.options.keyRight;
                    case "jump" -> minecraft.options.keyJump;
                    case "sneak" -> minecraft.options.keyShift;
                    default -> throw new IllegalArgumentException("Unknown key: " + parts2[0]);
                };
                boolean on = parts2[1].equals("on");
                key.setDown(on);
                if (on) { HELD.add(key); } else { HELD.remove(key); }
            }
            case "camera" -> minecraft.options.setCameraType(switch (argument) {
                case "back" -> net.minecraft.client.CameraType.THIRD_PERSON_BACK;
                case "front" -> net.minecraft.client.CameraType.THIRD_PERSON_FRONT;
                default -> net.minecraft.client.CameraType.FIRST_PERSON;
            });
            case "look" -> {
                String[] angles = argument.split("\\s+");
                if (minecraft.player != null) {
                    minecraft.player.setYRot(Float.parseFloat(angles[0])); minecraft.player.setXRot(Float.parseFloat(angles[1]));
                    minecraft.player.yRotO = minecraft.player.getYRot(); minecraft.player.xRotO = minecraft.player.getXRot();
                }
            }
            case "clouds" -> {
                if (savedClouds == null) { savedClouds = minecraft.options.cloudStatus().get(); }
                minecraft.options.cloudStatus().set(argument.equals("on") ? net.minecraft.client.CloudStatus.FANCY : net.minecraft.client.CloudStatus.OFF);
            }
            case "fps" -> {
                IntegratedServer server = minecraft.getSingleplayerServer();
                long worstTick = 0;
                if (server != null) { for (long t : server.getTickTimesNanos()) { worstTick = Math.max(worstTick, t); } }
                LOGGER.info("AeroCore autoshot: {} fps, slowest frame {} ms, {} frames over 50 ms, slowest server tick {} ms (last 100); "
                        + "region updates {}, sections remeshed {}; sea {}", minecraft.getFps(), String.format("%.1f", worstFrame / 1e6), slowFrames,
                        String.format("%.1f", worstTick / 1e6), ClientWaveRegions.updates, ClientWaveRegions.remeshed, WaveSurfaceRenderer.timing());
                worstFrame = 0; slowFrames = 0; ClientWaveRegions.updates = 0; ClientWaveRegions.remeshed = 0;
            }
            case "quit" -> leave(minecraft);
            default -> throw new IllegalArgumentException("Unknown autoshot step: " + step);
        }
    }

    /** As the pause screen's "Save and Quit to Title": the integrated server saves the world and stops. */
    private static void leave(Minecraft minecraft) {
        leaving = true;
        WaveSurfaceRenderer.debugView = 0;
        for (net.minecraft.client.KeyMapping key : HELD) { key.setDown(false); }
        HELD.clear();
        minecraft.options.setCameraType(net.minecraft.client.CameraType.FIRST_PERSON);
        if (savedClouds != null) { minecraft.options.cloudStatus().set(savedClouds); savedClouds = null; }
        minecraft.options.hideGui = false;
        minecraft.level.disconnect();
        minecraft.disconnect(new GenericMessageScreen(Component.literal("Saving")));
        minecraft.setScreen(new TitleScreen());
    }

    private static void finish(Minecraft minecraft) {
        leaving = false;
        steps = List.of();
        restoreFocus(minecraft);
        LOGGER.info("AeroCore autoshot: done");
        minecraft.stop();
    }

    private static void restoreFocus(Minecraft minecraft) {
        if (focusChanged) { minecraft.options.pauseOnLostFocus = pauseOnLostFocus; focusChanged = false; }
    }
}
