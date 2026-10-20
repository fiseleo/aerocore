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
 * world &lt;folder&gt;     the QA world (required, first; or server)
 * server &lt;address&gt;   0.32: instead of world, joins a server on this computer (localhost or 127.0.0.1, an optional
 *                    :port) as the development player, for multiplayer checks; cmd needs a world, so use client
 * size &lt;w&gt; &lt;h&gt;       window size in pixels
 * monitor &lt;n&gt;       the window on the n-th monitor (1 the primary), centred; applied when the plan is read at the
 *                    title screen (before the world opens) and again after a size step (0.30: the game out of the way
 *                    of whoever is using the computer). 0.33.1: the development runs already open the window there
 *                    without focus (DevWindow), and their monitor is the default
 * cmd &lt;command&gt;     a command on the integrated server, without the slash
 * wait &lt;ticks&gt;
 * shot &lt;name&gt;        screenshots/aerocore-&lt;name&gt;.png with the GUI hidden
 * gui &lt;name&gt;         the same with the GUI and any open screen shown
 * client &lt;command&gt;  a command as the player types it, without the slash (client commands of mods run on the client)
 * debug &lt;view&gt;      the sea shader's development view (WaveSurfaceRenderer.debugView; 0 the picture)
 * fps                logs the frame rate of the last second, the slowest frame and server tick since the last fps
 * clouds &lt;on|off&gt;     draws clouds for this run (the player's setting comes back at the end)
 * key &lt;k&gt; &lt;on|off|tap&gt; holds or releases a movement key (forward, back, left, right, jump, sneak), e.g. to row a
 *                    boat, or (0.32) any key mapping by its name (key.hdfv.shift_up); tap presses it for two ticks
 *                    and counts one click
 * camera &lt;c&gt;         first, back or front person view
 * look &lt;yaw&gt; &lt;pitch&gt; turns the player's view (degrees, as /tp)
 * mark &lt;text&gt;       0.37: writes "AutoShot mark: text" to the log (to find what happened at that step, e.g. sounds)
 * quit               leave the world (the server saves it as usual) and close the game
 * </pre>
 * Lines starting with # are comments. The game is kept from pausing when its window loses focus while the plan runs,
 * and (0.33.1) it never keeps the mouse: steps work without focus, and a click into the window is let go at once.
 * Frames slower than 50 ms are logged on their own with where the time went (collector, JIT, the sea, the client tick).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class AutoShot {
    private static final Logger LOGGER = LogUtils.getLogger();
    private static final String PLAN = "aerocore-autoshot.txt", WORLD_PREFIX = "AeroCore";
    /** Null until the first title screen; empty when there is nothing (more) to do. */
    private static List<String> steps;
    private static int next, waiting;
    private static boolean entered, leaving, focusChanged, pauseOnLostFocus, remote;
    private static final java.util.Set<net.minecraft.client.KeyMapping> HELD = new java.util.HashSet<>();
    /** Keys a tap step pressed, and the ticks until they are let go. */
    private static final java.util.Map<net.minecraft.client.KeyMapping, Integer> TAPPED = new java.util.HashMap<>();
    private static net.minecraft.client.CloudStatus savedClouds;
    private static int framesSinceFps;
    /** The monitor the plan put the window on (1 the primary; 0 none); the development run's (DevWindow) unless the plan names one. */
    private static int monitor;
    private static long lastFrame, worstFrame, slowFrames, tickStarted, tickNanos, lastGcMillis, lastGcCount, lastJitMillis, lastAccept;

    private AutoShot() {}

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        if (FMLEnvironment.production) { return; }
        Minecraft minecraft = Minecraft.getInstance();
        try {
            if (steps == null) { start(minecraft); return; }
            // 0.33.1: the mouse stays with whoever uses the computer (a click into the window, or closing a screen
            // while the window has focus, grabs it)
            if (!steps.isEmpty() && minecraft.mouseHandler.isMouseGrabbed()) { minecraft.mouseHandler.releaseMouse(); }
            if (leaving) {
                if (minecraft.level == null) { finish(minecraft); }
                return;
            }
            if (entered && minecraft.level == null) {
                // 0.32: the server went away (stopped, or the connection dropped): close the game as quit would
                LOGGER.info("AeroCore autoshot: disconnected");
                steps = List.of();
                entered = false;
                finish(minecraft);
                return;
            }
            if (steps.isEmpty() || next >= steps.size()) { return; }
            if (!entered) {
                if (remote && minecraft.screen instanceof net.minecraft.client.gui.screens.DisconnectedScreen) {
                    // 0.32: the server could not be joined (not running, another address): close the game rather than wait
                    LOGGER.error("AeroCore autoshot: could not join the server");
                    steps = List.of();
                    finish(minecraft);
                    return;
                }
                if (minecraft.level == null || minecraft.player == null || minecraft.screen != null) { return; }
                entered = true;
            }
            // 0.32: a pause menu (opened when the window lost focus on a server) would cover every picture
            if (minecraft.screen instanceof net.minecraft.client.gui.screens.PauseScreen) { minecraft.setScreen(null); }
            // Held keys are pressed again every tick (losing focus releases them).
            for (net.minecraft.client.KeyMapping key : HELD) { key.setDown(true); }
            TAPPED.replaceAll((key, ticks) -> ticks - 1);
            TAPPED.entrySet().removeIf(e -> {
                if (e.getValue() > 0) { e.getKey().setDown(true); return false; }
                if (!HELD.contains(e.getKey())) { e.getKey().setDown(false); }
                return true;
            });
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
        framesSinceFps++;
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
            String s = line.replace("﻿", "").strip();   // a byte-order mark (PowerShell 5 writes one) is not text
            if (!s.isEmpty() && !s.startsWith("#")) { lines.add(s); }
        }
        Files.delete(file);
        remote = !lines.isEmpty() && lines.getFirst().startsWith("server ");
        if (lines.isEmpty() || !(remote || lines.getFirst().startsWith("world "))) {
            throw new IllegalArgumentException("The plan must start with: world <folder> (or server <address>)");
        }
        String world = lines.getFirst().substring(remote ? 7 : 6).strip();
        if (remote) {
            String host = world.contains(":") ? world.substring(0, world.lastIndexOf(':')) : world;
            if (!(host.equals("localhost") || host.equals("127.0.0.1")) || !net.minecraft.client.multiplayer.resolver.ServerAddress.isValidAddress(world)) {
                throw new IllegalArgumentException("Autoshot joins only a server on this computer (localhost or 127.0.0.1): " + world);
            }
        } else if (!world.startsWith(WORLD_PREFIX) || world.contains("/") || world.contains("\\") || !minecraft.getLevelSource().levelExists(world)) {
            throw new IllegalArgumentException("Autoshot opens only an existing QA world whose folder starts with " + WORLD_PREFIX + ": " + world);
        }
        LOGGER.info("AeroCore autoshot: {} steps in {}{}", lines.size() - 1, remote ? "the server at " : "", world);
        monitor = DevWindow.monitor();
        for (String line : lines) {
            if (line.startsWith("monitor ")) { monitor = Integer.parseInt(line.substring(8).strip()); }
        }
        if (monitor > 0) { toMonitor(minecraft, monitor); }
        steps = lines;
        next = 1;
        pauseOnLostFocus = minecraft.options.pauseOnLostFocus;
        focusChanged = true;
        minecraft.options.pauseOnLostFocus = false;
        if (remote) {
            net.minecraft.client.gui.screens.ConnectScreen.startConnecting(minecraft.screen, minecraft,
                    net.minecraft.client.multiplayer.resolver.ServerAddress.parseString(world),
                    new net.minecraft.client.multiplayer.ServerData("AeroCore autoshot", world, net.minecraft.client.multiplayer.ServerData.Type.OTHER), false, null);
            return;
        }
        minecraft.createWorldOpenFlows().openWorld(world, () -> {
            LOGGER.error("AeroCore autoshot: could not open {}", world);
            steps = List.of();
            restoreFocus(minecraft);
        });
    }

    /** Moves the game's window, centred, onto the n-th monitor (GLFW's order: the primary first); logs and leaves it where there is none. */
    private static void toMonitor(Minecraft minecraft, int n) {
        long window = minecraft.getWindow().getWindow();
        int[] w = new int[1], h = new int[1];
        org.lwjgl.glfw.GLFW.glfwGetWindowSize(window, w, h);
        int[] at = DevWindow.centredOn(n, w[0], h[0]);
        if (at == null) {
            LOGGER.warn("AeroCore autoshot: no monitor {} ({} found); the window stays where it is", n, DevWindow.monitors());
            return;
        }
        org.lwjgl.glfw.GLFW.glfwSetWindowPos(window, at[0], at[1]);   // SetWindowPos without activating
        LOGGER.info("AeroCore autoshot: the window on monitor {} of {}", n, DevWindow.monitors());
    }

    private static void run(Minecraft minecraft, String step) {
        String[] parts = step.split("\\s+", 2);
        String argument = parts.length > 1 ? parts[1] : "";
        switch (parts[0]) {
            case "size" -> {
                String[] size = argument.split("\\s+");
                minecraft.getWindow().setWindowed(Integer.parseInt(size[0]), Integer.parseInt(size[1]));
                if (monitor > 0) { toMonitor(minecraft, monitor); }
            }
            case "monitor" -> { }   // applied when the plan was read
            case "cmd" -> {
                IntegratedServer server = minecraft.getSingleplayerServer();
                if (server == null) { throw new IllegalStateException("cmd runs on a world's own server; on a server use: client <command> (as an operator)"); }
                server.execute(() -> server.getCommands().performPrefixedCommand(server.createCommandSourceStack(), argument));
            }
            case "wait" -> waiting = Integer.parseInt(argument);
            case "mark" -> LOGGER.info("AutoShot mark: {}", argument);   // 0.37: a line in the log to line events up with the plan
            case "shot" -> {
                minecraft.options.hideGui = true;
                Screenshot.grab(minecraft.gameDirectory, "aerocore-" + argument + ".png", minecraft.getMainRenderTarget(),
                        message -> LOGGER.info("AeroCore autoshot: {}", message.getString()));
            }
            case "gui" -> {
                minecraft.options.hideGui = false;
                Screenshot.grab(minecraft.gameDirectory, "aerocore-" + argument + ".png", minecraft.getMainRenderTarget(),
                        message -> LOGGER.info("AeroCore autoshot: {}", message.getString()));
            }
            case "client" -> {
                if (minecraft.player == null) { throw new IllegalStateException("No player"); }
                minecraft.player.connection.sendCommand(argument);
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
                    default -> java.util.Arrays.stream(minecraft.options.keyMappings).filter(k -> k.getName().equals(parts2[0])).findFirst()
                            .orElseThrow(() -> new IllegalArgumentException("Unknown key: " + parts2[0]));
                };
                switch (parts2[1]) {
                    case "on" -> { key.setDown(true); HELD.add(key); }
                    case "off" -> { key.setDown(false); HELD.remove(key); }
                    case "tap" -> { key.setDown(true); click(key); TAPPED.put(key, 2); }
                    default -> throw new IllegalArgumentException("key " + parts2[0] + " on, off or tap: " + parts2[1]);
                }
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
                int slowTicks = 0;
                if (server != null) {
                    for (long t : server.getTickTimesNanos()) { worstTick = Math.max(worstTick, t); if (t > 50_000_000L) { slowTicks++; } }
                }
                double meanTick = server == null ? 0 : server.getAverageTickTimeNanos() / 1e6;
                LOGGER.info("AeroCore autoshot: {} fps, slowest frame {} ms, {} frames over 50 ms, slowest server tick {} ms (last 100: mean {} ms, {} over 50 ms); "
                        + "region updates {}, sections remeshed {}; sea {}", minecraft.getFps(), String.format("%.1f", worstFrame / 1e6), slowFrames,
                        String.format("%.1f", worstTick / 1e6), String.format("%.1f", meanTick), slowTicks, ClientWaveRegions.updates, ClientWaveRegions.remeshed,
                        WaveSurfaceRenderer.timing());
                LOGGER.info("AeroCore autoshot: {}", PartStats.report(framesSinceFps));   // 0.29: drawing parts
                worstFrame = 0; slowFrames = 0; ClientWaveRegions.updates = 0; ClientWaveRegions.remeshed = 0; framesSinceFps = 0;
            }
            case "quit" -> leave(minecraft);
            default -> throw new IllegalArgumentException("Unknown autoshot step: " + step);
        }
    }

    /** One click of a key mapping (as a press counts it: KeyMapping.click goes by key, which several mappings may share). */
    private static void click(net.minecraft.client.KeyMapping key) {
        try {
            java.lang.reflect.Field count = net.minecraft.client.KeyMapping.class.getDeclaredField("clickCount");
            count.setAccessible(true);
            count.setInt(key, count.getInt(key) + 1);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("Cannot click " + key.getName(), e);
        }
    }

    /** As the pause screen's "Save and Quit to Title": the integrated server saves the world and stops (a server: leaves it). */
    private static void leave(Minecraft minecraft) {
        leaving = true;
        WaveSurfaceRenderer.debugView = 0;
        for (net.minecraft.client.KeyMapping key : HELD) { key.setDown(false); }
        HELD.clear();
        for (net.minecraft.client.KeyMapping key : TAPPED.keySet()) { key.setDown(false); }
        TAPPED.clear();
        minecraft.options.setCameraType(net.minecraft.client.CameraType.FIRST_PERSON);
        if (savedClouds != null) { minecraft.options.cloudStatus().set(savedClouds); savedClouds = null; }
        minecraft.options.hideGui = false;
        if (minecraft.level != null) { minecraft.level.disconnect(); }
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
