package dev.mcmod.aerocore.neoforge.client;

import net.neoforged.fml.loading.FMLEnvironment;
import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFW;

/**
 * The development game window out of the way of whoever is using the computer (0.33.1; the user 2026-10-18, while
 * playing a full-screen game on the primary monitor: the development Minecraft should open on the second monitor and
 * not take the mouse). With -Daerocore.window.monitor=n (the client runs of AeroCore and hdfv set 2) the window is
 * created hidden, centred on the n-th monitor and shown without taking focus (DevWindowMixin), and the game starts
 * knowing it has no focus (DevWindowFocusMixin), so it grabs the mouse only once someone clicks into it. NeoForge's
 * early loading window opens on the primary monitor and takes focus before any mod runs, so those runs turn it off
 * (earlyWindowControl in config/fml.toml, written by the run tasks). Inert in production.
 */
public final class DevWindow {
    /** The system property naming the monitor (1 the primary; 0 or absent: the game opens as usual). */
    public static final String MONITOR = "aerocore.window.monitor";
    /** Room above the content area for the window's title bar, in screen pixels. */
    private static final int TITLE_BAR = 30;

    private DevWindow() {}

    /** The monitor the development window opens on (1 the primary, GLFW's order), or 0 when the game opens as usual. */
    public static int monitor() {
        if (FMLEnvironment.production) { return 0; }
        Integer n = Integer.getInteger(MONITOR, 0);
        return Math.max(0, n);
    }

    /**
     * Where the content area of a window this size sits, centred in the work area of the n-th monitor (GLFW's order,
     * the primary first), the title bar kept on it; null when there is no such monitor.
     */
    public static int[] centredOn(int n, int width, int height) {
        PointerBuffer monitors = GLFW.glfwGetMonitors();
        if (monitors == null || n < 1 || n > monitors.limit()) { return null; }
        int[] x = new int[1], y = new int[1], w = new int[1], h = new int[1];
        GLFW.glfwGetMonitorWorkarea(monitors.get(n - 1), x, y, w, h);
        if (w[0] <= 0 || h[0] <= 0) { return null; }
        return new int[] {x[0] + Math.max(0, (w[0] - width) / 2), y[0] + Math.max(TITLE_BAR, (h[0] - height) / 2)};
    }

    /** How many monitors GLFW sees. */
    public static int monitors() {
        PointerBuffer monitors = GLFW.glfwGetMonitors();
        return monitors == null ? 0 : monitors.limit();
    }
}
