package dev.mcmod.aerocore.devtools.mixin;

import com.mojang.blaze3d.platform.Window;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.devtools.DevWindow;
import org.lwjgl.glfw.GLFW;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Development only (DevWindow): the game's window is created hidden on the chosen monitor and shown there without
 * taking focus, so nothing appears on the primary monitor or pulls a player out of another game. Without the system
 * property the window opens as vanilla and NeoForge make it. 0.41: in the development tools mod (aerocore_devtools).
 */
@Mixin(Window.class)
abstract class DevWindowMixin {
    @Shadow @Final private long window;
    @Shadow private int windowedX;
    @Shadow private int windowedY;
    @Shadow private int windowedWidth;
    @Shadow private int windowedHeight;
    @Shadow private int x;
    @Shadow private int y;
    @Shadow private boolean fullscreen;
    @Shadow private boolean actuallyFullscreen;

    /** Hints for the window NeoForge creates next (glfwCreateWindow when its early window is off). */
    @Inject(method = "<init>", at = @At(value = "INVOKE", target = "Lnet/neoforged/fml/loading/ImmediateWindowHandler;setupMinecraftWindow(Ljava/util/function/IntSupplier;Ljava/util/function/IntSupplier;Ljava/util/function/Supplier;Ljava/util/function/LongSupplier;)J"))
    private void aerocore$hidden(CallbackInfo info) {
        int n = DevWindow.monitor();
        if (n == 0) { return; }
        // GLFW shows and focuses a window created full screen (on the primary monitor); the game's own toggle at
        // startup makes it full screen later on the monitor the window is on.
        fullscreen = actuallyFullscreen = false;
        GLFW.glfwWindowHint(GLFW.GLFW_VISIBLE, GLFW.GLFW_FALSE);
        GLFW.glfwWindowHint(GLFW.GLFW_FOCUSED, GLFW.GLFW_FALSE);
        GLFW.glfwWindowHint(GLFW.GLFW_FOCUS_ON_SHOW, GLFW.GLFW_FALSE);
        int[] at = DevWindow.centredOn(n, windowedWidth, windowedHeight);
        if (at != null) {
            GLFW.glfwWindowHint(GLFW.GLFW_POSITION_X, at[0]);
            GLFW.glfwWindowHint(GLFW.GLFW_POSITION_Y, at[1]);
        }
    }

    /** The position setMode applies (vanilla centres the window on the primary monitor). */
    @Inject(method = "<init>", at = @At(value = "INVOKE", target = "Lcom/mojang/blaze3d/platform/Window;setMode()V"))
    private void aerocore$place(CallbackInfo info) {
        int n = DevWindow.monitor();
        if (n == 0) { return; }
        int[] at = DevWindow.centredOn(n, windowedWidth, windowedHeight);
        if (at == null) {
            LogUtils.getLogger().warn("AeroCore dev window: no monitor {} ({} found); the window opens on the primary monitor, without focus", n, DevWindow.monitors());
            return;
        }
        x = windowedX = at[0];
        y = windowedY = at[1];
    }

    @Inject(method = "<init>", at = @At("TAIL"))
    private void aerocore$show(CallbackInfo info) {
        int n = DevWindow.monitor();
        if (n == 0) { return; }
        GLFW.glfwSetWindowAttrib(window, GLFW.GLFW_FOCUS_ON_SHOW, GLFW.GLFW_FALSE);
        if (GLFW.glfwGetWindowAttrib(window, GLFW.GLFW_VISIBLE) == GLFW.GLFW_TRUE) {
            // NeoForge's early window (earlyWindowControl in config/fml.toml) was already shown, with focus, on the primary monitor
            LogUtils.getLogger().warn("AeroCore dev window: NeoForge's early loading window opened it on the primary monitor with focus; set earlyWindowControl = false in config/fml.toml");
        } else {
            GLFW.glfwShowWindow(window);   // SW_SHOWNA: shown without being activated
        }
        LogUtils.getLogger().info("AeroCore dev window: on monitor {} of {} at {}, {}, focused {}", n, DevWindow.monitors(), x, y,
                GLFW.glfwGetWindowAttrib(window, GLFW.GLFW_FOCUSED) == GLFW.GLFW_TRUE);
    }
}
