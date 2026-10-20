package dev.mcmod.aerocore.devtools.mixin;

import dev.mcmod.aerocore.devtools.DevWindow;
import net.minecraft.client.Minecraft;
import org.lwjgl.glfw.GLFW;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.ModifyArg;

/**
 * Development only (DevWindow): vanilla starts as if its new window had focus, so a world opened while someone works in
 * another window would grab the mouse (MouseHandler.grabMouse checks only that flag). The development window opens
 * without focus; the game starts knowing that, and the window's focus callback updates it as usual.
 */
@Mixin(Minecraft.class)
abstract class DevWindowFocusMixin {
    @ModifyArg(method = "<init>", at = @At(value = "INVOKE", target = "Lnet/minecraft/client/Minecraft;setWindowActive(Z)V"))
    private boolean aerocore$activeAsItIs(boolean active) {
        if (DevWindow.monitor() == 0) { return active; }
        return GLFW.glfwGetWindowAttrib(((Minecraft) (Object) this).getWindow().getWindow(), GLFW.GLFW_FOCUSED) == GLFW.GLFW_TRUE;
    }
}
