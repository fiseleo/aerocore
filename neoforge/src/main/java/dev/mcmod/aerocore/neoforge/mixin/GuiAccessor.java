package dev.mcmod.aerocore.neoforge.mixin;

import net.minecraft.client.gui.Gui;
import net.minecraft.client.gui.components.SubtitleOverlay;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/** The subtitle list (0.37, client.sound.SoundHooks.subtitle): events inside a synthesized mix are shown through it. */
@Mixin(Gui.class)
public interface GuiAccessor {
    @Accessor("subtitleOverlay") SubtitleOverlay aerocore$subtitleOverlay();
}
