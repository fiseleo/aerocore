package dev.mcmod.aerocore.neoforge;

import net.neoforged.api.distmarker.Dist;
import net.neoforged.fml.ModContainer;
import net.neoforged.fml.common.Mod;
import net.neoforged.neoforge.client.gui.ConfigurationScreen;
import net.neoforged.neoforge.client.gui.IConfigScreenFactory;

/**
 * AeroCore's client-only entry point (0.39): the mod list's "Config" button opens NeoForge's settings screen for
 * aerocore-client.toml (part shading, beautified shading).
 */
@Mod(value = AeroCore.MOD_ID, dist = Dist.CLIENT)
public final class AeroCoreClient {
    public AeroCoreClient(ModContainer container) {
        container.registerExtensionPoint(IConfigScreenFactory.class, ConfigurationScreen::new);
    }
}
