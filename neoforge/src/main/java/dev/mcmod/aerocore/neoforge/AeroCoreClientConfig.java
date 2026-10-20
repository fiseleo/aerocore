package dev.mcmod.aerocore.neoforge;

import net.neoforged.neoforge.common.ModConfigSpec;

/**
 * AeroCore's client settings (0.29, the first): how finished parts are shaded. AUTO uses AeroCore's part shader
 * (highlights, reflections) unless a shader pack is in use (Iris), SHADER always, VANILLA never (Minecraft's entity
 * shaders with the same textures). 0.39: beautified shading (client.Beautify), on by default; off is 0.38's look.
 */
public final class AeroCoreClientConfig {
    private AeroCoreClientConfig() {}

    public enum PartShading { AUTO, SHADER, VANILLA }

    public static final ModConfigSpec SPEC;
    public static final ModConfigSpec.EnumValue<PartShading> PART_SHADING;
    public static final ModConfigSpec.BooleanValue BEAUTIFY;

    static {
        ModConfigSpec.Builder b = new ModConfigSpec.Builder();
        PART_SHADING = b.comment("How finished vehicle parts are shaded: AUTO (AeroCore's part shader unless a shader pack is in use), SHADER, or VANILLA")
                .translation("aerocore.configuration.partShading")
                .defineEnum("partShading", PartShading.AUTO);
        BEAUTIFY = b.comment("Beautified shading (0.39): normal maps, gloss, reflective glass, finer occlusion, lamp halos, contact shadows; off draws as 0.38 did")
                .translation("aerocore.configuration.beautify")
                .define("beautify", true);
        SPEC = b.build();
    }
}
