package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.AeroCoreClientConfig;
import net.minecraft.client.renderer.RenderStateShard;
import net.minecraft.client.renderer.ShaderInstance;
import net.minecraft.resources.ResourceLocation;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.ModList;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RegisterShadersEvent;
import org.slf4j.Logger;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * AeroCore's part shader (0.29, shaders/core/rendertype_aerocore_part): highlights and reflections on finished parts.
 * Used when the client setting allows it and the shader loaded; AUTO steps back to Minecraft's entity shaders while
 * an Iris shader pack is in use (a pack lights entities its own way). A shader that fails to load logs one error and
 * leaves the vanilla look.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public final class PartShader {
    private PartShader() {}

    private static final Logger LOGGER = LogUtils.getLogger();
    private static volatile ShaderInstance shader;
    static final RenderStateShard.ShaderStateShard STATE = new RenderStateShard.ShaderStateShard(() -> shader);
    private static final Map<PartFinishes.Look, RenderStateShard.TexturingStateShard> SURFACES = new ConcurrentHashMap<>();

    @SubscribeEvent public static void register(RegisterShadersEvent event) {
        try {
            event.registerShader(new ShaderInstance(event.getResourceProvider(), ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "rendertype_aerocore_part"),
                    DefaultVertexFormat.NEW_ENTITY), loaded -> { shader = loaded; PartRenderTypes.clear(); });
        } catch (java.io.IOException | RuntimeException e) {
            LOGGER.error("AeroCore part shader failed to load; parts are drawn with Minecraft's entity shaders", e);
            shader = null;
            PartRenderTypes.clear();
        }
    }

    /** Whether parts are drawn with this shader now. */
    public static boolean active() {
        if (shader == null) { return false; }
        return switch (AeroCoreClientConfig.PART_SHADING.get()) {
            case VANILLA -> false;
            case SHADER -> true;
            case AUTO -> !irisPackInUse();
        };
    }

    /** The per-finish state: metalness and gloss scale set before each batch, with the frame's sun and sky. */
    static RenderStateShard.TexturingStateShard surface(PartFinishes.Look look) {
        return SURFACES.computeIfAbsent(look, l -> new RenderStateShard.TexturingStateShard("aerocore_part_surface", () -> {
            ShaderInstance s = shader;
            if (s == null) { return; }
            boolean opted = l.shading() >= 2;
            s.safeGetUniform("PartSurface").set((float) l.metal(), opted ? (float) l.glossScale() : 1.0f, (float) l.clearcoat(), (float) l.flip());
            s.safeGetUniform("PartSurface2").set((float) l.cutout(), (float) l.fresnel(), 0f, 0f);
            // 0.38: the opt-in shading (identity values for every finish that does not ask)
            ResourceLocation normal = PartFinishes.bindableNormal(l);
            if (normal != null) { com.mojang.blaze3d.systems.RenderSystem.setShaderTexture(3, normal); }
            s.safeGetUniform("PartSurface3").set(normal == null ? 0f : (float) l.normalStrength(), l.reflectiveGlass() ? 1f : 0f,
                    (float) (l.farGlass() / PartFinishes.OPAQUE_GLASS), 0f);
            int tint = opted ? l.tint() : 0xffffffff;
            s.safeGetUniform("PartTint").set(((tint >> 16) & 255) / 255f, ((tint >> 8) & 255) / 255f, (tint & 255) / 255f);
            PartLighting.apply(s);
        }, () -> {}));
    }

    /** 0.31: forgets a look's state (a finish defined at run time and taken back). */
    static void forget(PartFinishes.Look look) { SURFACES.remove(look); }

    private static Boolean irisPresent;
    private static java.lang.reflect.Method irisInstance, irisInUse;

    /** Whether an Iris shader pack is being used (asked by reflection; false without Iris). */
    static boolean irisPackInUse() {
        if (irisPresent == null) {
            irisPresent = ModList.get() != null && (ModList.get().isLoaded("iris") || ModList.get().isLoaded("oculus"));
            if (irisPresent) {
                try {
                    Class<?> api = Class.forName("net.irisshaders.iris.api.v0.IrisApi");
                    irisInstance = api.getMethod("getInstance");
                    irisInUse = api.getMethod("isShaderPackInUse");
                } catch (ReflectiveOperationException e) {
                    irisInstance = null;
                }
            }
        }
        if (!irisPresent) { return false; }
        if (irisInstance == null) { return true; }   // a pack mod we cannot ask: stay with vanilla
        try { return (Boolean) irisInUse.invoke(irisInstance.invoke(null)); }
        catch (ReflectiveOperationException | RuntimeException e) { return true; }
    }
}
