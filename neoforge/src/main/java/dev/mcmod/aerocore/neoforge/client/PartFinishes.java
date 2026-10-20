package dev.mcmod.aerocore.neoforge.client;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.mesh.Finish;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.packs.resources.Resource;
import net.minecraft.server.packs.resources.ResourceManager;
import net.minecraft.server.packs.resources.ResourceManagerReloadListener;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RegisterClientReloadListenersEvent;
import org.slf4j.Logger;

import java.io.Reader;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * What each finish looks like (0.29): its tiling texture, gloss (0-1), metalness (0-1) and a tint. Read on every
 * resource reload from {@code assets/<namespace>/finishes.json} of every namespace, keyed by finish id:
 * <pre>
 * { "hdfv:brushed_steel": { "texture": "hdfv:textures/part/brushed_steel.png", "tile": 0.25, "gloss": 0.7, "metal": 1 } }
 * </pre>
 * 0.30 adds optional keys for the part shader: "clearcoat" (0-1, a lacquer's sharp second highlight and reflection
 * over the paint), "flip" (0-1, a pearl paint's colour shifting toward the grazing view), "cutout" (pixels whose alpha
 * is below it are not drawn: lettering, a grille) and "fresnel" (0-1, glass growing opaque and reflective toward
 * grazing views).
 * 0.38 adds opt-in keys (without them a finish draws exactly as in 0.37): "shading": 2 (apply "tint" and
 * "glossScale", a factor on the texture's gloss), "normal" (a tangent-space normal map PNG: red x along +u, green y
 * along +v, blue a cavity term; "normalStrength" scales it), "glass": "reflective" (the sky reflected on top of what
 * shows through, see the part shader) and "farGlass" (0-1, how bright such glass drawn opaque far away is relative
 * to its colour; 0.55 as before).
 * 0.39: "beautify", an object of any of these keys laid over the finish's own while beautified shading is on
 * ({@link Beautify}); off, the finish draws from its own keys (as in 0.38).
 * The tile in the file is the one the mesh code's {@link Finish} uses (kept here for checking). A quad without a
 * finish, or with one no file names, is drawn with {@link #PLAIN} (Minecraft's white concrete, as before 0.29).
 * <p>
 * 0.31: a mod may also {@link #define} finishes at run time, with a texture made in the game (a picture a player
 * uploaded, say); they last over resource reloads until {@link #undefine}d.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public final class PartFinishes {
    private PartFinishes() {}

    private static final Logger LOGGER = LogUtils.getLogger();

    /**
     * How a finish is drawn: its texture (the PNG) and surface. 0.38 (all off by default, so older finishes draw as
     * before): shading 2 applies the tint and glossScale; a normal map (its PNG) at a strength; reflective glass, and
     * the brightness of such glass drawn opaque far away (relative to its colour; MeshBaker darkens it to 0.55).
     */
    public record Look(ResourceLocation texture, double tileM, double gloss, double metal, int tint, double clearcoat, double flip, double cutout, double fresnel,
                       int shading, double glossScale, ResourceLocation normal, double normalStrength, boolean reflectiveGlass, double farGlass) {
        public Look(ResourceLocation texture, double tileM, double gloss, double metal, int tint) { this(texture, tileM, gloss, metal, tint, 0, 0, 0, 0); }

        public Look(ResourceLocation texture, double tileM, double gloss, double metal, int tint, double clearcoat, double flip, double cutout, double fresnel) {
            this(texture, tileM, gloss, metal, tint, clearcoat, flip, cutout, fresnel, 1, 1, null, 0, false, OPAQUE_GLASS);
        }
    }

    /** How bright MeshBaker draws glass made opaque (far away), relative to its colour (0.30). */
    public static final double OPAQUE_GLASS = 0.55;

    /** The look of quads without a finish (and of unknown ones). */
    public static final Look PLAIN = new Look(PartMesh.TEXTURE, Finish.DEFAULT_TILE_M, 0.3, 0, 0xffffffff);

    private static volatile Map<String, Look> looks = Map.of();
    /** 0.39: the files' finishes with their "beautify" objects laid over them (drawn while Beautify is on). */
    private static volatile Map<String, Look> beautified = Map.of();
    /** The files' finishes (as last read) and those defined at run time (0.31), which give looks together. */
    private static volatile Map<String, Look> files = Map.of();
    private static final Map<String, Look> RUNTIME = new ConcurrentHashMap<>();
    private static final Set<String> WARNED = ConcurrentHashMap.newKeySet();
    /** The finish textures made so far, by source PNG: the key they are registered under. */
    private static final Map<ResourceLocation, ResourceLocation> REGISTERED = new ConcurrentHashMap<>();

    public static Look look(Finish finish) {
        if (finish == null) { return PLAIN; }
        if (Beautify.on()) {
            Look b = beautified.get(finish.id());
            if (b != null) { return b; }
        }
        Look l = looks.get(finish.id());
        if (l == null) {
            if (WARNED.add(finish.id())) { LOGGER.warn("No finishes.json entry for finish {}; drawn plain", finish.id()); }
            return PLAIN;
        }
        return l;
    }

    /** Every finish read (for a mod's checks and the development log). */
    public static Map<String, Look> all() { return looks; }

    /**
     * The texture to bind for a look: its PNG registered as a {@link FinishTexture} under aerocore:finish/... the first
     * time (render thread).
     */
    public static ResourceLocation bindable(Look look) {
        return REGISTERED.computeIfAbsent(look.texture(), src -> {
            ResourceLocation key = ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "finish/" + src.getNamespace() + "/" + src.getPath());
            Minecraft.getInstance().getTextureManager().register(key, new FinishTexture(src));
            return key;
        });
    }

    /** 0.38: the normal map to bind for a look (registered like its texture, with mips), or null without one. */
    public static ResourceLocation bindableNormal(Look look) {
        if (look.normal() == null) { return null; }
        return REGISTERED.computeIfAbsent(look.normal(), src -> {
            ResourceLocation key = ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "finish/" + src.getNamespace() + "/" + src.getPath());
            Minecraft.getInstance().getTextureManager().register(key, new FinishTexture(src));
            return key;
        });
    }

    /**
     * 0.31: defines (or redefines) a finish at run time: its texture is one made in the game, registered with the
     * texture manager under look.texture() as it is (bindable gives that key; no PNG is read; a {@link RuntimeTexture}
     * keeps its picture over reloads). Meshes that name the id draw with it from the next frame: baked meshes keep only
     * the id, so nothing is baked again. A file's finish of the same id gives way to it. Use ids and texture keys of
     * your own namespace. Render thread.
     */
    public static void define(String id, Look look, net.minecraft.client.renderer.texture.AbstractTexture texture) {
        com.mojang.blaze3d.systems.RenderSystem.assertOnRenderThread();
        Look before = RUNTIME.put(id, look);
        if (before != null) {
            forget(before);
            if (!before.texture().equals(look.texture())) { Minecraft.getInstance().getTextureManager().release(before.texture()); }
        }
        Minecraft.getInstance().getTextureManager().register(look.texture(), texture);
        REGISTERED.put(look.texture(), look.texture());
        merge();
    }

    /**
     * 0.31: takes back a finish defined at run time and frees its texture (a file's finish of the same id comes back;
     * otherwise meshes that still name it draw plain, so keep it defined while they are drawn). Render thread.
     */
    public static void undefine(String id) {
        com.mojang.blaze3d.systems.RenderSystem.assertOnRenderThread();
        Look look = RUNTIME.remove(id);
        if (look == null) { return; }
        forget(look);
        Minecraft.getInstance().getTextureManager().release(look.texture());
        merge();
    }

    /** Whether a finish was defined at run time (and not taken back). */
    public static boolean defined(String id) { return RUNTIME.containsKey(id); }

    /** Drops what was made for a look: its texture's key and render types, its shader state. */
    private static void forget(Look look) {
        REGISTERED.remove(look.texture());
        if (look.normal() != null) { REGISTERED.remove(look.normal()); }
        PartRenderTypes.forget(look.texture());
        PartShader.forget(look);
    }

    private static void merge() {
        Map<String, Look> all = new HashMap<>(files);
        all.putAll(RUNTIME);
        looks = Map.copyOf(all);
    }

    static void read(ResourceManager manager) {
        Map<String, Look> out = new HashMap<>(), beautify = new HashMap<>();
        for (String namespace : manager.getNamespaces()) {
            Optional<Resource> file = manager.getResource(ResourceLocation.fromNamespaceAndPath(namespace, "finishes.json"));
            if (file.isEmpty()) { continue; }
            try (Reader in = file.get().openAsReader()) {
                JsonObject root = JsonParser.parseReader(in).getAsJsonObject();
                for (Map.Entry<String, JsonElement> e : root.entrySet()) {
                    JsonObject o = e.getValue().getAsJsonObject();
                    out.put(e.getKey(), parse(o));
                    // 0.39: "beautify": keys laid over the finish's own while beautified shading is on
                    if (o.has("beautify") && o.get("beautify").isJsonObject()) {
                        JsonObject over = o.deepCopy();
                        over.remove("beautify");
                        for (Map.Entry<String, JsonElement> k : o.getAsJsonObject("beautify").entrySet()) { over.add(k.getKey(), k.getValue()); }
                        beautify.put(e.getKey(), parse(over));
                    }
                }
            } catch (Exception ex) {
                LOGGER.error("Could not read {}:finishes.json", namespace, ex);
            }
        }
        files = Map.copyOf(out);
        beautified = Map.copyOf(beautify);
        merge();
        WARNED.clear();
        // the textures reload with the texture manager; keys stay valid
        MeshBakery.clear();
        LOGGER.info("Part finishes: {} read ({} with a beautified look)", out.size(), beautify.size());
    }

    private static Look parse(JsonObject o) {
        ResourceLocation texture = ResourceLocation.parse(o.get("texture").getAsString());
        double tile = o.has("tile") ? o.get("tile").getAsDouble() : Finish.DEFAULT_TILE_M;
        double gloss = o.has("gloss") ? o.get("gloss").getAsDouble() : 0.3, metal = o.has("metal") ? o.get("metal").getAsDouble() : 0;
        int tint = o.has("tint") ? (int) Long.parseLong(o.get("tint").getAsString().replace("#", ""), 16) | 0xff000000 : 0xffffffff;
        double clearcoat = o.has("clearcoat") ? o.get("clearcoat").getAsDouble() : 0, flip = o.has("flip") ? o.get("flip").getAsDouble() : 0;
        double cutout = o.has("cutout") ? o.get("cutout").getAsDouble() : 0, fresnel = o.has("fresnel") ? o.get("fresnel").getAsDouble() : 0;
        // 0.38: opt-in shading
        int shading = o.has("shading") ? o.get("shading").getAsInt() : 1;
        double glossScale = o.has("glossScale") ? o.get("glossScale").getAsDouble() : 1;
        ResourceLocation normal = o.has("normal") ? ResourceLocation.parse(o.get("normal").getAsString()) : null;
        double normalStrength = normal == null ? 0 : o.has("normalStrength") ? o.get("normalStrength").getAsDouble() : 1;
        boolean reflective = o.has("glass") && o.get("glass").getAsString().equals("reflective");
        double farGlass = o.has("farGlass") ? o.get("farGlass").getAsDouble() : OPAQUE_GLASS;
        return new Look(texture, tile, Math.clamp(gloss, 0, 1), Math.clamp(metal, 0, 1), tint, Math.clamp(clearcoat, 0, 1), Math.clamp(flip, 0, 1),
                Math.clamp(cutout, 0, 1), Math.clamp(fresnel, 0, 1), shading, Math.clamp(glossScale, 0, 4), normal, Math.clamp(normalStrength, 0, 4),
                reflective, Math.clamp(farGlass, 0, 1));
    }

    @SubscribeEvent public static void register(RegisterClientReloadListenersEvent event) {
        event.registerReloadListener((ResourceManagerReloadListener) PartFinishes::read);
    }

    /** Finishes named in mesh code that no file names (for a development check). */
    public static Set<String> missing(Iterable<Finish> used) {
        Set<String> out = new HashSet<>();
        for (Finish f : used) { if (!looks.containsKey(f.id())) { out.add(f.id()); } }
        return out;
    }
}
