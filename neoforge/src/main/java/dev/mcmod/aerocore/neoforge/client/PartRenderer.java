package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.neoforge.vehicle.PartVehicle;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * How PartVehicleRenderer draws one kind of compiled part, in the part's frame (x forward, y up, z starboard; the pose
 * already at the part's mount). Register one per Part class (or PartDesign class) with register; AeroCore registers
 * its own part classes.
 * <p>
 * 0.29: the static mesh is baked on a background thread (MeshBakery: finishes, ambient occlusion, levels of detail,
 * enclosed faces left out) as {@link #bake} says, and drawn textured by finish; moving pieces may draw into
 * {@link PartLayers} (textured, several passes, glow) by overriding the PartLayers form of animate. The
 * VertexConsumer form still works: by default the PartLayers form records what it writes.
 */
public interface PartRenderer {
    /** Quads that do not move with inputs or rotors; built once per compiled vehicle (0.29: on a background thread: no world or GL). */
    default void mesh(Part part, PartMesh out) {}

    /** 0.29: how the static mesh is baked: default finish, occlusion, levels, culling of what cannot be seen. */
    default MeshBakery.Spec bake(Part part) { return MeshBakery.Spec.WORLD; }

    /** Moving pieces drawn every frame (rotors, deflecting surfaces); firstRotor indexes the part's first rotor in the vehicle. */
    default void animate(Part part, PartVehicle entity, String partId, int firstRotor, float partialTick, PoseStack stack, VertexConsumer out, int light) {}

    /**
     * 0.29: moving pieces drawn into layers (finished meshes, glow). By default the VertexConsumer form, recorded
     * through layers.legacy(); override this one to draw textured.
     */
    default void animate(Part part, PartVehicle entity, String partId, int firstRotor, float partialTick, PoseStack stack, PartLayers layers, int light) {
        animate(part, entity, partId, firstRotor, partialTick, stack, layers.legacy(), light);
    }

    /**
     * 0.30: what of a part's bake is drawn this frame: the bake (with its moving pieces, MeshBuilder.piece) and the
     * level of detail its fixed part was drawn at. A renderer draws {@link #piece} under each piece's pose, so moving
     * pieces are baked once (finishes, occlusion, the GPU cache) instead of built every frame.
     */
    record Drawn(MeshBakery.Baked baked, int level, float[] lampLevels) {
        public Drawn(MeshBakery.Baked baked, int level) { this(baked, level, new float[baked.lamps().size()]); }

        /** A piece at the drawn level (an empty mesh where there is none). */
        public dev.mcmod.aerocore.api.mesh.PackedMesh piece(int p) { return baked.piece(p, level); }

        /** How many pieces, the fixed part (0) included. */
        public int pieces() { return baked.pieceCount(); }
    }

    /**
     * 0.30: how bright each of the bake's lamps is now (Lamp; in the order of the bake's list): 0 off, 1 on, above 1 a
     * flash. Asked every frame; the rules (lights switched on, braking, a beacon's flashes) are the renderer's.
     */
    default void lamps(Part part, PartVehicle entity, String partId, float partialTick, java.util.List<dev.mcmod.aerocore.api.mesh.Lamp> lamps, float[] level) {}

    /**
     * 0.30: moving pieces drawn into layers, with the part's bake (null while it is being baked). By default the
     * PartLayers form above; a renderer that bakes its pieces overrides this one.
     */
    default void animate(Part part, PartVehicle entity, String partId, int firstRotor, float partialTick, PoseStack stack, PartLayers layers, int light, Drawn drawn) {
        animate(part, entity, partId, firstRotor, partialTick, stack, layers, light);
    }

    /**
     * 0.29: light the static mesh's hot faces give off now (those built with the hot pen: a nozzle, an exhaust
     * manifold), as an ARGB added over them (alpha its strength; 0 for none): a colour from Blackbody at the metal's
     * temperature, say. Asked every frame; the part's temperature is the renderer's to keep.
     */
    default int glow(Part part, PartVehicle entity, String partId, float partialTick) { return 0; }

    Map<Class<?>, PartRenderer> REGISTRY = new ConcurrentHashMap<>();
    /** 0.30: looks asked for last (AeroCore's own for its parts), so any registration by a mod wins, by part or design class. */
    Map<Class<?>, PartRenderer> DEFAULTS = new ConcurrentHashMap<>();

    /** Registers how to draw a Part class, or the parts of a PartDesign class (replacing any earlier registration for it). */
    static void register(Class<?> partOrDesign, PartRenderer renderer) { REGISTRY.put(partOrDesign, renderer); }

    /** 0.30: registers a look used only when nothing else is registered for a part (by its class or its design's). */
    static void registerDefault(Class<?> partOrDesign, PartRenderer renderer) { DEFAULTS.put(partOrDesign, renderer); }

    /**
     * The renderer for a part: by its class (or superclass), else by its design's class, else (0.30) a default by
     * either; null draws nothing.
     */
    static PartRenderer of(Part part) {
        for (Map<Class<?>, PartRenderer> table : List.of(REGISTRY, DEFAULTS)) {
            for (Class<?> c = part.getClass(); c != null; c = c.getSuperclass()) {
                PartRenderer r = table.get(c);
                if (r != null) { return r; }
            }
            for (Class<?> c = part.design().getClass(); c != null; c = c.getSuperclass()) {
                PartRenderer r = table.get(c);
                if (r != null) { return r; }
            }
        }
        return null;
    }
}
