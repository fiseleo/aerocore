package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.AeroCoreClientConfig;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Beautified shading (0.39), the client setting "beautify" (on by default): everything 0.39 adds to how parts look
 * (finishes' beautify blocks with normal maps and gloss, reflective glass, occlusion refined at face centres, lens
 * halos, contact shadows, warm block light, the ground's colour from the biomes) is drawn only while it is on; off,
 * parts are drawn exactly as 0.38 drew them. Content mods ask {@link #on()} for their own beautified looks, and may
 * listen ({@link #onChange}) to drop what they keep built. A flip bakes the meshes again ({@link MeshBakery#refresh},
 * the old bakes drawn until the new ones are done).
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class Beautify {
    private Beautify() {}

    /** A development override (/aerocoredev parts beautify on|off|config, from the development tools mod), null: the setting. */
    public static volatile Boolean override;
    private static volatile Boolean last;
    private static volatile int generation;
    private static final List<Runnable> LISTENERS = new CopyOnWriteArrayList<>();

    /** Whether beautified shading is on (the setting, or the development override). */
    public static boolean on() {
        Boolean o = override;
        if (o != null) { return o; }
        try { return AeroCoreClientConfig.BEAUTIFY.get(); } catch (IllegalStateException notLoaded) { return true; }
    }

    /** Counts the flips (a cache keyed by it is rebuilt when it changes). */
    public static int generation() { return generation; }

    /** Runs on the client thread after every flip (meshes are already rebaking). */
    public static void onChange(Runnable r) { LISTENERS.add(r); }

    /** Checks for a flip once a tick (the settings screen, a file edit, the development command). */
    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        boolean now = on();
        Boolean was = last;
        last = now;
        if (was == null || was == now) { return; }
        generation++;
        MeshBakery.refresh();
        for (Runnable r : LISTENERS) { r.run(); }
    }
}
