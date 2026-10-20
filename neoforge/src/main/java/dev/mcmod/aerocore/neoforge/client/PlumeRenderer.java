package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.engine.Plume;
import dev.mcmod.aerocore.api.mesh.Finish;
import dev.mcmod.aerocore.api.mesh.Mesh;
import dev.mcmod.aerocore.api.mesh.MeshBaker;
import dev.mcmod.aerocore.api.mesh.MeshBuilder;
import dev.mcmod.aerocore.api.mesh.PackedMesh;

import java.util.List;

/**
 * Draws an exhaust plume as added light (0.29), from {@link Plume}'s shape and look: an envelope swelling past the
 * exit and closing to a point, a brighter core inside it as long as the potential core, shock diamonds at the cell
 * spacing fading downstream, and the Mach disk of a strongly under-expanded jet. The texture (a mod's flame noise
 * finish; plain without one) scrolls downstream and the length flickers a little. Along -x from an exit at (x, 0, 0)
 * in the pose's frame. A few hundred quads, built each frame.
 */
public final class PlumeRenderer {
    private PlumeRenderer() {}

    private static final int AROUND = 20;

    /**
     * A plume at a level (0-1: idle to full power or burning) and a time (seconds, for flicker and scrolling), from an
     * exit at x of a radius.
     */
    public static void draw(PartLayers layers, PoseStack stack, Plume.Shape shape, Plume.Look look, Finish texture, double exitX, double exitRadius,
                            double level, double seconds) {
        if (!(level > 0.02)) { return; }
        double r = exitRadius, jet = Math.max(0.5 * shape.jetDiameterM(), 0.9 * r);
        double flicker = 1 + 0.05 * Math.sin(seconds * 37) + 0.03 * Math.sin(seconds * 91 + 1.3);
        double length = look.visibleDiameters() * 2 * r * (0.35 + 0.65 * level) * flicker;
        Mesh m = new Mesh().finish(texture);
        Vec3d exit = new Vec3d(exitX, 0, 0), tip = exit.add(new Vec3d(-length, 0, 0));
        int envelope = faded(look.envelope(), level), envelopeTip = envelope & 0x00ffffff;
        m.lathe(exit, tip, new double[]{0, 0.3, 1}, new double[]{0.95 * r, 1.25 * jet, 0.02 * r}, new int[]{envelope, envelope, envelopeTip}, AROUND, envelope, false);
        double core = Math.min(0.6 * length, Math.max(shape.coreLengthM(), 0.25 * length));
        // the layers add up (both faces of each, the core inside the envelope, the cells inside both): kept below white,
        // where they would saturate to yellow whatever their colour and turn olive in the fog
        int coreColour = faded(look.core(), 0.7 * level);
        m.lathe(exit.add(new Vec3d(-0.005 * r, 0, 0)), exit.add(new Vec3d(-core, 0, 0)), new double[]{0, 0.25, 1}, new double[]{0.6 * r, 0.7 * jet, 0.01 * r},
                new int[]{coreColour, coreColour, coreColour & 0x00ffffff}, AROUND, coreColour, false);
        if (shape.hasCells() && look.diamonds() != 0) {
            double strength = shape.cellStrength() * level;
            for (int k = 1; k <= 8; k++) {
                double x = k * shape.cellSpacingM();
                if (x > 0.8 * length) { break; }
                int alpha = (int) Math.round(0.6 * ((look.diamonds() >>> 24) & 255) * strength * Math.pow(0.78, k - 1));
                if (alpha < 6) { break; }
                int c = (alpha << 24) | (look.diamonds() & 0xffffff);
                Vec3d at = exit.add(new Vec3d(-x, 0, 0));
                double half = 0.28 * shape.cellSpacingM(), radius = 0.42 * jet * Math.pow(0.93, k);
                m.lathe(at.add(new Vec3d(half, 0, 0)), at.add(new Vec3d(-half, 0, 0)), new double[]{0, 0.5, 1}, new double[]{0, radius, 0}, null, 12, c, false);
            }
        }
        if (shape.hasMachDisk() && shape.machDiskM() < length) {
            int disk = faded(look.diamonds() != 0 ? look.diamonds() : look.core(), level);
            Vec3d at = exit.add(new Vec3d(-shape.machDiskM(), 0, 0));
            m.cylinder(at.add(new Vec3d(0.02 * r, 0, 0)), at.add(new Vec3d(-0.02 * r, 0, 0)), shape.machDiskDiameterM() / 2, shape.machDiskDiameterM() / 2, 16, disk);
        }
        scroll(m, seconds * 2.5 * Math.max(texture == null ? Finish.DEFAULT_TILE_M : texture.tileM(), 1e-3));
        layers.glow(MeshBaker.bake(m, texture), stack, 0xffffffff, false);
    }

    /** A colour's alpha scaled by a level. */
    static int faded(int argb, double level) {
        int a = (int) Math.round(((argb >>> 24) & 255) * Math.max(0, Math.min(1, level)));
        return (a << 24) | (argb & 0xffffff);
    }

    /** Moves texture coordinates along the run (v) by a distance, so the texture flows downstream. */
    private static void scroll(Mesh m, double by) {
        List<MeshBuilder.Quad> quads = m.quads();
        for (int i = 0; i < quads.size(); i++) {
            MeshBuilder.Quad q = quads.get(i);
            if (q.uv() == null) { continue; }
            float[] uv = q.uv().clone();
            for (int k = 1; k < 8; k += 2) { uv[k] -= (float) by; }
            quads.set(i, new MeshBuilder.Quad(q.a(), q.b(), q.c(), q.d(), q.color(), q.normal(), q.normals(), q.colors(), uv, q.finish(), q.tags()));
        }
    }

    /** A packed mesh for a finished glow (exposed for callers that keep one, e.g. a pulsing flame they shake). */
    public static PackedMesh bake(Mesh m, Finish texture) { return MeshBaker.bake(m, texture); }
}
