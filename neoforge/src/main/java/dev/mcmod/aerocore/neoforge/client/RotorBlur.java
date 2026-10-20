package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.PoseStack;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.mesh.Finish;
import dev.mcmod.aerocore.api.mesh.Mesh;
import dev.mcmod.aerocore.api.mesh.MeshBaker;

/**
 * The disc a fast propeller or rotor seems to be (0.29): above a speed the eye no longer follows the blades, so they
 * are drawn as a see-through disc in the plane square to x, darker toward the tips' track where the blades sweep
 * fastest, as dense as the blades cover the disc (solidity). Below the speed, draw the blades instead; between, a
 * renderer may draw both, the disc fading in.
 */
public final class RotorBlur {
    private RotorBlur() {}

    /** Turns per second above which blades blur fully into a disc, and below which no disc shows. */
    public static final double FULL = 12, NONE = 4;

    /** How much of the disc shows at a speed (turns per second): 0 below NONE, 1 above FULL. */
    public static double amount(double turnsPerSecond) {
        double t = (Math.abs(turnsPerSecond) - NONE) / (FULL - NONE);
        return Math.max(0, Math.min(1, t));
    }

    /**
     * A disc from a hub radius to a tip radius at x = 0 in the pose's frame, of a colour (the blades'), its alpha the
     * solidity times how blurred the blades are.
     */
    public static void draw(PartLayers layers, PoseStack stack, double hubRadius, double tipRadius, double solidity, int colour, double turnsPerSecond,
                            Finish finish, int light) {
        double amount = amount(turnsPerSecond);
        if (amount <= 0 || tipRadius <= hubRadius) { return; }
        int inner = withAlpha(colour, (int) Math.round(255 * Math.min(0.8, solidity * 1.2) * amount));
        int outer = withAlpha(colour, (int) Math.round(255 * Math.min(0.6, solidity * 0.8) * amount));
        Mesh m = new Mesh().finish(finish);
        // a thin lathe (both faces): hub ring to the tip, fading at the very tip
        m.lathe(new Vec3d(0.002, 0, 0), new Vec3d(-0.002, 0, 0), new double[]{0, 0, 1, 1}, new double[]{hubRadius, tipRadius, tipRadius, hubRadius},
                new int[]{inner, outer & 0x00ffffff, outer & 0x00ffffff, inner}, 32, inner, false);
        layers.draw(MeshBaker.bake(m, finish), stack, light, 0xffffffff, true, PartLayers.TRANSLUCENT);
    }

    private static int withAlpha(int argb, int alpha) { return (Math.max(0, Math.min(255, alpha)) << 24) | (argb & 0xffffff); }
}
