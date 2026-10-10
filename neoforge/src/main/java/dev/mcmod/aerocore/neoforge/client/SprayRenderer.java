package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.BowSpray;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.probe.SeaSurface;
import dev.mcmod.aerocore.neoforge.probe.Wakes;
import dev.mcmod.aerocore.neoforge.probe.WaveRegion;
import net.minecraft.client.Camera;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.texture.TextureAtlasSprite;
import net.minecraft.core.BlockPos;
import net.minecraft.util.RandomSource;
import net.minecraft.world.inventory.InventoryMenu;
import net.minecraft.world.level.material.Fluids;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import net.neoforged.neoforge.client.event.RenderLevelStageEvent;
import net.neoforged.neoforge.client.extensions.common.IClientFluidTypeExtensions;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Bow spray of moving wake sources (Wakes.moving): where BowSpray says the bow wave breaks, droplets leave both
 * sides of the bow with the BowSpray velocity (plus the source's own) and fly under real gravity and quadratic air drag
 * toward the local wind (the sea region's log profile; still air elsewhere), g |w - v| (w - v) / v_t^2, which makes a
 * drop's fall speed in still air tend to its terminal speed v_t, until they fall back through the still surface. The
 * drops are taken as about 1 mm across (v_t = 3.9 m/s, Gunn and Kinzer 1949), scattered between 0.5 and 2 mm
 * (2.1-6.5 m/s) for display: bow-spray drop sizes are not modelled. How many droplets are drawn (a few per side per
 * tick, with +-15 % scatter in launch speed) is a display budget, not a spray mass flux.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class SprayRenderer {
    private static final int MAX_DROPLETS = 1500, PER_SIDE = 3, MAX_AGE_TICKS = 80;
    private static final double MIN_CREST_M = 0.03, SIZE_M = 0.09;
    private static final double G = Atmosphere.GRAVITY_MPS2, DT = MinecraftUnits.TICK_SECONDS;
    /** Terminal fall speeds of 0.5, 1 and 2 mm water drops in air, m/s (Gunn and Kinzer 1949). */
    private static final double FALL_SMALL = 2.06, FALL_MEDIUM = 3.89, FALL_LARGE = 6.49;
    private static final int SUBSTEPS = 4;
    private static final class Droplet {
        double x, y, z, px, py, pz, vx, vy, vz, floor, fall;
        int age;
    }
    private static final List<Droplet> DROPLETS = new ArrayList<>();
    private static final RandomSource RANDOM = RandomSource.create();

    private SprayRenderer() {}

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        ClientLevel level = Minecraft.getInstance().level;
        if (level == null) { DROPLETS.clear(); return; }
        if (Minecraft.getInstance().isPaused()) { return; }
        List<WaveRegion> regions = ClientWaveRegions.current();
        for (Iterator<Droplet> it = DROPLETS.iterator(); it.hasNext(); ) {
            Droplet d = it.next();
            d.px = d.x; d.py = d.y; d.pz = d.z;
            WaveRegion region = SeaSurface.regionAt(regions, d.x, d.z);
            Vec3d wind = region == null ? Vec3d.ZERO : region.wind().at(d.y - region.stillWaterY()).scale(region.weight(d.x, d.z));
            // Gravity and quadratic drag toward the wind, semi-implicit in the drag (stable at any relative speed).
            double h = DT / SUBSTEPS;
            for (int k = 0; k < SUBSTEPS; k++) {
                double rx = wind.x() - d.vx, ry = -d.vy, rz = wind.z() - d.vz, rate = G * Math.sqrt(rx * rx + ry * ry + rz * rz) / (d.fall * d.fall);
                double keep = 1 / (1 + rate * h);
                d.vx = (d.vx + rate * h * wind.x()) * keep; d.vz = (d.vz + rate * h * wind.z()) * keep; d.vy = (d.vy - G * h) * keep;
                d.x += d.vx * h; d.y += d.vy * h; d.z += d.vz * h;
            }
            if (++d.age > MAX_AGE_TICKS || (d.vy < 0 && d.y < d.floor)) { it.remove(); }
        }
        for (Wakes.Moving m : Wakes.moving(level)) {
            Wakes.Bow bow = m.design().bow(m.speed());
            BowSpray spray = bow.spray();
            if (!spray.breaking() || spray.crestHeightM() < MIN_CREST_M) { continue; }
            double bx = m.originX() + m.dirX() * bow.forwardM(), bz = m.originZ() + m.dirZ() * bow.forwardM();
            for (int side = -1; side <= 1; side += 2) {
                // Outward normal of this side: the motion direction turned by 90 degrees.
                double nx = -m.dirZ() * side, nz = m.dirX() * side;
                for (int i = 0; i < PER_SIDE && DROPLETS.size() < MAX_DROPLETS; i++) {
                    double scatter = 1 + 0.15 * (2 * RANDOM.nextDouble() - 1), along = RANDOM.nextDouble();
                    Droplet d = new Droplet();
                    d.x = bx + nx * bow.halfWidthM() - m.dirX() * along * 0.3; d.z = bz + nz * bow.halfWidthM() - m.dirZ() * along * 0.3;
                    d.y = m.stillY(); d.floor = m.stillY();
                    d.px = d.x; d.py = d.y; d.pz = d.z;
                    d.vx = (m.dirX() * spray.forwardMps() + nx * spray.lateralMps()) * scatter;
                    d.vz = (m.dirZ() * spray.forwardMps() + nz * spray.lateralMps()) * scatter;
                    d.vy = spray.verticalMps() * scatter;
                    double size = RANDOM.nextDouble();
                    d.fall = size < 0.5 ? FALL_SMALL + (FALL_MEDIUM - FALL_SMALL) * 2 * size : FALL_MEDIUM + (FALL_LARGE - FALL_MEDIUM) * (2 * size - 1);
                    DROPLETS.add(d);
                }
            }
        }
    }

    @SubscribeEvent public static void render(RenderLevelStageEvent event) {
        if (event.getStage() != RenderLevelStageEvent.Stage.AFTER_TRANSLUCENT_BLOCKS || DROPLETS.isEmpty()) { return; }
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        if (level == null) { return; }
        Camera camera = event.getCamera();
        Vec3 eye = camera.getPosition();
        Vector3f left = camera.getLeftVector(), up = camera.getUpVector();
        float partial = event.getPartialTick().getGameTimeDeltaPartialTick(false);
        MultiBufferSource.BufferSource buffers = minecraft.renderBuffers().bufferSource();
        VertexConsumer out = buffers.getBuffer(RenderType.translucent());
        TextureAtlasSprite sprite = minecraft.getTextureAtlas(InventoryMenu.BLOCK_ATLAS)
                .apply(IClientFluidTypeExtensions.of(Fluids.WATER).getStillTexture());
        float u0 = sprite.getU(0.4f), u1 = sprite.getU(0.6f), v0 = sprite.getV(0.4f), v1 = sprite.getV(0.6f);
        float h = (float) (SIZE_M / 2);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        for (Droplet d : DROPLETS) {
            double x = d.px + (d.x - d.px) * partial - eye.x, y = d.py + (d.y - d.py) * partial - eye.y, z = d.pz + (d.z - d.pz) * partial - eye.z;
            if (x * x + y * y + z * z > 96 * 96) { continue; }
            pos.set(d.x, d.y, d.z);
            int light = LevelRenderer.getLightColor(level, pos);
            float lx = left.x() * h, ly = left.y() * h, lz = left.z() * h, ux = up.x() * h, uy = up.y() * h, uz = up.z() * h;
            vertex(out, x + lx - ux, y + ly - uy, z + lz - uz, u0, v1, light);
            vertex(out, x + lx + ux, y + ly + uy, z + lz + uz, u0, v0, light);
            vertex(out, x - lx + ux, y - ly + uy, z - lz + uz, u1, v0, light);
            vertex(out, x - lx - ux, y - ly - uy, z - lz - uz, u1, v1, light);
        }
        buffers.endBatch(RenderType.translucent());
    }

    private static void vertex(VertexConsumer out, double x, double y, double z, float u, float v, int light) {
        out.addVertex((float) x, (float) y, (float) z).setColor(0.92f, 0.96f, 1f, 0.85f).setUv(u, v).setLight(light).setNormal(0, 1, 0);
    }
}
