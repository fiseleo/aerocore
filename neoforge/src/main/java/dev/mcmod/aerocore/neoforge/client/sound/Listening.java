package dev.mcmod.aerocore.neoforge.client.sound;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.api.acoustics.Propagation;
import dev.mcmod.aerocore.api.acoustics.RoomAcoustics;
import dev.mcmod.aerocore.api.acoustics.Surface;
import dev.mcmod.aerocore.neoforge.probe.BlockMaterials;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * Where the listener is, acoustically (0.37): every {@link #EVERY_TICKS} ticks {@link #RAYS} rays from the camera
 * (a Fibonacci sphere, out to {@link #RANGE_M}) find the blocks round it, and {@link RoomAcoustics} makes a room of
 * them (open rays are sky or a space too large to matter); its reverb goes to {@link Efx}. {@link #between} counts the
 * solid blocks between two points for a sound's occlusion.
 * <p>
 * Occlusion rule (a listening rule, not a wall model): each solid block crossed keeps 0.55 of the sound broadband and
 * 0.3 of what is left at 5 kHz (about -5 dB and a further -10 dB high), at most three blocks counted; a sound behind
 * a wall is muffled, never cut.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class Listening {
    private Listening() {}

    public static final int RAYS = 32, EVERY_TICKS = 10, MAX_BLOCKS = 3;
    public static final double RANGE_M = 48;
    private static final Vec3[] DIRECTIONS = new Vec3[RAYS];
    static {
        double golden = Math.PI * (3 - Math.sqrt(5));
        for (int i = 0; i < RAYS; i++) {
            double y = 1 - 2 * (i + 0.5) / RAYS, r = Math.sqrt(1 - y * y), t = golden * i;
            DIRECTIONS[i] = new Vec3(r * Math.cos(t), y, r * Math.sin(t));
        }
    }

    private static volatile RoomAcoustics.Room room = RoomAcoustics.OUTDOORS;
    private static volatile RoomAcoustics.Reverb reverb = RoomAcoustics.DRY;
    private static long ticks;
    private static volatile boolean scanning = true;

    /** The listener's room now (OUTDOORS until scanned). */
    public static RoomAcoustics.Room room() { return room; }
    /** Its reverb as given to EFX. */
    public static RoomAcoustics.Reverb reverb() { return reverb; }

    /** Turns the room scan on or off (off: outdoors, dry). */
    public static void scan(boolean on) {
        scanning = on;
        if (!on) { room = RoomAcoustics.OUTDOORS; reverb = RoomAcoustics.DRY; Efx.room(reverb); }
    }

    @SubscribeEvent static void tick(ClientTickEvent.Post event) {
        Minecraft mc = Minecraft.getInstance();
        if (!scanning || mc.level == null || mc.player == null || mc.isPaused() || ticks++ % EVERY_TICKS != 0) { return; }
        Vec3 ear = mc.gameRenderer.getMainCamera().getPosition();
        List<RoomAcoustics.Ray> rays = new ArrayList<>(RAYS);
        for (Vec3 d : DIRECTIONS) {
            Vec3 to = ear.add(d.scale(RANGE_M));
            BlockHitResult hit = mc.level.clip(new ClipContext(ear, to, ClipContext.Block.COLLIDER, ClipContext.Fluid.NONE, mc.player));
            if (hit.getType() != HitResult.Type.BLOCK) { rays.add(new RoomAcoustics.Ray(RANGE_M, Surface.OPEN)); continue; }
            BlockState s = mc.level.getBlockState(hit.getBlockPos());
            Vec3 n = Vec3.atLowerCornerOf(hit.getDirection().getNormal());
            rays.add(new RoomAcoustics.Ray(hit.getLocation().distanceTo(ear), Surface.of(BlockMaterials.of(s)), Math.abs(n.dot(d))));
        }
        BlockPos here = BlockPos.containing(ear);
        double temperatureK = 273.15 + Math.clamp(-10 + 30 * mc.level.getBiome(here).value().getBaseTemperature(), -30, 45);
        double humidity = mc.level.isRainingAt(here) ? 100 : 70;
        RoomAcoustics.Room r = RoomAcoustics.estimate(rays, RANGE_M, temperatureK, humidity, 101_325);
        room = r;
        reverb = RoomAcoustics.reverb(r, Propagation.speedOfSound(temperatureK));
        Efx.room(reverb);
    }

    /** How many solid blocks lie between two points (at most MAX_BLOCKS). */
    public static int between(ClientLevel level, Vec3 from, Vec3 to) {
        int n = 0;
        Vec3 at = from;
        Vec3 step = to.subtract(from);
        double length = step.length();
        if (length < 1e-3) { return 0; }
        Vec3 dir = step.scale(1 / length);
        for (int k = 0; k < MAX_BLOCKS; k++) {
            BlockHitResult hit = level.clip(new ClipContext(at, to, ClipContext.Block.COLLIDER, ClipContext.Fluid.NONE, net.minecraft.world.phys.shapes.CollisionContext.empty()));
            if (hit.getType() != HitResult.Type.BLOCK) { break; }
            n++;
            // through the block: on past its far side
            BlockPos p = hit.getBlockPos();
            double past = 0;
            Vec3 probe = hit.getLocation();
            while (past < 1.8 && BlockPos.containing(probe).equals(p)) { probe = probe.add(dir.scale(0.1)); past += 0.1; }
            at = probe;
            if (at.distanceTo(from) >= length) { break; }
        }
        return n;
    }

    /** The occlusion rule's gain and 5 kHz gain for a number of blocks crossed. */
    public static float[] occlusion(int blocks) {
        int b = Math.min(MAX_BLOCKS, Math.max(0, blocks));
        return new float[]{(float) Math.pow(0.55, b), (float) Math.pow(0.3, b)};
    }

    /** The air's gain at 5 kHz over a distance (ISO 9613-1), relative to its broadband loss (for a low-pass). */
    public static float airHf(double distanceM, double temperatureK, double humidityPercent) {
        double db = Propagation.absorptionDbPerM(5000, temperatureK, humidityPercent, 101_325) * Math.max(0, distanceM);
        return (float) Math.max(0.001, Math.pow(10, -db / 20));
    }
}
