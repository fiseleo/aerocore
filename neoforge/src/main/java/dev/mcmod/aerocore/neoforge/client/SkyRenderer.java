package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.BufferBuilder;
import com.mojang.blaze3d.vertex.BufferUploader;
import com.mojang.blaze3d.vertex.DefaultVertexFormat;
import com.mojang.blaze3d.vertex.Tesselator;
import com.mojang.blaze3d.vertex.VertexFormat;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.SkyModel;
import dev.mcmod.aerocore.api.SurfaceWind;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.probe.WorldWind;
import net.minecraft.client.Camera;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.renderer.DimensionSpecialEffects;
import net.minecraft.client.renderer.ShaderInstance;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.level.biome.Biome;
import net.minecraft.world.level.material.FogType;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import net.neoforged.neoforge.client.event.RegisterShadersEvent;
import net.neoforged.neoforge.client.event.ViewportEvent;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.slf4j.Logger;

/**
 * The overworld's daylight sky, fog, clouds and falling weather, made physical and moved by the level's wind.
 * <ul>
 * <li>Sky: after vanilla draws its flat sky colour, a full-screen pass (shaders/core/aerocore_sky) paints the
 * Preetham clear sky (SkyModel; turbidity 2.5, hazier up to 6 as rain comes) blended toward the CIE overcast sky by the
 * rain level, for the game's sun; vanilla's sunrise fan is left out (the model has its own twilight colours) and the
 * sun, moon and stars are drawn over it as usual. Display choices, not physics: exposure for the hemisphere's mean
 * luminance (an adapting eye; the clear sky's mean maps to EXPOSURE), a tone curve on luminance (1 - exp(-L)) with
 * colours outside sRGB desaturated at equal luminance and highlights rolling off to white, dimming by the game's own day
 * curve (clamp(2 cos(time angle) + 0.5)) and halving in a thunderstorm, the overcast sky's 0.55.</li>
 * <li>Fog: the colour of the same sky just above the horizon in the direction the camera looks (in air only), so
 * distant terrain fades into the sky behind it; the water's reflections use the same zenith and horizon.</li>
 * <li>Clouds: they drift with the wind at 200 m (WorldWind's reference speed times the log profile to 200 m),
 * integrated each client tick, instead of vanilla's 0.6 m/s toward -x.</li>
 * <li>Rain and snow lean with the wind at the camera (mean and gust) over their fall speed: rain 6.5 m/s (2 mm drops,
 * Gunn and Kinzer 1949), snow 1.0 m/s (aggregates, Locatelli and Hobbs 1974), the lean capped at 4 (76 degrees);
 * which one by the camera's biome.</li>
 * </ul>
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class SkyRenderer {
    private static final Logger LOGGER = LogUtils.getLogger();
    /** Linear value the clear zenith is exposed to (a display choice). */
    public static final double EXPOSURE = 0.5;
    private static final double CLEAR_TURBIDITY = 2.5, HAZY_TURBIDITY = 6, OVERCAST = 0.55;
    private static final double RAIN_FALL = 6.5, SNOW_FALL = 1.0, MAX_LEAN = 4;

    private static volatile ShaderInstance shader;
    private static double cloudX, cloudZ, previousCloudX, previousCloudZ;

    private SkyRenderer() {}

    @EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
    public static final class Shaders {
        private Shaders() {}
        @SubscribeEvent public static void register(RegisterShadersEvent event) {
            try {
                event.registerShader(new ShaderInstance(event.getResourceProvider(), ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "aerocore_sky"),
                        DefaultVertexFormat.POSITION), loaded -> shader = loaded);
            } catch (java.io.IOException | RuntimeException e) {
                LOGGER.error("AeroCore sky shader failed to load; vanilla's sky is drawn", e);
                shader = null;
            }
        }
    }

    /** Whether the physical sky replaces vanilla's in the current level (overworld-type skies). */
    public static boolean active() {
        ClientLevel level = Minecraft.getInstance().level;
        return shader != null && level != null && level.effects().skyType() == DimensionSpecialEffects.SkyType.NORMAL;
    }

    /** The sky's state this frame: the clear sky for the sun, the sun direction, daylight, overcast and storm shares. */
    private record State(SkyModel.Clear clear, Vector3f sun, double daylight, double overcast, double storm, double exposure) {}

    /** The last state and its inputs: several readers ask each frame, and the hemisphere mean costs 512 sky samples. */
    private static State lastState;
    private static double lastAngle = Double.NaN, lastRain, lastStorm;

    private static State state(ClientLevel level, float partial) {
        double angle = level.getTimeOfDay(partial) * 2 * Math.PI, sunY = Math.cos(angle);
        double rain = level.getRainLevel(partial), storm = level.getThunderLevel(partial);
        State last = lastState;
        if (last != null && angle == lastAngle && rain == lastRain && storm == lastStorm) { return last; }
        Vector3f sun = new Vector3f((float) -Math.sin(angle), (float) sunY, 0);
        double turbidity = CLEAR_TURBIDITY + (HAZY_TURBIDITY - CLEAR_TURBIDITY) * rain;
        double daylight = Math.clamp(Math.cos(angle) * 2 + 0.5, 0, 1);
        SkyModel.Clear clear = SkyModel.clear(turbidity, Math.acos(Math.clamp(sunY, -1, 1)));
        // Exposure relative to the zenith (the shader's unit), for the hemisphere's mean.
        double exposure = EXPOSURE * clear.zenithLuminance() / clear.meanLuminance();
        State s = new State(clear, sun, daylight, rain, storm, exposure);
        lastAngle = angle; lastRain = rain; lastStorm = storm; lastState = s;
        return s;
    }

    /** Display colour (sRGB, 0..1) of the sky toward a direction (unit, y up), as the shader draws it. */
    private static float[] colour(State s, double dx, double dy, double dz) {
        double theta = Math.acos(Math.clamp(dy, 0.0017, 1)), gamma = Math.acos(Math.clamp(dx * s.sun().x + dy * s.sun().y + dz * s.sun().z, -1, 1));
        double[] yxy = s.clear().xyY(theta, gamma);
        double[] clear = SkyModel.linearRgb(yxy[1], yxy[2], yxy[0] / s.clear().zenithLuminance());
        double overcast = SkyModel.overcast(theta) * OVERCAST / s.exposure() * EXPOSURE, scale = s.exposure() * s.daylight() * (1 - 0.5 * s.storm());
        double[] linear = new double[3];
        for (int i = 0; i < 3; i++) { linear[i] = (clear[i] * (1 - s.overcast()) + overcast * s.overcast()) * scale; }
        return display(linear);
    }

    /**
     * Linear sRGB to display: colours outside the gamut desaturated toward their luminance until inside, the luminance
     * through 1 - exp(-L), highlights past 1 rolled toward white, sRGB-encoded (as the sky shader).
     */
    private static float[] display(double[] lin) {
        double l = 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2];
        float[] out = new float[3];
        if (!(l > 0)) { return out; }
        double m = Math.min(lin[0], Math.min(lin[1], lin[2]));
        double[] c = lin.clone();
        if (m < 0) { for (int i = 0; i < 3; i++) { c[i] = l + (c[i] - l) * l / (l - m); } }
        double mapped = 1 - Math.exp(-l), top = 0;
        for (int i = 0; i < 3; i++) { c[i] *= mapped / l; top = Math.max(top, c[i]); }
        if (top > 1) { double w = Math.min(top - 1, 1); for (int i = 0; i < 3; i++) { c[i] = c[i] / top * (1 - w) + w; } }
        for (int i = 0; i < 3; i++) { out[i] = (float) SkyModel.encode(c[i]); }
        return out;
    }

    /** The sky colour straight up now (for the water's reflections), or null when the physical sky is off. */
    public static float[] zenithColour(ClientLevel level, float partial) {
        return active() ? colour(state(level, partial), 0, 1, 0) : null;
    }

    /** Called by the sky mixin after vanilla's sky colour: paints the physical sky over it. */
    public static void drawDome(Matrix4f modelView, Matrix4f projection, float partial) {
        ClientLevel level = Minecraft.getInstance().level;
        ShaderInstance program = shader;
        if (level == null || program == null || !active()) { return; }
        State s = state(level, partial);
        SkyModel.Clear c = s.clear();
        program.safeGetUniform("InverseViewProjection").set(new Matrix4f(projection).mul(modelView).invert());
        program.safeGetUniform("PerezA").set((float) c.luminance()[0], (float) c.chromaX()[0], (float) c.chromaY()[0]);
        program.safeGetUniform("PerezB").set((float) c.luminance()[1], (float) c.chromaX()[1], (float) c.chromaY()[1]);
        program.safeGetUniform("PerezC").set((float) c.luminance()[2], (float) c.chromaX()[2], (float) c.chromaY()[2]);
        program.safeGetUniform("PerezD").set((float) c.luminance()[3], (float) c.chromaX()[3], (float) c.chromaY()[3]);
        program.safeGetUniform("PerezE").set((float) c.luminance()[4], (float) c.chromaX()[4], (float) c.chromaY()[4]);
        program.safeGetUniform("Zenith").set((float) c.zenithLuminance(), (float) c.zenithX(), (float) c.zenithY());
        program.safeGetUniform("Normalizer").set((float) SkyModel.perez(c.luminance(), 0, c.sunZenith()),
                (float) SkyModel.perez(c.chromaX(), 0, c.sunZenith()), (float) SkyModel.perez(c.chromaY(), 0, c.sunZenith()));
        program.safeGetUniform("SunDirection").set(s.sun().x, s.sun().y, s.sun().z);
        program.safeGetUniform("Exposure").set((float) s.exposure());
        program.safeGetUniform("Daylight").set((float) s.daylight());
        program.safeGetUniform("Overcast").set((float) s.overcast());
        program.safeGetUniform("Storm").set((float) s.storm());
        RenderSystem.setShader(() -> program);
        RenderSystem.disableDepthTest();
        RenderSystem.disableBlend();
        BufferBuilder buffer = Tesselator.getInstance().begin(VertexFormat.Mode.TRIANGLES, DefaultVertexFormat.POSITION);
        buffer.addVertex(-1, -1, 0); buffer.addVertex(3, -1, 0); buffer.addVertex(-1, 3, 0);
        BufferUploader.drawWithShader(buffer.buildOrThrow());
        RenderSystem.enableDepthTest();
    }

    /** Fog in air takes the sky's colour just above the horizon where the camera looks. */
    @SubscribeEvent public static void fogColour(ViewportEvent.ComputeFogColor event) {
        Camera camera = event.getCamera();
        ClientLevel level = Minecraft.getInstance().level;
        if (level == null || !active() || camera.getFluidInCamera() != FogType.NONE) { return; }
        Vector3f look = camera.getLookVector();
        double h = Math.hypot(look.x, look.z);
        double dx = h > 1e-6 ? look.x / h : 1, dz = h > 1e-6 ? look.z / h : 0, elevation = Math.toRadians(2);
        float[] c = colour(state(level, (float) event.getPartialTick()), dx * Math.cos(elevation), Math.sin(elevation), dz * Math.cos(elevation));
        event.setRed(c[0]); event.setGreen(c[1]); event.setBlue(c[2]);
    }

    /** Clouds drift with the wind at 200 m. */
    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        ClientLevel level = minecraft.level;
        previousCloudX = cloudX; previousCloudZ = cloudZ;
        if (level == null || minecraft.isPaused()) { return; }
        WorldWind.Reference wind = WorldWind.reference(level);
        double speed = SurfaceWind.reference(wind.speedMps(), SurfaceWind.TOP_HEIGHT) * MinecraftUnits.TICK_SECONDS;
        cloudX += speed * wind.directionX(); cloudZ += speed * wind.directionZ();
    }

    /** How far the clouds have drifted (blocks) along x and z, for the cloud mixin. */
    public static double cloudOffsetX(float partial) { return previousCloudX + (cloudX - previousCloudX) * partial; }
    public static double cloudOffsetZ(float partial) { return previousCloudZ + (cloudZ - previousCloudZ) * partial; }

    /**
     * Horizontal lean {dx/dy, dz/dy} of falling rain or snow at the camera: the wind (mean and gust) over the fall
     * speed, capped at MAX_LEAN.
     */
    public static float[] precipitationLean(double camX, double camY, double camZ, float partial) {
        ClientLevel level = Minecraft.getInstance().level;
        if (level == null) { return new float[]{0, 0}; }
        double t = (level.getGameTime() + partial) * MinecraftUnits.TICK_SECONDS;
        Vec3d wind = WorldWind.at(level, camX, camY, camZ, t);
        BlockPos pos = BlockPos.containing(camX, camY, camZ);
        double fall = level.getBiome(pos).value().getPrecipitationAt(pos) == Biome.Precipitation.SNOW ? SNOW_FALL : RAIN_FALL;
        double lx = wind.x() / fall, lz = wind.z() / fall, lean = Math.hypot(lx, lz);
        if (lean > MAX_LEAN) { lx *= MAX_LEAN / lean; lz *= MAX_LEAN / lean; }
        return new float[]{(float) lx, (float) lz};
    }
}
