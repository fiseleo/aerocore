package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.arguments.IntegerArgumentType;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.acoustics.Voice;
import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.sounds.SoundSource;
import net.minecraft.world.phys.Vec3;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.fml.loading.FMLEnvironment;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import net.neoforged.neoforge.client.event.RegisterClientCommandsEvent;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * Development client commands for the sound hooks (0.37, not registered in production):
 * <ul>
 * <li>/aerocoredev sound sine &lt;hz&gt;: a live test tone (a sine with a 2 Hz beat) at the player's position; several
 *     at once fill the streaming channels.</li>
 * <li>/aerocoredev sound click: a runtime buffer (a damped 1.2 kHz click) on a static channel.</li>
 * <li>/aerocoredev sound stop; /aerocoredev sound stats (streams, free channels, ring fill, underruns, restarts,
 *     the synthesis thread's load, runtime sounds) in the chat and the log.</li>
 * <li>/aerocoredev sound stall &lt;ms&gt;: blocks the game thread (a stream must not run dry meanwhile).</li>
 * <li>/aerocoredev sound record &lt;seconds&gt;: SoundRecorder.</li>
 * </ul>
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class DevSoundCommands {
    private DevSoundCommands() {}

    private static final Logger LOGGER = LogUtils.getLogger();
    private record Test(LiveSound sound, Vec3 at) {}
    private static final List<Test> TESTS = new ArrayList<>();
    private static int made;

    /** A sine at a frequency with a slow beat (so a recording shows it is alive), at a tenth of full scale. */
    static Voice sine(double hz) {
        return new Voice() {
            double phase, beat;
            @Override public void render(float[] out, int offset, int n) {
                for (int i = 0; i < n; i++) {
                    phase += hz / RATE;
                    beat += 2.0 / RATE;
                    if (phase >= 1) { phase -= 1; }
                    if (beat >= 1) { beat -= 1; }
                    out[offset + i] = (float) (0.1 * Math.sin(2 * Math.PI * phase) * (0.75 + 0.25 * Math.cos(2 * Math.PI * beat)));
                }
            }
        };
    }

    @SubscribeEvent static void register(RegisterClientCommandsEvent event) {
        if (FMLEnvironment.production) { return; }
        event.getDispatcher().register(Commands.literal("aerocoredev").then(Commands.literal("sound")
                .then(Commands.literal("sine").then(Commands.argument("hz", DoubleArgumentType.doubleArg(20, 20000)).executes(c -> {
                    Minecraft mc = Minecraft.getInstance();
                    double hz = DoubleArgumentType.getDouble(c, "hz");
                    ResourceLocation id = ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "test_sine_" + (++made));
                    TESTS.add(new Test(LiveSound.streamed(id, SoundSource.NEUTRAL, sine(hz)), mc.player.position()));
                    say("sine " + hz + " Hz (" + TESTS.size() + " playing)");
                    return 1;
                })))
                .then(Commands.literal("click").executes(c -> {
                    Minecraft mc = Minecraft.getInstance();
                    ResourceLocation id = RuntimeSounds.id(AeroCore.MOD_ID, "test_click");
                    if (!RuntimeSounds.defined(id)) {
                        float[] s = new float[Voice.RATE / 5];
                        for (int i = 0; i < s.length; i++) { double t = i / (double) Voice.RATE; s[i] = (float) (0.5 * Math.exp(-t * 40) * Math.sin(2 * Math.PI * 1200 * t)); }
                        RuntimeSounds.define(id, s);
                    }
                    SynthInstance click = SynthInstance.buffered(id, SoundSource.NEUTRAL, false);
                    Vec3 p = mc.player.position();
                    click.at(p.x, p.y, p.z);
                    mc.getSoundManager().play(click);
                    return 1;
                }))
                .then(Commands.literal("stop").executes(c -> {
                    TESTS.forEach(t -> t.sound().stop());
                    TESTS.clear();
                    say("stopped");
                    return 1;
                }))
                .then(Commands.literal("stall").then(Commands.argument("ms", IntegerArgumentType.integer(1, 5000)).executes(c -> {
                    int ms = IntegerArgumentType.getInteger(c, "ms");
                    LOGGER.info("AeroCore sound: stalling the game thread for {} ms", ms);
                    try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
                    return 1;
                })))
                .then(Commands.literal("stats").executes(c -> {
                    StringBuilder b = new StringBuilder(String.format(java.util.Locale.ROOT,
                            "AeroCore sound: %d streams on the synth thread (load %.1f %%), %d free streaming channels, generation %d, %d runtime sounds (%d KB)",
                            SynthThread.streams(), 100 * SynthThread.load(), SoundHooks.freeStreams(), SoundHooks.generation(),
                            RuntimeSounds.count(), RuntimeSounds.bytes() / 1024));
                    for (Test t : TESTS) {
                        SynthStream s = t.sound().instance() == null ? null : t.sound().instance().stream();
                        b.append(String.format(java.util.Locale.ROOT, "\n  %s: %s, ready %d frames, underruns %d, rescued %d, restarts %d",
                                t.sound().id(), s == null ? "no stream" : "streaming", s == null ? 0 : s.ready(), s == null ? 0 : s.underruns(),
                                s == null ? 0 : s.rescued(), t.sound().restarts()));
                    }
                    LOGGER.info(b.toString());
                    say(b.toString());
                    return 1;
                }))
                .then(Commands.literal("record").then(Commands.argument("seconds", DoubleArgumentType.doubleArg(0.1, 60)).executes(c -> {
                    double s = DoubleArgumentType.getDouble(c, "seconds");
                    if (!SoundRecorder.start(s, "aerocore", dir -> say("written to " + dir))) { say("already recording"); }
                    return 1;
                })))));
    }

    @SubscribeEvent static void tick(ClientTickEvent.Post event) {
        if (TESTS.isEmpty()) { return; }
        if (Minecraft.getInstance().level == null) {
            TESTS.forEach(t -> t.sound().stop());
            TESTS.clear();
            return;
        }
        for (Test t : TESTS) { t.sound().update(t.at().x, t.at().y, t.at().z, 1, 1, false); }
    }

    private static void say(String text) {
        Minecraft mc = Minecraft.getInstance();
        if (mc.player != null) { mc.player.displayClientMessage(Component.literal(text), false); }
    }
}
