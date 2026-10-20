package dev.mcmod.aerocore.neoforge.client.sound;

import com.mojang.blaze3d.audio.Channel;
import com.mojang.logging.LogUtils;
import dev.mcmod.aerocore.api.acoustics.RoomAcoustics;
import dev.mcmod.aerocore.neoforge.mixin.ChannelAccessor;
import net.neoforged.fml.ModList;
import org.lwjgl.openal.AL;
import org.lwjgl.openal.AL10;
import org.lwjgl.openal.ALCapabilities;
import org.lwjgl.openal.EXTEfx;
import org.slf4j.Logger;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * OpenAL's effects extension for code-made sounds (0.37): one reverb (the listener's room) on an auxiliary slot, and
 * per sound a low-pass on its direct path (walls, a cabin, the air) and one on its send to the reverb (how much of it
 * the room returns). Only {@link SynthInstance}s are touched: Minecraft's own sounds stay as they are.
 * <ul>
 * <li>All OpenAL calls run on Minecraft's sound thread ({@link SoundHooks#executor}); each batch ends by clearing
 *     {@code alGetError}, so an error of ours never surfaces in Minecraft's own checks.</li>
 * <li>The objects belong to the sound engine's context: a reload (F3+T, a resource pack, a new device) makes them
 *     again ({@link SoundHooks#generation}). A sound's filters are deleted when it finishes.</li>
 * <li>No EFX (the device lacks ALC_EXT_EFX), Sound Physics Remastered installed (it owns the sends), or the caller
 *     turning it off: {@link #available} is false and callers fall back to volume only.</li>
 * </ul>
 * EFX's low-pass is two numbers: the gain, and the gain at 5 kHz relative to it ({@link Path#gainHf}).
 */
public final class Efx {
    private Efx() {}

    private static final Logger LOGGER = LogUtils.getLogger();

    /** A sound's direct path: its broadband gain and its gain at 5 kHz; and its send to the room's reverb (0 = dry). */
    public record Path(float gain, float gainHf, float send) {
        public static final Path CLEAR = new Path(1, 1, 0);
        public Path {
            gain = clamp(gain); gainHf = clamp(gainHf); send = clamp(send);
        }
        private static float clamp(float v) { return Float.isFinite(v) ? Math.max(0, Math.min(1, v)) : 1; }
        /** Both filters together (a wall on top of the air). */
        public Path times(float g, float hf) { return new Path(gain * g, gainHf * hf, send); }
        public Path withSend(float s) { return new Path(gain, gainHf, s); }
    }

    private static volatile boolean wanted = true;
    private static final boolean OTHER_MOD = ModList.get() != null && ModList.get().isLoaded("sound_physics_remastered");

    // ---- the sound thread's state
    private static int made = Integer.MIN_VALUE;
    private static boolean ok;
    private static int slot, effect;
    private static final Map<SynthInstance, int[]> FILTERS = new HashMap<>();
    private static RoomAcoustics.Reverb applied;
    private static int purgeCalls;
    private static volatile RoomAcoustics.Reverb room = RoomAcoustics.DRY;
    private static volatile boolean usable;

    /** Turns the effects on or off (a mod's setting); off clears every send and filter on the next update. */
    public static void want(boolean on) { wanted = on; }

    /** Whether filtering and reverb work now (EFX present, wanted, no other mod owning the sends). */
    public static boolean available() { return usable && wanted && !OTHER_MOD; }

    /** The room the listener is in, for the shared reverb (applied on the sound thread). */
    public static void room(RoomAcoustics.Reverb reverb) {
        room = reverb == null ? RoomAcoustics.DRY : reverb;
        run(Efx::applyRoom);
    }

    /** Sets a playing sound's paths (game thread; applied on the sound thread when it has its channel). */
    public static void path(SynthInstance sound, Path path) {
        if (sound == null) { return; }
        Path p = path == null ? Path.CLEAR : path;
        run(() -> applyPath(sound, p));
    }

    private static void run(Runnable r) {
        var executor = SoundHooks.executor();
        if (executor == null || OTHER_MOD) { return; }
        executor.execute(() -> {
            try {
                ensure();
                if (ok) { r.run(); }
            } catch (Throwable t) {
                if (ok) { LOGGER.warn("AeroCore: OpenAL effects failed, turning them off", t); }
                ok = false;
                usable = false;
            } finally {
                AL10.alGetError();
            }
        });
    }

    /** Sound thread: makes the slot and reverb for this engine's context (again after a reload). */
    private static void ensure() {
        int generation = SoundHooks.generation();
        if (made == generation) { return; }
        made = generation;
        FILTERS.clear();
        applied = null;
        ALCapabilities caps = AL.getCapabilities();
        ok = caps != null && caps.ALC_EXT_EFX;
        if (!ok) { usable = false; return; }
        AL10.alGetError();
        slot = EXTEfx.alGenAuxiliaryEffectSlots();
        effect = EXTEfx.alGenEffects();
        EXTEfx.alEffecti(effect, EXTEfx.AL_EFFECT_TYPE, EXTEfx.AL_EFFECT_REVERB);
        EXTEfx.alAuxiliaryEffectSloti(slot, EXTEfx.AL_EFFECTSLOT_AUXILIARY_SEND_AUTO, AL10.AL_TRUE);
        ok = AL10.alGetError() == AL10.AL_NO_ERROR;
        usable = ok;
        if (ok) { applyRoom(); }
    }

    private static void applyRoom() {
        RoomAcoustics.Reverb r = wanted ? room : RoomAcoustics.DRY;
        if (r.equals(applied)) { return; }
        applied = r;
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_DENSITY, (float) r.density());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_DIFFUSION, (float) r.diffusion());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_GAIN, (float) r.gain());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_GAINHF, (float) r.gainHf());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_DECAY_TIME, (float) r.decayS());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_DECAY_HFRATIO, (float) r.hfRatio());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_REFLECTIONS_GAIN, (float) r.reflectionsGain());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_REFLECTIONS_DELAY, (float) r.reflectionsDelayS());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_LATE_REVERB_GAIN, (float) r.lateGain());
        EXTEfx.alEffectf(effect, EXTEfx.AL_REVERB_LATE_REVERB_DELAY, (float) r.lateDelayS());
        EXTEfx.alAuxiliaryEffectSloti(slot, EXTEfx.AL_EFFECTSLOT_EFFECT, effect);
    }

    private static void applyPath(SynthInstance sound, Path p) {
        purge();
        Channel channel = sound.channel();
        if (channel == null || sound.finished()) { return; }
        int source = ((ChannelAccessor) channel).aerocore$source();
        if (!AL10.alIsSource(source)) { return; }
        int[] f = FILTERS.get(sound);
        if (f == null) {
            f = new int[]{EXTEfx.alGenFilters(), EXTEfx.alGenFilters(), source};
            for (int k = 0; k < 2; k++) { EXTEfx.alFilteri(f[k], EXTEfx.AL_FILTER_TYPE, EXTEfx.AL_FILTER_LOWPASS); }
            FILTERS.put(sound, f);
        }
        boolean on = wanted;
        EXTEfx.alFilterf(f[0], EXTEfx.AL_LOWPASS_GAIN, on ? p.gain() : 1);
        EXTEfx.alFilterf(f[0], EXTEfx.AL_LOWPASS_GAINHF, on ? Math.max(0.001f, p.gainHf()) : 1);
        AL10.alSourcei(source, EXTEfx.AL_DIRECT_FILTER, f[0]);
        EXTEfx.alFilterf(f[1], EXTEfx.AL_LOWPASS_GAIN, on ? p.send() : 0);
        EXTEfx.alFilterf(f[1], EXTEfx.AL_LOWPASS_GAINHF, 1);
        AL11Send.send(source, on && p.send() > 0 ? slot : EXTEfx.AL_EFFECTSLOT_NULL, f[1]);
    }

    /** Deletes the filters of sounds that have finished or lost their source. */
    private static void purge() {
        if (FILTERS.size() < 16 || ++purgeCalls % 64 != 0) { return; }
        for (Iterator<Map.Entry<SynthInstance, int[]>> it = FILTERS.entrySet().iterator(); it.hasNext(); ) {
            var e = it.next();
            Channel c = e.getKey().channel();
            if (e.getKey().finished() || c == null || ((ChannelAccessor) c).aerocore$source() != e.getValue()[2] || !AL10.alIsSource(e.getValue()[2])) {
                EXTEfx.alDeleteFilters(e.getValue()[0]);
                EXTEfx.alDeleteFilters(e.getValue()[1]);
                it.remove();
            }
        }
    }

    /** How many sounds have filters now (the debug overlay). */
    public static int filtered() { return FILTERS.size(); }

    /** The auxiliary send (EFX's three-integer source property). */
    private static final class AL11Send {
        static void send(int source, int slot, int filter) {
            org.lwjgl.openal.AL11.alSource3i(source, EXTEfx.AL_AUXILIARY_SEND_FILTER, slot, 0, filter);
        }
    }
}
