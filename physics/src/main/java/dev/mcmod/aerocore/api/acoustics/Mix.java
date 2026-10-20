package dev.mcmod.aerocore.api.acoustics;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Several voices summed into one (0.37): a vehicle's engine, tyres and wind in its one stream. Each input has a gain
 * its owner sets from any thread; the mix ramps to it over a block so changes never click. Inputs may be added and
 * removed while it renders.
 */
public final class Mix implements Voice {
    /** One voice in the mix and its gain. */
    public static final class Input {
        private final Voice voice;
        private volatile float gain;
        private float now;
        private volatile boolean removed;

        Input(Voice voice, float gain) { this.voice = voice; this.gain = gain; this.now = gain; }
        public Voice voice() { return voice; }
        public void gain(double gain) { this.gain = Double.isFinite(gain) ? (float) Math.max(0, gain) : 0; }
        public float gain() { return gain; }
        public void remove() { removed = true; }
    }

    private final List<Input> inputs = new CopyOnWriteArrayList<>();
    private float[] scratch = new float[0];

    public Input add(Voice voice, double gain) {
        Input in = new Input(voice, (float) Math.max(0, gain));
        inputs.add(in);
        return in;
    }

    public int size() { return inputs.size(); }

    @Override public void render(float[] out, int offset, int n) {
        java.util.Arrays.fill(out, offset, offset + n, 0f);
        if (scratch.length < n) { scratch = new float[n]; }
        for (Input in : inputs) {
            if (in.removed) { inputs.remove(in); continue; }
            float target = in.gain, start = in.now;
            if (target == 0 && start == 0) {
                in.voice.render(scratch, 0, n);   // keeps its state moving (a silent engine still turns)
                continue;
            }
            in.voice.render(scratch, 0, n);
            float step = (target - start) / n;
            for (int i = 0; i < n; i++) { out[offset + i] += scratch[i] * (start + step * (i + 1)); }
            in.now = target;
        }
    }
}
