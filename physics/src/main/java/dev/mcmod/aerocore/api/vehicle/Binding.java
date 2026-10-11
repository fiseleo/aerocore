package dev.mcmod.aerocore.api.vehicle;

import java.util.Map;
import java.util.Objects;

/**
 * Connects one of a part's inputs (role: "throttle" of a motor, "deflection" of a control surface) to a named control
 * channel the vehicle mod sets: input = offset + scale * channel value. Several parts may read one channel (both
 * elevators from "pitch"), one with scale -1 for opposite motion (ailerons from "roll"). Channel names are free.
 */
public record Binding(String role, String channel, double scale, double offset) {
    public Binding {
        Objects.requireNonNull(role, "role");
        Objects.requireNonNull(channel, "channel");
        if (!Double.isFinite(scale) || !Double.isFinite(offset)) { throw new IllegalArgumentException("Binding scale and offset must be finite"); }
    }

    public static Binding of(String role, String channel) { return new Binding(role, channel, 1, 0); }

    Map<String, Object> toJson() {
        Map<String, Object> m = Fields.object("role", role, "channel", channel);
        if (scale != 1) { m.put("scale", scale); }
        if (offset != 0) { m.put("offset", offset); }
        return m;
    }

    static Binding fromJson(Fields f) {
        return new Binding(f.string("role"), f.string("channel"), f.number("scale", 1), f.number("offset", 0));
    }
}
