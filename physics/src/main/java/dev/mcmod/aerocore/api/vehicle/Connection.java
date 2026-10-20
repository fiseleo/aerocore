package dev.mcmod.aerocore.api.vehicle;

import java.util.Map;
import java.util.Objects;

/**
 * A link between two installed parts (by id).
 * <ul>
 * <li>Shaft: a rotor of from drives a rotor of to through a fixed gear ratio = speed of from / speed of to (2 reduces
 *     the speed by half, negative reverses it) with mechanical efficiency 0..1. Rotors joined by shafts turn as one
 *     train. A freewheel shaft (one-way clutch, a helicopter's sprag clutch) only drives: the to side may turn faster
 *     than the from side would make it (a rotor autorotating after its engine stops). fromPort and toPort name the
 *     rotors (Part.Rotor.name, a gearbox's "input" and "output"); null joins the part's first rotor (0.24).</li>
 * <li>Slipstream: the control surface lies in the propeller's race; its inflow is raised by the actuator-disk
 *     slipstream of that propeller's thrust (Rudder).</li>
 * </ul>
 */
public sealed interface Connection permits Connection.Shaft, Connection.Slipstream {
    record Shaft(String from, String to, double ratio, double efficiency, boolean freewheel, String fromPort, String toPort) implements Connection {
        public Shaft {
            Objects.requireNonNull(from, "from"); Objects.requireNonNull(to, "to");
            if (fromPort != null && fromPort.isBlank()) { fromPort = null; }
            if (toPort != null && toPort.isBlank()) { toPort = null; }
        }
        /** A shaft between the parts' first rotors. */
        public Shaft(String from, String to, double ratio, double efficiency, boolean freewheel) { this(from, to, ratio, efficiency, freewheel, null, null); }
        /** A shaft between the parts' first rotors that drives both ways. */
        public Shaft(String from, String to, double ratio, double efficiency) { this(from, to, ratio, efficiency, false); }
        /** A shaft between named rotors (null for a part's first) that drives both ways. */
        public static Shaft ports(String from, String fromPort, String to, String toPort, double ratio, double efficiency) {
            return new Shaft(from, to, ratio, efficiency, false, fromPort, toPort);
        }
    }
    record Slipstream(String propeller, String surface) implements Connection {
        public Slipstream { Objects.requireNonNull(propeller, "propeller"); Objects.requireNonNull(surface, "surface"); }
    }

    default Map<String, Object> toJson() {
        if (this instanceof Shaft s) {
            Map<String, Object> m = Fields.object("type", "shaft", "from", s.from(), "to", s.to(), "ratio", s.ratio(), "efficiency", s.efficiency());
            if (s.freewheel() || s.fromPort() != null || s.toPort() != null) { m = new java.util.LinkedHashMap<>(m); }
            if (s.freewheel()) { m.put("freewheel", true); }
            if (s.fromPort() != null) { m.put("fromPort", s.fromPort()); }
            if (s.toPort() != null) { m.put("toPort", s.toPort()); }
            return m;
        }
        Slipstream s = (Slipstream) this;
        return Fields.object("type", "slipstream", "propeller", s.propeller(), "surface", s.surface());
    }

    static Connection fromJson(Fields f) {
        return switch (f.string("type")) {
            case "shaft" -> new Shaft(f.string("from"), f.string("to"), f.number("ratio", 1), f.number("efficiency", 1), f.bool("freewheel", false),
                    f.has("fromPort") ? f.string("fromPort") : null, f.has("toPort") ? f.string("toPort") : null);
            case "slipstream" -> new Slipstream(f.string("propeller"), f.string("surface"));
            default -> throw new IllegalArgumentException(f.path() + ".type: unknown connection " + f.string("type"));
        };
    }
}
