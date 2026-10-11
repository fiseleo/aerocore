package dev.mcmod.aerocore.api.vehicle;

import java.util.Map;
import java.util.Objects;

/**
 * A link between two installed parts (by id).
 * <ul>
 * <li>Shaft: the first rotor of from drives the first rotor of to through a fixed gear ratio = speed of from / speed
 *     of to (2 reduces the speed by half, negative reverses it) with mechanical efficiency 0..1. Rotors joined by
 *     shafts turn as one train.</li>
 * <li>Slipstream: the control surface lies in the propeller's race; its inflow is raised by the actuator-disk
 *     slipstream of that propeller's thrust (Rudder).</li>
 * </ul>
 */
public sealed interface Connection permits Connection.Shaft, Connection.Slipstream {
    record Shaft(String from, String to, double ratio, double efficiency) implements Connection {
        public Shaft { Objects.requireNonNull(from, "from"); Objects.requireNonNull(to, "to"); }
    }
    record Slipstream(String propeller, String surface) implements Connection {
        public Slipstream { Objects.requireNonNull(propeller, "propeller"); Objects.requireNonNull(surface, "surface"); }
    }

    default Map<String, Object> toJson() {
        if (this instanceof Shaft s) {
            return Fields.object("type", "shaft", "from", s.from(), "to", s.to(), "ratio", s.ratio(), "efficiency", s.efficiency());
        }
        Slipstream s = (Slipstream) this;
        return Fields.object("type", "slipstream", "propeller", s.propeller(), "surface", s.surface());
    }

    static Connection fromJson(Fields f) {
        return switch (f.string("type")) {
            case "shaft" -> new Shaft(f.string("from"), f.string("to"), f.number("ratio", 1), f.number("efficiency", 1));
            case "slipstream" -> new Slipstream(f.string("propeller"), f.string("surface"));
            default -> throw new IllegalArgumentException(f.path() + ".type: unknown connection " + f.string("type"));
        };
    }
}
