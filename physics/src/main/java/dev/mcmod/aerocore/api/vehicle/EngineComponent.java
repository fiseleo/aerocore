package dev.mcmod.aerocore.api.vehicle;

import java.util.List;
import java.util.Map;

/**
 * One designed component of an engine (block, head, pistons, ...): a player's blueprint with free parameters, its
 * own checks and an estimated mass. Components are not mounted on a vehicle by themselves; PistonEngineDesign
 * assembles them and checks that they fit together. JSON: {"type", "name", "parameters"}.
 */
public interface EngineComponent {
    /** Component type id, "aerocore:engine_block" and so on. */
    String type();
    String name();
    /** Problems of this component alone (INVALID stops the engine compiling; EXTRAPOLATION is a warning). */
    List<DesignIssue> check();
    Map<String, Object> parameters();
    /** Mass of the component as fitted to one engine (all pistons, all valves, ...), kg: given, or estimated from geometry and material. */
    double massKg();

    default Map<String, Object> toJson() { return Fields.object("type", type(), "name", name(), "parameters", parameters()); }

    /** Helpers for component checks. */
    final class Rules {
        private Rules() {}
        static void positive(List<DesignIssue> issues, String parameter, double value) {
            if (!(value > 0) || !Double.isFinite(value)) { issues.add(DesignIssue.invalid(parameter, parameter + " must be positive")); }
        }
        static void nonnegative(List<DesignIssue> issues, String parameter, double value) {
            if (!(value >= 0) || !Double.isFinite(value)) { issues.add(DesignIssue.invalid(parameter, parameter + " must not be negative")); }
        }
        static void count(List<DesignIssue> issues, String parameter, int value, int min, int max) {
            if (value < min || value > max) { issues.add(DesignIssue.invalid(parameter, parameter + " must be " + min + " to " + max)); }
        }
        /** A given mass (NaN = estimate) must be positive when given. */
        static void mass(List<DesignIssue> issues, double massKg) {
            if (!Double.isNaN(massKg) && !(massKg > 0 && Double.isFinite(massKg))) { issues.add(DesignIssue.invalid("massKg", "A given mass must be positive")); }
        }
        static <E extends Enum<E>> E choice(Fields f, String key, Class<E> type, E fallback) {
            if (!f.has(key)) { return fallback; }
            String s = f.string(key);
            for (E e : type.getEnumConstants()) { if (e.name().equalsIgnoreCase(s)) { return e; } }
            throw new IllegalArgumentException(f.path() + "." + key + ": unknown value " + s);
        }
        static double massOr(Fields f) { return f.number("massKg", Double.NaN); }
        static EngineMaterial material(Fields f, String key, EngineMaterial fallback) {
            return f.has(key) ? EngineMaterial.fromJson(f.raw(key), f.path() + "." + key) : fallback;
        }
        static Object mass(double massKg) { return Double.isNaN(massKg) ? null : massKg; }
        /** Wall thickness of a sheet-metal shell of a diameter for mass estimates: 0.4 % of it, at least 0.3 mm (estimate), m. */
        static double shell(double diameterM) { return Math.max(0.0003, 0.004 * diameterM); }
    }
}
