package dev.mcmod.aerocore.api.vehicle;

import java.util.Objects;

/**
 * Something about a design worth telling its maker. INVALID: the value is not physical (a negative length, a zero
 * diameter) and the design does not compile. EXTRAPOLATION: the value is allowed but lies outside the range a model's
 * source was fitted or measured over (value, min, max, source say which), so the result is an extrapolation a design
 * screen may warn about. parameter is a dotted path ("hull.lengthM", "parts.prop.pitchRatio"); message is English for
 * logs, the other fields let a mod write its own text.
 */
public record DesignIssue(Kind kind, String parameter, double value, double min, double max, String source, String message) {
    public enum Kind { INVALID, EXTRAPOLATION }

    public DesignIssue {
        Objects.requireNonNull(kind, "kind");
        Objects.requireNonNull(parameter, "parameter");
        source = source == null ? "" : source;
        Objects.requireNonNull(message, "message");
    }

    public static DesignIssue invalid(String parameter, String message) {
        return new DesignIssue(Kind.INVALID, parameter, Double.NaN, Double.NaN, Double.NaN, "", message);
    }

    public static DesignIssue extrapolation(String parameter, double value, double min, double max, String source) {
        return new DesignIssue(Kind.EXTRAPOLATION, parameter, value, min, max, source,
                parameter + " = " + value + " is outside " + min + ".." + max + " (" + source + ")");
    }

    /** Checks that a value lies in [min, max], adding an EXTRAPOLATION issue when it does not. */
    static void range(java.util.List<DesignIssue> issues, String parameter, double value, double min, double max, String source) {
        if (value < min || value > max) { issues.add(extrapolation(parameter, value, min, max, source)); }
    }

    /** The same issue under a parent path ("prop" + "pitchRatio" = "prop.pitchRatio"). */
    public DesignIssue under(String prefix) {
        String path = prefix + "." + parameter;
        return new DesignIssue(kind, path, value, min, max, source, message.startsWith(parameter) ? prefix + "." + message : message);
    }
}
