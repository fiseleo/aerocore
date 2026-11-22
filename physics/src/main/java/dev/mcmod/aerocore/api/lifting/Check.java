package dev.mcmod.aerocore.api.lifting;

/** Argument checks of the lifting package (as the frame package's Require, which is package-private there). */
final class Check {
    private Check() {}

    static double positive(double value, String name) {
        if (!(value > 0) || !Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be positive and finite"); }
        return value;
    }

    static double nonNegative(double value, String name) {
        if (!(value >= 0) || !Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be non-negative and finite"); }
        return value;
    }

    static double finite(double value, String name) {
        if (!Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be finite"); }
        return value;
    }

    /** A share or efficiency in (0, 1]. */
    static double share(double value, String name) {
        if (!(value > 0 && value <= 1)) { throw new IllegalArgumentException(name + " must be in (0, 1]"); }
        return value;
    }

    static int atLeastOne(int value, String name) {
        if (value < 1) { throw new IllegalArgumentException(name + " must be at least 1"); }
        return value;
    }
}
