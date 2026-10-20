package dev.mcmod.aerocore.api.body;

/** Argument checks of the body package (as dev.mcmod.aerocore.api.Checks, which is package-private there). */
final class Require {
    private Require() {}

    static void positive(double value, String name) {
        if (!(value > 0) || !Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be positive and finite"); }
    }
    static void nonnegative(double value, String name) {
        if (!(value >= 0) || !Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be non-negative and finite"); }
    }
    static void finite(double value, String name) {
        if (!Double.isFinite(value)) { throw new IllegalArgumentException(name + " must be finite"); }
    }
}
