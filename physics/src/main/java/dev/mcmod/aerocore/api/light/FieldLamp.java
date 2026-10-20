package dev.mcmod.aerocore.api.light;

import dev.mcmod.aerocore.api.Vec3d;

/**
 * A light fixed in the world (0.38): a runway edge, threshold or approach light, a PAPI unit, a taxiway light, an
 * obstruction light, a beacon, a harbour or lighthouse light. Where it is (world metres), the way its beam points,
 * how wide it shows (horizontally all round when horizontalDeg is 360 or more; vertically the band verticalDeg wide
 * centred elevationDeg above the horizontal), how strong it is (candela in its beam), its colour (or colours by where
 * it is seen from), how it shows over time, the circuit it is switched with (its owner's numbering) and the size of
 * its lens.
 * <p>
 * Outside its beam it fades over 5 degrees to nothing. The beam is a simplification of a fitting's isocandela
 * diagram: the same intensity across it.
 */
public record FieldLamp(Vec3d at, Vec3d axis, double horizontalDeg, double verticalDeg, double elevationDeg, double intensityCd,
                        int argb, SectorColours colours, LightSignal signal, int circuit, double lensM) {
    /** How far past its beam's edge a light fades out (degrees). */
    public static final double FEATHER_DEG = 5;

    public FieldLamp {
        if (axis.lengthSquared() < 1e-18) { throw new IllegalArgumentException("A field lamp needs an axis"); }
        if (!(intensityCd >= 0) || !(lensM > 0) || !(horizontalDeg > 0) || !(verticalDeg > 0)) {
            throw new IllegalArgumentException("A field lamp needs a positive beam, lens and intensity");
        }
        axis = axis.normalized();
        if (signal == null) { signal = LightSignal.steady(); }
    }

    /** Shows all round, from below the horizon to straight up (an elevated edge light, an obstruction light). */
    public static FieldLamp omni(Vec3d at, double intensityCd, int argb, int circuit, double lensM) {
        return new FieldLamp(at, Vec3d.X, 360, 180, 0, intensityCd, argb, null, LightSignal.steady(), circuit, lensM);
    }

    /** Shows within a beam about its axis (a threshold light toward the approach, a PAPI unit, an approach light). */
    public static FieldLamp beam(Vec3d at, Vec3d axis, double horizontalDeg, double verticalDeg, double elevationDeg, double intensityCd, int argb, int circuit, double lensM) {
        return new FieldLamp(at, axis, horizontalDeg, verticalDeg, elevationDeg, intensityCd, argb, null, LightSignal.steady(), circuit, lensM);
    }

    public FieldLamp with(LightSignal s) { return new FieldLamp(at, axis, horizontalDeg, verticalDeg, elevationDeg, intensityCd, argb, colours, s, circuit, lensM); }

    public FieldLamp with(SectorColours c) { return new FieldLamp(at, axis, horizontalDeg, verticalDeg, elevationDeg, intensityCd, argb, c, signal, circuit, lensM); }

    /** The bearing off the axis of a direction from the light (radians, -pi to pi, positive to the left, seen from above). */
    public double azimuthOff(Vec3d toEye) {
        double ax = Math.atan2(axis.z(), axis.x()), ex = Math.atan2(toEye.z(), toEye.x());
        // y up, x east, z south: a turn from x toward z is to the right seen from above
        double d = ax - ex;
        return Math.atan2(Math.sin(d), Math.cos(d));
    }

    /** The elevation of a direction from the light above the horizontal (radians). */
    public static double elevation(Vec3d toEye) { return Math.atan2(toEye.y(), Math.hypot(toEye.x(), toEye.z())); }

    /** How much of the beam is seen from a direction (from the light toward the eye; need not be unit): 1 within it, 0 well outside. */
    public double coverage(Vec3d toEye) {
        double h = 1;
        if (horizontalDeg < 360) { h = fade(Math.abs(Math.toDegrees(azimuthOff(toEye))), horizontalDeg / 2); }
        if (h <= 0) { return 0; }
        double e = Math.toDegrees(elevation(toEye)) - elevationDeg;
        return h * fade(Math.abs(e), verticalDeg / 2);
    }

    /** The colour seen from a direction at a moment. */
    public int colour(Vec3d toEye, double timeS) {
        int c = colours == null ? argb : colours.colour(azimuthOff(toEye), elevation(toEye));
        return signal.colour(timeS, c);
    }

    private static double fade(double offDeg, double halfDeg) {
        if (offDeg <= halfDeg) { return 1; }
        return Math.max(0, 1 - (offDeg - halfDeg) / FEATHER_DEG);
    }
}
