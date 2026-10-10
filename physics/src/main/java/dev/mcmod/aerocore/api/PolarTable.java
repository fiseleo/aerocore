package dev.mcmod.aerocore.api;

import java.util.List;
import java.util.Objects;

/** Linear interpolation of caller-supplied CL, CD and Cm. Never extrapolates beyond the data. */
public final class PolarTable implements AirfoilPolar {
    public record Point(double angleRadians, AeroCoefficients coefficients) {
        public Point {
            if (!Double.isFinite(angleRadians) || Math.abs(angleRadians) > Math.PI) {
                throw new IllegalArgumentException("Polar angle must be in [-pi, pi]");
            }
            Objects.requireNonNull(coefficients, "coefficients");
        }
    }
    private final List<Point> points;

    public PolarTable(List<Point> points) {
        this.points = List.copyOf(points);
        if (points.size() < 2) { throw new IllegalArgumentException("At least two polar points are required"); }
        for (int i = 1; i < points.size(); i++) {
            if (points.get(i).angleRadians() <= points.get(i - 1).angleRadians()) {
                throw new IllegalArgumentException("Polar angles must be strictly increasing");
            }
        }
    }

    @Override public AeroCoefficients sample(double angle) {
        if (!Double.isFinite(angle) || angle < points.getFirst().angleRadians() || angle > points.getLast().angleRadians()) {
            throw new IllegalArgumentException("Angle is outside the supplied polar data");
        }
        for (int i = 0; i < points.size(); i++) {
            Point high = points.get(i);
            if (angle == high.angleRadians()) { return high.coefficients(); }
            if (angle < high.angleRadians()) {
                Point low = points.get(i - 1);
                return interpolate(low.coefficients(), high.coefficients(),
                        (angle - low.angleRadians()) / (high.angleRadians() - low.angleRadians()));
            }
        }
        throw new AssertionError("Unreachable polar interval");
    }

    static AeroCoefficients interpolate(AeroCoefficients a, AeroCoefficients b, double fraction) {
        return new AeroCoefficients(
                a.lift() + fraction * (b.lift() - a.lift()),
                a.drag() + fraction * (b.drag() - a.drag()),
                a.pitchingMoment() + fraction * (b.pitchingMoment() - a.pitchingMoment()),
                a.stalled() || b.stalled());
    }
}
