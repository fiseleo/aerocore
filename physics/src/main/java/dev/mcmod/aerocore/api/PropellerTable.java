package dev.mcmod.aerocore.api;

import java.util.List;
import java.util.Objects;

/** Linear interpolation of caller-supplied or precomputed four-quadrant data. Never extrapolates. */
public final class PropellerTable implements PropellerModel {
    public record Point(double betaRadians, PropellerCoefficients coefficients) {
        public Point {
            if (!Double.isFinite(betaRadians) || Math.abs(betaRadians) > Math.PI) {
                throw new IllegalArgumentException("Hydrodynamic pitch angle must be in [-pi, pi]");
            }
            Objects.requireNonNull(coefficients, "coefficients");
        }
    }
    private final double[] beta, thrust, torque;

    public PropellerTable(List<Point> points) {
        if (points.size() < 2) { throw new IllegalArgumentException("At least two propeller points are required"); }
        beta = new double[points.size()]; thrust = new double[points.size()]; torque = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            Point point = points.get(i);
            if (i > 0 && point.betaRadians() <= beta[i - 1]) {
                throw new IllegalArgumentException("Propeller angles must be strictly increasing");
            }
            beta[i] = point.betaRadians(); thrust[i] = point.coefficients().thrust(); torque[i] = point.coefficients().torque();
        }
    }

    @Override public PropellerCoefficients sample(double angle) {
        if (!Double.isFinite(angle) || angle < beta[0] || angle > beta[beta.length - 1]) {
            throw new IllegalArgumentException("Angle is outside the supplied propeller data");
        }
        int high = 1;
        while (high < beta.length - 1 && beta[high] < angle) { high++; }
        double fraction = (angle - beta[high - 1]) / (beta[high] - beta[high - 1]);
        return new PropellerCoefficients(thrust[high - 1] + fraction * (thrust[high] - thrust[high - 1]),
                torque[high - 1] + fraction * (torque[high] - torque[high - 1]));
    }
}
