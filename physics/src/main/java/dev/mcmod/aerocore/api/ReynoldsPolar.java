package dev.mcmod.aerocore.api;

import java.util.List;
import java.util.Objects;

/** Log-Re interpolation between caller-supplied polars, with an explicit upper Mach validity limit. */
public final class ReynoldsPolar implements AirfoilPolar {
    public record Slice(double reynoldsNumber, PolarTable polar) {
        public Slice {
            Checks.positive(reynoldsNumber, "Reference Reynolds number");
            Objects.requireNonNull(polar, "polar");
        }
    }
    private final List<Slice> slices;
    private final double maxMach;

    public ReynoldsPolar(List<Slice> slices, double maxMach) {
        this.slices = List.copyOf(slices);
        Checks.positive(maxMach, "Maximum Mach");
        this.maxMach = maxMach;
        if (slices.size() < 2) { throw new IllegalArgumentException("At least two Reynolds slices are required"); }
        for (int i = 1; i < slices.size(); i++) {
            if (slices.get(i).reynoldsNumber() <= slices.get(i - 1).reynoldsNumber()) {
                throw new IllegalArgumentException("Reference Reynolds numbers must be strictly increasing");
            }
        }
    }

    @Override public AeroCoefficients sample(double angle) {
        throw new IllegalArgumentException("ReynoldsPolar requires FlowConditions; use the flow-aware overload");
    }

    @Override public AeroCoefficients sample(double angle, FlowConditions flow) {
        double re = flow.reynoldsNumber();
        if (flow.machNumber() > maxMach || re < slices.getFirst().reynoldsNumber() || re > slices.getLast().reynoldsNumber()) {
            throw new IllegalArgumentException("Flow is outside the supplied polar Reynolds/Mach envelope");
        }
        for (int i = 0; i < slices.size(); i++) {
            Slice high = slices.get(i);
            if (re == high.reynoldsNumber()) { return high.polar().sample(angle); }
            if (re < high.reynoldsNumber()) {
                Slice low = slices.get(i - 1);
                double fraction = (Math.log(re) - Math.log(low.reynoldsNumber()))
                        / (Math.log(high.reynoldsNumber()) - Math.log(low.reynoldsNumber()));
                return PolarTable.interpolate(low.polar().sample(angle), high.polar().sample(angle), fraction);
            }
        }
        throw new AssertionError("Unreachable Reynolds interval");
    }
}
