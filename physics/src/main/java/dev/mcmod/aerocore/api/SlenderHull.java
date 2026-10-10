package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A slender hull described by transverse sections along its model-local +X axis (bow forward), y up, z to starboard.
 * Each section stands for a strip reaching halfway to its neighbours (trapezoidal rule along x). Geometry only;
 * HullModel adds the hydrodynamic coefficients and HullDynamics integrates it.
 */
public final class SlenderHull {
    /** Upright particulars at one waterline: L, B, T on the waterline, volume, form coefficients, lcb, wetted area. */
    public record Particulars(double waterlineY, double lengthM, double beamM, double draftM, double volumeM3,
                              double blockCoefficient, double prismaticCoefficient, double midshipCoefficient,
                              double waterplaneCoefficient, double lcbPercent, double wettedAreaM2, double xForwardM, double xAftM,
                              double centreOfBuoyancyX) {}

    private final List<HullSection> sections;
    private final double[] strip;

    public SlenderHull(List<HullSection> sections) {
        Objects.requireNonNull(sections, "sections");
        if (sections.size() < 3) { throw new IllegalArgumentException("A hull needs at least three sections"); }
        for (int i = 1; i < sections.size(); i++) {
            if (!(sections.get(i).xM() > sections.get(i - 1).xM())) { throw new IllegalArgumentException("Sections must be in increasing x"); }
        }
        this.sections = List.copyOf(sections);
        int n = sections.size();
        strip = new double[n];
        for (int i = 0; i < n; i++) {
            double back = i == 0 ? sections.get(0).xM() : 0.5 * (sections.get(i - 1).xM() + sections.get(i).xM());
            double front = i == n - 1 ? sections.get(n - 1).xM() : 0.5 * (sections.get(i).xM() + sections.get(i + 1).xM());
            strip[i] = front - back;
        }
    }

    /**
     * Hull of uniform section shape scaled along its length: half-beam and keel depth follow a fullness profile
     * w(s) in [0, 1] of s = x / (L/2) in [-1, 1] (for example 1 - s^n); the deck edge stays at topY.
     * Sections thinner than 1 % of the midship values are clipped to that so every strip keeps a valid polygon.
     */
    public static SlenderHull tapered(double length, double halfBeam, double keelY, double topY, double bilgeRadius, int stations,
                                      java.util.function.DoubleUnaryOperator fullness) {
        Checks.positive(length, "Length");
        if (stations < 5) { throw new IllegalArgumentException("Use at least five stations"); }
        List<HullSection> list = new ArrayList<>();
        for (int i = 0; i < stations; i++) {
            double s = -1 + 2.0 * i / (stations - 1), w = Math.max(0.01, Math.clamp(fullness.applyAsDouble(s), 0, 1));
            double keel = topY - (topY - keelY) * Math.max(0.15, Math.sqrt(w));
            list.add(HullSection.rounded(s * length / 2, halfBeam * w, keel, topY, bilgeRadius * w, 6));
        }
        return new SlenderHull(list);
    }

    public List<HullSection> sections() { return sections; }
    public int stations() { return sections.size(); }
    public double stripLength(int i) { return strip[i]; }
    public double xAft() { return sections.getFirst().xM(); }
    public double xForward() { return sections.getLast().xM(); }
    public double length() { return xForward() - xAft(); }

    /** Enclosed volume of all sections (m^3), for buoyancy limits and mass checks. */
    public double totalVolume() {
        double v = 0;
        for (int i = 0; i < sections.size(); i++) { v += sections.get(i).area() * strip[i]; }
        return v;
    }

    /** Upright, level displaced volume below local height y. */
    public double volumeBelow(double y) {
        double v = 0;
        for (int i = 0; i < sections.size(); i++) { v += sections.get(i).clip(1, 0, y).areaM2() * strip[i]; }
        return v;
    }

    /** Upright, level waterline height (model-local y) at which the hull displaces the given volume. */
    public double waterlineFor(double volumeM3) {
        Checks.positive(volumeM3, "Displaced volume");
        double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
        for (HullSection s : sections) { low = Math.min(low, s.minY()); high = Math.max(high, s.maxY()); }
        if (volumeM3 >= volumeBelow(high)) { throw new IllegalArgumentException("Hull cannot displace that volume; it would sink"); }
        for (int i = 0; i < 80; i++) {
            double middle = 0.5 * (low + high);
            if (volumeBelow(middle) < volumeM3) { low = middle; } else { high = middle; }
        }
        return 0.5 * (low + high);
    }

    /** Particulars of the upright hull floating at local waterline height y (used for resistance and Clarke). */
    public Particulars particulars(double waterlineY) {
        double volume = 0, moment = 0, wetted = 0, waterplane = 0, maxArea = 0, beam = 0, keel = Double.POSITIVE_INFINITY;
        double forward = Double.NEGATIVE_INFINITY, aft = Double.POSITIVE_INFINITY;
        for (int i = 0; i < sections.size(); i++) {
            HullSection s = sections.get(i);
            HullSection.Wet w = s.clip(1, 0, waterlineY);
            if (w.areaM2() <= 0) { continue; }
            volume += w.areaM2() * strip[i];
            moment += w.areaM2() * strip[i] * s.xM();
            wetted += w.girthM() * strip[i];
            waterplane += w.waterlineBreadthM() * strip[i];
            maxArea = Math.max(maxArea, w.areaM2());
            beam = Math.max(beam, w.waterlineBreadthM());
            keel = Math.min(keel, s.minY());
            forward = Math.max(forward, s.xM()); aft = Math.min(aft, s.xM());
        }
        if (volume <= 0) { throw new IllegalArgumentException("Hull is dry at that waterline"); }
        // Stations are strip centres; the waterline runs to the strip ends.
        int first = 0, last = sections.size() - 1;
        while (sections.get(first).clip(1, 0, waterlineY).areaM2() <= 0) { first++; }
        while (sections.get(last).clip(1, 0, waterlineY).areaM2() <= 0) { last--; }
        double xf = sections.get(last).xM() + (last == sections.size() - 1 ? 0 : 0.5 * (sections.get(last + 1).xM() - sections.get(last).xM()));
        double xa = sections.get(first).xM() - (first == 0 ? 0 : 0.5 * (sections.get(first).xM() - sections.get(first - 1).xM()));
        double length = xf - xa, draft = waterlineY - keel, xb = moment / volume;
        double block = volume / (length * beam * draft), midship = maxArea / (beam * draft);
        double lcb = 100 * (xb - 0.5 * (xf + xa)) / length;
        return new Particulars(waterlineY, length, beam, draft, volume, block, block / midship, midship, waterplane / (length * beam), lcb,
                wetted, xf, xa, xb);
    }

    /**
     * Half-breadth y = f(x, depth) of the upright hull below local waterline height waterlineY, by linear
     * interpolation between stations; zero beyond the end stations. For Michell's thin-ship integral.
     */
    public double halfBreadth(double x, double depthBelowWaterline, double waterlineY) {
        if (x < xAft() || x > xForward() || depthBelowWaterline < 0) { return 0; }
        int i = 1;
        while (i < sections.size() - 1 && sections.get(i).xM() < x) { i++; }
        HullSection a = sections.get(i - 1), b = sections.get(i);
        double t = (x - a.xM()) / (b.xM() - a.xM()), y = waterlineY - depthBelowWaterline;
        return (1 - t) * a.halfBreadthAt(y) + t * b.halfBreadthAt(y);
    }
}
