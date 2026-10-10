package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class PolarTableTest {
    private static PolarTable table(double scale) {
        return new PolarTable(List.of(
                new PolarTable.Point(-0.2, new AeroCoefficients(-scale, 0.04, -0.1, false)),
                new PolarTable.Point(0.2, new AeroCoefficients(scale, 0.08, -0.3, true))));
    }

    @Test void interpolatesAllCoefficientsAndPreservesNodes() {
        AeroCoefficients middle = table(1).sample(0);
        assertEquals(0, middle.lift(), 1e-12);
        assertEquals(0.06, middle.drag(), 1e-12);
        assertEquals(-0.2, middle.pitchingMoment(), 1e-12);
        assertTrue(middle.stalled(), "Between nodes either stalled flag is treated conservatively");
        assertEquals(-1, table(1).sample(-0.2).lift());
        assertEquals(1, table(1).sample(0.2).lift());
    }

    @Test void reynoldsInterpolationUsesLogarithmicCoordinate() {
        ReynoldsPolar polar = new ReynoldsPolar(List.of(new ReynoldsPolar.Slice(1e4, table(1)),
                new ReynoldsPolar.Slice(1e6, table(3))), 0.3);
        assertEquals(2, polar.sample(0.2, new FlowConditions(20, 1e5, 0.1)).lift(), 1e-12);
        assertEquals(1, polar.sample(0.2, new FlowConditions(20, 1e4, 0.1)).lift(), 1e-12);
        assertEquals(3, polar.sample(0.2, new FlowConditions(20, 1e6, 0.1)).lift(), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> polar.sample(0.2));
        assertThrows(IllegalArgumentException.class, () -> polar.sample(0, new FlowConditions(20, 1e3, 0.1)));
        assertThrows(IllegalArgumentException.class, () -> polar.sample(0, new FlowConditions(20, 1e5, 0.4)));
    }

    @Test void invalidOrOutOfRangeDataIsRejectedInsteadOfExtrapolated() {
        assertThrows(IllegalArgumentException.class, () -> table(1).sample(0.21));
        assertThrows(IllegalArgumentException.class, () -> table(1).sample(Double.NaN));
        PolarTable.Point point = new PolarTable.Point(0, new AeroCoefficients(0, 0, false));
        assertThrows(IllegalArgumentException.class, () -> new PolarTable(List.of(point, point)));
        assertThrows(IllegalArgumentException.class, () -> new AeroCoefficients(0, 0, Double.NaN, false));
    }
}
