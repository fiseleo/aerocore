package dev.mcmod.aerocore.api.vehicle;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Clearance (0.33): the least height over the ground and the SAE J1100 ramp angles, against constructed ramps. */
class ClearanceTest {
    static final double R = 0.33;

    /** A point on the ramp at angle a tangent to a wheel at x (centre height R), s along it from the tangent point, toward dir. */
    static double[] onRamp(double x, double a, double s, int dir) {
        double tx = x + dir * R * Math.sin(a), ty = R - R * Math.cos(a);
        return new double[]{tx + dir * s * Math.cos(a), ty + s * Math.sin(a)};
    }

    @Test void aFlatFloorBetweenTheWheels() {
        List<double[]> floor = new ArrayList<>();
        for (int i = 0; i <= 40; i++) { floor.add(new double[]{-2 + 0.1 * i, 0.2}); }
        Clearance.Result r = Clearance.of(floor, List.of(new Clearance.Wheel(1.35, R, R), new Clearance.Wheel(-1.35, R, R)));
        assertEquals(0.2, r.minClearanceM(), 1e-12);
        assertTrue(r.approachRad() > 0 && r.departureRad() > 0 && r.breakoverRad() > 0);
    }

    @Test void theRampsAreTangentToTheTyres() {
        double a = Math.toRadians(30), d = Math.toRadians(22), b1 = Math.toRadians(9), b2 = Math.toRadians(7);
        List<double[]> under = new ArrayList<>();
        under.add(onRamp(1.4, a, 0.25, 1));        // the front overhang's lower corner
        under.add(onRamp(1.4, a + 0.2, 0.6, 1));   // higher up the bumper: not limiting
        under.add(onRamp(-1.4, d, 0.4, -1));        // the rear overhang
        // the breakover point: where a ramp at b1 from the rear tyre meets one at b2 from the front tyre
        double[] fromRear = onRamp(-1.4, b1, 0, 1), fromFront = onRamp(1.4, b2, 0, -1);
        // solve fromRear + s1 (cos b1, sin b1) = fromFront + s2 (-cos b2, sin b2)
        double det = Math.cos(b1) * Math.sin(b2) + Math.sin(b1) * Math.cos(b2);
        double dx = fromFront[0] - fromRear[0], dy = fromFront[1] - fromRear[1];
        double s1 = (dx * Math.sin(b2) + dy * Math.cos(b2)) / det;
        under.add(new double[]{fromRear[0] + s1 * Math.cos(b1), fromRear[1] + s1 * Math.sin(b1)});
        Clearance.Result r = Clearance.of(under, List.of(new Clearance.Wheel(1.4, R, R), new Clearance.Wheel(-1.4, R, R)));
        assertEquals(a, r.approachRad(), 1e-9, "approach");
        assertEquals(d, r.departureRad(), 1e-9, "departure");
        assertEquals(b1 + b2, r.breakoverRad(), 1e-9, "breakover");
        // the same tilted: the ground is under the wheels whatever the vehicle's pitch
        double t = Math.toRadians(6), c = Math.cos(t), sn = Math.sin(t);
        List<double[]> tilted = new ArrayList<>();
        for (double[] p : under) { tilted.add(new double[]{p[0] * c - p[1] * sn, p[0] * sn + p[1] * c}); }
        Clearance.Result q = Clearance.of(tilted, List.of(new Clearance.Wheel(1.4 * c - R * sn, 1.4 * sn + R * c, R), new Clearance.Wheel(-1.4 * c - R * sn, -1.4 * sn + R * c, R)));
        assertEquals(a, q.approachRad(), 1e-9);
        assertEquals(d, q.departureRad(), 1e-9);
        assertEquals(b1 + b2, q.breakoverRad(), 1e-9);
    }

    @Test void threeAxlesHaveTwoBreakoversAndALowPointIsFound() {
        List<double[]> under = List.of(new double[]{2, 0.5}, new double[]{0.7, 0.4}, new double[]{-0.7, -0.05}, new double[]{-2, 0.5});
        Clearance.Result r = Clearance.of(under, List.of(new Clearance.Wheel(1.5, 0.45, 0.45), new Clearance.Wheel(0, 0.45, 0.45), new Clearance.Wheel(-1.5, 0.45, 0.45)));
        assertEquals(2, r.breakoversRad().length);
        assertEquals(-0.05, r.minClearanceM(), 1e-12);
        assertEquals(-0.7, r.minAtXM(), 1e-12);
        assertEquals(0, r.breakoversRad()[1], 1e-12, "a point under the ground between the middle and rear axles");
        assertTrue(r.breakoversRad()[0] > 0);
        // nothing ahead of the front wheel: no limit
        assertEquals(Math.PI / 2, Clearance.of(List.of(new double[]{0, 0.3}), List.of(new Clearance.Wheel(1, R, R), new Clearance.Wheel(-1, R, R))).approachRad(), 0);
    }
}
