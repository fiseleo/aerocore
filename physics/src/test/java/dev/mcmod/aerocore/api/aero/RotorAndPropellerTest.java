package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.PropellerCoefficients;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Air propellers against NACA Report 640 (Hartman and Biermann, 1938: the three-blade Clark Y propeller 5868-9, its
 * figs. 11, 12 and 43 read by the 0.22 research pass) and a UH-60A-class rotor against momentum theory and the hover
 * figures of merit of its full-scale, model and flight tests (Norman et al., AHS Forum 2002, fig. 22).
 */
class RotorAndPropellerTest {
    private static final Wing.Air AIR = Wing.Air.SEA_LEVEL;

    /**
     * Report 640's propeller 5868-9 roughly: 3.05 m, three blades, chord about 0.07 D inboard and 0.04 D at the tip
     * (fig. 2 peaks at 0.076 D at half radius), Clark Y-like sections from 20 % to 8 % thick.
     */
    private static AirPropeller report640(double bladeAngle75) {
        double pitch = Math.PI * 0.75 * Math.tan(Math.toRadians(bladeAngle75));
        return new AirPropeller(new AirPropeller.Geometry(3, 3.05, 0.2, 0.07, 0.04, pitch, 0,
                Airfoil.naca4(0.035, 0.3, 0.20), Airfoil.naca4(0.035, 0.3, 0.08)));
    }

    @Test void propellerCurvesFollowReport640() {
        // {blade angle at 0.75 R, peak efficiency, J at the peak, J at zero thrust} (fig. 12)
        double[][] data = {{15, 0.78, 0.64, 0.80}, {25, 0.85, 1.05, 1.28}, {35, 0.85, 1.48, 1.83}, {45, 0.825, 1.93, 2.62}};
        for (double[] d : data) {
            AirPropeller p = report640(d[0]);
            double best = 0, at = 0, zero = Double.NaN;
            for (double j = 0.02; j <= 3.2; j += 0.02) {
                double[] c = p.classic(j, 0, 150, AIR);
                if (c[2] > best) { best = c[2]; at = j; }
                if (Double.isNaN(zero) && c[0] < 0) { zero = j; }
            }
            // blade-element momentum theory without the nacelle: peak efficiency 5-12 % high
            assertEquals(d[1] + 0.06, best, 0.06, "peak efficiency at " + d[0] + " deg");
            assertEquals(d[2], at, 0.12 + 0.12 * d[2], "advance ratio of the (broad) peak at " + d[0] + " deg");
            assertEquals(d[3], zero, 0.12, "advance ratio of zero thrust at " + d[0] + " deg");
        }
        // static thrust at the fine blade angles a governor holds for take-off (fig. 43 with fig. 11's power): 0.14, 0.145
        assertEquals(0.14, report640(15).classic(0, 0, 150, AIR)[0], 0.02);
        assertEquals(0.145, report640(20).classic(0, 0, 150, AIR)[0], 0.02);
    }

    @Test void propellerTableMatchesTheDirectSolveAndAStoppedDiskIsRegular() {
        AirPropeller p = report640(25);
        AirPropeller.Table t = p.tabulate(new AirPropeller.Grid(Math.toRadians(-5), Math.toRadians(15), 5, new double[]{0.2, 0.5}, 2e7));
        // between the table's angles and settings, on one of its Mach numbers
        double beta = Math.toRadians(21.25);
        PropellerCoefficients direct = p.sample(beta, Math.toRadians(2.5), 0.2, 2e7), table = t.model(Math.toRadians(2.5), 0.2).sample(beta);
        assertEquals(direct.thrust(), table.thrust(), 0.03 * Math.abs(direct.thrust()) + 2e-3);
        assertEquals(direct.torque(), table.torque(), 0.03 * Math.abs(direct.torque()) + 2e-4);
        PropellerCoefficients stopped = p.sample(Math.PI / 2, 0, 0.2, 2e7);
        assertTrue(Double.isFinite(stopped.thrust()) && stopped.thrust() < 0, "a stopped propeller drags: " + stopped);
        assertTrue(t.model(Math.toRadians(15), 0.2).sample(Math.toRadians(35)).thrust() > t.model(Math.toRadians(-5), 0.2).sample(Math.toRadians(35)).thrust(),
                "coarser pitch, more thrust at a high pitch angle (fine pitch windmills there)");
    }

    /** A UH-60A-like main rotor: 8.18 m, four blades of 0.527 m, -16 deg twist (-18 deg equivalent in Norman et al.), 9.5 % slightly cambered sections. */
    private static HelicopterRotor blackHawk(boolean clockwise) {
        Airfoil sc = Airfoil.naca4(0.009, 0.3, 0.095);
        return new HelicopterRotor(new HelicopterRotor.Geometry(4, 8.18, 0.2, 0.527, 0.527, -16, sc, sc, 0.0465, 116, 0, clockwise));
    }

    @Test void hoverPowerAndFigureOfMeritAreRealistic() {
        HelicopterRotor r = blackHawk(false);
        double omega = 27, thrust = 74_700;   // 16 800 lb at 258 rpm
        // collective for that thrust by bisection
        double lo = Math.toRadians(4), hi = Math.toRadians(14);
        HelicopterRotor.Loads l = null;
        for (int k = 0; k < 30; k++) {
            double mid = 0.5 * (lo + hi);
            l = r.trimmed(Vec3d.ZERO, omega, AIR, new HelicopterRotor.Controls(mid, 0, 0), Double.POSITIVE_INFINITY);
            if (l.thrustN() < thrust) { lo = mid; } else { hi = mid; }
        }
        assertEquals(thrust, l.thrustN(), 0.01 * thrust);
        double ideal = Math.pow(thrust, 1.5) / Math.sqrt(2 * AIR.densityKgM3() * Math.PI * 8.18 * 8.18);
        double fm = ideal / l.powerW();
        // Norman et al. fig. 22 at C_T/sigma about 0.07: full scale 0.67, models 0.72-0.73, flight 0.66-0.72
        assertEquals(0.70, fm, 0.06, "figure of merit");
        assertEquals(ideal / 0.70, l.powerW(), 0.12 * ideal / 0.70, "main rotor hover power");
        assertTrue(Math.toDegrees(l.next().beta0()) > 1.5 && Math.toDegrees(l.next().beta0()) < 5, "coning " + Math.toDegrees(l.next().beta0()));
        assertEquals(Math.sqrt(thrust / (2 * AIR.densityKgM3() * Math.PI * 8.18 * 8.18)) * HelicopterRotor.INDUCED_FACTOR, l.next().inducedMps(), 0.5,
                "momentum induced velocity");
    }

    @Test void cyclicTiltsTheThrustAndBothSensesOfTurningMirror() {
        for (boolean clockwise : new boolean[]{false, true}) {
            HelicopterRotor r = blackHawk(clockwise);
            double c = Math.toRadians(9), cyc = Math.toRadians(4);
            HelicopterRotor.Loads hover = r.trimmed(Vec3d.ZERO, 27, AIR, new HelicopterRotor.Controls(c, 0, 0), Double.POSITIVE_INFINITY);
            HelicopterRotor.Loads fwd = r.trimmed(Vec3d.ZERO, 27, AIR, HelicopterRotor.Controls.fromSticks(c, 1, 0, cyc, clockwise), Double.POSITIVE_INFINITY);
            HelicopterRotor.Loads right = r.trimmed(Vec3d.ZERO, 27, AIR, HelicopterRotor.Controls.fromSticks(c, 0, 1, cyc, clockwise), Double.POSITIVE_INFINITY);
            String sense = clockwise ? "clockwise" : "counter-clockwise";
            assertEquals(0, hover.force().x(), 0.01 * hover.thrustN(), sense);
            assertTrue(fwd.force().x() > 0.04 * fwd.thrustN(), sense + " forward stick tilts the thrust forward: " + fwd.force());
            assertTrue(right.force().z() > 0.04 * right.thrustN(), sense + " right stick tilts it to starboard: " + right.force());
            // the offset hinge makes a hub moment the same way: nose down (about -z) and right wing down (about +x)
            assertTrue(fwd.moment().z() < 0 && right.moment().x() > 0, sense + " " + fwd.moment() + " " + right.moment());
            assertTrue(hover.torqueNm() > 0, "a hovering rotor needs torque");
        }
    }

    @Test void groundEffectAndAutorotation() {
        HelicopterRotor r = blackHawk(false);
        HelicopterRotor.Controls c = new HelicopterRotor.Controls(Math.toRadians(9), 0, 0);
        double free = r.trimmed(Vec3d.ZERO, 27, AIR, c, Double.POSITIVE_INFINITY).thrustN();
        double near = r.trimmed(Vec3d.ZERO, 27, AIR, c, 0.6 * 8.18).thrustN();
        assertTrue(near > 1.05 * free, "Cheeseman and Bennett: more thrust at the same pitch near the ground");
        // descending at 12 m/s with the collective down, the air drives the rotor (negative shaft torque)
        HelicopterRotor.Loads auto = r.trimmed(new Vec3d(20, -12, 0), 27, AIR, new HelicopterRotor.Controls(Math.toRadians(1), 0, 0), Double.POSITIVE_INFINITY);
        assertTrue(auto.torqueNm() < 0, "autorotation drives the rotor: " + auto.torqueNm());
        assertTrue(auto.thrustN() > 0, "while still lifting");
        // a stopped rotor gives finite loads
        HelicopterRotor.Loads stopped = r.loads(new Vec3d(10, 0, 0), Vec3d.ZERO, 0, AIR, c, Double.POSITIVE_INFINITY, null, 0.005);
        assertTrue(Double.isFinite(stopped.force().length()) && Double.isFinite(stopped.torqueNm()));
    }

    /**
     * A rotor stopped or barely turning on a helicopter that sinks and drifts a little (as when its engine starts): its
     * advance ratio is in the tens, outside Drees' inflow model, which once made the inflow variation hundreds of times
     * the mean, so the thrust fed on itself step after step and the vehicle blew up within a third of a second.
     */
    @Test void aStoppedOrCreepingRotorStaysBoundedOverSteps() {
        HelicopterRotor r = blackHawk(false);
        double bladeArea = 4 * 0.527 * 8.18 * (1 - 0.2);
        for (double omega : new double[]{0, 0.3, 3}) {
            HelicopterRotor.State s = null;
            Vec3d hub = new Vec3d(-0.11, -2, 0.11), rate = new Vec3d(0.05, 0, 0.05);
            double speed2 = hub.lengthSquared() + Math.pow(omega * 8.18, 2), most = 0;
            for (int i = 0; i < 400; i++) {
                HelicopterRotor.Loads l = r.loads(hub, rate, omega, AIR, HelicopterRotor.Controls.ZERO, Double.POSITIVE_INFINITY, s, 0.005);
                most = Math.max(most, l.force().length());
                s = l.next();
            }
            // no more than every blade broadside to the whole relative speed (a flat plate's 2) and some lift
            assertTrue(most < 0.5 * AIR.densityKgM3() * speed2 * bladeArea * 2.5, "at " + omega + " rad/s the hub force reached " + most + " N");
            assertTrue(s.inducedMps() < 2, "and the inflow " + s.inducedMps() + " m/s");
        }
    }
}
