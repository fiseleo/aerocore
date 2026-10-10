package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class RowboatTest {
    private static final double RHO = 1000, DT = 0.05, G = Atmosphere.GRAVITY_MPS2;
    private static final Rowboat.Passenger PERSON = new Rowboat.Passenger(62, 0, Rowboat.FLOOR + Rowboat.SEATED_HEIGHT, 5.5);

    private static Rowboat.Water calm(double level) {
        return new Rowboat.Water() {
            @Override public void sample(double x, double z, double[] out) { out[0] = level; out[1] = 0; out[2] = 0; }
            @Override public Vec3d velocity(double x, double z) { return Vec3d.ZERO; }
        };
    }

    private static WaveRadiation.Memory memory() {
        List<Vec3d> points = new ArrayList<>();
        for (double[] p : Rowboat.RADIATION_POINTS) { points.add(new Vec3d(p[0], 0, p[1])); }
        return new WaveRadiation.Memory(points, DT);
    }

    private static Rowboat.State at(double y) { return new Rowboat.State(new Vec3d(0, y, 0), Vec3d.ZERO, 0, 0, 0, 0, 0, 0); }

    @Test void floatsAtItsHydrostaticDraftAndTrim() {
        // Two people at vanilla's seats (0.2 m forward, 0.6 m aft): sinkage and trim follow from the waterplane.
        List<Rowboat.Passenger> crew = List.of(new Rowboat.Passenger(62, 0.2, PERSON.heightM(), 5.5), new Rowboat.Passenger(62, -0.6, PERSON.heightM(), 5.5));
        Rowboat.Mass mass = Rowboat.mass(crew);
        WaveRadiation.Memory memory = memory();
        Rowboat.State s = at(-0.02);
        Rowboat.Result r = null;
        for (int i = 0; i < 400; i++) { r = Rowboat.step(s, crew, Rowboat.Oars.NONE, calm(0), null, RHO, DT, memory); s = r.state(); }
        double draft = Rowboat.draft(mass.massKg(), RHO);
        assertEquals(draft, -s.bottom().y(), 0.002, "sinkage");
        double inertia = Rowboat.BOTTOM_BEAM * Math.pow(Rowboat.BOTTOM_LENGTH, 3) / 12;
        // Small trim: theta = -LCG / GM_L, GM_L = KB + I_L / V - KG (bow up positive when the weight is aft).
        double gml = draft / 2 + inertia / (mass.massKg() / RHO) - mass.heightM(), expected = -mass.forwardM() / gml;
        System.out.printf("Two aboard: %.1f kg, draft %.4f m (hydrostatic %.4f), pitch %.2f deg (expected %.2f)%n", mass.massKg(), -s.bottom().y(), draft,
                Math.toDegrees(s.pitch()), Math.toDegrees(expected));
        assertEquals(expected, s.pitch(), 0.1 * Math.abs(expected));
        assertTrue(s.pitch() > 0, "Bow up (stern down) with the aft seat taken");
        assertFalse(r.swamped());
    }

    @Test void rollsWithTheExpectedPeriod() {
        List<Rowboat.Passenger> crew = List.of(PERSON);
        Rowboat.Mass mass = Rowboat.mass(crew);
        double draft = Rowboat.draft(mass.massKg(), RHO);
        Rowboat.State s = new Rowboat.State(new Vec3d(0, -draft, 0), Vec3d.ZERO, 0, 0, Math.toRadians(4), 0, 0, 0);
        WaveRadiation.Memory memory = memory();
        double previous = s.roll(), lastCross = Double.NaN;
        List<Double> periods = new ArrayList<>();
        for (int i = 1; i < 200; i++) {
            s = Rowboat.step(s, crew, Rowboat.Oars.NONE, calm(0), null, RHO, DT, memory).state();
            if (previous > 0 && s.roll() <= 0) {
                double t = (i - 1 + previous / (previous - s.roll())) * DT;
                if (!Double.isNaN(lastCross)) { periods.add(t - lastCross); }
                lastCross = t;
            }
            previous = s.roll();
        }
        // GM = KB + BM - KG for the bottom waterplane; period 2 pi sqrt(I / (W GM)) without added inertia.
        double bm = Rowboat.BOTTOM_LENGTH * Math.pow(Rowboat.BOTTOM_BEAM, 3) / 12 / (mass.massKg() / RHO);
        double gm = draft / 2 + bm - mass.heightM();
        double dry = 2 * Math.PI * Math.sqrt(mass.roll() / (mass.massKg() * G * gm));
        System.out.printf("Roll: GM %.2f m, dry period %.2f s, simulated periods %s%n", gm, dry, periods);
        assertFalse(periods.isEmpty());
        // Added inertia lengthens the period; 0.6-1.6 s is the range for a crewed dinghy of this beam.
        assertTrue(periods.get(0) > dry && periods.get(0) < 1.6, "Roll period " + periods.get(0));
    }

    @Test void rowsAtDinghySpeedAndTurnsOnOneOar() {
        List<Rowboat.Passenger> crew = List.of(PERSON);
        double draft = Rowboat.draft(Rowboat.mass(crew).massKg(), RHO);
        WaveRadiation.Memory memory = memory();
        Rowboat.State s = at(-draft);
        Rowboat.Result r = null;
        for (int i = 0; i < 1200; i++) { r = Rowboat.step(s, crew, new Rowboat.Oars(0.5, 0.5), calm(0), null, RHO, DT, memory); s = r.state(); }
        double speed = s.velocity().dot(Rowboat.forward(s.yaw()));
        System.out.printf("Rowing at %.0f W: %.2f m/s, thrust %.1f N, resistance %.1f N (waves %.1f N), yaw %.4f%n", Rowboat.ROWER_POWER, speed, r.thrustN(),
                r.resistanceN(), r.waveResistanceN(), s.yaw());
        // Prams top out around 3-4 knots; cruising 1.0-1.4 m/s and sprinting 1.6-1.8 m/s for a 2 m boat.
        assertTrue(speed > 1.1 && speed < 1.9, "Speed " + speed);
        assertEquals(r.thrustN(), r.resistanceN(), 0.05 * r.thrustN(), "Steady: thrust balances resistance");
        assertEquals(0, s.yaw(), 1e-6, "Equal oars row straight");
        // Pulling only the right oar turns the boat left (Minecraft yaw decreases), as vanilla's left key does.
        Rowboat.State turn = at(-draft);
        for (int i = 0; i < 40; i++) { turn = Rowboat.step(turn, crew, new Rowboat.Oars(0, 0.5), calm(0), null, RHO, DT, memory()).state(); }
        System.out.printf("One oar for 2 s: yaw %.1f deg, rate %.1f deg/s%n", Math.toDegrees(turn.yaw()), Math.toDegrees(turn.yawRate()));
        assertTrue(turn.yaw() < Math.toRadians(-10), "One oar must turn the boat: " + Math.toDegrees(turn.yaw()));
    }

    @Test void followsLongWaves() {
        // A 4 s, 25 m wave: much longer than the boat and its ~1 s natural periods, so it rides the surface and its slope.
        LinearWave wave = new LinearWave(0.25, 25, 0, Double.POSITIVE_INFINITY, 0);
        double[] time = {0};
        Rowboat.Water water = new Rowboat.Water() {
            @Override public void sample(double x, double z, double[] out) {
                out[0] = wave.elevation(x, z, time[0]);
                Vec3d slope = wave.slope(x, z, time[0]);
                out[1] = slope.x(); out[2] = slope.z();
            }
            @Override public Vec3d velocity(double x, double z) { return wave.velocity(new Vec3d(x, wave.elevation(x, z, time[0]), z), 0, time[0]); }
        };
        List<Rowboat.Passenger> crew = List.of(PERSON);
        double draft = Rowboat.draft(Rowboat.mass(crew).massKg(), RHO);
        // Bow toward +x (yaw -90 deg: forward = (1, 0, 0)), into the wave's direction of travel.
        Rowboat.State s = new Rowboat.State(new Vec3d(0, wave.elevation(0, 0, 0) - draft, 0), Vec3d.ZERO, -Math.PI / 2, 0, 0, 0, 0, 0);
        WaveRadiation.Memory memory = memory();
        double minY = 1e9, maxY = -1e9, maxPitch = 0;
        for (int i = 0; i < 400; i++) {
            s = Rowboat.step(s, crew, Rowboat.Oars.NONE, water, null, RHO, DT, memory).state();
            time[0] += DT;
            if (i > 160) { minY = Math.min(minY, s.bottom().y()); maxY = Math.max(maxY, s.bottom().y()); maxPitch = Math.max(maxPitch, Math.abs(s.pitch())); }
        }
        double heave = (maxY - minY) / 2, slope = wave.amplitudeM() * wave.wavenumber();
        System.out.printf("Long wave: heave %.3f m (wave %.3f m), pitch %.2f deg (slope %.2f deg)%n", heave, wave.amplitudeM(), Math.toDegrees(maxPitch), Math.toDegrees(slope));
        assertEquals(wave.amplitudeM(), heave, 0.15 * wave.amplitudeM());
        assertEquals(slope, maxPitch, 0.25 * slope);
    }

    @Test void anOverloadedBoatSwamps() {
        List<Rowboat.Passenger> cows = List.of(new Rowboat.Passenger(650, 0.2, 0.9, 0), new Rowboat.Passenger(650, -0.6, 0.9, 0));
        assertTrue(Double.isNaN(Rowboat.draft(Rowboat.mass(cows).massKg(), RHO)), "1335 kg exceeds the boat's 1266 kg displacement");
        WaveRadiation.Memory memory = memory();
        Rowboat.State s = at(-0.3);
        Rowboat.Result r = null;
        for (int i = 0; i < 100; i++) { r = Rowboat.step(s, cows, Rowboat.Oars.NONE, calm(0), null, RHO, DT, memory); s = r.state(); }
        assertTrue(r.swamped() && s.bottom().y() < -Rowboat.DEPTH, "It sinks: " + s.bottom().y());
    }

    /** Rows (or turns) with the boat's own unsteady wake in its water; returns the final state. */
    private static Rowboat.State coupled(Rowboat.Oars oars, int ticks, double[] resistance) {
        List<Rowboat.Passenger> crew = List.of(PERSON);
        double draft = Rowboat.draft(Rowboat.mass(crew).massKg(), RHO);
        UnsteadyWake.Forcing forcing = Rowboat.wakeForcing(draft, RHO);
        UnsteadyWake wake = new UnsteadyWake(128, 0.25, Double.POSITIVE_INFINITY, RHO, G, DT);
        Rowboat.State s = at(-draft);
        wake.reset(0, 0, new UnsteadyWake.Load(forcing, 0, 0, 0, 1, 0));
        WaveRadiation.Memory memory = memory();
        double[] local = new double[3];
        for (int i = 0; i < ticks; i++) {
            UnsteadyWake.Snapshot snap = wake.snapshot();
            Rowboat.Water own = new Rowboat.Water() {
                @Override public void sample(double x, double z, double[] out) {
                    snap.sample(x, z, local);
                    out[0] = local[0]; out[1] = local[1]; out[2] = local[2];
                }
                @Override public Vec3d velocity(double x, double z) { return Vec3d.ZERO; }
            };
            Rowboat.Result r = Rowboat.step(s, crew, oars, calm(0), own, RHO, DT, memory);
            s = r.state();
            if (resistance != null && i >= ticks - 100) { resistance[0] += r.resistanceN() / 100; }
            Vec3d f = Rowboat.forward(s.yaw());
            // Grid centred 8 m ahead along the heading, as Wakes leads it.
            wake.step(s.bottom().x() + 8 * f.x(), s.bottom().z() + 8 * f.z(),
                    new UnsteadyWake.Load(forcing, s.bottom().x(), s.bottom().z(), f.x(), f.z(), s.velocity().dot(f)));
        }
        return s;
    }

    @Test void itsOwnWakeCarriesTheHavelockResistance() {
        double[] friction = {0};
        Rowboat.State s = coupled(new Rowboat.Oars(0.5, 0.5), 1200, friction);
        double speed = s.velocity().dot(Rowboat.forward(s.yaw()));
        // Same rower in the quasi-steady mode.
        List<Rowboat.Passenger> crew = List.of(PERSON);
        double draft = Rowboat.draft(Rowboat.mass(crew).massKg(), RHO);
        Rowboat.State q = at(-draft);
        WaveRadiation.Memory memory = memory();
        for (int i = 0; i < 1200; i++) { q = Rowboat.step(q, crew, new Rowboat.Oars(0.5, 0.5), calm(0), null, RHO, DT, memory).state(); }
        double steady = q.velocity().dot(Rowboat.forward(q.yaw()));
        System.out.printf("Own wake: %.3f m/s (resistance %.1f N); quasi-steady Havelock: %.3f m/s%n", speed, friction[0], steady);
        assertEquals(steady, speed, 0.1 * steady);
        // One oar, with the wake: the turning hull's waves resist the yaw.
        Rowboat.State turn = coupled(new Rowboat.Oars(0, 0.5), 40, null);
        System.out.printf("One oar for 2 s with its wake: yaw %.1f deg, rate %.1f deg/s%n", Math.toDegrees(turn.yaw()), Math.toDegrees(turn.yawRate()));
    }

    @Test void oarThrustFollowsBladeSlip() {
        // At rest all the drive power goes into the water: (rho/2) C A v^3 = P / f.
        double k = 0.5 * RHO * Rowboat.BLADE_DRAG * Rowboat.BLADE_AREA, drive = 75 / Rowboat.DRIVE_FRACTION;
        double v = Math.cbrt(drive / k);
        assertEquals(Rowboat.DRIVE_FRACTION * k * v * v, Rowboat.oarThrust(75, 0, RHO), 1e-9);
        // Moving, the blade efficiency U / (U + v) rises with speed and thrust x (U + v) stays the drive power.
        double t1 = Rowboat.oarThrust(75, 1, RHO), t2 = Rowboat.oarThrust(75, 2, RHO);
        assertTrue(t2 < t1);
        double slip = Math.sqrt(t1 / Rowboat.DRIVE_FRACTION / k);
        assertEquals(drive, t1 / Rowboat.DRIVE_FRACTION * (1 + slip), 1e-6);
    }
}
