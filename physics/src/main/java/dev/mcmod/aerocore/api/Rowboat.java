package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * A small flat-bottomed rowing boat (a pram) with the dimensions of Minecraft's boat, rowed by a person: the physics
 * behind vanilla boats. Heave, pitch, roll, surge, sway and yaw are integrated in a heading frame with small pitch and
 * roll (their gyroscopic coupling is neglected).
 * <ul>
 * <li>Geometry (Minecraft's boat model, 1 px = 1/16 m): a 1.75 x 1.0 m bottom slab up to FLOOR, then side and end walls
 * whose outline is 2.0 x 1.25 m, up to DEPTH. The bottom-centre point is the boat's reference (Minecraft's position).</li>
 * <li>Hydrostatics: vertical columns over 52 cells of the hull outline, each displacing rho g A (surface - bottom),
 * capped at the gunwale (a swamped boat sinks), on the instantaneous surface (waves and wakes included), and the
 * long-wave Froude-Krylov force -rho g V grad(eta) horizontally.</li>
 * <li>Heave, pitch and roll radiation: four hemisphere-equivalent points (one per quarter of the bottom) with Hulme's
 * added mass at infinite frequency and the Cummins memory of WaveRadiation, as for AeroCore's floats; plus quadratic
 * cross-flow drag C_D = 1 on each wet cell's vertical motion (Zarnick's C_Dc = cos(deadrise) at zero deadrise).</li>
 * <li>Resistance: ITTC-1957 friction on the wetted surface with Hoerner's form factor of the double body; Hoerner's
 * cross-flow drag on the immersed sides strip by strip; and wave resistance, either from the boat's own unsteady wake
 * (wakeForcing: the bottom as the surface pressure rho g T of Tuck's flat ship, each wave reduced by e^{-k T} for the
 * bottom's depth) pushing on the hull through the Froude-Krylov term, or quasi-steadily from Havelock's integral for
 * the same pressure (waveResistance).</li>
 * <li>Oars: each blade is a flat plate (C_D 1.2, Hoerner) slipping through the water at v_s, with
 * (rho/2) C_D A v_s^2 (U + v_s) = P_drive: the drive power goes partly into the boat (U) and partly into the water
 * (v_s), Kleshnev's (1999) blade efficiency U / (U + v_s). The stroke-averaged thrust is the drive fraction times
 * the drive force. Oar size by Shaw and Tenney's rule for the beam. The blade moves aft relative to the boat no faster
 * than the rower sweeps it (BLADE_SPEED, from handle speed and oar geometry), so its slip through the water is at most
 * BLADE_SPEED - U: a fast boat outruns its oars, and one oar cannot spin the boat faster than its blade moves aft
 * (the oarlock side swings forward at the yaw rate times the reach).</li>
 * <li>Wind: quadratic drag of the relative wind on the hull's above-water profile, C_D 0.90 on its side (Blendermann
 * 2004, Proc. IMechE M 218:1, speed boat) and, for lack of small-craft data, the same on its end (a short bluff box),
 * acting at the freeboard's middle; and on each person aboard, C_D A 0.35 m^2 facing the wind and 0.25 m^2 side-on for
 * a seated adult above the gunwale (upright cyclists 0.27-0.52 m^2 with their bicycle, Crouch et al. 2017, Sports Eng.
 * 20:81), scaled by (mass / 75 kg)^(2/3) for other riders (geometric similarity, a rule), 0.15 m above their centre of
 * mass. The wind at each height comes from SeaWind's log profile above the still water level.</li>
 * </ul>
 * Not modelled: the stroke-by-stroke surge, the crew balancing the boat, spray and water shipped over the gunwale,
 * gusts, and ground contact (on land the boat is vanilla's level box; the caller relaxes tilt there).
 */
public final class Rowboat {
    public static final double BOTTOM_LENGTH = 1.75, BOTTOM_BEAM = 1.0, FLOOR = 0.1875, LENGTH = 2.0, BEAM = 1.25, DEPTH = 0.5625;
    /** Hull mass, kg: plywood prams of this size weigh 29.5 kg (CLC Eastport, 2.36 x 1.22 m) and 41 kg (Nutshell 7'7"); their mean. */
    public static final double HULL_MASS = 35;
    /** Sustained rowing power, W: the middle of the 100-200 W a fit amateur keeps up for 10-60 min (Wilson 2004, Bicycling Science fig. 2.4; Concept2). */
    public static final double ROWER_POWER = 150;
    /** Dinghy oar blade 0.13 x 0.45 m (typical 0.12-0.15 x 0.4-0.5 m) and its normal drag coefficient (Hoerner 1965, flat plates of aspect 3-4). */
    public static final double BLADE_AREA = 0.13 * 0.45, BLADE_DRAG = 1.2;
    /** Share of a stroke spent driving: 35-50 % for fixed-seat rowing; the middle. */
    public static final double DRIVE_FRACTION = 0.45;
    /** Strokes per second: leisure fixed-seat rowing at 15-25 strokes per minute; 24. */
    public static final double STROKE_RATE = 24 / 60.0;
    /**
     * Blade centre to the boat's centreline with the oar square to the boat: oarlocks at the gunwales (span = BEAM),
     * Shaw and Tenney: inboard = span / 2 + 2 in, length = inboard x 25 / 7; the blade centre half a blade from the tip.
     */
    public static final double OAR_REACH;
    /**
     * Handle speed averaged over the drive, m/s: on-water single scullers average 1.28 m/s at 20 and 1.59 m/s at 32
     * strokes per minute (Kleshnev 2005, ISBS Beijing pp. 130-133; peaks 2.06-2.36 m/s). No dinghy measurement was
     * found; a recreational fixed-seat stroke is taken at the low end, 1.3 m/s.
     */
    public static final double HANDLE_SPEED = 1.3;
    /**
     * Sweep arc of a dinghy stroke, rad: 85 deg (international sweep crews 85-90 deg, scullers 100-105 deg, Koerner,
     * FISA Coaching Level III; a fixed-seat stroke without leg drive is estimated at 70-100 deg).
     */
    public static final double SWEEP_ARC = Math.toRadians(85);
    /**
     * Fastest the blade centre moves aft relative to the boat, averaged over the drive, m/s: HANDLE_SPEED / inboard
     * times the pin-to-blade-centre length, times the mean of cos over the arc, sin(arc/2) / (arc/2).
     */
    public static final double BLADE_SPEED;
    /** Height of a seated person's centre of mass above the floor, m (0.25-0.3 m above the seat). */
    public static final double SEATED_HEIGHT = 0.30;
    /** Wind drag coefficient of the above-water hull, side and end (Blendermann 2004, speed boat side-on). */
    public static final double WINDAGE = 0.90;
    /** Drag areas C_D A of a seated 75 kg adult above the gunwale, facing the wind and side-on, m^2. */
    public static final double PERSON_DRAG_FRONT = 0.35, PERSON_DRAG_SIDE = 0.25, PERSON_MASS = 75;
    /** Cross-flow drag coefficient of a wet cell moving vertically (Zarnick 1978, C_Dc = cos(beta), beta = 0). */
    public static final double HEAVE_DRAG = 1.0;
    static {
        double inboard = BEAM / 2 + 0.0508, length = inboard * 25 / 7, blade = length - inboard - 0.45 / 2;
        OAR_REACH = BEAM / 2 + blade;
        BLADE_SPEED = HANDLE_SPEED / inboard * blade * Math.sin(SWEEP_ARC / 2) / (SWEEP_ARC / 2);
    }

    private static final double G = Atmosphere.GRAVITY_MPS2;
    private static final int SUBSTEPS = 10;
    /** Hydrostatic cells: centre (forward a, right c), size (da, dc) and the height their column starts at. */
    private static final double[][] CELLS;
    /** Strips for the lateral cross-flow drag: centre a and length. */
    private static final double[][] STRIPS;
    /** Radiation points: quarter-bottom centres (forward, right). */
    public static final double[][] RADIATION_POINTS = {{0.4375, 0.25}, {0.4375, -0.25}, {-0.4375, 0.25}, {-0.4375, -0.25}};
    static {
        List<double[]> cells = new ArrayList<>(), strips = new ArrayList<>();
        for (int i = 0; i < 7; i++) {
            double a = -BOTTOM_LENGTH / 2 + 0.25 * (i + 0.5);
            strips.add(new double[]{a, 0.25});
            for (int j = 0; j < 4; j++) { cells.add(new double[]{a, -BOTTOM_BEAM / 2 + 0.25 * (j + 0.5), 0.25, 0.25, 0}); }
            for (int side = -1; side <= 1; side += 2) { cells.add(new double[]{a, side * (BOTTOM_BEAM / 2 + 0.0625), 0.25, 0.125, FLOOR}); }
        }
        for (int end = -1; end <= 1; end += 2) {
            strips.add(new double[]{end * (BOTTOM_LENGTH / 2 + 0.0625), 0.125});
            for (int j = 0; j < 5; j++) { cells.add(new double[]{end * (BOTTOM_LENGTH / 2 + 0.0625), -BEAM / 2 + 0.25 * (j + 0.5), 0.125, 0.25, FLOOR}); }
        }
        CELLS = cells.toArray(new double[0][]);
        STRIPS = strips.toArray(new double[0][]);
    }

    private Rowboat() {}

    /** Someone or something on board: mass, position of its centre of mass (forward, height above the bottom), own inertia. */
    public record Passenger(double massKg, double forwardM, double heightM, double ownInertiaKgM2) {}

    /** Oar commands: share of the rower's power on each oar, -1 to 1 (negative backs water). */
    public record Oars(double left, double right) {
        public static final Oars NONE = new Oars(0, 0);
    }

    /**
     * State: world position of the bottom centre (Minecraft's boat position), velocity of the centre of mass (m/s),
     * yaw as Minecraft's yRot (rad: forward = (-sin, 0, cos)), pitch (bow up positive) and roll (right side down
     * positive), and their rates (rad/s).
     */
    public record State(Vec3d bottom, Vec3d velocity, double yaw, double pitch, double roll, double yawRate, double pitchRate, double rollRate) {}

    /**
     * The water around the boat: sample writes the surface height (NaN where there is none) and its slopes d/dx, d/dz
     * into out[0..2]; velocity is the water's velocity at the surface.
     */
    public interface Water {
        void sample(double x, double z, double[] out);
        Vec3d velocity(double x, double z);
    }

    /** The air: the 10 m wind (horizontal, m/s, the way it blows) and the air density (kg/m^3). */
    public record Air(Vec3d wind10, double density) {
        public static final Air CALM = new Air(Vec3d.ZERO, 1.225);
    }

    /** One tick's result with diagnostics (forces averaged over the tick). */
    public record Result(State state, double draftM, double thrustN, double resistanceN, double waveResistanceN, double wetFraction, boolean swamped) {}

    /** Mass properties: total mass, centre of mass (forward, height) and inertias about it (pitch, roll, yaw), kg m^2. */
    public record Mass(double massKg, double forwardM, double heightM, double pitch, double roll, double yaw) {}

    private static final double[][] PANELS;
    static {
        // Hull shell as thin plywood panels {a, c, h, area-weight}: bottom, sides, ends, discretised in 0.125 m patches.
        List<double[]> panels = new ArrayList<>();
        double step = 0.125;
        for (double a = -BOTTOM_LENGTH / 2 + step / 2; a < BOTTOM_LENGTH / 2; a += step)
            for (double c = -BOTTOM_BEAM / 2 + step / 2; c < BOTTOM_BEAM / 2; c += step) { panels.add(new double[]{a, c, 0, step * step}); }
        for (double h = FLOOR + step / 2; h < DEPTH; h += step) {
            for (double a = -LENGTH / 2 + step / 2; a < LENGTH / 2; a += step) for (int side = -1; side <= 1; side += 2) { panels.add(new double[]{a, side * BEAM / 2, h, step * step}); }
            for (double c = -BEAM / 2 + step / 2; c < BEAM / 2; c += step) for (int end = -1; end <= 1; end += 2) { panels.add(new double[]{end * LENGTH / 2, c, h, step * step}); }
        }
        PANELS = panels.toArray(new double[0][]);
    }

    /** Hull (HULL_MASS spread over its plywood panels) plus passengers as point masses with their own inertia. */
    public static Mass mass(List<Passenger> passengers) {
        double area = 0;
        for (double[] p : PANELS) { area += p[3]; }
        double m = HULL_MASS, sa = 0, sh = 0;
        for (double[] p : PANELS) { double w = HULL_MASS * p[3] / area; sa += w * p[0]; sh += w * p[2]; }
        for (Passenger q : passengers) { m += q.massKg(); sa += q.massKg() * q.forwardM(); sh += q.massKg() * q.heightM(); }
        double a0 = sa / m, h0 = sh / m, pitch = 0, roll = 0, yaw = 0;
        for (double[] p : PANELS) {
            double w = HULL_MASS * p[3] / area, a = p[0] - a0, c = p[1], h = p[2] - h0;
            pitch += w * (a * a + h * h); roll += w * (c * c + h * h); yaw += w * (a * a + c * c);
        }
        for (Passenger q : passengers) {
            double a = q.forwardM() - a0, h = q.heightM() - h0;
            pitch += q.massKg() * (a * a + h * h) + q.ownInertiaKgM2(); roll += q.massKg() * h * h + q.ownInertiaKgM2(); yaw += q.massKg() * a * a + q.ownInertiaKgM2();
        }
        return new Mass(m, a0, h0, pitch, roll, yaw);
    }

    /**
     * The bottom as a surface pressure for its waves: the draft distribution of a pram bottom with rocker, parabolic along
     * the length (peak 1.5 rho g T, so it carries the weight rho g T A) and uniform across the beam, acting at the
     * bottom's depth (UnsteadyWake.submerged). Minecraft's model is a flat box; real prams curve their bottom up toward
     * both transoms by more than their draft, and the sharp-edged box would make humps and hollows of wave resistance
     * no pram shows (Doctors and Sharma 1972 on sharp pressure edges). Body frame centred on the bottom.
     */
    public static UnsteadyWake.Forcing wakeForcing(double draft, double density) {
        if (!(draft > 0)) { return UnsteadyWake.Forcing.NONE; }
        return UnsteadyWake.submerged(UnsteadyWake.parabolic(1.5 * density * G * draft, -BOTTOM_LENGTH / 2, BOTTOM_LENGTH / 2, BOTTOM_BEAM / 2), draft);
    }

    /** Level-floating draft in still water, m (NaN if the boat cannot float). */
    public static double draft(double massKg, double density) {
        double low = BOTTOM_LENGTH * BOTTOM_BEAM * FLOOR * density;
        if (massKg <= low) { return massKg / (density * BOTTOM_LENGTH * BOTTOM_BEAM); }
        double high = low + LENGTH * BEAM * (DEPTH - FLOOR) * density;
        return massKg > high ? Double.NaN : FLOOR + (massKg - low) / (density * LENGTH * BEAM);
    }

    /** Heading-frame unit vectors for Minecraft yaw (rad). */
    static Vec3d forward(double yaw) { return new Vec3d(-Math.sin(yaw), 0, Math.cos(yaw)); }
    static Vec3d right(double yaw) { return new Vec3d(-Math.cos(yaw), 0, -Math.sin(yaw)); }

    /** World offset of a body point (forward a, right c, height h above the bottom) from the bottom centre. */
    static Vec3d offset(double yaw, double pitch, double roll, double a, double c, double h) {
        double cp = Math.cos(pitch), sp = Math.sin(pitch), cr = Math.cos(roll), sr = Math.sin(roll);
        return forward(yaw).scale(a * cp - h * sp).add(right(yaw).scale(c * cr + h * sr)).add(Vec3d.Y.scale(h * cp * cr + a * sp - c * sr));
    }

    /**
     * Advances one tick of dt seconds. The surface is sampled once per tick at every cell (body frame at the tick start)
     * and the water velocity at the bottom centre. memory is this boat's radiation memory (time step dt,
     * RADIATION_POINTS). water excludes this boat's own wake. ownWake, when given, samples that wake (an
     * UnsteadyWake driven by wakeForcing): the bottom pressure of wakeForcing pushing on its slope, int p grad(eta) dA,
     * is then the wave resistance in any motion (turning, accelerating, stopping); with ownWake null the quasi-steady
     * Havelock resistance of the present velocity is applied instead. The own wake's elevation is not added to the
     * hydrostatics (no squat).
     */
    public static Result step(State s, List<Passenger> passengers, Oars oars, Water water, Water ownWake, double density, double dt,
                              WaveRadiation.Memory memory) {
        return step(s, passengers, oars, water, ownWake, Air.CALM, density, dt, memory);
    }

    /** step with wind: air.wind10 at 10 m above the still water level, which is the surface's mean under the boat. */
    public static Result step(State s, List<Passenger> passengers, Oars oars, Water water, Water ownWake, Air air, double density, double dt,
                              WaveRadiation.Memory memory) {
        Mass mass = mass(passengers);
        double[] heights = new double[CELLS.length], slopeX = new double[CELLS.length], slopeZ = new double[CELLS.length], out = new double[3];
        double[] ownX = new double[CELLS.length], ownZ = new double[CELLS.length];
        double still = draft(mass.massKg(), density), peak = 1.5 * density * G * (Double.isNaN(still) ? DEPTH : still);
        Vec3d bottom0 = s.bottom();
        for (int i = 0; i < CELLS.length; i++) {
            Vec3d p = bottom0.add(offset(s.yaw(), 0, 0, CELLS[i][0], CELLS[i][1], 0));
            out[0] = Double.NaN; out[1] = 0; out[2] = 0;
            water.sample(p.x(), p.z(), out);
            heights[i] = out[0]; slopeX[i] = out[1]; slopeZ[i] = out[2];
            if (ownWake != null && CELLS[i][4] == 0) {
                out[0] = 0; out[1] = 0; out[2] = 0;
                ownWake.sample(p.x(), p.z(), out);
                ownX[i] = out[1]; ownZ[i] = out[2];
            }
        }
        Vec3d flow = water.velocity(bottom0.x(), bottom0.z());
        double h0 = mass.heightM(), a0 = mass.forwardM();
        double[] radiation = memory.verticalForces(density);
        double addedTotal = 0, addedPitch = 0, addedRoll = 0;
        double[] radii = new double[4];
        State x = s;
        double thrustSum = 0, resistanceSum = 0, waveSum = 0, wetSum = 0, draftSum = 0;
        boolean swamped = false;
        double h = dt / SUBSTEPS;
        Vec3d[] cellPoint = new Vec3d[CELLS.length];
        for (int step = 0; step < SUBSTEPS; step++) {
            Vec3d f = forward(x.yaw()), r = right(x.yaw());
            Vec3d cg = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), a0, 0, h0));
            Vec3d omega = r.scale(x.pitchRate()).add(f.scale(x.rollRate())).add(Vec3d.Y.scale(-x.yawRate()));
            Vec3d force = new Vec3d(0, -mass.massKg() * G, 0), torque = Vec3d.ZERO;
            double volume = 0, wetArea = 0, wetCells = 0, immersedSum = 0, immersedArea = 0;
            Vec3d ownForce = Vec3d.ZERO;
            double[] quarterWet = new double[4];
            for (int i = 0; i < CELLS.length; i++) {
                double[] cell = CELLS[i];
                Vec3d point = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), cell[0], cell[1], cell[4]));
                cellPoint[i] = point;
                // Columns over no water (NaN) are dry.
                double surface = heights[i];
                if (Double.isNaN(surface)) { continue; }
                double area = cell[2] * cell[3], depth = Math.clamp(surface - point.y(), 0, DEPTH - cell[4]);
                if (depth <= 0) { continue; }
                if (depth >= DEPTH - cell[4]) { swamped = true; }
                double v = area * depth;
                volume += v; wetCells += area;
                if (cell[4] == 0) { immersedSum += depth * area; immersedArea += area; }
                // Buoyancy at the column, the long-wave Froude-Krylov force on its volume (surface slope), heave drag.
                Vec3d rel = x.velocity().add(omega.cross(point.subtract(cg))).subtract(flow);
                double w = rel.y();
                Vec3d load = new Vec3d(-density * G * v * slopeX[i], density * G * v - 0.5 * density * HEAVE_DRAG * area * w * Math.abs(w), -density * G * v * slopeZ[i]);
                Vec3d at = point.add(Vec3d.Y.scale(depth / 2)).subtract(cg);
                force = force.add(load); torque = torque.add(at.cross(load));
                if (cell[4] == 0) {
                    int quarter = (cell[0] > 0 ? 0 : 2) + (cell[1] > 0 ? 0 : 1);
                    quarterWet[quarter] += area;
                }
                if (ownWake != null && cell[4] == 0) {
                    // The bottom pressure of wakeForcing on its own wake's slope.
                    double pressure = peak * (1 - Math.pow(2 * cell[0] / BOTTOM_LENGTH, 2)) * area;
                    Vec3d push = new Vec3d(-pressure * ownX[i], 0, -pressure * ownZ[i]);
                    force = force.add(push); torque = torque.add(point.subtract(cg).cross(push));
                    ownForce = ownForce.add(push);
                }
                // Wetted surface: the bottom cell plus the wall faces it borders.
                wetArea += cell[4] == 0 ? area : 0;
            }
            double draft = immersedArea > 0 ? immersedSum / immersedArea : 0;
            draftSum += draft;
            wetArea += 2 * (BOTTOM_LENGTH + BOTTOM_BEAM) * Math.min(draft, FLOOR);
            wetSum += wetCells / (LENGTH * BEAM);
            // Radiation (memory force constant over the tick) and its infinite-frequency added mass.
            addedTotal = 0; addedPitch = 0; addedRoll = 0;
            for (int q = 0; q < 4; q++) {
                double a = RADIATION_POINTS[q][0], c = RADIATION_POINTS[q][1];
                radii[q] = Math.sqrt(quarterWet[q] / Math.PI);
                double added = WaveRadiation.HEMISPHERE_ADDED_MASS_INFINITY * density * 2.0 / 3 * Math.PI * Math.pow(radii[q], 3);
                addedTotal += added; addedPitch += added * (a - a0) * (a - a0); addedRoll += added * c * c;
                Vec3d point = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), a, c, 0));
                Vec3d load = Vec3d.Y.scale(radiation[q]);
                force = force.add(load); torque = torque.add(point.subtract(cg).cross(load));
            }
            if (volume > 0) {
                // Resistance along the motion through the water.
                Vec3d rel = x.velocity().subtract(flow);
                Vec3d horizontal = new Vec3d(rel.x(), 0, rel.z());
                double speed = horizontal.length(), along = rel.dot(f);
                double friction = 0, wave = 0;
                if (speed > 1e-6) {
                    double reynolds = Math.abs(along) * BOTTOM_LENGTH * density / FluidState.FRESH_WATER_20C.dynamicViscosityPaS();
                    double diameter = Math.sqrt(8 * BOTTOM_BEAM * Math.max(draft, 1e-3) / Math.PI);
                    friction = 0.5 * density * wetArea * HullResistance.ittc1957(reynolds) * HullResistance.hoernerFormFactor(diameter / BOTTOM_LENGTH) * speed * speed;
                    double drift = Math.acos(Math.min(1, Math.abs(along) / speed));
                    wave = ownWake != null ? 0 : waveResistance(Math.max(draft, 1e-3), speed, drift, density);
                    Vec3d drag = horizontal.scale(-(friction + wave) / speed);
                    Vec3d at = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), 0, 0, draft / 2)).subtract(cg);
                    force = force.add(drag); torque = torque.add(at.cross(drag));
                }
                if (ownWake != null && speed > 1e-6) { wave = -ownForce.dot(horizontal) / speed; }
                resistanceSum += friction + wave; waveSum += wave;
                // Cross-flow drag on the immersed sides, strip by strip.
                double cd = HullManoeuvring.hoernerCrossFlow(BOTTOM_BEAM / (2 * Math.max(draft, 1e-3)));
                for (double[] strip : STRIPS) {
                    Vec3d point = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), strip[0], 0, draft / 2));
                    double lateral = x.velocity().add(omega.cross(point.subtract(cg))).subtract(flow).dot(r);
                    Vec3d drag = r.scale(-0.5 * density * cd * draft * strip[1] * lateral * Math.abs(lateral));
                    force = force.add(drag); torque = torque.add(point.subtract(cg).cross(drag));
                }
                // Oars, when the boat floats and someone rows.
                double thrust = 0;
                for (int side = -1; side <= 1; side += 2) {
                    double share = side < 0 ? oars.left() : oars.right();
                    if (share == 0) { continue; }
                    Vec3d blade = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), 0, side * OAR_REACH, draft));
                    double speedAlong = x.velocity().add(omega.cross(blade.subtract(cg))).subtract(flow).dot(f) * Math.signum(share);
                    double t = Math.signum(share) * oarThrust(Math.abs(share) * ROWER_POWER, speedAlong, density);
                    Vec3d push = f.scale(t);
                    force = force.add(push); torque = torque.add(blade.subtract(cg).cross(push));
                    thrust += t;
                }
                thrustSum += thrust;
            }
            double windSpeed = air.wind10().length();
            if (windSpeed > 0) {
                // Heights above the water from the bottom's depth below it (draft) at this substep.
                Vec3d direction = air.wind10().scale(1 / windSpeed);
                double freeboard = Math.max(0, DEPTH - draft);
                if (freeboard > 0) {
                    Vec3d point = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), 0, 0, draft + freeboard / 2));
                    Vec3d push = windLoad(air, direction, windSpeed, freeboard / 2, point, x, omega, cg, f, r, WINDAGE * BEAM * freeboard, WINDAGE * LENGTH * freeboard);
                    force = force.add(push); torque = torque.add(point.subtract(cg).cross(push));
                }
                for (Passenger q : passengers) {
                    double scale = Math.pow(q.massKg() / PERSON_MASS, 2.0 / 3), height = q.heightM() + 0.15;
                    Vec3d point = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), q.forwardM(), 0, height));
                    Vec3d push = windLoad(air, direction, windSpeed, height - draft, point, x, omega, cg, f, r, PERSON_DRAG_FRONT * scale, PERSON_DRAG_SIDE * scale);
                    force = force.add(push); torque = torque.add(point.subtract(cg).cross(push));
                }
            }
            // Integrate: added mass in heave, pitch and roll only (flat bottom).
            double mz = mass.massKg() + addedTotal;
            Vec3d acceleration = new Vec3d(force.x() / mass.massKg(), force.y() / mz, force.z() / mass.massKg());
            Vec3d velocity = x.velocity().add(acceleration.scale(h));
            double pitchRate = x.pitchRate() + torque.dot(r) / (mass.pitch() + addedPitch) * h;
            double rollRate = x.rollRate() + torque.dot(f) / (mass.roll() + addedRoll) * h;
            double yawRate = x.yawRate() - torque.dot(Vec3d.Y) / mass.yaw() * h;
            double yaw = x.yaw() + yawRate * h, pitch = x.pitch() + pitchRate * h, roll = x.roll() + rollRate * h;
            Vec3d cgNext = cg.add(velocity.scale(h));
            Vec3d bottomNext = cgNext.subtract(offset(yaw, pitch, roll, a0, 0, h0));
            x = new State(bottomNext, velocity, yaw, pitch, roll, yawRate, pitchRate, rollRate);
        }
        // Record the radiation points' vertical velocities for the memory.
        double[] w = new double[4];
        Vec3d f = forward(x.yaw()), r = right(x.yaw());
        Vec3d cg = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), a0, 0, h0));
        Vec3d omega = r.scale(x.pitchRate()).add(f.scale(x.rollRate())).add(Vec3d.Y.scale(-x.yawRate()));
        for (int q = 0; q < 4; q++) {
            Vec3d point = x.bottom().add(offset(x.yaw(), x.pitch(), x.roll(), RADIATION_POINTS[q][0], RADIATION_POINTS[q][1], 0));
            w[q] = x.velocity().add(omega.cross(point.subtract(cg))).y();
        }
        memory.record(w, radii);
        return new Result(x, draftSum / SUBSTEPS, thrustSum / SUBSTEPS, resistanceSum / SUBSTEPS, waveSum / SUBSTEPS, wetSum / SUBSTEPS, swamped);
    }

    /** Horizontal wind force on drag areas along the boat (end) and across it (side) at a point and height above the water. */
    private static Vec3d windLoad(Air air, Vec3d direction, double u10, double height, Vec3d point, State x, Vec3d omega, Vec3d cg,
                                  Vec3d f, Vec3d r, double endArea, double sideArea) {
        Vec3d moving = x.velocity().add(omega.cross(point.subtract(cg)));
        Vec3d relative = direction.scale(SeaWind.speedAt(u10, height)).subtract(new Vec3d(moving.x(), 0, moving.z()));
        double along = relative.dot(f), across = relative.dot(r);
        return f.scale(0.5 * air.density() * endArea * along * Math.abs(along)).add(r.scale(0.5 * air.density() * sideArea * across * Math.abs(across)));
    }

    /**
     * Stroke-averaged thrust of one oar (N) for the drive power the rower puts into it (W) at the boat's speed through
     * the water along the stroke direction (m/s; held at zero or more: an oar pulled against the boat's motion is
     * treated as starting from rest). Solves (rho/2) C_D A v^2 (U + v) = P / DRIVE_FRACTION by Newton's method, then
     * holds the slip v to BLADE_SPEED - U (the rower then delivers less than P); none once U reaches BLADE_SPEED.
     */
    public static double oarThrust(double power, double speed, double density) {
        if (!(power > 0)) { return 0; }
        double u = Math.max(0, speed), drive = power / DRIVE_FRACTION, k = 0.5 * density * BLADE_DRAG * BLADE_AREA;
        double v = Math.cbrt(drive / k);
        for (int i = 0; i < 40; i++) {
            double residual = k * v * v * (u + v) - drive, derivative = k * (2 * v * u + 3 * v * v);
            double next = v - residual / derivative;
            if (Math.abs(next - v) < 1e-10) { v = next; break; }
            v = Math.max(next, 1e-9);
        }
        v = Math.min(v, BLADE_SPEED - u);
        return v > 0 ? DRIVE_FRACTION * k * v * v : 0;
    }

    private static final Map<Long, Double> WAVE_RESISTANCE = new ConcurrentHashMap<>();

    /**
     * Havelock wave resistance (N) of the bottom's pressure (wakeForcing) moving at speed U at drift angle beta to its
     * length. Cached on 5 mm of draft, 0.05 m/s and
     * 15 degrees and interpolated linearly in speed and angle.
     */
    public static double waveResistance(double draft, double speed, double drift, double density) {
        if (!(speed > 0.05) || !(draft > 0)) { return 0; }
        double u = speed / 0.05, b = Math.clamp(drift, 0, Math.PI / 2) / Math.toRadians(15);
        int iu = (int) u, ib = Math.min(5, (int) b);
        double fu = u - iu, fb = b - ib;
        long t = Math.round(draft / 0.005);
        double r00 = cached(t, iu, ib, density), r10 = cached(t, iu + 1, ib, density), r01 = cached(t, iu, ib + 1, density), r11 = cached(t, iu + 1, ib + 1, density);
        return (r00 * (1 - fb) + r01 * fb) * (1 - fu) + (r10 * (1 - fb) + r11 * fb) * fu;
    }

    private static double cached(long draftKey, int speedKey, int driftKey, double density) {
        if (speedKey <= 1) { return 0; }
        long key = (draftKey << 32) ^ ((long) speedKey << 8) ^ driftKey;
        return WAVE_RESISTANCE.computeIfAbsent(key, k -> havelock(draftKey * 0.005, speedKey * 0.05, Math.toRadians(15 * driftKey), 1000)) * density / 1000;
    }

    /** The uncached Havelock integral behind waveResistance, for wakeForcing. */
    public static double havelock(double draft, double speed, double drift, double density) {
        double c = Math.cos(drift), s = Math.sin(drift), scale = 1 / (4 * Math.PI * Math.PI);
        UnsteadyWake.Forcing forcing = wakeForcing(draft, density);
        // Wavenumber in the motion frame turned into the body frame; KelvinWake's transform carries 1 / 4 pi^2.
        KelvinWake.PressureTransform moving = (alpha, beta, out) -> {
            forcing.transform(alpha * c - beta * s, alpha * s + beta * c, speed, out);
            out[0] *= scale; out[1] *= scale;
        };
        double reach = Math.hypot(BOTTOM_LENGTH, BOTTOM_BEAM) / 2;
        return KelvinWake.pressure(moving, -reach, reach, reach, density, speed, G, 0.1, 2 * LENGTH).resistance(density);
    }
}
