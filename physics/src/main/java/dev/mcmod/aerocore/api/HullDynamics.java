package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * One time step of a rigid body carried by a slender hull (HullModel), strip by strip:
 * <ul>
 * <li>hydrostatics from each section clipped by the local free surface (exact polygon area at any heel and trim);
 *     in waves the part between the still and instantaneous waterlines is scaled by exp(-k d), d the mean bottom
 *     depth A/B, as the linear dynamic pressure decays with depth;</li>
 * <li>strip added mass (surge share of m11, Lewis or ellipse sway m22 and heave m33) assembled into a 6 x 6
 *     body-frame matrix A, applied implicitly in the velocity update, with the ideal-fluid Kirchhoff terms
 *     A (omega x v) - C_A(nu) nu explicit, so the Munk moment and added-mass Coriolis coupling follow from A;</li>
 * <li>wave excitation per strip: Froude-Krylov rho A a (horizontal; also vertical when fully submerged) and Morison
 *     diffraction with the strip added mass;</li>
 * <li>calm-water resistance (HullModel) along the hull axis, the lift element at the trailing station, and Hoerner
 *     cross-flow drag per strip (water and air), all relative to the local flow.</li>
 * </ul>
 * World y must be up (gravity along -y). Heave/pitch wave radiation is not included here; supply it as external loads
 * (for example WaveRadiation.Memory with radiationPoints()). Do not add weight, buoyancy or hull drag to externalLoads.
 */
public final class HullDynamics {
    /**
     * Windage of the dry part: lateral 0.90, Blendermann's (1994, J. Wind Eng. Ind. Aerodyn. 51:339) transverse drag
     * coefficient for a speed boat (CD_t, about +-25 %); vertically 1.9, the flat plate normal to the flow at
     * Re > 1e4 (DNV-RP-C205 Table E-1 no. 8). Longitudinal windage is not modelled.
     */
    public static final double WINDAGE_LATERAL = 0.90, WINDAGE_VERTICAL = 1.9;
    private HullDynamics() {}

    /**
     * Water at one strip: instantaneous free-surface height (NaN or -infinity when there is no water column) and its
     * slope (d eta/dx, 0, d eta/dz); the still level and an effective wavenumber for the pressure decay (NaN and 0 in
     * calm water); local water velocity and acceleration; water and air properties.
     */
    public record Water(double surfaceY, Vec3d surfaceSlope, double stillWaterY, double decayWavenumber,
                        Vec3d velocity, Vec3d acceleration, FluidState water, FluidState air) {
        public Water {
            Objects.requireNonNull(surfaceSlope); Objects.requireNonNull(velocity); Objects.requireNonNull(acceleration);
            Objects.requireNonNull(water); Objects.requireNonNull(air);
            Checks.nonnegative(decayWavenumber, "Decay wavenumber");
        }
        public static Water calm(double surfaceY, FluidState water, FluidState air) {
            return new Water(surfaceY, Vec3d.ZERO, Double.NaN, 0, water.velocity(), Vec3d.ZERO, water, air);
        }
        public boolean hasWater() { return Double.isFinite(surfaceY); }
    }
    @FunctionalInterface public interface Environment { Water sample(Vec3d worldPoint); }

    /**
     * Wrenches use the step's starting COM and sum to the momentum change: hydrostatics (weight and buoyancy),
     * drag (resistance and cross-flow), lift (lift element), inertia (added-mass reaction, wave excitation and
     * Kirchhoff terms), contact, external. kirchhoff is the ideal-fluid part of inertia alone (Munk moment and
     * Coriolis coupling at the start velocity). waterlineBreadth is each strip's waterline breadth (m) at the start.
     */
    public record Result(RigidBodyState state, Wrench hydrostatics, Wrench drag, Wrench lift, Wrench inertia, Wrench contact,
                         Wrench external, Wrench kirchhoff, List<AppliedForce> forces, double wetVolumeM3, double surgeSpeedMps,
                         double resistanceN, double dragPowerW, double contactFrictionW, double[] waterlineBreadth) {}

    /**
     * Fluid loads of one state, before the velocity update (world axes, torques about the COM): hydro (buoyancy, and
     * the body's weight when asked), drag, lift, wave excitation, kirchhoff (ideal-fluid terms including the frame term),
     * steady (-C_A(nu) nu alone), the strip added mass in body axes about the COM, display forces, wet volume (m^3),
     * surge speed (m/s), calm-water resistance (N), dissipated power (W) and each strip's waterline breadth (m).
     */
    public record Loads(Wrench hydro, Wrench drag, Wrench lift, Wrench excitation, Wrench kirchhoff, Wrench steady, double[][] addedBody,
                        List<AppliedForce> forces, double wetVolume, double surge, double resistance, double dragPower, double[] breadth) {}

    /**
     * The hull's loads at one state (as prepare() integrates them), for composite bodies (VehicleDynamics): includeWeight
     * false leaves out p's weight, so p may describe a larger body the hull belongs to (its COM and inertia set the
     * moment arms and the angular velocity).
     */
    public static Loads loads(RigidBodyState s, RigidBodyProperties p, HullModel model, Environment environment, Vec3d gravity, boolean includeWeight) {
        return assemble(s, p, model, environment, gravity, includeWeight);
    }

    public static Result step(RigidBodyState s, RigidBodyProperties p, HullModel model, Environment environment, Vec3d gravity,
                              List<AppliedForce> externalLoads, FloatingBodyDynamics.Collision collision, double dt) {
        Prepared prepared = prepare(s, p, model, environment, gravity, externalLoads, dt);
        if (collision == null) { return finish(prepared, prepared.damped(), Wrench.ZERO, List.of(), 0, null); }
        List<ContactSolver.Contact> contacts = ContactSolver.generate(prepared.damped(), p, collision.shape(), collision.obstacles(), dt);
        ContactSolver.Result solved = ContactSolver.solve(prepared.damped(), p, contacts, collision.material(), dt);
        return finish(prepared, solved.state(), solved.wrench(), solved.forces(), solved.frictionDissipationW(), collision);
    }

    /** First half of step(): loads and the implicit velocity update; damped is the pre-contact state. */
    public record Prepared(RigidBodyProperties properties, RigidBodyState damped, Wrench hydrostatics, Wrench drag, Wrench lift, Wrench inertia,
                           Wrench external, Wrench kirchhoff, List<AppliedForce> forces, double wetVolumeM3, double surgeSpeedMps, double resistanceN,
                           double dragPowerW, double[] waterlineBreadth, double dt) {}

    /** Second half of step(): rigid drift from the post-contact state and projection against static obstacles. */
    public static Result finish(Prepared p, RigidBodyState afterContact, Wrench contact, List<AppliedForce> contactForces, double contactFrictionW,
                                FloatingBodyDynamics.Collision collision) {
        List<AppliedForce> forces = new ArrayList<>(p.forces());
        forces.addAll(contactForces);
        RigidBodyState next = RigidBodyDynamics.drift(afterContact, p.properties(), p.dt());
        if (collision != null) { next = ContactSolver.separate(next, p.properties(), collision.shape(), collision.obstacles()); }
        return new Result(next, p.hydrostatics(), p.drag(), p.lift(), p.inertia(), contact, p.external(), p.kirchhoff(), List.copyOf(forces),
                p.wetVolumeM3(), p.surgeSpeedMps(), p.resistanceN(), p.dragPowerW(), contactFrictionW, p.waterlineBreadth());
    }

    public static Prepared prepare(RigidBodyState s, RigidBodyProperties p, HullModel model, Environment environment, Vec3d gravity,
                                   List<AppliedForce> externalLoads, double dt) {
        Checks.positive(dt, "Time step");
        Loads loads = assemble(s, p, model, environment, gravity, true);
        List<AppliedForce> forces = new ArrayList<>(loads.forces());
        Wrench external = Wrench.ZERO;
        for (AppliedForce load : externalLoads) { external = external.add(load.wrench()); forces.add(load); }
        Wrench load = loads.hydro().add(loads.drag()).add(loads.lift()).add(loads.excitation()).add(loads.kirchhoff()).add(external);
        double[][] addedWorld = rotate(loads.addedBody(), s.orientation());
        RigidBodyState damped = RigidBodyDynamics.implicitVelocity(s, p, load, addedWorld, dt);
        // Added-mass reaction = -A (nu1 - nu0) / dt; together with excitation and Kirchhoff terms it is "inertia".
        Vec3d omega = s.angularVelocity(p), dv = damped.velocity().subtract(s.velocity()), dw = damped.angularVelocity(p).subtract(omega);
        double[] change = {dv.x(), dv.y(), dv.z(), dw.x(), dw.y(), dw.z()}, reaction = new double[6];
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { reaction[i] -= addedWorld[i][j] * change[j] / dt; }
        Wrench inertia = loads.excitation().add(loads.kirchhoff())
                .add(new Wrench(new Vec3d(reaction[0], reaction[1], reaction[2]), new Vec3d(reaction[3], reaction[4], reaction[5])));
        if (inertia.force().lengthSquared() > 0 || inertia.torque().lengthSquared() > 0) {
            forces.add(new AppliedForce(AppliedForce.Kind.INERTIA, Vec3d.ZERO, inertia.force(), inertia.torque()));
        }
        return new Prepared(p, damped, loads.hydro(), loads.drag(), loads.lift(), inertia, external, loads.kirchhoff(), List.copyOf(forces),
                loads.wetVolume(), loads.surge(), loads.resistance(), loads.dragPower(), loads.breadth(), dt);
    }

    /**
     * Steady hydrodynamic load on the hull held at this state with constant body-frame velocities, as in a captive
     * (towing-tank or rotating-arm) test: hydrostatics, drag, lift, wave excitation and the ideal-fluid -C_A(nu) nu,
     * without any added-mass reaction. World axes, torque about the COM.
     */
    public static Wrench captiveLoad(RigidBodyState s, RigidBodyProperties p, HullModel model, Environment environment, Vec3d gravity) {
        Loads loads = assemble(s, p, model, environment, gravity, true);
        return loads.hydro().add(loads.drag()).add(loads.lift()).add(loads.excitation()).add(loads.steady());
    }

    private static Loads assemble(RigidBodyState s, RigidBodyProperties p, HullModel model, Environment environment, Vec3d gravity, boolean includeWeight) {
        SlenderHull hull = model.hull();
        int n = hull.stations();
        Orientation q = s.orientation();
        Vec3d ex = q.rotate(Vec3d.X), ey = q.rotate(Vec3d.Y), ez = q.rotate(Vec3d.Z);
        Vec3d omega = s.angularVelocity(p);
        double g = gravity.length();
        Wrench hydro = includeWeight ? new Wrench(gravity.scale(p.massKg()), Vec3d.ZERO) : Wrench.ZERO, drag = Wrench.ZERO, lift = Wrench.ZERO, excitation = Wrench.ZERO;
        List<AppliedForce> forces = new ArrayList<>();
        if (includeWeight) { forces.add(new AppliedForce(AppliedForce.Kind.WEIGHT, Vec3d.ZERO, gravity.scale(p.massKg()))); }
        HullSection.Wet[] wets = new HullSection.Wet[n];
        Water[] waters = new Water[n];
        Vec3d[] offsets = new Vec3d[n];
        double[] breadth = new double[n];
        double wetVolume = 0, wetSurface = 0, density = 0;
        Vec3d flow = Vec3d.ZERO, wetMoment = Vec3d.ZERO;
        boolean piercing = false;
        // Pass 1: wet geometry of every strip against its local surface.
        for (int i = 0; i < n; i++) {
            HullSection section = hull.sections().get(i);
            HullSection.Wet full = section.clip(1, 0, 1e9);
            Vec3d origin = s.worldPoint(p, new Vec3d(section.xM(), 0, 0));
            Vec3d reference = origin.add(ey.scale(full.centroidY())).add(ez.scale(full.centroidZ()));
            Water water = environment.sample(reference);
            waters[i] = water;
            HullSection.Wet wet = water.hasWater() ? clip(section, origin, reference, ey, ez, water.surfaceY(), water.surfaceSlope()) : HullSection.Wet.DRY;
            wets[i] = wet;
            breadth[i] = wet.waterlineBreadthM();
            Vec3d centroid = wet.areaM2() > 0 ? origin.add(ey.scale(wet.centroidY())).add(ez.scale(wet.centroidZ())) : reference;
            offsets[i] = centroid.subtract(s.position());
            double volume = wet.areaM2() * hull.stripLength(i);
            wetVolume += volume;
            wetSurface += wet.girthM() * hull.stripLength(i);
            flow = flow.add(water.velocity().scale(volume));
            wetMoment = wetMoment.add(offsets[i].scale(volume));
            density += water.water().densityKgPerM3() * volume;
            piercing |= wet.piercing();
        }
        Vec3d meanFlow = wetVolume > 0 ? flow.scale(1 / wetVolume) : Vec3d.ZERO;
        double rho = wetVolume > 0 ? density / wetVolume : 0;
        double[][] addedBody = new double[6][6];
        double dragPower = 0;
        Vec3d up = gravity.lengthSquared() > 0 ? gravity.scale(-1 / g) : Vec3d.Y;
        // Pass 2: per-strip buoyancy, added mass, wave excitation and cross-flow drag.
        for (int i = 0; i < n; i++) {
            HullSection section = hull.sections().get(i);
            HullSection.Wet wet = wets[i];
            Water water = waters[i];
            double dx = hull.stripLength(i), rhoWater = water.water().densityKgPerM3(), rhoAir = water.air().densityKgPerM3();
            Vec3d r = offsets[i], rBody = q.inverse().rotate(r);
            // Air buoyancy on the dry part, water pressure on the wet part.
            double area = wet.areaM2(), buoyant = area;
            if (wet.piercing() && Double.isFinite(water.stillWaterY()) && water.decayWavenumber() > 0) {
                Vec3d origin = s.worldPoint(p, new Vec3d(section.xM(), 0, 0));
                HullSection.Wet still = clipLevel(section, origin, ey, ez, water.stillWaterY());
                double depth = still.waterlineBreadthM() > 0 ? still.areaM2() / still.waterlineBreadthM() : wet.draftM();
                buoyant = still.areaM2() + (area - still.areaM2()) * Math.exp(-water.decayWavenumber() * depth);
            }
            Vec3d buoyancy = up.scale(g * dx * (rhoWater * buoyant + rhoAir * (section.area() - area)));
            if (buoyancy.lengthSquared() > 0) {
                hydro = hydro.add(Wrench.atOffset(buoyancy, r));
                forces.add(new AppliedForce(AppliedForce.Kind.BUOYANCY, r, buoyancy));
            }
            Vec3d pointVelocity = s.velocity().add(omega.cross(r));
            if (area > 0) {
                double[] sectional = HullModel.sectionAddedMass(section, wet, rhoWater);
                double m11 = model.surgeAddedMass() * area * dx / wetVolume, m22 = sectional[0] * dx, m33 = sectional[1] * dx;
                addPointMass(addedBody, rBody, m11, m33, m22);
                // Froude-Krylov (horizontal; vertical too when fully submerged, as the surface pressure integral then
                // carries no wave term) and Morison diffraction with the strip added mass.
                Vec3d a = water.acceleration();
                Vec3d froudeKrylov = (wet.piercing() ? a.subtract(up.scale(a.dot(up))) : a).scale(rhoWater * area * dx);
                Vec3d diffraction = ex.scale(m11 * a.dot(ex)).add(ey.scale(m33 * a.dot(ey))).add(ez.scale(m22 * a.dot(ez)));
                excitation = excitation.add(Wrench.atOffset(froudeKrylov.add(diffraction), r));
                // Cross-flow drag on the wet part.
                Vec3d relative = pointVelocity.subtract(water.velocity());
                double lateral = wet.piercing() ? wet.draftM() : section.maxY() - section.minY();
                double vertical = wet.piercing() ? wet.waterlineBreadthM() : 2 * section.maxHalfBreadth();
                double cLateral, cVertical;
                if (model.type() == HullModel.Type.SURFACE) {
                    double ratio = lateral > 0 ? Math.max(vertical, 2 * section.halfBreadthAt(section.minY() + 0.5 * lateral)) / (2 * lateral) : 1;
                    cLateral = HullManoeuvring.hoernerCrossFlow(ratio);
                    cVertical = model.heaveCrossFlow();
                } else {
                    double cross = Math.hypot(relative.dot(ey), relative.dot(ez)) * model.diameter() / model.kinematicViscosity();
                    double eta = HullManoeuvring.jorgensenEta(hull.length() / model.diameter());
                    cLateral = cVertical = eta * HullManoeuvring.cylinderCrossFlow(cross);
                }
                Vec3d f = crossFlow(relative, ey, ez, 0.5 * rhoWater * dx, cLateral * lateral, cVertical * vertical);
                if (f.lengthSquared() > 0) {
                    drag = drag.add(Wrench.atOffset(f, r));
                    forces.add(new AppliedForce(AppliedForce.Kind.DRAG, r, f));
                    dragPower -= f.dot(relative);
                }
            }
            // Windage of the dry part, at the section's centre.
            double dryHeight = Math.max(0, section.maxY() - section.minY() - (wet.piercing() ? wet.draftM() : area > 0 ? section.maxY() - section.minY() : 0));
            if (dryHeight > 0 && rhoAir > 0) {
                Vec3d relative = pointVelocity.subtract(water.air().velocity());
                Vec3d f = crossFlow(relative, ey, ez, 0.5 * rhoAir * dx, WINDAGE_LATERAL * dryHeight, area > 0 ? 0 : WINDAGE_VERTICAL * 2 * section.maxHalfBreadth());
                if (f.lengthSquared() > 0) {
                    drag = drag.add(Wrench.atOffset(f, r));
                    forces.add(new AppliedForce(AppliedForce.Kind.DRAG, r, f));
                    dragPower -= f.dot(relative);
                }
            }
        }
        double surge = 0, resistance = 0;
        Wrench kirchhoff = Wrench.ZERO, steady = Wrench.ZERO;
        if (wetVolume > 0) {
            Vec3d relative = s.velocity().subtract(meanFlow), centre = wetMoment.scale(1 / wetVolume);
            surge = relative.dot(ex);
            // Calm-water resistance along the hull axis; wave resistance only while the hull pierces the surface.
            double speed = Math.abs(surge), coefficient = model.formFactor() * HullResistance.ittc1957(speed * model.design().lengthM() / model.kinematicViscosity())
                    + (piercing ? model.waveResistanceCoefficient(speed / Math.sqrt(Atmosphere.GRAVITY_MPS2 * model.design().lengthM())) : 0);
            resistance = 0.5 * rho * surge * surge * wetSurface * coefficient;
            Vec3d resistanceForce = ex.scale(-Math.signum(surge) * resistance);
            if (resistance > 0) {
                drag = drag.add(Wrench.atOffset(resistanceForce, centre));
                forces.add(new AppliedForce(AppliedForce.Kind.DRAG, centre, resistanceForce));
                dragPower += resistance * speed;
            }
            // Lift element at the trailing station, faded out as the hull leaves the water.
            double immersion = Math.min(1, wetVolume / model.designVolume()), station = model.liftStation(surge >= 0);
            Vec3d rT = s.worldOffset(p, new Vec3d(station, q.inverse().rotate(centre).y() + p.centerOfMassLocal().y(), 0));
            Vec3d atStation = relative.add(omega.cross(rT));
            double gain = speed * model.liftMass() * immersion;
            Vec3d liftForce = ez.scale(-gain * atStation.dot(ez));
            if (model.type() == HullModel.Type.SUBMERGED) { liftForce = liftForce.add(ey.scale(-gain * atStation.dot(ey))); }
            if (liftForce.lengthSquared() > 0) {
                lift = Wrench.atOffset(liftForce, rT);
                forces.add(new AppliedForce(AppliedForce.Kind.LIFT, rT, liftForce));
                dragPower -= liftForce.dot(atStation);
            }
            kirchhoff = kirchhoff(addedBody, q, relative, omega);
            steady = coriolis(addedBody, q, relative, omega);
        }
        return new Loads(hydro, drag, lift, excitation, kirchhoff, steady, addedBody, forces, wetVolume, surge, resistance, dragPower, breadth);
    }

    /**
     * Local x of heave-radiation points: the hull split into equal-length segments; pair them with segmentRadii for
     * WaveRadiation.Memory (point absorbers of the segments' waterplane areas, a heave-only approximation).
     */
    public static List<Vec3d> radiationPoints(SlenderHull hull, int segments) {
        if (segments < 1) { throw new IllegalArgumentException("Need at least one segment"); }
        List<Vec3d> points = new ArrayList<>();
        for (int k = 0; k < segments; k++) { points.add(new Vec3d(hull.xAft() + hull.length() * (k + 0.5) / segments, 0, 0)); }
        return points;
    }
    /** Equivalent waterplane radius sqrt(A_wp / pi) of each segment from a step's strip waterline breadths. */
    public static double[] segmentRadii(SlenderHull hull, int segments, double[] waterlineBreadth) {
        double[] area = new double[segments];
        for (int i = 0; i < hull.stations(); i++) {
            int k = Math.min(segments - 1, (int) ((hull.sections().get(i).xM() - hull.xAft()) / hull.length() * segments));
            area[k] += waterlineBreadth[i] * hull.stripLength(i);
        }
        for (int k = 0; k < segments; k++) { area[k] = Math.sqrt(area[k] / Math.PI); }
        return area;
    }

    /** Wet part of a section below a locally plane surface through (reference, surfaceY) with the given slope. */
    static HullSection.Wet clip(HullSection section, Vec3d origin, Vec3d reference, Vec3d ey, Vec3d ez, double surfaceY, Vec3d slope) {
        // Height above the surface of section point (y, z): origin + y ey + z ez, minus surfaceY + slope . (horizontal offset).
        double a = ey.y() - slope.x() * ey.x() - slope.z() * ey.z();
        double b = ez.y() - slope.x() * ez.x() - slope.z() * ez.z();
        double d = surfaceY - origin.y() + slope.x() * (origin.x() - reference.x()) + slope.z() * (origin.z() - reference.z());
        if (Math.hypot(a, b) < 1e-9) { return d > 0 ? section.clip(1, 0, 1e9) : HullSection.Wet.DRY; }
        return section.clip(a, b, d);
    }
    private static HullSection.Wet clipLevel(HullSection section, Vec3d origin, Vec3d ey, Vec3d ez, double level) {
        return clip(section, origin, origin, ey, ez, level, Vec3d.ZERO);
    }
    /** -k |u_n| u_n on the lateral (ez) and vertical (ey) components of the relative velocity. */
    static Vec3d crossFlow(Vec3d relative, Vec3d ey, Vec3d ez, double half, double lateralArea, double verticalArea) {
        double w = relative.dot(ez), h = relative.dot(ey);
        return ez.scale(-half * lateralArea * Math.abs(w) * w).add(ey.scale(-half * verticalArea * Math.abs(h) * h));
    }
    /** a += J^T diag(mx, my, mz) J for a point at body offset r, J nu = v + omega x r (body frame). */
    static void addPointMass(double[][] a, Vec3d r, double mx, double my, double mz) {
        double[][] j = {{1, 0, 0, 0, r.z(), -r.y()}, {0, 1, 0, -r.z(), 0, r.x()}, {0, 0, 1, r.y(), -r.x(), 0}};
        double[] m = {mx, my, mz};
        for (int row = 0; row < 6; row++) for (int col = 0; col < 6; col++) for (int k = 0; k < 3; k++) { a[row][col] += m[k] * j[k][row] * j[k][col]; }
    }
    /** World 6 x 6 matrix T A T^T, T = diag(R, R). */
    public static double[][] rotate(double[][] body, Orientation q) {
        double[][] r = new double[3][3];
        Vec3d[] axes = {q.rotate(Vec3d.X), q.rotate(Vec3d.Y), q.rotate(Vec3d.Z)};
        for (int c = 0; c < 3; c++) { r[0][c] = axes[c].x(); r[1][c] = axes[c].y(); r[2][c] = axes[c].z(); }
        double[][] world = new double[6][6];
        for (int bi = 0; bi < 2; bi++) for (int bj = 0; bj < 2; bj++) {
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
                double sum = 0;
                for (int k = 0; k < 3; k++) for (int l = 0; l < 3; l++) { sum += r[i][k] * body[3 * bi + k][3 * bj + l] * r[j][l]; }
                world[3 * bi + i][3 * bj + j] = sum;
            }
        }
        // Remove rounding asymmetry.
        for (int i = 0; i < 6; i++) for (int j = i + 1; j < 6; j++) { double m = 0.5 * (world[i][j] + world[j][i]); world[i][j] = m; world[j][i] = m; }
        return world;
    }
    /**
     * Ideal-fluid terms besides -A nu_dot (Fossen 2011, Handbook of Marine Craft Hydrodynamics, 6.3): in body axes
     * A (omega x v, 0) - C_A(nu) nu with (P, H) = A nu and C_A(nu) nu = (omega x P, omega x H + v x P); the first term
     * converts the world-frame acceleration used by the implicit update into the body-frame derivative.
     * Returned in world axes, as a force and a torque about the COM.
     */
    /** -C_A(nu) nu alone, in world axes: the ideal-fluid load on a body held at constant body-frame velocity. */
    static Wrench coriolis(double[][] body, Orientation q, Vec3d relativeVelocity, Vec3d omega) {
        Orientation inverse = q.inverse();
        Vec3d v = inverse.rotate(relativeVelocity), w = inverse.rotate(omega);
        double[] nu = {v.x(), v.y(), v.z(), w.x(), w.y(), w.z()}, pa = new double[6];
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { pa[i] += body[i][j] * nu[j]; }
        Vec3d linear = new Vec3d(pa[0], pa[1], pa[2]), angular = new Vec3d(pa[3], pa[4], pa[5]);
        return new Wrench(q.rotate(w.cross(linear).scale(-1)), q.rotate(w.cross(angular).add(v.cross(linear)).scale(-1)));
    }
    static Wrench kirchhoff(double[][] body, Orientation q, Vec3d relativeVelocity, Vec3d omega) {
        Orientation inverse = q.inverse();
        Vec3d v = inverse.rotate(relativeVelocity), w = inverse.rotate(omega), wv = w.cross(v);
        double[] nu = {v.x(), v.y(), v.z(), w.x(), w.y(), w.z()}, frame = {wv.x(), wv.y(), wv.z(), 0, 0, 0}, pa = new double[6], fa = new double[6];
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { pa[i] += body[i][j] * nu[j]; fa[i] += body[i][j] * frame[j]; }
        Vec3d linear = new Vec3d(pa[0], pa[1], pa[2]), angular = new Vec3d(pa[3], pa[4], pa[5]);
        Vec3d force = new Vec3d(fa[0], fa[1], fa[2]).subtract(w.cross(linear));
        Vec3d torque = new Vec3d(fa[3], fa[4], fa[5]).subtract(w.cross(angular)).subtract(v.cross(linear));
        return new Wrench(q.rotate(force), q.rotate(torque));
    }
}
