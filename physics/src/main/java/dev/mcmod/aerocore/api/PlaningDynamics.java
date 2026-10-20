package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;

/**
 * One time step of a rigid body carried by a hard-chine planing hull (PlaningHull), with Zarnick's (1978, DTNSRDC-78/032)
 * two-dimensional, time-domain ("2D+t") strip theory for the load normal to the keel:
 * <pre>  f = D/Dt(m_a V) + C_Dc rho b V|V|,   D/Dt(m_a V) = m_a dV/dt + (dm_a/dt) V - U d(m_a V)/d xi</pre>
 * per unit length, with V the water velocity into the bottom normal to the keel, U along it, m_a = (pi/2) rho b^2
 * (k_a = 1), Wagner's wetted half-beam b = (pi/2) h cot(beta) up to the chine and the chine half-beam after it
 * (dm_a/dt = 0 once the chine is wet) and C_Dc = cos(beta). The first term is added mass in the implicit velocity
 * update, with the turning-frame part of dV/dt (centripetal acceleration, rotation of the bottom normal, which
 * Zarnick's V-dot contains through the pitch rate) added explicitly; the convective term -U d(m_a V)/d xi is integrated strip by strip from the transom to the stem, so the
 * transom momentum m_a U V leaves the hull there as in Zarnick. Where a section shrinks as the water passes aft (it
 * leaves the water: a bow-down hull's run, the bottom just behind a step) its dm_a gives no force, as Zarnick's
 * dm_a/dt = 0 in water exit, rather than suction (since 0.23: with suction a body trimmed bow down dived).
 * <p>Hydrostatics: the full Archimedes load of each clipped section. With the transom dry its bottom pressure is no
 * longer balanced by a transom face, so it acts normal to the keel like Zarnick's sectional loads and carries the
 * pressure drag (Delta tan trim) of a ventilated bottom. Mercier and Savitsky (1973) find the transom fully ventilated
 * at C_vT = U / sqrt(g B_T) = 0.5 and planing developed by about 1.5; the load direction and dry hull sides (friction
 * area) are blended in over that interval (smoothstep, a modelling choice). Near the dry transom the pressure falls
 * to atmospheric, so the hydrostatic load is scaled by Garme's (2005) tanh(2.5 x_T / (0.34 B C_V)) with his published
 * coefficient, faded in with the ventilation (TRANSOM_REDUCTION). Applied to the hydrodynamic strip loads as well (as
 * Garme did) it made the hull unstable here, and Zarnick's halving of the buoyancy overshot the trims, so only the
 * hydrostatics are reduced.
 * <p>Deadrise (LIFT_RECOVERY, 2026-10-12, found with hdfv 0.5's body bench): at a given trim and wetted length the 2D+t normal load hardly depends on the
 * deadrise -- once its chines are wet a V section carries the flat plate's momentum (pi/2) rho (B/2)^2 V -- while
 * Savitsky's (1964) prismatic data lose a quarter to a half of the flat surface's lift at 20 to 30 deg
 * (C_Lbeta = C_L0 - 0.0065 beta C_L0^0.6, most at small lift coefficients). The 2D+t also lacks the growth of lift with
 * the chine-wetted length (Savitsky's lambda^0.5). The two errors cancel for Fridsma's long wetted lengths, which is why
 * the model agreed with his tank, and add up when the chines wet near the transom: a light hull, an aft centre of
 * gravity or high speed ran 1 deg flatter than Savitsky over a longer wetted bottom, with more friction and whisker spray
 * (up to 1.7 times his resistance by C_V 4, a 6.3 m runabout at C_Delta 0.15). Each planing surface's normal loads
 * (hydrostatic, hydrodynamic, the added mass, the normal diffraction) are therefore scaled by
 * <pre>  k = 1 - sin(beta) + 0.13 L_C / B   (0.3 to 1)</pre>
 * faded in with its ventilation: 1 - sin(beta) is Shuford's (1958, NACA Report 1355) deadrise factor on a V-bottom's
 * linear lift, and the recovery of 0.13 per beam of chine-wetted length L_C ahead of the transom (or step) was fitted
 * to Savitsky's lift at a fixed trim and wetted length (C_V 4, 20 and 30 deg; at 10 deg the chines wet far enough
 * forward that k is about 1). Scaling the whole sectional load keeps the 2D+t's distribution and its centre of pressure,
 * which is close to Savitsky's. Tried and rejected: Garme's reduction or a longer near-transom relief on the dynamic
 * lift (the momentum is built where the wetted beam grows, mostly forward of Garme's length, and any relief there
 * removed the LCG 70 % porpoising of Fridsma's model), Payne's (1 - beta / 2 pi)^2 added mass and a deadrise pile-up
 * (11 % of the lift, 0.1 deg of trim), Savitsky's deadrise ratio on its own (a degree too much trim at Fridsma's
 * lengths, where the 2D+t's missing length effect compensates it) and scaling only the quasi-steady part (stable
 * everywhere, so the LCG 70 % model no longer porpoised). L_C is interpolated where Wagner's half-beam reaches the chine
 * (behind the transom, negative, when the chines are dry there), so k changes smoothly with the attitude; measured on
 * Savitsky's prismatic relation from the wetted keel instead, the runabout's curved bow made it porpoise at 18 m/s.
 * <p>Heave radiation at low speed (Cummins memory, WaveRadiation) is not part of this step; a caller supplies it as an
 * external load, as for HullDynamics, faded out with the ventilation since the zero-speed memory does not hold for a
 * planing hull.
 * <p>Sideways the same slender-body form F = -D/Dt(m_22 w) is used with the Lewis sectional added mass of the wet
 * section, plus Hoerner's cross-flow drag; the time derivative of m_22 is neglected. Aft of the largest wet section the
 * cross-flow leaves the keel into the wake, so m_22 is held at its largest from the stem aft instead of being recovered
 * (Jones's 1946 slender-wing assumption for decreasing span, since 0.23): recovered, a hull tapering to its stern had
 * potential flow's full Munk moment and a float plane turned circles on the water. The deadrise factor is a lift
 * correction and does not act sideways.
 * Friction follows ITTC-1957 on the wetted bottom (Reynolds number on the wetted keel length).
 * <p>Whisker spray drag (Savitsky, DeLorme and Datla 2007) is added to the friction. Its sheet lies between the
 * stagnation line and the spray edge, out to the chines and, when the chines are dry at the transom, only as far aft as
 * the transom (2026-10-12; before, the drag vanished there, a step of a fifth of the resistance).
 * <p>Steps (PlaningHull.of, since 0.23): a step is a transom for the forebody (its ventilation against the step's own
 * C_V = U / sqrt(g B), and Garme's reduction ahead of it), and behind it the afterbody runs in the forebody's wake.
 * Savitsky and Morabito (2010, Marine Technology 47(1)) give the wake's centreline above the forebody's keel line extended
 * aft: H / B = 0.17 (A + 0.03 L_k tau^1.5) sin((pi / C_V) (X / 3)^1.5), X beams aft of the step, L_k the forebody's wetted
 * keel in beams, tau the trim in degrees, A = 1.5 at 10 deg deadrise and 2.0 from 20 (valid to X = 3 B, C_V 4 to 8;
 * the sine is held at its peak beyond). Each afterbody station sees the water surface no higher than that wake, faded
 * in with the step's ventilation (from wet at C_V 2 to open at 4: NACA TN 594's unventilated step jumped near 1.9), so in
 * the hump the hull is an unstepped one. H is taken above the horizontal through the step's keel, not along the trimmed
 * keel line as the formula is usually read: read that way the afterbody of Sajedi et al.'s model B (PlaningDynamicsTest)
 * stayed so dry that it trimmed 9.6 deg at 9 m/s (tank 3.55) and porpoised beyond 11 m/s. Their depth, VCG and LCG datum
 * are not given, so this is a calibration, not a proof. Each surface (afterbody, forebody) has its own deadrise factor,
 * from its own chine-wetted length ahead of its transom or step; while a step is wet the hull is one surface, in the hump,
 * and the factor is faded in with the first step's ventilation as well as the transom's (Sajedi's model at 5 m/s, C_V 2.15:
 * 10.1 deg without that fade, 8.0 with it, tank 6.44; the same hull without its step runs at 8.8, Savitsky 8.2).
 * <p>Calm-water validation (PlaningDynamicsTest): against Fridsma (1969), 20 deg deadrise, V/sqrt(L) 4 to 6 (C_V 2.7 to
 * 4), running trim within 0.35 deg in all five Table 1 configurations (light and heavy) and Fig. 11, R/Delta 6 to 17 %
 * high, steady where Fridsma ran steadily, and the LCG 70 % model starts to porpoise between V/sqrt(L) 5.5 and 6.2
 * (tank: about 5.85); against Savitsky (1964) with the whisker spray of Savitsky, DeLorme and Datla (2007) over C_V 2
 * to 5 and C_Delta 0.15 to 0.6, see the test. 2D+t is valid above C_V of about 2; it under-predicts trim and resistance
 * in the pre-planing range. Not modelled: spray drag other than the whisker spray, the flare and bow flow of the curved
 * bow, asymmetric (heeled) impact, aerodynamic lift and transom-draft effects below ventilation.
 */
public final class PlaningDynamics {
    /** Transom-beam speed coefficients bounding the change from displacement to planing hydrostatics. */
    public static final double VENTILATED = 0.5, PLANING = 1.5;
    /** A step's speed coefficients (on its beam) between which it ventilates: wet in the hump (NACA TN 594 found an
     * unventilated step's discontinuity near C_V 1.9), open as Savitsky and Morabito's wake data (from C_V 4). */
    public static final double STEP_WET = 2, STEP_VENTILATED = 4;
    /**
     * Garme's (2005) near-transom reduction: the pressure falls to atmospheric at a dry transom, so sectional lift
     * builds up only over a length 0.34 B C_V forward of it, mu = tanh(2.5 x_T / (0.34 B C_V)) (coefficient as published,
     * fitted by Garme to Fridsma's calm-water attitudes; form also in Ghadimi et al. 2016 eq. 19).
     */
    public static final double TRANSOM_REDUCTION = 0.34;
    /**
     * The deadrise factor on a planing surface's normal loads, k = 1 - sin(beta) + LIFT_RECOVERY L_C / B within
     * [MIN_LIFT_FACTOR, 1]: Shuford's (1958) 1 - sin(beta), recovering by 0.13 per beam of chine-wetted length L_C
     * (fitted to Savitsky's 1964 lift; see the class description). The lower bound is a limit for deep vees beyond
     * Savitsky's 30 deg.
     */
    public static final double LIFT_RECOVERY = 0.13, MIN_LIFT_FACTOR = 0.3;

    private PlaningDynamics() {}

    /** Wrenches about the step's starting COM; hydrodynamic = normal and lateral 2D+t forces and cross-flow drag. */
    public record Result(RigidBodyState state, Wrench hydrostatics, Wrench hydrodynamic, Wrench friction, Wrench inertia, Wrench contact,
                         Wrench external, List<AppliedForce> forces, Diagnostics diagnostics, double contactFrictionW) {}
    /** Wetted keel length, surge speed through the water, friction force, transom ventilation 0..1, wet volume, dissipated power. */
    public record Diagnostics(double wettedLengthM, double surgeMps, double frictionN, double ventilation, double wetVolumeM3, double dragPowerW,
                              double[] waterlineBreadth) {}
    public record Prepared(RigidBodyProperties properties, RigidBodyState damped, Wrench hydrostatics, Wrench hydrodynamic, Wrench friction,
                           Wrench inertia, Wrench external, List<AppliedForce> forces, Diagnostics diagnostics, double dt) {}
    /** Loads of one state (world axes, torques about the COM), the strip added mass in body axes about the COM, display forces and diagnostics. */
    public record Loads(Wrench hydro, Wrench dynamic, Wrench friction, Wrench excitation, double[][] addedBody, List<AppliedForce> forces, Diagnostics diagnostics) {}

    /** The planing hull's loads at one state, for composite bodies; includeWeight false leaves out p's weight (see HullDynamics.loads). */
    public static Loads loads(RigidBodyState s, RigidBodyProperties p, PlaningHull hull, HullDynamics.Environment environment, Vec3d gravity, boolean includeWeight) {
        return assemble(s, p, hull, environment, gravity, includeWeight);
    }

    public static Prepared prepare(RigidBodyState s, RigidBodyProperties p, PlaningHull hull, HullDynamics.Environment environment, Vec3d gravity,
                                   List<AppliedForce> externalLoads, double dt) {
        Checks.positive(dt, "Time step");
        Loads loads = assemble(s, p, hull, environment, gravity, true);
        List<AppliedForce> forces = new ArrayList<>(loads.forces());
        Wrench external = Wrench.ZERO;
        for (AppliedForce load : externalLoads) { external = external.add(load.wrench()); forces.add(load); }
        Wrench load = loads.hydro().add(loads.dynamic()).add(loads.friction()).add(loads.excitation()).add(external);
        double[][] addedWorld = HullDynamics.rotate(loads.addedBody(), s.orientation());
        RigidBodyState damped = RigidBodyDynamics.implicitVelocity(s, p, load, addedWorld, dt);
        Vec3d omega = s.angularVelocity(p), dv = damped.velocity().subtract(s.velocity()), dw = damped.angularVelocity(p).subtract(omega);
        double[] change = {dv.x(), dv.y(), dv.z(), dw.x(), dw.y(), dw.z()}, reaction = new double[6];
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { reaction[i] -= addedWorld[i][j] * change[j] / dt; }
        Wrench inertia = loads.excitation().add(new Wrench(new Vec3d(reaction[0], reaction[1], reaction[2]), new Vec3d(reaction[3], reaction[4], reaction[5])));
        if (inertia.force().lengthSquared() > 0 || inertia.torque().lengthSquared() > 0) {
            forces.add(new AppliedForce(AppliedForce.Kind.INERTIA, Vec3d.ZERO, inertia.force(), inertia.torque()));
        }
        return new Prepared(p, damped, loads.hydro(), loads.dynamic(), loads.friction(), inertia, external, List.copyOf(forces), loads.diagnostics(), dt);
    }

    public static Result finish(Prepared p, RigidBodyState afterContact, Wrench contact, List<AppliedForce> contactForces, double contactFrictionW,
                                FloatingBodyDynamics.Collision collision) {
        List<AppliedForce> forces = new ArrayList<>(p.forces());
        forces.addAll(contactForces);
        RigidBodyState next = RigidBodyDynamics.drift(afterContact, p.properties(), p.dt());
        if (collision != null) { next = ContactSolver.separate(next, p.properties(), collision.shape(), collision.obstacles()); }
        return new Result(next, p.hydrostatics(), p.hydrodynamic(), p.friction(), p.inertia(), contact, p.external(), List.copyOf(forces),
                p.diagnostics(), contactFrictionW);
    }

    public static Result step(RigidBodyState s, RigidBodyProperties p, PlaningHull hull, HullDynamics.Environment environment, Vec3d gravity,
                              List<AppliedForce> externalLoads, double dt) {
        Prepared prepared = prepare(s, p, hull, environment, gravity, externalLoads, dt);
        return finish(prepared, prepared.damped(), Wrench.ZERO, List.of(), 0, null);
    }

    /** Transom ventilation 0..1 against the transom-beam speed coefficient (smoothstep from 0.5 to 1.5). */
    public static double ventilation(double cvt) {
        double s = Math.clamp((cvt - VENTILATED) / (PLANING - VENTILATED), 0, 1);
        return s * s * (3 - 2 * s);
    }

    /** The deadrise factor of a planing surface (see LIFT_RECOVERY) of deadrise beta (rad), chine-wetted length and beam (m). */
    public static double liftFactor(double deadrise, double chineWettedLengthM, double beamM) {
        return Math.clamp(1 - Math.sin(deadrise) + LIFT_RECOVERY * chineWettedLengthM / beamM, MIN_LIFT_FACTOR, 1);
    }

    /**
     * The hull's steps this step: for each, its x, beam, keel point (world), speed coefficient, ventilation, Garme length,
     * the forebody's wetted keel (beams), the wake constant A; and the trim (degrees).
     */
    private record Steps(double[] x, double[] beam, Vec3d[] keel, double[] cv, double[] vent, double[] reduction, double[] keelLength,
                         double[] a, double trimDeg) {
        static final Steps NONE = new Steps(new double[0], null, null, null, null, null, null, null, 0);

        static Steps of(RigidBodyState s, RigidBodyProperties p, PlaningHull planing, HullDynamics.Environment environment, Vec3d ex, Vec3d up,
                        Vec3d omega, double g) {
            double[] xs = planing.steps();
            if (xs.length == 0) { return NONE; }
            SlenderHull hull = planing.hull();
            int n = hull.stations(), m = xs.length;
            double[] beam = new double[m], cv = new double[m], vent = new double[m], reduction = new double[m], lk = new double[m], a = new double[m];
            Vec3d[] keel = new Vec3d[m];
            for (int k = 0; k < m; k++) {
                int fore = 0;
                while (fore < n - 1 && hull.sections().get(fore).xM() <= xs[k]) { fore++; }
                beam[k] = 2 * planing.chineHalfBeam(fore);
                keel[k] = s.worldPoint(p, new Vec3d(xs[k], planing.keelY(fore), 0));
                HullDynamics.Water w = environment.sample(keel[k]);
                Vec3d relative = s.velocity().add(omega.cross(keel[k].subtract(s.position()))).subtract(w.velocity());
                cv[k] = Math.abs(relative.dot(ex)) / Math.sqrt(g * beam[k]);
                double ramp = Math.clamp((cv[k] - STEP_WET) / (STEP_VENTILATED - STEP_WET), 0, 1);
                vent[k] = ramp * ramp * (3 - 2 * ramp);
                reduction[k] = TRANSOM_REDUCTION * beam[k] * cv[k];
                // the forebody's wetted keel: from the step forward while the keel is under the surface
                double wet = 0;
                for (int i = fore; i < n; i++) {
                    Vec3d point = s.worldPoint(p, new Vec3d(hull.sections().get(i).xM(), planing.keelY(i), 0));
                    HullDynamics.Water here = environment.sample(point);
                    if (!here.hasWater() || point.y() >= here.surfaceY()) { break; }
                    wet = hull.sections().get(i).xM() - xs[k];
                }
                lk[k] = wet / beam[k];
                double beta = Math.toDegrees(planing.deadrise(fore));
                a[k] = 1.5 + 0.5 * Math.clamp((beta - 10) / 10, 0, 1);
            }
            return new Steps(xs, beam, keel, cv, vent, reduction, lk, a, Math.toDegrees(Math.asin(Math.clamp(ex.dot(up), -1, 1))));
        }

        /** Garme's pressure reduction ahead of each ventilated step (1 elsewhere). */
        double reduction(double station) {
            double mu = 1;
            for (int k = 0; k < x.length; k++) {
                if (station > x[k] && reduction[k] > 0) { mu *= 1 - vent[k] * (1 - Math.tanh(2.5 * (station - x[k]) / reduction[k])); }
            }
            return mu;
        }

        /** Which planing surface a station belongs to: 0 behind the first step (or the whole unstepped hull), k ahead of step k - 1. */
        int surface(double station) {
            int k = 0;
            while (k < x.length && station > x[k]) { k++; }
            return k;
        }

        /** The water an afterbody station sees: the surface no higher than the wake of the nearest step ahead of it. */
        HullDynamics.Water wake(double station, HullDynamics.Water water, Vec3d ex) {
            int k = -1;
            for (int j = 0; j < x.length; j++) { if (x[j] > station && (k < 0 || x[j] < x[k])) { k = j; } }
            if (k < 0 || !(vent[k] > 0) || !water.hasWater()) { return water; }
            double behind = (x[k] - station) / beam[k];
            double phase = Math.min(Math.PI / Math.max(cv[k], 1e-6) * Math.pow(behind / 3, 1.5), Math.PI / 2);
            double h = 0.17 * (a[k] + 0.03 * keelLength[k] * Math.pow(Math.max(trimDeg, 0), 1.5)) * Math.sin(phase) * beam[k];
            // H above the horizontal through the step's keel (see the class description for why not the trimmed keel line)
            double wakeY = keel[k].y() + h;
            double drop = vent[k] * Math.max(0, water.surfaceY() - wakeY);
            if (!(drop > 0)) { return water; }
            return new HullDynamics.Water(water.surfaceY() - drop, water.surfaceSlope(), water.stillWaterY(), water.decayWavenumber(), water.velocity(),
                    water.acceleration(), water.water(), water.air());
        }
    }

    /**
     * The deadrise factor of each station's planing surface (LIFT_RECOVERY). A surface's chine-wetted length is measured
     * from its aft end (the transom or its step) to where Wagner's half-beam reaches the chine, interpolated between
     * stations (ratio = Wagner half-beam / chine half-beam, immersion of the keel, negative where dry); behind the aft end
     * (negative) when the chines are dry there. A wet step joins its forebody to the surface behind it, and fades the factor out.
     */
    private static double[] liftFactors(PlaningHull planing, SlenderHull hull, Steps steps, double transomVentilation, double trim,
                                        double[] immersion, double[] ratio) {
        int n = hull.stations(), surfaces = steps.x().length + 1;
        int[] surface = new int[n], aft = new int[surfaces], end = new int[surfaces];
        java.util.Arrays.fill(aft, -1);
        for (int i = 0; i < n; i++) {
            surface[i] = steps.surface(hull.sections().get(i).xM());
            if (aft[surface[i]] < 0) { aft[surface[i]] = i; }
            end[surface[i]] = i;
        }
        double[] own = new double[surfaces];
        for (int k = 0; k < surfaces; k++) {
            if (aft[k] < 0) { own[k] = 1; continue; }
            // the length ahead runs into the next surface as far as the step between them is wet
            double ventAhead = k + 1 < surfaces ? steps.vent()[k] : 1;
            double length = ventAhead * chineWetted(planing, hull, aft[k], end[k], trim, immersion, ratio)
                    + (1 - ventAhead) * chineWetted(planing, hull, aft[k], n - 1, trim, immersion, ratio);
            own[k] = Double.isNaN(length) ? 1 : liftFactor(planing.deadrise(aft[k]), length, 2 * planing.chineHalfBeam(aft[k]));
        }
        // a planing correction: faded in with the transom's ventilation and, on a stepped hull, the first step's (wet, the
        // hull is in the hump, where buoyancy carries it)
        double fade = transomVentilation * (surfaces > 1 ? steps.vent()[0] : 1);
        double[] perSurface = new double[surfaces];
        for (int k = 0; k < surfaces; k++) {
            perSurface[k] = k == 0 ? 1 - fade * (1 - own[0]) : steps.vent()[k - 1] * own[k] + (1 - steps.vent()[k - 1]) * perSurface[k - 1];
        }
        double[] factor = new double[n];
        for (int i = 0; i < n; i++) { factor[i] = perSurface[surface[i]]; }
        return factor;
    }

    /** Chine-wetted length (m) from station first's x over stations first..last; NaN where the keel is dry or the hull trims bow down. */
    private static double chineWetted(PlaningHull planing, SlenderHull hull, int first, int last, double trim, double[] immersion, double[] ratio) {
        double x0 = hull.sections().get(first).xM();
        int front = -1, keel = -1;
        for (int i = first; i <= last; i++) {
            if (ratio[i] >= 1) { front = i; }
            if (keel < 0 && immersion[i] > 0) { keel = i; }
        }
        if (front >= 0) {
            double x = hull.sections().get(front).xM();
            if (front < last && ratio[front + 1] < 1) {
                double next = hull.sections().get(front + 1).xM();
                x += (next - x) * (ratio[front] - 1) / (ratio[front] - ratio[front + 1]);
            }
            return x - x0;
        }
        if (keel < 0 || !(trim > 1e-3)) { return Double.NaN; }
        // chines dry: where they would wet behind the aftmost wet station, on the keel's slope
        double growth = 0.5 * Math.PI / Math.tan(planing.deadrise(keel)) * Math.tan(trim);
        return hull.sections().get(keel).xM() - x0 - (1 - ratio[keel]) * planing.chineHalfBeam(keel) / growth;
    }

    /** The keel's wetted length (m) from the transom: where the foremost wet station's keel leaves the water, interpolated. */
    private static double keelWettedTo(SlenderHull hull, int front, double[] immersion) {
        double x = hull.sections().get(front).xM();
        if (front + 1 < hull.stations() && immersion[front + 1] <= 0) {
            double next = hull.sections().get(front + 1).xM();
            x += (next - x) * immersion[front] / (immersion[front] - immersion[front + 1]);
        }
        return x;
    }

    /** One side's whisker-spray plan area between the keel apex and a line at slope t from it, out to the chine c, over x. */
    private static double sprayWedge(double x, double c, double t) {
        return x * t <= c ? 0.5 * t * x * x : 0.5 * c * c / t + c * (x - c / t);
    }

    private static Loads assemble(RigidBodyState s, RigidBodyProperties p, PlaningHull planing, HullDynamics.Environment environment, Vec3d gravity, boolean includeWeight) {
        SlenderHull hull = planing.hull();
        int n = hull.stations();
        Orientation q = s.orientation();
        Vec3d ex = q.rotate(Vec3d.X), ey = q.rotate(Vec3d.Y), ez = q.rotate(Vec3d.Z);
        Vec3d omega = s.angularVelocity(p);
        double g = gravity.length();
        Vec3d up = g > 0 ? gravity.scale(-1 / g) : Vec3d.Y;
        Wrench hydro = includeWeight ? new Wrench(gravity.scale(p.massKg()), Vec3d.ZERO) : Wrench.ZERO, dynamic = Wrench.ZERO, friction = Wrench.ZERO, excitation = Wrench.ZERO;
        List<AppliedForce> forces = new ArrayList<>();
        if (includeWeight) { forces.add(new AppliedForce(AppliedForce.Kind.WEIGHT, Vec3d.ZERO, gravity.scale(p.massKg()))); }
        double[][] addedBody = new double[6][6];
        double[] dx = new double[n], momentumY = new double[n], momentumZ = new double[n], surge = new double[n], girth = new double[n], density = new double[n];
        double[] breadth = new double[n], addedY = new double[n], addedZ = new double[n], crossY = new double[n], crossZ = new double[n];
        // Normal loads of each station, kept until its surface's deadrise factor is known: buoyancy (and where), the 2D+t
        // normal force, added mass and diffraction; the lateral rest of the station's force.
        Vec3d[] buoyancy = new Vec3d[n], buoyancyAt = new Vec3d[n], lateral = new Vec3d[n];
        double[] normalForce = new double[n], normalAdded = new double[n], normalDiffraction = new double[n], immersion = new double[n], wetRatio = new double[n];
        java.util.Arrays.fill(immersion, -1);
        boolean[] keelWet = new boolean[n], chinesWet = new boolean[n];
        Vec3d[] keelOffset = new Vec3d[n], relative = new Vec3d[n];
        HullSection.Wet[] wets = new HullSection.Wet[n];
        double wetVolume = 0, wettedLength = 0, kinematicViscosity = 0;
        // Transom-beam speed coefficient from the surge speed through the water at the transom.
        HullDynamics.Water transomWater = environment.sample(s.worldPoint(p, new Vec3d(0, planing.keelY(0), 0)));
        Vec3d transomRelative = s.velocity().add(omega.cross(s.worldOffset(p, new Vec3d(0, planing.keelY(0), 0)))).subtract(transomWater.velocity());
        double cvt = Math.abs(transomRelative.dot(ex)) / Math.sqrt(g * 2 * planing.chineHalfBeam(0));
        double ventilation = ventilation(cvt);
        double dragPower = 0, reductionLength = TRANSOM_REDUCTION * 2 * planing.chineHalfBeam(0) * cvt;
        double trimAngle = Math.asin(Math.clamp(ex.dot(up), -1, 1));
        Steps steps = Steps.of(s, p, planing, environment, ex, up, omega, g);
        for (int i = 0; i < n; i++) {
            HullSection section = hull.sections().get(i);
            dx[i] = planing.boundary(i + 1) - planing.boundary(i);
            // Applied to the hydrostatic load only, faded in with the transom's ventilation (it needs a dry transom).
            double mu = reductionLength > 0 ? 1 - ventilation * (1 - Math.tanh(2.5 * section.xM() / reductionLength)) : 1;
            mu *= steps.reduction(section.xM());
            Vec3d origin = s.worldPoint(p, new Vec3d(section.xM(), 0, 0));
            Vec3d keel = origin.add(ey.scale(planing.keelY(i)));
            keelOffset[i] = keel.subtract(s.position());
            HullDynamics.Water water = steps.wake(section.xM(), environment.sample(keel), ex);
            double rho = water.water().densityKgPerM3();
            density[i] = rho;
            kinematicViscosity = water.water().dynamicViscosityPaS() / rho;
            Vec3d pointVelocity = s.velocity().add(omega.cross(keelOffset[i]));
            relative[i] = pointVelocity.subtract(water.velocity());
            surge[i] = relative[i].dot(ex);
            if (!water.hasWater()) { wets[i] = HullSection.Wet.DRY; continue; }
            HullSection.Wet wet = HullDynamics.clip(section, origin, keel, ey, ez, water.surfaceY(), water.surfaceSlope());
            wets[i] = wet;
            breadth[i] = wet.waterlineBreadthM();
            // Buoyancy of the clipped section (Archimedes), scaled by a_B; the full value is kept for the moment factor.
            if (wet.areaM2() > 0) {
                buoyancyAt[i] = origin.add(ey.scale(wet.centroidY())).add(ez.scale(wet.centroidZ())).subtract(s.position());
                // Unventilated: Archimedes (vertical). With a dry transom the bottom pressure is no longer balanced by a
                // transom face, so the hydrostatic load acts normal to the bottom (Zarnick: all sectional loads normal to the keel).
                Vec3d direction = up.scale(1 - ventilation).add(ey.scale(ventilation)).normalized();
                buoyancy[i] = direction.scale(g * rho * wet.areaM2() * dx[i] * mu);
                wetVolume += wet.areaM2() * dx[i];
            }
            // Immersion of the keel along the body normal, to the local plane surface.
            Vec3d slope = water.surfaceSlope();
            double denominator = ey.y() - slope.x() * ey.x() - slope.z() * ey.z();
            double h = denominator > 0.1 ? (water.surfaceY() - keel.y()) / denominator : 0;
            if (denominator > 0.1) { immersion[i] = h; }
            if (!(h > 0)) { continue; }
            wettedLength += dx[i];
            double cotBeta = 1 / Math.tan(planing.deadrise(i)), cosBeta = Math.cos(planing.deadrise(i));
            double chine = planing.chineHalfBeam(i), wagner = 0.5 * Math.PI * h * cotBeta;
            boolean chineWet = wagner >= chine;
            keelWet[i] = true; chinesWet[i] = chineWet;
            wetRatio[i] = wagner / chine;
            double b = Math.min(wagner, chine), ma = 0.5 * Math.PI * rho * b * b;
            double m22 = wet.areaM2() > 0 ? HullModel.sectionAddedMass(section, wet, rho)[0] : 0;
            HullDynamics.addPointMass(addedBody, q.inverse().rotate(keelOffset[i]), 0, 0, m22 * dx[i]);
            normalAdded[i] = ma * dx[i];
            double v = -relative[i].dot(ey), w = relative[i].dot(ez);
            addedY[i] = ma; addedZ[i] = m22; crossY[i] = v; crossZ[i] = w;
            // Rate of immersion at this body station: the surface's vertical motion minus the keel point's.
            double immersionRate = (water.velocity().y() - pointVelocity.y()) / denominator;
            double dmdt = chineWet ? 0 : Math.PI * rho * b * 0.5 * Math.PI * cotBeta * immersionRate;
            double normal = dmdt * v + cosBeta * rho * b * v * Math.abs(v);
            // The implicit update supplies -m_a (a + alpha x r) . e; the rest of the strip's d(V)/dt in the turning body
            // frame -- centripetal acceleration and the rotation of the normal -- is added here explicitly.
            Vec3d centripetal = omega.cross(omega.cross(keelOffset[i]));
            double frameY = -ma * (centripetal.dot(ey) + relative[i].dot(omega.cross(ey)));
            double frameZ = -m22 * (centripetal.dot(ez) + relative[i].dot(omega.cross(ez)));
            normalForce[i] = (normal + frameY) * dx[i];
            // Lateral cross-flow drag on the wet draft (Hoerner, as for displacement hulls).
            double draft = Math.max(wet.draftM(), 1e-6), ratio = Math.max(wet.waterlineBreadthM(), 2 * chine) / (2 * draft);
            Vec3d fz = ez.scale(-0.5 * rho * HullManoeuvring.hoernerCrossFlow(ratio) * draft * dx[i] * Math.abs(w) * w);
            lateral[i] = ez.scale(frameZ * dx[i]).add(fz);
            // Wave excitation: diffraction with the strip added masses, horizontal Froude-Krylov on the wet area.
            Vec3d a = water.acceleration();
            Vec3d froudeKrylov = a.subtract(up.scale(a.dot(up))).scale(rho * wet.areaM2() * dx[i]);
            normalDiffraction[i] = ma * dx[i] * a.dot(ey);
            excitation = excitation.add(Wrench.atOffset(froudeKrylov.add(ez.scale(m22 * dx[i] * a.dot(ez))), keelOffset[i]));
            // Wetted girth: the bottom out to the wetted half-beam; the sides only while the chines are not ventilated.
            girth[i] = Math.max(2 * b / cosBeta, (1 - ventilation) * wet.girthM());
        }
        // Each surface's deadrise factor on its normal loads (LIFT_RECOVERY), now that its chine-wetted length is known.
        double[] factor = liftFactors(planing, hull, steps, ventilation, trimAngle, immersion, wetRatio);
        for (int i = 0; i < n; i++) {
            if (buoyancy[i] != null) {
                Vec3d load = buoyancy[i].scale(factor[i]);
                hydro = hydro.add(Wrench.atOffset(load, buoyancyAt[i]));
                forces.add(new AppliedForce(AppliedForce.Kind.BUOYANCY, buoyancyAt[i], load));
            }
            if (!keelWet[i]) { continue; }
            HullDynamics.addPointMass(addedBody, q.inverse().rotate(keelOffset[i]), 0, factor[i] * normalAdded[i], 0);
            Vec3d f = ey.scale(factor[i] * normalForce[i]).add(lateral[i]);
            dynamic = dynamic.add(Wrench.atOffset(f, keelOffset[i]));
            forces.add(new AppliedForce(AppliedForce.Kind.LIFT, keelOffset[i], f));
            dragPower -= f.dot(relative[i]);
            excitation = excitation.add(Wrench.atOffset(ey.scale(factor[i] * normalDiffraction[i]), keelOffset[i]));
        }
        // Sideways, aft of a wet run's largest section the cross-flow leaves the keel (a sharp edge) into the wake: the
        // sectional added mass is held at its largest from the stem aft rather than recovered (Jones's slender-wing
        // assumption for decreasing span). A dry station ends the run (a ventilated step or transom sheds it all). The
        // normal term keeps its own: behind a step the afterbody planes afresh on the forebody's wake.
        for (int i = n - 1; i >= 0; i--) {
            if (addedZ[i] > 0 && i < n - 1) { addedZ[i] = Math.max(addedZ[i], addedZ[i + 1]); }
            momentumY[i] = addedY[i] * crossY[i];
            momentumZ[i] = addedZ[i] * crossZ[i];
        }
        // Convective (steady) terms, integrated over each strip between its boundaries: -U [m_a V], +U [m_22 w].
        for (int i = 0; i < n; i++) {
            if (momentumY[i] == 0 && momentumZ[i] == 0 && (i == 0 || (momentumY[i - 1] == 0 && momentumZ[i - 1] == 0))
                    && (i == n - 1 || (momentumY[i + 1] == 0 && momentumZ[i + 1] == 0))) { continue; }
            // Normal: d(m_a V) = m_a dV + V dm_a over the strip; where the section shrinks as the water passes aft (it leaves
            // the water) the flow separates and dm_a gives no force (Zarnick's dm_a/dt = 0 in water exit), not suction.
            double aftMa = i == 0 ? addedY[0] : 0.5 * (addedY[i - 1] + addedY[i]), foreMa = i == n - 1 ? addedY[i] : 0.5 * (addedY[i] + addedY[i + 1]);
            double aftV = i == 0 ? crossY[0] : 0.5 * (crossY[i - 1] + crossY[i]), foreV = i == n - 1 ? crossY[i] : 0.5 * (crossY[i] + crossY[i + 1]);
            double normalChange = 0.5 * (aftMa + foreMa) * (foreV - aftV) + (foreMa > aftMa ? 0 : 0.5 * (aftV + foreV) * (foreMa - aftMa));
            double aftZ = i == 0 ? momentumZ[0] : 0.5 * (momentumZ[i - 1] + momentumZ[i]);
            double foreZ = i == n - 1 ? momentumZ[i] : 0.5 * (momentumZ[i] + momentumZ[i + 1]);
            Vec3d f = ey.scale(-surge[i] * normalChange * factor[i]).add(ez.scale(surge[i] * (foreZ - aftZ)));
            if (f.lengthSquared() == 0) { continue; }
            dynamic = dynamic.add(Wrench.atOffset(f, keelOffset[i]));
            forces.add(new AppliedForce(AppliedForce.Kind.LIFT, keelOffset[i], f));
            dragPower -= f.dot(relative[i]);
        }
        // Friction (ITTC-1957) on the wetted bottom, Reynolds number on the wetted keel length.
        double frictionTotal = 0;
        if (wettedLength > 0) {
            double meanSurge = 0, weight = 0;
            for (int i = 0; i < n; i++) { if (girth[i] > 0) { meanSurge += Math.abs(surge[i]) * girth[i] * dx[i]; weight += girth[i] * dx[i]; } }
            meanSurge = weight > 0 ? meanSurge / weight : 0;
            double cf = HullResistance.ittc1957(Math.max(meanSurge, 1e-6) * wettedLength / Math.max(kinematicViscosity, 1e-12));
            for (int i = 0; i < n; i++) {
                if (girth[i] <= 0 || surge[i] == 0) { continue; }
                double magnitude = 0.5 * density[i] * cf * surge[i] * surge[i] * girth[i] * dx[i];
                Vec3d f = ex.scale(-Math.signum(surge[i]) * magnitude);
                friction = friction.add(Wrench.atOffset(f, keelOffset[i]));
                frictionTotal += magnitude;
                dragPower += magnitude * Math.abs(surge[i]);
            }
            forces.add(new AppliedForce(AppliedForce.Kind.DRAG, Vec3d.ZERO, friction.force()));
        }
        // Whisker spray drag (Savitsky, DeLorme and Datla 2007, Marine Technology 44(1)): the thin sheet thrown forward
        // of the stagnation line rubs on the bottom between the stagnation line (angle a to the keel in plan,
        // tan a = pi tan(trim) / (2 tan beta)) and the spray edge (2a), out to the chines: both sides together
        // D = rho V^2 C_f b^2 cos(2a) / (8 sin(2a) cos(beta)), C_f by ITTC-1957 on the sheet's path b / (2 cos(beta) sin(2a)).
        // When the chines are dry at the transom the sheet reaches only as far aft as the transom (sprayWedge), which
        // gives the same drag once they are wet ahead of it. Faded in with the ventilation like the rest.
        int front = -1, chineFront = -1;
        for (int i = 0; i < n; i++) { if (keelWet[i]) { front = i; } if (chinesWet[i]) { chineFront = i; } }
        if (front >= 0 && front > chineFront && trimAngle > 0 && ventilation > 0) {
            int m = (front + Math.max(chineFront, 0) + 1) / 2;
            double beta = planing.deadrise(m), cosBeta = Math.cos(beta);
            double alpha = Math.atan(Math.PI * Math.tan(trimAngle) / (2 * Math.tan(beta))), edge = 2 * alpha;
            double b = 2 * planing.chineHalfBeam(m), speed = Math.abs(surge[m]);
            if (edge < Math.PI / 2 && speed > 0) {
                double path = b / (2 * cosBeta * Math.sin(edge));
                double cf = HullResistance.ittc1957(speed * path / Math.max(kinematicViscosity, 1e-12));
                double length = keelWettedTo(hull, front, immersion);
                double area = 2 * (sprayWedge(length, 0.5 * b, Math.tan(edge)) - sprayWedge(length, 0.5 * b, Math.tan(alpha)));
                double spray = ventilation * 0.5 * density[m] * speed * speed * cf * area * Math.cos(edge) / cosBeta;
                Vec3d f = ex.scale(-Math.signum(surge[m]) * spray);
                friction = friction.add(Wrench.atOffset(f, keelOffset[m]));
                forces.add(new AppliedForce(AppliedForce.Kind.DRAG, keelOffset[m], f));
                frictionTotal += spray;
                dragPower += spray * speed;
            }
        }
        double surgeSpeed = s.velocity().subtract(transomWater.velocity()).dot(ex);
        return new Loads(hydro, dynamic, friction, excitation, addedBody, forces,
                new Diagnostics(wettedLength, surgeSpeed, frictionTotal, ventilation, wetVolume, dragPower, breadth));
    }
}
