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
 * transom momentum m_a U V leaves the hull there as in Zarnick.
 * <p>Hydrostatics: the full Archimedes load of each clipped section. With the transom dry its bottom pressure is no
 * longer balanced by a transom face, so it acts normal to the keel like Zarnick's sectional loads and carries the
 * pressure drag (Delta tan trim) of a ventilated bottom. Mercier and Savitsky (1973) find the transom fully ventilated
 * at C_vT = U / sqrt(g B_T) = 0.5 and planing developed by about 1.5; the load direction and dry hull sides (friction
 * area) are blended in over that interval (smoothstep, a modelling choice). Near the dry transom the pressure falls
 * to atmospheric, so the hydrostatic load is scaled by Garme's (2005) tanh(2.5 x_T / (0.34 B C_V)) with his published
 * coefficient, faded in with the ventilation (TRANSOM_REDUCTION). Applied to the hydrodynamic strip loads as well (as
 * Garme did) it made the hull unstable here, and Zarnick's halving of the buoyancy overshot the trims, so only the
 * hydrostatics are reduced.
 * <p>Heave radiation at low speed (Cummins memory, WaveRadiation) is not part of this step; a caller supplies it as an
 * external load, as for HullDynamics, faded out with the ventilation since the zero-speed memory does not hold for a
 * planing hull.
 * <p>Sideways the same slender-body form F = -D/Dt(m_22 w) is used with the Lewis sectional added mass of the wet
 * section, plus Hoerner's cross-flow drag; the time derivative of m_22 is neglected. Friction follows ITTC-1957 on the
 * wetted bottom (Reynolds number on the wetted keel length).
 * <p>Whisker spray drag (Savitsky, DeLorme and Datla 2007) is added to the friction.
 * <p>Calm-water validation against Fridsma (1969), 20 deg deadrise, V/sqrt(L) 4 to 6 (C_V 2.7 to 4), PlaningDynamicsTest:
 * running trim within 0.35 deg in all five Table 1 configurations (light and heavy) and Fig. 11; R/Delta 5 to 13 % high;
 * steady where Fridsma ran steadily, and the LCG 70 % model starts to porpoise between V/sqrt(L) 5.5 and 6.2 (tank:
 * about 5.85). 2D+t is valid above C_V of about 2; it under-predicts trim and resistance in the pre-planing range. Not
 * modelled: spray drag other than the whisker spray, the flare and bow flow of the curved bow, asymmetric (heeled)
 * impact, aerodynamic lift and transom-draft effects below ventilation.
 */
public final class PlaningDynamics {
    /** Transom-beam speed coefficients bounding the change from displacement to planing hydrostatics. */
    public static final double VENTILATED = 0.5, PLANING = 1.5;
    /**
     * Garme's (2005) near-transom reduction: the pressure falls to atmospheric at a dry transom, so sectional lift
     * builds up only over a length 0.34 B C_V forward of it, mu = tanh(2.5 x_T / (0.34 B C_V)) (coefficient as published,
     * fitted by Garme to Fridsma's calm-water attitudes; form also in Ghadimi et al. 2016 eq. 19).
     */
    public static final double TRANSOM_REDUCTION = 0.34;

    private PlaningDynamics() {}

    /** Wrenches about the step's starting COM; hydrodynamic = normal and lateral 2D+t forces and cross-flow drag. */
    public record Result(RigidBodyState state, Wrench hydrostatics, Wrench hydrodynamic, Wrench friction, Wrench inertia, Wrench contact,
                         Wrench external, List<AppliedForce> forces, Diagnostics diagnostics, double contactFrictionW) {}
    /** Wetted keel length, surge speed through the water, friction force, transom ventilation 0..1, wet volume, dissipated power. */
    public record Diagnostics(double wettedLengthM, double surgeMps, double frictionN, double ventilation, double wetVolumeM3, double dragPowerW,
                              double[] waterlineBreadth) {}
    public record Prepared(RigidBodyProperties properties, RigidBodyState damped, Wrench hydrostatics, Wrench hydrodynamic, Wrench friction,
                           Wrench inertia, Wrench external, List<AppliedForce> forces, Diagnostics diagnostics, double dt) {}
    private record Loads(Wrench hydro, Wrench dynamic, Wrench friction, Wrench excitation, double[][] addedBody, List<AppliedForce> forces, Diagnostics diagnostics) {}

    public static Prepared prepare(RigidBodyState s, RigidBodyProperties p, PlaningHull hull, HullDynamics.Environment environment, Vec3d gravity,
                                   List<AppliedForce> externalLoads, double dt) {
        Checks.positive(dt, "Time step");
        Loads loads = assemble(s, p, hull, environment, gravity);
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

    private static Loads assemble(RigidBodyState s, RigidBodyProperties p, PlaningHull planing, HullDynamics.Environment environment, Vec3d gravity) {
        SlenderHull hull = planing.hull();
        int n = hull.stations();
        Orientation q = s.orientation();
        Vec3d ex = q.rotate(Vec3d.X), ey = q.rotate(Vec3d.Y), ez = q.rotate(Vec3d.Z);
        Vec3d omega = s.angularVelocity(p);
        double g = gravity.length();
        Vec3d up = g > 0 ? gravity.scale(-1 / g) : Vec3d.Y;
        double beta = planing.deadrise(), cotBeta = 1 / Math.tan(beta), cosBeta = Math.cos(beta);
        Wrench hydro = new Wrench(gravity.scale(p.massKg()), Vec3d.ZERO), dynamic = Wrench.ZERO, friction = Wrench.ZERO, excitation = Wrench.ZERO;
        List<AppliedForce> forces = new ArrayList<>();
        forces.add(new AppliedForce(AppliedForce.Kind.WEIGHT, Vec3d.ZERO, gravity.scale(p.massKg())));
        double[][] addedBody = new double[6][6];
        double[] dx = new double[n], momentumY = new double[n], momentumZ = new double[n], surge = new double[n], girth = new double[n], density = new double[n];
        double[] breadth = new double[n];
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
        for (int i = 0; i < n; i++) {
            HullSection section = hull.sections().get(i);
            dx[i] = planing.boundary(i + 1) - planing.boundary(i);
            // Applied to the hydrostatic load only, faded in with the transom's ventilation (it needs a dry transom).
            double mu = reductionLength > 0 ? 1 - ventilation * (1 - Math.tanh(2.5 * section.xM() / reductionLength)) : 1;
            Vec3d origin = s.worldPoint(p, new Vec3d(section.xM(), 0, 0));
            Vec3d keel = origin.add(ey.scale(planing.keelY(i)));
            keelOffset[i] = keel.subtract(s.position());
            HullDynamics.Water water = environment.sample(keel);
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
                Vec3d centroid = origin.add(ey.scale(wet.centroidY())).add(ez.scale(wet.centroidZ())).subtract(s.position());
                // Unventilated: Archimedes (vertical). With a dry transom the bottom pressure is no longer balanced by a
                // transom face, so the hydrostatic load acts normal to the bottom (Zarnick: all sectional loads normal to the keel).
                Vec3d direction = up.scale(1 - ventilation).add(ey.scale(ventilation)).normalized();
                Vec3d buoyancy = direction.scale(g * rho * wet.areaM2() * dx[i] * mu);
                hydro = hydro.add(Wrench.atOffset(buoyancy, centroid));
                forces.add(new AppliedForce(AppliedForce.Kind.BUOYANCY, centroid, buoyancy));
                wetVolume += wet.areaM2() * dx[i];
            }
            // Immersion of the keel along the body normal, to the local plane surface.
            Vec3d slope = water.surfaceSlope();
            double denominator = ey.y() - slope.x() * ey.x() - slope.z() * ey.z();
            double h = denominator > 0.1 ? (water.surfaceY() - keel.y()) / denominator : 0;
            if (!(h > 0)) { continue; }
            wettedLength += dx[i];
            double chine = planing.chineHalfBeam(i), wagner = 0.5 * Math.PI * h * cotBeta;
            boolean chineWet = wagner >= chine;
            keelWet[i] = true; chinesWet[i] = chineWet;
            double b = Math.min(wagner, chine), ma = 0.5 * Math.PI * rho * b * b;
            double m22 = wet.areaM2() > 0 ? HullModel.sectionAddedMass(section, wet, rho)[0] : 0;
            Vec3d rBody = q.inverse().rotate(keelOffset[i]);
            HullDynamics.addPointMass(addedBody, rBody, 0, ma * dx[i], m22 * dx[i]);
            double v = -relative[i].dot(ey), w = relative[i].dot(ez);
            momentumY[i] = ma * v;
            momentumZ[i] = m22 * w;
            // Rate of immersion at this body station: the surface's vertical motion minus the keel point's.
            double immersionRate = (water.velocity().y() - pointVelocity.y()) / denominator;
            double dmdt = chineWet ? 0 : Math.PI * rho * b * 0.5 * Math.PI * cotBeta * immersionRate;
            double normal = dmdt * v + cosBeta * rho * b * v * Math.abs(v);
            // The implicit update supplies -m_a (a + alpha x r) . e; the rest of the strip's d(V)/dt in the turning body
            // frame -- centripetal acceleration and the rotation of the normal -- is added here explicitly.
            Vec3d centripetal = omega.cross(omega.cross(keelOffset[i]));
            double frameY = -ma * (centripetal.dot(ey) + relative[i].dot(omega.cross(ey)));
            double frameZ = -m22 * (centripetal.dot(ez) + relative[i].dot(omega.cross(ez)));
            Vec3d fn = ey.scale((normal + frameY) * dx[i]).add(ez.scale(frameZ * dx[i]));
            // Lateral cross-flow drag on the wet draft (Hoerner, as for displacement hulls).
            double draft = Math.max(wet.draftM(), 1e-6), ratio = Math.max(wet.waterlineBreadthM(), 2 * chine) / (2 * draft);
            Vec3d fz = ez.scale(-0.5 * rho * HullManoeuvring.hoernerCrossFlow(ratio) * draft * dx[i] * Math.abs(w) * w);
            Vec3d f = fn.add(fz);
            dynamic = dynamic.add(Wrench.atOffset(f, keelOffset[i]));
            forces.add(new AppliedForce(AppliedForce.Kind.LIFT, keelOffset[i], f));
            dragPower -= f.dot(relative[i]);
            // Wave excitation: diffraction with the strip added masses, horizontal Froude-Krylov on the wet area.
            Vec3d a = water.acceleration();
            Vec3d froudeKrylov = a.subtract(up.scale(a.dot(up))).scale(rho * wet.areaM2() * dx[i]);
            Vec3d diffraction = ey.scale(ma * dx[i] * a.dot(ey)).add(ez.scale(m22 * dx[i] * a.dot(ez)));
            excitation = excitation.add(Wrench.atOffset(froudeKrylov.add(diffraction), keelOffset[i]));
            // Wetted girth: the bottom out to the wetted half-beam; the sides only while the chines are not ventilated.
            girth[i] = Math.max(2 * b / cosBeta, (1 - ventilation) * wet.girthM());
        }
        // Convective (steady) terms, integrated over each strip between its boundaries: -U [m_a V], +U [m_22 w].
        for (int i = 0; i < n; i++) {
            if (momentumY[i] == 0 && momentumZ[i] == 0 && (i == 0 || (momentumY[i - 1] == 0 && momentumZ[i - 1] == 0))
                    && (i == n - 1 || (momentumY[i + 1] == 0 && momentumZ[i + 1] == 0))) { continue; }
            double aftY = i == 0 ? momentumY[0] : 0.5 * (momentumY[i - 1] + momentumY[i]);
            double foreY = i == n - 1 ? momentumY[i] : 0.5 * (momentumY[i] + momentumY[i + 1]);
            double aftZ = i == 0 ? momentumZ[0] : 0.5 * (momentumZ[i - 1] + momentumZ[i]);
            double foreZ = i == n - 1 ? momentumZ[i] : 0.5 * (momentumZ[i] + momentumZ[i + 1]);
            Vec3d f = ey.scale(-surge[i] * (foreY - aftY)).add(ez.scale(surge[i] * (foreZ - aftZ)));
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
        // It exists where the keel is wet ahead of the chines (planing); faded in with the ventilation like the rest.
        int front = -1, chineFront = -1;
        for (int i = 0; i < n; i++) { if (keelWet[i]) { front = i; } if (chinesWet[i]) { chineFront = i; } }
        double trimAngle = Math.asin(Math.clamp(ex.dot(up), -1, 1));
        if (front > chineFront && chineFront >= 0 && trimAngle > 0 && ventilation > 0) {
            int m = (front + chineFront + 1) / 2;
            double alpha = Math.atan(Math.PI * Math.tan(trimAngle) / (2 * Math.tan(beta))), edge = 2 * alpha;
            double b = 2 * planing.chineHalfBeam(m), speed = Math.abs(surge[m]);
            if (edge < Math.PI / 2 && speed > 0) {
                double path = b / (2 * cosBeta * Math.sin(edge));
                double cf = HullResistance.ittc1957(speed * path / Math.max(kinematicViscosity, 1e-12));
                double spray = ventilation * 0.5 * density[m] * speed * speed * cf * b * b * Math.cos(edge) / (4 * Math.sin(edge) * cosBeta);
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
