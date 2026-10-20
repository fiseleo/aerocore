package dev.mcmod.aerocore.api.drive;

/**
 * Formulas for designing gearboxes, clutches and differentials (0.24); sources and checks in docs/DRIVETRAIN.md.
 * <ul>
 * <li>Friction clutch (uniform wear): T = z mu F r_m, r_m = (d_i + d_o) / 4, z the friction faces (2 for a single
 *     plate; LuK's clutch course: d 134/190 mm, 3500 N, mu 0.27 gives 153 N m).</li>
 * <li>Synchroniser cone: T = j mu F d / (2 sin alpha) (Naunheimer et al. eq. 9.2, Schaeffler); the time to bring a
 *     gear to speed is the inertia to be swung times the speed difference over that torque.</li>
 * <li>Push-belt or chain variator: T = 2 mu F_ax R / cos beta (Pulles et al. 2005; beta 11 degrees, belt-pulley mu
 *     about 0.09-0.105).</li>
 * <li>Planetary gearsets by Willis' equation (S sun, R ring teeth): Simpson three-speed (common sun, front ring
 *     input) 1 + S/R_f + R_r/R_f, 1 + S/R_f, 1, reverse -R_r/S; Ravigneaux four-speed R/S_f, (1 + S_r/S_f)/(1 + S_r/R), 1,
 *     R/(R + S_r), reverse -R/S_r; Lepelletier six-speed (simple set S1/R1 then a Ravigneaux S2, S3, R3) as US 5,106,352.
 *     They give the TH400's 2.48/1.48/1/-2.08 and the ZF 6HP26's 4.171 ... 0.691, -3.403 from their tooth counts.</li>
 * <li>Ramp limited-slip differential: lock factor L = mu N_f (r_m / r_ramp) cot theta, bias ratio (1 + L)/(1 - L).</li>
 * <li>Viscous coupling (Newtonian): T = N pi eta dOmega (r_o^4 - r_i^4) / (2 h).</li>
 * </ul>
 */
public final class Driveline {
    private Driveline() {}

    /** A friction clutch's torque capacity: faces, friction, clamp force (N), inner and outer diameters (m), N m. */
    public static double clutchCapacity(int faces, double mu, double clampN, double innerDiameterM, double outerDiameterM) {
        return faces * mu * clampN * (innerDiameterM + outerDiameterM) / 4;
    }

    /** A synchroniser's cone torque: cones, friction, axial force (N), mean cone diameter (m), cone half-angle (rad), N m. */
    public static double synchroniserTorque(int cones, double mu, double axialN, double meanDiameterM, double halfAngleRad) {
        return cones * mu * axialN * meanDiameterM / (2 * Math.sin(halfAngleRad));
    }

    /** A variator's torque capacity: friction, clamp force (N), running radius (m), pulley half-angle (rad), N m. */
    public static double beltCapacity(double mu, double clampN, double radiusM, double halfAngleRad) {
        return 2 * mu * clampN * radiusM / Math.cos(halfAngleRad);
    }

    /** Simpson three-speed ratios {1st, 2nd, 3rd, reverse} from the sun's and the front and rear rings' teeth. */
    public static double[] simpson(double sun, double frontRing, double rearRing) {
        return new double[]{1 + sun / frontRing + rearRing / frontRing, 1 + sun / frontRing, 1, -rearRing / sun};
    }

    /** Ravigneaux four-speed ratios {1st .. 4th, reverse} from the forward and reverse suns' and the ring's teeth. */
    public static double[] ravigneaux(double forwardSun, double reverseSun, double ring) {
        return new double[]{ring / forwardSun, (1 + reverseSun / forwardSun) / (1 + reverseSun / ring), 1, ring / (ring + reverseSun), -ring / reverseSun};
    }

    /** Lepelletier six-speed ratios {1st .. 6th, reverse}: simple set S1/R1, Ravigneaux forward sun S2, reverse sun S3, ring R3. */
    public static double[] lepelletier(double s1, double r1, double s2, double s3, double r3) {
        double a = 1 + s1 / r1;
        return new double[]{a * r3 / s2, a * (1 + s3 / s2) / (1 + s3 / r3), a, 1 / (1 - (s2 / r3) / (1 + r1 / s1)), 1 / (1 + (s3 / r3) / (1 + r1 / s1)),
                r3 / (r3 + s3), -a * r3 / s3};
    }

    /** A ramp limited-slip differential's lock factor: friction, active faces, mean friction radius, ramp radius (m), ramp angle from the plane of rotation (rad). */
    public static double lockFactor(double mu, int faces, double frictionRadiusM, double rampRadiusM, double rampAngleRad) {
        return mu * faces * frictionRadiusM / rampRadiusM / Math.tan(rampAngleRad);
    }

    /** The torque bias ratio of a lock factor (below 1). */
    public static double biasRatio(double lockFactor) {
        double l = Math.clamp(lockFactor, 0, 0.98);
        return (1 + l) / (1 - l);
    }

    /** A viscous coupling's coefficient (N m s/rad): gaps, fluid viscosity (Pa s), plate radii (m) and gap (m). */
    public static double viscousCoefficient(int gaps, double viscosityPaS, double innerRadiusM, double outerRadiusM, double gapM) {
        return gaps * Math.PI * viscosityPaS * (Math.pow(outerRadiusM, 4) - Math.pow(innerRadiusM, 4)) / (2 * gapM);
    }
}
