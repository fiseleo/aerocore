package dev.mcmod.aerocore.api.wheel;

import dev.mcmod.aerocore.api.GroundContact;

/**
 * One tyre's contact over one step (0.24): Pacejka's transient slip model (Tire and Vehicle Dynamics, 2012, ch. 7)
 * with the contact deflections u (along the heading) and v (to the left) as states, so forces build up over the
 * relaxation lengths, do not flip sign at low speed and hold a parked vehicle as a spring:
 * <pre>
 *   du/dt = -V_sx - |V_x| u / sigma_x,   dv/dt = -V_sy - |V_x| v / sigma_y,   s_x = u / sigma_x,   s_y = v / sigma_y
 * </pre>
 * with the slip velocities V_sx = V_x - omega r_e and V_sy = V_y of the contact over the ground, and the forces the
 * tyre's steady curves at the transient slips (Tyre.forces). In steady rolling s_x is the slip ratio and s_y minus the
 * slip angle's tangent.
 *
 * <p>Backward Euler on the deflections makes the forces depend on the step's end speeds; step() gives them at the start
 * speeds and their slopes against the contact's slip velocities (N s/m), which the vehicle step takes implicitly: the
 * wheel's spin in the drivetrain solve (PartStep.rollingForce, slope times r_e) and the body's velocity through line
 * dampers (PartStep.damper). Below LOW_SPEED a damper on the slip velocity fades in (Pacejka's low-speed damping),
 * 0.7 of critical for the tyre's spring against its share of the vehicle's weight; it is bounded, with the spring,
 * by the friction. The deflections are bounded at a combined slip of 1.2 (deep sliding: a wheel spinning on the spot
 * or dragged locked does not wind them up without end). after() of the wheel part moves them on with deflections().
 */
public final class TyreContact {
    private TyreContact() {}

    /** Speed below which the low-speed damping fades in, m/s. */
    public static final double LOW_SPEED = 3;
    /** Damping ratio of the low-speed damping. */
    public static final double LOW_DAMPING = 0.7;
    /** Largest combined transient slip the deflections reach. */
    public static final double MAX_SLIP = 1.2;
    private static final double G = 9.80665;

    /**
     * A step's contact: forces at the start speeds (N, along the heading and to the left), aligning moment (N m, about
     * the ground normal, positive turning the heading to the left; -trail x lateral force), the forces' slopes against the slip velocities
     * (N s/m, each force falls by that per m/s of its slip velocity; times r_e it is the longitudinal force's slope
     * against the wheel's spin), rolling resistance (N), the peak friction and the transient slips used.
     */
    public record Step(double longitudinalN, double lateralN, double aligningNm, double longitudinalPerMps, double lateralPerMps,
                       double rollingResistanceN, double peakFriction, double slipX, double slipY) {}

    /**
     * The contact for a step: tyre, surface, load (N), the contact's velocity over the ground along the heading and to
     * the left (m/s), the wheel's rolling speed omega r_e (m/s, forward), camber (rad, positive pushes left),
     * deflections u, v (m) and the step dt (s).
     */
    public static Step step(Tyre tyre, GroundContact.Surface surface, double loadN, double vx, double vy, double rollingMps, double camberRad,
                            double u, double v, double dt) {
        if (!(loadN > 0)) { return new Step(0, 0, 0, 0, 0, 0, 0, 0, 0); }
        double sigmaX = tyre.longitudinalRelaxationM(loadN), sigmaY = tyre.lateralRelaxationM(loadN), speed = Math.abs(vx);
        double slipX = vx - rollingMps, slipY = vy;
        double denX = 1 + dt * speed / sigmaX, denY = 1 + dt * speed / sigmaY;
        double[] predicted = bound(sigmaX, sigmaY, (u - dt * slipX) / denX, (v - dt * slipY) / denY);
        double sx = predicted[0] / sigmaX, sy = predicted[1] / sigmaY, gamma = 0.12 * camberRad;
        Tyre.Curve curve = tyre.curve(surface, loadN, speed);
        double[] f = curve.forces(sx, sy + gamma), slope = curve.slopes(sx, sy + gamma);
        double fade = speed < LOW_SPEED ? 0.5 * (1 + Math.cos(Math.PI * speed / LOW_SPEED)) : 0, mass = loadN / G;
        double lowX = fade * 2 * LOW_DAMPING * Math.sqrt(tyre.slipStiffness(loadN) / sigmaX * mass);
        double lowY = fade * 2 * LOW_DAMPING * Math.sqrt(tyre.corneringStiffness(loadN) / sigmaY * mass);
        double fx = f[0] - lowX * slipX, fy = f[1] - lowY * slipY;
        double kx = Math.max(slope[0], 0) / sigmaX * dt / denX + lowX, ky = Math.max(slope[1], 0) / sigmaY * dt / denY + lowY;
        // spring and damper together cannot pass the friction
        double limit = curve.mu * loadN, total = Math.hypot(fx, fy);
        if (total > limit) {
            double scale = limit / total;
            fx *= scale; fy *= scale; kx = 0; ky = 0;
        }
        double nx = sx / curve.peakX, ny = (sy + gamma) / curve.peakY, n = Math.hypot(nx, ny);
        double trail = tyre.contactLengthM(loadN) / 6 * Math.max(0, 1 - n / 1.5);
        return new Step(fx, fy, -trail * fy, kx, ky, tyre.rollingResistanceN(surface, loadN, vx), curve.mu, sx, sy);
    }

    /**
     * The deflections {u, v} at the step's end from those at its start and the end speeds (the same backward Euler
     * step() predicted with), bounded like there.
     */
    public static double[] deflections(Tyre tyre, double loadN, double vx, double vy, double rollingMps, double u, double v, double dt) {
        if (!(loadN > 0)) {
            // off the ground the tread springs back within a step
            return new double[]{0, 0};
        }
        double sigmaX = tyre.longitudinalRelaxationM(loadN), sigmaY = tyre.lateralRelaxationM(loadN), speed = Math.abs(vx);
        double nu = (u - dt * (vx - rollingMps)) / (1 + dt * speed / sigmaX), nv = (v - dt * vy) / (1 + dt * speed / sigmaY);
        return bound(sigmaX, sigmaY, nu, nv);
    }

    /** Deflections bounded at the combined transient slip MAX_SLIP. */
    private static double[] bound(double sigmaX, double sigmaY, double u, double v) {
        double s = Math.hypot(u / sigmaX, v / sigmaY);
        if (s <= MAX_SLIP) { return new double[]{u, v}; }
        double scale = MAX_SLIP / s;
        return new double[]{u * scale, v * scale};
    }
}
