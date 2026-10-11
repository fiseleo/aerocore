package dev.mcmod.aerocore.api.engine;

/**
 * Flow through poppet valves and throttles as a compressible, isentropic orifice (Heywood 1988, appendix C):
 * mdot = C_D A p0 / sqrt(R T0) * psi(pT/p0), choked below the critical pressure ratio. The effective valve area is
 * C_D times the smaller of the curtain area (n pi D L) and the port throat (n pi (D_t^2 - D_stem^2) / 4); the
 * discharge coefficient is a property of the head's porting, given by its design.
 */
public final class ValveFlow {
    private ValveFlow() {}

    /** Geometric flow area of n valves of head diameter D at lift L with port throat Dt and stem Ds (m), m^2. */
    public static double area(int valves, double headM, double liftM, double throatM, double stemM) {
        if (liftM <= 0) { return 0; }
        double curtain = Math.PI * headM * liftM, port = Math.PI * (throatM * throatM - stemM * stemM) / 4;
        return valves * Math.min(curtain, port);
    }

    /**
     * Mass flow (kg/s) from upstream stagnation pressure p0 (Pa) and temperature T0 (K) to downstream pressure pT
     * through effective area cdA (m^2), for a gas of gas constant R and ratio of heats gamma. Zero if pT >= p0.
     */
    public static double massFlow(double cdA, double p0, double t0, double pT, double gasConstant, double gamma) {
        if (cdA <= 0 || pT >= p0) { return 0; }
        double ratio = pT / p0, critical = Math.pow(2 / (gamma + 1), gamma / (gamma - 1));
        double psi;
        if (ratio <= critical) { psi = Math.sqrt(gamma) * Math.pow(2 / (gamma + 1), (gamma + 1) / (2 * (gamma - 1))); }
        else { psi = Math.sqrt(2 * gamma / (gamma - 1) * (Math.pow(ratio, 2 / gamma) - Math.pow(ratio, (gamma + 1) / gamma))); }
        return cdA * p0 / Math.sqrt(gasConstant * t0) * psi;
    }

    /**
     * Throttle plate open area for a plate of bore D closed at psi0 and opened to psi (radians from the bore's normal),
     * the leading term of Heywood's eq. 7.1: (pi D^2 / 4)(1 - cos psi / cos psi0), the shaft's own area ignored.
     */
    public static double throttleArea(double boreM, double closedRad, double angleRad) {
        double a = Math.PI * boreM * boreM / 4 * (1 - Math.cos(angleRad) / Math.cos(closedRad));
        return Math.max(0, a);
    }
}
