package dev.mcmod.aerocore.api;

/**
 * Drag of a smooth sphere: the standard drag curve of Clift, Grace and Weber (1978, Bubbles, Drops and Particles,
 * Table 5.2), C_D(Re) in ten Reynolds-number ranges from creeping flow (C_D = 24/Re + 3/16) through the subcritical
 * plateau and the drag crisis (about 3.4e5 to 4e5) to C_D = 0.19 - 8e4/Re above 1e6. Smooth spheres in low free-stream
 * turbulence; roughness or turbulence moves the drag crisis to lower Re. The supercritical ranges above 4e5 were
 * checked for continuity but not against the original table.
 */
public final class SphereDrag {
    private SphereDrag() {}

    public static double coefficient(double reynolds) {
        if (!(reynolds > 0)) { throw new IllegalArgumentException("Reynolds number must be positive"); }
        double w = Math.log10(reynolds);
        if (reynolds < 0.01) { return 3.0 / 16 + 24 / reynolds; }
        if (reynolds <= 20) { return 24 / reynolds * (1 + 0.1315 * Math.pow(reynolds, 0.82 - 0.05 * w)); }
        if (reynolds <= 260) { return 24 / reynolds * (1 + 0.1935 * Math.pow(reynolds, 0.6305)); }
        if (reynolds <= 1.5e3) { return Math.pow(10, 1.6435 - 1.1242 * w + 0.1558 * w * w); }
        if (reynolds <= 1.2e4) { return Math.pow(10, -2.4571 + 2.5558 * w - 0.9295 * w * w + 0.1049 * w * w * w); }
        if (reynolds <= 4.4e4) { return Math.pow(10, -1.9181 + 0.6370 * w - 0.0636 * w * w); }
        if (reynolds <= 3.38e5) { return Math.pow(10, -4.3390 + 1.5809 * w - 0.1546 * w * w); }
        if (reynolds <= 4e5) { return 29.78 - 5.3 * w; }
        if (reynolds <= 1e6) { return 0.1 * w - 0.49; }
        return 0.19 - 8e4 / reynolds;
    }

    /**
     * Linear drag coefficient c (N s/m) with F = -c u for a sphere of the given radius moving at speed through the
     * fluid: c = (1/2) rho pi R^2 C_D(Re) |u|, Re = 2 R |u| / nu; finite as u goes to 0 (Stokes, 6 pi mu R).
     */
    public static double resistance(FluidState fluid, double radiusM, double speedMps) {
        Checks.positive(radiusM, "Sphere radius"); Checks.nonnegative(speedMps, "Speed");
        double rho = fluid.densityKgPerM3(), mu = fluid.dynamicViscosityPaS();
        if (rho == 0) { return 0; }
        double area = Math.PI * radiusM * radiusM, reynolds = rho * speedMps * 2 * radiusM / mu;
        // Creeping flow: 0.5 rho A (24/Re + 3/16) u = 6 pi mu R + (3/32) rho A u, which stays finite at u = 0.
        if (reynolds < 0.01) { return 6 * Math.PI * mu * radiusM + 3.0 / 32 * rho * area * speedMps; }
        return 0.5 * rho * area * coefficient(reynolds) * speedMps;
    }
}
