package dev.mcmod.aerocore.api.engine;

/**
 * Exact slider-crank kinematics of one cylinder (no pin offset): bore B, stroke S (crank radius a = S/2), connecting
 * rod length l centre to centre, clearance volume V_c. Crank angle theta in radians from top dead centre. Volume
 * V = V_c + (pi B^2 / 4)(l + a - s), s = a cos(theta) + sqrt(l^2 - a^2 sin^2(theta)) (Heywood 1988, eq. 2.4-2.6).
 */
public record CrankKinematics(double boreM, double strokeM, double rodM, double clearanceM3) {
    public CrankKinematics {
        if (!(boreM > 0) || !(strokeM > 0) || !(clearanceM3 > 0)) { throw new IllegalArgumentException("Bore, stroke and clearance must be positive"); }
        if (!(rodM > strokeM / 2)) { throw new IllegalArgumentException("Rod must be longer than the crank radius"); }
    }

    public double crankRadius() { return strokeM / 2; }
    public double pistonArea() { return Math.PI * boreM * boreM / 4; }
    public double displacement() { return pistonArea() * strokeM; }
    public double compressionRatio() { return (displacement() + clearanceM3) / clearanceM3; }

    /** Distance from the crank axis to the piston pin, m. */
    public double pinHeight(double theta) {
        double a = crankRadius(), sin = Math.sin(theta);
        return a * Math.cos(theta) + Math.sqrt(rodM * rodM - a * a * sin * sin);
    }
    /** Cylinder volume, m^3. */
    public double volume(double theta) { return clearanceM3 + pistonArea() * (rodM + crankRadius() - pinHeight(theta)); }
    /** dV/dtheta, m^3/rad. */
    public double volumeRate(double theta) {
        double a = crankRadius(), sin = Math.sin(theta), cos = Math.cos(theta);
        return pistonArea() * a * sin * (1 + a * cos / Math.sqrt(rodM * rodM - a * a * sin * sin));
    }
    /** Cylinder liner area exposed above the piston (crown at the deck at TDC), m^2. */
    public double exposedLiner(double theta) { return Math.PI * boreM * (rodM + crankRadius() - pinHeight(theta)); }

    /** Piston acceleration along the bore (positive away from the crank) per unit crank speed squared, m/rad^2. */
    public double accelerationPerOmegaSquared(double theta) {
        double a = crankRadius(), sin = Math.sin(theta), cos = Math.cos(theta);
        double root = Math.sqrt(rodM * rodM - a * a * sin * sin);
        double cos2 = cos * cos - sin * sin;
        return -a * cos - a * a * cos2 / root - Math.pow(a, 4) * sin * sin * cos * cos / (root * root * root);
    }
}
