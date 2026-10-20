package dev.mcmod.aerocore.api.storage;

/**
 * Insulation of a cryogenic store (0.28): heat flux through a layer by its effective conductivity, q = k dT / t.
 * <ul>
 * <li>VACUUM_MLI: multilayer insulation in a vacuum jacket, k about 8e-5 W/(m K) (good blankets measure 1e-5 to 1e-4;
 * Barron, Cryogenic Systems, ch. 7). 30 mm round a 170 l automotive liquid-hydrogen tank leaks about 1-2 W, a few
 * percent of its fill a day, as BMW's Hydrogen 7 did.</li>
 * <li>FOAM: closed-cell polyurethane foam sprayed on the tank (as on the Shuttle's external tank), k about 0.017 W/(m K)
 * at a cryogenic mean temperature: hundreds of watts per square metre at 25 mm; fine for a rocket filled just before
 * flight, not for keeping.</li>
 * <li>NONE: a bare metal wall — frost and boiling; taken as free convection and condensation on the outside, about
 * 25 W/(m^2 K) of temperature difference.</li>
 * </ul>
 */
public enum Insulation {
    VACUUM_MLI(8e-5), FOAM(0.017), NONE(Double.NaN);

    public final double conductivityWPerMK;

    Insulation(double conductivityWPerMK) { this.conductivityWPerMK = conductivityWPerMK; }

    /** Heat flux into the store through this insulation of a thickness between ambient and the stored liquid, W/m^2. */
    public double heatFlux(double thicknessM, double ambientK, double storedK) {
        double dt = Math.max(0, ambientK - storedK);
        if (this == NONE || !(thicknessM > 0)) { return 25 * dt; }
        return conductivityWPerMK * dt / thicknessM;
    }
}
