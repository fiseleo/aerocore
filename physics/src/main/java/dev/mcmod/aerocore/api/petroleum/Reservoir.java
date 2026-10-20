package dev.mcmod.aerocore.api.petroleum;

/**
 * An oil reservoir and the wells into it (0.26), in SI units (permeability m^2, viscosity Pa s, pressure Pa, rates m^3/s
 * of stock-tank oil).
 * <ul>
 * <li>Oil in place: area x net pay x porosity x oil saturation / formation volume factor.</li>
 * <li>A well's productivity index (pseudo-steady radial flow, Darcy): J = 2 pi k h / (mu B (ln(r_e / r_w) - 0.75)).</li>
 * <li>Inflow below the bubble point (Vogel, J. Pet. Tech. 1968): q / q_max = 1 - 0.2 x - 0.8 x^2, x = p_wf / p_r,
 *     q_max = J p_r / 1.8.</li>
 * <li>Depletion: the average pressure falls linearly from the initial to the abandonment pressure as the produced oil
 *     reaches the recoverable share (recovery factor) of the oil in place (a tank model).</li>
 * <li>Natural flow: the well flows to the surface while the inflow at the bottom-hole pressure that lifts the oil
 *     (rho g H plus the wellhead's) is positive; otherwise it needs a pump.</li>
 * <li>A wellhead choke (0.27): an orifice the oil passes at the rate q = Cd A sqrt(2 dp / rho) (Cd 0.85, the oil
 *     leaving at atmospheric pressure), the bottom-hole pressure the column's weight plus the choke's drop, solved with
 *     the inflow. Single-phase liquid: real chokes pass oil with its gas and flow critical (Gilbert's correlation), so
 *     this reads high for gassy wells.</li>
 * </ul>
 * (Craft and Hawkins, Applied Petroleum Reservoir Engineering; Economides et al., Petroleum Production Systems.)
 */
public record Reservoir(double areaM2, double payM, double porosity, double oilSaturation, double formationFactor, double permeabilityM2,
                        double viscosityPaS, double initialPressurePa, double abandonmentPressurePa, double recoveryFactor) {
    public Reservoir {
        if (!(areaM2 > 0) || !(payM > 0) || !(porosity > 0 && porosity < 1) || !(oilSaturation > 0 && oilSaturation <= 1) || !(formationFactor >= 1)
                || !(permeabilityM2 > 0) || !(viscosityPaS > 0) || !(initialPressurePa > abandonmentPressurePa) || !(abandonmentPressurePa >= 0)
                || !(recoveryFactor > 0 && recoveryFactor < 1)) {
            throw new IllegalArgumentException("Reservoir: positive sizes, fractions in range, initial above abandonment pressure");
        }
    }

    public static final double MILLIDARCY_M2 = 9.869233e-16;

    /** Stock-tank oil in place, m^3. */
    public double oilInPlaceM3() { return areaM2 * payM * porosity * oilSaturation / formationFactor; }

    /** Oil that can be produced, m^3. */
    public double recoverableM3() { return recoveryFactor * oilInPlaceM3(); }

    /** Average reservoir pressure after producing a volume (m^3), Pa. */
    public double pressurePa(double producedM3) {
        double share = Math.clamp(producedM3 / recoverableM3(), 0, 1);
        return initialPressurePa - share * (initialPressurePa - abandonmentPressurePa);
    }

    /** A well's productivity index (m^3/s per Pa) into a drainage radius from a wellbore radius (m). */
    public double productivityIndex(double drainageRadiusM, double wellRadiusM) {
        return 2 * Math.PI * permeabilityM2 * payM / (viscosityPaS * formationFactor * (Math.log(drainageRadiusM / wellRadiusM) - 0.75));
    }

    /** Vogel's inflow: the rate at a bottom-hole pressure for a reservoir pressure, m^3/s. */
    public static double vogelRate(double productivityIndex, double reservoirPa, double bottomHolePa) {
        if (bottomHolePa >= reservoirPa || reservoirPa <= 0) { return 0; }
        double x = Math.max(bottomHolePa, 0) / reservoirPa, qMax = productivityIndex * reservoirPa / 1.8;
        return qMax * (1 - 0.2 * x - 0.8 * x * x);
    }

    /** A choke's discharge coefficient (0.27). */
    public static final double CHOKE_DISCHARGE = 0.85;

    /**
     * The rate a well flows by itself through a wellhead choke of a bore (m; 0 or less: none) from a depth (m) after
     * producing a volume, m^3/s (0.27; no choke: naturalRate against no wellhead pressure).
     */
    public double chokedRate(double productivityIndex, double depthM, double oilDensity, double chokeBoreM, double producedM3) {
        double open = naturalRate(productivityIndex, depthM, oilDensity, 0, producedM3);
        if (!(chokeBoreM > 0) || !(open > 0)) { return open; }
        double column = oilDensity * 9.81 * depthM, reservoir = pressurePa(producedM3);
        double area = Math.PI / 4 * chokeBoreM * chokeBoreM;
        // the bottom-hole pressure where the inflow meets the choke's flow: the inflow falls and the choke's rises with it
        double lo = column, hi = reservoir;
        for (int i = 0; i < 100; i++) {
            double p = 0.5 * (lo + hi);
            double inflow = vogelRate(productivityIndex, reservoir, p);
            double through = CHOKE_DISCHARGE * area * Math.sqrt(2 * Math.max(p - column, 0) / oilDensity);
            if (inflow > through) { lo = p; } else { hi = p; }
        }
        return vogelRate(productivityIndex, reservoir, 0.5 * (lo + hi));
    }

    /** The rate a well flows by itself from a depth (m) against a wellhead pressure (Pa) after producing a volume, m^3/s (0: it needs a pump). */
    public double naturalRate(double productivityIndex, double depthM, double oilDensity, double wellheadPa, double producedM3) {
        double lift = oilDensity * 9.81 * depthM + wellheadPa;
        return vogelRate(productivityIndex, pressurePa(producedM3), lift);
    }
}
