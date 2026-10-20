package dev.mcmod.aerocore.api.power;

/**
 * A storage battery (0.26): its energy (J), the efficiencies of charging and of discharging and the largest rate (a
 * C-rate: the share of the capacity per hour). Presets per kWh of capacity:
 * <ul>
 * <li>Nickel-iron (Edison cell): about 25 Wh/kg, 65-75 % round trip, charged at C/5 or slower (Linden and Reddy,
 *     Handbook of Batteries, 4th ed., ch. 25).</li>
 * <li>Lead-acid (flooded): about 35 Wh/kg, 80-85 % round trip, C/5 (ibid., ch. 16).</li>
 * </ul>
 */
public record Battery(double capacityJ, double chargeEfficiency, double dischargeEfficiency, double maxCRatePerHour, double massKg) {
    public Battery {
        if (!(capacityJ > 0) || !(chargeEfficiency > 0 && chargeEfficiency <= 1) || !(dischargeEfficiency > 0 && dischargeEfficiency <= 1)
                || !(maxCRatePerHour > 0) || !(massKg > 0)) {
            throw new IllegalArgumentException("Battery: capacity, rate and mass positive, efficiencies in (0, 1]");
        }
    }

    public static final double J_PER_KWH = 3.6e6;

    /** A nickel-iron battery of a capacity (kWh): 0.84 each way (70 % round trip), C/5, 25 Wh/kg. */
    public static Battery nickelIron(double kWh) { return new Battery(kWh * J_PER_KWH, 0.84, 0.84, 0.2, kWh * 1000 / 25); }

    /** A flooded lead-acid battery of a capacity (kWh): 0.91 each way (83 % round trip), C/5, 35 Wh/kg. */
    public static Battery leadAcid(double kWh) { return new Battery(kWh * J_PER_KWH, 0.91, 0.91, 0.2, kWh * 1000 / 35); }

    /** The largest power in or out, W. */
    public double maxPowerW() { return maxCRatePerHour * capacityJ / 3600; }

    /** Energy delivered back for energy put in, both ways. */
    public double roundTrip() { return chargeEfficiency * dischargeEfficiency; }
}
