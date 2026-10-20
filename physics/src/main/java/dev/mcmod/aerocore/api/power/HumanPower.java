package dev.mcmod.aerocore.api.power;

/**
 * What a person can deliver (0.26): a healthy adult keeps up about 75 W with the legs (pedalling, a treadle) for hours
 * and about 50 W cranking with the arms; much more only for minutes (Wilson, Bicycling Science, 3rd ed., fig. 2.4).
 */
public final class HumanPower {
    private HumanPower() {}

    public static final double LEGS_W = 75, ARMS_W = 50;
}
