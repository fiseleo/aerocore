package dev.mcmod.aerocore.api.power;

/**
 * Solid fuels for boilers and furnaces (0.26): lower heating values as fired (typical values: Perry's Chemical
 * Engineers' Handbook, 8th ed., section 24; air-dried wood at about 20 % moisture).
 */
public record SolidFuel(String name, double heatingValueJPerKg) {
    public static final SolidFuel COAL = new SolidFuel("coal", 27e6);
    public static final SolidFuel CHARCOAL = new SolidFuel("charcoal", 29e6);
    public static final SolidFuel COKE = new SolidFuel("coke", 28e6);
    public static final SolidFuel WOOD = new SolidFuel("wood", 15e6);
}
