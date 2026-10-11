package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.Fuel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Fuel supply (供油系統): the fuel (gasoline of given research and motor octane numbers, methanol, propane or
 * isooctane), how it is delivered (carburettor, port injection or direct injection), injectors per cylinder, fuel
 * pressure (above the manifold for port injection, absolute for direct), and the relative air-fuel ratio the system
 * targets at full load and at part load (rich at full load cools the charge and resists knock; lambda 1 at part load).
 */
public record FuelSystemDesign(String name, String fuel, double researchOctane, double motorOctane, Delivery delivery,
                               int injectorsPerCylinder, double fuelPressurePa, double fullLoadLambda, double partLoadLambda,
                               double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:fuel_system";

    public enum Delivery { CARBURETTOR, PORT_INJECTION, DIRECT_INJECTION }

    public FuelSystemDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(fuel, "fuel"); Objects.requireNonNull(delivery, "delivery"); }

    @Override public String type() { return TYPE; }

    /** The Fuel this system burns. */
    public Fuel fuelModel() {
        return switch (fuel) {
            case "gasoline" -> Fuel.gasoline(researchOctane, motorOctane);
            case "methanol" -> Fuel.METHANOL;
            case "propane" -> Fuel.PROPANE;
            case "isooctane" -> Fuel.ISOOCTANE;
            default -> throw new IllegalArgumentException("Unknown fuel " + fuel);
        };
    }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!List.of("gasoline", "methanol", "propane", "isooctane").contains(fuel)) { issues.add(DesignIssue.invalid("fuel", "Fuel must be gasoline, methanol, propane or isooctane")); }
        if (fuel.equals("gasoline")) {
            EngineComponent.Rules.positive(issues, "researchOctane", researchOctane);
            EngineComponent.Rules.positive(issues, "motorOctane", motorOctane);
            if (motorOctane > researchOctane) { issues.add(DesignIssue.invalid("motorOctane", "A fuel's motor octane number is below its research octane number")); }
            DesignIssue.range(issues, "researchOctane", researchOctane, 80, 102, "pump and racing gasolines");
        }
        if (delivery != Delivery.CARBURETTOR) { EngineComponent.Rules.count(issues, "injectorsPerCylinder", injectorsPerCylinder, 1, 3); }
        EngineComponent.Rules.positive(issues, "fuelPressurePa", fuelPressurePa);
        for (String k : new String[]{"fullLoadLambda", "partLoadLambda"}) {
            double l = k.startsWith("full") ? fullLoadLambda : partLoadLambda;
            if (!(l > 0.5 && l < 2)) { issues.add(DesignIssue.invalid(k, "Lambda must be between 0.5 and 2")); }
            else { DesignIssue.range(issues, k, l, 0.75, 1.3, "spark-ignition mixtures that burn reliably"); }
        }
        if (delivery == Delivery.DIRECT_INJECTION && fuelPressurePa < 3e6) { issues.add(DesignIssue.invalid("fuelPressurePa", "Direct injection needs at least 3 MPa")); }
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Pump, rail and lines, kg: given, or 3 kg (an estimate). */
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 3 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("fuel", fuel, "researchOctane", researchOctane, "motorOctane", motorOctane, "delivery", delivery.name().toLowerCase(),
                "injectorsPerCylinder", injectorsPerCylinder, "fuelPressurePa", fuelPressurePa, "fullLoadLambda", fullLoadLambda,
                "partLoadLambda", partLoadLambda, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static FuelSystemDesign read(String name, Fields f) {
        Delivery delivery = EngineComponent.Rules.choice(f, "delivery", Delivery.class, Delivery.PORT_INJECTION);
        return new FuelSystemDesign(name, f.string("fuel", "gasoline"), f.number("researchOctane", 95), f.number("motorOctane", 85), delivery,
                f.integer("injectorsPerCylinder", 1), f.number("fuelPressurePa", delivery == Delivery.DIRECT_INJECTION ? 15e6 : 3e5),
                f.number("fullLoadLambda", 0.88), f.number("partLoadLambda", 1.0), EngineComponent.Rules.massOr(f));
    }
}
