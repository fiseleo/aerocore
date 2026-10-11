package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** A point mass at the part origin (ballast, cargo, a battery): mass only, no loads of its own. */
public record BallastDesign(String name, double massKg) implements PartDesign {
    public static final String TYPE = "aerocore:ballast";

    public BallastDesign { Objects.requireNonNull(name, "name"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!(massKg > 0) || !Double.isFinite(massKg)) { issues.add(DesignIssue.invalid("massKg", "Mass must be positive")); }
        return issues;
    }

    @Override public Part compile() {
        DesignException.throwIfInvalid(check());
        MassProperties mass = MassProperties.point(massKg, Vec3d.ZERO);
        PartDesign self = this;
        return new Part() {
            @Override public PartDesign design() { return self; }
            @Override public MassProperties mass() { return mass; }
            @Override public List<CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public void loads(PartStep step) {}
        };
    }

    @Override public Map<String, Object> parameters() { return Fields.object("massKg", massKg); }

    static PartDesign read(Fields f) { return new BallastDesign(f.string("name", "ballast"), f.number("massKg")); }
}
