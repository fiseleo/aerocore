package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/** Mass with no loads of its own (ballast, cargo, a battery), optionally with rider seats (where a rider's feet rest), in the part frame. */
public final class MassPart implements Part {
    private final PartDesign design;
    private final MassProperties mass;
    private final List<Vec3d> seats;

    public MassPart(PartDesign design, MassProperties mass, List<Vec3d> seats) {
        this.design = Objects.requireNonNull(design, "design");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.seats = List.copyOf(seats);
    }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return List.of(); }
    @Override public List<Vec3d> seats() { return seats; }
    @Override public void loads(PartStep step) {}
}
