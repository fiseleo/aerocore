package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Isotropic body drag with a fixed reference area (m^2); offset is world-space metres from COM. */
public record DragBody(Vec3d offset, double areaM2, double dragCoefficient) {
    public DragBody {
        Objects.requireNonNull(offset, "offset");
        Checks.positive(areaM2, "Body reference area");
        Checks.nonnegative(dragCoefficient, "Body drag coefficient");
    }
}

