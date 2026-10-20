package dev.mcmod.aerocore.api.body;

import java.util.Objects;

/**
 * One length of a body: from the section where the module ahead of it ends (or the body's nose) back to its own rear
 * section, over its length. Between the two the sections blend (a Minkowski combination, so they stay convex) along a
 * superellipse profile measured from the larger end, of exponent fullness: 1 is a straight taper, 2 exactly elliptical
 * (a round nose or a full tail), 0.5 hollow (a sharp point), larger fuller still. step: the bottom rises by this much where the module starts (a planing hull's
 * step, the afterbody's keel above the forebody's). sill: the height above the body's reference line from which the
 * module is open to the water (an open cockpit or boat); infinite when it is closed all round.
 */
public record BodyModule(String name, double lengthM, SectionShape rear, double fullness, double stepM, double sillM) {
    public BodyModule {
        Objects.requireNonNull(name, "name");
        Require.positive(lengthM, "Module length");
        Objects.requireNonNull(rear, "rear");
        if (!(fullness >= 0.25 && fullness <= 4)) { throw new IllegalArgumentException("Fullness is 0.25..4"); }
        Require.nonnegative(stepM, "Step");
        if (Double.isNaN(sillM)) { throw new IllegalArgumentException("Sill must be a height or infinite"); }
    }

    /** A closed module without a step, tapering straight. */
    public static BodyModule straight(String name, double lengthM, SectionShape rear) {
        return new BodyModule(name, lengthM, rear, 1, 0, Double.POSITIVE_INFINITY);
    }

    public boolean open() { return Double.isFinite(sillM); }
}
