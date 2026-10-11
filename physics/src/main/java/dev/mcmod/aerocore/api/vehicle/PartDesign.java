package dev.mcmod.aerocore.api.vehicle;

import java.util.List;
import java.util.Map;

/**
 * A part blueprint a player makes: a type, a name and parameters only (shapes included), with no game objects, so it
 * can be saved, sent and shared as JSON. check() lists INVALID values (the design will not compile) and values outside
 * the ranges the models were validated over (EXTRAPOLATION); compile() turns a valid design into a Part.
 * New part types implement this and register a reader with PartTypes.
 */
public interface PartDesign {
    /** Blueprint format version written by this AeroCore; readers accept this and older. */
    int FORMAT = 1;

    /** Registered type id, for example "aerocore:float". */
    String type();
    /** The maker's name for this design. */
    String name();
    /** INVALID and EXTRAPOLATION issues of the parameters, paths relative to this design. */
    List<DesignIssue> check();
    /** The compiled part; throws DesignException when check() has INVALID issues. */
    Part compile();
    /** The type's own parameters (no "type", "name" or "format"). */
    Map<String, Object> parameters();

    default Map<String, Object> toJson() {
        Map<String, Object> m = Fields.object("format", FORMAT, "type", type(), "name", name());
        m.putAll(parameters());
        return m;
    }
}
