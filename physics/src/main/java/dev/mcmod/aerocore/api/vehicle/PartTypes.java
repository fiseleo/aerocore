package dev.mcmod.aerocore.api.vehicle;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Readers of part designs by type id. AeroCore registers its own ("aerocore:float" and so on); a mod adds its part
 * types with register() before reading blueprints that use them (ids namespaced by the mod, "modid:part").
 * Compiled parts are cached by their design's JSON, so the same design (the same hull on every boat of a class, on the
 * server and each client) is compiled once.
 */
public final class PartTypes {
    private static final Map<String, Function<Fields, PartDesign>> READERS = new LinkedHashMap<>();
    private static final int CACHE = 64;
    private static final Map<String, Part> COMPILED = java.util.Collections.synchronizedMap(new LinkedHashMap<>(16, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<String, Part> eldest) { return size() > CACHE; }
    });

    static {
        register(FloatDesign.TYPE, FloatDesign::read);
        register(HullDesign.TYPE, HullDesign::read);
        register(PlaningHullDesign.TYPE, PlaningHullDesign::read);
        register(PropellerDesign.TYPE, PropellerDesign::read);
        register(ControlSurfaceDesign.TYPE, ControlSurfaceDesign::read);
        register(BoxDesign.TYPE, BoxDesign::read);
        register(MotorDesign.TYPE, MotorDesign::read);
        register(BallastDesign.TYPE, BallastDesign::read);
        register(SeatDesign.TYPE, SeatDesign::read);
    }

    private PartTypes() {}

    public static synchronized void register(String type, Function<Fields, PartDesign> reader) {
        if (!type.matches("[a-z0-9_.-]+:[a-z0-9_./-]+")) { throw new IllegalArgumentException("Part type ids are namespace:path in lower case: " + type); }
        if (READERS.containsKey(type)) { throw new IllegalArgumentException("Part type already registered: " + type); }
        READERS.put(type, reader);
    }

    public static synchronized List<String> types() { return List.copyOf(READERS.keySet()); }

    /** A design from its JSON object (as PartDesign.toJson writes it). */
    public static PartDesign read(Object json, String path) {
        Fields f = Fields.of(json, path);
        int format = f.integer("format", PartDesign.FORMAT);
        if (format > PartDesign.FORMAT) { throw new IllegalArgumentException(path + ": made by a newer AeroCore (format " + format + ")"); }
        String type = f.string("type");
        Function<Fields, PartDesign> reader;
        synchronized (PartTypes.class) { reader = READERS.get(type); }
        if (reader == null) { throw new IllegalArgumentException(path + ".type: unknown part type " + type); }
        return reader.apply(f);
    }

    /** design.compile(), cached by the design's JSON. */
    public static Part compile(PartDesign design) {
        String key = Json.write(design.toJson());
        Part part = COMPILED.get(key);
        if (part == null) {
            part = design.compile();
            COMPILED.put(key, part);
        }
        return part;
    }
}
