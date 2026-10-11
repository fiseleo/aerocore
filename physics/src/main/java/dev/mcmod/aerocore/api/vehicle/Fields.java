package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A JSON object read with its path, so a malformed blueprint names the field ("parts[2].design.radiusM: expected a
 * number"). Shape errors (missing field, wrong type) are IllegalArgumentException; values are judged later by the
 * designs' check(), so an unphysical number still reads and is reported as a DesignIssue.
 */
public final class Fields {
    private final Map<String, Object> map;
    private final String path;

    private Fields(Map<String, Object> map, String path) { this.map = map; this.path = path; }

    @SuppressWarnings("unchecked")
    public static Fields of(Object value, String path) {
        if (!(value instanceof Map<?, ?> m)) { throw new IllegalArgumentException(path + ": expected an object"); }
        return new Fields((Map<String, Object>) m, path);
    }

    /** An ordered object from key, value pairs. */
    public static Map<String, Object> object(Object... keysAndValues) {
        Map<String, Object> out = new LinkedHashMap<>();
        for (int i = 0; i < keysAndValues.length; i += 2) { out.put((String) keysAndValues[i], keysAndValues[i + 1]); }
        return out;
    }

    public static List<Object> vector(Vec3d v) { return List.of(v.x(), v.y(), v.z()); }

    public String path() { return path; }
    public boolean has(String key) { return map.containsKey(key) && map.get(key) != null; }
    public Object raw(String key) { return map.get(key); }
    public Map<String, Object> map() { return map; }

    public double number(String key) {
        Object v = map.get(key);
        if (!(v instanceof Number n)) { throw new IllegalArgumentException(path + "." + key + ": expected a number"); }
        return n.doubleValue();
    }
    public double number(String key, double fallback) { return has(key) ? number(key) : fallback; }

    public int integer(String key) {
        double d = number(key);
        if (d != Math.rint(d) || Math.abs(d) > Integer.MAX_VALUE) { throw new IllegalArgumentException(path + "." + key + ": expected a whole number"); }
        return (int) d;
    }
    public int integer(String key, int fallback) { return has(key) ? integer(key) : fallback; }

    public String string(String key) {
        Object v = map.get(key);
        if (!(v instanceof String s)) { throw new IllegalArgumentException(path + "." + key + ": expected a string"); }
        return s;
    }
    public String string(String key, String fallback) { return has(key) ? string(key) : fallback; }

    public boolean bool(String key, boolean fallback) {
        if (!has(key)) { return fallback; }
        if (!(map.get(key) instanceof Boolean b)) { throw new IllegalArgumentException(path + "." + key + ": expected true or false"); }
        return b;
    }

    public Vec3d vec(String key) { return vec(map.get(key), path + "." + key); }
    public Vec3d vec(String key, Vec3d fallback) { return has(key) ? vec(key) : fallback; }

    static Vec3d vec(Object v, String where) {
        if (!(v instanceof List<?> l) || l.size() != 3 || !(l.get(0) instanceof Number a) || !(l.get(1) instanceof Number b) || !(l.get(2) instanceof Number c)) {
            throw new IllegalArgumentException(where + ": expected [x, y, z]");
        }
        return new Vec3d(a.doubleValue(), b.doubleValue(), c.doubleValue());
    }

    public Fields child(String key) { return of(map.get(key), path + "." + key); }

    public List<Object> list(String key) {
        Object v = map.get(key);
        if (!(v instanceof List<?> l)) { throw new IllegalArgumentException(path + "." + key + ": expected a list"); }
        return new ArrayList<>(l);
    }
    public List<Object> list(String key, List<Object> fallback) { return has(key) ? list(key) : fallback; }

    public List<Fields> objects(String key) {
        List<Fields> out = new ArrayList<>();
        List<Object> l = list(key);
        for (int i = 0; i < l.size(); i++) { out.add(of(l.get(i), path + "." + key + "[" + i + "]")); }
        return out;
    }
}
