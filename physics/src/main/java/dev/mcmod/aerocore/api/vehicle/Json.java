package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimal JSON for blueprints, so the physics module needs no library: objects are LinkedHashMap (key order kept),
 * arrays List, numbers Double, plus String, Boolean and null. Numbers are written with Double.toString's shortest
 * round-trip form (integral values without a fraction), so writing and reading a blueprint gives the same doubles.
 */
public final class Json {
    private Json() {}

    /** Parses a JSON text; throws IllegalArgumentException with the position of the first error. */
    public static Object parse(String text) {
        Parser p = new Parser(text);
        p.space();
        Object value = p.value(0);
        p.space();
        if (p.i != text.length()) { throw p.error("Unexpected text after the value"); }
        return value;
    }

    /** Compact JSON. */
    public static String write(Object value) {
        StringBuilder out = new StringBuilder();
        write(out, value, -1, 0);
        return out.toString();
    }

    /** Indented JSON (two spaces), for files people edit. */
    public static String pretty(Object value) {
        StringBuilder out = new StringBuilder();
        write(out, value, 2, 0);
        return out.append('\n').toString();
    }

    private static void write(StringBuilder out, Object value, int indent, int depth) {
        if (value == null) { out.append("null"); }
        else if (value instanceof Boolean b) { out.append(b); }
        else if (value instanceof Number n) {
            double d = n.doubleValue();
            if (!Double.isFinite(d)) { throw new IllegalArgumentException("JSON numbers must be finite"); }
            if (d == Math.rint(d) && Math.abs(d) < 1e15) { out.append((long) d); } else { out.append(Double.toString(d)); }
        } else if (value instanceof String s) { string(out, s); }
        else if (value instanceof Map<?, ?> map) {
            out.append('{');
            boolean first = true;
            for (Map.Entry<?, ?> e : map.entrySet()) {
                if (!first) { out.append(','); }
                first = false;
                newline(out, indent, depth + 1);
                string(out, String.valueOf(e.getKey()));
                out.append(indent >= 0 ? ": " : ":");
                write(out, e.getValue(), indent, depth + 1);
            }
            if (!map.isEmpty()) { newline(out, indent, depth); }
            out.append('}');
        } else if (value instanceof List<?> list) {
            out.append('[');
            boolean scalar = list.stream().noneMatch(v -> v instanceof Map || v instanceof List);
            for (int k = 0; k < list.size(); k++) {
                if (k > 0) { out.append(scalar && indent >= 0 ? ", " : ","); }
                if (!scalar) { newline(out, indent, depth + 1); }
                write(out, list.get(k), indent, depth + 1);
            }
            if (!scalar && !list.isEmpty()) { newline(out, indent, depth); }
            out.append(']');
        } else { throw new IllegalArgumentException("Not a JSON value: " + value.getClass().getName()); }
    }

    private static void newline(StringBuilder out, int indent, int depth) {
        if (indent < 0) { return; }
        out.append('\n');
        out.append(" ".repeat(indent * depth));
    }

    private static void string(StringBuilder out, String s) {
        out.append('"');
        for (int k = 0; k < s.length(); k++) {
            char c = s.charAt(k);
            switch (c) {
                case '"' -> out.append("\\\"");
                case '\\' -> out.append("\\\\");
                case '\n' -> out.append("\\n");
                case '\r' -> out.append("\\r");
                case '\t' -> out.append("\\t");
                default -> {
                    if (c < 0x20) { out.append(String.format("\\u%04x", (int) c)); } else { out.append(c); }
                }
            }
        }
        out.append('"');
    }

    private static final class Parser {
        /** Nesting limit: blueprints are shallow; deep input is refused rather than overflowing the stack. */
        private static final int MAX_DEPTH = 64;
        final String s;
        int i;
        Parser(String s) { this.s = s; }

        IllegalArgumentException error(String message) { return new IllegalArgumentException(message + " at character " + i); }

        void space() { while (i < s.length() && Character.isWhitespace(s.charAt(i))) { i++; } }

        Object value(int depth) {
            if (depth > MAX_DEPTH) { throw error("JSON nested too deeply"); }
            if (i >= s.length()) { throw error("Unexpected end of JSON"); }
            char c = s.charAt(i);
            return switch (c) {
                case '{' -> object(depth);
                case '[' -> array(depth);
                case '"' -> string();
                case 't' -> literal("true", Boolean.TRUE);
                case 'f' -> literal("false", Boolean.FALSE);
                case 'n' -> literal("null", null);
                default -> number();
            };
        }

        Object literal(String word, Object value) {
            if (!s.startsWith(word, i)) { throw error("Unexpected character"); }
            i += word.length();
            return value;
        }

        Map<String, Object> object(int depth) {
            Map<String, Object> map = new LinkedHashMap<>();
            i++;
            space();
            if (i < s.length() && s.charAt(i) == '}') { i++; return map; }
            while (true) {
                space();
                if (i >= s.length() || s.charAt(i) != '"') { throw error("Expected a key"); }
                String key = string();
                space();
                if (i >= s.length() || s.charAt(i) != ':') { throw error("Expected ':'"); }
                i++;
                space();
                if (map.containsKey(key)) { throw error("Duplicate key " + key); }
                map.put(key, value(depth + 1));
                space();
                if (i >= s.length()) { throw error("Unexpected end of object"); }
                char c = s.charAt(i++);
                if (c == '}') { return map; }
                if (c != ',') { throw error("Expected ',' or '}'"); }
            }
        }

        List<Object> array(int depth) {
            List<Object> list = new ArrayList<>();
            i++;
            space();
            if (i < s.length() && s.charAt(i) == ']') { i++; return list; }
            while (true) {
                space();
                list.add(value(depth + 1));
                space();
                if (i >= s.length()) { throw error("Unexpected end of array"); }
                char c = s.charAt(i++);
                if (c == ']') { return list; }
                if (c != ',') { throw error("Expected ',' or ']'"); }
            }
        }

        String string() {
            StringBuilder out = new StringBuilder();
            i++;
            while (true) {
                if (i >= s.length()) { throw error("Unterminated string"); }
                char c = s.charAt(i++);
                if (c == '"') { return out.toString(); }
                if (c != '\\') { out.append(c); continue; }
                if (i >= s.length()) { throw error("Unterminated escape"); }
                char e = s.charAt(i++);
                switch (e) {
                    case '"', '\\', '/' -> out.append(e);
                    case 'b' -> out.append('\b');
                    case 'f' -> out.append('\f');
                    case 'n' -> out.append('\n');
                    case 'r' -> out.append('\r');
                    case 't' -> out.append('\t');
                    case 'u' -> {
                        if (i + 4 > s.length()) { throw error("Bad unicode escape"); }
                        out.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
                        i += 4;
                    }
                    default -> throw error("Bad escape");
                }
            }
        }

        Double number() {
            int start = i;
            while (i < s.length() && "+-0123456789.eE".indexOf(s.charAt(i)) >= 0) { i++; }
            if (start == i) { throw error("Unexpected character"); }
            try {
                double d = Double.parseDouble(s.substring(start, i));
                if (!Double.isFinite(d)) { throw error("Number out of range"); }
                return d;
            } catch (NumberFormatException e) {
                throw error("Bad number");
            }
        }
    }
}
