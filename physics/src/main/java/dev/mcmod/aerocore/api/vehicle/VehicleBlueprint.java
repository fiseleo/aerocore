package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * A vehicle as a player designs it: named part designs installed at mounts with their control bindings, and the
 * connections between them. Plain data (JSON round trip); compile() checks it and builds the Vehicle.
 */
public record VehicleBlueprint(String name, List<Entry> parts, List<Connection> connections) {
    /** One installed part: an id unique in the blueprint, its design, where it sits and what drives its inputs. */
    public record Entry(String id, PartDesign design, Mount mount, List<Binding> bindings) {
        public Entry {
            Objects.requireNonNull(id, "id"); Objects.requireNonNull(design, "design"); Objects.requireNonNull(mount, "mount");
            bindings = List.copyOf(bindings);
        }
        public Entry(String id, PartDesign design, Mount mount) { this(id, design, mount, List.of()); }
    }

    public VehicleBlueprint {
        Objects.requireNonNull(name, "name");
        parts = List.copyOf(parts);
        connections = List.copyOf(connections);
    }

    /** INVALID and EXTRAPOLATION issues of the parts (under "parts.<id>") and of the assembly (ids, connections). */
    public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        Set<String> ids = new HashSet<>();
        if (parts.isEmpty()) { issues.add(DesignIssue.invalid("parts", "A vehicle needs at least one part")); }
        for (Entry e : parts) {
            if (!e.id().matches("[A-Za-z0-9_.-]{1,64}")) { issues.add(DesignIssue.invalid("parts." + e.id(), "Part ids are 1 to 64 letters, digits, _ . -")); }
            if (!ids.add(e.id())) { issues.add(DesignIssue.invalid("parts." + e.id(), "Duplicate part id")); }
            for (DesignIssue i : e.design().check()) { issues.add(i.under("parts." + e.id())); }
        }
        for (int c = 0; c < connections.size(); c++) {
            String path = "connections[" + c + "]";
            Connection connection = connections.get(c);
            if (connection instanceof Connection.Shaft s) {
                if (!ids.contains(s.from())) { issues.add(DesignIssue.invalid(path + ".from", "No part " + s.from())); }
                if (!ids.contains(s.to())) { issues.add(DesignIssue.invalid(path + ".to", "No part " + s.to())); }
                if (s.from().equals(s.to())) { issues.add(DesignIssue.invalid(path, "A shaft joins two different parts")); }
                if (!Double.isFinite(s.ratio()) || s.ratio() == 0) { issues.add(DesignIssue.invalid(path + ".ratio", "Gear ratio must be finite and not zero")); }
                if (!(s.efficiency() > 0 && s.efficiency() <= 1)) { issues.add(DesignIssue.invalid(path + ".efficiency", "Efficiency must be in (0, 1]")); }
            } else if (connection instanceof Connection.Slipstream s) {
                if (!ids.contains(s.propeller())) { issues.add(DesignIssue.invalid(path + ".propeller", "No part " + s.propeller())); }
                if (!ids.contains(s.surface())) { issues.add(DesignIssue.invalid(path + ".surface", "No part " + s.surface())); }
            }
        }
        return issues;
    }

    /** The compiled vehicle; throws DesignException when check() or the assembly finds INVALID issues. */
    public Vehicle compile() { return Vehicle.compile(this); }

    public Entry part(String id) {
        for (Entry e : parts) { if (e.id().equals(id)) { return e; } }
        return null;
    }

    public Map<String, Object> toJson() {
        List<Object> list = new ArrayList<>();
        for (Entry e : parts) {
            Map<String, Object> m = Fields.object("id", e.id(), "design", e.design().toJson());
            m.putAll(e.mount().toJson());
            if (!e.bindings().isEmpty()) {
                List<Object> b = new ArrayList<>();
                for (Binding binding : e.bindings()) { b.add(binding.toJson()); }
                m.put("bindings", b);
            }
            list.add(m);
        }
        List<Object> links = new ArrayList<>();
        for (Connection c : connections) { links.add(c.toJson()); }
        return Fields.object("format", PartDesign.FORMAT, "name", name, "parts", list, "connections", links);
    }

    public String toJsonText() { return Json.pretty(toJson()); }

    public static VehicleBlueprint fromJson(Object json) {
        Fields f = Fields.of(json, "vehicle");
        int format = f.integer("format", PartDesign.FORMAT);
        if (format > PartDesign.FORMAT) { throw new IllegalArgumentException("vehicle: made by a newer AeroCore (format " + format + ")"); }
        List<Entry> parts = new ArrayList<>();
        for (Fields p : f.objects("parts")) {
            List<Binding> bindings = new ArrayList<>();
            if (p.has("bindings")) { for (Fields b : p.objects("bindings")) { bindings.add(Binding.fromJson(b)); } }
            parts.add(new Entry(p.string("id"), PartTypes.read(p.raw("design"), p.path() + ".design"), Mount.fromJson(p), bindings));
        }
        List<Connection> connections = new ArrayList<>();
        if (f.has("connections")) { for (Fields c : f.objects("connections")) { connections.add(Connection.fromJson(c)); } }
        return new VehicleBlueprint(f.string("name", "vehicle"), parts, connections);
    }

    public static VehicleBlueprint fromJsonText(String text) { return fromJson(Json.parse(text)); }
}
