package dev.mcmod.aerocore.api.frame;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * A frame of straight beams (0.33) for FrameAnalysis: nodes at positions (m; x forward, y up, z to starboard, the
 * vehicle's body axes) and members between them, rigidly joined at the nodes. Each member has a section, a material, an
 * up vector that sets its section's y axis (up's part across the member; x runs from node a to node b, z = x × y) and a
 * joint factor: how far its ends hold the section's warping (WELDED 1, RIVETED 0.7, BOLTED 0.5, UNRESTRAINED 0), which
 * only an open section feels (torsionNm2).
 * <p>A builder: add nodes and members, then analyse; an analysis reads the frame as it is when called. An open member's
 * warping is held at its ends, so split an open member into several only where something joins it.
 */
public final class Frame {
    public static final double WELDED = 1, RIVETED = 0.7, BOLTED = 0.5, UNRESTRAINED = 0;

    /** A member from node a to node b. */
    public record Member(int a, int b, Section section, FrameMaterial material, Vec3d up, double jointFactor) {}

    private final List<Vec3d> nodes = new ArrayList<>();
    private final List<Member> members = new ArrayList<>();

    /** Adds a node; returns its index. */
    public int node(Vec3d position) {
        nodes.add(Objects.requireNonNull(position, "position"));
        return nodes.size() - 1;
    }

    /** Adds a member between two nodes; returns its index. */
    public int member(int a, int b, Section section, FrameMaterial material, Vec3d up, double jointFactor) {
        checkNode(a); checkNode(b);
        if (a == b) { throw new IllegalArgumentException("A member joins two nodes"); }
        Objects.requireNonNull(section, "section"); Objects.requireNonNull(material, "material"); Objects.requireNonNull(up, "up");
        if (!(jointFactor >= 0 && jointFactor <= 1)) { throw new IllegalArgumentException("The joint factor is 0..1"); }
        Vec3d along = nodes.get(b).subtract(nodes.get(a));
        double length = along.length();
        if (!(length > 1e-6)) { throw new IllegalArgumentException("Nodes " + a + " and " + b + " are at the same place"); }
        Vec3d x = along.scale(1 / length);
        Vec3d across = up.subtract(x.scale(up.dot(x)));
        if (!(across.length() > 1e-6 * up.length())) { throw new IllegalArgumentException("A member's up vector must not lie along it"); }
        members.add(new Member(a, b, section, material, up, jointFactor));
        return members.size() - 1;
    }

    public int nodeCount() { return nodes.size(); }
    public int memberCount() { return members.size(); }
    public Vec3d position(int node) { checkNode(node); return nodes.get(node); }
    public Member member(int index) { return members.get(index); }
    public List<Vec3d> nodes() { return Collections.unmodifiableList(nodes); }
    public List<Member> members() { return Collections.unmodifiableList(members); }

    /** A member's length (m). */
    public double lengthM(int member) {
        Member m = members.get(member);
        return nodes.get(m.b()).subtract(nodes.get(m.a())).length();
    }

    /** A member's axes in the frame's axes, unit vectors: x along a → b, y up's part across it, z = x × y. */
    public Vec3d[] axes(int member) {
        Member m = members.get(member);
        Vec3d x = nodes.get(m.b()).subtract(nodes.get(m.a())).normalized();
        Vec3d y = m.up().subtract(x.scale(m.up().dot(x))).normalized();
        return new Vec3d[]{x, y, x.cross(y)};
    }

    /**
     * A member's torsional stiffness (N m^2): G J raised toward its warping-restrained value (Section.restrainedTorsionNm2)
     * by the joint factor, G J + f (G J_eff - G J). G J for a closed section.
     */
    public double torsionNm2(int member) {
        Member m = members.get(member);
        double gj = m.material().shearModulusPa() * m.section().torsionM4();
        return gj + m.jointFactor() * (m.section().restrainedTorsionNm2(m.material(), lengthM(member)) - gj);
    }

    private void checkNode(int node) {
        if (node < 0 || node >= nodes.size()) { throw new IllegalArgumentException("No node " + node); }
    }
}
