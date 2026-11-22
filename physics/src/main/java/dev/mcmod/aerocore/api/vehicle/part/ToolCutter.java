package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.earthmoving.Cut;
import dev.mcmod.aerocore.api.earthmoving.Digging;
import dev.mcmod.aerocore.api.earthmoving.Material;
import dev.mcmod.aerocore.api.earthmoving.RockCutting;
import dev.mcmod.aerocore.api.ground.Terrain;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartStep;

/**
 * 0.43: a tool's edge in the ground for one step (Part.Tool; LinkPart's tools use it): the force it meets and what it
 * cuts, cell by cell.
 *
 * <p>Edges and teeth: the edge is sampled every SAMPLE_M or so; at each sample in a cell of material, the depth of the
 * cut is the material standing above the edge (its cell and the cells over it, as much of each as is left, at most
 * the tool's depth), and api.earthmoving.Digging gives the resistance for that slice's width at the edge's speed into
 * the cut (its velocity along the cut direction: a tool moving back or sideways cuts nothing). The resistance acts on
 * the tool against the cut direction, the material's lift along the world's up; below CREEP_MPS it is a damper of that
 * slope (taken implicitly), so a stopped tool is not pushed back. The slice's bank volume (width x depth x speed x dt)
 * goes to the cells it passed in proportion to how much of each it cut (PartStep.cut). A cell no tool may dig stops the
 * edge like a wall: a stiff damper against moving into it.
 *
 * <p>Mills (Part.Tool.Kind.MILL): the edge is the drum's axis at its lowest line; the picks cut at the rim's speed on
 * the tool's rotor "cutter" (RockCutting.rotary: power = specific energy x removal, torque on the rotor, the rim force
 * back on the machine), the drum advancing along the cut direction. Faces (FACE): a cutterhead of radius |b - a|
 * about a pushes along the cut direction; each disc's normal force at the penetration per turn (RockCutting.disc) adds
 * up to the thrust the machine must give, the rolling forces to the head's torque; it cuts every cell the face sweeps.
 *
 * <p>"full" (above 0.5) on the part says its hold is full: the edge still meets the ground but takes nothing in.
 * "pushed" (kg) is the material heaped ahead of a blade (the game's count): its weight presses the ground ahead
 * as a surcharge (its weight over the cut's width times its depth) and drags as friction.
 */
public final class ToolCutter {
    private ToolCutter() {}

    /** Spacing of the samples along an edge, m. */
    public static final double SAMPLE_M = 0.25;
    /** Below this speed into the cut the resistance is a damper (m/s). */
    public static final double CREEP_MPS = 0.05;
    /** The slowest advance a cutterhead's thrust is taken at (m/s): a boring machine's millimetres a second. */
    public static final double FACE_CREEP_MPS = 1e-4;
    /** A wall's damping per sample (N s/m): enough to stop an excavator's bucket within a step or two. */
    public static final double WALL_NS_PER_M = 2e6;

    /** The rotor speed (rad/s) a mill or a cutterhead turns at this step, and the inertia of nothing else: what the part knows. */
    public record Drive(int rotor, double radiusM) {}

    /** Works a tool in this step: forces on the part, its cuts recorded; returns the bank volume it cut (m^3). */
    public static double work(PartStep step, Part.Tool tool, Drive drive, boolean full, double pushedKg) {
        Terrain terrain = step.environment().terrain();
        if (terrain == Terrain.NONE) { return 0; }
        return switch (tool.kind()) {
            case EDGE, TOOTH -> edge(step, terrain, tool, full, pushedKg);
            case MILL -> mill(step, terrain, tool, drive, full);
            case FACE -> face(step, terrain, tool, drive, full);
        };
    }

    private static int floor(double v) { return (int) Math.floor(v); }

    /** The material standing over a point from its cell up (bank m), at most a depth, and the cells it is in with their shares of it. */
    private record Column(double depthM, int[] ys, double[] metres, Terrain.Cell cell) {}

    private static Column column(Terrain t, Vec3d p, double most) {
        int x = floor(p.x()), z = floor(p.z()), y = floor(p.y());
        Terrain.Cell first = t.cell(x, y, z);
        if (first == null) { return null; }
        int[] ys = new int[8];
        double[] metres = new double[8];
        int n = 0;
        double depth = 0, bottom = p.y();
        for (int cy = y; n < 8 && depth < most; cy++) {
            Terrain.Cell c = cy == y ? first : t.cell(x, cy, z);
            if (c == null) { break; }
            // what is left of a cell lies at its bottom (a partly dug block)
            double top = cy + c.remaining();
            double from = Math.max(bottom, cy), to = Math.min(top, cy + 1);
            double here = Math.min(Math.max(to - from, 0), most - depth);
            if (here > 0) { ys[n] = cy; metres[n] = here; n++; depth += here; }
            if (c.remaining() < 1) { break; }   // nothing stands on a partly dug cell's open top
            bottom = cy + 1;
        }
        return new Column(depth, java.util.Arrays.copyOf(ys, n), java.util.Arrays.copyOf(metres, n), first);
    }

    private static double edge(PartStep step, Terrain t, Part.Tool tool, boolean full, double pushedKg) {
        Vec3d a = step.point(tool.a()), b = step.point(tool.b()), cut = step.axis(tool.cut());
        double width = b.subtract(a).length();
        int n = Math.max(1, (int) Math.ceil(width / SAMPLE_M));
        double slice = width / n, dt = step.dt(), total = 0;
        RigidBodyState s = step.state();
        Vec3d up = Vec3d.Y;
        double surcharge = 0;
        Material pushedOn = null;
        for (int i = 0; i < n; i++) {
            Vec3d p = a.add(b.subtract(a).scale((i + 0.5) / n));
            Column col = column(t, p, tool.depthM());
            if (col == null) { continue; }
            Vec3d offset = p.subtract(s.position());
            double speed = step.velocity(step.partPoint(p)).dot(cut);
            if (!col.cell().diggable()) {
                if (speed > 0) { step.damper(offset, cut, WALL_NS_PER_M, 0); }
                continue;
            }
            if (!(col.depthM() > 1e-4) || speed <= 0) { continue; }
            Material m = col.cell().material();
            if (surcharge == 0 && pushedKg > 0) { surcharge = pushedKg * Material.G / Math.max(width * tool.depthM(), 0.1); }
            Cut c = tool.kind() == Part.Tool.Kind.TOOTH ? Digging.tooth(m, tool.rakeRad(), col.depthM(), slice, speed)
                    : Digging.edge(m, tool.rakeRad(), col.depthM(), slice, speed, surcharge);
            double resist = c.horizontalN();
            // the resistance as a damper of F / v on the speed into the cut (implicit: rock stops the edge within the
            // step instead of throwing it back); the lift no more than what would stop the machine in the step
            double creep = Math.max(speed, CREEP_MPS);
            step.damper(offset, cut, resist / creep, 0);
            double reach = step.properties().massKg() * speed / dt, weight = step.properties().massKg() * Material.G;
            double lift = c.liftN() * Math.min(1, speed / CREEP_MPS) * (resist > reach ? reach / resist : 1);
            // what the edge is lifted (or drawn in) by is explicit: at most half the machine's weight over the whole edge, so
            // a stalled edge in rock does not throw the machine up and round itself in a step
            lift = Math.clamp(lift, -0.5 * weight / n, 0.5 * weight / n);
            if (lift != 0) { step.force(new AppliedForce(AppliedForce.Kind.CONTACT, offset, up.scale(lift))); }
            pushedOn = m;
            if (full) { continue; }
            double bank = slice * col.depthM() * speed * dt;
            total += bank;
            int x = floor(p.x()), z = floor(p.z());
            for (int k = 0; k < col.ys().length; k++) {
                step.cut(tool.name(), x, col.ys()[k], z, bank * col.metres()[k] / col.depthM(), tool.hold());
            }
        }
        // the heap ahead of a blade drags on the ground it is pushed over (Blade.pushN: its weight x the material's friction)
        if (pushedKg > 0 && pushedOn != null) {
            Vec3d mid = a.add(b).scale(0.5), offset = mid.subtract(s.position());
            double speed = step.velocity(step.partPoint(mid)).dot(cut);
            double drag = dev.mcmod.aerocore.api.earthmoving.Blade.pushN(pushedOn, pushedKg / pushedOn.looseKgM3());
            if (speed >= CREEP_MPS) { step.force(new AppliedForce(AppliedForce.Kind.CONTACT, offset, cut.scale(-drag))); }
            else if (speed > 0) { step.damper(offset, cut, drag / CREEP_MPS, 0); }
            step.diagnostic(tool.name() + "PushN", drag);
        }
        step.diagnostic(tool.name() + "CutM3PerS", total / dt);
        return total;
    }

    private static double mill(PartStep step, Terrain t, Part.Tool tool, Drive drive, boolean full) {
        if (drive == null) { return 0; }
        Vec3d a = step.point(tool.a()), b = step.point(tool.b()), cut = step.axis(tool.cut());
        double width = b.subtract(a).length();
        int n = Math.max(1, (int) Math.ceil(width / SAMPLE_M));
        double slice = width / n, dt = step.dt(), omega = step.rotorSpeed(drive.rotor()), torque = 0, total = 0;
        RigidBodyState s = step.state();
        Vec3d axis = b.subtract(a).normalized();
        for (int i = 0; i < n; i++) {
            Vec3d p = a.add(b.subtract(a).scale((i + 0.5) / n));
            Column col = column(t, p, tool.depthM());
            if (col == null) { continue; }
            Vec3d offset = p.subtract(s.position());
            double advance = Math.max(step.velocity(step.partPoint(p)).dot(cut), 0);
            if (!col.cell().diggable()) {
                if (advance > 0) { step.damper(offset, cut, WALL_NS_PER_M, 0); }
                continue;
            }
            if (!(col.depthM() > 1e-4)) { continue; }
            Material m = col.cell().material();
            double se = Digging.specificEnergyPa(m, tool.rakeRad(), col.depthM(), Math.max(Math.abs(omega) * drive.radiusM(), advance));
            RockCutting.Rotary r = RockCutting.rotary(se, slice, col.depthM(), advance, drive.radiusM(), omega);
            torque += r.torqueNm();
            // the rim force on the machine: opposite the picks' motion at the drum's bottom
            Vec3d pick = axis.cross(Vec3d.Y.scale(-1)).scale(Math.signum(omega));
            if (r.rimForceN() > 0 && pick.lengthSquared() > 0) {
                step.force(new AppliedForce(AppliedForce.Kind.CONTACT, offset, pick.normalized().scale(-r.rimForceN())));
            }
            if (advance < CREEP_MPS) { step.damper(offset, cut, se * slice * col.depthM() / Math.max(Math.abs(omega) * drive.radiusM(), CREEP_MPS), 0); }
            if (full || !(advance > 0)) { continue; }
            double bank = r.removalM3PerS() * dt;
            total += bank;
            int x = floor(p.x()), z = floor(p.z());
            for (int k = 0; k < col.ys().length; k++) { step.cut(tool.name(), x, col.ys()[k], z, bank * col.metres()[k] / col.depthM(), tool.hold()); }
        }
        if (torque > 0 && omega != 0) { step.rotorTorque(drive.rotor(), -Math.signum(omega) * torque, -torque / Math.max(Math.abs(omega), 1), true); }
        step.diagnostic(tool.name() + "CutM3PerS", total / dt);
        step.diagnostic(tool.name() + "TorqueNm", torque);
        return total;
    }

    private static double face(PartStep step, Terrain t, Part.Tool tool, Drive drive, boolean full) {
        if (drive == null) { return 0; }
        Vec3d centre = step.point(tool.a()), cut = step.axis(tool.cut());
        double radius = tool.widthM(), dt = step.dt(), omega = step.rotorSpeed(drive.rotor());
        double advance = Math.max(step.velocity(tool.a()).dot(cut), 0);
        Part.Face face = tool.face();
        // the cells the face sweeps this step: those whose centre lies within the disc's radius, at most a cell ahead
        Vec3d u = Math.abs(cut.y()) < 0.9 ? cut.cross(Vec3d.Y).normalized() : cut.cross(Vec3d.X).normalized(), v = cut.cross(u);
        int samples = Math.max(4, (int) Math.ceil(2 * radius / 0.5));
        double hardest = 0, tensile = 0, cut3 = 0;
        boolean wall = false;
        Material soft = null;
        java.util.Set<Long> seen = new java.util.HashSet<>();
        java.util.List<int[]> cells = new java.util.ArrayList<>();
        for (int i = 0; i < samples; i++) {
            for (int j = 0; j < samples; j++) {
                double du = -radius + (i + 0.5) * 2 * radius / samples, dv = -radius + (j + 0.5) * 2 * radius / samples;
                if (du * du + dv * dv > radius * radius) { continue; }
                Vec3d p = centre.add(u.scale(du)).add(v.scale(dv)).add(cut.scale(0.05));
                int x = floor(p.x()), y = floor(p.y()), z = floor(p.z());
                if (!seen.add(Terrain.key(x, y, z))) { continue; }
                Terrain.Cell c = t.cell(x, y, z);
                if (c == null) { continue; }
                if (!c.diggable()) { wall = true; continue; }
                Material m = c.material();
                if (m.isRock() && m.compressivePa() > hardest) { hardest = m.compressivePa(); tensile = m.tensilePa(); }
                if (!m.isRock()) { soft = m; }
                cells.add(new int[]{x, y, z});
            }
        }
        RigidBodyState s = step.state();
        Vec3d offset = centre.subtract(s.position());
        if (wall && advance > 0) { step.damper(offset, cut, WALL_NS_PER_M * samples, 0); }
        if (cells.isEmpty()) { step.diagnostic(tool.name() + "ThrustN", 0); return 0; }
        double turn = Math.max(Math.abs(omega) / (2 * Math.PI), 1e-3);
        double penetration = Math.min(advance / turn, face.discDiameterM() / 4);
        double thrust, torque;
        if (hardest > 0) {
            RockCutting.Disc d = RockCutting.disc(hardest, tensile, face.discDiameterM(), face.tipWidthM(), face.spacingM(), Math.max(penetration, 1e-4), 0);
            thrust = face.discs() * d.normalN();
            torque = RockCutting.cutterheadTorqueNm(face.discs(), d.rollingN(), 2 * radius);
        } else {
            // soft ground: the face is pushed through as a blade of its area
            Material m = soft == null ? Material.LOAM : soft;
            double se = Digging.specificEnergyPa(m, Math.PI / 2, Math.max(penetration, 0.01), advance);
            thrust = se * Math.PI * radius * radius * 0.1;
            torque = se * Math.PI * radius * radius * Math.max(penetration, 0.01) * radius / 3;
        }
        // the thrust the face takes resists the advance as a damper of thrust / speed (a boring machine creeps at
        // millimetres a second: the force is the discs' at the penetration per turn it advances at)
        step.damper(offset, cut, thrust / Math.max(advance, FACE_CREEP_MPS), 0);
        if (omega != 0) { step.rotorTorque(drive.rotor(), -Math.signum(omega) * torque, -torque / Math.max(Math.abs(omega), 1), true); }
        if (!full && advance > 0) {
            double bank = Math.PI * radius * radius * advance * dt;
            cut3 = bank;
            for (int[] c : cells) { step.cut(tool.name(), c[0], c[1], c[2], bank / cells.size(), tool.hold()); }
        }
        step.diagnostic(tool.name() + "ThrustN", thrust);
        step.diagnostic(tool.name() + "TorqueNm", torque);
        step.diagnostic(tool.name() + "PenetrationM", penetration);
        return cut3;
    }
}
