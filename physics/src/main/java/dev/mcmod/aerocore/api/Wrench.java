package dev.mcmod.aerocore.api;

import java.util.Objects;

/** World-space force in newtons and torque about the center of mass in newton-metres. */
public record Wrench(Vec3d force, Vec3d torque) {
    public static final Wrench ZERO = new Wrench(Vec3d.ZERO, Vec3d.ZERO);
    public Wrench {
        Objects.requireNonNull(force, "force");
        Objects.requireNonNull(torque, "torque");
    }
    public static Wrench atOffset(Vec3d force, Vec3d offset) {
        return new Wrench(force, offset.cross(force));
    }
    public Wrench add(Wrench other) {
        return new Wrench(force.add(other.force), torque.add(other.torque));
    }
}

