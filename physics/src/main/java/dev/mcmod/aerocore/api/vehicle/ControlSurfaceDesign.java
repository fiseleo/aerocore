package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AirfoilPolar;
import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.Rudder;
import dev.mcmod.aerocore.api.SurfaceForces;
import dev.mcmod.aerocore.api.SymmetricAirfoil;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * An all-moving control surface (rudder, elevator, fin) whose centre of pressure is at the part origin: span along the
 * part's +Y, chord along its +X (leading edge forward), turning about +Y by "deflection" (-1..1) times the maximum
 * deflection; positive moves the trailing edge toward +Z. Mount it rotated for a horizontal surface.
 * <ul>
 * <li>rootAtBody: root against a hull or fuselage. Polar of Whicker and Fehlner (1958) for low-aspect rudders on a
 *     ground plane (Rudder.lowAspectPolar), measured for geometric aspect ratio span/chord 1 to 3.</li>
 * <li>Free at both ends: lift slope 2 pi A / (2 + sqrt(A^2 + 4)) (Helmbold 1942; tends to the lifting-line value at large
 *     A and to slender-wing theory at small A), NACA 0015 profile drag 0.0065, induced drag with span efficiency 0.9,
 *     stall at 15 deg blended over 10 deg (a typical thick symmetric section near Re 1e6).</li>
 * </ul>
 * Loads come from AeroSolver through Rudder.load, blended by the surface's immersion in water and air; inside a
 * propeller's race (Connection.Slipstream) the inflow is raised by its actuator-disk slipstream. The model clamps
 * deflection at 35 deg (Rudder.MAX_ANGLE_RADIANS). Mass: a solid NACA 4-digit section of the thickness ratio
 * (area 0.685 t c).
 */
public record ControlSurfaceDesign(String name, double chordM, double spanM, double maxDeflectionDeg, boolean rootAtBody,
                                   double thicknessRatio, MassSpec mass) implements PartDesign {
    public static final String TYPE = "aerocore:control_surface";

    public ControlSurfaceDesign {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(mass, "mass");
    }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        MotorDesign.positive(issues, "chordM", chordM);
        MotorDesign.positive(issues, "spanM", spanM);
        if (!(maxDeflectionDeg > 0 && maxDeflectionDeg <= Math.toDegrees(Rudder.MAX_ANGLE_RADIANS))) {
            issues.add(DesignIssue.invalid("maxDeflectionDeg", "Maximum deflection must be in (0, 35] degrees (the model's limit)"));
        }
        if (!(thicknessRatio > 0 && thicknessRatio <= 0.3)) { issues.add(DesignIssue.invalid("thicknessRatio", "Thickness ratio must be in (0, 0.3]")); }
        mass.check(issues, "mass", false);
        if (issues.isEmpty() && rootAtBody) {
            DesignIssue.range(issues, "aspectRatio", spanM / chordM, 1, 3, "Whicker and Fehlner 1958 rudder tests");
        }
        return issues;
    }

    @Override public Part compile() {
        List<DesignIssue> issues = check();
        DesignException.throwIfInvalid(issues);
        return new Compiled(this, issues);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("chordM", chordM, "spanM", spanM, "maxDeflectionDeg", maxDeflectionDeg, "rootAtBody", rootAtBody,
                "thicknessRatio", thicknessRatio, "mass", mass.toJson());
    }

    static PartDesign read(Fields f) {
        return new ControlSurfaceDesign(f.string("name", "control surface"), f.number("chordM"), f.number("spanM"),
                f.number("maxDeflectionDeg", 35), f.bool("rootAtBody", true), f.number("thicknessRatio", 0.15),
                f.has("mass") ? MassSpec.fromJson(f.child("mass")) : MassSpec.solid(Material.STEEL));
    }

    /** The section polar used by Rudder.load. */
    AirfoilPolar polar() {
        if (rootAtBody) { return Rudder.lowAspectPolar(chordM, spanM); }
        double aspect = spanM / chordM, slope = 2 * Math.PI * aspect / (2 + Math.sqrt(aspect * aspect + 4));
        return new SymmetricAirfoil(slope, Math.toRadians(15), Math.toRadians(10), 0.0065, aspect, 0.9);
    }

    private static final class Compiled implements Part {
        private final ControlSurfaceDesign design;
        private final List<DesignIssue> issues;
        private final Rudder surface;
        private final MassProperties mass;

        Compiled(ControlSurfaceDesign design, List<DesignIssue> issues) {
            this.design = design;
            this.issues = issues.stream().filter(i -> i.kind() == DesignIssue.Kind.EXTRAPOLATION).toList();
            this.surface = new Rudder(Vec3d.ZERO, design.chordM, design.spanM, design.polar());
            double c = design.chordM, s = design.spanM, t = design.thicknessRatio * c, volume = 0.685 * t * c * s;
            MassProperties unit = new MassProperties(volume, Vec3d.ZERO, volume * (s * s + t * t) / 12, volume * (c * c + t * t) / 12,
                    volume * (c * c + s * s) / 12, 0, 0, 0);
            this.mass = design.mass.resolve(th -> unit);
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, 0.5 * Math.min(design.chordM, design.spanM))); }
        @Override public List<DesignIssue> issues() { return issues; }
        @Override public List<String> inputs() { return List.of("deflection"); }

        @Override public void loads(PartStep step) {
            double angle = Math.clamp(step.input("deflection", 0), -1, 1) * Math.toRadians(design.maxDeflectionDeg);
            FloatingBodyDynamics.Contact contact = step.environment().sphere(step.point(Vec3d.ZERO), design.spanM / 2);
            double f = contact.fraction();
            double[] race = step.slipstream();
            double waterThrust = race == null ? 0 : race[0], airThrust = race == null ? 0 : race[1], diameter = race == null ? 0 : race[2];
            Wrench total = Wrench.ZERO;
            if (f > 0) {
                SurfaceForces w = surface.load(step.partState(), step.partProperties(), angle, contact.water(), waterThrust, diameter);
                total = total.add(scale(w.wrench(), f));
            }
            if (f < 1) {
                SurfaceForces a = surface.load(step.partState(), step.partProperties(), angle, contact.air(), airThrust, diameter);
                total = total.add(scale(a.wrench(), 1 - f));
            }
            if (total.force().lengthSquared() > 0 || total.torque().lengthSquared() > 0) {
                step.load(total, AppliedForce.Kind.LIFT);
                step.display(new AppliedForce(AppliedForce.Kind.LIFT, step.offset(Vec3d.ZERO), total.force()));
            }
            step.diagnostic("deflectionDeg", Math.toDegrees(angle));
            step.diagnostic("forceN", total.force().length());
            step.diagnostic("immersion", f);
        }

        private static Wrench scale(Wrench w, double k) { return new Wrench(w.force().scale(k), w.torque().scale(k)); }
    }
}
