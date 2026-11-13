package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import dev.mcmod.aerocore.api.aero.HelicopterRotor;
import dev.mcmod.aerocore.api.aero.Wing;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * A helicopter rotor (or a tail rotor mounted on its side) whose hub is at the part origin, its shaft along the part's
 * +Y, from HelicopterRotor. Inputs: "collective" (0..1 between its stops), "cyclicPitch" (-1..1, +1 tilts the disk
 * forward) and "cyclicRoll" (-1..1, +1 to starboard); a part wrapping it can give the blade pitch itself. The induced
 * velocity and flapping are kept as internal states (the inflow lags as Pitt and Peters'; the flapping converges over the
 * steps). The aerodynamic torque acts on the rotor (its shaft train must give it); the hub's force and moment act on
 * the vehicle. Ground effect over the water surface; over land it arrives with the ground query.
 *
 * <p>0.42: a rotor given a Wearing (wearing) has one condition, BLADES (0), which only the game changes (it does not wear
 * itself). Damaged blades give less: the hub's force and moment are times Wearing.thrustShare (1 - thrustLoss x used,
 * failedThrust at 1) while the torque the air asks of the shaft does not fall. Diagnostic "bladesUsed"; "thrustN" is
 * the damaged one. A part wrapping it lists the condition first. Without a Wearing: no conditions, as before 0.42.
 */
public final class RotorPart implements Part {
    private final PartDesign design;
    private final HelicopterRotor rotor;
    private final double minCollectiveDeg, maxCollectiveDeg, cyclicDeg, spinInertia;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final Wearing wearing;

    /** 0.42: the name of a wearing rotor's condition (index 0). */
    public static final String BLADES = "blades";
    private static final List<String> CONDITIONS = List.of(BLADES);

    /**
     * 0.42: what damage does to the blades (the caller's rule): the share of the thrust lost as they come to the end
     * (in proportion before), and the share of the thrust left with them used up (broken off short).
     */
    public record Wearing(double thrustLoss, double failedThrust) {
        public Wearing {
            if (!(thrustLoss >= 0 && thrustLoss <= 1) || !(failedThrust >= 0 && failedThrust <= 1)) { throw new IllegalArgumentException("Thrust shares 0..1"); }
        }
        /** Rule: bent and chipped blades lose up to 0.6 of their thrust; stubs give a tenth. */
        public static Wearing typical() { return new Wearing(0.6, 0.1); }
        /** The share of its thrust a rotor keeps with so much of its blades used. */
        public double thrustShare(double used) {
            if (!(used > 0)) { return 1; }
            return used >= 1 ? failedThrust : 1 - thrustLoss * used;
        }
    }

    public RotorPart(PartDesign design, HelicopterRotor.Geometry geometry, double minCollectiveDeg, double maxCollectiveDeg, double cyclicDeg,
                     double hubInertiaKgM2, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.rotor = new HelicopterRotor(Objects.requireNonNull(geometry, "geometry"));
        if (!(maxCollectiveDeg >= minCollectiveDeg) || !(cyclicDeg >= 0)) { throw new IllegalArgumentException("Bad pitch travel"); }
        this.minCollectiveDeg = minCollectiveDeg; this.maxCollectiveDeg = maxCollectiveDeg; this.cyclicDeg = cyclicDeg;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
        // blades about the shaft (uniform from the hinge to the tip) and the hub
        double r = geometry.radiusM(), e = geometry.hingeOffset() * r;
        double blade = geometry.bladeMassKg() * ((r * r * r - e * e * e) / (3 * (r - e)));
        this.spinInertia = Math.max(geometry.blades() * blade + hubInertiaKgM2, 1e-6);
        this.wearing = null;
    }

    private RotorPart(RotorPart of, Wearing wearing) {
        design = of.design; rotor = of.rotor; minCollectiveDeg = of.minCollectiveDeg; maxCollectiveDeg = of.maxCollectiveDeg; cyclicDeg = of.cyclicDeg;
        spinInertia = of.spinInertia; mass = of.mass; issues = of.issues;
        this.wearing = wearing;
    }

    /** 0.42: this rotor damageable by this rule, with the condition BLADES; null: one that is not. A copy (sharing the rotor's model). */
    public RotorPart wearing(Wearing wearing) { return new RotorPart(this, wearing); }
    /** 0.42: what damage does to this rotor, or null. */
    public Wearing wearing() { return wearing; }
    @Override public List<String> conditions() { return wearing == null ? List.of() : CONDITIONS; }

    public HelicopterRotor rotor() { return rotor; }
    public HelicopterRotor.Geometry geometry() { return rotor.geometry(); }
    public double minCollectiveDeg() { return minCollectiveDeg; }
    public double maxCollectiveDeg() { return maxCollectiveDeg; }
    public double cyclicDeg() { return cyclicDeg; }
    /** Moment of inertia of blades and hub about the shaft (kg m^2). */
    public double spinInertia() { return spinInertia; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() {
        double r = rotor.geometry().radiusM();
        return List.of(new CollisionSphere(Vec3d.ZERO, Math.max(0.1, 0.12 * r)));
    }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor(rotor.geometry().clockwise() ? Vec3d.Y.scale(-1) : Vec3d.Y, spinInertia, false)); }
    @Override public List<String> inputs() { return List.of("collective", "cyclicPitch", "cyclicRoll"); }
    @Override public int internalStates() { return 4; }

    /**
     * 0.30: shown for drawing: coning, the tip path's forward and sideways tilt (beta0, beta1c, beta1s), the blade pitch's
     * collective and cyclic parts (theta0, theta1c, theta1s; rad in 0.001 steps), and the induced velocity (m/s, 0.1).
     */
    @Override public int shownValues() { return SHOWN; }

    /** How many shown values a rotor has (a part wrapping it forwards them). */
    public static final int SHOWN = 7;

    @Override public void loads(PartStep step) {
        double collective = Math.toRadians(minCollectiveDeg + Math.clamp(step.input("collective", 0), 0, 1) * (maxCollectiveDeg - minCollectiveDeg));
        loads(step, HelicopterRotor.Controls.fromSticks(collective, step.input("cyclicPitch", 0), step.input("cyclicRoll", 0),
                Math.toRadians(cyclicDeg), rotor.geometry().clockwise()));
    }

    /** The loads at a blade pitch (rad, the rotor's own azimuth). */
    public void loads(PartStep step, HelicopterRotor.Controls controls) {
        Vec3d hub = step.point(Vec3d.ZERO);
        HullDynamics.Water water = step.environment().point(hub);
        FluidState air = water.air();
        Vec3d velocity = step.partDirection(step.velocity(Vec3d.ZERO).subtract(air.velocity()));
        Vec3d rate = step.partDirection(step.angularVelocity());
        double omega = step.rotorSpeed(0);
        double height = water.hasWater() ? hub.y() - water.surfaceY() : Double.POSITIVE_INFINITY;
        HelicopterRotor.State previous = new HelicopterRotor.State(step.internal(0), step.internal(1), step.internal(2), step.internal(3));
        HelicopterRotor.Loads l = rotor.loads(velocity, rate, omega, new Wing.Air(air.densityKgPerM3(), air.dynamicViscosityPaS(), air.speedOfSoundMps()),
                controls, height, previous, step.dt());
        HelicopterRotor.State next = l.next();
        step.setInternal(0, next.inducedMps()); step.setInternal(1, next.beta0()); step.setInternal(2, next.beta1c()); step.setInternal(3, next.beta1s());
        if (step.shownCount() >= SHOWN) {   // a part wrapping it may not show them
            double[] shown = {next.beta0(), next.beta1c(), next.beta1s(), controls.collective(), controls.cyclicCos(), controls.cyclicSin()};
            for (int i = 0; i < shown.length; i++) { step.show(i, PartStep.stepped(step.shown(i), shown[i], 1e-3)); }
            step.show(6, PartStep.stepped(step.shown(6), next.inducedMps(), 0.1));
        }
        Vec3d hubForce = l.force(), hubMoment = l.moment();
        double thrust = l.thrustN();
        if (wearing != null) {
            // 0.42: damaged blades carry less for the same torque
            double used = step.used(0);
            if (used > 0) { double left = wearing.thrustShare(used); hubForce = hubForce.scale(left); hubMoment = hubMoment.scale(left); thrust *= left; }
            step.diagnostic("bladesUsed", used);
        }
        Vec3d force = step.axis(hubForce), origin = step.offset(Vec3d.ZERO);
        Vec3d moment = step.axis(hubMoment).add(origin.cross(force));
        step.load(new Wrench(force, moment), AppliedForce.Kind.THRUST);
        step.display(new AppliedForce(AppliedForce.Kind.THRUST, origin, force));
        // the air resists the rotor's turning; the torque grows about as the square of its speed
        double torque = l.torqueNm();
        step.rotorTorque(0, -torque, -2 * Math.max(torque, 0) / Math.max(omega, 1), true);
        step.diagnostic("rpm", omega * 60 / (2 * Math.PI));
        step.diagnostic("thrustN", thrust);
        step.diagnostic("torqueNm", torque);
        step.diagnostic("powerW", l.powerW());
        step.diagnostic("coningDeg", Math.toDegrees(next.beta0()));
        step.diagnostic("tiltForwardDeg", Math.toDegrees(next.beta1c()));
        step.diagnostic("inducedMps", next.inducedMps());
        step.diagnostic("stalledShare", l.stalledShare());
        step.diagnostic("collectiveDeg", Math.toDegrees(controls.collective()));
    }
}
