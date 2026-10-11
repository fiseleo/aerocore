package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.BSeriesFourQuadrant;
import dev.mcmod.aerocore.api.BladeElementPropeller;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.Propeller;
import dev.mcmod.aerocore.api.PropellerModel;
import dev.mcmod.aerocore.api.PropellerSolver;
import dev.mcmod.aerocore.api.TestPlatform;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WageningenB;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A propeller whose hub is at the part origin, pushing along the part's +X when turning positively about it.
 * <ul>
 * <li>Inside the Wageningen B-series range (2 to 7 blades, expanded area ratio 0.30 to 1.05, pitch ratio 0.5 to 1.4;
 *     Oosterveld and van Oossanen 1975): the B-series regression ahead, and astern and windmilling MARIN's measured
 *     four-quadrant data for the three tested screws (B3-65 P/D 1.0, B4-70 P/D 1.0 and 1.4) or blade-element momentum
 *     theory of the same geometry otherwise, joined by 15 degree ramps (WageningenB.fourQuadrant).</li>
 * <li>Outside it: blade-element momentum theory in all quadrants (BladeElementPropeller), reported as an extrapolation
 *     of the B-series. This is also the model for air propellers far from B-series shapes.</li>
 * </ul>
 * Constant-chord blades: chord ratio c/D = (A_E/A_0) pi / (2 Z (1 - h)), h the hub ratio; the BEMT section is the
 * marine fit of TestPlatform.PROPELLER_SECTION. Loads are blended by the disk's immersion in water and air
 * (PropellerSolver; no ventilation of single blades, no hull wake). The fluid torque acts on the rotor; a motor
 * joined by a shaft must supply it, and the vehicle feels it through the shaft train (VehicleDynamics).
 * Mass: blades of mean thickness tau c (tau the thickness ratio) and a hub cylinder 0.3 D long, of the material.
 */
public record PropellerDesign(String name, double diameterM, int blades, double areaRatio, double pitchRatio, double hubRatio,
                              double thicknessRatio, MassSpec mass) implements PartDesign {
    public static final String TYPE = "aerocore:propeller";
    private static final String B_SERIES = "Wageningen B-series (Oosterveld and van Oossanen 1975)";

    public PropellerDesign {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(mass, "mass");
    }

    @Override public String type() { return TYPE; }

    /** Constant blade chord over diameter giving the expanded area ratio. */
    public double chordRatio() { return areaRatio * Math.PI / (2 * blades * (1 - hubRatio)); }

    public boolean inBSeriesRange() {
        return blades >= 2 && blades <= 7 && areaRatio >= 0.30 && areaRatio <= 1.05 && pitchRatio >= 0.5 && pitchRatio <= 1.4;
    }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!(diameterM > 0) || !Double.isFinite(diameterM)) { issues.add(DesignIssue.invalid("diameterM", "Diameter must be positive")); }
        if (blades < 2 || blades > 12) { issues.add(DesignIssue.invalid("blades", "Blade count must be 2 to 12")); }
        if (!(areaRatio > 0) || !Double.isFinite(areaRatio)) { issues.add(DesignIssue.invalid("areaRatio", "Area ratio must be positive")); }
        if (!(pitchRatio > 0 && pitchRatio <= 3)) { issues.add(DesignIssue.invalid("pitchRatio", "Pitch ratio must be in (0, 3]")); }
        if (!(hubRatio > 0 && hubRatio < 0.5)) { issues.add(DesignIssue.invalid("hubRatio", "Hub ratio must be in (0, 0.5)")); }
        if (!(thicknessRatio > 0 && thicknessRatio <= 0.5)) { issues.add(DesignIssue.invalid("thicknessRatio", "Blade thickness ratio must be in (0, 0.5]")); }
        if (issues.isEmpty() && chordRatio() > 1) { issues.add(DesignIssue.invalid("areaRatio", "Blades this wide overlap: lower the area ratio or add blades")); }
        mass.check(issues, "mass", false);
        if (issues.isEmpty()) {
            DesignIssue.range(issues, "blades", blades, 2, 7, B_SERIES + "; blade-element theory used");
            DesignIssue.range(issues, "areaRatio", areaRatio, 0.30, 1.05, B_SERIES + "; blade-element theory used");
            DesignIssue.range(issues, "pitchRatio", pitchRatio, 0.5, 1.4, B_SERIES + "; blade-element theory used");
        }
        return issues;
    }

    @Override public Part compile() {
        List<DesignIssue> issues = check();
        DesignException.throwIfInvalid(issues);
        return new Compiled(this, issues);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("diameterM", diameterM, "blades", blades, "areaRatio", areaRatio, "pitchRatio", pitchRatio,
                "hubRatio", hubRatio, "thicknessRatio", thicknessRatio, "mass", mass.toJson());
    }

    static PartDesign read(Fields f) {
        return new PropellerDesign(f.string("name", "propeller"), f.number("diameterM"), f.integer("blades"), f.number("areaRatio"),
                f.number("pitchRatio"), f.number("hubRatio", 0.18), f.number("thicknessRatio", 0.1),
                f.has("mass") ? MassSpec.fromJson(f.child("mass")) : MassSpec.solid(Material.BRONZE));
    }

    /** The four-quadrant model, tabulated at 1 degree. */
    PropellerModel model() {
        BladeElementPropeller bemt = new BladeElementPropeller(blades, hubRatio, chordRatio(), pitchRatio, TestPlatform.PROPELLER_SECTION, 20);
        if (!inBSeriesRange()) { return bemt.tabulate(361); }
        WageningenB series = new WageningenB(blades, areaRatio, pitchRatio);
        PropellerModel elsewhere = bemt;
        if (blades == 3 && areaRatio == 0.65 && pitchRatio == 1.0) { elsewhere = BSeriesFourQuadrant.B3_65_PITCH_1_0; }
        else if (blades == 4 && areaRatio == 0.70 && pitchRatio == 1.0) { elsewhere = BSeriesFourQuadrant.B4_70_PITCH_1_0; }
        else if (blades == 4 && areaRatio == 0.70 && pitchRatio == 1.4) { elsewhere = BSeriesFourQuadrant.B4_70_PITCH_1_4; }
        return series.fourQuadrant(elsewhere, Math.toRadians(15), 361);
    }

    /** Unit-density mass: blades as plates from the hub to the tip, a hub cylinder 0.3 D long. */
    private MassProperties unitMass() {
        double r = diameterM / 2, rh = hubRatio * r, chord = chordRatio() * diameterM, hubLength = 0.3 * diameterM;
        double blade = blades * chord * (r - rh) * thicknessRatio * chord, hub = Math.PI * rh * rh * hubLength;
        // Spin axis: blades as rods from rh to r, hub a solid cylinder. Across: half the spin value for the disk of blades.
        double spin = blade * (r * r * r - rh * rh * rh) / (3 * (r - rh)) + 0.5 * hub * rh * rh;
        double across = 0.5 * blade * (r * r * r - rh * rh * rh) / (3 * (r - rh)) + hub * (3 * rh * rh + hubLength * hubLength) / 12;
        return new MassProperties(blade + hub, Vec3d.ZERO, spin, across, across, 0, 0, 0);
    }

    private static final class Compiled implements Part {
        private final PropellerDesign design;
        private final List<DesignIssue> issues;
        private volatile PropellerModel model;
        private final MassProperties mass;
        private final double spinInertia;

        Compiled(PropellerDesign design, List<DesignIssue> issues) {
            this.design = design;
            this.issues = issues.stream().filter(i -> i.kind() == DesignIssue.Kind.EXTRAPOLATION).toList();
            this.mass = design.mass.resolve(t -> design.unitMass());
            this.spinInertia = Math.max(mass.xx(), 1e-9);
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, design.diameterM / 2)); }
        @Override public List<DesignIssue> issues() { return issues; }
        @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, spinInertia, false)); }
        @Override public boolean early() { return true; }
        @Override public void warmUp() { model(); }

        /** The four-quadrant table, built on the first step (a client only drawing the propeller never builds it). */
        private PropellerModel model() {
            PropellerModel m = model;
            if (m == null) {
                synchronized (this) {
                    m = model;
                    if (m == null) { model = m = design.model(); }
                }
            }
            return m;
        }

        @Override public void loads(PartStep step) {
            double d = design.diameterM, rps = step.rotorSpeed(0) / (2 * Math.PI);
            Vec3d hub = step.point(Vec3d.ZERO);
            FloatingBodyDynamics.Contact disk = step.environment().sphere(hub, d / 2);
            Propeller propeller = new Propeller(step.vehiclePoint(Vec3d.ZERO), step.mount().rotation().rotate(Vec3d.X), d, model());
            PropellerSolver.Load water = PropellerSolver.load(step.state(), step.properties(), propeller, rps, disk.water());
            PropellerSolver.Load air = PropellerSolver.load(step.state(), step.properties(), propeller, rps, disk.air());
            double f = disk.fraction(), g = 1 - f;
            Vec3d force = water.force().force().scale(f).add(air.force().force().scale(g));
            double torque = f * water.torqueNm() + g * air.torqueNm();
            // Torque slope with speed, for the shaft train's implicit step.
            double dn = Math.max(1e-3, 1e-3 * Math.abs(rps));
            double ahead = f * PropellerSolver.load(step.state(), step.properties(), propeller, rps + dn, disk.water()).torqueNm()
                    + g * PropellerSolver.load(step.state(), step.properties(), propeller, rps + dn, disk.air()).torqueNm();
            double slope = (ahead - torque) / (dn * 2 * Math.PI);
            if (force.lengthSquared() > 0) { step.force(new AppliedForce(AppliedForce.Kind.THRUST, water.force().offset(), force)); }
            // The fluid turns the rotor with -Q about the axis.
            step.rotorTorque(0, -torque, -slope, true);
            step.thrust(f * water.thrustN(), g * air.thrustN(), d);
            step.diagnostic("rps", rps);
            step.diagnostic("thrustN", f * water.thrustN() + g * air.thrustN());
            step.diagnostic("torqueNm", torque);
            step.diagnostic("advanceMps", f * water.advanceSpeedMps() + g * air.advanceSpeedMps());
            step.diagnostic("shaftPowerW", f * water.shaftPowerW() + g * air.shaftPowerW());
            step.diagnostic("immersion", f);
        }
    }
}
