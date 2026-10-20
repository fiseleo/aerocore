package dev.mcmod.aerocore.api.body;

import dev.mcmod.aerocore.api.FlightAtmosphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.aero.Airfoil;
import dev.mcmod.aerocore.api.aero.Wing;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A road vehicle's aero kit (0.34, the user's 「外掛空力套件」 in hdfv 0.16): the loads its add-ons put on a body,
 * over and above those of the body's smooth shape (BodyAerodynamics), in the body's frame (x forward from the rear end,
 * y up, z starboard). Each element is a correlation or a small model, not a flow solution:
 * <ul>
 * <li><b>Underbody</b>: the floor between the body and the ground as a one-dimensional channel (Katz, Race Car
 *   Aerodynamics, 1995, ch. 6; the venturi view of ground-effect cars). Air enters under the front at free-stream total
 *   pressure through an inlet loss (K_IN, more behind an air dam: the dam narrows the inlet and the flow expands again
 *   behind it, a Borda-Carnot loss), runs along the floor with the friction of the channel (lambda L / 2h: a rough
 *   road-car floor ROUGH, a road car's undertray SMOOTH) and leaves through the diffuser into the wake at the base
 *   pressure (C_PB). The diffuser recovers pressure as a flat-walled diffuser of its area ratio, with an efficiency that
 *   falls past THETA_STALL of ramp angle and past an area ratio of AR_STALL (Kline's stall map: a long ramp close to the
 *   ground stalls; this is where a real diffuser's downforce peaks and falls off as the car gets lower, Cooper et al., SAE
 *   980030, 1998); behind a rough floor it keeps ROUGH_RECOVERY of it (Marklund 2013: without the underbody covers the
 *   diffuser's recovery is lost). The losses are taken at the front (the separation off the bumper's lower edge, the
 *   wheels), so the floor is at the diffuser inlet's pressure along its length.</li>
 * <li><b>Wheels</b> (0.35.1, calibrated on production cars): a road car's wheels stand at the floor's corners. The front
 *   wheels' squirt and the wheel houses venting into the floor take K_WHEELS of the floor flow's dynamic pressure, and
 *   the rear wheels' wakes running into the diffuser leave WHEEL_RECOVERY of its recovery (Breslouer &amp; George, SAE
 *   2008-01-0326: wheels beside a diffuser body take as much as 65 % of its downforce; Marklund, Chalmers 2013: the
 *   outer sections of a car's diffuser are disturbed by the wheels and suspension).</li>
 * <li><b>Sides and front</b>: air leaking in under the sides spoils the suction. The leak is the sides' open area: an
 *   open floor keeps b / (b + LEAK L) of it, skirts close a share (sealing) of that area, b / (b + LEAK L (1 - sealing))
 *   (sealing 1: skirts to the ground the whole length). Behind an air dam the flow is separated for some DAM_REACH dam
 *   depths at DAM_WAKE, in place of the floor's pressure there (the front lift an air dam takes off). A splitter ahead of
 *   the floor has the bumper's stagnation pressure on top (SPLITTER_TOP) and the inlet's suction below.</li>
 * <li><b>Rear wing</b>: an inverted wing (AeroCore's lifting line, Wing) with end plates (the induced velocities times
 *   1 / (1 + 1.9 h / b), Hoerner, Fluid-Dynamic Lift, 1985) and the ground's image at its design height above the road
 *   (Wieselsberger); its coefficients are tabled against angle of attack once and looked up each step. Not modelled:
 *   the body's own downwash and wake at the wing.</li>
 * <li><b>Fence</b>: a lip spoiler on the rear deck, a Gurney-flap-like fence that raises the deck's pressure ahead of it
 *   (lift down FENCE_LIFT and drag FENCE_DRAG times its area; of the order Hucho, Aerodynamics of Road Vehicles,
 *   1998, gives for notchback spoilers: some 0.1 off the rear lift coefficient for a 5 cm lip).</li>
 * <li><b>Bluff</b>: drag only, from a frontal area and a drag coefficient (roof bars, a basket, a bull bar, flares,
 *   scoops): CD_TUBE for round bars (Hoerner, subcritical cylinder), CD_BLUFF for blunt pieces, CD_SHIELDED where the
 *   body's own wake shelters them.</li>
 * </ul>
 * The underbody's loads are those of the kit's floor less those of a plain road-car floor of the same size at the same
 * height (no diffuser, no dam, no splitter, open sides, ROUGH, the same wheels), so a body without a kit is exactly as
 * before and a kit adds only its difference. They fade out above FADE_FROM m of ride height (gone at FADE_TO: off the
 * ground, over water). Backwards, the floor, splitter and fence give nothing.
 * <p>
 * 0.35.1 calibration (docs/BODY.md): 0.34 had no wheels, took skirts as a linear blend to a sealed floor and added the
 * dam's separated zone on top of the floor's suction; hdfv's sample sedan with a mild kit got 2.96 kN from its floor at
 * 200 km/h. The wheels, the skirts' leak area and the dam zone in place of the floor bring a production diffuser and
 * undertray (Porsche 718 Cayman GT4, 2019: the diffuser 30 % of 122 kg rear downforce at 304 km/h) and a 40 mm underbody
 * dam (Mercedes-AMG GT R, 2016: 40 kg less front lift at 250 km/h) within 15 %.
 */
public final class RoadAero {
    /** Inlet loss, base pressure coefficient, friction factors of a rough floor and of a road car's undertray (its exhaust tunnel, suspension and tank left open). */
    public static final double K_IN = 0.3, C_PB = -0.15, ROUGH = 0.07, SMOOTH = 0.045;
    /** The diffuser's best efficiency, the ramp angle (deg) and area ratio past which it stalls; the share of its recovery left behind a rough floor. */
    public static final double ETA_DIFFUSER = 0.85, THETA_STALL = 18, AR_STALL = 2.5, ROUGH_RECOVERY = 0.3;
    /** The wheels at the floor's corners: the front wheels' loss on the floor flow, the share of a diffuser's recovery the rear wheels' wakes leave. */
    public static final double K_WHEELS = 1.0, WHEEL_RECOVERY = 0.4;
    /** Side leakage over the floor's length, the splitter's upper pressure coefficient, an air dam's drag coefficient. */
    public static final double LEAK = 1, SPLITTER_TOP = 0.5, CD_DAM = 0.4;
    /** The separated zone behind an air dam: its length in dam depths and its pressure coefficient (in place of the floor's). */
    public static final double DAM_REACH = 10, DAM_WAKE = -0.3;
    /** A lip spoiler's downforce and drag per its area (fence height times width) and dynamic pressure. */
    public static final double FENCE_LIFT = 3.0, FENCE_DRAG = 0.6;
    /** Drag coefficients on frontal area: a round bar, a blunt piece, one in the body's wake. */
    public static final double CD_TUBE = 1.2, CD_BLUFF = 1.0, CD_SHIELDED = 0.4;
    /** Ride heights (m) between which the floor's loads fade out, and the least gap the channel is taken at. */
    public static final double FADE_FROM = 0.5, FADE_TO = 1.0, MIN_GAP = 0.005;
    /** The wing tables' speed (m/s; Reynolds number effects are left out) and angle range and step (deg). */
    static final double TABLE_SPEED = 30, TABLE_FROM = -20, TABLE_TO = 30, TABLE_STEP = 1;

    /** One add-on. */
    public sealed interface Element permits Underbody, RearWing, Fence, Bluff {}

    /**
     * The floor from frontXM (its front, where the air comes in) back to rearXM (the rear end, the diffuser's exit),
     * widthM across, its flat part at floorYM; a diffuser ramp over the last diffuserLengthM rising at diffuserAngleDeg
     * (0: none); sealing: the share of its sides' leak area that skirts close (0 open, 1 skirts to the ground all along
     * it; skirts only between the wheel arches, or with a gap to the road, close less); an air dam damDepthM below the
     * floor at its front; a splitter splitterLengthM ahead of the front and splitterWidthM across at the dam's lower edge
     * (0: none). smooth: an undertray (else a road car's rough floor, behind which a diffuser recovers little).
     */
    public record Underbody(double frontXM, double rearXM, double widthM, double floorYM, double diffuserLengthM, double diffuserAngleDeg,
                            double sealing, double damDepthM, double splitterLengthM, double splitterWidthM, boolean smooth) implements Element {
        public Underbody {
            if (!(frontXM > rearXM) || !(widthM > 0) || !Double.isFinite(floorYM)) { throw new IllegalArgumentException("A floor has a length and a width"); }
            if (!(diffuserLengthM >= 0) || diffuserLengthM >= frontXM - rearXM) { throw new IllegalArgumentException("The diffuser is shorter than the floor"); }
            if (!(diffuserAngleDeg >= 0 && diffuserAngleDeg < 45)) { throw new IllegalArgumentException("Diffuser angle 0 to 45 degrees"); }
            if (!(sealing >= 0 && sealing <= 1)) { throw new IllegalArgumentException("Sealing 0 to 1"); }
            if (!(damDepthM >= 0) || !(splitterLengthM >= 0) || !(splitterWidthM >= 0)) { throw new IllegalArgumentException("Dam and splitter sizes are not negative"); }
        }
        /** Where the diffuser starts (its inlet). */
        public double diffuserStartXM() { return rearXM + diffuserLengthM; }
    }

    /**
     * An inverted wing with its quarter-chord's middle at quarterChordM: span and chord (m), its angle of attack for
     * downforce (deg), the section's camber and thickness (shares of the chord, a NACA four-digit with the camber at
     * 40 %), end plates endplateHeightM tall (0: none) and its height above the road at rest (m).
     */
    public record RearWing(Vec3d quarterChordM, double spanM, double chordM, double angleDeg, double camber, double thickness,
                           double endplateHeightM, double designHeightM) implements Element {
        public RearWing {
            Objects.requireNonNull(quarterChordM, "quarterChordM");
            if (!(spanM > 0) || !(chordM > 0) || !(thickness > 0 && thickness < 0.4) || !(camber >= 0 && camber < 0.12)) {
                throw new IllegalArgumentException("A wing has a span, a chord, a thickness of 0 to 40 % and a camber of 0 to 12 %");
            }
            if (!Double.isFinite(angleDeg) || !(endplateHeightM >= 0) || !(designHeightM > 0)) { throw new IllegalArgumentException("Bad wing angle, end plates or height"); }
        }
        public double areaM2() { return spanM * chordM; }
    }

    /** A lip spoiler (fence) at atM, widthM across and heightM tall. */
    public record Fence(Vec3d atM, double widthM, double heightM) implements Element {
        public Fence {
            Objects.requireNonNull(atM, "atM");
            if (!(widthM > 0) || !(heightM > 0)) { throw new IllegalArgumentException("A fence has a width and a height"); }
        }
    }

    /** A piece that only adds drag: its frontal area (m^2) and drag coefficient, at atM. */
    public record Bluff(Vec3d atM, double frontalAreaM2, double dragCoefficient) implements Element {
        public Bluff {
            Objects.requireNonNull(atM, "atM");
            if (!(frontalAreaM2 >= 0) || !(dragCoefficient >= 0)) { throw new IllegalArgumentException("Area and drag coefficient are not negative"); }
        }
    }

    /** Height above the ground of a point of the body (body frame), m; infinite where there is none below. */
    @FunctionalInterface
    public interface Heights { double heightM(Vec3d bodyPoint); }

    /**
     * The kit's loads: force and moment about the body's origin (body frame, N and N m), the downforce (N, along -y)
     * and the drag (N, along the flow), for diagnostics.
     */
    public record Loads(Vec3d force, Vec3d moment, double downforceN, double dragN) {
        public static final Loads NONE = new Loads(Vec3d.ZERO, Vec3d.ZERO, 0, 0);
    }

    private final List<Element> elements;
    private volatile WingTable[] tables;

    public RoadAero(List<? extends Element> elements) { this.elements = List.copyOf(elements); }

    public List<Element> elements() { return elements; }
    public boolean isEmpty() { return elements.isEmpty(); }

    /** Builds the wings' tables (a second or so per wing; loads builds them itself the first time otherwise). */
    public void warmUp() { tables(); }

    private WingTable[] tables() {
        WingTable[] t = tables;
        if (t == null) {
            synchronized (this) {
                t = tables;
                if (t == null) {
                    t = new WingTable[elements.size()];
                    for (int i = 0; i < t.length; i++) { if (elements.get(i) instanceof RearWing w) { t[i] = WingTable.of(w); } }
                    tables = t;
                }
            }
        }
        return t;
    }

    /**
     * The loads at an air velocity relative to the body (body frame, m/s: the flow, -x when driving forward), air density
     * (kg/m^3) and the ground's heights below the body's points.
     */
    public Loads loads(Vec3d air, double densityKgM3, Heights heights) {
        double speed = air.length();
        if (elements.isEmpty() || !(speed > 1e-6) || !(densityKgM3 > 0)) { return Loads.NONE; }
        WingTable[] t = tables();
        double forward = Math.max(0, -air.x()), qForward = 0.5 * densityKgM3 * forward * forward, q = 0.5 * densityKgM3 * speed * speed;
        Vec3d flow = air.scale(1 / speed);
        List<Vec3d[]> forces = new ArrayList<>();   // {point, force}
        for (int i = 0; i < elements.size(); i++) {
            switch (elements.get(i)) {
                case Underbody u -> { if (qForward > 0) { underbody(u, qForward, heights, forces, true, true); } }
                case RearWing w -> t[i].add(w, air, q, forces);
                case Fence f -> {
                    if (qForward > 0) {
                        double a = f.widthM() * f.heightM();
                        forces.add(new Vec3d[]{f.atM(), new Vec3d(-FENCE_DRAG * qForward * a, -FENCE_LIFT * qForward * a, 0)});
                    }
                }
                case Bluff b -> forces.add(new Vec3d[]{b.atM(), flow.scale(q * b.frontalAreaM2() * b.dragCoefficient())});
            }
        }
        return sum(forces, flow);
    }

    private static Loads sum(List<Vec3d[]> forces, Vec3d flow) {
        Vec3d force = Vec3d.ZERO, moment = Vec3d.ZERO;
        for (Vec3d[] f : forces) { force = force.add(f[1]); moment = moment.add(f[0].cross(f[1])); }
        return new Loads(force, moment, -force.y(), force.dot(flow));
    }

    /** A floor's own loads driving forward at speedMps (no plain floor taken off), with wheels at its corners or without: for the tests. */
    static Loads floorAlone(Underbody u, double speedMps, double densityKgM3, Heights heights, boolean wheels) {
        List<Vec3d[]> forces = new ArrayList<>();
        underbody(u, 0.5 * densityKgM3 * speedMps * speedMps, heights, forces, wheels, false);
        return sum(forces, new Vec3d(-1, 0, 0));
    }

    // ---------------------------------------------------------------- the floor

    /** What a channel under a floor gives: its pressure coefficient, the diffuser's mean one, the velocity ratio squared. */
    record Channel(double floorCp, double diffuserCp, double velocity2, double inletCp) {}

    /**
     * The floor channel at its heights: front gap, gap at the diffuser's start; the dam's depth, the diffuser's length and
     * angle, the flat part's length, whether it is an undertray (else rough), whether wheels stand at its corners.
     */
    static Channel channel(double frontGapM, double rearGapM, double damM, double diffuserM, double angleDeg, double flatM, boolean smooth, boolean wheels) {
        double hf = Math.max(frontGapM, MIN_GAP), hr = Math.max(rearGapM, MIN_GAP);
        double hIn = Math.max(hf - damM, MIN_GAP), hm = 0.5 * (hf + hr);
        double ratio = hm / hIn;
        double kInlet = K_IN * ratio * ratio + (ratio - 1) * (ratio - 1);   // inlet loss and the expansion behind a dam, on the floor's velocity
        double kWheels = wheels ? K_WHEELS : 0;
        double kFriction = (smooth ? SMOOTH : ROUGH) * Math.max(flatM, 0) / (2 * hm);
        double recovery = 0;
        if (diffuserM > 0 && angleDeg > 0) {
            double ar = (hr + diffuserM * Math.tan(Math.toRadians(angleDeg))) / hr;
            double eta = ETA_DIFFUSER / (1 + Math.pow(angleDeg / THETA_STALL, 4)) / (1 + Math.pow((ar - 1) / AR_STALL, 3));
            if (wheels) { eta *= WHEEL_RECOVERY; }   // the rear wheels' wakes in it
            if (!smooth) { eta *= ROUGH_RECOVERY; }   // a thick, torn boundary layer at its inlet
            recovery = eta * (1 - 1 / (ar * ar));
        }
        double v2 = (1 - C_PB) / (1 + kInlet + kWheels + kFriction - recovery);
        double floor = C_PB - recovery * v2;
        double inlet = Math.max(1 - (1 + K_IN) * v2 * ratio * ratio, -3);
        return new Channel(floor, 0.5 * (floor + C_PB), v2, inlet);
    }

    /**
     * The floor's loads into out: the kit's floor, less a plain floor's (lessPlain; else the kit's floor alone, for the
     * tests), with the wheels at its corners or without (the tests' wheel-less body).
     */
    static void underbody(Underbody u, double q, Heights heights, List<Vec3d[]> out, boolean wheels, boolean lessPlain) {
        double xd = u.diffuserStartXM(), flat = u.frontXM() - xd, total = u.frontXM() - u.rearXM();
        double hf = heights.heightM(new Vec3d(u.frontXM(), u.floorYM(), 0)), hr = heights.heightM(new Vec3d(xd, u.floorYM(), 0));
        double fade = fade(0.5 * (hf + hr));
        if (fade <= 0) { return; }
        double b = u.widthM(), qf = q * fade;
        Vec3d flatAt = new Vec3d(0.5 * (u.frontXM() + xd), u.floorYM(), 0), diffuserAt = new Vec3d(0.5 * (xd + u.rearXM()), u.floorYM(), 0);
        // the kit's floor
        boolean diffuser = u.diffuserLengthM() > 0 && u.diffuserAngleDeg() > 0;
        Channel kit = channel(hf, hr, u.damDepthM(), u.diffuserLengthM(), u.diffuserAngleDeg(), flat, u.smooth(), wheels);
        double sealed = b / (b + LEAK * total * (1 - u.sealing()));   // the skirts close that share of the sides' leak
        double lambda = u.smooth() ? SMOOTH : ROUGH, rubbed = diffuser ? flat : total;   // the ramp's slowing flow rubs little
        out.add(new Vec3d[]{flatAt, new Vec3d(-qf * 0.25 * lambda * kit.velocity2() * b * rubbed, qf * sealed * kit.floorCp() * b * flat, 0)});
        if (diffuser) { out.add(new Vec3d[]{diffuserAt, new Vec3d(0, qf * sealed * kit.diffuserCp() * b * u.diffuserLengthM(), 0)}); }
        else if (u.diffuserLengthM() > 0) { out.add(new Vec3d[]{diffuserAt, new Vec3d(0, qf * sealed * kit.floorCp() * b * u.diffuserLengthM(), 0)}); }
        if (u.damDepthM() > 0) {
            out.add(new Vec3d[]{new Vec3d(u.frontXM(), u.floorYM() - 0.5 * u.damDepthM(), 0), new Vec3d(-qf * CD_DAM * b * u.damDepthM(), 0, 0)});
            // the separated zone behind it, in place of the floor's suction there (never less suction than the floor's)
            double reach = Math.min(DAM_REACH * u.damDepthM(), flat), zone = Math.min(DAM_WAKE - sealed * kit.floorCp(), 0);
            out.add(new Vec3d[]{new Vec3d(u.frontXM() - 0.5 * reach, u.floorYM(), 0), new Vec3d(0, qf * zone * b * reach, 0)});
        }
        if (u.splitterLengthM() > 0 && u.splitterWidthM() > 0) {
            double area = u.splitterLengthM() * u.splitterWidthM();
            out.add(new Vec3d[]{new Vec3d(u.frontXM() + 0.5 * u.splitterLengthM(), u.floorYM() - u.damDepthM(), 0),
                    new Vec3d(0, qf * area * (kit.inletCp() - SPLITTER_TOP), 0)});
        }
        if (!lessPlain) { return; }
        // less a plain floor's
        Channel plain = channel(hf, hr, 0, 0, 0, total, false, wheels);
        double open = b / (b + LEAK * total);
        Vec3d plainAt = new Vec3d(0.5 * (u.frontXM() + u.rearXM()), u.floorYM(), 0);
        out.add(new Vec3d[]{plainAt, new Vec3d(qf * 0.25 * ROUGH * plain.velocity2() * b * total, -qf * open * plain.floorCp() * b * total, 0)});
    }

    /** The floor loads' share at a ride height: 1 up to FADE_FROM, none from FADE_TO. */
    static double fade(double heightM) {
        if (!(heightM < FADE_TO)) { return 0; }
        return heightM <= FADE_FROM ? 1 : (FADE_TO - heightM) / (FADE_TO - FADE_FROM);
    }

    /** The induced velocities' factor of end plates heightM tall on a wing of spanM (Hoerner: aspect ratio times 1 + 1.9 h / b). */
    public static double endplateFactor(double heightM, double spanM) { return 1 / (1 + 1.9 * Math.max(heightM, 0) / spanM); }

    // ---------------------------------------------------------------- the wing

    /** A rear wing's lift (downforce) and drag coefficients against its angle of attack (deg, from TABLE_FROM by TABLE_STEP). */
    static final class WingTable {
        final double[] cl, cd;

        private WingTable(double[] cl, double[] cd) { this.cl = cl; this.cd = cd; }

        static WingTable of(RearWing w) {
            Airfoil section = Airfoil.naca4(w.camber(), 0.4, w.thickness());
            Wing wing = new Wing(new Wing.Planform(w.spanM(), w.chordM(), w.chordM(), 0, 0, 0, 0, true), section, section, List.of());
            double factor = Math.min(1, FlightAtmosphere.inducedDragFactor(w.designHeightM(), w.spanM()) * endplateFactor(w.endplateHeightM(), w.spanM()));
            int n = (int) Math.round((TABLE_TO - TABLE_FROM) / TABLE_STEP) + 1;
            double[] cl = new double[n], cd = new double[n];
            double q = 0.5 * Wing.Air.SEA_LEVEL.densityKgM3() * TABLE_SPEED * TABLE_SPEED * w.areaM2();
            double[] previous = null;
            // from no lift up through the stall, as a tunnel run with rising angle, then down from there
            int zero = (int) Math.round(-TABLE_FROM / TABLE_STEP);
            for (int k = zero; k < n; k++) { previous = point(wing, factor, k, q, cl, cd, previous); }
            previous = null;
            for (int k = zero; k >= 0; k--) { previous = point(wing, factor, k, q, cl, cd, previous); }
            return new WingTable(cl, cd);
        }

        private static double[] point(Wing wing, double factor, int k, double q, double[] cl, double[] cd, double[] previous) {
            double alpha = Math.toRadians(TABLE_FROM + k * TABLE_STEP);
            Wing.Solution s = wing.solve(TABLE_SPEED, alpha, 0, Wing.Air.SEA_LEVEL, null, factor, previous);
            Vec3d lift = new Vec3d(Math.sin(alpha), Math.cos(alpha), 0), along = new Vec3d(-Math.cos(alpha), Math.sin(alpha), 0);
            cl[k] = s.force().dot(lift) / q;
            cd[k] = s.force().dot(along) / q;
            return s.strength();
        }

        /** Coefficients at an angle (deg), held at the table's ends. */
        double[] at(double alphaDeg) {
            double x = (Math.clamp(alphaDeg, TABLE_FROM, TABLE_TO) - TABLE_FROM) / TABLE_STEP;
            int i = Math.min((int) x, cl.length - 2);
            double f = x - i;
            return new double[]{cl[i] + f * (cl[i + 1] - cl[i]), cd[i] + f * (cd[i + 1] - cd[i])};
        }

        /**
         * The inverted wing's force at the air velocity (body frame): its angle of attack is its set angle less the flow's
         * upward angle; the downforce is normal to the flow in the x-y plane, the drag along it.
         */
        void add(RearWing w, Vec3d air, double q, List<Vec3d[]> out) {
            double planar = Math.hypot(air.x(), air.y());
            if (!(planar > 1e-6)) { return; }
            double flowUp = Math.toDegrees(Math.atan2(air.y(), -air.x()));   // air rising past the wing
            double[] c = at(w.angleDeg() - flowUp);
            Vec3d along = new Vec3d(air.x() / planar, air.y() / planar, 0), up = new Vec3d(along.y(), -along.x(), 0);
            double s = q * w.areaM2();
            out.add(new Vec3d[]{w.quarterChordM(), up.scale(-c[0] * s).add(air.scale(c[1] * s / air.length()))});
        }
    }
}
