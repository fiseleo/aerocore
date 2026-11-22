package dev.mcmod.aerocore.neoforge.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.hydraulic.Motor;
import dev.mcmod.aerocore.api.vehicle.Binding;
import dev.mcmod.aerocore.api.vehicle.Coupling;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.Fields;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Mount;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartTypes;
import dev.mcmod.aerocore.api.vehicle.VehicleBlueprint;
import dev.mcmod.aerocore.api.vehicle.part.LinkPart;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.List;
import java.util.Map;

/** 0.43: joints and ropes of part-built vehicles in the world (PartVehicle's pose, its DISPLAY "j", the saved joints, reeved ropes). */
@PrefixGameTestTemplate(false)
public final class MachineryGameTests {
    static final String ARM = "aerocore:gametest_arm", POWER = "aerocore:gametest_power", PUMP = "aerocore:gametest_pump";
    static {
        PartTypes.register(ARM, f -> new Arm(f.string("name")));
        PartTypes.register(POWER, f -> new Power(f.string("name")));
        PartTypes.register(PUMP, f -> new PumpUnit(f.string("name")));
    }

    /** A 100 kW electric motor on "throttle" (the machine's engine). */
    record Power(String name) implements PartDesign {
        @Override public String type() { return POWER; }
        @Override public List<DesignIssue> check() { return List.of(); }
        @Override public Map<String, Object> parameters() { return Fields.object(); }
        @Override public Part compile() {
            return new dev.mcmod.aerocore.api.vehicle.part.MotorPart(this, new dev.mcmod.aerocore.api.ElectricMotor(600, 100e3, 2000), 0.5, 0.95,
                    new MassProperties(200, Vec3d.ZERO, 5, 5, 5, 0, 0, 0), List.of());
        }
    }

    /** A 100 cc/rev pressure-compensated pump at 300 bar on circuit "main". */
    record PumpUnit(String name) implements PartDesign {
        @Override public String type() { return PUMP; }
        @Override public List<DesignIssue> check() { return List.of(); }
        @Override public Map<String, Object> parameters() { return Fields.object(); }
        @Override public Part compile() {
            return new dev.mcmod.aerocore.api.vehicle.part.HydraulicPumpPart(this, dev.mcmod.aerocore.api.hydraulic.Pump.axialPiston(100e-6,
                    dev.mcmod.aerocore.api.hydraulic.Pump.Control.PRESSURE_COMPENSATED, 300e5, 0), "main", Vec3d.X, 0.02, MassProperties.point(50, Vec3d.ZERO), List.of());
        }
    }

    private MachineryGameTests() {}

    /** A 300 kg, 3 m arm on a slewing joint about y (a hydraulic swing motor on circuit "main"). */
    record Arm(String name) implements PartDesign {
        @Override public String type() { return ARM; }
        @Override public List<DesignIssue> check() { return List.of(); }
        @Override public Map<String, Object> parameters() { return Fields.object(); }
        @Override public Part compile() {
            MassProperties m = new MassProperties(300, new Vec3d(1.5, 0, 0), 5, 230, 230, 0, 0, 0);
            LinkPart.Slew slew = new LinkPart.Slew(Motor.axialPiston(50e-6), 100, 0.95, "main", 300e5, 2e-3, 2e5);
            return new LinkPart(this, m, List.of(new dev.mcmod.aerocore.api.CollisionSphere(new Vec3d(3, 0, 0), 0.2)), Map.of("tip", Mount.at(new Vec3d(3, 0, 0))),
                    Part.Joint.revolute(Vec3d.ZERO, Vec3d.Y, -Math.PI, Math.PI, 0), slew, List.of(), List.of(), List.of(), List.of());
        }
    }

    private static PartVehicle machine(GameTestHelper h, double x) {
        VehicleBlueprint blueprint = new VehicleBlueprint("test machine", List.of(
                new VehicleBlueprint.Entry("ball", new VehicleGameTests.Ball("ball", 0.6, 3000), Mount.ORIGIN, List.of(Binding.of("fail", "fail"))),
                new VehicleBlueprint.Entry("arm", new Arm("arm"), Mount.at(new Vec3d(0, 0.8, 0)), List.of(Binding.of("command", "swing"))),
                new VehicleBlueprint.Entry("engine", new Power("engine"), Mount.at(new Vec3d(-0.3, 0.2, 0)), List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("pump", new PumpUnit("pump"), Mount.at(new Vec3d(-0.3, 0.2, 0.3)))),
                List.of(dev.mcmod.aerocore.api.vehicle.Connection.Shaft.ports("engine", "", "pump", "input", 1, 0.98)));
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        PartVehicle v = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
        if (v == null) { throw new IllegalStateException("Vehicle factory failed"); }
        v.setBlueprint(blueprint, new Vec3d(o.getX() + x, o.getY() + 1.6, o.getZ() + 8), Orientation.IDENTITY);
        h.getLevel().addFreshEntity(v);
        v.setControl("throttle", 1);
        return v;
    }

    /** The arm swings on its slewing joint when commanded and holds when not; the joint is saved and its pose moves the arm's sphere. */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120)
    public static void aSlewingArmSwingsHoldsAndIsSaved(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        PartVehicle v = machine(h, 10);
        final double[] at = {0};
        h.runAfterDelay(20, () -> v.setControl("swing", 1));
        h.runAfterDelay(40, () -> {
            v.setControl("swing", 0);
            double q = v.vehicleState().jointValue(0);
            h.assertTrue(q > 0.05, "the arm swung: " + q);
            at[0] = q;
            Vec3d tip = v.pose().collision().get(v.pose().collision().size() - 1).centerLocal();
            h.assertTrue(Math.abs(tip.z()) > 0.1, "its sphere swings with it: " + tip);
        });
        h.runAfterDelay(60, () -> {
            double q = v.vehicleState().jointValue(0);
            h.assertTrue(Math.abs(q - at[0]) < 0.05, "the swing brake stops and holds it: " + at[0] + " then " + q);
            CompoundTag tag = new CompoundTag();
            v.saveWithoutId(tag);
            PartVehicle copy = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
            if (copy == null) { throw new IllegalStateException("Vehicle factory failed"); }
            copy.load(tag);
            h.assertTrue(copy.vehicleState() != null && Math.abs(copy.vehicleState().jointValue(0) - q) < 1e-9, "the joint comes back from the save");
            h.succeed();
        });
    }

    /** A ball dropped on a 3 m rope from a heavy anchor stops hanging 3 m under it and swings no further away. */
    @GameTest(templateNamespace = "aerocore", template = "hull_pool", timeoutTicks = 120)
    public static void aRopeCatchesItsLoadAtItsLength(GameTestHelper h) {
        for (int x = 0; x < 24; x++) for (int z = 0; z < 16; z++) { h.setBlock(new BlockPos(x, 0, z), Blocks.STONE); }
        PartVehicle anchor = machine(h, 6);
        BlockPos o = h.absolutePos(BlockPos.ZERO);
        VehicleBlueprint ball = new VehicleBlueprint("test load", List.of(new VehicleBlueprint.Entry("ball", new VehicleGameTests.Ball("ball", 0.2, 50), Mount.ORIGIN)), List.of());
        PartVehicle load = VehicleRegistry.PART_VEHICLE.get().create(h.getLevel());
        if (load == null) { throw new IllegalStateException("Vehicle factory failed"); }
        load.setBlueprint(ball, new Vec3d(o.getX() + 6, o.getY() + 7, o.getZ() + 8), Orientation.IDENTITY);
        h.getLevel().addFreshEntity(load);
        // the rope from 5 m above the anchor's centre (it rests 1.6 m up) to the load: it hangs 3 m below that, at y = o + 3.6
        Coupling rope = Coupling.rope(new Vec3d(0, 5, 0), Vec3d.ZERO, 3, 1e6, 8e3).withCapture(10);   // E A / L 3.3e5 N/m, about critically damped for 50 kg
        h.runAfterDelay(5, () -> h.assertTrue(anchor.couple(load, rope) == Couplings.Result.OK, "hooked on"));
        h.runAfterDelay(80, () -> {
            Vec3d point = anchor.vehicleState().body().worldPoint(anchor.vehicleState().properties(), new Vec3d(0, 5, 0));
            double hang = point.subtract(load.vehicleState().body().position()).length();
            h.assertTrue(Math.abs(hang - 3) < 0.05, "it hangs at the rope's length: " + hang);
            double pull = load.couplingForceN().length();
            // the anchor is a ball that rocks under the pull, so the load swings: the rope pulls, never pushes, and no more than a swing can ask
            h.assertTrue(pull > 0 && pull < 3 * 50 * 9.8, "the rope pulls on the load: " + pull);
            h.succeed();
        });
    }
}
