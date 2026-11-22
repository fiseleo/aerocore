package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.vehicle.PartVehicle;
import dev.mcmod.aerocore.neoforge.vehicle.VehicleNetwork;
import net.minecraft.client.Minecraft;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.Map;

/**
 * AeroCore's own key mapping for part-built vehicles that ask for it (PartVehicle.usesDefaultControls): while the
 * player drives, forward / back set "throttle" to +1 / -1 and left / right set "rudder" to -1 / +1 (positive turns to
 * starboard). Sent when they change and once a second. A vehicle mod with its own controls turns this off.
 * <p>
 * 0.43: a vehicle mod may also turn it off for a while on the client (a machine in work mode, whose movement keys are
 * joysticks): {@link #suppress} adds a test; while any says so for the vehicle, nothing is sent.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class VehicleControls {
    private static Map<String, Double> last = Map.of();
    private static int quiet;

    private static final java.util.List<java.util.function.Predicate<PartVehicle>> SUPPRESSED = new java.util.concurrent.CopyOnWriteArrayList<>();

    private VehicleControls() {}

    /** 0.43: adds a test that keeps this mapping quiet for a vehicle while it says so (checked each client tick). */
    public static void suppress(java.util.function.Predicate<PartVehicle> test) { SUPPRESSED.add(test); }

    private static boolean suppressed(PartVehicle v) {
        for (java.util.function.Predicate<PartVehicle> t : SUPPRESSED) { if (t.test(v)) { return true; } }
        return false;
    }

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.player == null || !(minecraft.player.getVehicle() instanceof PartVehicle vehicle) || !vehicle.usesDefaultControls()
                || vehicle.getFirstPassenger() != minecraft.player || suppressed(vehicle)) {
            last = Map.of();
            return;
        }
        var options = minecraft.options;
        double throttle = (options.keyUp.isDown() ? 1 : 0) - (options.keyDown.isDown() ? 1 : 0);
        double rudder = (options.keyRight.isDown() ? 1 : 0) - (options.keyLeft.isDown() ? 1 : 0);
        Map<String, Double> now = Map.of("throttle", throttle, "rudder", rudder);
        if (!now.equals(last) || ++quiet >= 20) {
            VehicleNetwork.send(vehicle, now);
            last = now;
            quiet = 0;
        }
    }
}
