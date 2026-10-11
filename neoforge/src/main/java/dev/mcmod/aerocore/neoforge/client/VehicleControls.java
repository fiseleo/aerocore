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
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class VehicleControls {
    private static Map<String, Double> last = Map.of();
    private static int quiet;

    private VehicleControls() {}

    @SubscribeEvent public static void tick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.player == null || !(minecraft.player.getVehicle() instanceof PartVehicle vehicle) || !vehicle.usesDefaultControls()
                || vehicle.getFirstPassenger() != minecraft.player) {
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
