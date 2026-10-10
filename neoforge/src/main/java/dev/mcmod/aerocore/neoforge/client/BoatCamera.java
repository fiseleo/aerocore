package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.probe.RowingBoats;
import net.minecraft.client.Minecraft;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.vehicle.Boat;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ViewportEvent;

/**
 * First-person view from a boat rolls with the hull: its roll when looking ahead, its pitch when looking to the side.
 * Only the roll about the line of sight is applied, so the crosshair still points where the player aims.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class BoatCamera {
    private BoatCamera() {}

    @SubscribeEvent public static void angles(ViewportEvent.ComputeCameraAngles event) {
        if (!Minecraft.getInstance().options.getCameraType().isFirstPerson()) { return; }
        Entity viewer = event.getCamera().getEntity();
        if (viewer == null || !(viewer.getVehicle() instanceof Boat boat)) { return; }
        event.setRoll(event.getRoll() + RowingBoats.cameraRoll(boat, event.getYaw(), (float) event.getPartialTick()));
    }
}
