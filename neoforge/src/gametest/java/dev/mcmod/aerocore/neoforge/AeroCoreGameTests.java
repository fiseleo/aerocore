package dev.mcmod.aerocore.neoforge;

import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.RegisterGameTestsEvent;

/**
 * Registers AeroCore's GameTests (0.41: they live in the gametest source set, which only the development runs load, so
 * the release JAR carries none of them or their structures). Only the GameTest run sets aerocore.gametests;
 * ChunkUnloadGameTests registers itself by its annotation.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, bus = EventBusSubscriber.Bus.MOD)
public final class AeroCoreGameTests {
    private AeroCoreGameTests() {}

    @SubscribeEvent static void register(RegisterGameTestsEvent event) {
        if (!Boolean.getBoolean("aerocore.gametests")) { return; }
        event.register(dev.mcmod.aerocore.neoforge.probe.ProbeGameTests.class);
        event.register(dev.mcmod.aerocore.neoforge.probe.PlatformGameTests.class);
        event.register(dev.mcmod.aerocore.neoforge.probe.HullGameTests.class);
        event.register(dev.mcmod.aerocore.neoforge.probe.SeaGameTests.class);
        event.register(dev.mcmod.aerocore.neoforge.vehicle.VehicleGameTests.class);
    }
}
