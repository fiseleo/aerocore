package dev.mcmod.aerocore.neoforge.client;

import dev.mcmod.aerocore.neoforge.AeroCore;
import net.minecraft.client.Minecraft;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.RenderLevelStageEvent;

/**
 * 0.30: draws what world renderers asked to draw late (PartLayers.late: glass) after the world's translucent blocks,
 * so water and other translucent blocks behind a window show through it. Entities' poses are relative to the camera,
 * as what this stage draws is, so the recorded vertices draw as they are.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID, value = Dist.CLIENT)
public final class PartLate {
    private PartLate() {}

    @SubscribeEvent public static void render(RenderLevelStageEvent event) {
        if (event.getStage() == RenderLevelStageEvent.Stage.AFTER_TRANSLUCENT_BLOCKS) {
            PartLayers.flushLate(Minecraft.getInstance().renderBuffers().bufferSource());
            FieldLamps.draw(Minecraft.getInstance().renderBuffers().bufferSource(), event.getCamera(), event.getProjectionMatrix());   // 0.38
        } else if (event.getStage() == RenderLevelStageEvent.Stage.AFTER_LEVEL) {
            PartLayers.dropLate();   // nothing left from a frame that drew no translucent blocks
        }
    }
}
