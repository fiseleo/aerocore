package dev.mcmod.aerocore.neoforge.mixin;

import it.unimi.dsi.fastutil.longs.Long2ObjectLinkedOpenHashMap;
import net.minecraft.server.level.ChunkHolder;
import net.minecraft.server.level.ChunkMap;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Drops a stale chunk-unload task instead of letting it requeue itself forever, which hung the server while saving on
 * shutdown (vanilla 1.21.1; seen 2026-10-19 after a teleport and a fast vehicle: "Saving worlds" for 20 minutes).
 * <p>
 * Vanilla: {@code processUnloads} moves a dropped holder into {@code pendingUnloads} and calls {@code scheduleUnload},
 * whose task runs from {@code unloadQueue} on the server thread. If the holder is not ready for saving, the task calls
 * {@code scheduleUnload} again; with the save future already done that queues it again at once. A ticket can take the
 * same holder back into {@code updatingChunkMap} ({@code updateChunkScheduling} removes it from {@code pendingUnloads})
 * while its task is still queued, and a neighbour's generation then holds a generation reference on it. That task now
 * requeues itself on every run. In a normal tick the queue loop only burns the tick's spare time, but on shutdown
 * {@code processUnloads(() -> true)} never leaves the loop, and the generation that would release the reference needs
 * the server thread: a livelock, and the world is never saved.
 * <p>
 * Why dropping it changes nothing: the only call that can reach here with a holder that is not the one pending at its
 * position is that requeue from the not-ready branch ({@code processUnloads} puts the holder into
 * {@code pendingUnloads} right before its own call). A holder only enters {@code pendingUnloads} through
 * {@code processUnloads}, which always schedules a fresh task for it, and a pending holder gains no generation
 * references ({@code acquireGeneration} reads {@code updatingChunkMap}). So a task whose holder is not pending would,
 * once ready, only fail {@code pendingUnloads.remove(pos, holder)} and do nothing; or, if the holder had been dropped
 * again by then, do the same unload the fresh task does. Everything here runs on the server thread, like vanilla's
 * own reads of {@code pendingUnloads}.
 */
@Mixin(ChunkMap.class)
abstract class ChunkUnloadMixin {
    @Shadow @Final private Long2ObjectLinkedOpenHashMap<ChunkHolder> pendingUnloads;

    @Inject(method = "scheduleUnload(JLnet/minecraft/server/level/ChunkHolder;)V", at = @At("HEAD"), cancellable = true)
    private void aerocore$dropStaleUnload(long pos, ChunkHolder holder, CallbackInfo info) {
        if (pendingUnloads.get(pos) != holder) info.cancel();
    }
}
