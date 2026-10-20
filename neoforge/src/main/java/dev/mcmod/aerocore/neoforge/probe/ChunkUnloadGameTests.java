package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.neoforge.mixin.ChunkMapAccessor;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.server.level.ChunkHolder;
import net.minecraft.server.level.ChunkMap;
import net.minecraft.server.level.ServerChunkCache;
import net.minecraft.server.level.TicketType;
import net.minecraft.world.level.ChunkPos;
import net.neoforged.neoforge.gametest.GameTestHolder;
import net.neoforged.neoforge.gametest.PrefixGameTestTemplate;

import java.util.Comparator;
import java.util.concurrent.locks.LockSupport;

/**
 * Replays the stale chunk-unload task that hung the server while saving on shutdown (ChunkUnloadMixin): a chunk is
 * dropped and its unload task queued, a ticket takes the same holder back, a neighbour's generation holds it, then the
 * unload queue is drained the way shutdown drains it. Vanilla requeues the task on every run, so the drain only stops at
 * the poll cap here; with the mixin the queue empties, and the chunk still unloads once its ticket goes.
 * Registered by its {@link GameTestHolder} annotation; loaded only by the explicit AeroCore GameTest run.
 */
@GameTestHolder("aerocore")
@PrefixGameTestTemplate(false)
public final class ChunkUnloadGameTests {
    private ChunkUnloadGameTests() {}

    private static final TicketType<ChunkPos> TICKET = TicketType.create("aerocore_unload_test", Comparator.comparingLong(ChunkPos::toLong));
    /** The vanilla drain never ends on its own: it is capped so that a regression fails the test instead of hanging the run. */
    private static final int POLL_CAP = 100_000;

    @GameTest(template = "probe_empty", timeoutTicks = 600)
    public static void staleUnloadTaskDoesNotRequeueForever(GameTestHelper helper) {
        ServerChunkCache chunks = helper.getLevel().getChunkSource();
        ChunkMapAccessor map = (ChunkMapAccessor) chunks.chunkMap;
        ChunkPos test = new ChunkPos(helper.absolutePos(BlockPos.ZERO));
        // 1 km from the test structures, so that only this ticket keeps it loaded.
        ChunkPos pos = new ChunkPos(test.x + 64, test.z + 64);
        long key = pos.toLong();
        chunks.addRegionTicket(TICKET, pos, 0, pos);
        helper.startSequence()
            .thenWaitUntil(() -> {
                ChunkHolder holder = map.aerocore$updatingChunk(key);
                helper.assertTrue(holder != null && chunks.getChunkNow(pos.x, pos.z) != null && holder.isReadyForSaving(), "Chunk " + pos + " is not loaded and settled yet");
            })
            .thenExecute(() -> replay(helper, chunks, map, pos))
            .thenWaitUntil(() -> helper.assertTrue(map.aerocore$updatingChunk(key) == null && !map.aerocore$pendingUnloads().containsKey(key), "Chunk " + pos + " must still unload after its ticket goes"))
            .thenSucceed();
    }

    /** All in one tick on the server thread, so that the server's own unload pass cannot run the queued task first. */
    private static void replay(GameTestHelper helper, ServerChunkCache chunks, ChunkMapAccessor map, ChunkPos pos) {
        ChunkMap chunkMap = chunks.chunkMap;
        long key = pos.toLong();
        ChunkHolder holder = map.aerocore$updatingChunk(key);
        // The ticket goes: the holder waits in pendingUnloads and its unload task is queued (its save future is done, so
        // at once). With no time left, processUnloads moves up to 200 dropped holders a call and runs no queued task.
        chunks.removeRegionTicket(TICKET, pos, 0, pos);
        chunkMap.getDistanceManager().runAllUpdates(chunkMap);
        for (int i = 0; i < 20 && map.aerocore$pendingUnloads().get(key) != holder; i++) map.aerocore$processUnloads(() -> false);
        helper.assertTrue(map.aerocore$pendingUnloads().get(key) == holder, "The dropped holder must wait in pendingUnloads");
        helper.assertTrue(!map.aerocore$unloadQueue().isEmpty(), "Its unload task must be queued");
        // A ticket takes the same holder back before the task runs (a teleport back, a fast vehicle's keep-ahead ticket).
        chunks.addRegionTicket(TICKET, pos, 0, pos);
        chunkMap.getDistanceManager().runAllUpdates(chunkMap);
        helper.assertTrue(map.aerocore$updatingChunk(key) == holder && !map.aerocore$pendingUnloads().containsKey(key), "The ticket must take the same holder back");
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (!holder.getSaveSyncFuture().isDone()) {
            helper.assertTrue(System.nanoTime() < deadline, "Chunk " + pos + " did not become full again");
            if (!chunks.pollTask()) LockSupport.parkNanos(100_000);
        }
        // A neighbour's generation holds it, and the queue is drained the way shutdown drains it (always more time).
        holder.increaseGenerationRefCount();
        int[] polls = {0};
        try {
            map.aerocore$processUnloads(() -> ++polls[0] < POLL_CAP);
        } finally {
            holder.decreaseGenerationRefCount();
            chunks.removeRegionTicket(TICKET, pos, 0, pos);
        }
        helper.assertTrue(map.aerocore$unloadQueue().isEmpty(), "A stale unload task must not requeue itself: " + map.aerocore$unloadQueue().size() + " still queued after " + polls[0] + " polls");
    }
}
