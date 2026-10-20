package dev.mcmod.aerocore.neoforge.mixin;

import it.unimi.dsi.fastutil.longs.Long2ObjectLinkedOpenHashMap;
import net.minecraft.server.level.ChunkHolder;
import net.minecraft.server.level.ChunkMap;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;
import org.spongepowered.asm.mixin.gen.Invoker;

import java.util.Queue;
import java.util.function.BooleanSupplier;

/** Vanilla's chunk-unload bookkeeping, for the GameTest that replays a stale unload task (ChunkUnloadMixin). */
@Mixin(ChunkMap.class)
public interface ChunkMapAccessor {
    @Invoker("processUnloads") void aerocore$processUnloads(BooleanSupplier hasMoreTime);
    @Invoker("getUpdatingChunkIfPresent") ChunkHolder aerocore$updatingChunk(long pos);
    @Accessor("pendingUnloads") Long2ObjectLinkedOpenHashMap<ChunkHolder> aerocore$pendingUnloads();
    @Accessor("unloadQueue") Queue<Runnable> aerocore$unloadQueue();
}
