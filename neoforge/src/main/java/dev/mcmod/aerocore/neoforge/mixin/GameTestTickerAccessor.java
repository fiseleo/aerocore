package dev.mcmod.aerocore.neoforge.mixin;

import net.minecraft.gametest.framework.GameTestInfo;
import net.minecraft.gametest.framework.GameTestTicker;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

import java.util.Collection;

/** The running GameTests, so that a body whose physics fails in a GameTest run fails its test (BodyWorld). */
@Mixin(GameTestTicker.class)
public interface GameTestTickerAccessor {
    @Accessor("testInfos") Collection<GameTestInfo> aerocore$testInfos();
}
