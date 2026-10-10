package dev.mcmod.aerocore.neoforge.probe;

import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import dev.mcmod.aerocore.api.GroundContact;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.core.BlockPos;
import net.minecraft.network.chat.Component;
import net.minecraft.tags.BlockTags;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.state.BlockState;

import java.util.Locale;

/**
 * Which GroundContact surface a block is for wheels: ice, snow, sand, mud, soil (dirt, grass, moss: firm clay), gravel,
 * and everything else hard (stone, concrete, wood, metal: tarmac), wet in the rain where the sky reaches it. The
 * mapping by block tags and sound types is a rule. /aerocore ground reports the surface under the caller with a
 * passenger car wheel (load 4 kN, diameter 0.65 m, width 0.2 m) on it.
 */
public final class GroundSurfaces {
    /** The example wheel the command reports: load (N), diameter and width (m). */
    private static final double LOAD = 4000, DIAMETER = 0.65, WIDTH = 0.2;

    private GroundSurfaces() {}

    /** The surface of the block at pos (the block a wheel rests on). */
    public static GroundContact.Surface of(Level level, BlockPos pos) {
        BlockState state = level.getBlockState(pos);
        SoundType sound = state.getSoundType();
        if (state.is(BlockTags.ICE)) { return GroundContact.Surface.ICE; }
        if (state.is(BlockTags.SNOW) || sound == SoundType.SNOW || sound == SoundType.POWDER_SNOW) { return GroundContact.Surface.SNOW; }
        if (state.is(BlockTags.SAND) || sound == SoundType.SAND || sound == SoundType.SOUL_SAND) { return GroundContact.Surface.SAND; }
        if (sound == SoundType.MUD) { return GroundContact.Surface.MUD; }
        if (state.is(BlockTags.DIRT) || sound == SoundType.GRASS || sound == SoundType.ROOTED_DIRT || sound == SoundType.MOSS) { return GroundContact.Surface.SOIL; }
        if (sound == SoundType.GRAVEL) { return GroundContact.Surface.GRAVEL; }
        return level.isRainingAt(pos.above()) ? GroundContact.Surface.WET_TARMAC : GroundContact.Surface.TARMAC;
    }

    public static LiteralArgumentBuilder<CommandSourceStack> tree() {
        return Commands.literal("ground").executes(context -> {
            CommandSourceStack s = context.getSource();
            BlockPos below = BlockPos.containing(s.getPosition()).below();
            GroundContact.Surface surface = of(s.getLevel(), below);
            double resistance = GroundContact.rollingResistance(surface, LOAD, DIAMETER, WIDTH);
            if (surface.soil() != null) {
                GroundContact.Sinkage sink = GroundContact.rigidWheel(surface.soil(), LOAD, DIAMETER, WIDTH);
                s.sendSuccess(() -> Component.translatable("command.aerocore.ground.soft", surface.name(), f(surface.grip().d()), f(resistance / LOAD),
                        f(100 * sink.sinkageM()), f(sink.maxTractionN() / LOAD)), false);
            } else {
                s.sendSuccess(() -> Component.translatable("command.aerocore.ground.firm", surface.name(), f(surface.grip().d()), f(resistance / LOAD)), false);
            }
            return 1;
        });
    }

    private static String f(double v) { return String.format(Locale.ROOT, "%.3f", v); }
}
