package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.ContactMaterial;
import net.minecraft.tags.BlockTags;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.state.BlockState;

/**
 * Contact material of a steel body against a block, by material class. Minecraft keeps no material data, so blocks are
 * classified by their tags and sound type (a game-data proxy for what they are made of): ice, wood, glass, metal,
 * sand, gravel, snow, wool, mud, soil; everything else (stone, brick, concrete, terracotta, ores ...) counts as rock.
 * Coefficients and their sources are in ContactMaterial. Frictional heating still splits heat as for granite.
 */
public final class BlockMaterials {
    private BlockMaterials() {}

    public static ContactMaterial of(BlockState state) {
        if (state.is(BlockTags.ICE)) { return ContactMaterial.STEEL_ON_ICE; }
        SoundType sound = state.getSoundType();
        if (sound == SoundType.MUD) { return ContactMaterial.STEEL_ON_MUD; }
        if (state.is(BlockTags.LOGS) || state.is(BlockTags.PLANKS) || sound == SoundType.WOOD || sound == SoundType.CHERRY_WOOD
                || sound == SoundType.BAMBOO_WOOD || sound == SoundType.NETHER_WOOD || sound == SoundType.LADDER || sound == SoundType.SCAFFOLDING
                || sound == SoundType.BAMBOO) { return ContactMaterial.STEEL_ON_WOOD; }
        if (sound == SoundType.GLASS) { return ContactMaterial.STEEL_ON_GLASS; }
        if (sound == SoundType.METAL || sound == SoundType.COPPER || sound == SoundType.ANVIL || sound == SoundType.CHAIN
                || sound == SoundType.NETHERITE_BLOCK || sound == SoundType.LANTERN) { return ContactMaterial.STEEL_ON_STEEL; }
        if (sound == SoundType.SAND || sound == SoundType.SOUL_SAND || state.is(BlockTags.SAND)) { return ContactMaterial.STEEL_ON_SAND; }
        if (sound == SoundType.GRAVEL) { return ContactMaterial.STEEL_ON_GRAVEL; }
        if (sound == SoundType.SNOW || sound == SoundType.POWDER_SNOW || state.is(BlockTags.SNOW)) { return ContactMaterial.STEEL_ON_SNOW; }
        if (sound == SoundType.WOOL || state.is(BlockTags.WOOL) || state.is(BlockTags.WOOL_CARPETS)) { return ContactMaterial.STEEL_ON_WOOL; }
        if (state.is(BlockTags.DIRT) || sound == SoundType.GRASS || sound == SoundType.ROOTED_DIRT || sound == SoundType.MOSS
                || sound == SoundType.SOUL_SOIL) { return ContactMaterial.STEEL_ON_SOIL; }
        return ContactMaterial.STEEL_ON_ROCK;
    }
}
