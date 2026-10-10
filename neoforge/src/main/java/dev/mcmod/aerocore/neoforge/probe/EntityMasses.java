package dev.mcmod.aerocore.neoforge.probe;

import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.EntityType;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.phys.AABB;

import java.util.IdentityHashMap;
import java.util.Map;

/**
 * Masses of vanilla entities, which Minecraft does not track. Real animals take published adult masses: people 62 kg
 * (global adult mean, Walpole et al. 2012, BMC Public Health 12:439), cattle 612 kg (beef cow, Scasta et al. 2019),
 * pig 131 kg (US market hog, USDA NASS 2025), sheep 71 kg (ewe, NMSU Circular 685), chicken 2.0 kg (brown layer,
 * Hy-Line), horse 500 kg (Merck Vet Manual), donkey 165 kg (Donkey Sanctuary; Gichure 2020: 161), mule 360 kg
 * (Tadich et al. 2025), llama 140 kg, goat 45 kg, wolf 50 kg, cat 4.5 kg, fox 6 kg, rabbit 2.0 kg, polar bear 345 kg
 * (mean of male 470 and female 220), panda 100 kg, camel 450 kg, armadillo 5.5 kg, axolotl 0.085 kg, frog 0.2 kg
 * (American bullfrog), parrot 1.1 kg (scarlet macaw) and bee 1e-4 kg (Animal Diversity Web unless stated),
 * green turtle 150 kg, bottlenose dolphin 300 kg and Atlantic salmon 4.5 kg (NOAA Fisheries), Atlantic cod 2.5 kg
 * (FAO via VIMS). Humanoid mobs take the human mass. Squid and every fictional mob (creepers, slimes, golems ...) and
 * dropped items have no real counterpart: they are given the human density per bounding-box volume of the player's
 * box, 62 kg / (0.6 x 1.8 x 0.6 m^3) = 95.7 kg/m^3, which is a game-scale rule rather than data.
 */
public final class EntityMasses {
    public static final double HUMAN_KG = 62.0;
    /** Human mass over the player's bounding-box volume, kg/m^3. */
    public static final double HITBOX_DENSITY = HUMAN_KG / (0.6 * 1.8 * 0.6);
    private static final Map<EntityType<?>, Double> MASSES = new IdentityHashMap<>();
    static {
        put(HUMAN_KG, EntityType.VILLAGER, EntityType.WANDERING_TRADER, EntityType.ZOMBIE, EntityType.ZOMBIE_VILLAGER, EntityType.HUSK,
                EntityType.DROWNED, EntityType.SKELETON, EntityType.STRAY, EntityType.BOGGED, EntityType.PILLAGER, EntityType.VINDICATOR,
                EntityType.EVOKER, EntityType.ILLUSIONER, EntityType.WITCH, EntityType.PIGLIN, EntityType.PIGLIN_BRUTE, EntityType.ZOMBIFIED_PIGLIN);
        put(612, EntityType.COW, EntityType.MOOSHROOM);
        put(131, EntityType.PIG);
        put(71, EntityType.SHEEP);
        put(2.0, EntityType.CHICKEN, EntityType.RABBIT);
        put(500, EntityType.HORSE, EntityType.SKELETON_HORSE, EntityType.ZOMBIE_HORSE);
        put(165, EntityType.DONKEY);
        put(360, EntityType.MULE);
        put(140, EntityType.LLAMA, EntityType.TRADER_LLAMA);
        put(45, EntityType.GOAT);
        put(50, EntityType.WOLF);
        put(4.5, EntityType.CAT, EntityType.OCELOT, EntityType.SALMON);
        put(6, EntityType.FOX);
        put(345, EntityType.POLAR_BEAR);
        put(100, EntityType.PANDA);
        put(450, EntityType.CAMEL);
        put(5.5, EntityType.ARMADILLO);
        put(0.085, EntityType.AXOLOTL);
        put(0.2, EntityType.FROG);
        put(1.1, EntityType.PARROT);
        put(1e-4, EntityType.BEE);
        put(150, EntityType.TURTLE);
        put(300, EntityType.DOLPHIN);
        put(2.5, EntityType.COD);
    }
    private EntityMasses() {}

    private static void put(double kg, EntityType<?>... types) { for (EntityType<?> type : types) { MASSES.put(type, kg); } }

    /** Mass in kg. */
    public static double massKg(Entity entity) {
        if (entity instanceof Player) { return HUMAN_KG; }
        Double known = MASSES.get(entity.getType());
        if (known != null) { return known; }
        AABB box = entity.getBoundingBox();
        return Math.max(1e-3, HITBOX_DENSITY * box.getXsize() * box.getYsize() * box.getZsize());
    }
}
