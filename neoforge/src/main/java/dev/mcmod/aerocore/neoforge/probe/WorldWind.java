package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.SeaWind;
import dev.mcmod.aerocore.api.SurfaceWind;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WindGusts;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.core.BlockPos;
import net.minecraft.core.HolderLookup;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.network.codec.ByteBufCodecs;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceKey;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.tags.BlockTags;
import net.minecraft.tags.FluidTags;
import net.minecraft.util.Mth;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.levelgen.Heightmap;
import net.minecraft.world.level.saveddata.SavedData;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.entity.player.PlayerEvent;
import net.neoforged.neoforge.event.tick.LevelTickEvent;
import net.neoforged.neoforge.network.PacketDistributor;

import java.util.Map;
import java.util.WeakHashMap;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The wind everywhere in a level, for every reader (AeroCore bodies, vanilla boats, spray, whitecaps, the sky). The
 * level has one reference wind: the 10 m wind over open terrain, its speed following the weather (clear 5, rain 10,
 * thunder 15 m/s, WaveRegion.Wind.WEATHER_DEFAULTS, a rule) times a slowly varying synoptic factor, its direction
 * wandering about a prevailing one (SurfaceWind.Synoptic, seeded by the world seed); or a fixed wind the
 * /aerocore wind command sets (saved). Dimensions without sky light or with a ceiling (the Nether, the End) are calm.
 * The server owns it and sends it to the dimension's players when it changes, so both sides compute the same wind.
 * <p>
 * At a point the reference becomes local (SurfaceWind): the logarithmic profile over the surface roughness, the
 * logarithmic mean of the roughness lengths along the upwind line (weights exp(-s / 64 m)), each column classified from
 * the top of its motion-blocking blocks (MOTION_BLOCKING, the heightmap clients also have): water (sea roughness),
 * leaves over the ground (a forest canopy of that height: d = 0.7 h, z0 = 0.1 h within 0.25-2 m), sand, snow and ice
 * (smooth, 0.005 m), man-made blocks (very rough, 0.5 m), anything else (open, 0.03 m). Heights are above the column's
 * ground or water surface. Hills raise it (Taylor and Lee: hill height and half-length found along the upwind line,
 * the crest share from how far the ground still rises downwind, a rule), and barriers upwind that stand above the point
 * shelter it like fences or tree belts (SeaWind.shelter, Brandle et al.). Chunks that are not loaded count as level
 * ground at the point's height (they are never loaded for the wind); barrier blocks are passed through like air (map
 * makers' invisible walls), and points below their column's ground (under roofs and overhangs, underground) are still
 * air, a rule. The upwind analysis is cached per 4 m cell and
 * 5 degree wind sector for a minute, and the two sectors around the wind are blended.
 * <p>
 * Gusts are the level's WindGusts (Kaimal spectra, frozen turbulence drifting downwind, seeded by the dimension so
 * both sides agree), scaled from the sea's turbulence to the local intensity 1 / ln((z - d) / z0) times the local mean.
 * <p>
 * Vertical air (FlightAtmosphere): ridge lift w = U dh/ds exp(-z / L) from the ground's slope along the wind at the
 * point (over 4 m either side) and the hill's half-length L (50 m without a hill), rising up windward slopes and
 * sinking down lee ones; and thermals over sunlit land, Allen's scaling in a mixed layer of THERMAL_LAYER m with Gedeon's
 * profile, w* from a sensible heat flux of HEAT_FLUX W/m^2 times the sine of the sun's elevation (0 at night, by rain
 * level toward 0 in rain; a rule of thumb for dry land, Stull 1988 gives 100-400 W/m^2 by day), one thermal per cell of
 * Allen's spacing at 100 m (about 320 m), placed in its cell by a hash of the cell, drifting with the wind at 200 m,
 * each living THERMAL_LIFE s (strength sin^2 over its life, phases by hash) and only over land (its column not water).
 * <p>
 * Not modelled: thermal stratification (no lee waves or rotors), the Ekman turning aloft, flow around single
 * buildings, and terrain that changes the wind's direction.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class WorldWind {
    /** A level's reference wind: 10 m speed over open terrain (m/s) and the direction it blows toward (rad, like wave headings). */
    public record Reference(double speedMps, double directionRadians) {
        public static final Reference CALM = new Reference(0, 0);
        public Reference {
            if (!(speedMps >= 0 && speedMps <= SeaWind.MAX_WIND) || !Double.isFinite(directionRadians)) { throw new IllegalArgumentException("Invalid wind"); }
        }
        public double directionX() { return Math.cos(directionRadians); }
        public double directionZ() { return Math.sin(directionRadians); }
    }

    private static final int SYNC_TICKS = 20, SECTORS = 72, CELL = 4, CACHE_LIMIT = 50000, CACHE_TICKS = 1200;
    private static final double SYNC_SPEED = 0.005, SYNC_ANGLE = Math.toRadians(0.5);
    /** Upwind sampling step and reach, the roughness footprint, and the least hill height that counts, m. */
    private static final double STEP = 4, REACH = 256, FOOTPRINT = 64, LEAST_HILL = 4;
    private static final int SAMPLES = (int) (REACH / STEP);
    /** Deepest scan through leaves for the ground below a canopy, blocks. */
    private static final int CANOPY_SCAN = 32;
    private static final boolean GAMETESTS = Boolean.getBoolean("aerocore.gametests");
    /** Midday sensible heat flux over dry land (W/m^2), the mixed layer's depth (m) and a thermal's life (s): rules of thumb. */
    private static final double HEAT_FLUX = 300, THERMAL_LAYER = 1200, THERMAL_LIFE = 1200, LIFT_LENGTH = 50;
    private static final double THERMAL_CELL = dev.mcmod.aerocore.api.FlightAtmosphere.thermalSpacing(100, THERMAL_LAYER);

    private static final Map<ResourceKey<Level>, Reference> CLIENT = new ConcurrentHashMap<>();
    private static final Map<ServerLevel, Reference> SENT = new WeakHashMap<>();
    private static final Map<Level, Map<Long, Upwind>> CACHES = java.util.Collections.synchronizedMap(new WeakHashMap<>());
    private static final Map<ResourceKey<Level>, WindGusts> GUSTS = new ConcurrentHashMap<>();
    private static final Map<ServerLevel, SurfaceWind.Synoptic> SYNOPTIC = java.util.Collections.synchronizedMap(new WeakHashMap<>());

    private WorldWind() {}

    // ---------------------------------------------------------------- reference

    /** The level's reference wind now: the server's own, or on a client the one last received. */
    public static Reference reference(Level level) {
        if (level.isClientSide()) { return CLIENT.getOrDefault(level.dimension(), Reference.CALM); }
        ServerLevel server = (ServerLevel) level;
        Saved saved = Saved.of(server);
        if (saved.override != null) { return saved.override; }
        if (GAMETESTS || !level.dimensionType().hasSkyLight() || level.dimensionType().hasCeiling()) { return Reference.CALM; }
        double[] w = WaveRegion.Wind.WEATHER_DEFAULTS;
        double base = w[0] + (w[1] - w[0]) * level.getRainLevel(1) + (w[2] - w[1]) * level.getThunderLevel(1);
        SurfaceWind.Synoptic synoptic = SYNOPTIC.computeIfAbsent(server, s -> new SurfaceWind.Synoptic(s.getSeed() ^ s.dimension().location().hashCode()));
        double t = level.getGameTime() * MinecraftUnits.TICK_SECONDS;
        return new Reference(Math.clamp(base * synoptic.speedFactor(t), 0, SeaWind.MAX_WIND), synoptic.direction(t));
    }

    /** Fixes the level's wind (null: back to the weather and synoptic wind). */
    public static void setOverride(ServerLevel level, Reference wind) {
        Saved saved = Saved.of(level);
        saved.override = wind;
        saved.setDirty();
        sync(level);
    }

    public static Reference override(ServerLevel level) { return Saved.of(level).override; }

    /** Drops the level's cached terrain analysis (after blocks are placed in a test). */
    static void forget(Level level) { Map<Long, Upwind> cache = CACHES.get(level); if (cache != null) { cache.clear(); } }

    // ---------------------------------------------------------------- local wind

    /** Mean horizontal wind at a point, m/s. */
    public static Vec3d mean(Level level, double x, double y, double z) {
        Reference ref = reference(level);
        if (ref.speedMps() == 0) { return Vec3d.ZERO; }
        double speed = speed(level, ref, x, y, z);
        return new Vec3d(speed * ref.directionX(), 0, speed * ref.directionZ());
    }

    /** Gust at a point at time t (s), m/s (horizontal). */
    public static Vec3d gust(Level level, double x, double y, double z, double t) {
        Reference ref = reference(level);
        if (ref.speedMps() == 0) { return Vec3d.ZERO; }
        Upwind u = upwind(level, x, z, ref.directionRadians());
        double height = y - u.ground, local = speed(level, ref, x, y, z);
        WindGusts gusts = GUSTS.computeIfAbsent(level.dimension(), key -> new WindGusts(key.location().hashCode() * 31L + 7));
        double sea = gusts.deviations(ref.speedMps())[0];
        if (!(sea > 0) || local <= 0) { return Vec3d.ZERO; }
        double scale = SurfaceWind.intensity(height, u.z0, u.displacement) * local / sea;
        double ex = ref.directionX(), ez = ref.directionZ();
        double[] g = new double[2];
        gusts.sample(ref.speedMps(), t, x * ex + z * ez, g);
        return new Vec3d(scale * (g[0] * ex - g[1] * ez), 0, scale * (g[0] * ez + g[1] * ex));
    }

    /**
     * What the wind is at a point, for the wind command: height above the ground or water (m), effective roughness (m),
     * canopy height (m), hill speed-up (fraction), shelter (share kept), mean speed (m/s).
     */
    public record Details(double heightM, double roughnessM, double canopyM, double hillSpeedUp, double shelter, double meanMps) {}

    public static Details describe(Level level, double x, double y, double z) {
        Reference ref = reference(level);
        Upwind u = upwind(level, x, z, ref.directionRadians());
        double height = y - u.ground, share = 1;
        for (int i = 0; i < SAMPLES; i++) {
            double above = u.tops[i] - y;
            if (above > 0.5) { share = Math.min(share, SeaWind.shelter((i + 1) * STEP / above, u.trees[i])); }
        }
        return new Details(height, u.z0, u.canopy, SurfaceWind.hillSpeedUp(u.hillHeight, u.halfLength, height, u.crestShare), share,
                mean(level, x, y, z).length());
    }

    /** Mean wind plus gust at a point, m/s. */
    public static Vec3d at(Level level, double x, double y, double z, double t) {
        return mean(level, x, y, z).add(gust(level, x, y, z, t)).add(new Vec3d(0, vertical(level, x, y, z, t), 0));
    }

    /** Vertical air speed at a point (ridge lift and thermals), m/s. */
    public static double vertical(Level level, double x, double y, double z, double t) {
        Reference ref = reference(level);
        Upwind u = upwind(level, x, z, ref.directionRadians());
        double height = y - u.ground;
        if (height < -0.5) { return 0; }
        double w = 0;
        if (ref.speedMps() > 0 && u.slope != 0) {
            double local = speed(level, ref, x, y, z);
            w += dev.mcmod.aerocore.api.FlightAtmosphere.ridgeLift(local, u.slope, height, u.halfLength > 0 ? u.halfLength : LIFT_LENGTH);
        }
        return w + thermal(level, ref, x, height, z, t);
    }

    /** Thermals at a point at a height above the ground, m/s (see the class notes). */
    private static double thermal(Level level, Reference ref, double x, double height, double z, double t) {
        if (GAMETESTS || !level.dimensionType().hasSkyLight() || level.dimensionType().hasCeiling()) { return 0; }
        double sun = Math.cos(level.getTimeOfDay(1) * 2 * Math.PI) * (1 - level.getRainLevel(1));
        if (!(sun > 0) || height <= 0) { return 0; }
        double wStar = dev.mcmod.aerocore.api.FlightAtmosphere.convectiveVelocity(HEAT_FLUX * sun, THERMAL_LAYER, 300);
        // Cells drift downwind with the wind at 200 m.
        double drift = SurfaceWind.reference(ref.speedMps(), SurfaceWind.TOP_HEIGHT) * t;
        double fx = x - drift * ref.directionX(), fz = z - drift * ref.directionZ();
        int ci = Mth.floor(fx / THERMAL_CELL), cj = Mth.floor(fz / THERMAL_CELL);
        long seed = level.dimension().location().hashCode();
        double w = 0;
        for (int i = ci - 1; i <= ci + 1; i++) for (int j = cj - 1; j <= cj + 1; j++) {
            long h = seed * 0x9E3779B97F4A7C15L + i * 0xC2B2AE3D27D4EB4FL + j * 0x165667B19E3779F9L;
            java.util.SplittableRandom random = new java.util.SplittableRandom(h);
            double tx = (i + 0.2 + 0.6 * random.nextDouble()) * THERMAL_CELL, tz = (j + 0.2 + 0.6 * random.nextDouble()) * THERMAL_CELL;
            double life = (t / THERMAL_LIFE + random.nextDouble()) % 1, strength = Math.pow(Math.sin(Math.PI * life), 2);
            double r = Math.hypot(fx - tx, fz - tz);
            if (r > 3 * dev.mcmod.aerocore.api.FlightAtmosphere.thermalRadius(height, THERMAL_LAYER)) { continue; }
            // The source: land under the thermal's present position.
            Column source = column(level, tx + drift * ref.directionX(), tz + drift * ref.directionZ());
            if (source != null && source.z0() == SurfaceWind.SEA) { continue; }
            w += strength * dev.mcmod.aerocore.api.FlightAtmosphere.thermalUpdraft(wStar, r, height, THERMAL_LAYER);
        }
        return w;
    }

    /** Local mean speed, blending the two 5 degree sectors around the wind's direction. */
    private static double speed(Level level, Reference ref, double x, double y, double z) {
        double sectors = ref.directionRadians() / (2 * Math.PI) * SECTORS, f = sectors - Math.floor(sectors);
        int s0 = Math.floorMod((int) Math.floor(sectors), SECTORS), s1 = (s0 + 1) % SECTORS;
        return (1 - f) * speed(upwind(level, x, z, s0), ref, y) + f * speed(upwind(level, x, z, s1), ref, y);
    }

    private static double speed(Upwind u, Reference ref, double y) {
        double height = y - u.ground;
        // Below the column's ground: under a roof, an overhang or underground (a rule: still air).
        if (height < -0.5) { return 0; }
        double base = SurfaceWind.speed(ref.speedMps(), height, u.z0, u.displacement, u.canopy);
        double hill = SurfaceWind.hillSpeedUp(u.hillHeight, u.halfLength, height, u.crestShare);
        double share = 1;
        for (int i = 0; i < SAMPLES; i++) {
            double above = u.tops[i] - y;
            if (above > 0.5) { share = Math.min(share, SeaWind.shelter((i + 1) * STEP / above, u.trees[i])); }
        }
        return base * (1 + hill) * share;
    }

    // ---------------------------------------------------------------- terrain

    /**
     * What the wind meets at a point for one wind sector: the local ground (or water surface) height, canopy, its
     * displacement, the upwind effective roughness, the hill seen upwind, and the tops of the columns upwind (for
     * shelter), with whether each is trees.
     */
    private record Upwind(double ground, double canopy, double displacement, double z0, double hillHeight, double halfLength,
                          double crestShare, float[] tops, boolean[] trees, double slope, long tick) {}

    /** One column: its ground or water surface height, the top of its blocks including leaves, roughness, canopy height. */
    private record Column(double ground, double top, double z0, double canopy) {}

    private static Upwind upwind(Level level, double x, double z, double direction) {
        double sectors = direction / (2 * Math.PI) * SECTORS;
        return upwind(level, x, z, Math.floorMod((int) Math.round(sectors), SECTORS));
    }

    private static Upwind upwind(Level level, double x, double z, int sector) {
        int cx = Mth.floor(x / CELL), cz = Mth.floor(z / CELL);
        long key = ((long) cx & 0x3FFFFFFL) | (((long) cz & 0x3FFFFFFL) << 26) | ((long) sector << 52);
        Map<Long, Upwind> cache = CACHES.computeIfAbsent(level, l -> new ConcurrentHashMap<>());
        long now = level.getGameTime();
        Upwind u = cache.get(key);
        if (u != null && now - u.tick() < CACHE_TICKS && now >= u.tick()) { return u; }
        if (cache.size() > CACHE_LIMIT) { cache.clear(); }
        u = analyse(level, (cx + 0.5) * CELL, (cz + 0.5) * CELL, sector * 2 * Math.PI / SECTORS, now);
        cache.put(key, u);
        return u;
    }

    private static Upwind analyse(Level level, double x, double z, double direction, long tick) {
        double ex = Math.cos(direction), ez = Math.sin(direction);
        Column here = column(level, x, z);
        double ground = here == null ? level.getSeaLevel() : here.ground();
        double canopy = here == null ? 0 : here.canopy();
        double[] z0 = new double[SAMPLES + 1], weight = new double[SAMPLES + 1];
        z0[0] = here == null ? SurfaceWind.OPEN : here.z0(); weight[0] = 1;
        float[] tops = new float[SAMPLES];
        boolean[] trees = new boolean[SAMPLES];
        double[] groundUp = new double[SAMPLES + 1];
        groundUp[0] = ground;
        for (int i = 1; i <= SAMPLES; i++) {
            double s = i * STEP;
            Column c = column(level, x - ex * s, z - ez * s);
            if (c == null) { c = new Column(ground, ground, z0[0], 0); }
            z0[i] = c.z0(); weight[i] = Math.exp(-s / FOOTPRINT);
            tops[i - 1] = (float) c.top(); trees[i - 1] = c.canopy() > 1; groundUp[i] = c.ground();
        }
        double effective = SurfaceWind.effectiveRoughness(z0, weight);
        double displacement = canopy >= 2 ? 0.7 * canopy : 0;
        // Hill: height above the lowest ground upwind, half-length to half that height, and how much is crest.
        double lowest = ground;
        for (double g : groundUp) { lowest = Math.min(lowest, g); }
        double hill = ground - lowest, half = 0, crest = 0;
        if (hill >= LEAST_HILL) {
            for (int i = 1; i <= SAMPLES; i++) { if (groundUp[i] <= ground - hill / 2) { half = i * STEP; break; } }
            double rise = 0;
            for (double s = STEP; s <= Math.min(half, REACH / 2); s += STEP) {
                Column c = column(level, x + ex * s, z + ez * s);
                if (c != null) { rise = Math.max(rise, c.ground() - ground); }
            }
            crest = Math.clamp(1 - rise / hill, 0, 1);
        }
        // The ground's slope along the wind over a step either side (water and unknown ground: level).
        Column down = column(level, x + ex * STEP, z + ez * STEP);
        double slope = here == null || down == null ? 0 : (down.ground() - groundUp[1]) / (2 * STEP);
        return new Upwind(ground, canopy, displacement, effective, half > 0 ? hill : 0, half, crest, tops, trees, slope, tick);
    }

    /** A column from the client-visible heightmap, or null where its chunk is not loaded (never loads one). */
    private static Column column(Level level, double x, double z) {
        int bx = Mth.floor(x), bz = Mth.floor(z);
        if (!level.hasChunk(bx >> 4, bz >> 4)) { return null; }
        int highest = level.getHeight(Heightmap.Types.MOTION_BLOCKING, bx, bz);
        BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos(bx, highest - 1, bz);
        int ground = highest, top = Integer.MIN_VALUE;
        // Down through barriers (and air) to the first leaves, the canopy's top, then through the leaves to the ground.
        for (int i = 0; i < CANOPY_SCAN && pos.getY() > level.getMinBuildHeight(); i++) {
            BlockState state = level.getBlockState(pos);
            boolean leaves = state.is(BlockTags.LEAVES);
            if (leaves && top == Integer.MIN_VALUE) { top = pos.getY() + 1; }
            if (leaves || state.isAir() || state.is(net.minecraft.world.level.block.Blocks.BARRIER)
                    || (!state.blocksMotion() && state.getFluidState().isEmpty())) { pos.move(0, -1, 0); continue; }
            ground = pos.getY() + 1;
            break;
        }
        if (top == Integer.MIN_VALUE) { top = ground; }
        BlockState below = level.getBlockState(pos.set(bx, ground - 1, bz));
        double canopy = Math.max(0, top - ground);
        if (below.getFluidState().is(FluidTags.WATER)) { return new Column(ground - 1 + 8 / 9.0, top, SurfaceWind.SEA, canopy); }
        double z0;
        if (canopy >= 2) { z0 = Math.clamp(0.1 * canopy, SurfaceWind.ROUGH, SurfaceWind.CHAOTIC); }
        else if (below.is(BlockTags.SAND) || below.is(BlockTags.SNOW) || below.is(BlockTags.ICE)) { z0 = SurfaceWind.SMOOTH; }
        else if (manMade(below)) { z0 = SurfaceWind.VERY_ROUGH; }
        else { z0 = SurfaceWind.OPEN; }
        return new Column(ground, top, z0, canopy);
    }

    private static boolean manMade(BlockState state) {
        SoundType sound = state.getSoundType();
        return state.is(BlockTags.PLANKS) || state.is(BlockTags.WOOL) || sound == SoundType.GLASS || sound == SoundType.METAL
                || sound == SoundType.COPPER || state.is(BlockTags.STONE_BRICKS) || state.is(BlockTags.SLABS) || state.is(BlockTags.STAIRS);
    }

    // ---------------------------------------------------------------- server state and sync

    /** The level's fixed wind, if set by command. */
    public static final class Saved extends SavedData {
        private static final String NAME = "aerocore_world_wind";
        private static final Factory<Saved> FACTORY = new Factory<>(Saved::new, Saved::load, null);
        Reference override;

        static Saved of(ServerLevel level) { return level.getDataStorage().computeIfAbsent(FACTORY, NAME); }

        private static Saved load(CompoundTag tag, HolderLookup.Provider registries) {
            Saved saved = new Saved();
            if (tag.contains("Speed")) {
                try { saved.override = new Reference(tag.getDouble("Speed"), tag.getDouble("Direction")); } catch (IllegalArgumentException e) { saved.override = null; }
            }
            return saved;
        }

        @Override public CompoundTag save(CompoundTag tag, HolderLookup.Provider registries) {
            if (override != null) { tag.putDouble("Speed", override.speedMps()); tag.putDouble("Direction", override.directionRadians()); }
            return tag;
        }
    }

    /** One dimension's reference wind for its players. */
    public record WindPayload(ResourceLocation dimension, double speedMps, double directionRadians) implements CustomPacketPayload {
        public static final Type<WindPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(AeroCore.MOD_ID, "world_wind"));
        public static final StreamCodec<FriendlyByteBuf, WindPayload> STREAM_CODEC = StreamCodec.composite(
                ResourceLocation.STREAM_CODEC, WindPayload::dimension, ByteBufCodecs.DOUBLE, WindPayload::speedMps,
                ByteBufCodecs.DOUBLE, WindPayload::directionRadians, WindPayload::new);
        @Override public Type<WindPayload> type() { return TYPE; }
    }

    /** Client side: stores a received reference wind. */
    public static void accept(WindPayload payload) {
        try {
            CLIENT.put(ResourceKey.create(net.minecraft.core.registries.Registries.DIMENSION, payload.dimension()),
                    new Reference(payload.speedMps(), payload.directionRadians()));
        } catch (IllegalArgumentException ignored) {
            // A malformed payload leaves the last wind in place.
        }
    }

    /** Client side: forgets every received wind (leaving a server). */
    public static void clearClient() { CLIENT.clear(); }

    private static WindPayload payload(ServerLevel level) {
        Reference r = reference(level);
        return new WindPayload(level.dimension().location(), r.speedMps(), r.directionRadians());
    }

    static void sync(ServerLevel level) {
        WindPayload p = payload(level);
        SENT.put(level, new Reference(p.speedMps(), p.directionRadians()));
        PacketDistributor.sendToPlayersInDimension(level, p);
    }

    @SubscribeEvent public static void tick(LevelTickEvent.Post event) {
        if (!(event.getLevel() instanceof ServerLevel level) || level.getGameTime() % SYNC_TICKS != 0) { return; }
        Reference now = reference(level), sent = SENT.get(level);
        if (sent == null || Math.abs(now.speedMps() - sent.speedMps()) > SYNC_SPEED * Math.max(1, sent.speedMps())
                || Math.abs(Mth.wrapDegrees(Math.toDegrees(now.directionRadians() - sent.directionRadians()))) > Math.toDegrees(SYNC_ANGLE)) {
            sync(level);
        }
    }

    private static void send(ServerPlayer player) {
        if (player.level() instanceof ServerLevel level) { PacketDistributor.sendToPlayer(player, payload(level)); }
    }

    @SubscribeEvent public static void login(PlayerEvent.PlayerLoggedInEvent event) {
        if (event.getEntity() instanceof ServerPlayer player) { send(player); }
    }
    @SubscribeEvent public static void changeDimension(PlayerEvent.PlayerChangedDimensionEvent event) {
        if (event.getEntity() instanceof ServerPlayer player) { send(player); }
    }
    @SubscribeEvent public static void respawn(PlayerEvent.PlayerRespawnEvent event) {
        if (event.getEntity() instanceof ServerPlayer player) { send(player); }
    }
}
