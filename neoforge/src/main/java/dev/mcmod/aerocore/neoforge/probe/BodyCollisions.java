package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.item.ItemEntity;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.tick.EntityTickEvent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

/**
 * Vanilla entities against AeroCore bodies. Entity movement (Entity.collide, via mixin) is clipped against each body's
 * solid boxes in the body frame, one local axis at a time like vanilla (up first, then the larger horizontal one); the
 * entity is treated as an upright box of its width, which ignores its own yaw relative to the body and is exact only
 * for small heel and trim. An entity resting on a body is carried with it (translation and yaw) on the next tick:
 * non-players by the server, the local player by its client. On the server a resting entity loads the body with its
 * weight m g at its feet, an entity walking into a body pushes it with the momentum its blocked movement carried, and
 * a body moving into an entity pushes it out and receives the reaction. Entity masses come from EntityMasses.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class BodyCollisions {
    private static final double EPSILON = 1e-7, TOP_TOLERANCE = 0.03, MAX_PUSH_SPEED = 6;
    /** Client-side bodies, touched only on the client thread. */
    private static final Set<SolidBody> CLIENT = Collections.newSetFromMap(new WeakHashMap<>());
    private static final Map<Entity, Support> SERVER_SUPPORT = new WeakHashMap<>(), CLIENT_SUPPORT = new WeakHashMap<>();
    private record Support(SolidBody body, Vec3d feetLocal, Vec3d centerThen, Orientation orientationThen) {}
    private BodyCollisions() {}

    public static void trackClient(SolidBody body) { CLIENT.add(body); }

    /** Entity mass in kg (EntityMasses). */
    public static double massKg(Entity entity) { return EntityMasses.massKg(entity); }

    private static boolean anyBodies(Entity entity) {
        return entity.level().isClientSide ? !CLIENT.isEmpty() : entity.level() instanceof ServerLevel level && !BodyWorld.bodies(level).isEmpty();
    }
    private static List<SolidBody> bodies(Entity entity) {
        List<SolidBody> list = new ArrayList<>();
        if (entity.level().isClientSide) {
            for (SolidBody body : CLIENT) { if (!body.entity().isRemoved() && body.entity().level() == entity.level()) { list.add(body); } }
        } else if (entity.level() instanceof ServerLevel level) {
            for (CoupledBody body : BodyWorld.bodies(level)) { if (body instanceof SolidBody solid && !body.entity().isRemoved()) { list.add(solid); } }
        }
        return list;
    }

    /** Further clips the movement vanilla allowed (Entity.collide's result) against nearby bodies. */
    public static Vec3 clip(Entity entity, Vec3 movement) {
        if (!anyBodies(entity) || entity instanceof SolidBody || entity.noPhysics || entity.isSpectator() || entity.isPassenger()) { return movement; }
        AABB swept = entity.getBoundingBox().expandTowards(movement).inflate(0.1);
        Vec3 result = movement;
        for (SolidBody body : bodies(entity)) {
            if (body.entity().getBoundingBox().inflate(0.5).intersects(swept)) { result = clip(entity, body, result); }
        }
        return result;
    }

    private static Vec3 clip(Entity entity, SolidBody body, Vec3 movement) {
        Orientation q = body.poseOrientation();
        AABB box = entity.getBoundingBox();
        Vec3d half = new Vec3d(box.getXsize() / 2, box.getYsize() / 2, box.getZsize() / 2);
        double halfWidth = Math.max(half.x(), half.z());
        Vec3d center = body.toLocal(new Vec3d(box.getCenter().x, box.getCenter().y, box.getCenter().z));
        Vec3d requested = q.inverse().rotate(MinecraftUnits.fromMinecraft(movement));
        double[] c = {center.x(), center.y(), center.z()}, h = {halfWidth, half.y(), halfWidth}, m = {requested.x(), requested.y(), requested.z()};
        double[] allowed = m.clone();
        List<WorldBox> boxes = body.solidBoxesLocal();
        boolean supported = false;
        int first = Math.abs(m[0]) >= Math.abs(m[2]) ? 0 : 2;
        for (int axis : new int[]{1, first, 2 - first}) {
            double move = allowed[axis];
            for (WorldBox b : boxes) { move = clipAxis(axis, c, h, b, move); }
            if (axis == 1 && move > m[1] + EPSILON && m[1] < 0) { supported = true; }
            allowed[axis] = move;
            c[axis] += move;
        }
        if (Math.abs(allowed[0] - m[0]) < EPSILON && Math.abs(allowed[1] - m[1]) < EPSILON && Math.abs(allowed[2] - m[2]) < EPSILON) {
            return movement;
        }
        Vec3d world = q.rotate(new Vec3d(allowed[0], allowed[1], allowed[2]));
        boolean server = !entity.level().isClientSide;
        double mass = massKg(entity);
        if (supported) {
            // Weight is applied by the server's standing check (standing()), which also sees players at rest.
            (server ? SERVER_SUPPORT : CLIENT_SUPPORT).put(entity, new Support(body, new Vec3d(c[0], c[1] - h[1], c[2]), body.poseCenter(), q));
        }
        if (server) {
            // Blocked horizontal movement (in the body frame) hands its momentum to the body over the next tick.
            Vec3d blocked = q.rotate(new Vec3d(m[0] - allowed[0], 0, m[2] - allowed[2])).scale(1 / MinecraftUnits.TICK_SECONDS);
            if (blocked.lengthSquared() > 1e-8) {
                double speed = Math.min(blocked.length(), MAX_PUSH_SPEED);
                Vec3d point = new Vec3d(c[0], c[1], c[2]);
                body.addEntityLoad(point, blocked.normalized().scale(mass * speed / MinecraftUnits.TICK_SECONDS));
            }
        }
        return new Vec3(world.x(), world.y(), world.z());
    }

    /**
     * Vanilla-style clip of a movement along one axis by one box, when the entity overlaps it on the other two. Landing
     * on a top face tolerates up to TOP_TOLERANCE of overlap (and then lifts the entity onto it): an upright entity box
     * on a heeled deck dips slightly below its own feet in the body frame, and the box moves between ticks.
     */
    private static double clipAxis(int axis, double[] c, double[] h, WorldBox box, double move) {
        double[] min = {box.min().x(), box.min().y(), box.min().z()}, max = {box.max().x(), box.max().y(), box.max().z()};
        for (int other = 0; other < 3; other++) {
            if (other == axis) { continue; }
            if (c[other] + h[other] <= min[other] + EPSILON || c[other] - h[other] >= max[other] - EPSILON) { return move; }
        }
        double tolerance = axis == 1 ? TOP_TOLERANCE : EPSILON;
        if (move > 0 && c[axis] + h[axis] <= min[axis] + EPSILON) { return Math.min(move, min[axis] - (c[axis] + h[axis])); }
        if (move < 0 && c[axis] - h[axis] >= max[axis] - tolerance) { return Math.max(move, max[axis] - (c[axis] - h[axis])); }
        return move;
    }

    /**
     * Carries entities resting on a body by the body's motion since their support was last recorded (translation and
     * yaw). The support persists while the entity's feet stay on the same spot of the body (within 10 cm vertically
     * and 60 cm horizontally in the body frame): living entities only request a downward move every other tick, so a
     * support found by the movement clip must outlive the tick in which it was found.
     */
    @SubscribeEvent public static void carry(EntityTickEvent.Pre event) {
        Entity entity = event.getEntity();
        boolean client = entity.level().isClientSide;
        Map<Entity, Support> map = client ? CLIENT_SUPPORT : SERVER_SUPPORT;
        Support support = map.get(entity);
        if (support == null) { return; }
        SolidBody body = support.body();
        // The client moves only its own player; the server moves everything else.
        if (body.entity().isRemoved() || body.entity().level() != entity.level() || (client ? !isLocalPlayer(entity) : entity instanceof Player)) {
            map.remove(entity);
            return;
        }
        Vec3d feet = new Vec3d(entity.getX(), entity.getY(), entity.getZ());
        Vec3d local = support.orientationThen().inverse().rotate(feet.subtract(support.centerThen())).add(body.centerOfMassLocal());
        Vec3d offset = local.subtract(support.feetLocal());
        if (Math.abs(offset.y()) > 0.1 || Math.hypot(offset.x(), offset.z()) > 0.6) { map.remove(entity); return; }
        Vec3d now = body.toWorld(local);
        Vec3d shift = now.subtract(feet);
        map.put(entity, new Support(body, local, body.poseCenter(), body.poseOrientation()));
        Vec3d ahead0 = support.orientationThen().rotate(Vec3d.X), ahead1 = body.poseOrientation().rotate(Vec3d.X);
        // Rotation about +y between the bow directions; Minecraft yaw decreases for a positive rotation about +y.
        double turn = Math.atan2(ahead0.z() * ahead1.x() - ahead0.x() * ahead1.z(), ahead0.x() * ahead1.x() + ahead0.z() * ahead1.z());
        if (shift.lengthSquared() > 4 || !Double.isFinite(turn)) { return; }
        entity.setPos(entity.getX() + shift.x(), entity.getY() + shift.y(), entity.getZ() + shift.z());
        float degrees = (float) Math.toDegrees(turn);
        entity.setYRot(entity.getYRot() - degrees);
        if (entity instanceof LivingEntity living) { living.yBodyRot -= degrees; living.yHeadRot -= degrees; }
    }

    /**
     * Pushes entities out of a body's solid boxes after it moved (server: all but players; client: the local player),
     * along the axis of least penetration in the body frame; on the server the body receives the reaction.
     */
    public static void pushOut(SolidBody body) {
        Entity self = body.entity();
        boolean server = !self.level().isClientSide;
        List<Entity> candidates = self.level().getEntities(self, self.getBoundingBox().inflate(0.1),
                e -> !(e instanceof SolidBody) && !e.noPhysics && !e.isSpectator() && !e.isPassenger()
                        && (server ? !(e instanceof Player) : isLocalPlayer(e)));
        Orientation q = body.poseOrientation();
        for (Entity entity : candidates) {
            AABB box = entity.getBoundingBox();
            double halfWidth = Math.max(box.getXsize(), box.getZsize()) / 2, halfHeight = box.getYsize() / 2;
            Vec3d center = body.toLocal(new Vec3d(box.getCenter().x, box.getCenter().y, box.getCenter().z));
            double best = Double.POSITIVE_INFINITY;
            Vec3d push = null;
            for (WorldBox b : body.solidBoxesLocal()) {
                double[] c = {center.x(), center.y(), center.z()}, h = {halfWidth, halfHeight, halfWidth};
                double[] min = {b.min().x(), b.min().y(), b.min().z()}, max = {b.max().x(), b.max().y(), b.max().z()};
                double[] depthUp = new double[3], depthDown = new double[3];
                boolean overlap = true;
                for (int a = 0; a < 3 && overlap; a++) {
                    depthUp[a] = max[a] - (c[a] - h[a]);
                    depthDown[a] = (c[a] + h[a]) - min[a];
                    overlap = depthUp[a] > EPSILON && depthDown[a] > EPSILON;
                }
                if (!overlap) { continue; }
                for (int a = 0; a < 3; a++) {
                    // Prefer lifting: horizontal escapes count double, so entities on a rising deck are raised, not shoved.
                    double weightUp = a == 1 ? 1 : 2;
                    if (depthUp[a] * weightUp < best) { best = depthUp[a] * weightUp; push = axis(a, depthUp[a]); }
                    if (a != 1 && depthDown[a] * 2 < best) { best = depthDown[a] * 2; push = axis(a, -depthDown[a]); }
                }
            }
            if (push == null) { continue; }
            Vec3d shift = q.rotate(push);
            entity.setPos(entity.getX() + shift.x(), entity.getY() + shift.y(), entity.getZ() + shift.z());
            if (!server) { continue; }
            // Give the entity at least the body's speed along the push. A sideways shove hands the body the reaction;
            // a lift does not, because a resting entity's weight is already applied by standing() with real gravity
            // (its falling speed comes from Minecraft's much larger entity gravity).
            Vec3d normal = shift.normalized(), contact = body.toWorld(center);
            Vec3d bodySpeed = body.velocityAt(contact), entitySpeed = MinecraftUnits.fromMinecraft(entity.getDeltaMovement()).scale(1 / MinecraftUnits.TICK_SECONDS);
            double gain = bodySpeed.dot(normal) - entitySpeed.dot(normal);
            if (gain > 0) {
                Vec3d velocity = entitySpeed.add(normal.scale(gain));
                entity.setDeltaMovement(MinecraftUnits.toBlocksPerTick(velocity));
                entity.hurtMarked = true;
                Vec3d sideways = normal.subtract(Vec3d.Y.scale(normal.y()));
                if (Math.abs(normal.y()) < 0.7) { body.addEntityLoad(center, sideways.scale(-massKg(entity) * gain / MinecraftUnits.TICK_SECONDS)); }
            }
        }
    }
    /**
     * Server, after each entity's tick: an entity whose feet rest on the top of a body box (within 5 cm, inside its
     * footprint) loads that body with its weight m g, using real gravity rather than Minecraft's entity gravity.
     * Players are included: the server sees them through their position packets.
     */
    @SubscribeEvent public static void standing(EntityTickEvent.Post event) {
        Entity entity = event.getEntity();
        if (entity.level().isClientSide || !anyBodies(entity) || entity instanceof SolidBody || entity.noPhysics || entity.isSpectator() || entity.isPassenger()) { return; }
        AABB box = entity.getBoundingBox();
        for (SolidBody body : bodies(entity)) {
            if (!body.entity().getBoundingBox().inflate(0.2).intersects(box)) { continue; }
            Vec3d feet = body.toLocal(new Vec3d(box.getCenter().x, box.minY, box.getCenter().z));
            double halfWidth = Math.max(box.getXsize(), box.getZsize()) / 2;
            for (WorldBox b : body.solidBoxesLocal()) {
                boolean above = Math.abs(feet.y() - b.max().y()) < 0.05;
                boolean inside = feet.x() + halfWidth > b.min().x() && feet.x() - halfWidth < b.max().x()
                        && feet.z() + halfWidth > b.min().z() && feet.z() - halfWidth < b.max().z();
                if (above && inside) {
                    body.addEntityLoad(feet, Vec3d.Y.scale(-massKg(entity) * Atmosphere.GRAVITY_MPS2));
                    return;
                }
            }
        }
    }
    private static boolean isLocalPlayer(Entity entity) { return entity.level().isClientSide && entity instanceof Player player && player.isLocalPlayer(); }
    private static Vec3d axis(int a, double amount) { return a == 0 ? new Vec3d(amount, 0, 0) : a == 1 ? new Vec3d(0, amount, 0) : new Vec3d(0, 0, amount); }
}
