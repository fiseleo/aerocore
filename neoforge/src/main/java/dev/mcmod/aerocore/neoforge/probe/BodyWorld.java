package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import dev.mcmod.aerocore.neoforge.mixin.GameTestTickerAccessor;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTestInfo;
import net.minecraft.gametest.framework.GameTestTicker;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.entity.EntityJoinLevelEvent;
import net.neoforged.neoforge.event.entity.EntityLeaveLevelEvent;
import net.neoforged.neoforge.event.tick.LevelTickEvent;
import net.neoforged.neoforge.gametest.GameTestHooks;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

/**
 * Steps all AeroCore test bodies of a level together at the start of its tick: per substep every body prepares its
 * loads, one ContactSolver.solveAll handles static contacts and sphere-sphere contacts between bodies, every body
 * finishes, and body-body penetration is projected out. Bodies therefore push each other with equal and opposite
 * impulses; their mutual contact uses steel on steel (all AeroCore demonstrators have steel contact surfaces).
 *
 * <p>A body whose physics throws is taken out alone (CoupledBody.physicsFailed) and the others step on, so one
 * vehicle's error does not stop the server. In a GameTest run it also fails the test the body belongs to.
 *
 * <p>0.36: bodies joined by couplings (CoupledBody.leaderId, a trailer on a tractor) step as a group: all of them or
 * none (a member unloaded, not ready or faulted, or none in an entity-ticking chunk: the group waits, groupWaits). Their
 * joints go into the same solveAll as the contacts and into separateAll; two linked bodies do not collide with each
 * other unless their coupling says so. Without couplings nothing changes.
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class BodyWorld {
    /** Entity tag of a body whose physics a GameTest makes fail on purpose: its failure then fails no test. */
    public static final String EXPECTED_FAULT_TAG = "aerocore.expected_fault";
    private static final Logger LOGGER = com.mojang.logging.LogUtils.getLogger();
    /** Copy-on-write snapshots, read without allocation by the entity collision hook on every move. */
    private static final Map<ServerLevel, List<CoupledBody>> BODIES = new WeakHashMap<>();
    private BodyWorld() {}

    @SubscribeEvent public static void join(EntityJoinLevelEvent event) {
        if (event.getLevel() instanceof ServerLevel level && event.getEntity() instanceof CoupledBody body) {
            Set<CoupledBody> set = new LinkedHashSet<>(bodies(level));
            set.add(body);
            BODIES.put(level, List.copyOf(set));
        }
    }
    @SubscribeEvent public static void leave(EntityLeaveLevelEvent event) {
        if (event.getLevel() instanceof ServerLevel level && event.getEntity() instanceof CoupledBody body) {
            List<CoupledBody> next = bodies(level).stream().filter(b -> b != body).toList();
            BODIES.put(level, next);
        }
    }
    /** Bodies of a level currently registered (an immutable snapshot). */
    public static List<CoupledBody> bodies(ServerLevel level) {
        List<CoupledBody> list = BODIES.get(level);
        return list == null ? List.of() : list;
    }

    /** One body in one substep: its prepared state, its own static contacts and its envelope for the pair test. */
    private record Stepping(CoupledBody body, ContactSolver.Body prepared, List<ContactSolver.Contact> own, ContactMaterial material, AABB envelope) {}

    @SubscribeEvent public static void tick(LevelTickEvent.Pre event) {
        if (!(event.getLevel() instanceof ServerLevel level) || !level.tickRateManager().runsNormally()) { return; }
        // Bodies whose physics threw this tick: they take no further part in it.
        Set<CoupledBody> down = Collections.newSetFromMap(new IdentityHashMap<>());
        List<CoupledBody> bodies = new ArrayList<>();
        for (CoupledBody body : bodies(level)) {
            BlockPos at = BlockPos.containing(body.entity().position());
            if (body.entity().isRemoved() || !(level.isPositionEntityTicking(at) || body.stepsWhereLoaded() && level.hasChunkAt(at))) { continue; }
            try {
                if (body.beginTick(level)) { bodies.add(body); }
            } catch (RuntimeException e) { failed(level, body, "begin tick", e, down); }
        }
        if (bodies.isEmpty()) { return; }
        if (bodies.stream().anyMatch(BodyWorld::joined)) { bodies = grouped(level, bodies); }
        if (bodies.isEmpty()) { return; }
        double dt = MinecraftUnits.TICK_SECONDS / CoupledBody.SUBSTEPS;
        for (int substep = 0; substep < CoupledBody.SUBSTEPS; substep++) {
            // Each body prepares and finds its static contacts on its own, so one that throws drops out alone.
            List<Stepping> active = new ArrayList<>();
            for (Iterator<CoupledBody> it = bodies.iterator(); it.hasNext(); ) {
                CoupledBody body = it.next();
                try {
                    if (!body.active()) { continue; }
                    ContactSolver.Body prepared = new ContactSolver.Body(body.prepareSubstep(substep), body.properties());
                    List<ContactSolver.Contact> own = ContactSolver.generate(prepared.state(), prepared.properties(), body.collisionShape(),
                            body.staticObstacles(), dt);
                    active.add(new Stepping(body, prepared, own, body.contactMaterial(), body.physicsEnvelope()));
                } catch (RuntimeException e) {
                    it.remove();
                    failed(level, body, "substep " + substep + " (prepare)", e, down);
                }
            }
            if (active.isEmpty()) { break; }
            List<ContactSolver.Body> prepared = active.stream().map(Stepping::prepared).toList();
            List<ContactSolver.PairContact> contacts = new ArrayList<>();
            for (int i = 0; i < active.size(); i++) { contacts.addAll(ContactSolver.tag(i, active.get(i).own(), active.get(i).material())); }
            List<ContactSolver.Joint> joints = joints(active, down);
            List<int[]> pairs = pairs(active, joints);
            for (int[] pair : pairs) {
                try {
                    contacts.addAll(pairContacts(active, prepared, pair, dt));
                } catch (RuntimeException e) {
                    for (int i : pair) { failed(level, active.get(i).body(), "substep " + substep + " (contact with other bodies)", e, down); }
                }
            }
            if (!down.isEmpty()) {
                // A body stopped here pushes no other: drop its contacts (its own result is not used).
                contacts.removeIf(c -> down.contains(active.get(c.a()).body()) || c.b() >= 0 && down.contains(active.get(c.b()).body()));
                pairs.removeIf(p -> down.contains(active.get(p[0]).body()) || down.contains(active.get(p[1]).body()));
            }
            if (!down.isEmpty()) { joints.removeIf(j -> down.contains(active.get(j.leader()).body()) || down.contains(active.get(j.follower()).body())); }
            List<ContactSolver.BodyResult> results = solve(level, active, contacts, pairs, joints, substep, dt, down);
            for (int i = 0; i < active.size(); i++) {
                CoupledBody body = active.get(i).body();
                if (down.contains(body)) { continue; }
                try {
                    body.finishSubstep(substep, results.get(i));
                } catch (RuntimeException e) { failed(level, body, "substep " + substep + " (finish)", e, down); }
            }
            List<int[]> live = pairs.stream().filter(p -> !down.contains(active.get(p[0]).body()) && !down.contains(active.get(p[1]).body())).toList();
            List<ContactSolver.Joint> held = joints.stream().filter(j -> !down.contains(active.get(j.leader()).body()) && !down.contains(active.get(j.follower()).body())).toList();
            if (!live.isEmpty() || !held.isEmpty()) { separate(level, active, live, held, substep, down); }
            bodies.removeIf(down::contains);
        }
        for (CoupledBody body : bodies) {
            try {
                body.endTick();
            } catch (RuntimeException e) { failed(level, body, "end tick", e, down); }
        }
    }

    /**
     * The joint contact solve. When it throws, every body is solved alone with its static contacts, so that a body
     * whose state the solver cannot take is found and stopped; when all solve alone, the contacts between bodies were at
     * fault and the bodies touching another (all of them if none does) are stopped. Stopped bodies get no result.
     */
    private static List<ContactSolver.BodyResult> solve(ServerLevel level, List<Stepping> active, List<ContactSolver.PairContact> contacts,
                                                        List<int[]> pairs, List<ContactSolver.Joint> joints, int substep, double dt, Set<CoupledBody> down) {
        try {
            if (joints.isEmpty()) { return ContactSolver.solveAll(active.stream().map(Stepping::prepared).toList(), contacts, dt); }
            ContactSolver.Solution solved = ContactSolver.solveAll(active.stream().map(Stepping::prepared).toList(), contacts, joints, dt);
            for (int k = 0; k < joints.size(); k++) { active.get(joints.get(k).follower()).body().carried(solved.joints().get(k)); }
            return solved.bodies();
        } catch (RuntimeException joint) {
            List<ContactSolver.BodyResult> results = new ArrayList<>();
            boolean alone = false;
            for (Stepping s : active) {
                if (down.contains(s.body())) { results.add(null); continue; }
                try {
                    results.add(ContactSolver.solveAll(List.of(s.prepared()), ContactSolver.tag(0, s.own(), s.material()), dt).getFirst());
                } catch (RuntimeException e) {
                    results.add(null);
                    failed(level, s.body(), "substep " + substep + " (contact solve)", e, down);
                    alone = true;
                }
            }
            if (!alone) {
                for (int i = 0; i < active.size(); i++) {
                    int index = i;
                    if (down.contains(active.get(i).body())) { continue; }
                    if (pairs.isEmpty() || pairs.stream().anyMatch(p -> p[0] == index || p[1] == index)) {
                        failed(level, active.get(i).body(), "substep " + substep + " (contact with other bodies)", joint, down);
                    }
                }
            }
            return results;
        }
    }

    /**
     * Projects out the penetration between bodies of the live pairs and closes the held joints' gaps (0.36); if that
     * throws, the bodies of those pairs and joints stop.
     */
    private static void separate(ServerLevel level, List<Stepping> active, List<int[]> live, List<ContactSolver.Joint> joints, int substep, Set<CoupledBody> down) {
        List<RigidBodyState> separated;
        List<ContactSolver.Body> current = new ArrayList<>();
        try {
            for (Stepping s : active) { current.add(new ContactSolver.Body(s.body().physicsState(), s.body().properties())); }
            java.util.function.Function<List<ContactSolver.Body>, List<ContactSolver.PairContact>> contacts = states -> {
                List<ContactSolver.PairContact> list = new ArrayList<>();
                for (int[] pair : live) { list.addAll(pairContacts(active, states, pair, 0)); }
                return list;
            };
            separated = joints.isEmpty() ? ContactSolver.separateAll(current, contacts) : ContactSolver.separateAll(current, contacts, joints);
        } catch (RuntimeException e) {
            for (int[] pair : live) for (int i : pair) { failed(level, active.get(i).body(), "substep " + substep + " (separation from other bodies)", e, down); }
            for (ContactSolver.Joint j : joints) {
                failed(level, active.get(j.leader()).body(), "substep " + substep + " (separation of a coupling)", e, down);
                failed(level, active.get(j.follower()).body(), "substep " + substep + " (separation of a coupling)", e, down);
            }
            return;
        }
        for (int i = 0; i < active.size(); i++) {
            CoupledBody body = active.get(i).body();
            if (down.contains(body) || separated.get(i) == current.get(i).state()) { continue; }
            try {
                body.correctPose(separated.get(i));
            } catch (RuntimeException e) { failed(level, body, "substep " + substep + " (separation from other bodies)", e, down); }
        }
    }

    /** Takes a body out of this tick after its physics threw (once), and in a GameTest run fails the test it belongs to. */
    private static void failed(ServerLevel level, CoupledBody body, String phase, RuntimeException error, Set<CoupledBody> down) {
        if (!down.add(body)) { return; }
        body.physicsFailed(phase, error);
        failOwningTests(level, body.entity(), phase, error);
    }

    /**
     * In a GameTest run, a body's physics failure fails the test it belongs to, so that stopping the body does not hide
     * the error. The failure is thrown from the test's own tick (GameTestInfo reports only failures it catches there).
     * A body tagged EXPECTED_FAULT_TAG fails no test.
     */
    private static void failOwningTests(ServerLevel level, Entity entity, String phase, RuntimeException error) {
        if (!GameTestHooks.isGametestEnabled() || entity.getTags().contains(EXPECTED_FAULT_TAG)) { return; }
        try {
            IllegalStateException failure = new IllegalStateException("Physics of " + entity + " failed in " + phase + ": " + error, error);
            for (GameTestInfo test : owningTests(level, entity.position())) { test.setRunAtTickTime(0, () -> { throw failure; }); }
        } catch (RuntimeException e) {
            LOGGER.error("Could not fail the GameTest of {}", entity, e);
        }
    }

    /** The running GameTests whose structure holds a point (with a block of margin), or else the nearest running one. */
    public static List<GameTestInfo> owningTests(ServerLevel level, Vec3 at) {
        List<GameTestInfo> running = ((GameTestTickerAccessor) (Object) GameTestTicker.SINGLETON).aerocore$testInfos().stream()
                .filter(t -> t.getLevel() == level && t.hasStarted() && !t.isDone()).toList();
        List<GameTestInfo> holding = running.stream().filter(t -> t.getStructureBounds().inflate(1).contains(at)).toList();
        if (!holding.isEmpty() || running.isEmpty()) { return holding; }
        return List.of(running.stream().min(Comparator.comparingDouble(t -> t.getStructureBounds().distanceToSqr(at))).orElseThrow());
    }

    /**
     * Body pairs whose envelopes come within a metre of each other, and within their relative travel in a substep (0.28:
     * fast bodies); 0.36: not two bodies a joint links (unless its coupling lets them collide).
     */
    private static List<int[]> pairs(List<Stepping> bodies, List<ContactSolver.Joint> joints) {
        List<int[]> pairs = new ArrayList<>();
        double dt = MinecraftUnits.TICK_SECONDS / CoupledBody.SUBSTEPS;
        for (int i = 0; i < bodies.size(); i++) for (int j = i + 1; j < bodies.size(); j++) {
            if (linked(joints, i, j)) { continue; }
            double closing = bodies.get(i).prepared().state().velocity().subtract(bodies.get(j).prepared().state().velocity()).length() * dt;
            if (bodies.get(i).envelope().inflate(1 + closing).intersects(bodies.get(j).envelope())) { pairs.add(new int[]{i, j}); }
        }
        return pairs;
    }
    private static boolean linked(List<ContactSolver.Joint> joints, int i, int j) {
        for (ContactSolver.Joint k : joints) {
            if ((k.leader() == i && k.follower() == j || k.leader() == j && k.follower() == i) && !k.coupling().collide()) { return true; }
        }
        return false;
    }

    // ---- 0.36: couplings

    private static boolean joined(CoupledBody body) { return body.leaderId() != null || !body.followerIds().isEmpty(); }

    /**
     * The bodies that step this tick when some are joined: a group (bodies joined, through any chain) steps when all its
     * members are here and began their tick and one of them stands in an entity-ticking chunk; a waiting group's joined
     * members hear why (groupWaits). A body joined to none steps as before.
     */
    private static List<CoupledBody> grouped(ServerLevel level, List<CoupledBody> bodies) {
        int n = bodies.size();
        Map<java.util.UUID, Integer> index = new java.util.HashMap<>();
        for (int i = 0; i < n; i++) { index.put(bodies.get(i).entity().getUUID(), i); }
        int[] root = new int[n];
        for (int i = 0; i < n; i++) { root[i] = i; }
        boolean[] incomplete = new boolean[n];
        List<Set<java.util.UUID>> missing = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            CoupledBody b = bodies.get(i);
            Set<java.util.UUID> gone = new LinkedHashSet<>();
            List<java.util.UUID> partners = new ArrayList<>(b.followerIds());
            if (b.leaderId() != null) { partners.add(b.leaderId()); }
            for (java.util.UUID u : partners) {
                Integer j = index.get(u);
                if (j == null) {
                    incomplete[i] = true;
                    if (!(level.getEntity(u) instanceof CoupledBody)) { gone.add(u); }
                } else {
                    int a = find(root, i), c = find(root, j);
                    if (a != c) { root[a] = c; }
                }
            }
            missing.add(gone);
        }
        boolean[] waits = new boolean[n], ticking = new boolean[n], joinedGroup = new boolean[n];
        for (int i = 0; i < n; i++) {
            int r = find(root, i);
            CoupledBody b = bodies.get(i);
            if (incomplete[i]) { waits[r] = true; }
            if (joined(b)) { joinedGroup[r] = true; }
            if (level.isPositionEntityTicking(BlockPos.containing(b.entity().position()))) { ticking[r] = true; }
        }
        List<CoupledBody> stepping = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            int r = find(root, i);
            CoupledBody b = bodies.get(i);
            if (!joinedGroup[r] || !waits[r] && ticking[r]) { stepping.add(b); }
            else if (joined(b)) { b.groupWaits(level, missing.get(i)); }
        }
        return stepping;
    }
    private static int find(int[] root, int i) {
        while (root[i] != i) { root[i] = root[root[i]]; i = root[i]; }
        return i;
    }

    /** The joints between the bodies stepping in a substep: each follower on its leader, when both are there and up. */
    private static List<ContactSolver.Joint> joints(List<Stepping> active, Set<CoupledBody> down) {
        List<ContactSolver.Joint> joints = new ArrayList<>();
        Map<java.util.UUID, Integer> index = null;
        for (int i = 0; i < active.size(); i++) {
            CoupledBody b = active.get(i).body();
            if (b.leaderId() == null || b.leaderCoupling() == null || down.contains(b)) { continue; }
            if (index == null) {
                index = new java.util.HashMap<>();
                for (int k = 0; k < active.size(); k++) { index.put(active.get(k).body().entity().getUUID(), k); }
            }
            Integer l = index.get(b.leaderId());
            if (l != null && l != i && !down.contains(active.get(l).body())) { joints.add(new ContactSolver.Joint(l, i, b.leaderCoupling())); }
        }
        return joints;
    }

    private static List<ContactSolver.PairContact> pairContacts(List<Stepping> bodies, List<ContactSolver.Body> states, int[] pair, double dt) {
        CoupledBody a = bodies.get(pair[0]).body(), b = bodies.get(pair[1]).body();
        return ContactSolver.generatePair(pair[0], states.get(pair[0]), a.collisionShape(), pair[1], states.get(pair[1]), b.collisionShape(),
                ContactMaterial.STEEL_ON_STEEL, dt);
    }
}
