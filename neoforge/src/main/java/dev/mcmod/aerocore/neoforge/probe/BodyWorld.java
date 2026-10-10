package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.neoforge.AeroCore;
import dev.mcmod.aerocore.neoforge.MinecraftUnits;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.event.entity.EntityJoinLevelEvent;
import net.neoforged.neoforge.event.entity.EntityLeaveLevelEvent;
import net.neoforged.neoforge.event.tick.LevelTickEvent;

import java.util.ArrayList;
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
 */
@EventBusSubscriber(modid = AeroCore.MOD_ID)
public final class BodyWorld {
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

    @SubscribeEvent public static void tick(LevelTickEvent.Pre event) {
        if (!(event.getLevel() instanceof ServerLevel level) || !level.tickRateManager().runsNormally()) { return; }
        List<CoupledBody> bodies = new ArrayList<>();
        for (CoupledBody body : bodies(level)) {
            if (body.entity().isRemoved() || !level.isPositionEntityTicking(BlockPos.containing(body.entity().position()))) { continue; }
            if (body.beginTick(level)) { bodies.add(body); }
        }
        if (bodies.isEmpty()) { return; }
        double dt = MinecraftUnits.TICK_SECONDS / CoupledBody.SUBSTEPS;
        for (int substep = 0; substep < CoupledBody.SUBSTEPS; substep++) {
            List<CoupledBody> active = bodies.stream().filter(CoupledBody::active).toList();
            if (active.isEmpty()) { break; }
            List<ContactSolver.Body> prepared = new ArrayList<>();
            for (CoupledBody body : active) { prepared.add(new ContactSolver.Body(body.prepareSubstep(substep), body.properties())); }
            List<ContactSolver.PairContact> contacts = new ArrayList<>();
            for (int i = 0; i < active.size(); i++) {
                CoupledBody body = active.get(i);
                contacts.addAll(ContactSolver.tag(i, ContactSolver.generate(prepared.get(i).state(), body.properties(), body.collisionShape(),
                        body.staticObstacles(), dt), body.contactMaterial()));
            }
            List<int[]> pairs = pairs(active);
            for (int[] pair : pairs) { contacts.addAll(pairContacts(active, prepared, pair, dt)); }
            List<ContactSolver.BodyResult> results = ContactSolver.solveAll(prepared, contacts, dt);
            for (int i = 0; i < active.size(); i++) { active.get(i).finishSubstep(substep, results.get(i)); }
            if (!pairs.isEmpty()) {
                List<ContactSolver.Body> current = new ArrayList<>();
                for (CoupledBody body : active) { current.add(new ContactSolver.Body(body.physicsState(), body.properties())); }
                List<RigidBodyState> separated = ContactSolver.separateAll(current, states -> {
                    List<ContactSolver.PairContact> list = new ArrayList<>();
                    for (int[] pair : pairs) { list.addAll(pairContacts(active, states, pair, 0)); }
                    return list;
                });
                for (int i = 0; i < active.size(); i++) {
                    if (separated.get(i) != current.get(i).state()) { active.get(i).correctPose(separated.get(i)); }
                }
            }
        }
        for (CoupledBody body : bodies) { body.endTick(); }
    }

    /** Body pairs whose envelopes come within a metre of each other. */
    private static List<int[]> pairs(List<CoupledBody> bodies) {
        List<int[]> pairs = new ArrayList<>();
        for (int i = 0; i < bodies.size(); i++) for (int j = i + 1; j < bodies.size(); j++) {
            if (bodies.get(i).physicsEnvelope().inflate(1).intersects(bodies.get(j).physicsEnvelope())) { pairs.add(new int[]{i, j}); }
        }
        return pairs;
    }
    private static List<ContactSolver.PairContact> pairContacts(List<CoupledBody> bodies, List<ContactSolver.Body> states, int[] pair, double dt) {
        CoupledBody a = bodies.get(pair[0]), b = bodies.get(pair[1]);
        return ContactSolver.generatePair(pair[0], states.get(pair[0]), a.collisionShape(), pair[1], states.get(pair[1]), b.collisionShape(),
                ContactMaterial.STEEL_ON_STEEL, dt);
    }
}
