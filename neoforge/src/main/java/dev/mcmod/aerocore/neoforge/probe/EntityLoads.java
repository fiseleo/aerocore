package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.ArrayList;
import java.util.List;

/**
 * Forces vanilla entities put on a body (weight, pushes, reactions), collected during one tick's entity updates and
 * applied as constant contact loads through every substep of the next physics tick.
 */
public final class EntityLoads {
    private record Load(Vec3d local, Vec3d force) {}
    private List<Load> pending = new ArrayList<>(), active = List.of();

    public void add(Vec3d localPoint, Vec3d worldForce) {
        if (Double.isFinite(worldForce.x()) && worldForce.lengthSquared() > 0) { pending.add(new Load(localPoint, worldForce)); }
    }
    /** Makes last tick's loads the active ones and starts collecting anew. */
    public void beginTick() { active = pending; pending = new ArrayList<>(); }
    public List<AppliedForce> forces(RigidBodyState s, RigidBodyProperties p) {
        List<AppliedForce> list = new ArrayList<>();
        for (Load load : active) { list.add(new AppliedForce(AppliedForce.Kind.CONTACT, s.worldOffset(p, load.local()), load.force())); }
        return list;
    }
}
