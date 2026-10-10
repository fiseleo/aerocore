package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class BlockMaterialTest {
    @Test void hardnessBasedRestitutionMatchesTheReportedRange() {
        // Thornton with indentation hardness: ice (100 MPa) about 0.43 and oak (35 MPa) about 0.5-0.7 for a steel
        // sphere at 1 m/s; both fall with impact speed (Johnson's V^-1/4).
        double ice = ContactMaterial.STEEL_ON_ICE.restitutionAt(1), wood = ContactMaterial.STEEL_ON_WOOD.restitutionAt(1);
        System.out.printf("Steel at 1 m/s: on ice %.3f, on oak %.3f, on granite %.3f%n", ice, wood, ContactMaterial.STEEL_ON_ROCK.restitutionAt(1));
        assertTrue(ice > 0.35 && ice < 0.5, "ice " + ice);
        assertTrue(wood > 0.45 && wood < 0.75, "wood " + wood);
        assertTrue(ContactMaterial.STEEL_ON_ICE.restitutionAt(3) < ice);
        assertEquals(0, ContactMaterial.STEEL_ON_SAND.restitutionAt(2));
    }

    @Test void eachObstacleUsesItsOwnFriction() {
        // A 1 m steel ball sliding at 2 m/s without spin on ice and on rock: one step's friction impulse is mu N dt.
        RigidBodyProperties p = new RigidBodyProperties(4100, Vec3d.ZERO, InertiaTensor.box(4100, new Vec3d(1, 1, 1)));
        List<CollisionSphere> ball = List.of(new CollisionSphere(Vec3d.ZERO, 0.5));
        RigidBodyState s = new RigidBodyState(new Vec3d(0, 0.5, 0), new Vec3d(2, -0.2, 0), Orientation.IDENTITY, Vec3d.ZERO);
        double dt = 0.005;
        double[] loss = new double[2];
        ContactMaterial[] floors = {ContactMaterial.STEEL_ON_ICE, null};
        for (int i = 0; i < 2; i++) {
            WorldBox floor = new WorldBox(new Vec3d(-5, -1, -5), new Vec3d(5, 0, 5), floors[i]);
            List<ContactSolver.Contact> contacts = ContactSolver.generate(s, p, ball, List.of(floor), dt);
            ContactSolver.Result r = ContactSolver.solve(s, p, contacts, ContactMaterial.STEEL_ON_ROCK, dt);
            loss[i] = s.velocity().x() - r.state().velocity().x();
        }
        // The normal impulse stops the 0.2 m/s approach in both cases, so the speed lost scales with mu.
        assertEquals(ContactMaterial.STEEL_ON_ICE.friction() / ContactMaterial.STEEL_ON_ROCK.friction(), loss[0] / loss[1], 0.05);
    }
}
