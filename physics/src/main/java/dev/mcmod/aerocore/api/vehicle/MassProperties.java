package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;

/**
 * Mass, centre of mass and inertia about that centre (kg, m, kg m^2) in some frame. Unlike InertiaTensor the inertia
 * may be singular (a point mass has none); only a whole vehicle's sum must be positive definite.
 */
public record MassProperties(double massKg, Vec3d centre, double xx, double yy, double zz, double xy, double xz, double yz) {
    public static final MassProperties NONE = new MassProperties(0, Vec3d.ZERO, 0, 0, 0, 0, 0, 0);

    public MassProperties {
        if (!Double.isFinite(massKg) || massKg < 0) { throw new IllegalArgumentException("Mass must be finite and nonnegative"); }
        if (!Double.isFinite(xx + yy + zz + xy + xz + yz) || !Double.isFinite(centre.x() + centre.y() + centre.z())) {
            throw new IllegalArgumentException("Mass properties must be finite");
        }
    }

    public static MassProperties point(double massKg, Vec3d at) { return new MassProperties(massKg, at, 0, 0, 0, 0, 0, 0); }

    public static MassProperties of(double massKg, Vec3d centre, InertiaTensor i) {
        return new MassProperties(massKg, centre, i.xx(), i.yy(), i.zz(), i.xy(), i.xz(), i.yz());
    }

    /** The same distribution with every mass multiplied by factor. */
    public MassProperties scaled(double factor) {
        return new MassProperties(massKg * factor, centre, xx * factor, yy * factor, zz * factor, xy * factor, xz * factor, yz * factor);
    }

    /** The same body moved so that its centre is at centre (inertia about the centre unchanged). */
    public MassProperties at(Vec3d newCentre) { return new MassProperties(massKg, newCentre, xx, yy, zz, xy, xz, yz); }

    /** Expressed in the parent frame of a mount: centre moved and rotated, inertia R I R^T. */
    public MassProperties transformed(Vec3d position, Orientation rotation) {
        double[][] r = matrix(rotation), i = {{xx, xy, xz}, {xy, yy, yz}, {xz, yz, zz}}, out = new double[3][3];
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) {
            double sum = 0;
            for (int k = 0; k < 3; k++) for (int l = 0; l < 3; l++) { sum += r[a][k] * i[k][l] * r[b][l]; }
            out[a][b] = sum;
        }
        return new MassProperties(massKg, position.add(rotation.rotate(centre)), out[0][0], out[1][1], out[2][2],
                0.5 * (out[0][1] + out[1][0]), 0.5 * (out[0][2] + out[2][0]), 0.5 * (out[1][2] + out[2][1]));
    }

    /** Several bodies in one frame as one: total mass, mass-weighted centre, inertias moved there (parallel axis). */
    public static MassProperties combine(List<MassProperties> parts) {
        double mass = 0;
        Vec3d moment = Vec3d.ZERO;
        for (MassProperties p : parts) { mass += p.massKg; moment = moment.add(p.centre.scale(p.massKg)); }
        if (mass == 0) { return NONE; }
        Vec3d c = moment.scale(1 / mass);
        double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
        for (MassProperties p : parts) {
            Vec3d r = p.centre.subtract(c);
            double m = p.massKg;
            xx += p.xx + m * (r.y() * r.y() + r.z() * r.z());
            yy += p.yy + m * (r.x() * r.x() + r.z() * r.z());
            zz += p.zz + m * (r.x() * r.x() + r.y() * r.y());
            xy += p.xy - m * r.x() * r.y();
            xz += p.xz - m * r.x() * r.z();
            yz += p.yz - m * r.y() * r.z();
        }
        return new MassProperties(mass, c, xx, yy, zz, xy, xz, yz);
    }

    /** The inertia as an InertiaTensor; throws unless positive definite. */
    public InertiaTensor inertia() { return new InertiaTensor(xx, yy, zz, xy, xz, yz); }

    /** Columns are the rotated unit axes: v_parent = R v_local. */
    static double[][] matrix(Orientation q) {
        Vec3d ex = q.rotate(Vec3d.X), ey = q.rotate(Vec3d.Y), ez = q.rotate(Vec3d.Z);
        return new double[][]{{ex.x(), ey.x(), ez.x()}, {ex.y(), ey.y(), ez.y()}, {ex.z(), ey.z(), ez.z()}};
    }
}
