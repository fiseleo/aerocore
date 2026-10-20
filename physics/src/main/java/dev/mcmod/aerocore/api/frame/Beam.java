package dev.mcmod.aerocore.api.frame;

import dev.mcmod.aerocore.api.Vec3d;

/**
 * The 12-freedom space beam: Euler–Bernoulli bending in both planes, axial stretch and St Venant torsion (with the
 * member's restrained-warping stiffness, Frame.torsionNm2); no shear deformation. Freedoms per end: u v w along the
 * member's x y z, then rotations about them; matrices are 12 x 12, row-major.
 */
final class Beam {
    private Beam() {}

    /** Rows of the rotation to the member's axes: R[3 a + p] is component p of axis a (x, y, z). */
    static double[] rotation(Frame frame, int member) {
        Vec3d[] axes = frame.axes(member);
        double[] r = new double[9];
        for (int a = 0; a < 3; a++) { r[3 * a] = axes[a].x(); r[3 * a + 1] = axes[a].y(); r[3 * a + 2] = axes[a].z(); }
        return r;
    }

    /** The stiffness in the member's axes. */
    static double[] local(Frame frame, int member) {
        Frame.Member m = frame.member(member);
        double l = frame.lengthM(member), e = m.material().modulusPa();
        return local(e * m.section().areaM2(), frame.torsionNm2(member), e * m.section().iyM4(), e * m.section().izM4(), l);
    }

    static double[] local(double ea, double gj, double eiy, double eiz, double l) {
        double[] k = new double[144];
        double l2 = l * l, l3 = l2 * l;
        set(k, 0, 0, ea / l); set(k, 0, 6, -ea / l); set(k, 6, 6, ea / l);
        set(k, 3, 3, gj / l); set(k, 3, 9, -gj / l); set(k, 9, 9, gj / l);
        // v and the rotation about z: bending in the x-y plane, on Iz (dv/dx = θz)
        double a = 12 * eiz / l3, b = 6 * eiz / l2, c = 4 * eiz / l, d = 2 * eiz / l;
        set(k, 1, 1, a); set(k, 1, 5, b); set(k, 1, 7, -a); set(k, 1, 11, b);
        set(k, 5, 5, c); set(k, 5, 7, -b); set(k, 5, 11, d);
        set(k, 7, 7, a); set(k, 7, 11, -b); set(k, 11, 11, c);
        // w and the rotation about y: bending in the x-z plane, on Iy (dw/dx = -θy)
        a = 12 * eiy / l3; b = 6 * eiy / l2; c = 4 * eiy / l; d = 2 * eiy / l;
        set(k, 2, 2, a); set(k, 2, 4, -b); set(k, 2, 8, -a); set(k, 2, 10, -b);
        set(k, 4, 4, c); set(k, 4, 8, b); set(k, 4, 10, d);
        set(k, 8, 8, a); set(k, 8, 10, b); set(k, 10, 10, c);
        return k;
    }

    private static void set(double[] k, int i, int j, double v) { k[i * 12 + j] = v; k[j * 12 + i] = v; }

    /** Tᵀ k T: a member-axes matrix in the frame's axes (T the rotation on each of the four 3-vectors). */
    static double[] toFrame(double[] k, double[] r) {
        double[] out = new double[144], tmp = new double[9];
        for (int bi = 0; bi < 4; bi++) {
            for (int bj = 0; bj < 4; bj++) {
                for (int a = 0; a < 3; a++) {
                    for (int q = 0; q < 3; q++) {
                        double s = 0;
                        for (int b = 0; b < 3; b++) { s += k[(3 * bi + a) * 12 + 3 * bj + b] * r[3 * b + q]; }
                        tmp[3 * a + q] = s;
                    }
                }
                for (int p = 0; p < 3; p++) {
                    for (int q = 0; q < 3; q++) {
                        double s = 0;
                        for (int a = 0; a < 3; a++) { s += r[3 * a + p] * tmp[3 * a + q]; }
                        out[(3 * bi + p) * 12 + 3 * bj + q] = s;
                    }
                }
            }
        }
        return out;
    }

    /**
     * What the joints put on the member's ends in its axes (force then moment at a, then at b), from the frame's
     * displacements (6 per node by node index, frame axes).
     */
    static double[] endForces(Frame frame, int member, double[] r, double[] k, double[] displacements) {
        Frame.Member m = frame.member(member);
        double[] ul = new double[12];
        for (int blk = 0; blk < 4; blk++) {
            int base = (blk < 2 ? m.a() : m.b()) * 6 + (blk % 2) * 3;
            for (int a = 0; a < 3; a++) {
                ul[3 * blk + a] = r[3 * a] * displacements[base] + r[3 * a + 1] * displacements[base + 1] + r[3 * a + 2] * displacements[base + 2];
            }
        }
        double[] f = new double[12];
        for (int i = 0; i < 12; i++) {
            double s = 0;
            for (int j = 0; j < 12; j++) { s += k[i * 12 + j] * ul[j]; }
            f[i] = s;
        }
        return f;
    }

    /** A member-axes 3-vector (block blk of a 12-vector) in the frame's axes. */
    static Vec3d toFrame(double[] v, int blk, double[] r) {
        double a = v[3 * blk], b = v[3 * blk + 1], c = v[3 * blk + 2];
        return new Vec3d(r[0] * a + r[3] * b + r[6] * c, r[1] * a + r[4] * b + r[7] * c, r[2] * a + r[5] * b + r[8] * c);
    }
}
