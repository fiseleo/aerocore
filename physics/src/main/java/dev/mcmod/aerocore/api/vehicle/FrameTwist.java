package dev.mcmod.aerocore.api.vehicle;

/**
 * A frame that twists (0.33): stations along the part's x (m, increasing) and their twist stiffness, a symmetric N x N
 * matrix (N m/rad) with T_s = sum_t K[s][t] theta_t the torque at station s for twists theta (each station's section
 * rolled about +x against the body), every row summing to zero (twisting it all alike is the body's roll, not the
 * frame's). A ladder frame's or a tube frame's from the beam model (api.frame), a car body's from its shell
 * (BodyStructure.torsion). The part that has one keeps the stations' twists as its first N internal states.
 */
public record FrameTwist(double[] stationX, double[][] twistNmPerRad) {
    public FrameTwist {
        stationX = stationX.clone();
        int n = stationX.length;
        if (n < 2) { throw new IllegalArgumentException("A frame needs two stations or more"); }
        for (int i = 0; i < n; i++) {
            if (!Double.isFinite(stationX[i])) { throw new IllegalArgumentException("Station positions must be finite"); }
            if (i > 0 && !(stationX[i] > stationX[i - 1])) { throw new IllegalArgumentException("Stations must go along x"); }
        }
        if (twistNmPerRad.length != n) { throw new IllegalArgumentException("The stiffness is " + n + " by " + n); }
        double[][] k = new double[n][];
        for (int i = 0; i < n; i++) {
            if (twistNmPerRad[i].length != n) { throw new IllegalArgumentException("The stiffness is " + n + " by " + n); }
            k[i] = twistNmPerRad[i].clone();
            double sum = 0, scale = 0;
            for (int j = 0; j < n; j++) {
                if (!Double.isFinite(k[i][j])) { throw new IllegalArgumentException("Stiffness must be finite"); }
                sum += k[i][j]; scale = Math.max(scale, Math.abs(k[i][j]));
            }
            if (Math.abs(sum) > 1e-9 * Math.max(scale, 1)) { throw new IllegalArgumentException("Each row of the twist stiffness sums to zero"); }
        }
        for (int i = 0; i < n; i++) for (int j = 0; j < i; j++) {
            if (Math.abs(k[i][j] - k[j][i]) > 1e-9 * Math.max(Math.abs(k[i][j]), 1)) { throw new IllegalArgumentException("The twist stiffness is symmetric"); }
        }
        twistNmPerRad = k;
    }

    /** Two stations joined by a torsion spring (N m/rad). */
    public static FrameTwist between(double rearX, double frontX, double stiffnessNmPerRad) {
        return new FrameTwist(new double[]{rearX, frontX}, new double[][]{{stiffnessNmPerRad, -stiffnessNmPerRad}, {-stiffnessNmPerRad, stiffnessNmPerRad}});
    }

    /** Stations joined in a chain by torsion springs, each between neighbours (N m/rad, one fewer than the stations). */
    public static FrameTwist chain(double[] stationX, double[] segmentNmPerRad) {
        int n = stationX.length;
        if (segmentNmPerRad.length != n - 1) { throw new IllegalArgumentException("One spring between each two stations"); }
        double[][] k = new double[n][n];
        for (int i = 0; i + 1 < n; i++) {
            double s = segmentNmPerRad[i];
            k[i][i] += s; k[i + 1][i + 1] += s; k[i][i + 1] -= s; k[i + 1][i] -= s;
        }
        return new FrameTwist(stationX, k);
    }

    public int stations() { return stationX.length; }

    /** The twist between the end stations under a torque T at the front one held at the rear one, per N m (rad/(N m)); infinite when they are free. */
    public double endToEndFlexibility() {
        // the reduced stiffness of the chain's ends: condense the inner stations out
        int n = stationX.length;
        double[][] k = new double[n][];
        for (int i = 0; i < n; i++) { k[i] = twistNmPerRad[i].clone(); }
        // hold station 0, unit torque at n - 1: solve the (n - 1) system for the others
        double[][] a = new double[n - 1][n - 1];
        double[] b = new double[n - 1];
        for (int i = 1; i < n; i++) for (int j = 1; j < n; j++) { a[i - 1][j - 1] = k[i][j]; }
        b[n - 2] = 1;
        double[] t = FrameFlex.solveDense(a, b);
        return t == null ? Double.POSITIVE_INFINITY : t[n - 2];
    }
}
