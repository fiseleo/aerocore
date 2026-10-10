package dev.mcmod.aerocore.cfd;

import java.util.Objects;

/**
 * D2Q9 BGK, Guo body forcing, halfway bounce-back at static solids. Periodic X, solid Y walls.
 * Isothermal, single-phase, weakly compressible low-Mach experiment; not a 3D vehicle solver.
 * Mutable and not thread-safe. All inputs/outputs here use lattice units.
 */
public final class LbmFlow2D {
    private static final int[] CX = {0, 1, 0, -1, 0, 1, -1, -1, 1};
    private static final int[] CY = {0, 0, 1, 0, -1, 1, 1, -1, -1};
    private static final int[] OPP = {0, 3, 4, 1, 2, 7, 8, 5, 6};
    private static final double[] W = {4.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/36, 1.0/36, 1.0/36, 1.0/36};
    public static final double MAX_LATTICE_MACH = 0.1;

    @FunctionalInterface public interface Obstacle { boolean contains(int x, int y); }
    public record Cell(double density, double velocityX, double velocityY, boolean solid) {}
    public record Diagnostics(long steps, double mass, double maxMach, double maxDensityDeviation,
                              double obstacleForceX, double obstacleForceY) {}

    private final int width, height;
    private final double viscosity, omega, accelerationX, accelerationY;
    private final boolean[] solid, obstacle;
    private double[] populations, next;
    private long steps;
    private double obstacleForceX, obstacleForceY;

    public LbmFlow2D(int width, int height, double viscosity, double accelerationX,
                     double accelerationY, Obstacle geometry) {
        if (width < 4 || height < 4 || (long) width * height > 1_000_000) {
            throw new IllegalArgumentException("Grid must be at least 4x4 and at most 1000000 cells");
        }
        // Conservative range for this research prototype, not a general mathematical stability guarantee.
        if (!Double.isFinite(viscosity) || viscosity < 1.0 / 60 || viscosity > 0.5) {
            throw new IllegalArgumentException("Lattice viscosity must be in [1/60, 0.5] (tau in [0.55, 2])");
        }
        if (!Double.isFinite(accelerationX) || !Double.isFinite(accelerationY)
                || Math.hypot(accelerationX, accelerationY) > 0.001) {
            throw new IllegalArgumentException("Lattice acceleration magnitude must be <= 0.001");
        }
        Objects.requireNonNull(geometry, "geometry");
        this.width = width;
        this.height = height;
        this.viscosity = viscosity;
        omega = 1 / (3 * viscosity + 0.5);
        this.accelerationX = accelerationX;
        this.accelerationY = accelerationY;
        solid = new boolean[width * height];
        obstacle = new boolean[solid.length];
        populations = new double[solid.length * 9];
        next = new double[populations.length];
        int fluidCells = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int cell = y * width + x;
                boolean wall = y == 0 || y == height - 1;
                obstacle[cell] = !wall && geometry.contains(x, y);
                solid[cell] = wall || obstacle[cell];
                if (!solid[cell]) {
                    fluidCells++;
                    System.arraycopy(W, 0, populations, cell * 9, 9);
                }
            }
        }
        if (fluidCells == 0) { throw new IllegalArgumentException("Grid must contain fluid cells"); }
    }

    public int width() { return width; }
    public int height() { return height; }
    public double viscosity() { return viscosity; }

    /** Collision followed by push streaming. A failed stability check invalidates this run. */
    public void step() {
        double forceX = 0, forceY = 0;
        for (int y = 1; y < height - 1; y++) {
            for (int x = 0; x < width; x++) {
                int cell = y * width + x;
                if (solid[cell]) { continue; }
                int base = cell * 9;
                double rho = 0, momentumX = 0, momentumY = 0;
                for (int k = 0; k < 9; k++) {
                    double f = populations[base + k];
                    rho += f;
                    momentumX += f * CX[k];
                    momentumY += f * CY[k];
                }
                double ux = momentumX / rho + accelerationX * 0.5;
                double uy = momentumY / rho + accelerationY * 0.5;
                requireStable(rho, ux, uy);
                double speedSquared = ux * ux + uy * uy;
                for (int k = 0; k < 9; k++) {
                    double cu = CX[k] * ux + CY[k] * uy;
                    double equilibrium = W[k] * rho * (1 + 3 * cu + 4.5 * cu * cu - 1.5 * speedSquared);
                    double ca = CX[k] * accelerationX + CY[k] * accelerationY;
                    double source = W[k] * rho * (1 - 0.5 * omega)
                            * (3 * ((CX[k] - ux) * accelerationX + (CY[k] - uy) * accelerationY) + 9 * cu * ca);
                    double outgoing = populations[base + k] + omega * (equilibrium - populations[base + k]) + source;
                    if (!Double.isFinite(outgoing) || outgoing < 0) {
                        throw new IllegalStateException("LBM population became invalid; refine parameters and restart");
                    }
                    int nx = x + CX[k];
                    if (nx < 0) { nx = width - 1; }
                    else if (nx == width) { nx = 0; }
                    int target = (y + CY[k]) * width + nx;
                    if (solid[target]) {
                        next[base + OPP[k]] = outgoing;
                        if (obstacle[target]) {
                            // Momentum transferred to the obstacle by a reflected link, excluding walls.
                            forceX += 2 * outgoing * CX[k];
                            forceY += 2 * outgoing * CY[k];
                        }
                    } else {
                        next[target * 9 + k] = outgoing;
                    }
                }
            }
        }
        // Each fluid population has exactly one upstream fluid or bounce-back link; no clearing needed.
        double[] swap = populations;
        populations = next;
        next = swap;
        obstacleForceX = forceX;
        obstacleForceY = forceY;
        steps++;
    }

    public void advance(int count) {
        if (count < 0) { throw new IllegalArgumentException("Step count must be nonnegative"); }
        for (int i = 0; i < count; i++) { step(); }
    }

    public Cell cell(int x, int y) {
        Objects.checkIndex(x, width);
        Objects.checkIndex(y, height);
        int cell = y * width + x;
        if (solid[cell]) { return new Cell(0, 0, 0, true); }
        double rho = 0, mx = 0, my = 0;
        for (int k = 0; k < 9; k++) {
            double f = populations[cell * 9 + k];
            rho += f;
            mx += f * CX[k];
            my += f * CY[k];
        }
        double ux = mx / rho + accelerationX * 0.5;
        double uy = my / rho + accelerationY * 0.5;
        requireStable(rho, ux, uy);
        return new Cell(rho, ux, uy, false);
    }

    public Diagnostics diagnostics() {
        double mass = 0, maxSpeed = 0, maxDeviation = 0;
        for (int y = 1; y < height - 1; y++) {
            for (int x = 0; x < width; x++) {
                Cell cell = cell(x, y);
                if (cell.solid()) { continue; }
                mass += cell.density();
                maxSpeed = Math.max(maxSpeed, Math.hypot(cell.velocityX(), cell.velocityY()));
                maxDeviation = Math.max(maxDeviation, Math.abs(cell.density() - 1));
            }
        }
        return new Diagnostics(steps, mass, maxSpeed * Math.sqrt(3), maxDeviation, obstacleForceX, obstacleForceY);
    }

    private static void requireStable(double density, double ux, double uy) {
        if (!Double.isFinite(density) || Math.abs(density - 1) > 0.05 || !Double.isFinite(ux) || !Double.isFinite(uy)
                || 3 * (ux * ux + uy * uy) > MAX_LATTICE_MACH * MAX_LATTICE_MACH) {
            throw new IllegalStateException("LBM low-Mach envelope exceeded (Ma<=0.1, density deviation<=5%); restart with smaller forcing/finer grid");
        }
    }
}
