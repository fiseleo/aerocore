package dev.mcmod.aerocore.cfd;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Vec3d;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

/** Reproducible periodic channel/cylinder experiment, deliberately at modest Reynolds number. */
public final class FlowExperiment {
    private FlowExperiment() {}

    public static void main(String[] args) throws IOException {
        if (args.length > 3) { throw new IllegalArgumentException("Usage: [water|air] [steps:1..200000] [output-directory]"); }
        String medium = args.length > 0 ? args[0] : "water";
        FluidState fluid = switch (medium) {
            case "water" -> FluidState.FRESH_WATER_20C;
            case "air" -> Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
            default -> throw new IllegalArgumentException("Medium must be water or air");
        };
        int steps = args.length > 1 ? Integer.parseInt(args[1]) : 12000;
        if (steps < 1 || steps > 200000) { throw new IllegalArgumentException("Steps must be in [1, 200000]"); }
        Path output = args.length > 2 ? Path.of(args[2]) : Path.of("build", "experiments", medium);
        Files.createDirectories(output);
        int width = 128, height = 48;
        double viscosity = 0.05, diameter = 12, channelHeight = height - 2;
        double acceleration = 8 * viscosity * 0.035 / (channelHeight * channelHeight);
        LatticeUnits units = LatticeUnits.forFluid(fluid, 0.0005, viscosity);
        LbmFlow2D flow = new LbmFlow2D(width, height, viscosity, acceleration, 0,
                (x, y) -> Math.pow(x - width / 3.0, 2) + Math.pow(y - (height - 1) / 2.0, 2) <= diameter * diameter / 4);
        double initialMass = flow.diagnostics().mass();
        double[] previousVelocity = velocities(flow);
        double residual = Double.NaN;
        long start = System.nanoTime();
        try (BufferedWriter history = Files.newBufferedWriter(output.resolve("history.csv"))) {
            history.write("step,time_s,mass_relative_error,lattice_mach,max_density_deviation,obstacle_fx_N_per_m,obstacle_fy_N_per_m,velocity_change_over_batch\n");
            for (int completed = 0; completed < steps;) {
                int batch = Math.min(500, steps - completed);
                flow.advance(batch);
                completed += batch;
                double[] current = velocities(flow);
                residual = relativeChange(previousVelocity, current);
                previousVelocity = current;
                LbmFlow2D.Diagnostics d = flow.diagnostics();
                history.write(String.format(Locale.ROOT, "%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g%n",
                        d.steps(), d.steps() * units.timeStepSeconds(), (d.mass() - initialMass) / initialMass,
                        d.maxMach(), d.maxDensityDeviation(), units.forcePerDepthNpm(d.obstacleForceX()),
                        units.forcePerDepthNpm(d.obstacleForceY()), residual));
                if (completed % 3000 == 0 || completed == steps) {
                    System.out.printf(Locale.ROOT, "%s: %d/%d steps, lattice Ma=%.5f, mass error=%.3g, batch velocity change=%.3g%n",
                            medium, completed, steps, d.maxMach(), (d.mass() - initialMass) / initialMass, residual);
                }
            }
        }
        writeField(flow, units, output.resolve("field.csv"));
        LbmFlow2D.Diagnostics d = flow.diagnostics();
        double maxVelocity = d.maxMach() / Math.sqrt(3);
        String report = String.format(Locale.ROOT, """
                AeroCore 2D CFD experiment (%s)
                D2Q9 BGK + Guo forcing; periodic X; stationary no-slip Y walls and cylinder.
                Single phase, isothermal. This is a confined periodic array, not an open wind tunnel.
                Grid: %d x %d; cylinder nominal diameter: %.4g m (stair-step boundary).
                Cell size: %.6g m; time step: %.6g s; elapsed physical time: %.6g s.
                Density: %.8g kg/m3; dynamic viscosity: %.8g Pa.s; tau: %.8g.
                Body acceleration: %.8g m/s2; peak speed: %.8g m/s.
                Re using peak speed and cylinder diameter: %.8g.
                Numerical lattice Mach: %.8g; physical Mach: %.8g (different quantities).
                Relative mass change: %.8g; maximum density deviation: %.8g.
                Force on cylinder per unit extrusion depth: Fx=%.8g N/m; Fy=%.8g N/m.
                Last batch velocity change (relative L2): %.8g. Inspect history.csv for convergence.
                Snapshot after %d steps; not certified steady, mesh-independent, or validated cylinder coefficients.
                Wall-clock solver/export time: %.3f s.
                Field columns use SI; pressure is perturbation about reference density, not absolute ambient pressure.
                Run scripts/plot-cfd.py on field.csv for velocity and pressure plots.
                """, medium, width, height, diameter * units.cellSizeM(), units.cellSizeM(), units.timeStepSeconds(),
                steps * units.timeStepSeconds(), fluid.densityKgPerM3(), fluid.dynamicViscosityPaS(), 3 * viscosity + 0.5,
                units.accelerationMps2(acceleration), units.velocityMps(maxVelocity), maxVelocity * diameter / viscosity,
                d.maxMach(), units.velocityMps(maxVelocity) / fluid.speedOfSoundMps(),
                (d.mass() - initialMass) / initialMass, d.maxDensityDeviation(), units.forcePerDepthNpm(d.obstacleForceX()),
                units.forcePerDepthNpm(d.obstacleForceY()), residual, steps, (System.nanoTime() - start) / 1e9);
        Files.writeString(output.resolve("report.txt"), report);
        System.out.println(report);
        System.out.println("Results: " + output.toAbsolutePath().normalize());
    }

    private static void writeField(LbmFlow2D flow, LatticeUnits units, Path path) throws IOException {
        try (BufferedWriter writer = Files.newBufferedWriter(path)) {
            writer.write("x_m,y_m,solid,density_kg_per_m3,u_mps,v_mps,pressure_delta_pa\n");
            for (int y = 0; y < flow.height(); y++) {
                for (int x = 0; x < flow.width(); x++) {
                    LbmFlow2D.Cell c = flow.cell(x, y);
                    writer.write(String.format(Locale.ROOT, "%.12g,%.12g,%d,%.12g,%.12g,%.12g,%.12g%n",
                            x * units.cellSizeM(), y * units.cellSizeM(), c.solid() ? 1 : 0,
                            c.solid() ? 0 : c.density() * units.referenceDensityKgPerM3(), units.velocityMps(c.velocityX()),
                            units.velocityMps(c.velocityY()), c.solid() ? 0 : units.pressureDifferencePa(c.density())));
                }
            }
        }
    }

    private static double[] velocities(LbmFlow2D flow) {
        double[] values = new double[flow.width() * flow.height() * 2];
        for (int y = 0; y < flow.height(); y++) {
            for (int x = 0; x < flow.width(); x++) {
                LbmFlow2D.Cell c = flow.cell(x, y);
                int index = 2 * (y * flow.width() + x);
                values[index] = c.velocityX();
                values[index + 1] = c.velocityY();
            }
        }
        return values;
    }

    private static double relativeChange(double[] before, double[] after) {
        double difference = 0, norm = 0;
        for (int i = 0; i < before.length; i++) {
            difference += Math.pow(after[i] - before[i], 2);
            norm += after[i] * after[i];
        }
        return norm == 0 ? 0 : Math.sqrt(difference / norm);
    }
}
