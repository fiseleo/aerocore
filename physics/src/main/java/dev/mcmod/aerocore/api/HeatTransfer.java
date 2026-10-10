package dev.mcmod.aerocore.api;

import java.util.List;

/** Lumped, uniform-temperature bodies with constant heat capacity. No phase change or radiation. */
public final class HeatTransfer {
    private HeatTransfer() {}
    public record Body(double temperatureKelvin,double heatCapacityJPerK) {
        public Body {Checks.positive(temperatureKelvin,"Temperature");Checks.positive(heatCapacityJPerK,"Heat capacity");}
    }
    /** Thermal conductance G=h*A in W/K. Reservoir temperature is prescribed, not depleted. */
    public record Reservoir(double temperatureKelvin,double conductanceWPerK) {
        public Reservoir {Checks.positive(temperatureKelvin,"Temperature");Checks.nonnegative(conductanceWPerK,"Conductance");}
    }
    public record Result(Body body,double energyChangeJ,double externalEnergyJ,double heatFromReservoirsJ) {}
    public record Pair(Body first,Body second,double heatToFirstJ) {}

    /** Exact constant-coefficient solution of C*dT/dt = P + sum G_i*(T_i-T). */
    public static Result reservoirs(Body body,List<Reservoir> reservoirs,double powerW,double dt) {
        Checks.positive(dt,"Time step");
        if(!Double.isFinite(powerW)) {throw new IllegalArgumentException("Power must be finite");}
        double conductance=0,heatRate=powerW;
        for(Reservoir r:reservoirs) {
            conductance+=r.conductanceWPerK();
            heatRate+=r.conductanceWPerK()*(r.temperatureKelvin()-body.temperatureKelvin());
        }
        double delta=conductance==0?powerW*dt/body.heatCapacityJPerK():
                heatRate/conductance*(-Math.expm1(-conductance*dt/body.heatCapacityJPerK()));
        Body next=new Body(body.temperatureKelvin()+delta,body.heatCapacityJPerK());
        double energy=body.heatCapacityJPerK()*delta,external=powerW*dt;
        if(!Double.isFinite(energy)||!Double.isFinite(external)) {throw new IllegalArgumentException("Thermal step overflow");}
        return new Result(next,energy,external,energy-external);
    }
    /** Turbulent-boundary-layer recovery factor r = Pr^(1/3); laminar flow would use Pr^(1/2). */
    public static double turbulentRecoveryFactor(ThermalFluidState fluid) {return Math.cbrt(fluid.prandtlNumber());}
    /**
     * Adiabatic-wall (recovery) temperature rise T_aw - T = r u^2/(2 cp), K, for relative speed u.
     * With r = 1 this is the stagnation temperature rise. Incropera et al., high-speed boundary layers.
     */
    public static double recoveryTemperatureRise(ThermalFluidState fluid,double meanSquareSpeed,double recoveryFactor) {
        Checks.nonnegative(meanSquareSpeed,"Mean square speed");Checks.nonnegative(recoveryFactor,"Recovery factor");
        return recoveryFactor*meanSquareSpeed/(2*fluid.specificHeatCp());
    }
    /**
     * Viscous heating delivered to a wall, W: G (T_aw - T) with conductance G = h A. This heat is a share of
     * the mechanical power dissipated by drag, so it cannot exceed that power; the rest stays in the fluid.
     */
    public static double frictionHeatingW(double conductanceWPerK,ThermalFluidState fluid,double meanSquareSpeed,
                                          double recoveryFactor,double dissipatedPowerW) {
        Checks.nonnegative(conductanceWPerK,"Conductance");Checks.nonnegative(dissipatedPowerW,"Dissipated power");
        return Math.min(conductanceWPerK*recoveryTemperatureRise(fluid,meanSquareSpeed,recoveryFactor),dissipatedPowerW);
    }
    /** Thermal effusivity e = sqrt(k rho c) in W s^0.5/(m^2 K). */
    public static double effusivity(double conductivityWPerMK,double densityKgPerM3,double specificHeatJPerKgK) {
        Checks.positive(conductivityWPerMK,"Conductivity");Checks.positive(densityKgPerM3,"Density");Checks.positive(specificHeatJPerKgK,"Specific heat");
        return Math.sqrt(conductivityWPerMK*densityKgPerM3*specificHeatJPerKgK);
    }
    /**
     * Fraction of interface (friction) heat entering the first body when two semi-infinite solids in perfect
     * thermal contact are heated at their interface: e1/(e1+e2), which keeps both surface temperatures equal
     * (Blok 1937; Charron 1943). Stationary-source approximation: fast sliding shifts heat toward the solid
     * that sees a fresh surface, which this does not model.
     */
    public static double contactHeatPartition(double bodyEffusivity,double otherEffusivity) {
        Checks.positive(bodyEffusivity,"Body effusivity");Checks.positive(otherEffusivity,"Other effusivity");
        return bodyEffusivity/(bodyEffusivity+otherEffusivity);
    }
    /**
     * Forced convection from a sphere, Whitaker (1972, AIChE J. 18(2):361): Nu = 2 + (0.4 Re^1/2 + 0.06 Re^2/3) Pr^0.4,
     * properties at the free-stream temperature, the viscosity ratio (mu/mu_s)^1/4 taken as 1 (not known here).
     * Stated range 3.5 < Re < 7.6e4, 0.71 < Pr < 380, about +-30 %; Re below the range tends to conduction (Nu = 2).
     */
    public static double sphereForcedNusselt(double reynolds,double prandtl) {
        Checks.nonnegative(reynolds,"Reynolds number");Checks.positive(prandtl,"Prandtl number");
        return 2+(0.4*Math.sqrt(reynolds)+0.06*Math.pow(reynolds,2.0/3))*Math.pow(prandtl,0.4);
    }
    /**
     * Free convection from a sphere, Churchill (1983, Heat Exchanger Design Handbook 2.5.7; Incropera eq. 9.35):
     * Nu = 2 + 0.589 Ra^1/4 / [1 + (0.469/Pr)^(9/16)]^(4/9), Pr >= 0.7, Ra <= 1e11.
     */
    public static double sphereNaturalNusselt(double rayleigh,double prandtl) {
        Checks.nonnegative(rayleigh,"Rayleigh number");Checks.positive(prandtl,"Prandtl number");
        return 2+0.589*Math.pow(rayleigh,0.25)/Math.pow(1+Math.pow(0.469/prandtl,9.0/16),4.0/9);
    }
    /**
     * Convective conductance h A (W/K) of a sphere of diameter D in a fluid moving at speed relative to it, with the
     * surface at surfaceKelvin: forced and free Nusselt numbers combined as Nu^3 = Nu_F^3 + Nu_N^3 (Churchill 1977;
     * Incropera eq. 9.89, assisting flow). thermalExpansion is the fluid's volumetric expansion coefficient (1/K).
     */
    public static double sphereConductance(ThermalFluidState fluid,double thermalExpansion,double diameterM,double speedMps,
                                           double surfaceKelvin,double gravity) {
        Checks.positive(diameterM,"Diameter");Checks.nonnegative(speedMps,"Speed");
        double rho=fluid.fluid().densityKgPerM3(),mu=fluid.fluid().dynamicViscosityPaS(),nu=mu/rho,pr=fluid.prandtlNumber();
        double reynolds=speedMps*diameterM/nu;
        double rayleigh=gravity*Math.abs(thermalExpansion)*Math.abs(surfaceKelvin-fluid.temperatureKelvin())*Math.pow(diameterM,3)*pr/(nu*nu);
        double forced=sphereForcedNusselt(reynolds,pr),natural=sphereNaturalNusselt(rayleigh,pr);
        double nusselt=Math.cbrt(forced*forced*forced+natural*natural*natural);
        return nusselt*fluid.thermalConductivityWPerMK()/diameterM*Math.PI*diameterM*diameterM;
    }
    /** Exact two finite-body exchange. Equal/opposite heat, without temperature overshoot. */
    public static Pair exchange(Body a,Body b,double conductanceWPerK,double dt) {
        Checks.nonnegative(conductanceWPerK,"Conductance");Checks.positive(dt,"Time step");
        double inverseCapacity=1/a.heatCapacityJPerK()+1/b.heatCapacityJPerK();
        double heat=(b.temperatureKelvin()-a.temperatureKelvin())/inverseCapacity*
                (-Math.expm1(-conductanceWPerK*inverseCapacity*dt));
        return new Pair(new Body(a.temperatureKelvin()+heat/a.heatCapacityJPerK(),a.heatCapacityJPerK()),
                new Body(b.temperatureKelvin()-heat/b.heatCapacityJPerK(),b.heatCapacityJPerK()),heat);
    }
}
