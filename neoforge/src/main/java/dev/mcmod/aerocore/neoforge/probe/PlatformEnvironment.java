package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.*;

/**
 * Per-platform experiment controls. Never edits world weather, biomes, or fluid blocks. Waves are not per platform:
 * they belong to the world's WaveRegions, so every body and the client renderer see the same surface; so does the sea
 * wind, which the body passes in every tick (setSea) and which airAt adds, by its log profile at each point's height
 * above the still level, to the experiment's own uniform wind.
 */
public final class PlatformEnvironment {
    private Double airKelvin;
    private double waterKelvin=293.15;
    private Vec3d wind=Vec3d.ZERO,current=Vec3d.ZERO;
    private ThermalFluidState water=Thermodynamics.liquidWater(waterKelvin,100000,Vec3d.ZERO);
    private WaveRegions.Local sea=WaveRegions.Local.CALM;
    /** The experiment's own uniform wind (without the sea region's). */
    public Vec3d wind() {return wind;}
    /** The sea the body is in this tick, for its wind. */
    public void setSea(WaveRegions.Local local) {sea=local;}
    /** Experiment wind plus the sea region's wind at a point, m/s. */
    public Vec3d windAt(Vec3d position) {return wind.add(sea.windAt(position.x(),position.y(),position.z()));}
    public Vec3d current() {return current;}
    public double waterKelvin() {return waterKelvin;}
    public ThermalFluidState waterThermal() {return water;}
    public ThermalFluidState airAt(Vec3d position,int seaLevel) {
        Vec3d local=windAt(position);
        Atmosphere.Sample standard=Atmosphere.atAltitude(Math.clamp(position.y()-seaLevel,-500,20000),local);
        return Thermodynamics.dryAir(airKelvin==null?standard.temperatureKelvin():airKelvin,standard.pressurePa(),local);
    }
    public FluidState waterAt(Vec3d vanillaCurrent) {return water.fluid().withVelocity(vanillaCurrent.add(current));}
    public void setAirCelsius(double degrees) {
        validate(degrees,-60,80); airKelvin=degrees+273.15;
    }
    public void standardAir() {airKelvin=null;}
    public void setWaterCelsius(double degrees) {
        validate(degrees,0,80); waterKelvin=degrees+273.15;
        water=Thermodynamics.liquidWater(waterKelvin,100000,Vec3d.ZERO);
    }
    public void setWind(Vec3d value) {flowRange(value);wind=value;}
    public void setCurrent(Vec3d value) {flowRange(value);current=value;}
    private static void flowRange(Vec3d v) {validate(v.x(),-10,10);validate(v.y(),-10,10);validate(v.z(),-10,10);}
    private static void validate(double value,double min,double max) {
        if(!Double.isFinite(value)||value<min||value>max) {throw new IllegalArgumentException("Experiment value outside ["+min+", "+max+"]");}
    }
}
