package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class ThermodynamicsTest {
    @Test void airObeysEquationOfStateAndTemperatureTrends() {
        ThermalFluidState cool=Thermodynamics.dryAir(273.15,101325,Vec3d.ZERO);
        ThermalFluidState warm=Thermodynamics.dryAir(323.15,101325,Vec3d.ZERO);
        assertEquals(101325,cool.fluid().densityKgPerM3()*Thermodynamics.AIR_R*cool.temperatureKelvin(),1e-9);
        assertTrue(warm.fluid().densityKgPerM3()<cool.fluid().densityKgPerM3());
        assertTrue(warm.fluid().dynamicViscosityPaS()>cool.fluid().dynamicViscosityPaS());
        assertTrue(warm.fluid().speedOfSoundMps()>cool.fluid().speedOfSoundMps());
        assertEquals(0.71,warm.prandtlNumber(),1e-12);
    }
    @Test void idealAirPressureChangesDensityProportionally() {
        var a=Thermodynamics.dryAir(288.15,50000,Vec3d.X);
        var b=Thermodynamics.dryAir(288.15,100000,Vec3d.X);
        assertEquals(2*a.fluid().densityKgPerM3(),b.fluid().densityKgPerM3(),1e-12);
        assertEquals(a.fluid().dynamicViscosityPaS(),b.fluid().dynamicViscosityPaS());
        assertEquals(Vec3d.X,b.fluid().velocity());
    }
    @Test void waterMatchesPublishedIndependentVerificationPoint() {
        // Patek et al. (2009), table 8, 298.15 K at 0.1 MPa. Rounding tolerances reflect published digits.
        var w=Thermodynamics.liquidWater(298.15,100000,Vec3d.ZERO);
        assertEquals(997.047013,w.fluid().densityKgPerM3(),0.000001);
        assertEquals(4181.44618,w.specificHeatCp(),0.00002);
        assertEquals(1496.69922,w.fluid().speedOfSoundMps(),0.00002);
        assertEquals(889.996774e-6,w.fluid().dynamicViscosityPaS(),1e-11);
        assertEquals(0.607198237,w.thermalConductivityWPerMK(),1e-9);
    }
    @Test void waterHasDensityMaximumNearFourCelsiusAndLowerHotViscosity() {
        var zero=Thermodynamics.liquidWater(273.15,101325,Vec3d.ZERO);
        var four=Thermodynamics.liquidWater(277.15,101325,Vec3d.ZERO);
        var twenty=Thermodynamics.liquidWater(293.15,101325,Vec3d.ZERO);
        var hot=Thermodynamics.liquidWater(353.15,101325,Vec3d.ZERO);
        assertTrue(four.fluid().densityKgPerM3()>zero.fluid().densityKgPerM3());
        assertTrue(four.fluid().densityKgPerM3()>twenty.fluid().densityKgPerM3());
        assertEquals(998.2,twenty.fluid().densityKgPerM3(),0.1);
        assertTrue(hot.fluid().dynamicViscosityPaS()<twenty.fluid().dynamicViscosityPaS());
    }
    @Test void increasingWaterPressureIncreasesDensityWithinSupportedRange() {
        var a=Thermodynamics.liquidWater(298.15,100000,Vec3d.ZERO);
        var b=Thermodynamics.liquidWater(298.15,300000,Vec3d.ZERO);
        assertTrue(b.fluid().densityKgPerM3()>a.fluid().densityKgPerM3());
        assertTrue((b.fluid().densityKgPerM3()/a.fluid().densityKgPerM3()-1)<0.001);
    }
    @Test void reservoirCoolingMatchesAnalyticSolutionAndCannotOvershoot() {
        var start=new HeatTransfer.Body(373.15,1000);
        var reservoirs=List.of(new HeatTransfer.Reservoir(293.15,10));
        var result=HeatTransfer.reservoirs(start,reservoirs,0,100);
        assertEquals(293.15+80/Math.E,result.body().temperatureKelvin(),1e-12);
        assertEquals(result.energyChangeJ(),result.heatFromReservoirsJ(),1e-12);
        var end=HeatTransfer.reservoirs(start,reservoirs,0,1e9);
        assertEquals(293.15,end.body().temperatureKelvin(),1e-12);
    }
    @Test void heatInputObeysFirstLawAndReservoirBalance() {
        var start=new HeatTransfer.Body(300,500);
        var heated=HeatTransfer.reservoirs(start,List.of(),200,10);
        assertEquals(304,heated.body().temperatureKelvin(),1e-12);
        assertEquals(2000,heated.energyChangeJ(),1e-12);
        assertEquals(0,heated.heatFromReservoirsJ(),1e-12);
        var mixed=HeatTransfer.reservoirs(start,List.of(new HeatTransfer.Reservoir(280,5),new HeatTransfer.Reservoir(320,20)),200,10);
        assertEquals(mixed.energyChangeJ(),mixed.externalEnergyJ()+mixed.heatFromReservoirsJ(),1e-10);
    }
    @Test void finiteBodiesConserveEnergyAndApproachWeightedEquilibrium() {
        var a=new HeatTransfer.Body(400,100);
        var b=new HeatTransfer.Body(300,300);
        var result=HeatTransfer.exchange(a,b,100,1000);
        assertEquals(325,result.first().temperatureKelvin(),1e-12);
        assertEquals(325,result.second().temperatureKelvin(),1e-12);
        assertEquals(130000,result.first().temperatureKelvin()*100+result.second().temperatureKelvin()*300,1e-9);
        assertEquals(-7500,result.heatToFirstJ(),1e-9);
    }
    @Test void exactThermalUpdateIsIndependentOfStepPartition() {
        var a=new HeatTransfer.Body(350,2000);
        var reservoirs=List.of(new HeatTransfer.Reservoir(290,40),new HeatTransfer.Reservoir(310,100));
        var once=HeatTransfer.reservoirs(a,reservoirs,100,10).body();
        for(int i=0;i<100;i++) {a=HeatTransfer.reservoirs(a,reservoirs,100,0.1).body();}
        assertEquals(once.temperatureKelvin(),a.temperatureKelvin(),1e-10);
    }
    @Test void displayedPointForcesSumToActualIntegratedWrench() {
        var properties=TestPlatform.properties(new Vec3d(0.8,0.7,0.2));
        var body=new RigidBodyState(new Vec3d(0,0.1,0),new Vec3d(2,-1,0.4),Orientation.rotationVector(new Vec3d(0.1,0.2,0.3)),new Vec3d(20,-30,50));
        var result=FloatingBodyDynamics.step(body,properties,TestPlatform.FLOATS,TestPlatform.flatWater(0),Vec3d.Y.scale(-9.80665),0.47,0.005);
        Wrench sum=result.forces().stream().map(AppliedForce::wrench).reduce(Wrench.ZERO,Wrench::add);
        Wrench actual=result.hydrostatics().add(result.averageDrag()).add(result.inertia());
        assertEquals(0,sum.force().subtract(actual.force()).length(),1e-7);
        assertEquals(0,sum.torque().subtract(actual.torque()).length(),1e-7);
        assertEquals(1,result.forces().stream().filter(f->f.kind()==AppliedForce.Kind.WEIGHT).count());
        assertEquals(4,result.forces().stream().filter(f->f.kind()==AppliedForce.Kind.BUOYANCY).count());
        assertEquals(4,result.forces().stream().filter(f->f.kind()==AppliedForce.Kind.DRAG).count());
    }
    @Test void rejectsUnsupportedPhasesAndInvalidThermalInputs() {
        assertThrows(IllegalArgumentException.class,()->Thermodynamics.liquidWater(373.15,100000,Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class,()->Thermodynamics.liquidWater(293.15,1e6,Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class,()->Thermodynamics.dryAir(0,100000,Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class,()->new HeatTransfer.Body(300,0));
        assertThrows(IllegalArgumentException.class,()->new HeatTransfer.Reservoir(300,-1));
    }
}
