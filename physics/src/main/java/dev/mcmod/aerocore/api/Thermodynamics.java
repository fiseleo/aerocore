package dev.mcmod.aerocore.api;

/** Dry ideal air with constant cp; pure liquid water using Patek et al. (2009), DOI 10.1063/1.3043575. */
public final class Thermodynamics {
    public static final double AIR_R=287.05287, AIR_GAMMA=1.4;
    public static final double AIR_CP=AIR_R*AIR_GAMMA/(AIR_GAMMA-1);
    private static final double WATER_R=461.51805, P0=100000, TR=10;
    private static final double[] GA={-1.661470539e5,2.708781640e6,-1.557191544e8};
    private static final int[] GN={4,5,7};
    private static final double[] GB={-8.237426256e-1,1.908956353,-2.017597384,8.546361348e-1};
    private static final int[] GM={2,3,4,5};
    private static final double[] VA={6.74458446e3,-2.22521604e5,1.00231247e8,-1.63552118e9,8.32299658e9};
    private static final int[] VN={4,5,7,8,9};
    private static final double[] VB={5.78545292e-3,-1.53195665e-2,3.11337859e-2,-4.23546241e-2,3.38713507e-2,-1.19946761e-2};
    private static final int[] VM={1,2,3,4,5,6};
    private static final double[] PA={-7.5245878e-6,-1.3767418e-2,1.0627293e1,-2.0457795e2,1.2037414e3};
    private static final int[] PN={1,3,5,6,7};
    private static final double[] PB={-3.1091470e-6,2.8964919e-5,-1.3112763e-4,3.0410453e-4,-3.9034594e-4,2.3403117e-4,-4.8510101e-5};
    private static final int[] PM={1,3,4,5,6,7,9};
    private Thermodynamics() {}

    /** Calorically perfect air, 180..500 K and 1..300 kPa. Conductivity uses Pr=0.71 approximation. */
    public static ThermalFluidState dryAir(double temperatureKelvin,double pressurePa,Vec3d velocity) {
        range(temperatureKelvin,180,500,"Air temperature (K)"); range(pressurePa,1000,300000,"Air pressure (Pa)");
        double viscosity=1.458e-6*Math.pow(temperatureKelvin,1.5)/(temperatureKelvin+110.4);
        FluidState fluid=new FluidState(pressurePa/(AIR_R*temperatureKelvin),viscosity,
                Math.sqrt(AIR_GAMMA*AIR_R*temperatureKelvin),velocity);
        return new ThermalFluidState(temperatureKelvin,pressurePa,AIR_CP,viscosity*AIR_CP/0.71,fluid);
    }

    /** Pure liquid only: restricted to 0..80 C, 80..300 kPa to stay away from phase transitions. */
    public static ThermalFluidState liquidWater(double temperatureKelvin,double pressurePa,Vec3d velocity) {
        range(temperatureKelvin,273.15,353.15,"Water temperature (K)"); range(pressurePa,80000,300000,"Water pressure (Pa)");
        double alpha=TR/(593-temperatureKelvin),beta=TR/(temperatureKelvin-232),tau=temperatureKelvin/TR;
        double cp0=-WATER_R*(-8.983025854+tau*(sum(GA,GN,alpha,2)+sum(GB,GM,beta,2)));
        double v0=WATER_R*TR/P0*(1.93763157e-2+sum(VA,VN,alpha,0)+sum(VB,VM,beta,0));
        double vt0=WATER_R/P0*(sum(VA,VN,alpha,1)-sum(VB,VM,beta,1));
        double vtt0=WATER_R/(P0*TR)*(sum(VA,VN,alpha,2)+sum(VB,VM,beta,2));
        double vp0=WATER_R*TR/(P0*P0)*(sum(PA,PN,alpha,0)+sum(PB,PM,beta,0));
        double vpt0=WATER_R/(P0*P0)*(sum(PA,PN,alpha,1)-sum(PB,PM,beta,1));
        double dp=pressurePa-P0;
        double v=v0+vp0*dp,vt=vt0+vpt0*dp;
        double vp=vp0+3.24e-10*WATER_R*TR/(P0*P0*P0)*dp;
        double cp=cp0-temperatureKelvin*vtt0*dp;
        double sound=Math.sqrt(-v*v/(vp+temperatureKelvin*vt*vt/cp));
        double t=temperatureKelvin/300;
        double mu=1e-6*(280.68*Math.pow(t,-1.9)+511.45*Math.pow(t,-7.7)+61.131*Math.pow(t,-19.6)+0.45903*Math.pow(t,-40));
        double conductivity=0.80201*Math.pow(t,-0.32)-0.25992*Math.pow(t,-5.7)+0.10024*Math.pow(t,-12)-0.032005*Math.pow(t,-15);
        return new ThermalFluidState(temperatureKelvin,pressurePa,cp,conductivity,new FluidState(1/v,mu,sound,velocity));
    }
    private static double sum(double[] coefficients,int[] exponents,double x,int derivative) {
        double result=0;
        for(int i=0;i<coefficients.length;i++) {
            int n=exponents[i];
            double factor=derivative==0?1:derivative==1?n:n*(n+1);
            result+=coefficients[i]*factor*Math.pow(x,n+derivative);
        }
        return result;
    }
    private static void range(double value,double min,double max,String name) {
        if(!Double.isFinite(value)||value<min||value>max) {throw new IllegalArgumentException(name+" must be in ["+min+", "+max+"]");}
    }
}
