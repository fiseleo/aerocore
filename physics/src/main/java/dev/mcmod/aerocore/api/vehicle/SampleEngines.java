package dev.mcmod.aerocore.api.vehicle;

/**
 * Example engine designs, to show the format and exercise the model: starting points a player would change, not
 * engines to sell. Dimensions are of the kind production engines use; they copy no particular engine.
 */
public final class SampleEngines {
    private SampleEngines() {}

    /** A 2.0 l four-valve twin-cam inline four with port injection and intake cam phasing. */
    public static PistonEngineDesign inlineFour() {
        double bore = 0.086, stroke = 0.086, rod = 0.139, compressionHeight = 0.030;
        double vd = Math.PI / 4 * bore * bore * stroke, vc = vd / (11 - 1), gasket = 0.0008;
        double chamber = vc - Math.PI / 4 * bore * bore * gasket;
        return new PistonEngineDesign("inline four 2.0",
                new EngineBlockDesign("block", EngineBlockDesign.Layout.INLINE, 4, 0, bore, 0.096, stroke / 2 + rod + compressionHeight, 5,
                        EngineMaterial.ALUMINIUM_A356, 0.006, 0.8, Double.NaN),
                new CylinderHeadDesign("head", CylinderHeadDesign.Layout.DOHC, 2, 2, CylinderHeadDesign.Chamber.DOME, chamber, 40, 0.87, 0.87, 0.7, 1, 0,
                        gasket, EngineMaterial.ALUMINIUM_A356, Double.NaN),
                new PistonDesign("pistons", bore - 0.00004, compressionHeight, 0, 0.007, 0.035, 0.022, 3, 0.8, EngineMaterial.ALUMINIUM_4032, Double.NaN),
                new ConnectingRodDesign("rods", rod, 0.04803, 0.020, 0.02201, 1.5e-4, EngineMaterial.STEEL_4340, Double.NaN),
                new CrankshaftDesign("crank", stroke, 0.050, 0.020, 0.048, 0.021, 0.7, 0.10, EngineMaterial.STEEL_4340, Double.NaN),
                new ValveDesign("valves", 0.035, 0.0055, 0.030, 0.0055, 0.105, EngineMaterial.VALVE_STEEL, EngineMaterial.VALVE_STEEL, Double.NaN),
                new CamshaftDesign("cams", 290, 0.011, 108, 285, 0.0095, 110, 3, 5, EngineMaterial.DUCTILE_IRON, Double.NaN),
                new SparkPlugDesign("plugs", 0.012, Double.NaN),
                new FuelInjectorDesign("injectors", 310, 3e5, FuelInjectorDesign.Kind.PORT, Double.NaN),
                new ValvetrainDesign("valvetrain", ValvetrainDesign.Mechanism.FINGER_FOLLOWER, true, 250, 45000, 0.04, ValvetrainDesign.Drive.CHAIN, 25, 0, Double.NaN),
                new FuelSystemDesign("fuel", "gasoline", 95, 85, FuelSystemDesign.Delivery.PORT_INJECTION, 1, 3e5, 0.88, 1.0, Double.NaN),
                new IgnitionDesign("ignition", true, 45, 7800, 800, 120, Double.NaN),
                new LubricationDesign("oil", "5W-20", 0.0045, LubricationDesign.Sump.WET, Double.NaN),
                new CoolingDesign("cooling", CoolingDesign.Kind.LIQUID, 0.35, 0.03, 0, 0.6, 0.006, 88, Double.NaN),
                new IntakeDesign("intake", 0.35, 0.040, 0.004, 0.062, 1, 1.0, Double.NaN),
                new ExhaustDesign("exhaust", 0.7, Double.NaN));
    }

    /**
     * A J85-class single-spool turbojet: eight axial stages to pressure ratio 8 at 20 kg/s, turbine inlet 1250 K, two
     * lightly cooled turbine stages, convergent nozzle.
     */
    public static JetEngineDesign turbojet() {
        return new JetEngineDesign("turbojet 20 kg/s",
                new JetInletDesign("inlet", dev.mcmod.aerocore.api.engine.Turbojet.Inlet.PITOT, 0.10, 0.98, 0.1, EngineMaterial.ALUMINIUM_7075, Double.NaN),
                new CompressorDesign("compressor", CompressorDesign.Kind.AXIAL, 8, 8, 20, 0.87, 0.2, 0.42, 0.45, 16500, 0, EngineMaterial.TITANIUM_6AL4V, Double.NaN),
                new CombustorDesign("combustor", 0.05, 0.98, 0.45, 0.45, EngineMaterial.INCONEL_718, Double.NaN),
                new TurbineDesign("turbine", 2, 0.88, 0.44, 0.2, 0.03, 0.99, EngineMaterial.INCONEL_713C, Double.NaN),
                new JetNozzleDesign("nozzle", false, 1, 0.98, 0.25, EngineMaterial.STAINLESS_321, Double.NaN),
                new JetControlDesign("control", "kerosene", 1250, 0.55, 1.02, true, 0.3, 150, 0.15, Double.NaN));
    }

    /**
     * A model turbojet of the kind in radio-controlled jets: one centrifugal stage to pressure ratio 2.2 at 0.23 kg/s
     * and 112000 rpm, an uncooled single-stage turbine at 950 K, a short convergent tailpipe.
     */
    public static JetEngineDesign modelTurbojet() {
        return new JetEngineDesign("model turbojet",
                new JetInletDesign("inlet", dev.mcmod.aerocore.api.engine.Turbojet.Inlet.PITOT, 0.0025, 0.97, 0.05, EngineMaterial.ALUMINIUM_7075, Double.NaN),
                new CompressorDesign("impeller", CompressorDesign.Kind.CENTRIFUGAL, 1, 2.2, 0.23, 0.75, 0.15, 0.066, 0.3, 112000, 12, EngineMaterial.ALUMINIUM_2618, Double.NaN),
                new CombustorDesign("combustor", 0.06, 0.95, 0.12, 0.11, EngineMaterial.STAINLESS_321, Double.NaN),
                new TurbineDesign("turbine", 1, 0.80, 0.062, 0, 0, 0.98, EngineMaterial.INCONEL_713C, Double.NaN),
                new JetNozzleDesign("nozzle", false, 1, 0.98, 0.1, EngineMaterial.STAINLESS_321, Double.NaN),
                new JetControlDesign("control", "kerosene", 950, 0.35, 1.0, true, 0.3, 0.15, 0.12, Double.NaN));
    }
}
