package dev.mcmod.aerocore.api.acoustics;

import java.util.List;

/**
 * Sound through a wall (0.37): the field-incidence mass law TL = 20 log10(m'' f) - 47 dB (m'' kg/m^2, f Hz; Bies and
 * Hansen, Engineering Noise Control, eq. 7.40 for the region below coincidence), and a wall of several elements
 * (panels, glass, openings and leaks) by area-weighted transmission coefficients, tau = sum(S_i tau_i) / sum(S_i).
 * Coincidence dips and double-wall resonances are not modelled: a closed car's 0.8 mm steel and 4 mm glass with a seal
 * leak of a few cm^2 gives 25-30 dB at 500 Hz, what cabins measure.
 */
public final class Transmission {
    private Transmission() {}

    public static double massLawDb(double surfaceDensityKgM2, double f) {
        if (!(surfaceDensityKgM2 > 0)) { return 0; }
        return Math.max(0, 20 * Math.log10(surfaceDensityKgM2 * Math.max(1, f)) - 47);
    }

    /** A wall element: its area and surface density (0: an opening). */
    public record Element(double areaM2, double surfaceDensityKgM2) {
        public static Element opening(double areaM2) { return new Element(areaM2, 0); }
    }

    /** The wall's transmission loss at f, dB. */
    public static double compositeDb(List<Element> wall, double f) {
        double area = 0, tau = 0;
        for (Element e : wall) {
            if (!(e.areaM2() > 0)) { continue; }
            area += e.areaM2();
            tau += e.areaM2() * Math.pow(10, -massLawDb(e.surfaceDensityKgM2(), f) / 10);
        }
        return area <= 0 || tau <= 0 ? 0 : -10 * Math.log10(tau / area);
    }

    /** What a wall leaves of a sound: a broadband gain (at 500 Hz) and the 4 kHz gain relative to it (a low-pass's two numbers). */
    public record Insulation(double gain, double gainHf) {
        public static final Insulation NONE = new Insulation(1, 1);
    }

    public static Insulation insulation(List<Element> wall) {
        double low = Dsp.dbToGain(-compositeDb(wall, 500)), high = Dsp.dbToGain(-compositeDb(wall, 4000));
        return new Insulation(low, Math.max(1e-3, Math.min(1, high / Math.max(low, 1e-9))));
    }
}
