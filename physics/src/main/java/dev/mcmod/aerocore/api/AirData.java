package dev.mcmod.aerocore.api;

/**
 * Air data an aircraft's instruments and an airfield's reports use (0.38), on the standard atmosphere (ISA,
 * {@link Atmosphere}), in SI units.
 * <ul>
 *   <li>Pressure and density altitude: the ISA altitude with that static pressure or density (-500 to 32 000 m).</li>
 *   <li>QNH and QFE: the station's pressure reduced to mean sea level along the ISA, and back; an altimeter set to
 *   QNH reads the elevation on the ground (ICAO Doc 8896).</li>
 *   <li>Airspeeds: Mach from the speed of sound; calibrated airspeed from the impact pressure a pitot tube measures
 *   (isentropic below Mach 1, Rayleigh's pitot formula behind the normal shock above), referred to sea-level
 *   standard; equivalent airspeed from the density ratio.</li>
 *   <li>Wind on a course: head- and crosswind components; compass bearings with north toward -Z (Minecraft's).</li>
 * </ul>
 * The world's air is the ISA everywhere (no weather systems), so QNH is 1013.25 hPa wherever a station is; the
 * conversions are general for a mod that adds weather.
 */
public final class AirData {
    private AirData() {}

    private static final double R = 287.05287, GAMMA = 1.4, G = Atmosphere.GRAVITY_MPS2;
    /** Sea-level standard pressure (Pa), temperature (K), density (kg/m^3), speed of sound (m/s). */
    public static final double P0 = 101_325, T0 = 288.15, RHO0 = P0 / (R * T0), A0 = Math.sqrt(GAMMA * R * T0);
    private static final double LAPSE = 0.0065, UPPER_LAPSE = 0.001;
    private static final double P11 = Atmosphere.atAltitude(11_000, Vec3d.ZERO).pressurePa(), T11 = T0 - LAPSE * 11_000;
    private static final double P20 = Atmosphere.atAltitude(20_000, Vec3d.ZERO).pressurePa();

    /** The ISA altitude (m, geopotential) with that static pressure. */
    public static double pressureAltitudeM(double staticPa) {
        if (!(staticPa > 0)) { throw new IllegalArgumentException("Pressure must be positive"); }
        if (staticPa >= P11) { return T0 / LAPSE * (1 - Math.pow(staticPa / P0, R * LAPSE / G)); }
        if (staticPa >= P20) { return 11_000 + R * T11 / G * Math.log(P11 / staticPa); }
        return 20_000 + T11 / UPPER_LAPSE * (Math.pow(staticPa / P20, -R * UPPER_LAPSE / G) - 1);
    }

    /** The ISA altitude (m) with that density: where an aircraft performs as it does in this air. */
    public static double densityAltitudeM(double densityKgM3) {
        if (!(densityKgM3 > 0)) { throw new IllegalArgumentException("Density must be positive"); }
        // density falls monotonically with altitude: bisect on the ISA
        double lo = -500, hi = 32_000;
        if (densityKgM3 >= isaDensity(lo)) { return lo; }
        if (densityKgM3 <= isaDensity(hi)) { return hi; }
        for (int i = 0; i < 80; i++) {
            double mid = 0.5 * (lo + hi);
            if (isaDensity(mid) > densityKgM3) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }

    private static double isaDensity(double altitudeM) { return Atmosphere.atAltitude(altitudeM, Vec3d.ZERO).fluid().densityKgPerM3(); }

    /** QNH (Pa): a station's pressure at that elevation (m) reduced to mean sea level along the ISA. */
    public static double qnhPa(double stationPa, double elevationM) { return stationPa * P0 / isaPressure(elevationM); }

    /** QFE (Pa): the pressure at a station of that elevation (m) for a QNH. */
    public static double stationPa(double qnhPa, double elevationM) { return qnhPa * isaPressure(elevationM) / P0; }

    private static double isaPressure(double altitudeM) { return Atmosphere.atAltitude(altitudeM, Vec3d.ZERO).pressurePa(); }

    /** What a pressure altimeter set to that setting (Pa) reads at that static pressure (m). */
    public static double altimeterM(double staticPa, double settingPa) { return pressureAltitudeM(staticPa * P0 / settingPa); }

    /** Speed of sound (m/s) at that temperature. */
    public static double speedOfSound(double temperatureK) { return Math.sqrt(GAMMA * R * temperatureK); }

    /** Mach number of a true airspeed (m/s) at that temperature. */
    public static double mach(double tasMps, double temperatureK) { return tasMps / speedOfSound(temperatureK); }

    /** Impact pressure (Pa) a pitot tube feels at that Mach and static pressure: isentropic, or Rayleigh's behind a normal shock. */
    public static double impactPa(double mach, double staticPa) {
        double m = Math.abs(mach);
        if (m <= 1) { return staticPa * (Math.pow(1 + 0.2 * m * m, 3.5) - 1); }
        return staticPa * (166.9215801 * Math.pow(m, 7) / Math.pow(7 * m * m - 1, 2.5) - 1);
    }

    /** The Mach number giving that impact pressure at that static pressure (inverse of {@link #impactPa}). */
    public static double machFromImpact(double impactPa, double staticPa) {
        if (!(impactPa > 0)) { return 0; }
        double ratio = impactPa / staticPa;
        double subsonic = Math.sqrt(5 * (Math.pow(ratio + 1, 2.0 / 7) - 1));
        if (subsonic <= 1) { return subsonic; }
        double lo = 1, hi = 2;
        while (impactPa(hi, staticPa) < impactPa) { lo = hi; hi *= 2; }
        for (int i = 0; i < 80; i++) {
            double mid = 0.5 * (lo + hi);
            if (impactPa(mid, staticPa) < impactPa) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }

    /** Calibrated airspeed (m/s) for a true airspeed at that static pressure and temperature. */
    public static double casMps(double tasMps, double staticPa, double temperatureK) {
        double qc = impactPa(mach(tasMps, temperatureK), staticPa);
        return A0 * machFromImpact(qc, P0);
    }

    /** True airspeed (m/s) for a calibrated airspeed at that static pressure and temperature. */
    public static double tasMps(double casMps, double staticPa, double temperatureK) {
        double qc = impactPa(casMps / A0, P0);
        return machFromImpact(qc, staticPa) * speedOfSound(temperatureK);
    }

    /** Equivalent airspeed (m/s) for a true airspeed in air of that density. */
    public static double easMps(double tasMps, double densityKgM3) { return tasMps * Math.sqrt(densityKgM3 / RHO0); }

    /** Head- and crosswind (m/s): headwind positive against the course, crosswind positive from the right. */
    public record Components(double headwindMps, double crosswindMps) {}

    /** The components of a wind (the vector it blows toward, m/s) on a course (compass radians: 0 north = -Z, pi/2 east = +X). */
    public static Components components(Vec3d windToward, double courseRad) {
        double hx = Math.sin(courseRad), hz = -Math.cos(courseRad);   // the course's direction
        double rx = Math.cos(courseRad), rz = Math.sin(courseRad);    // to its right
        double along = windToward.x() * hx + windToward.z() * hz, across = windToward.x() * rx + windToward.z() * rz;
        return new Components(-along, -across);
    }

    /** Compass bearing (degrees, 0 to 360, north = -Z) of a horizontal direction. */
    public static double compassDeg(double dx, double dz) {
        double d = Math.toDegrees(Math.atan2(dx, -dz));
        return d < 0 ? d + 360 : d;
    }

    /** The bearing a wind blows from (degrees, as reported), for the vector it blows toward. */
    public static double bearingFromDeg(Vec3d windToward) {
        double d = compassDeg(windToward.x(), windToward.z()) + 180;
        return d >= 360 ? d - 360 : d;
    }
}
