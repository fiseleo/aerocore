package dev.mcmod.aerocore.api.earthmoving;

import dev.mcmod.aerocore.api.GroundContact;

import java.util.List;

/**
 * A material that is dug, pushed, carried and compacted: how heavy it is in place and loose, how it heaps, how strong
 * it is, and how it rubs on steel. Typical values only; which block is which material is the caller's.
 *
 * <ul>
 * <li>{@code bankKgM3}: in-situ ("bank") density, wet, kg/m^3. {@code looseKgM3}: the same material dug and heaped
 *     (in a bucket, a truck, a spoil heap), kg/m^3; the swell factor is bank / loose (bank m^3 to loose m^3). From
 *     the Caterpillar Performance Handbook's table of approximate material characteristics (kg/m^3, bank / loose):
 *     earth, loam 1540 / 1250 (23 % swell); sand, dry loose 1600 / 1420; sand, wet 2080 / 1840 (12 %); gravel, pit run
 *     2170 / 1930 (12 %); clay, natural bed 2020 / 1660 (22 %); clay, wet 25 %; earth, dry packed 1900 / 1510 (25 %);
 *     sandstone 2520 / 1510 (67 %); granite, broken 2730 / 1660 (64 %); magnetite iron ore 3260 / 2790 (17 %).</li>
 * <li>{@code reposeRad}: angle of repose of the loose heap, rad. From the widely reproduced table of bulk angles of
 *     repose (e.g. the Engineering ToolBox and Wikipedia tables after the bulk-handling literature): earth 30 to 45
 *     deg, sand dry 34, sand wet 45, gravel natural with sand 25 to 30, clay dry lump 25 to 40, clay wet excavated
 *     15, snow 38, granite 35 to 40, crushed stone 45, chalk 45, crushed asphalt 30 to 45.</li>
 * <li>{@code cohesionPa} c and {@code frictionRad} phi: Mohr-Coulomb strength in place. Soils from Wong, Theory of
 *     Ground Vehicles, table 2.3 (the same rows as {@link GroundContact.Soil}, so a wheel and a blade see the same
 *     soil), compacted fill from NAVFAC DM-7.02 table 1 (silty sand SM as compacted, c 1050 psf, phi' 34 deg), loose
 *     and blasted materials cohesionless at their angle of repose or, for rockfill, phi 45 deg (Leps 1970, Review of
 *     shearing strength of rockfill, 40 to 55 deg at low stress). For rock-like materials c follows from the strength
 *     and phi by the Mohr-Coulomb relation UCS = 2 c cos(phi) / (1 - sin(phi)).</li>
 * <li>{@code wallFrictionRad} delta and {@code adhesionPa} c_a: soil on steel. Taken as delta = 2/3 phi (the usual
 *     design value for a rough wall, e.g. Das, Principles of Foundation Engineering) and c_a = c / 2 for cohesive
 *     soils (Tomlinson's adhesion factor for steel in clay, 0.3 to 1), 0 for granular soils and rock.</li>
 * <li>{@code compressivePa} and {@code tensilePa}: uniaxial compressive and Brazilian tensile strength of intact
 *     rock-like materials, Pa, 0 for soils. Goodman, Introduction to Rock Mechanics (1989), table 3.1: Nevada Test
 *     Site granite 141 / 11.7 MPa, Pikes Peak granite 226 / 11.9, Nevada Test Site basalt 148 / 13.1, Berea sandstone
 *     74 / 1.2, Bedford limestone 51 / 1.6; ISRM (1981): medium strong rock 25 to 50 MPa, very strong 100 to 250.
 *     Concrete: a 30 MPa structural mix, splitting strength 0.56 sqrt(f'c) (ACI 318). Hot-mix asphalt at 25 C: about
 *     3 MPa unconfined, 1 MPa indirect tensile.</li>
 * <li>{@code moisture}: water content in place, kg of water per kg of solids. {@code proctorKgM3}: standard Proctor
 *     maximum dry density (ASTM D698), 0 for what is not compacted that way; NAVFAC DM-7.02 table 1 ranges (pcf): GW
 *     125 to 135, SP 100 to 120, SM 110 to 125, ML and CL 95 to 120, CH 75 to 105.</li>
 * <li>{@code abrasiveness}: equivalent quartz content, 0..1 (each mineral's share times its Rosiwal abrasiveness
 *     relative to quartz: feldspar about 0.3, calcite 0.04; Thuro 1997). Rough typical values for tool wear: quartz
 *     sand 0.9, granite 0.5, limestone and chalk below 0.1, clays 0.2 to 0.3.</li>
 * </ul>
 *
 * Densities are wet in-place densities; the unit weight is bank density times standard gravity.
 */
public record Material(String name, double bankKgM3, double looseKgM3, double reposeRad, double cohesionPa,
                       double frictionRad, double wallFrictionRad, double adhesionPa, double compressivePa,
                       double tensilePa, double moisture, double proctorKgM3, double abrasiveness) {

    /** Standard gravity, m/s^2: the weight of a kilogram everywhere in this package. */
    public static final double G = 9.80665;

    /** Density of soil solids, kg/m^3: specific gravity 2.70, the usual default for silts and clays (quartz sands 2.65). */
    public static final double SOLIDS_KG_M3 = 2700;

    /** Density of water, kg/m^3. */
    public static final double WATER_KG_M3 = 1000;

    /** Checks the values: densities positive and finite (loose not above bank), angles in range, the rest finite and not negative. */
    public Material {
        if (name == null) { throw new IllegalArgumentException("Material needs a name"); }
        if (!(bankKgM3 > 0) || !(looseKgM3 > 0) || !(looseKgM3 <= bankKgM3) || !Double.isFinite(bankKgM3)) {
            throw new IllegalArgumentException("Densities must be positive and finite, loose not above bank");
        }
        if (!(reposeRad >= 0 && reposeRad < Math.PI / 2) || !(frictionRad >= 0 && frictionRad < Math.toRadians(80))
                || !(wallFrictionRad >= 0 && wallFrictionRad < Math.PI / 2)) {
            throw new IllegalArgumentException("Angles out of range");
        }
        if (!(cohesionPa >= 0) || !(adhesionPa >= 0) || !(compressivePa >= 0) || !(tensilePa >= 0) || !(moisture >= 0)
                || !(proctorKgM3 >= 0) || !(abrasiveness >= 0 && abrasiveness <= 1) || !Double.isFinite(cohesionPa)
                || !Double.isFinite(adhesionPa) || !Double.isFinite(compressivePa) || !Double.isFinite(tensilePa)
                || !Double.isFinite(moisture) || !Double.isFinite(proctorKgM3)) {
            throw new IllegalArgumentException("Strengths, moisture and densities must be finite and not negative");
        }
    }

    /** Topsoil / loam: Caterpillar's earth, loam; strength of Wong's Michigan sandy loam at 11 % moisture. */
    public static final Material LOAM = soil("loam", 1540, 1250, 37, GroundContact.Soil.SANDY_LOAM, 0.5, 0.11, 1750, 0.6);

    /** Dry sand: Caterpillar's sand, dry loose; Wong's dry sand (c 1.04 kPa, phi 28 deg). */
    public static final Material DRY_SAND = soil("dry sand", 1600, 1420, 34, GroundContact.Soil.DRY_SAND, 0, 0, 1760, 0.9);

    /**
     * Wet sand: Caterpillar's sand, wet (97 % saturated at 20 % moisture with 2700 kg/m^3 solids); strength of the wettest
     * sandy soil in Wong's table 2.3, LLL sandy loam at 22 % (c 1.38 kPa, phi 38 deg).
     */
    public static final Material WET_SAND = soil("wet sand", 2080, 1840, 45, 1380, Math.toRadians(38), 0, 0.20, 1760, 0.9);

    /** Gravel: Caterpillar's pit-run gravel; cohesionless, phi 40 deg (Das: gravel with some sand 34 to 48 deg). */
    public static final Material GRAVEL = soil("gravel", 2170, 1930, 30, 0, Math.toRadians(40), 0, 0.05, 2080, 0.7);

    /**
     * Stiff clay: Caterpillar's clay, natural bed; Wong's heavy clay (WES) at 25 % (c 68.95 kPa, phi 34 deg). Moisture
     * taken as 22 %, so that Caterpillar's density stays below saturation.
     */
    public static final Material STIFF_CLAY = soil("stiff clay", 2020, 1660, 32, GroundContact.Soil.FIRM_CLAY, 0.5, 0.22, 1720, 0.3);

    /**
     * Soft, wet clay: Wong's clayey soil at 38 % moisture (c 4.14 kPa, phi 13 deg), all but saturated at that moisture
     * (1835 kg/m^3), Caterpillar's 25 % swell of wet clay.
     */
    public static final Material SOFT_CLAY = soil("soft clay", 1835, 1470, 15, GroundContact.Soil.CLAYEY, 0.5, 0.38, 1440, 0.25);

    /**
     * Mud: Wong's clayey soil at 55 % moisture (c 2.07 kPa, phi 10 deg), saturated at that moisture (1680 kg/m^3); a
     * saturated slurry has no voids to open when dug (swell 1). The angle of repose is taken below the 15 deg of wet
     * excavated clay.
     */
    public static final Material MUD = soil("mud", 1680, 1680, 10, 2070, Math.toRadians(10), 0.5, 0.55, 1440, 0.2);

    /**
     * Settled snow: 300 kg/m^3 (Cuffey and Paterson, The Physics of Glaciers, table 2.1: settled snow 200 to 300);
     * Wong's U.S. snow (c 1.03 kPa, phi 19.7 deg). The swell of broken snow (1.3) is an assumption, no handbook gives one.
     */
    public static final Material SNOW = soil("snow", 300, 230, 38, GroundContact.Soil.SNOW, 0, 0, 0, 0);

    /** Loose fill: dumped loam ({@link #dumped()} of {@link #LOAM}). */
    public static final Material LOOSE_FILL = LOAM.dumped();

    /**
     * Compacted fill: Caterpillar's earth, dry packed (1900 kg/m^3 at 10 % moisture, 96 % of its 1800 kg/m^3 Proctor);
     * NAVFAC's silty sand as compacted, c 50.3 kPa (1050 psf), phi' 34 deg.
     */
    public static final Material COMPACTED_FILL = soil("compacted fill", 1900, 1510, 37, 50.3e3, Math.toRadians(34), 0.5, 0.10, 1800, 0.6);

    /** Soft rock (sandstone, chalk, mudstone): Caterpillar's sandstone; 40 MPa, 4 MPa tensile, phi 35 deg. */
    public static final Material SOFT_ROCK = rock("soft rock", 2520, 1510, 40, 40e6, 4e6, 35, 0.5);

    /**
     * Blasted rock, counted in bank m^3 as mines do: granite broken in place (Caterpillar: 2730 / 1660, 64 % swell),
     * cohesionless rockfill at phi 45 deg (Leps 1970).
     */
    public static final Material BLASTED_ROCK = soil("blasted rock", 2730, 1660, 38, 0, Math.toRadians(45), 0, 0, 0, 0.5);

    /** Hard rock (granite, basalt): granite's densities (Caterpillar); 150 MPa, 12 MPa tensile (Goodman), phi 50 deg. */
    public static final Material HARD_ROCK = rock("hard rock", 2730, 1660, 38, 150e6, 12e6, 50, 0.5);

    /** Iron ore rock: Caterpillar's magnetite iron ore (3260 / 2790); 120 MPa, 10 MPa tensile, phi 45 deg (typical of magnetite ores, 100 to 200 MPa). */
    public static final Material IRON_ORE = rock("iron ore", 3260, 2790, 38, 120e6, 10e6, 45, 0.45);

    /**
     * Asphalt concrete in place: 2350 kg/m^3 (dense-graded mix at 94 % of a 2.5 maximum specific gravity); milled
     * reclaimed asphalt 1600 to 2000 kg/m^3 (FHWA-RD-97-148), 1800 taken; 3 MPa, 1 MPa tensile at 25 C, phi 40 deg.
     */
    public static final Material ASPHALT = rock("asphalt", 2350, 1800, 37, 3e6, 1e6, 40, 0.4);

    /** Concrete: 2400 kg/m^3, 30 MPa, 3 MPa splitting, phi 37 deg; broken, it bulks like crushed stone (67 %, Caterpillar). */
    public static final Material CONCRETE = rock("concrete", 2400, 1440, 40, 30e6, 3e6, 37, 0.45);

    /** Every material above, in that order. */
    public static final List<Material> TYPICAL = List.of(LOAM, DRY_SAND, WET_SAND, GRAVEL, STIFF_CLAY, SOFT_CLAY, MUD, SNOW,
            LOOSE_FILL, COMPACTED_FILL, SOFT_ROCK, BLASTED_ROCK, HARD_ROCK, IRON_ORE, ASPHALT, CONCRETE);

    /** A soil with Wong's strength (a {@link GroundContact.Soil}), delta = 2/3 phi and c_a = adhesionShare c. */
    private static Material soil(String name, double bank, double loose, double reposeDeg, GroundContact.Soil s, double adhesionShare,
                                 double moisture, double proctor, double eqc) {
        return soil(name, bank, loose, reposeDeg, s.cohesionKpa() * 1000, s.frictionAngle(), adhesionShare, moisture, proctor, eqc);
    }

    private static Material soil(String name, double bank, double loose, double reposeDeg, double c, double phi, double adhesionShare,
                                 double moisture, double proctor, double eqc) {
        return new Material(name, bank, loose, Math.toRadians(reposeDeg), c, phi, phi * 2 / 3, c * adhesionShare, 0, 0, moisture, proctor, eqc);
    }

    /** A rock-like material: c from the strength and phi, delta = 2/3 phi, no adhesion, dry, not Proctor-compacted. */
    private static Material rock(String name, double bank, double loose, double reposeDeg, double ucs, double tensile, double phiDeg, double eqc) {
        double phi = Math.toRadians(phiDeg);
        return new Material(name, bank, loose, Math.toRadians(reposeDeg), mohrCoulombCohesionPa(ucs, phi), phi, phi * 2 / 3, 0,
                ucs, tensile, 0, 0, eqc);
    }

    /** Mohr-Coulomb cohesion (Pa) of a material of uniaxial strength ucs (Pa) and friction angle phi (rad): c = UCS (1 - sin phi) / (2 cos phi). */
    public static double mohrCoulombCohesionPa(double ucsPa, double frictionRad) {
        if (!(ucsPa >= 0) || !Double.isFinite(ucsPa) || !(frictionRad >= 0 && frictionRad < Math.PI / 2)) {
            throw new IllegalArgumentException("Strength not negative, friction angle within [0, 90) deg");
        }
        return ucsPa * (1 - Math.sin(frictionRad)) / (2 * Math.cos(frictionRad));
    }

    /** Swell factor: loose m^3 per bank m^3 (bank / loose density). */
    public double swell() { return bankKgM3 / looseKgM3; }

    /** Unit weight in place, N/m^3: bank density times standard gravity. */
    public double unitWeightNPerM3() { return bankKgM3 * G; }

    /** Loose volume (m^3) that a bank volume (m^3) becomes when dug. */
    public double looseM3(double bankM3) { return bankM3 * swell(); }

    /** Mass (kg) of a bank volume (m^3). */
    public double massKg(double bankM3) { return bankM3 * bankKgM3; }

    /** Whether this is rock-like (has a compressive strength): cut by specific energy rather than by soil failure. */
    public boolean isRock() { return compressivePa > 0; }

    /** Dry density in place, kg/m^3: bank / (1 + moisture). */
    public double dryKgM3() { return bankKgM3 / (1 + moisture); }

    /** Relative compaction in place: dry density over the Proctor maximum, 0 when the material has none. */
    public double relativeCompaction() { return proctorKgM3 > 0 ? dryKgM3() / proctorKgM3 : 0; }

    /**
     * The material dug and dumped: bank density the loose density (swell 1, so a dumped heap dug again keeps its
     * volume), no cohesion and no compressive strength, friction at the angle of repose (a loose heap stands at its
     * critical-state friction), delta 2/3 of that; moisture, Proctor density and abrasiveness kept.
     */
    public Material dumped() {
        return new Material(name.endsWith(" (dumped)") ? name : name + " (dumped)", looseKgM3, looseKgM3, reposeRad, 0, reposeRad,
                reposeRad * 2 / 3, 0, 0, 0, moisture, proctorKgM3, abrasiveness);
    }

    /**
     * The material compacted to a relative compaction (0..1.2 of its Proctor maximum): bank density relative x Proctor
     * x (1 + moisture); the loose density stays (capped at the new bank density), so compacted soil swells more when dug
     * again; strength unchanged.
     */
    public Material compacted(double relative) {
        if (!(proctorKgM3 > 0)) { throw new IllegalArgumentException(name + " has no Proctor density"); }
        if (!(relative > 0 && relative <= 1.2)) { throw new IllegalArgumentException("Relative compaction within (0, 1.2]"); }
        double bank = relative * proctorKgM3 * (1 + moisture);
        return new Material(name, bank, Math.min(looseKgM3, bank), reposeRad, cohesionPa, frictionRad, wallFrictionRad, adhesionPa,
                compressivePa, tensilePa, moisture, proctorKgM3, abrasiveness);
    }

    /**
     * The rock as a jointed rock mass of geological strength index gsi (0..100; 100 intact, about 75 blocky, 50 very
     * blocky, 25 disintegrated): compressive and tensile strength and cohesion scaled by the Hoek-Brown factor
     * {@link RockCutting#rockMassFactor} with no blast damage. Not rock-like: unchanged.
     */
    public Material jointed(double gsi) {
        if (!isRock()) { return this; }
        double f = RockCutting.rockMassFactor(gsi, 0);
        return new Material(name + " (GSI " + Math.round(gsi) + ")", bankKgM3, looseKgM3, reposeRad, cohesionPa * f, frictionRad, wallFrictionRad,
                adhesionPa * f, compressivePa * f, tensilePa * f, moisture, proctorKgM3, abrasiveness);
    }
}
