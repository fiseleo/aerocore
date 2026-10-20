package dev.mcmod.aerocore.api.metallurgy;

/**
 * The compounds the metallurgy calculations react (0.40): standard enthalpy of formation at 298.15 K (J/mol) and molar
 * mass (kg/mol).
 *
 * <ul>
 * <li>Enthalpies from the NIST-JANAF tables (Chase 1998) and the CRC Handbook's table of standard thermodynamic
 *     properties; minerals (gibbsite, magnesite, chromite, ilmenite) from Robie and Hemingway, USGS Bulletin 2131
 *     (1995). Ilmenite's value differs by a few kJ/mol between compilations (about -1232 to -1239); -1236 is taken
 *     between them.</li>
 * <li>Molar masses from the IUPAC 2013 standard atomic weights.</li>
 * <li>Elements in their standard state are zero (ELEMENT entries carry only their molar mass).</li>
 * </ul>
 */
public enum Species {
    AL(0, 0.026982), CR(0, 0.051996), FE(0, 0.055845), MG(0, 0.024305), SI(0, 0.028086), TI(0, 0.047867), C(0, 0.012011), CL2(0, 0.070906),
    H2(0, 0.002016), O2(0, 0.031998), NI(0, 0.058693), SN(0, 0.118710), ZN(0, 0.065380), CU(0, 0.063546),
    AL2O3(-1675.7e3, 0.101961), ALOH3(-1293.1e3, 0.078004), CR2O3(-1139.7e3, 0.151990), FECR2O4(-1445.5e3, 0.223837),
    FE2O3(-824.2e3, 0.159688), FEO(-272.0e3, 0.071844), FETIO3(-1236.0e3, 0.151710), TIO2(-944.0e3, 0.079866), TICL4(-804.2e3, 0.189679),
    SIO2(-910.7e3, 0.060084), MGO(-601.6e3, 0.040304), MGCO3(-1095.8e3, 0.084313), MGCL2(-641.3e3, 0.095211),
    CO(-110.5e3, 0.028010), CO2(-393.5e3, 0.044009), H2O_LIQUID(-285.8e3, 0.018015), H2O_GAS(-241.8e3, 0.018015);

    /** Standard enthalpy of formation, J/mol. */
    public final double formationJPerMol;
    /** Molar mass, kg/mol. */
    public final double molarMassKg;

    Species(double formationJPerMol, double molarMassKg) {
        this.formationJPerMol = formationJPerMol;
        this.molarMassKg = molarMassKg;
    }
}
