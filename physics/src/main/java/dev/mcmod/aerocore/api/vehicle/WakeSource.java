package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.SlenderHull;

/** What a part makes a wake with, for the game's wake model (Part.wake). */
public sealed interface WakeSource {
    /** A displacement hull: Michell thin-ship sources of its geometry at the waterline of a displaced volume (m^3). */
    record ThinShip(SlenderHull hull, double designVolumeM3) implements WakeSource {}

    /** A planing hull: a pressure patch of its chine beam (m) and deadrise (rad); its wetted length is the part's diagnostic of that name. */
    record Planing(double chineBeamM, double deadriseRad, String wettedLengthDiagnostic) implements WakeSource {}
}
