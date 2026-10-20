package dev.mcmod.aerocore.devtools;

import net.neoforged.fml.common.Mod;

/**
 * AeroCore's development tools (0.41): AutoShot, the development window (DevWindow) and the /aerocoredev client
 * commands, as their own mod so the release JAR carries none of them. Never released: AeroCore's and hdfv's development
 * runs load it (hdfv from build/repo, artifact aerocore-devtools-1.21.1), and so does the clean game of the release
 * check (scripts/release-test). It uses only AeroCore's public classes; it must not be a dependency of anything.
 */
@Mod(AeroCoreDevtools.MOD_ID)
public final class AeroCoreDevtools {
    public static final String MOD_ID = "aerocore_devtools";
}
