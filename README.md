# AeroCore

**English** · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

The physics behind vehicle mods for Minecraft 1.21.1 (NeoForge). AeroCore computes air and water forces on any shape,
waves, wind, sky and ground, engines from a piston single to a scramjet, wings, propellers, rotors, tyres and
drivetrains, and runs a generic part-based vehicle. It adds no vehicles of its own: **[High Degree of Freedom
Vehicles](https://github.com/fiseleo/hdfv)** is the mod built on it.

> Beta (0.x): the API and the saved data may still change between minor versions.

## What it does in your world

AeroCore is a library, but it changes a few things you will see even on its own:

- **The sea moves.** Open water at sea level gets real waves (irregular, second-order seas grown by the wind, refracted
  and breaking on coasts), wakes behind boats, foam and spray. The water blocks stay as they are; the surface is drawn
  by AeroCore and every entity floats on it.
- **Wind and sky.** A world-wide wind that follows the weather, the terrain and the trees; a physical sky; clouds and
  rain that drift with the wind.
- **Vanilla boats** are rowed with real physics (mass, buoyancy, oar strokes) and ride the waves.
- **Sound** for the vehicles of mods built on it is synthesized as they run: there are no sound files.

## Requirements

| | |
| --- | --- |
| Minecraft | 1.21.1 |
| NeoForge | 21.1.252 or newer (21.1.x) |
| Java | 21 |
| Side | client and server (both need it) |

Put the JAR in `mods`. Install the mod that uses it (High Degree of Freedom Vehicles) next to it.

## Commands and settings

- `/aerocore wind`, `/aerocore ground`: the wind and the ground where you stand (anyone).
- Operators: `/aerocore waves ...` (sea regions, natural seas), `/aerocore wind set|natural`, `/aerocore vehicle ...`
  (spawn a blueprint, inspect, resume a vehicle stopped by a physics error), and the demonstrators `probe`, `platform`,
  `hull`, `planing`.
- Client settings (Mods, AeroCore, Config): **Part shading** (AeroCore's shader, or vanilla's under shader packs) and
  **Beautify** (normal maps, reflections, lamp glow, contact shadows; off draws like 0.38, for older graphics cards).

## For mod developers

AeroCore is a calculation API plus a game-side vehicle framework:

- `dev.mcmod.aerocore.api` (pure Java, no Minecraft): rigid bodies and contacts, `api.aero` (airfoils, wings,
  propellers, rotors), `api.engine` (piston cycle, turbojet, turbofan, free turbine, ramjet, scramjet, pulsejet),
  `api.body`, `api.wheel`, `api.drive`, `api.frame`, `api.vehicle` (part blueprints, `VehicleDynamics`),
  `api.acoustics`, `api.manufacture`, `api.metallurgy`, `api.petroleum`, `api.power`, `api.mesh`. Everything can be
  called offline, for test benches and design screens.
- `dev.mcmod.aerocore.neoforge.vehicle.PartVehicle`: a generic vehicle entity built from a blueprint, with sync,
  saving, sea, wind, ground and part renderers.

The game JAR contains the physics API; depend on it as a mod and do not add the physics JAR as well. The design notes
and validation of each model are in [docs](docs) (written in Traditional Chinese): start with [API](docs/API.md) and
[part-based vehicles](docs/VEHICLE_PARTS.md).

## Building from source

JDK 21. On Windows `.\dev.ps1 build` (it finds a JDK 21 and runs Gradle); elsewhere `./gradlew build`.

```
./gradlew build                 # the game JAR in neoforge's build/libs
./gradlew publish               # AeroCore's Maven folder build/repo, which hdfv builds against
./gradlew :physics:test         # the physics tests (about 700)
./gradlew :neoforge:runGameTestServer
```

The development runs also load `aerocore_devtools` (AutoShot screenshots, a development window that opens on another
monitor without taking focus, `/aerocoredev`): it lives in `neoforge/src/devtools` and is never released;
`verifyReleaseJar` keeps development code out of the release JAR.

## Limits

The models are validated against published data (each doc lists its sources and errors), but they are game physics:
no 3D CFD, no full wave propagation (coasts are geometric optics, wakes are linear), no rotor-wing interference, no
tyre temperatures. Very fast vehicles need the world ahead of them loaded.

## Community

Questions, bug reports, screenshots and news about AeroCore and High Degree of Freedom Vehicles: join the
[Discord server](https://discord.gg/4y2mubMg7e).

## License

AeroCore is released under the [GNU Lesser General Public License v3.0 only](COPYING.LESSER) (LGPL-3.0-only), which
builds on the [GNU GPL v3](COPYING). Files from NeoForge's MDK template keep their [MIT licence](TEMPLATE_LICENSE.txt).

Data from the literature is cited in the code and the docs; the four-quadrant propeller coefficients
(`BSeriesFourQuadrant`) are transcribed from the US Navy report NSWCCD-50-TR-2006/004. The Gradle wrapper is
Apache-2.0.

History: [CHANGELOG](CHANGELOG.md); the development log before the public release is in
[docs/HISTORY.zh-TW.md](docs/HISTORY.zh-TW.md).
