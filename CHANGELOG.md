# Changelog

AeroCore's versions, newest first. Before 0.41 AeroCore was not released; the detailed development log (in Traditional
Chinese) is [docs/HISTORY.zh-TW.md](docs/HISTORY.zh-TW.md), and each model's validation is in [docs](docs).

## 0.41.0: first public release (beta)

- Licence: LGPL-3.0-only (with the GPL v3 it builds on); the MDK template's files stay MIT.
- Development code is out of the release JAR: AutoShot, the development window and `/aerocoredev` are a separate,
  never released mod (`aerocore_devtools`, for the development runs), the GameTests are their own source set, and
  `verifyReleaseJar` checks the JAR.
- Simplified Chinese (zh_cn), made from Traditional Chinese with OpenCC and a table of mainland terms
  (`scripts/zh_cn.py`).
- Mod metadata: authors, links, logo; English, Traditional and Simplified Chinese READMEs.

## Before the release

- 0.40: metallurgy and electrochemistry (formation enthalpies, Hess, Faraday electrolysis, arc furnace, aluminothermics,
  Kroll, Pidgeon, mineral grades), methanol synthesis.
- 0.39: "beautify" shading (normal maps, lamp glow, contact shadows, warmer block light), faster part drawing.
- 0.38: field lamps (runway and approach lights, PAPI, visibility), air data, vehicle tracks.
- 0.37: synthesized sound (DSP, ISO 9613 propagation, Doppler, sonic booms, rooms, engine, turbine, rotor, tyre, wind
  and impact sources; streaming, EFX), far sounds; a fix for chunk unloading hanging the save on shutdown.
- 0.36: couplings (fifth wheel, ball hitch) and telescopic legs.
- 0.35: machining calculations (tool life, grinding, gears, broaching, sawing, threading, riveting, EDM, ECM, laser,
  plasma, waterjet, gases, air separation); 0.35.1 road aero kits calibrated against production cars.
- 0.34: refit (vehicles changed where they stand) and road aero kits.
- 0.33: twisting frames, beam FEM, hardpoint suspension kinematics, air springs, rear steer, clearance;
  0.33.1 the development window on another monitor without focus.
- 0.31 to 0.32: runtime textures (stickers), restyle and place.
- 0.29 to 0.30: part meshes, baking, ambient occlusion, part shaders, interiors, glass, lamps, moving pieces, downwash,
  ground marks, rotor wash.
- 0.28: gas dynamics, solid rockets, hydrogen storage, supersonic inlets, ramjet, scramjet, pulsejet, afterburner,
  turbo-ramjet; very fast vehicles.
- 0.26 to 0.27: power (motors, batteries, cables, belts, line shafts, steam), petroleum (assays, distillation, FCC,
  reforming, cracking, reservoirs, pumping, drilling, operating conditions), diesel and turbocharging, vehicle tanks.
- 0.25: manufacturing (ISO 286 tolerances, cutting, casting, forming, joining, composites).
- 0.24: drivetrain constraint solver, ground rays, tyres, suspension, brakes, axles, landing gear, gearboxes.
- 0.23: modular bodies and hulls, mounts, towing tank; planing deadrise and whisker spray.
- 0.22: airfoils, wings, air propellers, helicopter rotors; 0.22.1 a vehicle's physics error stops only that vehicle.
- 0.21: turbofan and free turbine.
- 0.20: AeroCore becomes a calculation API; part designs move to High Degree of Freedom Vehicles.
- 0.18 to 0.19: piston engine cycle simulation and the single-spool turbojet.
- 0.17: part-based vehicles (blueprints, `VehicleDynamics`, `PartVehicle`).
- 0.16: world-wide wind, natural seas, physical sky, flight atmosphere, ground vehicle physics.
- 0.9 to 0.15: wave regions, irregular and second-order seas, wakes, foam, coasts, sea wind, planing boats, vanilla
  boat physics, sea rendering.
- 0.4 to 0.8: six-degree-of-freedom bodies, buoyancy, contacts, Airy waves, propellers, wave radiation, heat.
