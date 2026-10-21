# AeroCore

**The physics behind vehicle mods: air, water, waves, wind, engines, wings, wheels and sound.**

AeroCore is a library mod for Minecraft 1.21.1 (NeoForge). It computes the forces of air and water on any shape,
runs engines from a piston single to a scramjet, wings, propellers, rotors, tyres and drivetrains, and provides a
generic part-based vehicle. It is the prerequisite of
**[High Degree of Freedom Vehicles](https://modrinth.com/mod/hdfv)**, where you design and build vehicles with it.

> **Beta.** The API and the saved data may still change between minor versions.

## What you will see with it

Even on its own, AeroCore changes the world a little:

- **The sea moves:** irregular, second-order waves grown by the wind, refracted and breaking on coasts; wakes, foam
  and spray. Water blocks stay as they are; everything floats on the drawn surface.
- **Wind and sky:** a world-wide wind that follows the weather and the terrain; a physical sky; clouds and rain that
  drift with the wind.
- **Vanilla boats** are rowed with real physics and ride the waves.

## For players

Install it next to High Degree of Freedom Vehicles, on both the client and the server. Client settings: part shading
and "beautify" (normal maps, reflections, lamp glow, contact shadows; off for older graphics cards).

## For mod developers

A pure-Java calculation API (aerodynamics, engines and gas dynamics, wheels and drivetrains, bodies and frames,
acoustics, manufacturing, metallurgy, petroleum) that runs offline, plus a game-side vehicle framework
(`PartVehicle`, blueprints, sync, saving, rendering). Documentation and validation of every model are in the source
repository.

## Requirements

- Minecraft 1.21.1, NeoForge 21.1.252 or newer; client and server.

## Links

- Source and issues: https://github.com/fiseleo/aerocore
- License: LGPL-3.0-only

<details>
<summary>繁體中文</summary>

# AeroCore

**載具 Mod 背後的物理：空氣、水、海浪、風、引擎、機翼、輪子與聲音。**

AeroCore 是 Minecraft 1.21.1（NeoForge）的前置 Mod。它計算任何形狀受到的空氣與水的力，執行從單缸活塞到超燃衝壓的引擎、
機翼、螺旋槳、旋翼、輪胎與傳動，並提供通用的零件化載具。它是 **[高自由度載具](https://modrinth.com/mod/hdfv)** 的前置，
在那裡你用它設計並建造載具。

> **Beta。** API 與存檔資料在小版本之間仍可能變動。

- **海會動：** 隨風成長的不規則二階海浪，在海岸折射、碎浪；尾浪、白沫與水花。水方塊本身不動，所有東西都浮在畫出來的水面上。
- **風與天空：** 隨天氣與地形變化的全世界的風；物理天空；隨風移動的雲與雨。
- **原版船**改成真實的划船物理，也會隨浪起伏。

和高自由度載具一起裝，客戶端與伺服器都要裝。客戶端設定：零件著色與「美化著色」。

需求：Minecraft 1.21.1、NeoForge 21.1.252 以上。授權 LGPL-3.0-only。

</details>

<details>
<summary>简体中文</summary>

# AeroCore

**载具 Mod 背后的物理：空气、水、海浪、风、发动机、机翼、轮子与声音。**

AeroCore 是 Minecraft 1.21.1（NeoForge）的前置 Mod。它计算任何形状受到的空气与水的力，运行从单缸活塞到超燃冲压的发动机、
机翼、螺旋桨、旋翼、轮胎与传动，并提供通用的零件化载具。它是 **[高自由度载具](https://modrinth.com/mod/hdfv)** 的前置，
在那里你用它设计并建造载具。

> **Beta。** API 与存盘数据在小版本之间仍可能变动。

- **海会动：** 随风成长的不规则二阶海浪，在海岸折射、碎浪；尾浪、白沫与水花。水方块本身不动，所有东西都浮在画出来的水面上。
- **风与天空：** 随天气与地形变化的全世界的风；物理天空；随风移动的云与雨。
- **原版船**改成真实的划船物理，也会随浪起伏。

和高自由度载具一起装，客户端与服务器都要装。客户端设置：零件着色与“美化着色”。

需求：Minecraft 1.21.1、NeoForge 21.1.252 以上。授权 LGPL-3.0-only。

</details>
