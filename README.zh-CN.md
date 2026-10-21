# AeroCore

[English](README.md) · [繁體中文](README.zh-TW.md) · **简体中文**

Minecraft 1.21.1（NeoForge）载具 Mod 背后的物理。AeroCore 计算任何形状受到的空气与水的力、海浪、风、天空与地面，
从单缸活塞到超燃冲压的发动机、机翼、螺旋桨、旋翼、轮胎与传动，并运行一个通用的零件化载具。它自己不加任何载具：
**[高自由度载具（High Degree of Freedom Vehicles）](https://github.com/fiseleo/hdfv)** 就是建在它上面的 Mod。

> Beta（0.x）：API 与存盘数据在小版本之间仍可能变动。

## 它在你的世界里做什么

AeroCore 是前置 Mod，但单独装也会看到一些改变：

- **海会动。** 海平面高度的开放水域会有真正的海浪（随风成长的不规则二阶海浪，在海岸折射、碎浪）、船后的尾浪、白沫与水花。
  水方块本身不动；水面由 AeroCore 画，所有实体都浮在上面。
- **风与天空。** 随天气、地形与树林变化的全世界的风；物理天空；随风移动的云与雨。
- **原版船**改成真实的划船物理（质量、浮力、划桨），也会随浪起伏。
- 建在它上面的 Mod 的载具**声音**都是即时合成的：没有音档。

## 需求

| | |
| --- | --- |
| Minecraft | 1.21.1 |
| NeoForge | 21.1.252 以上（21.1.x） |
| Java | 21 |
| 端 | 客户端与服务器都要装 |

把 JAR 放进 `mods`，再装使用它的 Mod（高自由度载具）。

## 指令与设置

- `/aerocore wind`、`/aerocore ground`：你所在位置的风与地面（所有人）。
- 管理员：`/aerocore waves ...`（海域、自然生成浪）、`/aerocore wind set|natural`、`/aerocore vehicle ...`（从蓝图生成、查看、让因物理错误停住的载具重新开始），
  以及示范用的 `probe`、`platform`、`hull`、`planing`。
- 客户端设置（Mod 清单 → AeroCore → 设置）：**零件着色**（AeroCore 的着色器，使用光影包时改原版）与
  **美化着色**（法线贴图、反射、灯的光晕、接触阴影；关掉时照 0.38 的样子画，给较旧的显卡）。

## 给 Mod 开发者

AeroCore 是计算 API 加上游戏端的载具框架：

- `dev.mcmod.aerocore.api`（纯 Java，不依赖 Minecraft）：刚体与接触、`api.aero`（翼型、机翼、螺旋桨、旋翼）、
  `api.engine`（活塞循环、涡喷、涡扇、自由涡轮、冲压、超燃、脉冲喷射）、`api.body`、`api.wheel`、`api.drive`、`api.frame`、
  `api.vehicle`（零件蓝图、`VehicleDynamics`）、`api.acoustics`、`api.manufacture`、`api.metallurgy`、`api.petroleum`、`api.power`、`api.mesh`。
  全部都能脱机呼叫，给试算台与设计界面用。
- `dev.mcmod.aerocore.neoforge.vehicle.PartVehicle`：由蓝图创建的通用载具实体，包含同步、存盘、海、风、地面与零件绘制。

游戏 JAR 已包含物理 API；当成 Mod 依赖它，不要再另外加物理 JAR。每个模型的设计说明与验证在 [docs](docs)：
从 [API](docs/API.md) 与 [零件化载具](docs/VEHICLE_PARTS.md) 开始。

## 从原代码建置

JDK 21。Windows 用 `.\dev.ps1 build`（会自己找 JDK 21 并运行 Gradle）；其他系统用 `./gradlew build`。

```
./gradlew build                 # 遊戲 JAR 在 neoforge 的 build/libs
./gradlew publish               # AeroCore 的 Maven 資料夾 build/repo，hdfv 從這裡建置
./gradlew :physics:test         # 物理測試（約 700 個）
./gradlew :neoforge:runGameTestServer
```

开发 run 也会加载 `aerocore_devtools`（AutoShot 自动截屏、开在另一个屏幕且不抢焦点的开发窗口、`/aerocoredev`）：
它在 `neoforge/src/devtools`，不会发布；`verifyReleaseJar` 确保发布的 JAR 里没有开发用程序。

## 限制

模型都对照过公开数据（每份文档都列出来源与误差），但它们是游戏用的物理：没有三维 CFD、没有完整的波浪传播
（海岸是几何光学、尾浪是线性解）、没有旋翼与机翼的干扰、没有轮胎温度。极高速的载具需要前方的世界已加载。

## 授权

AeroCore 以 [GNU 较宽松通用公共授权条款第 3 版（仅此版）](COPYING.LESSER)（LGPL-3.0-only）发布，它创建在
[GNU GPL 第 3 版](COPYING) 之上。来自 NeoForge MDK 范本的文件沿用它们的 [MIT 授权](TEMPLATE_LICENSE.txt)。

文献数据在代码与文档中注明出处；四象限螺旋桨系数（`BSeriesFourQuadrant`）转录自美国海军报告
NSWCCD-50-TR-2006/004。Gradle wrapper 是 Apache-2.0。

历史：[CHANGELOG](CHANGELOG.md)；公开之前的开发纪录在 [docs/HISTORY.zh-TW.md](docs/HISTORY.zh-TW.md)。
