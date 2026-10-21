# AeroCore

[English](README.md) · **繁體中文** · [简体中文](README.zh-CN.md)

Minecraft 1.21.1（NeoForge）載具 Mod 背後的物理。AeroCore 計算任何形狀受到的空氣與水的力、海浪、風、天空與地面，
從單缸活塞到超燃衝壓的引擎、機翼、螺旋槳、旋翼、輪胎與傳動，並執行一個通用的零件化載具。它自己不加任何載具：
**[高自由度載具（High Degree of Freedom Vehicles）](https://github.com/fiseleo/hdfv)** 就是建在它上面的 Mod。

> Beta（0.x）：API 與存檔資料在小版本之間仍可能變動。

## 它在你的世界裡做什麼

AeroCore 是前置 Mod，但單獨裝也會看到一些改變：

- **海會動。** 海平面高度的開放水域會有真正的海浪（隨風成長的不規則二階海浪，在海岸折射、碎浪）、船後的尾浪、白沫與水花。
  水方塊本身不動；水面由 AeroCore 畫，所有實體都浮在上面。
- **風與天空。** 隨天氣、地形與樹林變化的全世界的風；物理天空；隨風移動的雲與雨。
- **原版船**改成真實的划船物理（質量、浮力、划槳），也會隨浪起伏。
- 建在它上面的 Mod 的載具**聲音**都是即時合成的：沒有音檔。

## 需求

| | |
| --- | --- |
| Minecraft | 1.21.1 |
| NeoForge | 21.1.252 以上（21.1.x） |
| Java | 21 |
| 端 | 客戶端與伺服器都要裝 |

把 JAR 放進 `mods`，再裝使用它的 Mod（高自由度載具）。

## 指令與設定

- `/aerocore wind`、`/aerocore ground`：你所在位置的風與地面（所有人）。
- 管理員：`/aerocore waves ...`（海域、自然生成浪）、`/aerocore wind set|natural`、`/aerocore vehicle ...`（從藍圖生成、查看、讓因物理錯誤停住的載具重新開始），
  以及示範用的 `probe`、`platform`、`hull`、`planing`。
- 客戶端設定（Mod 清單 → AeroCore → 設定）：**零件著色**（AeroCore 的著色器，使用光影包時改原版）與
  **美化著色**（法線貼圖、反射、燈的光暈、接觸陰影；關掉時照 0.38 的樣子畫，給較舊的顯示卡）。

## 給 Mod 開發者

AeroCore 是計算 API 加上遊戲端的載具框架：

- `dev.mcmod.aerocore.api`（純 Java，不依賴 Minecraft）：剛體與接觸、`api.aero`（翼型、機翼、螺旋槳、旋翼）、
  `api.engine`（活塞循環、渦噴、渦扇、自由渦輪、衝壓、超燃、脈衝噴射）、`api.body`、`api.wheel`、`api.drive`、`api.frame`、
  `api.vehicle`（零件藍圖、`VehicleDynamics`）、`api.acoustics`、`api.manufacture`、`api.metallurgy`、`api.petroleum`、`api.power`、`api.mesh`。
  全部都能離線呼叫，給試算台與設計畫面用。
- `dev.mcmod.aerocore.neoforge.vehicle.PartVehicle`：由藍圖建立的通用載具實體，包含同步、存檔、海、風、地面與零件繪製。

遊戲 JAR 已包含物理 API；當成 Mod 依賴它，不要再另外加物理 JAR。每個模型的設計說明與驗證在 [docs](docs)：
從 [API](docs/API.md) 與 [零件化載具](docs/VEHICLE_PARTS.md) 開始。

## 從原始碼建置

JDK 21。Windows 用 `.\dev.ps1 build`（會自己找 JDK 21 並執行 Gradle）；其他系統用 `./gradlew build`。

```
./gradlew build                 # 遊戲 JAR 在 neoforge 的 build/libs
./gradlew publish               # AeroCore 的 Maven 資料夾 build/repo，hdfv 從這裡建置
./gradlew :physics:test         # 物理測試（約 700 個）
./gradlew :neoforge:runGameTestServer
```

開發 run 也會載入 `aerocore_devtools`（AutoShot 自動截圖、開在另一個螢幕且不搶焦點的開發視窗、`/aerocoredev`）：
它在 `neoforge/src/devtools`，不會發布；`verifyReleaseJar` 確保發布的 JAR 裡沒有開發用程式。

## 限制

模型都對照過公開資料（每份文件都列出來源與誤差），但它們是遊戲用的物理：沒有三維 CFD、沒有完整的波浪傳播
（海岸是幾何光學、尾浪是線性解）、沒有旋翼與機翼的干擾、沒有輪胎溫度。極高速的載具需要前方的世界已載入。

## 授權

AeroCore 以 [GNU 較寬鬆通用公共授權條款第 3 版（僅此版）](COPYING.LESSER)（LGPL-3.0-only）發布，它建立在
[GNU GPL 第 3 版](COPYING) 之上。來自 NeoForge MDK 範本的檔案沿用它們的 [MIT 授權](TEMPLATE_LICENSE.txt)。

文獻資料在程式碼與文件中註明出處；四象限螺旋槳係數（`BSeriesFourQuadrant`）轉錄自美國海軍報告
NSWCCD-50-TR-2006/004。Gradle wrapper 是 Apache-2.0。

歷史：[CHANGELOG](CHANGELOG.md)；公開之前的開發紀錄在 [docs/HISTORY.zh-TW.md](docs/HISTORY.zh-TW.md)。
