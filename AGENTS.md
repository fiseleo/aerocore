# AeroCore 開發代理指南

本文件適用於整個專案。開始修改前先讀本文件，再讀相關模組與文件；功能狀態以實際原始碼、設定和本次測試為準。
與使用者溝通以繁體中文為主，說明完成的功能、驗證結果及仍存在的模型限制。

## 專案目標與現況

- 為 Minecraft Java 載具 Mod 建立通用的空氣／水動力前置。2026-10-11 使用者決定：
  - 載具是另一個獨立 Mod **High Degree of Freedom Vehicles**，在 `Z:\MCMod\HighDegreeOfFreedomVehicles`，有自己的 git 倉庫。AeroCore 只當它的前置，不在 AeroCore 裡做可駕駛的載具。
  - 載具 Mod 走《Automation》式高自由度：玩家自己設計零件（每個參數都能自由設定、形狀也是參數），再用零件組合出載具；合成是 CNC／機加工類。
  - 測試台、設計畫面、物品與模型都在載具 Mod。AeroCore 提供零件藍圖格式、「設計參數 → 效能」的計算、組裝與連接、通用載具物理，
    以及可以離線呼叫的計算。設計見 [docs/VEHICLE_PARTS.md](docs/VEHICLE_PARTS.md)（0.17 範圍已經使用者確認）。
- 先維持可驗證的物理核心，再逐步接入載具玩法。不要把近似模型描述成工程級 CFD。
- 截至 2026-10-11，版本為 **0.19.0**：具備六自由度剛體、四角浮力、可調配重、受力／環境流速粒子顯示、溫壓流體性質與基礎熱交換；
  0.6 加入球形元件接觸反力（摩擦／反彈）、指定的線性 Airy 波浪、BEMT 螺旋槳推力（平台艉部示範槳）與阻力摩擦生熱；
  0.7 加入接觸摩擦生熱、波壓深度衰減（垂直 Froude–Krylov）、水平 Froude–Krylov 力與附加質量，測試球改用六自由度剛體與 `ContactSolver`；
  0.8 加入浮筒垂直方向的波浪輻射阻尼（Hulme 半球資料、Cummins 記憶力）與浮筒間點吸收體干擾；
  0.9（2026-10-10）加入世界波浪區域與客戶端波面、與輻射一致的浮筒垂直繞射表、Wageningen B 系列螺旋槳、
  細長船體（阻力與操縱）與指令操作的測試船／舵、物體之間的雙向接觸（`BodyWorld`），以及原版實體站上甲板（mixin）；
  0.10（2026-10-10）加入 JONSWAP 不規則海況、所有讀取者共用的移動水面（原版實體／原版船／鏡頭霧 mixin）、Kelvin 尾浪與水花、
  Zarnick 2D+t 滑行艇（對 Fridsma 1969 驗證）與指令操作的滑行測試艇，並把示範係數換成文獻值（見 docs/PHYSICS.md〈0.10 係數來源〉）；
  0.11（2026-10-10）把水面改成二階（Choppy Wave Model）、尾浪改成每個來源一個 FFT 網格的線性非定常解、加入碎浪白沫（Monahan 白冠覆蓋率），
  原版船改成真實划船物理（`Rowboat`／`RowingBoats`，使用者選擇；可駕駛的新載具使用者決定先不做），方塊依材質分類給摩擦與恢復係數，滑行艇加鬚狀水花阻力；
  0.12（2026-10-10，使用者要求「浪頭、波浪碰到地形、風、乘客會跟船一起傾斜、單槳轉向、滑行艇船頭仰角」）加入 Sharma–Dean 完整二階浪頭、
  海岸（測量海床、折射／淺化／遮蔽／碎浪，`CoastalWaves`）、海域的風（`SeaWind`：風浪、白冠、表層流、船與物體受風、水花受風）、
  乘客隨船傾斜與第一人稱滾轉、槳葉速度上限（單槳轉向）、滑行艇的 Garme 艉板修正（只乘在靜水壓）；
  0.13（2026-10-10，使用者回報「有浪的時候沒有浪的動畫、FPS 很卡」並要求「要有陣風和 Minecraft 天氣連動」）修正窄水道相連的湖被當成湧浪陰影而沒浪
  （`windsea` 改為每點依自己的直線風域成長）、白沫與畫面的效能，加入陣風（`WindGusts`）與隨天氣的風（`wind weather`）、`survey clear`、畫面略去混疊的短波；
  0.14（2026-10-10，使用者要求「把不擬真的地方改到擬真、浪渲染效果不好（只有白色一層）」）水面改光學著色（`WaterOptics`）、
  風浪隨時間（`SeaState`）、地形擋風、靠岸浪頭變尖（當地核）、原版船逐槳推力；
  0.14.1（同日，使用者回報）近處的陽光閃爍改為 16～48 m 漸入（原本中午往下看有一圈亮盤），修正池塘風浪在水底附近 0/0 造成的玩家 tick 崩潰；
  0.15（2026-10-10，使用者要求「波浪要真實一點，只有白白的一層不好看」）水面改自訂 shader 逐像素著色（網格以下的波與風的小波紋、天空漸層、
  岸邊倒影、Smith 遮蔽的閃爍），風浪成長受陡度上限、白冠依 Monahan 校準、遠處畫平均白沫，水面計算跳過零振幅分量（遊戲內 FPS 約 2～3 倍），
  並加入開發用自動截圖 `AutoShot`；
  0.16（2026-10-11，使用者要求「船的軌跡是白白一層不好看、自然生成浪、空中地上也要真實、陸海空都要」並選了全世界的風、天空畫面、
  地面載具物理、飛行用大氣四項）白沫改泡沫網紋、自然生成浪（`NaturalSeas`）、全世界的風（`SurfaceWind`／`WorldWind`）、
  物理天空與隨風的雲與雨雪（`SkyModel`／`SkyRenderer`）、飛行用大氣（`FlightAtmosphere`）、地面載具物理（`GroundContact`／`GroundSurfaces`）；
  0.16.1（2026-10-11，使用者回報「crash 了 優化一下 有點卡」）修正微風時 NaN 造成的划船當機（`SeaWind.LIGHT_AIR`），
  消除自然海域換區的卡頓（只重建改變的 section 並分散、水面快取移交、水色格點、新區塊預算），並快取天空狀態與波浪輻射核函數；
  0.17（2026-10-11，使用者確認範圍：藍圖格式、零件編譯、組裝與連接、通用載具物理；測試台在載具 Mod）加入 `api.vehicle`
  （九種零件、JSON 藍圖、質量性質、動力軸傳動系、`VehicleDynamics`）與遊戲端 `PartVehicle`，並修正原版船在單人遊戲兩端共用狀態造成的客戶端崩潰；
  0.18（2026-10-11，使用者要求「先做零件」「引擎可以自己做」並列出內燃機元件）加入活塞引擎：`api.engine` 的逐曲軸角循環模擬、
  16 個各自設計的引擎元件組成 `aerocore:piston_engine` 零件，以及零件內部狀態（`Part.internalStates`）；
  0.19（2026-10-11，使用者列出噴射引擎元件並要求「開始做噴射引擎」）加入單軸渦輪噴射：`api.engine.Turbojet`（變比熱循環、設計點、
  非設計點平衡、依飛行馬赫數的運轉線、準穩態暫態），進氣道、壓縮機、燃燒室、渦輪、噴嘴／尾管、燃油控制六個元件組成 `aerocore:jet_engine` 零件。
- 獨立二維 CFD 實驗與離線流場圖仍保留。遊戲內粒子顯示目前是環境取樣；尾浪是線性自由面的非定常解（勢流），不是黏性流場求解。
- AeroCore 不出可駕駛的載具（`PartVehicle` 是給載具 Mod 用的通用實體，AeroCore 只用範例藍圖與指令測它）；尚無增壓、柴油、渦扇與後燃器、輪子與懸吊、機翼零件、旋翼、三維 CFD、碎浪動力、
  波浪反射、遠方風暴的湧浪、大氣層結（背風波）、氣象動力學、相變或完整熱流耦合；海岸是幾何光學（無反射、島後只有一個到達波）。
- 船體模型刻意做成水面船與潛航體都能用的細長體；載具玩法屬於載具 Mod，不要放進 AeroCore。

## 固定開發基底

| 項目 | 目前設定 |
| --- | --- |
| Minecraft | Java Edition 1.21.1 |
| NeoForge | 21.1.252 |
| Java | JDK 21，64 位元 |
| ModDevGradle | 2.0.148 |
| Gradle Wrapper | 9.2.1 |
| Mod ID / Java package | `aerocore` / `dev.mcmod.aerocore` |
| IDE | IntelliJ IDEA Community Edition |

版本號以 `gradle.properties` 和建置設定為準。不要為一般功能修改順便升級 Minecraft、NeoForge、Gradle 或 Java。
對外授權尚未決定，中繼資料為 `All Rights Reserved`；不要自行更改授權。

## 工作目錄與本機資料

- 2026-10-11 起 `Z:\MCMod` 底下分成兩個專案資料夾，`Z:\MCMod` 本身不是 git 倉庫，只有一份說明兩者關係的 `AGENTS.md`：
  - `AeroCore`：本專案。
  - `HighDegreeOfFreedomVehicles`：載具 Mod，獨立倉庫。
- `Z:\MCMod\AeroCore` 與 `\\100.96.25.71\share\MCMod\AeroCore` 是同一份專案的別名，不是兩個 checkout。
- Windows Shell 操作優先使用 `Z:\MCMod\AeroCore`，避免 CMD 的 UNC 工作目錄限制。
- 2026-10-11 從 `Z:\MCMod` 搬進 `Z:\MCMod\AeroCore`（`.git`、`.idea` 一起搬）。兩台電腦的 IntelliJ 要改開新位置，再重新載入 Gradle；
  `.idea/workspace.xml` 裡舊路徑的執行設定快取會在重新載入後重建。
- 原始碼及 JAR 在專案內；遊戲世界、設定、記錄與截圖必須留在本機。
- 本機執行根目錄：`%USERPROFILE%\.mcmod\runs\aerocore`，每台電腦各自一份：原開發機（`DESKTOP-AH3A4MO`）為 `C:\Users\ZHU YU TA\.mcmod\runs\aerocore`，`DESKTOP-KLLC9QQ` 為 `C:\Users\CHU YU TA\.mcmod\runs\aerocore`。
- 2026-10-10 起專案是 git 倉庫，依使用者要求上傳到公開的 GitHub `fiseleo/aerocore`；網路磁碟上的 git 指令要加
  `-c safe.directory=Z:/MCMod/AeroCore -c safe.directory=%(prefix)///100.96.25.71/share/MCMod/AeroCore`（不改全域設定）。推送是對外公開的動作，每次先經使用者同意。
  大改前仍把原始碼備份到 `%USERPROFILE%\.mcmod\backups\`（0.9 開工前 `aerocore-src-0.8.0-20261010-002942`，0.11 開工前 `aerocore-src-0.10.0-20261010-130423`；
  0.12 開工時的狀態是 git commit `be82bc5`，沒有另做資料夾備份）。
- `client` 放開發客戶端資料，`server` 放開發伺服器資料，`gametest` 放自動測試世界。
- `aerocore_runtime_id=aerocore` 保持磁碟代號與 UNC 啟動結果一致。只有獨立實驗專案才使用不同 ID。
- JDK 位於 `%USERPROFILE%\.mcmod\java\jdk-21.0.12.1+1`，Java 暫存位於 `%USERPROFILE%\.mcmod\tmp`。優先使用啟動腳本偵測 JDK，不依賴舊的 AppData 路徑。
- NeoForm 暫存已放在 Gradle 使用者目錄的 `caches/aerocore-work`；不要改回網路磁碟。
- 所有遊戲 run（client／server／gameTestServer）在 `neoforge/build.gradle` 固定 `java.io.tmpdir` 與 `jdk.net.unixdomain.tmpdir`
  為 `%USERPROFILE%\.mcmod\tmp`。`DESKTOP-KLLC9QQ` 的系統 TEMP 無法建立 Java NIO 的 Unix socket（`Invalid argument: connect`），
  會讓建立單人世界時崩潰；不要移除這兩個設定。

`neoforge/run` 是遷移前的備份，**不是目前遊玩的存檔**。原開發機已複製兩個世界與設定並逐檔驗證，78 個檔案的紀錄在該機本機根目錄的 `migration-manifest.json`。
`DESKTOP-KLLC9QQ` 於 2026-10-07 建立環境時依使用者決定不複製世界，沒有 manifest；需要世界時由使用者從原開發機複製 `client/saves` 或新建 QA 世界。
2026-10-09 首次啟動客戶端時，使用者在此機另建了自己的「新的世界」（與原開發機的同名世界無關）。
不要自動重跑遷移、覆寫本機進度或刪除備份。`scripts/migrate-runtime.ps1` 僅供尚未遷移的環境；來源和目標世界均須關閉，腳本會取得鎖並拒絕內容衝突。
測試使用 `AeroCore 0.4 Platform QA` 或新建的 QA 世界，兩台電腦都不修改使用者的「新的世界」。測試結束由遊戲正常存檔離開，不以強制結束程序代替存檔。
`DESKTOP-KLLC9QQ` 有 Chrome Remote Desktop（`remoting_host`）。經它遠端操作時 Minecraft 滑鼠視角異常（只能低頭左右轉，視窗常失焦暫停）；
先在「選項 → 控制 → 滑鼠設定」關閉「原始輸入」，仍不行就改在本機或用支援相對滑鼠的遠端工具（Parsec、Sunshine＋Moonlight）實測畫面。關閉原始輸入的效果尚未確認。

## 常用指令

在專案根目錄執行：

```powershell
.\open-ide.cmd --check
.\open-ide.cmd
.\dev.ps1 build
.\dev.ps1 :physics:test :cfd:test
.\dev.ps1 :neoforge:runGameTestServer
.\dev.ps1 :neoforge:showRuntimeDirectories
.\dev.ps1 :neoforge:runClient
.\dev.ps1 :cfd:run
.\dev.ps1 :cfd:run --args="air"
```

網路磁碟的未簽署 PowerShell 腳本可能被拒絕；使用單次程序設定即可，不更改整台電腦的執行原則：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\dev.ps1 build
```

代理執行時的已知狀況：
- 把 `dev.ps1` 的輸出導到檔案再管線讀取時，Gradle daemon 會佔住 stdout，前景指令可能不結束；改用背景執行並讀取記錄檔。
- 在網路磁碟上用 `--rerun` 強制重編，偶爾出現 `could not create parent directories`；不加 `--rerun` 重跑即可，不是程式錯誤。
- 搬動專案資料夾或大量刪除 `build` 之後，沿用的 Gradle daemon 對網路磁碟的檔案狀態會過時
  （`Failed to create MD5 hash ... does not exist`、`Cannot create directory`、`Unable to delete file`），重跑也一樣。
  先 `dev.ps1 --stop` 停掉 daemon，第一次建置加 `--no-watch-fs` 就正常（2026-10-11 搬進 `AeroCore` 時遇到）。

IDE 使用 `.run/` 裡的 Gradle 執行設定。建置設定變更後重新載入 Gradle，再執行 **Minecraft Client**。
IDE 優先用 `open-ide.cmd` 開：專案的 Gradle JVM 設為 `#JAVA_HOME`，而 `DESKTOP-KLLC9QQ` 的系統 `JAVA_HOME` 是 JDK 17、系統 TEMP 無法建立 Unix socket。
從開始選單開啟時，遊戲 run 仍因 `neoforge/build.gradle` 的暫存設定可以啟動，但不要依賴系統環境變數。
兩台電腦共用專案內的 `.idea`，不要同時在兩台開同一個專案。
不要自動重裝 IDE、改 Windows 安全設定或更換既有 JDK；先查看腳本、路徑和錯誤記錄。

## 模組邊界與入口

| 位置 | 職責 |
| --- | --- |
| `physics/src/main/java/dev/mcmod/aerocore/api/` | 純 Java 物理 API，不得依賴 Minecraft／NeoForge |
| `cfd/` | 獨立二維 D2Q9 LBM、解析驗證與 CSV 輸出，不在遊戲 tick 中執行 |
| `neoforge/src/main/java/dev/mcmod/aerocore/neoforge/` | Mod 註冊、指令、Minecraft 整合 |
| 上述目錄的 `probe/` | 測試球、平台、環境取樣、粒子顯示與 GameTest |
| 上述目錄的 `client/` | 客戶端模型與渲染（含波面），避免專用伺服器載入客戶端類別 |
| physics 的 `api/vehicle/` | 零件化載具（0.17）：零件藍圖、組裝、`VehicleDynamics`；0.18 的引擎元件與 `PistonEngineDesign`；0.19 的噴射引擎元件與 `JetEngineDesign`；純 Java，載具 Mod 的測試台可離線呼叫 |
| physics 的 `api/engine/` | 活塞引擎的物理（0.18）：氣體、燃料、循環模擬、摩擦、點火標定；渦輪噴射的循環（0.19，`Turbojet`）；純 Java，可離線呼叫（測功機、測試台） |
| neoforge 的 `vehicle/` | 給載具 Mod 用的公開遊戲端 API：`PartVehicle`、`VehicleRegistry`、`VehicleNetwork`、`VehicleCommands`、`VehicleGameTests` |
| neoforge 的 `mixin/` | `aerocore.mixins.json`：雙端 `EntityCollideMixin`（實體對物體碰撞）、`EntityWaterMixin`（原版水高度與視線讀海面）、`BoatWaterMixin`（原版船水面）、`BoatPhysicsMixin`（原版船改由 `RowingBoats` 移動、槳速）、`BoatSeatMixin`（座位隨船傾斜）、`BoatAccessor`、`LivingEntityAccessor`；客戶端 `LiquidBlockRendererMixin`（隱藏海面頂面）、`CameraFogMixin`（水下霧）、`BoatRendererMixin`（船的俯仰橫搖）、`BoatClientMixin`（槳動畫內插）、`RiderTiltMixin`（乘客模型隨船傾斜）、`SkyDomeMixin`（物理天空）、`CloudWindMixin`（雲隨風）、`WeatherWindMixin`（雨雪隨風傾斜）；只能放 mixin 類別 |
| `scripts/` | JDK／IDE／本機資料與離線 CFD 繪圖工具 |
| `docs/` | API、模型假設、操作和驗證說明 |

主要類別：`AeroSolver`、`RigidBodyDynamics`、`FloatingBodyDynamics`、`ContactSolver`、`LinearWave`／`WaveField`、
`PropellerSolver`／`BladeElementPropeller`／`WageningenB`、`WaveRadiation`、`SlenderHull`／`HullSection`／`HullModel`／`HullDynamics`／`HullResistance`／`HullManoeuvring`、
`Rudder`、`Thermodynamics`、`HeatTransfer`；0.10：`SeaSpectrum`／`CompiledWaves`、`KelvinWake`、`BowSpray`、`PlaningHull`／`PlaningDynamics`／`TestPlaningBoat`、
`SphereDrag`、`BSeriesFourQuadrant`、`ContactMaterial`（Thornton）；0.11：`UnsteadyWake`／`Fft2D`、`Rowboat`、`Whitecaps`；
0.12：`SecondOrderCorrection`、`CoastalWaves`、`SeaWind`；0.13：`WindGusts`；0.14：`WaterOptics`（0.15 加 `ripples`、`smithLambda`）；
遊戲端入口為 `FloatingPlatform`、`FloatingHull`、`PhysicsProbe`（都實作 `CoupledBody`、`SolidBody`）、`BodyWorld`（每 tick 聯合推進）、
`BodyCollisions`（原版實體）、`WaveRegions`／`WaveNetwork`／`WaveCommands`、`PlatformCommands`、`HullCommands`、`PlatformDebug`、`WorldWaterSampler`；
0.10：`SeaSurface`（共用水面）、`VanillaWater`（原版實體）、`Wakes`（尾浪與靜水區）、`EntityMasses`、`PlaningBoat`／`PlaningCommands`，客戶端 `WaveSurfaceRenderer`、`SprayRenderer`；
0.11：`RowingBoats`（原版船）、`BlockMaterials`，客戶端 `SeaFoam`；0.12：客戶端 `BoatCamera`（第一人稱隨船滾轉）；
0.15：客戶端水面 shader `assets/aerocore/shaders/core/rendertype_aerocore_sea.{json,vsh,fsh}`、開發用 `AutoShot`；
0.16：`SurfaceWind`、`FlightAtmosphere`、`SkyModel`、`GroundContact`；遊戲端 `WorldWind`（全世界的風）、`WindCommands`、`NaturalSeas`、`GroundSurfaces`，
客戶端 `SkyRenderer` 與天空 shader `aerocore_sky`；
0.17：`api.vehicle` 的 `PartDesign`／`PartTypes`／`Part`／`PartStep`、九種零件設計（`FloatDesign`、`HullDesign`、`PlaningHullDesign`、`PropellerDesign`、
`ControlSurfaceDesign`、`BoxDesign`、`MotorDesign`、`BallastDesign`、`SeatDesign`）、`VehicleBlueprint`／`Vehicle`／`VehicleState`／`VehicleDynamics`／`VehicleEnvironment`、
`Json`、`SampleBlueprints`；遊戲端 `PartVehicle`，客戶端 `PartVehicleRenderer`、`VehicleControls`；
0.18：`api.engine` 的 `Species`／`Gas`、`Fuel`、`CrankKinematics`、`ValveLift`、`ValveFlow`、`Combustion`、`EngineOil`、`Friction`、`EngineCycle`、`Dyno`；
`api.vehicle` 的 `EngineComponent` 與 16 個元件設計、`EngineMaterial`、`PistonEngineDesign`、`SampleEngines`；
0.19：`api.engine.Turbojet`；`api.vehicle` 的 `JetInletDesign`、`CompressorDesign`、`CombustorDesign`、`TurbineDesign`、`JetNozzleDesign`、`JetControlDesign`、
`JetEngineDesign`，範例 `SampleEngines.turbojet()`／`modelTurbojet()`、`SampleBlueprints.jetBoat()`。
修改來源檔，不直接修改 `build/`、Gradle 快取或生成的 Mod 中繼資料。

## 物理與顯示約定

- 核心使用 SI：m、kg、s、N、Pa、J、W、K、rad。1 格視為 1 m；Minecraft tick 為 0.05 s，平台每 tick 10 個 0.005 s 小步。
- `RigidBodyState.position` 是世界重心；速度與角動量也是世界向量。姿態為 body → world 的單位四元數，建構順序 `(w,x,y,z)`。
- `centerOfMassLocal`、配重與浮筒幾何使用模型局部座標；`AppliedForce.offset` 是從該步開始重心出發的世界向量。
- 水取樣的 `wetCentroidOffset` 是從浮筒球心出發的世界向量。不可與重心相對向量混用；力矩為 `r × F`。
- `FloatingBodyDynamics.step` 已加入重量、浮力和阻力，不得再重複加力；推力等外加負載經 `externalLoads` 傳入，接觸經 `Collision` 傳入。
  動量收支為 hydrostatics＋averageDrag＋inertia＋contact＋external（輻射力屬於 external）。顯示箭頭應來自同一份實際積分資料。
- 保持正定慣性張量、有限輸入、單位檢查，以及既有阻力耗能與對稱性；不以任意能量夾值掩蓋數值不穩定。
- 平台以伺服器為物理權威。避免用渲染頻率推進物理，也不要為平台強制載入世界區塊。
- 地形接觸採 `CollisionSphere` 對靜止 `WorldBox` 的序列衝量（庫侖摩擦、恢復係數、推測接觸、只改位置的穿入修正）。
  只有單一運動物體，障礙物不被推動；推測接觸可能在表面前 v·dt 內反彈。不要用速度夾值或能量夾值掩蓋接觸問題。
- 波浪是給定的一階 Airy 流場（H/L ≤ 0.1、H ≤ 0.78h），只改變平台取樣，不改水方塊、不反射。
  浮力用沿球弦積分的線性波壓（`pressureFraction`，隨深度衰減）；水平 Froude–Krylov 力與附加質量（C_a 0.5·ρV_wet）
  由 `FloatingBodyDynamics` 加入，附加質量在隱式速度解與「物體＋夾帶水」的 drift 中處理。靜水也有附加質量；接觸衝量不含附加質量。
  環境 `Contact` 只回報幾何、壓力與水的加速度，不要在外部再加一次附加質量或 Froude–Krylov 力。
- 波浪輻射由呼叫者持有的 `WaveRadiation.Memory` 提供（每個物體一份、固定小步長；每步先 `loads` 再 `record`）。
  0.5·ρ·V_wet 是高頻極限 A∞，頻率相依附加質量與阻尼來自 8 s 記憶卷積；單浮筒用同水線面半徑的 Hulme 半球資料，
  浮筒間用 √(B_iB_j)·J₀(kd) 干擾。只有垂直輻射；深水、開放水域假設，無池壁反射與近場干擾。不要用 Froude–Krylov＋Haskind 取代（已驗證誤差達 2 倍）。
- 0.9：穿出水面的浮筒垂直繞射改用半球激振力表（本專案以多極法算出，以 Hulme 阻尼經 Haskind 關係驗證）。
  `Contact.surfaceDiffraction` 與 `Memory.diffractionLoads` 必須成對使用，否則會少算或重算垂直繞射；水平仍是 Morison。
- 波浪是世界波浪區域（`WaveRegions`，SavedData，同步給客戶端），不再屬於單一平台；只有高度等於區域靜水面的開放水面會動，
  外圈振幅漸弱是視覺／數值接合。客戶端以 mixin 隱藏該水面的原版頂面並重畫；水方塊不動。
- 螺旋槳用四象限 C_T*／C_Q*；遊戲用 1° 插值表。0.10 的示範槳前進象限是 Wageningen 回歸式（平台／測試船 B3-65、滑行艇 B4-70 P/D 1.4），
  其他象限是 MARIN 量測四象限 Fourier 資料（`BSeriesFourQuadrant`），以 15° 漸變區接合。反扭矩 −Q·axis 以 `AppliedForce.couple` 傳遞。
- 船體（0.9）：條帶靜水壓、ITTC-57 × Holtrop–Mennen ＋ Michell 阻力、條帶附加質量（6×6 隱式）＋ 顯式 Kirchhoff 項（Munk 力矩）、
  以 Clarke 1983 校準的尾端升力元件、Hoerner 橫流阻力。升力元件已扣除 Munk 偶力，不要再另加 Munk 力矩或 Clarke 的 Y_v／N_v。
  Michell 不適用於寬的浸水艉板與滑行；`HullModel` 建構時算表約 1 s。
- 物體之間的接觸（0.9）由 `BodyWorld` 在 `LevelTickEvent.Pre` 聯合求解：實體的 `tick()` 不再推進物理，新物體需實作 `CoupledBody`。
  原版實體：`EntityCollideMixin` 呼叫 `BodyCollisions.clip`；重量用真實 g 在伺服器的站立檢查中加入（Minecraft 實體重力約 32 m/s²，不可拿它的動量當重量）；
  被載運的位移在 `EntityTickEvent.Pre`：伺服器搬非玩家、客戶端搬本機玩家。
- 粒子箭頭預設關閉、只傳給附近擁有者且有數量預算；世界流速與相對平台流速必須標明。比例尺／截斷值變更時同步改提示和文件。
- 0.10 海面：海域（常駐，或尾浪自動帶出的臨時靜水區）內所有讀取者都經 `SeaSurface` 讀同一個水面（靜水面＋漸弱環境波＋尾浪），
  時間為遊戲刻 × 0.05 s；擁有判定是靜水面高度的水源方塊、上方開放。不要讓某個讀取者自行改用水方塊頂面或另算波高。
  不規則海用 32 個單一求和分量（`SeaSpectrum`），物理取樣用 `CompiledWaves`（結果與逐點公式相同，不要改回逐分量呼叫 `LinearWave`，太慢）。
  原版實體的波浪漂移用其原版水中阻尼 f（原版船除外）。
- 0.11 水面與尾浪：水面形狀是 CWM 二階（`CompiledWaves.surfaceTerms`／`gridSecondOrder`，漸弱 w 對二階乘 w²），水下速度與壓力仍線性；
  所有讀取者都要用二階水面。尾浪由 `Wakes` 為每個來源維持一個 `UnsteadyWake`（背景執行緒每 tick 一步、讀快照），
  只以水面高低作用在其他物體上，物體排除自己的尾浪；原版船例外，以自身尾浪坡度計算興波阻力。GameTest 要設 `Wakes.synchronous = true`。
  `KelvinWake` 只作為驗證的定常解。碎浪門檻由 `Whitecaps` 對準 Monahan 覆蓋率，白沫是客戶端視覺（`SeaFoam`）。
- 0.11 原版船：`RowingBoats` 在移動船的那一端取代 floatBoat／controlBoat，位移交給原版 `move()`；其他客戶端只算俯仰橫搖。
  船底壓力（尾浪與波阻）用有翹度的拋物線分布，不要改回方形銳邊（會產生假的阻力起伏）；按鍵到槳的對應是操作規則。
  0.12：槳葉相對船的速度上限 `Rowboat.BLADE_SPEED`（槳把 1.3 m/s，Kleshnev）限制滑移，單槳首搖穩定約 58°/s；小艇單槳轉向沒有量測可對照。
  乘客隨船傾斜只改座位點與模型、鏡頭滾轉（顯示），不是新的力。
- 0.12 浪頭：二階水面 = CWM（便宜）＋ `SecondOrderCorrection`（Sharma–Dean 與 CWM 的差，低秩二次式，依波向量與水深快取、振幅以 fold 摺入）。
  畫面 32 m 外只畫便宜部分；球體取樣的修正項只在球心欄算一次。不要把修正項改成逐對雙重和（太慢）。
- 0.12 海岸：有波浪的海域建立時測量海床（`WaveRegions.survey`，2 m 格點，存檔並同步），`CoastalWaves` 在背景執行緒建立（完成前用開放海面）。
  所有讀取者都要用海岸水面：點查詢 `CompiledWaves.surfaceTerms(..., coast, ...)`，物體用 `WaveRegion.localWaves`（WKB 局部波場，頻率不變），
  畫面用 `gridCoastal`。自由場的陸地用最近水域的深度（不要改回外海深度：會讓波穿過陸地抄捷徑、把淺岸誤判成陰影）；
  封閉水域的波源取自由場的相位；路徑差小於 `SHADOW_NOISE` 視為照亮。寬度不到 2 m 的牆可能被測點漏掉。
- 0.13 風浪：有 `Sea.windMps`／`fetchBeyondM` 的海是風浪，`CoastalWaves` 用 `WindSea` 模式（每點直線風域的成長取代陰影，相位用自由場）；
  沒有的是湧浪，用傳播＋Sommerfeld 遮蔽（被陸地隔開、只靠窄水道相連的水域幾乎沒有湧浪，這是對的）。短風域 `SeaWind.sea` 以 DNV 陡度 1/15 為上限。
  畫面用 `WaveRegion.drawnWaves(間距)`（略去短於 4 個頂點間距的分量，避免混疊）；物理仍用完整波場。
- 0.13 陣風與天氣：`WaveRegion.Wind` 可帶雨／雷雨風速（隨 `level.getRainLevel`／`getThunderLevel` 內插，是規則）；`Wind.mean(level)` 給平均風，
  `WaveRegion.gust` 給陣風（`WindGusts`，種子取自海域位置，兩端相同）。平均風用於白冠與表層流，陣風加在船、物體、水花的風上。
  物體每 tick 用 `WaveRegions.at(level, x, z)` 取含風的 `Local`。
- 0.14 畫面：頂點顏色由 `WaterOptics` 算（Fresnel 天空反射、Cox-Munk 閃爍、水體與水底依水深、水下 Snell 窗），不再用坡度明暗；
  `BODY_SHADE`、`SUN_OVER_SKY` 是有註明的顯示選擇。顏色在平行計算中完成，送頂點只讀陣列。
- 0.14 風浪隨時間：`SeaState` 在伺服器每 tick 把風浪的 `WaveRegion.scale` 朝目前風速的受風域限制高度鬆弛（CEM II-2-35 時間的三分之一），
  `ScalesPayload` 同步；`compiled()` 回傳目前比例的波場（`waves()` 仍是合成時的）。不超過合成時的高度；湧浪不變。
- 0.14 擋風：`Survey` 帶地面／樹冠高度，`WaveRegion.shelter` 依 `SeaWind.shelter`（Brandle 等 2002）；舊測量沒有高度，重新 survey 才生效。
- 0.14 靠岸浪頭：海岸上 CWM 二階項用 `CoastalWaves.kernels`（當地 k、coth），三個讀取路徑（`surfaceTerms`、`gridCoastal`、`at`）必須一致；
  Sharma-Dean 修正維持外海核（淺水會發散）。
- 0.14 划船：`Rowboat.Oars` 帶相位，`RowingBoats` 由原版槳角（`BoatAccessor.paddlePositions`）取得，驅動期半正弦推力。
- 0.15 水面 shader：顏色只在 `rendertype_aerocore_sea.fsh` 計算；`WaveSurfaceRenderer` 傳高度、網格坡度與打包屬性（格式見 `.vsh` 註解，
  Java 的 `vertex()` 與 shader 必須一起改），波譜與小波紋以 `Waves[96]` 原始 GL uniform 上傳（相位以 double 在原點算好再取模）。
  網格畫 λ ≥ 4 倍間距的分量，shader 畫其餘的波（截止波長由頂點傳入）；兩邊用同一份外海波數分界，不要只改一邊。
  `WaterOptics.ripples` 的小波紋只用於畫面，物理不感受；像素以下的波併入閃爍變異量，不要再加距離漸入規則或改回頂點著色。
  `debugView` 1～4 是開發用除錯畫面（`AutoShot` 的 `debug` 步驟）。
- 0.15 白沫與風浪：`Whitecaps.thresholdAtWind` 不再壓在 0.388 g 以下（短風域的海會整片起白沫）；`SeaFoam` 逐點用擋風後的當地風速，40 m 外由
  `SeaFoam.meanFoam` 畫平均覆蓋率；`CoastalWaves.growth` 在合成 Tp 的陡度 1/15 停止（`WindSea.peakPeriodS`），不要拿掉（頻率固定的波場只能放大振幅）。
- 0.16 全世界的風：`WorldWind` 是所有讀取者的風（海域外的物體、原版船、水花、雲、雨雪；沒有自己風的海域也用它）。
  參考風由伺服器決定（天氣 × 綜觀起伏，或 `/aerocore wind set` 存檔覆寫）並以 `WindPayload` 同步，客戶端只讀；兩端用同一高度圖（`MOTION_BLOCKING`，往下穿過樹葉）。
  不要為了算風載入區塊；屏障方塊當空氣、屋簷下與地底無風是規則。GameTest 預設無風（測試自己設 override，結束要還原）；
  GameTest 世界裡其他測試留下的結構也會擋風，斷言要比較相對量（擋風比例），不要假設四周空曠。垂直風（坡面上升氣流、熱氣流）只在 `vertical`／`at` 裡。
- 0.16 自然生成浪：`NaturalSeas` 只建不存檔的自然海域（`WaveRegion.natural()`），避開指令海域；波譜依風域、水深、風向分級快取以免換區跳動，
  不要改成每次換區重新合成。`SeaState` 對自然海域用全世界的風。GameTest 預設關閉（`WaveRegions.naturalSeas`）。
- 0.16 天空：顏色在 `aerocore_sky.fsh` 與 `SkyRenderer.colour/display` 兩處，公式必須一致（霧色與水面反射用 Java 端）；曝光依半球平均亮度。
  原版的日出扇形由 `SkyDomeMixin` 拿掉，不要再畫回來（會重複）。雲的位移與雨雪傾斜在 mixin 裡，雨的傾斜方向（上端往上風）已用截圖確認。
- 0.16 白沫：白沫值是覆蓋率，`foamLace` 依覆蓋率決定白色面積（門檻的三次式是對網紋場分位數的擬合，改網紋就要重新擬合）；船艏只蓋 浪峰高/半徑 的覆蓋率。
- 0.16 地面載具：`GroundContact` 的軟地面下陷用精確積分（不是 Bekker 的 (3 − n)/3）；方塊對應在 `GroundSurfaces`（規則）。
- 0.16 水面效能：網格間距不細於 `meshNeed`（振幅 2 cm 以上的分量），尾浪區塊的環境浪在粗網格算再內插（`Job.ambient`，shader 的截止波長跟著它），
  包絡近乎均勻的區塊走 `CompiledWaves.at` 快速路徑。改這些時用 `AutoShot` 的 `fps` 量各段時間；自然海域的浪比池塘重，要在那裡量。
  自動截圖會在使用者的桌面跳出遊戲視窗，使用者可能正在用電腦，跑之前先說明。
- 0.16.1 幾乎無風：U₁₀ 低於 `SeaWind.LIGHT_AIR`（0.1 m/s）時剖面、摩擦速度與 `SurfaceWind.overWater` 依 0.1 m/s 的值線性縮小；不要移除，
  海域外圈與屋簷下的風可小到 1e-8 m/s，0.16.0 在那裡算出 NaN 讓 `Rowboat.windLoad` 拋例外、伺服器當機（使用者回報）。
- 0.16.1 效能：海域清單在同一維度內改變時，客戶端不要再整片 `setBlocksDirty` 或清空 `WaveSurfaceRenderer`（用 `ClientWaveRegions.remeshChanges`
  與 `migrate`：只排入水面那層、隱藏狀態改變的 section，每 tick 32 個由近到遠；水面區塊與白沫移交給新海域）。`intern` 讓內容相同的海域沿用原物件，
  快取（`TileKey`、`NEEDS` 等）才不會每次比對整個海床陣列。水面區塊的水色只在第一次初始化時讀 4 格格點（原版色彩快取只有 256 個區塊）；
  新區塊每幀 16、到期更新每幀 8、白沫重建每 tick 4 是預算。`SkyRenderer.state` 依太陽角度與天氣快取。`WaveRadiation.Memory` 的核函數全域共用、
  陣列唯讀不可修改；`record` 觸發的重算半徑取 2% 格點，`setRadii` 用精確值。其他客戶端的船只在玩家 64 m 內算傾斜。
  卡頓要看最慢的一幀，不是平均 FPS：`AutoShot` 的 `fps` 與慢幀記錄（GC、JIT、水面各段、客戶端 tick；0.17 加伺服器 tick 平均與超過 50 ms 的數量）就是為此；
  更細的用 `jcmd <pid> JFR.start settings=profile`（遊戲的暫存目錄改過，`jcmd -l` 看不到，要加 `-J-Djava.io.tmpdir=%USERPROFILE%\.mcmod\tmp` 並直接給 PID）
  附加到開發客戶端（`jfr print` 輸出文字再分析，JSON 太大）。
- 0.19 噴射引擎（詳見 docs/JET.md）：
  - 性能只來自 `Turbojet` 的匹配（設計點 → 平衡 → 運轉線 → 準穩態）；元件只給參數與檢查，不要在零件裡另算推力。
    準穩態在運轉線油量下必須就是平衡解：改平衡、運轉線或準穩態任何一個，都用 `TurbojetTest.quasiSteadyBalances...`
    與兩具範例在靜止、100 m/s、11 km 確認運轉線油量的功率差在 1 % 內，全油門推力等於平衡解。
  - 渦輪是跨越自身壓比的噴嘴（不是恆阻塞）：低壓比的模型引擎在設計點就沒有阻塞，改回恆阻塞會讓它在 935 K 以下沒有平衡解。
  - 平衡的匹配式有退化根（轉子幾乎不轉、流量很大）：從大渦輪壓比往下掃描再二分的做法要保留。進氣損失直接在平衡式內由修正流量求
    （與 pt2 無關）；不要改回外圈迭代（會在兩根之間跳，運轉線列的 pt2 與流量不一致）。
  - 壓損（進氣道、燃燒室、尾管）隨修正流量平方縮放；固定比例的壓損會讓低轉速的低壓比引擎渦輪拿不到壓力，起不來。
  - 運轉線依飛行馬赫數各一條、依衝壓總壓比內插、T4 範圍換算到最冷標準大氣；只用靜止的一條時，噴嘴未阻塞的引擎在飛行中推力差 20 % 以上，
    高空寒冷時修正轉速超出表格。
  - 燃油控制、起動機（45 % 後永久切離）、點火、熄火、喘振（30 %、0.6 s）是規則，寫在 `JetControlDesign`／`JetEngineDesign` 的註解；
    起動機若在點火時就切離，模型引擎會在點火轉速附近反覆熄火。
  - 質量係數以 J85-GE-17A（181 kg）與 JetCat P80（1.32 kg）一起定；改任何元件的質量估計都要重看兩者。
  - 運轉線在背景建立（約 0.2 s），依設計 JSON 快取；`Compiled` 被所有載具共用，每台載具的狀態放 `PartStep.internal`。
  - 噴射艇 GameTest 會把船推過整個水池，放在自己的批次（`aerocore_jet_boat`）。
- 0.18 活塞引擎（詳見 docs/ENGINE.md）：
  - 元件各自檢查自己；能不能裝在一起（活塞對缸徑、連桿對曲柄銷與活塞銷、缸面間隙、氣門排列、噴油嘴種類）由 `PistonEngineDesign.check` 檢查。
    壓縮比不是參數，由燃燒室、床墊、缸面間隙與活塞頂算出；不要加一個壓縮比欄位繞過它。
  - 性能只來自 `EngineCycle`＋`Dyno`＋`Friction`；不要為了「好玩」直接改 MAP 的值。燃燒相關式（Bonatesta）只在一具引擎擬合，
    火焰行程與燃料縮放、爆震校準 `KNOCK_CALIBRATION = 1.45` 都是有註明的假設或單點校準，改動時要更新 ENGINE.md 的理由。
  - 循環的顯式步長在稀薄氣體會發散：孔口流量限制（每步不超過讓兩側壓力相等之量的一半）與歧管氣柱倒流限制都要保留；
    改步長或流量公式時，用 `EngineDesignTest` 與 12～20 % 歧管壓力的點確認不會出現 NaN 或負質量。
  - MAP 在背景建立（`Part.warmUp`，數秒，各轉速平行），依設計 JSON 快取；`Compiled` 被所有載具共用，不可放每台載具的狀態（用 `PartStep.internal`）。
  - 引擎船 GameTest 會跑很久（等 MAP），尾浪會影響同批次的鄰居，所以放在自己的批次（`aerocore_engine_boat`）。
- 0.17 零件化載具：
  - 零件受力必須和原本的積分器逐步一致：`FloatingBodyDynamics.sphereLoads`、`HullDynamics.loads`、`PlaningDynamics.loads` 是共用的同一份程式，
    不要在零件裡另寫一份。`VehicleTest` 以 1e-9 比對浮筒平台、測試船、滑行艇，改受力時要一起過。
  - 船體、控制面這類「以自身座標寫的模型」用 `PartStep.partState()`／`partProperties()`（同一剛體以零件軸表示），不要自己轉換座標。
  - 傳動系：轉子的外力扭矩（水對槳）與內力扭矩（馬達）要分開傳（`rotorTorque(..., external)`），載具才會承受正確的反扭矩；
    螺旋槳的 `AppliedForce` 不再帶偶力（反扭矩改由傳動系給）。
  - 垂直輻射記憶以載具座標的水平面量點距：船體零件只能水平安裝（只改 yaw），傾斜安裝會被當成另一種排列。
  - 船體的 Michell 表與螺旋槳四象限表延後到第一步（`Part.warmUp`），`PartVehicle` 在背景執行緒預建、建好前原地等待；
    客戶端只畫不算，不建這些表。不要改回在伺服器執行緒同步建表（第一次生成新船體會卡 1.9 s）。
  - 輻射記憶在第一步 `prime`（以浮在水上的半徑開始）；尾浪設計在背景建立；空氣取樣每 tick 每格快取。這些是生成時不卡的原因，不要拿掉。
  - 輻射核函數的餘弦遞迴走頻率、內圈跑時間點（可向量化，快 3.8 倍）；不要改回每個頻率對時間遞迴。
  - 球對球接觸（`ContactSolver.generatePair`）的保守略過與預先放置是多船相鄰時不卡的原因；`Vec3d.length` 不要改回 `hypot`（慢數倍，在每個接觸迴圈裡）。
  - 以實體為鍵的靜態 Map 必須伺服器／客戶端分開：`Entity.equals` 比較網路 ID，單人遊戲兩端的同一實體 ID 相同。
    `RowingBoats` 0.11～0.16 共用一份狀態，0.17 開發時重現客戶端 `memory is null` 崩潰後才分開。
  - 藍圖 JSON 的數字以最短往返形式寫出；`format` 比目前新的拒絕讀。新增參數要有預設值，舊藍圖才讀得回來。
  - 範例藍圖只示範格式與物理，不是要出的載具；AeroCore 不加可駕駛載具的玩法（AGENTS 開頭）。
- 0.15 `AutoShot`：只在非正式環境、遊戲目錄有 `aerocore-autoshot.txt` 時啟動（讀後即刪），只開資料夾名稱以 AeroCore 開頭的世界，最後正常存檔離開。
  0.16 加 `key`（按住移動鍵，例如划船）、`camera`、`look`、`clouds`（只在這次執行打開雲，結束還原使用者設定）；1.21.1 的船實體 ID 是 `minecraft:boat`。
  `AeroCore Render QA` 是 2026-10-10 在原開發機從使用者的「新的世界」複製的 QA 世界（複製時遊戲已關閉，只讀原世界，`level.dat` 名稱已改），
  可隨意修改；不要對「新的世界」使用 AutoShot。截圖在 `client/screenshots/aerocore-*.png`。
- 0.14.1：`CompiledWaves` 的水底鏡像項 exp(-2kh)/exp(kz) 一律經 `image()`（衰減下溢時取 0），不要改回直接相除：
  短風域的最短分量只有約 2 cm，水深 2.7 m 以下 exp(kz) 低於倍精度，0/0 會讓 `Vec3d` 拋例外、玩家 tick 崩潰（使用者的池塘）。
  這些分量已在表面張力範圍（波長 4 cm 以下頻率差 9% 以上），模型仍用重力波色散；振幅不到 1 mm，畫面低通已濾掉。
- 0.13 效能：白沫只算 `WaveSurfaceRenderer.surfaceMask` 為海面的格子、1 m 網格、每 tick 半數區塊；畫面各區塊在 common pool 平行計算、渲染執行緒送頂點
  （計算中只讀執行緒安全的資料）；`gridCorrection` 用四組部分和。不要把白沫改回對整片區塊（含陸地）逐 0.5 m 計算。
- 0.12 風：`WaveRegion.Wind`（10 m 風速、吹向，與波浪方向同一慣例）只在海域內、外圈漸弱；剖面 `SeaWind.speedAt`（COARE 3.0 對數律）。
  風驅動表層流 0.53 u*（不另加 Stokes 漂流：解析的波浪已給浮體）經 `WaveRegion.windDrift`／`WorldWaterSampler.drifting`／`SeaSurface.waterVelocity` 加入；
  物體的空氣負載經 `PlatformEnvironment.setSea`／`airAt` 取海域風加上各物體的測試風；白冠用 `Whitecaps.thresholdAtWind`。
- 0.10 滑行：`PlaningDynamics` 用 Zarnick 2D+t（Wagner 堆水、C_Dc = cosβ、轉動座標項），靜水壓用 Archimedes，通氣後垂直於龍骨；
  0.12 起靜水壓乘上 Garme 艉板修正 tanh(2.5x_T/(0.34 B C_V))、隨通氣淡入。不要把它也乘在動力項上（Fridsma B 組態會大幅縱搖不穩），
  Zarnick 的浮力減半也不要加回。低速垂直輻射由呼叫者提供、隨通氣淡出。
  驗證基準：Fridsma 表 1（航行俯仰差 0.35° 內、阻力偏高 5～13%），LCG 70% 的縱搖不穩起始在 V/√L 5.5～6.2 之間（水槽 5.85）。
- 0.10 係數：浮筒阻力用 `FloatingBodyDynamics.STANDARD_DRAG`（Clift 標準曲線）；物體為鋼；0.11 起方塊由 `BlockMaterials`（標籤＋音效類型）
  分成冰、木、沙、礫石、泥土、泥巴、雪、羊毛、玻璃、金屬、岩石，材質放在 `WorldBox` 上；Thornton 的 p_y 用硬度而非降伏強度；實體碰撞箱仍用物體預設。沒有資料的值（虛構生物質量、原版水流速度、顯示用水滴數量）要在註解寫明是規則或顯示預算，不可寫成量測值。
- 熱力學 API 使用 K，指令使用 °C。空氣支援 180～500 K、1～300 kPa；純水支援 273.15～353.15 K、80～300 kPa，超界拒絕。
- 熱交換是固定熱容量的均溫模型；環境為固定溫度熱庫。阻力耗散只有回復溫度模型算出的部分（且不超過耗散功率）進入本體，
  其餘留在流體；接觸摩擦熱（Σ−f_t·u_t）依熱滲透率比分給本體與地面（示範：類碳鋼對類花崗岩），法向撞擊損失與軸功率不生熱；
  未解空間溫度場或相變。測試球沒有熱模型。
- 新物理公式須註明來源、範圍、近似與單位；以解析解、獨立參考點、守恆／耗散性或步長收斂驗證，避免只測實作自己的結果。

## 修改與驗證流程

1. 先讀相關程式及文件，確認既有功能。不要把討論中的下一步當成已完成，也不要擅自擴展成載具玩法。
2. 保持修改範圍集中；新增指令訊息時同時補 `en_us.json` 與 `zh_tw.json`。避免在現有 API 中無聲改變單位或座標語意。
3. 核心物理變更跑相應 JUnit；Minecraft 取樣、碰撞或實體行為變更跑 GameTest；畫面與互動變更用 QA 世界實測。
4. 交付程式變更前執行 `build`，確認 JAR；已通過的檢查只在新修改或未解決疑慮需要時重跑。純文件修改不必啟動遊戲或重跑整套測試。
5. 更新相關文件及必要版本資訊。最後報告實際完成、實際測試與未驗證項目，不把歷史測試結果當成本次新執行結果。

已驗證基準（測試數量會隨功能改變，以實際報告為準）：

| 版本 | 日期／電腦 | JUnit（physics＋CFD） | GameTest | 客戶端 |
| --- | --- | --- | --- | --- |
| 0.5.0 | 2026-10-07，原開發機 | 75＋5 | 17 | 已驗證粒子／風／溫度指令與世界存檔、重新載入 |
| 0.6.0 | 2026-10-07，DESKTOP-KLLC9QQ | 103＋5 | 21 | 未實測 |
| 0.7.0 | 2026-10-08，DESKTOP-KLLC9QQ | 111＋5 | 23 | 未實測 |
| 0.8.0 | 2026-10-09，DESKTOP-KLLC9QQ | 119＋5 | 24 | 修正暫存目錄後可建立、載入、存檔世界，記錄無錯誤；功能畫面未實測 |
| 0.9.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 147＋5 | 29 | 見下方待實測清單 |
| 0.10.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 168＋5 | 36 | 只確認開發客戶端能啟動到標題畫面、客戶端 mixin 套用無錯誤；功能畫面未實測 |
| 0.11.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 189＋5 | 38 | 只確認開發客戶端能啟動到標題畫面、客戶端 mixin（含船的渲染）無錯誤；功能畫面未實測 |
| 0.12.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 202＋5 | 40 | 只確認開發客戶端能啟動到標題畫面、新的客戶端 mixin（乘客傾斜）無錯誤；功能畫面未實測 |
| 0.13.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 206＋5 | 42 | 只確認開發客戶端能啟動到標題畫面、無錯誤；功能畫面未實測 |
| 0.14.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 210＋5 | 43 | 只確認開發客戶端能啟動到標題畫面、無錯誤；水面光學著色以離線預覽圖檢查，遊戲中未實測 |
| 0.14.1 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 211＋5 | 43 | 未重新啟動客戶端；修正使用者實玩 0.14.0 的崩潰（新測試在未修正時失敗）與近處亮盤，遊戲中未實測 |
| 0.15.0 | 2026-10-10，原開發機 DESKTOP-AH3A4MO | 216＋5 | 43 | 以 `AutoShot` 在 `AeroCore Render QA` 截圖：淺水風浪區的早晨／中午／傍晚／夜晚／水下／平視／俯視外觀，FPS 83～117（1280×720，RTX 5070） |
| 0.16.0 | 2026-10-11，原開發機 DESKTOP-AH3A4MO | 234＋5 | 47 | `AutoShot` 截圖：划船的泡沫網紋與停船後消散、自然海域晴天與雷雨、天空正午／日出／日落／雨／雷雨、雲兩張相隔 5 秒的位移、雨的傾斜 |
| 0.18.0 | 2026-10-11，原開發機 DESKTOP-AH3A4MO | 280＋5 | 51 | 未啟動客戶端；引擎船以 GameTest 確認起動與推進 |
| 0.19.0 | 2026-10-11，原開發機 DESKTOP-AH3A4MO | 293＋5 | 52 | 未啟動客戶端；噴射艇以 GameTest 確認點火與推進；引擎船 GameTest 改成撞牆前判定（原本靠反彈才過） |
| 0.17.0 | 2026-10-11，原開發機 DESKTOP-AH3A4MO | 257＋5 | 50 | `AutoShot` 在 `AeroCore Render QA` 生成三個範例載具：浮在水面、給油門後前進並留下尾浪；四個載具靜置時伺服器 tick 平均 31～34 ms（無載具 17 ms）；手動駕駛未實測 |
| 0.16.1 | 2026-10-11，原開發機 DESKTOP-AH3A4MO | 237＋5 | 47 | `AutoShot` 沿 320 m 航線移動量測：自然海域換區時最慢一幀 18～25 ms（修正前 66～118 ms），平均 105～119 FPS；換區邊緣的過渡畫面未截圖確認 |

**客戶端仍待實測**（0.6～0.9）：螺旋槳模型與轉動、洋紅接觸／橙推力／淡紫慣性／墨綠輻射／藍升力箭頭、淺藍波面粒子、
測試球反彈與滾動及其旋轉顯示、平台在靜水中幾秒內停止起伏、新的中英文指令訊息；
0.9：客戶端波面（mixin 是否套用、波形、漸弱接縫、效能、Fabulous 模式）、測試船網格／舵／槳的外觀、
玩家站在平台與測試船甲板上的碰撞、被載運與伺服器回拉，以及平台壓斜；
0.10：不規則海面與分級細節接縫、整片海域的效能、尾浪與臨時靜水區的出現／移動（區塊重建）、水花、鏡頭水下霧切換、
原版船與生物隨浪起伏、玩家游泳與溺水判定、滑行艇外觀與操作、多人連線；
0.11：二階波形外觀、非定常尾浪（轉彎、停船）、白沫與白水、划船手感與轉向、船的俯仰橫搖畫面與乘客位置、各方塊材質的手感、尾浪背景執行緒的負載；
0.12：浪頭外觀、海岸的折射／遮蔽／碎浪帶白沫與海岸區塊的渲染效能、海岸計算在客戶端完成前後的切換、風中的漂流與水花、
乘客隨船傾斜與第一人稱滾轉（含其他玩家看到的乘客）、單槳轉向手感、滑行艇船艏仰角的外觀、`survey`／`wind`／`windsea` 的中英文訊息；
0.13：使用者在 0.12 實玩回報浪不見與卡頓（已修正，見 docs/SEA.md〈效能〉〈風浪〉）；修正後的實際 FPS、陣風與天氣變化的手感、`wind weather`／`survey clear` 訊息；
0.14：使用者回報「浪渲染效果不好（只有白色一層）」（改光學著色，離線預覽確認浪形可見）；遊戲中的實際外觀（顏色、閃爍、水底透明度、夜晚、水下）、
風浪隨雷雨起落、擋風、靠岸浪頭、逐槳前衝的手感；
0.14.1：使用者實玩 0.14.0 回報近處亮盤與池塘崩潰（已修正）；閃爍漸入後的近處外觀、使用者世界的池塘重新進入是否正常；
0.15：已用自動截圖看過淺水風浪區（見上表）；仍未看過大浪外海、碎浪帶、尾浪與水花在新 shader 下、下雨與雷雨、其他生態域水色、Fabulous 模式、低階顯示卡 FPS、多人連線；
0.19：噴射艇在遊戲中的表現（`/aerocore vehicle spawn jetboat`：起動、怠速推力、油門反應、熄火，沒有聲音與尾焰）、飛行器上的噴射引擎（還沒有機翼零件）；
0.18：引擎船在遊戲中的表現（`/aerocore vehicle spawn engineboat`：起動時間、怠速、加速、聲音沒有）、引擎 MAP 建立期間的等待、冷車到熱車；
0.17：坐上零件載具用預設按鍵駕駛、站在甲板上、受力箭頭（`/aerocore vehicle debug`）、多人連線的控制與同步、範例以外的藍圖（自己寫 JSON）、
其他客戶端看到的螺旋槳與舵動畫；
0.16.1：使用者世界與使用者電腦上的實際手感（量測在原開發機的 QA 世界、旁觀者模式瞬移）；換區邊緣 0.1～0.2 s 的過渡畫面；
0.16：見上表；仍未看過使用者自己划船時的手感與白沫、熱氣流與坡面上升氣流（沒有飛行器）、雪地的雪傾斜、Fabulous 模式下的天空、多人連線的風同步。
JUnit 報告在各模組 `build/reports/tests/test/` 與 `build/test-results/test/`；網路磁碟偶爾寫入 HTML 報告失敗，以 XML 結果為準後重跑即可。

JAR 在 `neoforge/build/libs/aerocore-neoforge-1.21.1-<版本>.jar`，已包含 physics API。測試安裝只保留一個 AeroCore 版本，不把 sources JAR 放入 mods。
CFD CSV／圖在 `cfd/build/experiments/water/` 或 `air/`；`scripts/plot-cfd.py` 只繪製既有數值結果。

## 延伸文件

- [README](README.md)：啟動、IDE、產物與功能入口。
- [API](docs/API.md)：接入載具的資料型別與呼叫方式。
- [物理模型](docs/PHYSICS.md)、[六自由度平台](docs/PLATFORM.md)、[測試船](docs/HULL.md)、[測試球](docs/PROBE.md)。
- [海況與尾浪](docs/SEA.md)：不規則海、二階水面、共用水面、非定常尾浪、白沫、水花；[滑行艇](docs/PLANING.md)：2D+t 模型與 Fridsma 驗證。
- [原版船](docs/ROWBOAT.md)：划船物理、參數來源與驗證。
- [熱力學與視覺化](docs/THERMAL_DEBUG.md)：指令、比例尺、熱模型及實測圖。
- [本機執行資料](docs/LOCAL_RUNTIME.md)：世界遷移、備份與暫存目錄造成的建立世界崩潰。
- [CFD](docs/CFD.md)：獨立流道求解與離線圖，不等同於遊戲環境流場。
- [大氣](docs/ATMOSPHERE.md)：全世界的風、天空、雲與雨雪、飛行用大氣；[地面載具](docs/GROUND.md)：輪胎、滾動阻力、軟地面。
- [零件化載具](docs/VEHICLE_PARTS.md)：零件藍圖、組裝與連接、通用載具物理、遊戲端 `PartVehicle`、驗證。
- [活塞引擎](docs/ENGINE.md)：16 個元件、循環模擬、標定與 MAP、執行時、驗證；[噴射引擎](docs/JET.md)：6 個元件、循環匹配、運轉線、暫態、驗證。
