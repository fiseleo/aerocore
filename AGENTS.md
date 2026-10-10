# AeroCore 開發代理指南

本文件適用於整個專案。開始修改前先讀本文件，再讀相關模組與文件；功能狀態以實際原始碼、設定和本次測試為準。
與使用者溝通以繁體中文為主，說明完成的功能、驗證結果及仍存在的模型限制。

## 專案目標與現況

- 為 Minecraft Java 載具 Mod 建立通用的空氣／水動力前置；載具形式尚未決定。
- 先維持可驗證的物理核心，再逐步接入載具玩法。不要把近似模型描述成工程級 CFD。
- 截至 2026-10-10，版本為 **0.10.0**：具備六自由度剛體、四角浮力、可調配重、受力／環境流速粒子顯示、溫壓流體性質與基礎熱交換；
  0.6 加入球形元件接觸反力（摩擦／反彈）、指定的線性 Airy 波浪、BEMT 螺旋槳推力（平台艉部示範槳）與阻力摩擦生熱；
  0.7 加入接觸摩擦生熱、波壓深度衰減（垂直 Froude–Krylov）、水平 Froude–Krylov 力與附加質量，測試球改用六自由度剛體與 `ContactSolver`；
  0.8 加入浮筒垂直方向的波浪輻射阻尼（Hulme 半球資料、Cummins 記憶力）與浮筒間點吸收體干擾；
  0.9（2026-10-10）加入世界波浪區域與客戶端波面、與輻射一致的浮筒垂直繞射表、Wageningen B 系列螺旋槳、
  細長船體（阻力與操縱）與指令操作的測試船／舵、物體之間的雙向接觸（`BodyWorld`），以及原版實體站上甲板（mixin）；
  0.10（2026-10-10）加入 JONSWAP 不規則海況、所有讀取者共用的移動水面（原版實體／原版船／鏡頭霧 mixin）、Kelvin 尾浪與水花、
  Zarnick 2D+t 滑行艇（對 Fridsma 1969 驗證）與指令操作的滑行測試艇，並把示範係數換成文獻值（見 docs/PHYSICS.md〈0.10 係數來源〉）。
- 獨立二維 CFD 實驗與離線流場圖仍保留。遊戲內粒子顯示目前是環境取樣；尾浪是給定的線性 Kelvin 波型，不是流場求解。
- 尚無可駕駛載具、方塊組裝、引擎模型、旋翼、三維 CFD、波浪傳播求解（碎浪、反射）、相變或完整熱流耦合。
- 載具形式仍未決定；船體模型刻意做成水面船與潛航體都能用的細長體，不要擅自改成特定載具玩法。

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

- `Z:\MCMod` 與 `\\100.96.25.71\share\MCMod` 是同一份專案的別名，不是兩個 checkout。
- Windows Shell 操作優先使用 `Z:\MCMod`，避免 CMD 的 UNC 工作目錄限制。
- 原始碼及 JAR 在專案內；遊戲世界、設定、記錄與截圖必須留在本機。
- 本機執行根目錄：`%USERPROFILE%\.mcmod\runs\aerocore`，每台電腦各自一份：原開發機（`DESKTOP-AH3A4MO`）為 `C:\Users\ZHU YU TA\.mcmod\runs\aerocore`，`DESKTOP-KLLC9QQ` 為 `C:\Users\CHU YU TA\.mcmod\runs\aerocore`。
- 專案不是 git 倉庫；大改前把原始碼備份到 `%USERPROFILE%\.mcmod\backups\`（0.9 開工前的備份為 `aerocore-src-0.8.0-20261010-002942`）。
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
| neoforge 的 `mixin/` | `aerocore.mixins.json`：雙端 `EntityCollideMixin`（實體對物體碰撞）、`EntityWaterMixin`（原版水高度與視線讀海面）、`BoatWaterMixin`（原版船）、`LivingEntityAccessor`；客戶端 `LiquidBlockRendererMixin`（隱藏海面頂面）、`CameraFogMixin`（水下霧）；只能放 mixin 類別 |
| `scripts/` | JDK／IDE／本機資料與離線 CFD 繪圖工具 |
| `docs/` | API、模型假設、操作和驗證說明 |

主要類別：`AeroSolver`、`RigidBodyDynamics`、`FloatingBodyDynamics`、`ContactSolver`、`LinearWave`／`WaveField`、
`PropellerSolver`／`BladeElementPropeller`／`WageningenB`、`WaveRadiation`、`SlenderHull`／`HullSection`／`HullModel`／`HullDynamics`／`HullResistance`／`HullManoeuvring`、
`Rudder`、`Thermodynamics`、`HeatTransfer`；0.10：`SeaSpectrum`／`CompiledWaves`、`KelvinWake`、`BowSpray`、`PlaningHull`／`PlaningDynamics`／`TestPlaningBoat`、
`SphereDrag`、`BSeriesFourQuadrant`、`ContactMaterial`（Thornton）；
遊戲端入口為 `FloatingPlatform`、`FloatingHull`、`PhysicsProbe`（都實作 `CoupledBody`、`SolidBody`）、`BodyWorld`（每 tick 聯合推進）、
`BodyCollisions`（原版實體）、`WaveRegions`／`WaveNetwork`／`WaveCommands`、`PlatformCommands`、`HullCommands`、`PlatformDebug`、`WorldWaterSampler`；
0.10：`SeaSurface`（共用水面）、`VanillaWater`（原版實體）、`Wakes`（尾浪與靜水區）、`EntityMasses`、`PlaningBoat`／`PlaningCommands`，客戶端 `WaveSurfaceRenderer`、`SprayRenderer`。
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
  尾浪是隨速度分格的定常 Kelvin 波型（直線等速近似），只以水面高低作用（靜水壓），物體排除自己的尾浪；原版實體的波浪漂移用其原版水中阻尼 f。
- 0.10 滑行：`PlaningDynamics` 用 Zarnick 2D+t（Wagner 堆水、C_Dc = cosβ、轉動座標項），靜水壓用完整 Archimedes，通氣後垂直於龍骨；
  Zarnick 的浮力減半與 Garme 艉板修正都試過、會讓 Fridsma B 組態縱搖不穩，不要再加回。低速垂直輻射由呼叫者提供、隨通氣淡出。
  驗證基準：Fridsma 表 1（阻力 −15%～0%、俯仰低 0.5～1.1°、重載低 2.4°），縱搖不穩起始比水槽晚。
- 0.10 係數：浮筒阻力用 `FloatingBodyDynamics.STANDARD_DRAG`（Clift 標準曲線）；物體為鋼、方塊當岩石（`ContactMaterial.STEEL_ON_ROCK`，Thornton 恢復係數）；
  方塊材質尚未分類。沒有資料的值（虛構生物質量、原版水流速度、顯示用水滴數量）要在註解寫明是規則或顯示預算，不可寫成量測值。
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

**客戶端仍待實測**（0.6～0.9）：螺旋槳模型與轉動、洋紅接觸／橙推力／淡紫慣性／墨綠輻射／藍升力箭頭、淺藍波面粒子、
測試球反彈與滾動及其旋轉顯示、平台在靜水中幾秒內停止起伏、新的中英文指令訊息；
0.9：客戶端波面（mixin 是否套用、波形、漸弱接縫、效能、Fabulous 模式）、測試船網格／舵／槳的外觀、
玩家站在平台與測試船甲板上的碰撞、被載運與伺服器回拉，以及平台壓斜；
0.10：不規則海面與分級細節接縫、整片海域的效能、尾浪與臨時靜水區的出現／移動（區塊重建）、水花、鏡頭水下霧切換、
原版船與生物隨浪起伏、玩家游泳與溺水判定、滑行艇外觀與操作、多人連線。
JUnit 報告在各模組 `build/reports/tests/test/` 與 `build/test-results/test/`；網路磁碟偶爾寫入 HTML 報告失敗，以 XML 結果為準後重跑即可。

JAR 在 `neoforge/build/libs/aerocore-neoforge-1.21.1-<版本>.jar`，已包含 physics API。測試安裝只保留一個 AeroCore 版本，不把 sources JAR 放入 mods。
CFD CSV／圖在 `cfd/build/experiments/water/` 或 `air/`；`scripts/plot-cfd.py` 只繪製既有數值結果。

## 延伸文件

- [README](README.md)：啟動、IDE、產物與功能入口。
- [API](docs/API.md)：接入載具的資料型別與呼叫方式。
- [物理模型](docs/PHYSICS.md)、[六自由度平台](docs/PLATFORM.md)、[測試船](docs/HULL.md)、[測試球](docs/PROBE.md)。
- [海況與尾浪](docs/SEA.md)：不規則海、共用水面、尾浪、水花；[滑行艇](docs/PLANING.md)：2D+t 模型與 Fridsma 驗證。
- [熱力學與視覺化](docs/THERMAL_DEBUG.md)：指令、比例尺、熱模型及實測圖。
- [本機執行資料](docs/LOCAL_RUNTIME.md)：世界遷移、備份與暫存目錄造成的建立世界崩潰。
- [CFD](docs/CFD.md)：獨立流道求解與離線圖，不等同於遊戲環境流場。
