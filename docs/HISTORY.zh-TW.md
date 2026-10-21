# AeroCore 開發紀錄（舊 README，0.4～0.40）

為後續 Minecraft 載具 Mod 準備的通用前置。暫定名稱為 **AeroCore**，Mod ID 為 `aerocore`。
**0.22.1** 提供空氣／水的即時受力 API、六自由度剛體、四浮筒平台、受力／流速粒子箭頭、基礎熱交換，以及獨立二維 CFD 實驗模組；
0.6 新增接觸反力（摩擦／反彈）、線性波浪、葉素動量螺旋槳推力與阻力摩擦生熱；
0.7 新增接觸摩擦生熱、波壓隨深度衰減與附加質量，測試球也改用六自由度剛體與接觸求解器（會反彈與滾動）；
0.8 新增浮筒的波浪輻射阻尼與浮筒間干擾；
0.9 新增世界波浪區域與畫面上起伏的水面、與輻射一致的浮筒繞射力、Wageningen B 系列螺旋槳、
細長船體的阻力與操縱模型（含指令操作的 6 m 測試船與舵）、物體之間的雙向碰撞，以及原版實體站上甲板並被載著走；
0.10 新增 JONSWAP 不規則海況、原版實體／原版船／鏡頭霧與物理共用同一個移動水面、船隻的 Kelvin 尾浪與水花、
依 Fridsma 拖曳水槽資料驗證的 2D+t 滑行艇，並把示範係數換成有出處的資料；
0.11 把水面改成二階（尖峰平谷）、尾浪改成依實際航跡積分的非定常解、海面會碎浪起白沫，
原版船改成真實的划船物理（質量、浮力、人力功率、隨浪起伏與傾斜），方塊依材質給不同的摩擦與反彈，滑行艇加上鬚狀水花阻力；
0.12 把浪頭改成完整的二階理論、讓波浪依地形折射／淺化／受遮蔽並在淺處碎浪、加入海域的風（風浪、白冠、表層流、船受風、水花受風），
原版船的乘客跟著船身傾斜、單槳轉向受划槳速度限制，滑行艇加入艉板修正使船艏仰角與水槽一致；
0.13 加入陣風（Kaimal 紊流頻譜）與隨 Minecraft 天氣變化的風，由風生成的海改成依每個位置自己的風域成長，並大幅減輕水面與白沫的計算負擔；
0.14 把水面改成光學著色（Fresnel 反射、陽光閃爍、水深決定的透明度、水下 Snell 窗），風浪隨風隨時間起落，地形與樹林擋風，靠岸浪頭變尖，划船逐槳前衝；
0.15 改用自訂 shader 逐像素畫出網格以下的波與風的小波紋、岸邊倒影與受波浪遮蔽的陽光閃爍，修正過量的白沫，水面的 FPS 約提升一倍以上；
0.16 加入全世界的風（天氣、高度、地形、樹林、山丘、陣風）、自然生成浪、物理天空與隨風移動的雲與雨雪、飛行用大氣（Dryden 亂流、地面效應、熱氣流、坡面上升氣流）、
地面載具物理（Magic Formula 輪胎、Bekker–Wong 軟地面），船的白沫改為泡沫網紋；
0.16.1 修正微風時的當機，並消除自然海域隨玩家移動時的卡頓；
0.18 加入玩家自己設計的活塞引擎：16 個元件各自是藍圖（缸體、汽缸蓋、活塞、連桿、曲軸、氣門、凸輪軸、火星塞、噴油嘴、配氣、供油、點火、潤滑、冷卻、進氣、排氣），
壓縮比、進排氣、爆震、摩擦與極限由零件決定；性能來自逐曲軸角的循環模擬（見 [docs/ENGINE.md](docs/ENGINE.md)）。
0.19 加入玩家自己設計的噴射引擎：進氣道、壓縮機、燃燒室、渦輪、噴嘴／尾管、燃油控制各自是藍圖，組成單軸渦輪噴射引擎；
推力、耗油、起動與加速、喘振與熄火來自變比熱的循環匹配（以 Seitzman 例題、J79／J85 驗證，見 [docs/JET.md](docs/JET.md)）。
0.20 依使用者決定把 AeroCore 定位成「計算 API」：所有零件設計（含引擎元件）與範例搬到載具 Mod **hdfv**，
AeroCore 留下物理、從設計抽出的計算、零件的執行時受力、通用載具框架與海浪／尾浪的渲染（見 [零件化載具](docs/VEHICLE_PARTS.md)）。
0.21 加入雙轉子燃氣渦輪的計算：渦扇（分開或混合排氣）與自由渦輪（渦軸；加減速齒輪箱就是渦槳），
以 CFM56-7B、FJ44、PT6A-27、T700 級引擎驗證，hdfv 0.3 用它們組出玩家設計的引擎（見 [docs/TURBOFAN.md](docs/TURBOFAN.md)）。
0.22 加入「形狀 → 性能」的翼型、機翼、空氣螺旋槳與直升機旋翼：翼型由面板法加積分邊界層算出極曲線（含失速、可壓縮性、襟翼），
機翼用數值升力線（後掠、上反、扭轉、副翼、地面效應）與翼盒結構，螺旋槳與旋翼用葉素理論（可變距、揮舞、動量入流、渦環、自轉），
傳動加單向離合器與陀螺力矩；以 Abbott & von Doenhoff、Helmbold／DATCOM、NACA Report 640、UH-60 級懸停驗證（見 [docs/WING.md](docs/WING.md)）。
0.22.1 讓一台載具的物理錯誤只停住那台載具（記錄錯誤、存檔保留原因，`/aerocore vehicle info` 顯示、`resume` 重新開始），伺服器與其他載具照常運作。
0.17 加入零件化載具的前置：零件藍圖（浮筒、排水船體、滑行船體、螺旋槳、控制面、方塊體、電動馬達、壓艙、座位）、組裝與動力軸、
通用載具物理與遊戲中的通用載具實體，給獨立的載具 Mod 使用（見 [零件化載具](docs/VEHICLE_PARTS.md)）。
0.29 加入零件繪製的通用工具（給 hdfv 更逼真的引擎與機器）：網格的公尺 UV 與材質鍵、Blender 細節零件的 OBJ 讀寫、
背景烘焙的環境光遮蔽與看不見的面、可無縫鋪排的材質貼圖、零件著色器（高光與天空反射）、尾焰的形狀與顏色（見 [docs/RENDERING.md](docs/RENDERING.md)）。
目前沒有可駕駛載具；CFD 是已通過基礎數值驗證的研究原型，還不是完整三維工程求解器。

## 開發基底

| 項目 | 版本 |
| --- | --- |
| Minecraft Java Edition | 1.21.1 |
| NeoForge | 21.1.252 |
| Java | 21，64 位元 |
| ModDevGradle | 2.0.148 |
| Gradle Wrapper | 9.2.1 |

選擇 1.21.1 是以成熟的 NeoForge 生態作為起點，而不是宣稱它是全平台下載量第一。
[Create 的發行頁](https://modrinth.com/mod/create/versions?g=1.21.1&l=neoforge)仍列出這個遊戲版本；
[NeoForge 的 2026 年 26.1 公告](https://neoforged.net/news/26.1release/)也將 1.21.1 作為前一個穩定模組生態的比較基準，
但公告對下一個主流版本的預測屬作者判斷。
本專案沒有依賴 Create；未來可另外寫相容層。

## 快速開始

在專案根目錄開啟 PowerShell：

```powershell
# 首次準備專案用 JDK；下載到使用者的 .mcmod\java
.\scripts\setup-java.ps1

# 編譯三個模組、執行物理與 CFD 測試、產生 JAR
.\dev.ps1 build

# 只執行物理測試
.\dev.ps1 :physics:test

# 啟動開發用 Minecraft
.\dev.ps1 :neoforge:runClient

# 獨立 CFD 水流／氣流實驗，不需要開遊戲
.\dev.ps1 :cfd:run
.\dev.ps1 :cfd:run --args="air"
```

## IntelliJ IDEA 開發介面

安裝 IntelliJ IDEA 後，在專案目錄的 CMD 或 PowerShell 執行 `./open-ide.cmd` 開啟專案。
Windows PowerShell 可能拒絕網路磁碟上的未簽署 `.ps1`，因此優先使用不依賴 PowerShell 的 CMD 入口。
這個入口不會調整 Windows 的腳本執行原則，也不會自動安裝 IDE。
此啟動入口會傳入專案的 JDK 21 和本機暫存路徑，IDE 本身仍使用隨附的 JetBrains Runtime。
第一次啟動的條款、資料分享與帳號選項請依自己的需求選擇。
核心 Java／Gradle 功能可使用免費方案，無須購買 Ultimate。

待 Gradle 匯入完成後，可從右上角的執行設定選擇：

- **Build AeroCore**：編譯 Mod 與執行測試。
- **Test Physics**：執行即時物理與 CFD 數值測試。
- **Minecraft Client**：啟動開發用遊戲。
- **Test Minecraft Physics**：在獨立測試世界驗證落地、碰撞、浮沉與水流，執行完自動關閉。
- **CFD Water Experiment**／**CFD Air Experiment**：執行二維水流／氣流實驗。

若 IDE 尚未出現 `cfd` 模組，按 Gradle 面板的 **Reload All Gradle Projects**。
CFD 結果位於 `cfd/build/experiments/water/` 或 `air/`，包含流場 CSV、歷史紀錄與報告。

遊戲世界、設定與執行記錄改放在本機 `%USERPROFILE%\.mcmod\runs\aerocore\client`；
伺服器與 GameTest 分別使用同層的 `server`、`gametest`。Z: 和 UNC 路徑會共用同一個本機目錄。
既有兩個開發世界已完成複製與 SHA-256 比對，原本 `neoforge/run` 保留為備份。
在 IDEA 按 **Reload All Gradle Projects** 後啟動 Minecraft Client。路徑與遷移操作見 [本機執行資料](docs/LOCAL_RUNTIME.md)。

主要程式入口為 `physics/src/main/java/dev/mcmod/aerocore/api/AeroSolver.java`，
NeoForge 入口為 `neoforge/src/main/java/dev/mcmod/aerocore/neoforge/AeroCore.java`。
若從其他捷徑啟動 IDE 而找不到 Gradle JVM，請在設定的 Gradle 頁面指定
`%USERPROFILE%\.mcmod\java\jdk-21.0.12.1+1`。

JDK 存在使用者目錄 `.mcmod/java`，避免封裝應用程式將 AppData 寫入重新導向到隔離目錄。
啟動腳本會依序檢查有效的 `JAVA_HOME`、專案用 JDK、`.jdks` 以及常見的系統 JDK 21 安裝位置。

如果 IDE 安裝在自訂位置，可以指定實際執行檔：

```bat
open-ide.cmd "D:\Apps\IntelliJ IDEA\bin\idea64.exe"
```

用 `open-ide.cmd --check` 可只檢查 JDK 與標準 IDE 安裝位置，不開啟視窗。

參考：[IntelliJ 官方安裝說明](https://www.jetbrains.com/help/idea/installation-guide.html)、
[Gradle JVM 設定](https://www.jetbrains.com/help/idea/gradle-jvm-selection.html)。

## 建置與產物

腳本使用 Microsoft OpenJDK 21 ZIP，驗證 SHA-256 後解壓縮。
`dev.ps1` 暫時設定該程序的 `JAVA_HOME`，並使用使用者目錄的 `.mcmod/tmp` 作為本機暫存空間，
避免 Windows 封裝應用程式的暫存路徑影響 Java 通訊；結束後恢復環境變數原值。
如果 PowerShell 阻擋本機腳本，可以對單次執行使用
`powershell -ExecutionPolicy Bypass -File .\dev.ps1 build`，無須更改整台電腦的執行原則。

其他作業系統或已有 JDK 21 的環境可直接使用 `./gradlew build`（Windows 為 `gradlew.bat build`）。
IDE 可開啟根目錄的 `settings.gradle`，並將 Gradle JVM 設為 JDK 21。
初次建置會下載 Gradle、Minecraft 與 NeoForge，相依檔案需要一些時間。
Minecraft 解編譯暫存放在 Gradle 使用者目錄的 `caches/aerocore-work`，以改善網路磁碟上的建置速度。
建置輸出（2026-10-18 起）也在本機：`%USERPROFILE%\.mcmod\build\aerocore-<路徑雜湊>\<模組>`，
`.\dev.ps1 showBuildDirectories -q` 印出位置；`-Paerocore_local_build=false` 改回專案內的 `build`。
`build publish` 仍寫到專案內的 `build/repo`（hdfv 取用的 Maven 資料夾）。

產物（在上述建置資料夾）：

- `neoforge\libs\aerocore-neoforge-1.21.1-<版本>.jar`：遊戲用前置與測試物件；放進 NeoForge 1.21.1 的 `mods` 資料夾，只保留一個 AeroCore 版本。
- `physics\libs\aerocore-physics-<版本>.jar`：不依賴 Minecraft 的 Java 計算函式庫。
- `cfd\distributions\cfd-<版本>.zip`：獨立 CFD 工具與 Java 相依檔；需要 Java 21。
- `*-sources.jar`：原始碼附件，無須放進遊戲。

遊戲中可用：

```text
/aerocore sample
/aerocore sample 30 5
/aerocore sample 30 25
/aerocore air 3000 30 5
/aerocore water 0.5 0
/aerocore water 0.5 0.5
```

`sample` 參數依序為空速（0–100 m/s）與攻角（−180–180 度）；預設為 30 m/s、5 度。
指令會計算 10 m² 翼面在 1.225 kg/m³ 空氣中的升力、阻力、動壓與失速狀態，無須生成載具。
只輸入速度時使用 5 度攻角。這是固定條件的驗算，不會讀取玩家的速度或改變世界。

`air` 參數依序為**位勢高度 m、空速 m/s、攻角度數**，缺省值是 0、30、5。
高度允許 −500～20000 m；輸出氣溫、氣壓、密度、Re、Mach 與升阻力，翼面 10 m²、弦長 1 m。
預設翼型仍為近似曲線，Re／Mach 是診斷值；要改變翼型係數需提供相應資料。

`water` 參數為**球半徑 m、球心深度 m**，預設 0.5、0。
深度正值在水下，0 表示球心位於平水面，因此恰好半浸水；深度等於半徑時完全浸水。
輸出排水體積、浮力與浮心。尚未扣除物體重量，指令本身不會生成物件或改動 Minecraft 的水。

## CFD 實驗與流場圖

先執行 `:cfd:run`。繪圖是選用的 Python 工具，不影響 Mod 或 CFD 的 Java 執行：

```powershell
python -m pip install -r scripts/requirements-cfd-plot.txt
python scripts/plot-cfd.py cfd/build/experiments/water/field.csv
```

圖檔會存成同一目錄的 `flow.png`。CFD 的邊界條件、單位、驗證誤差與限制見 [CFD 實驗說明](docs/CFD.md)。

## 在遊戲中測試浮沉

從 IDE 的 **Minecraft Client** 重新啟動遊戲，進入已開啟作弊指令的世界，朝向空曠處或水面：

```text
/aerocore probe spawn 250
/aerocore probe inspect
/aerocore probe spawn 1000
/aerocore probe current 1 0 0
/aerocore probe wind 10 0 0
/aerocore probe clear
```

球的直徑固定 1 m：250 kg 球會漂浮，1000 kg 球會下沉；兩者都會在乾地落下並碰撞停止。
水池需要足夠深／寬，避免球先碰到池底或岸邊。右鍵球體可查看速度、浸水比例、浮力、重量與阻力。
風／水流指令作用於 32 格內你生成的最近一顆球，`clear` 清除 64 格內你的測試球。
測試球為暫時實體，不寫入存檔，運作 12000 tick 後自動消失。
完整操作與模型限制見 [物理測試球](docs/PROBE.md)。

## 四角浮力與可調配重平台

重新啟動 **Minecraft Client**，在開啟作弊的世界中朝向空曠、至少 3 格深的靜水：

```text
/aerocore platform spawn
/aerocore platform tilt 0 15
/aerocore platform ballast 0.8 -0.35 0
/aerocore platform inspect
/aerocore platform reset
/aerocore platform clear
```

每次改動後等平台穩定再測下一項。平台具有三軸平移／旋轉、四角獨立浮力及 200 kg 可調配重；
右鍵可查看角速度、重心、浸水率和流體力矩。配重／傾角設定會清除運動，屬於外部實驗調整。
轉動碰撞使用保守包圍盒，碰岸後停止。完整操作、翻覆實驗和驗證見 [六自由度平台](docs/PLATFORM.md)。

## 受力、流場與熱交換

生成平台後：

```text
/aerocore platform debug all
/aerocore platform wind 4 0 0
/aerocore platform thermal body 80
/aerocore platform thermal water 5
/aerocore platform inspect
```

紅／青／黃／綠箭頭分別表示重力、浮力、阻力與合力；白／藍箭頭表示周圍空氣／水的流速。
箭頭由伺服器取樣後用粒子顯示，只對平台擁有者可見。`debug all_relative` 改看相對平台的流速，`debug off` 關閉。
流場是平台使用的環境取樣，尚未求解平台造成的尾流；離線二維 CFD 圖仍可獨立產生。
溫度會改變流體密度、黏度和聲速，平台會與空氣／水交換熱量。完整指令、比例尺與模型限制見 [熱力學與視覺化](docs/THERMAL_DEBUG.md)。

## 接觸、波浪、螺旋槳與摩擦生熱（0.6）

```text
/aerocore platform propeller 15
/aerocore platform propeller -10
/aerocore platform waves 0.2 16 0
/aerocore platform waves off
/aerocore platform debug forces
/aerocore platform inspect
```

- **接觸反力**：平台碰到方塊、可碰撞實體或世界邊界時，以序列衝量求出法向反力與庫侖摩擦（μ 0.5、恢復係數 0.2），
  能落地靜置、滑動、彈開與受力翻倒；洋紅箭頭是每個接觸點的力。
- **螺旋槳**：艉部 0.3 m 三葉槳，指令設定轉速（−30～30 轉/秒，正值推向平台局部 +X／紅線方向）。
  推力與軸扭矩來自葉素動量理論表，反扭矩會讓平台橫傾；橙色箭頭是推力。
- **波浪**：振幅（0～2 m）、波長（2～64 m）、方向（度，0 為 +X、90 為 +Z）的線性波，水深在設定時從平台下方水柱量出。
  只影響這個平台的浮力與水流取樣，Minecraft 水方塊本身不動；`debug flow` 會用淺藍點畫出波面。
- **摩擦生熱**：`inspect` 顯示阻力消耗的機械功率，以及依回復溫度模型傳進平台的熱；其餘熱量留在流體。
  0.7 起，平台在地面滑動時的接觸摩擦熱也會依兩種材料的熱滲透率分給平台與地面。
- **波壓與附加質量（0.7）**：波浪中的浮力改用隨深度衰減的線性波壓積分，並加入水平 Froude–Krylov 力與附加質量；
  淡紫箭頭是這些水動力慣性力。靜水中也有附加質量，所以平台與測試球的起伏會比 0.6 慢一些。

模型與驗證見 [六自由度平台](docs/PLATFORM.md)、[物理模型](docs/PHYSICS.md) 與 [熱力學與視覺化](docs/THERMAL_DEBUG.md)。

## 世界波浪、測試船與站上甲板（0.9）

```text
/aerocore waves 0.3 16 0 24
/aerocore hull spawn
/aerocore hull propeller 12
/aerocore hull rudder 20
/aerocore hull inspect
/aerocore waves off
```

- **世界波浪區域**：以執行位置為中心、半徑 4～64 m 的線性波區域，隨世界存檔並同步給玩家；區域內的平台、測試船與測試球
  取樣同一片水面，客戶端隱藏原版水面頂面並畫出起伏的波面（外圈漸弱接回靜水）。`/aerocore platform waves` 改為以平台為中心建立區域。
- **測試船**：6 m 圓舭小艇，阻力為 ITTC-1957 摩擦 × Holtrop–Mennen 形狀因子 ＋ Michell 造波阻力，
  操縱力為條帶附加質量（含 Munk 力矩）＋ 依 Clarke 1983 校準的尾端升力 ＋ 橫流阻力；B 系列螺旋槳與在槳流中的舵。
  只能用指令設定，不能駕駛。見 [測試船](docs/HULL.md)。
- **螺旋槳**：前進象限改用 Wageningen B 系列回歸式（0.10 起平台與測試船為 B3-65，其他象限用 MARIN 量測四象限資料）。
- **浮筒波浪力**：垂直繞射改用與 Hulme 輻射資料一致的半球激振力表。
- **雙向碰撞**：平台、測試船與測試球互撞時以等大反向衝量推開彼此。
- **站上甲板**：玩家與生物可以站在轉動、傾斜的甲板上並被載著移動，重量會壓斜平台，走進物體會推動它。

畫面與玩家互動需在遊戲中實測；細節與限制見 [平台說明](docs/PLATFORM.md) 和 [物理模型](docs/PHYSICS.md)。

## 海況、尾浪、水花與滑行艇（0.10）

```text
/aerocore waves sea 1.0 6 30 96
/aerocore planing spawn
/aerocore planing propeller 35
/aerocore planing inspect
/aerocore waves calm 128
```

- **不規則海況**：JONSWAP 波譜（DNV-RP-C205，γ 依 Tp/√Hs 自動決定）、cos⁴ 方向分布、32 個分量合成；海域半徑可到 128 m。
- **同一個水面**：在海域內，AeroCore 物體、掉落物、生物、玩家（含視線與溺水）、原版船與鏡頭水下霧都讀同一個移動水面，
  原版實體也會被波浪的軌道速度帶著漂移；畫面重畫整片海域（隨距離分級細節）。
- **尾浪**：測試船（Michell 細船）、滑行艇與原版船（水面壓力）都產生 Kelvin 尾浪，畫面看得到，其他物體與原版實體也會感受到；
  在常駐海域外會自動帶一塊臨時靜水區。
- **水花**：艏波碎波時（Noblesse 判據）與滑行面（Savitsky 鬚狀水花）以真實重力拋射水滴。
- **滑行艇**：Fridsma 系列 6 m 硬舭艇，Zarnick 2D+t 條帶理論；對水槽資料阻力在 −15%～0%、航行俯仰低 0.5～1.1°（重載 2.4°）。
- **係數來源**：球體阻力、接觸摩擦與恢復係數、對流熱傳、風阻、舵、螺旋槳四象限、原版實體質量都改用文獻值。

見 [海況與尾浪](docs/SEA.md)、[滑行艇](docs/PLANING.md) 與 [物理模型](docs/PHYSICS.md) 的〈0.10 係數來源〉。客戶端畫面尚未實測。

## 二階海面、非定常尾浪、白沫與原版船（0.11）

```text
/aerocore waves sea 2.0 7 30 96
（坐上原版船，用前後左右划船）
/aerocore planing spawn
/aerocore planing propeller 35
```

- **二階水面**：Choppy Wave Model 二階展開，單一深水波即 Stokes 二階；所有讀取者（物理、原版實體、畫面）共用。
- **非定常尾浪**：每個來源一個 128² FFT 網格，線性自由面在移動座標中精確積分；轉彎的尾浪是彎的、停船後尾浪繼續擴散。
  對定常 Kelvin／Havelock 解：波阻差 0.5～1.1%、波形差約 1%。
- **碎浪與白沫**：向下加速度超過門檻處碎浪，面積對準 Monahan 白冠覆蓋率（風速由 Pierson–Moskowitz 從 Hs 推回），白沫以 3.5 s 衰減；
  船艏碎浪與滑行艇乾艉板留下白水。
- **原版船**：35 kg 合板小艇加上乘客的真實質量，52 格靜水力、輻射阻尼、ITTC 摩擦、以自己尾浪算出的興波阻力，
  150 W 划手經槳葉滑移模型推進，一人划速約 1.65 m/s；畫面顯示俯仰與橫搖。見 [原版船](docs/ROWBOAT.md)。
- **方塊材質**：冰、木、沙、礫石、泥土、泥巴、雪、羊毛、玻璃、金屬、岩石各有文獻摩擦與恢復係數。
- **滑行艇**：加入 Savitsky 2007 鬚狀水花阻力，對 Fridsma 阻力誤差改為 −10%～+8%。

見 [海況與尾浪](docs/SEA.md)、[原版船](docs/ROWBOAT.md)、[滑行艇](docs/PLANING.md) 與 [物理模型](docs/PHYSICS.md)〈0.11 新增模型與係數來源〉。客戶端畫面尚未實測。

## 浪頭、海岸、風與小船（0.12）

```text
/aerocore waves windsea 12 30
/aerocore waves wind weather 30
/aerocore waves survey
（坐上原版船，只按左或右用單槳轉向）
/aerocore planing spawn
/aerocore planing propeller 35
```

- **浪頭**：加入 Sharma–Dean 完整二階理論（低秩分解，與雙重和差約 1%），浪頭分布與 Forristall 2000 的擬合差 0.6% 內。
- **波浪碰到地形**：建立海域時以 2 m 間距測量海床；程函方程求相位、沿波線保持能量通量、島與岬角後方以 Sommerfeld 半平面解遮蔽、
  水深與陡度上限造成碎浪並在碎浪帶起白沫。物理、原版實體與畫面讀同一個海岸水面。改地形後執行 `survey`。
- **風**：海域的 10 m 風速與吹向；`windsea` 以 JONSWAP 受風域限制成長生成海況。風決定白冠、帶動表層水（0.53 u*）、推動原版船與 AeroCore 物體、吹偏水花。
  載一人的原版船在 10 m/s 風中漂流 6.2% 風速。
- **0.13**：陣風（σ_u ≈ 2.2 u*，陣風因子 1.18，對 Wieringa 1.22）；`wind weather` 讓風隨晴／雨／雷雨變化（預設 5／10／15 m/s，是規則）；
  `windsea` 量測每個位置順風到岸的風域，小湖只會有小波紋，下風處與島後隨距離重新長大；`survey clear` 取消海岸效果；
  白沫只算畫出來的海面、1 m 網格、每 tick 半數區塊，畫面平行計算並略去網格畫不出的短波。
- **0.14**：水面依光學著色（之前只依坡度明暗，長浪看起來是平的一層）；風浪朝目前風速能吹出的高度鬆弛（湖幾分鐘、海數小時，CEM）；
  地形與樹林依實測擋風表降低下風處的風（重新 `survey` 後生效）；靠岸時二階項用當地水深使浪頭變尖；原版船逐槳前衝（24% 擺盪）。
- **0.15**：水面 shader 逐像素算法線（網格略去的波譜分量＋Cox-Munk 小波紋到毛細極限），像素以下的波併入陽光閃爍；
  反射天空漸層與岸邊地形的暗色倒影；閃爍加 Smith 遮蔽；風浪成長不超過陡度上限、白冠依 Monahan 覆蓋率、40 m 外畫平均白沫。
  開發用 `AutoShot`：在 `%USERPROFILE%\.mcmod\runs\aerocore\client` 放 `aerocore-autoshot.txt` 後啟動開發客戶端，
  會開啟名稱以 AeroCore 開頭的 QA 世界、依步驟（`cmd`、`wait`、`shot`、`debug`、`fps`、`key`、`camera`、`look`、`clouds`、`quit`；0.20 加 `gui`、`client`）截圖後正常存檔離開，見 `AutoShot` 類別說明。
  0.33.1 起開發客戶端（`runClient`）一開就在第 2 個螢幕、不取得焦點、不鎖滑鼠（`-Pdev_window_monitor=<n>` 換螢幕，0 照原樣），
  並關掉 NeoForge 的早期載入視窗（它開在主螢幕並取得焦點）；見 [API](docs/API.md)〈0.33.1 開發用視窗〉。
- **0.16**：`/aerocore wind`（全世界的風：查詢、`set`、`natural`）、`/aerocore waves natural on|off`（自然生成浪）、`/aerocore ground`（腳下地面的輪胎與土壤數值）。
  見 [大氣](docs/ATMOSPHERE.md) 與 [地面載具](docs/GROUND.md)。
- **0.24 輪子與傳動**（給載具 Mod 用的計算 API）：傳動約束求解（齒輪、離合器、煞車、差速器、限滑）、車輪的地面射線、
  由尺寸與胎壓算出的輪胎（暫態滑移）、懸吊與煞車、車軸與起落架零件、手排／自排／雙離合／CVT 與三種四驅的機構，以及離線測試場。
  見 [傳動](docs/DRIVETRAIN.md)、[輪子](docs/WHEELS.md)、[地面](docs/GROUND.md)。
- **0.25 製造計算**：ISO 286 公差等級、車削與鑽孔時間、依功率的粗加工、粗糙度、Chvorinov 凝固時間、鍛造與軋延力、焊道與鉚接、
  複材積層與固化、輪胎硫化（給載具 Mod 的機床與組裝用）。見 [製造](docs/MANUFACTURE.md)。
- **0.26 動力、石油、柴油與油箱**：動力（電機、電池、電纜、皮帶與天軸、蒸汽機；見 [動力](docs/POWER.md)）、石油（原油分析、蒸餾、FCC、重組、蒸汽裂解、油藏、抽油、鑽井；
  見 [石油](docs/PETROLEUM.md)）、柴油引擎與渦輪增壓（見 [引擎](docs/ENGINE.md)）、載具油箱與耗油。
- **原版船**：乘客跟船一起傾斜，第一人稱鏡頭隨船滾轉；槳葉速度受划手拉槳速度限制，單槳轉向穩定在 58°/s。
- **滑行艇**：Garme 艉板修正後，航行俯仰與 Fridsma 差 0.35° 內，縱搖不穩起始點與水槽一致。

見 [海況與尾浪](docs/SEA.md)、[原版船](docs/ROWBOAT.md)、[滑行艇](docs/PLANING.md) 與 [物理模型](docs/PHYSICS.md)〈0.12 新增模型與係數來源〉。客戶端畫面尚未實測。

## 改裝與車用空氣套件（0.34）

給 hdfv 0.16 的改裝廠：

- **原地改裝**（`PartVehicle.refit`）：物理可以不同的新藍圖在原地換上，車停在原處；零件狀態與軸轉速依零件 id 帶過，油截在新油箱容量內並回報倒掉多少。
- **車用空氣套件**（`body.RoadAero`，機體可帶）：車底通道與擴散器、側裙、擋風板、分流板、擾流板唇、倒裝尾翼（升力線查表）、外掛件的阻力；
  車底從地面射線取離地高度。是一維通道與文獻相關式，見 [機體](docs/BODY.md)〈車用空氣套件〉。沒有套件的機體物理不變。
- **0.35.1 車底校準**：車底通道加上車輪（前輪損失、後輪尾流讓擴散器回收少六成）、側裙改成封住漏氣面積、擋風板分離區取代車底壓力，
  對 Porsche 718 Cayman GT4 的擴散器與 AMG GT R 的車底擋板在 ±15 % 內；hdfv 範例轎車的溫和套件在 200 km/h 從 4.0 kN 變 1.78 kN。

## 機加工的計算（0.35）

給 hdfv 0.17 的九族機床與耗材：刀具壽命（Taylor）、磨削、研磨與珩磨（Preston）、滾齒與其他齒輪加工、拉削、鋸切、螺紋、鉚接、
放電（Weller）與線切割、電化學（Faraday）、雷射與電漿的熱平衡、水刀（Hashish）、輔助氣體的阻塞流與氣瓶、空氣分離的能耗。
見 [製造](docs/MANUFACTURE.md)〈機加工的計算（0.35）〉。

## 聲音（0.37）

給 hdfv 0.19 的程式合成聲音（不帶任何音檔）：

- **聲源模型**（`api.acoustics`）：
  - 活塞引擎依曲柄角點火、經排氣管波導；渦輪與噴流（Lighthill）、衝壓、火箭、脈衝噴射。
  - 螺旋槳與旋翼（Gutin）、電動機與變頻器；輪胎（胎紋、滑移、路面）、風（Curle）、水（Wagner 拍水）。
  - 撞擊（模態合成）、放氣（阻塞噴流）、機器的製程、喇叭與船笛。
- **傳播**：ISO 9613 的擴散與空氣吸收、都卜勒與延遲時間（超音速時錐外聽不到、錐掃過時音爆，Carlson 法）。
  房間的迴響（從聽者射線估 Eyring T60）、牆與車艙的隔音（質量定律），真實聲壓級對播放音量的響度對應。
- **接 Minecraft 的聲音引擎**：
  - 程式產生的靜態聲音、自己的合成與補塊執行緒的串流（主執行緒卡住也不斷聲）。
  - OpenAL EFX 的迴響與低通（只碰程式合成的聲音）。
  - 伺服器送的撞擊與刮擦事件、追蹤範圍外的大聲載具。

見 [聲音](docs/ACOUSTICS.md)。

## 機場的燈與空氣資料（0.38）

給 hdfv 0.20 的機場塔台：

- **大量固定燈**（`api.light`、客戶端 `FieldLamps`）：跑道燈、進場順閃燈、PAPI 的分色、燈標的交替；亮度依 Allard 定律與能見度，
  一批畫到 8 km（視距外沒有區塊也看得到），千盞約每幀 0.3～0.5 ms。
- **空氣資料**（`AirData`、`probe/WorldAir`）：壓力與密度高度、QNH、空速換算、頂風與側風；一點的風與空氣和載具感受到的相同，十分鐘最大陣風、盛行風、能見度。
- **載具清單**（`VehicleTracks`）：一點附近的載具與它們的位置、速度、航向、離地高。
- **選用的材質著色**：法線貼圖、反射玻璃、遠處玻璃亮度（材質自己選用，舊材質不變）。

見 [零件的繪製](docs/RENDERING.md)〈大量固定燈〉〈選用的材質著色〉、[大氣](docs/ATMOSPHERE.md)〈空氣資料與一點的風〉。

## 美化著色（0.39）

給 hdfv 0.22 的全面美化。客戶端設定「美化著色」（Mod 清單的設定畫面，預設開；關掉時畫面和 0.38 一樣）：

- **材質**：材質 JSON 的 `beautify` 子物件在開著時蓋上去（hdfv 用它給每種材質法線貼圖與反射玻璃）；切換時零件重烤，新的烤好前照舊畫舊的。
- **光**：方塊光在被照到的地方加暖、在光滑面上有暖色高光（白天淡）；地面反射依鏡頭附近的地面顏色。
- **遮蔽**：四角都暗、中間開闊的長面也會細分（比較面中心），不再整面發黑。
- **燈罩光暈**：零件的燈罩依亮度與朝向畫光暈（256 m 內）。
- **接觸陰影**：載具底下依底部大小的柔和陰影，取代原版的圓形影子。
- **繪製**（不受開關影響）：GPU 上的零件每種材質只設定一次著色器，可動件多的載具畫得更快。
- 截圖比對工具 `scripts/ShotDiff.java`。

見 [零件的繪製](docs/RENDERING.md)〈美化著色（0.39）〉。

## 專案分工

```text
physics/       純 Java：大氣、流體、翼型、升阻力、浮力、力矩與測試
cfd/           獨立 Java：二維 D2Q9 流場實驗、資料輸出、解析解驗證
neoforge/      Mod 入口、測試球、世界水體取樣、碰撞／同步、驗算指令與 GameTest
scripts/       本機開發環境準備
docs/          API 使用方式與物理模型說明
```

從 [API 使用說明](docs/API.md)開始接入載具；模型假設與單位見 [物理模型](docs/PHYSICS.md)。

## 目前範圍

即時核心已提供高度大氣模型、密度／黏滯性、Re／Mach、局部風速／水流取樣、
升阻力與俯仰力矩、角度／雷諾數翼型表插值、球形浮體的部分浸水體積／浮心，以及球體黏滯阻力。
舊版 `AirState` 與簡易翼面建構式仍可使用；需要實際弦長的功能請改用新建構式。

新增平移積分，以及測試球的重力／流體受力、Minecraft 方塊碰撞、伺服器運算與客戶端位置插值。
0.7 起測試球是六自由度實心球，使用與平台相同的受力、附加質量與球形接觸。
0.4 新增四元數姿態、完整慣性張量、世界角動量積分、多點浮力與耦合阻力。
浮力平台可測試回復、偏心傾斜及高重心翻覆；操作與限制見 [六自由度平台](docs/PLATFORM.md)。
0.5 加入溫壓流體性質、均溫物體熱交換、受力和環境流速顯示；尚未解空間溫度場、相變或熱浮力對流。
0.6 以球形接觸元件的序列衝量取代平台的「碰撞即停止」，加入指定線性波、BEMT 螺旋槳與阻力摩擦生熱。
接觸只有單一運動物體對靜止障礙；波浪是給定的流場，不會反射、繞射或改變水方塊；螺旋槳不含空化、通氣、斜流與船體伴流。
0.7 加入接觸摩擦生熱、線性波壓的深度衰減（垂直 Froude–Krylov）、水平 Froude–Krylov 力與球形附加質量（C_a 0.5）。
0.8 加入浮筒的波浪輻射阻尼（Hulme 半球精確資料、Cummins 時域記憶力）與浮筒間的點吸收體干擾，靜水起伏會在幾秒內衰減；測試球也有輻射阻尼。
0.9 加入與輻射一致的浮筒繞射力、B 系列螺旋槳、細長船體與舵、物體之間的雙向接觸、原版實體在甲板上的碰撞／載運／重量，
以及世界波浪區域與客戶端波面。
0.10 加入不規則海況、共用水面（原版實體、原版船與鏡頭也跟著波浪）、Kelvin 尾浪、水花與 2D+t 滑行艇；水方塊本身仍不動。
0.11 加入二階水面、非定常尾浪、碎浪白沫、原版船物理與方塊材質；0.12 加入完整二階浪頭、海岸的折射與遮蔽、海域的風。
本版不包含引擎模型、旋翼、方塊組裝，也沒有全世界流場、三維湍流、完整的波浪傳播求解（海岸以幾何光學處理，沒有反射；尾浪是線性解）、
碎浪動力、空化或雙向流固耦合。
CFD 不會在 Minecraft tick 中執行，也尚未自動生成可用於載具的翼型資料。
失速曲線是遊戲用途的近似，並非真實翼型的風洞資料。

專案尚未選擇對外開源授權，Mod 中繼資料暫設為 `All Rights Reserved`。
上游開發範本授權另見 `TEMPLATE_LICENSE.txt`。
