# AeroCore 開發代理指南

本文件適用於整個專案。開始修改前先讀本文件，再讀相關模組與文件；功能狀態以實際原始碼、設定和本次測試為準。
與使用者溝通以繁體中文為主，說明完成的功能、驗證結果及仍存在的模型限制。

## 專案目標與現況

- 為 Minecraft Java 載具 Mod 建立通用的空氣／水動力前置。2026-10-11 使用者決定：
  - 載具是另一個獨立 Mod **High Degree of Freedom Vehicles**，在 `Z:\MCMod\HighDegreeOfFreedomVehicles`，有自己的 git 倉庫。AeroCore 只當它的前置，不在 AeroCore 裡做可駕駛的載具。
  - 載具 Mod 走《Automation》式高自由度：玩家自己設計零件（每個參數都能自由設定、形狀也是參數），再用零件組合出載具；合成是 CNC／機加工類。
  - 測試台、設計畫面、物品與模型都在載具 Mod（Mod ID `hdfv`）。
  - 2026-10-11（0.20）使用者決定「AeroCore 是計算 API」，零件「全部搬到 hdfv」：零件設計（參數、檢查、JSON、質量估計）與引擎的控制規則在 hdfv；
    AeroCore 提供物理與從設計抽出的計算、零件的執行時受力（`api.vehicle.part`）、通用載具框架（藍圖格式、組裝與連接、`VehicleDynamics`）、
    同步與存檔，以及海浪與尾浪的計算與渲染（使用者：「浪的渲染要在 AeroCore」）。hdfv 缺什麼物理或計算，就在 AeroCore 補成通用 API。
    架構見 [docs/VEHICLE_PARTS.md](docs/VEHICLE_PARTS.md)。
- 先維持可驗證的物理核心，再逐步接入載具玩法。不要把近似模型描述成工程級 CFD。
- 截至 2026-10-20，版本為 **0.40.0**：具備六自由度剛體、四角浮力、可調配重、受力／環境流速粒子顯示、溫壓流體性質與基礎熱交換；
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
  非設計點平衡、依飛行馬赫數的運轉線、準穩態暫態），進氣道、壓縮機、燃燒室、渦輪、噴嘴／尾管、燃油控制六個元件組成 `aerocore:jet_engine` 零件；
  0.20（2026-10-11，使用者決定 AeroCore 是計算 API）把所有零件設計與範例搬到 hdfv（類型 ID 改 `hdfv:*`），零件的受力留成 `api.vehicle.part`
  的執行時零件，從設計抽出計算 API（`PistonGeometry`、`EngineStrength`、`EngineMap`、`EngineRuntime`、`Turbomachinery`、`ElectricMotor`、
  `PropellerModels`…），渲染與尾浪改由零件／登記提供，並發佈到工作區 Maven 資料夾供 hdfv 取用；
  0.21（2026-10-11，使用者要求加入渦扇、渦槳、渦軸等噴射推進家族）加入 `Turbofan`、`FreeTurbine`（共用套件內的 `GasPath`、`TwoSpool`），
  驗證與限制見 [docs/TURBOFAN.md](docs/TURBOFAN.md)；
  0.22（2026-10-11，使用者要求「機翼與輪子也是高自由度」）加入 `api.aero`：翼型（`Airfoil`、`AirfoilAnalysis`、`SectionPolar`）、
  機翼（`Wing`、`WingStructure`）、空氣螺旋槳（`AirPropeller`）、直升機旋翼（`HelicopterRotor`），執行時零件 `WingPart`、`AirPropellerPart`、`RotorPart`，
  傳動的單向離合器與陀螺力矩，驗證與限制見 [docs/WING.md](docs/WING.md)；
  0.22.1（2026-10-11，hdfv 0.4 開發時一台直升機的物理錯誤讓 GameTest 伺服器當機）把物理例外隔離在單一載具：
  `BodyWorld` 逐一包住每個物體的子步驟呼叫，`PartVehicle` 出錯時停在原處、記錄並存檔原因（`/aerocore vehicle info`、`resume`），
  GameTest 中改為讓所屬測試失敗；
  0.23（2026-10-11～12，使用者要求「機倉也要高自由度 像模塊一樣」「車身 船身 也是」）加入 `api.body`：模組串成的機體
  （`SectionShape`、`BodyModule`、`BodyShape`、`BodyAerodynamics`、`BodyStructure`）、執行時零件 `BodyPart`、零件的掛載點與 `Mount` 的接上、
  任意站位與斷階的滑行船體、離線的拖曳水槽與橫傾試驗 `TowingTank`，驗證與限制見 [docs/BODY.md](docs/BODY.md)；
  同版（2026-10-12，hdfv 0.5 的快艇試算台發現輕載滑行船體縱傾低 1°、阻力到 1.7 倍）加入 2D+t 的船底斜升係數（`PlaningDynamics.liftFactor`）
  與舭乾時到艉板為止的鬚狀水花，對 Savitsky 驗證見 [docs/PLANING.md](docs/PLANING.md)（版本號仍是 0.23.0）；
  0.24（2026-10-12，使用者要求「車子要有前驅、後驅、4WD 和前置、中置、後置」，選了四種變速箱、三種四驅、手排自己踩離合器、
  真實地形加路面坡道方塊）加入傳動約束求解（`Drivetrain`：齒輪、離合器、煞車、差速器、限滑、單向、黏性、變扭器）、
  地面射線（`api.ground`、遊戲端 `WorldGround`）、由尺寸與胎壓算出的輪胎與暫態滑移（`api.wheel`）、懸吊、煞車、
  車軸與起落架零件（`AxlePart`、`LegPart`）、變速箱與分動箱的機構（`TransmissionPart`、`TransferPart`、`api.drive`）、
  離線測試場 `TestTrack` 與給客戶端的零件顯示值；見 [docs/DRIVETRAIN.md](docs/DRIVETRAIN.md)、[docs/WHEELS.md](docs/WHEELS.md)。
  0.25 加製造計算 `api.manufacture`（ISO 286 公差、切削、鑄造、成形、接合、複材與硫化；給 hdfv 的機床、CNC、焊接與組裝），
  見 [docs/MANUFACTURE.md](docs/MANUFACTURE.md)。
  0.26 加動力（`api.power`：電機、電池、電纜、皮帶、天軸、蒸汽機）、石油（`api.petroleum`：原油分析、蒸餾、FCC、重組、蒸汽裂解、
  油藏、抽油、鑽井）、柴油引擎與渦輪增壓（`EngineCycle.CompressionIgnition`、`DieselMap`、`Turbocharger`）與載具油箱（`Part.Tank`、
  `PartStep.burn`）；見 [docs/POWER.md](docs/POWER.md)、[docs/PETROLEUM.md](docs/PETROLEUM.md)。
  0.27 加石油的操作條件（FCC 反應器出口溫度、重組辛烷值、裂解爐出口溫度、加熱爐的閃蒸溫度、井口油嘴；給 hdfv 0.9 的煉油與油田畫面）。
  0.28（2026-10-14，使用者「下一步」：衝壓、超燃、脈衝噴射，加上後燃器、渦輪衝壓、固體火箭助推器、氫燃料與極高速載具）加
  `api.gasdynamics`（震波、錐形流、Rayleigh／Fanno、廣義一維流、局部傾角表面壓力、氣動加熱）、`api.propellant`（KNSB／APCP 與固體火箭）、
  `api.storage`（液氫與 700 bar 氫）、`SupersonicInlet`、`Ramjet`、`Reheat`、`TurboRamjet`、`Scramjet`、`Pulsejet`，
  零件自有油箱與拋棄（`Part.Tank.own`、`PartStep.jettison`、`PartVehicle.castOff`），機體的超音速阻力、超音速機翼，
  以及極高速載具的碰撞走廊、路徑區塊票與暫停；見 docs 的 GASDYNAMICS、ROCKET、HYDROGEN、RAMJET、SCRAMJET、PULSEJET、HIGH_SPEED。
  0.29（2026-10-15，使用者「3d model 還要更逼真 我有裝blender」「機器也要用blender」，hdfv 0.11／0.12）加零件繪製的通用工具：
  純 Java 的 `api.mesh`（`MeshBuilder` 形狀與公尺 UV、材質鍵與細節標記、`MeshOps`、OBJ 讀寫、環境光遮蔽與看不見的面、烘焙），
  客戶端的背景烘焙、材質貼圖（mip、重複）、記錄後分組送出的 `PartLayers`、零件著色器（高光、天空反射，Iris 時改原版）、
  `api.engine.Plume`／`Blackbody` 與尾焰、旋翼模糊的繪製；見 [docs/RENDERING.md](docs/RENDERING.md)。物理數字不變。
  0.30（2026-10-15，使用者「機器 載具 也用blender 做」，hdfv 0.12：機器與載具一起，完整內裝、燈、可動件、地面與水面效果）加：
  網格的內裝、玻璃、燈、遠處版本與件的標記，`SurfaceCut`（在放樣格子上挖窗、門），可動件的烘焙與 `PartRenderer.Drawn`，燈（燈罩、燈暈、
  光束與光斑）、晚段半透明佇列、清漆／珍珠漆／鏤空／玻璃的著色，車輪的滑移與載重、可收放起落架（含放下時的阻力）、旋翼揮舞與槳距、
  螺旋槳槳葉角的顯示值，`Downwash`、`GroundMarks`、`RotorWash`；既有物理數字不變，新功能預設關閉。
  0.31（2026-10-17，使用者「加入痛車」，hdfv 0.13：上傳圖片當貼紙、烤漆房）加執行時材質（`PartFinishes.define`、`RuntimeTexture`）
  與只換外觀的藍圖（`PartVehicle.replaceBlueprint`、`VehicleState.restyled`）；物理數字不變。
  0.32（2026-10-17，使用者選「先補 0.13 的驗證」，hdfv 0.14）加 `PartVehicle.place`／`VehicleState.placed`（搬車保留油箱與狀態），
  `AutoShot` 的 `server`（連本機伺服器）與 `key <按鍵名稱>`；`BodyShape.overhang`（零件放不放得進機體，hdfv 的引擎室規則）。
  0.33（2026-10-17，使用者「底盤也要高度自由化」，hdfv 0.15：玩家設計的底盤、車軸自由擺放、硬點懸吊、車高與離地，車架剛性也進物理、
  承載式車身也會扭）加會扭的車架（`FrameTwist`、`FramePart`、`FrameFlex`：車輪先探測、站點扭角隱式求解）、設計時的梁 FEM（`api.frame`）、
  承載式車身的扭轉剛度（`BodyStructure.torsion`）、硬點懸吊的運動學（`api.wheel.kinematics`：外傾、bump steer、抗點頭／抗後蹲）、
  氣壓彈簧與調高閥、有號距離的阿克曼（後輪轉向）、離地幾何（`Clearance`）；見 [docs/FRAME.md](docs/FRAME.md)、WHEELS.md。
  沒有車架、硬點表、氣壓彈簧的載具，物理逐位元不變。
  0.33.1（2026-10-18，使用者「我在打lol 的時候 開minecraft 都在第2個螢幕 還有把不要搶滑鼠」）只改開發環境：開發客戶端一開就在第 2 個螢幕、
  不取得焦點、不鎖滑鼠（`DevWindow`，見下方 AutoShot 一節與 docs/API.md〈0.33.1 開發用視窗〉）；API 與物理不變。
  0.34（2026-10-18，使用者「加入載具調整系統 可以有自由外觀和大小」，hdfv 0.16 的改裝廠）加改裝換車
  （`VehicleState.refitted`、`PartVehicle.refit`：物理可變、停在原地、依零件 id 帶過狀態、油截在新容量內）與車用空氣套件
  （`body.RoadAero`：車底通道與擴散器、側裙、擋風板、分流板、擾流板唇、倒裝尾翼、外掛件阻力；`BodyPart` 可帶；見 docs/BODY.md）。
  沒有套件、不改裝的載具物理逐位元不變。
  0.35（2026-10-18，使用者「加入所有機加工機器」，hdfv 0.17：九族機床、新舊機器的耗材、空氣分離機）在 `api.manufacture` 加機加工的計算：
  刀具壽命（`ToolLife`：Taylor、經濟壽命）、磨削（`Grinding`）、研磨與珩磨（`Finishing`：Preston）、齒輪加工（`GearCutting`）、拉削（`Broaching`）、
  鋸切（`Sawing`）、螺紋（`Threading`）、鉚接（`Riveting`）、放電（`Edm`：Weller）、電化學（`Ecm`：Faraday）、雷射與電漿（`ThermalCutting`）、
  水刀（`Waterjet`：Hashish）、氣體（`Gases`：阻塞流、氣瓶）與空氣分離（`AirSeparation`）；見 [docs/MANUFACTURE.md](docs/MANUFACTURE.md)。只加新類別，其他不變。
  0.35.1（2026-10-18，hdfv 0.16 的改裝廠發現範例轎車的溫和套件在 200 km/h 有 4.0 kN、車底通道 2.96 kN）把 `RoadAero` 的車底對量產車資料校準：
  車輪（前輪損失 `K_WHEELS`、後輪尾流後擴散器只剩 `WHEEL_RECOVERY`；Breslouer & George 2008）、側裙封住漏氣面積的比例、擋風板分離區取代車底壓力、
  量產車的平整底板 `SMOOTH` 0.045、粗糙車底後的擴散器 `ROUGH_RECOVERY`（Marklund 2013）；對 718 Cayman GT4 的擴散器與 AMG GT R 的車底擋板在 ±15 % 內，
  範例套件 1.78 kN（車底 0.75 kN）；見 docs/BODY.md。簽名不變，沒有套件的機體逐位元不變。
  0.36（2026-10-18，使用者「做鉸接拖車」，hdfv 0.18 的平頭牽引車與鵝頸低底板拖車）加兩台載具的鉸接：`api.vehicle.Coupling`（球頭、第五輪：俯仰、折刀、側傾的限位與彈簧）、
  `ContactSolver` 的接頭列（點 3×3 精確解、角度列、位置修正）、離線的 `TestTrack.convoy`、世界中的連結（`PartVehicle.couple`、`Couplings`、
  `BodyWorld` 整組走、存讀、區塊票、客戶端從前車的掛點畫）、伸縮支柱 `LegPart.Telescope`（支撐腳）與梁 FEM 的 pins（主銷）；見 [docs/COUPLING.md](docs/COUPLING.md)。
  沒有接頭、沒有伸縮支柱、沒有 pins 時逐位元不變。
  0.37（2026-10-18，使用者「加入聲音」，hdfv 0.19；全部程式合成、不帶音檔）加聲音：純 Java 的 `api.acoustics`（DSP 與分析、
  ISO 9613 的傳播與空氣吸收、都卜勒與延遲時間、音爆、房間、隔音、響度對應、引擎、渦輪、旋翼、電動機、輪胎、風、水、撞擊、放氣、機器製程、喇叭的聲源模型），
  客戶端接 Minecraft 聲音引擎的通用接口（`client/sound`：執行時的靜態聲音、自己的合成與補塊執行緒的串流、`LiveSound`、EFX 迴響與濾波、聽者的房間掃描），
  伺服器的接觸事件（`VehicleContacts`：撞擊、刮擦，`ContactSolver` 的接觸只讀）與追蹤範圍外的大聲載具（`FarSounds`）；見 [docs/ACOUSTICS.md](docs/ACOUSTICS.md)。
  物理逐位元不變。同一版另修關閉世界時的區塊卸載卡死（`ChunkUnloadMixin`，見約定）。
  0.38（2026-10-19，使用者「加入機場塔台」，hdfv 0.20）加大量固定燈（`api.light`：燈號、分色、Allard 可見度；客戶端 `FieldLamps` 一批畫到 8 km）、
  空氣資料（`AirData`、`probe/WorldAir`：一點的風與空氣和載具感受的相同、能見度、十分鐘最大陣風、盛行風）、載具清單（`VehicleTracks`、`PartVehicle.touching`）、
  方塊標籤 `#aerocore:smooth` 與選用的材質著色（法線貼圖、反射玻璃、遠處玻璃亮度）；見 [docs/RENDERING.md](docs/RENDERING.md)、[docs/ATMOSPHERE.md](docs/ATMOSPHERE.md)。
  0.39（2026-10-20，使用者「開始做 0.22 全面美化」，hdfv 0.22）加客戶端設定「美化著色」（`client/Beautify`，預設開，關掉＝0.38 的畫面）：
  材質的 `beautify` 子物件、方塊光的暖色與高光、依地面上色的反射、比較面中心的遮蔽細分（`AmbientOcclusion.Refine`）、燈罩光暈（`LensPatches`）、
  接觸陰影（`ContactShadows`）；`PartGpu` 每種材質只套一次著色器（可動件多時快很多，`GPU_MIN_QUADS` 96 → 32）；截圖比對 `scripts/ShotDiff.java`。
  物理逐位元不變，沒選用新鍵的材質畫面不變。
  0.40（2026-10-20，使用者「先改合成配方」，hdfv 0.23 的科技樹、電弧爐與電解槽）加冶金與電化學的計算 `api.metallurgy`：
  物種的生成焓與莫耳質量（`Species`）、配平的反應與 Hess 反應焓、鋁熱能否自持（`Reaction`）、Faraday 電解（`Electrolysis`：鋁、鎂、氯酸鹽、
  過氯酸鹽、水）、電爐（`Smelting`：電弧爐熔鋼、矽鐵、鉻鐵、鈦渣、Kroll、鋁熱法、Pidgeon、煅燒）、礦物品位（`Minerals`），以及甲醇合成
  （`api.petroleum.Methanol`）；見 [docs/METALLURGY.md](docs/METALLURGY.md)。只加新類別，其他不變。
- 獨立二維 CFD 實驗與離線流場圖仍保留。遊戲內粒子顯示目前是環境取樣；尾浪是線性自由面的非定常解（勢流），不是黏性流場求解。
- AeroCore 不出可駕駛的載具與零件設計（`PartVehicle` 是給載具 Mod 用的通用實體；0.20 起載具 GameTest 在 hdfv）；尚無簧下質量與輪胎溫度、三維 CFD、碎浪動力、
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
- `Z:\MCMod\AeroCore` 與網路資料夾的 UNC 路徑（實際位址在工作區的 `Z:\MCMod\AGENTS.md`，不在倉庫裡）是同一份專案的別名，不是兩個 checkout。
- Windows Shell 操作優先使用 `Z:\MCMod\AeroCore`，避免 CMD 的 UNC 工作目錄限制。
- 2026-10-11 從 `Z:\MCMod` 搬進 `Z:\MCMod\AeroCore`（`.git`、`.idea` 一起搬）。兩台電腦的 IntelliJ 要改開新位置，再重新載入 Gradle；
  `.idea/workspace.xml` 裡舊路徑的執行設定快取會在重新載入後重建。
- 原始碼與發佈的 Maven 資料夾（`build\repo`）在專案內；建置輸出、遊戲世界、設定、記錄與截圖必須留在本機。
- 建置輸出在本機（2026-10-18，和 hdfv 一樣；hdfv 的開發遊戲經網路磁碟讀建置資料夾，啟動拖到 5 分鐘以上）：
  根 `build.gradle` 把每個專案的 `build` 放在 `%USERPROFILE%\.mcmod\build\aerocore-<路徑雜湊>\{root,physics,neoforge,cfd}`
  （雜湊是小寫的專案正式路徑的 SHA-256 前 8 碼；`Z:\MCMod\AeroCore` 是 `aerocore-e6babf45`，從 UNC 開的是另一份）。
  `.\dev.ps1 showBuildDirectories -q` 印出位置。`-Paerocore_local_build=false` 改回專案裡的 `build`。
  - 發佈仍寫專案的 `build\repo`（`physics`、`neoforge` 的 `publishing` 固定 `rootProject.file('build/repo')`，不跟著建置資料夾走）：
    hdfv 從那裡鏡像、另一台電腦也讀那裡。不要改成 `layout.buildDirectory` 底下。`clean` 也不再刪到它。
  - 網路資料夾裡的 `physics\build`、`neoforge\build`、`cfd\build`（`experiments` 以外）、`build\reports` 從此是舊的，不要讀。
    CFD 結果仍寫在專案的 `cfd\build\experiments`（`FlowExperiment` 的預設是相對於 `cfd` 的路徑）。
  - 兩台電腦各自建置，第一次建置要全部重編、重做 Minecraft 產物。
- 本機執行根目錄：`%USERPROFILE%\.mcmod\runs\aerocore`，每台電腦各自一份（兩台電腦的使用者資料夾不同）。
- 2026-10-10 起專案是 git 倉庫，依使用者要求上傳到公開的 GitHub `fiseleo/aerocore`；網路磁碟上的 git 指令要加
  `-c safe.directory=Z:/MCMod/AeroCore -c safe.directory=%(prefix)///<網路資料夾>/MCMod/AeroCore`（實際位址見工作區的 AGENTS.md；不改全域設定）。推送是對外公開的動作，每次先經使用者同意。
  大改前仍把原始碼備份到 `%USERPROFILE%\.mcmod\backups\`（0.9 開工前 `aerocore-src-0.8.0-20261010-002942`，0.11 開工前 `aerocore-src-0.10.0-20261010-130423`；
  0.12 開工時的狀態是 git commit `be82bc5`，沒有另做資料夾備份）。
- `client` 放開發客戶端資料，`server` 放開發伺服器資料，`gametest` 放自動測試世界。
- `aerocore_runtime_id=aerocore` 保持磁碟代號與 UNC 啟動結果一致。只有獨立實驗專案才使用不同 ID。
- JDK 位於 `%USERPROFILE%\.mcmod\java\jdk-21.0.12.1+1`，Java 暫存位於 `%USERPROFILE%\.mcmod\tmp`。優先使用啟動腳本偵測 JDK，不依賴舊的 AppData 路徑。
- NeoForm 暫存已放在 Gradle 使用者目錄的 `caches/aerocore-work`；不要改回網路磁碟。
- 所有遊戲 run（client／server／gameTestServer）在 `neoforge/build.gradle` 固定 `java.io.tmpdir` 與 `jdk.net.unixdomain.tmpdir`
  為 `%USERPROFILE%\.mcmod\tmp`。`第二台電腦` 的系統 TEMP 無法建立 Java NIO 的 Unix socket（`Invalid argument: connect`），
  會讓建立單人世界時崩潰；不要移除這兩個設定。

`neoforge/run` 是遷移前的備份，**不是目前遊玩的存檔**。原開發機已複製兩個世界與設定並逐檔驗證，78 個檔案的紀錄在該機本機根目錄的 `migration-manifest.json`。
`第二台電腦` 於 2026-10-07 建立環境時依使用者決定不複製世界，沒有 manifest；需要世界時由使用者從原開發機複製 `client/saves` 或新建 QA 世界。
2026-10-09 首次啟動客戶端時，使用者在此機另建了自己的「新的世界」（與原開發機的同名世界無關）。
不要自動重跑遷移、覆寫本機進度或刪除備份。`scripts/migrate-runtime.ps1` 僅供尚未遷移的環境；來源和目標世界均須關閉，腳本會取得鎖並拒絕內容衝突。
測試使用 `AeroCore 0.4 Platform QA` 或新建的 QA 世界，兩台電腦都不修改使用者的「新的世界」。測試結束由遊戲正常存檔離開，不以強制結束程序代替存檔。
`第二台電腦` 有 Chrome Remote Desktop（`remoting_host`）。經它遠端操作時 Minecraft 滑鼠視角異常（只能低頭左右轉，視窗常失焦暫停）；
先在「選項 → 控制 → 滑鼠設定」關閉「原始輸入」，仍不行就改在本機或用支援相對滑鼠的遠端工具（Parsec、Sunshine＋Moonlight）實測畫面。關閉原始輸入的效果尚未確認。

## 常用指令

在專案根目錄執行：

```powershell
.\open-ide.cmd --check
.\open-ide.cmd
.\dev.ps1 build
.\dev.ps1 build publish          # 改動後給 hdfv 用：輸出到 build\repo
.\dev.ps1 :physics:test :cfd:test
.\dev.ps1 :neoforge:runGameTestServer
.\dev.ps1 :neoforge:showRuntimeDirectories
.\dev.ps1 showBuildDirectories -q    # 這台電腦的建置資料夾與發佈位置
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
IDE 優先用 `open-ide.cmd` 開：專案的 Gradle JVM 設為 `#JAVA_HOME`，而 `第二台電腦` 的系統 `JAVA_HOME` 是 JDK 17、系統 TEMP 無法建立 Unix socket。
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
| physics 的 `api/vehicle/` | 通用載具框架（0.17）：零件介面與登記、藍圖、組裝、`VehicleDynamics`；`part/` 是零件的執行時受力（0.20）；零件設計在 hdfv；純 Java，載具 Mod 的測試台可離線呼叫 |
| physics 的 `api/body/` | 0.23 的模組化機體：截面、模組 → 站位與特徵量、空氣負載、結構；純 Java |
| physics 的 `api/ground/`、`api/wheel/`、`api/drive/` | 0.24：地面射線（`Ground`、`BoxGround`）；輪胎、接地、懸吊、煞車、車輪（`Tyre`、`TyreContact`、`Suspension`、`Brake`、`WheelCorner`）；變扭器與傳動公式（`TorqueConverter`、`Driveline`）；純 Java |
| physics 的 `api/aero/` | 0.22 的「形狀 → 性能」：翼型、極曲線表、機翼與其結構、空氣螺旋槳、直升機旋翼；純 Java |
| physics 的 `api/gasdynamics/`、`api/propellant/`、`api/storage/` | 0.28：可壓縮流（震波、錐形流、加熱與摩擦管流、廣義一維流、表面壓力、氣動加熱）；固體推進劑與火箭；氫的儲存；純 Java |
| physics 的 `api/acoustics/` | 0.37 聲音：`Voice`、`Noise`、`Filters`、`DelayLine`、`Oscillator`、`Modal`、`Mix`、`Dsp`、`Spectrum`、`Wav`；`Propagation`、`SourceTrack`、`SonicBoom`、`RoomAcoustics`、`Surface`、`Transmission`、`Loudness`；聲源 `PistonEngineNoise`、`TurbineNoise`、`RotorNoise`、`MotorNoise`、`TyreNoise`、`FlowNoise`、`WaterNoise`、`ImpactNoise`、`GasJetNoise`、`ProcessNoise`、`Horn`；純 Java |
| neoforge 的 `client/sound/` | 0.37：`RuntimeSounds`（程式產生的靜態聲音）、`SynthInstance`、`SynthStream`／`SynthThread`（合成與補塊執行緒）、`LiveSound`、`SoundHooks`、`Efx`、`Listening`、`SoundRecorder`、`DevSoundCommands`（`/aerocoredev sound`） |
| physics 的 `api/metallurgy/` | 0.40：`Species`、`Reaction`、`Electrolysis`、`Smelting`、`Minerals`（冶金與電化學）；`api/petroleum/Methanol`；純 Java |
| physics 的 `api/light/`、`api/AirData` | 0.38：`LightSignal`、`SectorColours`、`FieldLamp`、`Allard`（大量固定燈的資料與可見度）；`AirData`（ISA 的高度、QNH、空速、風分量）；純 Java |
| neoforge 的 `probe/WorldAir`、`vehicle/VehicleTracks`、`client/FieldLamps` | 0.38：一點的風與空氣（和載具感受的相同）、能見度、峰值陣風；一點附近的載具清單；大量固定燈的客戶端（自己的著色器 `rendertype_aerocore_field_lamp`，在 `PartLate` 畫） |
| physics 的 `api/mesh/` | 0.29：零件網格（`MeshBuilder`、`Mesh`、`Finish`、`MeshOps`、`ObjReader`／`ObjWriter`、`AmbientOcclusion`、`PackedMesh`、`MeshBaker`）；0.30 `Lamp`、`SurfaceCut`；純 Java，載具 Mod 的模型在這裡建、在客戶端畫 |
| physics 的 `api/engine/` | 活塞引擎的物理（0.18）：氣體、燃料、循環模擬、摩擦、點火標定；渦輪噴射的循環（0.19，`Turbojet`）；0.20 從設計抽出的幾何、強度、MAP 建立、執行時關係、渦輪機械；0.21 渦扇與自由渦輪（`Turbofan`、`FreeTurbine`，共用 `GasPath`、`TwoSpool`）；純 Java，可離線呼叫（測功機、測試台） |
| neoforge 的 `vehicle/` | 給載具 Mod 用的公開遊戲端 API：`PartVehicle`、`VehicleRegistry`、`VehicleNetwork`、`VehicleCommands`；客戶端 `PartVehicleRenderer`／`PartRenderer`／`PartMesh` |
| neoforge 的 `mixin/` | `aerocore.mixins.json`：雙端 `EntityCollideMixin`（實體對物體碰撞）、`EntityWaterMixin`（原版水高度與視線讀海面）、`BoatWaterMixin`（原版船水面）、`BoatPhysicsMixin`（原版船改由 `RowingBoats` 移動、槳速）、`BoatSeatMixin`（座位隨船傾斜）、`BoatAccessor`、`LivingEntityAccessor`、`GameTestTickerAccessor`（物體物理失敗時找它所屬的 GameTest）、`ChunkUnloadMixin`（0.37：丟掉過時的區塊卸載工作，關閉時不再卡在存檔）、`ChunkMapAccessor`（它的 GameTest）；客戶端 `LiquidBlockRendererMixin`（隱藏海面頂面）、`CameraFogMixin`（水下霧）、`BoatRendererMixin`（船的俯仰橫搖）、`BoatClientMixin`（槳動畫內插）、`RiderTiltMixin`（乘客模型隨船傾斜）、`SkyDomeMixin`（物理天空）、`CloudWindMixin`（雲隨風）、`WeatherWindMixin`（雨雪隨風傾斜）、0.37 的聲音：`SoundBufferLibraryMixin`（執行時的靜態聲音）、`ChannelAccessor`、`GuiAccessor`、`SoundEngineAccessor`、`SoundManagerAccessor`；只能放 mixin 類別 |
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
（0.17～0.19 的零件設計與範例在 0.20 搬到 hdfv）；
0.20：`api.vehicle.part` 的 `SpherePart`、`HullPart`、`PlaningHullPart`、`PropellerPart`、`ControlSurfacePart`、`BoxPart`、`MotorPart`、`MassPart`；
`api.vehicle` 的 `MassShapes`、`WakeSource`；`api.engine` 的 `PistonGeometry`、`EngineStrength`、`EngineMap`、`EngineRuntime`、`Turbomachinery`；
`api` 的 `ElectricMotor`、`PropellerModels`；客戶端 `PartRenderer`、`PartMesh`；
0.21：`api.engine` 的 `Turbofan`、`FreeTurbine`（套件內 `GasPath`、`TwoSpool`），`Turbomachinery` 的風扇、渦輪速比效率、減速齒輪關係；
0.22：`api.aero` 的 `Airfoil`、`AirfoilAnalysis`、`SectionPolar`、`Wing`、`WingStructure`、`AirPropeller`、`HelicopterRotor`；
`api.vehicle.part` 的 `WingPart`、`AirPropellerPart`、`RotorPart`；`Connection.Shaft.freewheel`；
0.23：`api.body` 的 `SectionShape`、`BodyModule`、`BodyShape`、`BodyAerodynamics`、`BodyStructure`；`api.vehicle.part.BodyPart`；
`api.vehicle.TowingTank`；`Part.mountPoints`、`Mount.on`／`attachPart`／`attachPoint`；`PlaningHull.of`（斷階）；
0.22.1：`CoupledBody.physicsFailed`、`PartVehicle.fault`／`resume`、`BodyWorld.owningTests`／`EXPECTED_FAULT_TAG`，GameTest `VehicleGameTests`；
0.24：`api.vehicle` 的 `Drivetrain`、`PartAfter`、`TestTrack`，`Part.links`／`after`／`shownValues`／`port`，`Connection.Shaft` 的埠；`api.ground`、`api.wheel`、`api.drive`；
`api.vehicle.part` 的 `AxlePart`、`LegPart`、`TransmissionPart`、`TransferPart`；`RigidBodyDynamics.LineDamper`；遊戲端 `WorldGround`、`PartVehicle.shown`。
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
  自動截圖會在使用者的桌面跳出遊戲視窗，使用者可能正在用電腦，跑之前先說明（0.33.1 起視窗開在第 2 個螢幕、不取焦點，
  但使用者可能正在主螢幕玩全螢幕遊戲：仍先告知、取得同意）。
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
- 0.20 計算 API 與零件分工：
  - AeroCore 沒有零件設計與範例；`PartTypes` 沒有內建類型。不要再把零件設計、控制規則、質量估計或遊戲平衡加回 AeroCore，放 hdfv。
  - 從設計抽出的計算（`PistonGeometry`、`Turbomachinery`…）與執行時零件（`api.vehicle.part`）的結果必須和 0.19 的設計一致：
    抽出時以 hdfv 的 `EngineDesignTest`／`JetEngineTest` 與 AeroCore 的 `VehicleTest`（1e-9）確認。改這些時兩邊的測試都要跑。
  - `VehicleTest`／`BlueprintTest` 用測試原始碼的 `TestParts`；新的執行時零件要在那裡有夾具與對照測試。
  - 海浪、尾浪、白沫、水花的計算與渲染留在 AeroCore（使用者 2026-10-11）；零件只以 `Part.wake()` 說明自己的尾浪形式。
  - 零件外觀用客戶端 `PartRenderer` 登記（零件或設計類別）；AeroCore 只登記自己的執行時零件。
    `PartMesh` 是共用的網格工具（圓柱／圓錐、旋轉體、沿路徑的管子、擠出、面片、六面體、網格合併、平面切開、逐角法線與顏色）；
    hdfv 的引擎模型與設計畫面的 3D 預覽用它，新的通用形狀加在這裡，零件專屬的造型留在 hdfv。
    `Quad` 的四角繞向決定正面：`lathe`、`tube`、`extrude`、`hexahedron` 都朝外；世界中的載具用會剔除背面的 `entitySolid`，新形狀要保持朝外。
  - API 改動後 `.\dev.ps1 build publish`，hdfv 才拿得到；`gradle.properties` 的版本與 hdfv 的 `aerocore_version` 要一起改。
- 0.29 零件的繪製（詳見 docs/RENDERING.md）：
  - `PartMesh` 是 `MeshBuilder<PartMesh>`：0.28 的形狀幾何要保持（`MeshCompatTest` 對凍結副本 1e-12）；新形狀要給 UV、四邊形朝外。
  - 繪製只經 `PartLayers` 送出、最後 `flush` 一次：每種材質一個 render type，逐零件 `getBuffer` 會讓 buffer source 不停結束批次。
  - 零件網格在背景執行緒建立與烘焙（`MeshBakery`）：`PartRenderer.mesh` 不能碰世界或 GL。
  - 環境光遮蔽：射線起點在表面外 2 個體素（斜面的體素離自己到 1.73 個）、角的快取鍵是量化座標本身（不要雜湊成一個 long）、
    長面要自適應細分、「被覆蓋」用精確幾何（不要用體素：大零件的體素有幾公分）。改這些時跑 `AmbientOcclusionTest`，
    並用 hdfv 的 `hdfvdev export` 加 Blender 看一具大引擎、一具小引擎（0.29 開發時這四點各自讓引擎變黑、出現條紋或破洞）。
  - 零件著色器：光澤在貼圖 alpha（0.5～1），金屬度是每個材質的 uniform；設計畫面靠正交投影判斷（攝影棚、畫面 y 朝下）。
    AUTO 在 Iris 光影包使用中時改原版；著色器載入失敗只記一行錯誤。
  - 世界中的渲染器用 `PartLayers.gpu(true)`（`PartGpu`：烘焙好的不透明群組上傳一次，之後只設 `PartPose`／`PartNormalMat`／`PartLight` 再畫）。
    這三個 uniform 在零件著色器裡，逐頂點寫入時要是單位矩陣與 -1；改著色器時兩條路徑都要看（`/aerocoredev parts gpu on|off` 的畫面要一樣）。
  - 替身（`MeshBuilder.STAND_IN`）只在兩層以上的最遠層、且那層只有它；遮蔽不能把它算進去（0.29 開發時替身包住整具引擎，所有面都被判封閉、整具消失）。
  - 開發用：`/aerocoredev parts occlusion|culling|gpu on|off`、`/aerocoredev parts shading auto|shader|vanilla`、記錄行 `Part mesh baked`、
    `AutoShot` 的 `fps` 會印 `PartStats`（每幀面數、render type 數、GPU 繪製次數、輸出毫秒、烘焙）。
- 0.30 開口、內裝、可動件、燈（詳見 docs/RENDERING.md、WHEELS.md、WING.md）：
  - 玻璃標 `CLEAR`（或半透明）：不進遮蔽體素、不算覆蓋；窗後的內裝標 `CABIN`：第 0 層不判封閉、第 1 層起拿掉（玻璃那時改成不透明）。
    沒標的話，內裝在世界中被判封閉而剔掉，或被玻璃遮黑。
  - 有遠處版本（`levels(1, 3)` 的沒開孔外殼）時遠近各算一次遮蔽：一起算的話兩層外殼互判覆蓋、全部剔掉。
  - 會動的件用 `piece(n)` 烘在零件的網格裡，`animate(..., Drawn)` 畫 `drawn.piece(n)`；不要每幀現建網格（`layers.legacy()` 與
    `layers.draw(MeshBuilder ...)` 每幀寫頂點、沒有遮蔽、不進 GPU 快取）。小於 96 面的群組不進 GPU 快取（繪製次數）。
  - `SurfaceCut` 的洞是凸多邊形的聯集，依給的順序由內往外切；新點用格子的雙線性面（不是光滑曲面），密合靠這點。
  - 燈的亮度是載具 Mod 的規則（`PartRenderer.lamps`）；AeroCore 只畫。燈暈、光束、光斑與玻璃在 `PartLate`（半透明方塊之後）畫。
  - 顯示值只往後加、在產生處用 `PartStep.stepped` 量化（¾ 級的遲滯）：停著的載具不送封包。包住別的零件的零件要轉送 `shownValues()`，
    被包的零件用 `shownCount()` 判斷能不能寫。
  - 可收放起落架的內部狀態只加在可收放的（`LegPart.Retraction`）；固定的輸入、內部狀態、受力都和 0.29 一樣（舊存檔照讀、`VehicleTest` 1e-9）。
  - Fabulous 畫質時，`AFTER_TRANSLUCENT_BLOCKS` 觸發時綁的是半透明目標（NeoForge 在 `clearRenderState` 之前發事件），合成時依 alpha 疊上去
    （alpha 0 的像素整個略過）。在這個階段畫的加色光（`LampRenderer`）頂點 alpha 要是 1/255：寫 255 會變成不透明的黑方塊。
    一般半透明（玻璃、胎痕、水花）照原版的 `TRANSLUCENT_TRANSPARENCY` 就對。
- 0.30 效能（2026-10-17 使用者「你先優化」；結果逐位元不變，`VehicleTest` 1e-9）：
  - `ContactSolver.generate` 用 `BoxIndex`（1 m 格子，依清單的身分在每個執行緒記最近 8 份）查附近的方塊；查到的索引照清單順序，
    接觸的順序和逐一掃過一樣。不要改成無序（接觸順序會改變求解結果）。
  - `generatePair` 先用 B 的包圍球跳過碰不到的 A 球；`MovingBody` 記住反向姿態與角速度（衝量後才重算）；`HullSection` 記住邊長。
  - `WaveRadiation` 跳過全為 0 的核；`VehicleDynamics` 沒有零件浸水時不算輻射記憶。`PartVehicle.wakeDesign` 每個零件只寫一次 JSON。
- 0.31 執行時材質與換外觀（詳見 docs/RENDERING.md、API.md）：
  - `PartFinishes.define` 的材質在資源重載後仍在；烘焙的網格只記 id，所以換圖不重新烘焙。拿掉的材質畫成白色：要隱藏就換透明貼圖。
  - 零件著色器把貼圖 alpha 當光澤與鏤空門檻；貼紙的圖要轉換（透明 0、不透明 128＋127×光澤），材質設 `cutout`。
  - `VehicleState.restyled` 要逐位元接過所有狀態（`RestyleTest`）：VehicleState 加新欄位時，`restyled` 也要複製它。
- 0.32 搬車：`VehicleState.placed` 也要接過車上的東西（`PlaceTest`）：VehicleState 加新欄位時，想一下搬車時要不要保留它。
  組裝室推出載具時用 `place`，不要用 `setBlueprint`（hdfv 0.13 以前推出時把清空的油箱又加滿）。
- 0.40 冶金（詳見 docs/METALLURGY.md）：同 0.35，只放公式與有出處的典型值（槽電壓、電流效率、出爐熱、電效率、礦物比例）；
  哪台機器、一批多少、配方的電量由 hdfv 決定（hdfv 的產生器呼叫這些算好寫進配方，測試核對）。新的物種要有生成焓與莫耳質量的出處；
  `Reaction` 建立時檢查質量守恆，原子由呼叫者配平。改典型值時同時改 METALLURGY.md 的表與 `MetallurgyTest` 的實廠範圍。
- 0.35 機加工的計算（詳見 docs/MANUFACTURE.md）：
  - 和 0.25 一樣只放公式：輸入都是數字，哪台機器用多少速度、進給、係數是 hdfv 的規則。擬合或典型的係數（Preston 的 K_p、磨削比、電極損耗比、
    熱切割效率、水刀的比侵蝕能、空分的第二定律效率）由呼叫者給，不要在 AeroCore 寫死成某台機器的值。
  - 書上的常數（Taylor 表 23.2、Weller 的 664 與 1.23、FeO 的生成熱、乾空氣成分）在類別裡有出處；改它們要同時改 MANUFACTURE.md 與測試的註解。
- 0.34 改裝與空氣套件（詳見 docs/API.md、BODY.md）：
  - `refitted` 和 `separate`、`placed`、`restyled` 一樣要在 `VehicleState` 加欄位時一起改；零件狀態的帶過共用 `carryParts`。
  - `RoadAero` 的車底負載是「套件的車底減掉同尺寸的一般車底」：沒有擴散器、擋風板、分流板、側裙的套件必須剛好是零（`aPlainFloorAddsNothing`）。
  - 0.35.1 的車底校準（詳見 BODY.md）：一般車底也帶同樣的車輪，「沒有套件就是零」才成立；改 `K_WHEELS`、`WHEEL_RECOVERY`、`DAM_WAKE`、`SMOOTH`、
    `ROUGH_RECOVERY` 或側裙的漏氣面積時，跑 `RoadAeroTest` 全部（GT4 155 N、AMG 392 N 的 ±30 %、車輪拿掉 35～65 %、範例套件 1～2 kN）
    與 hdfv 的 `BodyKitTest`，並更新 BODY.md 的驗證表。不要再把擋風板的分離區疊加在車底吸力上（0.34 同一段算兩次），
    也不要把 `sealing` 改回線性內插（封一半就得一半的增益，6 cm 側裙讓車底下壓力加倍）。
  - 尾翼查表（30 m/s 建、`warmUp` 先建）；不要改成每步解升力線。
- 0.36 鉸接（詳見 docs/COUPLING.md）：
  - 舊的 `solveAll`／`separateAll` 是新多載以空接頭清單呼叫：改求解迴圈時，接頭的程式要在「沒有接頭就不做任何浮點運算」的位置，
    `CouplingTest.withoutJointsTheSolveAndTheSeparationAreAsBefore` 比對逐位元相同。
  - 點約束要用 3×3 區塊一次解（不要拆成三個獨立的列）：輕牽引車拖重拖車時三列互相耦合，逐列解會變軟；`JOINT_SLOP` 0.2 mm（1 mm 時間隙停在 1 mm）。
  - 連結不放進 `VehicleState`：在實體（`PartVehicle` 的 `Coupling`／`Towing` 標籤）與 `BodyWorld` 的分組；`separate`、`placed`、`restyled`、`refitted` 不用管連結。
  - 連著的載具 `stepsWhereLoaded` 為真，整組至少一台在實體會 tick 的區塊才走，且每走一次替自己的區塊續 2 秒的路徑票；不要改成各自判斷（會在區塊邊界被拆開）。
  - 客戶端的後車畫在前車畫出的掛點（`drawnShift`），`PartVehicleRenderer` 與 `partPoint` 已用；新的畫法若自己算位置，也要加上它。
- 0.39 美化著色（詳見 docs/RENDERING.md〈美化著色（0.39）〉）：
  - 新的著色都包在 `PartGlobal.x > 0.5` 裡、舊敘述一字不動，關掉時與 0.38 逐像素相同（只有會動的東西不同）；新的東西加在開關後面，並用 `ShotDiff` 比對關掉的截圖。
  - 法線貼圖的凹陷（藍色）乘在底色上：深度要依材質的起伏（hdfv 0.22 初版每種都 0.3，車漆看起來像布）。
  - `MeshBaker` 對所有件「建好時的位置」一起算遮蔽與剔除：載具 Mod 的可動件要建在靜止位置（hdfv 的控制面以鉸鏈為原點時被主翼蓋住剔掉）。
  - `PartGpu` 不要再用 `VertexBuffer.drawWithShader` 畫每個工作：它每次重套整個著色器（每個 sampler 查位置），五台載具的車輪件每幀多 5 ms。
- 0.38 機場的燈與空氣資料（詳見 docs/RENDERING.md、docs/ATMOSPHERE.md）：
  - `FieldLamps` 的燈由載具 Mod 交（伺服器同步的清單），不要改成方塊實體渲染器畫：視距外沒有區塊、也沒有方塊實體。燈的四邊形 alpha 是 1/255
    （Fabulous 的加色混合），所以要用自己的著色器：原版 position_tex_color 丟掉 alpha < 0.1 的像素（0.38 開發時燈一盞都看不見）。
  - 燈的亮度是顯示規則（Allard 門檻上的對數），不是光度計；閃燈用 `LightSignal.peak` 看兩幀之間，不要改回看當下（短閃會被幀率吃掉）。
  - `WorldAir.at` 必須和 `PartVehicle` 的空氣負載用同一個來源（有自己的風的海域先用它）；改其中一邊時兩邊一起改。氣壓與溫度是 ISA（世界沒有天氣氣壓）。
  - `VehicleTracks.near` 走 `BodyWorld`，不掃全部實體；`groundBelow` 從上方往下掃（GameTest 區有屏障天花板，直接取高度圖會拿到天花板）。
    它的 GameTest 要自己的批次：放在預設批次時同批的 `hullFloatsLevelAtItsDesignWaterline` 失敗。
  - 材質的選用鍵（`shading`、`normal`、`glass`、`farGlass`）不寫就和 0.37 逐像素相同；改零件著色器的預設路徑前，用前後截圖逐像素比對。
- 0.37 聲音（詳見 docs/ACOUSTICS.md）：
  - `api.acoustics` 只放物理與計算；哪個零件發什麼聲、何時、分配與設定在載具 Mod。聲源的參數放在 `volatile` 的狀態快照（遊戲執行緒設、
    合成執行緒讀），`render` 不配置記憶體、不丟例外。新聲源要有 `soundPowerDb` 與種子決定的雜訊（同種子同樣本）。
  - 串流的規則（原版的限制見 ACOUSTICS.md）：`read()` 只複製環形緩衝、不在裡面合成（`SoundEngine.play` 在主執行緒等聲音執行緒）、不丟例外、
    不回 null；補塊由 `SynthThread` 每 10 ms 交給聲音執行緒。Windows 上不要改回 `LockSupport.parkNanos`（解析度 15.6 ms，第一輪約 40 % 的塊放空）。
  - 開串流前留 `LiveSound.reservedStreams`（2）條給音樂與唱片機；`createHandle` 沒聲道時靜靜回 null，`LiveSound` 會在下一個 tick 再試。
  - OpenAL 呼叫只在聲音執行緒（`SoundHooks.executor()`），EFX 每批結束清 `alGetError`；只碰 `SynthInstance`，原版的聲音不動；
    裝了 Sound Physics Remastered 時 `Efx.available()` 為 false。
  - 執行時的靜態聲音 id 定義後不換內容（OpenAL 可能還在播），內容不同就換 id；`undefine` 只丟 Java 的那份。
  - 接觸事件只讀 `ContactSolver` 的結果：不要在求解迴圈裡為聲音做任何浮點運算（`VehicleTest` 1e-9）。DISPLAY 同步的 `v`、`k` 沒有時不寫。
  - `FarSounds` 的簽名在伺服器每 5 tick 對每台載具算一次：提供者只能算等級（不要在伺服器合成或排事件；hdfv 用 `VehicleSound.updateEngines`）。
  - 音爆（`SourceTrack.foldArrives`）看到達時間 T(t) = t + |聽者 − 聲源(t)| / c 的折點落在上一次與這一次 `hear` 之間，不要改回「到達數從 0 跳到 ≥1」
    或只看那一刻有沒有錐內的聲音：2026-10-19 hdfv 多人驗證時，從 64 m 外以 600 m/s 發射的車，錐內那段聲音只有 8 ms、夾在兩個 tick 之間，
    它靜止時的聲音也還在路上，兩個聽者都沒有音爆（`PropagationTest.aSourceLaunchedPastTheListenerBoomsWhileItsStandingSoundIsStillArriving`）。
    一段快於 `JUMP_MPS`（3000 m/s）的是被搬走的載具，不算。客戶端的位置會抖（伺服器慢時客戶端先推過頭再被拉回，拉向聽者的那一步比音速快，
    就是一個假的折點）：載具 Mod 餵給軌跡的點要自己處理，hdfv 的 `VehicleSounds.Entry.trackPoint` 用同步速度推、再往畫的位置拉。
  - 客戶端的快速載具（0.28）用同步的真實速度（`velocity()`，DISPLAY 的 `v`）判斷與往前推，不要改回 `getDeltaMovement()`：原版運動封包每軸截在
    3.9 格／tick，沿一軸飛的快速載具在客戶端從不算快、落後兩 tick 滑行。收到伺服器的第一個位置前從進來的位置推，`targetTick` 是 `tickCount - 1`
    （等級在 `tick()` 前先數這一 tick；寫成 `tickCount` 時進入追蹤的第一步是兩個 tick 的路，hdfv 多人驗證時聽者因此聽到 Mach 3.4 的假音爆）。
- 0.37 關閉世界時的區塊卸載（雙端 mixin `ChunkUnloadMixin`；2026-10-19 hdfv 0.19 第一次 AutoShot 在瞬移、騎水上飛機之後 `quit`，
  伺服器停在 `Saving worlds` 20 分鐘、伺服器執行緒 100 % CPU，經使用者同意強制結束）：
  - 原因在原版 1.21.1：`ChunkMap.scheduleUnload` 的工作在持有者沒準備好存檔（`isReadyForSaving`：生成參照為 0 且存檔 future 完成）時再呼叫
    `scheduleUnload`，存檔 future 已完成就立刻排回 `unloadQueue`。持有者若已被票拉回 `updatingChunkMap`（`updateChunkScheduling` 從
    `pendingUnloads` 拿回同一個），旁邊的生成又握著它，這個過時的工作每次跑都重排：平常只吃掉 tick 剩下的時間，關閉時
    `processUnloads(() -> true)` 永遠出不了迴圈，能放開參照的生成又要伺服器執行緒才能完成，活鎖、世界沒存完。
    瞬移與快速載具的路徑票（0.28 的 keepAhead）讓「剛要卸載又被拉回、旁邊在生成」更常見。
  - mixin 在 `scheduleUnload` 開頭：`pendingUnloads.get(pos) != holder` 就不排。只有沒準備好時的重排會帶著不在 `pendingUnloads` 的持有者走到這裡
    （`processUnloads` 是先放進去才呼叫）。和原版等價：過時的工作等到準備好，也只會在 `pendingUnloads.remove(pos, holder)` 失敗而什麼都不做；
    持有者只經 `processUnloads` 進 `pendingUnloads`，那時會排新的工作；在 `pendingUnloads` 裡的持有者拿不到生成參照（`acquireGeneration` 讀
    `updatingChunkMap`），所以不會再有自己重排不停的工作。不要拿掉；也不要改成限制迴圈次數或跳出迴圈（會改變卸載與存檔的時機）。
  - GameTest `probe/ChunkUnloadGameTests.staleUnloadTaskDoesNotRequeueForever`（`@GameTestHolder` 自動登記，不在 `AeroCore.java`；讀寫用
    `ChunkMapAccessor`）在一個 tick 內重演：1 km 外的區塊放票、拿掉票讓它進 `pendingUnloads` 並排好卸載工作、再放票拉回同一個持有者、
    等它重新變成完整區塊、加一個生成參照，然後像關閉時那樣清佇列（輪詢上限 10 萬次，所以退步時是測試失敗而不是卡住）；
    最後拿掉票確認區塊仍會正常卸載。2026-10-19 在原開發機：只登記 accessor、不登記這個 mixin 時這個測試失敗
    （`1 still queued after 100001 polls`），其餘 54 個照過；登記後 55 個全過，關閉存檔 1 秒內完成。
  - AutoShot 計畫不必再為此在 `quit` 前多等。沒有在客戶端重現原本的瞬移＋水上飛機情境（機率性），只以上面的 GameTest 驗證。
- 0.33 車架、硬點、氣壓（詳見 docs/FRAME.md、WHEELS.md）：
  - 新功能全部在「有才走」的分支後面：沒有車架不探測、沒有表時 `WheelCorner` 走原本的式子、空氣倍數是 1 時呼叫原本的 `Suspension.step`。
    改這幾處時保持舊路徑逐位元不變（`VehicleTest` 1e-9），新路徑用自己的測試（`FrameFlexTest`、`KinematicsTest`、`AirSpringTest`）。
  - 車架的扭角放在會扭的零件自己的前 N 個內部狀態（`VehicleDynamics` 寫入），所以 `restyled`、`placed` 不用改；不要另加 `VehicleState` 欄位。
  - 探測（`probe`）時不能寫負載；射線結果用 `PartStep.keep` 留給同一步的 `loads`，不要投兩次。新的車輪零件要受車架影響，就實作 `probes()`／`probe()`；
    `LegPart` 沒有（飛機沒有會扭的機身）。
  - 扭轉的剛度矩陣約定：站點依 x、T = K θ、對稱、每列和為零（`api.frame.FrameAnalysis.twistStiffness` 與 `BodyStructure.torsion` 都這樣給）。
    `api.frame.Frame` 是設計時的梁模型，`api.vehicle.FrameTwist` 是交給物理的矩陣，名字不要再混。
  - 驗證扭轉的公式要含重力翻轉力矩：只用串聯彈簧估計時剛體的對角載重少 18%（`aWarpIsSharedLikeSeriesSprings` 的說明）。
  - 硬點：拉桿內端由 `fromSimple` 沿到瞬心的線移到上下 50 mm 束角相同；連桿在行程內卡死是 `LinkageLockException`，載具 Mod 要當成 INVALID。
  - 調高閥是比例閥（超過死區後開度隨誤差增加，4 倍死區全開）：全開到死區邊緣的閥會衝過頭、來回充放（開發時遇到）。
  - 梁模型的壕溝是一輪下的地面下降 `DITCH_DROP_M`（0.15 m，輪子懸空時再解一次），不是拿掉那一輪；開口截面兩端受限時扭矩分成
    St Venant 與翹曲兩部分，應力加上翹曲正應力。最初的寫法（拿掉一輪、整個扭矩算 St Venant 剪應力）讓 hdfv 的三軸卡車槽鋼大樑使用率到 14 倍。
- 0.28 衝壓、超燃、脈衝噴射（詳見 docs/RAMJET.md、SCRAMJET.md、PULSEJET.md）：
  - 乾推的 `Turbojet`／`Turbofan` 數字不能變：後燃器只在點燃時改噴嘴。`TurbojetTest`、`TurbofanTest` 要全過。
  - 衝壓、超燃、脈衝噴射的工作圖在背景建（`Ramjet` 的進氣表、`Scramjet.Map` 約 0.3 s、`Pulsejet.Map` 約 3.4 s）；遊戲 tick 只查表。
  - 脈衝噴射的一維模型只有在「充量低於進氣總壓時不燃燒」這個條件下才會自持共振。開發時試過的時間延遲模型、Livengood-Wu 自燃積分、
    完全攪拌：都只有弱振盪或火焰錨在閥上。不要拿掉這個條件。輸入是燃料流量：無閥引擎用吸氣量回授定油會一開始就斷油。
  - Argus 的簧片開口面積沒有公開（取 0.04 m²）；推力大致與它成正比。改燃燒常數或閥模型時，跑 `PulsejetTest`，並更新 PULSEJET.md 的表。
- 0.28 極高速載具（詳見 docs/HIGH_SPEED.md）：
  - 低於 `PartVehicle.FAST_MPS`（80 m/s）照舊；以上用碰撞走廊（逐子步掃過的包絡盒＋高度圖剔除）、路徑區塊票（半徑 1，往前 1 秒、最多 48 個）。
    區塊沒好就暫停，速度保留。
  - 路徑票只到「已載入」層級：快速載具由 `stepsWhereLoaded` 在不跑實體 tick 的區塊推進。開發時用半徑 2、往前 1.5 秒的票，
    GameTest 伺服器記憶體不足（一般地形的世界），不要加大。
  - 讀方塊前檢查區塊已載入：讀未載入的區塊會在伺服器執行緒上同步產生它。
  - 機體的超音速壓力阻力（切錐、修正牛頓、船尾用切錐鏡像、底壓）在 Mach 1.2 起取代次音速的形狀阻力；次音速的數字不能變。
  - 氣動加熱只給溫度（`noseTemperatureK`、`skinTemperatureK`），沒有破壞。
- 0.28 拋棄零件：
  - `VehicleState.separate` 依零件 id 轉移狀態；剩下部分的運動是舊剛體在新重心的速度、同樣的角速度。
  - 改這部分時跑 `JettisonTest` 與 GameTest `aPartAskingToGoIsCastOff`。
- 0.19 噴射引擎（詳見 docs/JET.md；元件與控制規則 0.20 起在 hdfv）：
  - 性能只來自 `Turbojet` 的匹配（設計點 → 平衡 → 運轉線 → 準穩態）；元件只給參數與檢查，不要在零件裡另算推力。
    準穩態在運轉線油量下必須就是平衡解：改平衡、運轉線或準穩態任何一個，都用 `TurbojetTest.quasiSteadyBalances...`
    與兩具範例的循環在靜止、100 m/s、11 km 確認運轉線油量的功率差在 1 % 內，全油門推力等於平衡解。
  - 渦輪是跨越自身壓比的噴嘴（不是恆阻塞）：低壓比的模型引擎在設計點就沒有阻塞，改回恆阻塞會讓它在 935 K 以下沒有平衡解。
  - 平衡的匹配式有退化根（轉子幾乎不轉、流量很大）：從大渦輪壓比往下掃描再二分的做法要保留。進氣損失直接在平衡式內由修正流量求
    （與 pt2 無關）；不要改回外圈迭代（會在兩根之間跳，運轉線列的 pt2 與流量不一致）。
  - 壓損（進氣道、燃燒室、尾管）隨修正流量平方縮放；固定比例的壓損會讓低轉速的低壓比引擎渦輪拿不到壓力，起不來。
  - 運轉線依飛行馬赫數各一條、依衝壓總壓比內插、T4 範圍換算到最冷標準大氣；只用靜止的一條時，噴嘴未阻塞的引擎在飛行中推力差 20 % 以上，
    高空寒冷時修正轉速超出表格。
  - 燃油控制、起動機、喘振規則與質量係數的約定移到 hdfv 的 AGENTS.md。
- 0.23 模組化機體（詳見 docs/BODY.md）：
  - 截面族都輸出凸多邊形（`SectionShape.convex`），模組之間用支撐函數做 Minkowski 混合；加新截面族也要輸出凸的，`HullSection` 與開口的切割都假設凸。
    `convex` 先把點對齊到 1 nm 並去掉重複：混合時每個角有十幾個只差最後一位的副本，同一高度兩個角的副本在排序裡交錯時，
    單調鏈把真的角當成共線刪掉（0.23 開發時浮筒船艏每隔一站少一個舷緣角，3D 模型成鋸齒）。
  - 空氣阻力的校正：鈍尾 k = 0.042（和 Ahmed 後斜一起擬合）、橫流起始因子 ((α − 4°)/26°)^0.8（彈形圓柱對 Jorgensen）；
    改任何一個跑 `BodyAerodynamicsTest`（Hoerner ±15%、Jorgensen、Ahmed ±10%、Munk ±20%）並更新 BODY.md 的驗證表。
  - 額外阻力面積沿氣流、用全部動壓（不沿機身）：沿機身時阻力大的機身在正攻角得到向下的升力。
  - 2D+t（`PlaningDynamics`）：法向在離水的截面不給吸力（Zarnick 的 dm_a/dt = 0），側向 m_22 從船艏往後保持最大值（Jones）。
    兩者拿掉任何一個：水上飛機在水面打轉（側向），重心偏前的快艇栽頭（法向）。法向不能也用「保持最大值」：斷階後的後體要自己重新滑行
    （Sajedi 的斷階船體縱傾會偏高 2°）。改這部分跑 `PlaningDynamicsTest`（Fridsma、Savitsky 對照的容許不能放寬）與 hdfv 的 `AirframeTest`、`BodyTest`。
  - 船底斜升係數（2026-10-12，詳見 PLANING.md）：每個滑行面（斷階分開，斷階還濕時合為一個，並隨第一個斷階的通氣淡入）的法向負載——靜水壓、動力項、附加質量、
    法向繞射——一起乘 k = 1 − sinβ + 0.13 L_C/B（0.3～1），隨通氣淡入。原因是 2D+t 的升力不隨斜升角變（舭濕後就是平板的動量），
    又缺濕舭長度的效應，兩者只在 Fridsma 的長濕長抵消。
    - 要縮放整個法向條帶力：只縮放準穩態部分（靜水壓＋前進速度的動量）時 Fridsma LCG 70% 不再縱搖不穩；縮放力而不縮放附加質量時處處縱搖不穩；
      只縮放動力項、不縮放靜水壓時 C_V 2 縱傾低 1°。
    - 不要用艉板附近的釋壓（Garme 長度或更長）乘在動力項上修縱傾：升力只改 2～10%，而且縱搖不穩消失。
    - L_C 用 Wagner 半寬到達舭的位置在站位間內插（舭在艉板還乾時為負）；用 Savitsky 的稜柱關係由龍骨濕長推時，彎曲船艏的快艇在 18 m/s 縱搖不穩。
    - 改係數或這段程式時跑 `hullsPlaneAsSavitskyPredicts`（縱傾 ±0.7°、阻力 ±15%，對照 Savitsky 1964 加 2007 鬚狀水花）、Fridsma、Sajedi，
      並用 PLANING.md 的 10°／30° 掃描看其他斜升角。
  - 鬚狀水花的水層在舭乾時只到艉板（`sprayWedge`）；不要改回「舭沒濕就不算」：阻力在那裡會掉五分之一，還會隨姿態跳動。
  - 斷階尾流的中線高度從斷階龍骨的水平線量起（不是沿著有縱傾的龍骨線）；沿龍骨線量時 Sajedi 的船體縱傾 9～15°。
  - 結構的支點：只有一個支點時加遠端支點，反力用最小平方同時滿足力與力矩；船用容積加權的浮力支點。單一支點不平衡的力矩曾讓機身彎曲應力算成 634 MPa。
  - `BodyPart` 排水型的設計水線不超過密封容積一半的水；會沉的機體（全部開放、太重）要建得出來，GameTest `aBodyFloatsOnItsSealedVolume` 檢查。
  - `TowingTank` 是給試算畫面的離線計算（每趟 10 秒、橫傾每角 6 秒，每秒數千個子步）；不要在遊戲 tick 裡呼叫。
  - 網格的四邊形法線用對角線的外積 (c − a) × (d − b)；兩個角法線相加的寫法在平的平行四邊形上抵消成零（0.23 前 `PartMesh.sheet` 的光照與 hdfv 的機體、機翼的剔除方向因此錯）。
- 0.24 傳動（詳見 docs/DRIVETRAIN.md）：
  - 每一步會變的連接（檔位、離合器、煞車、差速器、鎖）是零件在 `loads` 裡寫的約束列，和軸系一起由 `Drivetrain` 解；不要改成在零件裡
    自己算接合後的轉速（多個零件的列要同時滿足，且反扭矩要從列的扭矩來）。
  - 沒有約束列與交叉斜率的一步必須走原本的程式：`VehicleTest`（1e-9）與 `DrivetrainTest.theSolverRepeatsTheTrainsAlone...` 一起過。
  - D 只在奇異時加正則化（重複的鎖）；一律加的話，煞住的輪子每步會有 1e-11 rad/s 的殘餘轉速。
  - 依另一列扭矩的限滑容量要在互補問題裡當線性關係解；用外圈迭代時托森在四次內只到 TBR 的 96%。
  - 單向離合器軸在求解器路徑上的方向取驅動側單獨推進後的轉向（和舊程式一樣，靜止起步那一步就接合）。
  - 滾動力（輪胎）先在軸系解、再以步末轉速加在車身；車身側的斜率用 `LineDamper`。兩者分開解的暫態誤差在 DRIVETRAIN.md，不要只改一邊。
  - 換檔中，升檔的退出元件與降檔的進入元件只傳驅動扭矩（`TransmissionPart.elementRow`）；兩個都雙向時，進入元件把引擎拉慢、
    退出元件仍鎖著，會把車往後拉（0.24 開發時雙離合換檔中車速掉 2.4 rad/s）。
  - 何時換檔、起步滑差、CVT 目標齒比、鎖止、ABS 是 hdfv 的規則；AeroCore 的零件只做機構（同步器、換檔斜坡、交接、齒比變化率、P 檔棘爪）。
- 0.24 輪子（詳見 docs/WHEELS.md）：
  - 輪胎性質只由 `Tyre` 的關係算（尺寸、胎壓、構造、胎紋）；改係數或關係時跑 `TyreTest`（對公開資料）與 `WheelTest`，並更新 WHEELS.md 的表。
  - 法向力沿地面法線、作用在接地點，大小是懸吊力除以行程與法線的夾角餘弦；不要改回沿車身垂直方向施加
    （車身側傾時懸吊力的水平分量是假的側力，0.24 開發時彎中的轉向不足多了 20%）。
  - 輪胎平面內的線阻尼以「接地點速度隨車身轉過一步」為參考；不要改回步初的世界方向（穩定轉彎時那是向心加速度，不是滑移）。
  - 台階邊的二分法（`WheelCorner.EDGE`）讓爬路緣的抬升連續；拿掉時每過一條射線抬升跳一次，力的尖峰會把車彈回去。
  - 暫態變形限制在合成滑移 1.2（`TyreContact.MAX_SLIP`），低速阻尼和彈簧一起不超過摩擦；兩者讓停車與原地打滑都穩定。
  - 新載具的懸吊從設計壓縮量開始（放在地上就接近平衡）；在空中生成的起落架會在落地前伸出，測試落震時要從全伸開始。
  - `TestTrack` 與 `TowingTank` 一樣只給試算與測試，不要在遊戲 tick 裡用。
- 0.22 翼型、機翼、螺旋槳、旋翼（詳見 docs/WING.md）：
  - 翼型分析不做黏性—無黏耦合（試過的吹氣耦合在尾緣發散）；升力斜率靠 `viscousFactor`，失速靠尾緣分離（Head ×2）與前緣壓差準則取較前者。
    改 `SEPARATION_H`、`TRAILING_EDGE_SCALE`、`PDR_*`、`SONIC_PEAK` 任何一個，都要跑 `SectionPolarTest`（Abbott 的斜率、零升角、力矩、最大升力 ±15 %）
    並更新 WING.md 的驗證表。
  - `SectionPolar` 從不丟例外（任何攻角、Re、Mach，包括 NaN），零件每個子步都查；新增的修正也要保持這點（`neverThrowsAndStaysFiniteEverywhere`）。
  - `Wing` 的後掠用簡單後掠理論（法向速度與弦長）加翼根圓滑；拿掉任何一個，後掠翼的升力斜率會偏高 16 %（0.22 開發時量到）。
  - `Wing` 的求解在牛頓步降不下殘差時改走 Levenberg–Marquardt（失速後），試算掃攻角要用帶 `previous` 的 `solve` 延續；
    改求解器時跑 `WingTest.aSweepThroughTheStallIsContinuous`（失速後阻力只升不降、升力不跳、殘差 < 0.15）。
  - 執行時效能：`Wing` 以上一步的渦強度暖啟動、容差 1e-4（`WingPart` 把渦強度存成內部狀態）；旋翼每步只做一次揮舞的準牛頓步，
    跨步收斂。不要改成每步完整收斂（每個子步會從 50 µs 變成數毫秒）。
  - 旋翼的誘導功率係數 1.15 乘在動量理論的誘導速度上（渦環經驗曲線已含此係數）；改它要重看 UH-60 級的 FM（`hoverPowerAndFigureOfMeritAreRealistic`）。
  - 單向離合器：鎖住時以兩邊帶進來的角動量開始（剛性嚙合）；陀螺力矩對所有轉子（含船的引擎與螺旋槳）都有，數值很小但不是零。
  - 陀螺力矩在 `VehicleDynamics.finish` 的漂移裡隱式處理（`RigidBodyDynamics.drift(s, p, spin, dt)`），不要改回 `prepare` 的顯式外力：
    直升機旋翼的 H／I 約 19 rad/s，顯式時每步多出能量（`aFreeGyrostatKeepsItsMomentumAndEnergy`）。
  - 旋翼在低轉速時，以葉尖速度無因次化的量（前進比、C_T）沒有意義；用它們的經驗式要有界（`DREES_MU_MAX`、`TIP_LOSS_MIN`），
    改旋翼入流時跑 `aStoppedOrCreepingRotorStaysBoundedOverSteps`。一個載具的物理丟例外會讓那台載具停用（0.22.1 以前是整個伺服器當掉），
    GameTest 也會失敗，所以界限不能省。
- 0.22.1 物理例外的隔離：
  - `BodyWorld` 對每個物體的 `beginTick`、`prepareSubstep`＋自己的靜態接觸、`finishSubstep`、`correctPose`、`endTick` 各自 try；
    出錯的物體本 tick 不再參與，交給 `CoupledBody.physicsFailed`。聯合接觸（`solveAll`／`separateAll`）出錯時先各自單獨解來找出是哪個物體。
    不要改成捕捉後繼續用出錯的狀態推進，也不要在 `BodyWorld` 外圈整個包一個 try（一台出錯會停掉全部）。
  - `PartVehicle.physicsFailed` 以本子步開始時的狀態（`lastGood`）建一個新的靜止狀態（`Vehicle.newState`：速度、角動量、軸轉速歸零，
    零件內部狀態回初始值，因為失敗的那步可能寫了一半），記錄一次 ERROR，`fault` 存檔；之後 `beginTick` 回傳 false 直到 `resume()`。
  - 平台、測試船、滑行艇、測試球用預設的 `physicsFailed`（重新丟出），行為和以前一樣；要隔離就覆寫它。
  - GameTest 中物體失敗會讓所屬測試失敗：`GameTestInfo.fail` 從外部呼叫不會通知 runner，所以用 `setRunAtTickTime(0, 丟例外)` 讓它在測試自己的 tick 丟出。
    故意出錯的測試在物體加 `BodyWorld.EXPECTED_FAULT_TAG`。0.22.1 開發時暫時拿掉 `VehicleGameTests` 的標籤，確認該測試以物理例外失敗、其他照過、伺服器不當機。
- 0.21 渦扇與自由渦輪（詳見 docs/TURBOFAN.md）：
  - `Turbojet` 的公開介面與數字不能變；共用的氣體路徑在 `GasPath`（改它時 `TurbojetTest` 全部要過）。
  - 雙轉子的匹配是 `TwoSpool` 的一組方程（牛頓法）；平衡、運轉線、準穩態都解同一組，只差哪些是未知數。
    準穩態在運轉線油量下必須是平衡解：`TurbofanTest`／`FreeTurbineTest` 的 `quasiSteadyBalances...` 要在 1 % 內（含兩條運轉線之間的飛行速度）。
  - 流量方程用平方差（`mismatch`）：低功率時噴嘴出口接近大氣壓，流量對壓力的斜率無界，改回 a − b 會讓牛頓法在慢車附近失敗。
  - 運轉線每 0.1 Mach 一條；改成較疏時渦扇低壓轉子在兩條之間的功率差會到 3 %。
  - 運轉線表的燃油空氣比欄位要存實際值（給 T4 求解時它不是未知數，曾存成設計值使暖啟動猜值錯誤）。
  - 熄火、冷轉這種離運轉線很遠的燃油，靠從運轉線 T4 出發、分步改燃油的延續法；不要拿掉。
- 0.18 活塞引擎（詳見 docs/ENGINE.md；元件、組裝檢查與執行時規則 0.20 起在 hdfv）：
  - 性能只來自 `EngineCycle`＋`Dyno`＋`Friction`（MAP 由 `EngineMap` 建立）；不要為了「好玩」直接改 MAP 的值。燃燒相關式（Bonatesta）只在一具引擎擬合，
    火焰行程與燃料縮放、爆震校準 `KNOCK_CALIBRATION = 1.45` 都是有註明的假設或單點校準，改動時要更新 ENGINE.md 的理由。
  - 循環的顯式步長在稀薄氣體會發散：孔口流量限制（每步不超過讓兩側壓力相等之量的一半）與歧管氣柱倒流限制都要保留；
    改步長或流量公式時，用 hdfv 的 `EngineDesignTest` 與 12～20 % 歧管壓力的點確認不會出現 NaN 或負質量。
- 0.17 零件化載具：
  - 零件受力必須和原本的積分器逐步一致：`FloatingBodyDynamics.sphereLoads`、`HullDynamics.loads`、`PlaningDynamics.loads` 是共用的同一份程式，
    不要在零件裡另寫一份。`VehicleTest` 以 1e-9 比對浮筒平台、測試船、滑行艇（0.20 起用 `TestParts` 夾具），改受力時要一起過。
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
  - AeroCore 不加可駕駛載具的玩法與零件（AGENTS 開頭）；範例藍圖在 hdfv。
- 0.15 `AutoShot`：只在非正式環境、遊戲目錄有 `aerocore-autoshot.txt` 時啟動（讀後即刪），只開資料夾名稱以 AeroCore 開頭的世界，最後正常存檔離開。
  0.16 加 `key`（按住移動鍵，例如划船）、`camera`、`look`、`clouds`（只在這次執行打開雲，結束還原使用者設定）；1.21.1 的船實體 ID 是 `minecraft:boat`。
  `AeroCore Render QA` 是 2026-10-10 在原開發機從使用者的「新的世界」複製的 QA 世界（複製時遊戲已關閉，只讀原世界，`level.dat` 名稱已改），
  可隨意修改；不要對「新的世界」使用 AutoShot。截圖在 `client/screenshots/aerocore-*.png`。
  0.20 加 `gui`（連同介面與開著的畫面截圖）與 `client`（以玩家身分送出指令，Mod 的客戶端指令在客戶端執行）；hdfv 用它們操作設計畫面截圖。
  AutoShot 也會在使用 AeroCore 的其他 Mod 的開發客戶端執行（遊戲目錄各自不同）。
  0.30 加 `monitor <n>`：讀到計畫時（標題畫面）就把視窗移到第 n 個螢幕置中，`size` 之後再移一次（使用者 2026-10-17「Minecraft 開在第2個螢幕」：
  自動截圖不要蓋住使用者正在用的主螢幕）。標題畫面之前的 NeoForge 載入視窗仍在主螢幕（0.33.1 解決，見下）。
  0.33.1 開發用視窗（`client.DevWindow`、mixin `DevWindowMixin`／`DevWindowFocusMixin`；使用者 2026-10-18 在主螢幕玩英雄聯盟時被開發用 Minecraft 切出來）：
  - client 執行帶 `-Daerocore.window.monitor=2`（Gradle 屬性 `dev_window_monitor`，0 照原樣開）：視窗隱藏建立、放在該螢幕工作區中央、
    以 `SW_SHOWNA` 顯示（`GLFW_FOCUS_ON_SHOW` 關），遊戲一開始就知道沒有焦點，所以世界開好時不鎖滑鼠（原版建好視窗後直接 `setWindowActive(true)`）。
  - NeoForge 的早期載入視窗（`fmlearlydisplay` 的 `DisplayWindow`）在任何 Mod 前就開在主螢幕中央並 `glfwShowWindow`（取得焦點），沒有設定可改位置或焦點；
    所以 AeroCore 與 hdfv 的 `runClient`（hdfv 也有 `runClient2`）在啟動前把執行資料夾的 `config/fml.toml` 改成 `earlyWindowControl = false`。
    不要改回 true，也不要拿掉 build.gradle 裡的這段；記錄有 `AeroCore dev window: NeoForge's early loading window opened it` 表示早期視窗又開了。
  - `AutoShot` 的 `monitor` 預設是這個螢幕；計畫執行中鎖住的滑鼠下一個 tick 就放開（有人點進視窗時）。步驟不需要焦點。
  - 只在 `FMLEnvironment.production` 為 false 時作用。沒有第 2 個螢幕的電腦開在主螢幕，但仍不取焦點。
  0.32 加 `server <位址>`（取代 `world`，只連 `localhost`／`127.0.0.1`，多人連線測試用）與 `key <按鍵名稱> on|off|tap`
  （任何按鍵設定，例如 hdfv 的換檔鍵；`tap` 按兩個 tick 並算一次點擊）。連上伺服器時 `cmd` 會停下計畫，改用 `client`；連線中斷時遊戲照 `quit` 關掉。
  0.37 加 `mark <文字>`：在記錄寫一行 `AutoShot mark: <文字>`，對照聲音記錄與錄音的時間。
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
  驗證基準：Fridsma 表 1（航行俯仰差 0.35° 內、阻力偏高 6～17%），LCG 70% 的縱搖不穩起始在 V/√L 5.5～6.2 之間（水槽 5.85）；
  2026-10-12 起加上 Savitsky（1964 加 2007 鬚狀水花）在 C_V 2～5、C_Δ 0.15～0.6 的對照（縱傾 ±0.7°、阻力 ±15%，船底斜升係數見 0.23 一節）。
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
| 0.6.0 | 2026-10-07，第二台電腦 | 103＋5 | 21 | 未實測 |
| 0.7.0 | 2026-10-08，第二台電腦 | 111＋5 | 23 | 未實測 |
| 0.8.0 | 2026-10-09，第二台電腦 | 119＋5 | 24 | 修正暫存目錄後可建立、載入、存檔世界，記錄無錯誤；功能畫面未實測 |
| 0.9.0 | 2026-10-10，原開發機 | 147＋5 | 29 | 見下方待實測清單 |
| 0.10.0 | 2026-10-10，原開發機 | 168＋5 | 36 | 只確認開發客戶端能啟動到標題畫面、客戶端 mixin 套用無錯誤；功能畫面未實測 |
| 0.11.0 | 2026-10-10，原開發機 | 189＋5 | 38 | 只確認開發客戶端能啟動到標題畫面、客戶端 mixin（含船的渲染）無錯誤；功能畫面未實測 |
| 0.12.0 | 2026-10-10，原開發機 | 202＋5 | 40 | 只確認開發客戶端能啟動到標題畫面、新的客戶端 mixin（乘客傾斜）無錯誤；功能畫面未實測 |
| 0.13.0 | 2026-10-10，原開發機 | 206＋5 | 42 | 只確認開發客戶端能啟動到標題畫面、無錯誤；功能畫面未實測 |
| 0.14.0 | 2026-10-10，原開發機 | 210＋5 | 43 | 只確認開發客戶端能啟動到標題畫面、無錯誤；水面光學著色以離線預覽圖檢查，遊戲中未實測 |
| 0.14.1 | 2026-10-10，原開發機 | 211＋5 | 43 | 未重新啟動客戶端；修正使用者實玩 0.14.0 的崩潰（新測試在未修正時失敗）與近處亮盤，遊戲中未實測 |
| 0.15.0 | 2026-10-10，原開發機 | 216＋5 | 43 | 以 `AutoShot` 在 `AeroCore Render QA` 截圖：淺水風浪區的早晨／中午／傍晚／夜晚／水下／平視／俯視外觀，FPS 83～117（1280×720，RTX 5070） |
| 0.16.0 | 2026-10-11，原開發機 | 234＋5 | 47 | `AutoShot` 截圖：划船的泡沫網紋與停船後消散、自然海域晴天與雷雨、天空正午／日出／日落／雨／雷雨、雲兩張相隔 5 秒的位移、雨的傾斜 |
| 0.18.0 | 2026-10-11，原開發機 | 280＋5 | 51 | 未啟動客戶端；引擎船以 GameTest 確認起動與推進 |
| 0.19.0 | 2026-10-11，原開發機 | 293＋5 | 52 | 未啟動客戶端；噴射艇以 GameTest 確認點火與推進；引擎船 GameTest 改成撞牆前判定（原本靠反彈才過） |
| 0.23.0 | 2026-10-12，原開發機 | 348＋5 | 48 | 機體、掛載點、斷階、拖曳水槽的 JUnit；遊戲中由 hdfv 0.5 確認：`AutoShot` 在 `AeroCore HDFV QA`（1280×720）看到水上飛機的斷階浮筒與機身、直升機的模組化機身、四邊形法線修正後的浮筒；hdfv GameTest 13 個 |
| 0.23.0（2026-10-12 重新發佈） | 2026-10-12，原開發機 | 351＋5 | 48 | 2D+t 的船底斜升係數與鬚狀水花修正：新 JUnit `hullsPlaneAsSavitskyPredicts`（對 Savitsky 縱傾 ±0.45°、阻力 −8～+2%）、`whiskerSprayIsContinuousWhereTheChinesWetAtTheTransom`、`theDeadriseFactorRecoversAlongTheWettedChines`；Fridsma、Sajedi 的容許不變；hdfv JUnit 60（快艇試算台縱傾 3.40～1.47°、水上飛機 34.8 s 離水）、GameTest 13；未啟動客戶端 |
| 0.25.0 | 2026-10-13，原開發機 | 406＋5 | 49 | `api.manufacture`（ISO 286 公差、切削、鑄造、成形、接合、複材與硫化）的 `ManufactureTest` 5 項對 ISO 表值與 Groover 例題；hdfv 0.7 的製造以它建置 |
| 0.30.0 | 2026-10-17，原開發機 | 539＋5 | 52 | 新 JUnit：`SurfaceCutTest`（面積守恆、邊界環封閉、頂點在格面上、繞向、巢狀孔、零寬格、圓角、逐格材質）、`MeshPiecesTest`（件的聯集、遮蔽相同、細長四邊形不讓烘焙失敗）、`LegRetractionTest`、`RotorAndPropellerTest`、`DownwashTest`、`ShownValuesTest`（停著的車輪與起落架顯示值不變）；`MeshCompatTest`、`VehicleTest`（1e-9）不變。客戶端由 hdfv 0.12 的 `AutoShot` 看：車窗開孔與內裝、車門、旋翼錐角、煞車燈、胎痕與揚塵、施工中的載具、機器方塊與成形多方塊、爐口與電弧的燈暈；燈的光束與地面光斑、航行燈閃爍、起落架收起的畫面未在遊戲中確認。開發中修正：Fabulous 畫質下燈暈是黑方塊（加色光的 alpha）；效能（見 0.30 效能，剖析的場景伺服器 tick 平均 36～50 → 22～26 ms，JFR 伺服器執行緒取樣減半）；細長四邊形的法線讓整個機身烘焙成空的（`MeshOps.faceNormal` 門檻 1e-24）、乘客頭穿出車頂（`PartVehicle.SEATED_HIP_M`）、停著的車輪仍送顯示值（量化與爬行門檻）、胎痕畫在實心方塊之後時位置錯（改在半透明方塊之後）。AutoShot 加 `monitor`（使用者「Minecraft 開在第2個螢幕」） |
| 0.33.0 | 2026-10-17，原開發機 | 588＋5 | 52 | 新 JUnit：`FrameFlexTest` 6（剛性車架與剛體逐位元相同、扭曲路面的載重轉移對兩站靜力平衡 3%、車架不抗扭時依兩端質量分配、K 1e5～1e7 崎嶇路 20 秒穩定、三軸、時間步長收斂）、`KinematicsTest` 6（平行等長臂外傾不變、十種型式對簡化幾何的側傾中心 ±1 mm 與外傾增益 ±5%、bump steer 50 mm 小於 0.02°、拖曳臂的圓弧、支點前傾的抗點頭、連桿卡死）、`WheelTest` 加硬點的煞車點頭（簡化轉成硬點差 10% 內、支點前傾 12° 點頭小 15% 以上）與反相後輪轉向、`AirSpringTest` 3、`ClearanceTest` 3、`api.frame` 16（同日為 hdfv 的卡車大樑補：壕溝改成地面下降、開口截面的翹曲扭矩與應力，多 3 個）（另一個工作階段）、`BodyTorsionTest` 5（另一個工作階段，範例轎車每度 33.9 kN·m）；`VehicleTest` 1e-9 不變。客戶端由 hdfv 0.15 確認 |
| 0.33.1 | 2026-10-18，原開發機 | build 通過（physics 91 個測試類別無失敗、CFD 5） | 未跑（只改開發用客戶端） | hdfv `runClient`＋`AutoShot`（`AeroCore HDFV QA`，計畫刻意不寫 `monitor`），使用者同意在主螢幕玩英雄聯盟時測：NeoForge 早期視窗沒有出現（`splash screen is disabled`），遊戲視窗隱藏建立在第 2 個螢幕、0.5 秒後顯示（記錄 `focused false`）；啟動到關閉 82 秒內前景一直是英雄聯盟、游標限制區域不變；`key forward` 在沒有焦點時讓玩家前進 18 m；兩張截圖正常；正常存檔離開。離開時記錄一次 `ClosedChannelException`（剛往前飛、伺服器還在送區塊時斷線；之前 40 份 AutoShot 記錄沒有，原因未再重現確認，存檔完整）。未確認：有人點進視窗後放開滑鼠、`runClient2`、AeroCore 自己的 `runClient`、全螢幕選項、只有一個螢幕的電腦 |
| 0.34.0 | 2026-10-18，原開發機 | 604＋5 | 52 | 新 JUnit `RefitTest` 4（同一份藍圖 refit 等於 placed 停在原地；換重的車架質量更新、速度歸零、零件狀態依 id 帶過、原本的不受影響；油箱變小截掉並回報、拿掉的油箱全倒、新油箱是空的、呼叫端自己的密度；狀態數量變了的零件重來、換順序的照帶）、`RoadAeroTest` 12（一般車底的套件剛好是零；擴散器 10° 在 0.12 m 最大、0.015 m 剩不到三成；0.12 m 時最好的斜角 7～16°；側裙封住 3.5 倍以上、1.5～1.7；擋風板 0.24、分流板更多、壓低車頭；離地淡出、倒車沒有；下壓力隨動壓；唇 0.10；車頂橫桿的阻力；尾翼等於升力線 0.1 % 內、端板多 5 %、失速、地面鏡像；氣流上仰減少下壓力）、`RoadAeroBodyTest`（機體在平地 0.12 m、30 m/s 的套件下壓力等於直接算的、沒有套件時機體負載不變）；`build publish` 給 hdfv 0.16。客戶端：hdfv 0.16 的 `AutoShot`（`AeroCore HDFV QA`）在改裝廠用 `PartVehicle.refit` 原地換車三次（套件與車燈、整台 1.3 倍、0.7 倍），車停回地板、油量保留；機體帶 `RoadAero` 套件編譯與繪製，200 km/h 的下壓力 4029 N（車底通道約 2.96 kN，比量產性能車高一倍以上，待對量產車資料校準） |
| 0.35.0 | 2026-10-18，原開發機 | 617＋5（新增 `AbrasiveTest` 3、`OperationsTest` 5、`NontraditionalTest` 5：Taylor 壽命與經濟壽命，磨削時間、去除率與砂輪磨耗，精研、拋光、珩磨（Preston 與網紋），齒輪加工、拉削、鋸、螺紋、鉚接，放電去除率、線切割與鑽孔，電化學（Faraday 與歐姆），雷射與電漿的熱平衡，水刀（Bernoulli、混合與沖蝕），輔助氣瓶與空氣分離；對課本例題） | 52 | 未啟動客戶端（只加離線計算；hdfv 0.17 的機加工用到） |
| 0.35.1 | 2026-10-18，原開發機 | 622＋5 | 52 | 只改 `RoadAero` 的車底（含 0.35.0 的機加工類別）。`RoadAeroTest` 16（新增 5：718 Cayman GT4 的擴散器 174 N 對官方換算 155 N、AMG GT R 的 40 mm 車底擋板 432 N 對 392 N、車輪拿掉 50～59 %（Breslouer & George 最多 65 %）、粗糙車底的擴散器只有平整底板的 26 %、hdfv 範例的溫和套件 1～2 kN；改寫：開著側面的擴散器 0.03～0.2、封住的地面效應車底沒有車輪時 0.8～3、有車輪時 0.3～1.5、封一半的增益不到一半）、`RoadAeroBodyTest` 的量級改成 30 m/s 超過 400 N；`build publish` 給 hdfv 0.16.1（範例轎車的溫和套件 200 km/h：4.0 → 1.78 kN，車底 2.96 → 0.75 kN）。未啟動客戶端 |
| 0.36.0 | 2026-10-18，原開發機 | 633＋5（新增 `CouplingTest` 6：零重力兩台亂轉 10 s 動量守恆 1e-9、動能不升超過 0.1 %、間隙 < 1 mm、側傾維持、俯仰停在限位；球頭單擺週期 ±1 %；第五輪承受側傾力矩；1:5 與 1:10 質量比；沒有接頭時與舊方法逐位元相同；JSON 往返。`ConvoyTest` 3：8 t＋20 t 低速 0.6 倍滿舵的折角 42.5° 對幾何 39.5°、間隙 0.2 mm；倒車折到 80° 限位停；只煞拖車兩台一起減速、轉送煞車。`ShownValuesTest` 多伸縮支柱（5 kN 縮起 0.3 m 再放下，1 cm 內）、`FrameAnalysisTest` 多主銷 pins（12 m、20 t 拖車：主銷約四成、煞車水平力平衡、壕溝落在拖車自己的左前輪；沒有 pins 逐位元相同）） | 53（新增 `aCoupledVehicleIsDraggedAlong`：兩顆球用球頭連著，推一顆另一顆跟著走、間隙 < 1 cm、連結存讀、拒絕太遠／已掛／成環、故障與移除解開） | `build publish` 給 hdfv 0.18；未啟動客戶端（世界中的牽引車與拖車在 hdfv 的 GameTest 與 AutoShot） |
| 0.37.0 | 2026-10-19，原開發機 | 665＋5，同日補驗後 666＋5（新增聲學 32＋`aSourceLaunchedPastTheListenerBoomsWhileItsStandingSoundIsStillArriving`：`DspTest`（雜訊、濾波、延遲線、振盪器、模態、WAV 讀寫、同種子同樣本）、`PropagationTest`（ISO 9613-1 表值、球面擴散、都卜勒、延遲時間：次音速一根、超音速錐內兩根、錐外沒有、去重；Sabine 與 Eyring 的已知房間、質量定律、響度對應）、`SourcesTest`、`MoreSourcesTest`（直列四缸 3000 rpm 的 100 Hz、十字形曲軸 V8 的半階、V 型雙缸的不等間隔、渦輪嘯聲、噴流 80 log V、Gutin 葉片頻率、Concorde 音爆約 2 psf ±25 %、胎紋頻率隨車速、放氣隨壓力衰減、拍水的悶響、撞擊的模態）；`VehicleTest` 1e-9 不變） | 55（新增 `aLandingIsAnImpactAndASlideAScrape`、另一個工作階段的 `ChunkUnloadGameTests`） | hdfv 0.19 的 `AutoShot`（`AeroCore HDFV QA`，第 2 個螢幕，使用者同意）三輪：第一輪四條測試正弦波與主執行緒卡 300／600 ms 時串流約 40 % 的塊放空、有喀聲，改成自己的補塊執行緒、`read()` 補算、`Thread.sleep(1)`、領先 2048 後第二輪放空 0；EFX 開（石室 RT60 8.8 s、汽車廠 6.8 s、室外 0.6～0.9 s）；第一輪關閉時伺服器卡在區塊卸載（使用者同意強制結束，另一個工作階段修好，之後兩輪 1 秒內存檔關閉）。第二輪兩台測試車疊在一起每 tick 撞擊（1853 次）：撞擊封包改成一串只送第一次（之後 2 分鐘 2 次）。同日（另一個工作階段，hdfv 本機專用伺服器＋Dev、Dev2，`AeroCore MP QA`，使用者同意）補驗：多人連線（Dev2 用同步的狀態合成 Dev 的車、喇叭、方向燈繼電器只在車內）、`FarSounds`（Dev2 在 515 m 與 1245 m 外，伺服器 view distance 8 的追蹤範圍 128 m 外，聽到 Dev 空檔踩油的車 46、44 dB，放油門後消失）、音爆（`/hdfv dev drive launch 600`：測試車從 100 m 高、Dev 側方 40 m 外 64 m 處發射）。音爆找到並修掉三件：`SourceTrack` 的「到達數從 0 跳到 ≥1」抓不到（錐內那段聲音只有 8 ms，又有靜止時的聲音還在到達）→ 改看到達時間的折點（`foldArrives`，新 JUnit，physics 666）；客戶端快速載具用被截到 78 m/s 的原版運動判斷、沿一軸飛時落後兩 tick 滑行 → 用同步的 `v`；進入追蹤的第一步多一個 tick（假的 Mach 3.4 音爆）→ `targetTick` 減 1。修好後兩個聽者各一次音爆（Dev 4.0～4.7 kPa、41～45 m，Dev2 3.9～4.2 kPa、41～43 m，在預期的錐到達時間）。另見：以 600 m/s 從石台上發射時車撞到石台、被拋到 y 210；一次頭朝下壓在地上發射後停在原地、速度一直保留（快速載具彈出走廊時每 tick 退回，0.28 的 `finishSubstep`；沒有修，另開工作）。未確認：實際聽起來如何、Sound Physics Remastered 並存 |
| 0.38.0 | 2026-10-19，原開發機 | 683＋5（新增 `LightSignalTest`（每個序號每週期亮一次、短閃不被幀率吃掉）、`SectorColoursTest`（PAPI 在 3° 兩紅兩白、偏高全白、偏低全紅、羽化）、`AllardTest`（10 000 cd 在 MOR 800 m：夜 1672 m、日 812 m）、`AirDataTest` 4（ISA 反算、QNH／QFE 與高度表、空速含超音速皮托、風分量與方位）、`WindGustsTest` 的峰值、`SurfaceWindTest` 的盛行風；`VehicleTest` 1e-9 不變） | 57（新增 `airAtAPointIsWhatVehiclesFeel`（海域內外）、`tracksFindTheVehiclesNearAPoint`（自己的批次：放在預設批次時同批的 `hullFloatsLevelAtItsDesignWaterline` 失敗）） | hdfv 0.20 開發時的 `AutoShot`（`AeroCore HDFV QA`，第 2 個螢幕，使用者同意）：`/aerocoredev lamps grid` 1638 盞約每幀 0.3～0.5 ms；PAPI 高 4 白、在下滑道上 2 白 2 紅（紅在靠跑道側）、低 4 紅；夜間 1、2、4 km 都看得到；順閃燈；白天淡。選用著色的回歸：沒選用的材質前後截圖逐像素比對只差 ≤ 2 級（浪、遠處區塊、引擎預覽的皮帶這類會動的地方）。原版 position_tex_color 丟掉 alpha < 0.1 的像素，燈先是看不見的，改成自己的著色器 |
| 0.39.0 | 2026-10-20，原開發機 | 686＋5（新增 `AmbientOcclusionTest.aWalledFloorDarkAtItsCornersIsSplitAtItsOpenMiddle`、`LensPatchesTest` 2） | 57 | 美化著色（`Beautify`、材質的 `beautify`、暖色方塊光、地面色、`Refine.CENTRE`、燈罩光暈、接觸陰影、`ShotDiff`）；`PartGpu` 每種 render type 只套一次著色器、`GPU_MIN_QUADS` 96 → 32。hdfv 0.22 的 AutoShot：關掉時與 0.38 相同（只差會動的東西）；hdfv 的排列場景 5.75 → 4.47 ms、95 → 113 FPS（車輪成為件之後；沒有這次的 `PartGpu` 是 11 ms、58 FPS） |
| 0.32.0 | 2026-10-17，原開發機 | 544＋5 | 52 | 新 JUnit `PlaceTest`（搬到別處：油箱、內部狀態、顯示值、軸轉速逐位元保留，車身在新位置靜止，波浪記憶重來；複本不影響原本）；遊戲中由 hdfv 0.14 確認：組裝室推出的車保留油（GameTest）；`AutoShot` 的 `server`（本機專用伺服器與兩個客戶端）、`key <按鍵名稱> tap`（hdfv 的換檔、手煞車、四驅鍵）、連線中斷時關掉遊戲、暫停選單自動關掉；未確認：連不上伺服器時自己關掉（只寫了程式） |
| 0.31.0 | 2026-10-17，原開發機 | 542＋5 | 52 | 新 JUnit `RestyleTest`（換外觀後與原本逐位元相同 300 步、複本不影響原本、質量或零件不同時拒絕）；遊戲中由 hdfv 0.13 確認：上傳的圖當貼紙材質（暫代透明、到了換圖、不重新烘焙）、烤漆房完成時 `replaceBlueprint` 讓轎車在原地變色並保留狀態（GameTest：質量不變、位置不動） |
| 0.29.0 | 2026-10-15，原開發機 | 517＋5 | 52 | 新 JUnit：`MeshCompatTest`（0.28 形狀對凍結副本 1e-12）、`MeshUvTest`、`MeshOpsTest`（含 OBJ 來回與 Blender 式的 OBJ）、`AmbientOcclusionTest`（接觸處變暗、封閉與被覆蓋、5 m 機匣外 5 cm 的環管不算覆蓋、斜面不自遮、長面細分、9.3 萬面 1.2 秒）、`PlumeTest`（Prandtl-Pack 格距、音速噴口壓比 3 的 1.1～1.3 D、Ashkenas-Sherman 馬赫盤、Lau 勢核、Charity 黑體表 ±30）；既有測試數字不變。客戶端由 hdfv 0.11 的 `AutoShot` 看：載具改走背景烘焙（貼圖、遮蔽、細節層級）、零件著色器在正午／黃昏／夜晚與設計畫面；開發中發現並修正遮蔽的四個問題（雜湊鍵相撞、長面條紋、斜面自遮、體素覆蓋挖洞，見 docs/RENDERING.md）。hdfv 0.11 開發中補上（版本號仍是 0.29.0）：`PartRenderer.glow`、`PartVehicle.partPoint`、替身 `STAND_IN`（`MeshOpsTest.aStandInShowsAloneAtTheFarthestLevelOnly`，含遮蔽）、GPU 快取 `PartGpu`（同一畫面 12.1 萬面：13.1 → 1.5 ms／幀，畫面一致）、尾焰核心與菱形的強度（疊加後不飽和成黃綠） |
| 0.28.0 | 2026-10-14，原開發機 | 482＋5 | 52 | 新 JUnit：`GasDynamicsTest`、`RocketTest`、`HydrogenTest`、`HydrogenFuelTest`、`ReheatTest`、`RamjetTest`、`TurboRamjetTest`、`ScramjetTest`、`PulsejetTest`、`JettisonTest`、`HighSpeedTest`（對照 NACA 1135、Nakka、Mirai 儲氫罐、J79、J58、Heiser & Pratt、Argus、Sears-Haack、線性超音速翼、SR-71／Concorde／X-15 的蒙皮溫度）；新 GameTest：拋棄零件、Mach 2／5 平飛不停、2000 m/s 撞牆不穿過；既有測試數字不變；未在 AeroCore 啟動客戶端 |
| 0.27.0 | 2026-10-14，原開發機 | 428＋5 | 49 | 石油的操作條件（`PetroleumConditionsTest` 5：FCC 在 530 °C 就是原表、轉化率隨溫度上升、汽油在 540 °C 附近最高、500 °C 輕循環油多、550 °C 液化石油氣多；重組辛烷值越高重組油越少、氫越多；裂解越深乙烯越多丙烯越少、丙烯／乙烯 0.67 到 0.44；閃蒸溫度越高加熱爐燃料越多；油嘴越小流量越少且滿足孔口方程式）；既有測試數字不變；未在 AeroCore 啟動客戶端 |
| 0.26.0 | 2026-10-13，原開發機；2026-10-14 同版號加機體油箱 | 423＋5 | 49 | 動力（`PowerTest` 6：IE3、IEC 電纜表、皮帶與天軸、蒸汽表與 Rankine、柴油空燃比、管路壓降與泵）、柴油與渦輪增壓（`DieselTest` 4：Hardenberg–Hase 手算、Watson、85×88 mm 汽缸 8.1 bar／45.6 %、增壓匹配）、載具油箱（`FuelTankTest` 4：含機體的油放油箱與放進自身質量吃水、縱傾差 1 mm 內）、石油（`PetroleumTest` 3）；既有測試數字不變；hdfv 0.8 以它建置；未在 AeroCore 啟動客戶端 |
| 0.24.0 | 2026-10-12，原開發機；2026-10-13 同版號修正 | 401＋5 | 49 | 傳動、地面、輪胎、車輪、變速箱與分動箱的 JUnit（`DrivetrainTest` 17、`GroundTest` 5、`TyreTest` 9、`WheelTest` 10、`DriveTest` 3、`TransmissionTest` 6）；新 GameTest `wheelRaysFollowBlockShapesAndSurfaces`；2026-10-13 修油氣支柱的孔口阻尼（割線、迭代；`anOrificeDampsWithItsSquareLaw`）、加 `PartStep.override` 與變速箱狀態（`aWrappingPartCommandsTheBoxThroughOverride`），`build publish` 後 hdfv 的 H1～H3 以它建置；未在 AeroCore 啟動客戶端 |
| 0.22.1 | 2026-10-11，原開發機 | 328＋5 | 48 | 新 GameTest：一顆測試球的 `loads` 在半空丟例外，伺服器照常、只有它停住（記錄一次 ERROR、存檔保留原因、`resume` 後落下），另一顆照常落地；暫時拿掉 `EXPECTED_FAULT_TAG` 時該測試以物理例外失敗、其餘 47 個照過、伺服器不當機；未啟動客戶端 |
| 0.22.0 | 2026-10-11，原開發機 | 328＋5 | 47 | 翼型、機翼、螺旋槳、旋翼與傳動的 JUnit；遊戲中由 hdfv 0.4 的範例確認 |
| 0.21.0 | 2026-10-11，原開發機 | 303＋5 | 47 | `Turbofan`、`FreeTurbine` 與 `TurbojetTest` 全過；hdfv 0.3 的渦扇船、渦軸船以 GameTest 確認推進；客戶端畫面在 hdfv 截圖 |
| 0.20.0 | 2026-10-11，原開發機 | 290＋5 | 47 | 零件與載具 GameTest 搬到 hdfv（見該專案）；改接執行時零件前後的 52 個 GameTest 一致通過；客戶端只在 hdfv 開發環境啟動（`AutoShot` 的 `gui`／`client` 截 hdfv 設計畫面），零件外觀與尾浪未看 |
| 0.17.0 | 2026-10-11，原開發機 | 257＋5 | 50 | `AutoShot` 在 `AeroCore Render QA` 生成三個範例載具：浮在水面、給油門後前進並留下尾浪；四個載具靜置時伺服器 tick 平均 31～34 ms（無載具 17 ms）；手動駕駛未實測 |
| 0.16.1 | 2026-10-11，原開發機 | 237＋5 | 47 | `AutoShot` 沿 320 m 航線移動量測：自然海域換區時最慢一幀 18～25 ms（修正前 66～118 ms），平均 105～119 FPS；換區邊緣的過渡畫面未截圖確認 |

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
0.24：車輛在遊戲中的手感與外觀（hdfv 0.6）、`WorldGround` 在各種方塊與未載入區塊邊緣、顯示值同步的頻寬、`/aerocore ground` 的射線訊息；
0.23：機體在真實海況（浪、風）中的表現、駕駛水上飛機用水舵轉向與起飛、車身（0.24 才有輪子）、多人連線的機體同步、`PartMesh.sheet` 法線修正後 AeroCore 其他零件的外觀；
0.22.1：遊戲中 `/aerocore vehicle info` 的停用訊息與 `resume`（只在 GameTest 驗證）、客戶端看到停用載具停在半空中的樣子、真實世界存檔重新載入後仍停著；
0.20：零件外觀改由 `PartRenderer` 登記後的實際畫面（船體、浮筒、螺旋槳、舵、hdfv 的引擎）、尾浪改由 `Part.wake()` 後的實際畫面；
0.19：噴射艇在遊戲中的表現（`/hdfv vehicle samples` 後 `/aerocore vehicle spawn jetboat`：起動、怠速推力、油門反應、熄火，沒有聲音與尾焰）、飛行器上的噴射引擎（還沒有機翼零件）；
0.18：引擎船在遊戲中的表現（`/aerocore vehicle spawn engineboat`：起動時間、怠速、加速、聲音沒有）、引擎 MAP 建立期間的等待、冷車到熱車；
0.17：坐上零件載具用預設按鍵駕駛、站在甲板上、受力箭頭（`/aerocore vehicle debug`）、多人連線的控制與同步、範例以外的藍圖（自己寫 JSON）、
其他客戶端看到的螺旋槳與舵動畫；
0.16.1：使用者世界與使用者電腦上的實際手感（量測在原開發機的 QA 世界、旁觀者模式瞬移）；換區邊緣 0.1～0.2 s 的過渡畫面；
0.16：見上表；仍未看過使用者自己划船時的手感與白沫、熱氣流與坡面上升氣流（沒有飛行器）、雪地的雪傾斜、Fabulous 模式下的天空、多人連線的風同步。
JUnit 報告在本機建置資料夾（`showBuildDirectories`）各模組的 `reports/tests/test/` 與 `test-results/test/`，
例如 `%USERPROFILE%\.mcmod\build\aerocore-e6babf45\physics\test-results\test`；建置輸出改回網路磁碟（`-Paerocore_local_build=false`）時偶爾寫入 HTML 報告失敗，以 XML 結果為準後重跑即可。

JAR 在本機建置資料夾的 `neoforge\libs\aerocore-neoforge-1.21.1-<版本>.jar`，`build publish` 後也在專案的
`build\repo\dev\mcmod\aerocore\aerocore-neoforge-1.21.1\<版本>\`；已包含 physics API。測試安裝只保留一個 AeroCore 版本，不把 sources JAR 放入 mods。
CFD CSV／圖在 `cfd/build/experiments/water/` 或 `air/`；`scripts/plot-cfd.py` 只繪製既有數值結果。

## 延伸文件

- [README](README.md)：啟動、IDE、產物與功能入口。
- [API](docs/API.md)：接入載具的資料型別與呼叫方式。
- [零件的繪製](docs/RENDERING.md)：網格、UV、材質、環境光遮蔽、背景烘焙、零件著色器、尾焰（0.29）；大量固定燈、選用的材質著色（0.38）；美化著色（0.39）。
- [物理模型](docs/PHYSICS.md)、[六自由度平台](docs/PLATFORM.md)、[測試船](docs/HULL.md)、[測試球](docs/PROBE.md)。
- [海況與尾浪](docs/SEA.md)：不規則海、二階水面、共用水面、非定常尾浪、白沫、水花；[滑行艇](docs/PLANING.md)：2D+t 模型與 Fridsma 驗證。
- [原版船](docs/ROWBOAT.md)：划船物理、參數來源與驗證。
- [熱力學與視覺化](docs/THERMAL_DEBUG.md)：指令、比例尺、熱模型及實測圖。
- [本機執行資料](docs/LOCAL_RUNTIME.md)：世界遷移、備份、本機的建置輸出與暫存目錄造成的建立世界崩潰。
- [CFD](docs/CFD.md)：獨立流道求解與離線圖，不等同於遊戲環境流場。
- [大氣](docs/ATMOSPHERE.md)：全世界的風、天空、雲與雨雪、飛行用大氣、空氣資料與一點的風（0.38）；[地面載具](docs/GROUND.md)：輪胎、滾動阻力、軟地面。
- [車架](docs/FRAME.md)：會扭的車架進物理、梁 FEM、承載式車身的扭轉（0.33）。
- [鉸接](docs/COUPLING.md)：兩台載具的接頭、求解、世界中的連結、驗證（0.36）。
- [冶金與電化學](docs/METALLURGY.md)：反應焓、電解、電爐、礦物品位、甲醇（0.40）。
- [聲音](docs/ACOUSTICS.md)：程式合成的聲源、傳播、房間、響度、Minecraft 聲音引擎的接口、接觸事件與遠處的聲源（0.37）。
- [零件化載具](docs/VEHICLE_PARTS.md)：零件藍圖、組裝與連接、通用載具物理、遊戲端 `PartVehicle`、驗證。
- [活塞引擎](docs/ENGINE.md)：16 個元件、循環模擬、標定與 MAP、執行時、驗證；[噴射引擎](docs/JET.md)：6 個元件、循環匹配、運轉線、暫態、驗證。
- [渦扇、渦槳、渦軸](docs/TURBOFAN.md)；[翼型、機翼、螺旋槳、旋翼](docs/WING.md)：形狀 → 性能、執行時零件、單向離合器、陀螺力矩、驗證。
