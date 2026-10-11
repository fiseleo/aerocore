# 零件化載具（0.17）

## 分工

- 載具是另一個獨立 Mod **High Degree of Freedom Vehicles**（`Z:\MCMod\HighDegreeOfFreedomVehicles`），AeroCore 只當前置。
- 方向像《Automation》，分兩層：
  1. **自己做零件**：零件不是從目錄裡挑現成的，而是玩家自己設計：畫船體線型、決定翼形與平面形，引擎從缸體、汽缸蓋、曲軸、
     活塞、凸輪、進排氣到增壓逐項設定。設計好的零件是有名字的藍圖，可以重複使用、分享。
  2. **零件組合出載具**：把零件藍圖裝到載具上、決定位置與數量，再連接動力（馬達或引擎 → 齒輪 → 螺旋槳，之後也接輪子）與控制通道。
- **AeroCore 負責**：零件的參數定義與檢查、「設計參數 → 效能」的計算、組裝後的質量性質、行駛模擬、與海／風／地面／其他物體的互動、
  同步與存檔格式。所有計算都能離線呼叫（不需要世界或實體），所以測試台可以用同一套物理。
- **載具 Mod 負責**：設計畫面、測試台（零件與整台載具的效能數字）、CNC／機加工合成、零件物品、模型與動畫、按鍵、遊戲平衡。

## 參數原則

1. **每個參數都開放**：零件以參數記錄描述，預設值只是起點，不鎖任何組合。
2. **只擋不合物理的值**：負的長度、零直徑、葉片多到互相重疊等。被擋的參數以 `DesignIssue`（`INVALID`）說明，路徑像 `parts.screw.blades`。
3. **超出模型驗證範圍照算，但標示是外推**：`DesignIssue`（`EXTRAPOLATION`）帶數值、驗證範圍與來源，設計畫面可以自己寫警告文字。
4. **代價由物理而來**：尺寸與材質決定質量；推力與功率決定耗能。不加「平衡用」的懲罰，平衡是載具 Mod 的事。
5. **質量可以算也可以指定**：`MassSpec.shell(材質, 殼厚)`、`MassSpec.solid(材質)`，或 `MassSpec.given(質量)`（分布沿用幾何，可另給重心）。
6. **形狀也是參數**：船體由任意站位、任意凸多邊形斷面構成；`HullDesign.tapered(...)` 這類長、寬、吃水的產生器只是快速起點。
   斷面必須是凸的，凹槽與隧道船底無法表示。

## 架構

### 純物理：`dev.mcmod.aerocore.api.vehicle`

| 類別 | 作用 |
| --- | --- |
| `PartDesign` | 零件藍圖介面：`type()`（`aerocore:float` 這類命名空間 ID）、`name()`、`check()`、`compile()`、`toJson()`。第一批零件各是一個 record |
| `PartTypes` | 依類型 ID 讀回藍圖的註冊表；載具 Mod 用 `register("modid:part", reader)` 加自己的零件類型。編譯結果依藍圖 JSON 快取（同一個船體在伺服器與每個客戶端只編譯一次） |
| `Part` | 編譯後的零件：在零件座標系下的質量性質、碰撞球、外推警告、轉子、輻射點、甲板、座位、可綁定的輸入，以及每小步的受力 `loads(PartStep)` |
| `PartStep` | 一個零件在一小步看到的東西（載具狀態、環境、輸入、轉子轉速、所在螺旋槳尾流）與寫入受力的方法；`partState()`／`partProperties()` 是以零件軸看同一個剛體，讓「以自身座標寫的模型」（船體）裝在任意位置與朝向都能直接用 |
| `Mount` | 零件裝在載具上的位置與 yaw／pitch／roll（度）；載具與零件座標同為 x 前、y 上、z 右舷 |
| `Binding` | 把零件的輸入（`throttle`、`deflection`）接到具名控制通道：輸入 = offset + scale × 通道值。通道名稱自由，按鍵由載具 Mod 決定 |
| `Connection` | `Shaft`（動力軸：from 的轉速 = ratio × to 的轉速，含效率）、`Slipstream`（控制面在某個螺旋槳的尾流裡） |
| `VehicleBlueprint` | 一台載具的藍圖：零件（id、藍圖、安裝、綁定）與連接；JSON 往返不失真；`check()`、`compile()` |
| `Vehicle` | 編譯後的載具：整體質量、重心、慣性張量（平行軸定理）、碰撞球、甲板、座位、輻射點、轉子與動力軸傳動系，全在載具座標系 |
| `VehicleState` | 會變的部分：剛體狀態、各傳動系轉速、波浪輻射記憶、上一步的受力與診斷值；`toJson`／`fromJson` 存檔 |
| `VehicleEnvironment` | 一小步裡零件取樣的水與空氣（球體浸沒、任一點的水面／流速／加速度／含風的空氣、波場）；`calm(...)` 給離線測試 |
| `VehicleDynamics` | `prepare`／`finish`／`step`：各零件受力、整車重量、輻射記憶、傳動系，加上所有零件的附加質量與阻力，一次隱式求解；可與其他物體共用 `ContactSolver` |
| `SampleBlueprints` | 範例：`platform`、`launch`、`cruiser`（格式與物理的示範，不是載具 Mod 要出的設計） |

**一小步的順序**（`VehicleDynamics.prepare`）：
1. 先算「早」零件（螺旋槳，它們的推力給尾流裡的控制面用），再算其他零件。
2. 加上整車重量與所有輻射點的垂直輻射記憶力；各點力乘以其零件給的比例（滑行船體隨艉板通氣淡出）。
3. 傳動系：每條動力軸串成一個自由度，根部轉速以後向歐拉推進。I_eff = Σ I_r k_r²，τ_eff = Σ k_r e_r τ_r，k_r 是轉子轉速對根部的比，e_r 是齒輪效率：
   往根部反映的負載扭矩除以效率，驅動扭矩乘以效率。載具承受 −Σ (I_r α_r − τ_外,r) 沿各轉軸：馬達轉螺旋槳時，水對槳的扭矩經由馬達座傳到船體；轉子加速時，船體被反向踢。
   轉子的陀螺力矩（載具轉向時）不計。
4. 6×6 附加質量（船體條帶）、點附加質量（浮筒的 C_a ρV）與點阻力一起進入 `RigidBodyDynamics` 的通用隱式解。
   沒有 6×6 項時，它和浮筒原本的解法相同；沒有點項時，和船體原本的解法相同。
5. `finish`：物體加上夾帶水一起漂移（`RigidBodyDynamics.driftWithPointMasses`），再與障礙物分離，最後寫回狀態與輻射記憶。

**重構但不改結果**：`FloatingBodyDynamics.sphereLoads`、`HullDynamics.loads`、`PlaningDynamics.loads` 是原本積分器裡逐浮筒或整船的受力，
抽出來給零件用。原本的 API 不變，237 項舊測試全部照舊通過。

### 遊戲端：`dev.mcmod.aerocore.neoforge.vehicle`

- **`PartVehicle`**：通用載具實體，載具 Mod 可以直接用，或繼承後改操作與互動。
  - 伺服器權威，每 tick 10 小步，由 `BodyWorld` 和其他物體一起推進，解與方塊、AeroCore 物體、原版實體的接觸。
  - 環境用共用水面（波浪、海岸、尾浪）、全世界的風與天氣。
  - 客戶端收到藍圖、姿態、控制值與轉子轉速，只做動畫，不模擬。
  - 人可以站上零件甲板（`SolidBody`），坐在座位上；乘客體重壓在該座位上。
  - 船體零件會產生尾浪與水花：排水船體用 Michell 薄船，滑行船體用壓力片。
  - 受力箭頭（`DebugBody`）、存檔（藍圖、狀態、控制值）。
  - 藍圖之後無法編譯時（例如提供零件類型的 Mod 被移除），實體保留藍圖原樣存回，但不移動。
  - 效能：船體的 Michell 表與螺旋槳四象限表在背景執行緒預建（`Part.warmUp`，同一份藍圖只建一次），建好前載具原地不動；
    尾浪的薄船設計也在背景建立。第一步以當時的水線面半徑啟動輻射記憶（`WaveRadiation.Memory.prime`）。空氣每 tick 每個方塊格只取樣一次。
    原開發機量測：生成第一艘新船體時伺服器 tick 從 1.9 s 降到 75 ms 以下。兩艘巡航艇＋兩座平台在 QA 世界的自然海域靜置（旁觀者）：
    伺服器 tick 平均 31～37 ms（移除後 15～17 ms），主要是輻射記憶卷積、波浪取樣與方塊接觸；相鄰船體的球對球接觸在 0.17 開發中修正前曾讓平均升到 150 ms 以上。
    生成滑行艇後約 5 秒內 V 形船體吃水一直變、要算新的輻射核函數：最慢 tick 64 ms、7 個超過 50 ms（核函數向量化前 165 ms、30 個）。
- **`VehicleNetwork.ControlsPayload`**：客戶端送控制值給伺服器，`PartVehicle.canControl` 決定誰可以操作（預設是第一個座位上的人）。
- **`VehicleRegistry`**：AeroCore 自己的實體類型 `aerocore:part_vehicle`，以及同步長字串藍圖用的資料序列化器（上限 4 MB）。
- 客戶端 **`PartVehicleRenderer`**：由藍圖生成的樸素外觀，包括放樣船體、浮筒球、方塊、會轉的螺旋槳、會偏轉的控制面。
  載具 Mod 通常自己畫，也可以把它註冊給自己的實體類型。
- 客戶端 **`VehicleControls`**：AeroCore 的品管用按鍵對應。
  - 駕駛時前後鍵設定 `throttle` 為 ±1，左右鍵設定 `rudder` 為 ∓1（正值轉右舷）。
  - 載具 Mod 把 `usesDefaultControls()` 改回傳 false 就能關閉。
- **指令** `/aerocore vehicle`：
  - `samples`：把範例藍圖寫到伺服器目錄的 `aerocore/blueprints/`。
  - `spawn <名稱>`：在前方 4 m 生成，朝向視線方向。
  - `control <通道> <值>`、`info`、`debug <模式>`、`remove`：都作用於 64 格內最近的載具。

## 零件與參數（第一批）

| 零件（類型 ID） | 參數 | 計算與來源 | 驗證範圍（超出時標示外推） |
| --- | --- | --- | --- |
| 浮筒 `aerocore:float` | 半徑、質量（殼或實心或給定） | 與 `FloatingBodyDynamics` 的浮筒相同：精確球冠浮力含波壓、Clift 光滑球阻力、C_a ρV 附加質量與 Morison 激振、穿出水面時的精確半球垂直繞射、Hulme 輻射 | — |
| 排水船體 `aerocore:hull` | 任意站位與凸斷面、水面／潛航、設計排水量、Holtrop 艉形係數、輻射分段、質量 | 與 `HullDynamics` 相同：條帶浮力、ITTC-57 × Holtrop–Mennen ＋ Michell 阻力、條帶附加質量與理想流體項、Clarke 校準升力元件、Hoerner 橫流 | Holtrop–Mennen 1982：3.9 < L/B < 15、2.1 < B/T < 4、0.55 < C_P < 0.85；艉形 −25～10 |
| 滑行船體 `aerocore:planing_hull` | 長、舭寬、底部斜升角、型深、站數、輻射分段、質量 | 與 `PlaningDynamics` 相同：Zarnick 2D+t、Garme 艉板修正、ITTC 摩擦、鬚狀水花 | Fridsma 1969：斜升角 10～30°、L/B 4～7 |
| 螺旋槳 `aerocore:propeller` | 直徑、葉數、展開面積比、螺距比、轂比、葉厚比、質量 | B 系列範圍內：Wageningen 回歸＋MARIN 四象限量測（三個量測過的槳）或同幾何的葉素動量理論；範圍外：全用葉素動量理論。依槳盤浸水比例混合水與空氣 | Oosterveld 與 van Oossanen 1975：2～7 葉、A_E/A_0 0.30～1.05、P/D 0.5～1.4 |
| 控制面 `aerocore:control_surface` | 弦長、展長、最大偏角（≤ 35°）、根部貼船體與否、厚度比、質量 | `Rudder`／`AeroSolver`：根部貼船體用 Whicker 與 Fehlner 1958 的低展弦比舵；兩端自由用 Helmbold 升力斜率。在螺旋槳尾流裡時，入流依致動盤尾流增強 | 根部貼船體：幾何展弦比 1～3（Whicker 與 Fehlner） |
| 方塊體 `aerocore:box` | 尺寸、阻力係數（預設 1.05：正對的立方體，Hoerner）、封閉與否、每邊格數、質量 | 分格取樣：封閉時依各格浸水比例給浮力；阻力 ½ρC_D A_p(u)\|u\|u，A_p 是沿相對流向的投影面積，平均分給各格（也阻尼轉動），隱式處理 | 沒有附加質量、輻射與局部靜水壓以外的 Froude–Krylov（在浪裡阻尼偏低） |
| 電動馬達 `aerocore:electric_motor` | 最大扭矩、最大功率、最高轉速、轉子慣量、效率、質量 | 場弱化馬達的教科書扭矩—轉速包絡：基速以下定扭矩、以上定功率、最高轉速以上為零；乘以 `throttle`（−1～1，反向轉時是煞車）。耗電 = 軸功率 ÷ 效率（回充時 × 效率）。外殼慣性以 4000 kg/m³ 的立方體估計 | 比功率 ≤ 5.2 kW/kg（Siemens SP260D，2015） |
| 活塞引擎 `aerocore:piston_engine`（0.18） | 16 個各自設計的元件：缸體、汽缸蓋、活塞、連桿、曲軸、氣門、凸輪軸、火星塞、噴油嘴、配氣、供油、點火、潤滑、冷卻、進氣、排氣 | 逐曲軸角單區循環模擬＋Patton／Sandoval 摩擦，標定成轉速 × 歧管壓力 MAP；執行時有歧管壓力、起動、怠速、斷油、暖車與散熱。詳見 [ENGINE.md](ENGINE.md) | 見 ENGINE.md：壓縮比 6～14、平均活塞速度 ≤ 25 m/s、燃燒相關式 1500～3500 rpm 等 |
| 噴射引擎 `aerocore:jet_engine`（0.19） | 6 個各自設計的元件：進氣道、壓縮機、燃燒室、渦輪、噴嘴／尾管、燃油控制 | 變比熱的單軸渦輪噴射循環：設計點定渦輪與噴嘴面積，非設計點平衡成依飛行馬赫數的運轉線，執行時準穩態（起動、點火、怠速、加速排程、喘振、熄火）；推力沿零件 +X、轉子繞 +X。詳見 [JET.md](JET.md) | 見 JET.md：壓縮機級負荷 ≤ 1、進口修正流率 ≤ 241 kg/(s·m²)、轉盤應力、渦輪葉片溫度、渦輪級負荷 ≤ 4、尾管材料 |
| 壓艙 `aerocore:ballast` | 質量 | 點質量 | — |
| 座位 `aerocore:seat` | 座椅質量 | 乘客位置（腳的位置）；乘客體重由遊戲端加上 | — |

## 藍圖格式（JSON）

```json
{
  "format": 1, "name": "launch",
  "parts": [
    {"id": "hull", "design": {"format": 1, "type": "aerocore:planing_hull", "name": "fridsma hull", "lengthM": 6, "chineBeamM": 1.2,
      "deadriseDeg": 20, "depthM": 0.75, "stations": 48, "radiationSegments": 6, "mass": {"massKg": 880}}, "position": [0, 0, 0]},
    {"id": "motor", "design": {"format": 1, "type": "aerocore:electric_motor", "name": "60 kW motor", "maxTorqueNm": 250, "maxPowerW": 60000,
      "maxSpeedRpm": 4000, "rotorInertiaKgM2": 0.05, "efficiency": 0.94, "massKg": 60}, "position": [1.2, 0.25, 0],
      "bindings": [{"role": "throttle", "channel": "throttle"}]},
    {"id": "rudder", "design": {"...": "..."}, "position": [-0.5, -0.22, 0], "rotation": {"yaw": 0, "pitch": 0, "roll": 0},
      "bindings": [{"role": "deflection", "channel": "rudder", "scale": 1}]}
  ],
  "connections": [
    {"type": "shaft", "from": "motor", "to": "screw", "ratio": 1.5, "efficiency": 0.96},
    {"type": "slipstream", "propeller": "screw", "surface": "rudder"}
  ]
}
```

- 數字以最短可往返的十進位寫出，讀回的 double 完全相同；比新版本 AeroCore 更新的 `format` 會被拒絕。
- 材質寫成 `{"name": "steel", "density": 7850}`。標準材質只寫名稱也可以。

## 驗證

| 測試 | 內容 |
| --- | --- |
| `VehicleTest.floatsReproduceFloatingBodyDynamicsWithRadiation` | 四個浮筒零件組成的平台在斜浪中 2 s（含輻射記憶）與 `FloatingBodyDynamics`＋同樣的輻射記憶逐步相同（1e-9） |
| `hullReproducesHullDynamicsAndDoesNotCareHowItIsMounted` | 測試船的斷面做成船體零件，1.5 s 與 `HullDynamics` 相同（1e-9）；同一船體轉 70° 安裝、載具反轉回來，世界中的運動相同 |
| `planingHullReproducesPlaningDynamics` | Fridsma 船體以 8 m/s 滑行 1 s，與 `PlaningDynamics`＋淡出的輻射相同（1e-9），通氣 > 0.5 |
| `motorAndScrewSettleWhereTorquesBalance...` | 馬達經 1:1 與 2:1（效率 0.95）齒輪驅動 B4-70 槳繫柱：穩態時馬達扭矩 = 槳扭矩 ÷ 齒比 ÷ 效率（0.2%），推力 = K_T(0) ρ n² D⁴（1%），船體承受的反扭矩 = 槳扭矩 |
| `anUnloadedMotorRunsUpToItsTopSpeedAndStays` | 空載馬達升到最高轉速；轉子加船體的角動量守恆（1e-6） |
| `rudderPushesTheSternToPort...` | 正舵把艉推向左舷、船艏轉右舷；轉 90° 安裝成水平面時，同樣大小的力變成垂直 |
| `aClosedBoxFloatsAtItsArchimedesDraft` | 2×1×1 m、800 kg 的方塊在計算吃水 ± 偏移處，淨力 = (ρ_w − ρ_a) g A × 偏移 |
| `massPropertiesFollowTheParallelAxisTheorem`、`mountsTurnAndMovePartMass` | 平行軸定理手算、薄殼與實心方塊的閉式解；轉 90° 安裝時慣性主軸互換 |
| `BlueprintTest` | JSON 往返（含每種零件、安裝角度、綁定、連接）、不合物理的值附路徑並阻止編譯、外推警告、輸入名稱檢查、零件類型 ID 規則、新格式拒絕 |
| `VehicleGameTests` | 範例平台在水池中由四個浮筒撐起全部重量（阿基米德 3% 內）並保持水平；範例巡航艇在油門下前進、正舵轉右舷、2:1 齒輪讓槳轉速減半；乘客坐在座位上，存檔後藍圖、狀態、轉速、控制值都回來 |

## 尚未做的

- 引擎設計（缸徑、行程、缸數、排列、壓縮比、凸輪、進排氣、增壓、燃料 → 扭力曲線、油耗、質量）、離合器與變速箱、輪子與懸吊、機翼：之後的版本。
- 零件之間的流體交互作用只有「控制面在螺旋槳尾流中」；機翼下洗到尾翼、船體伴流打在槳上、多船體間的波浪干擾（輻射記憶以外）都沒有。
- 方塊體在浪中的附加質量與輻射；船體只能水平安裝（垂直輻射記憶以載具水平面量點距，傾斜安裝的船體會被當成另一種排列）。
- 傳動系不含離合器（打滑）、軸的扭轉彈性與轉子陀螺力矩。
- 結構強度、損壞、成本、耐久：屬於遊戲平衡，由載具 Mod 決定。
- 遊戲中的實際外觀與手感：見 AGENTS.md 待實測清單。
