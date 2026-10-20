# 零件化載具（0.17；0.20 起零件設計在 hdfv）

## 分工

- 載具是另一個獨立 Mod **High Degree of Freedom Vehicles**（Mod ID `hdfv`，`Z:\MCMod\HighDegreeOfFreedomVehicles`），AeroCore 只當前置。
- 方向像《Automation》，分兩層：
  1. **自己做零件**：零件不是從目錄裡挑現成的，而是玩家自己設計：畫船體線型、決定翼形與平面形，引擎從缸體、汽缸蓋、曲軸、
     活塞、凸輪、進排氣到增壓逐項設定。設計好的零件是有名字的藍圖，可以重複使用、分享。
  2. **零件組合出載具**：把零件藍圖裝到載具上、決定位置與數量，再連接動力（馬達或引擎 → 齒輪 → 螺旋槳，之後也接輪子）與控制通道。
- 2026-10-11 使用者決定「AeroCore 是計算 API」，零件「全部搬到 hdfv」（0.20）：
  - **AeroCore 負責**：
    - 物理與從設計抽出的計算：`api.*`、`api.engine.*`。
    - 零件的執行時受力：`api.vehicle.part`。
    - 通用載具框架：藍圖、組裝與連接、質量性質、`VehicleDynamics`。
    - 與海／風／地面／其他物體的互動、尾浪、同步與存檔格式。
    - 所有計算都能離線呼叫（不需要世界或實體），所以測試台可以用同一套物理。
  - **hdfv 負責**：
    - 零件設計：參數、檢查與容差、外推範圍、JSON、質量估計。
    - 引擎的控制規則。
    - 設計畫面、測試台、CNC／機加工合成、零件物品、模型與外觀、按鍵、遊戲平衡。

## 參數原則（零件設計遵守，AeroCore 提供工具）

1. **每個參數都開放**：零件以參數記錄描述，預設值只是起點，不鎖任何組合。
2. **只擋不合物理的值**：負的長度、零直徑、葉片多到互相重疊等。被擋的參數以 `DesignIssue`（`INVALID`）說明，路徑像 `parts.screw.blades`。
3. **超出模型驗證範圍照算，但標示是外推**：`DesignIssue`（`EXTRAPOLATION`，`DesignIssue.range`）帶數值、驗證範圍與來源，設計畫面可以自己寫警告文字。
4. **代價由物理而來**：尺寸與材質決定質量；推力與功率決定耗能。不加「平衡用」的懲罰，平衡是載具 Mod 的事。
5. **質量可以算也可以指定**：`MassSpec.shell(材質, 殼厚)`、`MassSpec.solid(材質)`，或 `MassSpec.given(質量)`（分布沿用幾何，可另給重心）；
   幾何分布用 `MassShapes`（球、方塊、船殼）。
6. **形狀也是參數**：船體由任意站位、任意凸多邊形斷面構成。斷面必須是凸的，凹槽與隧道船底無法表示。

## 架構

### 純物理：`dev.mcmod.aerocore.api.vehicle`

| 類別 | 作用 |
| --- | --- |
| `PartDesign` | 零件藍圖介面：`type()`（`hdfv:float` 這類命名空間 ID）、`name()`、`check()`、`compile()`、`toJson()`。由載具 Mod 實作 |
| `PartTypes` | 依類型 ID 讀回藍圖的登記表（AeroCore 沒有內建類型）；載具 Mod 在建構時用 `register("modid:part", reader)` 加自己的零件類型。編譯結果依藍圖 JSON 快取（同一個船體在伺服器與每個客戶端只編譯一次） |
| `Part` | 編譯後的零件：在零件座標系下的質量性質、碰撞球、外推警告、轉子（0.24 起有名稱＝埠）、輻射點、甲板、座位、可綁定的輸入、內部狀態、尾浪（`wake()`），以及每小步的受力 `loads(PartStep)`；0.24 起還有 `links()`（自己耦合的轉子對）、`after(PartAfter)`（步末）、`shownValues()`（給客戶端畫的值） |
| `PartStep` | 一個零件在一小步看到的東西（載具狀態、環境、輸入、轉子轉速、所在螺旋槳尾流）與寫入受力的方法；`partState()`／`partProperties()` 是以零件軸看同一個剛體，讓「以自身座標寫的模型」（船體）裝在任意位置與朝向都能直接用；0.24 起有傳動的約束列、滾動力、線阻尼（[DRIVETRAIN.md](DRIVETRAIN.md)） |
| `PartAfter`、`Drivetrain` | 0.24：步末給零件的視圖（列扭矩、步末轉速與狀態）；軸系與約束列的求解（套件內） |
| `Mount`、`Binding`、`Connection` | 安裝位置與 yaw／pitch／roll（度）；把零件輸入接到具名控制通道；`Shaft`（動力軸，0.24 起可指定兩端的埠）、`Slipstream`（控制面在螺旋槳尾流裡） |
| `VehicleBlueprint`、`Vehicle`、`VehicleState`、`VehicleEnvironment`、`VehicleDynamics` | 載具藍圖（JSON 往返不失真）、編譯後的載具（整體質量性質、傳動系）、會變的狀態（含存檔）、一小步的水與空氣、一步的推進 |
| `MassProperties`、`MassSpec`、`MassShapes`、`Material` | 質量與慣性、殼／實心／給定質量、單位密度的形狀、結構材料 |
| `DesignIssue`、`DesignException`、`Fields`、`Json` | 檢查結果、編譯失敗、讀 JSON 的路徑化欄位、精簡 JSON |
| `WakeSource` | 零件的尾浪形式：`ThinShip`（Michell 薄船）、`Planing`（壓力片） |

### 執行時零件：`dev.mcmod.aerocore.api.vehicle.part`

載具 Mod 的設計 `compile()` 時建構它們（傳入物理量與擁有它的設計）。受力程式從 0.17～0.19 的零件設計原樣搬來，結果不變。

| 類別 | 計算與來源 | 模型的驗證範圍（零件設計據此標示外推） |
| --- | --- | --- |
| `SpherePart` 浮筒 | 與 `FloatingBodyDynamics` 的浮筒相同：精確球冠浮力含波壓、Clift 光滑球阻力、C_a ρV 附加質量與 Morison 激振、穿出水面時的精確半球垂直繞射、Hulme 輻射 | — |
| `HullPart` 排水船體／潛航體 | 與 `HullDynamics` 相同：條帶浮力、ITTC-57 × Holtrop–Mennen ＋ Michell 阻力、條帶附加質量與理想流體項、Clarke 校準升力元件、Hoerner 橫流；表面船體的薄船尾浪 | Holtrop–Mennen 1982：3.9 < L/B < 15、2.1 < B/T < 4、0.55 < C_P < 0.85；艉形 −25～10 |
| `PlaningHullPart` 滑行船體 | 與 `PlaningDynamics` 相同：Zarnick 2D+t、Garme 艉板修正、ITTC 摩擦、鬚狀水花；壓力片尾浪 | Fridsma 1969：斜升角 10～30°、L/B 4～7 |
| `PropellerPart` 螺旋槳 | 四象限模型由 `PropellerModels.fourQuadrant`：B 系列範圍內 Wageningen 回歸＋MARIN 四象限量測（三個量測過的槳）或同幾何的葉素動量理論；範圍外全用葉素動量理論。依槳盤浸水比例混合水與空氣 | Oosterveld 與 van Oossanen 1975：2～7 葉、A_E/A_0 0.30～1.05、P/D 0.5～1.4（`PropellerModels.inBSeriesRange`） |
| `ControlSurfacePart` 控制面 | `Rudder`／`AeroSolver`：根部貼船體用 Whicker 與 Fehlner 1958 的低展弦比舵（`Rudder.lowAspectPolar`）；兩端自由用 Helmbold 升力斜率（`Rudder.freeEndsPolar`）。在螺旋槳尾流裡時，入流依致動盤尾流增強 | 根部貼船體：幾何展弦比 1～3（Whicker 與 Fehlner） |
| `BoxPart` 方塊體 | 分格取樣：封閉時依各格浸水比例給浮力；阻力 ½ρC_D A_p(u)\|u\|u，A_p 是沿相對流向的投影面積，平均分給各格（也阻尼轉動），隱式處理 | 沒有附加質量、輻射與局部靜水壓以外的 Froude–Krylov（在浪裡阻尼偏低） |
| `MotorPart` 電動馬達 | `ElectricMotor`：場弱化馬達的教科書扭矩—轉速包絡；乘以 `throttle`（−1～1，反向轉時是煞車）。耗電 = 軸功率 ÷ 效率（回充時 × 效率） | 比功率 ≤ 5.2 kW/kg（Siemens SP260D，2015），由設計檢查 |
| `MassPart` | 純質量（壓艙、貨物），可帶座位（乘客腳的位置；體重由遊戲端加上） | — |
| `AxlePart` 車軸（0.24） | 兩個 `WheelCorner`：射線找地面、輪胎（`Tyre`、`TyreContact` 暫態滑移）與懸吊串聯、頂起、煞車；差速器與 LSD、轉向與阿克曼、防傾桿（[WHEELS.md](WHEELS.md)） | 輪胎關係的資料範圍見 WHEELS.md |
| `LegPart` 起落架（0.24） | 一個 `WheelCorner`：油氣支柱、彈簧鋼或剛性；可轉、自由轉向或固定 | Smiley & Horne 的飛機胎關係 |
| `TransmissionPart` 變速箱（0.24） | 手排（離合器、同步器）、自排（變扭器、換檔元件）、雙離合、CVT 的機構（[DRIVETRAIN.md](DRIVETRAIN.md)） | — |
| `TransferPart` 分動箱（0.24） | 全時（中央差速器）、分時（2H／4H／4L）、適時（耦合器） | — |

活塞引擎與噴射引擎有控制規則，執行時零件在 hdfv；它們的物理在 `api.engine`（見 [ENGINE.md](ENGINE.md)、[JET.md](JET.md)）。

**一小步的順序**（`VehicleDynamics.prepare`）：
1. 先算「早」零件（螺旋槳，它們的推力給尾流裡的控制面用），再算其他零件。
2. 加上整車重量與所有輻射點的垂直輻射記憶力；各點力乘以其零件給的比例（滑行船體隨艉板通氣淡出）。
3. 傳動系（0.24 起有約束列時由 `Drivetrain` 一起解，見 [DRIVETRAIN.md](DRIVETRAIN.md)）：
   - 每條動力軸串成一個自由度，根部轉速以後向歐拉推進。
   - I_eff = Σ I_r k_r²，τ_eff = Σ k_r e_r τ_r；k_r 是轉子轉速對根部的比，e_r 是齒輪效率。
   - 往根部反映的負載扭矩除以效率，驅動扭矩乘以效率。
   - 載具承受 −Σ (I_r α_r − τ_外,r) 沿各轉軸：馬達轉螺旋槳時，水對槳的扭矩經由馬達座傳到船體；轉子加速時，船體被反向踢。
   - 轉子的陀螺力矩：載具以 ω 轉動時受 −ω × H（0.22 起；在漂移步驟和轉動一起隱式處理）；單向離合器（`Connection.Shaft` 的 `freewheel`）把轉子串分開，主動側跟得上時鎖在一起（0.22 起）。
4. 6×6 附加質量（船體條帶）、點附加質量（浮筒的 C_a ρV）與點阻力一起進入 `RigidBodyDynamics` 的通用隱式解。
5. `finish`：物體加上夾帶水一起漂移（`RigidBodyDynamics.driftWithPointMasses`），再與障礙物分離，最後寫回狀態、輻射記憶與零件內部狀態。

### 遊戲端：`dev.mcmod.aerocore.neoforge.vehicle`

- **`PartVehicle`**：通用載具實體，載具 Mod 可以直接用，或繼承後改操作與互動。
  - 伺服器權威，每 tick 10 小步，由 `BodyWorld` 和其他物體一起推進，解與方塊、AeroCore 物體、原版實體的接觸。
  - 環境用共用水面（波浪、海岸、尾浪）、全世界的風與天氣。
  - 客戶端收到藍圖、姿態、控制值與轉子轉速，只做動畫，不模擬。
  - 人可以站上零件甲板（`SolidBody`），坐在座位上；乘客體重壓在該座位上。
  - 尾浪：第一個 `wake()` 不為 null 的零件。排水船體用 Michell 薄船，滑行船體用壓力片；尾浪的計算與畫面都在 AeroCore。
  - 受力箭頭（`DebugBody`）、存檔（藍圖、狀態、控制值）。
  - 藍圖之後無法編譯時（例如提供零件類型的 Mod 被移除），實體保留藍圖原樣存回，但不移動。
  - 表格（船體 Michell 表、螺旋槳四象限表、引擎 MAP 與運轉線）在背景執行緒預建（`Part.warmUp`），建好前載具原地不動。
  - 物理丟例外時（0.22.1）只停住那台：停在出錯子步開始時的姿態、靜止、傳動軸停轉、零件內部狀態回到初始值，不再推進；
    記錄一次 ERROR（實體 ID、藍圖名稱、位置、例外與堆疊），原因存在 `fault()` 並隨存檔保存（重新載入仍停著），伺服器與其他物體照常。
    `resume()` 從靜止重新開始。GameTest 執行中，這種失敗會讓載具所屬的測試失敗（見 [API](API.md)〈0.22.1 相容性變更〉）。
  - 拋棄零件（0.28）：
    - 零件以 `PartStep.jettison()` 要求，或由內容 Mod 呼叫 `castOff(ids)`。
    - 載具改用不含它們的藍圖（掛在它們上面的零件一起走），保持位置與運動（`VehicleState.separate`）；
      零件是同一份已編譯的零件（`PartTypes` 快取），所以不必等建表。
    - 坐在被拋零件座位上的乘客下車。
    - 之後送出 NeoForge 事件 `PartVehicle.Jettisoned`，內容 Mod 依它掉落物品。
  - 自有油箱（`Part.Tank.own`，0.28）：例如助推器的藥柱，只供給自己的零件。
  - 換外觀（0.31，`replaceBlueprint`）：只改外觀的藍圖（重漆、貼紙）換上去，狀態原樣接過（`VehicleState.restyled`），
    之後和沒換時逐位元相同（`RestyleTest`）。物理不同的藍圖會被拒絕；改設計要用 `refit`（0.34）或 `setBlueprint`（重設成靜止、加滿油箱）。
  - 搬到別處（0.32，`place`）：放到新位置、靜止，車上的油、引擎狀態、軸轉速都保留（`VehicleState.placed`）。
    組裝室把造好的載具推出門外用這個；`setBlueprint` 會把出廠時清空的油箱又加滿。
  - 改裝（0.34，`refit`）：物理可以不同的新藍圖（改造型、大小、調校）在原地換上，車子停在原處、速度歸零（`VehicleState.refitted`）。
    依零件 id 帶過內部狀態、顯示值（數量相同時）與軸轉速；油依油箱 id 帶過、截在新容量內，截掉的回報給呼叫端；新的油箱是空的；
    座位少了的乘客下車（`RefitTest`）。
  - 極高速（0.28，見 [HIGH_SPEED.md](HIGH_SPEED.md)）：碰撞走廊、路徑區塊票、區塊沒好時暫停並保留速度、客戶端外推。
- **`VehicleNetwork.ControlsPayload`**：客戶端送控制值給伺服器，`PartVehicle.canControl` 決定誰可以操作（預設是第一個座位上的人）。
- **`VehicleRegistry`**：實體類型 `aerocore:part_vehicle`，以及同步長字串藍圖用的資料序列化器（上限 4 MB）。
- 客戶端：
  - **`PartVehicleRenderer`**：逐零件用登記的 `PartRenderer` 畫（`PartRenderer.register(零件或設計類別, …)`，`PartMesh` 有方塊、球、任意軸的圓柱與圓錐、船體放樣與網格合併）。
    AeroCore 登記自己的執行時零件：放樣船體、浮筒球、方塊、會轉的螺旋槳、會偏轉的控制面。載具 Mod 登記自己的零件，或自己畫。
  - **`VehicleControls`**：AeroCore 的品管用按鍵對應。駕駛時前後鍵設定 `throttle` 為 ±1，左右鍵設定 `rudder` 為 ∓1（正值轉右舷）；
    載具 Mod 把 `usesDefaultControls()` 改回傳 false 就能關閉。
- **指令** `/aerocore vehicle`：
  - `spawn <名稱>`：讀伺服器目錄 `aerocore/blueprints/<名稱>.json`，在前方 4 m 生成，朝向視線方向。範例藍圖由載具 Mod 寫入（hdfv：`/hdfv vehicle samples`）。
  - `control <通道> <值>`、`info`、`debug <模式>`、`remove`：都作用於 64 格內最近的載具。`info` 會顯示因物理錯誤停用的原因。
  - `resume`：讓最近一台因物理錯誤停用的載具從靜止重新開始（0.22.1）。

## 藍圖格式（JSON）

```json
{
  "format": 1, "name": "launch",
  "parts": [
    {"id": "hull", "design": {"format": 1, "type": "hdfv:planing_hull", "name": "fridsma hull", "lengthM": 6, "chineBeamM": 1.2,
      "deadriseDeg": 20, "depthM": 0.75, "stations": 48, "radiationSegments": 6, "mass": {"massKg": 880}}, "position": [0, 0, 0]},
    {"id": "motor", "design": {"format": 1, "type": "hdfv:electric_motor", "name": "60 kW motor", "maxTorqueNm": 250, "maxPowerW": 60000,
      "maxSpeedRpm": 4000, "rotorInertiaKgM2": 0.05, "efficiency": 0.94, "massKg": 60}, "position": [1.2, 0.25, 0],
      "bindings": [{"role": "throttle", "channel": "throttle"}]},
    {"id": "rudder", "design": {"...": "..."}, "position": [-0.5, -0.22, 0], "rotation": {"yaw": 0, "pitch": 0, "roll": 0},
      "bindings": [{"role": "deflection", "channel": "rudder", "scale": 1}]}
  ],
  "connections": [
    {"type": "shaft", "from": "motor", "to": "screw", "ratio": 1.5, "efficiency": 0.96},
    {"type": "shaft", "from": "engine", "to": "gearbox", "toPort": "input", "ratio": 1, "efficiency": 1},
    {"type": "slipstream", "propeller": "screw", "surface": "rudder"}
  ]
}
```

- 數字以最短可往返的十進位寫出，讀回的 double 完全相同；比新版本 AeroCore 更新的 `format` 會被拒絕。
- 材質寫成 `{"name": "steel", "density": 7850}`。標準材質只寫名稱也可以。
- 0.17～0.19 的類型 ID 是 `aerocore:*`；0.20 起由 hdfv 提供 `hdfv:*`，舊藍圖要改類型 ID 才讀得回來。

## 驗證（AeroCore）

| 測試 | 內容 |
| --- | --- |
| `VehicleTest.floatsReproduceFloatingBodyDynamicsWithRadiation` | 四個浮筒（`SpherePart`）組成的平台在斜浪中 2 s（含輻射記憶）與 `FloatingBodyDynamics`＋同樣的輻射記憶逐步相同（1e-9） |
| `hullReproducesHullDynamicsAndDoesNotCareHowItIsMounted` | 測試船的斷面做成 `HullPart`，1.5 s 與 `HullDynamics` 相同（1e-9）；同一船體轉 70° 安裝、載具反轉回來，世界中的運動相同 |
| `planingHullReproducesPlaningDynamics` | Fridsma 船體（`PlaningHullPart`）以 8 m/s 滑行 1 s，與 `PlaningDynamics`＋淡出的輻射相同（1e-9），通氣 > 0.5 |
| `motorAndScrewSettleWhereTorquesBalance...` | 馬達經 1:1 與 2:1（效率 0.95）齒輪驅動 B4-70 槳繫柱：穩態時馬達扭矩 = 槳扭矩 ÷ 齒比 ÷ 效率（0.2%），推力 = K_T(0) ρ n² D⁴（1%），船體承受的反扭矩 = 槳扭矩 |
| `anUnloadedMotorRunsUpToItsTopSpeedAndStays` | 空載馬達升到最高轉速；轉子加船體的角動量守恆（1e-6） |
| `rudderPushesTheSternToPort...` | 正舵把艉推向左舷、船艏轉右舷；轉 90° 安裝成水平面時，同樣大小的力變成垂直 |
| `aClosedBoxFloatsAtItsArchimedesDraft` | 2×1×1 m、800 kg 的方塊在計算吃水 ± 偏移處，淨力 = (ρ_w − ρ_a) g A × 偏移 |
| `massPropertiesFollowTheParallelAxisTheorem`、`mountsTurnAndMovePartMass` | 平行軸定理手算、薄殼與實心方塊的閉式解；轉 90° 安裝時慣性主軸互換 |
| `BlueprintTest` | JSON 往返（安裝角度、綁定、連接）、不合物理的值附路徑並阻止編譯、外推警告、輸入名稱檢查、零件類型 ID 規則、未知類型與新格式拒絕 |

這些測試用測試原始碼裡的 `TestParts`（AeroCore 的執行時零件配上最小的設計）。零件設計本身、範例與載具 GameTest 在 hdfv。

AeroCore 只留一個載具 GameTest（0.22.1）：`VehicleGameTests.aPhysicsErrorStopsOnlyThatVehicle`。兩顆測試球（只在 GameTest 登記的零件類型
`aerocore:gametest_ball`，`SpherePart` 加上一個「fail」輸入，設定後 `loads` 丟例外）從 7 m 落下；其中一顆在半空中出錯。
伺服器繼續運作，出錯的球停在原處（靜止、`fault()` 帶原因、存檔讀回仍停著），另一顆照常落到地面；清掉輸入、`resume()` 後出錯的球也落下。
它也確認 `BodyWorld.owningTests` 找到的就是這個測試（沒有 `EXPECTED_FAULT_TAG` 時，失敗會記在它身上）。

## 尚未做的

- 離合器與變速箱、輪子與懸吊、機翼與旋翼、增壓：零件設計在 hdfv，所需的計算補在 AeroCore。
- 零件之間的流體交互作用只有「控制面在螺旋槳尾流中」；機翼下洗到尾翼、船體伴流打在槳上、多船體間的波浪干擾（輻射記憶以外）都沒有。
- 方塊體在浪中的附加質量與輻射；船體只能水平安裝（垂直輻射記憶以載具水平面量點距，傾斜安裝的船體會被當成另一種排列）。
- 軸沒有扭轉彈性（0.22 起有單向離合器與陀螺力矩；0.24 起零件可用約束列做離合器、煞車、齒輪、差速器，見 [DRIVETRAIN.md](DRIVETRAIN.md)）。
- 結構強度、損壞、成本、耐久：屬於遊戲平衡，由載具 Mod 決定。
- 遊戲中的實際外觀與手感：見 AGENTS.md 待實測清單。
