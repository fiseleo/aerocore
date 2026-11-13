# 零件的狀況、損壞的計算與貨艙（0.42）

零件會磨損、會被撞壞，載具會載貨。AeroCore 只提供通用的部分：狀況存在哪裡、怎麼存檔同步、零件怎麼讀寫它、
損壞的公式，以及貨物的質量怎麼進物理。哪個零件有哪些狀況、撞多大力壞多少、怎麼修、貨艙裝什麼，都是載具 Mod 的規則
（hdfv 0.25 的損壞、修理廠、儲物箱）。

## 狀況項目

零件用 `Part.conditions()` 宣告 0 到數個具名項目（車軸的 `tyres`、`brakes`、`suspension`，機翼的 `structure`）。
每個項目在 `VehicleState` 有一個數字：**已用掉的比例**，0 是全新，1 是失效。

| 誰 | 做什麼 | 方法 |
| --- | --- | --- |
| 零件，在 `loads` | 讀這一步開始時的值，依它降低性能 | `PartStep.used(i)` |
| 零件，在 `loads` | 累積這一步的磨損（壽命的比例，不可為負） | `PartStep.wear(i, 增量)` |
| 零件，在 `after` | 依這一步實際傳的扭矩、力、轉速累積磨損（離合器、煞車、輪胎） | `PartAfter.used(i)`、`PartAfter.wear(i, 增量)` |
| 遊戲，在兩步之間 | 外來的損壞（撞擊、腐蝕、攻擊）與修理 | `VehicleState.setUsed`、`addUsed`；遊戲端 `PartVehicle.setUsed`、`addUsed` |
| 任何人 | 讀 | `VehicleState.used(零件 id, 索引或名稱)`、`worstUsed()`；`PartVehicle.used`、`worstUsed`（兩端） |

約定：

- 預設沒有項目：沒宣告的零件永遠不會壞，載具的每一步和 0.41 逐位元相同。
- 有項目的零件，在所有項目都是 0 時也必須和 0.41 逐位元相同（用 `if (used > 0)` 分支，不要乘一個剛好是 1 的係數）。
- 磨損只影響之後的步：`PartStep.wear` 的增量在 `VehicleDynamics.finish` 才加進狀態；`prepare` 與 `finish` 之間遊戲設的值不會被蓋掉
  （`finish` 是把增量加在當時的值上）。值永遠在 0～1。
- 索引超出零件項目數時，`used` 回傳 0、`wear` 不做事。包住別的零件的零件（hdfv 的變速箱包 `TransmissionPart`）把內層的項目放在
  自己清單的最前面，內層就能照自己的索引讀寫；內層沒被宣告的項目讀到 0，表現成全新。
- 一個零件的項目名稱不能重複（編譯載具時檢查）。

### 存檔、同步、跟著載具走

- 存檔：`VehicleState.toJson` 的 `"condition": {零件 id: {名稱: 值}}`，只寫大於 0 的；全新的載具不寫這個鍵，存檔和 0.41 一樣。
  讀取時依零件 id 與名稱對，所以藍圖改了（零件換順序、項目增減）也讀得回來；0.41 以前的存檔沒有這個鍵，全部是 0。
- 狀況不放在零件的內部狀態（internals）：內部狀態在數量改變時整台重設、物理出錯時回初始值，也不同步；狀況屬於硬體，不該跟著消失。
- `placed`、`restyled`、`refitted`、`separate` 都依零件 id 與名稱帶過去（`ConditionTest.carriedWhereverTheStateGoes`）。
  物理出錯後重建的狀態用 `VehicleState.keepHardware(舊狀態)` 接回（`PartVehicle.physicsFailed` 已這樣做）。
- 同步：`PartVehicle` 的 DISPLAY 加 `"u": [百分比…]`（安裝順序、整數百分比），只在有東西不是 0 時寫；客戶端讀到的是百分比的精度。

## 損壞的計算（`api.damage`）

純 Java、只有公式與有出處的典型值，可離線呼叫。哪個零件多耐撞、壽命多長、遊戲的一年多快，由呼叫者給。

| 類別 | 內容 |
| --- | --- |
| `ImpactDamage` | 撞擊留在結構裡的能量 E = J v / 2 × (1 − (v₀/v)²)（J 法向衝量、v 接近速度、v₀ 彈性速度 4 km/h）；能量依 m / (1 + (d/r)²) 分給各零件；每個零件用掉的比例 = 分到的能量 ÷（韌性 × 質量）；爆炸的衝量能量 (iA)²/2m |
| `Wear` | 摩擦片（Archard 的能量形式：體積 = 比磨耗率 × 摩擦功）與高溫加速；胎面（滑動摩擦功 × 可磨性 × 嚴苛度）；運轉壽命（時間 ÷ 壽命 × 轉速比ⁿ × 負載比ᵐ）；Larson-Miller 潛變壽命；Miner 法則與 Basquin 線的疲勞 |
| `Corrosion` | 七種暴露（乾燥室內、空氣、淋雨、海風、淡水、海水、飛濺帶）× 八種金屬的均勻腐蝕速率（mm／年）；塗層；壁厚用掉的比例 |
| `Casualty` | 乘員的速度變化 Δv = J/m 與嚴重受傷（MAIS 3+）的風險曲線；水線下破洞的進水量（Torricelli） |

### 來源與量級

- 撞擊能量：完全非彈性的撞擊中，質量 m 的物體停下來損失 m v²/2、法向衝量 J = m v，所以 E = J v / 2 與後面的質量無關。
  彈性速度取 4 km/h（UN R42 的保險桿）。
- 韌性是「把整個零件用掉」每 kg 要的能量，不是材料在理想壓潰下的比吸能（軟鋼薄壁管 15～30 kJ/kg、鋁擠型 20～40、碳纖管 50～100；
  Lu & Yu 2003）：只有受力路徑上的結構會摺疊，而且離理想很遠。整車的量級：1400 kg 的車在 64 km/h 偏置撞擊帶 221 kJ，
  大約用掉 330 kg 車身殼的前四分之一，等於每 kg 被用掉的車身 2.7 kJ（`SHELL_STEEL_J_PER_KG`；由這幾個整數估的量級，不是量測）。
- 能量的分配 m / (1 + (d/r)²) 是規則：質量大又靠近撞擊點的結構擋下其他部分；r 約是零件尺寸的一半。
- 煞車來令片的比磨耗率 10～30 mm³/MJ（Limpert, Brake Design and Safety）；取 20 時，1400 kg 的車從 100 km/h 煞停一次用掉一組來令片的
  1/18 500。約 200～250 °C 以上樹脂黏結劑分解，磨耗指數上升（`liningHeat`）。
- 胎面橡膠的磨耗 10～100 mm³/kJ 摩擦能（Grosch 2008）；取 20、一般行駛每公里 2 kJ 滑動功時，一條胎約 4 萬公里。
  滑動速度高時每焦耳磨掉的橡膠急升（`treadSeverity`：1 + (v/v₀)²，鎖死的輪子 30 m/s 時是一百倍）。
- 軸承壽命的負載指數 3（滾珠）或 10/3（滾子），ISO 281。
- Larson-Miller：T (C + log₁₀ t) 在同一應力下相同，C = 20；1200 K 附近每多 15～20 K 壽命減半。
- 腐蝕：碳鋼在空氣中是 ISO 9223:2012 的第一年速率（C2 到 25 µm／年、C5 80～200）；浸在海水 0.1～0.15 mm／年、飛濺帶 2～3 倍、
  淡水約一半（Uhlig's Corrosion Handbook）。其他金屬是同類手冊的典型值（ASM Handbook 13B），以「造成同樣損害的均勻損失」表示：
  鋁、不鏽鋼、鈦靠氧化膜，壞的時候是孔蝕；鎂在海水裡很快。都是量級。
- 乘員：繫安全帶的前座在正面撞擊，嚴重受傷的風險一半約在 Δv 60～65 km/h、十分之一約在 35～40 km/h（美國事故樣本擬合的風險曲線，
  例如 Kononen 等 2011）；`seriousRisk` 是通過這兩點的邏輯曲線，取整數的錨點，不是某一篇的擬合。風險怎麼換成扣血是遊戲的規則。

測試：`DamageTest` 10 個（手算與上面的量級）。

## 貨艙

`Part.Hold(name, capacityM3, centre)` 是零件上的一個貨艙；`Vehicle.holds()` 把它們列出來，id 是 `零件 id.名稱`。
`VehicleState.setHold(索引, kg)` 設定裡面東西的質量，`VehicleDynamics` 把它當成貨艙中心的點質量加上重量，和油箱的油一樣
（`RigidBodyDynamics.PointMass`：隱式速度解與漂移都含它）。

- AeroCore 不知道貨物是什麼：物品清單、每種物品多重、放不放得下，都是遊戲的。質量不受容積限制：超載的後果由物理表現（下沉、加速變差）。
- 新載具的貨艙是空的。存檔 `"holds": {貨艙 id: kg}`，只寫大於 0 的；`placed`、`restyled` 照抄，`refitted`、`separate` 依貨艙 id 帶過去。
- 同步：DISPLAY 的 `"h": [kg…]`（整數公斤），只在有貨時寫。遊戲端 `PartVehicle.holdCount`、`holdKg`、`setHold`。
- 設計時算進零件質量的東西（hdfv 車身的設計載重）和貨艙裡的東西是兩回事，不要兩邊都算。

測試：`ConditionTest.cargoWeighsAndAcceleratesWithTheVehicle`（貨和車一起以 g 落下、不產生力矩；通過合質心的 300 N 讓 150 kg 加速 2 m/s²；
回報的重量是車加貨）。

## 遊戲端

- `PartVehicle.Hurt`（NeoForge 事件，伺服器）：原版的 `hurt()` 轉出來的（玩家或怪物攻擊、箭、爆炸）。AeroCore 不做任何事；
  監聽者依來源損壞零件後呼叫 `take()`，`hurt()` 才回傳 true。0.41 以前載具對這些完全沒反應，沒有監聽者時仍然如此。
  火與岩漿不會到這裡：載具的實體類型是 `fireImmune()`（原版對防火的實體不呼叫火與岩漿的 `hurt`）。要燒壞零件的載具 Mod 自己看零件所在的方塊
  （hdfv 0.25 的 `VehicleDamage` 每秒檢查一次）。
- 遊戲端連續呼叫好幾次 `VehicleState.setUsed` 之後，要讓客戶端看到，呼叫一次 `PartVehicle.setUsed` 或 `addUsed`（量 0）：
  只有 `PartVehicle` 的方法會送出同步。
- 撞擊仍是 0.37 的 `VehicleContacts.Contacted`（位置、法向衝量、接近速度、材質）：把它交給 `ImpactDamage` 就是碰撞損壞。

## 執行時零件

`api.vehicle.part` 的零件可以磨損，預設關閉：`零件.wearing(X.Wearing)` 回傳一份會磨損的複本（給 null 是關掉），沒有 `Wearing` 的零件
`conditions()` 是空的。原本的建構子都不變。沒有 `Wearing`，或有 `Wearing` 但項目都是 0 時，走原本的程式（用分支，不是乘 1.0）。
`Wearing` 裡的體積、壽命、額定值是載具 Mod 的規則；`typical(...)` 給有出處的預設。

| 零件 | 項目（索引） | 自己的磨損 | 效果（u 是已用掉的比例） |
| --- | --- | --- | --- |
| `AxlePart`、`LegPart`（共用 `WheelCorner.Wearing`） | `tyres`(0)、`brakes`(1)、`suspension`／`strut`(2) | 胎面：`Wear.tread(滑動摩擦功, 可磨性, treadSeverity(滑動速度), 胎面體積)`；來令片：`Wear.lining(煞車熱) × liningHeat(碟盤溫度)`；懸吊：擋塊力超過額定時 `Wear.running` | 抓地 1 − (1 − w)u，乾地與冰 w = 0.9、濕地鬆土雪 0.6；u ≥ 1 爆胎：抓地 0.4、滾動阻力 5 倍。煞車到 0.8 不變，之後線性降到 1.0 時的 0.3（手煞車不降）。阻尼 1 − 0.8u。起落架支柱 u ≥ 1 塌陷：彈簧與擋塊力剩 0.1 |
| `TransmissionPart` | `gears`(0)、`clutch`(1) | 離合器（手排離合器、自排的換檔元件與鎖止、雙離合、CVT 的鎖止與皮帶）：Σ\|列扭矩 × 滑差\|·dt 經 `Wear.lining`；齒輪：每個齒輪列 `Wear.running`（扭矩、輸入軸轉速；指數 1 與 3，ISO 281） | 離合器容量 1 − 0.85u²（0.5 時 0.79、1 時 0.15）；齒輪效率 1 − 0.1u，u ≥ 1 不再有齒輪、元件、同步器、皮帶的列（變扭器、離合器、P 檔棘爪仍在） |
| `TransferPart` | `gears`(0) | 檔位列的扭矩與輸入轉速，`Wear.running` | 同變速箱的齒輪 |
| `MotorPart` | `motor`(0) | 給出的扭矩與轉速，`Wear.running` | 扭矩與它的斜率 × (1 − u) |
| `WingPart` | `structure`(0) | 無（外來的損壞） | 沿氣流的阻力 × (1 + 2u)，其餘的力與整個力矩 × (1 − 0.7u) |
| `ControlSurfacePart` | `structure`(0) | 無 | 偏角 × (1 − u)；1 時停在中立，仍有自己攻角的負載 |
| `AirPropellerPart`、`RotorPart`、`PropellerPart` | `blades`(0) | 無 | 推力（旋翼：槳轂力與力矩）與給下游的滑流 × (1 − 0.6u)，u ≥ 1 剩 0.1；吸收的扭矩不降 |
| `BodyPart` | `structure`(0) | 無 | 多一份空氣阻力 0.3 u × 原阻力；水的負載、空氣套件、扭轉不變。另有帶 `List<Part.Hold>` 的建構子（貨艙） |
| `FramePart` | `structure`(0) | 無 | 扭轉剛度整個矩陣 × (1 − 0.6u)（對稱、列和為零）；剛性車架只帶著這個項目 |

`SpherePart`、`HullPart`、`PlaningHullPart`、`BoxPart`、`MassPart` 沒有狀況項目。

為了這些效果加的介面：`Part.frameStiffness(PartStep)`（預設 1；只在車架零件有狀況項目時才問）、`FrameFlex.solve(..., share)`、
`Suspension.step(..., dampingShare, springShare)`、`Suspension.stopForce`、`TyreContact.step(..., grip)`、`WheelCorner` 的 `Worn` 與對應的多載。

包住別的零件的零件要轉送 `conditions()`（內層的排最前面）、車身的 `holds()`、車架的 `frameStiffness()`。

測試（`api/vehicle`，28 個）：`WheelWearTest` 10、`DrivelineWearTest` 10、`StructureDamageTest` 8。每個零件都比對
「沒有 `Wearing`」與「有 `Wearing`、每步把項目歸零」兩台載具的 `toJson` 逐位元相同；磨損率對手算（鎖死煞停的胎面在 3 % 內、
煞停的能量在 2 % 內、離合器滑差功在 0.5 % 內）；效果在 0.5 與 1.0 的大小；全部用到 1 時跑幾秒不出錯。

### 模型的限制

- 輪胎：一根車軸一個項目；只改抓地與（爆胎後的）滾動阻力，不改剛度、半徑、水漂速度。典型數字下，從 20 m/s 鎖死煞停一次約用掉 4 % 的胎面。
  摩擦功用步初的值；輪子是慢慢停轉而不是一開始就鎖死時，一部分能量進煞車，胎面磨得少很多。
- 減速度不會剛好隨抓地等比：抓地少、前移的載重少，輕載的輪胎抓地係數較高（爆胎煞停量到新胎的 0.43，不是 0.40）。
- 煞車：手煞車的滑動也磨來令片；高溫係數用碟盤的集總溫度。
- 懸吊：壓在擋塊上的彈跳阻尼很差，重落地後擋塊的磨損可能持續幾秒。
- 變速箱：同步器與變扭器不磨損；齒輪到 1 之後排檔桿仍顯示檔位。
- 旋翼：入流仍照沒損壞的推力算。機翼的損壞左右對稱（自己不產生滾轉）。
- 沒有凍結的 0.41 參考值：沒有 `Wearing` 的零件與 0.41 逐位元相同，是靠舊呼叫經 null 走原路徑與既有的容差測試（`VehicleTest` 1e-9）保證的。
