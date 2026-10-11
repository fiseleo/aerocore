# 噴射引擎（0.19）

使用者 2026-10-11 列出噴射引擎的元件「進氣道、壓縮機、燃燒室、渦輪、噴嘴／排氣尾管」，並要求「開始做噴射引擎」。
0.19 把它們做成各自獨立設計的元件，再加一個燃油控制，組成一具單軸渦輪噴射引擎零件（`aerocore:jet_engine`）。
引擎沿零件 +X 推（進氣口在前、噴口在後）；轉子繞 +X 轉，所以加速轉子時載具會受反扭力。

## 元件與參數

每個元件都是玩家自己的藍圖（`EngineComponent`，與 0.18 活塞引擎相同的格式）：參數自由，
各元件自己檢查不合物理的值、標示外推，並依幾何與材料估算質量（也可以直接給質量）。

| 元件 | 類別 | 參數 |
| --- | --- | --- |
| 進氣道 | `JetInletDesign` | 皮托式（超音速時有正震波）／外壓縮斜板（MIL-E-5008B）、捕獲面積、次音速總壓恢復、長度、材料 |
| 壓縮機 | `CompressorDesign` | 軸流／離心、級數、設計壓比與流量（海平面靜止）、多變效率、設計點喘振裕度、葉尖直徑、進口輪轂比、設計轉速、葉輪葉片數（離心）、材料 |
| 燃燒室 | `CombustorDesign` | 總壓損失、燃燒效率、長度、外殼直徑、材料 |
| 渦輪 | `TurbineDesign` | 級數、多變效率、葉尖直徑、葉片冷卻效果與冷卻空氣比例、轉子機械效率、材料 |
| 噴嘴／排氣尾管 | `JetNozzleDesign` | 收斂或收斂-擴張（固定出口面積比）、總壓保持、尾管長度、材料（要耐得住渦輪出口溫度） |
| 燃油控制 | `JetControlDesign` | 燃料（煤油、汽油、甲醇、丙烷）、渦輪入口溫度（設計點，也是控制上限）、怠速與最高轉速、是否有加速排程與保留的喘振裕度、起動機扭力、點火轉速 |

材料（`EngineMaterial`）加了使用溫度 `serviceTemperature`，新增不鏽鋼 321（1088 K）、Inconel 718（923 K）、Inconel 713C（1253 K）、
CMSX-4 單晶（1373 K）。舊材料也補上使用溫度；JSON 沒寫時預設 600 K。

### 檢查（INVALID：不能運作）

- 軸流壓縮機每級的功除以葉尖速度平方（級負荷）超過 1：沒有葉片能把氣流轉那麼多。實務上約 0.2～0.45（Dixon & Hall），超過時標示外推。
- 離心葉輪的功超過 σU²（徑向葉片、無預旋；Stanitz 滑移係數 σ = 1 − 0.63π/Z）。
- 壓縮機進口（軸流的環面、離心的葉輪眼，取葉尖直徑的 0.6）或進氣道捕獲面積的修正流率超過 241 kg/(s·m²)：在設計流量前就阻塞。設計值約 Mach 0.5～0.7（219 以下）。
- 轉盤中心應力 (3 + ν)/8·ρU²（Roark，旋轉實心圓盤，ν = 0.3）超過降伏強度；超過三分之二時標示外推。
- 渦輪葉片金屬溫度 T4 − ε(T4 − T3) 超過材料使用溫度（Saravanamuttoo 等：冷卻效果 ε，冷卻空氣取壓縮機出口）。
- 渦輪級負荷（每級功除以葉尖速度平方）超過 4；實務上到約 2.5（Smith 圖），超過時標示外推。
- 排氣尾管材料的使用溫度低於渦輪出口溫度。
- 循環不閉合：渦輪推動壓縮機後，已經沒有壓力把排氣推出噴嘴。

## 模型（`api.engine.Turbojet`）

### 氣體

- 空氣與煤油燃燒產物用 0.18 的 NASA 多項式（`Species`／`Gas`），變比熱。稀薄混合氣的產物是「空氣」與「理論空燃比產物」的質量混合。
- `Gas` 加了熵函數 φ(T) = ∫c_p/T dT（以每 1 K 表格的 c_p 精確積分）：等熵關係 ln(p₂/p₁) = (φ₂ − φ₁)/R。
- 多變過程：壓縮 ln π = e(φ₃ − φ₂)/R，膨脹 ln π = (φ₄ − φ₅)/(eR)。
- 燃料空氣比：用燃燒室的能量守恆求閉合解（燃料以 298 K 進入，燃燒效率乘在低熱值上）。新燃料 `Fuel.KEROSENE` 是 C12H23，低熱值 43.2 MJ/kg。

### 設計點（`size`）

由壓縮機的壓比與流量、渦輪入口溫度和各元件效率算出完整循環，再決定兩個固定面積：

- **渦輪的流通面積**：渦輪被當成跨越自身壓比的噴嘴，壓比約 1.85 以上時阻塞（導向葉片阻塞；單級渦輪實際上稍晚才阻塞），以下則按次音速流量計算。
  這是 Stodola 錐定律的噴嘴形式，讓低壓比引擎與低推力也能匹配。
- **噴嘴喉部**：阻塞；壓比太低時（模型噴射引擎）以設計點的次音速排到大氣來定。

冷卻空氣（壓縮機出口的一部分）繞過燃燒室與渦輪，在噴嘴前混回。

### 非設計點的平衡（`equilibrium`）

給定渦輪入口溫度 T4，求渦輪壓比，使渦輪流量、噴嘴流量與轉子功率三者同時平衡（Mattingly《Elements of Propulsion》第 8 章、
Seitzman《AE4803 Engine off-design performance》的單軸匹配）。

- 效率固定，沒有元件特性圖。轉速由壓縮機功依 Euler 方程（速度三角形不變）得出：N ∝ √(w_c / w_c,D)。
- 進氣道、燃燒室、尾管的總壓損失以設計點為準，非設計點隨進入的修正流量平方（動壓）縮放
  （Saravanamuttoo 等《Gas Turbine Theory》的壓損係數；燃燒室的熱損失比例維持設計值）。轉得慢時幾乎沒有壓損，才點得著、轉得起來。
- 壓縮機進口的修正流量與 pt2 無關（在式中相消），所以進氣道損失直接在平衡式內求，不需要外圈迭代
  （外圈迭代會在兩個根之間來回跳，0.19 開發時在模型噴射引擎遇到）。
- 匹配式有兩個根：靠近渦輪壓比 1 的退化根（轉子幾乎不轉、流量卻很大）與物理根。求解時從最大壓比往下掃描，找到第一個變號區間再二分。

### 運轉線（`runningLines`）

- 在 11 個飛行馬赫數（0～3）各掃一條平衡運轉線，以修正量記錄：修正轉速、壓比、修正流量、修正油量、T4。
  噴嘴阻塞時各條線幾乎重合；噴嘴未阻塞（低推力、低壓比引擎）時衝壓會改變運轉線。兩條線之間依衝壓總壓比 (1 + 0.2M²)^3.5 線性內插，
  反函數用割線法求，3 馬赫以上用 3 馬赫的線。
- T4 範圍從 1.15·Tt2 到「設計 T4 × 1.08 換算到最冷標準大氣（216.65 K）的修正值」，所以高空寒冷時的高修正轉速也在表內。
- 表格從 `MIN_LINE_SPEED`（設計轉速的 35 %）或轉子能自持的轉速開始，以下用相似律外推（壓升 ∝ N²、流量與油量 ∝ N）：
  固定效率描述不了遠低於設計轉速的壓縮機。

### 暫態（`quasiSteady`）

- 壓縮機特性採用平衡解隱含的那一種：轉速決定功與壓比，流量由渦輪決定。
- 渦輪在運轉線壓比下的流量，隨流量參數 flux(T4)/(1 + f) 縮放（約 1/√T4）。
  - 比運轉線熱的氣體，在同一壓比下流量較小，因此朝喘振線移動。所用喘振裕度 = 壓比 ÷ 運轉線在該流量時的壓比 − 1。
  - 比運轉線冷的氣體，流量維持在運轉線上（壓縮機阻塞）。
- 渦輪與噴嘴依實際氣體匹配；渦輪功率減去壓縮機功率，就是讓轉子加速的功率。給運轉線上的油量時，結果就是平衡解。
- 控制用的反解：
  - `lineFuel`：運轉線油量。
  - `fuelForTurbineInlet`：讓 T4 等於某值的油量。
  - `fuelForSurgeMargin`：用掉某個喘振裕度時的油量。

### 執行時（`JetEngineDesign` 的零件）

- 輸入：`throttle` 0～1、`ignition`（沒綁定 = 開）。
- 內部狀態：
  - 油量（kg/s）。
  - 燃燒狀態：0 熄火、1 已點火且起動機仍在幫忙、2 自持運轉。
  - 喘振剩餘時間。
- 燃油控制（規則，不是量測）：
  - 油門把目標轉速設在怠速與最高轉速之間。
  - 油量 = 目標轉速的運轉線油量，加上比例修正（每單位轉速誤差給 1.5 倍設計油量）。
  - 上限：T4 不超過控制的渦輪入口溫度；有加速排程時，喘振裕度最多用到「壓縮機裕度 ×（1 − 保留）」。
  - 下限：運轉線油量的一半（貧油熄火）。
  - 超速 1 % 以上時只給下限。
  - 油閥以 0.1 s 落後跟上。
- 起動：點火開著時，起動機扭力把轉子帶到點火轉速並持續幫忙，45 % 轉速後永久切離（熄火後才會再接合）。
- 熄火：轉速低於 10 %，或進氣口在水面下。
- 喘振：所用裕度超過壓縮機的喘振裕度時，0.6 s 內流量與功率剩 30 %（規則）。
- 轉子扭力 = (P_turbine − P_compressor)/ω，加上起動機扭力，以隱式斜率交給傳動系。推力（含衝壓阻力）作用在噴口，沿零件軸向。
- 空氣取自進氣口那一點（溫度、壓力、相對風速）。
- 每個設計的運轉線在背景建立（`Part.warmUp`，約 0.2 s），並依設計 JSON 快取。
- 每一小步約 60～70 µs。
- 診斷值：speedFraction、rpm、thrustN、fuelKgPerS、turbineInletK、exhaustK、pressureRatio、airflowKgPerS、surgeMarginUsed、lit、starter、surging。

### 質量（估計）

| 部位 | 估計方式 |
| --- | --- |
| 壓縮機 | 軸流每級 0.03ρD³、離心每級 0.08ρD³，含轉子、靜子與機匣；轉子佔 20 % |
| 渦輪 | 每級 0.04ρD³，含轉子、導向葉片與機匣；轉子佔 25 % |
| 燃燒室、進氣道、尾管 | 殼體，厚度為直徑的 0.4 %、至少 0.3 mm |
| 附件（燃油控制與泵、起動機、齒輪箱、滑油系統） | 0.5 kg 加每 kg/s 流量 1.2 kg |

係數是用兩具真實引擎一起定的：J85-GE-17A 乾重 181 kg、長 1.03 m（Smithsonian 館藏資料），JetCat P80 含起動機 1.318 kg。
範例估出 179 kg 與 1.26 kg。

## 驗證

| 測試 | 內容 |
| --- | --- |
| `TurbojetTest.theEntropyFunctionGivesTheIsentropeOfAnIndependentIntegration` | 熵函數的等熵溫度，對直接用物種多項式的 RK4 積分（0.05 K 內）；c_p 是焓的斜率 |
| `seitzmansWorkedExampleIsReproducedOnDesignAndInCruise` | Seitzman 例題（壓比 25、70.6 kg/s、T4 1773 K）：見下表 |
| `theEquilibriumAtTheDesignTemperatureIsTheDesignPoint` | 三具引擎（含噴嘴未阻塞的模型引擎）在設計 T4 的平衡解就是設計點（1e-6），且轉子功率平衡 |
| `realTurbojetsAtSeaLevelStatic` | J79-GE-17 推力 ±5 %、TSFC ±10 %；J85-GE-21 推力 ±15 % |
| `inletRecoveryLosesANormalShockOrFollowsTheMilSpec` | Mach 2 正震波 0.7209、Mach 1.1 為 0.9989（正震波表）；MIL-E-5008B 在 Mach 2 為 0.925；次音速損失隨流量平方 |
| `theRunningLinesPassThroughTheDesignPointAndRiseWithSpeed` | 運轉線通過設計點、各欄隨轉速單調上升、低到 45 % 以下；模型引擎在 Mach 0.6 的運轉線流量較大（衝壓） |
| `quasiSteadyBalancesOnTheRunningLineAndOverFuellingHeadsForSurge` | 兩具範例在靜止與 100 m/s、70 % 與 100 % 轉速：運轉線油量功率平衡（1 % 內）；多 10 % 油會加速並朝喘振移動、少 10 % 會減速；T4 與喘振裕度的反解 |
| `aStoppedEngineMakesNoThrustAndAConvergentDivergentNozzleHelpsAtHighPressureRatio` | 停轉沒有推力；Seitzman 設計點用面積比 1.5 的擴張噴嘴推力多 3 % 以上；面積比 3 在噴嘴壓比約 2.4 時分離、以大氣壓排出 |
| `JetEngineTest.theSampleTurbojetsAreValidRoundTripAndWeighLikeTheirKind` | 兩具範例有效、JSON 往返；質量與長度對 J85-GE-17A、JetCat P80（25 % 內） |
| `enginesThatCannotWorkAreRejected` | 2 級壓比 8、轉速 2.4 倍降伏、1500 K 燒壞 713C 葉片、700 K 循環不閉合、鋁尾管、進氣道太小、單級小渦輪 |
| `bothSamplesStartIdleRunUpToDesignThrustAndSpoolDown` | 起動→自持→怠速 ±3 %、全油門到設計轉速與推力（1 % 內）、T4 守住上限、不喘振、推力沿軸向、收油回怠速、內部狀態存檔、關點火後熄火停轉 |
| `withoutItsAccelerationScheduleASlammedThrottleSurges` | 沒有加速排程時，猛推油門會喘振，之後恢復到全速 |
| `aDrownedInletPutsTheFlameOut` | 進氣口沒入水中就熄火、沒有推力 |
| GameTest `jetBoatLightsItsTurbojetAndDrivesAhead` | 範例噴射艇（測試船體＋J85 級引擎在後甲板）在遊戲中自己起動、怠速推力超過 1 kN、推船前進 |

### Seitzman 例題

講義用固定比熱（壓縮 γ 1.4、膨脹 1.3），而且不計燃料質量；本模型用變比熱並計入燃料，所以設計點推力高 7.6 %。
巡航相對於設計點的變化（推力比）兩者一致。

| 項目 | 本模型 | 講義 |
| --- | --- | --- |
| 設計推力 | 76.4 kN | 71 kN |
| 設計排氣速度 | 672 m/s | 665 m/s |
| 設計 Tt5 | 1388 K | 1330 K |
| 巡航壓比 | 20.95 | 21.2 |
| 巡航流量 | 31.5 kg/s | 31.7 kg/s |
| 巡航修正轉速 | 5744 rpm | 5754 rpm |
| 巡航推力 | 25.5 kN | 24 kN |
| 巡航排氣速度 | 647 m/s | 641 m/s |
| 巡航／設計推力比 | 0.334 | 0.338 |

### 真實引擎（海平面靜止）

e_c 0.87、e_t 0.88、π_b 0.95、η_b 0.98、π_n 0.98、進氣恢復 0.99。

| 引擎 | 推力（本模型） | 推力（公布） | TSFC（本模型） | TSFC（公布） |
| --- | --- | --- | --- | --- |
| J79-GE-17 | 53.0 kN | 52.8 kN | 22.3 g/(kN·s) | 23.8 g/(kN·s) |
| J85-GE-21 | 17.7 kN | 16.0 kN | 26 g/(kN·s) | 不測（資料來源互相矛盾，35 或 27） |

J85 推力高 11 %，因為小引擎實際的元件效率比給的低。

### 範例

| 項目 | J85 級（`SampleEngines.turbojet`） | 模型引擎（`SampleEngines.modelTurbojet`） |
| --- | --- | --- |
| 構造 | 8 級軸流、壓比 8、20 kg/s、T4 1250 K、2 級輕度冷卻渦輪、收斂噴嘴 | 1 級離心、壓比 2.2、0.23 kg/s、112000 rpm、T4 950 K、未冷卻單級渦輪 |
| 海平面靜止推力 | 14.28 kN | 69.9 N（噴嘴未阻塞） |
| TSFC | 26.2 g/(kN·s) | 50.1 g/(kN·s) |
| 質量 | 179 kg | 1.26 kg |
| 長度 | 1.09 m | 0.30 m |
| 起動到怠速 | 約 4 s | — |
| 怠速 | 55 %，1.58 kN | 35 %，6.8 N，排氣 781 K |
| 怠速到全推力 | 約 4 s | — |
| 飛行中全推力 | 100 m/s 12.93 kN；250 m/s 12.88 kN；11 km、250 m/s 5.60 kN（平衡解 5.56 kN） | — |
| 沒有加速排程時猛推油門 | 喘振約 2.4 s | 先受 T4 上限限制，不會喘振 |
| 參考的真實引擎 | J85-GE-17A：12.68 kN、181 kg | JetCat P80：93.5 N、123000 rpm、全推力 256 g/min（45.6 g/(kN·s)）、1.318 kg；P80 SE：怠速 3 N、排氣 510～700 °C |

## 限制

- 只有單軸渦輪噴射：沒有後燃器、旁通（渦扇）、雙軸、渦軸／渦槳、可調噴嘴、可調靜子或放氣閥。
- 沒有元件特性圖：效率固定。壓縮機特性是平衡解隱含的那一種（轉速定功與壓比），渦輪是噴嘴形式的 Stodola 定律，喘振影響是規則；轉子失速、旋轉失速與閉鎖失速都不模擬。
- 準穩態：轉子、機匣與燃燒室沒有熱慣性，排氣溫度沒有起動峰值以外的過衝；燃燒室的火焰穩定、液膜溫度、汙染物不計。
- 起動、點火、熄火、喘振持續時間、怠速控制與油閥落後都是規則。
- 運轉線表格以下用相似律外推。在高馬赫數、低修正轉速（衝壓把轉子帶快、穩態到不了的區域），運轉線油量的功率差可達數十 %：
  - Mach 0.9 時，J85 級轉速 50 % 以下（45 % 時 +7 %）、模型引擎 65 % 以下屬於這一區。
  - Mach 1.1 時，J85 級 55 % 以下屬於這一區。
  - 在這一區以外，兩具範例的功率差在 2 % 內。
- 進氣道沒有溢流阻力、畸變、不起動（unstart）；超音速只有固定的激波損失式。雷諾數效應（高空）與濕度不計。
- 質量與長度是粗估，只用兩具引擎定係數。

## 來源

- J. Seitzman, AE4803 講義（Georgia Tech）〈Engine off-design performance〉例題。
- J. D. Mattingly,《Elements of Propulsion: Gas Turbines and Rockets》（單軸匹配；MIL-E-5008B 進氣恢復式；附錄 B 引擎資料）。
- H. I. H. Saravanamuttoo, G. F. C. Rogers, H. Cohen, P. V. Straznicky,《Gas Turbine Theory》（燃燒室壓損係數、渦輪冷卻效果）。
- S. L. Dixon, C. A. Hall,《Fluid Mechanics and Thermodynamics of Turbomachinery》（級負荷）。
- 其他公式來源：
  - Stanitz 滑移係數。
  - Smith 渦輪效率圖（級負荷）。
  - Roark 旋轉圓盤應力。
  - Summerfield 噴嘴分離準則（出口低於 0.4 倍大氣壓）。
  - Stodola 錐定律。
- 引擎資料：
  - J79／J85（Wikipedia、Mattingly 附錄 B）。
  - [J85-GE-17A（Smithsonian 國家航太博物館）](https://airandspace.si.edu/collection-objects/turbojet-engine-cutaway-general-electric-j85-ge-17a/nasm_A19800072000)。
  - [JetCat P80 SE（JetCat）](https://www.jetcat.de/en/productdetails/produkte/jetcat/produkte/RC%20ENGINES/Engines/p80_se)。
  - [P80 舊版規格比較（barnardmicrosystems）](https://barnardmicrosystems.com/UAV/engines/turbine.html)。
