# 翼型、機翼、空氣螺旋槳、直升機旋翼（0.22）

使用者 2026-10-11 要求加入渦槳／渦軸並「接到載具傳動軸」，選了「機翼與輪子都做」，並要求「機翼與輪子也是高自由度」：
機翼、螺旋槳、旋翼像引擎一樣由玩家設計，形狀也是參數，性能由形狀算出。0.22 在 AeroCore 做「形狀 → 性能」的計算與執行時零件，
hdfv 0.4 用它們做設計、檢查、3D 模型與範例。輪子在 0.23。

| 類別 | 用途 |
| --- | --- |
| `aero.Airfoil` | 翼型形狀：NACA 四位數、CST（Kulfan）、座標點（Selig 順序）；以值比較，可當快取鍵 |
| `aero.AirfoilAnalysis` | 由形狀算單一攻角、Reynolds 數的升力、阻力、力矩、分離、轉捩 |
| `aero.SectionPolar` | 每個翼型一張表（−180～180°、Reynolds 5e4～3e7），加可壓縮性、深失速、反向流、襟翼；依形狀快取 |
| `aero.Wing` | 平面形狀 → 展向載荷：數值升力線（Phillips–Snyder）＋非線性翼型、後掠、上反、扭轉、襟翼副翼、地面效應 |
| `aero.WingStructure` | 設計升力下的剪力、彎矩、翼盒應力、質量、翼尖撓度 |
| `aero.AirPropeller` | 空氣螺旋槳的葉素動量理論（弦長、扭轉、可變距、翼型、Mach），遊戲用表 |
| `aero.HelicopterRotor` | 直升機旋翼（也當尾槳）：方位角 × 半徑的葉素、一階揮舞、動量入流、渦環、地面效應 |
| `vehicle.part.WingPart`、`AirPropellerPart`、`RotorPart` | 執行時零件；螺旋槳與旋翼另有直接給槳距的 `loads` 給 hdfv 包裝控制規則 |
| `vehicle.Connection.Shaft` | 加 `freewheel`（單向離合器） |
| `VehicleDynamics` | 加轉子的陀螺力矩（在漂移步驟隱式處理） |
| `RigidBodyDynamics` | 陀螺體的漂移（`drift(s, p, spin, dt)`） |

## 翼型（`AirfoilAnalysis`）

- 無黏：Hess–Smith 面板法（每片常強度源、全部共用一個渦強度、尾緣兩片切向速度相等的 Kutta 條件，Moran 的寫法），160 片。
- 邊界層：從駐點沿上下表面，Thwaites 層流、Michel 轉捩（或層流分離即轉為紊流）、Head 紊流（Ludwieg–Tillmann 摩擦），形狀因子 2.4 分離。
- 不做黏性—無黏耦合：0.22 開發時試過以吹氣速度耦合，尾緣附近（無黏流在尾緣停滯）發散，改用依厚度與 Reynolds 數的升力斜率修正
  `viscousFactor`（12 % 厚、6×10⁶ 時 0.875，擬合 Abbott & von Doenhoff 與 Sheldahl & Klimas 的斜率）。
- 失速：
  - 尾緣分離取 Head 分離點，往前放大 `TRAILING_EDGE_SCALE` = 2 倍（Kirchhoff 的 f 是擬合升力的「等效」分離，未耦合的邊界層分離得太晚；以 NACA 4412 的最大升力校準）。
  - 前緣失速用 Valarezo–Chin 的壓差準則（吸力峰與尾緣的無黏 Cp 差達 15 × (Re/6×10⁶)^0.1）。
  - 兩者取較前者為 f，代入 Kirchhoff 流模型（Beddoes–Leishman 靜態失速）：法向力 × ((1+√f)/2)²、前緣吸力 × √f；
    法向力不低於 (1 − f) 倍平板（Lindenburg：Cn = 2 sin α / (0.56 + 0.44 sin α)，Hoerner 的二維平板 1.98）。
- 阻力：上下表面尾緣的 Squire–Young，加 Kirchhoff 力的壓力阻力；分離點在 95 % 弦長之前時邊界層凍結在分離點（分離區的阻力由壓力阻力算）。

## 極曲線表（`SectionPolar`）

- 攻角每 1°、±32° 內用分析；26°～32° 漸變到平板，之外是平板（壓力中心 30° 時 0.4 弦長、90° 時 0.5、180° 時 0.75，規則）；
  接近 180°（倒飛）是尖前緣的翼型：升力斜率 0.8 倍，10° 內失速（規則）。
- Reynolds 數：5×10⁴、1×10⁵、2×10⁵、5×10⁵、1×10⁶、2×10⁶、5×10⁶、1×10⁷、3×10⁷，log 內插、範圍外取端點。
- 可壓縮：附著部分 Prandtl–Glauert（0.9 以上不再增加）；失速角隨 Mach 降低：在 a₀ + (a − a₀)k 查不可壓縮的分離，
  k = 不可壓縮失速角 ÷「M × 吸力峰速度比達 1.35」的攻角（Beddoes 的 NACA 0012 失速角在 Mach 0.3～0.8 都落在 1.1～1.4）；
  臨界 Mach 由吸力峰以 Karman–Tsien 與音速 Cp 求，之上加 Lock 的 20(M − Mcr)⁴（不超過雙凸翼型在 Mach 1.2 的厚度波阻）；
  Mach 0.95～1.2 漸變到 Ackeret 線性超音速理論。跨音速與超音速是粗估。
- 襟翼（`flapped`）：Glauert 的 τ(cf)、偏角效率（簡單與分裂 10° 以上、開縫與 Fowler 20° 以上下降，(線性範圍/δ)^0.6，DATCOM 趨勢的規則）、
  零升角移動 τηδ、最大升力增加升力增量的 0.7（簡單、分裂）、0.85（開縫）、0.9（Fowler）、薄翼理論力矩、阻力增量 K cf^1.38 sin²δ（規則）、
  Fowler 弦長延伸。
- 建表約 0.3 s（每個形狀一次，最多快取 128 個）；查表約 150 ns。從不丟例外（任何有限或非有限輸入都給有限值）。

## 機翼（`Wing`）

- Phillips & Snyder（J. Aircraft 37(4), 2000）的數值升力線：每條 1/4 弦線段一個馬蹄渦（尾渦沿機翼 −x），
  牛頓法解「局部速度（含誘導）的 Kutta–Joukowski 力 = 該截面在局部攻角的升力」；截面用 `SectionPolar` 的非線性極曲線，
  所以失速擴展、襟翼都由極曲線進來。翼根與翼尖翼型的係數沿展向線性混合。
- 後掠：截面在垂直於 1/4 弦線的平面工作（簡單後掠理論：法向速度、弦長 c cos Λ、該分量的 Re 與 Mach），與升力線一起重現 DATCOM 的後掠升力斜率；
  翼根的 1/4 弦線以約一個翼根弦長的雙曲線圓滑（Reid & Hunsaker 2021 的作法；尖角會讓兩半的附著渦在翼根互相誘導出不實的載荷）。
- 地面效應：Wieselsberger 的係數乘在誘導速度上（翼根離水面的高度；陸地在 0.23）。
- 執行時以上一步的渦強度暖啟動，容差 1e-4；12 條／半翼、翼根翼尖不同、有襟翼副翼時一次約 45 µs，8 條／半翼約 10 µs。
- 失速後截面升力隨攻角下降，Jacobian 接近奇異，牛頓步（含 5 次減半回溯）常降不下殘差：這時改走 Levenberg–Marquardt 的阻尼最小平方步
  （(JᵀJ + μ diag JᵀJ) δ = JᵀR，μ 從 1e-3 每次 ×10）。冷啟動最多 40 次迭代，暖啟動 12 次（遊戲中下一步接著收斂，不增加每步成本）。
  失速後升力線可能有多個解或沒有精確解（回傳殘差最小的，`Solution.residual`）；試算掃攻角時用 `solve(speed, alpha, beta, Air, inputs, previous)`
  從上一個攻角的解延續（像風洞由小到大調攻角），曲線只走一個分支。0.22 開發時逐點冷啟動，失速後阻力會從 0.21 掉回 0.18、殘差到 0.4。

## 結構（`WingStructure`）

Schrenk 展向載荷（弦長比例與同總升力的橢圓分布取平均）、從翼尖往內積分剪力與彎矩；前後翼梁之間的翼盒：
兩個梁緣（翼盒上下蒙皮板）在 0.9 倍截面厚度、兩片腹板，σ = M/(A h)、τ = V/(2 h t)、撓度由 M/(E I)、I = A h²/2；
其餘蒙皮與翼肋只計質量。不檢查挫曲、扭轉、疲勞、接頭。

## 空氣螺旋槳（`AirPropeller`）

葉素動量理論（Prandtl 葉尖與槳轂損失、旋流），入流角用 Ning 的單一殘差，從無誘導角往外找根再以 Illinois 假位法收斂；
旋轉槳葉的失速延遲用 Snel 的修正（失速截面找回 3 (c/r)² 的升力損失，ECN-C--94-107）；
弦長線性、固定幾何螺距 P 的螺旋扭轉再加線性扭轉、整片槳葉的槳距設定（可變距）；截面用 `SectionPolar`（根梢混合）在各站的 Re、Mach。
係數用 `PropellerSolver` 的四象限形式（對 0.7R 合速度的槳距角 β），停止的槳也正常。遊戲用表：β（−45～90° 每 2.5°，其餘每 15°）×
槳距設定（7 點）× Mach（0.1～1.0 每 0.1），背景建立約 0.7 s。不含攻角下的法向力與 P 因子、滑流對機體、槳葉變形。

## 直升機旋翼（`HelicopterRotor`）

- 12 個方位角 × 10 個半徑站的葉素，`SectionPolar`（後行槳葉的反向流、前行槳葉的可壓縮性）；Prandtl 的 B = 1 − √(2C_T)/N_b 外無升力
  （B 不低於 0.9：這個式子是給輕載旋翼的，轉得很慢的旋翼以葉尖速度算出的 C_T 沒有意義；懸停 C_T 0.006 時 B 約 0.97，不受影響）。
- 一階揮舞（鉸鏈偏置、彈簧、均勻槳葉），方位角上的氣動揮舞力矩與離心、彈簧、陀螺力矩（Johnson：β'' + ν²β = γM + 2/Ω(p cos ψ − q sin ψ)）
  做諧波平衡；每次呼叫走一步準牛頓（阻尼 γ/8 的 Jacobian），載具每步都呼叫，所以跨步收斂。
- 入流：Glauert 動量（乘誘導功率係數 1.15）、前飛的 Drees 線性分布、下降時的渦環經驗曲線（Leishman eq. 2.138）、
  Cheeseman–Bennett 地面效應、Pitt–Peters 一階延遲（τ = 0.849 R / (4 V_m)）。
  Drees 的係數以不超過 0.5 的前進比計算（模型的適用範圍）：旋翼幾乎不轉時前進比是幾十，0.22 開發時 kx 因此到 −780，
  誘導速度的分布是平均值的數百倍、推力餵大誘導速度，直升機在世界中起動時 0.3 秒內發散、讓 GameTest 伺服器當掉。
- 順時針旋轉的旋翼以鏡像計算；週期變距以旋翼自己的方位角表示，`Controls.fromSticks` 把桿量換成兩個方向都正確的 θ1c、θ1s。
- 一次約 60 µs（12 × 10，根梢不同翼型）。

## 顯示值（0.30）

- `RotorPart`：β0、β1c、β1s（錐角與翼尖平面的前後、左右傾斜）、θ0、θ1c、θ1s（總距與週期變距），0.001 rad 一級；誘導速度 0.1 m/s 一級。
  包住它的零件要轉送 `shownValues()`，否則不寫（`PartStep.shownCount()`）。
- `AirPropellerPart`：槳葉角（加在設計角度上的，0.001 rad 一級）。
- `api.aero.Downwash`：尾流隨深度從誘導速度趨近兩倍（動量理論），貼地後的外洗峰值超過一個半徑後約按 r^-1.1 衰減，
  高於兩個半徑後再按 exp(−(h − 2R)/2R) 減弱（Leishman 第 2、10 章的趨勢；只給畫面用）。`DownwashTest`。

## 傳動

- `Connection.Shaft(..., freewheel)`：單向離合器把兩邊分成兩個轉子串；每步先各自推進，主動側不會落後時鎖在一起
  （以兩邊帶進來的角動量開始，等同剛性嚙合），否則被動側可以超轉（引擎停了旋翼自轉）。JSON 只有 `true` 時寫出 `"freewheel": true`。
- 陀螺力矩：每個轉子的自旋角動量 H，機體以 ω 轉動時受 −ω × H。它不當外力，而是在漂移步驟裡把 H（機體座標、這一步的轉速）
  和機體一起轉（陀螺體：保持機體加轉子的世界角動量，中點迭代）。0.22 開發時原本當顯式外力：旋翼的 H 對機身慣量很大
  （直升機約 19 rad/s，5 ms 子步），每步多出一點能量，`VehicleTest.aFreeGyrostatKeepsItsMomentumAndEnergy` 的情況 10 秒內機身轉動能量變 55 倍。

## 驗證（`SectionPolarTest`、`WingTest`、`RotorAndPropellerTest`、`VehicleTest`）

參考資料由研究代理從原始報告的表格與圖讀出（圖讀值約 ±0.01～0.02 cl、±0.0003 cd、±0.5°、±0.01 效率）。

| 項目 | 資料 | 模型 |
| --- | --- | --- |
| NACA 0012／2412／4412 升力斜率 | 0.105～0.11 /°（TN 1945、Report 824、TM 4074） | 0.1056 /° |
| 零升角 | 0、−2.0°、−4.0°（Report 824） | 0、−2.12°、−4.22° |
| 1/4 弦力矩 | 0、−0.045～−0.05、−0.09～−0.095 | 0、−0.050、−0.097 |
| 最小阻力（6×10⁶） | 0.0057（0012，TN 1945；Ladson 0.0065）、0.0060、0.0062 | 0.0064、0.0060、0.0058 |
| NACA 0012 最大升力 | TN 1945：3×10⁶ 1.53、6×10⁶ 1.60；Ladson：2×10⁶ 1.41、3.94×10⁶ 1.48、5.97×10⁶ 1.66 | 1.31、1.43；1.25、1.36、1.43（−8～−14 %） |
| NACA 0012 低 Re 最大升力（Sheldahl & Klimas 表 2） | 3.6×10⁵ 0.98、7×10⁵ 1.07、1×10⁶ 1.12、2×10⁶ 1.22 | 0.94、1.06、1.13、1.25 |
| NACA 2412 最大升力（Report 824） | 3.1×10⁶ 1.58、5.7×10⁶ 1.66、8.9×10⁶ 1.67 | 1.49、1.61、1.69（−6～+1 %） |
| NACA 4412 最大升力（Report 824） | 3×10⁶ 1.53 @ 13°、6×10⁶ 1.64 @ 14～15°、9×10⁶ 1.68 | 1.66、1.78、1.85 @ 16～17°（+9～+10 %，失速角晚 2～3°） |
| NACA 0012 失速升力對 Mach（Beddoes 推得） | 0.3：1.68、0.5：1.27、0.6：1.12、0.8：0.61 | 1.43、1.08、0.91、0.71 |
| NACA 0012 臨界 Mach（α = 0） | 約 0.72～0.74 | 0.73 |
| 有限翼升力斜率（Helmbold，Lowry & Polhamus TN 3911 的寫法；DATCOM 後掠） | 矩形 A6、梯形 0.4 A8、後掠 30° A6、後掠 45° A4 | +1～2 %、+2.6 %、+0.6 %、+3 % |
| 誘導阻力效率 u（Anderson, Report 572 圖 4，圓翼尖） | λ 0.4、A 8：0.990；λ 1、A 6：0.977（方翼尖約 0.95） | 測試在 0.99 ± 0.03、0.96 ± 0.03 |
| 機翼最大升力／翼型 | 直翼 0.8～1.0（Raymer、DATCOM） | 在範圍內 |
| 輕型機鋁翼質量 | 約 8～12 kg/m²（Raymer） | hdfv 的 C172 級翼 9.0 kg/m² |
| 螺旋槳峰值效率（Report 640 圖 12，三葉 Clark Y 5868-9） | 15°：0.78、25°：0.85、35°：0.85、45°：0.825 | 0.83、0.89、0.91、0.92（+5～+12 %） |
| 峰值處的前進比 | 0.64、1.05、1.48、1.93 | 0.62、1.08、1.58、2.26 |
| 零推力的前進比 | 0.80、1.28、1.83、2.62 | 0.78、1.28、1.88、2.68 |
| 靜推力 C_T（圖 43 與圖 11 推得） | 15°：0.14、20°：0.145、25°：0.16、35°：0.18 | 0.128、0.143、0.106、0.109 |
| UH-60A 級旋翼懸停 FM（Norman et al. 2002 圖 22，C_T/σ ≈ 0.07） | 全尺寸 0.67、模型 0.72～0.73、飛行 0.66～0.72 | 0.74（C_T/σ 0.0725） |
| UH-60A 懸停總功率 16,500 lb（NDARC 驗證圖 15 的飛行 C_P 換算） | 約 2,000 shp（1,490 kW，含尾槳、傳動、下洗） | 主旋翼 1,216 kW（74.7 kN）；加尾槳約 10 %、傳動 3 %、下洗 3.6 % 約 1,420 kW |
| UH-60A 級前飛（平飛配平，機身等效平板 2.4 m²） | 總功率最低約 700 kW（約 40 m/s） | 主旋翼 40 m/s 約 570 kW（含寄生功率），未計尾槳與傳動 |

其他測試：對稱翼型的升力奇、阻力偶；任何攻角、Re、Mach 都有限值；Prandtl–Glauert；襟翼的薄翼增量、Fowler > 開縫 > 簡單；
地面效應（展長十分之一高度升力 +5 % 以上、誘導阻力 −30 % 以上）；副翼滾轉方向；暖啟動與冷啟動相同；上反角的側滑滾轉；
翼結構的 Schrenk 根部彎矩、應力與梁緣厚度成反比、撓度與 E 成反比；螺旋槳表與直接計算在 3 % 內、停止的槳產生阻力；
旋翼週期變距的推力傾斜（兩種旋轉方向）、地面效應、自轉（下降時氣流驅動旋翼）；停止或很慢的旋翼在下沉、側飄、機身轉動時逐步計算仍有界
（`aStoppedOrCreepingRotorStaysBoundedOverSteps`）；單向離合器（驅動、超轉、重新嚙合）、陀螺力矩（含無外力矩的陀螺體守恆角動量與能量）、機翼零件。

## 限制

- 翼型最大升力 ±15 % 內：對稱翼型偏低（NACA 0012 高 Re 時 −8～−14 %），彎度大的偏高（4412 +10 %、失速角晚 2～3°）；
  最大升力後的形狀（緩和或陡降）是 Kirchhoff 加平板，不是量測；層流分離泡、粗糙度（標準粗糙度使最大升力降約 25 %）不模擬。
- 可壓縮性只是修正：Mach 0.5～0.6 的失速升力約低 15～20 %，0.8 約高 17 %；跨音速、超音速翼型是粗估。
- 機翼之間沒有下洗（尾翼看到自由流）、螺旋槳滑流不吹機翼、沒有機身；後掠 35° 以上翼尖失速提早（升力線的已知限制）。
- 失速後的升力線不是良置問題：殘差在少數攻角留到 0.1 左右（最小平方解），失速後的力是哪個分支取決於來的路徑（延續求解、遊戲中的上一步），
  沒有遲滯以外的動態失速模型。
- 螺旋槳峰值效率偏高 5～12 %（葉素動量理論、沒有發動機艙）；前進比的形狀很準。靜推力在細距（15～20°，起飛時調速器所在）準，
  粗距（25° 以上）偏低約 35 %（深失速的截面）。旋翼沒有擺振、彈性、高次諧波與振動、斜盤相位（鉸鏈偏置時週期響應少於 90°，兩軸耦合）、
  機身干擾；前飛只做了粗略的配平檢查。
- 地面效應目前只對水面；陸地在 0.23 的地面查詢。

## 來源

- I. H. Abbott, A. E. von Doenhoff,《Theory of Wing Sections》（NACA 四位數族、附錄 IV 的翼型資料）。
- R. E. Sheldahl, P. C. Klimas, SAND80-2114（NACA 0012 等在低 Reynolds 數與大攻角的資料）。
- J. L. Hess, A. M. O. Smith（面板法）；J. Moran,《An Introduction to Theoretical and Computational Aerodynamics》。
- B. Thwaites；R. Michel；M. R. Head；H. B. Squire, A. D. Young（積分邊界層與型阻）；T. Cebeci, P. Bradshaw（擬合式）。
- W. O. Valarezo, V. D. Chin, AIAA Journal 32(4)（壓差失速準則）。
- J. G. Leishman,《Principles of Helicopter Aerodynamics》（Beddoes–Leishman 靜態失速、動量理論、渦環、Drees、Pitt–Peters、地面效應）；
  W. Johnson,《Helicopter Theory》（揮舞方程）。
- C. Lindenburg, ECN-C--03-025；S. F. Hoerner,《Fluid-Dynamic Drag》（平板法向力）。
- W. F. Phillips, D. O. Snyder, J. Aircraft 37(4), 2000（數值升力線）；J. T. Reid, D. F. Hunsaker, J. Aircraft 58(2), 2021（後掠翼的翼根處理）。
- C. Wieselsberger（地面效應）；I. C. Cheeseman, W. E. Bennett, ARC R&M 3021（旋翼地面效應）。
- O. Schrenk, NACA TM 948（展向載荷近似）；T. H. G. Megson,《Aircraft Structures》；M. C. Y. Niu,《Airframe Structural Design》；D. P. Raymer,《Aircraft Design》。
- E. P. Hartman, D. Biermann, NACA Report 640（螺旋槳）；S. A. Ning, Wind Energy 17, 2014（BEM 的單一殘差）。
- USAF DATCOM（有限翼升力斜率、襟翼趨勢）；H. Glauert（襟翼效率、誘導阻力）；R. M. Kulfan（CST）。
- 研究代理讀取的原始資料：NACA Report 824（Abbott, von Doenhoff, Stivers；2412、4412）、Loftin & Smith NACA TN 1945（0012）、
  Ladson NASA TM 4074（0012）、Sheldahl & Klimas SAND80-2114 表 2、Lowry & Polhamus NACA TN 3911（Helmbold 與後掠、可壓縮的升力斜率）、
  Anderson NACA Report 572（梯形翼的誘導阻力）、Hartman & Biermann NACA Report 640（圖 2、11、12、43）、
  Norman et al. AHS Forum 2002（UH-60A 旋翼與 FM）、Johnson NDARC Validation 2010（UH-60A 參數與懸停功率）、
  Cheeseman & Bennett ARC R&M 3021（地面效應式）。UH-60A：半徑 26.83 ft、四葉、弦長 20.9 in、等效扭轉 −18°、258 rpm、
  實度 0.0826；尾槳半徑 5.5 ft、實度 0.1875、約 1,190 rpm。
- Snel, Houwink, Bosschers, ECN-C--94-107（旋轉槳葉的失速延遲）。
- 網址：[Report 824](https://ntrs.nasa.gov/citations/19930090976)、[TN 1945](https://ntrs.nasa.gov/citations/19930082618)、
  [TM 4074](https://ntrs.nasa.gov/citations/19880019495)、[SAND80-2114](https://digital.library.unt.edu/ark:/67531/metadc1191254/)、
  [TN 3911](https://ntrs.nasa.gov/citations/19930084818)、[Report 572](https://ntrs.nasa.gov/citations/19930091647)、
  [Report 640](https://ntrs.nasa.gov/citations/19930091715)、[Norman et al. 2002](https://rotorcraft.arc.nasa.gov/Publications/files/Norman_AHSF02.pdf)、
  [NDARC Validation](https://ntrs.nasa.gov/citations/20110002948)、[R&M 3021](https://reports.aerade.cranfield.ac.uk/bitstream/handle/1826.2/3590/arc-rm-3021.pdf)。
