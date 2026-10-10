# 物理模型與約定

`AeroSolver` 與 `HydroSolver` 是準穩態受力模型，每次呼叫只計算當下受力。
0.3 另提供 `LinearDynamics`／`SphereDynamics` 推進平移狀態；詳見 [測試球說明](PROBE.md)。
0.4 加入 `RigidBodyDynamics`／`FloatingBodyDynamics`，積分三軸平移與旋轉；詳見 [平台說明](PLATFORM.md)。
0.6 加入接觸反力、線性波浪、螺旋槳推力與阻力摩擦生熱，見本文後半。
0.9 加入與輻射一致的垂直繞射力、Wageningen B 系列螺旋槳、細長船體（阻力與操縱）、舵與多物體聯合接觸。
0.10 加入不規則海況、所有讀取者共用的水面、Kelvin 尾浪與水花（[海況與尾浪](SEA.md)）、2D+t 滑行艇（[滑行艇](PLANING.md)），
並把示範係數換成有出處的資料（本文〈0.10 係數來源〉）。
剛體 API 的模型幾何／重心／慣性採局部座標，狀態位置／速度／角動量、施力與力矩採世界座標；不要混用。
獨立 `cfd` 模組會推進二維流場，見 [CFD 說明](CFD.md)。兩者的適用尺度不同。
輸入應使用有限、合理的遊戲尺度。`AeroSolver` 的預設翼型係數仍是可調的通用值；示範物體使用的係數來源見〈0.10 係數來源〉。

## 座標與單位

| 輸入／輸出 | 單位與定義 |
| --- | --- |
| 線速度、風速 | m/s，同一世界座標系 |
| 角速度 | rad/s，右手定則 |
| offset | m，由質心到施力點的向量 |
| 面積 | m² |
| 密度 | kg/m³，允許 0 表示真空 |
| 動力黏度／聲速 | Pa·s／m/s，有限且正值 |
| 弦長／排水體積 | m／m³ |
| 攻角 | rad，範圍 [−π, π] |
| 動壓 | Pa |
| 力／力矩 | N／N·m，世界座標，力矩以質心為參考 |

核心沒有固定「載具前進軸」。範例選 +X 為前方、+Y 為上方，跨度方向為 `chord × normal`。
實際載具需傳入自身的世界座標方向。風速表示空氣正在移動的方向，不是氣象上的「風從哪裡來」。

## 局部氣流

```text
v_point = v_center + angular_velocity × offset
v_relative = v_point − wind_velocity
span = normalize(chord × normal)
v_section = v_relative − span × dot(v_relative, span)
alpha = atan2(−dot(v_section, normal), dot(v_section, chord))
```

上式中 `angular_velocity × offset` 和 `chord × normal` 為向量叉積；
`span × dot(...)` 則是向量乘以純量。
機翼使用弦向／法向平面內的速度；純沿翼展的氣流不產生機翼力。
需要側向阻力時，加入 `DragBody` 或額外翼面。靜止、極低速度或真空回傳零力。

## 升力、阻力與力矩

```text
q = 0.5 × density × |v_section|²
lift_magnitude = q × wing_area × CL
drag_magnitude = q × wing_area × CD
lift_direction = span cross normalize(v_section)
drag_direction = −normalize(v_section)
torque = offset cross force
```

升力方向垂直於局部速度，符號由 CL 決定；阻力永遠反向於氣流相對速度。
施力點是使用者指定的固定位置，不會模擬壓力中心隨攻角變化。
另加入獨立的翼型俯仰力矩 `q × area × chordLength × Cm × span`。
Cm 以施力點為參考，正值依 `chord × normal` 軸的右手定則旋轉。
若翼型資料的力矩參考點不同，必須先換算；不能同時把同一俯仰力矩算兩次。

公式基礎：[NASA 升力方程](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/)。

## 預設對稱翼型

失速前 `CL = liftSlope × alpha`；預設升力斜率為 5.5/rad。
從 15° 到 25° 以 smoothstep 混合到平板近似 `sin(2 × alpha)`，負攻角使用對稱曲線。
這個區間是平滑過渡，15° 並不保證剛好是升力峰值。超過 90° 的反向流也使用平板近似。

```text
CD = CD0 + CL² / (π × aspect_ratio × efficiency)
     + blend × 1.8 × sin(alpha)²
```

預設 `CD0=0.025`、展弦比 `6`、效率 `0.8`。
誘導阻力公式參考 [NASA 誘導阻力係數](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/induced-drag-coefficient/)。
混合函數和失速後項是本專案選定的遊戲近似，並非 NASA 的完整失速模型。
在深失速下沿用此阻力組合也是近似；有精度需求時應提供對應條件的翼型資料。

`DragBody` 使用完整三維速度、固定參考面積及固定 CD，力為
`−0.5 × density × |v_relative| × area × CD × v_relative`。
它是各向同性的簡化機身阻力，不根據迎風方向計算投影面積。

## 標準大氣、黏滯性與流況

`FluidState` 同時適用空氣和水，包含密度、動力黏度、聲速、局部介質速度。
`Atmosphere.atAltitude` 採乾燥標準大氣的前兩層（位勢高度 −500～20000 m；海平面以下是近地延伸）：

```text
0～11000 m: T = 288.15 − 0.0065 h
            p = 101325 × (T / 288.15)^[g / (R × 0.0065)]
11000～20000 m: T = 216.65
               p = p_11000 × exp[−g × (h−11000) / (R × T)]
rho = p / (R × T), R = 287.05287 J/(kg K), g = 9.80665 m/s²
mu = 1.458e−6 × T^1.5 / (T + 110.4)  [Pa s]
a = sqrt(1.4 × R × T)
Re = rho × speed × chordLength / mu
Mach = speed / a
```

這裡 T 使用 Kelvin，壓力 Pa。不是把 Minecraft 的絕對 Y 值直接當地球海拔；
環境適配層應先定義海平面與高度比例，也應自行提供風場。
資料基礎：[U.S. Standard Atmosphere 1976](https://ntrs.nasa.gov/citations/19770009539)、
[NASA 黏滯性與 Sutherland 關係](https://www.grc.nasa.gov/www/k-12/airplane/viscosity.html)。

`SymmetricAirfoil.DEFAULT` 仍是攻角近似，**取得 Re／Mach 並不代表它已會自動修正係數**。
`PolarTable` 對使用者提供的攻角、CL/CD/Cm 資料做線性插值；`ReynoldsPolar` 在不同資料表之間
依 log(Re) 插值，且要求指定最大 Mach。超出表格攻角、Re 或 Mach 範圍直接報錯，不盲目外插。
插值區間任一端標為失速，區間內就保守標為失速。沒有內建偽造的「實測翼型資料」。

`FluidField` 可依每個翼面／阻力元件的世界施力點與時間取樣。
元件跨越水面時，必須由呼叫者切分乾／濕面積；單點取樣不能替代浸水幾何計算。

## 水動力與浮力

水下翼可使用相同升阻力方程，但應提供水翼適用的 Re 係數；沒有波浪或空化模型。
內建淡水預設為 20°C 附近的近似值：密度 998.2 kg/m³、黏度 0.001002 Pa·s、聲速 1482 m/s。
這是固定設定，並未實作完整溫度／鹽度／壓力狀態方程。

`HydroSolver.sphereImmersion` 精確求球體在平面下的球冠體積與形心：

```text
capHeight = clamp(radius + centerDepth, 0, 2 × radius)
V = pi × capHeight² × (radius − capHeight / 3)
centroidHeight = −3 × (2 × radius − capHeight)² / [4 × (3 × radius − capHeight)]
F_buoyancy = −rho × V × gravityVector
torque = centerOfBuoyancyOffset cross F_buoyancy
```

完全離水的體積與浮力為零；半浸水時浮心在球心下方 3r/8；全浸水時浮心回到球心。
模型中的平面法向需與重力反向。多球近似船體時，球體不能互相重疊，否則排水量會重複累加。
也可以自行求任意形體的排水體積／形心，直接傳入 `Immersion`。
浮力不包含物體本身重量，重量必須另外算 `mass × gravity`。
依據：[NASA 阿基米德原理](https://www.grc.nasa.gov/www/k-12/WindTunnel/Activities/buoy_Archimedes.html)。

`HydroSolver.sphereDrag` 僅用於完全浸沒的孤立球體，在空氣中也可使用：

```text
Re <= 1000: Cd = 24/Re × (1 + 0.15 × Re^0.687)
1000 < Re <= 200000: Cd = 0.44
F_drag = −0.5 × rho × speed × pi × radius² × Cd × relativeVelocity
```

低速實作用等價 Stokes 形式避免 `Cd` 除零，Re 趨近零時力趨近 `−6 pi mu r v`。
相關式可見 [OpenFOAM 的 Schiller–Naumann 模型](https://cpp.openfoam.org/v8/SchillerNaumann_8C_source.html)。
Re=200000 是本專案保守設下的上限，並不是此模型涵蓋阻力危機的承諾；
不應用球體阻力代替整艘船的造波阻力、附加質量或入水衝擊。

## 接觸反力（0.6）

`ContactSolver` 處理一個運動剛體與靜止世界軸對齊盒（`WorldBox`）的接觸。物體形狀是模型局部的 `CollisionSphere`；
平面或方塊以角落小球近似，取樣間距決定平面與邊緣的解析度。

- **產生接觸**：取球心到盒的最近點；法向由障礙指向物體，separation 為負代表穿入深度。
  若最近點所在的面被另一個障礙覆蓋（相鄰方塊的共用面），就略過這個接觸；球心在盒內時，沿最近且非共用的面離開。
- **速度層**：投影 Gauss–Seidel 序列衝量，24 次迭代；法向衝量不為負，摩擦力限制在各向同性庫侖錐 |f| ≤ μN。
  參考：Catto,「Iterative Dynamics with Temporal Coherence」(GDC 2005)；
  [Catto, Fast and Simple Physics using Sequential Impulses (GDC 2006)](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf)。
- **推測接觸**：球在一步內可移動的距離（|v_point| dt + 0.02 m）內就建立接觸，目標法向速度為 −gap/dt，
  所以高速物體停在表面，不會穿過薄方塊。
- **恢復係數**：只對本步有承載、且接近速度大於 0.5 m/s 的接觸，以求解前的法向速度 v_n0 設目標 −e·v_n0。
  推測接觸在預測到撞擊的那一步就反彈，反彈點可能在表面前 |v| dt 以內（與
  [Box2D v3 的取捨](https://box2d.org/posts/2024/02/solver2d/)相同）；不會穿透。
- **位置修正**：穿入超過 0.005 m 時，以非線性 Gauss–Seidel 只移動位置與姿態（每次修正 20%，每次最多 0.2 m，最多 3 次）。
  速度與世界角動量不變，因此不注入動能；轉動修正時角動量固定，角速度會有極小的相應變化。

`FloatingBodyDynamics.step(..., externalLoads, collision, dt)` 的順序：
重力／浮力／外加力衝量 → 隱式阻力 → 接觸衝量 → 推進位置與姿態 → 位置修正。
`Result.contact` 是接觸衝量除以 dt，`forces` 含每個接觸點的 `CONTACT` 力；動量收支為
`Δp/dt = hydrostatics + averageDrag + contact + external`（位置修正不改動量）。

單元測試對照解析解：滑動實心球在 v = 5v0/7、t = 2v0/(7μg) 轉為純滾動；
斜面 tanθ ≤ μ 時靜止，超過時加速度為 g(sinθ − μ cosθ)；靜置箱體平均接觸力等於重量且不漂移；
e = 0.5 時反彈高度 ≈ e²h；e = 0 時速度層求解不增加動能（200 組隨機狀態）、翻滾撞進牆角時機械能不上升；
60 m/s 小球不穿過 5 cm 薄牆；相鄰方塊接縫不產生側向法向；平台在乾地靜止時接觸力等於重量減空氣浮力，重心落在接觸點範圍內。

限制：只有一個運動物體，障礙物不被推動（實體碰撞也是單向）；沒有滾動阻力，靜／動摩擦共用同一 μ；
沒有暖啟動，大量接觸或堆疊時收斂較慢；方塊形狀即 Minecraft 碰撞 AABB 的組合。

### 接觸摩擦生熱（0.7）

`ContactSolver.Result.frictionDissipationW` = Σ −f_t · u_t，f_t 為最終摩擦力、u_t 為最終滑動速度；
與隱式阻力相同，以步末速度計算，收斂時非負，黏住或純滾動時為 0。法向撞擊（恢復係數小於 1）的損失不算在內。
熱量在兩個半無限固體理想接觸、介面發熱時按熱滲透率 e = √(kρc) 分配，物體得到 e₁/(e₁+e₂)（Blok 1937；Charron 1943），
這樣兩邊表面溫度相等；屬於靜止熱源近似，高速滑動時熱會偏向不斷接觸新表面的一方，這裡沒有模擬。

單元測試：箱子在平地滑行到停，摩擦熱 = 初始動能（誤差 < 1%）；實心球由滑動轉滾動時摩擦熱 = 初始動能的 2/7，
差值為以步末滑速計算的 O(dt) 項（本例約 1%，已以解析式核對），轉為滾動後為 0；
以 Carslaw–Jaeger 等熱流半無限體解 ΔT = 2q√(t/π)/e 核對分配比使兩邊溫升相同；
500 組隨機的模擬型接觸（穿入在 slop 內、常同時碰到地面與牆）摩擦功率不為負，殘差 < 1e−4·KE/dt（24 次 PGS 迭代）。
模擬中不會出現的深穿入（數十公分）退化狀態，未收斂殘差可達 KE/dt 的千分之幾，報告值可能短暫為負。

## 波浪（0.6）

`LinearWave` 是一階 Airy 行進波，`WaveField` 疊加多個成分：

```text
θ = k (x cos d + z sin d) − ω t + φ,   k = 2π/L
η = A cos θ
ω² = g k tanh(k h)                     （h = +∞ 時 ω² = g k）
u_h = A ω cosh(k(z+h)) / sinh(k h) · cos θ   沿傳播方向
w   = A ω sinh(k(z+h)) / sinh(k h) · sin θ
```

z 為相對靜水面的高度（水下為負），夾在 [−h, 0]；靜水面以上沿用 z = 0 的值（一階理論只定義到平均水面）。
實作改寫成指數形式，深水時不溢位。限制 H/L ≤ 0.1（遠低於 Michell 碎波陡度約 1/7）與 H ≤ 0.78h（McCowan 碎波指標），超出直接拒絕。
來源：Dean & Dalrymple,《Water Wave Mechanics for Engineers and Scientists》，[World Scientific, DOI 10.1142/1232](https://worldscientific.com/doi/10.1142/1232)。

0.6 的浮體採「瞬時水面靜水壓」近似；0.7 改為下方的線性波壓積分與附加質量。
Minecraft 取樣逐柱抬高或降低自由水面（水柱頂端且上方是開放空間），並把軌道速度併入水流，參與阻力。
沒有繞射、輻射阻尼、慢漂力、淺化或碎波；波不會被岸邊反射，水方塊本身不動。

## 波壓隨深度衰減與附加質量（0.7）

線性波的動壓頭（Dean & Dalrymple）：

```text
p/(ρg) = P(z) = h_d(z) − z,   h_d = A K(z) cos θ,   K(z) = cosh(k(z+h)) / cosh(kh)
∂u/∂t = −g ∇h_d               （線性 Euler 方程；`LinearWave.acceleration` 與此一致）
```

z 為相對靜水面的高度，水面以上取 K = 1，所以瞬時水面 z = η 上 P = 0。
`HydroSolver.sphereInWaves` 與 Minecraft 取樣都沿每條垂直球弦積分壓力差 P(底) − P(頂)，加總後得到垂直方向的
Froude–Krylov 力（含浮力），以 `pressureFraction` = 力 /(ρgV) 傳給浮體；靜水時它等於幾何浸水比例。
因為 K 隨深度遞減（Smith 效應），波峰下的水下物體受到的升力小於「瞬時水面靜水壓」的估計，波谷下則較大；
小型完全浸水球的結果收斂到 Morison 的 ρV(g + a_z)。

每個濕浮筒另加：

```text
F = ρ V_wet a_f,h  +  C_a ρ V_wet (a_f − a_point),   C_a = 0.5
```

第一項是水平 Froude–Krylov 力（垂直部分已在壓力積分中），第二項是附加質量；a_f 取濕體積平均的波浪加速度，
a_point 是浮筒濕形心的加速度。C_a = 0.5 為球體在無界理想流體的位勢流值（Newman,《Marine Hydrodynamics》, 1977），
以幾何浸水體積縮放；不隨頻率、水面距離或牆面改變，也沒有輻射阻尼。空氣的附加質量忽略。

數值上，附加質量矩陣 M_a = Σ m_i J_iᵀJ_i 放入隱式速度更新：

```text
(M + M_a + C dt) ν₁ = (M + M_a) ν₀ + dt W + C dt u_f,   ν = (v, ω)
```

（`RigidBodyDynamics.implicitVelocity`），附加質量大於物體質量時也穩定；沒有附加質量時與 0.6 的 kick + 隱式阻力完全相同。
自由轉動（drift）把物體與附加質量點當成一個剛體一起推進，因此轉動時保持的是「物體＋夾帶水」的動量與動能；
這段期間物體與夾帶水交換的動量列為 `inertia` 的一部分，使 `Δp/dt = hydrostatics + averageDrag + inertia + contact + external` 仍精確成立。
接觸衝量只用物體本身的質量性質（撞擊時不含夾帶水）。

單元測試：∂u/∂t 與速度場數值微分一致，且 ∂u/∂t = −g∇h_d；水面處 h_d = η，深水按 e^{kz}、淺水底部按 1/cosh(kh) 衰減；
小球完全浸水時 pressureFraction = 1 + a_z/g（誤差 < 0.2%），深處趨近 1；半浸水球自由起伏週期 = 2π√(R/g)
（附加質量 0.5ρV_wet 的線性解析解；無附加質量時應為 2π√(2R/3g)，差 18%）；含附加質量動能的阻力耗散收支。

單元測試：深水與淺水的色散極限、速度場無散且無旋、線性運動邊界條件 ∂η/∂t = w、平底 w = 0、
波面週期與零平均、拒絕過陡與碎波；平台在 60 m 長波中的起伏與縱傾振幅（以波頻傅立葉投影）分別約為 A 與 A·k。

## 波浪輻射阻尼與浮筒間干擾（0.8）

浮筒上下運動時會向外輻射波浪並帶走能量。0.7 以前靜水中的起伏只有黏性阻力可消散；關掉二次阻力時平台起伏完全不衰減。
0.8 以 Cummins (1962) 的時域方程加入線性輻射力：

```text
F_i(t) = −A∞,i ẅ_i − Σ_j ∫₀^T K_ij(τ) w_j(t − τ) dτ,      K_ij(t) = (2/π) ∫₀^∞ B_ij(ω) cos(ωt) dω
A(ω) = A∞ − (1/ω) ∫₀^∞ K(t) sin(ωt) dt                    （Ogilvie / Kramers–Kronig）
```

w_j 是浮筒 j 中心的垂直速度。A∞ 就是 0.7 的瞬時附加質量 0.5·ρ·V_wet，與 Hulme 半球的高頻極限 0.5 相同；
記憶積分自動產生隨頻率變化的附加質量。記憶長度 8 s，以每個 0.005 s 小步的速度歷史做梯形積分。

- **單一浮筒**：B(ω) 取與浮筒水線面半徑 a 相同的浮動半球的精確值（Hulme 1982,《J. Fluid Mech.》121:443–463；
  數值取自 [LHEEA 的重製表](https://github.com/LHEEA/Hulme_Heaving_Sphere)），以 ka = ω²a/g 線性插值。
  這對半球精確，對其他吃水在長波極限也正確（阻尼只取決於水線面），中間是近似；未與其他吃水的精確解比較。
  水線面半徑由取樣的浸水比例反推，以 0.5 s 指數平均；變化超過 1 cm 或 3% 才重算核函數。剛入水時阻尼會在約 1 s 內建立。
- **浮筒間干擾**：點吸收體近似 B_ij = √(B_i B_j)·J₀(k d_ij)（Budal 1977；Evans 1980），d_ij 為模型座標中的水平距離。
  [J₀(k d_ij)] 是半正定矩陣，所以輻射只能帶走能量。波從一個浮筒傳到另一個需要時間（d/c_g，平台約 2–3 個週期），
  因此平台起伏不是單一衰減模態，每個週期的衰減會起伏。
- **為什麼不用 Froude–Krylov＋Haskind**：以半球驗證，忽略繞射的 FK 激振力代入 Haskind 關係 B = k|X|²/(4ρg c_g)，
  在 ka ≈ 1 時阻尼高估約 2 倍，A(0) 也偏離 Hulme 40%，所以改用精確資料。

單元測試：Hulme 表本身滿足 Kramers–Kronig（(1/π)∫B̂/ka d(ka) = A(0) − A(∞) = 0.331）；J₀ 近似式與積分表示一致；
規定正弦起伏時，記憶力的同相分量重現 B(ω)（誤差 < 2%），反相分量重現 Hulme 另一欄的附加質量（< 0.03 ρV）；
兩個重合浮筒同相輻射 4 倍功率、反相為 0，相距 60 m 幾乎不互相影響；隨機速度歷程下陣列輻射作功不為正；
半浸水球自由起伏的頻率與對數衰減率符合只用 Hulme 表推出的線性理論（ka = 3/(2(1+μ))、ζ = λ/(2(1+μ))，誤差 3% 與 15%）；
平台關掉二次阻力時，有輻射 10 s 內起伏降到 5% 以下，沒有輻射則不衰減。

限制：只有浮筒的垂直（heave）輻射；單一浮筒的水平與轉動輻射、近場（非波動）干擾、有限水深、池壁反射、
完全浸沒的浮筒（沒有水線面）都沒有模擬。

## 與輻射一致的垂直繞射力（0.9）

0.8 的入射波垂直力是 Froude–Krylov 壓力積分加 C_a ρ V a_z（C_a = 0.5），和 Hulme 的輻射資料不一致：
以固定半球對照精確解，ka ≤ 0.6 時誤差約 6% 以內，ka = 1 時低估約 30%，ka ≳ 1.5 時連方向都錯。
0.9 為穿出水面的浮筒改用半球的**垂直激振力表** X̂ = X₃/(ρgπa²A)（複數，e^{−iωt}，相對球軸處的入射波面）：

```text
F_diffraction = Σ ρ g π a² A Re{ (X̂ − X̂_FK)(k a) e^{iθ} },   θ 為該波在浮筒中心的相位
```

壓力積分已給出 Froude–Krylov 部分，所以只加上繞射部分（X̂ − X̂_FK），並在這些浮筒上拿掉垂直的 C_a ρ V a_z；
水平方向仍是 Morison 形式。a 是當下（未平滑）的水線面半徑，與輻射一樣用同水線面半徑的半球資料；
有限水深的波用其實際波數查表（表本身是深水）；ka 3～4 之間把繞射部分線性漸弱到零（數值接合，不是資料）。

**表的來源**：本專案以 Havelock (1955)／Hulme (1982) 的多極展開法自行求解半球的輻射與繞射問題（沒有找到公開的完整相位表）。
同一套計算以 4 位數重現 Hulme 的附加質量與阻尼；|X̂| 與 Hulme 公開阻尼經 Haskind 關係 B = ω³|X|²/(2ρg³)
推得的值一致到 0.2%；多極截斷改變 |X̂| 小於 2e−5、相位小於 0.02°。相位沒有公開資料可對照，
只有長波極限 Im X̂ ≈ −(2/3) ka · B/(ρVω)（力領先波面）可檢查。來源：Hulme 1982, JFM 121:443；Newman 1962, J. Ship Res. 6(4):10（Haskind 關係）。

單元測試：表的 |X̂| 與 Hulme 阻尼的 Haskind 關係（0.3% 內）；X̂_FK 欄與入射壓力在半球面上的直接積分（2e−4 內）；
長波相位；固定半球用取樣壓力積分＋表格繞射力重現 X̂ 的實部與虛部（0.01 內）；
半浸水球在規則波中的時域起伏反應，以 Hulme 的 A(ω)、B(ω) 與 X̂ 算出的頻域 RAO 對照，ka 0.3／0.9／1.4 誤差 4% 內，
舊的 Morison 繞射在 ka 1.4 的誤差至少是新方法的 3 倍。測試把球限制為只能起伏：未繫泊的球在起始暫態後會水平漂移約 1 m，改變受力相位。

## 螺旋槳推力（0.6）

使用四象限係數（Carlton,《Marine Propellers and Propulsion》；ITTC）：

```text
β    = atan2(V_A, 0.7 π n D)
V_r² = V_A² + (0.7 π n D)²
T = C_T*(β) · ½ ρ V_r² · π D²/4
Q = C_Q*(β) · ½ ρ V_r² · π D²/4 · D
J = V_A / (n D),   K_T = C_T* · (π/8)(J² + 0.49 π²)
```

n 是轉／秒，正值表示繞推力軸右手旋轉（由後方看順時針）並沿軸推進。V_A 是槳轂相對當地流體的速度在軸向的分量。
物體受到軸向推力，以及軸扭矩的反作用 −Q·axis；軸功率為 2πnQ。`PropellerTable` 可載入量測資料，超出範圍拒絕。

內建 `BladeElementPropeller` 是葉素動量理論（BEMT）：等弦長 c/D、等幾何螺距 P/D、Prandtl 葉尖與轂部損失
F = (2/π) acos(exp(−B(R−r)/(2r|sin φ|)))。每個環以入流角 φ 為唯一未知數，沿用
[Ning (2014), Wind Energy 17:1327, DOI 10.1002/we.1636](https://scholarsarchive.byu.edu/facpub/1673) 的單變數殘差思路：

```text
R(φ) = V_y (sin φ − σ C_n / (4F|sin φ|)) − V_A cos φ = 0
V_y  = Ω r / (1 + σ C_t / (4F |sin φ| cos φ)),   V_x = V_y tan φ
C_n = C_L cos φ − C_D sin φ,   C_t = C_L sin φ + C_D cos φ,   σ = B c / (2π r)
```

動量關係使用 dT = 4πrρ·u·|V_x|·F dr，從正常推進狀態連續延伸到風車制動狀態；
渦環與紊流尾流狀態中的實際流場不穩定，這個延伸不代表那些狀態正確。φ = 0 是可去奇點，所以靜推力（V_A = 0）正常。
反轉時假設葉剖面前後對稱（雙向推進器葉片），T(−n, −V) = −T(n, V)。

內建剖面 `THIN_SECTION` 為升力斜率 2π、CD0 0.01、無誘導阻力的示範值，不是量測資料。
失速的轂部剖面可能有多個平衡解，程式取最接近無誘導入流角的解；直接計算在某個解消失處會跳約 1%（靜推附近）到 4%（n 接近 0）。
遊戲使用每 1° 一點的線性插值表，力隨狀態連續變化，但沒有遲滯或非定常失速。
不含雷諾數效應、空化、通氣、斜流側力、船體伴流、槳後尾流與舵。

舊的示範槳（D 0.3 m、3 葉、P/D 1、c/D 0.25、轂比 0.2、薄翼剖面）的模型輸出：K_T(J=0) ≈ 0.39、靜推 C_Q* ≈ 0.023，零推力 J 約 0.9～1.0。

### Wageningen B 系列校準（0.9）

`WageningenB` 實作 Oosterveld & van Oossanen (1975) 的 K_T（39 項）與 K_Q（47 項）回歸式（Rn = 2×10⁶，Rn 更高時加 Bernitsas 等的修正）。
係數取自 Bernitsas, Ray & Kinley (1981), Univ. Michigan NA&ME Publ. 237, Table 1／2 的掃描，並與 Fossen MSS 等另外五份副本逐項比對
（只有一個 GitHub 版本的 K_Q 第 46 項有誤植）。適用範圍 2 ≤ Z ≤ 7、0.30 ≤ A_E/A_0 ≤ 1.05、0.5 ≤ P/D ≤ 1.4，只有第一象限（J 從 0 到推力為零）。

這是量測資料的回歸，也是對照基準。對稱剖面、等弦長的葉素動量理論無論怎麼調都配不上它的曲線形狀
（最佳擬合仍高估靜推 8%、零推力 J 0.99 對 1.087），所以示範槳改成：

- **前進象限**（ahead、β 從 0 到零推力）：直接用回歸式；
- **其他象限**：0.9 用同葉面積的葉素動量理論；0.10 起改用 MARIN 量測的四象限 Fourier 資料（見〈0.10 係數來源〉）；
- 兩段之間在 β 方向以 15° 的 cos² 漸變區接合，接合本身不是資料。

單元測試（對公開的範例，不是對同一回歸式自己）：MIT OCW 2.611 的 B5-75 設計例（J 0.64／0.74、η 0.61／0.67）、
密西根大學 POP 程式輸出（Z 4、A_E/A_0 0.6293、P/D 0.8171、J 0.5408、Rn 3.68×10⁷ 時 K_T 0.1615、K_Q 0.02205）、
Bernitsas 圖 21 的 B3-50 零推力 J（1.08～1.09，讀圖）；示範槳前進象限等於回歸式、漸變區沒有跳躍、鎖死時只產生阻力。
0.9 的限制（倒車與風車象限是假設前後對稱的葉素動量理論）已在 0.10 由量測的四象限資料取代；葉素動量理論仍保留在 API 中。

單元測試：靜推軸功率不低於致動盤理想功率 T^1.5/√(2ρA)（品質因數介於 0.3～1）；
J = 0.2～0.8 的效率低於 Froude 理想效率 2/(1+√(1+C_T))；零推力 J 落在幾何螺距 P/D 之下；
相似律（n、V 加倍時 T、Q ×4；D 加倍時 T ×16、Q ×32）；插值表與直接解相差 < 1%；
反轉對稱；鎖死槳只產生阻力；推力與反扭矩的世界方向。

## 細長船體：阻力與操縱（0.9）

`SlenderHull` 以一串橫剖面（凸多邊形，局部 y 上、z 右舷）描述船體，每個剖面代表到相鄰剖面一半的一段。
`HullModel` 在設計狀態（浮起給定質量的水線，或完全潛航）一次算好係數，`HullDynamics` 每步逐段積分。
適用於排水型的水面船與潛航體；**不是滑行艇模型**（體積 Froude 數超過約 1.2 開始半滑行，沒有滑行升力、下沉／縱傾）。

**靜水壓**：每段剖面被當地水面切開，算出精確的濕面積、形心、水線寬、吃水與濕周（任何橫傾、縱傾皆可）。
波浪中，靜水面與瞬時水面之間那一塊乘上 e^{−k d}（d 為平均底深 A/B），近似線性動壓隨深度衰減。

**阻力**（沿船軸，相對當地水流）：R = ½ρU²S[(1+k)C_F + C_W]。

- C_F：ITTC-1957 摩擦線 0.075/(log₁₀Re − 2)²（Re 小於 10⁵ 時固定）。
- 1+k：水面船用 Holtrop & Mennen (1982)，潛航體用 Hoerner 的旋轉體式 1 + 1.5(D/L)^1.5 + 7(D/L)³。
- C_W：Michell (1898) 薄船積分（Tuck 1989 的形式），在設計水線上一次算好 Fn 0.1～2.0 的表（Fn 0.6 以下每 0.02 一點）。

Michell 假設船體閉合、無下沉縱傾、不碎波，在 Fn 0.2～0.35 會高估峰谷；寬的浸水艉板會被當成強波源，所以艉板阻力不在範圍內。

**附加質量與理想流體力**：每段的縱向（按體積分攤 Söding 1982 的 m₁₁ = 2.7ρ∇^{5/3}/L²；潛航體用 Lamb 的長橢球 k₁）、
橫向 m₂₂（穿水面剖面用 Lewis 形低頻值，完全潛入用外接橢圓 ρπc²）與垂直 m₃₃（Lewis 高頻值，潛入時 ρπb²）
組成機體座標 6×6 的 A，在速度更新中隱式處理；另外顯式加上 Kirchhoff／Fossen 的 A(ω×v) − C_A(ν)ν，
所以 Munk 力矩 (m₂₂ − m₁₁)uv 與附加質量的科氏耦合自然出現。

**升力元件**：在尾端站位 x_T 加橫向力 −|u|·M_L·w(x_T)（倒車時 x_T 換到船首的鏡像位置）。
水面船以 Clarke, Gedling & Hine (1983) 的 Y'_v 與 N'_v 決定 M_L 與 x_T（扣掉 Munk 偶力），
所以這兩個導數等於 Clarke；Y'_r、N'_r 是條帶附加質量、Kirchhoff 項與升力元件的預測。
潛航體在兩個橫向平面都有升力元件，以 DARPA SUBOFF 裸船體（Roddy 1990）的 Y'_v/(D/L)² 與 x_T = x_B − 0.51L 縮放。

**橫流阻力**：每段 −½ρC_D·(投影高)·|w|w；水面剖面的 C_D 依 B/(2T) 查 Hoerner 資料（Leite 等 1998 數位化），
垂直方向取粗略的 1.0；潛航圓剖面用 DNV 的圓柱曲線 × Jorgensen η(L/D)。乾的部分另有空氣橫流阻力（C_D 1.2）。
波浪中每段另有 Froude–Krylov ρA a（水平；潛入時含垂直）與以條帶附加質量計的 Morison 繞射。
垂直方向的波浪輻射可由呼叫者把船體分成數段，交給 `WaveRadiation.Memory` 當點吸收體陣列（`radiationPoints`／`segmentRadii`）。

**驗證（單元測試）**：

| 項目 | 對照 | 結果 |
| --- | --- | --- |
| Michell 積分 | Wigley 船體 Fn 0.25／0.316／0.35／0.40 的獨立表（1.06／1.83／1.25／2.73 ×10⁻³） | 2% 內，含 Fn 0.50 的主峰 |
| Holtrop–Mennen | 原論文算例 L_R 81.385、1+k₁ 1.156 | 一致 |
| Wigley 總阻力 | Kajitani 等 (1983) Fn 0.267 的 C_T 4.51～4.71×10⁻³ | 本模型（4 m）落在 4.2～5.0×10⁻³ |
| 剖面幾何 | 箱形任意橫傾的精確面積、水線、吃水 | 1e−12 |
| Lewis 形 | 半圓 m₂₂ = m₃₃ = ρπR²/2、方形 1.433 | 一致 |
| 長橢球 | Lamb 的 k₁、k₂、k′（L/D 6：0.045／0.917／0.762） | 一致 |
| 理想流體 | 中性浮力長橢球自由運動：世界衝量與「物體＋流體」動能守恆，Munk 力矩使它轉向橫置 | 20 s 內 1% |
| 操縱導數 | KVLCC2 實測（Yasukawa & Yoshimura 2015）：Y'_v −0.0207、N'_v −0.0090、Y'_r 0.0040、N'_r −0.0032 | 本模型 −0.0257（同 Clarke，+24%）、−0.0089、0.0058（+44%）、−0.0026（−19%） |
| 浮態與動量 | 設計水線上平浮、一步的動量收支 | 5 mm、1e−6 |

`TestHull`（6 m 圓舭小艇，示範用，沒有實測資料）只做合理性檢查：12 轉/秒時約 3.3 m/s，舵 20° 時約 3 倍船長的迴旋直徑、迴旋時減速、
橫傾後有回正力矩。Clarke 式在原論文的適用範圍未查證；KVLCC2 的數字是二手（ShipMMG.jl）。

## 舵與螺旋槳尾流（0.9）

`Rudder` 是繞局部垂直軸轉動的懸舵，用 `AeroSolver` 的升阻力；正角度使後緣偏向右舷、船首右轉。
低展弦比舵的極曲線（0.10）：有效展弦比取幾何值的兩倍（船體當鏡面），升力斜率用 Whicker & Fehlner（1958）的 0.9·2πa_e/(√(a_e²+4) + 1.8)，
CD0 0.0065（NACA 0015）、誘導阻力效率 0.9；失速角 17.5° 取 Molland & Turnock（2007）開放水域 15～20° 的中點，漸變 10° 是模型形狀；
槳流中 30～40° 的延遲失速與 α² 橫流升力項沒有模擬。
舵在螺旋槳後方時，軸向來流以致動盤尾流提高：u² = u_a|u_a| + η·8T/(ρπD²)，η 為槳流覆蓋的展長比例（D/展長，最多 1）；
沒有尾流收縮、旋轉、船體伴流與舵桿。

## 多個物體的聯合接觸（0.9）

`ContactSolver.solveAll` 把方塊接觸與物體之間的接觸（`generatePair` 的球對球，含推測接觸）放進同一次投影 Gauss–Seidel，
物體之間以等大反向衝量作用；`separateAll` 依各自的逆質量把穿入投影開，只改姿態。配對材料用 `combine`（摩擦係數幾何平均、恢復係數取大）。
摩擦熱：方塊接觸照舊交給呼叫者依熱滲透率分配，物體之間的摩擦熱兩邊各分一半。
`FloatingBodyDynamics` 與 `HullDynamics` 都拆成 `prepare`（受力與隱式速度）和 `finish`（漂移與方塊投影），單一物體的 `step` 仍等於兩者相接。
單元測試：一維正碰的恢復係數公式（e 0／0.5／1）、隨機偏心摩擦接觸的總動量與角動量守恆、兩個箱子疊放的支撐力、投影只改姿態且依逆質量分配。

## 阻力耗散與摩擦生熱（0.6）

`FloatingBodyDynamics.Result.dissipation` 是本步阻力移除的機械功率 D = Σ c_i |u_i|²，
u 為各阻力元件相對當地流體的速度，c 為隱式步使用的係數；空氣與水分開，且不受參考系等速平移影響。
後向 Euler 阻力的能量收支為 KE₀ − KE₁ = D·dt + ½|Δv|²_M，所以 D 與實際失去的動能相差 O(dt)；
測試在 dt = 0.0025 s 時差距小於 1%，且隨步長減半而收斂。

傳到物體的熱採回復溫度（絕熱壁溫）模型：

```text
T_aw − T = r u² / (2 c_p),   紊流 r = Pr^(1/3)，層流 r = Pr^(1/2)
q_friction = min(G (T_aw − T), D)，G = h A
```

r = 1 時即停滯溫度，測試以理想氣體的 T₀/T = 1 + (γ−1)M²/2 獨立核對；空氣的紊流 r ≈ 0.89。
這部分熱量來自阻力耗散，所以以該介質的 D 為上限（熱力學第一定律），其餘留在流體；環境是固定溫度熱庫，不會升溫。
u² 取乾（空氣）或濕（水）面積加權平均。仍是均溫模型：沒有空間溫度場、隨流速變化的 h，接觸摩擦也不生熱。

## 0.10 係數來源

0.10 把示範物體與環境所用的係數換成文獻值；仍沒有資料的地方明確寫出採用的規則。

| 項目 | 現在用的值 | 來源 |
| --- | --- | --- |
| 浮筒／測試球阻力 | 光滑球標準曲線 C_D(Re)，十段 | Clift, Grace & Weber 1978 Table 5.2（`SphereDrag`；Re > 4×10⁵ 的兩段僅核對連續性） |
| 接觸摩擦 | 鋼對岩石 0.4；鋼對鋼 0.57 | Ziogos 等 2023（低碳鋼對安山岩殘餘摩擦角 21°；未量花崗岩）；NASA RP-1228 Table IV |
| 接觸恢復係數 | Thornton 彈性–完全塑性球撞擊 e(V)，V_y = 5.1√(Y⁵/(E*⁴ρ)) | Thornton 1997（經 Jackson 等 2010 eq. 2）、Johnson 1985；鋼 Y 250 MPa（ASTM A36）、E 200 GPa；花崗岩 E 59.3 GPa、ν 0.23（Johnson & DeGraff 1988） |
| 平台熱容與熱滲透率 | 普通碳鋼 c_p 434、k 60.5、ρ 7854；地面 Barre 花崗岩 2.79／2630／775 | Incropera Table A.1／A.3（經 Çengel） |
| 浮筒對流 | Whitaker 強制對流 ＋ Churchill 自然對流，Nu³ = Nu_F³ + Nu_N³ | Whitaker 1972；Churchill 1983；Incropera eq. 9.89 |
| 船體垂向橫流阻力 | 鏡像 2T×B 圓角矩形的 C_D | DNV-RP-C205 Table E-1 第 4 項（`HullModel.heaveCrossFlow`） |
| 乾舷風阻 | 側向 0.90、垂向 1.9；縱向未模擬 | Blendermann 1994（快艇 CD_t）；DNV-RP-C205 平板 |
| 舵 | 見上〈舵與螺旋槳尾流〉 | Whicker & Fehlner 1958；Molland & Turnock 2007 |
| 螺旋槳 | 前進象限回歸式，其他象限 MARIN 四象限 Fourier（B3-65、B4-70 P/D 1.0／1.4） | Roddy, Hess & Faller 2006 附錄 B（B4-70 P/D 1.0 已對原表核對；其他與回歸式在 J 0～0.9 相差 6% 以內） |
| 原版實體質量 | 依物種的成體代表質量；虛構生物與掉落物用人類質量／玩家碰撞箱體積 95.7 kg/m³ | 見 `EntityMasses` 註解（CDC、USDA、NOAA、ADW 等） |
| 原版水流速度 | 1 單位 = 1.12 m/s | 原版生物在滿強度水流中的漂移速度 0.014·0.8/0.2 格/tick |

測試：`CoefficientDataTest`（Clift 值、Churchill 檢查值 19.52、Thornton 極限、DNV 表點、Whicker–Fehlner 斜率 1.40／2.44／3.14）、
`BSeriesFourQuadrantTest`（量測四象限與回歸式比較、B4-70 繫泊值 K_T 0.4499）。

## 0.11 新增模型與係數來源

| 項目 | 模型 | 來源與文件 |
| --- | --- | --- |
| 二階水面 | Choppy Wave Model 二階展開 η − D·∇η + M；單一深水波即 Stokes 二階 | Nouguier, Guérin & Chapron 2009；[海況](SEA.md) |
| 非定常尾浪 | 線性自由面、FFT 模態在移動座標中精確積分；薄船以雙體壓力表示 | Wehausen & Laitone 1960；[海況](SEA.md) |
| 碎浪與白沫 | 向下加速度門檻，面積對準 Monahan 白冠覆蓋率；白沫 3.5 s 衰減 | Snyder & Kennedy 1983；Longuet-Higgins & Fox 1977；Monahan 1980、1988；[海況](SEA.md) |
| 原版船 | 52 格靜水力、Hulme 半球輻射、ITTC 摩擦、Hoerner 橫流、以自身尾浪計算的興波阻力、槳葉滑移模型 150 W | [原版船](ROWBOAT.md) |
| 滑行艇鬚狀水花阻力 | D = ρV²C_f b² cos2α/(8 sin2α cosβ)，tanα = π tanτ/(2 tanβ) | Savitsky, DeLorme & Datla 2007；[滑行艇](PLANING.md) |

**方塊材質（鋼對各類方塊，`ContactMaterial`、`BlockMaterials`）**：Minecraft 沒有材質資料，方塊依標籤與音效類型分類。

| 類別 | 摩擦 | 恢復係數 | 來源 |
| --- | --- | --- | --- |
| 岩石（石頭、磚、混凝土、陶瓦、礦石等，預設） | 0.4 | Thornton，鋼的降伏 | 見上 0.10 |
| 木材 | 0.3 | Thornton，p_y = 橡木側面硬度約 35 MPa、E_T 0.9 GPa；1 m/s 時 0.70 | Lemoine 1970、Atack & Tabor 1958（經 Zheng 等 2024）；Wood Handbook 表 5-1 |
| 冰 | 0.01（約 −10 °C；溫度未模擬） | Thornton，p_y = 壓痕硬度 100 MPa、E 9.33 GPa；1 m/s 時 0.43 | Weber 等 2018；Butkovich 1954（經 Tusima 1977）；Gammon 等 1983 |
| 沙 | 0.30 | 0（嵌入） | NAVFAC DM-7.02 表 1；Brisset 等 2018 |
| 礫石 | 0.40 | 0.31 | NAVFAC DM-7.02；RocFall 碎石坡 0.30～0.33 |
| 泥土、草 | 0.30 | 0.30 | 鋼對黏土 tanδ 0.28～0.33；RocFall 軟土 0.28～0.32 |
| 泥巴 | 0.25（附著力未模擬） | 0 | Birch, Ekwue & Phillip 2016 |
| 雪 | 0.113 | 0 | Aggarwal, Das & Gusain 2024 |
| 羊毛 | 0.3（規則：皮革對金屬 0.30～0.56 之間） | 0（規則） | 沒有找到鋼對織物的量測 |
| 玻璃 | 0.5（手冊 0.5～0.7 的低端） | 0.9（玻璃在破裂前是彈性的） | 破裂未模擬 |
| 金屬 | 0.57 | Thornton，鋼對鋼 | NASA RP-1228 |

- Thornton 的 p_y 用壓痕硬度：用體積降伏強度時，冰、木、混凝土的恢復係數遠低於量測值；用硬度時，冰對冰在 1 m/s 為 0.77，量測 0.68～0.79（Deckers & Teiser 2016）。
- 實體的碰撞箱仍用物體自己的預設材質；摩擦生熱的熱量分配仍以花崗岩計算。
- 測試：`BlockMaterialTest`（恢復係數範圍、冰面上一步的摩擦衝量與岩石成 μ 比）；GameTest：鋼球落在石頭上反彈、落在沙上不反彈。

## 0.12 新增模型與係數來源

| 項目 | 模型 | 來源與文件 |
| --- | --- | --- |
| 浪頭（完整二階） | Sharma–Dean 二階核減去 Choppy Wave Model 的差，以低秩二次式（Jacobi 特徵分解，捨去 1% Frobenius）加上；浪頭分布對 Forristall 在 0.6% 內 | Sharma & Dean 1981；Forristall 2000；[海況](SEA.md) |
| 波浪碰到地形 | 測量海床（2 m 格點）；程函方程 fast marching 求相位、沿波線保持 E c_g、焦散上限 2.5、Sommerfeld 半平面遮蔽、水深飽和 Hs ≤ 0.59 h 與陡度 0.1 的碎浪上限；包絡相量套用到所有讀取者，物體用局部 WKB 平面波 | Dean & Dalrymple 1991；CEM II-3、II-4；Sethian 1996；Penney & Price 1952；Thornton & Guza 1983；[海況](SEA.md) |
| 風 | 對數律與 COARE 3.0 粗糙長度；JONSWAP 受風域限制成長到 Pierson-Moskowitz；風驅動表層流 0.53 u*；白冠用實際風速 | Fairall 等 2003；Hasselmann 等 1973；Wu 1975；Monahan & O'Muircheartaigh 1980；[海況](SEA.md) |
| 原版船受風 | 乾舷 C_D 0.90、坐姿乘客 C_D A 0.35／0.25 m² | Blendermann 2004；Crouch 等 2017；[原版船](ROWBOAT.md) |
| 單槳轉向 | 槳葉相對船的速度上限 = 槳把速度 1.3 m/s ÷ 內段 × 槳葉臂 × 掃角平均 cos（85°）= 2.65 m/s | Kleshnev 2005；Körner（FISA）；[原版船](ROWBOAT.md) |
| 乘客隨船傾斜 | 座位點與模型隨船身俯仰、橫搖轉動；第一人稱鏡頭依視線加滾轉（顯示，不是新的力） | [原版船](ROWBOAT.md) |
| 滑行艇船艏仰角 | Garme 艉板附近壓力降 tanh(2.5x_T/(0.34 B C_V)) 乘在靜水壓、隨通氣淡入；航行俯仰與 Fridsma 差 0.35° 內，縱搖不穩起始點與水槽一致 | Garme 2005；Fridsma 1969；[滑行艇](PLANING.md) |
| 水花受風 | 二次空氣阻力，1 mm 水滴終端速度 3.9 m/s | Gunn & Kinzer 1949；[海況](SEA.md) |

## 適用範圍

適合原型、遊戲玩法與接入載具物理的初始版本。
未實作翼面互相干擾、尾流、遮蔽、地面效應、動態失速、可壓縮流、超音速、
通用可壓縮翼型修正、引擎模型、旋翼或氣象系統。Minecraft 環境需由載具／環境模組接入。
沒有三維流固耦合或空化；環境波浪是給定的流場（不規則海為 JONSWAP 合成，水面形狀到二階、水下仍線性），
海岸以幾何光學處理（無反射、島後只保留一個到達波），尾浪是每個來源一個 FFT 網格的線性非定常解；
碎浪只以振幅上限與白沫表現，沒有碎浪動力、湧升或沿岸流；風是穩定、水平均勻的，沒有陣風。
細長船體模型是排水型；滑行艇另有 2D+t 模型（[滑行艇](PLANING.md)）。艉板阻力、淺水效應、潛航體近水面興波與船間水動力干擾（尾浪除外）都未模擬。
0.4 已提供旋轉剛體積分，但平移測試球尚不模擬完整載具。
0.5 的溫壓流體性質、熱容量與熱交換模型見 [熱力學與視覺化](THERMAL_DEBUG.md)；不含相變或空間溫度場。

單元測試涵蓋公式數值、速度平方律、密度／面積縮放、相對風速不變性、
正負攻角、失速連續性、旋轉局部流、力矩、座標旋轉、阻力耗散、標準大氣參考值、
球冠體積／形心、浮力平衡、黏滯阻力極限、資料表插值，以及適用範圍拒絕等性質。
