# 氣體動力學工具（`api.gasdynamics`，0.28）

給衝壓、超燃衝壓、後燃器、進氣道與高速氣動共用的可壓縮流工具。純 Java、SI 單位、角度用弧度。

## 內容

| 類別 | 內容 | 氣體 |
| --- | --- | --- |
| `Thermo` | 熱完全氣體的介面：`enthalpy`、`cp`、`entropyFunction`（φ = ∫cp/T dT）、`gamma`、`speedOfSound`、反函數。`api.engine.Gas`（空氣、燃燒產物）與引擎的 `GasPath.Mix` 都實作它；`Thermo.perfect(γ, R)` 是表上的熱力完全氣體 | — |
| `GasState` | 靜溫、靜壓、速度；密度、馬赫數、總溫、總壓、質量通量、單位面積流推力 p + ρV² | — |
| `PerfectGas` | 等熵比值（T0/T、p0/p、ρ0/ρ）、面積比 A/A* 與反解（亞音速／超音速支）、質量流參數、Prandtl-Meyer 函數與反解、馬赫角、進氣道起動極限（等熵與 Kantrowitz-Donaldson） | 定 γ |
| `NormalShock` | 正震波比值；`behind(gas, 狀態)` 以質量、動量、焓守恆對熱完全氣體疊代密度比（Anderson，Hypersonic，14.3 節） | 定 γ／變物性 |
| `ObliqueShock` | θ-β-M、弱解與強解、最大偏折（脫離）、`behind(gas, 狀態, θ)` 的變物性斜震波 | 定 γ／變物性 |
| `ConicalShock` | Taylor-Maccoll 錐形流（RK4 從震波積到表面，二分找震波角）：震波角、表面馬赫數、表面壓力與溫度比 | 定 γ |
| `Rayleigh`、`Fanno` | 加熱與摩擦的定截面流比值與反解 | 定 γ |
| `ThermalFlow` | 變物性的音速狀態、膨脹到某壓力、給定馬赫數或質量通量（兩支）的狀態、Prandtl-Meyer 轉折（沿等熵線積分 dν = √(M²−1) dV/V） | 變物性 |
| `QuasiOneD` | 廣義一維管流（Shapiro 影響係數）：面積變化、加熱、壁面摩擦、加質（含注入的軸向速度比）同時作用，RK4 對 (ln p, ln T, ln V) 積分；在非喉部到達 Mach 1 就回報壅塞與位置 | 變物性 |

`QuasiOneD` 用的五個關係（連續、動量、狀態、能量 cpT dT/T + V² dV/V = dh0）解出
dV/V = (dh0/(cpT) − (c2 − c1)) / (1 − M²)，在 Mach 1 奇異：這就是熱壅塞與摩擦壅塞。混合造成的總焓變化由呼叫者算進加熱量。

## 驗證（`GasDynamicsTest`）

| 檢查 | 參考 | 容差 |
| --- | --- | --- |
| 等熵比值、A/A*、質量流參數、Prandtl-Meyer（M 0.5、1、1.5、2、3） | NACA Report 1135 表 I、II | 1e-4 |
| 正震波（M2、M3）、斜震波（M2 偏 10°、M3 偏 20°、M2 最大偏折 22.97°） | NACA 1135；pygasflow 的 Anderson 第 4 章習題 | 3e-4 |
| 錐形流：M2.35、10° 錐的表面馬赫數 2.1469；M2、10° 錐的震波角 31.2°；M6、震波 14.35° 的錐 9.9978° | NASA Glenn NPARC「cone10」；Sims（NASA SP-3004）圖；MATLAB Answers 1991953 的獨立 RK4 | 2e-4、0.1°、0.002° |
| Rayleigh、Fanno 比值與反解（M 0.5、2） | NACA 1135 表 IV、V | 1e-4 |
| Kantrowitz 極限：高馬赫趨近 0.600；大於等熵極限 | Van Wie（AIAA Progress 189） | 1e-3 |
| 變物性函數用完全氣體時等於表 | 本表 | 1e-9～3e-4 |
| 熱空氣正震波（M1.5～10）質量、動量、能量守恆；M6 以上比完全氣體冷而密 | 守恆 | 1e-9 |
| 管流：只有面積變化＝等熵、只有加熱＝Rayleigh（含熱壅塞位置）、只有摩擦＝Fanno（亞音速與超音速）、加質時三個積分守恆、熱空氣加熱的能量守恆、步數收斂 | 解析解、守恆 | 1e-6 |

NPARC「cone10」頁面列出的震波角 27.18° 與表面溫度比 1.1063 在 γ = 1.4 下不守恆總溫（表面馬赫數 2.1469 與本實作相同），所以只用表面馬赫數。

## 限制

- 變物性只含熱完全（cp 隨溫度），不含解離與化學平衡：空氣約 2500 K 以上（約 Mach 7 的正震波後）會高估溫度。
- 錐形流只有定 γ、零攻角；震波脫離時丟例外。
- `QuasiOneD` 是一維平均：沒有邊界層分離、震波串與混合不均（超燃的隔離段與燃燒室另外處理）。
