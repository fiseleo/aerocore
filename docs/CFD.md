# AeroCore 二維 CFD 實驗

`cfd` 是獨立的 Java 21 模組。它數值推進流場，不在 Minecraft tick 內執行，
不打包進遊戲 JAR。用途是驗證流體演算法和研究外形，尚不是完整工程級載具模擬。

## 執行

IntelliJ 選 **CFD Water Experiment** 或 **CFD Air Experiment**；或在專案根目錄執行：

```powershell
.\dev.ps1 :cfd:run
.\dev.ps1 :cfd:run --args="air"
.\dev.ps1 :cfd:run --args="water 24000"
.\dev.ps1 :cfd:test
```

參數依序為 `water|air`、步數（1～200000，預設12000）、可選輸出目錄。
相對輸出路徑以 `cfd` 模組目錄為基準。預設輸出在 `cfd/build/experiments/<介質>/`，
相同介質的下一次執行會覆寫該目錄內同名輸出；保留多次實驗時指定不同目錄。

| 檔案 | 內容 |
| --- | --- |
| `field.csv` | SI 座標、固體遮罩、密度、兩分量速度、壓力擾動 |
| `history.csv` | 每批步數的質量誤差、格子 Mach、密度偏差、障礙物受力、速度變化 |
| `report.txt` | 物理尺度、時間步長、邊界條件、Re、診斷值與限制 |
| `flow.png` | 選用繪圖腳本產生的速度／壓力圖 |

繪圖：`python scripts/plot-cfd.py cfd/build/experiments/water/field.csv`。
需先安裝 `scripts/requirements-cfd-plot.txt` 中的 Python 相依；Java 求解器本身不依賴 Python。
本機亦已備妥 `%USERPROFILE%\.mcmod\cfd-plot\Scripts\python.exe`，可用此解譯器執行繪圖。

## 方程與邊界

實作 D2Q9 單鬆弛 BGK 格子波茲曼法，搭配 Guo 體積力。
流體是等溫、單相、弱可壓縮的低 Mach 近似；並非粒子動畫或以紋理假裝流動。
方法參考 [Guo、Zheng、Shi (2002)](https://doi.org/10.1103/PhysRevE.65.046308)，
背景見 [Palabos 團隊的 Lattice Boltzmann 說明](https://palabos.unige.ch/get-started/lattice-boltzmann)。

```text
cs² = 1/3
nu_lattice = (tau − 1/2) / 3
f_eq_i = w_i rho [1 + 3(c_i·u) + 4.5(c_i·u)² − 1.5(u·u)]
u = sum_i(f_i c_i)/rho + acceleration/2
S_i = w_i rho (1 − omega/2) [3(c_i−u)·acceleration + 9(c_i·u)(c_i·acceleration)]
f_post_i = f_i + omega(f_eq_i − f_i) + S_i, omega = 1/tau
```

碰撞後沿九個方向串流；固體界面採 halfway bounce-back，壁面在相鄰流體／固體格點中間。
X 為週期邊界，Y 上下為不滑移壁面；圓柱靜止，用階梯網格近似邊界。
體積力驅動通道流，**沒有開放入口／出口**。圓柱實際上代表週期重複的受限陣列，
不能把受力直接當作無限遠均勻流中孤立圓柱的 Cd。
障礙物力由反彈動量交換累加，排除上下通道壁。

預設網格 128×48，格距 0.5 mm，名義圓柱直徑 6 mm，格子運動黏度 0.05、tau=0.65。
這是低 Re 的小尺度驗證案例，不代表真實飛機／船的尺寸或速度。
介質選 air/water 會改變 SI 時間、速度、壓力與力的對應；同一格子設定具有相同 Re，
因此兩個案例的無因次流場相同是設計結果。

## 單位與適用範圍

```text
nu_SI = dynamicViscosity / density
dt = nu_lattice × dx² / nu_SI
u_SI = u_lattice × dx / dt
pressurePerturbation_SI = (rho_lattice − 1)/3 × densityReference × (dx/dt)²
forcePerDepth_SI = force_lattice × densityReference × dx³ / dt²
```

`pressure_delta_pa` 是相對參考密度的壓力擾動，不包含大氣壓，也不包含體積力等效的線性壓力降。
二維的力是 **N/m（每單位出平面厚度）**；只有在幾何與流動可沿第三軸均勻延伸時，才可乘厚度換算成 N。
格子 Mach 使用格子聲速 `1/sqrt(3)`，與真實介質的物理 Mach 不同。
兩者都要符合各自的適用範圍，改動尺度不能只看其中一個。

程式限制格子 Mach ≤0.1、密度偏差 ≤5%、tau 在 [0.55,2]；若超出就報錯，
不以截斷速度製造假的穩定。這些限制不是通用穩定性或準確度保證。
高 Re 需要更細網格／適當碰撞模型，不能把水的黏度任意調大後仍聲稱保持原物理條件。

尚未提供三維、GPU、多重鬆弛、湍流模型、熱傳、聲學、自由水面／兩相、波浪、空化、
移動邊界、自動網格加密、流固雙向耦合，也未驗證高 Re 翼型的升阻力。

## 已完成的數值驗證

`LbmFlow2DTest` 包含靜水平衡、質量守恆、對稱圓柱零側向力、適用範圍保護與 SI 單位轉換。
平行板 Poiseuille 流與解析解比較：

```text
u(y) = acceleration / (2 nu) × y × (H − y)
```

| 通道高度 H（流體格點） | 相對 L2 速度誤差 |
| --- | --- |
| 16 | 0.00278138512（約 0.278%） |
| 32 | 0.000695376636（約 0.0695%） |

加密一倍後誤差約減為四分之一，符合這個平壁測試的二階收斂趨勢。
**平壁驗證不能替代圓柱／翼型的網格獨立性與實驗比較**。

預設 12000 步圓柱案例：質量相對變化約 −1.03×10⁻¹²、最大格子 Mach 約0.0311、
密度偏差約0.149%，最後500步的速度相對變化約2.98×10⁻⁴。
這是有限時間快照，沒有自動宣告已達穩態；應查看 `history.csv`，延長運算並做網格／邊界尺寸敏感度分析。
不同實作／平台的浮點誤差可能略有差異。

## 與遊戲核心的關係

遊戲使用 `physics` 即時求力；CFD 的目的在於提供經過驗證的流場和後續校準依據。
目前可以由 `PolarTable`／`ReynoldsPolar` 接入外部翼型資料，但沒有自動把這個圓柱案例轉成翼型表。
之後若研究實際載具，需要先決定幾何、尺度、速度、介質與邊界條件，再驗證係數並接入剛體運動。
