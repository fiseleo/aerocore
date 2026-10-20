# 氫燃料與儲存（0.28）

## 燃料（`Fuel.HYDROGEN`）

`Fuel` 從 0.28 起允許不含碳（碳加氫要大於零）。氫：LHV 119.96 MJ/kg、理論空燃比 34.3、液態密度 70.85 kg/m³（給噴油計算），
以氣體進燃燒室（汽化熱 0）；層流火焰速度在當量比 1.8 約 3.0 m/s、1.0 約 2.1 m/s（Dahoe 2005 的量測回顧，用同一個二次式擬合；
溫壓指數借用 Metghalchi-Keck，只在活塞引擎才用到）。富油產物只有水與剩下的氫，未燒的氫的熱值扣掉（`heatRelease`）。
`byName` 接受 `hydrogen` 與 `liquid_hydrogen`。

## 儲存（`api.storage`）

| 類別 | 內容 | 出處 |
| --- | --- | --- |
| `Hydrogen` | 液氫：沸點 20.37 K、70.85 kg/m³、汽化熱 445.6 kJ/kg；氣體壓縮因子 Z = 1 + α(T) p（293.15 K 時 α = 0.0068 /MPa，與 1/T 成正比）、密度；真實氣體等溫壓縮功 RT (ln(p2/p1) + α (p2 − p1)) / η；液化理想功 3.92 kWh/kg 與液化機性能係數（預設 0.33，約 11.9 kWh/kg）；蒸發 = 漏熱／汽化熱 | NIST（Leachman 2009 狀態方程）、arXiv 1702.06015、Barron《Cryogenic Systems》、Linde Leuna 廠 |
| `Insulation` | 真空多層隔熱（k ≈ 8e-5 W/(m·K)）、噴塗泡沫（k ≈ 0.017）、無隔熱（約 25 W/(m²·K)）的熱通量 k ΔT / t | Barron 第 7 章 |
| `PressureVessel` | 金屬圓筒與球、纏繞複材圓筒（網格分析 1.5 p r / σ）的壁厚與爆破壓力 | Roark、Peters《Composite Filament Winding》 |

## 驗證（`HydrogenTest`、`HydrogenFuelTest`）

| 檢查 | 參考 | 容差 |
| --- | --- | --- |
| Z：30 MPa 1.201、50 MPa 1.344（20 °C）、70 MPa 約 1.47；700 bar、15 °C 的密度 38.7～41.5 kg/m³ | arXiv 1702.06015（NIST） | 1～2 % |
| 20 → 700 bar 壓縮（等溫效率 0.55）2.2～2.8 kWh/kg；液化 11.9 kWh/kg | 加氣站文獻、Leuna | 範圍 |
| 170 L、30 mm 多層隔熱的車用液氫罐每天蒸發 1～6 %；同一罐用泡沫一天蒸發過半 | BMW Hydrogen 7 等級 | 範圍 |
| Mirai 等級的兩個 70 MPa Type IV 罐（122.4 L）質量 87.5 kg | Toyota；Argonne ANL-10/24 的有效層壓強度 2.0 GPa | 20 % |
| 氫的理論空燃比、貧與富的產物、富油熱值、燃燒室的燃油空氣比往返、理論燃燒溫度 2390～2700 K（無解離） | Heywood 附錄 D、量測 2390 K | 0.5 % |

## 限制

- 液氫只以漏熱算蒸發，沒有槽內分層、壓力上升與排氣閥的時序；正仲氫轉換只算在液化理想功裡。
- 壓縮因子是室溫附近的線性擬合（273～313 K、70 MPa 以下 1 %），不適用低溫壓縮氫。
