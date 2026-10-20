# 動力與燃料計算（0.26）

套件 `dev.mcmod.aerocore.api.power`，給載具 Mod 的工廠動力：電機、電池、電纜、皮帶與天軸、蒸汽機、人力與固體燃料。
全部是能離線呼叫的計算，單位 SI。使用者 2026-10-13 要求機床「要電或是動力」（hdfv 0.8）。

## 電機 `ElectricMachine`

- 損失分成固定（鐵損與摩擦，和負載無關）與銅損（隨負載平方），如 IEC 60034-2-1 的分法；效率在兩者相等處最高（約額定的 0.82）。
- `of(額定 W)`：IEC 60034-30-1 IE3 四極 50 Hz 的額定效率（0.75 kW 82.5 %、7.5 kW 90.4 %、110 kW 95.4 %、200 kW 以上 96.0 %，
  對數功率內插），額定時損失 40 % 固定、60 % 銅損。
- 電動機：`inputForOutput`；發電機：`outputForInput`（解二次式）。

## 電池 `Battery`

- 容量、充放電效率、最大 C 率、質量。鎳鐵（愛迪生電池）：約 25 Wh/kg、來回 70 %、C/5；鉛酸：約 35 Wh/kg、來回 83 %、C/5
  （Linden and Reddy, Handbook of Batteries）。

## 電纜 `Cable`

- 銅導體 PVC 絕緣、三條載流、明敷（安裝方式 C）、30 °C：IEC 60364-5-52 表 B.52.4（6 mm² 41 A、95 mm² 223 A…）。
- 三相功率 √3 V I pf；電阻損失 3 I² ρ L / A（銅 0.0172 Ω mm²/m）。

## 皮帶與天軸 `Belt`、`Shafting`

- 平皮帶：緊邊張力上限＝容許應力×斷面，鬆邊＝緊邊 / e^(μθ)（Euler–Eytelwein），離心張力 m′v²；
  功率 (T1 − Tc)(1 − e^(−μθ)) v。皮革對鑄鐵 μ 0.35、2.5 MPa、970 kg/m³，效率約 0.96（Shigley 第 17 章）。
- 天軸：容許剪應力 28 MPa 時 τ π d³ / 16 × ω；滑動軸承摩擦 μ F r ω（μ 0.02）。

## 蒸汽機 `SteamEngine`

- 鍋爐的飽和蒸汽膨脹到大氣（不凝汽）：Rankine 理想功＝等熵膨脹的焓降減給水泵功；引擎得到其中的指示圖係數（0.5～0.7），
  再扣機械效率，鍋爐把燃料熱量的一部分給蒸汽。
- 飽和水與蒸汽性質：IAPWS-IF97（Cengel and Boles 表 A-5），100 kPa～2 MPa 線性內插。
- 1 MPa：Rankine 效率 16.4 %；預設的小型工廠蒸汽機（1 MPa、0.6、85 %、70 % 鍋爐）整體約 5.9 %，和天軸時代的簡單蒸汽機（5～10 %）相符。

## 管路與泵 `PipeFlow`、`Pump`

- 管路壓降：Darcy–Weisbach，層流（Re < 2300）摩擦係數 64 / Re，紊流（Re > 4000）用 Swamee–Jain 對 Colebrook 的顯式近似，中間線性內插；
  管件與閥門以速度頭的損失係數 K 相加（Crane TP-410；White, Fluid Mechanics 第 6 章）。商用鋼管粗糙度 0.045 mm。給壓差求流量用二分法。
- 離心泵：揚程 H0 (1 − (Q / Qmax)²)，水力功率最大處 Qmax / √3 當最佳效率點，效率在兩側以拋物線下降；工作點是泵曲線和系統曲線的交點；
  軸功率 ρ g Q H / η（Karassik, Pump Handbook；Gülich, Centrifugal Pumps）。`forDuty` 給最佳效率點的流量與揚程。

## 人力與固體燃料

- `HumanPower`：用腿（踏板）可持續約 75 W，用手搖約 50 W（Wilson, Bicycling Science）。
- `SolidFuel`：煤 27、木炭 29、焦炭 28、風乾木材 15 MJ/kg（Perry's 第 24 節）。

## 燃料

- `Fuel.DIESEL`（C12.3H22.2，42.8 MJ/kg，840 kg/m³，十六烷值 51）、`Fuel.FUEL_OIL`（CH1.5，40.5 MJ/kg，980 kg/m³）、`Fuel.byName`、`Fuel.cetaneNumber`。

## 驗證

`PowerTest`：IE3 表值與損失分配、電纜表值與損失、皮帶與天軸的手算例、蒸汽表與 Rankine 效率、柴油化學計量空燃比 14.5、50 mm 鋼管 2 L/s 水的壓降（手算 24.8 kPa）、層流對 Hagen–Poiseuille、泵的工作點。
