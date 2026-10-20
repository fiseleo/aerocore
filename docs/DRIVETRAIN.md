# 傳動系（0.24）

使用者 2026-10-12 要求「車子要有前驅、後驅、4WD 和前置、中置、後置」，並選了四種變速箱（手排、自排、DCT、CVT）、
三種四驅（全時、分時、適時）與「手排要自己踩離合器」。這一頁是 AeroCore 提供的傳動計算與變速箱、分動箱的機構；
車軸與差速器見 [WHEELS.md](WHEELS.md)。何時換檔、起步時離合器怎麼滑、CVT 對準什麼齒比是 hdfv 的控制規則，以輸入給零件：
包住 `TransmissionPart`／`TransferPart` 的零件在自己的 `loads` 裡用 `PartStep.override(role, value)` 改寫這一步的輸入（駕駛的輸入
經規則變成 `gear`、`clutch`、`lockup`…），再呼叫硬體；規則讀 `TransmissionPart.engaged(step)`、`shifting(step)`、`cvtRatio(step)`，
自己的記憶放在 `TransmissionPart.STATES` 之後的內部狀態。

## 軸系與約束列

- 剛性的軸（`Connection.Shaft`）照舊在編譯時把轉子併成「軸系」：一個軸系一個自由度（根轉子的轉速），其他轉子的轉速是它乘上齒比。
- 每一步會變的連接由零件在 `loads(PartStep)` 裡寫成「約束列」，在軸系之間：

| 方法 | 約束 | 扭矩範圍 |
| --- | --- | --- |
| `row(rotors, c, target, min, max, efficiency)` | Σ cᵢ ωᵢ = target（步末轉速，相對載具） | [min, max] N·m |
| `gear(a, b, ratio, efficiency)` | ω_a = ratio · ω_b | 不限 |
| `clutch(a, b, ratio, capacity)` | 同上 | ±capacity（打開時 0） |
| `hold(k, capacity)` | ω_k = 0（煞車、駐車棘爪用無限大） | ±capacity |
| `freewheel(from, to, ratio, efficiency, direction)` | direction · (ω_to − ω_from/ratio) ≥ 0（單向） | ≥ 0 |
| `differential(input, a, b, ratio, shareA, efficiency)` | ω_in = ratio · (s ω_a + (1 − s) ω_b) | 不限 |
| `biasFriction(a, b, preload, per, row)` | ω_a = ω_b（限滑） | ±(preload + per · \|另一列的扭矩\|) |

- 扭矩作用在轉子 kᵢ 上是 cᵢ 乘列的扭矩，車身承受反作用（變速箱殼、煞車卡鉗）：車身的反扭矩照舊來自 −Σ(I α − 外力扭矩)，所以所有零件內部的力矩自動平衡。
- 差速器的扭矩分配不隨轉速：輸入扭矩乘 ratio 後按 s : 1 − s 分給兩邊（傘齒 0.5、行星中央差速器是太陽齒輪的比例）。
- `biasFriction` 的容量跟著另一列（通常是差速器）的扭矩：離合器片式 LSD 是預壓＋斜坡，托森是扭矩偏差比 TBR：per = ratio·(TBR − 1)/(2(TBR + 1))。
- 平滑的耦合放進同一個隱式解：`viscous(a, b, ratio, c)`（黏性耦合）、`torquePair(a, b, T_a, T_b, ∂T_a/∂ω_a, ∂T_a/∂ω_b, ∂T_b/∂ω_a, ∂T_b/∂ω_b)`（液力變扭器）。
  兩個扭矩的和不為零時，差額由車身承受（變扭器導輪的反作用）。

## 求解（`Drivetrain`）

- 後向 Euler：A (ω′ − ω) = dt (T + J_fᵀ Λ)，A = diag(M) − dt K。K 的對角只取阻尼（負的斜率；正的照舊顯式，和只有軸系時相同），非對角照原值；
  A 奇異時改成只取對角。
- 每列的殘差 w = q + D Λ，D = dt J A⁻¹ J_fᵀ，和扭矩範圍互補：範圍內 w = 0（鎖住），在下界 w ≥ 0，在上界 w ≤ 0（打滑）。
  跟著另一列扭矩的範圍在上下界時是兩列扭矩的線性關係，一起解。
- 效率：J 是運動學（齒比），J_f 是扭矩的雅可比，差在損失：列的效率乘在「從列拿到功率」的那一邊；轉子到軸系根部的路徑效率，
  驅動的扭矩乘、負載的除（和只有軸系時對扭矩的處理一樣）。哪一邊拿到功率要看答案，所以用上一次的答案定係數再解，到不變為止（最多四次；沒有損失時一次）。
- 互補問題用 block principal pivoting（Júdice & Pires 1994）；錯的列數不再下降時改成一次只換最小索引的一列（Murty，對 P 矩陣有限步結束），
  再不行用投影 Gauss–Seidel。重複的鎖（同一個軸系兩個駐車棘爪）讓 D 奇異時，對角加最大項的 1e-10，鎖之間平分扭矩，轉速誤差同量級。
- 沒有約束列與交叉斜率的一步完全走原本的程式（`VehicleTest` 的 1e-9 對照不變）。有約束列時，單向離合器軸（`Connection.Shaft.freewheel`）
  變成單向的列，方向取驅動側單獨推進後的轉向（和原本一樣，靜止起步那一步就接合）。

## 滾動力與線阻尼（輪胎）

- `rollingForce(k, offset, direction, F₀, ∂F/∂ω, arm)`：地面對輪子的力 F = F₀ + ∂F/∂ω · (ω′ − ω)，求解軸系時轉子看到 arm · F 與它的斜率；
  之後同一個力（步末轉速）作用在車身的接觸點（`CONTACT`）。
- `damper(offset, direction, c, u_ref)`：車身上沿一條線的隱式阻尼 −c (d·u − u_ref) d（`RigidBodyDynamics.LineDamper`），
  給輪胎對車身速度的斜率（u_ref 取步初接觸點沿 d 的速度）與懸吊的避震器。
- 兩者分開解（先軸系、再車身）：各自隱式，所以耦合穩定；但同一步裡輪子看到的力和車身看到的力差 c·Δv，
  輪子與車身之間的動量在暫態有 O(dt·c/m) 的誤差（穩態相同）。測試：線性輪胎讓 110 kg 的車身與 1 kg·m² 的輪子從 10 m/s 打滑到純滾動，
  終速和解析值差 0.5% 內。

## 步末與顯示

- `Part.after(PartAfter)`：接觸解完後，每個零件看到步末的車身狀態、轉子轉速、自己每一列的扭矩（`rowTorque`）、滾動力的終值、
  `loads()` 裡 `carry` 留下的東西；在這裡更新內部狀態（輪胎變形、煞車與離合器溫度）。
- `Part.shownValues()`、`PartStep.show`／`PartAfter.show`：給客戶端畫的值（輪子轉角、轉向、懸吊行程），存在 `VehicleState`，
  遊戲端同步（`VehicleState.shownValues()`／`setShownValues`）。

## 埠

- `Part.Rotor` 有名稱（`input`、`output`、`left`、`right`），`Part.port(name)`；`Connection.Shaft` 多 `fromPort`、`toPort`（空白＝第一個轉子），
  JSON 只在有名稱時寫出，舊藍圖不變。`Connection.Shaft.ports(from, fromPort, to, toPort, ratio, efficiency)`。
- `Part.links()`：零件自己用約束列耦合的轉子對（變速箱的輸入與輸出）。軸把一個 link 的兩端併進同一個軸系是設計錯誤（零件自己的耦合會和軸打架）。

## 驗證（`DrivetrainTest`）

| 測試 | 內容 |
| --- | --- |
| `theSolverRepeatsTheTrainsAloneAndTheirFreewheels` | 引擎與 2:1 的轉子（固定軸與單向離合器），多一個零容量煞車讓求解器接手：四段扭矩（含反向、重新接合）的轉速與車身反扭矩和原本相同（1e-9） |
| `gearsCarryTorqueWithTheirEfficiency...` | 3:1、效率 0.9 的齒輪接黏性負載：穩態輸出功率 = 輸入 × 0.9（1e-4），齒輪列的扭矩 = 輸入扭矩，車身只感受外部負載的扭矩 |
| `differentialsSplitTorqueByTheirShares...` | 傘齒（0.5）與行星（0.4）差速器在左右負載不同時的扭矩分配；差速鎖時兩邊同速、扭矩依負載 |
| `limitedSlipDifferentialsBiasTorqueToTheGrippingSide` | 左右附著 10 與 1000 N·m：開放式抓地側 10、托森 TBR 3 → 30、離合器片（預壓 4、斜坡 0.1）→ 25（1e-6） |
| `aClutchSlipsAtItsCapacityAndLocksWhereMomentumSays` | 0.5 與 1.5 kg·m²、100 與 0 rad/s、容量 20 N·m：打滑期的轉速、1.875 s 鎖合於 25 rad/s、損失 1875 J（解析解） |
| `aBrakeHoldsBelowItsCapacityAndSlidesAbove` | 80 N·m 對 100 N·m 煞車完全停住（車身反扭矩 0），±150 N·m 時以 (T − 100)/I 加速 |
| `aViscousCouplingDecaysTheSlipAsBackwardEuler` | 滑差依後向 Euler 的 (1 + dt c (1/I₁ + 1/I₂))⁻ⁿ 衰減，兩者角動量守恆 |
| `aTorqueConverterStallsWhereItsCapacityMeetsTheEngine` | 渦輪被煞住時泵在 √(T/C) 失速、煞車承受 TR × T，車身上的引擎、導輪、煞車反作用相抵 |
| `withoutDrivingTorquesTheRowsOnlyTakeEnergy` | 40 組隨機的齒輪、離合器、煞車、差速、限滑、單向、黏性：沒有驅動時動能每步不增加 |
| `shiftingKeepsTheVehiclesAngularMomentum` | 2:1 與 −1:1 的離合器在自由飄浮的車身上接合：車身加轉子的角動量守恆（1e-9） |
| `redundantAndContradictoryLocksHoldWithoutFailing` | 重複的齒輪與兩個無限煞車：停住且扭矩平衡；兩個不同齒比同時嚙合：卡死 |
| `randomDrivetrainsNeverThrowOrGoNonFinite` | 60 組隨機傳動加隨機扭矩與斜率：不丟例外、不出現非有限值、Gauss–Seidel 退路少於 2% 的步 |
| `aRollingForceSpinsTheWheelUpAsTheBodySlows` | 見上一節（舊路徑與求解器路徑） |
| `aLineDamperActsAlongItsLineOnly` | 只沿線的隱式阻尼；偏離重心時帶出轉動；沒有阻尼器時和原本的更新完全相同 |
| `shaftsJoinTheRotorsTheirPortsName`、`portsGoToJsonOnlyWhenNamed` | 埠、未知埠與 link 被軸併在一起的錯誤訊息、JSON |
| `afterSeesTheRowsAndKeepsStatesAndShownValues` | `after` 讀到列扭矩與 `carry`、寫內部狀態與顯示值 |

## 變速箱（`TransmissionPart`）

| 種類 | 轉子 | 機構 |
| --- | --- | --- |
| 手排 | `input`（離合器飛輪側）、`primary`（離合器片與主軸）、`output` | 離合器容量 ×（1 − `clutch` 踏板）；`gear`（−1 倒、0 空、1..n）把排檔桿移過去：原檔立刻退出，新檔要等同步器（主軸與齒輪間的摩擦錐，倒檔四分之一）把轉速對到 3 rad/s 內才進；離合器沒踩時錐要連引擎一起拖，常常對不上，0.6 s 後顯示打齒並停在空檔 |
| 自排 | `input`（變扭器泵）、`turbine`、`output` | `TorqueConverter`、鎖止離合器（`lockup`）；每檔一個換檔元件（渦輪上 clutchNm 的離合器）；`gear`（−2 P、−1 R、0 N、1..n）啟動離合器對離合器換檔：換檔時間內退出元件容量降、進入元件容量升；升檔的退出元件與降檔的進入元件在換檔中只傳驅動扭矩（換檔控制的做法），慣性相與扭矩交接由求解得出；P 檔在輸出 3 rad/s 以下才咬住 |
| 雙離合 | `input`、`odd`（1、3、5、7）、`even`（2、4、6、R）、`output` | 各軸的檔在該軸離合器開著時由同步器掛上；`gear` 是要開的檔、`preselect` 是另一軸要預備的檔（預設下一檔）；想要的檔在另一軸備好時兩個離合器在換檔時間內交接（同上的單向規則，不斷動力）；`clutch` 調整驅動中的離合器（起步、蠕行） |
| CVT | `input`（泵）、`turbine`（主動輪）、`output` | 變扭器與鎖止同自排，變速器是一列齒比，扭矩受皮帶容量限制（超過就滑）；齒比以最多 `ratioRatePerS` 走向 `ratio`；`gear` −2 P、−1 R（乘倒檔組的比）、0 N、1 D |

齒輪以箱的效率傳遞功率；內部軸有很小的攪油阻力（讓踩下離合器的主軸慢慢停）。顯示值：檔位（CVT 為齒比）、狀態（0 穩定、1 同步或換檔、2 打齒）、使用中的齒比。

## 分動箱（`TransferPart`）

- 轉子 `input`、`range`（高低速檔）、`front`、`rear`；低速比（例如 2.72）只在前後輸出都低於 5 rad/s 時切換（真的分動箱要停車或爬行）。
- 全時：中央差速器（前軸分到 frontShare：行星式；0.5 為傘齒），開放、托森（偏差比；不等分配時偏差以 |輸入扭矩| 近似）或黏性，可鎖（`lock`）。
- 分時：`mode` 0 = 2H（只驅動後軸，前軸自由）、1 = 4H（前後直結，沒有差速：在抓地好的路面轉彎時兩軸互相拉扯）、2 = 4L（低速檔）。
- 適時：主軸（後，或 frontPrimary 時前）直接驅動，另一軸經多片耦合器，容量隨兩軸滑差上升（Haldex 第一代：約與轉速差成正比，couplingSlope），
  或由 `couple` 預壓（後代的電動泵），上限 couplingMaxNm。

## 計算（`api.drive`）

- `TorqueConverter`：MathWorks 預設表的形狀（速度比 0～0.97：扭矩比 2.232～0.9983、K 12.29～50 rad/s/√N·m）拉伸到設計的失速扭矩比與 K；
  0.97 以上扭矩線性降到 ν = 1 時為零；渦輪比泵快時是反向的流體耦合（導輪自由輪，沒有倍增）；尺寸 K ∝ D^−2.5（表取 0.26 m 的變扭器，規則）。
- `Driveline`：離合器容量 z μ F (d_i + d_o)/4（LuK：134/190 mm、3500 N、μ 0.27 得 153 N·m）、同步錐 j μ F d / (2 sin α)、皮帶 2 μ F R / cos β、
  Simpson、Ravigneaux、Lepelletier 的齒比（由齒數重現 TH400、ZF 6HP26、Ford AOD）、斜坡 LSD 鎖定係數 μ N_f (r_m/r_ramp) cot θ 與偏差比 (1 + L)/(1 − L)、
  黏性耦合 N π η (r_o⁴ − r_i⁴)/(2h)。

## 驗證：變速箱與計算

| 測試 | 內容 |
| --- | --- |
| `DriveTest.theConverterKeepsMathWorksCurveAndStretchesToADesign` | 表上每點精確重現；200 N·m 失速時泵 K√T、渦輪 2.232 倍；耦合點約 0.94、效率 0.94；設計拉伸與尺寸縮放；滑行時反向 |
| `planetaryRatiosReproducePublishedBoxes` | TH400 2.481/1.481/1/−2.077、6HP26 4.171…0.691/−3.403、AOD 2.40/1.47/1/0.67/−2.00 |
| `clutchesSynchronisersBeltsAndDifferentials` | LuK 例 153 N·m（1 N·m 內）、同步錐、皮帶、L 0.25/0.5/0.6 → TBR 1.67/3/4、黏性 |
| `TransmissionTest.aManualBoxSlipsItsClutchSynchronisesAndGrindsWithoutTheClutch` | 鬆離合器起步到鎖合、一檔齒比；踩離合換二檔的同步時間 ≈ I Δω / T；不踩離合換三檔打齒、留在空檔 |
| `anAutomaticStallsAtItsConverterAndShiftsClutchToClutch` | 失速轉速 K√200（1%）；P 檔渦輪自由、輸出不動；一檔齒比；一換二在換檔時間結束時到二檔齒比、車速不降；高速時 P 不咬住 |
| `aDualClutchBoxHandsOverWithoutABreakInDrive` | 預選的二檔在換檔時間（0.15 s）內交接、車速不降、交接後二檔齒比 |
| `aCvtMovesItsRatioAtItsRateAndItsBeltSlipsAboveCapacity` | 齒比每秒最多 0.5；到達目標；皮帶容量內不滑、超過就滑 |
| `transferCasesSplitLockAndCouple` | 2H 前軸不驅動、4H 直結、行進中不進 4L、停下後進 4L 齒比 2.72；全時開放式依 40/60、托森在偏差內鎖住；適時耦合器靠滑差帶前軸、預壓後直結 |

## 限制

- 軸沒有扭轉彈性與齒隙；同步器、離合器沒有溫度與磨耗；換檔元件的油壓動態以線性斜坡代表。
- 變扭器只有一組形狀（MathWorks 的表）拉伸；鎖止、換檔、預選、CVT 目標齒比的時機是 hdfv 的規則。
- 滾動力與車身分開解（見上）；輪子沒有簧下質量（0.24 的懸吊是無質量輪的射線模型）。
- 效率依功率方向，最多四次迭代；在功率正負交界處可能停在不一致的係數（量級是損失本身）。
