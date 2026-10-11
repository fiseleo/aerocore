# 活塞引擎（0.18）

使用者 2026-10-11 要求「先做零件」「引擎可以自己做」，並列出內燃機的元件：引擎本體與汽缸、汽缸蓋、活塞、連桿、曲軸、氣門、凸輪軸、
火星塞與噴油嘴，以及配氣、供油、點火、潤滑、冷卻系統。0.18 把這些做成各自獨立設計的元件，組合成一具四行程火花點火引擎零件
（`aerocore:piston_engine`），經動力軸驅動螺旋槳（之後是變速箱與輪子）。0.19 的噴射引擎見 [JET.md](JET.md)。

## 元件與參數

每個元件是一份玩家自己的藍圖（`EngineComponent`）：參數自由、自己檢查不合物理的值、標示外推、依幾何與材料估質量（也可以直接給質量）。
引擎（`PistonEngineDesign`）把它們組起來，檢查彼此是否裝得上。

| 元件 | 類別 | 參數 |
| --- | --- | --- |
| 引擎本體與汽缸 | `EngineBlockDesign` | 排列（直列／V／水平對臥）、缸數、夾角、缸徑、缸距、缸體高度（曲軸中心到缸面）、主軸承數、材料、壁厚、缸壁粗糙度 |
| 汽缸蓋 | `CylinderHeadDesign` | 凸輪配置（OHV／SOHC／DOHC）、每缸進排氣門數、燃燒室形狀與容積、氣門夾角、氣道喉部比、氣道流量係數、每缸火星塞數與位置、汽缸床墊厚度、材料 |
| 活塞 | `PistonDesign` | 直徑、壓縮高度、頂部容積（凸頂為正、凹頂為負）、頂部厚度、裙長、活塞銷直徑、環數、環張力、材料 |
| 連桿 | `ConnectingRodDesign` | 中心距、大端孔徑與寬度、小端孔徑、桿身截面積、材料 |
| 曲軸 | `CrankshaftDesign` | 行程、主軸頸與曲柄銷直徑寬度、配重比例、飛輪慣量、材料 |
| 氣門 | `ValveDesign` | 進排氣門頭徑、桿徑、長度、材料 |
| 凸輪軸 | `CamshaftDesign` | 進排氣持續角（座到座）、升程、中心角、升程曲線飽滿度、每根軸承數、材料 |
| 火星塞 | `SparkPlugDesign` | 螺紋直徑（要裝得進汽缸蓋） |
| 噴油嘴 | `FuelInjectorDesign` | 靜態流量（cc/min）與額定壓力、進氣道／缸內直噴 |
| 配氣系統 | `ValvetrainDesign` | 直推、搖臂（finger）或搖桿、平面或滾子挺桿、氣門彈簧預載與剛度、每閥運動質量、正時傳動、進排氣可變正時範圍 |
| 供油系統 | `FuelSystemDesign` | 燃料（汽油〔RON／MON〕、甲醇、丙烷、異辛烷）、化油器／進氣道噴射／缸內直噴、每缸噴油嘴數、油壓、全負荷與部分負荷的 λ |
| 點火系統 | `IgnitionDesign` | 爆震控制、最大點火提前、斷油轉速、怠速、起動馬達扭力 |
| 潤滑系統 | `LubricationDesign` | 機油等級、油量、濕式／乾式油底殼 |
| 冷卻系統 | `CoolingDesign` | 水冷（水箱正面積、厚度、風扇流量、冷卻液量、節溫器）或氣冷（散熱片面積） |
| 進氣 | `IntakeDesign` | 歧管長度與直徑、穩壓箱容積、節氣門直徑與數量、空氣濾清器損失 |
| 排氣 | `ExhaustDesign` | 背壓係數（相對 Patton 等的量產排氣） |

進氣與排氣不在使用者的清單裡，是循環模擬需要的，另外加上。

**不是參數、由零件決定的**（像《Automation》）：壓縮比（燃燒室＋床墊＋缸面間隙－活塞頂）、火焰行程（火星塞數量與位置）、
進排氣（氣門、氣道、凸輪、歧管、節氣門）、爆震（辛烷值、壓縮、燃燒室、點火控制）、摩擦（軸承、活塞環、配氣、機油）、
轉動慣量（曲軸、飛輪、連桿、活塞）、質量，以及各種極限。

### 組裝檢查（INVALID：裝不上）

- 活塞比缸徑大，或間隙超過 0.5 mm。
- 連桿大端不合曲柄銷（軸承間隙 0～0.2 mm），小端不合活塞銷；V 型兩支連桿共用一個曲柄銷，寬度要放得下。
- 活塞上死點超出床墊、沒有餘隙容積（活塞撞汽缸蓋）。
- 氣門裝不進缸徑：投影在缸面上，進氣排一側、排氣排另一側，座與座之間 3 mm、座到缸壁 1.5 mm（常見座圈壁厚，規則）；
  傾斜的氣門投影成橢圓（`valveAngleDeg` 夾角的一半），這就是屋脊形燃燒室能放大氣門的原因。三維燃燒室與火星塞座不檢查。
- 直噴配進氣道噴油嘴、或反過來；節氣門比缸數多；火星塞太大。

外推（照算、標示）：壓縮比 6～14 以外、平均活塞速度在斷油轉速超過 25 m/s（F1 約此）、連桿比 1.4～2.2 以外、升程超過氣門頭徑 1/3（氣道限制，再高不會多進氣）等。

## 模型

### 循環模擬（`EngineCycle`）

單區、逐曲軸角（0.25°，Heun 二階）的四行程火花點火循環，一次算一缸（Heywood《Internal Combustion Engine Fundamentals》1988 第 14 章）：

- 汽缸質量與能量守恆：dU = h_in dm_in − h_out dm_out − p dV + dQ_燃燒 − dQ_壁。理想氣體，新鮮充量（空氣＋燃料，燃料以空氣的比熱計）與燃燒產物混合。
- 氣體性質：GRI-Mech 3.0 的 NASA 七係數多項式（O₂、N₂、Ar、H₂O、CO₂、CO、H₂）；乾空氣 20.95 % O₂、78.08 % N₂、0.93 % Ar。不含解離。
- 燃燒產物：碳、氫、氧守恆；濃混合氣以水煤氣平衡 K = 3.5（Heywood 4.9.2）分配 CO／H₂，未燃的 CO、H₂ 熱值不釋放。
  另乘 0.97 的燃燒效率（縫隙與淬熄，Heywood：火花點火引擎 95～98 %，典型值）。
- 氣門：可壓縮孔口流（Heywood 附錄 C），有效面積 = 流量係數 ×（簾面積 πDL 與喉部面積的較小者）。
  兩側壓力接近時孔口流的導數趨於無限，顯式步長會越過平衡、在稀薄氣體中發散；每步流量不超過讓兩側壓力相等之質量的一半（通量限制，只在會越過時作用）。
- 進氣歧管：每缸一個與歧管同容積的氣門口節點，經歧管氣柱（集中慣性 L/A、二次損失：Darcy 0.02、入口 0.1）接到定壓穩壓箱——
  Heywood 7.6 的 Helmholtz 共振腔模型，這是長歧管在下死點後慣性進氣（ram）的來源，VE 對轉速有共振峰。
  低歧管壓力時，重疊期的強烈倒灌可能讓集中氣柱一步抽空氣門口（真實歧管會以壓力波處理），每步最多抽走氣門口一半的氣體。
- 排氣：定背壓的排氣箱，背壓 = 大氣 + 係數 × 0.178 (p_i/p_a·S_p)² kPa（Patton 等的量產排氣，Sandoval 2002 式 13）。無排氣歧管調諧。
- 燃燒：Wiebe 函數（a = ln 10，90 % 在 Δθ₉₀），Δθ₉₀ 與形狀指數用 Bonatesta 等 2010 的相關式（Bonatesta 2013，InTech，doi 10.5772/55495，式 22、24），
  輸入點火時的充量密度、平均活塞速度、殘餘廢氣比例與點火提前。該相關式只在一具 1.6 L 四氣門中央火星塞引擎上擬合
  （1500～3500 rpm、2～7 bar IMEP、化學計量汽油、稀釋 6～26 %、點火 8～35°，誤差 11 % 內）。本模型另加兩個**假設**：
  燃燒角依「火焰行程／行程」相對該引擎縮放（紊流與火焰速度隨平均活塞速度，Bonatesta：u′ ≈ 0.38 S_p），
  以及依層流燃燒速度 (S_L,ref / S_L)^0.85 縮放其他燃料與 λ（指數取自相關式本身的稀釋項）。
- 層流燃燒速度：Metghalchi & Keck 1982（甲醇、丙烷、異辛烷）、Rhodes & Keck 1985（汽油），係數見 ANSYS Forte 理論手冊表 7.1。
- 熱傳：Woschni 1967（Heywood 式 12.19～12.20），單一壁溫（水冷：節溫器 + 60 K；氣冷 470 K，Heywood 400～500 K 壁溫的估計）。
- 爆震：Livengood–Wu 積分，Douaud & Eyzat 1978 的點火延遲 τ = 17.68 (ON/100)^3.402 p^−1.7 exp(3800/T) ms（p atm、T K、ON 取 RON），
  末端氣體從點火時的狀態等熵壓縮。積分在燒完 90 % 前到 1 即爆震。
  **校準**：τ 乘 1.45（`EngineCycle.KNOCK_CALIBRATION`），使 79 mm、壓縮比 11、95 RON 的四氣門引擎在 1500～1900 rpm、6～7 bar IMEP
  於最佳扭力點火提前再多約 3.5° 才開始爆震（Bonatesta 2013 4.3 節：「over-advance limited to 3–4 CA degrees」）。
  文獻常以調整前指數因子校準此相關式；這只用了一具引擎的資料。
- 噴油：進氣道噴射與化油器在進氣門前蒸發燃料（充量降溫 h_v/(AFR+1)/c_p）；直噴在進氣門關閉時加入燃料並由充量提供汽化熱。

每點從膨脹末開始跑整個循環，3 個循環後收斂（實測 VE 在第 3 個循環已穩定到 0.1 %）。

### 標定與 MAP（`Dyno`、`PistonEngineDesign.buildMap`）

- 每個轉速與歧管壓力：黃金分割找最佳扭力點火提前（MBT，每次 2 個循環），有爆震控制時退後到爆震積分低於 1；沒有時維持 MBT（不超過最大提前）並標示爆震。
- λ：歧管壓力比 0.7 以下用部分負荷值，到 1.0 線性過渡到全負荷值。噴油嘴不夠時以同樣的空氣與噴油嘴能給的燃料重算（變稀）。
- 可變正時：每個轉速在全負荷試 5 個進氣相位（再試 5 個排氣相位），取最大扭力，部分負荷沿用（ECU 在部分負荷的 EGR 策略不模擬）。
- MAP：40 個轉速（500 rpm 到斷油轉速 × 1.08）× 8 個歧管壓力比（0.12～1.0），在標準大氣（101.325 kPa、298 K，進氣加熱 10 K）。
  離開遊戲執行緒建立（`Part.warmUp`，各轉速平行），同一份設計只建一次。範例 2.0 L 在原開發機約 5 s。

### 摩擦（`Friction`）

Patton、Nitschke、Heywood（SAE 890836，1989）的元件模型，用 Sandoval & Heywood（SAE 2003-01-0725；Sandoval 2002 MIT 學士論文，式 7b～11、附錄 A-2）的修正：
曲軸（主軸封、主軸承流體動壓、軸承內油的紊流耗散）、往復（活塞裙、活塞環〔環張力 F_t/F_to、缸壁粗糙度 C_r〕、連桿軸承、氣壓對環的加載）、
配氣（凸輪軸承、平面或滾子挺桿、擺動件的流體動壓與混合潤滑，常數依配置取表 4.2）、附件（機油泵、水泵、空載發電機）。
流體動壓項乘 √(ν/ν_ref)，ν 由 Vogel 式與高剪切比（附錄 A.2 的機油表），參考油是 Patton 校準時的 10W-30 於 90 °C。
表 4.2 的 OHV 滾子常數印為 0.5，會讓滾子比平面挺桿摩擦大好幾倍，與其他各列相反；改用搖桿配置的 0.0151（推桿引擎經搖桿驅動氣門），屬估計。
附件項的轉速二次式在約 14600 rpm 達最大後持平（原式會下降）。泵氣損失不用 Patton 的式子，由循環模擬直接算出。

### 執行時（`PistonEngineDesign` 的零件）

- 曲軸繞零件 +X 轉，轉子慣量 = 曲軸（含配重、飛輪）＋ 連桿大端 ＋ 往復質量在一轉的平均（½ m r²）。扭力是內力，引擎座承受反扭力。
- 輸入：`throttle` 0～1、`ignition`（沒綁定 = 開）。內部狀態（0.18 新增 `Part.internalStates`，存在 `VehicleState`、隨存檔）：
  歧管壓力比、引擎溫度、運轉中。
- 歧管壓力：穩壓箱（含歧管）容積的質量守恆，每小步隱式平衡節氣門流入（孔口，流量係數 0.8，Heywood 式 7.1 首項的開度面積）與引擎吸入（MAP 內插）。
- 起動馬達把引擎帶到 400 rpm 點火，300 rpm 以下熄火；怠速控制在怠速以下打開旁通空氣（比例控制，規則）；超過斷油轉速、或節氣門全關且轉速高於怠速 600 rpm 時斷油；
  斷油或熄火時只有泵氣與摩擦。
- 摩擦用當時引擎溫度下的機油黏度：冷車約為熱車的兩倍（Sandoval：20 °C 對 90 °C 約 2.1 倍）。
- 溫度：一個集中溫度（金屬、冷卻液、機油），加熱 = 循環的壁熱 + 全部摩擦功率，散熱 = 水箱（ε-NTU，空氣側由沿鰭片的層流 Nu = 0.664 Re^0.5 Pr^(1/3)，
  表面積密度 1000 m²/m³〔Kays & London 小型熱交換器 700～1500〕、車速 30 % 通過水箱〔規則〕或風扇流量），節氣門以下不散熱，10 K 內全開（規則）。
- 外界空氣密度不同時，流量與 IMEP 依密度比縮放（MAP 在標準大氣算）。
- 診斷值：rpm、throttle、manifoldKpa、torqueNm、powerW、fuelKgPerS、engineC、running、knocking。

## 驗證

| 測試 | 內容 |
| --- | --- |
| `EngineModelTest.speciesMatchJanafAtRoomTemperature` | 298 K 的 c_p 與生成焓對 JANAF |
| `airHasItsTextbookGasConstantAndGamma` | R = 287.05、γ(300 K) = 1.400、能量表可逆 |
| `stoichiometricAirFuelRatiosAreHeywoods` | 汽油 14.6、甲醇 6.47、丙烷 15.7、異辛烷 15.1（Heywood 附錄 D） |
| `productsConserveTheElementsAndHoldTheWaterGasShift` | 各 λ 元素守恆、濃混合氣 K = 3.5 |
| `richMixturesReleaseLessOfTheFuelsEnergy` | λ 0.8 的釋熱比例對手算（氧平衡＋水煤氣平衡） |
| `crankKinematicsMatchTheirDefinitions` | 容積、dV/dθ、活塞加速度對有限差分與 a(1 + a/l) |
| `valveLiftIsSmoothAndItsFloatSpeedIsTheWeakestPointOfTheLobe` | 加速度對有限差分；浮閥轉速對逐角掃描 |
| `orificeFlowChokesAndReducesToBernoulliForSmallDrops` | 臨界流與小壓差的 Bernoulli |
| `adiabaticMotoringIsReversible` | 絕熱倒拖可逆、上死點溫度對獨立積分的等熵線 |
| `nearlyInstantAdiabaticCombustionGivesTheFuelAirOttoEfficiency` | 絕熱瞬間燃燒的功對獨立算的燃料－空氣 Otto 循環（0.6 % 內） |
| `frictionOfAV6IsInSandovalsRangeAndRoughlyDoublesWhenCold` | 3.0 L V6 於 2000 rpm 的 fmep 與冷熱比（Sandoval 圖 4-15：約 90 kPa、約 2.1 倍） |
| `aWarmFourValveEngineHasTextbookEfficiencyAndBreathing` | 指示效率 0.33～0.44、VE 0.82～1.05、殘餘 2～10 %、最高壓力 50～100 bar |
| `EngineDesignTest.theSampleEngineIsValidAndItsBlueprintRoundTrips` | 範例引擎有效、JSON 往返 |
| `compressionRatioComesFromChamberGasketDeckAndCrown` | 壓縮比隨凸頂、缸面切削改變（手算）；切過床墊即撞汽缸蓋 |
| `partsThatDoNotFitEachOtherAreRejected` | 活塞大於缸徑、連桿不合曲柄銷、直噴配進氣道噴油嘴 |
| `valvesMustFitTheBoreAndInclinationMakesRoom` | 35.5／30.5 mm 氣門在 86 mm 缸徑：平行放不下、40° 夾角放得下（手算） |
| `theSampleEngineHasAProductionLikeFullLoadCurve` | 範例 2.0 L：最大 BMEP 10～13.5 bar、45～80 kW/L、全負荷最低 BSFC 240～320 g/kWh |
| `itStartsIdlesRevsToItsLimiterSlowsAndStops` | 起動、怠速 ±300 rpm、全油門到斷油轉速、升溫、收油回落、關火停止、內部狀態存檔 |
| `aColdEngineFightsMoreFriction` | 15 °C 的摩擦超過 90 °C 的 1.5 倍 |
| GameTest `engineBoatStartsItsEngineAndDrivesAhead` | 範例引擎船：MAP 建好後自己起動、部分油門推船前進、3:1 齒輪 |

範例引擎（86 × 86 mm、壓縮比 11、四氣門雙凸輪、進氣可變正時、350 mm 歧管、95 RON）：最大扭力約 206 Nm／4400 rpm、最大功率約 125 kW／7600 rpm，
同級量產引擎約 190～200 Nm、140～150 kW；高轉的功率偏低。

## 限制

- 單區模型：沒有火焰前緣的兩區溫度，NO_x 與爆震強度都不算；爆震只判斷有沒有。解離不計，最高溫度與壓力略高。
- 燃燒相關式來自一具引擎；火焰行程與燃料的縮放是假設。極稀或極濃混合氣的失火不模擬（只是燃燒極慢）。
- 進氣只有集中的 Helmholtz 共振，沒有壓力波；VE 共振峰的高低可能比實際明顯。排氣沒有歧管調諧，汽缸之間不互相影響。
- 增壓（渦輪、機械增壓）、柴油、轉子引擎、二行程尚未支援。
- 火星塞間隙、熱值、電極材料、點火能量、潤滑系統的供油量與油溫分層、結構疲勞（只算連桿與活塞頂的估計強度）都還沒有模型。
- 質量是幾何估計；元件材料性質是常溫手冊值，未計溫度降低的強度。
