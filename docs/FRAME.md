# 車架：扭轉進物理與結構計算（0.33）

使用者 2026-10-17「底盤也要高度自由化」，選擇車架剛性「都要」：決定質量與強度檢查，也進入物理、影響操控；承載式車身也要會扭。

## 扭轉進物理（`FrameFlex`）

- 載具仍是一個剛體（質量、慣量、重心不變）。會扭的零件（`Part.frame()` 回傳 `FrameTwist`：底盤 `FramePart`、承載式車身 `BodyPart`）給沿 x 的站點與
  站點扭轉剛度 K（對稱、每列和為零：整個一起扭是車身的滾轉，不是車架的）。站點的截面扭角 θ（繞 +x，相對剛體）沒有慣量，每一步重新求平衡。
- 每一步：
  1. 會受車架影響的零件（車軸，`probes()`）先探測（`probe`）：找地面（射線結果留給正式那一輪用，只投一次），
     回報各輪在步初扭角下對車身的力 F⁰（沿行程）、這一步內的剛度 k（輪胎、彈簧、避震器串聯：`Suspension.Step` 的斜率 ÷ dt）、位置 x、z。
  2. 扭角讓 z 處的點上升 −z θ(x)（θ(x) 是站點的帽函數內插），各輪的力變成 F⁰ + k z nᵀ(θ − θⁿ)。各站平衡，車身取它們的平均
     （權重 w：各零件質量依重心 x 分到兩側的站，wᵀθ = 0，乘子是車身承受的滾轉力矩）：

     ```
     [ K + Σ k z² n nᵀ   w ] [θ]   [ Σ k z² n nᵀ θⁿ − Σ z F⁰ n ]
     [ wᵀ                0 ] [λ] = [ 0                         ]
     ```
  3. 正式的負載裡，各輪的掛點依扭角移動：吃進行程的量減掉步初的上升（`flexLift`），接近速度減掉這一步的上升速度（`flexRate`）；
     `after()` 推進壓縮時也減掉。所以步末的力就是求解時假設的。
- 兩站時就是串聯彈簧：地面的扭曲由前後懸吊的側傾剛度與車架分攤。隱式、沒有慣量，K 從 1e5 到 1e7 N·m/rad 都穩定。
  扭角用步初的車身速度求（先扭再動車身），暫態有一階的分步誤差。
- 只有扭轉進物理；車架的彎曲只用在強度與剛度檢查。起落架（`LegPart`）不探測：飛機沒有會扭的機身。

### 驗證（`FrameFlexTest`）

| 測試 | 內容 |
| --- | --- |
| `aRigidFrameIsTheRigidBody` | 不扭的 `FramePart` 和 `MassPart` 走 2 秒逐位元相同（1e-9）；1e9 N·m/rad 的車架和剛體的輪荷差 0.1% 內 |
| `aWarpIsSharedLikeSeriesSprings` | 左前輪墊高 2 cm：兩端的側傾 φ 由前後懸吊（角剛度 = 角輪率 × t²/2，輪胎串聯）、車架 K 與重力翻轉力矩（m g h_cg 依 w 分給兩端）的兩站靜力平衡算出，前後軸的扭曲載重轉移對到 3%（剛體、3e5、3e4 N·m/rad）；輪荷和等於重量 0.5% |
| `withoutFrameStiffnessEachEndCarriesItsOwnMass` | 4° 側坡、前軸粗防傾桿：剛體時橫向載重轉移依側傾剛度分配（3%），車架不抗扭時依兩端質量（w，2%） |
| `stiffFramesStayStableOnRoughGround` | K = 1e5、1e6、1e7，崎嶇路 15 m/s 開 20 秒：速度、姿態、扭角都有界 |
| `threeAxlesCarryTheWeightOnAWarpedGround` | 三軸三站的鏈：墊高一輪時六輪都著地，輪荷和等於重量 0.5% |
| `theStepConverges` | 落到墊塊上 0.3 秒後的輪荷：dt 與 dt/4 差 1%＋20 N 內 |

剛體的情況對照時發現：只用串聯彈簧（不含重力翻轉力矩）估計，對角載重少 18%；實測的車身側傾 0.4214° 和含重力的模型 0.421° 一致。

## 結構計算（`api.frame`，設計時用）

- `Section`：截面的面積、慣性矩、扭轉常數 J、翹曲常數 Cw、最外纖維、閉口或開口、外形輪廓（畫圖用）。
  `rectTube(b, h, t)`（Bredt：J = 2t(b−t)²(h−t)²/(b+h−2t)）、`box(b, h, tw, tf)`、`roundTube(d, t)`（J = 2I）、`channel(h, b, t)`、`iBeam(h, b, tw, tf)`；
  槽鋼與 I 型的 J、Cw 用中線尺寸，參數仍是外形尺寸。`restrainedTorsionNm2(材料, 長度)`、`torsionShearPa`、`torsionStress(材料, 長度, 接頭係數, 扭矩)`、
  `bendingPa`、`kgPerM`、`outline()`；`sectorialM2` 是對剪力中心的最大扇性座標 ω_max（I 型 h_o·b/4；槽鋼 h_m(b_m − e)/2，e = 3b_m²/(6b_m + h_m)；閉口為 0）。
- `FrameMaterial(密度, E, G, 降伏)`；`Frame`：`node(Vec3d)`、`member(a, b, 截面, 材料, 上方向, 接頭係數)`；接頭係數 `WELDED` 1、`RIVETED` 0.7、
  `BOLTED` 0.5、`UNRESTRAINED` 0，只用在開口截面：兩端受限翹曲的等效扭轉剛度 GJ_eff = GJ/(1 − tanh(κL/2)/(κL/2))，κ = √(GJ/(E Cw))，
  依接頭係數在 GJ 與 GJ_eff 之間混合（不加這一項，槽鋼梯形大樑軟得不真實：C150×60×5 長 1.2 m 時 GJ_eff/GJ = 4.33）；閉口截面一律 GJ。
- 元素：12 自由度 Euler–Bernoulli 空間梁加 St Venant 扭轉，節點剛接，負載在節點；不含剪切變形、開口截面的剪力中心偏移、大位移與挫屈。
- `FrameSolver.factor(frame, 彈簧, 固定)`、`solve(負載)`：節點依 x 排序、帶狀 Cholesky（n·b²）；不連通時 `FrameDisconnectedException` 說出哪個節點。
  支撐沒擋住的剛體運動固定在最靠近形心的節點（只固定剛好擋住的自由度），負載在這些運動上不平衡時丟例外。
- `FrameAnalysis`：
  - `twistStiffness(frame, 站點)`：站點 `Station.pair(x, 左, 右)`（兩條大樑：兩節點垂直位移差 ÷ 間距）或 `Station.node(x, 節點)`（脊骨：繞 x 的轉角）；
    在各站加力偶、在第一站平衡 → 柔度矩陣 → 反矩陣 → 加邊，得到給 `FrameTwist` 的矩陣（對稱、半正定、只有一個零特徵值、K·1 = 0）。
  - `bendingStiffness(frame, 後, 前, 加載)`、`loadCase(frame, 節點負載, 支撐)`：反力、各構件兩端的力、von Mises 使用率
    （軸向＋兩向彎曲在最遠角，扭轉剪應力閉口 T/(2A_m t)）。開口截面兩端受限翹曲時，扭矩分成 St Venant 與翹曲兩部分
    （Vlasov 的 T = GJφ′ − E Cw φ‴，兩端 φ′ = 0；Kollbrunner–Basler、Trahair）：剪應力只算 St Venant 的部分，
    最大在構件中點 T(1 − 1/cosh u)·t/J；翹曲正應力 B·ω_max/Cw，雙力矩最大在兩端 B = (T/κ)·tanh u，u = κL/2；
    依接頭係數 f 在自由（全部 T t/J、沒有翹曲應力）與受限之間線性混合。翹曲正應力加在彎曲正應力上再進 von Mises
    （各項都取構件上的最大值，峰值不在同一處，所以偏保守）；沒有算翹曲剪應力。
    短槽鋼的翹曲應力可以比原本「全部當 St Venant」還大：C150×60×5 長 1.2 m 焊接、500 N·m 時，剪應力 231 → 79 MPa，但翹曲正應力 396 MPa。
  - `standardCases(frame, 承載質量, 車輪, 地面高度, g)`：2.5 g 過坎、壕溝、0.8 g 煞車、0.8 g 過彎。
    0.36：`standardCases(frame, 承載質量, 車輪, pins, 地面高度, g)`，pins 是另一台車撐住的節點（半拖車的主銷在第五輪上）：像輪子一樣是垂直彈簧（FIXED 為剛性），
    承重，煞車與過彎時按承重比例承受水平力、作用在節點本身（力臂 0），不會是掉進溝的那個；輪子與 pins 的節點合計要 3 個以上，壕溝仍要 4 個輪子節點。
    沒有 pins 時與舊方法逐位元相同。測試：`FrameAnalysisTest.aTrailerStandsOnItsAxlesAndItsKingpin`（12 m、20 t 的拖車：主銷約四成、煞車時水平力平衡、壕溝落在拖車自己的左前輪）。
    壕溝是左前輪下的地面下降 `DITCH_DROP_M` = 0.15 m（規則：扭曲測試常用 150～200 mm 的坑或凸台），等於在該節點加 −k·d 的負載（剛性輪則強制位移）。
    彈簧只能推：被拉住的輪子放開，放開後會陷進地面的放回，一次改一個、最嚴重的先改，到都沒有為止。
    柔的車架靠扭來讓輪子貼地、扭矩較小；兩站時扭矩 = (d/t)/(1/C_前 + 1/C_後 + 1/K)，C = k t²/2。輪子節點少於 4 個時沒有壕溝；
    輪胎力在地面，連同到安裝點的力矩加到車架；支撐是各輪的彈簧，3 軸以上也解得出來。
  - `massKg`、`nodeMassesKg`。
- 規模：梯形大樑 20～40 個節點，管架 40～200 個；200 節點的管架整套計算暖機後約 21～27 ms（第一次含 JIT 70～109 ms）。在背景執行緒做，結果依設計快取。

| 測試（`SectionTest`、`FrameAnalysisTest`） | 內容 |
| --- | --- |
| `aCantileverBendsAsPLCubedOverThreeEI`、`aRoundTubeTwistsAsTLOverGJ`、`aSimplyHeldBeamIsAsStiffAs48EIOverLCubed` | 1e-9（這些元素是精確解） |
| `aPortalFrameSwaysAsSlopeDeflectionSays` | 門型框架對閉式解 1e-6 |
| `aRectangularTubeTwistsAsBredtSays`、`heldWarpingStiffensAnOpenSectionAsKollbrunnerAndBaslerSay` | Bredt；受限翹曲對微分方程的有限差分解 1e-6 |
| `aHeldOpenSectionCarriesItsTorqueByWarpingAsVlasovSays` | 槽鋼、I 型兩端受限：中點 St Venant 扭矩與端部雙力矩對有限差分解（1e-6、1e-5）；接頭係數混合；ω_max 由 Cw = ∫ω² t ds 驗證；構件的 von Mises |
| `aDitchTwistsTheFrameAsSpringsInSeries`、`aTruckLadderTakesTheDitchOnItsSprings` | 壕溝扭矩對串聯彈簧 1e-9；剛的車架讓輪子懸空時等於三輪的情況；剛性輪的強制位移；三軸卡車大樑試算 |
| `theTwistMatrixIsSymmetricSemidefiniteAndBlindToAWholeTwist`、`aBackboneTwistsAsItsTubesInSeries` | 矩陣性質；脊骨串聯 GJ/L 1e-9 |
| `reactionsBalanceTheLoads`、`aMembersStressCountsItsLoads`、`aFrameInTwoPiecesNamesTheNodeLeftOut` | 反力平衡、應力、斷開的車架 |
| `aTwoHundredNodeTubeFrameIsAnalysedInUnderFiftyMilliseconds`、`outlinesHoldTheSectionsAreasAndMoments` | 效能；輪廓的面積與慣性矩 |

注意：4.5 m 的輕槽鋼（C150）梯形大樑只有每度約 300 N·m（手算上限也是這樣）；卡車級（每度 2～10 kN·m）要重的大樑或 X 型橫樑。

## 承載式車身的扭轉（`BodyStructure.torsion`）

見 [BODY.md](BODY.md) 的結構一節。範例轎車（4.85 m、四門、0.8 mm 鋼殼）2.7 m 軸距每度 33.9 kN·m，在真實轎車的 10～40 內；
柔度約一半在擋風玻璃段、四成在車廂。敞篷（腰線以上開口）只有每度 2.5 kN·m。`BodyTorsionTest`：閉口圓管與方管對 Bredt 1e-6、
沒有開口時等於閉口殼、開口越多越軟（窗 > 兩門 > 四門）、矩陣性質、轎車的範圍。
