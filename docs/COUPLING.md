# 兩台載具的鉸接（0.36）

一台載具掛在另一台上：半拖車的主銷鎖在牽引車的第五輪，或牽引桿的環套在球頭上。AeroCore 只提供通用的接頭與在世界中的連結；
第五輪、支撐腳、掛上的條件與按鍵是 hdfv 的（hdfv 的 `drive/Hitch`、`truck/Hitching`）。

## 模型

`api.vehicle.Coupling` 描述一個接頭：

| 欄位 | 意義 |
| --- | --- |
| `leaderPoint`、`followerPoint` | 兩台各自模型座標中的一點，接頭讓它們重合（第五輪承盤中心、主銷軸上同高的點） |
| `pitchAxis`（前車座標）、`yawAxis`（後車座標） | 第五輪的鉸軸、主銷軸 |
| `pitchLimitRad`、`yawLimitRad` | 前後擺動與折刀角的單向限位，正負同角；PI 為不限 |
| `rollLimitRad`、`rollStiffnessNmPerRad`、`rollDampingNms` | 0：鉸軸與主銷保持垂直（第五輪承受拖車的側傾），經彈簧與阻尼，無限大為剛性；0～PI/2：間隙內自由、到限位停（擺動式第五輪）；PI/2 以上自由（球頭） |
| `captureM` | 掛上時兩點最多相距多遠 |
| `forwarded` | 後車每 tick 照抄前車的控制頻道（煞車、手煞車、燈、排檔的倒車燈） |
| `collide` | 兩台的碰撞球是否互撞（預設不撞：限位代替駕駛室與拖車的接觸） |

三個角由 `Coupling.geometry` 從兩個姿態算出：

- 俯仰：主銷軸繞鉸軸、從前車「向上」（前車 +y 投影到垂直鉸軸的平面）轉過的角。
- 偏擺（折角）：後車前方（+x 投影到垂直主銷軸的平面）繞主銷軸、從前車前方轉過的角。
- 側傾：asin(鉸軸 · 主銷軸)。三個角的變化率都是 (ω後 − ω前) · 軸（側傾的軸是 normalize(主銷軸 × 鉸軸)），所以限位與側傾都是同一種角度列。

## 求解

`ContactSolver.solveAll(bodies, contacts, joints, dt)` 在原本 24 次的 PGS 迴圈裡，每輪先解接觸、再解每個接頭：

1. 限位與側傾（角度列）：鎖住的側傾用 Baumgarte 0.2 的等式列；有彈簧時用 Catto 的軟約束（γ = 1/(h(c + h k))、β = h k/(c + h k)）；
   單向限位在到達前是投機的（允許的速率 (L − q)/h），超過後以 0.2 推回（最多 0.5 rad/s）。
2. 點：3×3 區塊精確解 K λ = −(v相對 + 偏差)，K = Σ(I/m − [r]× I⁻¹ [r]×)，所以輕的牽引車拖重的拖車也不會變軟（測試 1:5 與 1:10）。
   間隙超過 `JOINT_SLOP`（0.2 mm）時每步以 0.2 的比例收回（最多 0.5 m/s）。

衝量對兩台大小相等、方向相反，所以兩台的總動量只因外力改變。接頭的力以 `CONTACT` 力出現在兩台的受力清單裡，`JointResult` 另外給後車受的力與力矩。

`separateAll(bodies, contacts, joints)` 在接觸的位置修正之後，把接頭超過 0.2 mm 的間隙收回八成（每次最多 2 cm，3 次），鎖住的側傾或超過間隙的側傾也收回；只動位置，不加能量。

沒有接頭時兩個方法就是舊的那兩個，逐位元相同。

## 在世界中（neoforge）

- `PartVehicle.couple(follower, coupling)`（在前車上呼叫）：兩台都準備好（建表完成、沒有故障）、同一個維度、後車還沒掛在別台、不會成環、兩點在 `captureM` 內才掛上；
  回傳 `Couplings.Result`，成功時發 `Couplings.Coupled`。`uncouple()`、`uncoupleAll(reason)` 解開並發 `Couplings.Uncoupled`（原因 `COMMAND`、`FAULT`、`REMOVED`、`MOVED`、`BROKEN`）。
- 存檔：後車存 `Coupling`（前車 UUID、規格、前車上次的位置），前車存 `Towing`（後車 UUID 與位置）。沒有連結的載具不寫這兩個標籤。
- `BodyWorld`：連在一起的載具（任何鏈）是一組，全部在這裡、都開始了這個 tick，而且至少一台在實體會 tick 的區塊裡才一起走；否則整組等，告訴缺席者以外的成員（`groupWaits`）。
  連著的載具在只是載入的區塊也走，走的時候把自己的區塊再保留 2 秒（同高速載具的路徑票），所以不會在區塊邊界被拆開。
- 一台不在（卸載）時整組停在原地；它上次的位置已載入而它 `PATIENCE_TICKS`（200 tick）都沒出現，就當它不見了，連結結束（`REMOVED`）。兩點相距超過 `BREAK_M`（1 m）時結束（`BROKEN`）。
- 死亡、被移除、換維度、`place`、`refit`、物理故障都會解開；`replaceBlueprint`、`castOff` 保留。
- 客戶端：後車多一個同步值（前車的實體 id 與規格），畫的時候用前車畫出的掛點減去後車的旋轉（`drawnShift`），各自插值時主銷不會離開第五輪。
- 指令：`/aerocore vehicle couple` 把最近的載具用球頭掛在下一台最近的載具上（兩者碰撞球最近處），`/aerocore vehicle uncouple` 解開；`info` 顯示連結力與折角。

## 驗證

- `CouplingTest`：零重力下兩台亂轉 10 s，動量守恆 1e-9、動能不升超過 0.1 %、間隙 < 1 mm、側傾維持、俯仰停在限位；球頭單擺的週期與物理擺公式差 1 % 內；
  第五輪承受拖車的側傾力矩、俯仰停在 0.2 rad；1:5 與 1:10 的質量比間隙 < 1 mm；沒有接頭時與舊方法逐位元相同；JSON 往返。
- `ConvoyTest`（8 t 牽引車＋20 t 雙軸拖車，`TestTrack.convoy`）：0.6 倍滿舵的低速穩態折角 42.5° 對幾何 39.5°（差 8 %，輪胎側滑與雙軸刮地）、
  間隙 0.2 mm；倒車打舵折到 80° 的限位就停；只煞拖車兩台一起減速、主銷把牽引車往後拉；轉送煞車頻道時拖車也煞。
- GameTest `aCoupledVehicleIsDraggedAlong`：兩顆球用球頭連著，推一顆另一顆跟著走、間隙 < 1 cm、連結存讀、拒絕太遠／已掛／成環、故障與移除解開。
- hdfv 0.18：`HitchTest`（離線拖行 20 s 走 77 m，主銷在顎口內）與 `TruckGameTests`（世界中按鍵掛上、拖行、轉彎、放下支撐腳卸下）。

## 限制

- 輪胎力在接頭之前算（分步），靠 200 Hz 子步；非常硬的接觸鏈（三節以上）沒有測。
- 兩台互不碰撞（`collide` false）時，駕駛室與拖車的接觸只靠偏擺與俯仰限位代替。
- 全拖車（牽引桿＋轉向台車）只有 API 的 `BALL`，沒有範例。
