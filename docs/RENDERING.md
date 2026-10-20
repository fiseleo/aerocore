# 零件的繪製（0.29；0.30 加開口、內裝、可動件、燈）

使用者 2026-10-15 要求 3D 模型更逼真（hdfv 0.11／0.12）：幾何細節、材質貼圖、光影與世界中的效果。AeroCore 提供通用的工具，內容（材質貼圖、細節零件、引擎與機器的模型）在載具 Mod。

## 網格核心（`api.mesh`，純 Java）

- `MeshBuilder<M>`：所有形狀（方塊、圓角方塊、六面體、面片、球、圓柱／圓錐、旋轉體 `lathe`、沿路徑的管子、擠出、放樣船體、合併、平面切開）。
  `PartMesh`（客戶端）與 `Mesh`（純 Java）都繼承它，0.28 的呼叫照樣編譯、幾何不變（`MeshCompatTest` 對凍結的 0.28 副本，1e-12）。
- 每個形狀自動給 UV（公尺）：平面用最朝向的平面投影，車削與管子用弧長×軸向（圓周取材質重複長度的整數倍，沒有接縫），面片沿第一排×橫向。
- 「筆」：`finish(Finish)`、`detail(0-3)`、`interior`、`hot`、`decal`，之後建立的四邊形都蓋上。`Finish(id, tileM)` 只是鍵和重複長度，長什麼樣子由繪製端決定。
- `MeshOps`：依折角平滑法線、鏡射（保留朝外的繞向）、非等比放置細節零件（法線用反比例）、細分、細節層級過濾。
- `ObjReader`／`ObjWriter`：Blender 匯出的細節零件；遊戲模型匯出給 Blender 算圖對照。
- `PackedMesh`：烘焙後的陣列（約 37 位元組／頂點），依通道（實心、貼花、半透明）、材質、熾熱分組。
- 替身 `MeshBuilder.STAND_IN`：整個零件的粗略外形（一具引擎約一百面），只在有兩層以上的烘焙的最遠一層出現，而且那一層只有它；其他層和設計畫面都不畫。
  遮蔽與「看不見的面」只用替身以外的面算（替身包住全部，算進去會讓裡面所有面都被判成封閉）。

## 環境光遮蔽與看不見的面（`AmbientOcclusion`）

- 網格表面的體素格（約 1/128 尺寸），每個角對餘弦加權的半球打固定的 Hammersley 射線（16 條），射線長約 8 % 尺寸（3～30 cm），近的命中權重大（1 − (s/R)²）。
- 射線從表面外 2 個體素開始：斜的表面佔的體素離自己最遠到對角線 1.73 個體素（1.5 個時斜面會自己遮自己，0.29 開發時錐形噴嘴一塊塊變黑）。
- 角的快取鍵用量化後的完整座標（0.29 開發時把座標雜湊成一個 long 會撞，別處的暗值被抄到外殼上）。
- 自適應細分（`refined`）：一個四邊形的四角亮度差大於 0.12、邊長超過 6 個體素，就切成約 3 個體素的格重算，最多兩次；
  否則一個長面一角的暗會沿整面內插（0.29 開發時機匣出現長條紋）。第 0 層用細分後的，較遠的層用原本的面、遮蔽減半。
  限制：只看四角的差。四角都在凹角、中間開闊的長面（hdfv 0.20 的塔身、管制室天花板）四角一樣暗，不會被切，整面變暗；
  呼叫端先自己切（hdfv `TowerRenderer.split`）。0.39 的 `Refine.CENTRE` 也比較面中心（美化著色開著時用，見下）。
- 看不見的面：從外框的空氣沿空體素洪水填充，碰不到外面空氣的面是「封閉的」；另一個面以同向或反向、在很小的間隙內
  （尺寸 × 0.003，1～6 mm）蓋住全部取樣點的面是「被覆蓋的」。被覆蓋的判定用精確幾何（空間雜湊），不用體素：
  大零件的體素有好幾公分，會把外面幾公分的環管當成覆蓋而挖洞（0.29 開發時 J79 的燃燒室外殼）。
- 這是顯示用的近似，不是光傳輸。約 9 萬個面 1.2 秒（`AmbientOcclusionTest`），在背景執行緒。

## 客戶端

- `MeshBakery`：背景烘焙與快取（鍵弱參照；資源重載時清除）。`Spec`：預設材質、遮蔽、細節層級數、剔除。
  開發記錄 `Part mesh baked: N quads, level 0 keeps M, ...`；`/aerocoredev parts occlusion|culling on|off` 比較。
- `PartFinishes`：各命名空間的 `assets/<ns>/finishes.json`（`texture`、`tile`、`gloss`、`metal`、`tint`），資源重載時讀。
- `FinishTexture`：完整 mip 鏈、重複、線性過濾、各向異性（原版實體貼圖只上傳一層、取最近的點，斜看會閃）。
- 執行時材質（0.31，給玩家上傳的貼紙）：
  - `PartFinishes.define(id, look, texture)` 把遊戲中做的貼圖登記成材質，`undefine` 拿掉；資源重載後仍在。
  - 烘焙好的網格只記材質 id，所以定義或換圖都不必重新烘焙；`RuntimeTexture.replace` 在原地換圖（render type 照用）。
  - 貼圖：
    - `RuntimeTexture` 有 mip 與各向異性，貼紙要 clamp（否則遠端的邊滲到近端）。
    - alpha 是光澤與鏤空門檻：透明像素 0、不透明 128＋127×光澤，材質的 `cutout` 取 0.25（同字樣）。
    - 貼花層不混色，半透明的邊緣會變成硬邊。
  - 拿掉的材質在用它的網格上畫成白色（`PLAIN`）：要隱藏就換成透明的貼圖，不要拿掉。
- `PartRenderTypes`：每種貼圖 × 層（實心、雙面、貼花、半透明、加色發光）各一個，記住重用。
- `PartLayers`：一幀內先記錄，`flush` 時依 render type 一次送出（buffer source 換 type 就結束一批，逐零件送會結束幾十批）。
  舊程式寫的 VertexConsumer 用 `legacy()` 記錄、以原版白色混凝土畫。輸出頂點自己乘矩陣（原版的 pose 多載每個頂點配置兩個向量）。
- `PartVehicleRenderer`：零件網格在背景烘焙；依「半徑／距離」選細節層級（有遲滯）；每個零件取自己位置的光（在實心方塊裡用載具的）。
- `PartRenderer.bake(part)`、`animate(..., PartLayers, light)` 新多載（舊的照用）。
- 熱的面：建網格時用 `hot(true)` 筆標出（噴嘴、排氣歧管）；`PartRenderer.glow(part, entity, id, partialTick)` 回傳現在要加在這些面上的光
  （ARGB，alpha 是強度，例如 `Blackbody` 在金屬溫度的顏色），`PartVehicleRenderer` 在實心之後用加色把那幾組面再畫一次。溫度由零件的渲染器自己記。
- `PartVehicle.partPoint(installed, local, partialTick)`、`partDirection(...)`：零件座標中的一點或方向在世界的位置（和繪製同一個框架），給特效（噴嘴的煙）用。
- GPU 快取（`PartGpu`，`PartLayers.gpu(true)`）：烘焙好的實心、雙面與貼花群組第一次畫時上傳成顯示卡緩衝（網格自己的座標、遮蔽亮度併進顏色），
  之後每幀只設零件著色器的 `PartPose`、`PartNormalMat`、`PartLight`（逐頂點寫入時是單位矩陣與 -1）再畫。只在零件著色器啟用時（原版著色器從每個頂點拿
  位置與光）、沒有染色、不是發光或半透明時使用；網格被 `MeshBakery` 忘掉時釋放緩衝，`MeshBakery.clear` 時全部釋放。`PartLayers.draw(MeshBuilder …)`
  （當場烘焙）不上傳。hdfv 0.11 量到：同一畫面 12.1 萬面，逐頂點 13.1 ms／幀、GPU 快取 1.5 ms；`/aerocoredev parts gpu on|off` 比較。
- 零件著色器 `shaders/core/rendertype_aerocore_part`（`PartShader`、`PartShading`、`PartLighting`）：
  - 原版兩燈漫射×光照圖，加 Blinn-Phong 高光（世界中太陽或月亮，乘天空光的平方；設計畫面用第一盞燈）與天空漸層反射（Schlick；金屬 0.6、亮面漆最多 0.3）。
  - 光澤在貼圖的 alpha（0.5 + 0.5 g，原版 cutout 不會丟像素）；金屬度每個材質一個 uniform（`PartSurface`）。
  - 設計畫面（正交投影）是攝影棚：固定的環境顏色，畫面 y 朝下。
  - 用戶端設定 `partShading`：AUTO（預設；Iris 光影包使用中時改原版）、SHADER、VANILLA；著色器載入失敗時記一行錯誤並用原版。
    `/aerocoredev parts shading auto|shader|vanilla` 比較。
- `PlumeRenderer`：依 `Plume` 畫尾焰（外焰、核心、菱形震波、馬赫盤），加色發光、貼圖沿下游捲動、長度微幅閃動。
- `RotorBlur`：每秒 4～12 轉之間由葉片淡入成半透明的盤。

## 開口、內裝、可動件與燈（0.30）

- **標記**（`MeshBuilder`，筆與 `Quad` 的存取）：
  - `CABIN`（64）：只從開口看得到的內裝。第 0 層不做封閉判斷（玻璃在體素格子上不擋外面的空氣，但窗太小時仍會被判封閉）；
    第 1 層起拿掉。
  - `CLEAR`（128）：玻璃。不進遮蔽的體素、不算覆蓋。有內裝的網格在第 1 層起改成不透明、亮度約一半（走實心群組、進 GPU 快取）。
    半透明的四邊形同樣不擋、不覆蓋。
  - `LAMP`（256）加燈號（第 12～19 位元）：燈罩，烘焙成自己的群組（`PackedMesh.Group.lamp`）。
  - 最近層級（第 9～10 位元，`levels(near, far)`）：從第 near 層畫到第 far 層。網格有這種遠處版本（例如沒開孔的外殼）時，
    第 0 層和較遠的層各算一次遮蔽，兩者不會互判覆蓋而一起被剔掉。
  - 件號（第 20～27 位元，`piece(n)`）：0 是固定部分，1～255 是會動的件。
- **`SurfaceCut`**：在放樣格子（站 × 射線，可環繞）上挖洞。
  - 洞是 (u, v) 裡凸多邊形的聯集加外框；`DISCARD`（窗、輪拱）拿掉，`PLUG`（門、座艙罩）另成一塊可以轉的板。
  - 每格精確切割：凸區域減凸多邊形是幾塊不相交的凸多邊形。
  - 新點放在格子自己的雙線性面上：落在格邊上的點就在兩格共用的直邊上，和沒切的鄰格密合（多出共線的點）。
  - 洞依給的順序由內往外處理（門上的窗先拿掉，門剩下的才是門板）。
  - 外框回傳成格面上的一圈點（跨格線處加點），`strip` 把外殼的洞和內襯的同一個洞接成門框板。
- **可動件**：
  - `MeshBaker.pieces` 依件號烘焙：一次遮蔽、各件靜止姿態，回傳 [件][層]。`MeshBakery` 遇到有件號的網格自動這樣烘，
    `Baked.piece(p, level)`。
  - `PartRenderer.animate(..., Drawn)` 拿到這次畫的烘焙與層級，渲染器在各件自己的姿態下畫 `drawn.piece(p)`：轉的件不必每幀重建。
  - AeroCore 自己的螺旋槳與控制面改成這樣（件 1）。
- **燈**（`api.mesh.Lamp`：位置、軸、件、半徑、顏色、LENS／POINT／BEAM、扇區、光束角、射程、角色）：
  - 跟著網格移動：`add`、`MeshOps.mirror`、`placed`、`clipped`、`Mesh.of` 都帶著燈。
  - 亮度由零件的規則給（`PartRenderer.lamps`：0 關、1 開、大於 1 閃）。`PartVehicleRenderer` 畫固定部分的燈；件上的燈由渲染器
    在件的姿態下 `layers.lamps(drawn, piece, stack)`。
  - 燈罩加色發光；POINT 與 BEAM 另有面向鏡頭的燈暈（最小約 0.0025 × 距離，遠處的航行燈看得到；在扇區外淡出；夜間較亮）。
  - BEAM 在暗處有光束錐，並有落點光斑（每幀一條射線，48 m 內最近的 8 個）。只是顯示：不照亮方塊（原版沒有會動的光源）。
- **晚段半透明佇列**：世界中的渲染器 `PartLayers.late(true)`，玻璃、燈暈、光束在世界的半透明方塊之後畫（`PartLate`），
  水不會被窗戶擋掉。實體的姿態以鏡頭為原點，所以記下的頂點可以照原樣在那時候畫。
  Fabulous 畫質時這個階段畫進半透明目標，合成時依 alpha 疊上去；加色的燈暈、光束、光斑用 1/255 的 alpha（寫 255 會是黑方塊，寫 0 會被略過）。
- **著色器（per finish，`finishes.json` 新鍵）**：
  - `clearcoat`：清漆的第二道高光與清楚的天空反射。
  - `flip`：珍珠漆的顏色隨掠射角轉向。
  - `cutout`：貼圖 alpha 低於它的像素不畫（字、格柵）。
  - `fresnel`：玻璃越斜看越不透明、越像鏡子。
  - 原版／Iris 備援忽略這些鍵。
- **效能**：
  - 少於 96 面的群組不進 GPU 快取（每個群組一次繪製呼叫，很多小群組比寫幾個頂點還貴）。
  - `PartStats` 加印每幀 GPU 繪製次數。
  - LOD 的遲滯改以（載具實體、零件 id）為鍵：同一個設計的兩台載具不再互搶。
- **`PartRenderer.registerDefault`**：AeroCore 自己零件的外觀放在最後才查的預設表，載具 Mod 依零件或設計類別登記的外觀一定優先。
- **地面與水面**（客戶端）：
  - `GroundMarks.mark(key, point, width, argb, life)`：方塊頂面的胎痕帶，淡出，最多 4096 段。在半透明方塊之後畫（深度測試、不寫深度）；不要移到實心方塊之後：
    那個階段的視圖矩陣還沒有相機的轉向，以相機為原點的頂點會畫到別處（0.30 開發時胎痕一條都沒出現）。
  - `SprayRenderer.emit(...)`：丟一顆水滴（和船艏水花同一套飛行）。
  - `RotorWash.tick(...)`：旋翼或向下噴流吹起地面方塊的粒子、水面的水花環，強度用 `api.aero.Downwash`（顯示近似）。
- **開發**：
  - `MeshBakery.Spec` 的 `maxVoxelM`：大零件的遮蔽體素上限（機身的窗只有幾個體素寬），格子最多 2400 萬格。
  - 測試：`MeshPiecesTest`（內裝、玻璃、遠近層、件、燈）、`SurfaceCutTest`（面積守恆、巢狀洞、外框、朝向、門框板）。

## 尾焰（`api.engine.Plume`、`Blackbody`）

- 完全膨脹的噴流：從出口總壓等熵到大氣壓；直徑由 A/A*。
- 震波格間距 1.306 D_j √(M_j² − 1)（Prandtl、Pack 1950；Tam 1987 對量測約 10 %）；音速噴口壓比 3 時約 1.25 D（Powell 1953 量到 1.1～1.3）。
- 馬赫盤：p_e/p_a 大於約 2 時在 0.67 D √(p_0/p_a)（Ashkenas、Sherman 1966，音速噴口）。
- 勢核 L_c/D_j = 4.2 + 1.1 M_j²（Lau、Morris、Fisher 1979，未加熱），熱噴流乘 (T_j/T_a)^−0.2（趨勢的擬合）。
- 顏色：黑體（Planck × CIE 1931 2° 配色函數的 Wyman 等 2013 擬合 → sRGB），對 Charity 表 1000～6500 K 在 ±30／255 內；
  氫幾乎看不見（淡藍）、碳氫貧油藍、後燃與富油是煙粒的黑體橘黃、APCP 白光加白煙、KNSB 鉀的粉紫加白煙、甲烷火炬橘黃。
- 顯示用：軸對稱、沒有算黏性混合；超燃的二維噴嘴用等效直徑。

## 大量固定燈（0.38）

hdfv 0.20 的跑道燈、進場燈、PAPI、滑行道燈、障礙燈與燈標：要在遠處（公里級，視距外沒有區塊也沒有方塊實體）看得到，所以燈的清單由載具 Mod
（伺服器同步）交給客戶端的 `FieldLamps`，不靠方塊實體渲染器。

- **資料**（`api.light`，純 Java）：`FieldLamp` 是一盞燈的位置、軸、水平與垂直涵蓋角（在仰角上的中心）、光強（cd）、顏色或分色（`SectorColours`：PAPI 依仰角、
  跑道頭與跑道尾依方位）、訊號（`LightSignal`）、迴路、燈罩大小。涵蓋範圍外 5° 羽化。
- **亮度**（顯示規則）：照度 E = I e^{−σd}/d²（Koschmieder，σ = 3/MOR，MOR 由 `WorldAir.visibilityM`），門檻 Et 依背景亮度（Allard：log Et = −5.7 + 0.64 log B；
  夜 6.85 cd/m²、日 10⁴ cd/m²，依天光內插）。強度 s = clamp(log₁₀(E/Et)/3, 0, 1)：剛好看得到時淡、亮過門檻千倍時滿。10 000 cd 在 MOR 800 m：夜 1672 m、日 812 m（`AllardTest`）。
- **畫法**：每幀在 `PartLate`（半透明方塊之後）一批畫完：面向鏡頭的加色四邊形，最小 1.5 px、強光多 2.5 px 的暈；自己的著色器 `rendertype_aerocore_field_lamp`
  （POSITION_TEX_COLOR，不吃霧，不丟 alpha：原版的 position_tex_color 丟掉 alpha < 0.1，而加色混合靠 alpha 1/255 才在 Fabulous 下正確）。
  視錐與距離剔除，最遠 8 km；遠於遠裁切面 0.9 倍的燈沿視線拉近、等比放大（看起來的大小與方向不變）。閃燈用 `LightSignal.peak` 看上一幀到這一幀之間有沒有亮過。
- **介面**：`put(維度, 擁有者, 燈)`（換掉那個擁有者的全部燈）、`remove`、`circuit(擁有者, 迴路, 亮度 0～1)`、`count`、`clear`（登出自動清）。只畫目前維度的。
- **量測**（RTX 5070、1280×720）：1638 盞約每幀 0.3～0.5 ms；`PartStats` 的 field lights 一行（持有、畫出、毫秒）。開發指令 `/aerocoredev lamps grid <n> <間距>|clear`。

## 選用的材質著色（0.38）

材質（`finishes.json`）的新鍵都要自己選用；沒寫的材質畫面逐像素不變（0.38 開發時前後截圖比對，差 ≤ 2 級的只在會動的東西上）。

- `"shading": 2`：套用 `"tint"`（乘在貼圖上）與 `"glossScale"`（光澤貼圖的倍率）。
- `"normal": "<貼圖>"`：切線空間法線貼圖（R 沿 +u、G 沿 +v、B 是凹陷：乘在環境光上），`"normalStrength"` 倍率。TBN 在螢幕空間由 `dFdx`／`dFdy` 與 UV 建，
  逐頂點與 `PartGpu` 兩條路徑都能用（`Sampler3`、`PartSurface3`）；Iris 時照舊改走原版。
- `"glass": "reflective"`：反射加在 alpha 之後：`outA = a + (1 − a)·R`、`rgb = (a·底色 + 反射 + 高光) / outA`，Fast、Fancy、Fabulous 都對（原本天空反射被 alpha 乘掉，
  淡的玻璃看起來是黑的）。
- `"farGlass": 0.8`：有內裝的網格在第 1 層起把玻璃畫成不透明，亮度用這個值（預設 0.55）。

## 美化著色（0.39）

客戶端設定「美化著色」（`AeroCoreClientConfig.beautify`，預設開，Mod 清單的設定畫面）。關掉時畫面和 0.38 一樣：
0.39 開發時前後截圖比對，差的只有會動的東西（水面、旋翼、轉台）。

- **開關**：`client.Beautify`：`on()`、`generation()`、`onChange(Runnable)`。開發指令 `/aerocoredev parts beautify on|off|config` 暫時蓋過設定。
  每 tick 檢查是否切換；切換時 `MeshBakery.refresh()` 換一代重烤，新的烤好前照舊畫上一次的烘焙（載具不會消失），再通知 `onChange` 的監聽者。
  模型的形狀與貼圖不受開關影響；開關只管著色、遮蔽的細分、燈罩光暈與接觸陰影。
- **材質**：材質 JSON 的 `"beautify"` 子物件在開著時蓋在頂層的鍵上（`PartFinishes`）；頂層的鍵照舊是關掉時的樣子。hdfv 0.22 每種材質都有：
  `"shading": 2`、法線貼圖（凹陷的深度依材質的起伏：車漆約 3 %、鑄件 25 %），玻璃另加 `"glass": "reflective"` 與 `"farGlass"`。
- **著色器**（`rendertype_aerocore_part`）：`PartGlobal`（x 開關、y 方塊光高光強度 0.8、z 白天係數）、`BlockLightColor`（1, 0.74, 0.46）、
  逐頂點的方塊光 `blockLight`。新的項都在 `if (PartGlobal.x > 0.5 && !studio)` 裡，舊的敘述一字不動：
  - 方塊光把被照到的地方加暖：底色乘 `mix(1, 1.15·暖色, 0.25·bl²·夜)`，夜 = 1 − 0.8·白天；
  - 光滑面上的暖色高光：半向量取 (向上 + 視線)，`fresnel·暖色·bl³·夜·sheen·(0.2 + 0.8·光澤)·0.8`。
- **地面色**（`PartLighting`）：每 20 tick 取鏡頭周圍 5×5 個柱（間隔 8 m）的頂層方塊顏色，地面色 = 0.35·原本 + 0.65·(0.45·地色·光 + 0.03)，反射與環境光的下半球用它。
- **遮蔽細分**（`AmbientOcclusion.Settings.refine`）：`CORNERS`（0.38 的規則，舊的建構子預設）或 `CENTRE`：再取面中心，中心和四角平均差超過 0.12 也切。
  開著時烘焙用 `CENTRE`，四角都在凹角、中間開闊的長面不再整面變暗。
- **燈罩光暈**：純 Java 的 `LensPatches.of(PackedMesh, lampId)` 把一盞燈的燈罩四邊形合成小片（0.3 m 內、朝向差 45° 內的併在一起，長的切成約 0.6 m），
  依烘焙快取。開著時 `PartLayers.lamps` 在晚的那一輪把每片交給 `LampRenderer` 畫點狀光暈：強度 = 亮度·0.5·天暗·(法線·視線)^0.5，
  大小 = max(3·半徑, 0.002·距離)；256 m 內、每幀最多 384 個。`Lamp` 的 API 不變。（初版 0.35 與 1.4 倍半徑在機場世界只看得到幾個淡點。）
- **接觸陰影**（`ContactShadows`）：載具底部的軟矩形：腳印是各零件第 0 層外框中、離最低點 1.2 m 內的那些（加 12 % 與 0.1 m），
  3×3 個點各往下打一條射線（4 m 內；移動 5 cm 或每 4 tick 重打），強度 0.5·(1 − 高度/3)²·(0.35 + 0.65·天光)，大多在水上時減半；
  用原版的 `entityShadow` 畫，開著時原版的圓形影子（`shadowRadius`）是 0。
- **工具**：`scripts/ShotDiff.java`（`java ShotDiff.java 前 後 輸出資料夾`）：同名視角兩兩比對，印出最大差、差超過 2 與 16 級的比例，輸出熱度圖。
- **GPU 繪製**（不受開關影響）：`PartGpu.draw` 每種 render type 只 `setDefaultUniforms` 與 `apply` 一次，每個工作只上傳
  `PartPose`、`PartNormalMat`、`PartLight` 再畫；原本每次用 `VertexBuffer.drawWithShader`，每畫一次就整個重套著色器
  （每個 sampler 查位置、綁貼圖、檢查每個 uniform）。hdfv 0.22 把車輪改成真正的可動件後，五台載具的排列每幀多花 5 ms 在這裡。
  `PartLayers.GPU_MIN_QUADS` 因此從 96 降到 32：幾十個四邊形的小群組（螺帽、輪轂、卡鉗）從緩衝畫比每幀逐頂點寫便宜。

## 規則

- 新的通用形狀加在 `MeshBuilder`，零件專屬的造型放載具 Mod；形狀要給 UV、四邊形朝外。
- 繪製只經 `PartLayers` 送出（每種材質一個 render type，逐零件送會讓 buffer source 不停結束批次）。
- 世界中的零件剔除封閉與被覆蓋的面；剖視（設計畫面）的殼要和內部分開烘焙，剖開時內部才不會被剔掉。
- 車削剖面的繞向決定面朝哪裡：封閉剖面在 (t, r) 平面要順時針（法線朝外）。寫反的剖面看起來一樣，但遮蔽會往金屬裡取樣（大片暗斑），
  薄壁的兩面會互判被覆蓋而整個消失（hdfv 0.11 的脈衝噴射管與火箭噴嘴）。
- 世界中的渲染器（實體、方塊實體）用 `new PartLayers().gpu(true)`，畫的網格要來自 `MeshBakery`；氣流通道這類通到外面的內部，遮蔽的「外面的空氣」
  會流進去，要由模型標 `interior`。
- 遮蔽的參數（射線起點 2 個體素、覆蓋間隙、細分門檻）改動時跑 `AmbientOcclusionTest`，並用 `hdfvdev export` 加 Blender 看一具大引擎與一具小引擎。
- 玻璃要標 `CLEAR`（或半透明）、窗後的內裝標 `CABIN`，否則內裝在世界中被判封閉而剔掉，或被玻璃的體素遮黑。
- 會動的件用 `piece(n)` 烘在同一個網格裡、在 `animate(..., Drawn)` 畫：不要每幀現建網格（`layers.draw(MeshBuilder ...)` 只給很小的臨時東西）。
