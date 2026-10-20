# 聲音（0.37）

使用者 2026-10-18「加入聲音」（hdfv 0.19）：**全部程式合成**、不帶音檔；都卜勒與傳播延遲、真實距離衰減與 ISO 9613 空氣吸收、
室內迴響與隔音（OpenAL EFX）、音爆與超音速。AeroCore 放聲音的物理與計算（`api.acoustics`，純 Java）與接 Minecraft 聲音引擎的通用接口
（`neoforge/.../client/sound`）；哪個零件發什麼聲、何時發、分配與設定在載具 Mod（hdfv 的 `sound`、`client/sound`）。

## 純 Java：`dev.mcmod.aerocore.api.acoustics`

### 基礎

| 類別 | 內容 |
| --- | --- |
| `Voice` | 44.1 kHz 單聲道的聲源介面（`render(out, offset, n)`）；`Voice.render(v, 秒)` 離線算一段 |
| `Noise` | 種子決定的雜訊：白（xorshift64*）、粉紅（Paul Kellet，9 Hz～Nyquist 誤差 0.05 dB）、棕；同種子同樣本（兩個玩家聽同一台車一樣） |
| `Filters` | RBJ 雙二階（`lowpass`、`highpass`、`bandpass`、`peaking`，執行中重調用 `tune*`）、零延遲回授的狀態變數濾波（Zavalishin／Simper）、一階低通 |
| `DelayLine` | 分數延遲（三次 Hermite）：排氣管波導、梳狀與全通、變動延遲 |
| `Oscillator` | 相位累加器、PolyBLEP 鋸齒與脈衝（不混疊） |
| `Modal` | 模態合成：二極共振器組（Adrien 1991；van den Doel & Pai 1998） |
| `Dsp` | dB 換算（`P_REF` 20 µPa、`W_REF` 1 pW）、平滑係數、軟限幅 |
| `Mix` | 多個聲源混成一條（增益在一塊內斜坡，改增益不爆音） |
| `Spectrum` | FFT、Welch、峰值、1/3 倍頻程、A 加權（只給測試與匯出） |
| `Wav` | 16 bit 單聲道 WAV 讀寫 |

### 傳播與聽者

- `Propagation`：
  - 球面擴散 Lp = Lw + DI − 20 log r − 11（ISO 9613-2 式 7）。
  - ISO 9613-1 的空氣吸收（溫度、相對濕度、氣壓；20 °C、70 % 時 1 kHz 約 5 dB/km、4 kHz 約 23 dB/km）。
  - `airCutoffHz`：吸收到 3 dB 的頻率。
  - 都卜勒。
  - 延遲時間求解 `emissions`：
    - 次音速一根解；超音速時，聽者在錐內有兩根、錐外沒有。
    - 解剛好在零時去重。
- `SourceTrack`：一個移動聲源最近 6 秒的位置。`hear` 給出「現在聽到的」：
  - 從哪裡來（延遲後的看起來位置）、延遲多久、都卜勒係數。
  - `boom`：到達時間 T(t) = t + |聽者 − 聲源(t)| / c 的折點（T 停止下降處：聲源朝聽者接近得比音速快時發出的聲音同時到達）
    落在上一次與這一次 `hear` 之間時報一次（第一次 `hear` 不報：聽者可能已經在錐內）。直線段上 T 是凸的，最小值在視線方向的接近速度等於 c 處
    （離正側方還有 1/√(M²−1) 倍的側向距離）。
  - 看折點的時刻，不看那一刻有沒有聲音到達（hdfv 0.19 多人驗證時發現）：從 64 m 外以 600 m/s 發射的車，這段聲音只有 8 ms，夾在兩個 tick 之間；
    它靜止時發的聲音也還在路上（到達數 1 → 3，或一個 tick 內 1 → 2）。原本「到達數從 0 跳到 ≥1」的規則兩種都抓不到。
  - 一段快於 `JUMP_MPS`（3000 m/s）的是被搬走的載具，不是飛行，不算音爆。
  - 只有一筆記錄時，延遲模式下聽不到（還不知道它從哪來）。
- `SonicBoom`：Carlson 簡化法（NASA TP-1122）的 N 波超壓與持續時間（形狀因子 0.1、地面反射 1.9），`samples` 產生 N 波。
- `RoomAcoustics`：
  - 從聽者射出的射線（距離、表面、入射餘弦）估房間：體積 4π/3·mean(r³)、表面 4π·mean(r²/cos)。
  - 每個倍頻帶的平均吸音、Eyring T60（含空氣）、平均自由路徑、臨界距離。
  - `reverb` 把房間換成 EFX 迴響參數（名稱與範圍照 EFX；增益隨封閉度上升，是聆聽規則）。
  - `wetOverDry` 是迴響對直達的振幅比。
- `Surface`：倍頻帶吸音係數（Everest & Pohlmann、Kuttruff 表 6.2）與撞擊用的密度、損失因子、會不會響；`Surface.of(ContactMaterial)`。
- `Transmission`：質量定律與組合牆（面板、玻璃、開口的面積加權）→ `Insulation`（500 Hz 的寬頻增益、4 kHz 的相對增益：EFX 低通的兩個數）。
- `Loudness`：真實聲壓級 → 播放增益（使用者聽的是喇叭，遊戲世界比現實大聲）。
  - 75 dB SPL 以下每 1.6 dB 真實＝1 dB 播放，膝點在 −6 dBFS。
  - 以上軟膝趨近 0 dBFS，永遠不超過。
  - 怠速的車在 7.5 m（約 50 dB）播 −22 dBFS，全油門 75 dB 播 −6 dBFS。
  - 常數是聆聽選擇，對原版聲音調。

### 聲源模型

每個模型給聲功率級（`soundPowerDb`）、主要頻率與一個 `Voice`；參數在 `volatile` 的狀態快照裡，遊戲執行緒設、合成執行緒讀。

| 類別 | 物理 |
| --- | --- |
| `PistonEngineNoise` | 點火間隔由曲柄角算（`firingAngles`：直列、V、對臥、十字形與平面曲軸 V8、V 型雙缸）；每缸排氣脈衝依負載經各排集氣管與排氣管（波導、消音器）；進氣 Helmholtz、機械聲、柴油敲擊、爆震；增壓器葉片音（聽得到的是 2 倍軸頻 `turboWhistleHz`）、洩壓閥（Baldan 等 2015 的物理式合成） |
| `TurbineNoise` | 噴流混合（Lighthill U⁸、Strouhal 0.2、後方 30～40° 最大）、葉片通過頻率與諧波、葉尖超音速的 buzz-saw、燃燒室、後燃器、衝壓、超燃、火箭劈啪、脈衝噴射 |
| `RotorNoise` | Gutin 葉片音（聲功率隨葉尖馬赫 6 次方）、寬頻、直升機拍擊、船用螺旋槳的空蝕 |
| `MotorNoise` | 2 倍電頻的磁力、槽音、風扇、變頻器載波與邊帶；小馬達（雨刷、起落架泵） |
| `TyreNoise` | 胎紋撞擊 v÷節距、空氣泵送、30～40 log v、尖叫（滑移）、鎖死拖磨、碎石、沙、雪、濕路 |
| `FlowNoise` | 邊界層與邊緣的偶極（Curle，U⁶）、Strouhal 頻率、陣風、開窗或開放座艙的空腔低鳴 |
| `WaterNoise` | 船首浪花、Wagner 拍水（p = ½ρv²(π²/(4 tan²β)+1)）與船體的模態悶響、空蝕 |
| `ImpactNoise` | 模態合成：Kirchhoff 平板模態 f = 0.453 c_L h ((m/a)²+(n/b)²)、損失因子的衰減、Hertz 接觸時間的觸發；`carPanel`（0.8 mm 鋼、0.3×0.25 m） |
| `GasJetNoise` | 容器放氣：1.89 以上阻塞、等熵噴速、Lighthill 噴流（K≈3e-5）、壓力隨質量流衰減（氣煞、洩壓閥、蒸汽排汽、氣動工具） |
| `ProcessNoise` | 機器的製程：刀齒通過頻率（齒數×主軸轉速）敲響刀具與工件模態、車削的切屑撕裂、鑽孔、磨削的高頻、往復與衝程、放電、電漿、雷射、水刀、MIG／TIG、燃燒器、蒸汽、泵、風扇、噴槍、天車、2 倍電網頻率的嗡聲、皮帶；聲功率 70 + 10 log P_kW + 製程的修正 |
| `Horn` | 電喇叭雙音（約 420、510 Hz）、氣喇叭、COLREGs 附錄 III 的船笛（依船長 70～700 Hz） |

## Minecraft 的接口：`neoforge/.../client/sound`

查到的限制（原始碼）：串流聲道最多 8 條（`clamp(√來源數, 2, 8)`，和音樂、唱片機共用，沒空時 `createHandle` 靜靜回 null）；
串流只在每個客戶端 tick 補「播完的塊」，主執行緒卡住沒人補，一放空來源就停、Minecraft 把聲音整個丟掉；音高夾 0.5～2、音量 0～1；
NeoForge 在聲音執行緒發 `PlaySoundSourceEvent`／`PlayStreamingSourceEvent`（帶 `Channel`）。

- `RuntimeSounds`：程式產生的靜態聲音，id 是 `<ns>:runtime/<名字>`。
  - Java 保存 PCM；`SoundBufferLibrary.getCompleteBuffer` 的 mixin 在第一次播放時（以及每次重新載入資源後）交給 OpenAL。
  - 已交出的 id 不換內容，內容不同就換 id。
- `SynthInstance`：覆寫 `resolve`，不經 sounds.json。
  - 兩種：串流（`streamed`）或靜態緩衝（`buffered`）。
  - 衰減 NONE：呼叫端自己算增益。
  - 可以靜音開始（`canStartSilent`）。
  - 串流的循環加延遲 1 tick：真的放空時原版會在下一 tick 重開，而不是丟掉。
- `SynthStream`／`SynthThread`：
  - 自己的合成執行緒每塊算 1024 樣本（23 ms）進環形緩衝，領先最多 2048。
  - 補塊執行緒每 10 ms 把 `updateStream` 交給聲音執行緒：主執行緒卡住也不放空。
  - `read()` 只複製；緩衝不夠時當場補算（`rescue`，記數在 `rescued()`）。
  - Windows 用 `Thread.sleep(1)`：`parkNanos` 的解析度是 15.6 ms。
- `LiveSound`：給擁有者每 tick `update`（位置、音量、音高、是否相對聽者）的長聲音。
  - 被 Minecraft 丟掉時重開：F3+T、換資源包、換音訊裝置、`/stopsound`、類別音量 0 再調回、串流放空。
  - 開新串流時至少留 `reservedStreams`（2）條空的串流聲道給音樂與唱片機。
- `SoundHooks`：
  - 聲音執行緒的 executor 與重新載入的世代（`SoundEngineLoadEvent`）、空的串流聲道數。
  - 類別是否聽得到。
  - `subtitle`：混音裡的事件自己送字幕。
- `Efx`（EFX 擴充）：
  - 一個迴響（聽者的房間）放在輔助槽。
  - 每個 `SynthInstance` 兩個低通：直達路徑（牆、車艙、空氣）與送進迴響的量。
  - 所有 OpenAL 呼叫在聲音執行緒，每批結束清掉 `alGetError`，不讓錯誤漏到原版的檢查。
  - 重新載入時換代重建；聲音結束後刪掉它的濾波。
  - 沒有 `ALC_EXT_EFX`、裝了 Sound Physics Remastered、或呼叫端關掉時，`available()` 為 false，呼叫端只調音量。
  - 只碰 `SynthInstance`，原版的聲音不動。
- `Listening`：
  - 每 10 tick 從相機射 32 條射線（Fibonacci 球、48 m）估房間，迴響交給 `Efx`。
  - `between` 數兩點之間的實心方塊（最多 3）。
  - 遮擋規則（聆聽規則，不是牆的模型）：每塊留 0.55 寬頻、再留 0.3 的 5 kHz。
  - `airHf`：ISO 9613-1 在 5 kHz 的空氣損失。
- 開發指令 `/aerocoredev sound sine|click|stop|stall|stats|record`：測試正弦波、喀聲、故意卡主執行緒、串流統計（放空、補算、重開）、錄下實際送出的樣本（WAV 到 `%USERPROFILE%\.mcmod\exports\sound\`）。

## 伺服器：接觸與遠處的聲源

- `ContactSolver.BodyResult.touches`：每個接觸的接近速度、法向衝量、滑動速度、另一物體、材料。
  - 只讀，物理逐位元不變（`VehicleTest` 1e-9）。
- `VehicleContacts`（`PartVehicle` 每 tick 彙整）：
  - 撞擊（合併的衝量、接近速度、位置、方塊材料或另一台）：伺服器事件 `PartVehicle.Contacted`；每台每 tick 最多一個封包送給追蹤的玩家。
  - 刮擦（摩擦功率、滑動速度）跟著 DISPLAY 同步（`k`），真實速度也是（`v`：原版運動封包每軸截在 78 m/s）。
  - 客戶端的快速載具（0.28，80 m/s 以上）用同步的 `v` 判斷與往前推位置（2026-10-19 hdfv 多人驗證時改）：原本用原版的運動，
    沿一軸飛的快速載具在客戶端從不算快、落後兩 tick 滑行，聲音的軌跡一開始像次音速。收到伺服器的第一個位置前從進來的位置推。
  - 沒有時不寫，舊字串不變。
  - GameTest `aLandingIsAnImpactAndASlideAScrape`。
- `FarSounds`：追蹤範圍外的大聲載具。
  - 載具 Mod 用 `provide` 給每台車的簽名（聲功率、種類、速率）。
  - 每 5 tick 告訴每個玩家：它沒追蹤（`PartVehicle.seenBy`，用 `startSeenByPlayer`／`stopSeenByPlayer` 記）、聲功率 90 dB 以上、
    在它那裡至少 30 dB（r = 10^((Lw − 41)/20)，最多 1.5 km）的載具，最大聲的 12 台，含位置與速度。
  - 客戶端保留 15 tick。
  - 只有伺服器載入的載具聽得到。

## 驗證

- JUnit（`physics`）：`DspTest`、`PropagationTest`、`SourcesTest`、`MoreSourcesTest`，共 33 項。
  - 傳播與房間：ISO 9613-1 表值、Sabine 的已知房間、質量定律。
  - 都卜勒與延遲時間：次音速一根、超音速錐內兩根、錐外沒有。
  - 音爆的時刻：Mach 2 飛過 300 m 高只響一次、在錐到達時；靜止 2 s 後從 64 m 外以 600 m/s 發射，靜止時的聲音還在到達時也響一次、在錐到達時
    （規則還是「到達數從 0 跳到 ≥1」時這項 0 次）；一次瞬移 200 m、100 m/s 經過都不響。
  - 音爆與噴流：Concorde 的音爆約 2 psf（±25 %）、噴流 80 log V。
  - 引擎：Gutin 的葉片頻率、直列四缸 3000 rpm 的 100 Hz、十字形曲軸 V8 的半階、V 型雙缸的不等間隔。
  - 其他聲源：胎紋頻率隨車速、放氣隨壓力衰減、拍水的模態悶響。
  - 工具：WAV 讀寫、同種子同樣本。
- GameTest：`aLandingIsAnImpactAndASlideAScrape`（1 m 落地有撞擊、衝量與 m·Δv 相符；貼地滑行有刮擦；沒接觸沒有事件）。
- 遊戲中（hdfv 0.19 的 AutoShot）：
  - 測試正弦波 4 條、主執行緒卡 300／600 ms、`/stopsound`。
  - 第一輪找到並修掉串流放空（約 40 % 的塊、喀聲）：補算、`sleep(1)`、領先 2048。
- 多人與音爆（hdfv 0.19 補驗，2026-10-19，本機專用伺服器＋兩個客戶端）：
  - `FarSounds`：伺服器 view distance 8（追蹤 128 m），另一個玩家在 515 m 與 1245 m 外聽到空檔踩油的轎車（46、44 dB），放油門後消失。
  - 音爆：測試車在 100 m 高、從聽者側方 40 m、前方 50 m 與 200 m 處以 600 m/s 發射，兩個聽者各一次，在預期的錐到達時間（4～4.7 kPa、41～45 m）。
    這次找到上面的折點規則、客戶端快速載具的同步速度與進入追蹤的第一步。
- 截圖聽不到：音色與音量平衡要使用者自己聽。

## 已知限制

- 濕度沒有世界模型：預設 70 %，下雨 100 %；溫度由生態域的基礎溫度推。
- 遮擋是規則（每塊固定的增益），不是牆的透射模型；聽者在別的封閉載具裡時，其他聲音只經那個車艙的 `Insulation`。
- `FarSounds` 只看伺服器載入的區塊，單人時通常和追蹤範圍差不多；位置在兩次報告之間依速度外插。
- EFX 只有一個迴響（聽者的房間），聲源各自的房間沒有分別。
- `SourceTrack` 只看記下的點：伺服器慢時客戶端會把快速載具先推過頭再拉回，拉向聽者的那一步比音速快，就是一個假的折點（音爆）。
  餵什麼點是載具 Mod 的事（hdfv 用同步速度推、再往畫的位置拉）。
