# AeroCore API

核心套件為 `dev.mcmod.aerocore.api`，不依賴 Minecraft、NeoForge 或任何原生物理引擎。
輸入與輸出為不可變值；`AeroSolver` 沒有全域模擬狀態。
自訂 `AirfoilPolar` 若共享於多執行緒，須由實作者確保執行緒安全。

## 基本用法

```java
import dev.mcmod.aerocore.api.*;
import java.util.List;

// 同一世界座標系；線速度 m/s，角速度 rad/s。
BodyMotion motion = new BodyMotion(new Vec3d(30, -2, 0), Vec3d.ZERO);
FluidState air = Atmosphere.atAltitude(1000, new Vec3d(2, 0, 0)).fluid();

// offset 是從質心指向施力點的世界座標向量（m），不是世界絕對位置。
// chord 指向翼面前緣，normal 指向正升力側；兩方向必須垂直。
LiftingSurface wing = new LiftingSurface(
        new Vec3d(-1, 0, 0), Vec3d.X, Vec3d.Y, 10, 1.5, SymmetricAirfoil.DEFAULT);
DragBody fuselage = new DragBody(Vec3d.ZERO, 2, 0.4);

Wrench result = AeroSolver.solve(motion, air, List.of(wing), List.of(fuselage));
Vec3d forceNewtons = result.force();
Vec3d torqueNewtonMetres = result.torque();

// 面積 10 m²、弦長 1.5 m；診斷包括攻角、動壓、CL/CD/Cm、失速旗標、Re/Mach。
SurfaceForces detail = AeroSolver.surface(motion, air, wing);
double reynolds = detail.flow().reynoldsNumber();
```

每次載具姿態改變，先將局部的 offset、chord、normal 旋轉到世界座標，再傳入 solver。
不要將載具世界位置加到 offset。API 所有角度都用弧度，只有遊戲驗算指令接受度數。
`LiftingSurface` 會正規化方向並拒絕不垂直或零向量。

## 作為另一個 NeoForge Mod 的前置

AeroCore 發佈到工作區裡的 Maven 資料夾（`AeroCore\build\repo`），有改動時在 AeroCore 執行 `.\dev.ps1 build publish`。
載具 Mod 的 Gradle 設定：

```groovy
repositories { maven { url = file('../AeroCore/build/repo').toURI() } }
dependencies {
    // 純 Java 模組（不依賴 Minecraft）只在編譯與測試時用 physics JAR
    compileOnly 'dev.mcmod.aerocore:aerocore-physics:0.22.1'
    testImplementation 'dev.mcmod.aerocore:aerocore-physics:0.22.1'
    // NeoForge 模組依遊戲 JAR（已含 physics API）
    implementation 'dev.mcmod.aerocore:aerocore-neoforge-1.21.1:0.22.1'
}
```

遊戲 JAR 的發佈不帶依賴；執行時不要再放 physics JAR，同一個套件出現兩次會讓 NeoForge 的模組層衝突。
在載具 Mod 的 `META-INF/neoforge.mods.toml` 宣告依賴；將 `your_vehicle_mod` 改成實際 Mod ID：

```toml
[[dependencies.your_vehicle_mod]]
modId="aerocore"
type="required"
versionRange="[0.22.1,0.23.0)"
ordering="AFTER"
side="BOTH"
```

玩家安裝 AeroCore 的遊戲用 JAR 和載具 Mod。API 仍在 0.x 階段，後續次版本可能調整接口。

## Minecraft 單位

`dev.mcmod.aerocore.neoforge.MinecraftUnits` 採用 1 方塊 = 1 公尺、1 模擬 tick = 0.05 秒：

```java
Vec3d velocityMps = MinecraftUnits.toMetersPerSecond(entity.getDeltaMovement());
// 計算受力，再交給載具的運動積分器；不要直接把「力」當成「速度」。
```

若自行積分平移，`Δv = F / massKg × dtSeconds`，之後轉回方塊/tick。
旋轉需要質量慣性矩、姿態與角速度的剛體積分器，不能單純把力矩加到 Minecraft 的 yaw/pitch。
若接外部物理引擎，確認其單位、施力點、座標系和固定時間步長；避免同時由兩套系統重複積分。
伺服器應持有權威物理狀態。0.3 的測試球示範了平移、方塊碰撞與伺服器同步；通用旋轉剛體尚未實作。

## 平移積分與測試球

```java
TranslationState state = new TranslationState(new Vec3d(0, 10, 0), Vec3d.ZERO);
double massKg = 250;
// 本步將力視為常數；動態力應在小步長中重新取樣。
TranslationState next = LinearDynamics.constantForce(state, new Vec3d(0, -2451.6625, 0), massKg, 0.005);

SphereDynamics.Properties sphere = new SphereDynamics.Properties(massKg, 0.5, 0.47);
SphereDynamics.Result step = SphereDynamics.step(state, sphere,
        Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), FluidState.FRESH_WATER_20C,
        0.5, new Vec3d(0, -9.80665, 0), 0.005);
// step.state() 為平移狀態；step.netForce() 對應本步動量改變，不包含接觸力。
```

`LinearDynamics.quadraticDrag` 使用均勻流場中的解析阻力速度更新，避免高阻力時數值反向加速。
`SphereDynamics` 使用重量、乾／濕排水浮力與分裂阻力步驟；位置用更新後速度前進，
維持浮力振盪的數值穩定性。它是一階平移原型，應以時間步長加密檢查精度。
取樣的浸水比例在每小步內固定，阻力面積按乾／濕比例分配是近似。
不要將它當作完整六自由度船舶或飛機積分器。Minecraft 範例見 [測試球說明](PROBE.md)。

## 六自由度剛體與多點浮力（0.4）

```java
RigidBodyProperties properties = TestPlatform.properties(new Vec3d(0, -0.35, 0));
RigidBodyState body = TestPlatform.atModelOrigin(
        new Vec3d(0, 1, 0), Orientation.IDENTITY, properties);
FloatingBodyDynamics.Environment environment = TestPlatform.flatWater(0);
FloatingBodyDynamics.Result step = FloatingBodyDynamics.step(
        body, properties, TestPlatform.FLOATS, environment,
        new Vec3d(0, -9.80665, 0), 0.47, 0.005);
body = step.state();
Vec3d omegaWorld = body.angularVelocity(properties); // rad/s
```

`RigidBodyState.position` 是重心的世界位置；`velocity` 與 `angularMomentum` 也是世界座標。
`orientation` 是 body→world 的單位四元數，建構順序 `(w,x,y,z)`，不是 JOML 的 `(x,y,z,w)`。
`RigidBodyProperties.centerOfMassLocal` 是重心在模型座標中的位置；慣性張量以重心為原點，軸向與模型一致。
使用 `worldPoint(properties, localPoint)` 算施力點世界位置，`worldOffset(...)` 算從重心出發的世界向量。

`FloatingBodyDynamics` 接受模型座標中的球形排水元件。呼叫者應確保浮筒互不重疊，不要重複算排水體積。
`Contact.wetCentroidOffset` 是**從該浮筒球心出發的世界向量**；水取樣必須回傳正確浮心，才能計算偏心浮力。
`Result.hydrostatics` 含重量與浮力，`averageDrag` 是該步阻力衝量除以 dt，兩者都是以步驟開始重心為參考的世界力矩。
0.5 的 `Result.forces` 提供重量、各浮筒的等效浮力與阻力；`AppliedForce.offset` 是從步驟開始重心出發的世界向量。
可用 `state.position().add(force.offset())` 定位箭頭，`force.wrench()` 的總和與上述兩種力矩相加一致。
每小步重新取樣；同一浮筒的乾／濕阻力都施加在球心，這是目前的簡化。

純空氣載具也能使用同一套剛體 API，例如將模型局部機翼旋轉成既有 `AeroSolver` 所需的世界資料：

```java
Vec3d wingPointLocal = new Vec3d(1, 0, 0);
LiftingSurface worldWing = new LiftingSurface(
        body.worldOffset(properties, wingPointLocal),
        body.orientation().rotate(Vec3d.X), body.orientation().rotate(Vec3d.Y),
        2, 0.8, SymmetricAirfoil.DEFAULT);
BodyMotion motion = new BodyMotion(body.velocity(), body.angularVelocity(properties));
Wrench aerodynamic = AeroSolver.surface(motion, air, worldWing).wrench();
Wrench weight = new Wrench(new Vec3d(0, -9.80665 * properties.massKg(), 0), Vec3d.ZERO);
body = RigidBodyDynamics.kick(body, properties, aerodynamic.add(weight), 0.005);
body = RigidBodyDynamics.drift(body, properties, 0.005);
```

`kick` 更新線動量和角動量，`drift` 推進重心位置及四元數姿態。所有外力只加一次；
不要在 `FloatingBodyDynamics.step` 後再加它已包含的重力／浮力。
`RigidBodyDynamics.drag` 可另接多個 `PointDrag`，使用同時隱式求解；其 offset 是世界重心相對向量。
線性係數單位 N·s/m，二次係數單位 N·s²/m²，都必須非負。
如果已透過其他 solver 算過同一份阻力，就不要再重複加入。

這些 API 不處理儲存、網路或自動重算幾何的質量分布；0.6 起可另傳接觸資料處理方塊碰撞（見下方）。遊戲示例見 [平台說明](PLATFORM.md)。
需要一個非零、正定慣性張量；`InertiaTensor.box` 與 `shifted` 可協助依平行軸定理合成。
固定 0.005 s 是示例設定，不保證所有自訂尺寸、質量和角速度都穩定；請對自己的模型做步長收斂檢查。

## 更換翼型

`AirfoilPolar` 接受攻角，回傳無因次係數；可以用實測曲線插值、可調遊戲曲線或受控翼面資料。
既有角度 lambda 仍有效；solver 會呼叫 `sample(alpha, FlowConditions)`，預設轉交原本的角度取樣。
自訂模型可以覆寫該方法，或者使用 `ReynoldsPolar` 插值資料。預設對稱翼型不會因 Re／Mach 自動改變。

```java
AirfoilPolar custom = alpha -> new AeroCoefficients(
        Math.sin(2 * alpha), 0.04 + Math.pow(Math.sin(alpha), 2),
        Math.abs(alpha) > Math.toRadians(18));
```

回傳升力係數需為有限值，阻力係數需為有限且非負的值。
其他輸入如密度、面積與方向也會檢查；無效資料會直接拋出例外，避免 NaN 傳入載具狀態。

含俯仰力矩的建構式是 `new AeroCoefficients(cl, cd, cm, stalled)`；Cm 正方向沿 `chord × normal`，
並以指定施力點為參考。舊三參數形式令 Cm=0。舊翼面建構式令弦長=1 m，新功能應提供真實弦長。

## 資料表與適用範圍

```java
// 僅為展示格式的合成資料，不是真實翼型數據。
PolarTable lowRe = new PolarTable(List.of(
    new PolarTable.Point(Math.toRadians(-10), new AeroCoefficients(-0.8, 0.05, -0.03, false)),
    new PolarTable.Point(0, new AeroCoefficients(0, 0.02, -0.03, false)),
    new PolarTable.Point(Math.toRadians(10), new AeroCoefficients(0.8, 0.05, -0.03, false))));
PolarTable highRe = new PolarTable(List.of(
    new PolarTable.Point(Math.toRadians(-10), new AeroCoefficients(-0.9, 0.04, -0.02, false)),
    new PolarTable.Point(0, new AeroCoefficients(0, 0.015, -0.02, false)),
    new PolarTable.Point(Math.toRadians(10), new AeroCoefficients(0.9, 0.04, -0.02, false))));
AirfoilPolar measuredModel = new ReynoldsPolar(List.of(
    new ReynoldsPolar.Slice(100_000, lowRe),
    new ReynoldsPolar.Slice(1_000_000, highRe)), 0.3);
```

數據按攻角／Re 嚴格遞增。超出這裡 ±10°、Re 範圍或 Mach 0.3 的輸入會報錯。
正式載具需有覆蓋全飛行範圍的資料，或在自己的模型內明確指定超出範圍時的近似策略。
二維翼型資料若要用於有限展弦比機翼，必須另外處理有限翼修正；不要重複加已包含的誘導阻力。

## 水、浮力與水流

```java
FluidState water = FluidState.FRESH_WATER_20C.withVelocity(new Vec3d(0.2, 0, 0));
BuoyantSphere displacement = new BuoyantSphere(new Vec3d(1, 0, 0), 0.5);
// 球心深度 0：半浸水；平面法向 +Y。浮心 offset 仍相對物體質心。
Immersion immersion = HydroSolver.sphereImmersion(displacement, 0, Vec3d.Y);
Vec3d gravity = new Vec3d(0, -9.80665, 0);
Wrench buoyancy = HydroSolver.buoyancy(water, immersion, gravity);
double massKg = 100;
Wrench withWeight = buoyancy.add(new Wrench(gravity.scale(massKg), Vec3d.ZERO));

// 一個完全浸水、半徑 1 cm 的獨立阻力元件；只適用於模型的 Re 範圍。
Wrench drag = HydroSolver.sphereDrag(
    new BodyMotion(new Vec3d(1, 0, 0), Vec3d.ZERO), water,
    new BuoyantSphere(Vec3d.ZERO, 0.01));
```

水翼可直接使用 `AeroSolver.surface(motion, water, foil)`，水翼需完全浸水且使用合適的係數。
部分浸水的球冠浮力有精確幾何；部分浸水的船體阻力、造波、拍水等並未由這個 API 自動算出。
物體重量／浮力／阻力應合併後交給同一套運動積分器。

## 局部環境場

```java
double seaLevelM = 63; // 由環境模組決定，這裡只是示例。
FluidField environment = (position, time) -> {
    if (position.y() < seaLevelM) {
        return FluidState.FRESH_WATER_20C.withVelocity(new Vec3d(0.1, 0, 0));
    }
    return Atmosphere.atAltitude(position.y() - seaLevelM, new Vec3d(2, 0, 0)).fluid();
};
Wrench total = AeroSolver.solve(motion, new Vec3d(100, 80, 100), environment,
        0, List.of(wing), List.of(fuselage));
```

此例只是平水面／固定風的示範。正式接 Minecraft 必須辨識維度、真實水方塊與海平面，
也要切分跨水面的翼面；不要把所有低於海平面的洞穴當成水。
`FluidField` 不要求儲存整個世界的網格；可以由區域風、水流或經驗證的流場快照提供局部取樣。
目前 CFD CSV 尚未自動載入 Minecraft，避免把二維流道的資料當成通用三維載具係數。

## 溫壓流體性質與熱交換（0.5）

```java
ThermalFluidState airThermal = Thermodynamics.dryAir(303.15, 101325, new Vec3d(4, 0, 0));
ThermalFluidState waterThermal = Thermodynamics.liquidWater(278.15, 100000, Vec3d.ZERO);
FluidState airForForces = airThermal.fluid(); // 給既有的 AeroSolver／浮體環境
double prandtl = waterThermal.prandtlNumber();

HeatTransfer.Body warmBody = new HeatTransfer.Body(353.15, 250000); // K, J/K
HeatTransfer.Result cooling = HeatTransfer.reservoirs(warmBody, List.of(
        new HeatTransfer.Reservoir(303.15, 40), // K, G=h*A (W/K)
        new HeatTransfer.Reservoir(278.15, 500)), 0, 0.05); // 外部功率 W, dt s
warmBody = cooling.body();
double netHeatJ = cooling.energyChangeJ();

HeatTransfer.Pair exchange = HeatTransfer.exchange(
        new HeatTransfer.Body(350, 1000), new HeatTransfer.Body(290, 2000), 10, 1);
// 兩個有限物體：first 吸收 heatToFirstJ，second 放出相同熱量。
```

API 一律用絕對溫度 K。`ThermalFluidState` 包含 T、p、cp (J/kg/K)、k (W/m/K) 與原有 `FluidState`。
空氣支援 180～500 K、1～300 kPa；純液態水支援 273.15～353.15 K、80～300 kPa。超界拒絕，不靜默夾值。
密度／黏度與聲速會跟 T／p 改變；使用者仍需把 `.fluid()` 傳入力學計算，而不是同時另外添加一份受力。

`reservoirs` 是固定環境溫度的熱庫；`energyChangeJ = externalEnergyJ + heatFromReservoirsJ`，正值表示本體吸熱。
`exchange` 則為兩個有限、均溫、固定熱容量物體之間的等量反向傳熱。
沒有自動將耗散的機械能轉為熱能，也不更新 Minecraft 方塊。模型來源與示例限制見 [熱力學與視覺化](THERMAL_DEBUG.md)。

## 0.6 相容性變更

- `FloatingBodyDynamics.Result` 增加 `contact`、`external`（呼叫者外加力的總和）與 `dissipation` 欄位；欄位順序改變，請用存取方法而非建構式。
- `AppliedForce` 增加 `couple`（純力矩，N·m）與 `CONTACT`、`THRUST` 兩種類別；舊的三參數建構式仍可用。
  對 `AppliedForce.Kind` 做完整 `switch` 的程式需補上新類別。
- `TestPlatform.flatWater` 改由 `wavyWater(waterY, WaveField.NONE, 0)` 實作，結果與 0.5 相同。

## 接觸反力（0.6）

```java
RigidBodyProperties properties = TestPlatform.properties(TestPlatform.DEFAULT_BALLAST);
List<CollisionSphere> shape = TestPlatform.collisionShape(TestPlatform.DEFAULT_BALLAST); // 模型局部
List<WorldBox> obstacles = List.of(new WorldBox(new Vec3d(-10, -1, -10), new Vec3d(10, 0, 10))); // 世界 m
FloatingBodyDynamics.Collision collision = new FloatingBodyDynamics.Collision(
        shape, obstacles, new ContactMaterial(0.5, 0.2)); // 摩擦係數、恢復係數
FloatingBodyDynamics.Result step = FloatingBodyDynamics.step(body, properties, TestPlatform.FLOATS,
        TestPlatform.flatWater(-5), new Vec3d(0, -9.80665, 0), 0.47, List.of(), collision, 0.005);
Wrench support = step.contact(); // 本步接觸衝量 / dt，對步驟開始重心
```

`CollisionSphere` 用模型局部座標；方塊、甲板之類的形狀以角落小球近似。`WorldBox` 是靜止的世界軸對齊盒，
Minecraft 可由 `level.getBlockCollisions(entity, region)` 的 `VoxelShape.toAabbs()` 轉換。障礙物不會被推動。
每步會在物體可到達的範圍內收集障礙；若物體離開該範圍，應停止或重新收集，不要讓它進入未檢查空間。

只需要接觸、不需要浮力時，可直接用 `ContactSolver`：

```java
body = RigidBodyDynamics.kick(body, properties, gravityWrench, dt);
ContactSolver.Result contact = ContactSolver.solve(body, properties,
        ContactSolver.generate(body, properties, shape, obstacles, dt), material, dt);
body = RigidBodyDynamics.drift(contact.state(), properties, dt);
body = ContactSolver.separate(body, properties, shape, obstacles); // 只修位置／姿態，不改速度
```

`ContactSolver.Contact.offset` 是從重心到物體上最近點的世界向量，`normal` 由障礙指向物體，`separationM` 為負表示穿入。
`ContactSolver.SLOP`（0.005 m）以內的穿入不修正；接近速度低於 `RESTITUTION_THRESHOLD`（0.5 m/s）不反彈。

## 波浪（0.6）

```java
LinearWave wave = new LinearWave(0.2, 16, Math.toRadians(30), 3.0, 0); // 振幅 m、波長 m、方向 rad、水深 m、相位 rad
WaveField sea = new WaveField(List.of(wave));
double eta = sea.elevation(x, z, timeSeconds);                       // 相對靜水面的高度 m
Vec3d orbital = sea.velocity(point, stillWaterY, timeSeconds);       // m/s，加到水流
FloatingBodyDynamics.Environment env = TestPlatform.wavyWater(stillWaterY, sea, timeSeconds);
```

水深可用 `Double.POSITIVE_INFINITY` 代表深水。H/L > 0.1 或 H > 0.78h 會被拒絕。
時間相依環境需在每個小步用該步開始時間重新建立；`wavyWater` 以浮筒球心處的局部水面平面切割球體。

## 螺旋槳（0.6）

```java
PropellerModel model = new BladeElementPropeller(3, 0.2, 0.25, 1.0, BladeElementPropeller.THIN_SECTION, 24)
        .tabulate(361);                                   // 葉數、轂比、c/D、P/D、剖面、環數；建表一次
Propeller propeller = new Propeller(new Vec3d(-1.4, -0.4, 0), Vec3d.X, 0.3, model); // 局部位置、推力軸、直徑 m
PropellerSolver.Load load = PropellerSolver.load(body, properties, propeller, 15, water); // 15 轉/秒
List<AppliedForce> external = List.of(load.force());     // 推力與軸反扭矩，交給 FloatingBodyDynamics.step
double thrustN = load.thrustN(), shaftPowerW = load.shaftPowerW(), advanceRatio = load.advanceRatio();
```

正轉速繞推力軸右手旋轉並沿軸推進；`load.force().couple()` 是作用在物體上的 −Q·axis。
`PropellerTable` 可載入量測或文獻的四象限 C_T*、C_Q*（依 β 嚴格遞增），超出範圍報錯。
`BladeElementPropeller.sample` 每次都重解 BEMT（約 0.3 ms），遊戲中請先 `tabulate`。
跨水面的槳需由呼叫者決定介質；平台示例以槳盤浸水比例混合水／空氣負載，不模擬通氣。

## 阻力耗散與摩擦生熱（0.6）

```java
FloatingBodyDynamics.DragDissipation d = step.dissipation();      // W，空氣／水分開
ThermalFluidState air = Thermodynamics.dryAir(293.15, 101325, wind);
double heatW = HeatTransfer.frictionHeatingW(airConductanceWPerK, air, d.airMeanSquareSpeed(),
        HeatTransfer.turbulentRecoveryFactor(air), d.airW());     // 不超過 d.airW()
thermalBody = HeatTransfer.reservoirs(thermalBody, reservoirs, heatW, dt).body(); // HeatTransfer.Body
```

`dissipation` 是本步阻力移除的機械功率，與參考系的等速平移無關。只把 `frictionHeatingW` 當作傳入物體的部分；
不要再把整個 `d.totalW()` 加到物體，否則會重複計算本該留在流體中的熱。

## 0.7 相容性變更

- `FloatingBodyDynamics.step` 對濕浮筒**預設加入附加質量**（`ADDED_MASS_COEFFICIENT` = 0.5，以幾何浸水體積計）；
  靜水起伏週期會比 0.6 長。乾燥的物體結果與 0.6 相同。
- `FloatingBodyDynamics.Result` 增加 `inertia`（附加質量反作用、水平 Froude–Krylov 力，以及轉動時與夾帶水交換的動量）與
  `contactFrictionW`；動量收支改為 `hydrostatics + averageDrag + inertia + contact + external`。欄位順序改變，請用存取方法。
- `FloatingBodyDynamics.Contact` 增加 `pressureFraction`、`pressureCentroidOffset`、`waterAcceleration`；
  原本的四參數建構式仍可用，代表穩定水（壓力等於幾何、水不加速）。
- `ContactSolver.Result` 增加 `frictionDissipationW`；`AppliedForce.Kind` 增加 `INERTIA`。
- `TestPlatform.wavyWater` 有波浪時改用 15×15 柱積分的線性波壓；`flatWater` 仍是精確球冠。

## 波壓、附加質量與自訂環境（0.7）

```java
WaveImmersion w = HydroSolver.sphereInWaves(sphereCenter, radius, sea, stillWaterY, timeSeconds, 15);
FloatingBodyDynamics.Contact contact = new FloatingBodyDynamics.Contact(
        w.fraction(), w.wetCentroidOffset(), air, water.withVelocity(w.velocity()),
        w.pressureFraction(), w.pressureCentroidOffset(), w.acceleration());
double head = sea.dynamicPressureHead(point, stillWaterY, timeSeconds); // p_d/(ρg)，m
Vec3d dudt = sea.acceleration(point, stillWaterY, timeSeconds);       // m/s²，= −g∇head
```

`pressureFraction` 是垂直壓力合力除以 ρgV：它取代幾何浸水比例計算浮力，已含波的垂直 Froude–Krylov 力；
`waterAcceleration` 用於水平 Froude–Krylov 力 ρV_wet·a_h 與附加質量 C_a ρV_wet (a − a_point)。
自訂環境若沒有波，就用四參數建構式；不要自己再加一次附加質量或 Froude–Krylov 力。

需要自行組合剛體步進時，可直接使用隱式附加質量更新：

```java
List<RigidBodyDynamics.PointMass> added = List.of(new RigidBodyDynamics.PointMass(offsetFromCom, 0.5 * 998.2 * wetVolume));
body = RigidBodyDynamics.implicitVelocity(body, properties, loadWrench, pointDrags, added, dt); // 只更新速度
```

附加質量很大時仍穩定。之後的 `drift` 若要保持「物體＋夾帶水」的轉動一致性，請直接用 `FloatingBodyDynamics.step`，它會連同附加質量一起推進。

## 接觸摩擦生熱（0.7）

```java
ContactSolver.Result contact = ContactSolver.solve(body, properties, contacts, material, dt);
double frictionW = contact.frictionDissipationW();                  // 滑動介面產生的熱功率 W
double share = HeatTransfer.contactHeatPartition(
        HeatTransfer.effusivity(45, 7850, 470),                     // 物體：k W/m/K、ρ kg/m³、c J/kg/K
        HeatTransfer.effusivity(2.9, 2700, 790));                   // 地面
thermalBody = HeatTransfer.reservoirs(thermalBody, reservoirs, share * frictionW, dt).body();
```

`frictionDissipationW` 以步末滑速計算，收斂時非負；有限迭代下可能有極小的負殘差，深度穿入的退化狀態會更大。
法向撞擊損失（恢復係數小於 1）不計入；需要時可由撞擊前後動能另行估計。

## 波浪輻射阻尼（0.8）

`FloatingBodyDynamics.step` 本身**不含**輻射力：它需要過去的速度歷史，所以由呼叫者持有 `WaveRadiation.Memory`，
每個小步前取輻射負載、步後記錄狀態（平台與測試球都這樣做）：

```java
WaveRadiation.Memory radiation = new WaveRadiation.Memory(
        floats.stream().map(BuoyantSphere::offset).toList(), dt);       // 每個物體一份；dt 必須是固定小步長
List<AppliedForce> loads = new ArrayList<>(radiation.loads(state, properties, floats, Vec3d.Y, waterDensity));
loads.addAll(otherExternalLoads);
FloatingBodyDynamics.Result step = FloatingBodyDynamics.step(state, properties, floats, environment,
        gravity, cd, loads, collision, dt);
state = step.state();
radiation.record(state, properties, floats, Vec3d.Y, step.wetFractions()); // 記錄浮筒中心垂直速度與水線面半徑
```

`loads` 回傳每個浮筒中心的垂直 `RADIATION` 力；`record` 必須每個小步呼叫一次，否則記憶時間軸會錯。
浮筒水平佈局在建立時固定（小傾角近似），水線面半徑由浸水比例自動追蹤（0.5 s 平均）。
可用 `WaveRadiation.heaveDampingPerDensity(a, ω)`、`hemisphereAddedMass(ka)`、`hemisphereDamping(ka)` 查頻域係數，
`kernel(ai, aj, d, dt, n)` 取得每單位密度的時域核函數。物體被傳送或重設時，可建立新的 `Memory` 清掉歷史。

`FloatingBodyDynamics` 的 0.5·ρ·V_wet 附加質量現在被視為高頻極限 A∞；不要再另加一份頻率相依的附加質量。

## 0.9 相容性變更

- `FloatingBodyDynamics.Contact` 多一個 `surfaceDiffraction` 欄位；舊的 4 參數與 7 參數建構式保留，預設 `false`（Morison 繞射，與 0.8 相同）。
- `AppliedForce.Kind` 新增 `LIFT`；對它做 `switch` 的程式需補上這個分支。
- `FloatingBodyDynamics.step` 與新的 `HullDynamics.step` 都等於 `prepare` 加 `finish`，單一物體的結果不變。
- `LinearWave` 新增 `slope`、`scaled`、`phaseAt`；`WaveField` 新增 `slope`、`scaled`、`shortestWavelength`。

## 浮筒的垂直繞射（0.9）

穿出水面的浮筒用半球激振力表取代垂直的 C_a ρ V a_z。呼叫者在環境取樣中把 `surfaceDiffraction` 設為 `0 < fraction < 1`，
再把 `Memory.diffractionLoads` 和輻射負載一起傳入：

```java
FloatingBodyDynamics.Environment environment = (center, radius) -> {
    ...
    return new FloatingBodyDynamics.Contact(fraction, wetCentroid, air, water, pressureFraction, pressureCentroid,
            waterAcceleration, fraction > 0 && fraction < 1);
};
List<AppliedForce> loads = new ArrayList<>(radiation.loads(state, properties, floats, Vec3d.Y, rho));
loads.addAll(radiation.diffractionLoads(state, properties, floats, rho, waves, time)); // 依上一步記錄的水線面半徑
```

兩者要一起使用：只設旗標不加負載會少掉垂直繞射，只加負載不設旗標會重複計算。
查表函式：`WaveRadiation.hemisphereExcitation(ka)`（複數 X̂）、`hemisphereFroudeKrylov(ka)`、`hemisphereDiffraction(ka)`、
`heaveDiffraction(waves, x, z, t, a, rho)`。表的範圍是 ka 0～3，深水。

## Wageningen B 系列螺旋槳（0.9）

```java
WageningenB b350 = new WageningenB(3, 0.50, 1.0);           // Z、A_E/A_0、P/D，超出 B 系列範圍會拒絕
double kt = b350.thrust(0.6), kq = b350.torque(0.6);          // Rn = 2e6
double ktFull = b350.thrust(0.6, 3e7);                        // Rn > 2e6 時加修正
PropellerModel screw = b350.fourQuadrant(new BladeElementPropeller(3, 0.2, 0.327, 1.0, section, 24),
        Math.toRadians(15), 361);                             // 前進象限用回歸式，其他象限用給定模型
Propeller propeller = new Propeller(hubLocal, Vec3d.X, 0.4, screw);
```

`TestPlatform.PROPELLER` 與 `TestHull.PROPELLER` 用的就是這樣做出的 B3-50 表。

## 細長船體（0.9）

```java
List<HullSection> sections = ...;                             // 由船尾到船首，x 遞增；凸多邊形，局部 y 上、z 右舷
sections.add(HullSection.rounded(x, halfBeam, keelY, deckY, bilgeRadius, 6));
SlenderHull hull = new SlenderHull(sections);
HullModel model = new HullModel(hull, HullModel.Type.SURFACE, massKg, FluidState.FRESH_WATER_20C, 0); // 建立時算 Michell 表，約 1 s
HullDynamics.Environment water = point -> new HullDynamics.Water(surfaceY, slope, stillY, k, velocity, acceleration, water, air);
HullDynamics.Result r = HullDynamics.step(state, properties, model, water, gravity, externalLoads, collision, dt);
```

- 環境每段取樣一次：瞬時水面高度（沒有水時給 NaN）、水面坡度、靜水面與衰減波數（靜水給 NaN 與 0）、當地水速與加速度。
  靜水可用 `HullDynamics.Water.calm(y, water, air)`。
- `step` 已含重量、浮力、阻力、升力元件、橫流阻力、附加質量與 Kirchhoff 項；推力、舵力、垂直輻射從 `externalLoads` 傳入。
- `HullDynamics.captiveLoad(...)` 回傳固定機體速度下的穩態水動力（不含附加質量反作用），可做虛擬拖曳或旋臂試驗。
- 垂直輻射：`HullDynamics.radiationPoints(hull, n)` 與 `segmentRadii(hull, n, result.waterlineBreadth())` 搭配 `WaveRadiation.Memory.loadsAt`／`recordAt`。
- `HullModel.Type.SUBMERGED` 用於完全潛航的旋轉體類船形（Hoerner 形狀因子、Lamb 縱向附加質量、SUBOFF 縮放的升力元件、沒有造波阻力）。
- 船形需在模型範圍內：閉合或近閉合的船尾、長寬比與 C_P 在 Holtrop–Mennen 迴歸附近；寬艉板、滑行與淺水不適用。

舵：`new Rudder(positionLocal, chord, span, Rudder.lowAspectPolar(chord, span)).load(state, properties, angle, water, thrustN, propellerDiameter)`
回傳 `SurfaceForces`；正角度使船首右轉，螺旋槳推力為正時以致動盤尾流加速舵的來流。

## 多物體接觸（0.9）

```java
List<ContactSolver.Body> bodies = ...;                        // 每個物體 prepare 後、接觸前的狀態
List<ContactSolver.PairContact> contacts = new ArrayList<>();
contacts.addAll(ContactSolver.tag(0, ContactSolver.generate(s0, p0, shape0, obstacles0, dt), material0));
contacts.addAll(ContactSolver.generatePair(1, bodies.get(1), shape1, 0, bodies.get(0), shape0,
        ContactSolver.combine(material1, material0), dt));
List<ContactSolver.BodyResult> results = ContactSolver.solveAll(bodies, contacts, dt);
// 每個物體再以 FloatingBodyDynamics.finish / HullDynamics.finish 完成本步；物體之間的穿入用 separateAll 投影。
```

`BodyResult.staticFrictionW` 是與方塊接觸的摩擦功率（交給呼叫者分配），`pairFrictionW` 是與其他物體摩擦功率的一半。
Minecraft 端的 `BodyWorld` 就是這樣每 tick 一起推進平台、測試船與測試球。0.22.1 起一個物體的呼叫丟例外時，`BodyWorld` 只把那個物體拿掉
（`CoupledBody.physicsFailed`），其他物體照常推進；見〈0.22.1 相容性變更〉。

## 0.10 相容性變更

- `ContactMaterial` 多一個 `yieldVelocityMps` 欄位（Thornton 速度相依恢復係數）；舊的兩參數建構式保留，代表固定恢復係數。
  求解器改用 `restitutionAt(接近速度)`；`ContactSolver.combine` 另取較小的 yield velocity。
- `FloatingBodyDynamics.prepare/step` 的 `cd` 可傳 `FloatingBodyDynamics.STANDARD_DRAG`，改用光滑球標準曲線（`SphereDrag`）；
  傳正值時行為與 0.9 相同（Stokes ＋ 固定 C_D）。
- `HullDynamics.HEAVE_CROSS_FLOW` 移除，改為 `HullModel.heaveCrossFlow()`（DNV 圓角矩形表）；乾舷風阻改用 `WINDAGE_LATERAL`／`WINDAGE_VERTICAL`。
- `Rudder.lowAspectPolar` 改用 Whicker–Fehlner 升力斜率、CD0 0.0065、失速 17.5°，同尺寸舵的升力比 0.9 小。
- `TestPlatform.PROPELLER`／`TestHull.PROPELLER` 改為 B3-65（量測四象限）；`TestPlatform.CONTACT_MATERIAL`、`TestHull.CONTACT_MATERIAL` 改為 `ContactMaterial.STEEL_ON_ROCK`。
- Minecraft 端：`WaveRegion` 由 record 改為 class（存取方法名不變，另有 `sea()`、`compiled()`）；`WorldWaterSampler` 會加入尾浪，
  物體用 `excludingWakeOf(實體 id)` 排除自己的尾浪。

## 不規則海況與快速波場（0.10）

```java
double gamma = SeaSpectrum.recommendedGamma(1.2, 6);                  // DNV-RP-C205 由 Tp/√Hs 決定 γ
WaveField sea = SeaSpectrum.synthesize(1.2, 6, gamma, Math.toRadians(30), 4, depth, 32, seed);
CompiledWaves fast = new CompiledWaves(sea);                           // 預先算好波數、頻率與深度係數
double eta = fast.elevation(x, z, t);
fast.grid(x0, z0, dx, dz, nx, nz, t, height, slopeX, slopeZ);         // 規則網格：相位遞推
fast.gridPhases(...); fast.segment(cos, sin, offset, low, high, out);  // 一個水柱的壓力頭與速度、加速度
```

`synthesize` 會拒絕太陡（2πHs/(gTp²) > 1/15）或會碎浪（1.86 Hs > 0.78 h）的海況；結果與逐點的 `WaveField` 方法相同（1e-10）。

## Kelvin 尾浪與水花（0.10）

```java
KelvinWake hull = KelvinWake.thinShip((x, depth) -> halfBreadth(x, depth), xAft, xFwd, draft, 60, 12, speed, g, 2.0, 60);
KelvinWake patch = KelvinWake.pressure(KelvinWake.rectangle(pressurePa, 0, wettedLength, beam / 2), 0, wettedLength, beam / 2,
        rho, speed, g, 2.0, 60);
double r = hull.resistance(rho);                 // Havelock：等於 Michell 波阻
KelvinWake.Pattern pattern = hull.pattern(60, 0.5);  // 船體座標的 0.5 m 網格，船艏前為 0
BowSpray bow = BowSpray.displacementBow(speed, entranceHalfAngle, draft, g);    // Noblesse 艏波
BowSpray whisker = BowSpray.planing(speed, trim, deadrise, g);                  // Savitsky 鬚狀水花
```

`KelvinWake` 是遠場定常波型（船沿 +x 等速前進），船艏前方與兩側近場不含局部擾動。Minecraft 端由 `Wakes` 管理波型快取、交叉淡入與靜水區。

## 滑行艇（0.10）

```java
PlaningHull hull = PlaningHull.fridsma(6.0, 1.2, Math.toRadians(20), 0.75, 48);
PlaningDynamics.Prepared p = PlaningDynamics.prepare(state, properties, hull, water, gravity, externalLoads, dt);
PlaningDynamics.Result r = PlaningDynamics.finish(p, afterContact, contactWrench, contactForces, frictionW, collision);
r.diagnostics().wettedLengthM(); r.diagnostics().ventilation(); r.diagnostics().waterlineBreadth();
```

`HullDynamics.Environment`／`Water` 與細長船體共用。低速垂直輻射（`WaveRadiation.Memory`）由呼叫者以外加負載提供，並隨 `ventilation` 淡出。
`TestPlaningBoat` 是對應 Fridsma 模型的測試船。模型與驗證見 [滑行艇](PLANING.md)。

## 係數資料（0.10）

- `SphereDrag.coefficient(Re)`、`SphereDrag.resistance(fluid, R, speed)`：光滑球標準阻力曲線。
- `HeatTransfer.sphereForcedNusselt`、`sphereNaturalNusselt`、`sphereConductance`：Whitaker／Churchill 球體對流。
- `ContactMaterial.thornton`、`yieldVelocity`、`contactModulus`、`STEEL_ON_ROCK`、`STEEL_ON_STEEL`。
- `HullManoeuvring.roundedRectangleDrag(L/D, R/D)`：DNV-RP-C205 Table E-1 第 4 項。
- `BSeriesFourQuadrant`：B3-65、B4-70（P/D 1.0、1.4）的 MARIN 四象限 Fourier 資料，可作為 `WageningenB.fourQuadrant` 的 `elsewhere`。

## 0.11 相容性變更

- `CompiledWaves.maxAmplitude()` 改為二階水面的上界（大於 `WaveField.maxAmplitude()`）；線性方法（`elevation`、`grid`、`segment` 等）不變，
  另有二階的 `secondOrder`、`surfaceTerms`、`surface`、`gridSecondOrder`，以及 `verticalAcceleration`、`accelerationDeviation`、`meanLevel`。
- `WorldBox` 多一個 `material` 欄位（可為 null，表示用物體自己的材質）；舊的兩參數建構式保留。
  `ContactSolver.Contact` 也多一個 `material` 欄位，`generate` 會帶上障礙物的材質，`solve` 與 `tag` 優先使用它。
- `PlaningDynamics` 的摩擦（`friction` 力矩與 `Diagnostics.frictionN`）包含鬚狀水花阻力。
- Minecraft 端：`Wakes.Design` 改為 `forcing()`、`spacing()`、`bow()`（及可選的 `whiteWaterWidth()`）；`Wakes.Source` 多 `wakeForward()`；
  `Wakes.update` 多一個前進方向參數；波型快取（`bin`、`prefetch`、`computeNow`）與 `vanillaBoat` 移除。原版船由 `RowingBoats` 處理。

## 二階水面與碎浪（0.11）

```java
CompiledWaves fast = new CompiledWaves(sea);
double eta = fast.surface(x, z, t);                    // 線性 + 二階（Choppy Wave Model）
double[] terms = new double[6];
fast.surfaceTerms(x, z, t, terms);                     // {η1, ∂η1/∂x, ∂η1/∂z, η2, ∂η2/∂x, ∂η2/∂z}；漸弱 w：η1 乘 w、η2 乘 w²
fast.gridSecondOrder(x0, z0, dx, dz, nx, nz, t, h2, s2x, s2z);
double threshold = Whitecaps.threshold(fast.accelerationDeviation(), hs);   // 碎浪的向下加速度門檻
boolean breaking = fast.verticalAcceleration(cos, offset) < -threshold;
double foam = Whitecaps.foam(previous, breaking, dt);  // 3.5 s 衰減
```

## 非定常尾浪（0.11）

```java
UnsteadyWake wake = new UnsteadyWake(128, 0.25, Double.POSITIVE_INFINITY, rho, g, 0.05);
UnsteadyWake.Forcing boat = UnsteadyWake.rectangle(pressurePa, -1, 1, 0.5);        // 也有 parabolic、submerged、thinShip、of(KelvinWake 變換)
wake.reset(centreX, centreZ, new UnsteadyWake.Load(boat, x, z, forwardX, forwardZ, speed));   // 從靜水平衡開始
wake.step(newCentreX, newCentreZ, new UnsteadyWake.Load(boat, x, z, forwardX, forwardZ, speed)); // 推進 dt，網格中心移到新位置
UnsteadyWake.Snapshot s = wake.snapshot();             // 可在任何執行緒讀取：s.sample(x, z, out) 得高度與坡度
double r = wake.waveResistance();                      // ∫ p ∂η/∂ξ（Havelock 的定義）
```

壓力變換 `Forcing` 在物體座標（ξ 向前、ζ 向左）中給出 p̂(α, β) = ∫∫ p e^{−i(αξ+βζ)}；`thinShip` 以 Michell 源的雙體壓力表示細船。
模型、數值選擇與驗證見 [海況](SEA.md)。

## 划船（0.11）

```java
List<Rowboat.Passenger> crew = List.of(new Rowboat.Passenger(62, 0, Rowboat.FLOOR + Rowboat.SEATED_HEIGHT, 5.5));
WaveRadiation.Memory memory = new WaveRadiation.Memory(points /* RADIATION_POINTS */, dt);
Rowboat.Result r = Rowboat.step(state, crew, new Rowboat.Oars(0.5, 0.5), water, ownWake /* 可為 null */, rho, dt, memory);
UnsteadyWake.Forcing f = Rowboat.wakeForcing(Rowboat.draft(Rowboat.mass(crew).massKg(), rho), rho);  // 驅動自己尾浪的船底壓力
```

`Water.sample` 回傳水面高度與坡度（不含這艘船自己的尾浪）；`ownWake` 給自己的尾浪時，以 ∫p∇η 算興波阻力，否則用準定常 Havelock 值。
參數與驗證見 [原版船](ROWBOAT.md)。

## 方塊材質（0.11）

`ContactMaterial` 新增 `STEEL_ON_WOOD`、`STEEL_ON_ICE`、`STEEL_ON_SAND`、`STEEL_ON_GRAVEL`、`STEEL_ON_SOIL`、`STEEL_ON_MUD`、`STEEL_ON_SNOW`、
`STEEL_ON_WOOL`、`STEEL_ON_GLASS` 與 `yieldVelocityFromPressure`（以硬度為 p_y）。把材質放進 `new WorldBox(min, max, material)`，求解器就逐接觸使用。
Minecraft 端的分類在 `BlockMaterials.of(BlockState)`；係數來源見 [物理模型](PHYSICS.md)〈0.11 新增模型與係數來源〉。

## 0.12 相容性變更

- `CompiledWaves`：`surfaceTerms(x, z, t, coast, out)`、`at(coast, x, z, taper)`（局部波場）、`scaled(factor)`、`modulate`、`variance`、
  `meanWavenumber`、`gridCoastal`、`gridCorrection(coast, ...)`、`cheapSecondOrderHeight`、`correctionHeight`。原有方法的結果不變，
  `secondOrder`／`surfaceTerms` 多了 Sharma–Dean 修正（浪頭較高、波群下水位下降）。
- `PlaningDynamics` 的靜水壓在艉板通氣時乘上 Garme 的艉板修正，航行俯仰角變大。
- `Rowboat.oarThrust` 的滑移速度受 `BLADE_SPEED − U` 限制；`Rowboat.step` 多一個帶 `Rowboat.Air`（10 m 風與空氣密度）的多載，舊的簽名等於無風。
- `Whitecaps.thresholdAtWind(accelerationDeviation, u10)`：已知風速時的碎浪門檻。
- Minecraft 端：`WaveRegions.Local` 多 `compiled`（局部波場，給 `WorldWaterSampler` 的新多載）與 `windAt`；`WaveRegion` 多 `survey`、`wind`、`coast()`、
  `localWaves`、`orbitalVelocity`、`windDrift`、`reach`；`WorldWaterSampler.drifting(region)` 加入風驅動的表層流；
  `PlatformEnvironment.setSea` 每 tick 帶入海域，`airAt` 與 `windAt` 含海域的風。

## 海岸（0.12）

```java
float[] depth = ...;                                    // 格點 (x0 + i·d, z0 + j·d)、索引 i·nz + j，單位 m，0 以下為陸地
CoastalWaves coast = CoastalWaves.build(x0, z0, 2, nx, nz, depth, sea);   // 128 m 海域約 0.3 s，請在背景執行緒
CompiledWaves fast = new CompiledWaves(sea);
double[] terms = new double[6];
fast.surfaceTerms(x, z, t, coast, terms);               // 海岸上的二階水面與坡度（coast 為 null 時是開放海面）
CompiledWaves local = fast.at(coast, bodyX, bodyZ, taper);   // 物體位置的局部平面波（給取樣器與頻域負載，local.field()）
fast.gridCoastal(coast, x0, z0, dx, dz, nx, nz, t, h, sx, sz, h2, s2x, s2z);   // 畫面用的格點
double b = coast.breaking(x, z);                        // 水深上限移除的波能比例（碎浪帶白沫）
```

`envelopes(x, z, re, im, reX, imX, reZ, imZ)` 回傳每個分量的包絡相量 M 與其梯度；外海或格點外 M = 1。模型與驗證見 [海況](SEA.md)〈海岸〉。

## 風（0.12）

```java
double ustar = SeaWind.frictionVelocity(u10);           // COARE 3.0 粗糙長度的中性對數律
double u2 = SeaWind.speedAt(u10, 2);                    // 2 m 高的風速
double[] hsTp = SeaWind.sea(u10, fetchM, depthM);       // JONSWAP 受風域限制的 Hs、Tp（風域無限大時為充分發展）
double drift = SeaWind.drift(u10, depthM);              // 風驅動的表層流 0.53 u*，0.1 m e-folding
Rowboat.Result r = Rowboat.step(state, crew, oars, water, ownWake, new Rowboat.Air(wind10, 1.225), rho, dt, memory);
```

係數來源與驗證見 [海況](SEA.md)〈風〉與 [原版船](ROWBOAT.md)。

## 0.13 相容性變更

- `SeaWind.sea` 另以 DNV 的海況陡度上限（2πHs/(gTp²) ≤ 1/15；短風域的年輕海浪會碎浪）與 1.86 Hs ≤ 0.78 d 限制 Hs，結果都能由 `SeaSpectrum` 合成。
- `CoastalWaves.build(..., waves, WindSea)`：風浪模式，各點依自己的直線風域成長；`CoastalWaves.fetches` 回傳每個水格點的風域。
- `CompiledWaves.withoutShorterThan(minWavelength)`：畫面用，去掉網格畫不出的短波分量（分量數不變，振幅為 0）。
- `CompiledWaves.gridCorrection` 改為逐點內積（不再每次配置陣列），結果不變。
- `Rowboat.Air` 多 `gust`（舊的兩參數建構式等於無陣風）。
- 新的 `WindGusts`：Kaimal 頻譜合成的陣風（見下）。
- Minecraft 端：`WaveRegion.Sea` 多 `windMps`、`fetchBeyondM`（風浪才有）；`WaveRegion.Wind` 多 `rainMps`、`thunderMps`（隨天氣）與 `mean(level)`；
  `WaveRegion.meanWind`／`gust`／`windAt`／`windDrift(level, ...)`／`drawnWaves`；`WaveRegions.at(level, x, z)` 回傳含該點風的 `Local`（`wind10`、`gust`）；
  `SeaSurface.waterVelocity` 多 `level` 參數；`WaveSurfaceRenderer.surfaceMask`。

## 零件化載具（0.17；0.20 起零件設計在載具 Mod）

載具 Mod 定義自己的零件設計，AeroCore 算物理：通用框架（藍圖、組裝、`VehicleDynamics`、狀態與存檔）與零件的執行時受力
（`api.vehicle.part`）。完整說明與驗證見 [零件化載具](VEHICLE_PARTS.md)。

```java
// 1. 載具 Mod 的零件設計：參數、檢查、JSON；compile() 交給 AeroCore 的執行時零件
public record MyFloat(String name, double radiusM, MassSpec mass) implements PartDesign {
    public static final String TYPE = "mymod:float";
    @Override public String type() { return TYPE; }
    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!(radiusM > 0)) { issues.add(DesignIssue.invalid("radiusM", "Radius must be positive")); }
        return issues;
    }
    @Override public Part compile() {
        DesignException.throwIfInvalid(check());
        return new SpherePart(this, radiusM, mass.resolve(t -> MassShapes.sphere(radiusM, t)));
    }
    @Override public Map<String, Object> parameters() { return Fields.object("radiusM", radiusM, "mass", mass.toJson()); }
}
PartTypes.register(MyFloat.TYPE, f -> new MyFloat(f.string("name", "float"), f.number("radiusM"), MassSpec.fromJson(f.child("mass"))));

// 2. 載具藍圖：安裝位置、控制綁定、連接
VehicleBlueprint raft = new VehicleBlueprint("raft", List.of(
        new VehicleBlueprint.Entry("left", new MyFloat("float", 0.5, MassSpec.shell(Material.STEEL, 0.004)), Mount.at(new Vec3d(0, 0, -1))),
        new VehicleBlueprint.Entry("right", new MyFloat("float", 0.5, MassSpec.shell(Material.STEEL, 0.004)), Mount.at(new Vec3d(0, 0, 1)))),
        List.of());
String json = raft.toJsonText();                         // 存檔、網路、分享；VehicleBlueprint.fromJsonText(json) 讀回

// 3. 離線（測試台）：編譯、推進
Vehicle v = raft.compile();                              // 不合物理時拋 DesignException（issues() 附路徑）
v.warmUp();                                              // 建船體、螺旋槳、引擎的表（可在背景執行緒）
VehicleState s = v.newState(Vec3d.ZERO, Orientation.IDENTITY, 0.005);
VehicleEnvironment calm = VehicleEnvironment.calm(0.3, FluidState.FRESH_WATER_20C, Atmosphere.atAltitude(0, Vec3d.ZERO).fluid());
VehicleDynamics.Result r = VehicleDynamics.step(v, s, calm, Map.of(), Vec3d.Y.scale(-9.80665), List.of(), null, 0, 0.005);
```

執行時零件（`dev.mcmod.aerocore.api.vehicle.part`，建構子收物理量與擁有它的設計）：

| 類別 | 受力 |
| --- | --- |
| `SpherePart` | 浮筒：`FloatingBodyDynamics.sphereLoads`、繞射、輻射點 |
| `HullPart` | 排水船體／潛航體：`HullDynamics.loads`；表面船體帶薄船尾浪（`wake()`） |
| `PlaningHullPart` | 滑行船體：`PlaningDynamics.loads`；壓力片尾浪 |
| `PropellerPart` | 螺旋槳：`PropellerSolver`，四象限模型由 `PropellerModels.fourQuadrant` |
| `ControlSurfacePart` | 全動控制面：`Rudder.load`，極曲線 `Rudder.lowAspectPolar` 或 `Rudder.freeEndsPolar`，在螺旋槳尾流中 |
| `BoxPart` | 方塊體：逐格浮力與投影面積阻力 |
| `MotorPart` | 電動馬達：`ElectricMotor` 包絡 |
| `MassPart` | 純質量，可帶座位 |

引擎這類有控制規則的零件，由載具 Mod 自己實作 `Part`，物理呼叫 `api.engine`（`EngineMap`、`EngineRuntime`、`Turbojet`…）。

遊戲中：

```java
// 生成（伺服器）
PartVehicle entity = VehicleRegistry.PART_VEHICLE.get().create(level);   // 或載具 Mod 自己註冊的 EntityType<PartVehicle 子類>
entity.setBlueprint(raft, origin, orientation);
level.addFreshEntity(entity);
// 操作：伺服器 entity.setControls(Map.of("throttle", 0.5))；客戶端 VehicleNetwork.send(entity, Map.of(...))（canControl 決定是否接受）
```

- 新的零件類型：實作 `PartDesign`（record、`toJson`），`compile()` 回傳 AeroCore 的執行時零件或自己的 `Part`
  （`loads(PartStep)` 寫入受力），在 Mod 建構時以 `PartTypes.register("modid:part", reader)` 註冊（讀存檔前就要登記）。
  `PartStep` 提供載具狀態、環境、輸入、轉子轉速；`partState()`／`partProperties()` 讓以零件自身座標寫的模型裝在任意位置都能用。
- 尾浪：零件的 `wake()` 回傳 `WakeSource`（`ThinShip`／`Planing`），`PartVehicle` 取第一個有尾浪的零件；尾浪的計算與畫面在 AeroCore。
- 外觀：客戶端以 `PartRenderer.register(零件或設計的類別, renderer)` 登記（`mesh` 畫靜態網格，`animate` 畫轉子與舵面），
  `PartMesh` 有方塊（可旋轉）、八角點的六面體（`hexahedron`，錐形樑）、球、任意軸的圓柱與圓錐（`cylinder`）、
  旋轉體（`lathe`：輪廓繞軸，平滑法線，輪廓點可各有顏色）、沿路徑的管子（`tube`：彎管、彈簧、皮帶，可封閉；路徑工具 `bezier`、`helix`、`circle`）、
  擠出輪廓（`extrude`：凸輪、曲柄臂）、兩列點之間的面片（`sheet`：葉片、散熱片）、船體放樣、把另一個網格旋轉平移後加入（`add`）、
  外框（`bounds`），以及以平面切開（`clipped`：保留平面一側，跨越的面沿平面切開，剖視圖用）。
  曲面每個角有自己的法線（平滑著色），也可有自己的顏色（`Quad.normals`、`Quad.colors`）。
  顏色的 alpha 小於 0xff 的面可另用半透明的渲染類型畫（`emit(..., translucent)`），世界中的載具一律不透明。
  `PartMesh.TEXTURE` 是畫零件用的白色材質。AeroCore 已登記自己的執行時零件。
- 載具 Mod 繼承 `PartVehicle` 可以改 `canControl`、`usesDefaultControls`（關掉 AeroCore 的品管用按鍵）、`interact`；
  畫面自己畫，或註冊 `PartVehicleRenderer`。
- 控制通道名稱自由（綁定決定），`PartVehicle.input(partId, role)` 在客戶端給動畫用的輸入值，`rotorAngle(rotor, partialTick)` 給轉子角度。
- 物理例外（0.22.1）：零件的 `loads`、漂移、接觸求解在某台載具上丟出 `RuntimeException` 時，只有那台停住：
  停在本子步開始時的姿態、速度與角動量為零、傳動軸停轉、零件內部狀態回到初始值，之後不再推進；
  記錄一次 ERROR（實體 ID、藍圖名稱、位置、例外與堆疊），`fault()` 回傳原因（階段＋例外），隨存檔保存。
  伺服器與其他載具照常運作。`resume()`（指令 `/aerocore vehicle resume`）讓它從靜止重新開始；原因沒排除就會再停一次。
  零件仍應保持有限、有界（例如 `SectionPolar` 從不丟例外）：停用是保護，不是正常結果。

## 0.38 相容性變更

使用者 2026-10-19 要「加入機場塔台」（hdfv 0.20：塔台、跑道燈與標線、ATIS、起降許可、世界生成的機場）。只新增：物理逐位元不變（`VehicleTest` 1e-9），
沒有選用新鍵的材質畫面逐像素不變。燈與著色見 [RENDERING.md](RENDERING.md)〈大量固定燈（0.38）〉〈選用的材質著色（0.38）〉，空氣資料見 [ATMOSPHERE.md](ATMOSPHERE.md)〈空氣資料與一點的風（0.38）〉。

- 新套件 `api.light`（純 Java）：
  - `LightSignal`（`steady`、`flash(週期, 亮, 起點)`、`sequence(週期, 序號, 數量)`（進場順閃燈：每盞每週期亮一次、序號往跑道頭）、`alternate(週期, 亮, 顏色…)`
    （機場燈標的白綠交替）；`level(t)`、`peak(從, 到)`（一幀之間有沒有亮過：短閃不會被幀率吃掉）、`colour(t, 本色)`）。只看遊戲時間，各客戶端一致。
  - `SectorColours`（依仰角或方位分色，邊界羽化；`papi(設定角, 過渡角, 紅, 白)`）。
  - `FieldLamp`（位置、軸、水平與垂直涵蓋角、仰角、光強 cd、顏色、分色、訊號、迴路、燈罩大小；`omni`、`beam`、`with`、`coverage`、`colour(向眼, t)`）。
  - `Allard`（Koschmieder 消光 3/MOR、`illuminanceLux`、`thresholdLux(背景亮度)`：log Et = −5.7 + 0.64 log B、`rangeM`；`NIGHT_BACKGROUND`、`DAY_BACKGROUND`）。
- 新類別 `api.AirData`（純 Java）：ISA 的壓力高度與密度高度、QNH／QFE 與高度表讀數、音速與 Mach、皮托衝壓（次音速等熵、超音速 Rayleigh）、CAS↔TAS、EAS、
  頂風與側風（`components(風的去向, 航向)`）、羅盤方位（北是 −Z）、風的來向（`bearingFromDeg`）。
- `WindGusts.runningExtremes(序列, 視窗)`：一段陣風的滑動平均的最大與最小（ATIS 的十分鐘最大陣風）。
- `SurfaceWind.Synoptic.prevailing()`：盛行風的去向。
- 遊戲端：
  - `probe/WorldAir`（兩端）：`at(Level, x, y, z)` 回傳 `Air`（平均風、陣風、垂直風、海拔、溫度、氣壓、密度、能見度；`wind()`），和 `PartVehicle` 感受到的相同
    （有自己的風的海域先用海域的風）；`peakGustMps(…, 視窗秒, 平均秒)`；`prevailingTowardRad(種子, 維度)`；`visibilityM(Level)`（天氣規則：晴 20 km、雨 5 km、雷雨 1.5 km）。
  - 方塊標籤 `#aerocore:smooth`（預設空）：載具 Mod 的鋪面加進去，地表粗糙度算 SMOOTH。
  - `vehicle/VehicleTracks`：`Track`（id、實體號、位置、速度、姿態、地面高、接觸、ready、擁有者、駕駛、藍圖、質量；機頭、航向、航跡、地速、垂直速度、離地高、相對空氣的速度）、
    `of(PartVehicle)`、`near(ServerLevel, 中心, 半徑)`（走 `BodyWorld`，近的在前）、`groundBelow`。
  - `PartVehicle.touching()`：上一個 tick 有沒有接觸（讀接觸統計，不加浮點運算）。
- 客戶端：
  - `client/FieldLamps`：`put(維度, 擁有者, 燈)`、`remove`、`circuit(擁有者, 迴路, 亮度)`、`count`、`clear`（登出自動清）；每幀在 `PartLate` 一批畫完，
    自己的著色器 `rendertype_aerocore_field_lamp`（加色、不吃霧、不丟低 alpha），最遠 8 km，超過遠裁切面的燈沿視線拉近並等比放大。`PartStats` 多一行 field lights。
  - `PartFinishes`：材質的選用鍵 `shading`（2：套用 `tint` 與 `glossScale`）、`normal`（切線空間法線貼圖，B 通道是凹陷）、`normalStrength`、`glass: "reflective"`、`farGlass`。
  - 零件著色器多 `Sampler3`、`PartSurface3`、`PartTint`。
  - 開發指令 `/aerocoredev lamps grid <n> <間距>|clear`。

## 0.37 相容性變更

使用者 2026-10-18 要「加入聲音」（hdfv 0.19，全部程式合成）。只新增：物理與舊的方法逐位元不變（`VehicleTest` 1e-9），詳見 [ACOUSTICS.md](ACOUSTICS.md)。

- 新套件 `api.acoustics`（純 Java）：DSP 與分析（`Voice`、`Noise`、`Filters`、`DelayLine`、`Oscillator`、`Modal`、`Mix`、`Dsp`、`Spectrum`、`Wav`），
  傳播與聽者（`Propagation`、`SourceTrack`、`SonicBoom`、`RoomAcoustics`、`Surface`、`Transmission`、`Loudness`），聲源（`PistonEngineNoise`、`TurbineNoise`、
  `RotorNoise`、`MotorNoise`、`TyreNoise`、`FlowNoise`、`WaterNoise`、`ImpactNoise`、`GasJetNoise`、`ProcessNoise`、`Horn`）。每個聲源：`Spec`、`State`、`set`、
  `soundPowerDb`，`TurbineNoise`、`RotorNoise`、`MotorNoise`、`PistonEngineNoise` 有 `spec()`。
- `ContactSolver.BodyResult` 多 `touches`（`Touch`：位置、法向、接近速度、法向衝量、滑動速度、另一物體、材料）；舊的五參數建構子保留（空清單）。
- 遊戲端：
  - `vehicle/VehicleContacts`：`Impact`、`Scrape`、`listen`、`material`；伺服器事件 `PartVehicle.Contacted`、客戶端的撞擊封包。
  - `PartVehicle.velocity()`（客戶端是同步的真實速度）、`scrape()`、`seenBy(ServerPlayer)`。
  - `vehicle/FarSounds`：`Signature`、`Far`、`provide`、`rangeM`、`heard`、`age`。
- 客戶端 `client/sound`：
  - 聲音：`RuntimeSounds`（`id`、`define`、`defined`、`undefine`）、`SynthInstance`（`streamed`、`buffered`、`at`、`volume`、`pitch`、`relative`、`finish`）、
    `LiveSound`（`streamed`、`looped`、`update`、`stop`、`reservedStreams`）。
  - 引擎的掛鉤：`SoundHooks`（`executor`、`generation`、`freeStreams`、`audible`、`subtitle`）。
  - EFX 與聽者：`Efx`（`Path`、`want`、`available`、`room`、`path`）、`Listening`（`room`、`reverb`、`scan`、`between`、`occlusion`、`airHf`）。
  - 開發指令 `/aerocoredev sound`。
- AutoShot 的 `mark <文字>`：在記錄寫一行，對時用。
- 雙端 `ChunkUnloadMixin`：關閉世界時的區塊卸載不再卡住（見 AGENTS.md）。

## 0.36 相容性變更

使用者 2026-10-18 要 hdfv 做平頭牽引車與鵝頸低底板拖車（「做鉸接拖車」）。新增兩台載具之間的鉸接；舊的方法與數字逐位元不變
（`CouplingTest.withoutJointsTheSolveAndTheSeparationAreAsBefore` 比對舊的 `solveAll`、`separateAll`），詳見 [COUPLING.md](COUPLING.md)。

- `api.vehicle.Coupling` record：`kind`（`BALL`、`FIFTH_WHEEL`）、`leaderPoint`、`followerPoint`（各自的模型座標）、`pitchAxis`（前車）、`yawAxis`（後車）、
  `pitchLimitRad`、`yawLimitRad`（PI 為不限）、`rollLimitRad`（0 鎖住、0～PI/2 有間隙、PI/2 以上自由）、`rollStiffnessNmPerRad`（無限大為剛性）、`rollDampingNms`、
  `captureM`、`forwarded`（後車照抄的控制頻道）、`collide`；`fifthWheel`、`ball`、`with…`、`geometry`（兩個姿態下的偏移、間隙、軸與三個角）、`toJson`／`fromJson`／`parse`／`write`。
- `ContactSolver`：`Joint(leader, follower, coupling)`、`JointResult`（後車在接點受的力與限位的力矩、起始的三個角與間隙）、`Solution`；
  新多載 `solveAll(bodies, contacts, joints, dt)` 與 `separateAll(bodies, contacts, joints)`。舊的兩個方法改由它們以空清單呼叫，結果逐位元相同。
- `TestTrack`：`ConvoyDriver`、`ConvoySample`、`hitchedOrigin`、`convoy`（離線的牽引車與拖車）。
- `LegPart.Telescope(strokeM, timeS)` 與十參數建構子：`retract` 讓支柱縮起（有載重也動）；`WheelCorner.loads` 多一個 `mountLiftM` 多載（0 時逐位元相同）。
- `FrameAnalysis.standardCases(frame, carried, wheels, pins, groundYM, gravity)`：`pins` 是另一台車撐住的節點（拖車主銷），像輪子一樣承重、
  煞車與轉彎時按比例承受水平力（力臂 0），不會是掉進溝的那個；沒有 pins 時與舊方法逐位元相同。
- 遊戲端：`PartVehicle.couple`／`uncouple`／`uncoupleAll`／`leader`／`followers`／`coupling`／`couplingForceN`／`articulationRad`／`drawnShift`；
  `Couplings`（`Result`、`Reason`、`Coupled`、`Uncoupled` 事件、`PATIENCE_TICKS`、`BREAK_M`）；`CoupledBody` 多五個預設方法（`leaderId`、`leaderCoupling`、`followerIds`、
  `carried`、`groupWaits`）；`BodyWorld` 把連在一起的載具當一組；`/aerocore vehicle couple|uncouple`。存檔多 `Coupling`、`Towing` 兩個標籤（沒連結時不寫）。

## 0.35.1 相容性變更

hdfv 0.16 的改裝廠發現範例轎車的溫和套件在 200 km/h 有 4.0 kN（車底通道 2.96 kN），比量產賽道車整台還多；車底對量產車資料重新校準
（見 BODY.md〈車用空氣套件〉的 0.35.1 段與驗證表）。簽名不變，沒有套件的機體逐位元不變；有 `Underbody` 的套件數字變了：

- `RoadAero.Underbody.sealing` 的意義：側裙封住「側面漏氣面積」的比例，剩下的開口 b / (b + `LEAK` L (1 − sealing))
  （0.34 是在開著與全封之間線性內插）。側裙只在輪拱之間或離地還有縫時，呼叫端要按比例給小一點（hdfv 乘側裙長度占車底的比例）。
- 新常數 `K_WHEELS` 1.0（前輪對車底流動的損失）、`WHEEL_RECOVERY` 0.4（後輪尾流後擴散器剩下的回收）、`ROUGH_RECOVERY` 0.3（粗糙車底後的擴散器）；
  `SMOOTH` 0.03 → 0.045（量產車的平整底板）、`DAM_WAKE` −0.5 → −0.3，且擋風板的分離區取代那一段的車底壓力（0.34 是疊加）。
- 典型的變化：hdfv 範例轎車的溫和套件 4.0 → 1.78 kN（車底 2.96 → 0.75 kN）；開著側面的擴散器與平整底板約少 80 %，6 cm 擋風板少一半；尾翼、唇、外掛件不變。

## 0.35 相容性變更

使用者 2026-10-18「加入所有機加工機器」（hdfv 0.17）。只在 `api.manufacture` 加新類別，既有的類別、方法與數字不變。

- `ToolLife`：`lifeS`、`speedForLifeMps`、`fit`（兩點求 n 與 C）、`Taylor` record、`maxProductionLifeS`、`minCostLifeS`；常數 `N_HSS`、`N_CARBIDE`、`N_CERAMIC`、
  `C_HSS_STEEL`…（Groover 表 23.2，C 是一分鐘壽命的速度，m/s）。
- `Grinding`：`removalRate`、`powerW`、`wheelWearM3`、`surfaceS`、`cylindricalS`、`plungeS`、`throughfeedMps`、`centrelessS`。
- `Finishing`：`prestonMps`、`prestonS`、`honingS`、`crosshatchRad`、`strokingMps`。
- `GearCutting`：`approachM`、`hobRevPerS`、`hobbingS`、`shapingS`、`shavingS`、`profileGrindingS`。
- `Broaching`：`cuttingTeeth`、`engagedTeeth`、`forceN`、`strokeS`。
- `Sawing`：`feedMps`、`powerLimitedFeedMps`、`hacksawFeedMps`、`cutS`、`kerfKg`。
- `Threading`：`tappingS`、`singlePointS`、`threadMillingS`、`inFeedRollingS`、`flatDieS`。
- `Riveting`：`lengthM`、`squeezeForceN`（用 `Forming.upsetForceN`）。
- `Edm`：`removalRate`（Weller）、`sinkS`、`electrodeWearM3`、`kerfM`、`wireFeedMps`、`wireUsedM`、`drillS`、`powerW`。
- `Ecm`：`specificRemoval`、`removalRate`、`feedMps`、`gapM`、`machineS`。
- `ThermalCutting`：`meltJPerKg`、`withOxygenJPerKg`、`speedMps`、`cutS`、`arcW`；`IRON_OXIDATION_J_PER_KG`。
- `Waterjet`：`jetMps`、`waterM3PerS`、`hydraulicW`、`abrasiveMps`、`removalRate`、`cutMps`。
- `Gases`：`Gas`（空氣、氮、氧、氬）、`chokedKgPerS`、`choked`、`cylinderKg`、`isothermalJ`。
- `AirSeparation`：成分常數、`minimumJPerKg`、`plantJPerKg`、`airKg`。

## 0.34 相容性變更

使用者 2026-10-18「加入載具調整系統 可以有自由外觀和大小」（hdfv 0.16：改裝廠，改造型、大小、空氣套件、輪圈、調校、車燈）。
沒有空氣套件的機體與不改裝的載具，物理逐位元不變。

- 改裝換車：
  - `VehicleState.refitted(Vehicle next[, ToDoubleFunction<String> densityKgPerM3])` 回傳 `Refitted(state, spilledKg)`：
    物理可以完全不同（質量、零件內部狀態、傳動、油箱）。車子停在原地：車架原點與姿態不變、速度歸零。
  - 依零件 id 帶過：內部狀態與顯示值（數量相同才帶，否則那個零件從頭開始）、轉子轉速、`supply`、還在的零件的待拋棄。
  - 油箱依 id 帶過，截在新容量內（密度由呼叫端給；不給時照 `refuel`：`Fuel.byName`，不認得的用 1000）。
    放不下的與沒了的油箱的油列在 `spilledKg`；新的油箱是空的（改裝不生油）。波浪記憶重新開始。
  - `PartVehicle.refit(VehicleBlueprint next[, densities])`：換上編譯好的新藍圖與上面的狀態，清掉故障、待處理的步驟與顯示的力，
    座位少了的乘客從後面下車，零件的表在背景重建；回傳截掉的油（null：藍圖編譯不過，什麼都沒改）。
  - 之後 `VehicleState` 加欄位時，`refitted` 也要考慮（同 `restyled`、`placed`）。
- 車用空氣套件 `body.RoadAero`（見 BODY.md〈車用空氣套件〉）：
  - 元件：`Underbody`（車底通道：擴散器、側裙封住的比例、擋風板、分流板、平整底板）、`RearWing`（倒裝的尾翼，查表）、
    `Fence`（擾流板唇）、`Bluff`（只有阻力的外掛件；`CD_TUBE`、`CD_BLUFF`、`CD_SHIELDED`）。
  - `loads(air, density, heights)`：車身座標的氣流與各點離地高度 → 力、對原點的力矩、下壓力、阻力。`warmUp()` 先建尾翼的表。
  - `BodyPart` 多一個帶 `RoadAero` 的建構子：每步從車底的前端與擴散器起點往下投地面射線，力以 `LIFT` 加上；診斷值 `kitDownforceN`、`kitDragN`。
  - `Wing.solve(speed, alpha, beta, air, inputs, inducedFactor, previous)`：誘導速度乘上一個因數（地面與端板），給表用。
  - `RoadAero.endplateFactor(h, b)`：Hoerner 的端板等效展弦比。

## 0.33.1 開發用視窗

使用者 2026-10-18「我在打lol 的時候 開minecraft 都在第2個螢幕 還有把不要搶滑鼠」。只影響開發環境（`FMLEnvironment.production` 為 false），
沒有 API 或物理變更。

- 系統屬性 `aerocore.window.monitor=<n>`（`client.DevWindow`）：遊戲視窗建立時隱藏、放在第 n 個螢幕（GLFW 的順序，1 是主螢幕）的工作區中央，
  再不取得焦點地顯示（`DevWindowMixin`：`GLFW_VISIBLE`、`GLFW_FOCUSED`、`GLFW_FOCUS_ON_SHOW` 關、`GLFW_POSITION_X/Y`，建構子結尾 `glfwShowWindow`）；
  遊戲一開始就知道自己沒有焦點（`DevWindowFocusMixin`：原版建好視窗後直接當成有焦點，世界開好時 `grabMouse` 就會鎖滑鼠）。
  有人點進視窗時才取得焦點、鎖滑鼠，和平常一樣。沒有第 n 個螢幕時開在主螢幕中央，仍不取焦點。選項是全螢幕時先以視窗建立
  （GLFW 建立全螢幕視窗一定顯示並取得焦點），之後原版的切換在視窗所在的螢幕變成全螢幕。
- AeroCore 與 hdfv 的 client 執行（`runClient`、hdfv 的 `runClient2`）帶 `-Daerocore.window.monitor=2`（Gradle 屬性 `dev_window_monitor`，預設 2，0 照原樣開），
  並在啟動前把該執行資料夾的 `config/fml.toml` 設成 `earlyWindowControl = false`：NeoForge 的早期載入視窗在任何 Mod 執行前就開在主螢幕中央並取得焦點，
  沒有設定能改。關掉後沒有 NeoForge 的載入進度畫面，載入時改由原版的載入畫面顯示。視窗被早期視窗建立時，記錄會警告。
- `AutoShot`：`monitor` 的預設是上面的螢幕（計畫寫的優先，`size` 之後照樣置中）；計畫執行中不留滑鼠：點進視窗或在有焦點時關掉畫面而鎖住的滑鼠，
  下一個 tick 就放開。步驟本來就不需要焦點（按鍵直接 `setDown`、截圖讀畫面緩衝）。

## 0.33 相容性變更

使用者 2026-10-17「底盤也要高度自由化」（hdfv 0.15：玩家設計的底盤零件、車軸自由擺放、硬點懸吊、車高與離地，車架剛性也進物理）。
沒有車架、沒有硬點表、沒有氣壓彈簧的載具，物理逐位元不變（`VehicleTest` 1e-9）。

- 車架（見 FRAME.md）：
  - `FrameTwist(stationX, twistNmPerRad)`（`between`、`chain`）：零件 `frame()` 回傳它就會扭；它的前 N 個內部狀態是各站扭角。
    一台載具最多一個，要正著裝（不能轉）。`Vehicle.flex()` 給各站在載具座標的位置與質量分配。
  - `FramePart(design, mass, mountPoints, seats, frame, issues)`：梯形大樑、管架、脊骨式底盤；`BodyPart` 多一個帶 `FrameTwist` 的建構子（承載式車身）。
  - `Part.probes()`、`probe(PartStep)`；`PartStep.corner`、`keep`／`kept`、`flexing`、`flexLift`、`flexRate`；`FrameFlex.solve`、`shape`、`at`。
  - `WheelCorner.probe`、帶 `Probe` 的 `loads`；`WheelCorner.Carry` 多了 `flexRateMps`（建構它的程式要補一個值）。
  - 設計時的結構計算 `api.frame`（`Section`、`FrameMaterial`、`Frame`、`FrameSolver`、`FrameAnalysis`、`FrameDisconnectedException`），
    承載式車身的 `BodyStructure.torsion`（`Torsion`）。`api.frame.Frame` 是梁模型，`api.vehicle.FrameTwist` 是交給物理的站點扭轉剛度。
- 懸吊：
  - `api.wheel.kinematics`：`Hardpoints`（雙 A 臂、麥花臣、多連桿、拖曳臂、整體軸；`fromSimple`）、`KinematicTable`（`of`、`at`、`hubOffset`、
    `onUpright`、`steeringAxis`、`rollCentreHeightM`）、`LinkageLockException`。
  - `Suspension` 多兩個建構子（運動學表；再加 `Levelling`），`kinematics()`、`levelling()`，`force(c, scale)`、`rate(c, scale)`、帶 scale 的 `step`。
  - `Spring` 多了第六個值 `polytropic`（舊的五個值的建構子照用，油氣支柱的 1.12）；`Spring.air`、`AIR_POLYTROPIC`、`Suspension.Levelling`。
  - `AxlePart` 有調高閥時多 `ride_height` 輸入與兩個內部狀態；`levelling()`。
  - `AxlePart.Steering` 的第三個值改為到零轉向線的有號距離（負值：後輪轉向）；正值時結果不變。
- `Clearance.of(底面點, 車輪)`：最小離地間隙與位置、接近角、離去角、每對相鄰軸之間的通過角（SAE J1100 的切胎斜坡）。

## 0.32 相容性變更

使用者 2026-10-17 選「先補 0.13 的驗證」（hdfv 0.14：組裝室推出的車不再被加滿油、多人連線測試）。只有新增；既有物理數字不變。

- 載具：
  - `VehicleState.placed(origin, q)`：同一台載具放到別處、靜止（和 `Vehicle.newState` 的擺法相同），保留車上的東西：
    軸轉速（怠速的引擎繼續轉）、內部狀態、顯示值、油箱與供油、待拋棄的零件。波浪輻射記憶重新開始（和 `separate` 一樣）。
  - `BodyShape.outlineAt(x)`、`overhang(points)`、`overhang(min, max, q, at, step)` 與 `BodyShape.Overhang`：零件放不放得進機體
    （頂、兩舷、前、後各穿出多少；截面以下算在裡面，底部是開的）。hdfv 0.14 的引擎室規則用它。
  - `PartVehicle.place(origin, q)`（伺服器）：用 `placed` 把載具放到別處；`setBlueprint` 則會把油箱加滿到設計值、零件從頭開始。
    沒有載具時回傳 false。不要在子步中呼叫。
- 開發用 `AutoShot`：
  - 第一行可以是 `server <位址>`（只接受 `localhost`、`127.0.0.1`，可加連接埠）取代 `world`：以開發玩家身分連上本機的伺服器，測多人連線。
    連上伺服器時 `cmd` 不能用（沒有自己的伺服器），改用 `client`（以管理員身分）。伺服器關掉或連線中斷、連不上時，遊戲照 `quit` 關掉；
    計畫執行中出現的暫停選單（伺服器上視窗失去焦點時）會被關掉，以免蓋住截圖。位址用 `127.0.0.1`：`localhost` 在這台電腦先解析成 IPv6 的 `::1`，
    只聽 127.0.0.1 的伺服器會拒絕連線。
  - `key <名稱> on|off|tap`：除了 forward、back 等移動鍵，也接受任何按鍵設定的名稱（例如 `key.hdfv.shift_up`）；`tap` 按下兩個 tick 並算一次點擊。

## 0.31 相容性變更

使用者 2026-10-17「加入痛車」（hdfv 0.13：玩家上傳圖片當貼紙、烤漆房重漆）。只有新增；既有物理數字不變。

- 客戶端：
  - `PartFinishes.define(id, look, texture)`、`undefine(id)`、`defined(id)`：執行時的材質（遊戲中做的貼圖），資源重載後仍在，同 id 時蓋過檔案的。
    `look.texture()` 是貼圖登記的鍵（用自己的命名空間）。拿掉後，仍用這個 id 的網格畫成白色，所以畫著時要保持定義。
  - `RuntimeTexture(NativeImage, clamp)`、`replace(image)`：mip、可選的邊緣 clamp，留著圖片供重載再上傳；換圖不必重新烘焙。
  - 零件著色器把貼圖 alpha 當光澤（0.5～1）與 `cutout` 門檻：貼紙的透明像素寫 0、不透明寫 128＋127×光澤，材質設 `cutout`（見 RENDERING.md）。
- 載具：
  - `VehicleState.restyled(next)`：同一台載具換了外觀（重漆、貼紙）的狀態，運動相關的一切照原樣接過來（剛體、軸轉速、內部狀態、顯示值、
    油箱、波浪輻射記憶、上一步的負載），之後逐位元照原本的走。質量性質、零件（id、順序、狀態與顯示值數、轉子）、軸系、油箱或浮筒點不同時丟
    `IllegalArgumentException`。
  - `WaveRadiation.Memory.copyFrom(other)`：接過同樣浮筒的記憶。
  - `PartVehicle.replaceBlueprint(blueprint)`（伺服器）：只改外觀的藍圖，保留位置與運動；不能編譯或物理不同時回傳 false、不改任何東西。
    零件的表在背景重建，建好前原地等待。不要在子步中呼叫。

## 0.30 相容性變更

使用者 2026-10-15「機器 載具 也用blender 做」（hdfv 0.12：機器與載具，完整內裝、燈、可動件、地面與水面效果）。既有物理數字不變；
新功能預設關閉。見 [RENDERING.md](RENDERING.md)、[WHEELS.md](WHEELS.md)、[WING.md](WING.md)。

- `api.mesh`：`MeshBuilder` 的 `CABIN`、`CLEAR`、`LAMP`、`NEAR`、`LAMP_ID`、`PIECE` 與筆 `cabin`、`clear`、`levels`、`lamp`、`piece`；
  `Quad.near()`、`lamp()`、`piece()`、`seeThrough()`；`lamps()`、`addLamp`；`Lamp`；`SurfaceCut`；`MeshBaker.pieces`、`hasPieces`；
  `PackedMesh.Group` 多了 `lamp`（舊的 5 參數建構子照舊）。玻璃與半透明的四邊形不再擋遮蔽射線、不算覆蓋。
- 物理：`WheelCorner.EXTRA`、`after(..., extra)`、`free(...)`；`AxlePart` 與 `LegPart` 的顯示值往後加；`LegPart.Retraction` 與新建構子
  （舊的照舊＝固定）；`RotorPart.SHOWN`（7 個顯示值）、`AirPropellerPart` 一個顯示值；`PartStep.shown(i)`、`shownCount()`、`stepped(...)`；
  `api.aero.Downwash`。
- 客戶端：
  - `PartRenderer.Drawn`、`animate(..., Drawn)`、`lamps(...)`、`registerDefault`（AeroCore 自己的零件改用它）。
  - `MeshBakery.Spec` 的 `maxVoxelM`（4 參數建構子照舊）與 `voxel(m)`；`Baked` 的 `pieces`、`lamps`、`piece(p, level)`。
  - `PartLayers.late(boolean)`、`lamps(drawn, piece, stack)`；`PartLate`、`LampRenderer`、`GroundMarks`、`RotorWash`、`SprayRenderer.emit`。
  - `finishes.json` 的 `clearcoat`、`flip`、`cutout`、`fresnel`，零件著色器的 `PartSurface` z、w 與 `PartSurface2`。

## 0.29 相容性變更

使用者 2026-10-15 要求 3D 模型更逼真（hdfv 0.11／0.12：所有引擎與機器、幾何細節、材質貼圖、光影、世界中的效果，用 Blender 做細節零件與貼圖）。
物理數字全部不變；只有繪製工具與下列新增。見 [RENDERING.md](RENDERING.md)。

- 新套件 `api.mesh`（純 Java）：`MeshBuilder<M>`（形狀、UV、筆）、`Mesh`、`Finish`、`MeshOps`、`ObjReader`、`ObjWriter`、
  `AmbientOcclusion`、`PackedMesh`、`MeshBaker`。
- `PartMesh` 改成 `MeshBuilder<PartMesh>` 的子類別：`PartMesh.Quad` 就是 `MeshBuilder.Quad`，多了 `uv`、`finish`、`tags` 三個元件
  （8 參數與 6 參數建構子照舊）；`add(other, rotation, offset)` 收任何 `MeshBuilder`；`emit(...)` 照舊以白色混凝土、UV (0, 0) 畫。
  0.28 的形狀幾何不變（`MeshCompatTest`）。
- `api.engine.Plume`（尾焰形狀與顏色）、`api.engine.Blackbody`。
- 客戶端：`PartRenderer.bake(part)`、`PartRenderer.animate(..., PartLayers, light)`（預設轉給舊的 VertexConsumer 形式）、`PartLayers`、
  `MeshBakery`、`PartFinishes`（`finishes.json`）、`FinishTexture`、`PartRenderTypes`、`PartShading`、`PartShader`、`PartLighting`、
  `PartStats`（`AutoShot` 的 `fps` 步驟印出）、`PlumeRenderer`、`RotorBlur`、開發用 `/aerocoredev parts ...`。
- `PartRenderer.glow(...)`（熱的面發光，預設 0）；`PartVehicle.partPoint`、`partDirection`（零件上一點在世界的位置）。
- `MeshBuilder.STAND_IN`（遠處的替身）；`PartLayers.gpu(boolean)`（世界中的網格走 GPU 快取 `PartGpu`）；零件著色器多了 `PartPose`、`PartNormalMat`、
  `PartLight` 三個 uniform（預設值等於舊行為）。
- `PartVehicleRenderer` 的零件網格改在背景烘焙（第一次出現晚幾幀才畫），有細節層級、每個零件取自己的光、剔除看不見的面。
- 第一個用戶端設定 `aerocore-client.toml` 的 `partShading`（AUTO／SHADER／VANILLA）。

## 0.28 相容性變更

使用者 2026-10-14 說「下一步」：衝壓、超燃衝壓、脈衝噴射，加上後燃器、渦輪衝壓、固體火箭助推器、氫燃料與極高速載具（hdfv 0.10）。
既有的數字不變（`TurbojetTest`、`TurbofanTest` 與所有既有測試照過），只有下列改動。

- 新套件：
  - `api.gasdynamics`（[GASDYNAMICS.md](GASDYNAMICS.md)）：`Thermo`、`GasState`、`PerfectGas`、`NormalShock`、`ObliqueShock`、
    `ConicalShock`、`Rayleigh`、`Fanno`、`ThermalFlow`、`QuasiOneD`、`SurfacePressure`、`AeroHeating`。
  - `api.propellant`（[ROCKET.md](ROCKET.md)）：`Propellant`、`SolidRocket`。
  - `api.storage`（[HYDROGEN.md](HYDROGEN.md)）：`Hydrogen`、`Insulation`、`PressureVessel`。
- 引擎（`api.engine`）：
  - 新類別 `SupersonicInlet`、`Ramjet`、`Reheat`、`TurboRamjet`（[RAMJET.md](RAMJET.md)）、`Scramjet`（[SCRAMJET.md](SCRAMJET.md)）、
    `Pulsejet`（[PULSEJET.md](PULSEJET.md)）。
  - `Fuel`：允許無碳燃料，新增 `Fuel.HYDROGEN`；`byName("hydrogen"/"liquid_hydrogen")`；`ignitionDelayS(T, p)`。
  - `Gas` 與 `GasPath.Mix` 實作 `Thermo`。
- 載具框架：
  - `Part.Tank` 多最後一個元件 `own`（五參數建構子照舊＝`false`）：自有油箱只供給它的零件，有自有油箱的零件只從它們取。
  - `PartStep.jettison()`：零件要求被拋棄。
  - `VehicleState.takeJettisons()`、`VehicleState.separate(next)`：剩下的部分保持位置與運動，零件狀態與油箱依 id 轉過去。
  - `VehicleState.setInternal(partId, i, value)`：遊戲在步與步之間設定零件的內部狀態（保養，例如換掉脈衝噴射磨損的閥片）；不接受非有限值。
  - `VehicleBlueprint.without(ids)` → `Split(rest, removed)`：掛在被拋零件掛載點上的零件一起走。
  - `VehicleDynamics.Prepared` 多最後一個元件 `jettisons`。
- 機體與機翼：
  - `BodyAerodynamics.zeroLiftDragAreaM2` 在臨界馬赫數（`criticalMach()`）以上加入跨音速與超音速壓力阻力
    （`supersonicPressureDragAreaM2`、`warmUp`）；次音速的數字不變。
  - `BodyPart` 回報 `noseTemperatureK`、`skinTemperatureK`。
  - `Wing` 在 Mach 1 以上淡出誘導速度、加入翼尖馬赫錐損失（`Wing.machConeShare`）；次音速不變。
  - 見 [HIGH_SPEED.md](HIGH_SPEED.md)。
- 遊戲端：
  - `PartVehicle.castOff(ids)` 與事件 `PartVehicle.Jettisoned`（被拋的零件、位置、方向、速度、油箱內容）。
  - 快速載具（`FAST_MPS` 以上）的碰撞走廊、路徑區塊票、`CoupledBody.stepsWhereLoaded`、客戶端外推。
  - 區塊沒好時載具暫停並保留速度（0.27 以前歸零）。

## 0.27 相容性變更

使用者 2026-10-14 要求石油工業要有 GUI 配置、製程條件改變收率（hdfv 0.9）。只有新增，既有的常數、方法與數字不變。

- `Conversion.fcc(riserOutletC)`（480～560 °C）、`Conversion.reformer(researchOctane)`（95～104）、`Conversion.steamCracking(feed, coilOutletC)`
  （`"naphtha"` 或 `"lpg"`，800～860 °C）：在名義點（530 °C、98、835 °C）回傳原本的 `FCC`、`REFORMER`、`NAPHTHA_CRACKER`／`PROPANE_CRACKER`；
  範圍外拋 `IllegalArgumentException`。常數 `NOMINAL_RISER_C`、`NOMINAL_OCTANE`、`NOMINAL_COIL_C` 與各範圍。
- `Distillation.heaterJPerKg(crude, flashC)`、`heaterFuelShare(crude, flashC, heatingValue, efficiency)`；原本的方法＝在 `FLASH_C`。
- `Reservoir.chokedRate(productivityIndex, depthM, oilDensity, chokeBoreM, producedM3)`（`chokeBoreM` ≤ 0＝沒有油嘴＝`naturalRate` 對 0 井口壓力）；
  `CHOKE_DISCHARGE` 0.85。
- 衝壓、超燃、脈衝噴射順延到 0.28。

## 0.26 相容性變更

使用者 2026-10-13 要求機床要電或動力、CNC 多方塊、柴油引擎、石油工業與載具加油（hdfv 0.8）。

- 新套件 `api.power`（見 [POWER.md](POWER.md)）：`ElectricMachine`、`Battery`、`Cable`、`Belt`、`Shafting`、`SteamEngine`、`HumanPower`、`SolidFuel`、`PipeFlow`、`Pump`。
- 新套件 `api.petroleum`（見 [PETROLEUM.md](PETROLEUM.md)）：`CrudeAssay`、`Distillation`、`Conversion`、`Reservoir`、`Lift`。
- 引擎（見 [ENGINE.md](ENGINE.md) 的壓縮點火一節）：
  - `Fuel.DIESEL`、`Fuel.FUEL_OIL`、`Fuel.byName`、`Fuel.cetaneNumber()`。
  - `EngineCycle.Spec` 多最後一個元件 `compressionIgnition`（`EngineCycle.CompressionIgnition(peakPressureLimitPa)`，null＝火花點火）；
    原本 23 個參數的建構子照舊（＝火花點火）。柴油的 lambda 範圍是 1～12。`EngineCycle.Result` 多 `ignitionDelayDeg`、`premixedFraction`（火花點火為 0）。
  - `Combustion.dieselIgnitionDelayDeg`、`premixedFraction`、`DieselBurn`；`Dyno.operate` 對柴油改以峰值壓力限制噴油時刻。
  - 新 `DieselMap`（轉速×增壓×負載＝當量比）與 `Turbocharger`（離心壓縮機、渦輪孔口、洩壓閥、中冷、匹配與最大增壓）。
  - `EngineMap.Engine.intakeTemperature(calibrationK, pressureRatio)`（預設回傳校準溫度）：增壓汽油引擎的進氣（中冷後）溫度。
  - `EngineRuntime.manifoldRatio` 的上限改成上游壓力對大氣的比（原本固定 1.2），增壓時歧管壓力可以超過 1.2；上游不超過大氣時結果不變。
- 載具燃油：
  - `Part.Tank(name, capacityM3, centre, fuel, initialKg)`、`Part.tanks()`（預設沒有）；`Vehicle.Tank`、`Vehicle.tanks()`。
  - `PartStep.burn(fuels, kgPerS)`、`PartStep.fuelSupply()`；`VehicleState.tankKg/tankFuel/fuelKg/fuelSupply/setTank`，JSON 多 `"tanks"`（舊存檔沒有時用設計的初始油量）。
  - 油箱裡的油是點質量加它的重量；沒有油箱的載具什麼都照舊（供油比例永遠 1）。已知誤差：接觸衝量用乾重。
  - `VehicleDynamics.Prepared` 多最後一個元件 `burns`。
  - 遊戲端 `PartVehicle.tankCount/tankKg/tankFuel/tankCapacityM3/refuel`，顯示用 JSON 多 `"f"`；
    `PartVehicle.setTank(tank, fuel, kg, densityKgPerM3)`（伺服器：設定油箱內容，`""` 清空，密度由呼叫端給，例如 hdfv 自己的燃料）。
  - `BodyPart` 多一個帶 `List<Part.Tank>` 的建構子：機體的質量不含油箱裡的油（由載具帶），排水型船身的設計水線仍算進初始油量；
    原本的建構子＝沒有油箱。
- 衝壓、超燃、脈衝噴射順延到 0.27。

## 0.25 相容性變更

使用者 2026-10-13 要求機床、CNC、焊接與組裝（hdfv 0.7）。新增套件 `api.manufacture`（見 [MANUFACTURE.md](MANUFACTURE.md)）：
`Tolerance`（ISO 286 公差等級）、`Cutting`（車削、鑽孔、依功率的粗加工、粗糙度與精加工）、`Casting`（Chvorinov、熔化熱）、
`Forming`（流動應力、鐓粗與平軋力、軋延功率、加熱透的時間）、`Joining`（焊道金屬、焊工時間、鉚釘與點焊數）、
`Laminating`（積層、固化、硫化）。衝壓、超燃、脈衝噴射順延到 0.26。

- `PartVehicle.warmsUp()`（protected，預設 true）與 `warmUp()` 改成 protected：子類別可以先不建零件的表（例如還在組裝室裡、
  幾小時後才會動的載具），之後自己呼叫 `warmUp()`。沒有覆寫的載具行為不變。

## 0.24 相容性變更

使用者 2026-10-12 要求「車子要有前驅、後驅、4WD 和前置、中置、後置」：輪子、傳動與四驅。傳動的模型與驗證見 [DRIVETRAIN.md](DRIVETRAIN.md)。

- 傳動約束（A1）：
  - `Part.Rotor` 多一個名稱（第一個欄位；原本的三、四參數建構子照舊＝沒有名稱）與 `Rotor(name, axis, inertia, driver)`；`Part.port(name)`、
    `Part.links()`（`Part.Link(a, b)`）、`Part.after(PartAfter)`、`Part.shownValues()`。
  - `Connection.Shaft` 多 `fromPort`、`toPort`（七個欄位；四、五參數的建構子照舊＝第一個轉子），`Connection.Shaft.ports(...)`；
    JSON 只在有名稱時寫 `"fromPort"`／`"toPort"`。不存在的埠、軸把一個零件 link 的兩端併進同一個軸系，都是設計錯誤。
  - `PartStep`：`row`、`gear`、`clutch`、`hold`、`freewheel`、`differential`、`biasFriction`、`viscous`、`torquePair`、`rollingForce`、`damper`、
    `show`、`carry`。新類別 `PartAfter`（步末的狀態、轉速、列扭矩、滾動力、內部狀態與顯示值）。
  - `VehicleDynamics.Prepared` 多 `drive`（`VehicleDynamics.Drive`：列扭矩、滾動力、各零件的範圍、`carry`、顯示值、起始轉速、時間、
    樞軸次數與是否退回 Gauss–Seidel）；`finish` 在接觸之後呼叫每個零件的 `after`。沒有約束列的一步數字和 0.23 完全相同。
  - `Vehicle.Installed` 多 `firstShown`；`Vehicle.shownValues()`；`VehicleState.shown(id, i)`、`shownValues()`、`setShownValues(...)`（不存檔）。
  - `RigidBodyDynamics.LineDamper` 與 `implicitVelocity(..., dampers, dt)`；原本的多載照舊（＝沒有線阻尼）。
  - 滾動力與線阻尼作用在車身的力歸在 `AppliedForce.Kind.CONTACT`。
  - `PartStep.partPoint(world)`（point 的反向）。
- 地面（A2，[GROUND.md](GROUND.md)）：新套件 `api.ground`：`Ground`（`cast`、`Hit`、`NONE`、`flat`、`plane`、`split`、`boxes`、`nearest`）、`BoxGround`（`Box`、`stairs`）。
  `VehicleEnvironment.ground()`（預設 `Ground.NONE`）、`withGround`、`land`。遊戲端 `WorldGround`；`PartVehicle` 的環境帶它。`/aerocore ground` 多一行射線結果。
- 輪子（A3、A4，[WHEELS.md](WHEELS.md)）：新套件 `api.wheel`：`Tyre`（`Spec`、`Construction`、`Tread`、`Forces`）、`TyreContact`、`Suspension`（`Type`、`Geometry`、
  `Spring`、`Damper`、`Step`）、`Brake`、`BrakeControl`、`WheelCorner`。執行時零件 `AxlePart`、`LegPart`；離線 `TestTrack`。
- 變速與四驅（A5，[DRIVETRAIN.md](DRIVETRAIN.md)）：新套件 `api.drive`：`TorqueConverter`、`Driveline`；執行時零件 `TransmissionPart`、`TransferPart`。
- 遊戲端：`PartVehicle.shown(partId, i, partialTick)`；同步資料 `DISPLAY` 多 `"s"`（顯示值，四捨五入到 1e-4）。
- 2026-10-13（版本號仍是 0.24.0）：
  - `PartStep.override(role, value)`：零件改寫自己這一步的輸入（包住別的零件時）；`input` 先看它。
  - `TransmissionPart.STATES`、`engaged(step)`、`shifting(step)`、`cvtRatio(step)`：控制規則讀變速箱狀態。
  - `Suspension.step(tyre, P, rate, c, ownRate, extraN, dt)`（懸吊自己的速度起算；原本的多載以 P 的速度起算）。
    `Damper.coefficient` 改成割線（孔口 K|ċ|，以前是切線 2K|ċ|），並迭代到這一步的速度：孔口阻尼的支柱更硬（落震行程約少兩成），線性避震器不變。

## 0.23 相容性變更

使用者 2026-10-11 要求「機倉也要高自由度 像模塊一樣」「車身 船身 也是」：機身、船身、車身由模組串成。模型、驗證與限制見 [BODY.md](BODY.md)。

- `api.body`（新套件）：
  - `SectionShape`：`Rounded`（`ellipse`）、`Chined`、`Cabin`，`outline()`、`scaled`、`raised`；`convex(points)`。
  - `BodyModule(name, lengthM, rear, fullness, stepM, sillM)`，`straight(...)`。
  - `BodyShape(nose, modules)`：`stations()`、`moduleFrontX`／`moduleRearX`、`bottomAt`／`topAt`／`widthAt`／`centreAt`、`particulars()`、
    `hull()`、`waterHull()`（開口切掉）、`planingHull()`、`hardChined()`。
  - `BodyAerodynamics(shape, extraDragAreaM2)`：`loads(velocity, omega, air)`、`zeroLiftDragAreaM2`、`slantLiftAreaM2`，以及各分項的靜態式。
  - `BodyStructure`：`masses(shape, modules)`、`analyze(...)`、`bottomPressurePa`、`plateThicknessM`。
- `api.vehicle.part.BodyPart`：機體的執行時零件（空氣負載加滑行或排水型的水動力、掛載點、座位）；診斷 `airspeedMps`、`airDragN`、`airLiftN`、`airSideN`。
  排水型時阻力係數的設計水線取機體質量，但不超過密封容積一半的水（會沉的機體也建得出來）。
- `api.vehicle.TowingTank`：`rest`、`tow`、`run`（給定靜止狀態的一趟）、`heel`、`trimDeg`；任何載具在靜水中的拖曳（阻力、空氣阻力、縱傾、升沉、
  縱傾擺幅、`lost`）與扶正力臂 GZ。
- 掛載點：
  - `Part.mountPoints()`（預設沒有）。
  - `Mount` 多 `attachPart`、`attachPoint`（六個欄位；四參數的建構子照舊＝不接）：`Mount.on(part, point)`、`on(part, point, offset)`、
    `attached()`、`detached()`、`then(inner)`、`Mount.of(position, rotation)`；JSON 多 `"attach": {"part", "point"}`（舊藍圖沒有＝不接）。
  - `Vehicle` 組裝時把接上的掛載點解成座標（宿主的位置，再掛載點，再偏移）；不存在的零件、掛載點與互相接成環是設計問題。
- 滑行：
  - `PlaningHull.of(hull, length, keelY[], chineHalfBeam[], deadrise[], steps[])`：任意站位，每站的底升角與斷階；`deadrise(i)`、`steps()`。
    `PlaningHull.fridsma` 是它的特例，Fridsma 的測試數字不變。
  - `PlaningDynamics` 有斷階（前體的艉板、後體在 Savitsky–Morabito 尾流裡）。
  - 2D+t 的兩個行為變更（稜柱船體不受影響）：一個截面隨水往後流而縮小（離水）時，法向的 dm_a 不再給吸力；
    側向的 m_22 從船艏往後保持最大值，不回收。往船艉收窄、或船艏朝下的船體，受力和 0.22 不同。
  - 船底斜升係數（2026-10-12，版本號仍是 0.23.0）：`PlaningDynamics.liftFactor(deadrise, chineWettedLengthM, beamM)`、
    `LIFT_RECOVERY`（0.13）、`MIN_LIFT_FACTOR`（0.3）。每個滑行面的法向負載（靜水壓、動力項、附加質量、法向繞射）乘這個係數，
    所有硬舷船體在滑行時的縱傾、升沉與阻力都會變（輕載、重心偏後的船縱傾高約 1°、阻力少；Fridsma 的數字在 0.35° 內）。
    鬚狀水花在舭乾時只到艉板（以前是 0）。`Loads`、`Diagnostics` 的欄位不變。
- `HullDynamics.loads(..., windage)` 與 `HullPart` 的 `windage` 參數：關掉乾舷的風阻（機體自己算空氣負載）。原本的多載照舊有風阻。
- `HullResistance.holtropFormFactor`：lcb 限在 ±6%（超出 Holtrop 的範圍時跑長度會變負）。一般船體的數字不變。
- `BodyShape.Particulars.rearSlantDeg`：從車頂後緣到車尾頂端的連線角（三廂車的等效後窗角），不再只看最後一段；
  最後一段就是斜面的車身（快背、Ahmed 車體）數字不變。
- 客戶端 `PartMesh.sheet` 的法線改用對角線的外積（以前的寫法在平的平行四邊形上抵消成零，光照錯）；面的頂點順序不變。

## 0.22.1 相容性變更

- `CoupledBody.physicsFailed(phase, error)`：物體的物理在 `beginTick`、`prepareSubstep`、接觸、`finishSubstep`、`correctPose`、`endTick`
  丟出 `RuntimeException` 時呼叫。`BodyWorld` 本 tick 不再推進它、其他物體照常；預設實作重新丟出（伺服器照舊當機），
  能安全停住的物體要覆寫：從健全的狀態停下、記錄、之後 `beginTick` 回傳 false。只有 `PartVehicle` 覆寫；
  AeroCore 的示範物體（平台、測試船、滑行艇、測試球）維持舊行為。
- 聯合接觸求解（`solveAll`）丟例外時，`BodyWorld` 讓每個物體各自只跟方塊解一次：單獨也失敗的物體停用；
  都沒失敗就是物體之間的接觸出錯，停用有互相接觸的物體。物體之間的穿入投影（`separateAll`）出錯時，停用該組物體。
- `PartVehicle.fault()`、`resume()`；NBT 多一個 `Fault` 字串（舊存檔沒有＝正常）。`/aerocore vehicle info` 顯示停用與原因，新指令 `/aerocore vehicle resume`。
- GameTest：GameTest 執行中，物體的物理失敗會讓它所屬的測試失敗（結構範圍含該物體的測試，或最近的一個；
  例外從測試自己的 tick 丟出，`GameTestInfo` 才會回報），停用不會掩蓋物理錯誤，伺服器也不會當機。
  故意讓物理失敗的測試在物體加上實體標籤 `BodyWorld.EXPECTED_FAULT_TAG`；`BodyWorld.owningTests(level, 位置)` 查某點屬於哪個測試。
  載具 Mod 改依賴 0.22.1 後，它的載具 GameTest 不用改就適用。
- 新的 mixin `GameTestTickerAccessor`（讀執行中的 GameTest）。

## 0.22 相容性變更

- `api.aero`（新套件）：
  - `Airfoil`：`naca4(camber, position, thickness)`、`cst(upper, lower, teThickness)`、`cstFit`、`of(Selig 點)`；以值比較。
  - `AirfoilAnalysis`：`analyze(alpha, reynolds)` 或 `inviscid(alpha).at(reynolds)`（同一攻角多個 Re 共用面板解）。
  - `SectionPolar implements AirfoilPolar`：`of(airfoil)`（快取）、`sample(alpha, reynolds, mach)`、`flapped(..., FlapType, chordFraction, delta)`、
    `liftSlope`、`zeroLiftAngle`、`maxLift`、`stallAngle`、`minimumDrag`、`criticalMachAt`。可當 `LiftingSurface` 的極曲線用。
  - `Wing`：`Planform`、`ControlSurface`、`solve(flow[], Air, inputs, heightM, warm)`、`solve(speed, alpha, beta, Air, inputs)`、
    `solve(speed, alpha, beta, Air, inputs, previous)`（掃攻角時從上一點的渦強度延續，收斂得和冷啟動一樣緊），
    `Solution` 有力、力矩、型阻、每條的渦強度／攻角／升力／阻力／法向載荷／失速。
  - `WingStructure.analyze(planform, rootT, tipT, Box, Materials, liftN)`。
  - `AirPropeller`：`Geometry`、`sample(beta, setting, mach, reynolds)`（四象限 `PropellerCoefficients`）、`classic(J, ...)`、`tabulate(Grid)`。
  - `HelicopterRotor`：`Geometry`、`Controls`（`fromSticks`）、`State`、`loads(...)`、`trimmed(...)`。
- `api.vehicle.part`：`WingPart`、`AirPropellerPart`（`loads(step, setting)`）、`RotorPart`（`loads(step, Controls)`）；
  hdfv 可以包裝它們加上自己的控制規則（調速器、增穩）。
- `Connection.Shaft` 多一個 `freewheel` 欄位（四參數的建構子照舊）；`Vehicle.freewheels()`；JSON 的 `"freewheel": true`。
- `PartStep.partDirection(world)`：世界向量轉到零件座標。
- `Part.Rotor` 多一個 `momentumKgM2`（每 rad/s 的角動量；三參數的建構子照舊＝慣量）：減速箱後的輸出軸慣量是 × 減速比²，
  角動量只 × 減速比。`VehicleDynamics` 的陀螺力矩與轉子加速時的反扭力改用它（`Vehicle.Rotor` 也多這個欄位）。
- `VehicleDynamics` 加陀螺力矩（−ω × H）：轉動中的轉子（引擎、螺旋槳、旋翼）在機體轉向時的反力矩；原本沒有。
  它在 `finish` 的漂移裡和轉動一起算（陀螺體），不是 `prepare` 的外力，所以顯示成慣性交換（`Result.inertia()`），不是 `THRUST`。
- `RigidBodyDynamics.drift(s, p, spin, dt)` 與 `driftWithPointMasses(s, p, added, spin, dt)`：帶內部自旋角動量 `spin`（機體座標）的
  無外力矩漂移，保持機體加轉子的世界角動量；原本的多載＝`spin` 為零，數字不變。
- `HelicopterRotor.DREES_MU_MAX`（0.5）與 `TIP_LOSS_MIN`（0.9）：Drees 入流分布的前進比上限與 Prandtl 葉尖損失因子的下限。
- 模型、驗證與限制見 [WING.md](WING.md)。

## 0.21 相容性變更

- `api.engine.Turbofan`：雙轉子渦扇的設計點（`size`）、平衡（`equilibrium`）、運轉線（`runningLines`）、準穩態（`quasiSteady`，
  給兩個轉速與燃油；`solution` 暖啟動上一個解）與控制用的反解（`lineFuel`、`lineHighSpeed`、`lineLowSpeed`、`fuelForTurbineInlet`、`fuelForSurgeMargin`）；
  `Design` 以 `Turbojet.Design` 當核心，加旁通比、風扇與增壓級壓比、風扇與低壓渦輪效率、旁通道損失、分開或混合排氣。
- `api.engine.FreeTurbine`：自由渦輪軸引擎（燃氣產生器＋動力渦輪），同一組方法；準穩態另給動力渦輪轉速，`Point` 有軸功率與軸扭矩
  （轉子卡住時扭矩是設計點的 2 倍）。減速齒輪箱由呼叫者處理（`Turbomachinery.gearboxEfficiency`、`gearboxOutputTorque`）。
- `Turbomachinery` 新增 `turbineSpeedEfficiency`、`fanStagePressureRatio`、`fanStageLoading`、`relativeTipMach`、`faceMach`、
  `gearboxEfficiency`、`gearboxOutputTorque`。
- `Turbojet` 的公開介面與結果不變（內部改用 `GasPath`；`inletRecovery` 的斜板進氣道在 Mach 5 以上改用 MIL-E-5008B 的 800/(M⁴+935)）。
- 模型、驗證與限制見 [TURBOFAN.md](TURBOFAN.md)。

## 0.20 相容性變更

使用者 2026-10-11 決定「AeroCore 是計算 API」，零件「全部搬到」載具 Mod（High Degree of Freedom Vehicles，`hdfv`）。

- **移除**：
  - 所有零件設計：0.17 的九種、0.18 的活塞引擎元件與 `PistonEngineDesign`、0.19 的噴射引擎元件與 `JetEngineDesign`。
  - `EngineComponent`、`EngineMaterial`、`SampleBlueprints`、`SampleEngines`。
  - 指令 `/aerocore vehicle samples`，以及載具 GameTest。
  - 這些都在 hdfv（類型 ID `hdfv:*`）。`PartTypes` 不再有內建類型；存檔裡 `aerocore:*` 的載具在沒有提供該類型的 Mod 時無法編譯，會原地不動（既有行為）。
- **新的執行時零件**：`api.vehicle.part` 的 `SpherePart`、`HullPart`、`PlaningHullPart`、`PropellerPart`、`ControlSurfacePart`、`BoxPart`、
  `MotorPart`、`MassPart`。受力程式從原本的設計搬來，結果不變（`VehicleTest` 1e-9 對照照舊通過）。
- **新的計算 API**（從零件設計抽出）：
  - `api.engine.PistonGeometry`：餘隙容積、壓縮比、氣門排列、燃燒室面積、火焰行程、曲軸系慣量。
  - `api.engine.EngineStrength`：活塞頂壓力上限、連桿轉速上限。
  - `api.engine.EngineMap`：MAP 建立器，標定政策當參數。
  - `api.engine.EngineRuntime`：扭力、排氣背壓、濾清器壓降、歧管充放。
  - `api.engine.Turbomachinery`：壓縮機功與出口溫度（`Turbojet` 共用）、滑移係數、級負荷、進口面積與流率、轉盤應力、冷卻葉片溫度、轉子扭力。
  - `ValveFlow.injectorFlow`、`HeatTransfer.finnedCoreRejection`、`Rudder.freeEndsPolar`。
  - `api.ElectricMotor`、`api.PropellerModels`。
- **框架開放**：
  - `DesignIssue.range`、`DesignException.throwIfInvalid`、`Material.check`／`toJson`／`fromJson`、`Fields.vec(Object, String)` 改成 public。
  - `Shapes` 改名為公開的 `MassShapes`。
  - `Part.wake()` 與 `WakeSource`。
  - 客戶端 `PartRenderer`／`PartMesh` 登記制，取代 `PartVehicleRenderer` 依設計類別的分支。
- **發佈**：`maven-publish` 輸出到 `build/repo`（`aerocore-physics`、`aerocore-neoforge-1.21.1`）。
- `Dyno.operate` 的註解改正：排氣壓力是絕對壓力。

## 0.19 相容性變更

（0.20 起，下列零件設計、範例與 `/aerocore vehicle spawn jetboat` 都移到 hdfv。）

- `api.engine.Turbojet`：單軸渦輪噴射的設計點（`size`）、非設計點平衡（`equilibrium`）、依飛行馬赫數的運轉線（`runningLines`）、
  準穩態（`quasiSteady`）與控制用的反解（`lineFuel`、`fuelForTurbineInlet`、`fuelForSurgeMargin`），可離線呼叫（測試台、設計畫面）。
  模型與驗證見 [JET.md](JET.md)。
- `Gas` 新增 `enthalpy`、`entropyFunction`、`temperatureAtEntropy`、`temperatureAtEnthalpy`；`Fuel.KEROSENE`。
- `api.vehicle` 新增噴射引擎元件 `JetInletDesign`、`CompressorDesign`、`CombustorDesign`、`TurbineDesign`、`JetNozzleDesign`、`JetControlDesign`，
  組合成零件 `JetEngineDesign`（`aerocore:jet_engine`，輸入 `throttle`、`ignition`，三個內部狀態）；`SampleEngines.turbojet()`／`modelTurbojet()`、
  `SampleBlueprints.jetBoat()`（`/aerocore vehicle spawn jetboat`）。
- `EngineMaterial` 多一個欄位 `serviceTemperatureK`（JSON `serviceTemperature`，沒寫時 600 K）：自行建構 `EngineMaterial` 的程式要補上。
  新材料 `STAINLESS_321`、`INCONEL_718`、`INCONEL_713C`、`CMSX_4`。
- 活塞引擎與其他零件的行為不變。

## 0.18 相容性變更

（0.20 起，下列引擎元件、`PistonEngineDesign`、範例都移到 hdfv。）

- 新套件 `dev.mcmod.aerocore.api.engine`：`Species`／`Gas`（NASA 多項式氣體）、`Fuel`、`CrankKinematics`、`ValveLift`、`ValveFlow`、`Combustion`
  （Woschni、Bonatesta 燃燒角、Douaud–Eyzat）、`EngineOil`、`Friction`（Patton／Sandoval）、`EngineCycle`（循環模擬）、`Dyno`（點火標定）。純 Java，可離線呼叫（測功機）。
- `api.vehicle` 新增引擎元件 `EngineComponent` 與 `EngineBlockDesign`、`CylinderHeadDesign`、`PistonDesign`、`ConnectingRodDesign`、`CrankshaftDesign`、
  `ValveDesign`、`CamshaftDesign`、`SparkPlugDesign`、`FuelInjectorDesign`、`ValvetrainDesign`、`FuelSystemDesign`、`IgnitionDesign`、`LubricationDesign`、
  `CoolingDesign`、`IntakeDesign`、`ExhaustDesign`、`EngineMaterial`，組合成零件 `PistonEngineDesign`（`aerocore:piston_engine`）；`SampleEngines`、
  `SampleBlueprints.engineBoat()`。
- 零件內部狀態：`Part.internalStates()`／`initialInternals()`、`PartStep.internal(i)`／`setInternal(i, v)`（`VehicleDynamics.finish` 時寫回）、
  `VehicleState.internal(partId, i)`，存檔 JSON 多一個 `internals` 陣列（藍圖改變、數量不同時重新開始）。
  `Vehicle.Installed` 多 `firstInternal`，`VehicleDynamics.Prepared` 多 `internals`（自行建構這兩個 record 的程式要補上）。
- `PartVehicle` 不變；新的範例 `engineboat`（`/aerocore vehicle spawn engineboat`），引擎 MAP 在背景建好前原地等待。

## 0.17 相容性變更

- 新套件 `dev.mcmod.aerocore.api.vehicle`（見上）。
- `RigidBodyDynamics.implicitVelocity(s, p, load, addedMass6x6, pointDrags, pointMasses, dt)`：通用隱式解；`driftWithPointMasses`（原
  `FloatingBodyDynamics` 的私有方法）公開。
- `FloatingBodyDynamics.sphereLoads`、`HullDynamics.loads`、`PlaningDynamics.loads`：原積分器的受力，`includeWeight` false 時不含重量，給組合物體用；
  `HullDynamics.Loads`、`PlaningDynamics.Loads` 改為公開，`HullDynamics.rotate` 公開，`HullModel.collisionShape(SlenderHull)` 靜態版。結果不變。
- `WaveRadiation.Memory.prime(waterplaneRadii)`：以目前的水線面半徑啟動記憶（格點化），不從 0 爬升；既有物體不呼叫，行為不變。
- Minecraft 端：新套件 `dev.mcmod.aerocore.neoforge.vehicle`（`PartVehicle`、`VehicleRegistry`、`VehicleNetwork`、`VehicleCommands`、`VehicleGameTests`），
  客戶端 `PartVehicleRenderer`、`VehicleControls`；`EntityLoads`、`WorldObstacles` 公開；`Wakes.thinShip(hull, particulars)`（任意細長船體的尾浪）。
- 新指令：`/aerocore vehicle samples|spawn <名稱>|control <通道> <值>|info|debug <模式>|remove`。

## 0.16 相容性變更

- 新的 `SurfaceWind`：粗糙度分級、對數風剖面（60 m 混合高度、樹冠位移與冠層內衰減）、`overWater`、亂流強度、Taylor–Lee 山丘加速、有效粗糙度、`Synoptic`（緩慢變化的綜觀風）。
- 新的 `FlightAtmosphere`：MIL-F-8785C 低空 Dryden（`lowAltitude`、頻譜、`Gusts` 時域產生器）、Wieselsberger 地面效應、Allen／Gedeon 熱氣流、坡面上升氣流。
- 新的 `SkyModel`：Preetham 晴空（`clear(T, θs)`、`xyY`、`meanLuminance`）、CIE 陰天、xyY → sRGB。
- 新的 `GroundContact`：Magic Formula（`MagicFormula`、`tyreForces`）、滾動阻力、Bekker–Wong 軟地面（`Soil`、`rigidWheel`、`shear`）、`Surface` 預設。
- Minecraft 端：
  - `WorldWind`：`reference`、`mean`、`gust`、`vertical`、`at`、`describe`，以及 `setOverride`／`override`（存檔）。
  - `WaveRegions.Local` 多 `level` 與 `time`，有了它們，`windAt` 讀全世界的風。
  - 沒有自己風的 `WaveRegion` 的 `meanWind`／`gust`／`windAt` 改回傳全世界的風。
  - `NaturalSeas`、`WaveRegion.natural()`、`WaveRegions.setNatural`／`naturalSeas()`。
  - `GroundSurfaces.of(level, pos)`。
  - 客戶端 `SkyRenderer`，以及 mixin `SkyDomeMixin`、`CloudWindMixin`、`WeatherWindMixin`。
- 新指令：`/aerocore wind`、`/aerocore wind set <m/s> <°>`、`/aerocore wind natural`、`/aerocore waves natural on|off`、`/aerocore ground`。
- `Whitecaps.thresholdAtWind` 在碎浪比例小於 10⁻¹² 時回傳無限大（0.15 開發中邊緣無風時會拋例外）。

## 0.15 相容性變更

- `WaterOptics.ripples(風速, 風向, 波譜已有的斜率變異量, 起始波數, 數量, 種子)`：網格以下的風小波紋（每列 {kx, kz, 振幅, ω, 相位}），只用於畫面；
  `rippleFrequency(k)`（毛細重力波色散）、`slopeVariances(u)`（Cox-Munk 上風／側風）、`smithLambda(cos, 變異量)`；`glitter` 多乘 Smith 遮蔽。
- `CoastalWaves.WindSea` 多 `peakPeriodS`（合成時的 Tp；三參數建構子為 NaN，改用最大分量的週期）：風浪成長在該 Tp 的陡度上限 1/15 停止。
- `Whitecaps.thresholdAtWind` 不再壓在 0.388 g 以下；碎浪比例低於 10⁻¹² 時回傳無限大。
- `CompiledWaves.grid`／`gridSecondOrder`／`gridCoastal` 跳過振幅為 0 的分量（結果不變）。
- Minecraft 端：`WaveSurfaceRenderer` 改用 shader `aerocore:rendertype_aerocore_sea`（`RegisterShadersEvent` 註冊，頂點格式 `NEW_ENTITY`，屬性打包見 `.vsh`）；
  開發用 `AutoShot`（遊戲目錄有 `aerocore-autoshot.txt` 且非正式環境時，開啟名稱以 AeroCore 開頭的世界依步驟截圖後存檔離開；0.32 起也能連上本機的伺服器）。

## 0.14 相容性變更

- 新的 `WaterOptics`（Fresnel、水下 Fresnel、Cox-Munk 閃爍、水體透射）。
- `SeaWind.growthTime(u10, fetch)`（CEM II-2-35）、`SeaWind.shelter(下風倍數, 是否樹林)`（Brandle 等 2002）。
- `CoastalWaves.kernels(x, z, k, coth)`：當地波數與 coth(kh)；`CompiledWaves` 在海岸上的 Choppy Wave Model 部分改用它們（`surfaceTerms`、`gridCoastal`、`at`）。
- `Rowboat.Oars` 多兩個相位（0～1，NaN 為推力平均）；`Rowboat.strokeFactor(phase)`。
- Minecraft 端：`WaveRegion.Sea` 多 `fetchM`；`WaveRegion.scale()`／`setScale`（`SeaState` 每 tick 更新、`WaveNetwork.ScalesPayload` 同步）；
  `WaveRegion.Survey` 多地面與樹冠高度（`barriers()`）；`WaveRegion.shelter(x, z)`。

## 陣風（0.13）

```java
WindGusts gusts = new WindGusts(seed);                 // 同一個 seed 在伺服器與客戶端得到同一陣風
double[] g = new double[2];
gusts.sample(u10, t, alongWindDistance, g);            // g[0] 順風、g[1] 側風的陣風速度，m/s（凍結紊流隨平均風漂移）
double[] sigma = gusts.deviations(u10);                // 合成陣風的標準差
```

模型與驗證見 [海況](SEA.md)〈陣風〉。
