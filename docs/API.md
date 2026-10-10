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

開發期間可將遊戲用 JAR 放在載具專案的 `libs/`，在其 Gradle 相依設定加入：

```groovy
dependencies {
    implementation files('libs/aerocore-neoforge-1.21.1-0.8.0.jar')
}
```

在載具 Mod 的 `META-INF/neoforge.mods.toml` 宣告依賴；將 `your_vehicle_mod` 改成實際 Mod ID：

```toml
[[dependencies.your_vehicle_mod]]
modId="aerocore"
type="required"
versionRange="[0.8.0,0.9.0)"
ordering="AFTER"
side="BOTH"
```

玩家安裝 AeroCore 的遊戲用 JAR 和載具 Mod。不要再將 physics JAR 額外打包到載具 Mod，
以免出現重複類別。API 仍在 0.x 階段，後續次版本可能調整接口。

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
Minecraft 端的 `BodyWorld` 就是這樣每 tick 一起推進平台、測試船與測試球。

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
