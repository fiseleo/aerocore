## 0.43.0 (beta): heavy machinery

For Minecraft 1.21.1, NeoForge 21.1.252+. The prerequisite of High Degree of Freedom Vehicles 0.26.

- Joints inside a vehicle: booms, sticks, slewing rings, centre hinges, telescopes, masts and dump bodies turn or slide
  what is mounted on them. The vehicle stays one rigid body whose mass, collision, seats, decks and holds follow the
  joints; slewing the upper turns the undercarriage back; an actuator gives way beyond its reach.
- Hydraulics: pumps (fixed, pressure-compensated, power-limited, with speed sensing) loading the engine; rams, slew
  drives, motors and winches sharing the flow.
- Tracks and roller drums: traction from the soil's shear, skid steering, sinkage and compaction resistance; smooth and
  padfoot drums with their vibration.
- Tools and terrain: blades, buckets, teeth, milling drums and cutterheads cut the ground cell by cell; the cut volumes
  go to the vehicle mod, which edits the world.
- Ropes: a winch reels a hook block on a rope over a boom's sheaves, or under a tower crane's trolley.
- Calculations: soil and rock cutting, bucket and blade capacities, disc cutters, piling, compaction, hot mix cooling,
  concrete maturity, wire ropes, reeving, outrigger stability and load charts, hydraulic and track noise.
- A vehicle without joints, tools, hydraulics or ropes steps exactly as in 0.42; saves from 0.42 load unchanged.

---

- 載具內的關節：動臂、斗桿、迴轉環、中央鉸接、伸縮臂、門架與車斗帶著裝在上面的零件轉動或滑動。載具仍是一個剛體，質量、碰撞、座位、甲板與貨艙跟著關節走；上部迴轉時底盤反轉；致動器出力不夠時會被壓回。
- 液壓：泵（定量、恆壓、恆功率，會依轉速減少排量）吃引擎動力；油缸、迴轉馬達、行走馬達與捲揚機分用流量。
- 履帶與壓路機鋼輪：由土壤剪力產生牽引、滑移轉向、下陷與壓實阻力；光面與羊腳鋼輪及其振動。
- 工具與地形：推土鏟、鏟斗、斗齒、刨刀鼓與刀盤逐格切削地面，切下的體積交給載具 Mod 改世界。
- 鋼索：捲揚機以鋼索經吊臂頂的滑輪吊起吊鉤，或掛在塔吊的變幅小車下。
- 計算：土與岩石的切削、鏟斗與推土鏟容量、盤形滾刀、打樁、壓實、熱拌瀝青冷卻、混凝土成熟度、鋼索、穿繞、支腿穩定與起重量表、液壓與履帶的聲音。
- 沒有關節、工具、液壓與鋼索的載具和 0.42 完全相同；0.42 的存檔照讀。
