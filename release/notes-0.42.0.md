## 0.42.0 (beta): wear, damage and cargo

For Minecraft 1.21.1, NeoForge 21.1.252+. The prerequisite of High Degree of Freedom Vehicles 0.25.

- Part conditions: what wears out or gets damaged, kept per part from new to failed, saved and synced; the runtime
  parts (axles, landing gear, gearboxes, wings, propellers, rotors, motors, bodies, frames) can wear and lose
  performance, off unless the vehicle mod asks.
- Damage calculations: impact energy, lining and tread wear, running life, creep, fatigue, corrosion, occupants' risk,
  flooding.
- Cargo holds carry a mass the game sets; attacks and explosions reach the vehicle mod as an event; sloped blocks give
  wheels a real slope.
- A vehicle without conditions or cargo steps exactly as in 0.41; saves from 0.41 load unchanged.

---

- 零件的狀況：會磨損、會損壞的項目，每個零件從全新到失效，存檔並同步；執行時零件（車軸、起落架、變速箱、機翼、螺旋槳、旋翼、電動機、機體、車架）可以磨損並降低性能（載具 Mod 要求才開）。
- 損壞的計算：撞擊能量、來令片與胎面磨耗、運轉壽命、潛變、疲勞、腐蝕、乘員風險、進水。
- 貨艙的質量由遊戲設定；攻擊與爆炸以事件交給載具 Mod；斜面方塊對輪子是真的斜面。
- 沒有狀況項目與貨的載具和 0.41 完全相同；0.41 的存檔照讀。
