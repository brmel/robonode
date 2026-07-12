# Vention Application Product Specs (June 2026)

Concrete numbers to drop in interview — shows you studied THEIR machines, not just theory.

## Rapid Series Palletizer (flagship, 3rd-gen Feb 2026)

- Cobot config: **30 kg payload, 13 picks/min, 136 in (3.46 m) max pallet height** (claimed category-highest, via telescopic riser). Robots: **UR10e, UR20, FANUC CRX-25iA**, fixed or telescopic base. Single/dual/triple picks, 1–2 infeeds. [page](https://vention.io/cobot-palletizer) · [datasheet](https://docs.vention.io/docs/en/rapid-series-palletizer)
- Industrial variant: **up to 140 kg / 12 picks/min / 140 in** (FANUC M-710iC/45M or M-20iD/35 in older datasheet; 10 lifts/min). [page](https://vention.io/industrial-palletizer)
- Telescopic column: 870 mm stroke, 2,250 N, **±0.5 mm repeatability, 100 mm/s** (unverified detail).
- Deploy claims: palletizer in 4 weeks, 2-day install, ~6-mo payback.

**Motion-control reading**: 13 picks/min × multi-pick grippers + 3.46 m lift + telescopic axis = synchronized arm + vertical axis trajectories, throughput-bound → blending + TOPP is where cycle time lives.

## Machine Tending Kit

- 100 kg/drawer, drawer repeatability **±0.05 mm**, >8h CNC autonomy, footprint 1,185×720 mm.
- Robots: UR10e/20/30, **ABB GoFa CRB15000**, FANUC CRX-10iA(/L)/20iA/L.
- CNC handshake via **VersaBuilt Robot2CNC** kit (chuck, door, cycle start/complete, e-stop) + Flexxbotics redeployment. [datasheet](https://docs.vention.io/docs/machine-tending-hardware-kit)

## 7th Axis / Range Extender

- Actuator families: **timing belt, ball screw, belt rack, rack & pinion**; strokes <1.55 m to >3.5 m (stock designs 1.3–6 m; Acutec runs 10 m → 14.5 m custom). Robot-agnostic (FANUC/UR/Doosan/Epson).
- Speed/payload/repeatability = per-configuration, not published. First UR+-certified customizable range extender (2018). [page](https://vention.io/7th-axis)

**Motion-control reading**: rail actuator choice changes dynamics (belt compliance vs ball-screw stiffness) → trajectory limits are configuration-dependent → limits must be data, not constants. Good interview observation.

## End-of-Line Packaging (Feb 2026 launch + Interpack May 2026)

- Case erecting + conveying + case packing + sealing + palletizing, one platform. Case packer: **10 picks/min, 30 kg**, UR10e packing + UR20 palletizing pairing.
- **Conveyors: up to 20-motor daisy-chaining via MachineMotion AI**, built-in LTE, remote support <10 min.
- Full EOL system in 12 weeks (claimed 3–5× faster); 1.3-yr payback; replaces up to 10 FTE/shift. [PR](https://www.newswire.ca/news-releases/vention-introduces-one-stop-shop-end-of-line-packaging-automation-from-case-packing-to-palletizing-822289523.html)

**Motion-control reading**: 20 daisy-chained conveyor motors + arm = the multi-axis coordination + EtherCAT scale story.

## Rapid Sanding Solution (Feb 2025)

- **FANUC CRX-30iA** + **PushCorp** sanding tool (active compliance!) + laser panel measurement; 4 stations / up to 8 panels; panels 6.5×7.7 in → 36×90 in; runs on 120 V + shop air. [PR](https://www.prnewswire.com/news-releases/vention-launches-turnkey-robotic-sanding-solution-to-assist-cabinetmakers-and-other-woodworkers-302374300.html)

**Motion-control reading**: PushCorp = force control delegated to active end-effector TODAY → JD's "force control" likely = bringing it into the motion stack (robot-level hybrid force/position). Killer interview question: "RSS uses PushCorp active compliance — is the roadmap robot-level force control to widen the application envelope?"

## Other

- Welding cells (Automate 2025, no specs).
- AI bin picking: ABB GoFa + MachineMotion AI demo, sub-mm claim; Rapid Operator AI (GTC 2026) = productization, one unnamed deployment.

## Cheat line for interview

"Your portfolio is throughput-bound pick-place (palletizer 13/min, case packer 10/min), long-rail coordinated motion (7th axis to 10 m+), and contact tasks via PushCorp — so the motion stack's money problems are blending/time-optimality, redundant-axis sync, and moving force control from the tool into the controller. Which of those does this role own first?"
