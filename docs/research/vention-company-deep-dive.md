# Vention — Company Deep Dive (June 2026)

> Disambiguation: Vention Inc. (Montreal, vention.io), NOT Ventionteams (Ukrainian outsourcing firm).

## 1. Company state

- Founded **2016**, Montreal. **Etienne Lacroix** (CEO, ex-GE/McKinsey) + **Max Windisch** (co-founder/founding CTO; active on their forum answering MachineLogic questions). **Sept 2024 MachineMotion AI PR quotes "CTO François Giguère"** — sources conflict, verify current CTO before interview.
- **~330–360 employees**; 4,000+ factories; customers: Boeing, Lockheed Martin, L'Oréal, Tesla, Toyota, Airbus, Amazon, Google; Bloomberg (Oct 2025) adds **Hershey, 3M, Pratt & Whitney**; ~40% of revenue from $1B+ companies. ~70% US / 20% EU / 10% Canada.
- **CA$100M annual run-rate** (Dec 2025).

### Funding
| Round | Year | Amount | Investors |
|---|---|---|---|
| Seed | 2017 | $3.5M | White Star, Bolt, Real Ventures |
| A | 2019 | $13M | Bain Capital Ventures |
| B | 2020 | $30M | Georgian, Bain, White Star |
| C | 2022 | ~$95M | Georgian, Fidelity |
| **D** | **Jan 2026** | **$110M USD** | **Investissement Québec (lead), NVentures (NVIDIA), Desjardins, Fidelity** |

Total >CA$300M; valuation >US$1.2B. Series D funds: Physical-AI R&D, hardware portfolio, global expansion.

**NVIDIA = threefold**: strategic partnership (Jun 2024), MachineMotion AI on Jetson/Isaac (Sep 2024), equity via NVentures (Jan 2026).

Exec additions Nov 2024: Brendan Sterne (CPO), Sarah Webster (CMO).

### Motion-control milestones (cite these)

- **Coordinated Motion (FABTECH 2023, w/ UR)**: all 6 UR joints + Vention 7th axis synchronized for constant-TCP-speed multi-waypoint trajectories (welding pitch). JD's "constant-speed toolpath" = generalizing this beyond UR. [PR](https://www.prnewswire.com/news-releases/vention-unveils-coordination-motion-technology-at-fabtech-in-collaboration-with-universal-robots-301923595.html)
- **Developer Toolkit + Simulation Checker (Demo Day Oct 2025)**: CLI, project templates, state-machine libraries; sim checker validates gravity/collision/motion-model fidelity pre-deploy — sim parity is a live engineering theme.
- Partnership timeline: Doosan certified 2019 · FANUC direct-sale alliance May 2021 · UR Certified Solution Partner Jul 2023 · ABB GoFa Sep 2024 · Franka academic Jun 2025.

## 2. Product stack (design → program → deploy → operate, all in browser)

- **MachineScope** — requirements definition.
- **MachineBuilder** — 3D cloud CAD, 1,000+ modular parts, designs priced + orderable (e-commerce moat).
- **MachineLogic** — no-code + Python programming; cloud sim or edge controller.
- **MachineMotion AI** — 3rd-gen controller, GA May 2025. **NVIDIA Jetson Orin**; **EtherCAT master driving up to 30 daisy-chained servo drives**, 3kW; IP54 passive cooling; OTA updates; Wi-Fi + LTE. Marketed as "post-PLC".
- **MachineCloud / MachinePortal / MachineApps / MachineAnalytics / RemoteView** — deploy + operate layer.
- **GRIIP™** (Feb–Mar 2026) — "Generalized Robotic Industrial Intelligence Pipeline", claims **Zero-Shot Automation™** (new unstructured task in minutes, no training).
- **Rapid Operator AI** (NVIDIA GTC 2026) — turnkey deep-bin-picking on GRIIP, claims 99% first-pick success.
- Hardware apps: Rapid Series Palletizer (Jul 2025), 3rd-gen end-of-line packaging (Feb 2026), conveyor ecosystem (May 2026).

### Robots supported
Universal Robots, FANUC, ABB (GoFa), Doosan, Epson, Kinova.

## 3. THE technical detail for this interview (NVIDIA dev blog)

MachineMotion AI runs three layers entirely at edge on Jetson Orin:
1. **Perception**: Isaac **FoundationPose** (6-DoF pose) + **FoundationStereo** (depth from cheap RGB-D).
2. **Planning**: **cuMotion** — GPU-accelerated collision-free trajectory generation + IK.
3. **Actuation**: real-time trajectory streaming to arms + EtherCAT servo drives.

→ The motion-control role almost certainly sits at layer 3 + the boundary with cuMotion. Interview gold: "How do you reconcile cuMotion's GPU planning latency with the EtherCAT real-time cycle? Where does jerk-limited re-timing happen?" Also: GPU workloads colocated with RT motion on one SoC = the hard systems problem (cache/memory contention vs cyclictest numbers).

## 4. Competitors

| Company | What | vs Vention |
|---|---|---|
| **RobCo** (Munich) | Closest analog. Modular robot kits + software-defined automation, own arms, RaaS pricing. $100M Series C Jan 2026 (Lightspeed, Sequoia). Acquired Rapid Robotics Sep 2025. | Builds own arms + leasing; Vention robot-agnostic + e-commerce. |
| **Standard Bots** (NY) | AI-native US-made arms, teach-by-demo, ~30% cheaper. $200M Series C at $1B (Jun 2026). | Vertically-integrated arm OEM vs Vention full-cell platform. |
| **Wandelbots** (Dresden) | NOVA robot-agnostic OS/API layer + cloud. | Software layer only, sells to developers; Vention replaces the controller outright. |
| **Bright Machines** (SF) | Software-defined microfactories, now server-assembly for hyperscalers. $126M C (BlackRock, NVIDIA, Microsoft). | Enterprise verticalized; NVIDIA invested in both. |
| **Tulip** (Boston) | Frontline ops/MES apps, unicorn Jan 2026 (Mitsubishi Electric). | Adjacent — digitizes humans, no motion. |
| **Intrinsic** (Alphabet→Google Feb 2026) | Flowstate platform, top-tier vision research, Foxconn JV. | Upstream platform for integrators; most credible physical-AI rival. |
| **Flexxbotics** (Boston) | Robot↔CNC connectivity for machine tending. | Niche overlap only. |
| Mecademic / Robotiq / Kinova (QC) | Micro-arms / grippers / cobot arms. | Partners-suppliers, not competitors (Kinova in Vention parts library). |

## 5. Differentiation summary (interview sound bites)

1. **Only full-stack closed loop**: CAD → code → controller → cloud ops; the MachineBuilder digital twin is the single source of truth for kinematics, sim, deployment.
2. **Post-PLC thesis**: one Jetson-based EtherCAT master replaces PLC + robot controller + IPC + vision PC.
3. **NVIDIA depth, not logo**: FoundationPose/FoundationStereo/cuMotion run ON the controller, robot-agnostic.
4. **2026 narrative = GRIIP/Zero-Shot**: productized bin picking; probe sim-to-real, cycle-time SLAs.
5. **E-commerce economics** vs integrator quotes: days-not-months.

## 6. Open questions to probe in interview

- Real-time guarantees on Jetson/Linux vs IEC 61131 PLC ecosystems?
- Safety cert path for AI-driven motion (ISO 10218 / 13849)?
- Where does MachineLogic no-code degrade into Python, and who owns that boundary?
- Berlin engineering vs sales mandate?
- How does cuMotion output get re-timed/streamed to UR/Fanuc at their cycle rates?

Sources: [BetaKit Series D](https://betakit.com/montreal-industrial-ai-scaleup-vention-raises-110-million-usd-series-d/) · [Globe and Mail](https://www.theglobeandmail.com/business/article-vention-funding-round-investissement-quebec/) · [The Robot Report](https://www.therobotreport.com/vention-raises-110m-to-accelerate-physical-ai-deployments-in-manufacturing/) · [NVIDIA dev blog (cuMotion/MachineMotion AI)](https://developer.nvidia.com/blog/making-industrial-robots-more-nimble-with-nvidia-isaac-manipulator-and-vention-machinemotion-ai/) · [MachineMotion AI datasheet](https://docs.vention.io/docs/machinemotion-ai-controller-datasheet) · [Rapid Operator AI PR](https://www.prnewswire.com/news-releases/vention-debuts-rapid-operator-ai-at-nvidia-gtc-2026-delivering-a-productized-physical-ai-solution-for-autonomous-bin-picking-302715026.html) · [GRIIP launch](https://roboticsandautomationnews.com/2026/03/11/vention-launches-generalized-physical-ai-pipeline-for-manufacturing-automation/99476/) · [Wikipedia](https://en.wikipedia.org/wiki/Vention) · [Vention press](https://vention.io/press)
