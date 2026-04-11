

<p align="center">
  <img src="https://github.com/GyrodosRobotics/26Doc57-GyrodosRobotics-Repo/raw/9fe3f917eb2502e8cde8e8454340d48af9c0934f/docs/gyrodos-fulllogo.png" alt="Gyrodos Robotics Logo" width=300"/>
</p>


<h1 align="center">Gyrodos Robotics</h1>
<h3 align="center">La Salle College, Hong Kong • Est. 2024</h3>

<p align="center">
  <a href="https://sites.google.com/view/gyrodosrobotics">
    <img src="https://img.shields.io/badge/Website-Gyrodos%20Robotics-blue"/>
  </a>
  <a href="https://www.instagram.com/gyrodosrobotics">
    <img src="https://img.shields.io/badge/Instagram-@gyrodosrobotics-purple"/>
  </a>
  <img src="https://img.shields.io/badge/MATE%20ROV-2026%20Hong%20Kong%20Regional-red"/>
  <img src="https://img.shields.io/badge/License-GPL%203.0-green"/>
</p>

---

## About

Gyrodos Robotics is a Hong Kong-based underwater robotics team 
from La Salle College, competing in the MATE ROV Competition. 
Formed in 2024, we are composed of 24 members from Year 7 to 
Year 11 devoted to developing innovative solutions to ocean 
preservation challenges and contributing to the UN Sustainable 
Development Goals 13 and 14.

This repository contains the complete technical output of our 
2026 competition season, including mechanical designs, software systems, 
documentation and internal records that are released openly to support 
the underwater robotics community.

---

## Our Vehicles

### ROV Palkia (26Mec34)
Third-generation Gyrodos-class ROV. CNC-machined 6061T5 
aluminium frame, dual two-axis Dragon Claw grippers, 
8-camera Compound Eye array with stereo vision and YOLOv11 
invasive species detection, fully_vectored8 thruster layout 
with pseudo-inverse mixing matrix, and the LUCARIO software 
stack over RS422.

### Float Magikarp (26Mag42)
Vertical Profiling Float for below-ice environment monitoring. 
500cc syringe buoyancy engine, PID depth control, Bluetooth 
data transmission, and magnetic reed switch power system.
Built in one month alongside Palkia.

---

## Highlighted Systems

### LUCARIO (267X)
**Logistical Underwater Control and Relay Interface Operator**

Our primary software stack for ROV command, control and monitoring.
Split into independent Core and GUI processes communicating over ZMQ,
with RS422 serial connection to the Arduino Mega at 115200 baud.
Features a precomputed 6×8 pseudo-inverse thruster mixing matrix 
for full 6-DOF control, PID autostabilisation, and the False Swipe 
power management system.

→ [View LUCARIO Code](https://github.com/GyrodosRobotics/26Doc57-GyrodosRobotics-Repo/tree/main/Gyrodos%20G2.2/267X-LUCARIO)

### Dragon Claw (263X)
Dual two-axis gripper system with left and right claws using 
different drive modes to imitate arm and wrist movement. 
60kg IP68 waterproof servo and NEMA 17 stepper with worm gear 
adaptor. Compatible with the Grass Grip friction booster.

→ [View CAD Files](https://github.com/GyrodosRobotics/26Doc57-GyrodosRobotics-Repo/tree/main/Gyrodos%20G2.2/223X-Gyrodos%20ROV/26Mec34-Palkia)

### Grass Grip (223X)
Passive biomimetic extension that distributes clamping force across 
hundreds of contact points for improved grip on irregular objects.
Inspired by 3D printed grass and the principles behind SpiRob 
soft robotics research. Clips onto Dragon Claw in 15 seconds 
without tools.

→ [View STL Files](https://github.com/GyrodosRobotics/26Doc57-GyrodosRobotics-Repo/tree/main/Gyrodos%20G2.2/223X-Gyrodos%20ROV/26Mec34-Palkia)

### Crab Detection (261X)
YOLOv11 model trained on 419 annotated images to identify 
invasive European Green Crab against native Rock and Jonah Crab 
species. mAP50 of 97.4%. Includes undistortion pipeline for 
wide-angle AHD cameras.

→ [View Dedicated Repository](https://github.com/GyrodosRobotics/gyrodos-crab-detection)

---

## Competition Results

| Year | Competition | Result |
|---|---|---|
| 2025 | MATE ROV Hong Kong Regional | 9th |
| 2026 | MATE ROV Hong Kong Regional | TBD |

---

## Documentation

All documents submitted to the 2026 MATE ROV Competition 
are available in `265X-Documentation/`:

- **Technical Documentation** — 25-page full system description
- **Company Spec Sheet** — vehicle specifications
- **JSEA** — Job Safety and Environmental Analysis
- **SID** — System Integration Diagrams for ROV and Float
- **Float Magikarp Specs** — vertical profiling float documentation
- **Company Safety Review** — pre-competition safety verification

---

## Open Source Contributions

| Repository | Description |
|---|---|
| [gyrodos-crab-detection](https://github.com/GyrodosRobotics/gyrodos-crab-detection) | YOLOv11 invasive crab detection model and dataset |
| [This repository](https://github.com/GyrodosRobotics/26Doc57-GyrodosRobotics-Repo) | Complete 2026 season mechanical, software and documentation files |

---

## Team

**C-Suite:**
Keanu Lam (CEO) · Gordon Peng (CTO) · Yiming Huang (COO) · 
Jason Lo (CIO) · David Wang (CQO) · Nixon Li (CMO) · 
Mitchell Au (HR) · Henry Lau (Club Chairman)

**Mentors:** Rex · Jimmy · Cloof · Max

**Teacher Advisor:** Mr Terry Lam

**Full team roster:** See Technical Documentation p.4

---

## Acknowledgements

- Leaplight Limited — CNC machining of aluminium frame
- Makerholic Limited — 3D printing services
- Sparks Solutions — Arduino and relay components
- Autodesk Inc. — Fusion 360 Student licence
- La Salle College — laser cutting equipment and pool access
- MATE Center, MATE II and IET Hong Kong — organising the 
  2026 Hong Kong Regional

---

## Licence

Code: [MIT Licence](LICENSE)  
Dataset and model weights: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)  
Documentation: All rights reserved, Gyrodos Robotics 2026

---

<p align="center">
  <em>Gyrodos Robotics • La Salle College • Hong Kong • Est. 2024</em>
</p>
