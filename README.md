# 🛰️ SLARS-BCG-1
### SLARS — Bandaranayake College Gampaha — Unit 1

> *"From orbit, it observes floods, coastline shifts, and vegetation stress, turning broad environmental signs into knowledge that communities can use."*

[![Mission Status](https://img.shields.io/badge/Mission%20Status-Design%20Complete-brightgreen)](https://github.com/SLARS-BCG-1)
[![Form Factor](https://img.shields.io/badge/Form%20Factor-2U%20CubeSat-blue)](https://github.com/SLARS-BCG-1)
[![Orbit](https://img.shields.io/badge/Orbit-LEO%20500km%2010%C2%B0-blue)](https://github.com/SLARS-BCG-1)
[![Country](https://img.shields.io/badge/Country-Sri%20Lanka%20%F0%9F%87%B1%F0%9F%87%B0-red)](https://github.com/SLARS-BCG-1)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## 🌏 Who We Are

We are a team from **Bandaranayake College, Gampaha, Sri Lanka** — and we are building our country's **first school-made satellite**.

SLARS-BCG-1 is a **2U CubeSat** designed to monitor floods, coastline erosion, and vegetation stress over Sri Lanka from Low Earth Orbit at 500 km altitude. When disaster strikes our island — cyclones, floods, tsunamis — SLARS-BCG-1 will deliver flood maps to the National Disaster Management Service Centre (NDMSC) within 35 minutes of each pass, and relay emergency messages from field teams even when every terrestrial network has failed.

This is not a simulation. This is not a concept. **The full engineering design is complete.** We are building the real satellite.

---

## 🎯 Mission Purpose

Sri Lanka is one of South Asia's most disaster-vulnerable nations. Floods destroy crops, displace families, and cut off communities from help. Our satellite was designed with one purpose:

**To protect Sri Lankan lives from space.**

From orbit, SLARS-BCG-1:
- 📡 **Observes floods** using a 5-band multispectral imager with ~25 m ground resolution
- 🌿 **Monitors vegetation stress** by computing NDVI (Normalized Difference Vegetation Index) on every pass
- 🌊 **Tracks coastline shifts** using SWIR band imaging
- 🆘 **Relays emergency messages** from disaster zones via APRS store-and-forward when local networks fail
- 🚢 **Monitors maritime traffic** using an AIS coastal vessel receiver covering the full Sri Lankan EEZ

---

## 🛰️ Mission At a Glance

| Parameter | Value |
|---|---|
| **Satellite name** | SLARS-BCG-1 |
| **Full name** | SLARS — Bandaranayake College Gampaha — Unit 1 |
| **Form factor** | 2U CubeSat (100 × 100 × 227 mm) |
| **Total mass** | ~2,525 g (of 2,660 g maximum) |
| **Orbit** | Low Earth Orbit — 500 km altitude, 10° inclination |
| **Passes over Sri Lanka** | 5–6 per day |
| **Mission life** | 2+ years nominal |
| **Deorbit** | Passive atmospheric drag, ~4 years post-launch |
| **Backup network** | SatNOGS global ground station network |
| **Team** | Students of Bandaranayake College, Gampaha |
| **Design status** | ✅ All 8 design steps complete |
| **Build status** | 🔧 In progress |

---

## 🔬 Payloads

### Payload 1 — Multispectral Imager (Primary)
- **5 spectral bands:** Blue (450–495 nm), Green (495–570 nm), Red (620–750 nm), NIR (750–900 nm), SWIR (1550–1750 nm)
- **Resolution:** ~25 m ground sample distance at 500 km
- **Swath width:** ~85 km cross-track
- **Use:** Flood mapping, NDVI vegetation stress, coastline shift detection

### Payload 2 — SDR Emergency Relay (Secondary)
- **Protocol:** APRS / AX.25 store-and-forward
- **Uplink frequency:** 145.825 MHz VHF (international APRS satellite frequency)
- **TX power:** 0.8 W
- **Use:** Relay GPS-tagged distress messages from field teams when terrestrial networks fail

### Payload 3 — AIS Coastal Receiver (Tertiary)
- **Channels:** CH87B (161.975 MHz) + CH88B (162.025 MHz)
- **Coverage:** ~2,500 km radius from 500 km altitude
- **Use:** Vessel tracking, port disruption assessment, coastline change correlation

---

## 🏗️ Design Overview

SLARS-BCG-1 was designed through 8 engineering steps, each building on the previous:

| Step | Subsystem | Key Output |
|---|---|---|
| **1** | Orbit Design | 500 km LEO, 10° incl., 5–6 passes/day over Sri Lanka |
| **2** | Power Budget (EPS) | 12 Wh Li-ion, ~2.0 W avg, GaAs body-mounted solar cells |
| **3** | Payload Stack | 3 instruments in 67 mm, 2.15 W peak combined |
| **4** | COMMS Subsystem | UHF 9.6 kbps + S-band 128 kbps + APRS relay, all links positive margin |
| **5** | ADCS | 3-axis magnetorquers , ±5° nadir pointing |
| **6** | OBC and Flight Software | ARM Cortex-M4, FreeRTOS, 7 tasks, 32 TCs, 4-level FDIR |
| **7** | Structure and Thermal | Al 6061-T6 chassis, passive thermal control, CDS Rev 14 compliant |
| **8** | Ground Segment and Ops | Colombo GS, automated data pipeline, NDMSC integration |

---
---

## 🚀 Current Status

### ✅ Completed
- [x] Full 8-step mission design (orbit → structure → operations)
- [x] All subsystem specifications locked
- [x] Complete mass budget (2,525 g / 2,660 g)
- [x] Full link budget verified — all 4 channels positive margin
- [x] OBSW architecture designed — 8 FreeRTOS tasks, 32 TC commands
- [x] CDS Rev 14 compliance verified (static requirements)
- [x] Ground station design complete
- [x] Data pipeline design complete
- [x] NDMSC integration plan finalised
- [x] GitHub repository population — uploading all design documents
- [x] OBSW development — HAL and FreeRTOS kernel (Phase 1)
- [x] Ground station software — GNURadio flowgraphs
- [x] Data pipeline — NDVI computation on Sentinel-2 proxy data

### 🔧 In Progress
- [ ] IARU frequency coordination application (437 MHz + 145 MHz)
- [ ] TRCSL S-band coordination (2.401 GHz)
- [ ] Crowdfunding campaign launch

### 📋 Upcoming
- [ ] Engineering model hardware procurement
- [ ] Preliminary Design Review (PDR) — Month 9
- [ ] Flight model assembly — Month 12–17
- [ ] TVAC and vibration qualification testing — Month 18–19
- [ ] Critical Design Review (CDR) — Month 12
- [ ] Final Acceptance Review (FAR) — Month 21
- [ ] Launch — Month 22–30

---

## 📡 Frequencies

| Channel | Frequency | Direction | Purpose |
|---|---|---|---|
| UHF downlink/uplink | 437.550 MHz | Bidirectional (TDD) | Housekeeping TM + telecommand TC |
| S-band downlink | 2.401 GHz | Sat to Ground | Science imagery (128 kbps) |
| VHF APRS uplink | 145.825 MHz | Ground to Sat | Emergency relay uplink |
| AIS receive | 161.975 + 162.025 MHz | Vessels to Sat | Maritime vessel tracking (passive) |

> Frequency coordination pending with IARU (437 MHz, 145 MHz) and TRCSL (2.4 GHz).

---

## 🌐 Ground Segment

**Primary Ground Station:** University of Moratuwa, Katubedda Campus
- Coordinates: 6.79°N, 79.90°E
- Equipment: 1.0 m S-band dish + UHF Yagi + Az-El rotator + USRP B210

**Backup Network:** SatNOGS global amateur ground station network
- 900+ stations worldwide provide automatic UHF backup coverage

**Data Products (all freely available):**
- 5-band multispectral GeoTIFF imagery
- NDVI change maps (vegetation stress and flood indicator)
- Flood extent polygons (GeoJSON — delivered to NDMSC within 35 minutes)
- AIS vessel position tracks
- APRS emergency relay messages

---

## 🤝 How You Can Help

### 💰 Support Our Crowdfunding
We are a team students building with no institutional budget. Every contribution funds real hardware for a real satellite that will protect real lives in Sri Lanka.

> 🔗 **[Crowdfunding link — coming soon]**

### 🛰️ Receive Our Signal
Are you a licensed radio amateur with a ground station? We would love to have you in our SatNOGS network. When we launch, your station could be one of the first to hear SLARS-BCG-1 calling from orbit.

### 🏫 Use Our Educational Resources
All our design documents, simulations, and software are freely available under the MIT licence. If you teach space engineering, please use this project as a teaching resource. We will add classroom guides in SLARS-BCG-1 Official Website.

### 📢 Share Our Story
Share this repository. Tell people about building Sri Lanka's first school satellite. The more people know about SLARS-BCG-1, the better our chances of launching.

---

## 👩‍🚀 The Team

**School:** Bandaranayake College, Gampaha, Sri Lanka
**Team:** SLARS Team
**Team size:** 13 — join us
| Name | Role |
|---|---|
|K.R.Sasmitha Kaushan Kaushan|Team Leader & Project Manager|
|S.O.M.A.V. Samarasekara|Team Collaborator|
|R.P.L.L. Rajapaksha|Team Collaborator|
|R.M.V.M.V. Rathnakyeke|Team Collaborator|
|E. Sayul Mandil Wijewardena|Team Collaborator|
|K.S.K. Madusanka|Team Collaborator|
|K.A. Sandul Ranuga|Team Collaborator|
|U.R.S.D.M.R.H.B. Dasanayake|Team Collaborator|
|E.M.D.S. Edirisooriya|Team Collaborator|
|R. Sesath Ranbandara|Team Collaborator|
|K.A. Idoosha Sadil Thenuda|Team Collaborator|
|M. Kaveesha Mihiranga Premarathna|Team Collaborator|
|W. Viduna Dhamsilu Peiris|Team Collaborator|
|Chamath Shamal|Mentor|
|S.A. Ushan Dissanayake|Mentor|
|Nidula Maneth Gunawardana|Mentor|
|Andrew Greenberg|Mentor|
|Manuel|Mentor|

---

## 🏆 Why This Matters

Sri Lanka has never launched a satellite built by school students. The closest we have come is university-level missions still in development. SLARS-BCG-1 changes that.

We are not waiting to grow up to contribute to our country. We are doing it now. At school. With laptops, determination, and the complete engineering design of a real 2U CubeSat that will one day orbit 500 km above the island we love.

When SLARS-BCG-1 transmits its first flood map of Sri Lanka to the NDMSC — that map will have been made possible by school students from Gampaha who refused to believe that age was a barrier to protecting their motherland.

---

## 📄 Documentation

All 8 mission design step documents are available in the `/mission-design/` folder.
Each document covers one complete engineering subsystem from first principles to locked specifications.

| Document | Description |
|---|---|
| Step 1 — Orbit Design | Orbital mechanics, Sri Lanka coverage analysis, link budget baseline |
| Step 2 — Power Budget | Solar cell sizing, battery capacity, EPS architecture, power scheduling |
| Step 3 — Payload Stack | Imager specifications, SDR relay design, AIS receiver, combined RF board |
| Step 4 — COMMS | Link budgets for all 4 channels, antenna design, ground station specs |
| Step 5 — ADCS | B-dot detumbling, PD/PID nadir pointing, magnetorquer and RW sizing |
| Step 6 — OBC/OBSW | FreeRTOS tasks, SGP4 propagator, FDIR, TC/TM protocol, data management |
| Step 7 — Structure/Thermal | Chassis design, mass budget, vibration analysis, passive thermal control |
| Step 8 — Ground Ops | Ground station, data pipeline, disaster workflow, launch preparation |

---

## 🔗 Links and Contacts

| Resource | Link |
|---|---|
| **Mission repository** | [github.com/SLARS-BCG-1](https://github.com/SLARS-BCG-1) *(this repo)* |
| **Crowdfunding** | Coming soon |
| **SatNOGS satellite page** | Pending — registration in progress |
| **IARU coordination** | Pending — application submitted |
| **School website** | https://bcg.lk/ |
| **Project's Website** | https://slars-bcg-1.netlify.app/
| **Contact** | Via GitHub Issues, from the project website, or the school Inventor's Club. |

---

## 🙏 Acknowledgements

- Our teachers and principal at Bandaranayake College, Gampaha — for believing in us
- Our official mission partners — for guiding us through every engineering step
- The open source community — FreeRTOS, GNURadio, GPredict, GDAL, libsgp4, LittleFS, SatNOGS — whose free tools make student satellite missions possible
- The Sri Lankan amateur radio community — for frequency coordination support
- Every person who has shared, supported, or donated to this mission

---

<div align="center">

**SLARS-BCG-1**
*Built in Sri Lanka · Watching over Sri Lanka*

🇱🇰 Bandaranayake College, Gampaha · 2026

*"More than a scientific mission, it is a symbol of youth innovation and national hope."*

</div>
