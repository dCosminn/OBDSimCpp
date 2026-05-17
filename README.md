# OBD2 Simulator – Windows Bluetooth Classic

## Overview
Simulates an **ELM327 OBD2 adapter** over Bluetooth Classic (SPP) on Windows.  
Connect apps like **CarScanner** or **Torque Pro** to get realistic vehicle data without a real car.

**Features:**
- Standard OBD2 PIDs (Mode 01 / 09)  
- Ford proprietary PIDs (Mode 22)  
- Engine simulation: RPM, coolant, O2, fuel trim  
- DTC fault injection with MIL (Mode 03 / 07 / 0A / 04)  
- Windows Bluetooth Classic (RFCOMM/SPP)  

---

## Build
1. Clone in **Visual Studio 2022/2026**  
2. Set **Windows SDK Version 10.0** (latest) in project properties → General  
3. Build the project (F5 or Ctrl+B)  

---

## Usage
1. Run the `.exe` as Administrator  
2. Confirm console shows: `SDP registered OK` and `Listening for incoming connections`  
3. On Android: pair with your PC via Bluetooth  
4. Open CarScanner or your OBD app → scan for adapters → select your PC  
5. Console shows [TX] Adatpter info
6. Use the console menu to control the simulation (see Console Commands below)

---

## Console Commands

| Key | Action |
|-----|--------|
| `1` | Idle mode (~800 RPM) |
| `2` | Cruise mode (~80 km/h) |
| `3` | Inject DTCs — shows numbered list, enter indices (e.g. `1 3 5`) or `0` for random |
| `4` | Clear all DTCs (MIL off) |

---

## DTC Fault Injection

The simulator implements the full OBD2 DTC flow:

- **Mode 03** — stored DTCs (MIL-triggering, confirmed faults)  
- **Mode 07** — pending DTCs (subset of stored, not yet confirmed across 2 drive cycles)  
- **Mode 0A** — permanent DTCs (survive Mode 04 clear; empty by default on this profile)  
- **Mode 04** — clears stored and pending, turns MIL off; permanent codes are unaffected  
- **Mode 01 PID 01** — reports MIL on/off status and stored DTC count  

Press `3` to see the full list of 39 available fault codes for the Ford Fiesta 1.4, grouped by system (MAF, coolant, O2, catalyst, misfire, camshaft, throttle, EGR, EVAP, voltage, Ford-specific). 
Enter one or more numbers to activate them, or `0` for a random selection of 1–3 faults.

---

## Architecture

| Class | Responsibility |
|-------|----------------|
| `BluetoothServer` | Windows RFCOMM socket, SDP registration, send/receive bytes. Independent of OBD2. |
| `ELM327Handler` | Parses AT commands and OBD PID queries. Calls `VehicleProfile` & `EngineSimulator`. |
| `EngineSimulator` | Generic physics engine (RPM, coolant, O2, fuel trim). Car-independent. |
| `DTCManager` | Manages stored, pending and permanent DTC lists. Handles Mode 03/04/07/0A. |
| `VehicleProfile` | Abstract base class. Provides PID bitmasks, Mode 22 addresses, engine params, VIN. |
| `FordFiesta2006` | Concrete profile for Ford Fiesta MK5 1.4. Inherits `VehicleProfile`. |

**Adding a new car:**
- Copy `FordFiesta2006.cpp/h` and rename the class (e.g., `VWGolf2010`)  
- Update VIN, engine params, PID bitmasks, Mode 22 addresses  
- Update `main.cpp`: `std::make_unique<FordFiesta2006>()` → `std::make_unique<VWGolf2010>()`  

---

## Protocols

**ELM327 AT Commands:**
- `ATZ` – reset (ELM327 v1.5)  
- `ATE0/ATE1` – echo off/on  
- `ATH0/ATH1` – headers off/on  
- `ATS0/ATS1` – spaces off/on  
- `ATL0/ATL1` – line feeds off/on  
- `ATSP0-9` – set protocol (uses protocol 6)  
- `ATSH` – header for Mode 22 ECU routing  
- `ATAT0/1/2` – adaptive timing  
- `ATST` – timeout (ignored)  
- `ATAL` – allow long messages  
- `ATM0` – memory off  
- `ATDP` – describe protocol: ISO 15765-4 (CAN 11/500)  
- `STI` – adapter identity  
- `VTI` – version  

**OBD2 Modes:**
- Mode 01 – Current powertrain data (19 SAE J1979 PIDs)  
- Mode 03 – Stored DTCs (MIL codes)  
- Mode 04 – Clear DTCs / reset MIL  
- Mode 07 – Pending DTCs  
- Mode 09 – Vehicle info (VIN, calibration ID, ECU name)  
- Mode 0A – Permanent DTCs  
- Mode 22 – Ford UDS proprietary PIDs (18 PCM addresses)  
- Other – Returns NO DATA or `7F 22 11` (UDS negative response)  

### Mode 01 -- Standard SAE J1979 (19 PIDs)

| PID   | Name                      | Formula           | Unit  |
|-------|---------------------------|-------------------|-------|
| 0104  | Calculated Engine Load    | A*100/255        | %     |
| 0105  | Engine Coolant Temp       | A-40             | degC  |
| 0106  | Short Term Fuel Trim Bank 1 | A/1.28 - 100     | %     |
| 0107  | Long Term Fuel Trim Bank 1 | A/1.28 - 100     | %     |
| 0108  | Intake Manifold Pressure   | A                 | kPa   |
| 010C  | Engine RPM                | (A*256+B)/4      | RPM   |
| 010D  | Vehicle Speed             | A                 | km/h  |
| 010E  | Timing Advance            | A/2 - 64         | deg   |
| 010F  | Intake Air Temperature    | A-40             | degC  |
| 0110  | MAF Air Flow Rate         | (A*256+B)/100    | g/s   |
| 0111  | Throttle Position         | A*100/255        | %     |
| 0114  | O2 Sensor 1 Voltage       | A*0.005          | V     |
| 0115  | O2 Sensor 2 Voltage       | A*0.005          | V     |
| 011F  | Run Time Since Engine Start | A*256+B          | s     |
| 012C  | Commanded EGR             | A*100/255        | %     |
| 012F  | Fuel Tank Level           | A*100/255        | %     |
| 0133  | Barometric Pressure       | A                 | kPa   |
| 0142  | Control Module Voltage    | (A*256+B)/1000   | V     |
| 0146  | Ambient Air Temperature   | A-40             | degC  |

### Mode 22 -- Ford Proprietary PCM PIDs  (18 PIDs)

| Command | Name                      | Formula           | Unit  |
|---------|---------------------------|-------------------|-------|
| 221310  | Engine Oil Temperature    | (A*256+B)/100     | degC  |
| 221131  | Ignition Timing           | (A*256+B)/100-64  | deg   |
| 221440  | MAP Extended              | (A*256+B)/100     | kPa   |
| 22113C  | EGR Duty Cycle            | A*100/255        | %     |
| 22160E  | Misfire Count Cyl 1       | A*256+B          | count |
| 22160F  | Misfire Count Cyl 2       | A*256+B          | count |
| 221610  | Misfire Count Cyl 3       | A*256+B          | count |
| 221611  | Misfire Count Cyl 4       | A*256+B          | count |
| 22F405  | Coolant Temp (Ford)       | A-40             | degC  |
| 22F40F  | Intake Air Temp (Ford)    | A-40             | degC  |
| 22F40B  | MAP (Ford)                | A                 | kPa   |
| 22F433  | Barometric Pressure (Ford) | A                 | kPa   |
| 22F406  | Short Term Fuel Trim (Ford) | A/1.28-100      | %     |
| 22F407  | Long Term Fuel Trim (Ford) | A/1.28-100      | %     |
| 22F443  | Engine Load (Ford)        | (A*256+B)/100    | %     |
| 22F434  | Lambda (Ford)             | (A*256+B)/32768  | lambda|
| 22F40C  | RPM (Ford)                | (A*256+B)/4      | RPM   |
| 22F40D  | Vehicle Speed (Ford)      | A                 | km/h  |

### Ford Fiesta 1.4 DTC Pool (39 codes)

| Code  | Description |
|-------|-------------|
| P0102 | MAF/VAF Circuit Low Input |
| P0103 | MAF/VAF Circuit High Input |
| P0113 | Intake Air Temperature Sensor High Input |
| P0115 | Engine Coolant Temp Sensor 1 Circuit |
| P0117 | Engine Coolant Temp Sensor Circuit Low |
| P0118 | Engine Coolant Temp Sensor Circuit High |
| P0121 | Throttle/Pedal Position Sensor Range/Performance |
| P0122 | Throttle/Pedal Position Sensor Low Input |
| P0130 | O2 Sensor Circuit Bank 1 Sensor 1 |
| P0131 | O2 Sensor Circuit Low Bank 1 Sensor 1 |
| P0132 | O2 Sensor Circuit High Bank 1 Sensor 1 |
| P0133 | O2 Sensor Slow Response Bank 1 Sensor 1 |
| P0136 | O2 Sensor Circuit Bank 1 Sensor 2 |
| P0138 | O2 Sensor Circuit High Bank 1 Sensor 2 |
| P0171 | System Too Lean (Bank 1) |
| P0172 | System Too Rich (Bank 1) |
| P0174 | System Too Lean (Bank 2) |
| P0175 | System Too Rich (Bank 2) |
| P0300 | Random / Multiple Cylinder Misfire Detected |
| P0301 | Cylinder 1 Misfire Detected |
| P0302 | Cylinder 2 Misfire Detected |
| P0303 | Cylinder 3 Misfire Detected |
| P0304 | Cylinder 4 Misfire Detected |
| P0340 | Camshaft Position Sensor A Circuit Bank 1 |
| P0341 | Camshaft Position Sensor A Circuit Range/Performance |
| P0401 | EGR Flow Insufficient Detected |
| P0402 | EGR Excessive Flow Detected |
| P0420 | Catalyst System Efficiency Below Threshold Bank 1 |
| P0440 | Evaporative Emission Control System Malfunction |
| P0442 | EVAP System Small Leak Detected |
| P0455 | EVAP System Gross Leak Detected |
| P0562 | System Voltage Low |
| P0563 | System Voltage High |
| P0600 | Serial Communication Link Malfunction |
| P0606 | ECM/PCM Processor Fault |
| P1131 | Lack of HO2S-11 Switch - Sensor Indicates Lean |
| P1132 | Lack of HO2S-11 Switch - Sensor Indicates Rich |
| P1233 | Fuel Pump Driver Module Off-line |
| P1450 | Unable to Bleed Up Fuel Tank Vacuum |

---

**OBD2 Simulator – Built with Visual Studio 2026**