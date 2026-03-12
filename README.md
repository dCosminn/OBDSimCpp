# OBD2 Simulator – Windows Bluetooth Classic

## Overview
Simulates an **ELM327 OBD2 adapter** over Bluetooth Classic (SPP) on Windows.  
Connect apps like **CarScanner** or **Torque Pro** to get realistic vehicle data without a real car.

**Features:**
- Standard OBD2 PIDs (Mode 01 / 09)  
- Ford proprietary PIDs (Mode 22)  
- Engine simulation: RPM, coolant, O2, fuel trim  
- Windows Bluetooth Classic (RFCOMM/SPP)  
- Console logging with RX/TX display  

---

## Build
1. Open the solution in **Visual Studio 2022/2026**  
2. Set **Windows SDK Version 10.0** (latest) in project properties → General  
3. Build the project (F7 or Ctrl+B)  

---

## Usage
1. Run the `.exe` as Administrator  
2. Confirm console shows: `SDP registered OK` and `Listening for incoming connections`  
3. On Android: pair with your PC via Bluetooth  
4. Open CarScanner or your OBD app → scan for adapters → select your PC  
5. Console shows `APP CONNECTED` and starts displaying RX/TX commands  

---

## Architecture

| Class | Responsibility |
|-------|----------------|
| `BluetoothServer` | Windows RFCOMM socket, SDP registration, send/receive bytes. Independent of OBD2. |
| `ELM327Handler` | Parses AT commands and OBD PID queries. Calls `VehicleProfile` & `EngineSimulator`. |
| `EngineSimulator` | Generic physics engine (RPM, coolant, O2, fuel trim). Car-independent. |
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
- Mode 09 – Vehicle info (VIN, calibration ID, ECU name)  
- Mode 22 – Ford UDS proprietary PIDs (18 PCM addresses)  
- Mode 03 – Not implemented (returns NO DATA)  
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

---

**OBD2 Simulator – Built with Visual Studio 2026**
