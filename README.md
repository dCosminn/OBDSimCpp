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

---

**OBD2 Simulator – Built with Visual Studio 2026**
