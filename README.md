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
   
## Usage
    1. Right-click the .exe -- Run as administrator
    2. Confirm console shows: SDP registered OK and Listening for incoming connections
    3. On Android: Settings -- Bluetooth -- pair with your PC by its computer name
    4. Open CarScanner or your OBD app -- scan for adapters -- select your PC
    5. The console will show APP CONNECTED and start displaying RX/TX commands
---

## Architecture

| Class | Responsibility |
|-------|----------------|
| `BluetoothServer` | Windows RFCOMM socket, SDP registration, accept loop, send/receive bytes. Knows nothing about OBD2. |
| `ELM327Handler` | Parses AT commands and OBD PID queries. Calls `VehicleProfile` and `EngineSimulator` for data. |
| `EngineSimulator` | Generic physics engine. RPM oscillation, coolant warmup, O2 cycling, fuel trim. Car-independent. |
| `VehicleProfile` | Abstract base class. Provides PID bitmasks, Mode 22 addresses, engine parameters, VIN. |
| `FordFiesta2006` | Concrete vehicle profile. Inherits `VehicleProfile` with all Ford Fiesta MK5 1.4 specifics. |


**Adding a New Car**
- Copy FordFiesta2006.h and FordFiesta2006.cpp
- Rename the class (e.g. VWGolf2010)
- Update VIN, engine params, PID bitmasks, Mode 22 addresses
- In main.cpp change: std::make_unique<FordFiesta2006>()  to  std::make_unique<VWGolf2010>()
- Nothing else needs to change
---

## Protocols

**ELM327 AT Commands Supported:**
- `ATZ` – reset, returns ELM327 v1.5  
- `ATE0/ATE1` – echo off/on  
- `ATH0/ATH1` – headers off/on  
- `ATS0/ATS1` – spaces off/on  
- `ATL0/ATL1` – line feeds off/on  
- `ATSP0-9` – set protocol (uses protocol 6)  
- `ATSH` – set header for Mode 22 ECU routing  
- `ATAT0/1/2` – adaptive timing  
- `ATST` – set timeout (ignored)  
- `ATAL` – allow long messages  
- `ATM0` – memory off  
- `ATDP` – describe protocol: ISO 15765-4 (CAN 11/500)  
- `STI` – adapter identity  
- `VTI` – version  

**OBD2 Modes Supported:**
- Mode 01 – Current powertrain data (19 SAE J1979 PIDs)  
- Mode 09 – Vehicle info (VIN, calibration ID, ECU name)  
- Mode 22 – Ford UDS proprietary PIDs (18 PCM addresses)  
- Mode 03 – Not implemented (returns NO DATA)  
- Other – Returns NO DATA or `7F 22 11` (UDS negative response)

---
**OBD2 Simulator – Built with Visual Studio 2026**
