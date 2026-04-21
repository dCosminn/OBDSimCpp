#pragma once
// =============================================================================
//  FordFiesta2006.h  --  Ford Fiesta MK5 1.4 Petrol (80hp) 2006
//
//  Protocol:  ISO 15765-4 CAN 11-bit 500kbps  (ELM327 protocol 6)
//  ECU addr:  7E0  /  response: 7E8
//  Engine:    Duratec 1.4 SOHC, 4 cylinders, 80hp @ 6000rpm
//
//  Mode 01 PIDs: 19 standard SAE J1979 PIDs
//  Mode 22 PIDs: 18 Ford proprietary PCM PIDs
//
//  NOTE:
//  Cruise behavior (RPM, load, MAF, etc.) is now provided via EngineParams.
// =============================================================================
#include "VehicleProfile.h"

class FordFiesta2006 : public VehicleProfile {
public:
    // ── Identity ──────────────────────────────────────────────────────────────
    std::string getVIN()          const override { return "WF0FXXGAJF6R12345"; }
    std::string getCalibrationID() const override { return "4S61-14C204-NA"; }
    std::string getPartNumber()     const override { return "4S61-12A650-NA"; }
    std::string getEcuName()       const override { return "PCM"; }
    std::string getMake()         const override { return "Ford"; }
    std::string getModel()        const override { return "Fiesta MK5 1.4 Petrol"; }
    std::string getYear()         const override { return "2006"; }
    std::string getELMProtocol()  const override { return "6"; }
    std::string getProtocolDesc() const override { return "ISO 15765-4 (CAN 11/500)"; }
    std::string getEcuResponse()  const override { return "7E8"; }

    // ── Engine ────────────────────────────────────────────────────────────────
    // Includes idle + cruise calibration (RPM, load, MAF, throttle, MAP)
    EngineParams getEngineParams() const override;

    // ── Mode 01 bitmasks ─────────────────────────────────────────────────────
    std::map<int, uint32_t> getPidBitmasks() const override;

    // ── Mode 22 PIDs (Ford Duratec PCM @ 7E0) ────────────────────────────────
    std::map<std::string, std::string> getMode22Pids() const override;
};