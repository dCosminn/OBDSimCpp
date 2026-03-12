#pragma once
// =============================================================================
//  VehicleProfile.h  --  Abstract base for all simulated vehicles
//
//  To add a new car:
//    1. Create MyNewCar.h / MyNewCar.cpp
//    2. Inherit from VehicleProfile
//    3. Override all pure-virtual methods
//    4. In main.cpp: VehicleProfile* car = new MyNewCar();
//
//  Nothing else needs to change.
// =============================================================================
#include <string>
#include <map>
#include <cstdint>

// Parameters the EngineSimulator uses to produce realistic values.
// Each VehicleProfile supplies its own set.
struct EngineParams {
    float idleRpm = 800.0f;   // target idle RPM
    float idleRpmVariance = 40.0f;    // ±RPM oscillation at idle
    float normalCoolant = 89.0f;    // °C when fully warm
    float normalOilTemp = 86.0f;    // °C oil when warm (always slightly below coolant)
    float normalIntake = 24.0f;    // °C intake air at idle
    float normalBaro = 101.0f;   // kPa barometric (sea level = 101)
    float normalFuelLevel = 65.0f;    // % fuel in tank
    float normalVoltage = 14.1f;    // V battery (engine running = alternator ~14V)
    float normalMAF = 2.5f;     // g/s at idle
    float normalLoad = 22.0f;    // % engine load at idle
    int   cylinders = 4;
    bool  isDiesel = false;
};

// =============================================================================
class VehicleProfile {
public:
    virtual ~VehicleProfile() = default;

    // ── Identity ──────────────────────────────────────────────────────────────
    virtual std::string getVIN()          const = 0;
    virtual std::string getCalibrationID() const = 0;  // Mode 09 PID 04 -- PCM strategy
    virtual std::string getPartNumber()     const = 0;  // Ford DID F101 -- ECU part number
    virtual std::string getEcuName()       const = 0;  // Mode 09 PID 0A
    virtual std::string getMake()         const = 0;
    virtual std::string getModel()        const = 0;
    virtual std::string getYear()         const = 0;
    virtual std::string getELMProtocol()  const = 0;  // returned by ATDP, e.g. "6"
    virtual std::string getProtocolDesc() const = 0;  // human readable
    virtual std::string getEcuResponse()  const = 0;  // CAN response address, e.g. "7E8"

    // ── Engine ────────────────────────────────────────────────────────────────
    virtual EngineParams getEngineParams() const = 0;

    // ── Mode 01 PID support bitmasks ─────────────────────────────────────────
    // Key = range base (0x00, 0x20, 0x40 …)
    // Value = 32-bit bitmask where bit 31 = base+1, bit 0 = base+32
    // Bit 0 set means the next range is also supported.
    // These are returned verbatim in response to 0100, 0120, 0140 …
    virtual std::map<int, uint32_t> getPidBitmasks() const = 0;

    // ── Mode 22 proprietary PIDs ──────────────────────────────────────────────
    // Key = 4-char hex address, e.g. "1310"
    // Value = human-readable name (used for console logging only)
    virtual std::map<std::string, std::string> getMode22Pids() const = 0;

    // ── Convenience ───────────────────────────────────────────────────────────
    std::string getDisplayName() const {
        return getMake() + " " + getModel() + " (" + getYear() + ")";
    }
};