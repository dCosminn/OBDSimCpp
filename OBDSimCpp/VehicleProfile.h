#pragma once
#include <string>
#include <map>
#include <cstdint>

struct EngineParams {
    float idleRpm = 800.0f;
    float idleRpmVariance = 40.0f;

    float normalCoolant = 89.0f;
    float normalOilTemp = 86.0f;
    float normalIntake = 24.0f;
    float normalBaro = 101.0f;
    float normalFuelLevel = 65.0f;
    float normalVoltage = 14.1f;
    float normalMAF = 2.5f;
    float normalLoad = 22.0f;

    int cylinders = 4;
    bool isDiesel = false;

    // ✅ CRUISE PARAMETERS (ADDED)
    float cruiseSpeed = 80.0f;
    float cruiseRpm = 2250.0f;
    float cruiseLoad = 35.0f;
    float cruiseMAF = 11.5f;
    float cruiseThrottle = 20.0f;
    float cruiseMAP = 50.0f;
};

class VehicleProfile {
public:
    virtual ~VehicleProfile() = default;

    virtual std::string getVIN() const = 0;
    virtual std::string getCalibrationID() const = 0;
    virtual std::string getPartNumber() const = 0;
    virtual std::string getEcuName() const = 0;
    virtual std::string getMake() const = 0;
    virtual std::string getModel() const = 0;
    virtual std::string getYear() const = 0;
    virtual std::string getELMProtocol() const = 0;
    virtual std::string getProtocolDesc() const = 0;
    virtual std::string getEcuResponse() const = 0;

    virtual EngineParams getEngineParams() const = 0;
    virtual std::map<int, uint32_t> getPidBitmasks() const = 0;
    virtual std::map<std::string, std::string> getMode22Pids() const = 0;

    std::string getDisplayName() const {
        return getMake() + " " + getModel() + " (" + getYear() + ")";
    }
};