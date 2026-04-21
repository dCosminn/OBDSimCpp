#pragma once
#include "VehicleProfile.h"
#include <chrono>
#include <random>

struct EngineState {
    float rpm = 800.0f;
    float coolant = 20.0f;
    float oilTemp = 18.0f;
    float intake = 24.0f;
    float ambientTemp = 19.0f;
    float throttle = 11.0f;
    float load = 22.0f;
    float maf = 2.5f;
    float map_kpa = 35.0f;
    float baro = 101.0f;
    float stft = 0.0f;
    float ltft = 1.5f;
    float o2bank1 = 0.45f;
    float o2bank2 = 0.45f;
    float fuelLevel = 65.0f;
    float voltage = 14.1f;
    float timing = 8.0f;
    float egrDuty = 0.0f;
    float speed = 80.0f;
    float fuelRate = 5.6f;
    float runTime = 0.0f;
};

class EngineSimulator {
public:
    enum class Mode {
        Idle,
        Cruise
    };

    explicit EngineSimulator(const EngineParams& params);

    void update();
    const EngineState& getState() const { return state_; }

    void setMode(Mode m) { mode_ = m; }

private:
    EngineParams params_;
    EngineState state_;

    Mode mode_ = Mode::Idle;

    bool isMoving_ = false;

    using Clock = std::chrono::steady_clock;
    Clock::time_point startTime_;
    Clock::time_point lastUpdate_;

    float phRpm_ = 0.0f;
    float phO2_ = 0.0f;
    float phStft_ = 0.0f;
    float phSpeed_ = 0.0f;

    std::mt19937 rng_;
    std::uniform_real_distribution<float> noise_{ -1.0f, 1.0f };

    float rnd(float s) { return noise_(rng_) * s; }
};