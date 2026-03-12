#pragma once
// =============================================================================
//  EngineSimulator.h  —  Generic petrol/diesel engine physics
//
//  Produces realistic, continuously-changing sensor values based on
//  parameters supplied by a VehicleProfile.  Knows nothing about any
//  specific car — it just does the math.
//
//  Call update() regularly (every ~20ms) to advance the simulation.
//  Read current values via getState().
// =============================================================================
#include "VehicleProfile.h"
#include <chrono>
#include <random>

// Snapshot of every sensor value at a given moment
struct EngineState {
    float rpm          = 800.0f;
    float coolant      = 20.0f;   // starts cold, warms up
    float oilTemp      = 18.0f;
    float intake       = 24.0f;
    float ambientTemp  = 19.0f;
    float throttle     = 11.0f;   // %
    float load         = 22.0f;   // %
    float maf          = 2.5f;    // g/s
    float map_kpa      = 35.0f;   // manifold absolute pressure
    float baro         = 101.0f;  // barometric pressure
    float stft         = 0.0f;    // short term fuel trim %
    float ltft         = 1.5f;    // long term fuel trim %
    float o2bank1      = 0.45f;   // O2 sensor 1 voltage
    float o2bank2      = 0.45f;   // O2 sensor 2 voltage
    float fuelLevel    = 65.0f;   // %
    float voltage      = 14.1f;   // V
    float timing       = 8.0f;    // ° advance
    float egrDuty      = 0.0f;    // %
    float speed        = 0.0f;    // km/h (0 while idling)
    float runTime      = 0.0f;    // seconds since engine start
};

// =============================================================================
class EngineSimulator {
public:
    explicit EngineSimulator(const EngineParams& params);

    // Advance simulation — call every ~20ms
    void update();

    const EngineState& getState() const { return state_; }

private:
    EngineParams params_;
    EngineState  state_;

    // Timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point startTime_;
    Clock::time_point lastUpdate_;

    // Oscillator phases (radians)
    float phRpm_   = 0.0f;
    float phO2_    = 0.0f;
    float phStft_  = 0.0f;

    // Random number generator for small noise
    std::mt19937                    rng_;
    std::uniform_real_distribution<float> noise_{ -1.0f, 1.0f };

    float rnd(float scale) { return noise_(rng_) * scale; }
};
