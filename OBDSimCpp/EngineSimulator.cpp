#include "EngineSimulator.h"
#include <cmath>
#include <algorithm>

static constexpr float PI = 3.14159265358979f;
static constexpr float TWO_PI = 2.0f * PI;

EngineSimulator::EngineSimulator(const EngineParams& params)
    : params_(params), rng_(std::random_device{}())
{
    startTime_ = Clock::now();
    lastUpdate_ = startTime_;

    state_.coolant = 20.0f;
    state_.oilTemp = 18.0f;
    state_.fuelLevel = params_.normalFuelLevel;
    state_.baro = params_.normalBaro;
    state_.ambientTemp = 19.0f;
    state_.rpm = params_.idleRpm;
}

void EngineSimulator::update()
{
    auto now = Clock::now();
    float dt = std::chrono::duration<float>(now - lastUpdate_).count();
    float elapsed = std::chrono::duration<float>(now - startTime_).count();
    lastUpdate_ = now;

    dt = std::min(dt, 0.1f);
    state_.runTime = elapsed;

    phRpm_ += dt * 0.3f * TWO_PI;
    phO2_ += dt * 1.0f * TWO_PI;
    phStft_ += dt * 0.8f * TWO_PI;
    phSpeed_ += dt * 0.07f * TWO_PI;

    // =========================================================
    // MODE (ONLY SOURCE OF TRUTH)
    // =========================================================
    bool isIdle = (mode_ == Mode::Idle);

    // =========================================================
    // SPEED (NO SPIKES EVER IN IDLE)
    // =========================================================
    float targetSpeed = isIdle ? 0.0f : params_.cruiseSpeed;

    state_.speed = targetSpeed
        + (isIdle ? 0.0f : 1.0f * std::sin(phSpeed_))
        + (isIdle ? 0.0f : rnd(0.2f));

    if (isIdle)
        state_.speed = 0.0f;

    // =========================================================
    // RPM (CLEAN + STABLE)
    // =========================================================
    float targetRpm = isIdle ? params_.idleRpm : params_.cruiseRpm;

    state_.rpm = targetRpm
        + (isIdle ? 10.0f * std::sin(phRpm_) : 60.0f * std::sin(phRpm_))
        + rnd(isIdle ? 2.0f : 10.0f);

    if (isIdle)
        state_.rpm = std::clamp(state_.rpm,
            params_.idleRpm - 50.0f,
            params_.idleRpm + 120.0f);

    // =========================================================
    // LOAD
    // =========================================================
    float targetLoad = isIdle ? 15.0f : params_.cruiseLoad;

    state_.load = targetLoad
        + 2.0f * std::sin(phSpeed_)
        + rnd(0.3f);

    state_.load = std::clamp(state_.load, 0.0f, 100.0f);

    // =========================================================
    // MAF / THROTTLE / MAP
    // =========================================================
    float targetMAF = isIdle ? params_.normalMAF : params_.cruiseMAF;
    float targetThrottle = isIdle ? 5.0f : params_.cruiseThrottle;
    float targetMAP = isIdle ? 30.0f : params_.cruiseMAP;

    state_.maf = targetMAF + rnd(0.1f);
    state_.throttle = targetThrottle + rnd(0.1f);
    state_.map_kpa = std::clamp(targetMAP + rnd(0.2f), 20.0f, 101.0f);

    // =========================================================
    // COOLANT
    // =========================================================
    float warmFactor = 1.0f - std::exp(-elapsed / 80.0f);

    state_.coolant =
        20.0f + (params_.normalCoolant - 20.0f) * warmFactor
        + 0.3f * std::sin(elapsed * 0.1f);

    state_.oilTemp = state_.coolant * 0.97f - 2.0f;

    // =========================================================
    // O2 SENSORS
    // =========================================================
    state_.o2bank1 = std::clamp(0.5f + 0.38f * std::sin(phO2_), 0.05f, 0.95f);
    state_.o2bank2 = std::clamp(0.5f + 0.38f * std::sin(phO2_ + 0.8f), 0.05f, 0.95f);

    // =========================================================
    // FUEL TRIMS
    // =========================================================
    state_.stft = 3.0f * std::sin(phStft_ + PI) + rnd(0.2f);
    state_.ltft = std::clamp(state_.ltft + rnd(0.001f), -5.0f, 5.0f);

    // =========================================================
    // FUEL RATE
    // =========================================================
    float consumption = 6.5f + (state_.load / 100.0f) * 2.5f;

    state_.fuelRate =
        isIdle
        ? 0.2f
        : (consumption / 100.0f) * state_.speed;

    state_.fuelRate = std::max(0.0f, state_.fuelRate);
}