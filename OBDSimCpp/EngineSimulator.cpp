#include "EngineSimulator.h"
#include <cmath>
#include <algorithm>

static constexpr float PI = 3.14159265358979f;
static constexpr float TWO_PI = 2.0f * PI;

EngineSimulator::EngineSimulator(const EngineParams& params)
    : params_(params)
    , rng_(std::random_device{}())
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

    // ── SPEED (cruise locked) ───────────────────────────────────────────
    state_.speed = params_.cruiseSpeed
        + 1.0f * std::sin(phSpeed_)
        + rnd(0.15f);

    state_.speed = std::clamp(state_.speed, 79.0f, 81.0f);

    // ── LOAD (cruise based) ─────────────────────────────────────────────
    state_.load = params_.cruiseLoad
        + 2.0f * std::sin(phSpeed_)
        + rnd(0.8f);

    state_.load = std::clamp(state_.load, 5.0f, 100.0f);

    // ── RPM (FIXED to cruise target) ────────────────────────────────────
    state_.rpm = params_.cruiseRpm
        + 60.0f * std::sin(phRpm_)
        + rnd(10.0f);

    // ── MAF (cruise calibrated) ────────────────────────────────────────
    state_.maf = params_.cruiseMAF
        + 0.6f * std::sin(phSpeed_)
        + rnd(0.2f);

    state_.maf = std::max(0.5f, state_.maf);

    // ── Throttle (driver steady cruise) ────────────────────────────────
    state_.throttle = params_.cruiseThrottle
        + 1.0f * std::sin(phSpeed_)
        + rnd(0.3f);

    // ── MAP (stable cruise vacuum) ─────────────────────────────────────
    state_.map_kpa = params_.cruiseMAP
        + 1.2f * std::sin(phSpeed_)
        + rnd(0.3f);

    state_.map_kpa = std::clamp(state_.map_kpa, 20.0f, 101.0f);

    // ── Timing ─────────────────────────────────────────────────────────
    state_.timing = 8.0f
        + (state_.rpm - params_.idleRpm) * 0.0025f
        + rnd(0.2f);

    // ── Coolant ────────────────────────────────────────────────────────
    float targetCoolant = params_.normalCoolant;

    if (elapsed < 120.0f) {
        state_.coolant = targetCoolant - (targetCoolant - 20.0f) * std::exp(-elapsed / 60.0f);
    }
    else {
        state_.coolant = targetCoolant + 0.5f * std::sin(elapsed * 0.1f);
    }

    state_.oilTemp = state_.coolant * 0.97f - 2.0f;

    // ── Intake air ─────────────────────────────────────────────────────
    state_.intake = 24.0f + 2.0f * std::sin(elapsed * 0.05f);

    // ── O2 sensors ─────────────────────────────────────────────────────
    state_.o2bank1 = 0.5f + 0.38f * std::sin(phO2_);
    state_.o2bank1 = std::clamp(state_.o2bank1, 0.05f, 0.95f);

    state_.o2bank2 = 0.5f + 0.38f * std::sin(phO2_ + 0.8f);
    state_.o2bank2 = std::clamp(state_.o2bank2, 0.05f, 0.95f);

    // ── Fuel trims ─────────────────────────────────────────────────────
    state_.stft = 3.0f * std::sin(phStft_ + PI) + rnd(0.3f);

    state_.ltft += rnd(0.001f);
    state_.ltft = std::clamp(state_.ltft, -5.0f, 5.0f);

    // ── Voltage ────────────────────────────────────────────────────────
    state_.voltage = params_.normalVoltage + 0.05f * std::sin(elapsed * 2.0f);

    // ── Fuel rate (clean cruise model) ────────────────────────────────
    float consumption = 7.0f;
    state_.fuelRate = (consumption / 100.0f) * state_.speed + rnd(0.002f);

    state_.fuelRate = std::max(0.0f, state_.fuelRate);
}