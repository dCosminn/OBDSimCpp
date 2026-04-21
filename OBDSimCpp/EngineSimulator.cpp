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

    // ── COOLANT (FIXED WARM-UP MODEL) ────────────────────────────────
    float targetCoolant = params_.normalCoolant;

    // smooth warm-up curve (stable even if dt fluctuates)
    float warmFactor = 1.0f - std::exp(-elapsed / 80.0f);

    state_.coolant =
        20.0f + (targetCoolant - 20.0f) * warmFactor
        + 0.3f * std::sin(elapsed * 0.1f);

    // ── SPEED ─────────────────────────────────────────────
    state_.speed = params_.cruiseSpeed
        + 1.0f * std::sin(phSpeed_)
        + rnd(0.15f);

    state_.speed = std::clamp(state_.speed, 79.0f, 81.0f);

    // ── LOAD ─────────────────────────────────────────────
    state_.load = params_.cruiseLoad
        + 2.0f * std::sin(phSpeed_)
        + rnd(0.8f);

    state_.load = std::clamp(state_.load, 5.0f, 100.0f);

    // ── RPM (FIXED) ───────────────────────────────────────
    state_.rpm = params_.cruiseRpm
        + 50.0f * std::sin(phRpm_)
        + rnd(15.0f);

    // ── MAF ──────────────────────────────────────────────
    state_.maf = params_.cruiseMAF
        + 0.6f * std::sin(phSpeed_)
        + rnd(0.2f);

    state_.maf = std::max(0.5f, state_.maf);

    // ── THROTTLE ─────────────────────────────────────────
    state_.throttle = params_.cruiseThrottle
        + 1.0f * std::sin(phSpeed_)
        + rnd(0.3f);

    // ── MAP ──────────────────────────────────────────────
    state_.map_kpa = params_.cruiseMAP
        + 1.2f * std::sin(phSpeed_)
        + rnd(0.3f);

    state_.map_kpa = std::clamp(state_.map_kpa, 20.0f, 101.0f);

    // ── FUEL TRIM ────────────────────────────────────────
    state_.stft = 3.0f * std::sin(phStft_ + PI) + rnd(0.3f);

    state_.ltft += rnd(0.001f);
    state_.ltft = std::clamp(state_.ltft, -5.0f, 5.0f);

    // ── SPEED FUEL MODEL (FIXED REALISM) ─────────────────
    float base = 6.5f;
    float loadFactor = state_.load / 100.0f;

    float consumption = base + (loadFactor * 2.5f);

    state_.fuelRate = (consumption / 100.0f) * state_.speed
        + rnd(0.05f);

    state_.fuelRate = std::max(0.0f, state_.fuelRate);
}