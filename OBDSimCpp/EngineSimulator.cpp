// =============================================================================
//  EngineSimulator.cpp
// =============================================================================
#include "EngineSimulator.h"
#include <cmath>
#include <algorithm>

static constexpr float PI     = 3.14159265358979f;
static constexpr float TWO_PI = 2.0f * PI;

EngineSimulator::EngineSimulator(const EngineParams& params)
    : params_(params)
    , rng_(std::random_device{}())
{
    startTime_  = Clock::now();
    lastUpdate_ = startTime_;

    // Start cold — values warm up over the first ~2 minutes
    state_.coolant    = 20.0f;
    state_.oilTemp    = 18.0f;
    state_.fuelLevel  = params_.normalFuelLevel;
    state_.baro       = params_.normalBaro;
    state_.ambientTemp= 19.0f;
    state_.rpm        = params_.idleRpm;
}

void EngineSimulator::update()
{
    auto  now     = Clock::now();
    float dt      = std::chrono::duration<float>(now - lastUpdate_).count();
    float elapsed = std::chrono::duration<float>(now - startTime_).count();
    lastUpdate_   = now;

    // Clamp dt — if debugger paused or system was suspended, don't jump
    dt = std::min(dt, 0.1f);

    state_.runTime = elapsed;

    // ── Oscillator phases ────────────────────────────────────────────────────
    phRpm_  += dt * 0.3f  * TWO_PI;   // RPM wobble ~0.3 Hz
    phO2_   += dt * 1.0f  * TWO_PI;   // O2 cycle ~1 Hz (closed-loop lambda)
    phStft_ += dt * 0.8f  * TWO_PI;   // fuel trim follows O2

    // ── RPM ─────────────────────────────────────────────────────────────────
    // Idle: target ±variance oscillation + small random noise
    state_.rpm = params_.idleRpm
               + params_.idleRpmVariance * std::sin(phRpm_)
               + params_.idleRpmVariance * 0.3f * std::sin(phRpm_ * 2.1f)
               + rnd(5.0f);

    // ── Engine load follows RPM loosely ──────────────────────────────────────
    float rpmDelta = state_.rpm - params_.idleRpm;
    state_.load    = params_.normalLoad + rpmDelta * 0.02f + rnd(1.0f);
    state_.load    = std::max(5.0f, std::min(100.0f, state_.load));

    // ── MAF follows load ─────────────────────────────────────────────────────
    state_.maf = params_.normalMAF * (state_.load / params_.normalLoad)
               + rnd(0.05f);
    state_.maf = std::max(0.5f, state_.maf);

    // ── Throttle: nearly closed at idle ──────────────────────────────────────
    state_.throttle = 11.0f + rpmDelta * 0.005f + rnd(0.3f);

    // ── MAP: vacuum at idle, rises with throttle ──────────────────────────────
    state_.map_kpa = 38.0f - state_.throttle * 0.3f + rnd(0.4f);
    state_.map_kpa = std::max(20.0f, std::min(101.0f, state_.map_kpa));

    // ── Timing advance ────────────────────────────────────────────────────────
    state_.timing = 8.0f + rpmDelta * 0.003f + rnd(0.2f);

    // ── Coolant: asymptotic warmup then gentle oscillation ───────────────────
    // Time constant: fully warm in ~2 real minutes
    float targetCoolant = params_.normalCoolant;
    if (elapsed < 120.0f) {
        state_.coolant = targetCoolant - (targetCoolant - 20.0f) * std::exp(-elapsed / 60.0f);
    } else {
        state_.coolant = targetCoolant + 0.5f * std::sin(elapsed * 0.1f);
    }

    // ── Oil temp: always slightly behind coolant ──────────────────────────────
    state_.oilTemp = state_.coolant * 0.97f - 2.0f;

    // ── Intake air: ambient + engine bay heat ────────────────────────────────
    state_.intake = 24.0f + 2.0f * std::sin(elapsed * 0.05f);

    // ── O2 sensors: oscillate between rich (0.1V) and lean (0.9V) ────────────
    // Two sensors slightly out of phase with each other
    state_.o2bank1 = 0.5f + 0.38f * std::sin(phO2_);
    state_.o2bank1 = std::max(0.05f, std::min(0.95f, state_.o2bank1));
    state_.o2bank2 = 0.5f + 0.38f * std::sin(phO2_ + 0.8f);
    state_.o2bank2 = std::max(0.05f, std::min(0.95f, state_.o2bank2));

    // ── Short term fuel trim: follows O2 inversely (rich → trim down) ────────
    state_.stft = 3.0f * std::sin(phStft_ + PI) + rnd(0.3f);

    // ── Long term fuel trim: very slow drift ─────────────────────────────────
    state_.ltft += rnd(0.001f);
    state_.ltft  = std::max(-5.0f, std::min(5.0f, state_.ltft));

    // ── Battery voltage: small alternator ripple ─────────────────────────────
    state_.voltage = params_.normalVoltage + 0.05f * std::sin(elapsed * 2.0f);
}
