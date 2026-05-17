// =============================================================================
//  DTCManager.cpp
// =============================================================================
#include "DTCManager.h"
#include <algorithm>
#include <random>
#include <chrono>
#include <iostream>

// =============================================================================
DTCManager::DTCManager(const std::vector<DTCEntry>& pool)
    : pool_(pool)
{
    // permanent_ starts empty — a 2006 Fiesta has no permanent DTCs
    // unless the ECU has confirmed a fault across multiple drive cycles.
}

// =============================================================================
void DTCManager::randomizeFaults(int count)
{
    std::mt19937 rng(
        (uint32_t)std::chrono::steady_clock::now().time_since_epoch().count());

    if (count < 0) {
        std::uniform_int_distribution<int> cntDist(1, 3);
        count = cntDist(rng);
    }
    count = std::min(count, (int)pool_.size());

    std::vector<DTCEntry> shuffled = pool_;
    std::shuffle(shuffled.begin(), shuffled.end(), rng);

    stored_.assign(shuffled.begin(), shuffled.begin() + count);

    // Pending = roughly half of stored (at least 1 if stored is non-empty)
    int pendingCount = std::max(1, (int)stored_.size() / 2);
    pending_.assign(stored_.begin(), stored_.begin() + pendingCount);

    std::cout << "\n[DTC] MIL ON - " << stored_.size() << " fault(s) set:\n";
    for (const auto& d : stored_)
        std::cout << "      " << d.code << "  " << d.desc << "\n";
    std::cout << "\n";
}

// =============================================================================
void DTCManager::setFaults(const std::vector<int>& picks)
{
    stored_.clear();
    pending_.clear();

    for (int i : picks) {
        if (i < 1 || i >(int)pool_.size()) {
            std::cout << "  [!] Skipping invalid index " << i << "\n";
            continue;
        }
        stored_.push_back(pool_[i - 1]);
    }

    if (stored_.empty()) {
        std::cout << "\n[DTC] No valid faults selected.\n\n";
        return;
    }

    int pendingCount = std::max(1, (int)stored_.size() / 2);
    pending_.assign(stored_.begin(), stored_.begin() + pendingCount);

    std::cout << "\n[DTC] MIL ON - " << stored_.size() << " fault(s) set:\n";
    for (const auto& d : stored_)
        std::cout << "      " << d.code << "  " << d.desc << "\n";
    std::cout << "\n";
}

// =============================================================================
void DTCManager::clearFaults()
{
    // Mode 04 clears stored and pending only.
    // Permanent DTCs (Mode 0A) survive — they require the ECU to confirm
    // the fault is resolved across multiple drive cycles before removing them.
    stored_.clear();
    pending_.clear();
    std::cout << "\n[DTC] Faults cleared - MIL OFF\n";
    if (!permanent_.empty()) {
        std::cout << "[DTC] " << permanent_.size()
            << " permanent DTC(s) remain (Mode 0A — not cleared by Mode 04)\n";
    }
    std::cout << "\n";
}

// =============================================================================
void DTCManager::addPermanent(const DTCEntry& dtc)
{
    permanent_.push_back(dtc);
    std::cout << "[DTC] Permanent DTC added: " << dtc.code << "  " << dtc.desc << "\n";
}

// =============================================================================
void DTCManager::clearPermanent()
{
    permanent_.clear();
    std::cout << "[DTC] Permanent DTCs cleared (ECU monitor passed)\n";
}