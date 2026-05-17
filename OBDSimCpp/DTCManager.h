#pragma once
// =============================================================================
//  DTCManager.h  -  ECU Diagnostic Trouble Code manager
//
//  The fault pool comes from VehicleProfile::getDtcPool() so every car
//  has its own realistic set of codes.
//
//  Modes supported:
//    Mode 03  - stored (confirmed) DTCs
//    Mode 07  - pending DTCs   (subset of stored, not yet confirmed across 2 drive cycles)
//    Mode 0A  - permanent DTCs (separate list, survive Mode 04 clear)
//    Mode 04  - clears stored + pending only; permanent list is unaffected
// =============================================================================
#include "DTCEntry.h"
#include <vector>

class DTCManager {
public:
    // Initialised with the pool provided by the vehicle profile.
    explicit DTCManager(const std::vector<DTCEntry>& pool);

    // Pick `count` random DTCs from the pool and activate them.
    // Pass -1 to pick a random count (1-3).
    void randomizeFaults(int count = -1);

    // Activate specific DTCs by 1-based index from the pool list.
    void setFaults(const std::vector<int>& picks);

    // Clear stored + pending DTCs (MIL off).
    // Permanent DTCs are NOT cleared — they can only be removed by
    // addPermanent() being called then cleared via clearPermanent(),
    // which is not triggered by Mode 04.
    void clearFaults();

    // Add a DTC to the permanent list (Mode 0A).
    // Permanent codes are ECU-confirmed faults that survive Mode 04.
    // On a 2006 Fiesta these are always empty in normal use.
    void addPermanent(const DTCEntry& dtc);

    // Clear permanent DTCs (simulates ECU removing them after monitor passes).
    void clearPermanent();

    // -- Queries --------------------------------------------------------------
    bool                          hasMIL()      const { return !stored_.empty(); }
    int                           getDtcCount() const { return (int)stored_.size(); }
    const std::vector<DTCEntry>& getStored()   const { return stored_; }
    const std::vector<DTCEntry>& getPending()  const { return pending_; }
    const std::vector<DTCEntry>& getPermanent()const { return permanent_; }
    const std::vector<DTCEntry>& getPool()     const { return pool_; }

private:
    std::vector<DTCEntry> pool_;       // full catalogue supplied by the vehicle profile
    std::vector<DTCEntry> stored_;     // currently active stored DTCs (Mode 03)
    std::vector<DTCEntry> pending_;    // subset of stored, not yet confirmed (Mode 07)
    std::vector<DTCEntry> permanent_;  // survive Mode 04 clear (Mode 0A) — empty by default
};