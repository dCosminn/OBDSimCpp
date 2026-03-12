#pragma once
// =============================================================================
//  ELM327Handler.h  —  ELM327 protocol parser and response formatter
//
//  Handles all AT commands and OBD2 PID queries.
//  Knows nothing about Bluetooth — takes a string command, returns a string
//  response.  The transport layer (BluetoothServer) calls handle() and sends
//  whatever it returns.
//
//  Supports:
//    AT commands  — ATZ, ATE, ATL, ATS, ATH, ATSP, ATSH, ATAT, ATAL, ATST,
//                   ATM, ATD, STI, VTI, ATDP
//    Mode 01      — Standard SAE J1979 PIDs (bitmask discovery + data)
//    Mode 09      — Vehicle information (VIN, calibration ID, ECU name)
//    Mode 22      — Ford UDS proprietary PIDs
// =============================================================================
#include "VehicleProfile.h"
#include "EngineSimulator.h"
#include <string>

class ELM327Handler {
public:
    ELM327Handler(const VehicleProfile* profile, EngineSimulator* sim);

    // Process one command line (no \r).  Returns full response text without
    // the prompt character — BluetoothServer appends "\r>" itself.
    // Returns empty string if command should produce no output (empty line).
    std::string handle(const std::string& rawCmd);
    std::string getProfileName() const { return profile_->getDisplayName(); }

    // Reset session state — call when client disconnects
    void resetSession();

private:
    const VehicleProfile* profile_;
    EngineSimulator*      sim_;

    // ── ELM327 session state ─────────────────────────────────────────────────
    bool        spacesOn_  = true;
    bool        echoOn_    = true;
    bool        headersOn_ = false;
    std::string currentEcuAddr_ = "";  // set by ATSH, used for Mode 22 routing

    // ── Mode handlers ────────────────────────────────────────────────────────
    std::string handleMode01(const std::string& pid);
    std::string handleMode09(const std::string& pid);
    std::string handleMode22(const std::string& addr);   // 4-char address e.g. "1310"

    // ── Response formatting ──────────────────────────────────────────────────
    // Build a complete OBD response line with optional header + spaces
    // hdr      = "7E8 06"  (ECU address + CAN data length byte)
    // payload  = "41 00 1E 3F 98 03"  (mode response + pid + data bytes)
    std::string obdLine(const std::string& hdr, const std::string& payload) const;

    // Strip spaces from a string
    static std::string nosp(const std::string& s);

    // ── Byte encoding helpers (match ESP32 helpers exactly) ──────────────────
    std::string h8(int v)          const;  // 1 byte  → "XX"
    std::string h16(int v)         const;  // 2 bytes → "XX XX"
    std::string h32(int64_t v)     const;  // 4 bytes → "XX XX XX XX"
    std::string encTemp(float c)   const;  // A-40     → h8(c+40)
    std::string encPct(float p)    const;  // A*100/255 → h8(p*255/100)
    std::string encSpct(float p)   const;  // A/1.28-100 → h8((p+100)*1.28)
    std::string encKpa1(float k)   const;  // A         → h8(k)
    std::string encRpm()           const;  // (A*256+B)/4 → h16(rpm*4)
    std::string encMaf()           const;  // (A*256+B)/100 → h16(maf*100)
    std::string encVolt()          const;  // (A*256+B)/1000 → h16(volt*1000)
    std::string encTiming()        const;  // A/2-64 → h8((t+64)*2)
    std::string encO2(float v)     const;  // A*0.005V → h8(v/0.005)
};
