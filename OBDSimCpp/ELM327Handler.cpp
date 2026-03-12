// =============================================================================
//  ELM327Handler.cpp
// =============================================================================
#include "ELM327Handler.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <iostream>

// Free helper — must be defined before handleMode01 uses it
static bool isHeartbeatPid(const std::string& pid) {
    return pid == "00" || pid == "20" || pid == "40";
}

// ─────────────────────────────────────────────────────────────────────────────
ELM327Handler::ELM327Handler(const VehicleProfile* profile, EngineSimulator* sim)
    : profile_(profile), sim_(sim)
{}

void ELM327Handler::resetSession()
{
    spacesOn_       = true;
    echoOn_         = true;
    headersOn_      = false;
    currentEcuAddr_ = "";
}

// ─── Main entry point ────────────────────────────────────────────────────────
std::string ELM327Handler::handle(const std::string& rawCmd)
{
    // ── Sanitize ─────────────────────────────────────────────────────────────
    std::string cmd = rawCmd;

    // Trim whitespace
    while (!cmd.empty() && std::isspace((unsigned char)cmd.front())) cmd.erase(cmd.begin());
    while (!cmd.empty() && std::isspace((unsigned char)cmd.back()))  cmd.pop_back();
    if (cmd.empty()) return "";

    // To uppercase
    std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                   [](unsigned char c) { return (char)std::toupper(c); });

    // For OBD commands (not AT/ST), keep only hex digits and trim to valid length
    if (cmd.size() >= 2 && cmd[0] != 'A' && cmd[1] != 'T'
        && !(cmd[0] == 'S' && cmd[1] == 'T'))
    {
        std::string clean;
        for (char c : cmd)
            if (std::isxdigit((unsigned char)c)) clean += c;
        cmd = clean;
        // Truncate to valid OBD lengths to guard against BLE fragmentation glue
        if (cmd.size() >= 2 && cmd.substr(0,2) == "22" && cmd.size() > 6)
            cmd = cmd.substr(0, 6);
        else if (cmd.size() > 4 && cmd.substr(0,2) != "22")
            cmd = cmd.substr(0, 4);
    }

    if (cmd.empty()) return "";

    // ── Echo ─────────────────────────────────────────────────────────────────
    // (we return echo inline; BluetoothServer prints it separately if needed)

    // ── Console log (suppress heartbeat spam) ────────────────────────────────
    bool isHeartbeat = (cmd == "0100" || cmd == "0120" || cmd == "0140");
    if (!isHeartbeat)
        std::cout << "  [RX] " << cmd << "\n";

    // =========================================================================
    //  AT commands
    // =========================================================================
    if (cmd == "ATZ") {
        resetSession();
        std::cout << "  [TX] ELM327 v1.5\n";
        return "ELM327 v1.5";
    }
    if (cmd == "ATD" || cmd == "ATD0")                   return "OK";
    if (cmd == "ATE0") { echoOn_   = false; return "OK"; }
    if (cmd == "ATE1") { echoOn_   = true;  return "OK"; }
    if (cmd == "ATL0" || cmd == "ATL1")                  return "OK";
    if (cmd == "ATS0") { spacesOn_ = false; return "OK"; }
    if (cmd == "ATS1") { spacesOn_ = true;  return "OK"; }
    if (cmd == "ATH0") { headersOn_= false; return "OK"; }
    if (cmd == "ATH1") { headersOn_= true;  return "OK"; }
    if (cmd == "ATAL")                                   return "OK";
    if (cmd == "ATM0" || cmd == "ATM1")                  return "OK";
    if (cmd == "ATAT0"|| cmd == "ATAT1"|| cmd == "ATAT2")return "OK";

    // ATSP — set protocol
    if (cmd.size() >= 4 && cmd.substr(0,4) == "ATSP")   return "OK";

    // ATST — set timeout (ignored)
    if (cmd.size() >= 4 && cmd.substr(0,4) == "ATST")   return "OK";

    // ATSH — set header address (store for Mode 22 routing)
    if (cmd.size() >= 4 && cmd.substr(0,4) == "ATSH") {
        currentEcuAddr_ = cmd.substr(4);
        return "OK";
    }

    // ATDP — describe protocol
    if (cmd == "ATDP")
        return profile_->getProtocolDesc();

    // STI / VTI — adapter identity (CarScanner uses these to identify OBDLink)
    if (cmd == "STI") return "STN1110 v4.2.0";
    if (cmd == "VTI") return "OBDLink v1.0";

    // Unrecognised AT command
    if (cmd.size() >= 2 && cmd[0] == 'A' && cmd[1] == 'T') return "OK";
    if (cmd.size() >= 2 && cmd[0] == 'S' && cmd[1] == 'T') return "OK";

    // =========================================================================
    //  OBD Mode 01 — Standard SAE J1979
    // =========================================================================
    if (cmd.size() == 4 && cmd.substr(0,2) == "01")
        return handleMode01(cmd.substr(2));

    // =========================================================================
    //  OBD Mode 09 — Vehicle information
    // =========================================================================
    if (cmd.size() == 4 && cmd.substr(0,2) == "09")
        return handleMode09(cmd.substr(2));

    // =========================================================================
    //  Mode 22 — Ford UDS proprietary PIDs
    // =========================================================================
    if (cmd.size() == 6 && cmd.substr(0,2) == "22")
        return handleMode22(cmd.substr(2));

    // =========================================================================
    //  Everything else
    // =========================================================================
    return "NO DATA";
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 01
// ─────────────────────────────────────────────────────────────────────────────
std::string ELM327Handler::handleMode01(const std::string& pid)
{
    const auto& e     = sim_->getState();
    const auto& masks = profile_->getPidBitmasks();
    const std::string& ecu = profile_->getEcuResponse();

    // ── PID support bitmasks (discovery) ─────────────────────────────────────
    auto respondMask = [&](int base) -> std::string {
        auto it = masks.find(base);
        if (it == masks.end()) return "NO DATA";
        uint32_t m = it->second;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%02X %02X %02X %02X",
            (m >> 24) & 0xFF, (m >> 16) & 0xFF, (m >> 8) & 0xFF, m & 0xFF);
        char hdr[16];
        std::snprintf(hdr, sizeof(hdr), "%s 06", ecu.c_str());
        std::string resp = "41 " + pid + " " + buf;
        if (!isHeartbeatPid(pid))
            std::cout << "  [TX] (mask " << pid << ")\n";
        return obdLine(hdr, resp);
    };
    if (pid == "00") return respondMask(0x00);
    if (pid == "20") return respondMask(0x20);
    if (pid == "40") return respondMask(0x40);
    if (pid == "60") return respondMask(0x60);
    if (pid == "80") return respondMask(0x80);
    if (pid == "A0") return respondMask(0xA0);
    if (pid == "C0") return respondMask(0xC0);

    // Helper lambdas for building responses
    auto r3 = [&](const std::string& d) { // 1 data byte
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 03", ecu.c_str());
        return obdLine(hdr, "41 " + pid + " " + d);
    };
    auto r4 = [&](const std::string& d) { // 2 data bytes
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 04", ecu.c_str());
        return obdLine(hdr, "41 " + pid + " " + d);
    };
    auto r5 = [&](const std::string& d) { // 3 data bytes
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 05", ecu.c_str());
        return obdLine(hdr, "41 " + pid + " " + d);
    };
    auto r6 = [&](const std::string& d) { // 4 data bytes
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 06", ecu.c_str());
        return obdLine(hdr, "41 " + pid + " " + d);
    };

    // ── Status / monitor ─────────────────────────────────────────────────────
    if (pid == "01") return r6("00 07 65 00");   // no MIL, all monitors complete

    // ── Mode 01 data PIDs ────────────────────────────────────────────────────
    if (pid == "04") return r3(encPct(e.load));
    if (pid == "05") return r3(encTemp(e.coolant));
    if (pid == "06") return r3(encSpct(e.stft));
    if (pid == "07") return r3(encSpct(e.ltft));
    if (pid == "0B") return r3(encKpa1(e.map_kpa));
    if (pid == "0C") return r4(encRpm());
    if (pid == "0D") return r3(h8((int)e.speed));
    if (pid == "0E") return r3(encTiming());
    if (pid == "0F") return r3(encTemp(e.intake));
    if (pid == "10") return r4(encMaf());
    if (pid == "11") return r3(encPct(e.throttle));

    // O2 sensors — A=voltage, B=0xFF (fuel trim not used on this bank)
    if (pid == "14") return r4(encO2(e.o2bank1) + " FF");
    if (pid == "15") return r4(encO2(e.o2bank2) + " FF");

    // Run time since engine start
    if (pid == "1F") {
        int rt = (int)e.runTime;
        return r4(h16(rt));
    }

    if (pid == "2C") return r3(encPct(e.egrDuty));   // EGR commanded (0% — NA on 1.4)
    if (pid == "2F") return r3(encPct(e.fuelLevel));
    if (pid == "33") return r3(encKpa1(e.baro));
    if (pid == "42") return r4(encVolt());
    if (pid == "46") return r3(encTemp(e.ambientTemp));

    return "NO DATA";
}


// ─────────────────────────────────────────────────────────────────────────────
//  Mode 09 — Vehicle information
// ─────────────────────────────────────────────────────────────────────────────
std::string ELM327Handler::handleMode09(const std::string& pid)
{
    const std::string& ecu = profile_->getEcuResponse();
    const std::string  vin = profile_->getVIN();

    // VIN — technically multi-frame but we send simplified single-frame
    // CarScanner just needs a valid-looking response to stop probing
    if (pid == "02") {
        // Response: 49 02 01 [17 ASCII VIN bytes]
        // Encode first 6 chars of VIN as hex for a plausible response
        std::string resp = "49 02 01";
        for (size_t i = 0; i < std::min(vin.size(), (size_t)6); i++) {
            char b[4]; std::snprintf(b, sizeof(b), " %02X", (unsigned char)vin[i]);
            resp += b;
        }
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 10 14", ecu.c_str());
        return obdLine(hdr, resp);
    }
    if (pid == "04") {  // Calibration ID
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 05", ecu.c_str());
        return obdLine(hdr, "49 04 00 00 00 00");
    }
    if (pid == "0A") {  // ECU name
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 04", ecu.c_str());
        return obdLine(hdr, "49 0A 00 00 00");
    }
    return "NO DATA";
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 22 — Ford UDS proprietary PIDs
// ─────────────────────────────────────────────────────────────────────────────
std::string ELM327Handler::handleMode22(const std::string& addr)
{
    const auto& e   = sim_->getState();
    const auto& ecu = profile_->getEcuResponse();

    // Verify this PID is in the vehicle's Mode 22 list
    const auto& m22 = profile_->getMode22Pids();
    if (m22.find(addr) == m22.end()) {
        // Unknown PID — return proper UDS negative response
        // "NO DATA" = timeout → app retries forever
        // "7F 22 11" = serviceNotSupported → app accepts and moves on
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 03", ecu.c_str());
        return obdLine(hdr, "7F 22 11");
    }

    // UDS positive response prefix: 62 + 2-byte PID address
    // e.g. for addr "1310": "62 13 10 XX XX"
    std::string pidBytes;
    if (addr.size() == 4) {
        pidBytes = addr.substr(0,2) + " " + addr.substr(2,2);
    }

    // Helper lambdas for 22 responses
    auto u4 = [&](const std::string& d) {   // 3 data bytes (len=5)
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 05", ecu.c_str());
        return obdLine(hdr, "62 " + pidBytes + " " + d);
    };
    auto u3 = [&](const std::string& d) {   // 2 data bytes (len=4)
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 04", ecu.c_str());
        return obdLine(hdr, "62 " + pidBytes + " " + d);
    };
    auto u5 = [&](const std::string& d) {   // 4 data bytes (len=6)
        char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 06", ecu.c_str());
        return obdLine(hdr, "62 " + pidBytes + " " + d);
    };

    // ── PCM internal PIDs ─────────────────────────────────────────────────────

    // 1310 — Engine Oil Temp  formula: (A*256+B)/100 °C
    if (addr == "1310") {
        int v = (int)(e.oilTemp * 100.0f);
        return u4(h16(v));
    }
    // 1131 — Ignition Timing  formula: (A*256+B)/100 - 64 °
    if (addr == "1131") {
        int v = (int)((e.timing + 64.0f) * 100.0f);
        return u4(h16(v));
    }
    // 1440 — MAP Extended  formula: (A*256+B)/100 kPa
    if (addr == "1440") {
        int v = (int)(e.map_kpa * 100.0f);
        return u4(h16(v));
    }
    // 113C — EGR Duty Cycle  formula: A*100/255 %
    if (addr == "113C") {
        return u4(h8((int)(e.egrDuty * 255.0f / 100.0f)) + " 00");
    }
    // 160E-1611 — Misfire counters (0 at idle — no misfires)
    if (addr == "160E" || addr == "160F" || addr == "1610" || addr == "1611") {
        return u4("00 00");
    }

    // ── Ford F4xx extended PIDs (mirror of Mode 01 data) ─────────────────────

    // F405 — Coolant Temp (Ford)  formula: A-40
    if (addr == "F405") return u3(encTemp(e.coolant));
    // F40F — Intake Air Temp (Ford)  formula: A-40
    if (addr == "F40F") return u3(encTemp(e.intake));
    // F40B — MAP (Ford)  formula: A kPa
    if (addr == "F40B") return u3(encKpa1(e.map_kpa));
    // F433 — Barometric Pressure (Ford)  formula: A kPa
    if (addr == "F433") return u3(encKpa1(e.baro));
    // F406 — STFT (Ford)  formula: A/1.28-100 %
    if (addr == "F406") return u3(encSpct(e.stft));
    // F407 — LTFT (Ford)  formula: A/1.28-100 %
    if (addr == "F407") return u3(encSpct(e.ltft));
    // F443 — Engine Load (Ford)  formula: (A*256+B)/100 %
    if (addr == "F443") {
        int v = (int)(e.load * 100.0f);
        return u4(h16(v));
    }
    // F434 — Lambda  formula: (A*256+B)/32768  (stoich = 0x8000)
    if (addr == "F434") {
        int v = (int)(1.0f * 32768.0f);  // stoichiometric
        return u5(h16(v) + " 00 00");
    }
    // F40C — RPM (Ford)  formula: (A*256+B)/4 RPM
    if (addr == "F40C") return u4(encRpm());
    // F40D — Speed (Ford)  formula: A km/h
    if (addr == "F40D") return u3(h8((int)e.speed));

    // Should never reach here (we checked the map above) but just in case
    char hdr[16]; std::snprintf(hdr, sizeof(hdr), "%s 03", ecu.c_str());
    return obdLine(hdr, "7F 22 11");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Response formatting
// ─────────────────────────────────────────────────────────────────────────────
std::string ELM327Handler::nosp(const std::string& s)
{
    std::string r;
    r.reserve(s.size());
    for (char c : s) if (c != ' ') r += c;
    return r;
}

std::string ELM327Handler::obdLine(const std::string& hdr,
                                    const std::string& payload) const
{
    // hdr     = "7E8 06"         (address space length byte)
    // payload = "41 00 1E 3F ..."
    if (!headersOn_) {
        return spacesOn_ ? payload : nosp(payload);
    }
    std::string full = hdr + " " + payload;
    return spacesOn_ ? full : nosp(full);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Byte encoding helpers
// ─────────────────────────────────────────────────────────────────────────────
std::string ELM327Handler::h8(int v) const
{
    char b[3]; std::snprintf(b, sizeof(b), "%02X", v & 0xFF);
    return std::string(b);
}

std::string ELM327Handler::h16(int v) const
{
    char b[6]; std::snprintf(b, sizeof(b), "%02X %02X",
        (v >> 8) & 0xFF, v & 0xFF);
    return std::string(b);
}

std::string ELM327Handler::h32(int64_t v) const
{
    char b[12]; std::snprintf(b, sizeof(b), "%02X %02X %02X %02X",
        (int)((v >> 24) & 0xFF), (int)((v >> 16) & 0xFF),
        (int)((v >>  8) & 0xFF), (int)(v & 0xFF));
    return std::string(b);
}

std::string ELM327Handler::encTemp(float c) const
{
    return h8((int)(c + 40.0f));
}

std::string ELM327Handler::encPct(float p) const
{
    int v = (int)std::round(std::max(0.0f, std::min(100.0f, p)) * 255.0f / 100.0f);
    return h8(v);
}

std::string ELM327Handler::encSpct(float p) const
{
    // Signed percent: A/1.28 - 100   →   encode: (p+100)*1.28
    int v = (int)std::round((p + 100.0f) * 1.28f);
    v = std::max(0, std::min(255, v));
    return h8(v);
}

std::string ELM327Handler::encKpa1(float k) const
{
    return h8((int)std::round(k));
}

std::string ELM327Handler::encRpm() const
{
    int v = (int)(sim_->getState().rpm * 4.0f);
    return h16(v);
}

std::string ELM327Handler::encMaf() const
{
    int v = (int)(sim_->getState().maf * 100.0f);
    return h16(v);
}

std::string ELM327Handler::encVolt() const
{
    int v = (int)(sim_->getState().voltage * 1000.0f);
    return h16(v);
}

std::string ELM327Handler::encTiming() const
{
    // A/2 - 64  →  encode: (timing+64)*2
    int v = (int)std::round((sim_->getState().timing + 64.0f) * 2.0f);
    return h8(v);
}

std::string ELM327Handler::encO2(float v) const
{
    // A*0.005V  →  encode: v/0.005
    int enc = (int)std::round(v / 0.005f);
    enc = std::max(0, std::min(255, enc));
    return h8(enc);
}
