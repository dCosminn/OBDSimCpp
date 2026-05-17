#pragma once
// =============================================================================
//  DTCEntry.h  -  Plain DTC data struct shared between VehicleProfile
//                 and DTCManager.
//
//  Kept separate so VehicleProfile.h does not need to include DTCManager.h
//  and vice versa.
//
//  DTC byte encoding (SAE J2012):
//    P0xxx -> byte1 = 0x0y, byte2 = xx   (P0yxx)
//    e.g. P0300 -> 0x03 0x00
//         P0171 -> 0x01 0x71
//         P0420 -> 0x04 0x20
// =============================================================================
#include <string>
#include <cstdint>

struct DTCEntry {
    std::string code;   // human-readable, e.g. "P0300"
    std::string desc;   // description shown on console
    uint8_t     b1;     // encoded high byte
    uint8_t     b2;     // encoded low byte
};

// Helper used by every vehicle profile to build its DTC pool.
// Encodes a "P0300"-style code string into the two OBD bytes.
inline DTCEntry makeDTC(const std::string& code, const std::string& desc)
{
    // byte1: system | (type << 4) | hundreds
    //   P=0x00, C=0x40, B=0x80, U=0xC0
    //   type digit  (0-3) -> bits 4-5
    //   hundreds digit    -> bits 0-3
    // byte2: tens*16 + units

    uint8_t sys = 0;
    if      (code[0] == 'P') sys = 0x00;
    else if (code[0] == 'C') sys = 0x40;
    else if (code[0] == 'B') sys = 0x80;
    else                     sys = 0xC0;

    uint8_t type     = (uint8_t)(code[1] - '0') & 0x03;   // 0-3
    uint8_t hundreds = (uint8_t)(code[2] - '0') & 0x0F;
    uint8_t tens     = (uint8_t)(code[3] - '0') & 0x0F;
    uint8_t units    = (uint8_t)(code[4] - '0') & 0x0F;

    uint8_t b1 = sys | (type << 4) | hundreds;
    uint8_t b2 = (tens << 4) | units;

    return { code, desc, b1, b2 };
}