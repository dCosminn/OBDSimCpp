// =============================================================================
//  FordFiesta2006.cpp
// =============================================================================
#include "FordFiesta2006.h"

EngineParams FordFiesta2006::getEngineParams() const
{
    EngineParams p;
    p.idleRpm = 800.0f;
    p.idleRpmVariance = 40.0f;

    p.normalCoolant = 89.0f;
    p.normalOilTemp = 86.0f;
    p.normalIntake = 24.0f;
    p.normalBaro = 101.0f;
    p.normalFuelLevel = 65.0f;
    p.normalVoltage = 14.057f;
    p.normalMAF = 2.65f;
    p.normalLoad = 21.0f;

    // ✅ CRUISE VALUES (MOVED HERE)
    p.cruiseSpeed = 80.0f;
    p.cruiseRpm = 2250.0f;
    p.cruiseLoad = 35.0f;
    p.cruiseMAF = 11.5f;
    p.cruiseThrottle = 20.0f;
    p.cruiseMAP = 50.0f;

    p.cylinders = 4;
    p.isDiesel = false;

    return p;
}

std::map<int, uint32_t> FordFiesta2006::getPidBitmasks() const
{
    // Bitmask encoding:
    //   Bit 31 (MSB) = PID base+1 supported
    //   Bit 0  (LSB) = PID base+32 supported AND next range also exists
    //
    // Range 0x00 — covers PIDs 0x01–0x20
    //   Supported: 04 05 06 07 0B 0C 0D 0E 0F 10 11 14 15 1F
    //   + bit 0 set → range 0x20 also present
    //   Byte A (0x01-0x08): 04→bit4, 05→bit3, 06→bit2, 07→bit1  = 0x1E
    //   Byte B (0x09-0x10): 0B→bit5, 0C→bit4, 0D→bit3, 0E→bit2, 0F→bit1, 10→bit0 = 0x3F
    //   Byte C (0x11-0x18): 11→bit7, 14→bit4, 15→bit3 = 0x98
    //   Byte D (0x19-0x20): 1F→bit1, 20(next range)→bit0 = 0x03
    //
    // Range 0x20 — covers PIDs 0x21–0x40
    //   Supported: 2C 2F 33
    //   + bit 0 set → range 0x40 also present
    //   Byte A (0x21-0x28): none = 0x00
    //   Byte B (0x29-0x30): 2C→bit4, 2F→bit1 = 0x12
    //   Byte C (0x31-0x38): 33→bit5 = 0x20
    //   Byte D (0x39-0x40): 40(next range)→bit0 = 0x01
    //
    // Range 0x40 — covers PIDs 0x41–0x60
    //   Supported: 42 46
    //   Byte A (0x41-0x48): 42→bit6, 46→bit2 = 0x44
    //   Bytes B,C,D: none, no next range = 0x00
    return {
        { 0x00, 0x1E3F9803 },
        { 0x20, 0x00122001 },
        { 0x40, 0x44000004 },
    };
}

std::map<std::string, std::string> FordFiesta2006::getMode22Pids() const
{
    // All addresses are 2-byte UDS PIDs sent to ECU at 7E0
    // Formula notes are used by ELM327Handler to encode responses correctly
    return {
        // PCM confirmed PIDs on 1.4 Duratec petrol
        { "1310", "Engine Oil Temperature"     },  // (A*256+B)/100 °C
        { "1131", "Ignition Timing"            },  // (A*256+B)/100 - 64 °
        { "1440", "MAP Extended"               },  // (A*256+B)/100 kPa
        { "113C", "EGR Duty Cycle"             },  // A*100/255 %
        { "1172", "Battery Voltage"             },  // A/16 V
        { "125D", "Fuel Flow Rate"              },  // (A*256+B)/100 L/h

        // Per-cylinder misfire counters
        { "160E", "Misfire Count Cyl 1"        },  // A*256+B
        { "160F", "Misfire Count Cyl 2"        },
        { "1610", "Misfire Count Cyl 3"        },
        { "1611", "Misfire Count Cyl 4"        },

        // Ford extended F4xx PIDs (same data as Mode 01 but Ford address space)
        { "F405", "Coolant Temp (Ford)"        },  // A-40 °C
        { "F40F", "Intake Air Temp (Ford)"     },  // A-40 °C
        { "F40B", "MAP (Ford)"                 },  // A kPa
        { "F433", "Barometric Pressure (Ford)" },  // A kPa
        { "F406", "Short Term Fuel Trim (Ford)"},  // A/1.28-100 %
        { "F407", "Long Term Fuel Trim (Ford)" },  // A/1.28-100 %
        { "F443", "Engine Load (Ford)"         },  // (A*256+B)/100 %
        { "F434", "Lambda (Ford)"              },  // (A*256+B)/32768
        { "F40C", "RPM (Ford)"                 },  // (A*256+B)/4 RPM
        { "F40D", "Vehicle Speed (Ford)"       },  // A km/h
    };
}