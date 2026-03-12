// =============================================================================
//  main.cpp  —  OBD2 Simulator for Windows
//
//  To switch cars: change the VehicleProfile line below.
//  To add a new car: create MyNewCar.h/.cpp, inherit VehicleProfile, done.
// =============================================================================
#include <iostream>
#include <memory>
#include <csignal>

#include "FordFiesta2006.h"
#include "EngineSimulator.h"
#include "ELM327Handler.h"
#include "BluetoothServer.h"

// Global server pointer so Ctrl+C handler can stop it cleanly
static BluetoothServer* g_server = nullptr;

void onSignal(int) {
    std::cout << "\n[!] Shutting down...\n";
    if (g_server) g_server->stop();
}

int main()
{
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // ── Select vehicle profile ────────────────────────────────────────────────
    // To simulate a different car, replace this with another VehicleProfile
    std::unique_ptr<VehicleProfile> car = std::make_unique<FordFiesta2006>();

    std::cout << "================================================\n";
    std::cout << "  OBD2 Simulator - " << car->getDisplayName() << "\n";
    std::cout << "  Protocol : " << car->getProtocolDesc() << "\n";
    std::cout << "  VIN      : " << car->getVIN() << "\n";
    std::cout << "================================================\n\n";

    // ── Engine simulator (shared physics engine) ──────────────────────────────
    EngineSimulator sim(car->getEngineParams());

    // ── ELM327 protocol handler ───────────────────────────────────────────────
    ELM327Handler handler(car.get(), &sim);

    // ── Bluetooth server ──────────────────────────────────────────────────────
    // The name shown in Android's Bluetooth scan list
    BluetoothServer server("OBD2_" + car->getMake(), &handler, &sim);
    g_server = &server;

    if (!server.init()) {
        std::cerr << "\n[!] Failed to initialize Bluetooth server.\n";
        std::cerr << "    - Is Bluetooth enabled in Windows Settings?\n";
        std::cerr << "    - Try running as Administrator.\n";
        return 1;
    }

    std::cout << "Bluetooth server ready.\n";
    std::cout << "Device name: OBD2_" << car->getMake() << "\n";
    std::cout << "Pair this PC with your phone, then connect from the app.\n\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    // Blocking — returns when stop() is called or a fatal BT error occurs
    server.run();

    std::cout << "Simulator stopped.\n";
    return 0;
}