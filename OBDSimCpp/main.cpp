#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <atomic>
#include <string>

#include "FordFiesta2006.h"
#include "EngineSimulator.h"
#include "ELM327Handler.h"
#include "BluetoothServer.h"

// Global server pointer so Ctrl+C handler can stop it cleanly
static BluetoothServer* g_server = nullptr;

void onSignal(int)
{
    std::cout << "\n[!] Shutting down...\n";
    if (g_server) g_server->stop();
}

int main()
{
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::unique_ptr<VehicleProfile> car =
        std::make_unique<FordFiesta2006>();

    std::cout << "================================================\n";
    std::cout << "  OBD2 Simulator - " << car->getDisplayName() << "\n";
    std::cout << "  Protocol : " << car->getProtocolDesc() << "\n";
    std::cout << "  VIN      : " << car->getVIN() << "\n";
    std::cout << "================================================\n\n";

    EngineSimulator sim(car->getEngineParams());

    // ✅ FORCE START MODE = IDLE
    sim.setMode(EngineSimulator::Mode::Idle);

    ELM327Handler handler(car.get(), &sim);

    BluetoothServer server("OBD2_" + car->getMake(), &handler, &sim);
    g_server = &server;

    if (!server.init())
    {
        std::cerr << "\n[!] Failed to initialize Bluetooth server.\n";
        return 1;
    }

    std::cout << "Bluetooth server ready.\n";
    std::cout << "Device name: OBD2_" << car->getMake() << "\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    // ============================
    // MODE SWITCH THREAD (FIXED)
    // ============================
    std::atomic<bool> running{ true };

    std::thread inputThread([&]()
        {
            while (running)
            {
                std::cout << "\n[MODE] 1 = IDLE | 2 = CRUISE > ";
                std::string input;
                std::getline(std::cin, input);

                if (input == "1")
                {
                    sim.setMode(EngineSimulator::Mode::Idle);
                    std::cout << "IDLE MODE ACTIVE\n";
                }
                else if (input == "2")
                {
                    sim.setMode(EngineSimulator::Mode::Cruise);
                    std::cout << "CRUISE MODE ACTIVE\n";
                }
            }
        });

    server.run();

    running = false;
    inputThread.detach();

    std::cout << "Simulator stopped.\n";
    return 0;
}