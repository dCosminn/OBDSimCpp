#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <csignal>
#include <thread>
#include <atomic>
#include <string>

#include "FordFiesta2006.h"
#include "EngineSimulator.h"
#include "ELM327Handler.h"
#include "BluetoothServer.h"
#include "DTCManager.h"

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

    // DTCManager is seeded with this car's fault pool
    DTCManager dtcManager(car->getDtcPool());

    ELM327Handler handler(car.get(), &sim, &dtcManager);

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
    // MODE SWITCH THREAD
    // ============================
    std::atomic<bool> running{ true };

    std::thread inputThread([&]()
        {
            while (running)
            {
                std::cout << "[MODE] 1=IDLE | 2=CRUISE | 3=SET FAULTS | 4=CLEAR FAULTS\n ";
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
                else if (input == "3")
                {
                    // Print numbered list of all available DTCs for this car
                    const auto& pool = dtcManager.getPool();
                    std::cout << "\nAvailable faults:\n";
                    for (int i = 0; i < (int)pool.size(); i++)
                        std::cout << "  " << std::setw(2) << (i + 1) << ". "
                        << pool[i].code << "  " << pool[i].desc << "\n";

                    std::cout << "\nEnter numbers separated by spaces (e.g. 1 5 12), or 0 for random:\n";
                    std::string line;
                    std::getline(std::cin, line);

                    std::istringstream ss(line);
                    int n;
                    std::vector<int> picks;
                    while (ss >> n) picks.push_back(n);

                    if (picks.size() == 1 && picks[0] == 0)
                        dtcManager.randomizeFaults();
                    else
                        dtcManager.setFaults(picks);
                }
                else if (input == "4")
                {
                    dtcManager.clearFaults();
                }
            }
        });

    server.run();

    running = false;
    inputThread.detach();

    std::cout << "Simulator stopped.\n";
    return 0;
}