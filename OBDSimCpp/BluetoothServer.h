#pragma once
// =============================================================================
//  BluetoothServer.h  —  Windows Classic Bluetooth RFCOMM server (SPP)
// =============================================================================
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>   // must come before ws2bth.h
#include <ws2bth.h>     // SOCKADDR_BTH, BTH_ADDR, AF_BTH, BTHPROTO_RFCOMM etc.

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "bthprops.lib")

#include <string>
#include "ELM327Handler.h"
#include "EngineSimulator.h"

class BluetoothServer {
public:
    BluetoothServer(const std::string& deviceName,
        ELM327Handler* handler,
        EngineSimulator* sim);
    ~BluetoothServer();

    bool init();   // set up socket + register SDP service
    void run();    // blocking accept loop
    void stop();   // signal run() to exit

private:
    std::string      name_;
    ELM327Handler* handler_;
    EngineSimulator* sim_;

    SOCKET  serverSock_ = INVALID_SOCKET;
    SOCKET  clientSock_ = INVALID_SOCKET;
    bool    running_ = false;
    ULONG   channel_ = 0;

    WSAQUERYSET* sdpRecord_ = nullptr;

    bool registerSdpService(ULONG channel);
    void deregisterSdpService();
    void handleClient();
    bool sendResponse(const std::string& response);

    static void printBanner(const std::string& text);
    static void printBox(const std::string& text);
};