// =============================================================================
//  BluetoothServer.cpp  --  Windows Classic Bluetooth RFCOMM SPP server
// =============================================================================
#include "BluetoothServer.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>

static const GUID SPP_UUID = {
    0x00001101, 0x0000, 0x1000,
    { 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB }
};

BluetoothServer::BluetoothServer(const std::string& deviceName,
    ELM327Handler* handler,
    EngineSimulator* sim)
    : name_(deviceName), handler_(handler), sim_(sim)
{
}

BluetoothServer::~BluetoothServer()
{
    stop();
    deregisterSdpService();
    if (clientSock_ != INVALID_SOCKET) { closesocket(clientSock_); clientSock_ = INVALID_SOCKET; }
    if (serverSock_ != INVALID_SOCKET) { closesocket(serverSock_); serverSock_ = INVALID_SOCKET; }
    WSACleanup();
    delete sdpRecord_;
}

bool BluetoothServer::init()
{
    std::cout << "[BT] Initializing Winsock...\n";
    WSADATA wsa;
    int wsaErr = WSAStartup(MAKEWORD(2, 2), &wsa);
    if (wsaErr != 0) {
        std::cerr << "[BT] WSAStartup failed, error: " << wsaErr << "\n";
        return false;
    }
    std::cout << "[BT] Winsock OK (v" << (int)LOBYTE(wsa.wVersion)
        << "." << (int)HIBYTE(wsa.wVersion) << ")\n";

    std::cout << "[BT] Creating RFCOMM socket (AF_BTH=" << AF_BTH
        << ", BTHPROTO_RFCOMM=" << BTHPROTO_RFCOMM << ")...\n";
    serverSock_ = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
    if (serverSock_ == INVALID_SOCKET) {
        std::cerr << "[BT] socket() FAILED: " << WSAGetLastError() << "\n";
        std::cerr << "[BT] Is Bluetooth enabled in Windows Settings?\n";
        WSACleanup();
        return false;
    }
    std::cout << "[BT] Socket created OK (handle=" << (int)serverSock_ << ")\n";

    // Try channels 1-6 in order — Windows may have claimed some already
    SOCKADDR_BTH addr = {};
    addr.addressFamily = AF_BTH;
    bool bound = false;
    for (ULONG ch = 1; ch <= 6; ch++) {
        addr.port = ch;
        std::cout << "[BT] Trying RFCOMM channel " << ch << "...\n";
        if (bind(serverSock_, (SOCKADDR*)&addr, sizeof(addr)) == 0) {
            std::cout << "[BT] Bound OK on channel " << ch << "\n";
            bound = true;
            break;
        }
        int err = WSAGetLastError();
        std::cout << "[BT] Channel " << ch << " failed: " << err;
        if (err == 10013) std::cout << " (WSAEACCES - in use)";
        if (err == 10048) std::cout << " (WSAEADDRINUSE - in use)";
        std::cout << "\n";
    }
    if (!bound) {
        // Last resort: let Windows pick any free channel
        std::cout << "[BT] All fixed channels busy, trying BT_PORT_ANY...\n";
        addr.port = BT_PORT_ANY;
        if (bind(serverSock_, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            std::cerr << "[BT] bind() FAILED on all channels: " << WSAGetLastError() << "\n";
            closesocket(serverSock_); serverSock_ = INVALID_SOCKET;
            WSACleanup();
            return false;
        }
        std::cout << "[BT] Bound on dynamic channel.\n";
    }

    int addrLen = sizeof(addr);
    getsockname(serverSock_, (SOCKADDR*)&addr, &addrLen);
    channel_ = addr.port;
    std::cout << "[BT] Bound OK. RFCOMM channel = " << channel_ << "\n";

    std::cout << "[BT] Starting listen()...\n";
    if (listen(serverSock_, 1) == SOCKET_ERROR) {
        std::cerr << "[BT] listen() FAILED: " << WSAGetLastError() << "\n";
        closesocket(serverSock_); serverSock_ = INVALID_SOCKET;
        WSACleanup();
        return false;
    }
    std::cout << "[BT] Listening for incoming connections.\n";

    std::cout << "[BT] Registering SDP service...\n";
    if (!registerSdpService(channel_)) {
        std::cerr << "[BT] SDP registration FAILED.\n";
        std::cerr << "[BT] The phone will NOT find the service via UUID.\n";
        std::cerr << "[BT] Try: run this program as Administrator.\n";
        std::cerr << "[BT] Continuing anyway...\n";
    }

    return true;
}

bool BluetoothServer::registerSdpService(ULONG channel)
{
    SOCKADDR_BTH addrLocal = {};
    addrLocal.addressFamily = AF_BTH;
    addrLocal.port = channel;

    CSADDR_INFO csAddr = {};
    csAddr.LocalAddr.lpSockaddr = (SOCKADDR*)&addrLocal;
    csAddr.LocalAddr.iSockaddrLength = sizeof(addrLocal);
    csAddr.RemoteAddr.lpSockaddr = (SOCKADDR*)&addrLocal;
    csAddr.RemoteAddr.iSockaddrLength = sizeof(addrLocal);
    csAddr.iSocketType = SOCK_STREAM;
    csAddr.iProtocol = BTHPROTO_RFCOMM;

    std::wstring wname(name_.begin(), name_.end());

    sdpRecord_ = new WSAQUERYSET{};
    sdpRecord_->dwSize = sizeof(WSAQUERYSET);
    sdpRecord_->lpszServiceInstanceName = const_cast<LPWSTR>(wname.c_str());
    sdpRecord_->lpServiceClassId = const_cast<LPGUID>(&SPP_UUID);
    sdpRecord_->dwNameSpace = NS_BTH;
    sdpRecord_->dwNumberOfCsAddrs = 1;
    sdpRecord_->lpcsaBuffer = &csAddr;

    std::cout << "[BT] WSASetService RNRSERVICE_REGISTER on channel " << channel << "...\n";
    if (WSASetService(sdpRecord_, RNRSERVICE_REGISTER, 0) == SOCKET_ERROR) {
        int err = WSAGetLastError();
        std::cerr << "[BT] WSASetService FAILED: " << err << "\n";
        if (err == 10022) std::cerr << "[BT] Error 10022 = WSAEINVAL -- try running as Administrator\n";
        if (err == 10013) std::cerr << "[BT] Error 10013 = WSAEACCES -- need Administrator privileges\n";
        delete sdpRecord_;
        sdpRecord_ = nullptr;
        return false;
    }
    std::cout << "[BT] SDP registered OK. Service: \"" << name_ << "\"\n";
    std::cout << "[BT] SPP UUID: {00001101-0000-1000-8000-00805F9B34FB}\n";
    return true;
}

void BluetoothServer::deregisterSdpService()
{
    if (sdpRecord_) {
        std::cout << "[BT] Deregistering SDP service...\n";
        WSASetService(sdpRecord_, RNRSERVICE_DELETE, 0);
        delete sdpRecord_;
        sdpRecord_ = nullptr;
    }
}

void BluetoothServer::run()
{
    running_ = true;
    printBanner("OBD2 Simulator -- " + handler_->getProfileName());
    std::cout << "Waiting for incoming Bluetooth connection...\n";
    std::cout << "Make sure the phone is paired with this PC in Windows Settings.\n";
    std::cout << "Then connect from the app (select this PC from device list).\n\n";

    int selectLoops = 0;

    while (running_) {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(serverSock_, &readSet);
        timeval tv = { 1, 0 };  // 1 second

        int sel = select(0, &readSet, nullptr, nullptr, &tv);
        if (sel == SOCKET_ERROR) {
            int err = WSAGetLastError();
            if (!running_) break;
            std::cerr << "[BT] select() error: " << err << "\n";
            break;
        }
        if (sel == 0) {
            // Heartbeat every 10 seconds so user knows we're still alive
            selectLoops++;
            if (selectLoops % 10 == 0)
                std::cout << "[BT] Still waiting... (" << selectLoops << "s)\n";
            continue;
        }

        std::cout << "[BT] Incoming connection detected -- calling accept()...\n";
        SOCKADDR_BTH clientAddr = {};
        int clientAddrLen = sizeof(clientAddr);
        clientSock_ = accept(serverSock_, (SOCKADDR*)&clientAddr, &clientAddrLen);
        if (clientSock_ == INVALID_SOCKET) {
            int err = WSAGetLastError();
            if (!running_) break;
            std::cerr << "[BT] accept() FAILED: " << err << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Format client BT address for display
        char addrBuf[40] = {};
        BTH_ADDR bt = clientAddr.btAddr;
        std::snprintf(addrBuf, sizeof(addrBuf), "%02X:%02X:%02X:%02X:%02X:%02X",
            (int)((bt >> 40) & 0xFF), (int)((bt >> 32) & 0xFF),
            (int)((bt >> 24) & 0xFF), (int)((bt >> 16) & 0xFF),
            (int)((bt >> 8) & 0xFF), (int)(bt & 0xFF));

        printBox("APP CONNECTED  " + std::string(addrBuf));
        std::cout << "[BT] Client socket handle: " << (int)clientSock_ << "\n";
        std::cout << "[BT] Starting command loop...\n\n";

        handler_->resetSession();
        handleClient();

        closesocket(clientSock_);
        clientSock_ = INVALID_SOCKET;
        selectLoops = 0;
        printBox("DISCONNECTED");
        std::cout << "Waiting for next connection...\n\n";
    }

    std::cout << "[BT] Server loop exited.\n";
}

void BluetoothServer::stop()
{
    running_ = false;
    if (serverSock_ != INVALID_SOCKET) {
        closesocket(serverSock_);
        serverSock_ = INVALID_SOCKET;
    }
}

void BluetoothServer::handleClient()
{
    std::string buf;
    char        recvBuf[256];
    auto lastEngineUpdate = std::chrono::steady_clock::now();
    int  cmdCount = 0;

    while (running_) {
        // Update engine every 20ms
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEngineUpdate).count() >= 20) {
            sim_->update();
            lastEngineUpdate = now;
        }

        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(clientSock_, &readSet);
        timeval tv = { 0, 20000 };  // 20ms

        int sel = select(0, &readSet, nullptr, nullptr, &tv);
        if (sel == SOCKET_ERROR) {
            std::cerr << "[BT] select() error in handleClient: " << WSAGetLastError() << "\n";
            break;
        }
        if (sel == 0) continue;

        int bytes = recv(clientSock_, recvBuf, sizeof(recvBuf) - 1, 0);
        if (bytes == 0) {
            std::cout << "[BT] Client disconnected cleanly.\n";
            break;
        }
        if (bytes < 0) {
            std::cerr << "[BT] recv() error: " << WSAGetLastError() << "\n";
            break;
        }

        // Log raw bytes received (first 50 commands only to avoid spam)
        if (cmdCount < 50) {
            recvBuf[bytes] = 0;
            std::string raw(recvBuf, bytes);
            std::string escaped;
            for (char c : raw) {
                if (c == '\r') escaped += "\\r";
                else if (c == '\n') escaped += "\\n";
                else escaped += c;
            }
            std::cout << "[BT] RAW recv (" << bytes << " bytes): [" << escaped << "]\n";
        }

        for (int i = 0; i < bytes; i++) {
            char c = recvBuf[i];
            if (c == '\r' || c == '\n') {
                if (!buf.empty()) {
                    cmdCount++;
                    std::string response = handler_->handle(buf);
                    if (!sendResponse(response)) {
                        std::cerr << "[BT] sendResponse failed -- disconnecting.\n";
                        return;
                    }
                    buf.clear();
                }
            }
            else {
                if (buf.size() < 64) buf += c;
                else { std::cerr << "[BT] Buffer overflow -- clearing.\n"; buf.clear(); }
            }
        }
    }
    std::cout << "[BT] handleClient() exited. Commands processed: " << cmdCount << "\n";
}

bool BluetoothServer::sendResponse(const std::string& response)
{
    std::string full = response + "\r>";
    int sent = send(clientSock_, full.c_str(), (int)full.size(), 0);
    if (sent == SOCKET_ERROR) {
        std::cerr << "[BT] send() FAILED: " << WSAGetLastError() << "\n";
        return false;
    }
    if (!response.empty()) {
        bool isHeartbeat = (response.find("1E3F9803") != std::string::npos
            || response.find("00122001") != std::string::npos
            || response.find("44000000") != std::string::npos);
        if (!isHeartbeat)
            std::cout << "  [TX] " << response << "\n";
    }
    return true;
}

void BluetoothServer::printBanner(const std::string& text)
{
    std::string border(text.size() + 4, '=');
    std::cout << "\n" << border << "\n  " << text << "\n" << border << "\n\n";
}

void BluetoothServer::printBox(const std::string& text)
{
    std::string border(text.size() + 4, '=');
    std::cout << "\n+" << border << "+\n|  " << text << "  |\n+" << border << "+\n\n";
}