#include <network/network_manager.hpp>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

namespace
{

volatile std::sig_atomic_t running = 1;

constexpr char SERVER_ADDRESS[] = "192.168.10.20";
constexpr std::uint16_t SERVER_PORT = 3200;
constexpr char OPERATOR_CONSOLE_ADDRESS[] = "192.168.10.10";
constexpr std::uint16_t OPERATOR_CONSOLE_PORT = 3100;
constexpr char SCENARIO_SIMULATOR_ADDRESS[] = "192.168.10.100";
constexpr std::uint16_t SCENARIO_SIMULATOR_PORT = 3300;

void stopRunning(int)
{
    running = 0;
}

const char* peerName(comm::NetworkPeer peer)
{
    switch (peer)
    {
    case comm::NetworkPeer::OPERATOR_CONSOLE:
        return "operator-console";

    case comm::NetworkPeer::SCENARIO_SIMULATOR:
        return "scenario-simulator";
    }

    return "unknown";
}

const char* operationName(comm::NetworkOperation operation)
{
    switch (operation)
    {
    case comm::NetworkOperation::NONE:
        return "none";

    case comm::NetworkOperation::CONFIGURATION:
        return "configuration";

    case comm::NetworkOperation::SOCKET_API_STARTUP:
        return "socket-api-startup";

    case comm::NetworkOperation::CREATE_SOCKET:
        return "create-socket";

    case comm::NetworkOperation::SET_SOCKET_OPTION:
        return "set-socket-option";

    case comm::NetworkOperation::BIND:
        return "bind";

    case comm::NetworkOperation::LISTEN:
        return "listen";

    case comm::NetworkOperation::SELECT:
        return "select";

    case comm::NetworkOperation::ACCEPT:
        return "accept";

    case comm::NetworkOperation::ADDRESS_CONVERSION:
        return "address-conversion";

    case comm::NetworkOperation::RECEIVE:
        return "receive";

    case comm::NetworkOperation::SEND:
        return "send";

    case comm::NetworkOperation::CLOSE:
        return "close";

    case comm::NetworkOperation::SOCKET_API_CLEANUP:
        return "socket-api-cleanup";
    }

    return "unknown";
}

const char* connectionStateName(comm::ConnectionState state)
{
    switch (state)
    {
    case comm::ConnectionState::CONNECTED:
        return "connected";

    case comm::ConnectionState::RECONNECTED:
        return "reconnected";

    case comm::ConnectionState::DISCONNECTED:
        return "disconnected";
    }

    return "unknown";
}

const char* disconnectReasonName(comm::DisconnectReason reason)
{
    switch (reason)
    {
    case comm::DisconnectReason::NONE:
        return "none";

    case comm::DisconnectReason::REMOTE_CLOSED:
        return "remote-closed";

    case comm::DisconnectReason::RECEIVE_ERROR:
        return "receive-error";
    }

    return "unknown";
}

const char* sendStatusName(comm::SendStatus status)
{
    switch (status)
    {
    case comm::SendStatus::SUCCESS:
        return "success";

    case comm::SendStatus::TIMEOUT:
        return "timeout";

    case comm::SendStatus::NOT_STARTED:
        return "not-started";

    case comm::SendStatus::NOT_CONNECTED:
        return "not-connected";

    case comm::SendStatus::INVALID_ARGUMENT:
        return "invalid-argument";

    case comm::SendStatus::CONNECTION_LOST:
        return "connection-lost";

    case comm::SendStatus::ERROR:
        return "error";
    }

    return "unknown";
}

void printSystemError(std::ostream& output, int errorCode)
{
    if (errorCode != 0)
    {
        output << ", error=" << errorCode;
    }
}

void printBytes(const std::vector<std::uint8_t>& bytes)
{
    std::cout << std::hex << std::setfill('0');
    for (std::uint8_t byte : bytes)
    {
        std::cout << std::setw(2) << static_cast<unsigned int>(byte) << ' ';
    }
    std::cout << std::dec << std::setfill(' ') << '\n';
}

void handleConnectionEvent(const comm::ConnectionEvent& event)
{
    std::cout << "[connection] peer=" << peerName(event.peer)
              << ", state=" << connectionStateName(event.state);

    if (event.state == comm::ConnectionState::DISCONNECTED)
    {
        std::cout << ", reason=" << disconnectReasonName(event.disconnectReason);
        printSystemError(std::cout, event.errorCode);
    }

    std::cout << '\n';
}

void handleDataReceivedEvent(
    comm::NetworkManager& networkManager,
    const comm::DataReceivedEvent& event
)
{
    std::cout << "[receive] peer=" << peerName(event.peer)
              << ", bytes=" << event.bytes.size() << ", data=";
    printBytes(event.bytes);

    comm::SendResult sendResult = networkManager.send(event.peer, event.bytes);
    std::cout << "[echo] peer=" << peerName(event.peer)
              << ", status=" << sendStatusName(sendResult.status)
              << ", bytes=" << sendResult.bytesSent;

    if (sendResult.failedOperation != comm::NetworkOperation::NONE)
    {
        std::cout << ", operation=" << operationName(sendResult.failedOperation);
    }
    printSystemError(std::cout, sendResult.errorCode);
    std::cout << '\n';
}

void handleEvent(
    comm::NetworkManager& networkManager,
    const comm::NetworkEvent& event
)
{
    if (const auto* connection = std::get_if<comm::ConnectionEvent>(&event))
    {
        handleConnectionEvent(*connection);
        return;
    }

    if (const auto* received = std::get_if<comm::DataReceivedEvent>(&event))
    {
        handleDataReceivedEvent(networkManager, *received);
        return;
    }

    const auto& rejected = std::get<comm::ConnectionRejectedEvent>(event);
    std::cout << "[rejected] remote=" << rejected.remoteAddress
              << ':' << rejected.remotePort << '\n';
}

} // namespace

int main()
{
    const comm::NetworkEndpoint serverEndpoint
    {
        SERVER_ADDRESS,
        SERVER_PORT
    };
    const comm::NetworkEndpoint operatorConsoleEndpoint
    {
        OPERATOR_CONSOLE_ADDRESS,
        OPERATOR_CONSOLE_PORT
    };
    const comm::NetworkEndpoint scenarioSimulatorEndpoint
    {
        SCENARIO_SIMULATOR_ADDRESS,
        SCENARIO_SIMULATOR_PORT
    };
    comm::NetworkManager networkManager(
        serverEndpoint,
        operatorConsoleEndpoint,
        scenarioSimulatorEndpoint);

    comm::OperationResult startResult = networkManager.start();
    if (startResult.status != comm::OperationStatus::SUCCESS)
    {
        std::cerr << "Failed to start server: operation="
                  << operationName(startResult.operation);
        printSystemError(std::cerr, startResult.errorCode);
        std::cerr << '\n';
        return 1;
    }

    std::signal(SIGINT, stopRunning);
    std::signal(SIGTERM, stopRunning);

    std::cout << "comm test server=" << SERVER_ADDRESS
              << ':' << SERVER_PORT << '\n'
              << "operator-console=" << OPERATOR_CONSOLE_ADDRESS
              << ':' << OPERATOR_CONSOLE_PORT << '\n'
              << "scenario-simulator=" << SCENARIO_SIMULATOR_ADDRESS
              << ':' << SCENARIO_SIMULATOR_PORT << '\n'
              << "Press Ctrl+C to stop.\n";

    bool pollSucceeded = true;
    while (running != 0)
    {
        comm::PollResult pollResult = networkManager.poll(
            std::chrono::milliseconds{250});

        if (pollResult.status == comm::PollStatus::TIMEOUT)
        {
            continue;
        }

        if (pollResult.status != comm::PollStatus::EVENTS)
        {
            std::cerr << "Polling failed: operation="
                      << operationName(pollResult.failedOperation);
            printSystemError(std::cerr, pollResult.errorCode);
            std::cerr << '\n';
            pollSucceeded = false;
            break;
        }

        for (const comm::NetworkEvent& event : pollResult.events)
        {
            handleEvent(networkManager, event);
        }
    }

    comm::OperationResult stopResult = networkManager.stop();
    if (stopResult.status != comm::OperationStatus::SUCCESS)
    {
        std::cerr << "Failed to stop server: operation="
                  << operationName(stopResult.operation);
        printSystemError(std::cerr, stopResult.errorCode);
        std::cerr << '\n';
        return 1;
    }

    std::cout << "comm test server stopped.\n";
    return pollSucceeded ? 0 : 1;
}
