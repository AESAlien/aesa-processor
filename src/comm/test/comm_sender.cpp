#include <protocol/ST_MsgHeader.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string_view>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace
{

constexpr char SERVER_IP[] = "127.0.0.20";
constexpr std::uint16_t SERVER_PORT = 3200;
constexpr char CLIENT_IP[] = "127.0.0.100";
constexpr std::uint16_t CLIENT_PORT = 3300;
constexpr std::uint8_t MESSAGE_VERSION = 1;
constexpr std::uint8_t SOURCE_ID = 100;
constexpr std::uint8_t DESTINATION_ID = 20;
constexpr std::chrono::seconds RECEIVE_TIMEOUT{3};
constexpr std::string_view PAYLOAD = "Hello World";

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle INVALID_SOCKET_HANDLE = INVALID_SOCKET;
#else
using SocketHandle = int;
constexpr SocketHandle INVALID_SOCKET_HANDLE = -1;
#endif

int getLastSocketError()
{
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

void closeSocket(SocketHandle socket)
{
    if (socket == INVALID_SOCKET_HANDLE)
    {
        return;
    }

#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

bool initializeSocketApi()
{
#ifdef _WIN32
    WSADATA socketApiData{};
    return WSAStartup(MAKEWORD(2, 2), &socketApiData) == 0;
#else
    return true;
#endif
}

void cleanupSocketApi()
{
#ifdef _WIN32
    WSACleanup();
#endif
}

void waitForExit()
{
    std::cout << "Press Enter to exit..." << std::flush;
    std::cin.get();
}

bool setAddress(
    sockaddr_in& socketAddress,
    const char* ipAddress,
    std::uint16_t port
)
{
    socketAddress = {};
    socketAddress.sin_family = AF_INET;
    socketAddress.sin_port = htons(port);
    return inet_pton(
        AF_INET,
        ipAddress,
        &socketAddress.sin_addr) == 1;
}

std::vector<std::uint8_t> makeMessage()
{
    using Clock = std::chrono::system_clock;

    const Clock::duration elapsed = Clock::now().time_since_epoch();
    const auto elapsedSeconds =
        std::chrono::duration_cast<std::chrono::seconds>(elapsed);
    const auto elapsedNanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            elapsed - elapsedSeconds);

    comm::protocol::ST_MsgHeader header{};
    header.message_Id = static_cast<std::uint16_t>(
        comm::protocol::MessageId::RadarAttitudeFromSimulator);
    header.version = MESSAGE_VERSION;
    header.block_size = static_cast<std::uint32_t>(
        sizeof(header) + PAYLOAD.size());
    header.timeSec = static_cast<std::uint32_t>(elapsedSeconds.count());
    header.timeNsec = static_cast<std::uint32_t>(
        elapsedNanoseconds.count());
    header.source_id = SOURCE_ID;
    header.dest_id = DESTINATION_ID;

    std::vector<std::uint8_t> message(header.block_size);
    std::memcpy(message.data(), &header, sizeof(header));
    std::memcpy(
        message.data() + sizeof(header),
        PAYLOAD.data(),
        PAYLOAD.size());
    return message;
}

bool sendAll(
    SocketHandle socket,
    const std::vector<std::uint8_t>& message
)
{
    std::size_t bytesSent = 0;
    while (bytesSent < message.size())
    {
        const std::size_t remainingSize = message.size() - bytesSent;
        const int sendSize = remainingSize > static_cast<std::size_t>(
            std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

#ifdef _WIN32
        const int result = send(
            socket,
            reinterpret_cast<const char*>(message.data() + bytesSent),
            sendSize,
            0);
#else
        const int result = static_cast<int>(send(
            socket,
            message.data() + bytesSent,
            sendSize,
            0));
#endif

        if (result <= 0)
        {
            return false;
        }
        bytesSent += static_cast<std::size_t>(result);
    }

    return true;
}

bool waitForReadable(SocketHandle socket)
{
    fd_set readSockets;
    FD_ZERO(&readSockets);
    FD_SET(socket, &readSockets);

    timeval timeout{};
    timeout.tv_sec = static_cast<long>(RECEIVE_TIMEOUT.count());

#ifdef _WIN32
    return select(0, &readSockets, nullptr, nullptr, &timeout) > 0;
#else
    return select(socket + 1, &readSockets, nullptr, nullptr, &timeout) > 0;
#endif
}

bool receiveAll(
    SocketHandle socket,
    std::vector<std::uint8_t>& receivedMessage
)
{
    std::size_t bytesReceived = 0;
    while (bytesReceived < receivedMessage.size())
    {
        if (!waitForReadable(socket))
        {
            return false;
        }

        const std::size_t remainingSize =
            receivedMessage.size() - bytesReceived;
        const int receiveSize = remainingSize > static_cast<std::size_t>(
            std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

#ifdef _WIN32
        const int result = recv(
            socket,
            reinterpret_cast<char*>(
                receivedMessage.data() + bytesReceived),
            receiveSize,
            0);
#else
        const int result = static_cast<int>(recv(
            socket,
            receivedMessage.data() + bytesReceived,
            receiveSize,
            0));
#endif

        if (result <= 0)
        {
            return false;
        }
        bytesReceived += static_cast<std::size_t>(result);
    }

    return true;
}

int runSender()
{
    const SocketHandle socketHandle = static_cast<SocketHandle>(socket(
        AF_INET,
        SOCK_STREAM,
        IPPROTO_TCP));
    if (socketHandle == INVALID_SOCKET_HANDLE)
    {
        std::cerr << "Failed to create socket: error="
                  << getLastSocketError() << '\n';
        return 1;
    }

    int reuseAddress = 1;
    if (setsockopt(
            socketHandle,
            SOL_SOCKET,
            SO_REUSEADDR,
#ifdef _WIN32
            reinterpret_cast<const char*>(&reuseAddress),
#else
            &reuseAddress,
#endif
            sizeof(reuseAddress)) < 0)
    {
        std::cerr << "Failed to configure client socket: error="
                  << getLastSocketError() << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    linger closeBehavior{};
    closeBehavior.l_onoff = 1;
    closeBehavior.l_linger = 0;
    if (setsockopt(
            socketHandle,
            SOL_SOCKET,
            SO_LINGER,
#ifdef _WIN32
            reinterpret_cast<const char*>(&closeBehavior),
#else
            &closeBehavior,
#endif
            sizeof(closeBehavior)) < 0)
    {
        std::cerr << "Failed to configure socket close behavior: error="
                  << getLastSocketError() << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    sockaddr_in clientAddress{};
    if (!setAddress(clientAddress, CLIENT_IP, CLIENT_PORT))
    {
        std::cerr << "Invalid client IP address: " << CLIENT_IP << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    if (bind(
            socketHandle,
            reinterpret_cast<sockaddr*>(&clientAddress),
            sizeof(clientAddress)) < 0)
    {
        std::cerr << "Failed to bind " << CLIENT_IP << ':' << CLIENT_PORT
                  << ": error=" << getLastSocketError() << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    sockaddr_in serverAddress{};
    if (!setAddress(serverAddress, SERVER_IP, SERVER_PORT))
    {
        std::cerr << "Invalid server IP address: " << SERVER_IP << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    if (connect(
            socketHandle,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)) < 0)
    {
        std::cerr << "Failed to connect to " << SERVER_IP << ':'
                  << SERVER_PORT << ": error="
                  << getLastSocketError() << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    const std::vector<std::uint8_t> message = makeMessage();
    if (!sendAll(socketHandle, message))
    {
        std::cerr << "Failed to send message: error="
                  << getLastSocketError() << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    std::cout << "Sent " << message.size() << " bytes from "
              << CLIENT_IP << ':' << CLIENT_PORT << " to "
              << SERVER_IP << ':' << SERVER_PORT << '\n'
              << "Payload: " << PAYLOAD << '\n';

    std::vector<std::uint8_t> echoedMessage(message.size());
    if (!receiveAll(socketHandle, echoedMessage))
    {
        std::cerr << "Failed to receive the echo within "
                  << RECEIVE_TIMEOUT.count() << " seconds: error="
                  << getLastSocketError() << '\n';
        closeSocket(socketHandle);
        return 1;
    }

    if (echoedMessage != message)
    {
        std::cerr << "The echoed message does not match the sent message.\n";
        closeSocket(socketHandle);
        return 1;
    }

    std::cout << "Echo verified successfully.\n";
    waitForExit();
    closeSocket(socketHandle);
    return 0;
}

} // namespace

int main()
{
    if (!initializeSocketApi())
    {
        std::cerr << "Failed to initialize the socket API: error="
                  << getLastSocketError() << '\n';
        return 1;
    }

    const int result = runSender();
    if (result != 0)
    {
        waitForExit();
    }
    cleanupSocketApi();
    return result;
}
