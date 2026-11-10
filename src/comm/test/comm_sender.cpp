#include <comm/config/comm_endpoint.hpp>
#include <socket/native_socket.hpp>
#include <protocol/ST_MsgHeader.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string_view>
#include <vector>

namespace
{

constexpr char SERVER_IP[] = "127.0.0.20";
constexpr uint16_t SERVER_PORT = 3200;
constexpr char CLIENT_IP[] = "127.0.0.100";
constexpr uint16_t CLIENT_PORT = 3300;
constexpr uint8_t MESSAGE_VERSION = 1;
constexpr uint8_t SOURCE_ID = 100;
constexpr uint8_t DESTINATION_ID = 20;
constexpr std::chrono::seconds RECEIVE_TIMEOUT{3};
constexpr std::string_view PAYLOAD = "Hello World";

using comm::SocketHandle;
using comm::INVALID_SOCKET_HANDLE;

void waitForExit()
{
    std::cout << "Press Enter to exit..." << std::flush;
    std::cin.get();
}

std::vector<uint8_t> makeMessage()
{
    using Clock = std::chrono::system_clock;

    const Clock::duration elapsed = Clock::now().time_since_epoch();
    const auto elapsedSeconds =
        std::chrono::duration_cast<std::chrono::seconds>(elapsed);
    const auto elapsedNanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            elapsed - elapsedSeconds
        );

    comm::protocol::ST_MsgHeader header{};
    header.message_Id = static_cast<uint16_t>(
        comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR
    );
    header.version = MESSAGE_VERSION;
    header.block_size = static_cast<uint32_t>(
        sizeof(header) + PAYLOAD.size()
    );
    header.timeSec = static_cast<uint32_t>(elapsedSeconds.count());
    header.timeNsec = static_cast<uint32_t>(
        elapsedNanoseconds.count()
    );
    header.source_id = SOURCE_ID;
    header.dest_id = DESTINATION_ID;

    std::vector<uint8_t> message(header.block_size);
    std::memcpy(message.data(), &header, sizeof(header));
    std::memcpy(
        message.data() + sizeof(header),
        PAYLOAD.data(),
        PAYLOAD.size()
    );
    return message;
}

bool sendAll(
    SocketHandle socket,
    const std::vector<uint8_t>& message
) {
    std::size_t bytesSent = 0;
    while (bytesSent < message.size())
    {
        const std::size_t remainingSize = message.size() - bytesSent;
        const int sendSize = remainingSize > static_cast<std::size_t>(
            std::numeric_limits<int>::max()
        )
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

        const int result = comm::native_socket::sendBytes(socket, message.data() + bytesSent, sendSize);

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
    std::vector<SocketHandle> readySockets;
    return comm::native_socket::waitReadable({socket}, RECEIVE_TIMEOUT, readySockets) > 0;
}

bool receiveAll(
    SocketHandle socket,
    std::vector<uint8_t>& receivedMessage
) {
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
            std::numeric_limits<int>::max()
        )
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

        const int result = comm::native_socket::receiveBytes(socket, receivedMessage.data() + bytesReceived, receiveSize);

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
    const comm::CommEndpoint clientAddress(CLIENT_IP, CLIENT_PORT);
    const comm::CommEndpoint serverAddress(SERVER_IP, SERVER_PORT);
    const SocketHandle socketHandle = comm::native_socket::createTcpSocket();
    if (socketHandle == INVALID_SOCKET_HANDLE)
    {
        std::cerr << "Failed to create socket: error="
                  << comm::native_socket::lastError() << '\n';
        return 1;
    }

    if (comm::native_socket::setReuseAddress(socketHandle) < 0)
    {
        std::cerr << "Failed to configure client socket: error="
                  << comm::native_socket::lastError() << '\n';
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    if (comm::native_socket::setAbortiveClose(socketHandle) < 0)
    {
        std::cerr << "Failed to configure socket close behavior: error="
                  << comm::native_socket::lastError() << '\n';
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    if (comm::native_socket::bindSocket(
            socketHandle,
            clientAddress.getIntAddress(),
            clientAddress.getPort()
        ) < 0)
    {
        std::cerr << "Failed to bind " << CLIENT_IP << ':' << CLIENT_PORT
                  << ": error=" << comm::native_socket::lastError() << '\n';
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    if (comm::native_socket::connectSocket(
            socketHandle,
            serverAddress.getIntAddress(),
            serverAddress.getPort()
        ) < 0)
    {
        std::cerr << "Failed to connect to " << SERVER_IP << ':'
                  << SERVER_PORT << ": error="
                  << comm::native_socket::lastError() << '\n';
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    const std::vector<uint8_t> message = makeMessage();
    if (!sendAll(socketHandle, message))
    {
        std::cerr << "Failed to send message: error="
                  << comm::native_socket::lastError() << '\n';
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    std::cout << "Sent " << message.size() << " bytes from "
              << CLIENT_IP << ':' << CLIENT_PORT << " to "
              << SERVER_IP << ':' << SERVER_PORT << '\n'
              << "Payload: " << PAYLOAD << '\n';

    std::vector<uint8_t> echoedMessage(message.size());
    if (!receiveAll(socketHandle, echoedMessage))
    {
        std::cerr << "Failed to receive the echo within "
                  << RECEIVE_TIMEOUT.count() << " seconds: error="
                  << comm::native_socket::lastError() << '\n';
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    if (echoedMessage != message)
    {
        std::cerr << "The echoed message does not match the sent message.\n";
        comm::native_socket::closeSocket(socketHandle);
        return 1;
    }

    std::cout << "Echo verified successfully.\n";
    waitForExit();
    comm::native_socket::closeSocket(socketHandle);
    return 0;
}

} // namespace

int main()
{
    const int startupError = comm::native_socket::startup();
    if (startupError != 0)
    {
        std::cerr << "Failed to initialize the socket API: error="
                  << startupError << '\n';
        return 1;
    }

    const int result = runSender();
    if (result != 0)
    {
        waitForExit();
    }
    comm::native_socket::cleanup();
    return result;
}
