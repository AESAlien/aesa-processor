#include <network/session/session.hpp>

#include <comm/error/comm_error.hpp>

#include <limits>
#include <string>

namespace comm
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr int RECEIVE_BUFFER_SIZE = 4096;
constexpr char RECEIVE_FAILURE[] = "Communication data could not be received";

std::chrono::microseconds remainingTime(Clock::time_point deadline);

} // namespace

Session::Session(SocketHandle socket, NetworkPeer peer)
    : _socket(socket),
      _peer(peer)
{
}

Session::~Session()
{
    native_socket::closeSocket(_socket);
}

SocketHandle Session::socket() const noexcept
{
    return _socket;
}

bool Session::isOpen() const noexcept
{
    return _open;
}

std::vector<MessageFrame> Session::receive()
{
    uint8_t buffer[RECEIVE_BUFFER_SIZE];
    const int receivedSize = native_socket::receiveBytes(
        _socket,
        buffer,
        sizeof(buffer));

    if (receivedSize == 0)
    {
        _open = false;
        return {};
    }

    if (receivedSize < 0)
    {
        const int errorCode = native_socket::lastError();
        if (native_socket::isWouldBlock(errorCode))
        {
            return {};
        }

        _open = false;
        throw ReceiveError(
            std::string(RECEIVE_FAILURE) +
            ": peer=" + peerName(_peer) +
            ", operation=receive, systemError=" + std::to_string(errorCode));
    }

    try
    {
        return _framer.append(
            std::vector<uint8_t>(buffer, buffer + receivedSize));
    }
    catch (const FramingError& error)
    {
        _framer.reset();
        throw ReceiveError(
            std::string(RECEIVE_FAILURE) +
            ": stage=framing, peer=" + peerName(_peer) +
            ", error=" + error.what());
    }
}

void Session::send(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
)
{
    std::lock_guard<std::mutex> lock(_sendMutex);

    const Clock::time_point deadline = Clock::now() + timeout;
    std::size_t bytesSent = 0;

    while (bytesSent < data.size())
    {
        const std::size_t remainingSize = data.size() - bytesSent;
        const int sendSize = remainingSize > static_cast<std::size_t>(
            std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

        const int sentSize = native_socket::sendBytes(
            _socket,
            data.data() + bytesSent,
            sendSize);

        if (sentSize > 0)
        {
            bytesSent += static_cast<std::size_t>(sentSize);
            continue;
        }

        if (sentSize == 0)
        {
            failSend("connection-lost", "send", 0, bytesSent);
        }

        int errorCode = native_socket::lastError();
        if (native_socket::isInterrupted(errorCode))
        {
            continue;
        }

        if (!native_socket::isWouldBlock(errorCode))
        {
            failSend("connection-lost", "send", errorCode, bytesSent);
        }

        const std::chrono::microseconds remaining = remainingTime(deadline);
        if (remaining.count() == 0)
        {
            failSend("timeout", "select", 0, bytesSent);
        }

        const int selectedCount =
            native_socket::waitWritable(_socket, remaining);
        if (selectedCount > 0)
        {
            continue;
        }

        if (selectedCount == 0)
        {
            failSend("timeout", "select", 0, bytesSent);
        }

        errorCode = native_socket::lastError();
        if (native_socket::isInterrupted(errorCode) &&
            Clock::now() < deadline)
        {
            continue;
        }

        failSend("error", "select", errorCode, bytesSent);
    }
}

// A partially sent message corrupts the stream, so the connection is shut
// down. The receive thread then sees it closed and discards the session.
void Session::failSend(
    const char* status,
    const char* operation,
    int errorCode,
    std::size_t bytesSent
)
{
    if (bytesSent > 0)
    {
        native_socket::shutdownSocket(_socket);
    }

    throw SendError(
        std::string("Communication data could not be sent: peer=") +
        peerName(_peer) +
        ", status=" + status +
        ", operation=" + operation +
        ", systemError=" + std::to_string(errorCode) +
        ", bytesSent=" + std::to_string(bytesSent));
}

namespace
{

std::chrono::microseconds remainingTime(Clock::time_point deadline)
{
    const Clock::time_point now = Clock::now();
    if (now >= deadline)
    {
        return std::chrono::microseconds{0};
    }

    return std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - now);
}

} // namespace
} // namespace comm
