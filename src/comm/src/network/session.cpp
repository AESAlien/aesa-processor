#include <network/session.hpp>

#include <comm/error/comm_error.hpp>

#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace comm
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::size_t RECEIVE_BUFFER_SIZE = 4096;
constexpr char RECEIVE_FAILURE[] = "Communication data could not be received";

} // namespace

Session::Session(ConnectedSocket socket)
  : _remoteEndpoint(socket.getRemoteEndpoint()),
    _socket(std::move(socket))
{
}

bool Session::isOpen() const noexcept
{
    return _open;
}

const CommEndpoint& Session::getRemoteEndpoint() const noexcept
{
    return _remoteEndpoint;
}

const ConnectedSocket& Session::getSocket() const noexcept
{
    return _socket;
}

std::string Session::remoteDescription() const
{
    return _remoteEndpoint.getAddress() + ":" +
        std::to_string(_remoteEndpoint.getPort());
}

std::vector<MessageFrame> Session::receive()
{
    std::optional<std::vector<uint8_t>> received;
    try
    {
        received = _socket.receiveBytes(RECEIVE_BUFFER_SIZE);
    }
    catch (const std::system_error& error)
    {
        _open = false;
        throw ReceiveError(
            std::string(RECEIVE_FAILURE) +
            ": remote=" + remoteDescription() +
            ", operation=receive, systemError=" + std::to_string(error.code().value())
        );
    }

    if (!received)
    {
        return {};
    }
    if (received->empty())
    {
        _open = false;
        return {};
    }

    try
    {
        return _framer.append(*received);
    }
    catch (const FramingError& error)
    {
        _framer.reset();
        throw ReceiveError(
            std::string(RECEIVE_FAILURE) +
            ": stage=framing, remote=" + remoteDescription() +
            ", error=" + error.what()
        );
    }
}

void Session::send(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("Send requires a non-negative timeout");
    }
    const Clock::time_point deadline = Clock::now() + timeout;
    std::size_t bytesSent = 0;
    bool attempted = false;

    while (bytesSent < data.size())
    {
        // 타임아웃이 0이어도 즉시 수행하는 비차단 시도는 한 번 허용한다.
        // 인터럽트가 발생한 경우를 포함해 모든 재시도는 최초의 제한 시각을 공유한다.
        if (attempted && Clock::now() >= deadline)
        {
            failSend("timeout", "send", 0, bytesSent);
        }
        attempted = true;

        std::optional<std::size_t> sent;
        try
        {
            sent = _socket.sendBytes(data, bytesSent);
        }
        catch (const std::system_error& error)
        {
            failSend("connection-lost", "send", error.code().value(), bytesSent);
        }

        if (sent)
        {
            if (*sent == 0)
            {
                failSend("connection-lost", "send", 0, bytesSent);
            }
            bytesSent += *sent;
            continue;
        }

        if (Clock::now() >= deadline)
        {
            failSend("timeout", "wait-writable", 0, bytesSent);
        }

        try
        {
            _socket.waitWritableUntil(deadline);
        }
        catch (const std::system_error& error)
        {
            if (error.code() == std::errc::timed_out)
            {
                failSend("timeout", "wait-writable", 0, bytesSent);
            }
            failSend("error", "wait-writable", error.code().value(), bytesSent);
        }
    }
}

// 메시지가 일부만 송신되면 스트림이 손상되므로 연결을 종료한다.
// 이후 수신 스레드가 연결 종료를 확인하고 세션을 폐기한다.
void Session::failSend(
    const char* status,
    const char* operation,
    int errorCode,
    std::size_t bytesSent
) {
    if (bytesSent > 0)
    {
        try
        {
            _socket.shutdown();
        }
        catch (const std::system_error&)
        {
            // 상대가 이미 연결을 끊었더라도 원래의 송신 오류를 유지한다.
        }
    }

    throw SendError(
        std::string("Communication data could not be sent: remote=") +
        remoteDescription() +
        ", status=" + status +
        ", operation=" + operation +
        ", systemError=" + std::to_string(errorCode) +
        ", bytesSent=" + std::to_string(bytesSent)
    );
}

} // namespace comm
