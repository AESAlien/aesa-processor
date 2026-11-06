#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/socket/socket.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace comm
{

// One accepted, non-blocking TCP connection. After move/close only move
// assignment, close and destruction are supported. No listener operations.
class ConnectedSocket : public Socket
{
public:
    ConnectedSocket(const ConnectedSocket&) = delete;
    ConnectedSocket& operator=(const ConnectedSocket&) = delete;
    ConnectedSocket(ConnectedSocket&&) noexcept = default;
    ConnectedSocket& operator=(ConnectedSocket&&) noexcept = default;

    // Queries the connected peer. Native failures throw std::system_error.
    CommEndpoint getRemoteEndpoint() const;

    // Returns when writing is ready. Throws std::system_error with
    // std::errc::timed_out on timeout; other native failures also throw
    // std::system_error. Interruptions retry within the original deadline.
    // Negative timeouts are rejected; zero polls once.
    void waitWritable(std::chrono::microseconds timeout) const;

    // Uses the caller's absolute deadline, including across interruptions.
    // An expired deadline polls once. Uses the same exception contract as
    // waitWritable(). Its timeout does not apply to subsequent I/O.
    void waitWritableUntil(std::chrono::steady_clock::time_point deadline) const;

    // Sends from offset through the end without copying the input. Large
    // transfers may complete partially; native size limits apply internally.
    // offset may equal data.size(), returning zero without calling the API.
    // An offset beyond data.size() throws std::out_of_range.
    // Returns nullopt when sending would block or is interrupted.
    // Other failures throw std::system_error.
    std::optional<std::size_t> sendBytes(
        const std::vector<uint8_t>& data,
        std::size_t offset = 0
    );

    // Receives once, up to maximumBytes, and returns an owned chunk of bytes,
    // not necessarily a full message. maximumBytes must be greater than zero.
    // Native per-call size limits are applied before allocating the buffer.
    // An empty vector means peer closure. Returns nullopt when receiving would
    // block or is interrupted. Other failures throw std::system_error.
    std::optional<std::vector<uint8_t>> receiveBytes(std::size_t maximumBytes);

    // Stops both directions without releasing the handle.
    void shutdown();

private:
    friend class ListenSocket;
    explicit ConnectedSocket(SocketResource socket);
};

} // namespace comm
