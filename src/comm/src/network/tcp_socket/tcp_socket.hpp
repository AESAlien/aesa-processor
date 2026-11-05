#pragma once

#include <comm/config/comm_endpoint.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace comm
{

// Owns one IPv4 TCP socket and keeps the socket API initialized internally.
// Do not move, close, or destroy it while an operation is using it.
// Constructors validate acquired handles. Socket operations require a live
// owned handle; after close or move, only reassignment, close, and destruction
// are supported on the source object.
// Native failures throw std::system_error. Expected non-blocking and interrupted
// send, receive, and accept operations return nullopt.
class TcpSocket
{
public:
    // Creates an unconnected socket. Throws std::system_error on failure.
    // The default backlog uses the platform's SOMAXCONN value.
    explicit TcpSocket(int backlog = DEFAULT_BACKLOG);
    ~TcpSocket();

    TcpSocket(const TcpSocket&) = delete;
    TcpSocket& operator=(const TcpSocket&) = delete;
    TcpSocket(TcpSocket&& other) noexcept;
    TcpSocket& operator=(TcpSocket&& other) noexcept;

    void setReuseAddress();
    void setNonBlocking();

    void bind(const CommEndpoint& endpoint);
    void listen();

    // In non-blocking mode, returns nullopt when no connection is pending.
    // Also returns nullopt if accept is interrupted. Blocking mode waits for
    // a connection. The accepted socket's blocking mode is platform-dependent.
    // Other failures throw std::system_error. A successful result owns the
    // accepted socket and remains usable after the listener is destroyed.
    std::optional<TcpSocket> accept();

    // Queries the connected peer. Throws std::system_error if unconnected.
    CommEndpoint getRemoteEndpoint() const;

    // Returns when writing is ready. Throws std::system_error with
    // std::errc::timed_out on timeout; other native failures also throw
    // std::system_error. Interruptions retry within the original deadline.
    // Negative timeouts are rejected; zero polls once.
    void waitWritable(std::chrono::microseconds timeout) const;

    // Uses the caller's absolute deadline, including across interruptions.
    // An expired deadline polls once. Uses the same exception contract as
    // waitWritable(). Readiness does not impose a timeout on subsequent I/O.
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

    // Best-effort shutdown and close for destruction. Repeated calls are safe.
    void close() noexcept;

private:
    friend class SocketSelector;

    static const int DEFAULT_BACKLOG;

    // Distinguishes adopting a native handle from the int backlog constructor.
    struct AdoptHandle {};

#ifdef _WIN32
    using SocketHandle = std::uintptr_t;
    static constexpr SocketHandle INVALID_SOCKET_HANDLE = ~static_cast<SocketHandle>(0);
#else
    using SocketHandle = int;
    static constexpr SocketHandle INVALID_SOCKET_HANDLE = -1;
#endif

    TcpSocket(SocketHandle handle, AdoptHandle);

    void initializeApi();
    void cleanupApi() noexcept;
    void validate() const;

    SocketHandle _handle = INVALID_SOCKET_HANDLE;
    int _backlog = DEFAULT_BACKLOG;
#ifdef _WIN32
    // Owns one successful WSAStartup call until destruction or move assignment.
    bool _apiInitialized = false;
#endif
};

} // namespace comm
