#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace comm
{

#ifdef _WIN32
using SocketHandle = std::uintptr_t;
constexpr SocketHandle INVALID_SOCKET_HANDLE = ~static_cast<SocketHandle>(0);
#else
using SocketHandle = int;
constexpr SocketHandle INVALID_SOCKET_HANDLE = -1;
#endif

// Winsock/POSIX adapter. Handles are borrowed; ownership belongs to SocketResource.
// Negative int results and invalid handles report native failures. Capture
// SocketError immediately, before another native call overwrites the error.
namespace native_socket
{

// Startup/cleanup return zero on success, otherwise an error code.
int startup();
int cleanup() noexcept;
int defaultBacklog() noexcept;

std::optional<std::string> formatIpv4(uint32_t address);
SocketHandle createTcpSocket();
int setReuseAddress(SocketHandle socket);
int setNonBlocking(SocketHandle socket);
int bindSocket(SocketHandle socket, uint32_t address, uint16_t port);
int listenSocket(SocketHandle socket, int backlog);
SocketHandle acceptSocket(SocketHandle socket);
int remoteEndpoint(SocketHandle socket, uint32_t& address, uint16_t& port);

// Throws invalid_argument for platform registration limits or invalid inputs.
void validateRegistrationCount(std::size_t count);
enum class WaitMode
{
    READABLE,
    WRITABLE
};

// Clears readySockets. Empty registrations return zero. Negative timeout is
// rejected. Returns the native select result; interruptions remain failures
// for the caller to classify. Handles must remain live throughout the call.
int wait(
    const std::vector<SocketHandle>& sockets,
    WaitMode mode,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
);

// Buffers must contain size bytes and size must fit the native int API.
int sendBytes(SocketHandle socket, const uint8_t* data, int size);
int receiveBytes(SocketHandle socket, uint8_t* buffer, int size);
int shutdownSocket(SocketHandle socket);
// Best-effort shutdown followed by close. Returns the close result.
int closeSocket(SocketHandle socket) noexcept;

} // namespace native_socket
} // namespace comm
