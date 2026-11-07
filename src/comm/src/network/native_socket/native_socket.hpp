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

// Thin wrappers over the Winsock / POSIX socket API.
// Functions returning int follow the native convention (negative on failure)
// unless documented otherwise; read the cause with lastError().
// IPv4 addresses are in network byte order, ports are in host byte order.
namespace native_socket
{

// Returns 0 on success, otherwise the error code.
int startup();

// Returns 0 on success, otherwise the error code.
int cleanup();

int lastError();
bool isInterrupted(int errorCode);
bool isWouldBlock(int errorCode);
int defaultBacklog() noexcept;

// Throws std::invalid_argument if the platform cannot register this count.
void validateRegistrationCount(std::size_t count);

std::optional<std::string> formatIpv4(uint32_t address);

SocketHandle createTcpSocket();
int setReuseAddress(SocketHandle socket);
int setNonBlocking(SocketHandle socket);
int setAbortiveClose(SocketHandle socket);
int bindSocket(SocketHandle socket, uint32_t address, uint16_t port);
int connectSocket(SocketHandle socket, uint32_t address, uint16_t port);
int listenSocket(SocketHandle socket, int backlog);

SocketHandle acceptSocket(
    SocketHandle listenSocket,
    uint32_t& remoteAddress,
    uint16_t& remotePort
);

// Waits clear readySockets and reject negative timeouts or handles that cannot
// be registered with select. Empty registrations return zero.
// Returns the select() result and fills readySockets with readable sockets.
int waitReadable(
    const std::vector<SocketHandle>& sockets,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
);

// Returns the select() result and fills readySockets with writable sockets.
int waitWritable(
    const std::vector<SocketHandle>& sockets,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
);

// Waits for a single writable socket with the same validation contract.
int waitWritable(
    SocketHandle socket,
    std::chrono::microseconds timeout
);

int sendBytes(SocketHandle socket, const uint8_t* data, int size);
int receiveBytes(SocketHandle socket, uint8_t* buffer, int size);

// Stops both directions without releasing the handle.
void shutdownSocket(SocketHandle socket);

// Shuts down both directions, then closes the socket.
int closeSocket(SocketHandle socket);

} // namespace native_socket
} // namespace comm
