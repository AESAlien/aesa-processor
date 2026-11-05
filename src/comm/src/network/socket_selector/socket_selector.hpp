#pragma once

#include <network/tcp_socket/tcp_socket.hpp>

#include <chrono>
#include <vector>

namespace comm
{

enum class SocketWaitStatus
{
    READY,
    TIMED_OUT,
    INTERRUPTED
};

// Borrows registered sockets without transferring ownership. They must remain
// alive, open, and unmoved throughout the selector's lifetime. The same selector
// can wait repeatedly without registering its sockets again.
class SocketSelector
{
public:
    // Rejects null pointers and registrations exceeding the platform's limit.
    explicit SocketSelector(std::vector<const TcpSocket*> sockets);

    // Replaces readySockets on every call. Empty registrations return TIMED_OUT
    // immediately. Negative timeouts are rejected; zero polls without blocking.
    // Returns INTERRUPTED for signal interruption; other native failures throw
    // std::system_error. Ready sockets may represent data, acceptance, or closure.
    SocketWaitStatus waitReadable(
        std::chrono::microseconds timeout,
        std::vector<const TcpSocket*>& readySockets
    ) const;

    // Uses the same timeout and interruption contract as waitReadable().
    SocketWaitStatus waitWritable(
        std::chrono::microseconds timeout,
        std::vector<const TcpSocket*>& readySockets
    ) const;

private:
    enum class WaitMode
    {
        READABLE,
        WRITABLE
    };

    SocketWaitStatus wait(
        WaitMode mode,
        std::chrono::microseconds timeout,
        std::vector<const TcpSocket*>& readySockets
    ) const;

    std::vector<const TcpSocket*> _sockets;
};

} // namespace comm
