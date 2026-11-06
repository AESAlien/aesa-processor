#pragma once

#include <network/socket/socket.hpp>

#include <cstddef>

#include <chrono>
#include <vector>

namespace comm
{

// Borrows sockets without transferring ownership. They must remain alive, open,
// and unmoved throughout the selector's lifetime. The same selector can wait
// repeatedly without registering its sockets again.
class SocketSelector
{
public:
    // Rejects null pointers and socket counts exceeding the platform's limit.
    explicit SocketSelector(std::vector<const Socket*> sockets);

    // Returns ready sockets. Timeout or interruption returns an empty vector;
    // other native failures throw std::system_error. Negative timeouts are
    // rejected; zero polls without blocking.
    std::vector<const Socket*> waitReadable(std::chrono::microseconds timeout) const;

    // Uses the same timeout and interruption contract as waitReadable().
    std::vector<const Socket*> waitWritable(std::chrono::microseconds timeout) const;

private:
    enum class WaitMode
    {
        READABLE,
        WRITABLE
    };

    std::vector<const Socket*> wait(
        WaitMode mode,
        std::chrono::microseconds timeout
    ) const;

    std::vector<const Socket*> _sockets;
};

} // namespace comm
