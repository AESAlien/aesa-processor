#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/message_framer/message_framer.hpp>
#include <network/connected_socket/connected_socket.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace comm
{

// One accepted peer connection. Owns the socket and closes it on destruction.
// The accepted socket is already non-blocking for bounded send/receive.
// receive() is called from the receive thread only.
// send() may run concurrently with receive(), but must not run concurrently
// with another send(); each peer has a single sending thread.
class Session
{
public:
    // Takes ownership of an accepted non-blocking connection.
    explicit Session(ConnectedSocket socket);

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;

    // False once the connection has ended; the session should be discarded.
    bool isOpen() const noexcept;

    const CommEndpoint& getRemoteEndpoint() const noexcept;
    const ConnectedSocket& getSocket() const noexcept;

    // Returns the frames completed by the received bytes.
    // Throws ReceiveError.
    std::vector<MessageFrame> receive();

    // Throws SendError.
    void send(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout
    );

private:
    std::string remoteDescription() const;

    [[noreturn]] void failSend(
        const char* status,
        const char* operation,
        int errorCode,
        std::size_t bytesSent
    );

    CommEndpoint _remoteEndpoint;
    ConnectedSocket _socket;
    bool _open = true;
    MessageFramer _framer;
};

} // namespace comm
