#pragma once

#include <network/message_framer/message_framer.hpp>
#include <network/tcp_socket/tcp_socket.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace comm
{

// One accepted peer connection. Owns the socket and closes it on destruction.
// Construction makes the owned socket non-blocking for bounded send/receive.
// receive() is called from the receive thread only.
// send() may run concurrently with receive(), but must not run concurrently
// with another send(); each peer has a single sending thread.
class Session
{
public:
    // Throws std::system_error if configuring the socket fails.
    Session(TcpSocket socket, std::string peerName);

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;

    const TcpSocket& getSocket() const noexcept;

    // False once the connection has ended; the session should be discarded.
    bool isOpen() const noexcept;

    // Returns the frames completed by the received bytes.
    // Throws ReceiveError.
    std::vector<MessageFrame> receive();

    // Throws SendError.
    void send(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout
    );

private:
    [[noreturn]] void failSend(
        const char* status,
        const char* operation,
        int errorCode,
        std::size_t bytesSent
    );

    TcpSocket _socket;
    std::string _peerName;
    bool _open = true;
    MessageFramer _framer;
};

} // namespace comm
