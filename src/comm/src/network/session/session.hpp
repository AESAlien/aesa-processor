#pragma once

#include <network/message_framer/message_framer.hpp>
#include <network/native_socket/native_socket.hpp>
#include <network/network_manager/network_types.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

namespace comm
{

// One accepted peer connection. Owns the socket and closes it on destruction.
// receive() is called from the receive thread only.
// send() may run concurrently with receive().
class Session
{
public:
    Session(SocketHandle socket, NetworkPeer peer);
    ~Session();

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;

    SocketHandle socket() const noexcept;

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

    SocketHandle _socket;
    NetworkPeer _peer;
    bool _open = true;
    MessageFramer _framer;
    std::mutex _sendMutex;
};

} // namespace comm
