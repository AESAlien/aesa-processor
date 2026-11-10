#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <message/message_framer.hpp>
#include <socket/connected_socket.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace comm
{

// 수락한 상대 연결 하나를 나타낸다. 소켓을 소유하며 소멸 시 닫는다.
// 수락한 소켓은 송수신 대기 시간을 제한할 수 있도록 이미 비차단 상태이다.
// receive()는 수신 스레드에서만 호출한다.
// send()는 receive()와 동시에 실행할 수 있지만 다른 send()와는
// 동시에 실행하면 안 된다. 상대마다 송신 스레드는 하나만 사용한다.
class Session
{
public:
    // 수락한 비차단 연결의 소유권을 넘겨받는다.
    explicit Session(ConnectedSocket socket);

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;

    // 연결이 종료되면 false를 반환하며, 이때 세션을 폐기해야 한다.
    bool isOpen() const noexcept;

    const CommEndpoint& getRemoteEndpoint() const noexcept;
    const ConnectedSocket& getSocket() const noexcept;

    // 수신한 바이트로 완성된 프레임을 반환한다.
    // ReceiveError를 던질 수 있다.
    std::vector<MessageFrame> receive();

    // SendError를 던질 수 있다.
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
