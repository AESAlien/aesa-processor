#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <socket/listen_socket.hpp>
#include <network/session.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

namespace comm
{

enum class ServerEventKind
{
    NONE,
    NEW_SESSION,
    READABLE
};

struct ServerEvent
{
    ServerEventKind kind;
    std::shared_ptr<Session> session;
};

// 연결을 수락하고 주어진 세션들과 리스닝 소켓을 함께 대기한다.
// 세션을 직접 보관하지 않으므로 각 poll()에 전달된 세션만 감시한다.
// 어떤 세션이 현재 유효한지는 호출자가 결정한다.
class TcpServer
{
public:
    // ServerOpenError를 던질 수 있다.
    explicit TcpServer(const CommEndpoint& endpoint);

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;
    TcpServer(TcpServer&&) = delete;
    TcpServer& operator=(TcpServer&&) = delete;

    // NEW_SESSION이면 새로 수락한 세션을, READABLE이면 전달받은 세션 중 하나를 반환한다.
    // 널 세션은 무시한다. 타임아웃이나 인터럽트가 발생하면
    // NONE을 반환한다. 동시에 준비된 소켓은 차례로 반환한다.
    // 연결 수락 실패 시 ConnectionError를, 대기 실패 시 ReceiveError를 던진다.
    ServerEvent poll(
        const std::vector<std::shared_ptr<Session>>& sessions,
        std::chrono::milliseconds timeout
    );

private:
    ServerEvent accept();

    ListenSocket _listenSocket;
    std::size_t _nextReadyIndex = 0;
};

} // namespace comm
