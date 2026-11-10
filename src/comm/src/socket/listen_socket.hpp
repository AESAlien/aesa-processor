#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <socket/connected_socket.hpp>
#include <socket/socket.hpp>

#include <optional>

namespace comm
{

// 바인딩 후 연결을 대기하는 비차단 IPv4 TCP 소켓이다. 데이터 입출력은 지원하지 않는다.
// 이동되거나 닫힌 뒤에는 이동 대입, 닫기, 소멸만 지원한다.
class ListenSocket : public Socket
{
public:
    // 생성 시 리스닝에 필요한 모든 설정을 완료한다. ServerOpenError를 던질 수 있다.
    explicit ListenSocket(const CommEndpoint& endpoint);
    ListenSocket(const CommEndpoint& endpoint, int backlog);
    ListenSocket(const ListenSocket&) = delete;
    ListenSocket& operator=(const ListenSocket&) = delete;
    ListenSocket(ListenSocket&&) noexcept = default;
    ListenSocket& operator=(ListenSocket&&) noexcept = default;

    // 블로킹될 상황이거나 인터럽트가 발생하면 nullopt를 반환하고, 그 외에는 독립적으로
    // 소유하는 비차단 연결을 반환한다. 네이티브 API 실패 시 ConnectionError를 던진다.
    std::optional<ConnectedSocket> accept();
};

} // namespace comm
