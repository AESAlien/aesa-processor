#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/session.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace comm
{

// 설정된 상대 객체는 TCP 재연결 후에도 유지되며 현재 세션을 보유한다.
// 세션의 교체와 해제는 수신 스레드에서만 수행하고,
// 송신 스레드는 세션을 읽기만 한다. 세션의 공유 포인터를 복사해 두면
// 송신 측이 사용하는 동안 이전 연결도 유지된다.
class PeerConnection
{
public:
    PeerConnection(std::string name, CommEndpoint endpoint);

    PeerConnection(const PeerConnection&) = delete;
    PeerConnection& operator=(const PeerConnection&) = delete;
    PeerConnection(PeerConnection&&) = delete;
    PeerConnection& operator=(PeerConnection&&) = delete;

    const std::string& getName() const noexcept;
    bool matches(const CommEndpoint& remoteEndpoint) const;
    // 세션의 원격 엔드포인트를 이 상대의 엔드포인트와 비교한다.
    bool matches(const Session& session) const;

    // 연결이 끊긴 상태에서는 아무 작업도 하지 않는다. 송신 실패 시 SendError를 던진다.
    // Session의 요구에 따라 상대마다 송신 스레드를 하나만 사용한다.
    void send(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

    // 현재 세션에서 한 번 수신하고 완성된 프레임을 반환한다.
    // 연결이 종료되면 세션을 해제한다. 연결이 끊긴 상태에서는 프레임을 반환하지 않는다.
    // ReceiveError를 던질 수 있다. 수신 스레드에서만 호출한다.
    std::vector<MessageFrame> receive();

    // 현재 세션이 있으면 true를 반환한다. 어느 스레드에서든 호출할 수 있다.
    bool isConnected() const;

    // 현재 세션을 반환한다. 연결이 끊긴 상태이면 nullptr를 반환한다.
    std::shared_ptr<Session> getSession() const;

    // 현재 세션을 교체한다. nullptr를 전달하면 해제한다. 이전 세션은
    // 잠금 밖에서 해제한다. 수신 스레드에서만 호출한다.
    void setSession(std::shared_ptr<Session> session);

private:
    std::string _name;
    CommEndpoint _endpoint;
    mutable std::mutex _mutex;
    std::shared_ptr<Session> _session;
};

} // namespace comm
