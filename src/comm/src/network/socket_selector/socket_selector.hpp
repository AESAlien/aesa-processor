#pragma once

#include <network/socket/socket.hpp>

#include <cstddef>

#include <chrono>
#include <vector>

namespace comm
{

// 소유권을 넘겨받지 않고 소켓을 참조한다. 선택기가 살아 있는 동안 소켓은
// 열린 채로 유지되어야 하며 이동되면 안 된다. 같은 선택기로
// 소켓을 다시 등록하지 않고 반복해서 대기할 수 있다.
class SocketSelector
{
public:
    // 널 포인터와 플랫폼 제한을 초과하는 소켓 개수를 거부한다.
    explicit SocketSelector(std::vector<const Socket*> sockets);

    // 준비된 소켓을 반환한다. 타임아웃이나 인터럽트가 발생하면 빈 벡터를 반환하고,
    // 그 밖의 네이티브 API 실패 시 std::system_error를 던진다. 음수 타임아웃은
    // 거부하며, 0이면 블로킹 없이 확인한다.
    std::vector<const Socket*> waitReadable(std::chrono::microseconds timeout) const;

    // 타임아웃과 인터럽트 처리 규칙은 waitReadable()과 같다.
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
