#pragma once

#include <system_error>

namespace comm
{

// 네이티브 소켓 오류 하나를 저장한 값이다. 조회 시 네이티브 오류 상태를 다시 읽지 않는다.
// 소켓 작업이 실패한 직후 captureLast()를 호출해야 한다.
class SocketError
{
public:
    static SocketError captureLast() noexcept;
    explicit SocketError(int errorCode) noexcept;

    bool isInterrupted() const noexcept;
    bool isWouldBlock() const noexcept;
    std::error_code code() const noexcept;

private:
    std::error_code _code;
};

} // namespace comm
