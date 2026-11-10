#pragma once

#include <network/native_socket/native_socket.hpp>

namespace comm
{

// 리스닝 소켓과 연결된 소켓의 공통 소유자이다.
class Socket
{
public:
    virtual ~Socket();

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    // 핸들을 빌려준다. 소유권은 이 소켓에 남는다.
    SocketHandle getHandle() const noexcept;
    void close() noexcept;

protected:
    Socket();
    // 즉시 소유권을 넘겨받으며, 초기화에 실패해도 소유권을 책임진다.
    explicit Socket(SocketHandle handle);
    Socket(Socket&& other) noexcept;
    Socket& operator=(Socket&& other) noexcept;

    SocketHandle _handle = INVALID_SOCKET_HANDLE;

private:
    void initializeApi();
    void cleanupApi() noexcept;
    void validate() const;

    bool _apiInitialized = false;
};

} // namespace comm
