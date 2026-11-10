#pragma once

#include <socket/native_socket.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace comm::native_socket::test
{

struct ErrorCodes
{
    int interrupted;
    int wouldBlock;
    int tryAgain;
    int connectionReset;
};

ErrorCodes errorCodes() noexcept;
void setLastError(int errorCode) noexcept;
bool supportsBoundedSendBuffer() noexcept;

// 독립적인 네이티브 상대 소켓이다. 소켓 API의 수명은 테스트에서 관리한다.
class NativePeer
{
public:
    NativePeer();
    ~NativePeer();
    NativePeer(const NativePeer&) = delete;
    NativePeer& operator=(const NativePeer&) = delete;

    void close();
    int bind(uint16_t port = 0);
    int setReuseAddress();
    // close()가 TIME_WAIT를 남기지 않고 연결을 리셋하도록 한다.
    int setAbortiveClose();
    int connect(uint16_t port);
    int setReceiveBufferSize(int size);
    uint16_t localPort() const;
    int waitReadable() const;
    int sendByte(uint8_t value) const;
    int receiveByte(uint8_t& value) const;
    int receiveBytes(uint8_t* buffer, std::size_t size) const;

private:
    SocketHandle _handle;
};

class SignalInterruptor
{
public:
    static bool isSupported() noexcept;
    SignalInterruptor();
    ~SignalInterruptor();
    SignalInterruptor(const SignalInterruptor&) = delete;
    SignalInterruptor& operator=(const SignalInterruptor&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace comm::native_socket::test
