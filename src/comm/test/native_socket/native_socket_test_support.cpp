#include <native_socket/native_socket_test_support.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifdef __linux__
#include <pthread.h>
#include <signal.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace comm::native_socket::test
{
namespace
{

sockaddr_in loopbackAddress(uint16_t port)
{
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    return address;
}

#ifdef __linux__
void handleTestSignal(int)
{
}
#endif

} // namespace

ErrorCodes errorCodes() noexcept
{
#ifdef _WIN32
    return {WSAEINTR, WSAEWOULDBLOCK, WSAEWOULDBLOCK, WSAECONNRESET};
#else
    return {EINTR, EWOULDBLOCK, EAGAIN, ECONNRESET};
#endif
}

void setLastError(int errorCode) noexcept
{
#ifdef _WIN32
    WSASetLastError(errorCode);
#else
    errno = errorCode;
#endif
}

bool supportsBoundedSendBuffer() noexcept
{
    // 일부 소켓 공급자는 SO_RCVBUF 설정에도 불구하고 부하 테스트 데이터 전체를 버퍼링한다.
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

NativePeer::NativePeer()
  : _handle(static_cast<SocketHandle>(::socket(AF_INET, SOCK_STREAM, 0)))
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        throw std::runtime_error("Could not create native test peer");
    }
}

NativePeer::~NativePeer()
{
    close();
}

void NativePeer::close()
{
    if (_handle != INVALID_SOCKET_HANDLE)
    {
#ifdef _WIN32
        closesocket(_handle);
#else
        ::close(_handle);
#endif
        _handle = INVALID_SOCKET_HANDLE;
    }
}

int NativePeer::bind(uint16_t port)
{
    const sockaddr_in address = loopbackAddress(port);
    return ::bind(_handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
}

int NativePeer::setReuseAddress()
{
    const int enabled = 1;
    return setsockopt(
        _handle,
        SOL_SOCKET,
        SO_REUSEADDR,
        reinterpret_cast<const char*>(&enabled),
        sizeof(enabled)
    );
}

int NativePeer::setAbortiveClose()
{
    return comm::native_socket::setAbortiveClose(_handle);
}

int NativePeer::connect(uint16_t port)
{
    const sockaddr_in address = loopbackAddress(port);
    return ::connect(_handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
}

int NativePeer::setReceiveBufferSize(int size)
{
#ifdef _WIN32
    const char* optionValue = reinterpret_cast<const char*>(&size);
#else
    const int* optionValue = &size;
#endif
    return setsockopt(_handle, SOL_SOCKET, SO_RCVBUF, optionValue, sizeof(size));
}

uint16_t NativePeer::localPort() const
{
    sockaddr_in address{};
#ifdef _WIN32
    int length = sizeof(address);
#else
    socklen_t length = sizeof(address);
#endif
    if (getsockname(_handle, reinterpret_cast<sockaddr*>(&address), &length) < 0)
    {
        throw std::runtime_error("Could not read native test peer address");
    }
    return ntohs(address.sin_port);
}

int NativePeer::waitReadable() const
{
    fd_set sockets;
    FD_ZERO(&sockets);
    FD_SET(_handle, &sockets);
    timeval timeout{};
    timeout.tv_sec = 2;
#ifdef _WIN32
    return select(0, &sockets, nullptr, nullptr, &timeout);
#else
    return select(_handle + 1, &sockets, nullptr, nullptr, &timeout);
#endif
}

int NativePeer::sendByte(uint8_t value) const
{
    return static_cast<int>(::send(_handle, reinterpret_cast<const char*>(&value), 1, 0));
}

int NativePeer::receiveByte(uint8_t& value) const
{
    return static_cast<int>(recv(_handle, reinterpret_cast<char*>(&value), 1, 0));
}

int NativePeer::receiveBytes(uint8_t* buffer, std::size_t size) const
{
    const int nativeSize = static_cast<int>(std::min(size, std::size_t{65536}));
    return static_cast<int>(recv(_handle, reinterpret_cast<char*>(buffer), nativeSize, 0));
}

class SignalInterruptor::Impl
{
public:
#ifdef __linux__
    Impl()
    {
        struct sigaction action{};
        action.sa_handler = handleTestSignal;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGUSR1, &action, &_previous) < 0)
        {
            throw std::runtime_error("Could not install test signal handler");
        }
        const pthread_t target = pthread_self();
        try
        {
            _sender = std::thread([this, target]
            {
                while (!_stop.load())
                {
                    pthread_kill(target, SIGUSR1);
                    std::this_thread::sleep_for(std::chrono::milliseconds{1});
                }
            }
            );
        }
        catch (...)
        {
            sigaction(SIGUSR1, &_previous, nullptr);
            throw;
        }
    }

    ~Impl()
    {
        _stop.store(true);
        _sender.join();
        sigaction(SIGUSR1, &_previous, nullptr);
    }

private:
    struct sigaction _previous{};
    std::atomic<bool> _stop{false};
    std::thread _sender;
#endif
};

bool SignalInterruptor::isSupported() noexcept
{
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

SignalInterruptor::SignalInterruptor()
  : _impl(std::make_unique<Impl>())
{
}

SignalInterruptor::~SignalInterruptor() = default;

} // namespace comm::native_socket::test
