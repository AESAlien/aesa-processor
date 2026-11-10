#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace comm
{

#ifdef _WIN32
using SocketHandle = std::uintptr_t;
constexpr SocketHandle INVALID_SOCKET_HANDLE = ~static_cast<SocketHandle>(0);
#else
using SocketHandle = int;
constexpr SocketHandle INVALID_SOCKET_HANDLE = -1;
#endif

// Winsock / POSIX 소켓 API를 감싼 간단한 함수들이다.
// int를 반환하는 함수는 별도 설명이 없으면 네이티브 API의 규칙을 따르며 실패 시 음수를 반환한다.
// 오류 원인은 lastError()로 확인한다.
// IPv4 주소는 네트워크 바이트 순서이고, 포트는 호스트 바이트 순서이다.
namespace native_socket
{

// 성공 시 0을, 실패 시 오류 코드를 반환한다.
int startup();

// 성공 시 0을, 실패 시 오류 코드를 반환한다.
int cleanup();

int lastError();
bool isInterrupted(int errorCode);
bool isWouldBlock(int errorCode);
int defaultBacklog() noexcept;

// 플랫폼에서 이 개수만큼 등록할 수 없으면 std::invalid_argument를 던진다.
void validateRegistrationCount(std::size_t count);

std::optional<std::string> formatIpv4(uint32_t address);

SocketHandle createTcpSocket();
int setReuseAddress(SocketHandle socket);
int setNonBlocking(SocketHandle socket);
int setAbortiveClose(SocketHandle socket);
int bindSocket(SocketHandle socket, uint32_t address, uint16_t port);
int connectSocket(SocketHandle socket, uint32_t address, uint16_t port);
int listenSocket(SocketHandle socket, int backlog);

SocketHandle acceptSocket(
    SocketHandle listenSocket,
    uint32_t& remoteAddress,
    uint16_t& remotePort
);

// 대기 함수는 readySockets를 비우고, 음수 타임아웃이나 select에 등록할 수 없는 핸들을
// 거부한다. 등록된 소켓이 없으면 0을 반환한다.
// select()의 결과를 반환하고 읽기 가능한 소켓으로 readySockets를 채운다.
int waitReadable(
    const std::vector<SocketHandle>& sockets,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
);

// select()의 결과를 반환하고 쓰기 가능한 소켓으로 readySockets를 채운다.
int waitWritable(
    const std::vector<SocketHandle>& sockets,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
);

// 동일한 검증 규칙을 적용하여 소켓 하나의 쓰기 준비를 기다린다.
int waitWritable(
    SocketHandle socket,
    std::chrono::microseconds timeout
);

int sendBytes(SocketHandle socket, const uint8_t* data, int size);
int receiveBytes(SocketHandle socket, uint8_t* buffer, int size);

// 핸들을 해제하지 않고 양방향 통신을 중단한다.
void shutdownSocket(SocketHandle socket);

// 양방향 통신을 중단한 뒤 소켓을 닫는다.
int closeSocket(SocketHandle socket);

} // namespace native_socket
} // namespace comm
