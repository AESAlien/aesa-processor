#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <socket/socket.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace comm
{

// 수락한 비차단 TCP 연결 하나를 나타낸다. 이동되거나 닫힌 뒤에는
// 이동 대입, 닫기, 소멸만 지원한다. 리스닝 작업은 지원하지 않는다.
class ConnectedSocket : public Socket
{
public:
    ConnectedSocket(const ConnectedSocket&) = delete;
    ConnectedSocket& operator=(const ConnectedSocket&) = delete;
    ConnectedSocket(ConnectedSocket&&) noexcept = default;
    ConnectedSocket& operator=(ConnectedSocket&&) noexcept = default;

    // 연결 수락 시 저장한 상대의 엔드포인트를 반환한다.
    // 주소 문자열 변환에 실패하면 std::system_error를 던진다.
    CommEndpoint getRemoteEndpoint() const;

    // 쓰기 준비가 완료되면 반환한다. 타임아웃 시 std::errc::timed_out을 담은
    // std::system_error를 던지며, 다른 네이티브 API 오류에도
    // std::system_error를 던진다. 인터럽트 시 최초의 제한 시각 안에서 재시도한다.
    // 음수 타임아웃은 거부하며, 0이면 한 번만 확인한다.
    void waitWritable(std::chrono::microseconds timeout) const;

    // 인터럽트가 발생해도 호출자가 지정한 절대 제한 시각을 사용한다.
    // 제한 시각이 지났으면 한 번만 확인한다. 예외 처리 규칙은
    // waitWritable()과 같다. 이 타임아웃은 이후의 입출력에는 적용하지 않는다.
    void waitWritableUntil(std::chrono::steady_clock::time_point deadline) const;

    // 입력을 복사하지 않고 offset부터 끝까지 송신한다. 대용량 데이터는
    // 일부만 송신될 수 있으며, 내부적으로 네이티브 API의 크기 제한을 적용한다.
    // offset이 data.size()와 같으면 API를 호출하지 않고 0을 반환한다.
    // offset이 data.size()보다 크면 std::out_of_range를 던진다.
    // 송신이 블로킹될 상황이거나 인터럽트가 발생하면 nullopt를 반환한다.
    // 그 밖의 실패 시 std::system_error를 던진다.
    std::optional<std::size_t> sendBytes(
        const std::vector<uint8_t>& data,
        std::size_t offset = 0
    );

    // 최대 maximumBytes만큼 한 번 수신하고 바이트 묶음의 소유권을 반환한다.
    // 완전한 메시지가 아닐 수도 있다. maximumBytes는 0보다 커야 한다.
    // 버퍼를 할당하기 전에 네이티브 API의 호출당 크기 제한을 적용한다.
    // 빈 벡터는 상대의 연결 종료를 뜻한다. 수신이 블로킹될 상황이거나
    // 인터럽트가 발생하면 nullopt를 반환한다. 그 밖의 실패 시 std::system_error를 던진다.
    std::optional<std::vector<uint8_t>> receiveBytes(std::size_t maximumBytes);

    // 핸들을 해제하지 않고 양방향 통신을 중단한다.
    void shutdown();

private:
    friend class ListenSocket;
    ConnectedSocket(SocketHandle handle, uint32_t remoteAddress, uint16_t remotePort);

    uint32_t _remoteAddress;
    uint16_t _remotePort;
};

} // namespace comm
