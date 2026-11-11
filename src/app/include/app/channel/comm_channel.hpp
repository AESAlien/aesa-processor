#pragma once

#include <app/queue/message_queue.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/transmit_beam_command.hpp>
#include <comm/dto/monitor_status_request.hpp>
#include <target/dto/track_event.hpp>

#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

namespace app
{
// TransmitBeamCommand는 모의기 큐에, MonitorStatusRequest는 모니터 큐에만,
// 나머지는 전시기 큐에 넣는다. 모니터 큐에는 모든 이벤트의 복사본을 넣는다.
using CommEvent = std::variant<
    beam::TransmitBeamCommand,
    std::vector<target::TrackEvent>,
    beam::SetAttitudeCommand,
    comm::MonitorStatusRequest>;

class CommChannel final
{
public:
    // 모니터가 느리거나 연결되지 않아도 메모리가 계속 늘지 않도록
    // 이 개수를 넘으면 가장 오래된 모니터 이벤트를 버린다.
    static constexpr std::size_t MONITOR_QUEUE_CAPACITY = 1000;

    void write(CommEvent message);

    void write(const target::TrackEvent& message);

    std::optional<CommEvent> readSimulator();

    std::optional<CommEvent> readConsole();

    std::optional<CommEvent> readMonitor();

    void close();

private:
    MessageQueue<CommEvent> _simulatorQueue;
    MessageQueue<CommEvent> _consoleQueue;
    MessageQueue<CommEvent> _monitorQueue{MONITOR_QUEUE_CAPACITY};
};
} // namespace app