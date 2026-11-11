#include <app/channel/comm_channel.hpp>
#include <app/queue/message_queue.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <variant>
#include <vector>

namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

beam::TransmitBeamCommand makeBeamCommand(std::uint32_t beamId)
{
    return beam::TransmitBeamCommand(
        beam::BeamType::SEARCH,
        10ms,
        beamId,
        beamId,
        math::SphericalDirection(10.0_deg, 5.0_deg),
        beam::BeamWidth(6.0_deg, 6.0_deg)
    );
}

target::TrackEvent makeTrackEvent(std::uint16_t id)
{
    return target::TrackEvent::Builder()
        .id(id)
        .state(target::TrackEvent::TrackStatus::TRACKING)
        .slantRange(math::Distance(12.5_km))
        .groundRange(math::Distance(10.25_km))
        .azimuth(15.5_deg)
        .elevation(6.25_deg)
        .dopplerVelocity(-120.5_mps)
        .latitude(37.5_deg)
        .longitude(127.25_deg)
        .altitude(2.5_km)
        .speed(math::Speed(250.75_mps))
        .heading(45.5_deg)
        .flightPathAngle(-2.25_deg)
        .timestamp(5678ms)
        .build();
}

beam::SetAttitudeCommand makeAttitudeCommand()
{
    return beam::SetAttitudeCommand::Builder()
        .latitude(37.5_deg)
        .longitude(127.25_deg)
        .altitude(1250.0_m)
        .roll(-12.5_deg)
        .pitch(6.25_deg)
        .yaw(45.5_deg)
        .build();
}

} // namespace

// 용량을 지정하지 않은 큐는 넣은 메시지를 모두 순서대로 보관한다.
TEST(MessageQueueTest, KeepsAllMessagesWithoutCapacity)
{
    // 준비: 용량 제한이 없는 큐를 준비한다.
    app::MessageQueue<int> queue;

    // 실행: 메시지 세 개를 넣는다.
    queue.push(1);
    queue.push(2);
    queue.push(3);

    // 검증: 세 메시지를 넣은 순서대로 꺼낸다.
    EXPECT_EQ(queue.pop(), 1);
    EXPECT_EQ(queue.pop(), 2);
    EXPECT_EQ(queue.pop(), 3);
}

// 용량이 찬 큐에 넣으면 가장 오래된 메시지를 버린다.
TEST(MessageQueueTest, DropsOldestMessageWhenFull)
{
    // 준비: 용량이 2인 큐를 준비한다.
    app::MessageQueue<int> queue(2);

    // 실행: 용량보다 많은 메시지를 넣고 큐를 닫는다.
    queue.push(1);
    queue.push(2);
    queue.push(3);
    queue.close();

    // 검증: 가장 오래된 메시지만 버려지고 나머지는 순서대로 꺼낸다.
    EXPECT_EQ(queue.pop(), 2);
    EXPECT_EQ(queue.pop(), 3);
    EXPECT_EQ(queue.pop(), std::nullopt);
}

// 용량이 0인 큐는 만들 수 없다.
TEST(MessageQueueTest, RejectsZeroCapacity)
{
    // 준비: 용량 0으로 큐를 만드는 동작을 준비한다.
    const auto construct = [] { app::MessageQueue<int> queue(0); };

    // 실행 및 검증: 생성을 거부한다.
    EXPECT_THROW(construct(), std::invalid_argument);
}

// 통신 채널은 이벤트를 목적지 큐에 넣고, 모든 이벤트의 복사본을 모니터 큐에 넣는다.
TEST(CommChannelTest, CopiesEveryEventToMonitorQueue)
{
    // 준비: 통신 채널을 준비한다.
    app::CommChannel channel;

    // 실행: 빔 명령, 트랙 이벤트, 자세를 차례로 쓰고 채널을 닫는다.
    channel.write(makeBeamCommand(7));
    channel.write(makeTrackEvent(42));
    channel.write(makeAttitudeCommand());
    channel.close();

    // 검증: 빔 명령은 모의기 큐에, 트랙 이벤트와 자세는 전시기 큐에 들어간다.
    const std::optional<app::CommEvent> simulatorEvent = channel.readSimulator();
    ASSERT_TRUE(simulatorEvent);
    EXPECT_TRUE(std::holds_alternative<beam::TransmitBeamCommand>(*simulatorEvent));
    EXPECT_EQ(channel.readSimulator(), std::nullopt);

    const std::optional<app::CommEvent> trackEvent = channel.readConsole();
    ASSERT_TRUE(trackEvent);
    EXPECT_TRUE(std::holds_alternative<std::vector<target::TrackEvent>>(*trackEvent));
    const std::optional<app::CommEvent> attitudeEvent = channel.readConsole();
    ASSERT_TRUE(attitudeEvent);
    EXPECT_TRUE(std::holds_alternative<beam::SetAttitudeCommand>(*attitudeEvent));
    EXPECT_EQ(channel.readConsole(), std::nullopt);

    // 검증: 모니터 큐에는 세 이벤트가 쓴 순서대로 모두 들어간다.
    const std::optional<app::CommEvent> monitorBeam = channel.readMonitor();
    ASSERT_TRUE(monitorBeam);
    ASSERT_TRUE(std::holds_alternative<beam::TransmitBeamCommand>(*monitorBeam));
    EXPECT_EQ(std::get<beam::TransmitBeamCommand>(*monitorBeam).beamId, 7U);

    const std::optional<app::CommEvent> monitorTrack = channel.readMonitor();
    ASSERT_TRUE(monitorTrack);
    ASSERT_TRUE(std::holds_alternative<std::vector<target::TrackEvent>>(*monitorTrack));
    const auto& tracks = std::get<std::vector<target::TrackEvent>>(*monitorTrack);
    ASSERT_EQ(tracks.size(), 1U);
    EXPECT_EQ(tracks.front().id, 42);

    const std::optional<app::CommEvent> monitorAttitude = channel.readMonitor();
    ASSERT_TRUE(monitorAttitude);
    EXPECT_TRUE(std::holds_alternative<beam::SetAttitudeCommand>(*monitorAttitude));
    EXPECT_EQ(channel.readMonitor(), std::nullopt);
}

// 모니터가 이벤트를 읽지 않아도 모니터 큐는 용량을 넘지 않고 최근 이벤트를 유지한다.
TEST(CommChannelTest, MonitorQueueKeepsMostRecentEventsWhenFull)
{
    // 준비: 통신 채널과 모니터 큐 용량보다 하나 많은 빔 명령 수를 준비한다.
    app::CommChannel channel;
    const std::size_t writeCount = app::CommChannel::MONITOR_QUEUE_CAPACITY + 1;

    // 실행: 모니터 큐를 읽지 않은 채 빔 명령을 쓰고 채널을 닫는다.
    for (std::size_t index = 0; index < writeCount; ++index)
    {
        channel.write(makeBeamCommand(static_cast<std::uint32_t>(index)));
    }
    channel.close();

    // 검증: 모의기 큐는 모든 빔 명령을 유지한다.
    std::size_t simulatorCount = 0;
    while (channel.readSimulator())
    {
        ++simulatorCount;
    }
    EXPECT_EQ(simulatorCount, writeCount);

    // 검증: 모니터 큐는 가장 오래된 빔 명령 하나만 버리고 용량만큼 유지한다.
    std::vector<std::uint32_t> monitorBeamIds;
    while (const std::optional<app::CommEvent> event = channel.readMonitor())
    {
        monitorBeamIds.push_back(std::get<beam::TransmitBeamCommand>(*event).beamId);
    }
    ASSERT_EQ(monitorBeamIds.size(), app::CommChannel::MONITOR_QUEUE_CAPACITY);
    EXPECT_EQ(monitorBeamIds.front(), 1U);
    EXPECT_EQ(monitorBeamIds.back(), static_cast<std::uint32_t>(writeCount - 1));
}

// 모니터 상태 요청은 모니터 큐에만 넣는다.
TEST(CommChannelTest, RoutesMonitorStatusOnlyToMonitorQueue)
{
    // 준비: 통신 채널과 모니터 상태 요청을 준비한다.
    app::CommChannel channel;
    comm::MonitorStatusRequest status;
    status.processorTime = 42310ms;

    // 실행: 상태 요청을 쓰고 채널을 닫는다.
    channel.write(status);
    channel.close();

    // 검증: 모의기와 전시기 큐는 비어 있고 모니터 큐에만 상태 요청이 들어간다.
    EXPECT_EQ(channel.readSimulator(), std::nullopt);
    EXPECT_EQ(channel.readConsole(), std::nullopt);
    const std::optional<app::CommEvent> monitorEvent = channel.readMonitor();
    ASSERT_TRUE(monitorEvent);
    ASSERT_TRUE(std::holds_alternative<comm::MonitorStatusRequest>(*monitorEvent));
    EXPECT_EQ(std::get<comm::MonitorStatusRequest>(*monitorEvent).processorTime, 42310ms);
    EXPECT_EQ(channel.readMonitor(), std::nullopt);
}
