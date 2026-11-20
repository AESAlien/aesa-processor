#include "worker/target_worker.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace app
{
namespace
{
using namespace std::chrono_literals;
using namespace math::literals;
using beam::OperationState;
using Time = std::chrono::milliseconds;
using Reports = std::vector<target::TrackEvent>;

target::TargetConfig targetConfig()
{
    return target::TargetConfig(
        target::AssociationConfig(1000_mps, 30_mps, 11.345),
        target::MeasurementConfig(7_m),
        target::TrackingConfig(10_mps2, 0_mps2),
        target::TrackManagementConfig(100ms, 1000ms, 3, 100)
    );
}

target::DetectionEvent detection(
    beam::BeamType type,
    std::uint32_t id,
    Time time,
    math::Length range = 1000_m
) {
    std::vector<target::DetectionEvent::Plot> plots;
    plots.emplace_back(
        math::Distance(range),
        math::CircularAngle(0_deg),
        math::TiltAngle(0_deg),
        0_mps,
        -10.0f
    );
    return target::DetectionEvent(type, id, time, std::move(plots));
}

beam::SetOperationStateCommand operation(OperationState state)
{
    return beam::SetOperationStateCommand(state);
}

std::shared_ptr<spdlog::logger> nullLogger()
{
    return std::make_shared<spdlog::logger>("target_worker_test", std::make_shared<spdlog::sinks::null_sink_mt>());
}

class TargetWorkerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // 트랙을 위경도로 보고할 수 있도록 지리 변환을 먼저 반영한다.
        step(pose::SetEnuGeoTransformCommand(pose::EnuGeoTransform(pose::GeoPosition(math::TiltAngle(0_deg), math::CircularAngle(0_deg), 100_m))));
    }

    // 입력 하나를 넣고 워커가 한 번 처리하게 한다.
    void step(TargetInput input)
    {
        _targetQueue.write(std::move(input));
        _worker.runOnce();
    }

    // 표식을 넣고 바로 읽어, 그 전에 워커가 빔 요청을 쓰지 않았는지 확인한다.
    bool noBeamWasRequested()
    {
        constexpr Time MARKER_TIME = 999999ms;
        _beamQueue.write(TickEvent{MARKER_TIME});
        const auto message = _beamQueue.read();
        const auto* marker = std::get_if<TickEvent>(&message);
        return marker != nullptr && marker->currentTime == MARKER_TIME;
    }

    beam::RequestBeamCommand requestedBeam()
    {
        return std::get<beam::RequestBeamCommand>(_beamQueue.read());
    }

    // 틱을 처리하게 하고 콘솔로 보낸 트랙 보고를 꺼낸다.
    Reports reportAt(Time time)
    {
        step(TickEvent{time});
        return std::get<Reports>(_consoleQueue.read());
    }

    // 탐색 결과로 잠정 트랙을 만들고 확인 결과로 확정한 뒤 트랙 ID를 반환한다.
    std::uint32_t confirmTrack(Time searchTime, math::Length range = 1000_m)
    {
        step(detection(beam::BeamType::SEARCH, 1, searchTime, range));
        const auto confirmation = requestedBeam();
        EXPECT_EQ(confirmation.beamType, beam::RequestBeamType::CONFIRMATION);

        step(detection(beam::BeamType::CONFIRMATION, confirmation.requestId, confirmation.transmitTime, range));
        const auto tracking = requestedBeam();
        EXPECT_EQ(tracking.beamType, beam::RequestBeamType::TRACKING);
        return confirmation.requestId;
    }

    target::TargetController _controller{targetConfig()};
    MessageQueue<TargetInput> _targetQueue;
    MessageQueue<BeamInput> _beamQueue;
    MessageQueue<ConsoleInput> _consoleQueue;
    TargetWorker _worker{_controller, _targetQueue, _beamQueue, _consoleQueue, nullLogger(), nullptr};
};

// 운용 명령을 받기 전에는 탐지가 와도 트랙을 만들지 않는다.
TEST_F(TargetWorkerTest, CreatesNoTrackBeforeTurnedOn)
{
    step(detection(beam::BeamType::SEARCH, 1, 0ms));

    EXPECT_TRUE(noBeamWasRequested());
    EXPECT_TRUE(reportAt(100ms).empty());
}

// ON 상태에서는 기존처럼 탐지로 트랙을 만들고 확정 트랙을 보고한다.
TEST_F(TargetWorkerTest, TracksDetectionsWhileOn)
{
    step(operation(OperationState::ON));

    const auto id = confirmTrack(0ms);

    const auto reports = reportAt(100ms);
    ASSERT_EQ(reports.size(), 1u);
    EXPECT_EQ(reports.front().id, id);
}

// OFF가 되면 잠정·확정 트랙을 모두 지우고, 이전 트랙의 늦은 결과는 다시 ON이 되어도 무시한다.
TEST_F(TargetWorkerTest, TurningOffClearsAllTracks)
{
    step(operation(OperationState::ON));
    const auto confirmedId = confirmTrack(0ms);
    step(detection(beam::BeamType::SEARCH, 2, 200ms, 2000_m));
    const auto tentativeId = requestedBeam().requestId;
    ASSERT_EQ(reportAt(200ms).size(), 1u);

    step(operation(OperationState::OFF));
    EXPECT_TRUE(reportAt(300ms).empty());

    step(operation(OperationState::ON));
    step(detection(beam::BeamType::TRACKING, confirmedId, 1100ms));
    step(detection(beam::BeamType::CONFIRMATION, tentativeId, 300ms, 2000_m));
    EXPECT_TRUE(noBeamWasRequested());
    EXPECT_TRUE(reportAt(1100ms).empty());
}

// OFF 중에 온 탐지는 버려서 새 트랙을 만들지 않는다.
TEST_F(TargetWorkerTest, DropsDetectionsWhileOff)
{
    step(operation(OperationState::ON));
    step(operation(OperationState::OFF));

    step(detection(beam::BeamType::SEARCH, 1, 0ms));

    EXPECT_TRUE(noBeamWasRequested());
    EXPECT_TRUE(reportAt(100ms).empty());
}

// 같은 명령을 반복해서 받아도 트랙을 지우지 않는다.
TEST_F(TargetWorkerTest, RepeatedOnDoesNotClearTracks)
{
    step(operation(OperationState::ON));
    const auto id = confirmTrack(0ms);

    step(operation(OperationState::ON));
    step(operation(OperationState::ON));

    const auto reports = reportAt(100ms);
    ASSERT_EQ(reports.size(), 1u);
    EXPECT_EQ(reports.front().id, id);
}

// OFF를 반복해서 받아도 다시 ON이 되면 정상적으로 추적한다.
TEST_F(TargetWorkerTest, RepeatedOffKeepsStoppedUntilTurnedOnAgain)
{
    step(operation(OperationState::OFF));
    step(operation(OperationState::OFF));
    step(detection(beam::BeamType::SEARCH, 1, 0ms));
    EXPECT_TRUE(noBeamWasRequested());

    step(operation(OperationState::ON));
    const auto id = confirmTrack(1000ms);

    const auto reports = reportAt(1100ms);
    ASSERT_EQ(reports.size(), 1u);
    EXPECT_EQ(reports.front().id, id);
}

// ON/OFF를 연속으로 바꿔도 이전 트랙이 남지 않고, ON마다 새 탐지부터 다음 ID로 추적한다.
TEST_F(TargetWorkerTest, RestartsFromNewDetectionsOnEveryTurnOn)
{
    std::uint32_t previousId = 0;
    for (int cycle = 0; cycle < 3; ++cycle)
    {
        const auto base = Time(10000 * cycle);
        step(operation(OperationState::ON));
        const auto id = confirmTrack(base);
        EXPECT_GT(id, previousId) << "cycle " << cycle;
        previousId = id;

        const auto reports = reportAt(base + 100ms);
        ASSERT_EQ(reports.size(), 1u) << "cycle " << cycle;
        EXPECT_EQ(reports.front().id, id) << "cycle " << cycle;

        step(operation(OperationState::OFF));
        EXPECT_TRUE(reportAt(base + 200ms).empty()) << "cycle " << cycle;
    }
}

} // namespace
} // namespace app
