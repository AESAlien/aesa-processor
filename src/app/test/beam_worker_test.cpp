#include "worker/beam_worker.hpp"

#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

namespace app
{
namespace
{
using namespace std::chrono_literals;
using namespace math::literals;
using beam::OperationState;

beam::BeamConfig beamConfig()
{
    return beam::BeamConfig::Builder()
        .searchGridConfig(beam::SearchGridConfig::Builder()
            .azimuthCount(21)
            .elevationCount(9)
            .azimuthStart(-42_deg)
            .azimuthStep(4.2_deg)
            .elevationStart(3_deg)
            .elevationStep(4.4_deg)
            .beamWidth(beam::BeamWidth(6_deg, 6_deg))
            .build())
        .requestBeamConfig(beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .trackingBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .build())
        .requestCoverageConfig(beam::RequestCoverageConfig::Builder()
            .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg))
            .elevationRange(math::AngleRange(-10_deg, 60_deg))
            .build())
        .build();
}

pose::SetAntEnuTransformCommand attitude(math::Angle yaw = 0_deg)
{
    const pose::RadarAttitudeFactory factory(pose::RadarAttitudeFactoryConfig(89_deg));
    return pose::SetAntEnuTransformCommand(
        pose::AntEnuTransform(
            factory.create(0_deg, math::TiltAngle(0_deg), yaw),
            pose::AntEnuTransformConfig(math::SquareMatrix::identity(3))
        )
    );
}

beam::RequestBeamCommand trackingRequest(std::chrono::milliseconds transmitTime, std::uint32_t requestId)
{
    return beam::RequestBeamCommand::Builder()
        .beamType(beam::RequestBeamType::TRACKING)
        .transmitTime(transmitTime)
        .requestId(requestId)
        .direction_ant(math::SphericalDirection(7.5_deg, 15.5_deg))
        .build();
}

beam::SetOperationStateCommand operation(OperationState state)
{
    return beam::SetOperationStateCommand(state);
}

TickEvent tick(std::chrono::milliseconds currentTime)
{
    return TickEvent{currentTime};
}

std::shared_ptr<spdlog::logger> nullLogger()
{
    return std::make_shared<spdlog::logger>("beam_worker_test", std::make_shared<spdlog::sinks::null_sink_mt>());
}

class BeamWorkerTest : public ::testing::Test
{
protected:
    // 입력 하나를 넣고 워커가 한 번 처리하게 한다.
    void step(BeamInput input)
    {
        _beamQueue.write(std::move(input));
        _worker.runOnce();
    }

    // 표식을 넣고 바로 읽어, 그 전에 워커가 송신 명령을 쓰지 않았는지 확인한다.
    bool noBeamWasSent()
    {
        constexpr std::uint32_t MARKER_COUNT = 999999;
        _simulatorQueue.write(beam::TransmitBeamCommand::Builder()
            .beamType(beam::BeamType::SEARCH)
            .transmitTime(0ms)
            .beamId(0)
            .commandCount(MARKER_COUNT)
            .direction_ant(math::SphericalDirection(0_deg, 0_deg))
            .beamWidth(beam::BeamWidth(1_deg, 1_deg))
            .build());
        return _simulatorQueue.read().commandCount == MARKER_COUNT;
    }

    beam::TransmitBeamCommand sentBeam()
    {
        return _simulatorQueue.read();
    }

    beam::BeamController _controller{beamConfig()};
    MessageQueue<BeamInput> _beamQueue;
    MessageQueue<SimulatorInput> _simulatorQueue;
    BeamWorker _worker{_controller, _beamQueue, _simulatorQueue, nullLogger(), nullptr};
};

// 운용 명령을 받기 전에는 자세를 받아도 빔을 송신하지 않는다.
TEST_F(BeamWorkerTest, SendsNoBeamBeforeTurnedOn)
{
    step(attitude());
    step(tick(0ms));
    step(tick(10ms));

    EXPECT_TRUE(noBeamWasSent());
}

// ON이 되면 첫 탐색 빔부터 송신한다.
TEST_F(BeamWorkerTest, SendsSearchBeamsAfterTurnedOn)
{
    step(attitude());
    step(operation(OperationState::ON));
    step(tick(0ms));
    step(tick(10ms));

    const auto first = sentBeam();
    const auto second = sentBeam();
    EXPECT_EQ(first.beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(first.beamId, 0u);
    EXPECT_EQ(first.commandCount, 1u);
    EXPECT_EQ(second.beamId, 1u);
    EXPECT_EQ(second.commandCount, 2u);
}

// ON 상태에서 받은 요청은 기존처럼 요청 빔으로 송신한다.
TEST_F(BeamWorkerTest, SendsRequestedBeamWhileOn)
{
    step(attitude());
    step(operation(OperationState::ON));
    step(trackingRequest(0ms, 5));
    step(tick(0ms));

    const auto command = sentBeam();
    EXPECT_EQ(command.beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command.beamId, 5u);
}

// OFF가 되면 송신을 멈추고, 다시 ON이 되면 첫 탐색 빔과 첫 명령 수부터 시작한다.
TEST_F(BeamWorkerTest, StopsWhileOffAndRestartsFromFirstBeamWhenTurnedOnAgain)
{
    step(attitude());
    step(operation(OperationState::ON));
    step(tick(0ms));
    step(tick(10ms));
    sentBeam();
    sentBeam();

    step(operation(OperationState::OFF));
    step(tick(20ms));
    step(tick(30ms));
    EXPECT_TRUE(noBeamWasSent());

    step(operation(OperationState::ON));
    step(tick(40ms));
    const auto restarted = sentBeam();
    EXPECT_EQ(restarted.beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(restarted.beamId, 0u);
    EXPECT_EQ(restarted.commandCount, 1u);
}

// OFF 중 받은 요청과 OFF 전에 대기하던 요청은 다시 ON이 되어도 송신하지 않는다.
TEST_F(BeamWorkerTest, DropsRequestsReceivedBeforeOrWhileOff)
{
    step(attitude());
    step(operation(OperationState::ON));
    step(trackingRequest(50ms, 5));
    step(operation(OperationState::OFF));
    step(trackingRequest(50ms, 6));
    step(operation(OperationState::ON));

    step(tick(50ms));

    const auto command = sentBeam();
    EXPECT_EQ(command.beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command.beamId, 0u);
}

// OFF 중에도 자세정보를 반영해 ON 직후 최신 자세로 빔을 계산한다.
TEST_F(BeamWorkerTest, AppliesAttitudeReceivedWhileOff)
{
    step(attitude(0_deg));
    step(attitude(10_deg));
    step(operation(OperationState::ON));
    step(tick(0ms));

    const auto command = sentBeam();
    EXPECT_TRUE(command.direction_ant.azimuth.isEquivalentTo(-42_deg + 10_deg));
}

// 같은 명령을 반복해서 받아도 송신 순서와 명령 수를 초기화하지 않는다.
TEST_F(BeamWorkerTest, RepeatedCommandsDoNotResetSchedule)
{
    step(attitude());
    step(operation(OperationState::OFF));
    step(operation(OperationState::ON));
    step(tick(0ms));
    sentBeam();

    step(operation(OperationState::ON));
    step(operation(OperationState::ON));
    step(tick(10ms));

    const auto command = sentBeam();
    EXPECT_EQ(command.beamId, 1u);
    EXPECT_EQ(command.commandCount, 2u);
}

// ON/OFF를 연속으로 바꿔도 마지막 상태를 따르고, ON마다 처음부터 송신한다.
TEST_F(BeamWorkerTest, FollowsAlternatingCommands)
{
    step(attitude());

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        const auto base = std::chrono::milliseconds(100 * cycle);
        step(operation(OperationState::ON));
        step(tick(base));
        step(tick(base + 10ms));
        const auto first = sentBeam();
        const auto second = sentBeam();
        EXPECT_EQ(first.commandCount, 1u) << "cycle " << cycle;
        EXPECT_EQ(second.commandCount, 2u) << "cycle " << cycle;

        step(operation(OperationState::OFF));
        step(tick(base + 20ms));
        EXPECT_TRUE(noBeamWasSent()) << "cycle " << cycle;
    }
}

} // namespace
} // namespace app
