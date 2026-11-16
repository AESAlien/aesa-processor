#include <beam/controller/beam_controller.hpp>
#include <pose/domain/factory/radar_attitude_factory.hpp>
#include <beam/dto/log/beam_event_logger.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

beam::RequestBeamCommand makeRequestBeamCommand(
    beam::RequestBeamType beamType,
    std::chrono::milliseconds transmitTime,
    uint32_t requestId,
    math::Angle azimuth,
    math::Angle elevation
) {
    return beam::RequestBeamCommand::Builder()
        .beamType(beamType)
        .transmitTime(transmitTime)
        .requestId(requestId)
        .direction_ant(math::SphericalDirection(azimuth, elevation))
        .build();
}

pose::SetAntEnuTransformCommand makeAntEnuTransformCommand(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const pose::RadarAttitudeFactory factory(pose::RadarAttitudeFactoryConfig(89_deg));
    return pose::SetAntEnuTransformCommand(
        pose::AntEnuTransform(
            factory.create(roll, math::TiltAngle(pitch), yaw),
            pose::AntEnuTransformConfig(math::SquareMatrix::identity(3))
        )
    );
}

beam::SearchGridConfig::Builder validSearchGridConfigBuilder()
{
    return beam::SearchGridConfig::Builder()
        .azimuthCount(21)
        .elevationCount(9)
        .azimuthStart(-42_deg)
        .azimuthStep(4.2_deg)
        .elevationStart(3_deg)
        .elevationStep(4.4_deg)
        .beamWidth(beam::BeamWidth(6_deg, 6_deg));
}

beam::RequestBeamConfig::Builder validRequestBeamConfigBuilder()
{
    return beam::RequestBeamConfig::Builder()
        .confirmationBeamWidth(beam::BeamWidth(3_deg, 3_deg))
        .trackingBeamWidth(beam::BeamWidth(3_deg, 3_deg));
}

// 기본 추적 영역: 안테나 기준 방위각 -60~60도, 고각 -10~60도
beam::RequestCoverageConfig::Builder validRequestCoverageConfigBuilder()
{
    return beam::RequestCoverageConfig::Builder()
        .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg))
        .elevationRange(math::AngleRange(-10_deg, 60_deg));
}

beam::BeamConfig::Builder validBeamConfigBuilder()
{
    return beam::BeamConfig::Builder()
        .searchGridConfig(validSearchGridConfigBuilder().build())
        .requestBeamConfig(validRequestBeamConfigBuilder().build())
        .requestCoverageConfig(validRequestCoverageConfigBuilder().build());
}

beam::BeamController makeDefaultController()
{
    return beam::BeamController(validBeamConfigBuilder().build());
}

// 컨트롤러가 호출한 기록 이벤트를 호출 순서대로 모은다.
class SpyBeamEventLogger final : public beam::BeamEventLogger
{
public:
    void logOperationStateChanged(beam::OperationState state) noexcept override
    {
        events.emplace_back(state == beam::OperationState::ON ? "STATE_ON" : "STATE_OFF");
    }

    void logAttitudeApplied() noexcept override
    {
        events.emplace_back("ATTITUDE_APPLIED");
    }

    void logRequestBeamRejected(const beam::RequestBeamResult& result) noexcept override
    {
        events.emplace_back("REQUEST_REJECTED");
        rejectedResults.push_back(result);
    }

    std::vector<std::string> events;
    std::vector<beam::RequestBeamResult> rejectedResults;
};

} // namespace

// 컨트롤러는 운용 시작 전 송신 명령을 반환하지 않는다.
TEST(BeamControllerTest, ReturnsNothingBeforeStart)
{
    // Given: 초기 상태의 컨트롤러를 준비한다.
    auto controller = makeDefaultController();

    // When: 송신 명령을 요청한다.
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 송신할 명령이 없다.
    EXPECT_FALSE(command.has_value());
}

// 컨트롤러는 운용 시작과 자세 수신 후 탐색 명령을 반환한다.
TEST(BeamControllerTest, ReturnsSearchBeamAfterOnAndAttitude)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));

    // When: 송신 명령을 요청한다.
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 컨트롤러는 운용 시작과 자세 수신 후 탐색 명령을 반환한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 0u);
    EXPECT_EQ(command->commandCount, 1u);
}

// 컨트롤러는 자세 명령을 반영한 탐색 방향을 반환한다.
TEST(BeamControllerTest, AttitudeCommandChangesSearchDirection)
{
    // Given: 세 자세 필드의 전달 여부를 확인할 수 있는 단일 빔 입력을 준비한다.
    struct AttitudeCase
    {
        math::Angle gridAzimuth;
        math::Angle roll;
        math::Angle pitch;
        math::Angle yaw;
        math::Angle expectedAzimuth;
        math::Angle expectedElevation;
    };
    const AttitudeCase cases[] =
    {
        {0_deg, 0_deg, 20_deg, 90_deg, 90_deg, 20_deg}, // 정면의 피치와 요 회전
        {90_deg, 10_deg, 0_deg, 30_deg, 120_deg, -10_deg} // 오른쪽 방향의 롤과 요 회전
    };

    for (const auto& attitudeCase : cases)
    {
        const auto config = validBeamConfigBuilder()
            .searchGridConfig(validSearchGridConfigBuilder()
                .azimuthCount(1)
                .elevationCount(1)
                .azimuthStart(attitudeCase.gridAzimuth)
                .azimuthStep(0_deg)
                .elevationStart(0_deg)
                .elevationStep(0_deg)
                .build())
            .build();
        auto controller = beam::BeamController(config);
        controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));

        // When: 롤과 피치 및 요가 지정된 자세 명령을 전달하고 송신 명령을 꺼낸다.
        controller.setAntEnuTransform(makeAntEnuTransformCommand(attitudeCase.roll, attitudeCase.pitch, attitudeCase.yaw));
        const auto command = controller.nextBeamCommand(0ms);

        // Then: 알려진 축 방향의 회전 결과가 송신 명령에 반영된다.
        ASSERT_TRUE(command.has_value());
        EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(attitudeCase.expectedAzimuth));
        EXPECT_EQ(command->direction_ant.elevation, attitudeCase.expectedElevation);
    }
}

// 컨트롤러는 받은 요청 명령을 송신 명령에 반영한다.
TEST(BeamControllerTest, RequestBeamIsDeliveredToBeamScheduler)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));

    // When: 추적 빔을 요청하고 예정 시각의 송신 명령을 꺼낸다.
    controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 100ms, 5, 7.5_deg, 15.5_deg));

    const auto command = controller.nextBeamCommand(100ms);

    // Then: 컨트롤러는 받은 요청 명령을 송신 명령에 반영한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 5u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(7.5_deg));
    EXPECT_EQ(command->direction_ant.elevation, 15.5_deg);
}

// 컨트롤러는 꺼진 상태에서 받은 요청을 이후 송신하지 않는다.
TEST(BeamControllerTest, RequestWhileOffIsIgnored)
{
    // Given: 꺼진 컨트롤러와 유효한 추적 요청을 준비한다.
    auto controller = makeDefaultController();
    const auto request = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 15.5_deg);

    // When: 꺼진 상태에서 요청한 뒤 운용과 자세를 설정한다.
    const auto result = controller.requestBeam(request);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 준비 전 결과를 반환하고 거부된 요청 대신 탐색 빔을 송신한다.
    EXPECT_EQ(result.status, beam::RequestBeamStatus::NOT_READY);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
}

// 컨트롤러는 꺼진 상태에서도 요청 검증 예외를 전달한다.
TEST(BeamControllerTest, InvalidRequestThrowsEvenWhileOff)
{
    // Given: 꺼진 컨트롤러와 요청 ID가 잘못된 명령을 준비한다.
    auto controller = makeDefaultController();
    const auto request = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 0, 7.5_deg, 15.5_deg);

    // When: 잘못된 요청을 접수한다.
    const auto requestBeam = [&] { return controller.requestBeam(request); };

    // Then: 준비 전 결과 대신 검증 예외를 전달한다.
    EXPECT_THROW(requestBeam(), std::invalid_argument);
}

// 컨트롤러에서 거부된 요청은 송신 상태를 바꾸지 않는다.
TEST(BeamControllerTest, InvalidRequestIsNotQueued)
{
    // Given: 운용 준비된 컨트롤러와 요청 ID가 잘못된 명령을 준비한다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    const auto request = makeRequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 0, 7.5_deg, 15.5_deg);

    // When: 잘못된 요청을 접수한다.
    const auto requestBeam = [&] { return controller.requestBeam(request); };

    // Then: 요청 검증 예외를 전달한다.
    EXPECT_THROW(requestBeam(), std::invalid_argument);

    // When: 거부된 요청 이후 송신 명령을 요청한다.
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 거부된 요청 대신 첫 탐색 빔을 송신한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 0u);
}

// 컨트롤러는 운용 종료 시 송신을 멈추고 재시작 시 처음부터 송신한다.
TEST(BeamControllerTest, StopsAfterOffAndRestartsFromBeginningOnNextOn)
{
    // Given: 탐색 명령을 두 번 송신한 컨트롤러를 준비한다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);
    controller.nextBeamCommand(10ms);

    // When: 운용을 끄고 송신 명령을 요청한다.
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::OFF));
    const auto whileOff = controller.nextBeamCommand(20ms);

    // Then: 운용 종료 뒤에는 송신하지 않는다.
    EXPECT_FALSE(whileOff.has_value());

    // When: 운용을 다시 시작하고 송신 명령을 요청한다.
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    const auto restarted = controller.nextBeamCommand(30ms);

    // Then: 첫 탐색 빔과 첫 명령 수부터 다시 시작한다.
    ASSERT_TRUE(restarted.has_value());
    EXPECT_EQ(restarted->beamId, 0u);
    EXPECT_EQ(restarted->commandCount, 1u);
}

// 컨트롤러는 주입된 탐색 격자 설정을 송신에 반영한다.
TEST(BeamControllerTest, InjectedSearchGridConfigIsUsed)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    const auto config = validBeamConfigBuilder()
        .searchGridConfig(validSearchGridConfigBuilder()
            .azimuthCount(3)
            .elevationCount(1)
            .beamWidth(beam::BeamWidth(4_deg, 4_deg))
            .build())
        .build();
    auto controller = beam::BeamController(config);

    // When: 운용과 자세를 설정하고 사용자 격자의 명령을 꺼낸다.
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));

    // 빔이 3개뿐이므로 ID가 0, 1, 2, 0 순서로 순환한다.
    const uint32_t expectedIds[] = {0, 1, 2, 0};
    for (std::size_t index = 0; index < 4; ++index)
    {
        const auto command = controller.nextBeamCommand(std::chrono::milliseconds(10 * index));

        // Then: 컨트롤러는 주입된 탐색 격자 설정을 송신에 반영한다.
        ASSERT_TRUE(command.has_value());
        EXPECT_EQ(command->beamId, expectedIds[index]) << "index=" << index;
        EXPECT_EQ(command->beamWidth.azimuth, 4_deg);
    }
}

// 컨트롤러는 주입된 추적 빔 폭을 요청 송신에 반영한다.
TEST(BeamControllerTest, InjectedRequestBeamConfigIsUsed)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    const auto config = validBeamConfigBuilder()
        .requestBeamConfig(validRequestBeamConfigBuilder()
            .trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg))
            .build())
        .build();
    auto controller = beam::BeamController(config);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));

    // When: 추적 빔을 요청하고 송신 명령을 꺼낸다.
    controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 컨트롤러는 주입된 추적 빔 폭을 요청 송신에 반영한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamWidth.azimuth, 4_deg);
    EXPECT_EQ(command->beamWidth.elevation, 4_deg);
}

// 컨트롤러는 주입된 확인 빔 폭을 요청 송신에 반영한다.
TEST(BeamControllerTest, InjectedConfirmationBeamWidthIsUsed)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    const auto config = validBeamConfigBuilder()
        .requestBeamConfig(validRequestBeamConfigBuilder()
            .confirmationBeamWidth(beam::BeamWidth(5_deg, 5_deg))
            .trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg))
            .build())
        .build();
    auto controller = beam::BeamController(config);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));

    // When: 확인 빔을 요청하고 송신 명령을 꺼낸다.
    controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 컨트롤러는 주입된 확인 빔 폭을 요청 송신에 반영한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamWidth.azimuth, 5_deg);
    EXPECT_EQ(command->beamWidth.elevation, 5_deg);
}

// 컨트롤러 이동은 주입된 설정과 송신 진행 상태를 유지한다.
TEST(BeamControllerTest, MovedControllerKeepsInjectedConfigAndState)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    const auto config = validBeamConfigBuilder()
        .searchGridConfig(validSearchGridConfigBuilder()
            .azimuthCount(2)
            .elevationCount(1)
            .build())
        .build();
    auto controller = beam::BeamController(config);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    // When: 사용자 설정으로 운용 중인 컨트롤러를 이동하고 다음 명령들을 꺼낸다.
    auto moved = std::move(controller);
    const auto next = moved.nextBeamCommand(10ms);
    const auto wrapped = moved.nextBeamCommand(20ms);

    // Then: 컨트롤러 이동은 주입된 설정과 송신 진행 상태를 유지한다.
    ASSERT_TRUE(next.has_value());
    ASSERT_TRUE(wrapped.has_value());
    EXPECT_EQ(next->beamId, 1u);
    EXPECT_EQ(wrapped->beamId, 0u);
}

// 컨트롤러는 요청 접수 시 준비 전과 접수 성공 및 영역 밖 상태를 반환한다.
TEST(BeamControllerTest, RequestBeamReturnsResult)
{
    // Given: 초기 컨트롤러와 영역 안팎의 요청을 준비한다.
    auto controller = makeDefaultController();
    const auto inside = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 15.5_deg);
    const auto outside = makeRequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 6, 70_deg, 15.5_deg);

    // When: 운용 준비 단계별로 요청을 접수한다.
    const auto whileOff = controller.requestBeam(inside);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    const auto beforeAttitude = controller.requestBeam(inside);
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    const auto accepted = controller.requestBeam(inside);
    const auto outOfCoverage = controller.requestBeam(outside);

    // Then: 각 요청에 해당하는 접수 상태를 반환한다.
    EXPECT_EQ(whileOff.status, beam::RequestBeamStatus::NOT_READY);
    EXPECT_EQ(beforeAttitude.status, beam::RequestBeamStatus::NOT_READY);
    EXPECT_EQ(accepted.status, beam::RequestBeamStatus::ACCEPTED);
    EXPECT_EQ(outOfCoverage.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
}

// 컨트롤러는 영역 밖으로 거부된 요청을 송신하지 않는다.
TEST(BeamControllerTest, RequestOutsideCoverageIsNotTransmitted)
{
    // Given: 운용 준비된 컨트롤러와 영역 밖 요청을 준비한다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    const auto request = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 0_deg, 61_deg);

    // When: 영역 밖 요청을 접수한 뒤 송신 명령을 꺼낸다.
    const auto result = controller.requestBeam(request);
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 영역 밖 결과를 반환하고 탐색을 계속한다.
    EXPECT_EQ(result.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
}

// 컨트롤러는 전달받은 자세를 요청 영역 판정에 반영한다.
TEST(BeamControllerTest, CoverageUsesAttitudeOfAttitudeCommand)
{
    // Given: 검사할 자세와 설정으로 운용 준비된 컨트롤러를 만든다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 13.02429_deg));

    // ENU 75도는 안테나 기준 약 62도이므로 거부된다.

    // When: 설정된 영역 안팎의 요청을 접수한다.
    const auto result1 = controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 75_deg, 0_deg));
    // ENU -50도는 ENU 기준이면 안이지만 안테나 기준 약 -63도이므로 거부된다.
    const auto result2 = controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 6, -50_deg, 0_deg));

    // Then: 컨트롤러는 전달받은 자세를 요청 영역 판정에 반영한다.
    EXPECT_EQ(result1.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(result2.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
}

// 컨트롤러는 주입된 요청 영역 설정을 접수 결과에 반영한다.
TEST(BeamControllerTest, InjectedRequestCoverageConfigIsUsed)
{
    // Given: 검사할 자세와 설정으로 운용 준비된 컨트롤러를 만든다.
    const auto config = validBeamConfigBuilder()
        .requestCoverageConfig(validRequestCoverageConfigBuilder()
            .azimuthRange(math::CircularAngleRange(-5_deg, 5_deg))
            .elevationRange(math::AngleRange(0_deg, 10_deg))
            .build())
        .build();
    auto controller = beam::BeamController(config);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));

    // 기본 영역(-60~60도)이면 받아들일 7.5도가 이 설정(-5~5도)에서는 거부된다.

    // When: 설정된 영역 안팎의 요청을 접수한다.
    const auto result1 = controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 5_deg));
    const auto result2 = controller.requestBeam(makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 6, 4.5_deg, 5_deg));

    // Then: 컨트롤러는 주입된 요청 영역 설정을 접수 결과에 반영한다.
    EXPECT_EQ(result1.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(result2.status, beam::RequestBeamStatus::ACCEPTED);
}

// 컨트롤러는 영역 밖 요청이라도 잘못된 요청이면 예외를 전달한다.
TEST(BeamControllerTest, InvalidRequestThrowsBeforeCoverageCheck)
{
    // Given: 운용 준비된 컨트롤러와 ID가 잘못된 영역 밖 요청을 준비한다.
    auto controller = makeDefaultController();
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    const auto request = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 0, 90_deg, 0_deg);

    // When: 잘못된 요청을 접수한다.
    const auto requestBeam = [&] { return controller.requestBeam(request); };

    // Then: 영역 밖 결과 대신 요청 검증 예외를 전달한다.
    EXPECT_THROW(requestBeam(), std::invalid_argument);
}

// 컨트롤러의 모든 접수 결과에는 해당 요청의 ID가 포함된다.
TEST(BeamControllerTest, RequestBeamResultCarriesRequestIdForEveryStatus)
{
    // Given: 각 접수 상태를 구분할 요청 ID를 가진 명령들을 준비한다.
    auto controller = makeDefaultController();
    const auto first = makeRequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 120ms, 11, 7.5_deg, 15.5_deg);
    const auto second = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 1000ms, 12, 7.5_deg, 15.5_deg);
    const auto third = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 2000ms, 13, 90_deg, 15.5_deg);

    // When: 준비 전과 준비 후에 영역 안팎의 요청을 접수한다.
    const auto notReady = controller.requestBeam(first);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    const auto accepted = controller.requestBeam(second);
    const auto outOfCoverage = controller.requestBeam(third);

    // Then: 모든 결과가 상태와 함께 원래 요청 ID를 반환한다.
    EXPECT_EQ(notReady.status, beam::RequestBeamStatus::NOT_READY);
    EXPECT_EQ(notReady.requestId, first.requestId);
    EXPECT_EQ(accepted.status, beam::RequestBeamStatus::ACCEPTED);
    EXPECT_EQ(accepted.requestId, second.requestId);
    EXPECT_EQ(outOfCoverage.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(outOfCoverage.requestId, third.requestId);
}

// 컨트롤러는 운용 상태가 바뀐 경우만 기록한다.
TEST(BeamControllerTest, LogsOnlyChangedOperationState)
{
    // Given: 기록기를 주입한 꺼진 컨트롤러를 준비한다.
    const auto logger = std::make_shared<SpyBeamEventLogger>();
    beam::BeamController controller(validBeamConfigBuilder().build(), logger);
    const auto on = beam::SetOperationStateCommand(beam::OperationState::ON);
    const auto off = beam::SetOperationStateCommand(beam::OperationState::OFF);

    // When: OFF, ON, ON, OFF를 차례로 전달한다.
    controller.setOperationState(off);
    controller.setOperationState(on);
    controller.setOperationState(on);
    controller.setOperationState(off);

    // Then: 같은 상태의 중복 수신은 기록하지 않는다.
    const std::vector<std::string> expected = {"STATE_ON", "STATE_OFF"};
    EXPECT_EQ(logger->events, expected);
}

// 컨트롤러는 운용 상태에 관계없이 자세 반영을 기록한다.
TEST(BeamControllerTest, LogsEveryRadarTransformUpdate)
{
    // Given: 기록기를 주입한 꺼진 컨트롤러와 유효한 자세정보를 준비한다.
    const auto logger = std::make_shared<SpyBeamEventLogger>();
    beam::BeamController controller(validBeamConfigBuilder().build(), logger);
    const auto attitude = makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg);

    // When: OFF 상태, ON 후 첫 번째, ON 후 두 번째에 자세정보를 전달한다.
    controller.setAntEnuTransform(attitude);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(attitude);
    controller.setAntEnuTransform(attitude);

    // Then: 운용 상태 변경 전후의 자세 반영을 기록한다.
    const std::vector<std::string> expected = {
        "ATTITUDE_APPLIED",
        "STATE_ON",
        "ATTITUDE_APPLIED",
        "ATTITUDE_APPLIED"
    };
    EXPECT_EQ(logger->events, expected);
}

// 컨트롤러는 버려진 요청만 결과와 함께 기록한다.
TEST(BeamControllerTest, LogsOnlyRejectedRequests)
{
    // Given: 기록기를 주입한 컨트롤러와 각 접수 상태를 만들 요청들을 준비한다.
    const auto logger = std::make_shared<SpyBeamEventLogger>();
    beam::BeamController controller(validBeamConfigBuilder().build(), logger);
    const auto notReady = makeRequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 120ms, 11, 7.5_deg, 15.5_deg);
    const auto accepted = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 1000ms, 12, 7.5_deg, 15.5_deg);
    const auto outOfCoverage = makeRequestBeamCommand(beam::RequestBeamType::TRACKING, 2000ms, 13, 90_deg, 15.5_deg);

    // When: 준비 전과 준비 후에 영역 안팎의 요청을 접수한다.
    controller.requestBeam(notReady);
    controller.setOperationState(beam::SetOperationStateCommand(beam::OperationState::ON));
    controller.setAntEnuTransform(makeAntEnuTransformCommand(0_deg, 0_deg, 0_deg));
    controller.requestBeam(accepted);
    controller.requestBeam(outOfCoverage);

    // Then: 수락된 요청은 기록하지 않고, 버려진 요청은 상태와 요청 ID를 기록한다.
    ASSERT_EQ(logger->rejectedResults.size(), 2u);
    EXPECT_EQ(logger->rejectedResults[0].status, beam::RequestBeamStatus::NOT_READY);
    EXPECT_EQ(logger->rejectedResults[0].requestId, notReady.requestId);
    EXPECT_EQ(logger->rejectedResults[1].status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(logger->rejectedResults[1].requestId, outOfCoverage.requestId);
}
