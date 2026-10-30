#include <beam/controller/beam_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

beam::SetOperationStateCommand makeOperationState(beam::OperationState state)
{
    return beam::SetOperationStateCommand(state);
}

beam::SetAttitudeCommand makeAttitudeCommand(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::SetAttitudeCommand::Builder()
        .latitude(37.0_deg)
        .longitude(127.0_deg)
        .altitude(0.1_km)
        .roll(roll)
        .pitch(pitch)
        .yaw(yaw)
        .build();
}

beam::RequestBeamCommand makeRequestCommand(
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

// 기본 추적 영역: 안테나 기준 방위각 -60~60도, 고각 -10~60도
beam::RequestCoverageConfig makeCoverageConfig()
{
    return beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(-10_deg, 60_deg));
}

beam::BeamController makeDefaultController()
{
    return beam::BeamController(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)),
        beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)),
        makeCoverageConfig()));
}

} // namespace

// 컨트롤러는 운용 시작 전 송신 명령을 반환하지 않는다.
TEST(BeamControllerTest, ReturnsNothingBeforeStart)
{
    // Given: 초기 상태의 컨트롤러를 준비한다.
    beam::BeamController controller = makeDefaultController();

    // When: 송신 명령을 요청한다.
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 송신할 명령이 없다.
    EXPECT_FALSE(command.has_value());
}

// 컨트롤러는 운용 시작과 자세 수신 후 탐색 명령을 반환한다.
TEST(BeamControllerTest, ReturnsSearchBeamAfterOnAndAttitude)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)),
        beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)),
        makeCoverageConfig()));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // When: 송신 명령을 요청한다.
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 컨트롤러는 운용 시작과 자세 수신 후 탐색 명령을 반환한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
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
        const beam::BeamConfig config(
            beam::AttitudeConfig(89_deg, math::SquareMatrix::identity(3), math::Tolerance{1e-6, 0.0}),
            beam::SearchGridConfig(
                1, 1, math::CircularAngle(attitudeCase.gridAzimuth), 0_deg, math::TiltAngle(0_deg), 0_deg,
                beam::BeamWidth(6_deg, 6_deg)),
            beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)),
            makeCoverageConfig());
        beam::BeamController controller(config);
        controller.setOperationState(makeOperationState(beam::OperationState::ON));

        // When: 롤과 피치 및 요가 지정된 자세 명령을 전달하고 송신 명령을 꺼낸다.
        controller.setAttitude(makeAttitudeCommand(attitudeCase.roll, attitudeCase.pitch, attitudeCase.yaw));
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
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)),
        beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)),
        makeCoverageConfig()));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // When: 추적 빔을 요청하고 예정 시각의 송신 명령을 꺼낸다.
    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 100ms, 5, 7.5_deg, 15.5_deg));

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
    beam::BeamController controller = makeDefaultController();
    const auto request = makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 15.5_deg);

    // When: 꺼진 상태에서 요청한 뒤 운용과 자세를 설정한다.
    const auto result = controller.requestBeam(request);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
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
    beam::BeamController controller = makeDefaultController();
    const auto request = makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 0, 7.5_deg, 15.5_deg);

    // When: 잘못된 요청을 접수한다.
    const auto requestBeam = [&] { return controller.requestBeam(request); };

    // Then: 준비 전 결과 대신 검증 예외를 전달한다.
    EXPECT_THROW(requestBeam(), std::invalid_argument);
}

// 컨트롤러에서 거부된 요청은 송신 상태를 바꾸지 않는다.
TEST(BeamControllerTest, InvalidRequestIsNotQueued)
{
    // Given: 운용 준비된 컨트롤러와 요청 ID가 잘못된 명령을 준비한다.
    beam::BeamController controller = makeDefaultController();
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    const auto request = makeRequestCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 0, 7.5_deg, 15.5_deg);

    // When: 잘못된 요청을 접수한다.
    const auto requestBeam = [&] { return controller.requestBeam(request); };

    // Then: 요청 검증 예외를 전달한다.
    EXPECT_THROW(requestBeam(), std::invalid_argument);

    // When: 거부된 요청 이후 송신 명령을 요청한다.
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 거부된 요청 대신 첫 탐색 빔을 송신한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
}

// 컨트롤러는 운용 종료 시 송신을 멈추고 재시작 시 처음부터 송신한다.
TEST(BeamControllerTest, StopsAfterOffAndRestartsFromBeginningOnNextOn)
{
    // Given: 탐색 명령을 두 번 송신한 컨트롤러를 준비한다.
    beam::BeamController controller = makeDefaultController();
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);
    controller.nextBeamCommand(10ms);

    // When: 운용을 끄고 송신 명령을 요청한다.
    controller.setOperationState(makeOperationState(beam::OperationState::OFF));
    const auto whileOff = controller.nextBeamCommand(20ms);

    // Then: 운용 종료 뒤에는 송신하지 않는다.
    EXPECT_FALSE(whileOff.has_value());

    // When: 운용을 다시 시작하고 자세를 받아 송신 명령을 요청한다.
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    const auto restarted = controller.nextBeamCommand(30ms);

    // Then: 첫 탐색 빔과 첫 명령 수부터 다시 시작한다.
    ASSERT_TRUE(restarted.has_value());
    EXPECT_EQ(restarted->beamId, 1u);
    EXPECT_EQ(restarted->commandCount, 1u);
}

// ---------- 설정 주입 ----------

// 컨트롤러는 주입된 탐색 격자 설정을 송신에 반영한다.
TEST(BeamControllerTest, InjectedSearchGridConfigIsUsed)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(makeCoverageConfig())
        .searchGridConfig(beam::SearchGridConfig::Builder()
            .azimuthCount(21)
            .elevationCount(9)
            .azimuthStart(math::CircularAngle(-42_deg))
            .azimuthStep(4.2_deg)
            .elevationStart(math::TiltAngle(3_deg))
            .elevationStep(4.4_deg)
            .beamWidth(beam::BeamWidth(6_deg, 6_deg))
            .azimuthCount(3)
            .elevationCount(1)
            .beamWidth(beam::BeamWidth(4_deg, 4_deg))
            .build())
        .build();
    beam::BeamController controller(config);

    // When: 운용과 자세를 설정하고 사용자 격자의 명령을 꺼낸다.
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // 빔이 3개뿐이므로 ID가 1, 2, 3, 1 순서로 순환한다.
    const uint32_t expectedIds[] = {1, 2, 3, 1};
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
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(makeCoverageConfig())
        .requestBeamConfig(beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .trackingBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg))
            .build())
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // When: 추적 빔을 요청하고 송신 명령을 꺼낸다.
    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 7, 7.5_deg, 15.5_deg));
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
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(makeCoverageConfig())
        .requestBeamConfig(beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .trackingBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .confirmationBeamWidth(beam::BeamWidth(5_deg, 5_deg))
            .trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg))
            .build())
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // When: 확인 빔을 요청하고 송신 명령을 꺼낸다.
    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    // Then: 컨트롤러는 주입된 확인 빔 폭을 요청 송신에 반영한다.
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamWidth.azimuth, 5_deg);
    EXPECT_EQ(command->beamWidth.elevation, 5_deg);
}

// 컨트롤러는 주입된 피치 한계의 검증 결과를 반영한다.
TEST(BeamControllerTest, InjectedAttitudeConfigIsUsedForGimbalLock)
{
    // Given: 사용자 피치 한계를 가진 컨트롤러를 준비한다.
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(makeCoverageConfig())
        .attitudeConfig(beam::AttitudeConfig(
            60_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));

    // 기본 설정(89도)에서는 통과하는 pitch 70도가 이 설정(60도)에서는 짐벌락이다.
    const auto invalidAttitude = makeAttitudeCommand(0_deg, 70_deg, 0_deg);

    // When: 설정된 피치 한계를 넘는 자세를 전달한다.
    const auto setInvalidAttitude = [&] { controller.setAttitude(invalidAttitude); };

    // Then: 자세 검증 예외를 전달한다.
    EXPECT_THROW(setInvalidAttitude(), std::invalid_argument);
    EXPECT_FALSE(controller.nextBeamCommand(0ms).has_value());

    // When: 유효한 자세를 다시 전달한다.
    EXPECT_NO_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 50_deg, 0_deg)));

    // Then: 유효한 자세를 받은 뒤에는 송신할 수 있다.
    EXPECT_TRUE(controller.nextBeamCommand(10ms).has_value());
}

// 컨트롤러는 운용 상태와 관계없이 자세 검증 예외를 전달한다.
TEST(BeamControllerTest, GimbalLockThrowsEvenWhileOffOrAfterTableCreated)
{
    // Given: 사용자 피치 한계를 가진 컨트롤러를 준비한다.
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(makeCoverageConfig())
        .attitudeConfig(beam::AttitudeConfig(
            60_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .build();
    beam::BeamController controller(config);

    // OFF 상태에서도 설정 기준의 짐벌락 자세는 거부한다. 음의 pitch도 마찬가지다.
    const auto invalidAttitude = makeAttitudeCommand(0_deg, 70_deg, 0_deg);

    // When: 설정된 피치 한계를 넘는 자세를 전달한다.
    const auto setInvalidAttitude = [&] { controller.setAttitude(invalidAttitude); };

    // Then: 자세 검증 예외를 전달한다.
    EXPECT_THROW(setInvalidAttitude(), std::invalid_argument);
    EXPECT_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, -70_deg, 0_deg)), std::invalid_argument);

    // 탐색 빔 테이블이 만들어진 뒤에도 마찬가지이며, 기존 상태는 유지된다.

    // When: 운용을 시작하고 정상 자세를 받아 탐색을 진행한다.
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // Then: 첫 탐색 명령을 반환한다.
    ASSERT_EQ(controller.nextBeamCommand(0ms)->beamId, 1u);

    // When: 이미 준비된 상태에서 잘못된 자세를 전달한다.
    EXPECT_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg)), std::invalid_argument);

    // Then: 예외 이후에도 기존 송신 상태는 유지된다.
    EXPECT_EQ(controller.nextBeamCommand(10ms)->beamId, 2u);
}

// 컨트롤러는 꺼진 상태에서도 유효하지 않은 자세의 예외를 전달한다.
TEST(BeamControllerTest, InvalidAttitudeThrowsEvenWhileOff)
{
    // Given: 꺼진 컨트롤러와 잘못된 롤 값을 가진 자세 명령을 준비한다.
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)),
        beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)),
        makeCoverageConfig()));

    // roll 181도는 RadarAttitude 생성 단계에서 거부된다. 운용 상태가 OFF여도 마찬가지다.
    const beam::SetAttitudeCommand command(
        math::TiltAngle(37_deg), math::CircularAngle(127_deg), 0.1_km,
        181_deg, math::TiltAngle(0_deg), 0_deg);

    // When: 잘못된 자세 명령을 전달한다.
    const auto setAttitude = [&] { controller.setAttitude(command); };

    // Then: 컨트롤러는 꺼진 상태에서도 유효하지 않은 자세의 예외를 전달한다.
    EXPECT_THROW(setAttitude(), std::invalid_argument);
}

// 컨트롤러의 빔 방향은 자세 명령의 위치 필드에 영향을 받지 않는다.
TEST(BeamControllerTest, IgnoresLocationFieldsWhenSettingAttitude)
{
    // Given: 같은 설정의 컨트롤러 두 개와 서로 다른 위치의 자세 명령을 준비한다.
    beam::BeamController first = makeDefaultController();
    beam::BeamController second = makeDefaultController();
    first.setOperationState(makeOperationState(beam::OperationState::ON));
    second.setOperationState(makeOperationState(beam::OperationState::ON));
    const auto defaultLocation = makeAttitudeCommand(0_deg, 0_deg, 0_deg);
    const beam::SetAttitudeCommand otherLocation(
        math::TiltAngle(-30_deg), math::CircularAngle(-179_deg), 101_km,
        0_deg, math::TiltAngle(0_deg), 0_deg);

    // When: 같은 자세와 서로 다른 위치를 전달하고 명령을 꺼낸다.
    first.setAttitude(defaultLocation);
    second.setAttitude(otherLocation);
    const auto firstCommand = first.nextBeamCommand(0ms);
    const auto secondCommand = second.nextBeamCommand(0ms);

    // Then: 위치와 관계없이 같은 탐색 방향을 반환한다.
    ASSERT_TRUE(firstCommand.has_value());
    ASSERT_TRUE(secondCommand.has_value());
    EXPECT_EQ(firstCommand->direction_ant.azimuth, secondCommand->direction_ant.azimuth);
    EXPECT_EQ(firstCommand->direction_ant.elevation, secondCommand->direction_ant.elevation);
}

// 컨트롤러 이동은 주입된 설정과 송신 진행 상태를 유지한다.
TEST(BeamControllerTest, MovedControllerKeepsInjectedConfigAndState)
{
    // Given: 검사에 필요한 설정과 운용 상태의 컨트롤러를 준비한다.
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(makeCoverageConfig())
        .searchGridConfig(beam::SearchGridConfig(
            2, 1, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    // When: 사용자 설정으로 운용 중인 컨트롤러를 이동하고 다음 명령들을 꺼낸다.
    beam::BeamController moved = std::move(controller);
    const auto next = moved.nextBeamCommand(10ms);
    const auto wrapped = moved.nextBeamCommand(20ms);

    // Then: 컨트롤러 이동은 주입된 설정과 송신 진행 상태를 유지한다.
    ASSERT_TRUE(next.has_value());
    ASSERT_TRUE(wrapped.has_value());
    EXPECT_EQ(next->beamId, 2u);
    EXPECT_EQ(wrapped->beamId, 1u);
}

// ---------- 추적 영역 판정 ----------

// 컨트롤러는 요청 접수 시 준비 전과 접수 성공 및 영역 밖 상태를 반환한다.
TEST(BeamControllerTest, RequestBeamReturnsResult)
{
    // Given: 초기 컨트롤러와 영역 안팎의 요청을 준비한다.
    beam::BeamController controller = makeDefaultController();
    const auto inside = makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 15.5_deg);
    const auto outside = makeRequestCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 6, 70_deg, 15.5_deg);

    // When: 운용 준비 단계별로 요청을 접수한다.
    const auto whileOff = controller.requestBeam(inside);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    const auto beforeAttitude = controller.requestBeam(inside);
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
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
    beam::BeamController controller = makeDefaultController();
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    const auto request = makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 0_deg, 61_deg);

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
    beam::BeamController controller = makeDefaultController();
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 13.02429_deg));

    // ENU 75도는 안테나 기준 약 62도이므로 거부된다.

    // When: 설정된 영역 안팎의 요청을 접수한다.
    const auto result1 = controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 75_deg, 0_deg));
    // ENU -50도는 ENU 기준이면 안이지만 안테나 기준 약 -63도이므로 거부된다.
    const auto result2 = controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 6, -50_deg, 0_deg));

    // Then: 컨트롤러는 전달받은 자세를 요청 영역 판정에 반영한다.
    EXPECT_EQ(result1.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(result2.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
}

// 컨트롤러는 주입된 요청 영역 설정을 접수 결과에 반영한다.
TEST(BeamControllerTest, InjectedRequestCoverageConfigIsUsed)
{
    // Given: 검사할 자세와 설정으로 운용 준비된 컨트롤러를 만든다.
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::CircularAngle(-42_deg), 4.2_deg, math::TiltAngle(3_deg), 4.4_deg, beam::BeamWidth(6_deg, 6_deg)))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(3_deg, 3_deg), beam::BeamWidth(3_deg, 3_deg)))
        .requestCoverageConfig(beam::RequestCoverageConfig(math::CircularAngleRange(-5_deg, 5_deg), math::AngleRange(0_deg, 10_deg)))
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // 기본 영역(-60~60도)이면 받아들일 7.5도가 이 설정(-5~5도)에서는 거부된다.

    // When: 설정된 영역 안팎의 요청을 접수한다.
    const auto result1 = controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 5_deg));
    const auto result2 = controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 6, 4.5_deg, 5_deg));

    // Then: 컨트롤러는 주입된 요청 영역 설정을 접수 결과에 반영한다.
    EXPECT_EQ(result1.status, beam::RequestBeamStatus::OUT_OF_COVERAGE);
    EXPECT_EQ(result2.status, beam::RequestBeamStatus::ACCEPTED);
}

// 컨트롤러는 영역 밖 요청이라도 잘못된 요청이면 예외를 전달한다.
TEST(BeamControllerTest, InvalidRequestThrowsBeforeCoverageCheck)
{
    // Given: 운용 준비된 컨트롤러와 ID가 잘못된 영역 밖 요청을 준비한다.
    beam::BeamController controller = makeDefaultController();
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    const auto request = makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 0, 90_deg, 0_deg);

    // When: 잘못된 요청을 접수한다.
    const auto requestBeam = [&] { return controller.requestBeam(request); };

    // Then: 영역 밖 결과 대신 요청 검증 예외를 전달한다.
    EXPECT_THROW(requestBeam(), std::invalid_argument);
}

// 컨트롤러의 모든 접수 결과에는 해당 요청의 ID가 포함된다.
TEST(BeamControllerTest, RequestBeamResultCarriesRequestIdForEveryStatus)
{
    // Given: 각 접수 상태를 구분할 요청 ID를 가진 명령들을 준비한다.
    beam::BeamController controller = makeDefaultController();
    const auto first = makeRequestCommand(beam::RequestBeamType::CONFIRMATION, 120ms, 11, 7.5_deg, 15.5_deg);
    const auto second = makeRequestCommand(beam::RequestBeamType::TRACKING, 1000ms, 12, 7.5_deg, 15.5_deg);
    const auto third = makeRequestCommand(beam::RequestBeamType::TRACKING, 2000ms, 13, 90_deg, 15.5_deg);

    // When: 준비 전과 준비 후에 영역 안팎의 요청을 접수한다.
    const auto notReady = controller.requestBeam(first);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
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
