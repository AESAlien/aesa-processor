#include <beam/controller/beam_controller.hpp>

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"
#include "scheduler/search_grid.hpp"

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
    return beam::SetOperationStateCommand::Builder()
        .powerState(state)
        .build();
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
        .direction_ant(math::SphericalDirection(math::Azimuth(azimuth), math::Elevation(elevation)))
        .build();
}

} // namespace

TEST(BeamControllerTest, ReturnsNothingBeforeStart)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));

    EXPECT_FALSE(controller.nextBeamCommand(0ms).has_value());
}

TEST(BeamControllerTest, ReturnsSearchBeamAfterOnAndAttitude)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
    EXPECT_EQ(command->commandCount, 1u);
}

TEST(BeamControllerTest, AttitudeCommandFieldsReachTheSearchGrid)
{
    const math::Angle roll = 10_deg;
    const math::Angle pitch = 20_deg;
    const math::Angle yaw = 30_deg;

    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(roll, pitch, yaw));

    const beam::RadarAttitude attitude(37.0_deg, 127.0_deg, 0.1_km, roll, pitch, yaw);
    beam::SearchGrid expectedGrid(
        beam::AntEnuTransform(
            attitude,
            beam::AttitudeConfig(
                89_deg,
                math::SquareMatrix::identity(3),
                math::Tolerance{1e-6, 0.0})),
        beam::SearchGridConfig(21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))));

    for (std::size_t index = 0; index < 5; ++index)
    {
        const auto command = controller.nextBeamCommand(std::chrono::milliseconds(10 * index));
        const beam::SearchBeam& expected = expectedGrid.nextBeam();

        ASSERT_TRUE(command.has_value());
        EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(expected.direction_enu.azimuth)) << "index=" << index;
        EXPECT_EQ(command->direction_ant.elevation, expected.direction_enu.elevation) << "index=" << index;
    }
}

TEST(BeamControllerTest, RequestBeamIsDeliveredToScheduler)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 100ms, 5, 7.5_deg, 15.5_deg));

    const auto command = controller.nextBeamCommand(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 5u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(7.5_deg));
    EXPECT_EQ(command->direction_ant.elevation, 15.5_deg);
}

TEST(BeamControllerTest, RequestWhileOffIsIgnored)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 15.5_deg));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    EXPECT_EQ(controller.nextBeamCommand(0ms)->beamType, beam::BeamType::SEARCH);
}

TEST(BeamControllerTest, InvalidRequestThrowsEvenWhileOff)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));

    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 0, 7.5_deg, 15.5_deg)),
        std::invalid_argument);
    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, -1ms, 5, 7.5_deg, 15.5_deg)),
        std::invalid_argument);
    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(static_cast<beam::RequestBeamType>(0), 0ms, 5, 7.5_deg, 15.5_deg)),
        std::invalid_argument);
}

TEST(BeamControllerTest, InvalidRequestIsNotQueued)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 0, 7.5_deg, 15.5_deg)),
        std::invalid_argument);

    // 거부된 요청은 큐에 남지 않으므로 정상적으로 탐색 빔이 나간다.
    const auto command = controller.nextBeamCommand(0ms);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
}

TEST(BeamControllerTest, StopsAfterOffAndRestartsFromBeginningOnNextOn)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);
    controller.nextBeamCommand(10ms);

    controller.setOperationState(makeOperationState(beam::OperationState::OFF));
    EXPECT_FALSE(controller.nextBeamCommand(20ms).has_value());

    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 1u);
    EXPECT_EQ(command->commandCount, 1u);
}

TEST(BeamControllerTest, MovedControllerKeepsState)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    beam::BeamController moved = std::move(controller);
    const auto command = moved.nextBeamCommand(10ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 2u);
}

// ---------- 설정 주입 ----------

TEST(BeamControllerTest, InjectedSearchGridConfigIsUsed)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg))))
        .searchGridConfig(beam::SearchGridConfig::Builder()
            .azimuthCount(21)
            .elevationCount(9)
            .azimuthStart(math::Azimuth(-42_deg))
            .azimuthStep(4.2_deg)
            .elevationStart(math::Elevation(3_deg))
            .elevationStep(4.4_deg)
            .beamWidth(beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg)))
            .azimuthCount(3)
            .elevationCount(1)
            .beamWidth(beam::BeamWidth(math::AngularWidth(4_deg), math::AngularWidth(4_deg)))
            .build())
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // 빔이 3개뿐이므로 ID가 1, 2, 3, 1 순서로 순환한다.
    const uint32_t expectedIds[] = {1, 2, 3, 1};
    for (std::size_t index = 0; index < 4; ++index)
    {
        const auto command = controller.nextBeamCommand(std::chrono::milliseconds(10 * index));

        ASSERT_TRUE(command.has_value());
        EXPECT_EQ(command->beamId, expectedIds[index]) << "index=" << index;
        EXPECT_EQ(command->beamWidth.azimuth, 4_deg);
    }
}

TEST(BeamControllerTest, InjectedRequestBeamConfigIsUsed)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg))))
        .requestBeamConfig(beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(4_deg), math::AngularWidth(4_deg)))
            .build())
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::TRACKING, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamWidth.azimuth, 4_deg);
    EXPECT_EQ(command->beamWidth.elevation, 4_deg);
}

TEST(BeamControllerTest, InjectedConfirmationBeamWidthIsUsed)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg))))
        .requestBeamConfig(beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(5_deg), math::AngularWidth(5_deg)))
            .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(4_deg), math::AngularWidth(4_deg)))
            .build())
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    controller.requestBeam(makeRequestCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamType::CONFIRMATION);
    EXPECT_EQ(command->beamWidth.azimuth, 5_deg);
    EXPECT_EQ(command->beamWidth.elevation, 5_deg);
}

TEST(BeamControllerTest, InjectedAttitudeConfigIsUsedForGimbalLock)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg))))
        .attitudeConfig(beam::AttitudeConfig(
            60_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));

    // 기본 설정(89도)에서는 통과하는 pitch 70도가 이 설정(60도)에서는 짐벌락이다.
    try
    {
        controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg));
        ADD_FAILURE() << "AttitudeError not thrown";
    }
    catch (const beam::AttitudeError& error)
    {
        EXPECT_EQ(error.code(), beam::AttitudeErrorCode::GIMBAL_LOCK);
    }
    EXPECT_FALSE(controller.nextBeamCommand(0ms).has_value());

    EXPECT_NO_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 50_deg, 0_deg)));
    EXPECT_TRUE(controller.nextBeamCommand(10ms).has_value());
}

TEST(BeamControllerTest, GimbalLockThrowsEvenWhileOffOrAfterTableCreated)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg))))
        .attitudeConfig(beam::AttitudeConfig(
            60_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .build();
    beam::BeamController controller(config);

    // OFF 상태에서도 설정 기준의 짐벌락 자세는 거부한다.
    EXPECT_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg)), beam::AttitudeError);

    // 탐색 빔 테이블이 만들어진 뒤에도 마찬가지이며, 기존 상태는 유지된다.
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    ASSERT_EQ(controller.nextBeamCommand(0ms)->beamId, 1u);
    EXPECT_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg)), beam::AttitudeError);
    EXPECT_EQ(controller.nextBeamCommand(10ms)->beamId, 2u);
}

TEST(BeamControllerTest, InvalidAttitudeThrowsEvenWhileOff)
{
    beam::BeamController controller(beam::BeamConfig(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))),
        beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))));

    // 위도 95도는 RadarAttitude 생성 단계에서 거부된다. 운용 상태가 OFF여도 마찬가지다.
    const beam::SetAttitudeCommand command(95_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg);
    try
    {
        controller.setAttitude(command);
        ADD_FAILURE() << "AttitudeError not thrown";
    }
    catch (const beam::AttitudeError& error)
    {
        EXPECT_EQ(error.code(), beam::AttitudeErrorCode::OUT_OF_RANGE);
    }
}

TEST(BeamControllerTest, ControllerWithInjectedConfigCanBeMoved)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}))
        .searchGridConfig(beam::SearchGridConfig(
            21, 9, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .requestBeamConfig(beam::RequestBeamConfig(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)), beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg))))
        .searchGridConfig(beam::SearchGridConfig(
            2, 1, math::Azimuth(-42_deg), 4.2_deg, math::Elevation(3_deg), 4.4_deg, beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg))))
        .build();
    beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    beam::BeamController moved = std::move(controller);

    EXPECT_EQ(moved.nextBeamCommand(10ms)->beamId, 2u);
    EXPECT_EQ(moved.nextBeamCommand(20ms)->beamId, 1u);
}
