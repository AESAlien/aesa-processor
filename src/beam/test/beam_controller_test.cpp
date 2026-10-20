#include <beam/controller/beam_controller.hpp>

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"
#include "scheduler/search_grid.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;
using namespace aesa_processor::math::literals;

namespace
{

aesa_processor::beam::SetOperationStateCommand makeOperationState(aesa_processor::beam::OperationState state)
{
    return aesa_processor::beam::SetOperationStateCommand::Builder()
        .powerState(state)
        .build();
}

aesa_processor::beam::SetAttitudeCommand makeAttitudeCommand(aesa_processor::math::Angle roll, aesa_processor::math::Angle pitch, aesa_processor::math::Angle yaw)
{
    return aesa_processor::beam::SetAttitudeCommand::Builder()
        .latitude(37.0_deg)
        .longitude(127.0_deg)
        .altitude(0.1_km)
        .roll(roll)
        .pitch(pitch)
        .yaw(yaw)
        .build();
}

aesa_processor::beam::RequestBeamCommand makeRequestCommand(
    aesa_processor::beam::RequestBeamType beamType,
    std::chrono::milliseconds transmitTime,
    uint32_t requestId,
    aesa_processor::math::Angle azimuth,
    aesa_processor::math::Angle elevation
) {
    return aesa_processor::beam::RequestBeamCommand::Builder()
        .beamType(beamType)
        .transmitTime(transmitTime)
        .requestId(requestId)
        .direction_ant(aesa_processor::math::SphericalDirection(azimuth, elevation))
        .build();
}

} // namespace

TEST(BeamControllerTest, ReturnsNothingBeforeStart)
{
    aesa_processor::beam::BeamController controller;

    EXPECT_FALSE(controller.nextBeamCommand(0ms).has_value());
}

TEST(BeamControllerTest, ReturnsSearchBeamAfterOnAndAttitude)
{
    aesa_processor::beam::BeamController controller;
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, aesa_processor::beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
    EXPECT_EQ(command->commandCount, 1u);
}

TEST(BeamControllerTest, AttitudeCommandFieldsReachTheSearchGrid)
{
    const aesa_processor::math::Angle roll = 10_deg;
    const aesa_processor::math::Angle pitch = 20_deg;
    const aesa_processor::math::Angle yaw = 30_deg;

    aesa_processor::beam::BeamController controller;
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(roll, pitch, yaw));

    const aesa_processor::beam::RadarAttitude attitude(37.0_deg, 127.0_deg, 0.1_km, roll, pitch, yaw);
    aesa_processor::beam::SearchGrid expectedGrid(aesa_processor::beam::AntEnuTransform(attitude, aesa_processor::beam::AttitudeConfig{}), aesa_processor::beam::SearchGridConfig{});

    for (std::size_t index = 0; index < 5; ++index)
    {
        const auto command = controller.nextBeamCommand(std::chrono::milliseconds(10 * index));
        const aesa_processor::beam::SearchBeam& expected = expectedGrid.nextBeam();

        ASSERT_TRUE(command.has_value());
        EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(expected.direction_enu.azimuth)) << "index=" << index;
        EXPECT_EQ(command->direction_ant.elevation, expected.direction_enu.elevation) << "index=" << index;
    }
}

TEST(BeamControllerTest, RequestBeamIsDeliveredToScheduler)
{
    aesa_processor::beam::BeamController controller;
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::TRACKING, 100ms, 5, 7.5_deg, 15.5_deg));

    const auto command = controller.nextBeamCommand(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, aesa_processor::beam::BeamType::TRACKING);
    EXPECT_EQ(command->beamId, 5u);
    EXPECT_EQ(command->transmitTime, 100ms);
    EXPECT_TRUE(command->direction_ant.azimuth.isEquivalentTo(7.5_deg));
    EXPECT_EQ(command->direction_ant.elevation, 15.5_deg);
}

TEST(BeamControllerTest, RequestWhileOffIsIgnored)
{
    aesa_processor::beam::BeamController controller;
    controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::TRACKING, 0ms, 5, 7.5_deg, 15.5_deg));
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    EXPECT_EQ(controller.nextBeamCommand(0ms)->beamType, aesa_processor::beam::BeamType::SEARCH);
}

TEST(BeamControllerTest, InvalidRequestThrowsEvenWhileOff)
{
    aesa_processor::beam::BeamController controller;

    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::TRACKING, 0ms, 0, 7.5_deg, 15.5_deg)),
        std::invalid_argument);
    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::TRACKING, -1ms, 5, 7.5_deg, 15.5_deg)),
        std::invalid_argument);
    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(static_cast<aesa_processor::beam::RequestBeamType>(0), 0ms, 5, 7.5_deg, 15.5_deg)),
        std::invalid_argument);
}

TEST(BeamControllerTest, InvalidRequestIsNotQueued)
{
    aesa_processor::beam::BeamController controller;
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    EXPECT_THROW(
        controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::CONFIRMATION, 0ms, 0, 7.5_deg, 15.5_deg)),
        std::invalid_argument);

    // 거부된 요청은 큐에 남지 않으므로 정상적으로 탐색 빔이 나간다.
    const auto command = controller.nextBeamCommand(0ms);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, aesa_processor::beam::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
}

TEST(BeamControllerTest, StopsAfterOffAndRestartsFromBeginningOnNextOn)
{
    aesa_processor::beam::BeamController controller;
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);
    controller.nextBeamCommand(10ms);

    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::OFF));
    EXPECT_FALSE(controller.nextBeamCommand(20ms).has_value());

    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 1u);
    EXPECT_EQ(command->commandCount, 1u);
}

TEST(BeamControllerTest, MovedControllerKeepsState)
{
    aesa_processor::beam::BeamController controller;
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    aesa_processor::beam::BeamController moved = std::move(controller);
    const auto command = moved.nextBeamCommand(10ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 2u);
}

// ---------- 설정 주입 ----------

TEST(BeamControllerTest, DefaultConstructorUsesDefaultConfig)
{
    aesa_processor::beam::BeamController defaultController;
    aesa_processor::beam::BeamController explicitController{aesa_processor::beam::BeamConfig{}};
    for (aesa_processor::beam::BeamController* controller : {&defaultController, &explicitController})
    {
        controller->setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
        controller->setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    }

    for (std::size_t index = 0; index < 30; ++index)
    {
        const auto expected = explicitController.nextBeamCommand(std::chrono::milliseconds(10 * index));
        const auto actual = defaultController.nextBeamCommand(std::chrono::milliseconds(10 * index));

        ASSERT_TRUE(actual.has_value());
        ASSERT_TRUE(expected.has_value());
        EXPECT_EQ(actual->beamId, expected->beamId);
        EXPECT_TRUE(actual->direction_ant.azimuth.isEquivalentTo(expected->direction_ant.azimuth));
        EXPECT_EQ(actual->direction_ant.elevation, expected->direction_ant.elevation);
    }
}

TEST(BeamControllerTest, InjectedSearchGridConfigIsUsed)
{
    const aesa_processor::beam::BeamConfig config = aesa_processor::beam::BeamConfig::Builder()
        .searchGridConfig(aesa_processor::beam::SearchGridConfig::Builder()
            .azimuthCount(3)
            .elevationCount(1)
            .beamWidth(4_deg)
            .build())
        .build();
    aesa_processor::beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    // 빔이 3개뿐이므로 ID가 1, 2, 3, 1 순서로 순환한다.
    const uint32_t expectedIds[] = {1, 2, 3, 1};
    for (std::size_t index = 0; index < 4; ++index)
    {
        const auto command = controller.nextBeamCommand(std::chrono::milliseconds(10 * index));

        ASSERT_TRUE(command.has_value());
        EXPECT_EQ(command->beamId, expectedIds[index]) << "index=" << index;
        EXPECT_EQ(command->azimuthBeamWidth, 4_deg);
    }
}

TEST(BeamControllerTest, InjectedRequestBeamConfigIsUsed)
{
    const aesa_processor::beam::BeamConfig config = aesa_processor::beam::BeamConfig::Builder()
        .requestBeamConfig(aesa_processor::beam::RequestBeamConfig::Builder()
            .trackingBeamWidth(4_deg)
            .build())
        .build();
    aesa_processor::beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::TRACKING, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, aesa_processor::beam::BeamType::TRACKING);
    EXPECT_EQ(command->azimuthBeamWidth, 4_deg);
    EXPECT_EQ(command->elevationBeamWidth, 4_deg);
}

TEST(BeamControllerTest, InjectedConfirmationBeamWidthIsUsed)
{
    const aesa_processor::beam::BeamConfig config = aesa_processor::beam::BeamConfig::Builder()
        .requestBeamConfig(aesa_processor::beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(5_deg)
            .trackingBeamWidth(4_deg)
            .build())
        .build();
    aesa_processor::beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    controller.requestBeam(makeRequestCommand(aesa_processor::beam::RequestBeamType::CONFIRMATION, 0ms, 7, 7.5_deg, 15.5_deg));
    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, aesa_processor::beam::BeamType::CONFIRMATION);
    EXPECT_EQ(command->azimuthBeamWidth, 5_deg);
    EXPECT_EQ(command->elevationBeamWidth, 5_deg);
}

TEST(BeamControllerTest, InjectedAttitudeConfigIsUsedForGimbalLock)
{
    const aesa_processor::beam::BeamConfig config = aesa_processor::beam::BeamConfig::Builder()
        .attitudeConfig(aesa_processor::beam::AttitudeConfig::Builder().maxAbsPitch(60_deg).build())
        .build();
    aesa_processor::beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));

    // 기본 설정(89도)에서는 통과하는 pitch 70도가 이 설정(60도)에서는 짐벌락이다.
    try
    {
        controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg));
        ADD_FAILURE() << "AttitudeError not thrown";
    }
    catch (const aesa_processor::beam::AttitudeError& error)
    {
        EXPECT_EQ(error.code(), aesa_processor::beam::AttitudeErrorCode::GIMBAL_LOCK);
    }
    EXPECT_FALSE(controller.nextBeamCommand(0ms).has_value());

    EXPECT_NO_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 50_deg, 0_deg)));
    EXPECT_TRUE(controller.nextBeamCommand(10ms).has_value());
}

TEST(BeamControllerTest, GimbalLockThrowsEvenWhileOffOrAfterTableCreated)
{
    const aesa_processor::beam::BeamConfig config = aesa_processor::beam::BeamConfig::Builder()
        .attitudeConfig(aesa_processor::beam::AttitudeConfig::Builder().maxAbsPitch(60_deg).build())
        .build();
    aesa_processor::beam::BeamController controller(config);

    // OFF 상태에서도 설정 기준의 짐벌락 자세는 거부한다.
    EXPECT_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg)), aesa_processor::beam::AttitudeError);

    // 탐색 빔 테이블이 만들어진 뒤에도 마찬가지이며, 기존 상태는 유지된다.
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    ASSERT_EQ(controller.nextBeamCommand(0ms)->beamId, 1u);
    EXPECT_THROW(controller.setAttitude(makeAttitudeCommand(0_deg, 70_deg, 0_deg)), aesa_processor::beam::AttitudeError);
    EXPECT_EQ(controller.nextBeamCommand(10ms)->beamId, 2u);
}

TEST(BeamControllerTest, InvalidAttitudeThrowsEvenWhileOff)
{
    aesa_processor::beam::BeamController controller;

    // 위도 95도는 RadarAttitude 생성 단계에서 거부된다. 운용 상태가 OFF여도 마찬가지다.
    const aesa_processor::beam::SetAttitudeCommand command(95_deg, 127_deg, 0.1_km, 0_deg, 0_deg, 0_deg);
    try
    {
        controller.setAttitude(command);
        ADD_FAILURE() << "AttitudeError not thrown";
    }
    catch (const aesa_processor::beam::AttitudeError& error)
    {
        EXPECT_EQ(error.code(), aesa_processor::beam::AttitudeErrorCode::OUT_OF_RANGE);
    }
}

TEST(BeamControllerTest, ControllerWithInjectedConfigCanBeMoved)
{
    const aesa_processor::beam::BeamConfig config = aesa_processor::beam::BeamConfig::Builder()
        .searchGridConfig(aesa_processor::beam::SearchGridConfig::Builder().azimuthCount(2).elevationCount(1).build())
        .build();
    aesa_processor::beam::BeamController controller(config);
    controller.setOperationState(makeOperationState(aesa_processor::beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    aesa_processor::beam::BeamController moved = std::move(controller);

    EXPECT_EQ(moved.nextBeamCommand(10ms)->beamId, 2u);
    EXPECT_EQ(moved.nextBeamCommand(20ms)->beamId, 1u);
}
