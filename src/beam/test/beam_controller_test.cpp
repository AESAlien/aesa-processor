#include <beam/controller/beam_controller.hpp>

#include "attitude/ant_enu_transform.hpp"
#include "scheduler/beam_table.hpp"

#include <gtest/gtest.h>

#include <chrono>
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

} // namespace

TEST(BeamControllerTest, ReturnsNothingBeforeStart)
{
    beam::BeamController controller;

    EXPECT_FALSE(controller.nextBeamCommand(0ms).has_value());
}

TEST(BeamControllerTest, ReturnsSearchBeamAfterOnAndAttitude)
{
    beam::BeamController controller;
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    const auto command = controller.nextBeamCommand(0ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(command->beamId, 1u);
    EXPECT_EQ(command->commandCount, 1u);
}

TEST(BeamControllerTest, AttitudeCommandFieldsReachTheBeamTable)
{
    const math::Angle roll = 10_deg;
    const math::Angle pitch = 20_deg;
    const math::Angle yaw = 30_deg;

    beam::BeamController controller;
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(roll, pitch, yaw));

    beam::RadarAttitude attitude{};
    attitude.latitude = 37.0_deg;
    attitude.longitude = 127.0_deg;
    attitude.altitude = 0.1_km;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    const beam::BeamTable expectedTable(beam::AntEnuTransform(attitude, beam::AttitudeConfig{}));

    for (std::size_t index = 0; index < 5; ++index)
    {
        const auto command = controller.nextBeamCommand(std::chrono::milliseconds(10 * index));

        ASSERT_TRUE(command.has_value());
        EXPECT_NEAR(command->azimuth_ant.deg(), expectedTable.get(index).azimuth_ant.deg(), 1e-9) << "index=" << index;
        EXPECT_NEAR(command->elevation_ant.deg(), expectedTable.get(index).elevation_ant.deg(), 1e-9)
            << "index=" << index;
    }
}

TEST(BeamControllerTest, RequestBeamIsDeliveredToScheduler)
{
    beam::BeamController controller;
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));

    const beam::BeamRequest request = beam::BeamRequest::Builder()
        .beamType(beam::BeamRequest::BeamType::TRACKING)
        .transmitTime(100ms)
        .requestId(0)
        .azimuth_ant(7.5_deg)
        .elevation_ant(15.5_deg)
        .build();
    controller.requestBeam(request);

    const auto command = controller.nextBeamCommand(100ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamType, beam::BeamCommand::BeamType::TRACKING);
    EXPECT_DOUBLE_EQ(command->azimuth_ant.deg(), 7.5);
    EXPECT_DOUBLE_EQ(command->elevation_ant.deg(), 15.5);
}

TEST(BeamControllerTest, StopsAfterOffAndRestartsFromBeginningOnNextOn)
{
    beam::BeamController controller;
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
    beam::BeamController controller;
    controller.setOperationState(makeOperationState(beam::OperationState::ON));
    controller.setAttitude(makeAttitudeCommand(0_deg, 0_deg, 0_deg));
    controller.nextBeamCommand(0ms);

    beam::BeamController moved = std::move(controller);
    const auto command = moved.nextBeamCommand(10ms);

    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->beamId, 2u);
}
