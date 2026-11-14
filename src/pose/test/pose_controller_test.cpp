#include <pose/controller/pose_controller.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

using namespace math::literals;

namespace
{
const pose::PoseController CONTROLLER(
    pose::PoseConfig(
        pose::RadarAttitudeFactoryConfig(60_deg),
        pose::AntEnuTransformConfig(math::SquareMatrix::identity(3))
    )
);

pose::SetRadarPoseCommand makeCommand(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return pose::SetRadarPoseCommand::Builder()
        .latitude(37_deg)
        .longitude(127_deg)
        .altitude(0.1_km)
        .roll(roll)
        .pitch(pitch)
        .yaw(yaw)
        .build();
}
}

TEST(PoseControllerTest, ReturnsAntEnuTransformAndOriginalCommand)
{
    const auto command = makeCommand(-10_deg, 0_deg, -20_deg);
    const auto result = CONTROLLER.setRadarPose(command);

    EXPECT_EQ(result.pose.latitude, command.latitude);
    EXPECT_EQ(result.pose.longitude, command.longitude);
    EXPECT_EQ(result.pose.altitude, command.altitude);
    EXPECT_EQ(result.pose.roll, command.roll);
    EXPECT_EQ(result.pose.pitch, command.pitch);
    EXPECT_EQ(result.pose.yaw, command.yaw);

    const auto boresight = result.antEnuTransform.transform.antToEnu(
        math::SphericalDirection(0_deg, 0_deg)
    );
    EXPECT_TRUE(boresight.azimuth.isEquivalentTo(-20_deg));
}

TEST(PoseControllerTest, RejectsInvalidAttitudeAndConfiguredPitchLimit)
{
    EXPECT_THROW(CONTROLLER.setRadarPose(makeCommand(181_deg, 0_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW(CONTROLLER.setRadarPose(makeCommand(0_deg, 0_deg, 361_deg)), std::invalid_argument);
    EXPECT_THROW(CONTROLLER.setRadarPose(makeCommand(0_deg, 60_deg, 0_deg)), std::invalid_argument);
    EXPECT_THROW(CONTROLLER.setRadarPose(makeCommand(0_deg, -70_deg, 0_deg)), std::invalid_argument);
    EXPECT_NO_THROW(CONTROLLER.setRadarPose(makeCommand(0_deg, 50_deg, 0_deg)));
}
