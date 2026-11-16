#include <pose/config/pose_config.hpp>
#include <math/trigonometry.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<pose::PoseConfig>);

const double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
const double INF_VALUE = std::numeric_limits<double>::infinity();
const double MAX_VALUE = std::numeric_limits<double>::max();

} // namespace

// 자세 설정은 3차원 회전 장착행렬을 허용한다.
TEST(AntEnuTransformConfigTest, AcceptsRotationMount)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto mount = math::SquareMatrix({{0, -1, 0}, {1, 0, 0}, {0, 0, 1}});

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&] {
        return pose::AntEnuTransformConfig::Builder().mountAntToBodyRotation(mount).build();
    };

    // Then: 자세 설정은 3차원 회전 장착행렬을 허용한다.
    EXPECT_NO_THROW(build());
}

TEST(AntEnuTransformConfigTest, BuilderRequiresMountRotation)
{
    EXPECT_THROW(pose::AntEnuTransformConfig::Builder().build(), std::logic_error);
}

// 자세 설정은 부동소수점 성분을 가진 회전행렬을 허용한다.
TEST(AntEnuTransformConfigTest, AcceptsRotationMountWithFloatingPointEntries)
{
    // Given: 라디안 각도로 계산한 회전행렬 성분을 준비한다.
    const auto angle = 0.37_rad;
    const auto c = math::cos(angle);
    const auto s = math::sin(angle);

    // When: 부동소수점 성분의 장착행렬로 설정을 생성한다.
    const auto build = [&] {
        return pose::AntEnuTransformConfig(math::SquareMatrix({{c, -s, 0}, {s, c, 0}, {0, 0, 1}}));
    };

    // Then: 자세 설정은 부동소수점 성분을 가진 회전행렬을 허용한다.
    EXPECT_NO_THROW(build());
}

TEST(AntEnuTransformConfigTest, UsesDefaultRotationTolerance)
{
    const auto nearlyOrthogonal = math::SquareMatrix({{1, 1e-13, 0}, {0, 1, 0}, {0, 0, 1}});
    const auto skewed = math::SquareMatrix({{1, 1e-8, 0}, {0, 1, 0}, {0, 0, 1}});

    EXPECT_NO_THROW((pose::AntEnuTransformConfig(nearlyOrthogonal)));
    EXPECT_THROW((pose::AntEnuTransformConfig(skewed)), std::invalid_argument);
}

// 자세 설정은 회전행렬이 아닌 장착행렬을 거부한다.
TEST(AntEnuTransformConfigTest, NonRotationMountThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::SquareMatrix invalidMounts[] = {
        {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}, // 스케일
        {{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}, // 반사
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}, // 특이 행렬
        math::SquareMatrix::identity(2)
    };

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&](const math::SquareMatrix& mount) {
        return pose::AntEnuTransformConfig(mount);
    };

    // Then: 자세 설정은 회전행렬이 아닌 장착행렬을 거부한다.
    for (const auto& mount : invalidMounts)
    {
        EXPECT_THROW(build(mount), std::invalid_argument);
    }
}

// 자세 설정은 비유한 성분이나 연산 오버플로가 있는 장착행렬을 거부한다.
TEST(AntEnuTransformConfigTest, NonFiniteMountEntriesAndOverflowThrow)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::SquareMatrix invalidMounts[] = {
        {{NAN_VALUE, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {{INF_VALUE, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {{MAX_VALUE, 0, 0}, {0, 1, 0}, {0, 0, 1}}
    };

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const math::SquareMatrix& mount) {
        return pose::AntEnuTransformConfig(mount);
    };

    // Then: 자세 설정은 비유한 성분이나 연산 오버플로가 있는 장착행렬을 거부한다.
    for (const auto& mount : invalidMounts)
    {
        EXPECT_THROW(build(mount), std::invalid_argument);
    }
}

// 최대 피치 설정은 0도 초과 90도 이하만 허용한다.
TEST(RadarAttitudeFactoryConfigTest, MaxAbsPitchMustBeInZeroToNinetyDegrees)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::Angle validPitches[] = {90_deg, 0.1_deg};
    const math::Angle invalidPitches[] = {0_deg, -1_deg, 91_deg};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&](math::Angle pitch) {
        return pose::RadarAttitudeFactoryConfig::Builder().maxAbsPitch(pitch).build();
    };

    // Then: 최대 피치 설정은 0도 초과 90도 이하만 허용한다.
    for (const auto pitch : validPitches)
    {
        EXPECT_NO_THROW(build(pitch));
    }
    for (const auto pitch : invalidPitches)
    {
        EXPECT_THROW(build(pitch), std::invalid_argument);
    }
}

TEST(RadarAttitudeFactoryConfigTest, BuilderRequiresMaxAbsPitch)
{
    EXPECT_THROW(pose::RadarAttitudeFactoryConfig::Builder().build(), std::logic_error);
}

// 최대 피치는 주기상 90도와 같아도 실제 값이 범위 밖이면 거부한다.
TEST(RadarAttitudeFactoryConfigTest, MaxAbsPitchRejectsAnglesEquivalentToNinetyDegreesByPeriod)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::Angle pitches[] = {450_deg, 810_deg, -270_deg};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](math::Angle pitch) { return pose::RadarAttitudeFactoryConfig(pitch); };

    // Then: 최대 피치는 주기상 90도와 같아도 실제 값이 범위 밖이면 거부한다.
    for (const auto pitch : pitches)
    {
        EXPECT_THROW(build(pitch), std::invalid_argument);
    }
}
