#include <beam/config/attitude_config.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<beam::AttitudeConfig>);

const double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
const double INF_VALUE = std::numeric_limits<double>::infinity();
const double MAX_VALUE = std::numeric_limits<double>::max();

beam::AttitudeConfig::Builder attitudeBuilder()
{
    return beam::AttitudeConfig::Builder()
        .maxAbsPitch(89_deg)
        .mountAntToBodyRotation(math::SquareMatrix::identity(3))
        .mountOrthogonalTolerance(math::Tolerance{1e-6, 0.0});
}

} // namespace

// 필수 값이 누락된 빌더는 설정을 생성할 수 없다.
TEST(AttitudeConfigTest, BuilderRequiresEveryValue)
{
    // Given: 필수 값을 지정하지 않은 빌더를 준비한다.
    const beam::AttitudeConfig::Builder builder;

    // When: 빌더로 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 누락된 필수 값 때문에 예외가 발생한다.
    EXPECT_THROW(build(), std::logic_error);
}

// 자세 설정 빌더는 지정한 피치 한계와 장착행렬 및 허용오차를 반영한다.
TEST(AttitudeConfigTest, BuilderOverridesGivenValues)
{
    // Given: 90도 회전 장착행렬을 준비한다.
    const math::SquareMatrix mount{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};

    // When: 피치 한계와 장착행렬 및 허용오차를 지정해 설정을 생성한다.
    const beam::AttitudeConfig config = attitudeBuilder()
        .maxAbsPitch(80_deg)
        .mountAntToBodyRotation(mount)
        .mountOrthogonalTolerance(math::Tolerance{1e-3, 2e-3})
        .build();

    // Then: 자세 설정 빌더는 지정한 피치 한계와 장착행렬 및 허용오차를 반영한다.
    EXPECT_EQ(config.maxAbsPitch(), 80_deg);
    EXPECT_EQ(config.mountOrthogonalTolerance().absolute(), 1e-3);
    EXPECT_EQ(config.mountOrthogonalTolerance().relative(), 2e-3);
    EXPECT_EQ(config.mountAntToBodyRotation()(0, 1), -1.0);
    EXPECT_EQ(config.mountAntToBodyRotation()(1, 0), 1.0);
}

// 자세 설정은 3차원 회전 장착행렬을 허용한다.
TEST(AttitudeConfigTest, AcceptsRotationMount)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = attitudeBuilder()
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}});

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&] { return builder.build(); };

    // Then: 자세 설정은 3차원 회전 장착행렬을 허용한다.
    EXPECT_NO_THROW(build());
}

// 자세 설정은 부동소수점 성분을 가진 회전행렬을 허용한다.
TEST(AttitudeConfigTest, AcceptsRotationMountWithFloatingPointEntries)
{
    // Given: 라디안 각도로 계산한 회전행렬 성분을 준비한다.
    const math::Angle angle = 0.37_rad;
    const double c = std::cos(angle.rad());
    const double s = std::sin(angle.rad());

    // When: 부동소수점 성분의 장착행렬로 설정을 생성한다.
    const auto build = [&] {
        return attitudeBuilder()
            .mountAntToBodyRotation(math::SquareMatrix{{c, -s, 0}, {s, c, 0}, {0, 0, 1}})
            .build();
    };

    // Then: 자세 설정은 부동소수점 성분을 가진 회전행렬을 허용한다.
    EXPECT_NO_THROW(build());
}

// 자세 설정은 회전행렬이 아닌 장착행렬을 거부한다.
TEST(AttitudeConfigTest, NonRotationMountThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::SquareMatrix invalidMounts[] = {
        {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}, // 스케일
        {{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}, // 반사
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}, // 특이 행렬
        math::SquareMatrix::identity(2)};
    const auto builder = attitudeBuilder();

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&](const math::SquareMatrix& mount) {
        return beam::AttitudeConfig::Builder(builder).mountAntToBodyRotation(mount).build();
    };

    // Then: 자세 설정은 회전행렬이 아닌 장착행렬을 거부한다.
    for (const auto& mount : invalidMounts)
    {
        EXPECT_THROW(build(mount), std::invalid_argument);
    }
}

// 자세 설정은 비유한 성분이나 연산 오버플로가 있는 장착행렬을 거부한다.
TEST(AttitudeConfigTest, NonFiniteMountEntriesAndOverflowThrow)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::SquareMatrix invalidMounts[] = {
        {{NAN_VALUE, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {{INF_VALUE, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {{MAX_VALUE, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const math::SquareMatrix& mount) {
        return attitudeBuilder().mountAntToBodyRotation(mount).build();
    };

    // Then: 자세 설정은 비유한 성분이나 연산 오버플로가 있는 장착행렬을 거부한다.
    for (const auto& mount : invalidMounts)
    {
        EXPECT_THROW(build(mount), std::invalid_argument);
    }
}

// 자세 설정은 지정한 절대 허용오차로 장착행렬을 검사한다.
TEST(AttitudeConfigTest, MountOrthogonalityUsesConfiguredTolerance)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::SquareMatrix skewed{{1, 0.1, 0}, {0, 1, 0}, {0, 0, 1}};
    const math::Tolerance loose{0.2, 0.0};
    const math::Tolerance strict{0.05, 0.0};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&](const math::Tolerance& tolerance) {
        return attitudeBuilder().mountAntToBodyRotation(skewed)
            .mountOrthogonalTolerance(tolerance).build();
    };

    // Then: 자세 설정은 지정한 절대 허용오차로 장착행렬을 검사한다.
    EXPECT_NO_THROW(build(loose));
    EXPECT_THROW(build(strict), std::invalid_argument);
}

// 자세 설정은 지정한 상대 허용오차로 장착행렬을 검사한다.
TEST(AttitudeConfigTest, MountRotationUsesRelativeTolerance)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::SquareMatrix slightlyScaled{{1.0005, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const math::Tolerance loose{0.0, 0.002};
    const math::Tolerance strict{0.0, 0.0};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&](const math::Tolerance& tolerance) {
        return attitudeBuilder().mountAntToBodyRotation(slightlyScaled)
            .mountOrthogonalTolerance(tolerance).build();
    };

    // Then: 자세 설정은 지정한 상대 허용오차로 장착행렬을 검사한다.
    EXPECT_NO_THROW(build(loose));
    EXPECT_THROW(build(strict), std::invalid_argument);
}

// 직교성 허용오차가 0인 설정도 회전행렬을 허용한다.
TEST(AttitudeConfigTest, AcceptsZeroMountOrthogonalTolerance)
{
    // Given: 정확한 단위행렬과 오차를 허용하지 않는 설정을 준비한다.
    const auto builder = attitudeBuilder()
        .mountOrthogonalTolerance(math::Tolerance{0.0, 0.0});

    // When: 해당 허용오차로 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 정확한 회전행렬이므로 설정을 생성할 수 있다.
    EXPECT_NO_THROW(build());
}

// 최대 피치 설정은 0도 초과 90도 이하만 허용한다.
TEST(AttitudeConfigTest, MaxAbsPitchMustBeInZeroToNinetyDegrees)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = attitudeBuilder();
    const math::Angle validPitches[] = {90_deg, 0.1_deg};
    const math::Angle invalidPitches[] = {0_deg, -1_deg, 91_deg};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&](math::Angle pitch) {
        return beam::AttitudeConfig::Builder(builder).maxAbsPitch(pitch).build();
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

// 최대 피치는 주기상 90도와 같아도 실제 값이 범위 밖이면 거부한다.
TEST(AttitudeConfigTest, MaxAbsPitchRejectsAnglesEquivalentToNinetyDegreesByPeriod)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::Angle pitches[] = {450_deg, 810_deg, -270_deg};
    const auto mount = math::SquareMatrix::identity(3);
    const math::Tolerance tolerance{1e-6, 0.0};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](math::Angle pitch) { return attitudeBuilder().maxAbsPitch(pitch).build(); };
    const auto construct = [&] { return beam::AttitudeConfig(450_deg, mount, tolerance); };

    // Then: 최대 피치는 주기상 90도와 같아도 실제 값이 범위 밖이면 거부한다.
    for (const auto pitch : pitches)
    {
        EXPECT_THROW(build(pitch), std::invalid_argument);
    }
    EXPECT_THROW(construct(), std::invalid_argument);
}

// 자세 설정 생성자는 3차원 회전 장착행렬만 허용한다.
TEST(AttitudeConfigTest, ConstructorValidatesLikeBuilder)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto validMount = math::SquareMatrix::identity(3);
    const auto invalidMount = math::SquareMatrix::identity(2);
    const math::Tolerance tolerance{1e-6, 0.0};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto construct = [&](const math::SquareMatrix& mount) {
        return beam::AttitudeConfig(89_deg, mount, tolerance);
    };

    // Then: 자세 설정 생성자는 3차원 회전 장착행렬만 허용한다.
    EXPECT_NO_THROW(construct(validMount));
    EXPECT_THROW(construct(invalidMount), std::invalid_argument);
}
