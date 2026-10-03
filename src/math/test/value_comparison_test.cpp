#include "math/square_matrix.hpp"
#include "math/vector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <type_traits>

namespace
{

template<typename T>
T makeValue(double first, double last)
{
    if constexpr (std::is_same_v<T, math::SquareMatrix> || std::is_same_v<T, math::Matrix>)
    {
        return T{{first, 0.0}, {0.0, last}};
    }
    else
    {
        return T{first, last};
    }
}

template<typename T>
class EqualsTest : public testing::Test
{
};

using EqualsTypes = testing::Types<math::Matrix, math::SquareMatrix, math::Vector>;
TYPED_TEST_SUITE(EqualsTest, EqualsTypes);

template<typename T>
class AbsoluteEqualsTest : public testing::Test
{
};

using AbsoluteEqualsTypes = testing::Types<math::SquareMatrix, math::Vector>;
TYPED_TEST_SUITE(AbsoluteEqualsTest, AbsoluteEqualsTypes);

static_assert(std::is_same_v<decltype(&math::Matrix::equals),
    bool (math::Matrix::*)(const math::Matrix&, double) const noexcept>);

// 계산 오차를 허용오차로 처리하며 0 허용오차에서는 구분한다.
TYPED_TEST(EqualsTest, HandlesRoundingWithExplicitTolerances)
{
    const auto calculated = makeValue<TypeParam>(0.1 + 0.2, 1.0);
    const auto expected = makeValue<TypeParam>(0.3, 1.0);
    EXPECT_FALSE(calculated.equals(expected, 0.0));
    EXPECT_TRUE(calculated.equals(expected, 1e-14));
    if constexpr (!std::is_same_v<TypeParam, math::Matrix>)
    {
        EXPECT_FALSE(calculated.equals(expected, makeValue<TypeParam>(0.0, 0.0)));
        EXPECT_TRUE(calculated.equals(expected, makeValue<TypeParam>(1e-15, 0.0)));
    }
}

// 허용오차 비교는 추이성을 보장하지 않는다.
TYPED_TEST(AbsoluteEqualsTest, ToleranceComparisonIsNontransitive)
{
    const auto a = makeValue<TypeParam>(0.0, 0.0);
    const auto b = makeValue<TypeParam>(0.75, 0.0);
    const auto c = makeValue<TypeParam>(1.5, 0.0);
    const auto tolerance = makeValue<TypeParam>(1.0, 0.0);
    EXPECT_TRUE(a.equals(b, tolerance));
    EXPECT_TRUE(b.equals(c, tolerance));
    EXPECT_FALSE(a.equals(c, tolerance));
}

// 상대오차를 생략하면 공통 기본값을 사용하며 명시한 값으로 변경할 수 있다.
TYPED_TEST(EqualsTest, DefaultsToSharedRelativeTolerance)
{
    const auto value = makeValue<TypeParam>(4.0, 0.0);
    const auto nearby = makeValue<TypeParam>(4.0 + 2e-9, 0.0);
    const auto outside = makeValue<TypeParam>(4.0 + 8e-9, 0.0);
    EXPECT_TRUE(value.equals(nearby));
    EXPECT_TRUE(nearby.equals(value));
    EXPECT_FALSE(value.equals(outside));
    EXPECT_EQ(value.equals(nearby), value.equals(nearby, math::DEFAULT_RELATIVE_TOLERANCE));
    EXPECT_FALSE(value.equals(nearby, 0.0));
    EXPECT_TRUE(value.equals(outside, 1e-8));
    EXPECT_TRUE(TypeParam().equals(TypeParam()));
    EXPECT_FALSE(value.equals(TypeParam()));
    EXPECT_FALSE(makeValue<TypeParam>(0.0, 0.0).equals(makeValue<TypeParam>(1e-12, 0.0)));
}

// 절대오차는 원소마다 지정할 수 있고 경계를 포함한다.
TYPED_TEST(AbsoluteEqualsTest, AbsoluteToleranceIsElementwiseAndInclusive)
{
    const auto value = makeValue<TypeParam>(1.0, 1000.0);
    const auto nearby = makeValue<TypeParam>(1.25, 1001.0);
    const auto tolerance = makeValue<TypeParam>(0.25, 1.0);
    EXPECT_TRUE(value.equals(nearby, tolerance));
    EXPECT_TRUE(nearby.equals(value, tolerance));
    EXPECT_FALSE(value.equals(nearby, makeValue<TypeParam>(std::nextafter(0.25, 0.0), 1.0)));
    EXPECT_FALSE(value.equals(nearby, makeValue<TypeParam>(0.25, 0.0)));
    EXPECT_TRUE(value.equals(value, makeValue<TypeParam>(0.0, 0.0)));
    EXPECT_FALSE(value.equals(nearby, makeValue<TypeParam>(0.0, 0.0)));
    EXPECT_TRUE(makeValue<TypeParam>(0.0, 0.0).equals(
        makeValue<TypeParam>(1e-12, 0.0), makeValue<TypeParam>(1e-12, 0.0)));
}

// 상대오차는 큰 쪽 절댓값을 기준으로 모든 원소를 개별 비교한다.
TYPED_TEST(EqualsTest, RelativeToleranceIsElementwiseAndInclusive)
{
    const auto value = makeValue<TypeParam>(4.0, 0.0);
    const auto nearby = makeValue<TypeParam>(8.0, 0.0);
    EXPECT_TRUE(value.equals(nearby, 0.5));
    EXPECT_TRUE(nearby.equals(value, 0.5));
    EXPECT_FALSE(value.equals(nearby, std::nextafter(0.5, 0.0)));
    EXPECT_TRUE(value.equals(value, 0.0));
    EXPECT_FALSE(value.equals(nearby, 0.0));
    EXPECT_FALSE(makeValue<TypeParam>(1e20, 0.0).equals(makeValue<TypeParam>(1e20, 1.0), 1e-5));
    EXPECT_FALSE(makeValue<TypeParam>(0.0, 0.0).equals(makeValue<TypeParam>(1e-12, 0.0), 0.01));
}

// 두 비교 모두 차원을 검사하고 같은 형태의 빈 값은 같다고 판단한다.
TYPED_TEST(EqualsTest, ChecksDimensionsAndSupportsEmptyValues)
{
    const auto value = makeValue<TypeParam>(1.0, 2.0);
    EXPECT_FALSE(value.equals(TypeParam(), 0.1));
    EXPECT_TRUE(TypeParam().equals(TypeParam(), 0.0));
    if constexpr (!std::is_same_v<TypeParam, math::Matrix>)
    {
        const auto tolerance = makeValue<TypeParam>(0.1, 0.1);
        EXPECT_FALSE(value.equals(TypeParam(), tolerance));
        EXPECT_FALSE(value.equals(value, TypeParam()));
        EXPECT_TRUE(TypeParam().equals(TypeParam(), TypeParam()));
        EXPECT_FALSE(TypeParam().equals(TypeParam(), tolerance));
    }
}

// 동일한 값에서도 음수·무한대·NaN 허용오차는 거부한다.
TYPED_TEST(EqualsTest, RejectsInvalidTolerances)
{
    const auto value = makeValue<TypeParam>(1.0, 2.0);
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        if constexpr (!std::is_same_v<TypeParam, math::Matrix>)
        {
            EXPECT_FALSE(value.equals(value, makeValue<TypeParam>(invalid, 0.0)));
            EXPECT_FALSE(value.equals(value, makeValue<TypeParam>(0.0, invalid)));
        }
        EXPECT_FALSE(value.equals(value, invalid));
        EXPECT_FALSE(TypeParam().equals(TypeParam(), invalid));
    }
}

// 절대·상대오차 비교 모두 비유한 원소는 거부한다.
TYPED_TEST(EqualsTest, RejectsNonfiniteElements)
{
    const auto finite = makeValue<TypeParam>(1.0, 2.0);
    for (double invalid : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        const auto value = makeValue<TypeParam>(1.0, invalid);
        if constexpr (!std::is_same_v<TypeParam, math::Matrix>)
        {
            const auto tolerance = makeValue<TypeParam>(1.0, 1.0);
            EXPECT_FALSE(value.equals(value, tolerance));
            EXPECT_FALSE(value.equals(finite, tolerance));
            EXPECT_FALSE(finite.equals(value, tolerance));
        }
        EXPECT_FALSE(value.equals(value, 1.0));
        EXPECT_FALSE(value.equals(finite, 1.0));
        EXPECT_FALSE(finite.equals(value, 1.0));
    }
}

// 최댓값의 차 오버플로와 아주 작은 값도 올바르게 비교한다.
TYPED_TEST(EqualsTest, HandlesNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const double smallest = std::numeric_limits<double>::denorm_min();
    const auto positive = makeValue<TypeParam>(largest, 0.0);
    const auto negative = makeValue<TypeParam>(-largest, 0.0);
    EXPECT_FALSE(positive.equals(negative, 1.5));
    EXPECT_TRUE(positive.equals(negative, 2.0));
    const auto tiny = makeValue<TypeParam>(smallest, 0.0);
    const auto twiceTiny = makeValue<TypeParam>(2.0 * smallest, 0.0);
    if constexpr (!std::is_same_v<TypeParam, math::Matrix>)
    {
        EXPECT_FALSE(positive.equals(negative, makeValue<TypeParam>(largest, 0.0)));
        EXPECT_TRUE(tiny.equals(twiceTiny, makeValue<TypeParam>(smallest, 0.0)));
        EXPECT_FALSE(tiny.equals(twiceTiny, makeValue<TypeParam>(0.0, 0.0)));
    }
    EXPECT_TRUE(tiny.equals(twiceTiny, 0.5));
    EXPECT_FALSE(tiny.equals(twiceTiny, std::nextafter(0.5, 0.0)));
}

// 일반 행렬은 직사각형과 0차원 형태도 정확하게 검사한다.
TEST(MatrixEqualsTest, SupportsRectangularAndZeroDimensionShapes)
{
    const math::Matrix value{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const math::Matrix nearby{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.25}};
    EXPECT_TRUE(value.equals(nearby, 0.04));
    EXPECT_FALSE(value.equals(nearby, 0.02));
    EXPECT_FALSE(value.equals(value.transpose(), 1.0));
    EXPECT_TRUE(math::Matrix(0, 3).equals(math::Matrix(0, 3)));
    EXPECT_TRUE(math::Matrix(3, 0).equals(math::Matrix(3, 0), 0.0));
    EXPECT_FALSE(math::Matrix(0, 3).equals(math::Matrix(3, 0)));
    EXPECT_FALSE(math::Matrix(0, 3).equals(math::Matrix(0, 0), 0.0));
}

}
