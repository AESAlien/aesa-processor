#include "math/linear_algebra/square_matrix.hpp"
#include "math/linear_algebra/vector.hpp"
#include "math/quantity/angle.hpp"
#include "math/quantity/length.hpp"
#include "math/quantity/linear_acceleration.hpp"
#include "math/quantity/linear_velocity.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

template <typename T>
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

template <typename T>
class EqualsTest : public testing::Test
{
};

using EqualsTypes = testing::Types<math::Matrix, math::SquareMatrix, math::Vector>;
TYPED_TEST_SUITE(EqualsTest, EqualsTypes);

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, HandlesRoundingAndDefaultTolerances)
{
    const auto calculated = makeValue<TypeParam>(0.1 + 0.2, 1.0);
    const auto expected = makeValue<TypeParam>(0.3, 1.0);
    EXPECT_TRUE(calculated.equals(expected));
    EXPECT_TRUE(calculated == expected);
    EXPECT_FALSE(calculated != expected);
    EXPECT_FALSE(calculated.equals(expected, math::NONE_TOLERANCE));
    EXPECT_TRUE(calculated.equals(expected, math::DEFAULT_TOLERANCE));
    EXPECT_FALSE(expected == makeValue<TypeParam>(0.4, 1.0));
    EXPECT_FALSE(expected.equals(makeValue<TypeParam>(0.4, 1.0)));
    EXPECT_TRUE(expected != makeValue<TypeParam>(0.4, 1.0));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, AbsoluteToleranceIncludesBoundary)
{
    const auto zero = makeValue<TypeParam>(0.0, 0.0);
    const auto boundary = makeValue<TypeParam>(1e-12, 0.0);
    const auto outside = makeValue<TypeParam>(std::nextafter(1e-12, std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_TRUE(zero == boundary);
    EXPECT_TRUE(boundary == zero);
    EXPECT_FALSE(zero == outside);
    EXPECT_FALSE(boundary.equals(zero, math::Tolerance{0.0, 1e-9}));
    const math::Tolerance absolute{0.25, 0.0};
    EXPECT_TRUE(zero.equals(makeValue<TypeParam>(0.25, 0.25), absolute));
    EXPECT_FALSE(zero.equals(makeValue<TypeParam>(0.25, std::nextafter(0.25, 1.0)), absolute));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, RelativeToleranceIsSymmetricAndInclusive)
{
    const auto value = makeValue<TypeParam>(4.0, 0.0);
    const auto nearby = makeValue<TypeParam>(8.0, 0.0);
    const math::Tolerance relative{0.0, 0.5};
    EXPECT_TRUE(value.equals(nearby, relative));
    EXPECT_TRUE(nearby.equals(value, relative));
    EXPECT_FALSE(value.equals(nearby, math::Tolerance{0.0, std::nextafter(0.5, 0.0)}));
    EXPECT_FALSE(makeValue<TypeParam>(1e20, 0.0).equals(makeValue<TypeParam>(1e20, 1.0), math::Tolerance{0.0, 1e-5}));
    EXPECT_FALSE(makeValue<TypeParam>(0.0, 0.0).equals(makeValue<TypeParam>(1e-12, 0.0), math::Tolerance{0.0, 0.01}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, CombinesAbsoluteAndRelativeTolerancePerElement)
{
    const auto value = makeValue<TypeParam>(0.0, 1e6);
    const auto nearby = makeValue<TypeParam>(0.5e-12, 1e6 + 0.0005);
    EXPECT_FALSE(value.equals(nearby, math::Tolerance{1e-12, 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::Tolerance{0.0, 1e-9}));
    EXPECT_TRUE(value == nearby);
    EXPECT_TRUE(nearby == value);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, ChecksDimensionsAndSupportsEmptyValues)
{
    const auto value = makeValue<TypeParam>(1.0, 2.0);
    EXPECT_FALSE(value.equals(TypeParam(), math::DEFAULT_TOLERANCE));
    EXPECT_FALSE(value == TypeParam());
    EXPECT_TRUE(value != TypeParam());
    EXPECT_TRUE(TypeParam() == TypeParam());
    EXPECT_TRUE(TypeParam().equals(TypeParam(), math::Tolerance{0.0, 0.0}));
}

TYPED_TEST(EqualsTest, RejectsNonfiniteElements)
{
    const auto finite = makeValue<TypeParam>(1.0, 2.0);
    for (double invalid : {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        const auto value = makeValue<TypeParam>(1.0, invalid);
        EXPECT_FALSE(value.equals(value));
        EXPECT_FALSE(value == value);
        EXPECT_TRUE(value != value);
        EXPECT_FALSE(value.equals(finite, math::DEFAULT_TOLERANCE));
        EXPECT_FALSE(finite.equals(value, math::DEFAULT_TOLERANCE));
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TYPED_TEST(EqualsTest, HandlesNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const double smallest = std::numeric_limits<double>::denorm_min();
    const auto positive = makeValue<TypeParam>(largest, 0.0);
    const auto negative = makeValue<TypeParam>(-largest, 0.0);
    EXPECT_FALSE(positive.equals(negative, math::Tolerance{0.0, 1.5}));
    EXPECT_TRUE(positive.equals(negative, math::Tolerance{0.0, 2.0}));
    const auto tiny = makeValue<TypeParam>(smallest, 0.0);
    const auto twiceTiny = makeValue<TypeParam>(2.0 * smallest, 0.0);
    EXPECT_TRUE(tiny.equals(twiceTiny, math::Tolerance{smallest, 0.0}));
    EXPECT_FALSE(tiny.equals(twiceTiny, math::Tolerance{0.0, 0.0}));
    EXPECT_TRUE(tiny.equals(twiceTiny, math::Tolerance{0.0, 0.5}));
    EXPECT_FALSE(tiny.equals(twiceTiny, math::Tolerance{0.0, std::nextafter(0.5, 0.0)}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(ToleranceTest, ComparesFiniteNumbersUsingItsPolicy)
{
    const math::Tolerance combined{1e-12, 1e-9};
    EXPECT_TRUE(combined.isWithin(0.1 + 0.2, 0.3));
    EXPECT_TRUE(combined.isWithin(0.0, 1e-12));
    EXPECT_FALSE(combined.isWithin(0.0, std::nextafter(1e-12, 1.0)));
    EXPECT_TRUE(combined.isWithin(1e6, 1e6 + 0.0005));
    EXPECT_FALSE(combined.isWithin(1e6, 1e6 + 0.002));
    const double largest = std::numeric_limits<double>::max();
    EXPECT_FALSE((math::Tolerance{0.0, 1.5}).isWithin(largest, -largest));
    EXPECT_TRUE((math::Tolerance{0.0, 2.0}).isWithin(largest, -largest));
    for (double nonfinite : {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_FALSE(combined.isWithin(nonfinite, nonfinite));
        EXPECT_FALSE(combined.isWithin(0.0, nonfinite));
        EXPECT_FALSE(combined.isWithin(nonfinite, 0.0));
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(ToleranceTest, NoneToleranceRequiresExactFiniteEquality)
{
    EXPECT_DOUBLE_EQ(math::NONE_TOLERANCE.absolute(), 0.0);
    EXPECT_DOUBLE_EQ(math::NONE_TOLERANCE.relative(), 0.0);
    EXPECT_TRUE(math::NONE_TOLERANCE.isWithin(1.0, 1.0));
    EXPECT_TRUE(math::NONE_TOLERANCE.isWithin(0.0, -0.0));
    EXPECT_FALSE(math::NONE_TOLERANCE.isWithin(0.1 + 0.2, 0.3));
    EXPECT_FALSE(math::NONE_TOLERANCE.isWithin(1.0, std::nextafter(1.0, 2.0)));
    EXPECT_FALSE(math::NONE_TOLERANCE.isWithin(0.0, std::numeric_limits<double>::denorm_min()));
    for (double nonfinite : {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_FALSE(math::NONE_TOLERANCE.isWithin(nonfinite, nonfinite));
        EXPECT_FALSE(math::NONE_TOLERANCE.isWithin(0.0, nonfinite));
        EXPECT_FALSE(math::NONE_TOLERANCE.isWithin(nonfinite, 0.0));
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(ToleranceTest, ValidatesScalarPolicyAtConstruction)
{
    EXPECT_NO_THROW((math::Tolerance{-0.0, 0.0}));
    EXPECT_NO_THROW((math::Tolerance{std::numeric_limits<double>::max(), std::numeric_limits<double>::max()}));
    for (double invalid : {-0.5e-12, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((math::Tolerance{invalid, 0.0}), std::invalid_argument);
        EXPECT_THROW((math::Tolerance{0.0, invalid}), std::invalid_argument);
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(ToleranceTest, PreservesUnitsAndSupportsConstantPolicies)
{
    const math::Tolerance exact{0.0, 0.0};
    EXPECT_DOUBLE_EQ(exact.absolute(), 0.0);
    EXPECT_DOUBLE_EQ(exact.relative(), 0.0);
    static_assert(!std::is_constructible_v<math::QuantityTolerance<math::Angle>, math::Length, double>);
    static_assert(std::is_assignable_v<math::Tolerance&, math::Tolerance>);
    EXPECT_FALSE((math::Matrix{{0.1 + 0.2}}).equals(math::Matrix{{0.3}}, exact));
    EXPECT_TRUE((math::Matrix{{0.1 + 0.2}}).equals(math::Matrix{{0.3}}, math::DEFAULT_TOLERANCE));
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(ToleranceTest, OwnsAbsoluteValueAndPreservesItAfterCopyAndMove)
{
    auto absolute = 0.25_deg;
    const math::QuantityTolerance<math::Angle> original{absolute, 0.0};
    absolute = 0.0_deg;
    auto source = original;
    const auto moved = std::move(source);
    const auto zero = 0.0_deg;
    const auto boundary = 0.25_deg;
    EXPECT_TRUE(zero.equals(boundary, original));
    EXPECT_TRUE(zero.equals(boundary, source));
    EXPECT_TRUE(zero.equals(boundary, moved));
    EXPECT_FALSE(zero.equals(boundary, math::QuantityTolerance{absolute, 0.0}));
}

template <typename T>
T makeQuantity(double value)
{
    if constexpr (std::is_same_v<T, math::Angle>)
    {
        return math::Angle::fromDegrees(value);
    }
    else if constexpr (std::is_same_v<T, math::Length>)
    {
        return math::Length::fromMeters(value);
    }
    else if constexpr (std::is_same_v<T, math::LinearAcceleration>)
    {
        return math::LinearAcceleration::fromMetersPerSecondSquared(value);
    }
    else
    {
        return math::LinearVelocity::fromMetersPerSecond(value);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(QuantityTest, SharesStorageAndChecksWithoutLosingUnitSafety)
{
    static_assert(std::is_base_of_v<math::Quantity<math::Angle>, math::Angle>);
    static_assert(std::is_base_of_v<math::Quantity<math::Length>, math::Length>);
    static_assert(std::is_base_of_v<math::Quantity<math::LinearVelocity>, math::LinearVelocity>);
    static_assert(std::is_base_of_v<math::Quantity<math::LinearAcceleration>, math::LinearAcceleration>);
    static_assert(sizeof(math::Angle) == sizeof(double));
    static_assert(sizeof(math::Length) == sizeof(double));
    static_assert(sizeof(math::LinearVelocity) == sizeof(double));
    static_assert(sizeof(math::LinearAcceleration) == sizeof(double));
    static_assert(!std::is_assignable_v<math::Quantity<math::Length>&, math::Length>);
    static_assert(!std::is_constructible_v<math::Angle, math::Length>);

    const auto angle = -90.0_deg;
    const math::Quantity<math::Angle>& quantity = angle;
    EXPECT_DOUBLE_EQ(quantity.value(), angle.deg());
    EXPECT_FALSE(quantity.isNonnegative());
    EXPECT_DOUBLE_EQ((2.0_km).value(), 2000.0);
    EXPECT_DOUBLE_EQ((3.0_mps).value(), 3.0);
    EXPECT_DOUBLE_EQ((4.0_mps2).value(), 4.0);
}

// 새 물리량도 단위별 분기 없이 공통 정책을 사용할 수 있다.
class TestQuantity : public math::Quantity<TestQuantity>
{
public:
    static const math::QuantityTolerance<TestQuantity> DEFAULT_TOLERANCE;

    explicit TestQuantity(double value)
      : Quantity<TestQuantity>(value)
    {
    }
};

const math::QuantityTolerance<TestQuantity> TestQuantity::DEFAULT_TOLERANCE{
    TestQuantity(0.5), 0.0
};

// 계산 결과와 수치적 안정성을 검증한다.
TEST(QuantityTest, NewQuantityInheritsFiniteStorageValidation)
{
    const double invalidValues[] =
    {
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()
    };
    for (double value : invalidValues)
    {
        EXPECT_THROW(TestQuantity{value}, std::invalid_argument);
    }
    const double maxValue = std::numeric_limits<double>::max();
    EXPECT_DOUBLE_EQ(TestQuantity(maxValue).value(), maxValue);
    EXPECT_DOUBLE_EQ(TestQuantity(-maxValue).value(), -maxValue);
    EXPECT_TRUE(TestQuantity(-0.0).isNonnegative());
    EXPECT_FALSE(TestQuantity(-std::numeric_limits<double>::denorm_min()).isNonnegative());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(QuantityTest, NewQuantityInheritsComparisonsAndUsesItsOwnDefaultTolerance)
{
    const TestQuantity low(1.0);
    const TestQuantity nearby(1.5);
    const TestQuantity high(2.0);
    const math::QuantityTolerance exact{TestQuantity(0.0), 0.0};
    EXPECT_TRUE(low.equals(nearby));
    EXPECT_FALSE(low.equals(nearby, exact));
    EXPECT_FALSE(low.notEquals(nearby));
    EXPECT_TRUE(low.notEquals(nearby, exact));
    EXPECT_FALSE(low.lessThan(nearby));
    EXPECT_TRUE(low.lessThan(nearby, exact));
    EXPECT_TRUE(nearby.greaterThan(low, exact));
    EXPECT_TRUE(low.lessThanOrEqual(nearby));
    EXPECT_TRUE(low.greaterThanOrEqual(nearby));
    EXPECT_FALSE(low.greaterThanOrEqual(nearby, exact));
    EXPECT_FALSE(nearby.lessThanOrEqual(low, exact));
    EXPECT_TRUE(low == nearby);
    EXPECT_FALSE(low != nearby);
    EXPECT_FALSE(low < nearby);
    EXPECT_FALSE(nearby > low);
    EXPECT_TRUE(low <= nearby);
    EXPECT_TRUE(low >= nearby);
    EXPECT_TRUE(low != high);
    EXPECT_TRUE(low < high);
    EXPECT_TRUE(high > low);
}

template <typename Left, typename Right>
void checkComparisonTypes()
{
    constexpr bool SAME_TYPE = std::is_same_v<Left, Right>;
    using Policy = math::QuantityTolerance<Left>;
    static_assert(std::is_invocable_v<decltype(&Left::equals), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::notEquals), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::notEquals), const Left&, const Left&, const math::QuantityTolerance<Right>&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::lessThan), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::lessThanOrEqual), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::greaterThan), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::greaterThanOrEqual), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::operator==), const Left&, const Right&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::operator!=), const Left&, const Right&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::operator<), const Left&, const Right&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::operator<=), const Left&, const Right&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::operator>), const Left&, const Right&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::operator>=), const Left&, const Right&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::equals), const Left&, const Left&, const math::QuantityTolerance<Right>&> == SAME_TYPE);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(QuantityToleranceTest, SupportsNewQuantitiesAndRestrictsComparisonTypes)
{
    using AnglePolicy = math::QuantityTolerance<math::Angle>;
    static_assert(!std::is_invocable_v<decltype(&AnglePolicy::isWithin), const AnglePolicy&, math::Length, math::Length>);
    static_assert(!std::is_assignable_v<AnglePolicy&, AnglePolicy>);
    const math::QuantityTolerance policy{TestQuantity(0.25), 0.125};
    EXPECT_TRUE(policy.isWithin(TestQuantity(0.0), TestQuantity(0.25)));
    EXPECT_FALSE(policy.isWithin(TestQuantity(0.0), TestQuantity(0.5)));
    EXPECT_TRUE(policy.isWithin(TestQuantity(7.0), TestQuantity(8.0)));
    EXPECT_FALSE(policy.isWithin(TestQuantity(6.0), TestQuantity(8.0)));
    EXPECT_THROW((math::QuantityTolerance{TestQuantity(-1e-20), 0.0}), std::invalid_argument);
}

template <typename T>
class QuantityComparisonTest : public testing::Test
{
};

using QuantityTypes = testing::Types<
    math::Angle,
    math::Length,
    math::LinearVelocity,
    math::LinearAcceleration>;
TYPED_TEST_SUITE(QuantityComparisonTest, QuantityTypes);

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NoneToleranceDisablesAbsoluteAndRelativeTolerance)
{
    const auto& none = TypeParam::NONE_TOLERANCE;
    static_assert(std::is_same_v<decltype(none), const math::QuantityTolerance<TypeParam>&>);
    EXPECT_DOUBLE_EQ(none.absolute().value(), 0.0);
    EXPECT_DOUBLE_EQ(none.relative(), 0.0);
    const auto zero = makeQuantity<TypeParam>(0.0);
    EXPECT_TRUE(zero.equals(zero, none));
    EXPECT_FALSE(zero.notEquals(zero, none));
    for (double value : {0.0, 1e6})
    {
        const auto low = makeQuantity<TypeParam>(value);
        const auto high = makeQuantity<TypeParam>(std::nextafter(value, std::numeric_limits<double>::infinity()));
        EXPECT_TRUE(low.equals(high));
        EXPECT_FALSE(low.equals(high, none));
        EXPECT_TRUE(low.notEquals(high, none));
        EXPECT_TRUE(low.lessThan(high, none));
        EXPECT_TRUE(high.greaterThan(low, none));
        EXPECT_FALSE(low.greaterThanOrEqual(high, none));
        EXPECT_FALSE(high.lessThanOrEqual(low, none));
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, RestrictsComparisonsToMatchingQuantityTypes)
{
    checkComparisonTypes<TypeParam, math::Angle>();
    checkComparisonTypes<TypeParam, math::Length>();
    checkComparisonTypes<TypeParam, math::LinearVelocity>();
    checkComparisonTypes<TypeParam, math::LinearAcceleration>();
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NamedInequalitySupportsExplicitTolerances)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto boundary = makeQuantity<TypeParam>(0.25);
    const auto outside = makeQuantity<TypeParam>(std::nextafter(0.25, 1.0));
    const math::QuantityTolerance absolute{boundary, 0.0};
    const math::QuantityTolerance exact{zero, 0.0};
    EXPECT_FALSE(zero.notEquals(zero, exact));
    EXPECT_FALSE(zero.notEquals(boundary, absolute));
    EXPECT_FALSE(boundary.notEquals(zero, absolute));
    EXPECT_TRUE(zero.notEquals(outside, absolute));
    EXPECT_TRUE(zero.notEquals(boundary, exact));
    const auto low = makeQuantity<TypeParam>(4.0);
    const auto high = makeQuantity<TypeParam>(8.0);
    const math::QuantityTolerance relative{zero, 0.5};
    const math::QuantityTolerance belowBoundary{zero, std::nextafter(0.5, 0.0)};
    EXPECT_FALSE(low.notEquals(high, relative));
    EXPECT_FALSE(high.notEquals(low, relative));
    EXPECT_TRUE(low.notEquals(high, belowBoundary));
    const auto nearby = makeQuantity<TypeParam>(0.5e-12);
    EXPECT_FALSE(zero.notEquals(nearby));
    EXPECT_TRUE(zero.notEquals(nearby, exact));
    EXPECT_TRUE(zero.notEquals(high));
    EXPECT_EQ(zero.notEquals(nearby), zero != nearby);
    EXPECT_EQ(zero.notEquals(high), zero != high);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NamedOrderingUsesExplicitToleranceBoundaries)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const math::QuantityTolerance absolute{makeQuantity<TypeParam>(0.25), 0.0};
    const math::QuantityTolerance relative{zero, 0.5};
    const math::QuantityTolerance exact{zero, 0.0};
    for (double sign : {-1.0, 1.0})
    {
        const auto boundary = makeQuantity<TypeParam>(sign * 0.25);
        EXPECT_FALSE(zero.lessThan(boundary, absolute));
        EXPECT_FALSE(zero.greaterThan(boundary, absolute));
        EXPECT_TRUE(zero.lessThanOrEqual(boundary, absolute));
        EXPECT_TRUE(zero.greaterThanOrEqual(boundary, absolute));
        const auto outside = makeQuantity<TypeParam>(sign * std::nextafter(0.25, 1.0));
        EXPECT_EQ(zero.lessThan(outside, absolute), sign > 0.0);
        EXPECT_EQ(zero.greaterThan(outside, absolute), sign < 0.0);
        EXPECT_EQ(zero.lessThanOrEqual(outside, absolute), sign > 0.0);
        EXPECT_EQ(zero.greaterThanOrEqual(outside, absolute), sign < 0.0);
    }
    const auto low = makeQuantity<TypeParam>(4.0);
    const auto high = makeQuantity<TypeParam>(8.0);
    EXPECT_FALSE(low.lessThan(high, relative));
    EXPECT_FALSE(high.greaterThan(low, relative));
    EXPECT_TRUE(low.greaterThanOrEqual(high, relative));
    EXPECT_TRUE(high.lessThanOrEqual(low, relative));
    const math::QuantityTolerance belowBoundary{zero, std::nextafter(0.5, 0.0)};
    EXPECT_TRUE(low.lessThan(high, belowBoundary));
    EXPECT_TRUE(high.greaterThan(low, belowBoundary));
    EXPECT_FALSE(low.greaterThanOrEqual(high, belowBoundary));
    EXPECT_FALSE(high.lessThanOrEqual(low, belowBoundary));
    const auto nearby = makeQuantity<TypeParam>(0.5e-12);
    EXPECT_TRUE(zero.lessThan(nearby, exact));
    EXPECT_TRUE(nearby.greaterThan(zero, exact));
    EXPECT_FALSE(zero.greaterThanOrEqual(nearby, exact));
    EXPECT_FALSE(nearby.lessThanOrEqual(zero, exact));
    EXPECT_FALSE(zero.lessThan(zero, exact));
    EXPECT_FALSE(zero.greaterThan(zero, exact));
    EXPECT_TRUE(zero.lessThanOrEqual(zero, exact));
    EXPECT_TRUE(zero.greaterThanOrEqual(zero, exact));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NamedOrderingMatchesDefaultOperators)
{
    const double values[] =
    {
        -1.0,
        0.0,
        0.5e-12,
        1.0,
    };
    const math::QuantityTolerance exact{makeQuantity<TypeParam>(0.0), 0.0};
    for (double left : values)
    {
        for (double right : values)
        {
            const auto leftQuantity = makeQuantity<TypeParam>(left);
            const auto rightQuantity = makeQuantity<TypeParam>(right);
            EXPECT_EQ(leftQuantity.lessThan(rightQuantity), leftQuantity < rightQuantity);
            EXPECT_EQ(leftQuantity.lessThanOrEqual(rightQuantity), leftQuantity <= rightQuantity);
            EXPECT_EQ(leftQuantity.greaterThan(rightQuantity), leftQuantity > rightQuantity);
            EXPECT_EQ(leftQuantity.greaterThanOrEqual(rightQuantity), leftQuantity >= rightQuantity);
            EXPECT_EQ(leftQuantity.lessThan(rightQuantity, exact), left < right);
            EXPECT_EQ(leftQuantity.lessThanOrEqual(rightQuantity, exact), left <= right);
            EXPECT_EQ(leftQuantity.greaterThan(rightQuantity, exact), left > right);
            EXPECT_EQ(leftQuantity.greaterThanOrEqual(rightQuantity, exact), left >= right);
        }
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, RoundingUsesSamePolicyForEqualityAndOrdering)
{
    const auto calculated = makeQuantity<TypeParam>(0.1 + 0.2);
    const auto expected = makeQuantity<TypeParam>(0.3);
    EXPECT_TRUE(calculated.equals(expected));
    EXPECT_TRUE(expected.equals(calculated));
    EXPECT_FALSE(calculated.equals(makeQuantity<TypeParam>(0.4)));
    EXPECT_TRUE(calculated == expected);
    EXPECT_TRUE(expected == calculated);
    EXPECT_TRUE(calculated <= expected);
    EXPECT_TRUE(calculated >= expected);
    EXPECT_TRUE(expected <= calculated);
    EXPECT_TRUE(expected >= calculated);
    EXPECT_FALSE(calculated < expected);
    EXPECT_FALSE(calculated > expected);
    EXPECT_FALSE(expected < calculated);
    EXPECT_FALSE(expected > calculated);
    EXPECT_FALSE(calculated.equals(expected, math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 0.0}));
    EXPECT_FALSE(calculated.equals(expected, math::QuantityTolerance{makeQuantity<TypeParam>(0.0), 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, InequalityComplementsDefaultEquality)
{
    const auto calculated = makeQuantity<TypeParam>(0.1 + 0.2);
    const auto expected = makeQuantity<TypeParam>(0.3);
    const auto outside = makeQuantity<TypeParam>(0.4);
    EXPECT_FALSE(calculated != expected);
    EXPECT_FALSE(expected != calculated);
    EXPECT_TRUE(calculated != outside);
    EXPECT_TRUE(outside != calculated);
    EXPECT_TRUE(makeQuantity<TypeParam>(0.0) == makeQuantity<TypeParam>(-0.0));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, SupportsCombinedAbsoluteAndRelativeTolerance)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto boundary = makeQuantity<TypeParam>(0.25);
    const auto tolerance = makeQuantity<TypeParam>(0.25);
    EXPECT_TRUE(zero.equals(boundary, math::QuantityTolerance{tolerance, 0.0}));
    EXPECT_FALSE(zero.equals(makeQuantity<TypeParam>(std::nextafter(0.25, 1.0)), math::QuantityTolerance{tolerance, 0.0}));
    const auto large = makeQuantity<TypeParam>(1e6);
    const auto nearby = makeQuantity<TypeParam>(1e6 + 0.0005);
    EXPECT_TRUE(large.equals(nearby, math::QuantityTolerance{zero, 1e-9}));
    EXPECT_TRUE(nearby.equals(large, math::QuantityTolerance{zero, 1e-9}));
    EXPECT_FALSE(large.equals(nearby, math::QuantityTolerance{zero, 0.0}));
    EXPECT_EQ(large == nearby, large.equals(nearby, math::QuantityTolerance{makeQuantity<TypeParam>(1e-12), 1e-9}));
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(zero.equals(zero, math::QuantityTolerance{makeQuantity<TypeParam>(invalid), 1e-9}), std::invalid_argument);
        EXPECT_THROW(zero.equals(boundary, math::QuantityTolerance{tolerance, invalid}), std::invalid_argument);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, AcceptsUnitPreservingTolerancePolicy)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto boundary = makeQuantity<TypeParam>(0.25);
    const math::QuantityTolerance<TypeParam> absolute{boundary, 0.0};
    EXPECT_TRUE(zero.equals(boundary, absolute));
    EXPECT_FALSE(zero.equals(makeQuantity<TypeParam>(std::nextafter(0.25, 1.0)), absolute));
    const auto value = makeQuantity<TypeParam>(1e6);
    const auto nearby = makeQuantity<TypeParam>(1e6 + 0.0005);
    const math::QuantityTolerance<TypeParam> defaults{
        makeQuantity<TypeParam>(math::DEFAULT_TOLERANCE.absolute()),
        math::DEFAULT_TOLERANCE.relative()
    };
    EXPECT_EQ(value == nearby, value.equals(nearby, defaults));
    EXPECT_TRUE(value.equals(nearby, defaults));
    EXPECT_FALSE(value.equals(nearby, math::QuantityTolerance<TypeParam>{zero, 0.0}));
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(zero.equals(zero, math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(invalid), 0.0}), std::invalid_argument);
        EXPECT_THROW(zero.equals(boundary, math::QuantityTolerance<TypeParam>{boundary, invalid}), std::invalid_argument);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, PolicyValidatesStoredUnitsExactly)
{
    EXPECT_NO_THROW((math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 0.0}));
    for (double invalid : {-0.5e-12, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(invalid), 0.0}), std::invalid_argument);
        EXPECT_THROW((math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), invalid}), std::invalid_argument);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, ChecksSignsExactly)
{
    const auto negative = makeQuantity<TypeParam>(-0.5e-12);
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto positive = makeQuantity<TypeParam>(0.5e-12);

    EXPECT_TRUE(negative.isNegative());
    EXPECT_TRUE(negative.isNonpositive());
    EXPECT_FALSE(negative.isPositive());
    EXPECT_FALSE(negative.isNonnegative());

    EXPECT_FALSE(zero.isNegative());
    EXPECT_TRUE(zero.isNonpositive());
    EXPECT_FALSE(zero.isPositive());
    EXPECT_TRUE(zero.isNonnegative());
    EXPECT_TRUE(makeQuantity<TypeParam>(-0.0).isNonnegative());

    EXPECT_FALSE(positive.isNegative());
    EXPECT_FALSE(positive.isNonpositive());
    EXPECT_TRUE(positive.isPositive());
    EXPECT_TRUE(positive.isNonnegative());
    EXPECT_TRUE(makeQuantity<TypeParam>(std::numeric_limits<double>::max()).isNonnegative());
}

// 계산 결과와 수치적 안정성을 검증한다.
TYPED_TEST(QuantityComparisonTest, AbsoluteBoundaryIsInclusive)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto boundary = makeQuantity<TypeParam>(1e-12);
    const auto outside = makeQuantity<TypeParam>(std::nextafter(
        1e-12, std::numeric_limits<double>::infinity()
    ));
    EXPECT_TRUE(zero == boundary);
    EXPECT_TRUE(zero <= boundary);
    EXPECT_TRUE(zero >= boundary);
    EXPECT_FALSE(zero < boundary);
    EXPECT_FALSE(zero > boundary);
    EXPECT_FALSE(zero.equals(boundary, math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 1e-9}));
    EXPECT_FALSE(zero == outside);
    EXPECT_TRUE(zero < outside);
    EXPECT_TRUE(outside > zero);
    EXPECT_FALSE(zero >= outside);
    EXPECT_FALSE(outside <= zero);
    const auto negative = -boundary;
    EXPECT_TRUE(zero == negative);
    EXPECT_TRUE(zero <= negative);
    EXPECT_TRUE(zero >= negative);
    EXPECT_FALSE(zero < negative);
    EXPECT_FALSE(zero > negative);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, RelativeToleranceControlsLargeValues)
{
    const auto value = makeQuantity<TypeParam>(1e6);
    const auto nearby = makeQuantity<TypeParam>(1e6 + 0.0005);
    const auto outside = makeQuantity<TypeParam>(1e6 + 0.002);
    EXPECT_TRUE(value == nearby);
    EXPECT_TRUE(value <= nearby);
    EXPECT_TRUE(value >= nearby);
    EXPECT_FALSE(value < nearby);
    EXPECT_FALSE(value > nearby);
    EXPECT_TRUE(-value == -nearby);
    EXPECT_TRUE((-value) <= -nearby);
    EXPECT_TRUE((-value) >= -nearby);
    EXPECT_FALSE(value == outside);
    EXPECT_TRUE(value < outside);
    EXPECT_TRUE(outside > value);
    EXPECT_FALSE(value >= outside);
    EXPECT_FALSE(outside <= value);
    EXPECT_TRUE(-outside < -value);
}

TYPED_TEST(QuantityComparisonTest, ExplicitTolerancesRemainIndependentAndRejectTinyNegatives)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto nearby = makeQuantity<TypeParam>(0.5e-12);
    const auto negativeTolerance = makeQuantity<TypeParam>(
        -0.5e-12
    );
    EXPECT_TRUE(zero == nearby);
    EXPECT_FALSE(zero.equals(nearby, math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 0.0}));
    EXPECT_FALSE(zero.equals(nearby, math::QuantityTolerance{makeQuantity<TypeParam>(0.0), 0.0}));
    EXPECT_THROW(zero.equals(zero, math::QuantityTolerance{negativeTolerance, 0.0}), std::invalid_argument);
}

TYPED_TEST(QuantityComparisonTest, RejectsNonfiniteValuesAtConstruction)
{
    EXPECT_THROW(makeQuantity<TypeParam>(std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_THROW(makeQuantity<TypeParam>(-std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_THROW(makeQuantity<TypeParam>(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TYPED_TEST(EqualsTest, DefaultsHandleExtremeFiniteValuesAndDimensions)
{
    const double largest = std::numeric_limits<double>::max();
    const auto value = makeValue<TypeParam>(largest, 0.0);
    EXPECT_TRUE(value == makeValue<TypeParam>(std::nextafter(largest, 0.0), 0.0));
    EXPECT_FALSE(value == makeValue<TypeParam>(-largest, 0.0));
    EXPECT_TRUE(makeValue<TypeParam>(0.0, 0.0) == makeValue<TypeParam>(std::numeric_limits<double>::denorm_min(), 0.0));
    EXPECT_FALSE(value == TypeParam());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixEqualsTest, SupportsRectangularAndZeroDimensionShapes)
{
    const math::Matrix value{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const math::Matrix nearby{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.25}};
    EXPECT_TRUE(value.equals(nearby, math::Tolerance{0.25, 0.0}));
    EXPECT_FALSE(value.equals(nearby, math::Tolerance{std::nextafter(0.25, 0.0), 0.0}));
    EXPECT_TRUE(value.equals(nearby, math::Tolerance{0.0, 0.04}));
    EXPECT_FALSE(value.equals(nearby, math::Tolerance{0.0, 0.02}));
    EXPECT_FALSE(value.equals(math::transpose(value), math::Tolerance{0.0, 1.0}));
    EXPECT_TRUE(math::Matrix(0, 3) == math::Matrix(0, 3));
    EXPECT_TRUE(math::Matrix(3, 0).equals(math::Matrix(3, 0), math::Tolerance{0.0, 0.0}));
    EXPECT_FALSE(math::Matrix(0, 3) == math::Matrix(3, 0));
    EXPECT_FALSE(math::Matrix(0, 3).equals(math::Matrix(0, 0), math::Tolerance{0.0, 0.0}));
}

}
