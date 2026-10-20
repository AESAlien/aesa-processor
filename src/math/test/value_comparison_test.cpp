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

using namespace aesa_processor::math::literals;

namespace
{

template<typename T>
T makeValue(double first, double last)
{
    if constexpr (std::is_same_v<T, aesa_processor::math::SquareMatrix> || std::is_same_v<T, aesa_processor::math::Matrix>)
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

using EqualsTypes = testing::Types<aesa_processor::math::Matrix, aesa_processor::math::SquareMatrix, aesa_processor::math::Vector>;
TYPED_TEST_SUITE(EqualsTest, EqualsTypes);

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, HandlesRoundingAndDefaultTolerances)
{
    const auto calculated = makeValue<TypeParam>(0.1 + 0.2, 1.0);
    const auto expected = makeValue<TypeParam>(0.3, 1.0);
    EXPECT_TRUE(calculated.equals(expected));
    EXPECT_TRUE(calculated == expected);
    EXPECT_FALSE(calculated != expected);
    EXPECT_FALSE(calculated.equals(expected, aesa_processor::math::NONE_TOLERANCE));
    EXPECT_TRUE(calculated.equals(expected, aesa_processor::math::DEFAULT_TOLERANCE));
    EXPECT_FALSE(expected == makeValue<TypeParam>(0.4, 1.0));
    EXPECT_FALSE(expected.equals(makeValue<TypeParam>(0.4, 1.0)));
    EXPECT_TRUE(expected != makeValue<TypeParam>(0.4, 1.0));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, AbsoluteToleranceIncludesBoundary)
{
    const auto zero = makeValue<TypeParam>(0.0, 0.0);
    const auto boundary = makeValue<TypeParam>(aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE, 0.0);
    const auto outside = makeValue<TypeParam>(std::nextafter(
        aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE, std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_TRUE(zero == boundary);
    EXPECT_TRUE(boundary == zero);
    EXPECT_FALSE(zero == outside);
    EXPECT_FALSE(boundary.equals(zero, aesa_processor::math::Tolerance{0.0, aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}));
    const aesa_processor::math::Tolerance absolute{0.25, 0.0};
    EXPECT_TRUE(zero.equals(makeValue<TypeParam>(0.25, 0.25), absolute));
    EXPECT_FALSE(zero.equals(makeValue<TypeParam>(0.25, std::nextafter(0.25, 1.0)), absolute));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, RelativeToleranceIsSymmetricAndInclusive)
{
    const auto value = makeValue<TypeParam>(4.0, 0.0);
    const auto nearby = makeValue<TypeParam>(8.0, 0.0);
    const aesa_processor::math::Tolerance relative{0.0, 0.5};
    EXPECT_TRUE(value.equals(nearby, relative));
    EXPECT_TRUE(nearby.equals(value, relative));
    EXPECT_FALSE(value.equals(nearby, aesa_processor::math::Tolerance{0.0, std::nextafter(0.5, 0.0)}));
    EXPECT_FALSE(makeValue<TypeParam>(1e20, 0.0).equals(
        makeValue<TypeParam>(1e20, 1.0), aesa_processor::math::Tolerance{0.0, 1e-5}));
    EXPECT_FALSE(makeValue<TypeParam>(0.0, 0.0).equals(
        makeValue<TypeParam>(1e-12, 0.0), aesa_processor::math::Tolerance{0.0, 0.01}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, CombinesAbsoluteAndRelativeTolerancePerElement)
{
    const auto value = makeValue<TypeParam>(0.0, 1e6);
    const auto nearby = makeValue<TypeParam>(0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE, 1e6 + 0.0005);
    EXPECT_FALSE(value.equals(nearby, aesa_processor::math::Tolerance{aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE, 0.0}));
    EXPECT_FALSE(value.equals(nearby, aesa_processor::math::Tolerance{0.0, aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}));
    EXPECT_TRUE(value == nearby);
    EXPECT_TRUE(nearby == value);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(EqualsTest, ChecksDimensionsAndSupportsEmptyValues)
{
    const auto value = makeValue<TypeParam>(1.0, 2.0);
    EXPECT_FALSE(value.equals(TypeParam(), aesa_processor::math::DEFAULT_TOLERANCE));
    EXPECT_FALSE(value == TypeParam());
    EXPECT_TRUE(value != TypeParam());
    EXPECT_TRUE(TypeParam() == TypeParam());
    EXPECT_TRUE(TypeParam().equals(TypeParam(), aesa_processor::math::Tolerance{0.0, 0.0}));
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TYPED_TEST(EqualsTest, RejectsNonfiniteElements)
{
    const auto finite = makeValue<TypeParam>(1.0, 2.0);
    for (double invalid : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        const auto value = makeValue<TypeParam>(1.0, invalid);
        EXPECT_FALSE(value.equals(value));
        EXPECT_FALSE(value == value);
        EXPECT_TRUE(value != value);
        EXPECT_FALSE(value.equals(finite, aesa_processor::math::DEFAULT_TOLERANCE));
        EXPECT_FALSE(finite.equals(value, aesa_processor::math::DEFAULT_TOLERANCE));
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TYPED_TEST(EqualsTest, HandlesNumericLimits)
{
    const double largest = std::numeric_limits<double>::max();
    const double smallest = std::numeric_limits<double>::denorm_min();
    const auto positive = makeValue<TypeParam>(largest, 0.0);
    const auto negative = makeValue<TypeParam>(-largest, 0.0);
    EXPECT_FALSE(positive.equals(negative, aesa_processor::math::Tolerance{0.0, 1.5}));
    EXPECT_TRUE(positive.equals(negative, aesa_processor::math::Tolerance{0.0, 2.0}));
    const auto tiny = makeValue<TypeParam>(smallest, 0.0);
    const auto twiceTiny = makeValue<TypeParam>(2.0 * smallest, 0.0);
    EXPECT_TRUE(tiny.equals(twiceTiny, aesa_processor::math::Tolerance{smallest, 0.0}));
    EXPECT_FALSE(tiny.equals(twiceTiny, aesa_processor::math::Tolerance{0.0, 0.0}));
    EXPECT_TRUE(tiny.equals(twiceTiny, aesa_processor::math::Tolerance{0.0, 0.5}));
    EXPECT_FALSE(tiny.equals(twiceTiny, aesa_processor::math::Tolerance{0.0, std::nextafter(0.5, 0.0)}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(ToleranceTest, ComparesFiniteNumbersUsingItsPolicy)
{
    const aesa_processor::math::Tolerance combined{1e-12, 1e-9};
    EXPECT_TRUE(combined.isWithin(0.1 + 0.2, 0.3));
    EXPECT_TRUE(combined.isWithin(0.0, 1e-12));
    EXPECT_FALSE(combined.isWithin(0.0, std::nextafter(1e-12, 1.0)));
    EXPECT_TRUE(combined.isWithin(1e6, 1e6 + 0.0005));
    EXPECT_FALSE(combined.isWithin(1e6, 1e6 + 0.002));
    const double largest = std::numeric_limits<double>::max();
    EXPECT_FALSE((aesa_processor::math::Tolerance{0.0, 1.5}).isWithin(largest, -largest));
    EXPECT_TRUE((aesa_processor::math::Tolerance{0.0, 2.0}).isWithin(largest, -largest));
    for (double nonfinite : {std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_FALSE(combined.isWithin(nonfinite, nonfinite));
        EXPECT_FALSE(combined.isWithin(0.0, nonfinite));
        EXPECT_FALSE(combined.isWithin(nonfinite, 0.0));
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(ToleranceTest, NoneToleranceRequiresExactFiniteEquality)
{
    EXPECT_DOUBLE_EQ(aesa_processor::math::NONE_TOLERANCE.absolute, 0.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::NONE_TOLERANCE.relative, 0.0);
    EXPECT_TRUE(aesa_processor::math::NONE_TOLERANCE.isWithin(1.0, 1.0));
    EXPECT_TRUE(aesa_processor::math::NONE_TOLERANCE.isWithin(0.0, -0.0));
    EXPECT_FALSE(aesa_processor::math::NONE_TOLERANCE.isWithin(0.1 + 0.2, 0.3));
    EXPECT_FALSE(aesa_processor::math::NONE_TOLERANCE.isWithin(1.0, std::nextafter(1.0, 2.0)));
    EXPECT_FALSE(aesa_processor::math::NONE_TOLERANCE.isWithin(0.0, std::numeric_limits<double>::denorm_min()));
    for (double nonfinite : {std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_FALSE(aesa_processor::math::NONE_TOLERANCE.isWithin(nonfinite, nonfinite));
        EXPECT_FALSE(aesa_processor::math::NONE_TOLERANCE.isWithin(0.0, nonfinite));
        EXPECT_FALSE(aesa_processor::math::NONE_TOLERANCE.isWithin(nonfinite, 0.0));
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(ToleranceTest, ValidatesScalarPolicyAtConstruction)
{
    EXPECT_NO_THROW((aesa_processor::math::Tolerance{-0.0, 0.0}));
    EXPECT_NO_THROW((aesa_processor::math::Tolerance{std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()}));
    for (double invalid : {-0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((aesa_processor::math::Tolerance{invalid, 0.0}), std::invalid_argument);
        EXPECT_THROW((aesa_processor::math::Tolerance{0.0, invalid}), std::invalid_argument);
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(ToleranceTest, PreservesUnitsAndSupportsConstantPolicies)
{
    const aesa_processor::math::Tolerance exact{0.0, 0.0};
    EXPECT_DOUBLE_EQ(exact.absolute, 0.0);
    EXPECT_DOUBLE_EQ(exact.relative, 0.0);
    static_assert(std::is_const_v<decltype(exact.absolute)>);
    static_assert(std::is_const_v<decltype(exact.relative)>);
    static_assert(!std::is_constructible_v<aesa_processor::math::QuantityTolerance<aesa_processor::math::Angle>, aesa_processor::math::Length, double>);
    static_assert(!std::is_assignable_v<aesa_processor::math::Tolerance&, aesa_processor::math::Tolerance>);
    EXPECT_FALSE((aesa_processor::math::Matrix{{0.1 + 0.2}}).equals(aesa_processor::math::Matrix{{0.3}}, exact));
    EXPECT_TRUE((aesa_processor::math::Matrix{{0.1 + 0.2}}).equals(aesa_processor::math::Matrix{{0.3}}, aesa_processor::math::DEFAULT_TOLERANCE));
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(ToleranceTest, OwnsAbsoluteValueAndPreservesItAfterCopyAndMove)
{
    auto absolute = 0.25_deg;
    const aesa_processor::math::QuantityTolerance<aesa_processor::math::Angle> original{absolute, 0.0};
    absolute = 0.0_deg;
    auto source = original;
    const auto moved = std::move(source);
    const auto zero = 0.0_deg;
    const auto boundary = 0.25_deg;
    EXPECT_TRUE(zero.equals(boundary, original));
    EXPECT_TRUE(zero.equals(boundary, source));
    EXPECT_TRUE(zero.equals(boundary, moved));
    EXPECT_FALSE(zero.equals(boundary, aesa_processor::math::QuantityTolerance{absolute, 0.0}));
}

template<typename T>
T makeQuantity(double value)
{
    if constexpr (std::is_same_v<T, aesa_processor::math::Angle>)
    {
        return aesa_processor::math::Angle::fromDegrees(value);
    }
    else if constexpr (std::is_same_v<T, aesa_processor::math::Length>)
    {
        return aesa_processor::math::Length::fromMeters(value);
    }
    else if constexpr (std::is_same_v<T, aesa_processor::math::LinearAcceleration>)
    {
        return aesa_processor::math::LinearAcceleration::fromMetersPerSecondSquared(value);
    }
    else
    {
        return aesa_processor::math::LinearVelocity::fromMetersPerSecond(value);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(QuantityTest, SharesStorageAndChecksWithoutLosingUnitSafety)
{
    static_assert(std::is_base_of_v<aesa_processor::math::Quantity<aesa_processor::math::Angle>, aesa_processor::math::Angle>);
    static_assert(std::is_base_of_v<aesa_processor::math::Quantity<aesa_processor::math::Length>, aesa_processor::math::Length>);
    static_assert(std::is_base_of_v<aesa_processor::math::Quantity<aesa_processor::math::LinearVelocity>, aesa_processor::math::LinearVelocity>);
    static_assert(std::is_base_of_v<aesa_processor::math::Quantity<aesa_processor::math::LinearAcceleration>, aesa_processor::math::LinearAcceleration>);
    static_assert(sizeof(aesa_processor::math::Angle) == sizeof(double));
    static_assert(sizeof(aesa_processor::math::Length) == sizeof(double));
    static_assert(sizeof(aesa_processor::math::LinearVelocity) == sizeof(double));
    static_assert(sizeof(aesa_processor::math::LinearAcceleration) == sizeof(double));
    static_assert(!std::is_assignable_v<aesa_processor::math::Quantity<aesa_processor::math::Length>&, aesa_processor::math::Length>);
    static_assert(!std::is_constructible_v<aesa_processor::math::Angle, aesa_processor::math::Length>);

    const auto angle = -90.0_deg;
    const aesa_processor::math::Quantity<aesa_processor::math::Angle>& quantity = angle;
    EXPECT_DOUBLE_EQ(quantity.value(), angle.deg());
    EXPECT_FALSE(quantity.isNonnegative());
    EXPECT_DOUBLE_EQ((2.0_km).value(), 2000.0);
    EXPECT_DOUBLE_EQ((3.0_mps).value(), 3.0);
    EXPECT_DOUBLE_EQ((4.0_mps2).value(), 4.0);
}

// 새 물리량도 단위별 분기 없이 공통 정책을 사용할 수 있다.
class TestQuantity : public aesa_processor::math::Quantity<TestQuantity>
{
public:
    static const aesa_processor::math::QuantityTolerance<TestQuantity> DEFAULT_TOLERANCE;

    explicit TestQuantity(double value)
        : Quantity<TestQuantity>(value)
    {
    }
};

const aesa_processor::math::QuantityTolerance<TestQuantity> TestQuantity::DEFAULT_TOLERANCE{
    TestQuantity(0.5), 0.0
};

// 계산 결과와 수치적 안정성을 검증한다.
TEST(QuantityTest, NewQuantityInheritsFiniteStorageValidation)
{
    const double INVALID_VALUES[] =
    {
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()
    };
    for (double value : INVALID_VALUES)
    {
        EXPECT_THROW(TestQuantity{value}, std::invalid_argument);
    }
    const double MAX_VALUE = std::numeric_limits<double>::max();
    EXPECT_DOUBLE_EQ(TestQuantity(MAX_VALUE).value(), MAX_VALUE);
    EXPECT_DOUBLE_EQ(TestQuantity(-MAX_VALUE).value(), -MAX_VALUE);
    EXPECT_TRUE(TestQuantity(-0.0).isNonnegative());
    EXPECT_FALSE(TestQuantity(-std::numeric_limits<double>::denorm_min()).isNonnegative());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(QuantityTest, NewQuantityInheritsComparisonsAndUsesItsOwnDefaultTolerance)
{
    const TestQuantity LOW(1.0);
    const TestQuantity NEARBY(1.5);
    const TestQuantity HIGH(2.0);
    const aesa_processor::math::QuantityTolerance EXACT{TestQuantity(0.0), 0.0};
    EXPECT_TRUE(LOW.equals(NEARBY));
    EXPECT_FALSE(LOW.equals(NEARBY, EXACT));
    EXPECT_FALSE(LOW.notEquals(NEARBY));
    EXPECT_TRUE(LOW.notEquals(NEARBY, EXACT));
    EXPECT_FALSE(LOW.lessThan(NEARBY));
    EXPECT_TRUE(LOW.lessThan(NEARBY, EXACT));
    EXPECT_TRUE(NEARBY.greaterThan(LOW, EXACT));
    EXPECT_TRUE(LOW.lessThanOrEqual(NEARBY));
    EXPECT_TRUE(LOW.greaterThanOrEqual(NEARBY));
    EXPECT_FALSE(LOW.greaterThanOrEqual(NEARBY, EXACT));
    EXPECT_FALSE(NEARBY.lessThanOrEqual(LOW, EXACT));
    EXPECT_TRUE(LOW == NEARBY);
    EXPECT_FALSE(LOW != NEARBY);
    EXPECT_FALSE(LOW < NEARBY);
    EXPECT_FALSE(NEARBY > LOW);
    EXPECT_TRUE(LOW <= NEARBY);
    EXPECT_TRUE(LOW >= NEARBY);
    EXPECT_TRUE(LOW != HIGH);
    EXPECT_TRUE(LOW < HIGH);
    EXPECT_TRUE(HIGH > LOW);
}

template<typename Left, typename Right>
void checkComparisonTypes()
{
    constexpr bool SAME_TYPE = std::is_same_v<Left, Right>;
    using Policy = aesa_processor::math::QuantityTolerance<Left>;
    static_assert(std::is_invocable_v<decltype(&Left::equals), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::notEquals), const Left&, const Right&, const Policy&> == SAME_TYPE);
    static_assert(std::is_invocable_v<decltype(&Left::notEquals), const Left&, const Left&,
        const aesa_processor::math::QuantityTolerance<Right>&> == SAME_TYPE);
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
    static_assert(std::is_invocable_v<decltype(&Left::equals), const Left&, const Left&,
        const aesa_processor::math::QuantityTolerance<Right>&> == SAME_TYPE);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(QuantityToleranceTest, SupportsNewQuantitiesAndRestrictsComparisonTypes)
{
    using AnglePolicy = aesa_processor::math::QuantityTolerance<aesa_processor::math::Angle>;
    static_assert(!std::is_invocable_v<decltype(&AnglePolicy::isWithin),
        const AnglePolicy&, aesa_processor::math::Length, aesa_processor::math::Length>);
    static_assert(!std::is_assignable_v<AnglePolicy&, AnglePolicy>);
    const aesa_processor::math::QuantityTolerance policy{TestQuantity(0.25), 0.125};
    EXPECT_TRUE(policy.isWithin(TestQuantity(0.0), TestQuantity(0.25)));
    EXPECT_FALSE(policy.isWithin(TestQuantity(0.0), TestQuantity(0.5)));
    EXPECT_TRUE(policy.isWithin(TestQuantity(7.0), TestQuantity(8.0)));
    EXPECT_FALSE(policy.isWithin(TestQuantity(6.0), TestQuantity(8.0)));
    EXPECT_THROW((aesa_processor::math::QuantityTolerance{TestQuantity(-1e-20), 0.0}), std::invalid_argument);
}

template<typename T>
class QuantityComparisonTest : public testing::Test
{
};

using QuantityTypes = testing::Types<
    aesa_processor::math::Angle,
    aesa_processor::math::Length,
    aesa_processor::math::LinearVelocity,
    aesa_processor::math::LinearAcceleration>;
TYPED_TEST_SUITE(QuantityComparisonTest, QuantityTypes);

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NoneToleranceDisablesAbsoluteAndRelativeTolerance)
{
    const auto& NONE = TypeParam::NONE_TOLERANCE;
    static_assert(std::is_same_v<decltype(NONE), const aesa_processor::math::QuantityTolerance<TypeParam>&>);
    EXPECT_DOUBLE_EQ(NONE.absolute().value(), 0.0);
    EXPECT_DOUBLE_EQ(NONE.relative(), 0.0);
    const auto ZERO = makeQuantity<TypeParam>(0.0);
    EXPECT_TRUE(ZERO.equals(ZERO, NONE));
    EXPECT_FALSE(ZERO.notEquals(ZERO, NONE));
    for (double value : {0.0, 1e6})
    {
        const auto LOW = makeQuantity<TypeParam>(value);
        const auto HIGH = makeQuantity<TypeParam>(std::nextafter(value, std::numeric_limits<double>::infinity()));
        EXPECT_TRUE(LOW.equals(HIGH));
        EXPECT_FALSE(LOW.equals(HIGH, NONE));
        EXPECT_TRUE(LOW.notEquals(HIGH, NONE));
        EXPECT_TRUE(LOW.lessThan(HIGH, NONE));
        EXPECT_TRUE(HIGH.greaterThan(LOW, NONE));
        EXPECT_FALSE(LOW.greaterThanOrEqual(HIGH, NONE));
        EXPECT_FALSE(HIGH.lessThanOrEqual(LOW, NONE));
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, RestrictsComparisonsToMatchingQuantityTypes)
{
    checkComparisonTypes<TypeParam, aesa_processor::math::Angle>();
    checkComparisonTypes<TypeParam, aesa_processor::math::Length>();
    checkComparisonTypes<TypeParam, aesa_processor::math::LinearVelocity>();
    checkComparisonTypes<TypeParam, aesa_processor::math::LinearAcceleration>();
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NamedInequalitySupportsExplicitTolerances)
{
    const auto ZERO = makeQuantity<TypeParam>(0.0);
    const auto BOUNDARY = makeQuantity<TypeParam>(0.25);
    const auto OUTSIDE = makeQuantity<TypeParam>(std::nextafter(0.25, 1.0));
    const aesa_processor::math::QuantityTolerance ABSOLUTE{BOUNDARY, 0.0};
    const aesa_processor::math::QuantityTolerance EXACT{ZERO, 0.0};
    EXPECT_FALSE(ZERO.notEquals(ZERO, EXACT));
    EXPECT_FALSE(ZERO.notEquals(BOUNDARY, ABSOLUTE));
    EXPECT_FALSE(BOUNDARY.notEquals(ZERO, ABSOLUTE));
    EXPECT_TRUE(ZERO.notEquals(OUTSIDE, ABSOLUTE));
    EXPECT_TRUE(ZERO.notEquals(BOUNDARY, EXACT));
    const auto LOW = makeQuantity<TypeParam>(4.0);
    const auto HIGH = makeQuantity<TypeParam>(8.0);
    const aesa_processor::math::QuantityTolerance RELATIVE{ZERO, 0.5};
    const aesa_processor::math::QuantityTolerance BELOW_BOUNDARY{ZERO, std::nextafter(0.5, 0.0)};
    EXPECT_FALSE(LOW.notEquals(HIGH, RELATIVE));
    EXPECT_FALSE(HIGH.notEquals(LOW, RELATIVE));
    EXPECT_TRUE(LOW.notEquals(HIGH, BELOW_BOUNDARY));
    const auto NEARBY = makeQuantity<TypeParam>(0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE);
    EXPECT_FALSE(ZERO.notEquals(NEARBY));
    EXPECT_TRUE(ZERO.notEquals(NEARBY, EXACT));
    EXPECT_TRUE(ZERO.notEquals(HIGH));
    EXPECT_EQ(ZERO.notEquals(NEARBY), ZERO != NEARBY);
    EXPECT_EQ(ZERO.notEquals(HIGH), ZERO != HIGH);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NamedOrderingUsesExplicitToleranceBoundaries)
{
    const auto ZERO = makeQuantity<TypeParam>(0.0);
    const aesa_processor::math::QuantityTolerance ABSOLUTE{makeQuantity<TypeParam>(0.25), 0.0};
    const aesa_processor::math::QuantityTolerance RELATIVE{ZERO, 0.5};
    const aesa_processor::math::QuantityTolerance EXACT{ZERO, 0.0};
    for (double sign : {-1.0, 1.0})
    {
        const auto BOUNDARY = makeQuantity<TypeParam>(sign * 0.25);
        EXPECT_FALSE(ZERO.lessThan(BOUNDARY, ABSOLUTE));
        EXPECT_FALSE(ZERO.greaterThan(BOUNDARY, ABSOLUTE));
        EXPECT_TRUE(ZERO.lessThanOrEqual(BOUNDARY, ABSOLUTE));
        EXPECT_TRUE(ZERO.greaterThanOrEqual(BOUNDARY, ABSOLUTE));
        const auto OUTSIDE = makeQuantity<TypeParam>(sign * std::nextafter(0.25, 1.0));
        EXPECT_EQ(ZERO.lessThan(OUTSIDE, ABSOLUTE), sign > 0.0);
        EXPECT_EQ(ZERO.greaterThan(OUTSIDE, ABSOLUTE), sign < 0.0);
        EXPECT_EQ(ZERO.lessThanOrEqual(OUTSIDE, ABSOLUTE), sign > 0.0);
        EXPECT_EQ(ZERO.greaterThanOrEqual(OUTSIDE, ABSOLUTE), sign < 0.0);
    }
    const auto LOW = makeQuantity<TypeParam>(4.0);
    const auto HIGH = makeQuantity<TypeParam>(8.0);
    EXPECT_FALSE(LOW.lessThan(HIGH, RELATIVE));
    EXPECT_FALSE(HIGH.greaterThan(LOW, RELATIVE));
    EXPECT_TRUE(LOW.greaterThanOrEqual(HIGH, RELATIVE));
    EXPECT_TRUE(HIGH.lessThanOrEqual(LOW, RELATIVE));
    const aesa_processor::math::QuantityTolerance BELOW_BOUNDARY{ZERO, std::nextafter(0.5, 0.0)};
    EXPECT_TRUE(LOW.lessThan(HIGH, BELOW_BOUNDARY));
    EXPECT_TRUE(HIGH.greaterThan(LOW, BELOW_BOUNDARY));
    EXPECT_FALSE(LOW.greaterThanOrEqual(HIGH, BELOW_BOUNDARY));
    EXPECT_FALSE(HIGH.lessThanOrEqual(LOW, BELOW_BOUNDARY));
    const auto NEARBY = makeQuantity<TypeParam>(0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE);
    EXPECT_TRUE(ZERO.lessThan(NEARBY, EXACT));
    EXPECT_TRUE(NEARBY.greaterThan(ZERO, EXACT));
    EXPECT_FALSE(ZERO.greaterThanOrEqual(NEARBY, EXACT));
    EXPECT_FALSE(NEARBY.lessThanOrEqual(ZERO, EXACT));
    EXPECT_FALSE(ZERO.lessThan(ZERO, EXACT));
    EXPECT_FALSE(ZERO.greaterThan(ZERO, EXACT));
    EXPECT_TRUE(ZERO.lessThanOrEqual(ZERO, EXACT));
    EXPECT_TRUE(ZERO.greaterThanOrEqual(ZERO, EXACT));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, NamedOrderingMatchesDefaultOperators)
{
    const double VALUES[] =
    {
        -1.0,
        0.0,
        0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE,
        1.0,
    };
    const aesa_processor::math::QuantityTolerance EXACT{makeQuantity<TypeParam>(0.0), 0.0};
    for (double left : VALUES)
    {
        for (double right : VALUES)
        {
            const auto LEFT = makeQuantity<TypeParam>(left);
            const auto RIGHT = makeQuantity<TypeParam>(right);
            EXPECT_EQ(LEFT.lessThan(RIGHT), LEFT < RIGHT);
            EXPECT_EQ(LEFT.lessThanOrEqual(RIGHT), LEFT <= RIGHT);
            EXPECT_EQ(LEFT.greaterThan(RIGHT), LEFT > RIGHT);
            EXPECT_EQ(LEFT.greaterThanOrEqual(RIGHT), LEFT >= RIGHT);
            EXPECT_EQ(LEFT.lessThan(RIGHT, EXACT), left < right);
            EXPECT_EQ(LEFT.lessThanOrEqual(RIGHT, EXACT), left <= right);
            EXPECT_EQ(LEFT.greaterThan(RIGHT, EXACT), left > right);
            EXPECT_EQ(LEFT.greaterThanOrEqual(RIGHT, EXACT), left >= right);
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
    EXPECT_FALSE(calculated.equals(expected, aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 0.0}));
    EXPECT_FALSE(calculated.equals(expected, aesa_processor::math::QuantityTolerance{makeQuantity<TypeParam>(0.0), 0.0}));
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
    EXPECT_TRUE(zero.equals(boundary, aesa_processor::math::QuantityTolerance{tolerance, 0.0}));
    EXPECT_FALSE(zero.equals(makeQuantity<TypeParam>(std::nextafter(0.25, 1.0)), aesa_processor::math::QuantityTolerance{tolerance, 0.0}));
    const auto large = makeQuantity<TypeParam>(1e6);
    const auto nearby = makeQuantity<TypeParam>(1e6 + 0.0005);
    EXPECT_TRUE(large.equals(nearby, aesa_processor::math::QuantityTolerance{zero, aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}));
    EXPECT_TRUE(nearby.equals(large, aesa_processor::math::QuantityTolerance{zero, aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}));
    EXPECT_FALSE(large.equals(nearby, aesa_processor::math::QuantityTolerance{zero, 0.0}));
    EXPECT_EQ(large == nearby, large.equals(nearby, aesa_processor::math::QuantityTolerance{makeQuantity<TypeParam>(aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE), aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}));
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(zero.equals(zero, aesa_processor::math::QuantityTolerance{makeQuantity<TypeParam>(invalid), aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}), std::invalid_argument);
        EXPECT_THROW(zero.equals(boundary, aesa_processor::math::QuantityTolerance{tolerance, invalid}), std::invalid_argument);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, AcceptsUnitPreservingTolerancePolicy)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto boundary = makeQuantity<TypeParam>(0.25);
    const aesa_processor::math::QuantityTolerance<TypeParam> absolute{boundary, 0.0};
    EXPECT_TRUE(zero.equals(boundary, absolute));
    EXPECT_FALSE(zero.equals(makeQuantity<TypeParam>(std::nextafter(0.25, 1.0)), absolute));
    const auto value = makeQuantity<TypeParam>(1e6);
    const auto nearby = makeQuantity<TypeParam>(1e6 + 0.0005);
    const aesa_processor::math::QuantityTolerance<TypeParam> defaults{
        makeQuantity<TypeParam>(aesa_processor::math::DEFAULT_TOLERANCE.absolute),
        aesa_processor::math::DEFAULT_TOLERANCE.relative};
    EXPECT_EQ(value == nearby, value.equals(nearby, defaults));
    EXPECT_TRUE(value.equals(nearby, defaults));
    EXPECT_FALSE(value.equals(nearby, aesa_processor::math::QuantityTolerance<TypeParam>{zero, 0.0}));
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(zero.equals(zero,
            aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(invalid), 0.0}), std::invalid_argument);
        EXPECT_THROW(zero.equals(boundary, aesa_processor::math::QuantityTolerance<TypeParam>{boundary, invalid}), std::invalid_argument);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, PolicyValidatesStoredUnitsExactly)
{
    EXPECT_NO_THROW((aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 0.0}));
    for (double invalid : {-0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(invalid), 0.0}), std::invalid_argument);
        EXPECT_THROW((aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), invalid}), std::invalid_argument);
    }
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TYPED_TEST(QuantityComparisonTest, ChecksNonnegativity)
{
    EXPECT_TRUE(makeQuantity<TypeParam>(-0.0).isNonnegative());
    EXPECT_TRUE(makeQuantity<TypeParam>(std::numeric_limits<double>::max()).isNonnegative());
    const auto negative = makeQuantity<TypeParam>(-0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE);
    EXPECT_FALSE(negative.isNonnegative());
}

// 계산 결과와 수치적 안정성을 검증한다.
TYPED_TEST(QuantityComparisonTest, AbsoluteBoundaryIsInclusive)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto boundary = makeQuantity<TypeParam>(aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE);
    const auto outside = makeQuantity<TypeParam>(std::nextafter(
        aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE, std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(zero == boundary);
    EXPECT_TRUE(zero <= boundary);
    EXPECT_TRUE(zero >= boundary);
    EXPECT_FALSE(zero < boundary);
    EXPECT_FALSE(zero > boundary);
    EXPECT_FALSE(zero.equals(boundary, aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), aesa_processor::math::DEFAULT_RELATIVE_TOLERANCE}));
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

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TYPED_TEST(QuantityComparisonTest, ExplicitTolerancesRemainIndependentAndRejectTinyNegatives)
{
    const auto zero = makeQuantity<TypeParam>(0.0);
    const auto nearby = makeQuantity<TypeParam>(0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE);
    const auto negativeTolerance = makeQuantity<TypeParam>(
        -0.5 * aesa_processor::math::DEFAULT_ABSOLUTE_TOLERANCE);
    EXPECT_TRUE(zero == nearby);
    EXPECT_FALSE(zero.equals(nearby, aesa_processor::math::QuantityTolerance<TypeParam>{makeQuantity<TypeParam>(0.0), 0.0}));
    EXPECT_FALSE(zero.equals(nearby, aesa_processor::math::QuantityTolerance{makeQuantity<TypeParam>(0.0), 0.0}));
    EXPECT_THROW(zero.equals(zero, aesa_processor::math::QuantityTolerance{negativeTolerance, 0.0}), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
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
    EXPECT_TRUE(makeValue<TypeParam>(0.0, 0.0) ==
        makeValue<TypeParam>(std::numeric_limits<double>::denorm_min(), 0.0));
    EXPECT_FALSE(value == TypeParam());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixEqualsTest, SupportsRectangularAndZeroDimensionShapes)
{
    const aesa_processor::math::Matrix value{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const aesa_processor::math::Matrix nearby{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.25}};
    EXPECT_TRUE(value.equals(nearby, aesa_processor::math::Tolerance{0.25, 0.0}));
    EXPECT_FALSE(value.equals(nearby,
        aesa_processor::math::Tolerance{std::nextafter(0.25, 0.0), 0.0}));
    EXPECT_TRUE(value.equals(nearby, aesa_processor::math::Tolerance{0.0, 0.04}));
    EXPECT_FALSE(value.equals(nearby, aesa_processor::math::Tolerance{0.0, 0.02}));
    EXPECT_FALSE(value.equals(aesa_processor::math::transpose(value), aesa_processor::math::Tolerance{0.0, 1.0}));
    EXPECT_TRUE(aesa_processor::math::Matrix(0, 3) == aesa_processor::math::Matrix(0, 3));
    EXPECT_TRUE(aesa_processor::math::Matrix(3, 0).equals(aesa_processor::math::Matrix(3, 0), aesa_processor::math::Tolerance{0.0, 0.0}));
    EXPECT_FALSE(aesa_processor::math::Matrix(0, 3) == aesa_processor::math::Matrix(3, 0));
    EXPECT_FALSE(aesa_processor::math::Matrix(0, 3).equals(aesa_processor::math::Matrix(0, 0), aesa_processor::math::Tolerance{0.0, 0.0}));
}

}
