#include "math/linear_algebra/vector.hpp"
#include "math/linear_algebra/square_matrix.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

static_assert(!std::is_convertible_v<aesa_processor::math::Matrix, aesa_processor::math::Vector>);
static_assert(!std::is_assignable_v<aesa_processor::math::Vector&, aesa_processor::math::Matrix>);
static_assert(std::is_same_v<decltype(std::declval<aesa_processor::math::Vector&>().asMatrix()), const aesa_processor::math::Matrix&>);
// 비상수 객체는 원소를 수정할 수 있다.
static_assert(std::is_assignable_v<decltype(std::declval<aesa_processor::math::Vector&>().at(0)), double>);
static_assert(std::is_assignable_v<decltype(std::declval<aesa_processor::math::Vector&>()(0)), double>);
static_assert(std::is_assignable_v<decltype(std::declval<aesa_processor::math::Vector&>()[0]), double>);
static_assert(std::is_same_v<decltype(std::declval<const aesa_processor::math::Vector&>().at(0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const aesa_processor::math::Vector&>()(0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const aesa_processor::math::Vector&>()[0]), const double&>);



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(VectorTest, ConstructsColumnVectors)
{
    aesa_processor::math::Vector empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.size(), 0);
    EXPECT_EQ(empty.asMatrix().rows(), 0);
    EXPECT_EQ(empty.asMatrix().columns(), 1);
    aesa_processor::math::Vector filled(3, 4.5);
    ASSERT_EQ(filled.size(), 3);
    EXPECT_DOUBLE_EQ(filled(0), 4.5);
    EXPECT_DOUBLE_EQ(filled(2), 4.5);
    aesa_processor::math::Vector values{1.0, 2.0, 3.0};
    ASSERT_EQ(values.size(), 3);
    EXPECT_DOUBLE_EQ(values(0), 1.0);
    EXPECT_DOUBLE_EQ(values(2), 3.0);
    aesa_processor::math::Matrix matrix{{4.0}, {5.0}};
    aesa_processor::math::Vector converted(matrix);
    matrix(0, 0) = 9.0;
    EXPECT_DOUBLE_EQ(converted(0), 4.0);
    EXPECT_DOUBLE_EQ(converted(1), 5.0);
    EXPECT_TRUE(aesa_processor::math::Vector(aesa_processor::math::Matrix(0, 1)).empty());
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(VectorTest, CopyAndMovePreserveInvariants)
{
    aesa_processor::math::Vector original{1.0, 2.0};
    aesa_processor::math::Vector copy = original;
    EXPECT_DOUBLE_EQ(copy(0), 1.0);
    copy = aesa_processor::math::Vector{9.0};
    EXPECT_DOUBLE_EQ(copy(0), 9.0);
    EXPECT_DOUBLE_EQ(original(0), 1.0);
    aesa_processor::math::Vector assigned(1);
    assigned = original;
    EXPECT_EQ(assigned.size(), 2);
    EXPECT_DOUBLE_EQ(assigned(1), 2.0);
    aesa_processor::math::Vector moved(std::move(original));
    EXPECT_DOUBLE_EQ(moved(1), 2.0);
    EXPECT_EQ(original.size(), 0);
    EXPECT_EQ(original.asMatrix().columns(), 1);
    EXPECT_DOUBLE_EQ(original.norm(), 0.0);
    EXPECT_THROW(original(0), std::out_of_range);
    assigned = std::move(moved);
    EXPECT_DOUBLE_EQ(assigned(1), 2.0);
    EXPECT_TRUE(moved.empty());
    EXPECT_EQ(moved.asMatrix().columns(), 1);
    assigned = std::move(assigned);
    EXPECT_DOUBLE_EQ(assigned(1), 2.0);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(VectorTest, TransposesToRowMatrix)
{
    auto transposed = aesa_processor::math::transpose(aesa_processor::math::Vector{1.0, 2.0, 3.0});
    static_assert(std::is_same_v<decltype(transposed), aesa_processor::math::Matrix>);
    EXPECT_EQ(transposed.rows(), 1);
    ASSERT_EQ(transposed.columns(), 3);
    EXPECT_DOUBLE_EQ(transposed(0, 2), 3.0);
    auto empty = aesa_processor::math::transpose(aesa_processor::math::Vector());
    EXPECT_EQ(empty.rows(), 1);
    EXPECT_EQ(empty.columns(), 0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(VectorTest, AccessorsReadWriteAndCheckBounds)
{
    aesa_processor::math::Vector vector(3);
    vector.at(0) = 1.0;
    vector(1) = 2.0;
    vector[2] = 3.0;
    EXPECT_DOUBLE_EQ(vector.at(0), 1.0);
    EXPECT_DOUBLE_EQ(vector(1), 2.0);
    EXPECT_DOUBLE_EQ(vector[2], 3.0);
    EXPECT_EQ(vector.size(), 3);
    EXPECT_EQ(vector.asMatrix().columns(), 1);
    EXPECT_THROW(vector.at(3), std::out_of_range);
    EXPECT_THROW(vector(3), std::out_of_range);
    EXPECT_THROW(vector[3], std::out_of_range);
    EXPECT_THROW(aesa_processor::math::Vector().at(0), std::out_of_range);
    const aesa_processor::math::Vector& constant = vector;
    EXPECT_DOUBLE_EQ(constant.at(0), 1.0);
    EXPECT_DOUBLE_EQ(constant(1), 2.0);
    EXPECT_DOUBLE_EQ(constant[2], 3.0);
    EXPECT_THROW(constant.at(3), std::out_of_range);
    EXPECT_THROW(constant(3), std::out_of_range);
    EXPECT_THROW(constant[3], std::out_of_range);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(VectorTest, ArithmeticPreservesTypeAndValues)
{
    aesa_processor::math::Vector left{1.0, -2.0, 3.0};
    aesa_processor::math::Vector right{4.0, 5.0, 6.0};
    auto sum = left + right;
    auto difference = left - right;
    static_assert(std::is_same_v<decltype(sum), aesa_processor::math::Vector>);
    EXPECT_DOUBLE_EQ(sum(0), 5.0);
    EXPECT_DOUBLE_EQ(sum(1), 3.0);
    EXPECT_DOUBLE_EQ(sum(2), 9.0);
    EXPECT_DOUBLE_EQ(difference(0), -3.0);
    EXPECT_DOUBLE_EQ(difference(1), -7.0);
    EXPECT_DOUBLE_EQ(difference(2), -3.0);
    EXPECT_DOUBLE_EQ((left * 2.0)(1), -4.0);
    EXPECT_DOUBLE_EQ((2.0 * left)(2), 6.0);
    EXPECT_DOUBLE_EQ((left / 2.0)(2), 1.5);
    EXPECT_THROW(left + aesa_processor::math::Vector(2), std::invalid_argument);
    EXPECT_THROW(left - aesa_processor::math::Vector(2), std::invalid_argument);
    EXPECT_THROW(left / 0.0, std::domain_error);
    aesa_processor::math::Vector empty;
    EXPECT_TRUE((empty + empty).empty());
    EXPECT_TRUE((empty - empty).empty());
    EXPECT_EQ((empty * 2.0).asMatrix().columns(), 1);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(VectorTest, ComputesDotProductAndStableNorm)
{
    aesa_processor::math::Vector left{1.0, -2.0, 3.0};
    aesa_processor::math::Vector right{4.0, 5.0, 6.0};
    EXPECT_DOUBLE_EQ(left.dot(right), 12.0);
    EXPECT_DOUBLE_EQ(right.dot(left), 12.0);
    EXPECT_DOUBLE_EQ((aesa_processor::math::Vector{3.0, 4.0}).norm(), 5.0);
    EXPECT_DOUBLE_EQ((aesa_processor::math::Vector{-3.0, -4.0}).norm(), 5.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::Vector(3).norm(), 0.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::Vector().dot(aesa_processor::math::Vector()), 0.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::Vector().norm(), 0.0);
    EXPECT_THROW(left.dot(aesa_processor::math::Vector(2)), std::invalid_argument);
    EXPECT_NEAR((aesa_processor::math::Vector{3e200, 4e200}).norm() / 1e200, 5.0, 1e-14);
    EXPECT_NEAR((aesa_processor::math::Vector{3e-200, 4e-200}).norm() / 1e-200, 5.0, 1e-14);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(VectorTest, MultipliesWithMatrices)
{
    aesa_processor::math::Matrix matrix{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    aesa_processor::math::Vector vector{7.0, 8.0, 9.0};
    auto product = matrix * vector;
    static_assert(std::is_same_v<decltype(product), aesa_processor::math::Vector>);
    ASSERT_EQ(product.size(), 2);
    EXPECT_DOUBLE_EQ(product(0), 50.0);
    EXPECT_DOUBLE_EQ(product(1), 122.0);
    aesa_processor::math::SquareMatrix rotation{{0.0, -1.0}, {1.0, 0.0}};
    auto rotated = rotation * aesa_processor::math::Vector{2.0, 3.0};
    static_assert(std::is_same_v<decltype(rotated), aesa_processor::math::Vector>);
    EXPECT_DOUBLE_EQ(rotated(0), -3.0);
    EXPECT_DOUBLE_EQ(rotated(1), 2.0);
    EXPECT_THROW(matrix * aesa_processor::math::Vector(2), std::invalid_argument);
    EXPECT_THROW(rotation * vector, std::invalid_argument);
    auto zero = aesa_processor::math::Matrix(2, 0) * aesa_processor::math::Vector();
    ASSERT_EQ(zero.size(), 2);
    EXPECT_DOUBLE_EQ(zero(0), 0.0);
    EXPECT_DOUBLE_EQ(zero(1), 0.0);
    auto empty = aesa_processor::math::SquareMatrix() * aesa_processor::math::Vector();
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.asMatrix().columns(), 1);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(VectorTest, ChecksWhetherAllElementsAreFinite)
{
    aesa_processor::math::Vector vector{1.0, -2.0, 3.0};
    EXPECT_TRUE(vector.isFinite());
    EXPECT_TRUE(aesa_processor::math::Vector().isFinite());

    vector(1) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(vector.isFinite());
    vector(1) = -std::numeric_limits<double>::infinity();
    EXPECT_FALSE(vector.isFinite());
    vector(1) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(vector.isFinite());
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(VectorTest, RejectsMatricesWithoutExactlyOneColumn)
{
    EXPECT_THROW((aesa_processor::math::Vector(aesa_processor::math::Matrix(1, 3))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::Vector(aesa_processor::math::Matrix(3, 2))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::Vector(aesa_processor::math::Matrix(0, 0))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::Vector(aesa_processor::math::Matrix(0, 2))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::Vector(aesa_processor::math::Matrix(3, 0))), std::invalid_argument);
}
