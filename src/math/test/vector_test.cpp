#include "math/vector.hpp"
#include "math/square_matrix.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

static_assert(!std::is_convertible_v<math::Matrix, math::Vector>);
static_assert(!std::is_assignable_v<math::Vector&, math::Matrix>);
static_assert(std::is_same_v<decltype(std::declval<math::Vector&>().asMatrix()), const math::Matrix&>);
// 비상수 객체는 원소를 수정할 수 있다.
static_assert(std::is_assignable_v<decltype(std::declval<math::Vector&>().at(0)), double>);
static_assert(std::is_assignable_v<decltype(std::declval<math::Vector&>()(0)), double>);
static_assert(std::is_assignable_v<decltype(std::declval<math::Vector&>()[0]), double>);
static_assert(std::is_same_v<decltype(std::declval<const math::Vector&>().at(0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::Vector&>()(0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::Vector&>()[0]), const double&>);

// 빈 벡터도 열 수 1을 유지하며 크기 생성과 평탄한 초기화 목록을 지원한다.
TEST(VectorTest, ConstructsColumnVectors)
{
    math::Vector empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.size(), 0);
    EXPECT_EQ(empty.asMatrix().rows(), 0);
    EXPECT_EQ(empty.asMatrix().columns(), 1);
    math::Vector filled(3, 4.5);
    ASSERT_EQ(filled.size(), 3);
    EXPECT_DOUBLE_EQ(filled(0), 4.5);
    EXPECT_DOUBLE_EQ(filled(2), 4.5);
    math::Vector values{1.0, 2.0, 3.0};
    ASSERT_EQ(values.size(), 3);
    EXPECT_DOUBLE_EQ(values(0), 1.0);
    EXPECT_DOUBLE_EQ(values(2), 3.0);
    math::Matrix matrix{{4.0}, {5.0}};
    math::Vector converted(matrix);
    matrix(0, 0) = 9.0;
    EXPECT_DOUBLE_EQ(converted(0), 4.0);
    EXPECT_DOUBLE_EQ(converted(1), 5.0);
    EXPECT_TRUE(math::Vector(math::Matrix(0, 1)).empty());
}

// 행벡터·다중 열·0×0 행렬은 열벡터 생성 시 거부한다.
TEST(VectorTest, RejectsMatricesWithoutExactlyOneColumn)
{
    EXPECT_THROW((math::Vector(math::Matrix(1, 3))), std::invalid_argument);
    EXPECT_THROW((math::Vector(math::Matrix(3, 2))), std::invalid_argument);
    EXPECT_THROW((math::Vector(math::Matrix(0, 0))), std::invalid_argument);
    EXPECT_THROW((math::Vector(math::Matrix(0, 2))), std::invalid_argument);
    EXPECT_THROW((math::Vector(math::Matrix(3, 0))), std::invalid_argument);
}

// 인덱스 접근자로 원소를 수정하고 범위 밖 접근은 거부한다.
TEST(VectorTest, AccessorsReadWriteAndCheckBounds)
{
    math::Vector vector(3);
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
    EXPECT_THROW(math::Vector().at(0), std::out_of_range);
    const math::Vector& constant = vector;
    EXPECT_DOUBLE_EQ(constant.at(0), 1.0);
    EXPECT_DOUBLE_EQ(constant(1), 2.0);
    EXPECT_DOUBLE_EQ(constant[2], 3.0);
    EXPECT_THROW(constant.at(3), std::out_of_range);
    EXPECT_THROW(constant(3), std::out_of_range);
    EXPECT_THROW(constant[3], std::out_of_range);
}

// 벡터 산술은 결과 타입을 보존하며 길이 불일치와 0 나눗셈을 거부한다.
TEST(VectorTest, ArithmeticPreservesTypeAndValues)
{
    math::Vector left{1.0, -2.0, 3.0};
    math::Vector right{4.0, 5.0, 6.0};
    auto sum = left + right;
    auto difference = left - right;
    static_assert(std::is_same_v<decltype(sum), math::Vector>);
    EXPECT_DOUBLE_EQ(sum(0), 5.0);
    EXPECT_DOUBLE_EQ(sum(1), 3.0);
    EXPECT_DOUBLE_EQ(sum(2), 9.0);
    EXPECT_DOUBLE_EQ(difference(0), -3.0);
    EXPECT_DOUBLE_EQ(difference(1), -7.0);
    EXPECT_DOUBLE_EQ(difference(2), -3.0);
    EXPECT_DOUBLE_EQ((left * 2.0)(1), -4.0);
    EXPECT_DOUBLE_EQ((2.0 * left)(2), 6.0);
    EXPECT_DOUBLE_EQ((left / 2.0)(2), 1.5);
    EXPECT_THROW(left + math::Vector(2), std::invalid_argument);
    EXPECT_THROW(left - math::Vector(2), std::invalid_argument);
    EXPECT_THROW(left / 0.0, std::domain_error);
    math::Vector empty;
    EXPECT_TRUE((empty + empty).empty());
    EXPECT_TRUE((empty - empty).empty());
    EXPECT_EQ((empty * 2.0).asMatrix().columns(), 1);
}

// 내적과 노름을 알려진 결과와 비교하고 큰 값·작은 값에서도 노름을 보존한다.
TEST(VectorTest, ComputesDotProductAndStableNorm)
{
    math::Vector left{1.0, -2.0, 3.0};
    math::Vector right{4.0, 5.0, 6.0};
    EXPECT_DOUBLE_EQ(left.dot(right), 12.0);
    EXPECT_DOUBLE_EQ(right.dot(left), 12.0);
    EXPECT_DOUBLE_EQ((math::Vector{3.0, 4.0}).norm(), 5.0);
    EXPECT_DOUBLE_EQ((math::Vector{-3.0, -4.0}).norm(), 5.0);
    EXPECT_DOUBLE_EQ(math::Vector(3).norm(), 0.0);
    EXPECT_DOUBLE_EQ(math::Vector().dot(math::Vector()), 0.0);
    EXPECT_DOUBLE_EQ(math::Vector().norm(), 0.0);
    EXPECT_THROW(left.dot(math::Vector(2)), std::invalid_argument);
    EXPECT_NEAR((math::Vector{3e200, 4e200}).norm() / 1e200, 5.0, 1e-14);
    EXPECT_NEAR((math::Vector{3e-200, 4e-200}).norm() / 1e-200, 5.0, 1e-14);
}

// 벡터 전치는 행벡터 형태의 일반 행렬을 반환한다.
TEST(VectorTest, TransposesToRowMatrix)
{
    auto transposed = math::Vector{1.0, 2.0, 3.0}.transpose();
    static_assert(std::is_same_v<decltype(transposed), math::Matrix>);
    EXPECT_EQ(transposed.rows(), 1);
    ASSERT_EQ(transposed.columns(), 3);
    EXPECT_DOUBLE_EQ(transposed(0, 2), 3.0);
    auto empty = math::Vector().transpose();
    EXPECT_EQ(empty.rows(), 1);
    EXPECT_EQ(empty.columns(), 0);
}

// 일반 행렬·정방행렬과의 곱은 올바른 크기와 값의 벡터를 반환한다.
TEST(VectorTest, MultipliesWithMatrices)
{
    math::Matrix matrix{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    math::Vector vector{7.0, 8.0, 9.0};
    auto product = matrix * vector;
    static_assert(std::is_same_v<decltype(product), math::Vector>);
    ASSERT_EQ(product.size(), 2);
    EXPECT_DOUBLE_EQ(product(0), 50.0);
    EXPECT_DOUBLE_EQ(product(1), 122.0);
    math::SquareMatrix rotation{{0.0, -1.0}, {1.0, 0.0}};
    auto rotated = rotation * math::Vector{2.0, 3.0};
    static_assert(std::is_same_v<decltype(rotated), math::Vector>);
    EXPECT_DOUBLE_EQ(rotated(0), -3.0);
    EXPECT_DOUBLE_EQ(rotated(1), 2.0);
    EXPECT_THROW(matrix * math::Vector(2), std::invalid_argument);
    EXPECT_THROW(rotation * vector, std::invalid_argument);
    auto zero = math::Matrix(2, 0) * math::Vector();
    ASSERT_EQ(zero.size(), 2);
    EXPECT_DOUBLE_EQ(zero(0), 0.0);
    EXPECT_DOUBLE_EQ(zero(1), 0.0);
    auto empty = math::SquareMatrix() * math::Vector();
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.asMatrix().columns(), 1);
}

// 복사는 독립적이며 이동한 원본도 유효한 빈 열벡터로 사용할 수 있다.
TEST(VectorTest, CopyAndMovePreserveInvariants)
{
    math::Vector original{1.0, 2.0};
    math::Vector copy = original;
    EXPECT_DOUBLE_EQ(copy(0), 1.0);
    copy = math::Vector{9.0};
    EXPECT_DOUBLE_EQ(copy(0), 9.0);
    EXPECT_DOUBLE_EQ(original(0), 1.0);
    math::Vector assigned(1);
    assigned = original;
    EXPECT_EQ(assigned.size(), 2);
    EXPECT_DOUBLE_EQ(assigned(1), 2.0);
    math::Vector moved(std::move(original));
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
