#include "math/square_matrix.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

static_assert(!std::is_convertible_v<math::Matrix, math::SquareMatrix>);
static_assert(!std::is_assignable_v<math::SquareMatrix&, math::Matrix>);
static_assert(std::is_same_v<decltype(std::declval<math::SquareMatrix&>().asMatrix()), const math::Matrix&>);
// 비상수 객체는 원소를 수정할 수 있다.
static_assert(std::is_assignable_v<decltype(std::declval<math::SquareMatrix&>().at(0, 0)), double>);
static_assert(std::is_assignable_v<decltype(std::declval<math::SquareMatrix&>()(0, 0)), double>);
static_assert(std::is_same_v<decltype(std::declval<const math::SquareMatrix&>().at(0, 0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::SquareMatrix&>()(0, 0)), const double&>);

// 생성 시 형태를 검사하고 명시적으로 일반 행렬을 받아들인다.
TEST(SquareMatrixTest, ConstructsWithSquareShape)
{
    math::SquareMatrix empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.size(), 0);
    EXPECT_EQ(empty.rows(), empty.columns());

    math::SquareMatrix filled(2, 4.5);
    EXPECT_EQ(filled.size(), 2);
    EXPECT_DOUBLE_EQ(filled(0, 1), 4.5);
    EXPECT_DOUBLE_EQ(filled(1, 0), 4.5);

    math::Matrix original{{1.0, 2.0}, {3.0, 4.0}};
    math::SquareMatrix square(original);
    original(0, 0) = 9.0;
    EXPECT_DOUBLE_EQ(square(0, 0), 1.0);
    EXPECT_THROW((math::SquareMatrix(std::numeric_limits<std::size_t>::max())), std::length_error);
}

// 정방행렬이 아닌 입력은 0차원인 경우에도 거부한다.
TEST(SquareMatrixTest, RejectsNonSquareInputs)
{
    EXPECT_THROW((math::SquareMatrix(math::Matrix(2, 3))), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix(math::Matrix(0, 3))), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix(math::Matrix(3, 0))), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix{{1.0, 2.0}}), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix{{1.0, 2.0}, {3.0}}), std::invalid_argument);
}

// 원소 접근자는 범위를 검사하며 같은 저장 원소를 참조한다.
TEST(SquareMatrixTest, AccessorsCheckBounds)
{
    math::SquareMatrix matrix(2);
    matrix.at(1, 0) = 3.0;
    matrix(0, 1) = 4.0;
    EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(matrix.at(0, 1), 4.0);
    EXPECT_EQ(matrix.rows(), 2);
    EXPECT_EQ(matrix.columns(), 2);
    EXPECT_THROW(matrix.at(2, 0), std::out_of_range);
    EXPECT_THROW(matrix(0, 2), std::out_of_range);
    const math::SquareMatrix& constant = matrix;
    EXPECT_DOUBLE_EQ(constant(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(constant.at(0, 1), 4.0);
    EXPECT_THROW(constant.at(2, 0), std::out_of_range);
    EXPECT_THROW(constant(0, 2), std::out_of_range);
}

// 연산 결과가 정방행렬 타입과 올바른 원소 값을 유지한다.
TEST(SquareMatrixTest, ArithmeticPreservesTypeAndValues)
{
    math::SquareMatrix left{{1.0, 2.0}, {3.0, 4.0}};
    math::SquareMatrix right{{5.0, 6.0}, {7.0, 8.0}};
    auto sum = left + right;
    auto difference = left - right;
    auto product = left * right;
    auto transposed = left.transpose();
    static_assert(std::is_same_v<decltype(sum), math::SquareMatrix>);
    static_assert(std::is_same_v<decltype(product), math::SquareMatrix>);
    static_assert(std::is_same_v<decltype(transposed), math::SquareMatrix>);
    EXPECT_DOUBLE_EQ(sum(0, 1), 8.0);
    EXPECT_DOUBLE_EQ(difference(1, 0), -4.0);
    EXPECT_DOUBLE_EQ(product(0, 0), 19.0);
    EXPECT_DOUBLE_EQ(product(0, 1), 22.0);
    EXPECT_DOUBLE_EQ(product(1, 0), 43.0);
    EXPECT_DOUBLE_EQ(product(1, 1), 50.0);
    EXPECT_DOUBLE_EQ(transposed(0, 1), 3.0);
    EXPECT_DOUBLE_EQ(transposed(1, 0), 2.0);
    EXPECT_DOUBLE_EQ((left * 2.0)(1, 1), 8.0);
    EXPECT_DOUBLE_EQ((2.0 * left)(0, 1), 4.0);
    EXPECT_DOUBLE_EQ((left / 2.0)(1, 0), 1.5);
    EXPECT_THROW(left / 0.0, std::domain_error);
    EXPECT_THROW(left + math::SquareMatrix(3), std::invalid_argument);
    EXPECT_THROW(left - math::SquareMatrix(3), std::invalid_argument);
    EXPECT_THROW(left * math::SquareMatrix(3), std::invalid_argument);

    math::SquareMatrix empty;
    EXPECT_TRUE((empty + empty).empty());
    EXPECT_TRUE((empty - empty).empty());
    EXPECT_TRUE((empty * empty).empty());
    EXPECT_TRUE(empty.transpose().empty());
}

// 복사는 독립적이며 이동한 원본도 유효한 빈 정방행렬로 사용할 수 있다.
TEST(SquareMatrixTest, CopyAndMovePreserveInvariants)
{
    math::SquareMatrix original{{1.0, 2.0}, {3.0, 4.0}};
    math::SquareMatrix copy = original;
    EXPECT_DOUBLE_EQ(copy(0, 0), 1.0);
    copy = math::SquareMatrix{{9.0}};
    EXPECT_DOUBLE_EQ(copy(0, 0), 9.0);
    EXPECT_DOUBLE_EQ(original(0, 0), 1.0);
    math::SquareMatrix assigned(1);
    assigned = original;
    EXPECT_EQ(assigned.size(), 2);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);

    math::SquareMatrix moved(std::move(original));
    EXPECT_DOUBLE_EQ(moved(1, 0), 3.0);
    EXPECT_EQ(original.size(), 0);
    EXPECT_EQ(original.columns(), 0);
    EXPECT_DOUBLE_EQ(original.det(), 1.0);
    EXPECT_THROW(original(0, 0), std::out_of_range);
    assigned = std::move(moved);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);
    EXPECT_TRUE(moved.empty());
    assigned = std::move(assigned);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);
}

// 단위행렬의 대각 원소는 1, 나머지는 0이며 크기가 0인 경우도 지원한다.
TEST(SquareMatrixTest, CreatesIdentityMatrices)
{
    math::SquareMatrix identity = math::SquareMatrix::identity(3);

    ASSERT_EQ(identity.rows(), 3);
    ASSERT_EQ(identity.columns(), 3);
    for (std::size_t row = 0; row < identity.rows(); ++row)
    {
        for (std::size_t column = 0; column < identity.columns(); ++column)
        {
            if (row == column)
            {
                EXPECT_DOUBLE_EQ(identity(row, column), 1.0);
            }
            else
            {
                EXPECT_DOUBLE_EQ(identity(row, column), 0.0);
            }
        }
    }

    // 크기가 0인 단위행렬도 올바른 빈 정방행렬이어야 한다.
    math::SquareMatrix emptyIdentity = math::SquareMatrix::identity(0);
    EXPECT_TRUE(emptyIdentity.empty());
    EXPECT_EQ(emptyIdentity.rows(), 0);
    EXPECT_EQ(emptyIdentity.columns(), 0);
}

// 크기가 다른 정방행렬과 단위행렬의 행렬식을 알려진 결과와 비교한다.
TEST(SquareMatrixTest, ComputesDeterminantsOfSquareMatrices)
{
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{-3.5}}).det(), -3.5);
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{1.0, 2.0}, {3.0, 4.0}}).det(), -2.0);
    EXPECT_NEAR((math::SquareMatrix{{6.0, 1.0, 1.0}, {4.0, -2.0, 5.0}, {2.0, 8.0, 7.0}}).det(), -306.0, 1e-12);
    EXPECT_DOUBLE_EQ(math::SquareMatrix::identity(4).det(), 1.0);
}

// 행 교환 횟수에 따라 행렬식 부호를 계산하며 원본 행렬은 보존한다.
TEST(SquareMatrixTest, DeterminantAccountsForRowSwaps)
{
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{0.0, 1.0}, {2.0, 3.0}}).det(), -2.0);
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}}).det(), 1.0);

    // 행 교환과 소거를 수행해도 원본 행렬의 원소는 유지한다.
    math::SquareMatrix matrix{{0.0, 2.0}, {3.0, 4.0}};

    EXPECT_DOUBLE_EQ(matrix.det(), -6.0);
    EXPECT_DOUBLE_EQ(matrix(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(matrix(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
}

// 소거 후 또는 처음부터 피벗이 0인 특이행렬의 행렬식은 0이다.
TEST(SquareMatrixTest, SingularMatricesHaveZeroDeterminant)
{
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{1.0, 2.0}, {2.0, 4.0}}).det(), 0.0);
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{0.0, 1.0}, {0.0, 2.0}}).det(), 0.0);
    EXPECT_DOUBLE_EQ(math::SquareMatrix(3).det(), 0.0);
}

// 매우 작은 피벗도 0으로 간주하지 않고 0이 아닌 행렬식을 보존한다.
TEST(SquareMatrixTest, DeterminantPreservesSmallNonzeroPivots)
{
    EXPECT_DOUBLE_EQ((math::SquareMatrix{{1e-20, 0.0}, {0.0, 2.0}}).det(), 2e-20);
}

// 0×0 행렬의 행렬식은 생성 방법에 관계없이 정의에 따라 1이다.
TEST(SquareMatrixTest, EmptySquareMatrixHasUnitDeterminant)
{
    EXPECT_DOUBLE_EQ(math::SquareMatrix().det(), 1.0);
    EXPECT_DOUBLE_EQ(math::SquareMatrix::identity(0).det(), 1.0);
}
