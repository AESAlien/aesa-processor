#include "math/linear_algebra/matrix.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<decltype(std::declval<math::Matrix&>().at(0, 0)), double&>);
static_assert(std::is_same_v<decltype(std::declval<math::Matrix&>()(0, 0)), double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::Matrix&>().at(0, 0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::Matrix&>()(0, 0)), const double&>);



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(MatrixTest, DefaultAndZeroDimensionMatricesAreEmpty)
{
    math::Matrix defaultMatrix;
    math::Matrix zeroRows(0, 3);
    math::Matrix zeroColumns(3, 0);

    EXPECT_TRUE(defaultMatrix.empty());
    EXPECT_EQ(defaultMatrix.rows(), 0);
    EXPECT_EQ(defaultMatrix.columns(), 0);
    EXPECT_TRUE(zeroRows.empty());
    EXPECT_EQ(zeroRows.rows(), 0);
    EXPECT_EQ(zeroRows.columns(), 3);
    EXPECT_TRUE(zeroColumns.empty());
    EXPECT_EQ(zeroColumns.rows(), 3);
    EXPECT_EQ(zeroColumns.columns(), 0);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(MatrixTest, ConstructsAndInitializesRequestedDimensions)
{
    math::Matrix matrix(2, 3, 4.5);

    EXPECT_FALSE(matrix.empty());
    EXPECT_EQ(matrix.rows(), 2);
    EXPECT_EQ(matrix.columns(), 3);
    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(matrix(row, column), 4.5);
        }
    }
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(MatrixTest, ConstructsFromRectangularInitializerLists)
{
    math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};

    ASSERT_EQ(matrix.rows(), 3);
    ASSERT_EQ(matrix.columns(), 2);
    EXPECT_DOUBLE_EQ(matrix(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
    EXPECT_DOUBLE_EQ(matrix(2, 0), 5.0);
    EXPECT_DOUBLE_EQ(matrix(2, 1), 6.0);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(MatrixTest, CopyAndMovePreserveInvariants)
{
    math::Matrix original{{1.0, 2.0}, {3.0, 4.0}};
    math::Matrix copy(original);
    copy(0, 0) = 9.0;
    EXPECT_DOUBLE_EQ(original(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(copy(0, 0), 9.0);

    math::Matrix assigned(1, 1);
    assigned = original;
    EXPECT_EQ(assigned.rows(), 2);
    EXPECT_EQ(assigned.columns(), 2);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);

    math::Matrix moved(std::move(original));
    EXPECT_DOUBLE_EQ(moved(1, 0), 3.0);
    EXPECT_TRUE(original.empty());
    EXPECT_EQ(original.rows(), 0);
    EXPECT_EQ(original.columns(), 0);
    EXPECT_THROW(original(0, 0), std::out_of_range);

    assigned = std::move(moved);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);
    EXPECT_TRUE(moved.empty());
    EXPECT_EQ(moved.rows(), 0);
    EXPECT_EQ(moved.columns(), 0);

    assigned = std::move(assigned);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);
}

// 값의 변환과 정규화 결과를 검증한다.
TEST(MatrixTest, TransposesRectangularMatrices)
{
    math::Matrix matrix{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    math::Matrix transposed = math::transpose(matrix);

    ASSERT_EQ(transposed.rows(), 3);
    ASSERT_EQ(transposed.columns(), 2);
    EXPECT_DOUBLE_EQ(transposed(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(transposed(0, 1), 4.0);
    EXPECT_DOUBLE_EQ(transposed(1, 0), 2.0);
    EXPECT_DOUBLE_EQ(transposed(1, 1), 5.0);
    EXPECT_DOUBLE_EQ(transposed(2, 0), 3.0);
    EXPECT_DOUBLE_EQ(transposed(2, 1), 6.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixTest, AccessorsReadAndWriteElements)
{
    math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}};
    matrix(0, 1) = 5.0;
    matrix.at(1, 0) = 6.0;

    EXPECT_DOUBLE_EQ(matrix(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(matrix.at(1, 0), 6.0);
    const math::Matrix& constant = matrix;
    EXPECT_DOUBLE_EQ(constant(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(constant.at(1, 0), 6.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixTest, AddsAndSubtractsMatricesElementwise)
{
    math::Matrix left{{1.0, 2.0}, {3.0, 4.0}};
    math::Matrix right{{5.0, 6.0}, {7.0, 8.0}};
    math::Matrix sum = left + right;
    math::Matrix difference = left - right;

    EXPECT_DOUBLE_EQ(sum(0, 0), 6.0);
    EXPECT_DOUBLE_EQ(sum(0, 1), 8.0);
    EXPECT_DOUBLE_EQ(sum(1, 0), 10.0);
    EXPECT_DOUBLE_EQ(sum(1, 1), 12.0);
    EXPECT_DOUBLE_EQ(difference(0, 0), -4.0);
    EXPECT_DOUBLE_EQ(difference(0, 1), -4.0);
    EXPECT_DOUBLE_EQ(difference(1, 0), -4.0);
    EXPECT_DOUBLE_EQ(difference(1, 1), -4.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixTest, MultipliesRectangularMatrices)
{
    math::Matrix left{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    math::Matrix right{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}};
    math::Matrix product = left * right;

    ASSERT_EQ(product.rows(), 2);
    ASSERT_EQ(product.columns(), 2);
    EXPECT_DOUBLE_EQ(product(0, 0), 58.0);
    EXPECT_DOUBLE_EQ(product(0, 1), 64.0);
    EXPECT_DOUBLE_EQ(product(1, 0), 139.0);
    EXPECT_DOUBLE_EQ(product(1, 1), 154.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixTest, MultipliesCompatibleZeroDimensionMatrices)
{
    math::Matrix product = math::Matrix(2, 0) * math::Matrix(0, 3);

    ASSERT_EQ(product.rows(), 2);
    ASSERT_EQ(product.columns(), 3);
    for (std::size_t row = 0; row < product.rows(); ++row)
    {
        for (std::size_t column = 0; column < product.columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(product(row, column), 0.0);
        }
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixTest, MultipliesByScalarsFromEitherSide)
{
    math::Matrix matrix{{1.0, -2.0}, {3.5, 4.0}};
    math::Matrix rightProduct = matrix * 2.0;
    math::Matrix leftProduct = 2.0 * matrix;

    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(rightProduct(row, column), 2.0 * matrix(row, column));
            EXPECT_DOUBLE_EQ(leftProduct(row, column), 2.0 * matrix(row, column));
        }
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(MatrixTest, DividesByScalar)
{
    math::Matrix matrix{{2.0, -4.0}, {7.0, 0.0}};
    math::Matrix quotient = matrix / 2.0;

    EXPECT_DOUBLE_EQ(quotient(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(quotient(0, 1), -2.0);
    EXPECT_DOUBLE_EQ(quotient(1, 0), 3.5);
    EXPECT_DOUBLE_EQ(quotient(1, 1), 0.0);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(MatrixTest, ChecksWhetherAllElementsAreFinite)
{
    math::Matrix matrix{{1.0, -2.0}, {3.0, 4.0}};
    EXPECT_TRUE(matrix.isFinite());
    EXPECT_TRUE(math::Matrix().isFinite());

    matrix(0, 1) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(matrix.isFinite());
    matrix(0, 1) = -std::numeric_limits<double>::infinity();
    EXPECT_FALSE(matrix.isFinite());
    matrix(0, 1) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(matrix.isFinite());
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(MatrixTest, RejectsDimensionsThatOverflowStorageSize)
{
    EXPECT_THROW((math::Matrix(std::numeric_limits<std::size_t>::max(), 2)), std::length_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(MatrixTest, RejectsInitializerListsWithDifferentRowSizes)
{
    EXPECT_THROW((math::Matrix{{1.0, 2.0}, {3.0}}), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(MatrixTest, RejectsOutOfRangeAccess)
{
    math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}};

    EXPECT_THROW(matrix(2, 0), std::out_of_range);
    EXPECT_THROW(matrix(0, 2), std::out_of_range);
    EXPECT_THROW(matrix.at(2, 0), std::out_of_range);
    EXPECT_THROW(matrix.at(0, 2), std::out_of_range);
    const math::Matrix& constant = matrix;
    EXPECT_THROW(constant(2, 0), std::out_of_range);
    EXPECT_THROW(constant(0, 2), std::out_of_range);
    EXPECT_THROW(constant.at(2, 0), std::out_of_range);
    EXPECT_THROW(constant.at(0, 2), std::out_of_range);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(MatrixTest, RejectsAdditionAndSubtractionOfDifferentDimensions)
{
    math::Matrix matrix2x2{{1.0, 2.0}, {3.0, 4.0}};
    math::Matrix matrix2x1{{1.0}, {2.0}};

    EXPECT_THROW(matrix2x2 + matrix2x1, std::invalid_argument);
    EXPECT_THROW(matrix2x2 - matrix2x1, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(MatrixTest, RejectsMultiplicationOfIncompatibleDimensions)
{
    math::Matrix left(2, 3);
    math::Matrix right(2, 2);

    EXPECT_THROW(left * right, std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(MatrixTest, RejectsDivisionByZero)
{
    math::Matrix matrix{{1.0, 2.0}};

    EXPECT_THROW(matrix / 0.0, std::domain_error);
}
