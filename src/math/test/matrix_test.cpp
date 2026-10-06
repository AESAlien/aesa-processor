#include "math/matrix.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<decltype(std::declval<math::Matrix&>().at(0, 0)), double&>);
static_assert(std::is_same_v<decltype(std::declval<math::Matrix&>()(0, 0)), double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::Matrix&>().at(0, 0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const math::Matrix&>()(0, 0)), const double&>);

// 기본 행렬과 행 또는 열이 0인 행렬은 비어 있으며 지정한 차원은 보존한다.
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

// 지정한 행·열 크기로 행렬을 생성하고 모든 원소를 초기값으로 채운다.
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

// 행과 열의 곱이 저장 크기 범위를 넘으면 할당 전에 예외를 발생시킨다.
TEST(MatrixTest, RejectsDimensionsThatOverflowStorageSize)
{
    EXPECT_THROW((math::Matrix(std::numeric_limits<std::size_t>::max(), 2)), std::length_error);
}

// 직사각형 초기화 목록의 행·열 크기와 원소 배치를 검증한다.
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

// 행마다 원소 수가 다른 초기화 목록은 예외로 거부한다.
TEST(MatrixTest, RejectsInitializerListsWithDifferentRowSizes)
{
    EXPECT_THROW((math::Matrix{{1.0, 2.0}, {3.0}}), std::invalid_argument);
}

// 괄호 연산자와 at으로 원소를 수정하고 접근자로 같은 값을 읽는다.
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

// 행 또는 열 인덱스가 범위를 벗어나면 접근자가 예외를 발생시킨다.
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

// 같은 크기의 행렬 덧셈과 뺄셈을 각 원소에 적용한다.
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

// 차원이 다른 행렬끼리의 덧셈과 뺄셈은 예외로 거부한다.
TEST(MatrixTest, RejectsAdditionAndSubtractionOfDifferentDimensions)
{
    math::Matrix matrix2x2{{1.0, 2.0}, {3.0, 4.0}};
    math::Matrix matrix2x1{{1.0}, {2.0}};

    EXPECT_THROW(matrix2x2 + matrix2x1, std::invalid_argument);
    EXPECT_THROW(matrix2x2 - matrix2x1, std::invalid_argument);
}

// 직사각형 행렬 곱의 결과 차원과 각 행·열 내적을 검증한다.
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

// 내부 차원이 0인 행렬 곱도 결과 차원을 보존하고 모든 원소를 0으로 만든다.
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

// 왼쪽 열 수와 오른쪽 행 수가 다르면 행렬 곱을 예외로 거부한다.
TEST(MatrixTest, RejectsMultiplicationOfIncompatibleDimensions)
{
    math::Matrix left(2, 3);
    math::Matrix right(2, 2);

    EXPECT_THROW(left * right, std::invalid_argument);
}

// 스칼라가 행렬의 왼쪽 또는 오른쪽에 있어도 모든 원소에 같은 배율을 적용한다.
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

// 행렬의 양수·음수·0 원소를 스칼라로 올바르게 나눈다.
TEST(MatrixTest, DividesByScalar)
{
    math::Matrix matrix{{2.0, -4.0}, {7.0, 0.0}};
    math::Matrix quotient = matrix / 2.0;

    EXPECT_DOUBLE_EQ(quotient(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(quotient(0, 1), -2.0);
    EXPECT_DOUBLE_EQ(quotient(1, 0), 3.5);
    EXPECT_DOUBLE_EQ(quotient(1, 1), 0.0);
}

// 행렬을 0으로 나누면 예외를 발생시킨다.
TEST(MatrixTest, RejectsDivisionByZero)
{
    math::Matrix matrix{{1.0, 2.0}};

    EXPECT_THROW(matrix / 0.0, std::domain_error);
}

// 직사각형 행렬 전치 시 차원을 교환하고 각 원소의 행·열 위치를 바꾼다.
TEST(MatrixTest, TransposesRectangularMatrices)
{
    math::Matrix matrix{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    math::Matrix transposed = matrix.transpose();

    ASSERT_EQ(transposed.rows(), 3);
    ASSERT_EQ(transposed.columns(), 2);
    EXPECT_DOUBLE_EQ(transposed(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(transposed(0, 1), 4.0);
    EXPECT_DOUBLE_EQ(transposed(1, 0), 2.0);
    EXPECT_DOUBLE_EQ(transposed(1, 1), 5.0);
    EXPECT_DOUBLE_EQ(transposed(2, 0), 3.0);
    EXPECT_DOUBLE_EQ(transposed(2, 1), 6.0);
}
