#include "math/matrix.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <stdexcept>

TEST(MatrixTest, DefaultAndZeroDimensionMatricesAreEmpty)
{
    const math::Matrix defaultMatrix;
    const math::Matrix zeroRows(0, 3);
    const math::Matrix zeroColumns(3, 0);

    EXPECT_TRUE(defaultMatrix.Empty());
    EXPECT_EQ(defaultMatrix.Rows(), 0);
    EXPECT_EQ(defaultMatrix.Columns(), 0);
    EXPECT_TRUE(zeroRows.Empty());
    EXPECT_EQ(zeroRows.Rows(), 0);
    EXPECT_EQ(zeroRows.Columns(), 3);
    EXPECT_TRUE(zeroColumns.Empty());
    EXPECT_EQ(zeroColumns.Rows(), 3);
    EXPECT_EQ(zeroColumns.Columns(), 0);
}

TEST(MatrixTest, ConstructsAndInitializesRequestedDimensions)
{
    const math::Matrix matrix(2, 3, 4.5);

    EXPECT_FALSE(matrix.Empty());
    EXPECT_EQ(matrix.Rows(), 2);
    EXPECT_EQ(matrix.Columns(), 3);
    for (std::size_t row = 0; row < matrix.Rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.Columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(matrix(row, column), 4.5);
        }
    }
}

TEST(MatrixTest, RejectsDimensionsThatOverflowStorageSize)
{
    EXPECT_THROW((math::Matrix(std::numeric_limits<std::size_t>::max(), 2)), std::length_error);
}

TEST(MatrixTest, ConstructsFromRectangularInitializerLists)
{
    const math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};

    ASSERT_EQ(matrix.Rows(), 3);
    ASSERT_EQ(matrix.Columns(), 2);
    EXPECT_DOUBLE_EQ(matrix(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
    EXPECT_DOUBLE_EQ(matrix(2, 0), 5.0);
    EXPECT_DOUBLE_EQ(matrix(2, 1), 6.0);
}

TEST(MatrixTest, RejectsInitializerListsWithDifferentRowSizes)
{
    EXPECT_THROW((math::Matrix{{1.0, 2.0}, {3.0}}), std::invalid_argument);
}

TEST(MatrixTest, AccessorsReadAndWriteElements)
{
    math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}};
    matrix(0, 1) = 5.0;
    matrix.At(1, 0) = 6.0;

    EXPECT_DOUBLE_EQ(matrix(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(matrix.At(1, 0), 6.0);

    const math::Matrix& constMatrix = matrix;
    EXPECT_DOUBLE_EQ(constMatrix(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(constMatrix.At(1, 0), 6.0);
}

TEST(MatrixTest, RejectsOutOfRangeAccess)
{
    math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}};
    const math::Matrix& constMatrix = matrix;

    EXPECT_THROW(matrix(2, 0), std::out_of_range);
    EXPECT_THROW(matrix(0, 2), std::out_of_range);
    EXPECT_THROW(matrix.At(2, 0), std::out_of_range);
    EXPECT_THROW(matrix.At(0, 2), std::out_of_range);
    EXPECT_THROW(constMatrix(2, 0), std::out_of_range);
    EXPECT_THROW(constMatrix.At(0, 2), std::out_of_range);
}

TEST(MatrixTest, AddsAndSubtractsMatricesElementwise)
{
    const math::Matrix left{{1.0, 2.0}, {3.0, 4.0}};
    const math::Matrix right{{5.0, 6.0}, {7.0, 8.0}};
    const math::Matrix sum = left + right;
    const math::Matrix difference = left - right;

    EXPECT_DOUBLE_EQ(sum(0, 0), 6.0);
    EXPECT_DOUBLE_EQ(sum(0, 1), 8.0);
    EXPECT_DOUBLE_EQ(sum(1, 0), 10.0);
    EXPECT_DOUBLE_EQ(sum(1, 1), 12.0);
    EXPECT_DOUBLE_EQ(difference(0, 0), -4.0);
    EXPECT_DOUBLE_EQ(difference(0, 1), -4.0);
    EXPECT_DOUBLE_EQ(difference(1, 0), -4.0);
    EXPECT_DOUBLE_EQ(difference(1, 1), -4.0);
}

TEST(MatrixTest, RejectsAdditionAndSubtractionOfDifferentDimensions)
{
    const math::Matrix matrix2x2{{1.0, 2.0}, {3.0, 4.0}};
    const math::Matrix matrix2x1{{1.0}, {2.0}};

    EXPECT_THROW(matrix2x2 + matrix2x1, std::invalid_argument);
    EXPECT_THROW(matrix2x2 - matrix2x1, std::invalid_argument);
}

TEST(MatrixTest, MultipliesRectangularMatrices)
{
    const math::Matrix left{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const math::Matrix right{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}};
    const math::Matrix product = left * right;

    ASSERT_EQ(product.Rows(), 2);
    ASSERT_EQ(product.Columns(), 2);
    EXPECT_DOUBLE_EQ(product(0, 0), 58.0);
    EXPECT_DOUBLE_EQ(product(0, 1), 64.0);
    EXPECT_DOUBLE_EQ(product(1, 0), 139.0);
    EXPECT_DOUBLE_EQ(product(1, 1), 154.0);
}

TEST(MatrixTest, MultipliesCompatibleZeroDimensionMatrices)
{
    const math::Matrix product = math::Matrix(2, 0) * math::Matrix(0, 3);

    ASSERT_EQ(product.Rows(), 2);
    ASSERT_EQ(product.Columns(), 3);
    for (std::size_t row = 0; row < product.Rows(); ++row)
    {
        for (std::size_t column = 0; column < product.Columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(product(row, column), 0.0);
        }
    }
}

TEST(MatrixTest, RejectsMultiplicationOfIncompatibleDimensions)
{
    const math::Matrix left(2, 3);
    const math::Matrix right(2, 2);

    EXPECT_THROW(left * right, std::invalid_argument);
}

TEST(MatrixTest, MultipliesByScalarsFromEitherSide)
{
    const math::Matrix matrix{{1.0, -2.0}, {3.5, 4.0}};
    const math::Matrix rightProduct = matrix * 2.0;
    const math::Matrix leftProduct = 2.0 * matrix;

    for (std::size_t row = 0; row < matrix.Rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.Columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(rightProduct(row, column), 2.0 * matrix(row, column));
            EXPECT_DOUBLE_EQ(leftProduct(row, column), 2.0 * matrix(row, column));
        }
    }
}

TEST(MatrixTest, DividesByScalar)
{
    const math::Matrix matrix{{2.0, -4.0}, {7.0, 0.0}};
    const math::Matrix quotient = matrix / 2.0;

    EXPECT_DOUBLE_EQ(quotient(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(quotient(0, 1), -2.0);
    EXPECT_DOUBLE_EQ(quotient(1, 0), 3.5);
    EXPECT_DOUBLE_EQ(quotient(1, 1), 0.0);
}

TEST(MatrixTest, RejectsDivisionByZero)
{
    const math::Matrix matrix{{1.0, 2.0}};

    EXPECT_THROW(matrix / 0.0, std::domain_error);
}

TEST(MatrixTest, TransposesRectangularMatrices)
{
    const math::Matrix matrix{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const math::Matrix transposed = matrix.Transpose();

    ASSERT_EQ(transposed.Rows(), 3);
    ASSERT_EQ(transposed.Columns(), 2);
    EXPECT_DOUBLE_EQ(transposed(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(transposed(0, 1), 4.0);
    EXPECT_DOUBLE_EQ(transposed(1, 0), 2.0);
    EXPECT_DOUBLE_EQ(transposed(1, 1), 5.0);
    EXPECT_DOUBLE_EQ(transposed(2, 0), 3.0);
    EXPECT_DOUBLE_EQ(transposed(2, 1), 6.0);
}

TEST(MatrixTest, CreatesIdentityMatrices)
{
    const math::Matrix identity = math::Matrix::Identity(3);

    ASSERT_EQ(identity.Rows(), 3);
    ASSERT_EQ(identity.Columns(), 3);
    for (std::size_t row = 0; row < identity.Rows(); ++row)
    {
        for (std::size_t column = 0; column < identity.Columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(identity(row, column), row == column ? 1.0 : 0.0);
        }
    }
}

TEST(MatrixTest, CreatesEmptyIdentityMatrix)
{
    const math::Matrix identity = math::Matrix::Identity(0);

    EXPECT_TRUE(identity.Empty());
    EXPECT_EQ(identity.Rows(), 0);
    EXPECT_EQ(identity.Columns(), 0);
}
