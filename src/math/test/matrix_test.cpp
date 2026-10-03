#include "math/matrix.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>

TEST(MatrixTest, DefaultAndZeroDimensionMatricesAreEmpty)
{
    const math::Matrix defaultMatrix;
    const math::Matrix zeroRows(0, 3);
    const math::Matrix zeroColumns(3, 0);

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

TEST(MatrixTest, ConstructsAndInitializesRequestedDimensions)
{
    const math::Matrix matrix(2, 3, 4.5);

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

TEST(MatrixTest, RejectsDimensionsThatOverflowStorageSize)
{
    EXPECT_THROW((math::Matrix(std::numeric_limits<std::size_t>::max(), 2)), std::length_error);
}

TEST(MatrixTest, ConstructsFromRectangularInitializerLists)
{
    const math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};

    ASSERT_EQ(matrix.rows(), 3);
    ASSERT_EQ(matrix.columns(), 2);
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
    matrix.at(1, 0) = 6.0;

    EXPECT_DOUBLE_EQ(matrix(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(matrix.at(1, 0), 6.0);

    const math::Matrix& constMatrix = matrix;
    EXPECT_DOUBLE_EQ(constMatrix(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(constMatrix.at(1, 0), 6.0);
}

TEST(MatrixTest, RejectsOutOfRangeAccess)
{
    math::Matrix matrix{{1.0, 2.0}, {3.0, 4.0}};
    const math::Matrix& constMatrix = matrix;

    EXPECT_THROW(matrix(2, 0), std::out_of_range);
    EXPECT_THROW(matrix(0, 2), std::out_of_range);
    EXPECT_THROW(matrix.at(2, 0), std::out_of_range);
    EXPECT_THROW(matrix.at(0, 2), std::out_of_range);
    EXPECT_THROW(constMatrix(2, 0), std::out_of_range);
    EXPECT_THROW(constMatrix.at(0, 2), std::out_of_range);
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

    ASSERT_EQ(product.rows(), 2);
    ASSERT_EQ(product.columns(), 2);
    EXPECT_DOUBLE_EQ(product(0, 0), 58.0);
    EXPECT_DOUBLE_EQ(product(0, 1), 64.0);
    EXPECT_DOUBLE_EQ(product(1, 0), 139.0);
    EXPECT_DOUBLE_EQ(product(1, 1), 154.0);
}

TEST(MatrixTest, MultipliesCompatibleZeroDimensionMatrices)
{
    const math::Matrix product = math::Matrix(2, 0) * math::Matrix(0, 3);

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

    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.columns(); ++column)
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
    const math::Matrix transposed = matrix.transpose();

    ASSERT_EQ(transposed.rows(), 3);
    ASSERT_EQ(transposed.columns(), 2);
    EXPECT_DOUBLE_EQ(transposed(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(transposed(0, 1), 4.0);
    EXPECT_DOUBLE_EQ(transposed(1, 0), 2.0);
    EXPECT_DOUBLE_EQ(transposed(1, 1), 5.0);
    EXPECT_DOUBLE_EQ(transposed(2, 0), 3.0);
    EXPECT_DOUBLE_EQ(transposed(2, 1), 6.0);
}

TEST(MatrixTest, CreatesIdentityMatrices)
{
    const math::Matrix identity = math::Matrix::identity(3);

    ASSERT_EQ(identity.rows(), 3);
    ASSERT_EQ(identity.columns(), 3);
    for (std::size_t row = 0; row < identity.rows(); ++row)
    {
        for (std::size_t column = 0; column < identity.columns(); ++column)
        {
            EXPECT_DOUBLE_EQ(identity(row, column), row == column ? 1.0 : 0.0);
        }
    }
}

TEST(MatrixTest, CreatesEmptyIdentityMatrix)
{
    const math::Matrix identity = math::Matrix::identity(0);

    EXPECT_TRUE(identity.empty());
    EXPECT_EQ(identity.rows(), 0);
    EXPECT_EQ(identity.columns(), 0);
}

TEST(MatrixTest, RecognizesOrthogonalMatricesIncludingReflections)
{
    EXPECT_TRUE(math::Matrix::identity(4).isOrthogonal(0.0));
    EXPECT_TRUE((math::Matrix{{0.0, -1.0}, {1.0, 0.0}}).isOrthogonal(0.0));
    EXPECT_TRUE((math::Matrix{{1.0, 0.0}, {0.0, -1.0}}).isOrthogonal(0.0));
    const double angle = 0.37;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    EXPECT_TRUE((math::Matrix{{c, -s}, {s, c}}).isOrthogonal());
}

TEST(MatrixTest, RejectsNonOrthogonalMatrices)
{
    EXPECT_FALSE(math::Matrix(3, 3).isOrthogonal());
    EXPECT_FALSE((math::Matrix{{2.0, 0.0}, {0.0, 1.0}}).isOrthogonal());
    // Both columns have unit length, but they are not perpendicular.
    EXPECT_FALSE((math::Matrix{{1.0, 0.6}, {0.0, 0.8}}).isOrthogonal());
}

TEST(MatrixTest, RejectsRectangularMatricesEvenWithOrthonormalColumns)
{
    EXPECT_FALSE((math::Matrix{{1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}}).isOrthogonal());
    EXPECT_FALSE(math::Matrix(0, 3).isOrthogonal());
    EXPECT_FALSE(math::Matrix(3, 0).isOrthogonal());
}

TEST(MatrixTest, EmptySquareMatrixIsOrthogonal)
{
    EXPECT_TRUE(math::Matrix().isOrthogonal());
    EXPECT_TRUE(math::Matrix::identity(0).isOrthogonal(0.0));
}

TEST(MatrixTest, OrthogonalityUsesAbsoluteToleranceForEveryEntry)
{
    const math::Matrix scaled{{1.0 + 1e-7, 0.0}, {0.0, 1.0}};
    EXPECT_TRUE(scaled.isOrthogonal());
    EXPECT_FALSE(scaled.isOrthogonal(1e-8));
    const math::Matrix sheared{{1.0, 1e-7}, {0.0, 1.0}};
    EXPECT_TRUE(sheared.isOrthogonal(1e-7));
    EXPECT_FALSE(sheared.isOrthogonal(1e-8));
}

TEST(MatrixTest, OrthogonalityRejectsNonFiniteValuesAndOverflow)
{
    EXPECT_FALSE((math::Matrix{{std::numeric_limits<double>::quiet_NaN()}}).isOrthogonal());
    EXPECT_FALSE((math::Matrix{{std::numeric_limits<double>::infinity()}}).isOrthogonal());
    EXPECT_FALSE((math::Matrix{{std::numeric_limits<double>::max()}}).isOrthogonal());
}

TEST(MatrixTest, OrthogonalityRejectsInvalidTolerance)
{
    const math::Matrix matrix = math::Matrix::identity(2);
    EXPECT_THROW(matrix.isOrthogonal(-1.0), std::invalid_argument);
    EXPECT_THROW(matrix.isOrthogonal(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW(matrix.isOrthogonal(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(MatrixTest, ComputesDeterminantsOfSquareMatrices)
{
    EXPECT_DOUBLE_EQ((math::Matrix{{-3.5}}).det(), -3.5);
    EXPECT_DOUBLE_EQ((math::Matrix{{1.0, 2.0}, {3.0, 4.0}}).det(), -2.0);
    EXPECT_NEAR((math::Matrix{{6.0, 1.0, 1.0}, {4.0, -2.0, 5.0}, {2.0, 8.0, 7.0}}).det(), -306.0, 1e-12);
    EXPECT_DOUBLE_EQ(math::Matrix::identity(4).det(), 1.0);
}

TEST(MatrixTest, DeterminantAccountsForRowSwaps)
{
    EXPECT_DOUBLE_EQ((math::Matrix{{0.0, 1.0}, {2.0, 3.0}}).det(), -2.0);
    EXPECT_DOUBLE_EQ((math::Matrix{{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}}).det(), 1.0);
}

TEST(MatrixTest, SingularMatricesHaveZeroDeterminant)
{
    EXPECT_DOUBLE_EQ((math::Matrix{{1.0, 2.0}, {2.0, 4.0}}).det(), 0.0);
    EXPECT_DOUBLE_EQ((math::Matrix{{0.0, 1.0}, {0.0, 2.0}}).det(), 0.0);
    EXPECT_DOUBLE_EQ(math::Matrix(3, 3).det(), 0.0);
}

TEST(MatrixTest, DeterminantPreservesSmallNonzeroPivots)
{
    EXPECT_DOUBLE_EQ((math::Matrix{{1e-20, 0.0}, {0.0, 2.0}}).det(), 2e-20);
}

TEST(MatrixTest, DeterminantDoesNotModifyOriginalMatrix)
{
    const math::Matrix matrix{{0.0, 2.0}, {3.0, 4.0}};

    EXPECT_DOUBLE_EQ(matrix.det(), -6.0);
    EXPECT_DOUBLE_EQ(matrix(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(matrix(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
}

TEST(MatrixTest, EmptySquareMatrixHasUnitDeterminant)
{
    EXPECT_DOUBLE_EQ(math::Matrix().det(), 1.0);
    EXPECT_DOUBLE_EQ(math::Matrix::identity(0).det(), 1.0);
}

TEST(MatrixTest, RejectsDeterminantOfNonSquareMatrices)
{
    EXPECT_THROW(math::Matrix(2, 3).det(), std::invalid_argument);
    EXPECT_THROW(math::Matrix(0, 3).det(), std::invalid_argument);
    EXPECT_THROW(math::Matrix(3, 0).det(), std::invalid_argument);
}
