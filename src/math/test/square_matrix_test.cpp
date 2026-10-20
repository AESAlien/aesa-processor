#include "math/linear_algebra/square_matrix.hpp"
#include "math/linear_algebra/vector.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

static_assert(!std::is_convertible_v<aesa_processor::math::Matrix, aesa_processor::math::SquareMatrix>);
static_assert(!std::is_assignable_v<aesa_processor::math::SquareMatrix&, aesa_processor::math::Matrix>);
static_assert(std::is_same_v<decltype(std::declval<aesa_processor::math::SquareMatrix&>().asMatrix()), const aesa_processor::math::Matrix&>);
// 비상수 객체는 원소를 수정할 수 있다.
static_assert(std::is_assignable_v<decltype(std::declval<aesa_processor::math::SquareMatrix&>().at(0, 0)), double>);
static_assert(std::is_assignable_v<decltype(std::declval<aesa_processor::math::SquareMatrix&>()(0, 0)), double>);
static_assert(std::is_same_v<decltype(std::declval<const aesa_processor::math::SquareMatrix&>().at(0, 0)), const double&>);
static_assert(std::is_same_v<decltype(std::declval<const aesa_processor::math::SquareMatrix&>()(0, 0)), const double&>);



// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(SquareMatrixTest, ConstructsWithSquareShape)
{
    aesa_processor::math::SquareMatrix empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.size(), 0);
    EXPECT_EQ(empty.rows(), empty.columns());

    aesa_processor::math::SquareMatrix filled(2, 4.5);
    EXPECT_EQ(filled.size(), 2);
    EXPECT_DOUBLE_EQ(filled(0, 1), 4.5);
    EXPECT_DOUBLE_EQ(filled(1, 0), 4.5);

    aesa_processor::math::Matrix original{{1.0, 2.0}, {3.0, 4.0}};
    aesa_processor::math::SquareMatrix square(original);
    original(0, 0) = 9.0;
    EXPECT_DOUBLE_EQ(square(0, 0), 1.0);
    EXPECT_THROW((aesa_processor::math::SquareMatrix(std::numeric_limits<std::size_t>::max())), std::length_error);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(SquareMatrixTest, CopyAndMovePreserveInvariants)
{
    aesa_processor::math::SquareMatrix original{{1.0, 2.0}, {3.0, 4.0}};
    aesa_processor::math::SquareMatrix copy = original;
    EXPECT_DOUBLE_EQ(copy(0, 0), 1.0);
    copy = aesa_processor::math::SquareMatrix{{9.0}};
    EXPECT_DOUBLE_EQ(copy(0, 0), 9.0);
    EXPECT_DOUBLE_EQ(original(0, 0), 1.0);
    aesa_processor::math::SquareMatrix assigned(1);
    assigned = original;
    EXPECT_EQ(assigned.size(), 2);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);

    aesa_processor::math::SquareMatrix moved(std::move(original));
    EXPECT_DOUBLE_EQ(moved(1, 0), 3.0);
    EXPECT_EQ(original.size(), 0);
    EXPECT_EQ(original.columns(), 0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(original), 1.0);
    EXPECT_THROW(original(0, 0), std::out_of_range);
    assigned = std::move(moved);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);
    EXPECT_TRUE(moved.empty());
    assigned = std::move(assigned);
    EXPECT_DOUBLE_EQ(assigned(1, 1), 4.0);
}

// 객체 생성과 상태 보존 규칙을 검증한다.
TEST(SquareMatrixTest, CreatesIdentityMatrices)
{
    aesa_processor::math::SquareMatrix identity = aesa_processor::math::SquareMatrix::identity(3);

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
    aesa_processor::math::SquareMatrix emptyIdentity = aesa_processor::math::SquareMatrix::identity(0);
    EXPECT_TRUE(emptyIdentity.empty());
    EXPECT_EQ(emptyIdentity.rows(), 0);
    EXPECT_EQ(emptyIdentity.columns(), 0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, AccessorsCheckBounds)
{
    aesa_processor::math::SquareMatrix matrix(2);
    matrix.at(1, 0) = 3.0;
    matrix(0, 1) = 4.0;
    EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(matrix.at(0, 1), 4.0);
    EXPECT_EQ(matrix.rows(), 2);
    EXPECT_EQ(matrix.columns(), 2);
    EXPECT_THROW(matrix.at(2, 0), std::out_of_range);
    EXPECT_THROW(matrix(0, 2), std::out_of_range);
    const aesa_processor::math::SquareMatrix& constant = matrix;
    EXPECT_DOUBLE_EQ(constant(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(constant.at(0, 1), 4.0);
    EXPECT_THROW(constant.at(2, 0), std::out_of_range);
    EXPECT_THROW(constant(0, 2), std::out_of_range);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ArithmeticPreservesTypeAndValues)
{
    aesa_processor::math::SquareMatrix left{{1.0, 2.0}, {3.0, 4.0}};
    aesa_processor::math::SquareMatrix right{{5.0, 6.0}, {7.0, 8.0}};
    auto sum = left + right;
    auto difference = left - right;
    auto product = left * right;
    auto transposed = aesa_processor::math::transpose(left);
    static_assert(std::is_same_v<decltype(sum), aesa_processor::math::SquareMatrix>);
    static_assert(std::is_same_v<decltype(product), aesa_processor::math::SquareMatrix>);
    static_assert(std::is_same_v<decltype(transposed), aesa_processor::math::SquareMatrix>);
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
    EXPECT_THROW(left + aesa_processor::math::SquareMatrix(3), std::invalid_argument);
    EXPECT_THROW(left - aesa_processor::math::SquareMatrix(3), std::invalid_argument);
    EXPECT_THROW(left * aesa_processor::math::SquareMatrix(3), std::invalid_argument);

    aesa_processor::math::SquareMatrix empty;
    EXPECT_TRUE((empty + empty).empty());
    EXPECT_TRUE((empty - empty).empty());
    EXPECT_TRUE((empty * empty).empty());
    EXPECT_TRUE(aesa_processor::math::transpose(empty).empty());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ComputesDeterminantsOfSquareMatrices)
{
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{-3.5}}), -3.5);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{1.0, 2.0}, {3.0, 4.0}}), -2.0);
    EXPECT_NEAR(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{6.0, 1.0, 1.0}, {4.0, -2.0, 5.0}, {2.0, 8.0, 7.0}}), -306.0, 1e-12);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix::identity(4)), 1.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, DeterminantAccountsForRowSwaps)
{
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{0.0, 1.0}, {2.0, 3.0}}), -2.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}}), 1.0);

    // 행 교환과 소거를 수행해도 원본 행렬의 원소는 유지한다.
    aesa_processor::math::SquareMatrix matrix{{0.0, 2.0}, {3.0, 4.0}};

    EXPECT_DOUBLE_EQ(aesa_processor::math::det(matrix), -6.0);
    EXPECT_DOUBLE_EQ(matrix(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(matrix(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SingularMatricesHaveZeroDeterminant)
{
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{1.0, 2.0}, {2.0, 4.0}}), 0.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{0.0, 1.0}, {0.0, 2.0}}), 0.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix(3)), 0.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, DeterminantPreservesSmallNonzeroPivots)
{
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix{{1e-20, 0.0}, {0.0, 2.0}}), 2e-20);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, EmptySquareMatrixHasUnitDeterminant)
{
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix()), 1.0);
    EXPECT_DOUBLE_EQ(aesa_processor::math::det(aesa_processor::math::SquareMatrix::identity(0)), 1.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ComputesCholeskyDecomposition)
{
    const aesa_processor::math::SquareMatrix matrix{{4.0, 12.0, -16.0},
                                    {12.0, 37.0, -43.0},
                                    {-16.0, -43.0, 98.0}};
    const aesa_processor::math::SquareMatrix expected{{2.0, 0.0, 0.0},
                                      {6.0, 1.0, 0.0},
                                      {-8.0, 5.0, 3.0}};

    const aesa_processor::math::SquareMatrix lower = aesa_processor::math::cholesky(matrix);

    EXPECT_TRUE(lower.equals(expected, aesa_processor::math::Tolerance{1e-12, 1e-12}));
    EXPECT_TRUE((lower * aesa_processor::math::transpose(lower)).equals(matrix, aesa_processor::math::Tolerance{1e-12, 1e-12}));
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ComputesCholeskyDecompositionForArbitrarySize)
{
    const aesa_processor::math::SquareMatrix expected{{2.0, 0.0, 0.0, 0.0},
                                      {1.0, 3.0, 0.0, 0.0},
                                      {-1.0, 2.0, 4.0, 0.0},
                                      {3.0, 0.5, -2.0, 5.0}};
    const aesa_processor::math::SquareMatrix matrix = expected * aesa_processor::math::transpose(expected);

    const aesa_processor::math::SquareMatrix lower = aesa_processor::math::cholesky(matrix);

    ASSERT_EQ(lower.size(), 4);
    EXPECT_TRUE(lower.equals(expected, aesa_processor::math::Tolerance{1e-12, 1e-12}));
    for (std::size_t row = 0; row < lower.size(); ++row)
    {
        for (std::size_t column = row + 1; column < lower.size(); ++column)
        {
            EXPECT_DOUBLE_EQ(lower(row, column), 0.0);
        }
    }
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, HandlesSmallAndEmptyCholeskyDecompositions)
{
    const aesa_processor::math::SquareMatrix single = aesa_processor::math::cholesky(aesa_processor::math::SquareMatrix{{9.0}});
    EXPECT_DOUBLE_EQ(single(0, 0), 3.0);
    EXPECT_TRUE(aesa_processor::math::cholesky(aesa_processor::math::SquareMatrix()).empty());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesLowerTriangularSystems)
{
    const aesa_processor::math::SquareMatrix lower{{2.0, 0.0, 0.0},
                                   {3.0, 1.0, 0.0},
                                   {1.0, -1.0, 2.0}};
    const aesa_processor::math::Vector rhs{2.0, 5.0, 5.0};

    const aesa_processor::math::Vector solution = lower.solveLowerTriangular(rhs);

    EXPECT_TRUE(solution.equals(aesa_processor::math::Vector{1.0, 2.0, 3.0}, aesa_processor::math::Tolerance{1e-12, 1e-12}));
    EXPECT_TRUE((lower * solution).equals(rhs, aesa_processor::math::Tolerance{1e-12, 1e-12}));
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesSmallAndEmptyLowerTriangularSystems)
{
    const aesa_processor::math::Vector single = aesa_processor::math::SquareMatrix{{4.0}}.solveLowerTriangular(aesa_processor::math::Vector{10.0});

    EXPECT_DOUBLE_EQ(single(0), 2.5);
    EXPECT_TRUE(aesa_processor::math::SquareMatrix().solveLowerTriangular(aesa_processor::math::Vector()).empty());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesUpperTriangularSystems)
{
    const aesa_processor::math::SquareMatrix upper{{2.0, 3.0, 1.0},
                                   {0.0, 1.0, -1.0},
                                   {0.0, 0.0, 2.0}};
    const aesa_processor::math::Vector rhs{11.0, -1.0, 6.0};

    const aesa_processor::math::Vector solution = upper.solveUpperTriangular(rhs);

    EXPECT_TRUE(solution.equals(aesa_processor::math::Vector{1.0, 2.0, 3.0}, aesa_processor::math::Tolerance{1e-12, 1e-12}));
    EXPECT_TRUE((upper * solution).equals(rhs, aesa_processor::math::Tolerance{1e-12, 1e-12}));
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesSmallAndEmptyUpperTriangularSystems)
{
    const aesa_processor::math::Vector single = aesa_processor::math::SquareMatrix{{4.0}}.solveUpperTriangular(aesa_processor::math::Vector{10.0});

    EXPECT_DOUBLE_EQ(single(0), 2.5);
    EXPECT_TRUE(aesa_processor::math::SquareMatrix().solveUpperTriangular(aesa_processor::math::Vector()).empty());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksWhetherAllElementsAreFinite)
{
    aesa_processor::math::SquareMatrix matrix{{1.0, -2.0}, {3.0, 4.0}};
    EXPECT_TRUE(matrix.isFinite());
    EXPECT_TRUE(aesa_processor::math::SquareMatrix().isFinite());

    matrix(0, 1) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(matrix.isFinite());
    matrix(0, 1) = -std::numeric_limits<double>::infinity();
    EXPECT_FALSE(matrix.isFinite());
    matrix(0, 1) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(matrix.isFinite());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksSymmetry)
{
    EXPECT_TRUE((aesa_processor::math::SquareMatrix{{1.0, 2.0}, {2.0, 3.0}}).isSymmetric());
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{1.0, 2.0}, {3.0, 4.0}}).isSymmetric());

    const aesa_processor::math::SquareMatrix nearlySymmetric{{1.0, 2.0}, {2.0 + 1e-10, 3.0}};
    EXPECT_TRUE(nearlySymmetric.isSymmetric());
    EXPECT_FALSE(nearlySymmetric.isSymmetric(aesa_processor::math::NONE_TOLERANCE));
    EXPECT_TRUE(nearlySymmetric.isSymmetric(aesa_processor::math::Tolerance{1e-9, 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, HandlesSymmetryEdgeCases)
{
    EXPECT_TRUE(aesa_processor::math::SquareMatrix().isSymmetric());
    EXPECT_TRUE((aesa_processor::math::SquareMatrix{{5.0}}).isSymmetric());
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{std::numeric_limits<double>::infinity()}}).isSymmetric());
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{std::numeric_limits<double>::quiet_NaN()}}).isSymmetric());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksOrthogonality)
{
    EXPECT_TRUE(aesa_processor::math::SquareMatrix::identity(3).isOrthogonal());
    EXPECT_TRUE(aesa_processor::math::SquareMatrix::identity(3).isOrthogonal(aesa_processor::math::Tolerance{0.0, 0.0}));
    EXPECT_TRUE((aesa_processor::math::SquareMatrix{{1.0, 1e-4}, {0.0, 1.0}}).isOrthogonal(aesa_processor::math::Tolerance{1e-3, 0.0}));
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{1.0, 1.0}, {0.0, 1.0}}).isOrthogonal(aesa_processor::math::Tolerance{1e-6, 0.0}));
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{1.0, 0.0},
                                    {0.0, std::numeric_limits<double>::infinity()}})
        .isOrthogonal(aesa_processor::math::Tolerance{1e-6, 0.0}));

    EXPECT_THROW(aesa_processor::math::SquareMatrix::identity(2).isOrthogonal(aesa_processor::math::Tolerance{-1.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(aesa_processor::math::SquareMatrix::identity(2).isOrthogonal(aesa_processor::math::Tolerance{std::numeric_limits<double>::infinity(), 0.0}), std::invalid_argument);
    EXPECT_THROW(aesa_processor::math::SquareMatrix::identity(2).isOrthogonal(aesa_processor::math::Tolerance{std::numeric_limits<double>::quiet_NaN(), 0.0}), std::invalid_argument);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksRotationMatrices)
{
    const aesa_processor::math::SquareMatrix rotation{{0.0, -1.0, 0.0},
                                      {1.0, 0.0, 0.0},
                                      {0.0, 0.0, 1.0}};
    EXPECT_TRUE(rotation.isRotation());
    EXPECT_TRUE(rotation.isRotation(aesa_processor::math::Tolerance{1e-12, 0.0}));
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{1.0, 0.0, 0.0},
                                     {0.0, 1.0, 0.0},
                                     {0.0, 0.0, -1.0}})
        .isRotation(aesa_processor::math::Tolerance{1e-12, 0.0}));
    EXPECT_FALSE((aesa_processor::math::SquareMatrix{{2.0, 0.0, 0.0},
                                     {0.0, 1.0, 0.0},
                                     {0.0, 0.0, 1.0}})
        .isRotation(aesa_processor::math::Tolerance{1e-12, 0.0}));
    EXPECT_TRUE((aesa_processor::math::SquareMatrix{{0.0, -1.0}, {1.0, 0.0}}).isRotation(aesa_processor::math::Tolerance{1e-12, 0.0}));
    EXPECT_TRUE(aesa_processor::math::SquareMatrix::identity(4).isRotation(aesa_processor::math::Tolerance{1e-12, 0.0}));
    EXPECT_THROW(aesa_processor::math::SquareMatrix::identity(3).isRotation(aesa_processor::math::Tolerance{-1.0, 0.0}), std::invalid_argument);
    const aesa_processor::math::SquareMatrix nonfinite{{1.0, 0.0, 0.0},
                                       {0.0, 1.0, 0.0},
                                       {0.0, 0.0, std::numeric_limits<double>::infinity()}};
    EXPECT_FALSE(nonfinite.isRotation(aesa_processor::math::Tolerance{1e-12, 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, AppliesCombinedPolicyToOrthogonalityAndRotation)
{
    const aesa_processor::math::Tolerance relativeOnly{0.0, 1e-3};
    const aesa_processor::math::Tolerance combined{1e-3, 1e-3};
    const aesa_processor::math::SquareMatrix scaled{{1.0001, 0.0}, {0.0, 1.0}};
    EXPECT_FALSE(scaled.isOrthogonal(aesa_processor::math::Tolerance{0.0, 0.0}));
    EXPECT_TRUE(scaled.isOrthogonal(relativeOnly));
    EXPECT_TRUE(scaled.isRotation(relativeOnly));
    const aesa_processor::math::SquareMatrix nearRotation{{1.0001, 5e-4}, {0.0, 1.0}};
    EXPECT_FALSE(nearRotation.isOrthogonal(relativeOnly));
    EXPECT_FALSE(nearRotation.isRotation(relativeOnly));
    EXPECT_TRUE(nearRotation.isOrthogonal(combined));
    EXPECT_TRUE(nearRotation.isRotation(combined));
    const aesa_processor::math::SquareMatrix reflection{{1.0, 0.0}, {0.0, -1.0}};
    EXPECT_TRUE(reflection.isOrthogonal(combined));
    EXPECT_FALSE(reflection.isRotation(combined));
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsNonSquareInputs)
{
    EXPECT_THROW((aesa_processor::math::SquareMatrix(aesa_processor::math::Matrix(2, 3))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::SquareMatrix(aesa_processor::math::Matrix(0, 3))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::SquareMatrix(aesa_processor::math::Matrix(3, 0))), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::SquareMatrix{{1.0, 2.0}}), std::invalid_argument);
    EXPECT_THROW((aesa_processor::math::SquareMatrix{{1.0, 2.0}, {3.0}}), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidPolicyBeforeInspectingValues)
{
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((aesa_processor::math::Tolerance{invalid, 1e-3}), std::invalid_argument);
        EXPECT_THROW((aesa_processor::math::Tolerance{1.0, invalid}), std::invalid_argument);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidCholeskyInputs)
{
    EXPECT_THROW(aesa_processor::math::cholesky(aesa_processor::math::SquareMatrix{{1.0, 2.0}, {0.0, 1.0}}), std::domain_error);
    EXPECT_THROW(aesa_processor::math::cholesky(aesa_processor::math::SquareMatrix{{1.0, 1.0}, {1.0, 1.0}}), std::domain_error);
    EXPECT_THROW(aesa_processor::math::cholesky(aesa_processor::math::SquareMatrix{{1.0, 2.0}, {2.0, 1.0}}), std::domain_error);
    EXPECT_THROW(aesa_processor::math::cholesky(aesa_processor::math::SquareMatrix{{1.0, 0.0},
                                                    {0.0, std::numeric_limits<double>::infinity()}}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidLowerTriangularSystems)
{
    EXPECT_THROW(
        aesa_processor::math::SquareMatrix::identity(2).solveLowerTriangular(aesa_processor::math::Vector{1.0}),
        std::invalid_argument);
    EXPECT_THROW(
        (aesa_processor::math::SquareMatrix{{1.0, 2.0}, {0.0, 1.0}}).solveLowerTriangular(aesa_processor::math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (aesa_processor::math::SquareMatrix{{1.0, 0.0}, {2.0, 0.0}}).solveLowerTriangular(aesa_processor::math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (aesa_processor::math::SquareMatrix{{1.0, 0.0}, {2.0, std::numeric_limits<double>::infinity()}})
            .solveLowerTriangular(aesa_processor::math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        aesa_processor::math::SquareMatrix::identity(2).solveLowerTriangular(
            aesa_processor::math::Vector{1.0, std::numeric_limits<double>::quiet_NaN()}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsNonfiniteLowerTriangularSolutions)
{
    const double maximum = std::numeric_limits<double>::max();
    const aesa_processor::math::SquareMatrix lower{{1.0, 0.0}, {maximum, 1.0}};

    EXPECT_THROW(
        lower.solveLowerTriangular(aesa_processor::math::Vector{maximum, 0.0}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidUpperTriangularSystems)
{
    EXPECT_THROW(
        aesa_processor::math::SquareMatrix::identity(2).solveUpperTriangular(aesa_processor::math::Vector{1.0}),
        std::invalid_argument);
    EXPECT_THROW(
        (aesa_processor::math::SquareMatrix{{1.0, 0.0}, {2.0, 1.0}}).solveUpperTriangular(aesa_processor::math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (aesa_processor::math::SquareMatrix{{0.0, 2.0}, {0.0, 1.0}}).solveUpperTriangular(aesa_processor::math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (aesa_processor::math::SquareMatrix{{std::numeric_limits<double>::infinity(), 2.0}, {0.0, 1.0}})
            .solveUpperTriangular(aesa_processor::math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        aesa_processor::math::SquareMatrix::identity(2).solveUpperTriangular(
            aesa_processor::math::Vector{1.0, std::numeric_limits<double>::quiet_NaN()}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsNonfiniteUpperTriangularSolutions)
{
    const double maximum = std::numeric_limits<double>::max();
    const aesa_processor::math::SquareMatrix upper{{1.0, maximum}, {0.0, 1.0}};

    EXPECT_THROW(
        upper.solveUpperTriangular(aesa_processor::math::Vector{0.0, maximum}),
        std::domain_error);
}
