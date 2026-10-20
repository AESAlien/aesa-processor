#include "math/linear_algebra/square_matrix.hpp"
#include "math/linear_algebra/vector.hpp"

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



// 객체 생성과 상태 보존 규칙을 검증한다.
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

// 객체 생성과 상태 보존 규칙을 검증한다.
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
    EXPECT_DOUBLE_EQ(math::det(original), 1.0);
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

// 계산 결과와 수치적 안정성을 검증한다.
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

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ArithmeticPreservesTypeAndValues)
{
    math::SquareMatrix left{{1.0, 2.0}, {3.0, 4.0}};
    math::SquareMatrix right{{5.0, 6.0}, {7.0, 8.0}};
    auto sum = left + right;
    auto difference = left - right;
    auto product = left * right;
    auto transposed = math::transpose(left);
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
    EXPECT_TRUE(math::transpose(empty).empty());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ComputesDeterminantsOfSquareMatrices)
{
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{-3.5}}), -3.5);
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{1.0, 2.0}, {3.0, 4.0}}), -2.0);
    EXPECT_NEAR(math::det(math::SquareMatrix{{6.0, 1.0, 1.0}, {4.0, -2.0, 5.0}, {2.0, 8.0, 7.0}}), -306.0, 1e-12);
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix::identity(4)), 1.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, DeterminantAccountsForRowSwaps)
{
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{0.0, 1.0}, {2.0, 3.0}}), -2.0);
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}}), 1.0);

    // 행 교환과 소거를 수행해도 원본 행렬의 원소는 유지한다.
    math::SquareMatrix matrix{{0.0, 2.0}, {3.0, 4.0}};

    EXPECT_DOUBLE_EQ(math::det(matrix), -6.0);
    EXPECT_DOUBLE_EQ(matrix(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(matrix(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SingularMatricesHaveZeroDeterminant)
{
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{1.0, 2.0}, {2.0, 4.0}}), 0.0);
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{0.0, 1.0}, {0.0, 2.0}}), 0.0);
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix(3)), 0.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, DeterminantPreservesSmallNonzeroPivots)
{
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix{{1e-20, 0.0}, {0.0, 2.0}}), 2e-20);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, EmptySquareMatrixHasUnitDeterminant)
{
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix()), 1.0);
    EXPECT_DOUBLE_EQ(math::det(math::SquareMatrix::identity(0)), 1.0);
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ComputesCholeskyDecomposition)
{
    const math::SquareMatrix matrix{{4.0, 12.0, -16.0},
                                    {12.0, 37.0, -43.0},
                                    {-16.0, -43.0, 98.0}};
    const math::SquareMatrix expected{{2.0, 0.0, 0.0},
                                      {6.0, 1.0, 0.0},
                                      {-8.0, 5.0, 3.0}};

    const math::SquareMatrix lower = math::cholesky(matrix);

    EXPECT_TRUE(lower.equals(expected, math::Tolerance{1e-12, 1e-12}));
    EXPECT_TRUE((lower * math::transpose(lower)).equals(matrix, math::Tolerance{1e-12, 1e-12}));
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, ComputesCholeskyDecompositionForArbitrarySize)
{
    const math::SquareMatrix expected{{2.0, 0.0, 0.0, 0.0},
                                      {1.0, 3.0, 0.0, 0.0},
                                      {-1.0, 2.0, 4.0, 0.0},
                                      {3.0, 0.5, -2.0, 5.0}};
    const math::SquareMatrix matrix = expected * math::transpose(expected);

    const math::SquareMatrix lower = math::cholesky(matrix);

    ASSERT_EQ(lower.size(), 4);
    EXPECT_TRUE(lower.equals(expected, math::Tolerance{1e-12, 1e-12}));
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
    const math::SquareMatrix single = math::cholesky(math::SquareMatrix{{9.0}});
    EXPECT_DOUBLE_EQ(single(0, 0), 3.0);
    EXPECT_TRUE(math::cholesky(math::SquareMatrix()).empty());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesLowerTriangularSystems)
{
    const math::SquareMatrix lower{{2.0, 0.0, 0.0},
                                   {3.0, 1.0, 0.0},
                                   {1.0, -1.0, 2.0}};
    const math::Vector rhs{2.0, 5.0, 5.0};

    const math::Vector solution = lower.solveLowerTriangular(rhs);

    EXPECT_TRUE(solution.equals(math::Vector{1.0, 2.0, 3.0}, math::Tolerance{1e-12, 1e-12}));
    EXPECT_TRUE((lower * solution).equals(rhs, math::Tolerance{1e-12, 1e-12}));
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesSmallAndEmptyLowerTriangularSystems)
{
    const math::Vector single = math::SquareMatrix{{4.0}}.solveLowerTriangular(math::Vector{10.0});

    EXPECT_DOUBLE_EQ(single(0), 2.5);
    EXPECT_TRUE(math::SquareMatrix().solveLowerTriangular(math::Vector()).empty());
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesUpperTriangularSystems)
{
    const math::SquareMatrix upper{{2.0, 3.0, 1.0},
                                   {0.0, 1.0, -1.0},
                                   {0.0, 0.0, 2.0}};
    const math::Vector rhs{11.0, -1.0, 6.0};

    const math::Vector solution = upper.solveUpperTriangular(rhs);

    EXPECT_TRUE(solution.equals(math::Vector{1.0, 2.0, 3.0}, math::Tolerance{1e-12, 1e-12}));
    EXPECT_TRUE((upper * solution).equals(rhs, math::Tolerance{1e-12, 1e-12}));
}

// 계산 결과와 수치적 안정성을 검증한다.
TEST(SquareMatrixTest, SolvesSmallAndEmptyUpperTriangularSystems)
{
    const math::Vector single = math::SquareMatrix{{4.0}}.solveUpperTriangular(math::Vector{10.0});

    EXPECT_DOUBLE_EQ(single(0), 2.5);
    EXPECT_TRUE(math::SquareMatrix().solveUpperTriangular(math::Vector()).empty());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksWhetherAllElementsAreFinite)
{
    math::SquareMatrix matrix{{1.0, -2.0}, {3.0, 4.0}};
    EXPECT_TRUE(matrix.isFinite());
    EXPECT_TRUE(math::SquareMatrix().isFinite());

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
    EXPECT_TRUE((math::SquareMatrix{{1.0, 2.0}, {2.0, 3.0}}).isSymmetric());
    EXPECT_FALSE((math::SquareMatrix{{1.0, 2.0}, {3.0, 4.0}}).isSymmetric());

    const math::SquareMatrix nearlySymmetric{{1.0, 2.0}, {2.0 + 1e-10, 3.0}};
    EXPECT_TRUE(nearlySymmetric.isSymmetric());
    EXPECT_FALSE(nearlySymmetric.isSymmetric(math::NONE_TOLERANCE));
    EXPECT_TRUE(nearlySymmetric.isSymmetric(math::Tolerance{1e-9, 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, HandlesSymmetryEdgeCases)
{
    EXPECT_TRUE(math::SquareMatrix().isSymmetric());
    EXPECT_TRUE((math::SquareMatrix{{5.0}}).isSymmetric());
    EXPECT_FALSE((math::SquareMatrix{{std::numeric_limits<double>::infinity()}}).isSymmetric());
    EXPECT_FALSE((math::SquareMatrix{{std::numeric_limits<double>::quiet_NaN()}}).isSymmetric());
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksOrthogonality)
{
    EXPECT_TRUE(math::SquareMatrix::identity(3).isOrthogonal());
    EXPECT_TRUE(math::SquareMatrix::identity(3).isOrthogonal(math::Tolerance{0.0, 0.0}));
    EXPECT_TRUE((math::SquareMatrix{{1.0, 1e-4}, {0.0, 1.0}}).isOrthogonal(math::Tolerance{1e-3, 0.0}));
    EXPECT_FALSE((math::SquareMatrix{{1.0, 1.0}, {0.0, 1.0}}).isOrthogonal(math::Tolerance{1e-6, 0.0}));
    EXPECT_FALSE((math::SquareMatrix{{1.0, 0.0},
                                    {0.0, std::numeric_limits<double>::infinity()}})
        .isOrthogonal(math::Tolerance{1e-6, 0.0}));

    EXPECT_THROW(math::SquareMatrix::identity(2).isOrthogonal(math::Tolerance{-1.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(math::SquareMatrix::identity(2).isOrthogonal(math::Tolerance{std::numeric_limits<double>::infinity(), 0.0}), std::invalid_argument);
    EXPECT_THROW(math::SquareMatrix::identity(2).isOrthogonal(math::Tolerance{std::numeric_limits<double>::quiet_NaN(), 0.0}), std::invalid_argument);
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, ChecksRotationMatrices)
{
    const math::SquareMatrix rotation{{0.0, -1.0, 0.0},
                                      {1.0, 0.0, 0.0},
                                      {0.0, 0.0, 1.0}};
    EXPECT_TRUE(rotation.isRotation());
    EXPECT_TRUE(rotation.isRotation(math::Tolerance{1e-12, 0.0}));
    EXPECT_FALSE((math::SquareMatrix{{1.0, 0.0, 0.0},
                                     {0.0, 1.0, 0.0},
                                     {0.0, 0.0, -1.0}})
        .isRotation(math::Tolerance{1e-12, 0.0}));
    EXPECT_FALSE((math::SquareMatrix{{2.0, 0.0, 0.0},
                                     {0.0, 1.0, 0.0},
                                     {0.0, 0.0, 1.0}})
        .isRotation(math::Tolerance{1e-12, 0.0}));
    EXPECT_TRUE((math::SquareMatrix{{0.0, -1.0}, {1.0, 0.0}}).isRotation(math::Tolerance{1e-12, 0.0}));
    EXPECT_TRUE(math::SquareMatrix::identity(4).isRotation(math::Tolerance{1e-12, 0.0}));
    EXPECT_THROW(math::SquareMatrix::identity(3).isRotation(math::Tolerance{-1.0, 0.0}), std::invalid_argument);
    const math::SquareMatrix nonfinite{{1.0, 0.0, 0.0},
                                       {0.0, 1.0, 0.0},
                                       {0.0, 0.0, std::numeric_limits<double>::infinity()}};
    EXPECT_FALSE(nonfinite.isRotation(math::Tolerance{1e-12, 0.0}));
}

// 비교 정책과 판정 경계가 일관되게 적용되는지 검증한다.
TEST(SquareMatrixTest, AppliesCombinedPolicyToOrthogonalityAndRotation)
{
    const math::Tolerance relativeOnly{0.0, 1e-3};
    const math::Tolerance combined{1e-3, 1e-3};
    const math::SquareMatrix scaled{{1.0001, 0.0}, {0.0, 1.0}};
    EXPECT_FALSE(scaled.isOrthogonal(math::Tolerance{0.0, 0.0}));
    EXPECT_TRUE(scaled.isOrthogonal(relativeOnly));
    EXPECT_TRUE(scaled.isRotation(relativeOnly));
    const math::SquareMatrix nearRotation{{1.0001, 5e-4}, {0.0, 1.0}};
    EXPECT_FALSE(nearRotation.isOrthogonal(relativeOnly));
    EXPECT_FALSE(nearRotation.isRotation(relativeOnly));
    EXPECT_TRUE(nearRotation.isOrthogonal(combined));
    EXPECT_TRUE(nearRotation.isRotation(combined));
    const math::SquareMatrix reflection{{1.0, 0.0}, {0.0, -1.0}};
    EXPECT_TRUE(reflection.isOrthogonal(combined));
    EXPECT_FALSE(reflection.isRotation(combined));
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsNonSquareInputs)
{
    EXPECT_THROW((math::SquareMatrix(math::Matrix(2, 3))), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix(math::Matrix(0, 3))), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix(math::Matrix(3, 0))), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix{{1.0, 2.0}}), std::invalid_argument);
    EXPECT_THROW((math::SquareMatrix{{1.0, 2.0}, {3.0}}), std::invalid_argument);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidPolicyBeforeInspectingValues)
{
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW((math::Tolerance{invalid, 1e-3}), std::invalid_argument);
        EXPECT_THROW((math::Tolerance{1.0, invalid}), std::invalid_argument);
    }
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidCholeskyInputs)
{
    EXPECT_THROW(math::cholesky(math::SquareMatrix{{1.0, 2.0}, {0.0, 1.0}}), std::domain_error);
    EXPECT_THROW(math::cholesky(math::SquareMatrix{{1.0, 1.0}, {1.0, 1.0}}), std::domain_error);
    EXPECT_THROW(math::cholesky(math::SquareMatrix{{1.0, 2.0}, {2.0, 1.0}}), std::domain_error);
    EXPECT_THROW(math::cholesky(math::SquareMatrix{{1.0, 0.0},
                                                    {0.0, std::numeric_limits<double>::infinity()}}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidLowerTriangularSystems)
{
    EXPECT_THROW(
        math::SquareMatrix::identity(2).solveLowerTriangular(math::Vector{1.0}),
        std::invalid_argument);
    EXPECT_THROW(
        (math::SquareMatrix{{1.0, 2.0}, {0.0, 1.0}}).solveLowerTriangular(math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (math::SquareMatrix{{1.0, 0.0}, {2.0, 0.0}}).solveLowerTriangular(math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (math::SquareMatrix{{1.0, 0.0}, {2.0, std::numeric_limits<double>::infinity()}})
            .solveLowerTriangular(math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        math::SquareMatrix::identity(2).solveLowerTriangular(
            math::Vector{1.0, std::numeric_limits<double>::quiet_NaN()}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsNonfiniteLowerTriangularSolutions)
{
    const double maximum = std::numeric_limits<double>::max();
    const math::SquareMatrix lower{{1.0, 0.0}, {maximum, 1.0}};

    EXPECT_THROW(
        lower.solveLowerTriangular(math::Vector{maximum, 0.0}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsInvalidUpperTriangularSystems)
{
    EXPECT_THROW(
        math::SquareMatrix::identity(2).solveUpperTriangular(math::Vector{1.0}),
        std::invalid_argument);
    EXPECT_THROW(
        (math::SquareMatrix{{1.0, 0.0}, {2.0, 1.0}}).solveUpperTriangular(math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (math::SquareMatrix{{0.0, 2.0}, {0.0, 1.0}}).solveUpperTriangular(math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        (math::SquareMatrix{{std::numeric_limits<double>::infinity(), 2.0}, {0.0, 1.0}})
            .solveUpperTriangular(math::Vector{1.0, 2.0}),
        std::domain_error);
    EXPECT_THROW(
        math::SquareMatrix::identity(2).solveUpperTriangular(
            math::Vector{1.0, std::numeric_limits<double>::quiet_NaN()}),
        std::domain_error);
}

// 예외 입력과 경계 조건을 올바르게 처리하는지 검증한다.
TEST(SquareMatrixTest, RejectsNonfiniteUpperTriangularSolutions)
{
    const double maximum = std::numeric_limits<double>::max();
    const math::SquareMatrix upper{{1.0, maximum}, {0.0, 1.0}};

    EXPECT_THROW(
        upper.solveUpperTriangular(math::Vector{0.0, maximum}),
        std::domain_error);
}
