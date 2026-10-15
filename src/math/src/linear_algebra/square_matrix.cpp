#include "math/linear_algebra/square_matrix.hpp"
#include "math/linear_algebra/vector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace math
{

SquareMatrix::SquareMatrix() noexcept = default;

SquareMatrix::SquareMatrix(std::size_t size, double value)
    : _matrix(size, size, value)
{
}

SquareMatrix::SquareMatrix(std::initializer_list<std::initializer_list<double>> values)
    : SquareMatrix(Matrix(values))
{
}

SquareMatrix::SquareMatrix(Matrix matrix)
    : _matrix(std::move(matrix))
{
    if (_matrix.rows() != _matrix.columns())
    {
        throw std::invalid_argument("SquareMatrix requires equal row and column counts");
    }
}

SquareMatrix::SquareMatrix(const SquareMatrix&) = default;

SquareMatrix::SquareMatrix(SquareMatrix&& other) noexcept
{
    std::swap(_matrix, other._matrix);
}

SquareMatrix& SquareMatrix::operator=(const SquareMatrix&) = default;

SquareMatrix& SquareMatrix::operator=(SquareMatrix&& other) noexcept
{
    if (this != &other)
    {
        SquareMatrix moved(std::move(other));
        std::swap(_matrix, moved._matrix);
    }
    return *this;
}

SquareMatrix SquareMatrix::identity(std::size_t size)
{
    SquareMatrix result(size);
    for (std::size_t i = 0; i < size; ++i)
    {
        result._matrix(i, i) = 1.0;
    }

    return result;
}

std::size_t SquareMatrix::size() const noexcept
{
    return _matrix.rows();
}

std::size_t SquareMatrix::rows() const noexcept
{
    return _matrix.rows();
}

std::size_t SquareMatrix::columns() const noexcept
{
    return _matrix.columns();
}

bool SquareMatrix::empty() const noexcept
{
    return _matrix.empty();
}

bool SquareMatrix::isFinite() const noexcept
{
    return _matrix.isFinite();
}

bool SquareMatrix::isSymmetric(const Tolerance& tolerance) const noexcept
{
    if (!isFinite())
    {
        return false;
    }

    for (std::size_t row = 0; row < size(); ++row)
    {
        for (std::size_t column = 0; column < row; ++column)
        {
            if (!tolerance.isWithin(at(row, column), at(column, row)))
            {
                return false;
            }
        }
    }
    return true;
}

bool SquareMatrix::equals(const SquareMatrix& other, const Tolerance& tolerance) const noexcept
{
    return _matrix.equals(other._matrix, tolerance);
}

bool SquareMatrix::operator==(const SquareMatrix& other) const noexcept
{
    return equals(other);
}

bool SquareMatrix::operator!=(const SquareMatrix& other) const noexcept
{
    return !(*this == other);
}

bool SquareMatrix::isOrthogonal(const Tolerance& tolerance) const
{
    if (!isFinite())
    {
        return false;
    }

    for (std::size_t i = 0; i < columns(); ++i)
    {
        for (std::size_t j = i; j < columns(); ++j)
        {
            double dot = 0.0;
            for (std::size_t row = 0; row < rows(); ++row)
            {
                dot += at(row, i) * at(row, j);
            }
            const double expected = i == j ? 1.0 : 0.0;
            if (!tolerance.isWithin(dot, expected))
            {
                return false;
            }
        }
    }
    return true;
}

bool SquareMatrix::isRotation(const Tolerance& tolerance) const
{
    return isOrthogonal(tolerance) && tolerance.isWithin(det(*this), 1.0);
}

double& SquareMatrix::at(std::size_t row, std::size_t column)
{
    return _matrix.at(row, column);
}

const double& SquareMatrix::at(std::size_t row, std::size_t column) const
{
    return _matrix.at(row, column);
}

double& SquareMatrix::operator()(std::size_t row, std::size_t column)
{
    return at(row, column);
}

const double& SquareMatrix::operator()(std::size_t row, std::size_t column) const
{
    return at(row, column);
}

Vector SquareMatrix::solveLowerTriangular(const Vector& rhs) const
{
    if (size() != rhs.size())
    {
        throw std::invalid_argument("Lower triangular matrix and right-hand side dimensions must match");
    }
    if (!isFinite() || !rhs.isFinite())
    {
        throw std::domain_error("Lower triangular solve requires finite values");
    }

    for (std::size_t row = 0; row < size(); ++row)
    {
        if (at(row, row) == 0.0)
        {
            throw std::domain_error("Lower triangular matrix must have nonzero diagonal values");
        }
        for (std::size_t column = row + 1; column < size(); ++column)
        {
            if (at(row, column) != 0.0)
            {
                throw std::domain_error("Lower triangular matrix must be zero above the diagonal");
            }
        }
    }

    Vector solution(rhs.size());
    for (std::size_t row = 0; row < size(); ++row)
    {
        double value = rhs(row);
        for (std::size_t column = 0; column < row; ++column)
        {
            value -= at(row, column) * solution(column);
        }

        value /= at(row, row);
        if (!std::isfinite(value))
        {
            throw std::domain_error("Lower triangular solve failed to produce finite values");
        }
        solution(row) = value;
    }

    return solution;
}

SquareMatrix SquareMatrix::operator+(const SquareMatrix& other) const
{
    return SquareMatrix(_matrix + other._matrix);
}

SquareMatrix SquareMatrix::operator-(const SquareMatrix& other) const
{
    return SquareMatrix(_matrix - other._matrix);
}

SquareMatrix SquareMatrix::operator*(const SquareMatrix& other) const
{
    return SquareMatrix(_matrix * other._matrix);
}

SquareMatrix SquareMatrix::operator*(double scalar) const
{
    return SquareMatrix(_matrix * scalar);
}

SquareMatrix SquareMatrix::operator/(double scalar) const
{
    return SquareMatrix(_matrix / scalar);
}

const Matrix& SquareMatrix::asMatrix() const noexcept
{
    return _matrix;
}

SquareMatrix operator*(double scalar, const SquareMatrix& matrix)
{
    return matrix * scalar;
}

SquareMatrix transpose(const SquareMatrix& matrix)
{
    return SquareMatrix(transpose(matrix.asMatrix()));
}

double det(const SquareMatrix& matrix)
{
    // 원본을 보존하기 위해 복사본을 소거하고 대각 피벗의 곱을 누적한다.
    Matrix upper = matrix.asMatrix();
    double determinant = 1.0;
    for (std::size_t column = 0; column < matrix.size(); ++column)
    {
        // 남은 행 중 절댓값이 가장 큰 피벗을 선택해 나눗셈 오차를 줄인다.
        std::size_t pivotRow = column;
        for (std::size_t row = column + 1; row < matrix.size(); ++row)
        {
            if (std::abs(upper(row, column)) > std::abs(upper(pivotRow, column)))
            {
                pivotRow = row;
            }
        }

        // 피벗이 정확히 0이면 특이행렬이다. 작은 0이 아닌 값은 유지한다.
        if (upper(pivotRow, column) == 0.0)
        {
            return 0.0;
        }

        // 행을 교환할 때마다 행렬식의 부호를 반전한다.
        if (pivotRow != column)
        {
            for (std::size_t entry = column; entry < matrix.size(); ++entry)
            {
                std::swap(upper(column, entry), upper(pivotRow, entry));
            }

            determinant = -determinant;
        }

        const double pivot = upper(column, column);
        determinant *= pivot;

        for (std::size_t row = column + 1; row < matrix.size(); ++row)
        {
            const double factor = upper(row, column) / pivot;
            upper(row, column) = 0.0;

            for (std::size_t entry = column + 1; entry < matrix.size(); ++entry)
            {
                upper(row, entry) -= factor * upper(column, entry);
            }
        }
    }

    return determinant;
}

SquareMatrix cholesky(const SquareMatrix& matrix)
{
    if (!matrix.isFinite())
    {
        throw std::domain_error("Cholesky decomposition requires finite values");
    }

    if (!matrix.isSymmetric(NONE_TOLERANCE))
    {
        throw std::domain_error("Cholesky decomposition requires a symmetric matrix");
    }

    SquareMatrix lower(matrix.size());
    for (std::size_t row = 0; row < matrix.size(); ++row)
    {
        for (std::size_t column = 0; column <= row; ++column)
        {
            double value = matrix(row, column);
            for (std::size_t entry = 0; entry < column; ++entry)
            {
                value -= lower(row, entry) * lower(column, entry);
            }

            if (row == column)
            {
                if (!(value > 0.0) || !std::isfinite(value))
                {
                    throw std::domain_error("Cholesky decomposition requires a positive definite matrix");
                }
                lower(row, column) = std::sqrt(value);
            }
            else
            {
                value /= lower(column, column);
                if (!std::isfinite(value))
                {
                    throw std::domain_error("Cholesky decomposition failed to produce finite values");
                }
                lower(row, column) = value;
            }
        }
    }

    return lower;
}

}
