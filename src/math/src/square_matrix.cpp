#include "math/square_matrix.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace math
{

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

SquareMatrix::SquareMatrix(SquareMatrix&& other) noexcept
{
    std::swap(_matrix, other._matrix);
}

SquareMatrix& SquareMatrix::operator=(SquareMatrix&& other) noexcept
{
    if (this != &other)
    {
        SquareMatrix moved(std::move(other));
        std::swap(_matrix, moved._matrix);
    }
    return *this;
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

bool SquareMatrix::equals(const SquareMatrix& other, const SquareMatrix& absoluteTolerance) const noexcept
{
    if (size() != other.size() || size() != absoluteTolerance.size())
    {
        return false;
    }
    for (std::size_t row = 0; row < size(); ++row)
    {
        for (std::size_t column = 0; column < size(); ++column)
        {
            const double a = at(row, column);
            const double b = other.at(row, column);
            const double tolerance = absoluteTolerance.at(row, column);
            if (!std::isfinite(a) || !std::isfinite(b) ||
                !std::isfinite(tolerance) || tolerance < 0.0 || std::abs(a - b) > tolerance)
            {
                return false;
            }
        }
    }
    return true;
}

bool SquareMatrix::equals(const SquareMatrix& other, double relativeTolerance) const noexcept
{
    return _matrix.equals(other._matrix, relativeTolerance);
}

double& SquareMatrix::at(std::size_t row, std::size_t column)
{
    return _matrix.at(row, column);
}

double& SquareMatrix::operator()(std::size_t row, std::size_t column)
{
    return at(row, column);
}

const double& SquareMatrix::at(std::size_t row, std::size_t column) const
{
    return _matrix.at(row, column);
}

const double& SquareMatrix::operator()(std::size_t row, std::size_t column) const
{
    return at(row, column);
}

SquareMatrix SquareMatrix::transpose() const
{
    return SquareMatrix(_matrix.transpose());
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

SquareMatrix SquareMatrix::identity(std::size_t size)
{
    SquareMatrix result(size);
    for (std::size_t i = 0; i < size; ++i)
    {
        result._matrix(i, i) = 1.0;
    }

    return result;
}

double SquareMatrix::det() const
{
    // 원본을 보존하기 위해 복사본을 소거하고 대각 피벗의 곱을 누적한다.
    Matrix upper = _matrix;
    double determinant = 1.0;
    for (std::size_t column = 0; column < size(); ++column)
    {
        // 남은 행 중 절댓값이 가장 큰 피벗을 선택해 나눗셈 오차를 줄인다.
        std::size_t pivotRow = column;
        for (std::size_t row = column + 1; row < size(); ++row)
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
            for (std::size_t entry = column; entry < size(); ++entry)
            {
                std::swap(upper(column, entry), upper(pivotRow, entry));
            }

            determinant = -determinant;
        }

        const double pivot = upper(column, column);
        determinant *= pivot;

        for (std::size_t row = column + 1; row < size(); ++row)
        {
            const double factor = upper(row, column) / pivot;
            upper(row, column) = 0.0;

            for (std::size_t entry = column + 1; entry < size(); ++entry)
            {
                upper(row, entry) -= factor * upper(column, entry);
            }
        }
    }

    return determinant;
}

}
