#include "math/linear_algebra/matrix.hpp"


#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace math
{

Matrix::Matrix() noexcept = default;

Matrix::Matrix(std::size_t rows, std::size_t columns, double value)
    : _rows(rows),
      _columns(columns)
{
    if (columns != 0 && rows > std::numeric_limits<std::size_t>::max() / columns)
    {
        throw std::length_error("Matrix dimensions are too large");
    }

    _values.assign(rows * columns, value);
}

Matrix::Matrix(std::initializer_list<std::initializer_list<double>> values)
    : _rows(values.size())
{
    if (values.size() != 0)
    {
        _columns = values.begin()->size();
    }

    _values.reserve(_rows * _columns);

    for (const auto& row : values)
    {
        if (row.size() != _columns)
        {
            throw std::invalid_argument("Matrix rows must have the same size");
        }

        _values.insert(_values.end(), row.begin(), row.end());
    }
}

Matrix::Matrix(Matrix&& other) noexcept
{
    std::swap(_rows, other._rows);
    std::swap(_columns, other._columns);
    std::swap(_values, other._values);
}

Matrix& Matrix::operator=(Matrix&& other) noexcept
{
    if (this != &other)
    {
        Matrix moved(std::move(other));
        std::swap(_rows, moved._rows);
        std::swap(_columns, moved._columns);
        std::swap(_values, moved._values);
    }
    return *this;
}

std::size_t Matrix::rows() const noexcept
{
    return _rows;
}

std::size_t Matrix::columns() const noexcept
{
    return _columns;
}

bool Matrix::empty() const noexcept
{
    return _rows == 0 || _columns == 0;
}

bool Matrix::isFinite() const noexcept
{
    for (double value : _values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

bool Matrix::equals(const Matrix& other, const Tolerance& tolerance) const noexcept
{
    if (_rows != other._rows || _columns != other._columns)
    {
        return false;
    }
    for (std::size_t index = 0; index < _values.size(); ++index)
    {
        if (!tolerance.isWithin(_values[index], other._values[index]))
        {
            return false;
        }
    }
    return true;
}

bool Matrix::operator==(const Matrix& other) const noexcept
{
    return equals(other);
}

bool Matrix::operator!=(const Matrix& other) const noexcept
{
    return !(*this == other);
}

double& Matrix::at(std::size_t row, std::size_t column)
{
    if (row >= _rows || column >= _columns)
    {
        throw std::out_of_range("Matrix index out of range");
    }

    return _values[row * _columns + column];
}

const double& Matrix::at(std::size_t row, std::size_t column) const
{
    if (row >= _rows || column >= _columns)
    {
        throw std::out_of_range("Matrix index out of range");
    }
    return _values[row * _columns + column];
}

double& Matrix::operator()(std::size_t row, std::size_t column)
{
    return at(row, column);
}

const double& Matrix::operator()(std::size_t row, std::size_t column) const
{
    return at(row, column);
}

Matrix transpose(const Matrix& matrix)
{
    Matrix result(matrix.columns(), matrix.rows());
    for (std::size_t row = 0; row < matrix.rows(); ++row)
    {
        for (std::size_t column = 0; column < matrix.columns(); ++column)
        {
            result(column, row) = matrix(row, column);
        }
    }

    return result;
}

Matrix Matrix::operator+(const Matrix& other) const
{
    if (_rows != other._rows || _columns != other._columns)
    {
        throw std::invalid_argument("Matrix dimensions must match");
    }

    Matrix result(_rows, _columns);
    for (std::size_t i = 0; i < _values.size(); ++i)
    {
        result._values[i] = _values[i] + other._values[i];
    }

    return result;
}

Matrix Matrix::operator-(const Matrix& other) const
{
    if (_rows != other._rows || _columns != other._columns)
    {
        throw std::invalid_argument("Matrix dimensions must match");
    }

    Matrix result(_rows, _columns);
    for (std::size_t i = 0; i < _values.size(); ++i)
    {
        result._values[i] = _values[i] - other._values[i];
    }

    return result;
}

Matrix Matrix::operator*(const Matrix& other) const
{
    if (_columns != other._rows)
    {
        throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
    }

    Matrix result(_rows, other._columns);
    for (std::size_t row = 0; row < _rows; ++row)
    {
        for (std::size_t inner = 0; inner < _columns; ++inner)
        {
            for (std::size_t column = 0; column < other._columns; ++column)
            {
                result(row, column) += _values[row * _columns + inner]
                    * other._values[inner * other._columns + column];
            }
        }
    }

    return result;
}

Matrix Matrix::operator*(double scalar) const
{
    Matrix result(_rows, _columns);
    for (std::size_t i = 0; i < _values.size(); ++i)
    {
        result._values[i] = _values[i] * scalar;
    }

    return result;
}

Matrix Matrix::operator/(double scalar) const
{
    if (scalar == 0.0)
    {
        throw std::domain_error("Cannot divide a matrix by zero");
    }

    return *this * (1.0 / scalar);
}

Matrix operator*(double scalar, const Matrix& matrix)
{
    return matrix * scalar;
}

}
