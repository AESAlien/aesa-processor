#include "math/matrix.hpp"

#include <limits>
#include <stdexcept>

namespace math
{

Matrix::Matrix(std::size_t rows, std::size_t columns, double value)
    : rows_(rows), columns_(columns)
{
    if (columns != 0 && rows > std::numeric_limits<std::size_t>::max() / columns)
        throw std::length_error("Matrix dimensions are too large");
    values_.assign(rows * columns, value);
}

Matrix::Matrix(std::initializer_list<std::initializer_list<double>> values)
    : rows_(values.size()), columns_(values.size() == 0 ? 0 : values.begin()->size())
{
    values_.reserve(rows_ * columns_);
    for (const auto& row : values) {
        if (row.size() != columns_)
            throw std::invalid_argument("Matrix rows must have the same size");
        values_.insert(values_.end(), row.begin(), row.end());
    }
}

std::size_t Matrix::Rows() const noexcept
{
    return rows_;
}

std::size_t Matrix::Columns() const noexcept
{
    return columns_;
}

bool Matrix::Empty() const noexcept
{
    return rows_ == 0 || columns_ == 0;
}

double& Matrix::operator()(std::size_t row, std::size_t column)
{
    return At(row, column);
}

const double& Matrix::operator()(std::size_t row, std::size_t column) const
{
    return At(row, column);
}

double& Matrix::At(std::size_t row, std::size_t column)
{
    if (row >= rows_ || column >= columns_)
        throw std::out_of_range("Matrix index out of range");
    return values_[row * columns_ + column];
}

const double& Matrix::At(std::size_t row, std::size_t column) const
{
    if (row >= rows_ || column >= columns_)
        throw std::out_of_range("Matrix index out of range");
    return values_[row * columns_ + column];
}

Matrix Matrix::operator+(const Matrix& other) const
{
    if (rows_ != other.rows_ || columns_ != other.columns_)
        throw std::invalid_argument("Matrix dimensions must match");

    Matrix result(rows_, columns_);
    for (std::size_t i = 0; i < values_.size(); ++i)
        result.values_[i] = values_[i] + other.values_[i];
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const
{
    if (rows_ != other.rows_ || columns_ != other.columns_)
        throw std::invalid_argument("Matrix dimensions must match");

    Matrix result(rows_, columns_);
    for (std::size_t i = 0; i < values_.size(); ++i)
        result.values_[i] = values_[i] - other.values_[i];
    return result;
}

Matrix Matrix::operator*(const Matrix& other) const
{
    if (columns_ != other.rows_)
        throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");

    Matrix result(rows_, other.columns_);
    for (std::size_t row = 0; row < rows_; ++row)
        for (std::size_t inner = 0; inner < columns_; ++inner)
            for (std::size_t column = 0; column < other.columns_; ++column)
                result(row, column) += (*this)(row, inner) * other(inner, column);
    return result;
}

Matrix Matrix::operator*(double scalar) const
{
    Matrix result(rows_, columns_);
    for (std::size_t i = 0; i < values_.size(); ++i)
        result.values_[i] = values_[i] * scalar;
    return result;
}

Matrix Matrix::operator/(double scalar) const
{
    if (scalar == 0.0)
        throw std::domain_error("Cannot divide a matrix by zero");
    return *this * (1.0 / scalar);
}

Matrix Matrix::Transpose() const
{
    Matrix result(columns_, rows_);
    for (std::size_t row = 0; row < rows_; ++row)
        for (std::size_t column = 0; column < columns_; ++column)
            result(column, row) = (*this)(row, column);
    return result;
}

Matrix Matrix::Identity(std::size_t size)
{
    Matrix result(size, size);
    for (std::size_t i = 0; i < size; ++i)
        result(i, i) = 1.0;
    return result;
}

Matrix operator*(double scalar, const Matrix& matrix)
{
    return matrix * scalar;
}

} // namespace math