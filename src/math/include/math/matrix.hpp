#pragma once

#include <cstddef>
#include <initializer_list>
#include <vector>

namespace math
{

class Matrix
{
public:
    Matrix() = default;
    Matrix(std::size_t rows, std::size_t columns, double value = 0.0);
    Matrix(std::initializer_list<std::initializer_list<double>> values);

    std::size_t Rows() const noexcept;
    std::size_t Columns() const noexcept;
    bool Empty() const noexcept;

    double& operator()(std::size_t row, std::size_t column);
    const double& operator()(std::size_t row, std::size_t column) const;
    double& At(std::size_t row, std::size_t column);
    const double& At(std::size_t row, std::size_t column) const;

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator*(double scalar) const;
    Matrix operator/(double scalar) const;

    Matrix Transpose() const;
    static Matrix Identity(std::size_t size);

private:
    std::size_t rows_ = 0;
    std::size_t columns_ = 0;
    std::vector<double> values_;
};

Matrix operator*(double scalar, const Matrix& matrix);

} // namespace math