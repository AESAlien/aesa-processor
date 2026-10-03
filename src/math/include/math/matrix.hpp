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

    std::size_t rows() const noexcept;
    std::size_t columns() const noexcept;
    bool empty() const noexcept;

    double& operator()(std::size_t row, std::size_t column);
    const double& operator()(std::size_t row, std::size_t column) const;
    double& at(std::size_t row, std::size_t column);
    const double& at(std::size_t row, std::size_t column) const;

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator*(double scalar) const;
    Matrix operator/(double scalar) const;

    Matrix transpose() const;
    // Checks A^T * A against identity using an absolute tolerance; 0x0 returns true.
    // Returns false for non-square or non-finite matrices. Throws for invalid tolerance.
    bool isOrthogonal(double tolerance = 1e-6) const;
    // Returns the determinant; the 0x0 determinant is 1. Throws for non-square matrices.
    double det() const;
    static Matrix identity(std::size_t size);

private:
    std::size_t _rows = 0;
    std::size_t _columns = 0;
    std::vector<double> _values;
};

Matrix operator*(double scalar, const Matrix& matrix);

} // namespace math
