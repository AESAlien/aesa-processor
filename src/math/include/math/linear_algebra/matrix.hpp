#pragma once

#include "math/tolerance.hpp"

#include <cstddef>
#include <initializer_list>
#include <vector>

namespace math
{

class Matrix
{
public:
    Matrix() noexcept;
    // 행·열 크기의 곱이 오버플로하면 std::length_error를 던진다.
    Matrix(std::size_t rows, std::size_t columns, double value = 0.0);
    // 행 길이가 서로 다르면 std::invalid_argument를 던진다.
    Matrix(std::initializer_list<std::initializer_list<double>> values);

    std::size_t rows() const noexcept;
    std::size_t columns() const noexcept;
    // 행 또는 열의 수가 0이면 true를 반환한다.
    bool empty() const noexcept;
    // 모든 원소가 유한하면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool isFinite() const noexcept;

    // 차원이 같고 모든 원소가 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool equals(const Matrix& other, const Tolerance& tolerance) const noexcept;

    // 차원이 같고 모든 원소가 DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool operator==(const Matrix& other) const noexcept;
    // ==의 결과를 반전한다. 무한대·NaN 원소가 있으면 자기 자신과도 다르다.
    bool operator!=(const Matrix& other) const noexcept;

    // 원소 접근 시 범위를 검사한다. 범위 밖이면 std::out_of_range를 던진다.
    double& at(std::size_t row, std::size_t column);
    // 원소 접근 시 범위를 검사한다. 범위 밖이면 std::out_of_range를 던진다.
    const double& at(std::size_t row, std::size_t column) const;

    double& operator()(std::size_t row, std::size_t column);
    const double& operator()(std::size_t row, std::size_t column) const;

    Matrix transpose() const;

    // 두 행렬의 크기가 다르면 std::invalid_argument를 던진다.
    Matrix operator+(const Matrix& other) const;
    // 두 행렬의 크기가 다르면 std::invalid_argument를 던진다.
    Matrix operator-(const Matrix& other) const;
    // 행렬 곱을 반환한다. 왼쪽 열 수와 오른쪽 행 수가 다르면 std::invalid_argument를 던진다.
    Matrix operator*(const Matrix& other) const;

    Matrix operator*(double scalar) const;
    // 스칼라로 나눈 새 값을 반환한다. 0으로 나누면 std::domain_error를 던진다.
    Matrix operator/(double scalar) const;

private:
    std::size_t _rows = 0;
    std::size_t _columns = 0;
    std::vector<double> _values;
};

Matrix operator*(double scalar, const Matrix& matrix);

}
