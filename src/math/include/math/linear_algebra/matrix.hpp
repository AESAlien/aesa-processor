#pragma once

#include <cstddef>
#include <initializer_list>
#include <vector>

namespace math
{

inline constexpr double DEFAULT_RELATIVE_TOLERANCE = 1e-9;

class Matrix
{
public:
    Matrix() = default;
    // 행·열 크기의 곱이 오버플로하면 std::length_error를 던진다.
    Matrix(std::size_t rows, std::size_t columns, double value = 0.0);
    // 행 길이가 서로 다르면 std::invalid_argument를 던진다.
    Matrix(std::initializer_list<std::initializer_list<double>> values);

    std::size_t rows() const noexcept;
    std::size_t columns() const noexcept;
    // 행 또는 열의 수가 0이면 true를 반환한다.
    bool empty() const noexcept;

    // 원소별 큰 쪽 절댓값에 대한 차이의 비율로 비교한다. 예: 0.01은 1%이다.
    // 차원 불일치, 음수·무한대·NaN 허용오차와 무한대·NaN 원소는 false를 반환한다.
    bool equals(const Matrix& other, double relativeTolerance = DEFAULT_RELATIVE_TOLERANCE) const noexcept;

    // at과 괄호 연산자는 모두 범위를 검사하며 범위 밖이면 std::out_of_range를 던진다.
    double& at(std::size_t row, std::size_t column);
    const double& at(std::size_t row, std::size_t column) const;

    double& operator()(std::size_t row, std::size_t column);
    const double& operator()(std::size_t row, std::size_t column) const;

    Matrix transpose() const;

    // 덧셈과 뺄셈은 행·열 크기가 모두 같아야 하며 다르면 std::invalid_argument를 던진다.
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    // 행렬 곱을 반환한다. 왼쪽 열 수와 오른쪽 행 수가 다르면 std::invalid_argument를 던진다.
    Matrix operator*(const Matrix& other) const;

    Matrix operator*(double scalar) const;
    // 스칼라로 나눈 새 값을 반환하며 0으로 나누면 std::domain_error를 던진다.
    Matrix operator/(double scalar) const;

private:
    std::size_t _rows = 0;
    std::size_t _columns = 0;
    std::vector<double> _values;
};

Matrix operator*(double scalar, const Matrix& matrix);

}
