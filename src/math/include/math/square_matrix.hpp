#pragma once

#include "math/matrix.hpp"

namespace math
{

class SquareMatrix
{
public:
    SquareMatrix() = default;
    explicit SquareMatrix(std::size_t size, double value = 0.0);
    SquareMatrix(std::initializer_list<std::initializer_list<double>> values);
    // 비정방행렬은 std::invalid_argument를 던진다.
    explicit SquareMatrix(Matrix matrix);

    SquareMatrix(const SquareMatrix&) = default;
    SquareMatrix& operator=(const SquareMatrix&) = default;
    // 이동한 원본은 빈 0×0 정방행렬이 된다.
    SquareMatrix(SquareMatrix&& other) noexcept;
    SquareMatrix& operator=(SquareMatrix&& other) noexcept;

    static SquareMatrix identity(std::size_t size);

    std::size_t size() const noexcept;
    std::size_t rows() const noexcept;
    std::size_t columns() const noexcept;
    bool empty() const noexcept;

    // 세 행렬의 크기가 같고 각 원소의 절대 차이가 대응 허용오차 이하인지 비교한다.
    // 음수·무한대·NaN 허용오차와 무한대·NaN 원소는 false를 반환한다.
    bool equals(const SquareMatrix& other, const SquareMatrix& absoluteTolerance) const noexcept;
    // 원소별 큰 쪽 절댓값에 대한 차이의 비율로 비교한다. 예: 0.01은 1%이다.
    // 크기 불일치, 음수·무한대·NaN 허용오차와 무한대·NaN 원소는 false를 반환한다.
    bool equals(const SquareMatrix& other, double relativeTolerance = DEFAULT_RELATIVE_TOLERANCE) const noexcept;

    // 모든 원소 접근은 범위를 검사한다.
    double& at(std::size_t row, std::size_t column);
    const double& at(std::size_t row, std::size_t column) const;
    double& operator()(std::size_t row, std::size_t column);
    const double& operator()(std::size_t row, std::size_t column) const;

    SquareMatrix transpose() const;
    // 0×0 행렬의 행렬식은 1이다.
    double det() const;

    // 두 행렬의 크기가 다르면 std::invalid_argument를 던진다.
    SquareMatrix operator+(const SquareMatrix& other) const;
    SquareMatrix operator-(const SquareMatrix& other) const;
    SquareMatrix operator*(const SquareMatrix& other) const;
    SquareMatrix operator*(double scalar) const;
    // 0으로 나누면 std::domain_error를 던진다.
    SquareMatrix operator/(double scalar) const;

    const Matrix& asMatrix() const noexcept;

private:
    Matrix _matrix;
};

SquareMatrix operator*(double scalar, const SquareMatrix& matrix);

}
