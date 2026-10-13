#pragma once

#include "math/linear_algebra/matrix.hpp"

namespace math
{

class SquareMatrix
{
public:
    SquareMatrix() noexcept;
    explicit SquareMatrix(std::size_t size, double value = 0.0);
    SquareMatrix(std::initializer_list<std::initializer_list<double>> values);
    // 비정방행렬은 std::invalid_argument를 던진다.
    explicit SquareMatrix(Matrix matrix);

    SquareMatrix(const SquareMatrix&);
    SquareMatrix& operator=(const SquareMatrix&);
    // 이동한 원본은 빈 0×0 정방행렬이 된다.
    SquareMatrix(SquareMatrix&& other) noexcept;
    SquareMatrix& operator=(SquareMatrix&& other) noexcept;

    static SquareMatrix identity(std::size_t size);

    std::size_t size() const noexcept;
    std::size_t rows() const noexcept;
    std::size_t columns() const noexcept;
    bool empty() const noexcept;
    // 모든 원소가 유한한지 검사한다. 무한대나 NaN 원소가 있으면 false를 반환한다.
    bool isFinite() const noexcept;

    // 모든 원소에 동일한 절대·상대오차 정책을 적용한다. 비유한 원소는 false이다.
    bool equals(const SquareMatrix& other, const Tolerance& tolerance) const noexcept;

    // 차원이 같고 모든 원소가 DEFAULT 절대·상대오차 안이면 true이다. 비유한 원소는 false이다.
    bool operator==(const SquareMatrix& other) const noexcept;
    // ==의 부정이다. 비유한 원소가 있으면 자기 자신과도 다르다.
    bool operator!=(const SquareMatrix& other) const noexcept;

    // 내적에 절대·상대오차 정책을 적용한다.
    bool isOrthogonal(const Tolerance& tolerance) const;
    // 직교성과 행렬식 1을 같은 정책으로 검사한다.
    bool isRotation(const Tolerance& tolerance) const;

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
