#pragma once

#include "math/linear_algebra/matrix.hpp"

namespace aesa_processor::math
{

class Vector;

class SquareMatrix
{
public:
    SquareMatrix() noexcept;
    explicit SquareMatrix(std::size_t size, double value = 0.0);
    SquareMatrix(std::initializer_list<std::initializer_list<double>> values);
    // 행 수와 열 수가 다르면 std::invalid_argument를 던진다.
    explicit SquareMatrix(Matrix matrix);

    SquareMatrix(const SquareMatrix&);
    // 이동한 원본은 빈 0×0 정방행렬이 된다.
    SquareMatrix(SquareMatrix&& other) noexcept;
    SquareMatrix& operator=(const SquareMatrix&);
    SquareMatrix& operator=(SquareMatrix&& other) noexcept;

    static SquareMatrix identity(std::size_t size);

    std::size_t size() const noexcept;
    std::size_t rows() const noexcept;
    std::size_t columns() const noexcept;
    bool empty() const noexcept;
    // 모든 원소가 유한하면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool isFinite() const noexcept;
    // 전치 위치의 원소가 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool isSymmetric(const Tolerance& tolerance = DEFAULT_TOLERANCE) const noexcept;

    // 차원이 같고 모든 원소가 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool equals(const SquareMatrix& other, const Tolerance& tolerance = DEFAULT_TOLERANCE) const noexcept;

    // 차원이 같고 모든 원소가 DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool operator==(const SquareMatrix& other) const noexcept;
    // ==의 결과를 반전한다. 무한대·NaN 원소가 있으면 자기 자신과도 다르다.
    bool operator!=(const SquareMatrix& other) const noexcept;

    // 열벡터의 내적에 지정한 절대·상대오차를 적용해 직교행렬이면 true를 반환한다.
    bool isOrthogonal(const Tolerance& tolerance = DEFAULT_TOLERANCE) const;
    // 직교성과 행렬식 1에 지정한 절대·상대오차를 적용해 회전행렬이면 true를 반환한다.
    bool isRotation(const Tolerance& tolerance = DEFAULT_TOLERANCE) const;

    // 원소 접근 시 범위를 검사한다. 범위 밖이면 std::out_of_range를 던진다.
    double& at(std::size_t row, std::size_t column);
    // 원소 접근 시 범위를 검사한다. 범위 밖이면 std::out_of_range를 던진다.
    const double& at(std::size_t row, std::size_t column) const;
    double& operator()(std::size_t row, std::size_t column);
    const double& operator()(std::size_t row, std::size_t column) const;

    // 현재 행렬을 하삼각행렬 L로 보고 L * y = rhs를 만족하는 y를 반환한다.
    // 크기가 다르면 std::invalid_argument를 던진다.
    // 현재 행렬이 하삼각행렬이 아니거나 대각 원소가 0이거나 입력 또는 해에 비유한 값이 있으면 std::domain_error를 던진다.
    Vector solveLowerTriangular(const Vector& rhs) const;

    // 현재 행렬을 상삼각행렬 U로 보고 U * x = rhs를 만족하는 x를 반환한다.
    // 크기가 다르면 std::invalid_argument를 던진다.
    // 현재 행렬이 상삼각행렬이 아니거나 대각 원소가 0이거나 입력 또는 해에 비유한 값이 있으면 std::domain_error를 던진다.
    Vector solveUpperTriangular(const Vector& rhs) const;

    // 두 행렬의 크기가 다르면 std::invalid_argument를 던진다.
    SquareMatrix operator+(const SquareMatrix& other) const;
    // 두 행렬의 크기가 다르면 std::invalid_argument를 던진다.
    SquareMatrix operator-(const SquareMatrix& other) const;
    // 두 행렬의 크기가 다르면 std::invalid_argument를 던진다.
    SquareMatrix operator*(const SquareMatrix& other) const;
    SquareMatrix operator*(double scalar) const;
    // 0으로 나누면 std::domain_error를 던진다.
    SquareMatrix operator/(double scalar) const;

    const Matrix& asMatrix() const noexcept;

private:
    Matrix _matrix;
};

SquareMatrix operator*(double scalar, const SquareMatrix& matrix);
SquareMatrix transpose(const SquareMatrix& matrix);
// 0×0 행렬의 행렬식은 1이다.
double det(const SquareMatrix& matrix);
// 대칭 양의 정부호 행렬을 L * L^T로 분해한 하삼각행렬 L을 반환한다.
// 행렬이 대칭 양의 정부호가 아니거나 비유한 원소가 있으면 std::domain_error를 던진다.
SquareMatrix cholesky(const SquareMatrix& matrix);

}
