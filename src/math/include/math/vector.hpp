#pragma once

#include "math/matrix.hpp"

namespace math
{

class SquareMatrix;

class Vector
{
public:
    Vector();
    explicit Vector(std::size_t size, double value = 0.0);
    Vector(std::initializer_list<double> values);
    // 열 수가 1이 아니면 std::invalid_argument를 던진다.
    explicit Vector(Matrix matrix);

    Vector(const Vector&) = default;
    Vector& operator=(const Vector&) = default;
    // 이동한 원본은 빈 0×1 열벡터가 된다.
    Vector(Vector&& other);
    Vector& operator=(Vector&& other);

    std::size_t size() const noexcept;
    bool empty() const noexcept;

    // 세 벡터의 길이가 같고 각 원소의 절대 차이가 대응 허용오차 이하인지 비교한다.
    // 음수·무한대·NaN 허용오차와 무한대·NaN 원소는 false를 반환한다.
    bool equals(const Vector& other, const Vector& absoluteTolerance) const noexcept;
    // 원소별 큰 쪽 절댓값에 대한 차이의 비율로 비교한다. 예: 0.01은 1%이다.
    // 길이 불일치, 음수·무한대·NaN 허용오차와 무한대·NaN 원소는 false를 반환한다.
    bool equals(const Vector& other, double relativeTolerance = DEFAULT_RELATIVE_TOLERANCE) const noexcept;

    // 모든 원소 접근은 범위를 검사한다.
    double& at(std::size_t index);
    const double& at(std::size_t index) const;
    double& operator()(std::size_t index);
    const double& operator()(std::size_t index) const;
    double& operator[](std::size_t index);
    const double& operator[](std::size_t index) const;

    // 전치 결과는 1×N 일반 행렬이다.
    Matrix transpose() const;
    // 길이가 다르면 std::invalid_argument를 던진다. 빈 벡터의 내적은 0이다.
    double dot(const Vector& other) const;
    double norm() const;

    Vector operator+(const Vector& other) const;
    Vector operator-(const Vector& other) const;
    Vector operator*(double scalar) const;
    // 0으로 나누면 std::domain_error를 던진다.
    Vector operator/(double scalar) const;

    const Matrix& asMatrix() const noexcept;

private:
    Matrix _matrix;
};

Vector operator*(double scalar, const Vector& vector);
// 행렬 열 수와 벡터 길이가 다르면 std::invalid_argument를 던진다.
Vector operator*(const Matrix& matrix, const Vector& vector);
Vector operator*(const SquareMatrix& matrix, const Vector& vector);

}
